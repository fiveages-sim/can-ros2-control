# CAN ROS2 Control

灵巧手 SocketCAN / CAN FD 的 ROS2 Control 硬件接口。与 `modbus_ros2_control`（USB 串口）并列，用于机器人 `left_ee_drive_type:=usb_can` / `right_ee_drive_type:=usb_can` 的独立 per-side 系统。

**注意：** `hardware:=real` + `tianji_can` 走 **Marvin 场总线**（`marvin_ros2_control` 内嵌 CAN/CANFD 驱动），**不是**本包插件。本包用于外接 SocketCAN 网卡（如 `can0` / `can1`）。

## 1. 支持的末端执行器

本包注册 **3 个** `hardware_interface::SystemInterface` 插件（见 `can_ros2_control.xml`）：

| 插件 | 产品 | 识别 / 配置 |
|------|------|-------------|
| **`O6CanHardware`** | LinkerHand **O6** / **L6** | URDF 6 关节；CAN2.0 标准帧 |
| **`FreedomCanHardware`** | Freedom **V1**（6-DOF） | URDF 6 关节；CAN2.0 扩展帧；**仅 V1** |
| **`InspireCanfdHardware`** | Inspire **RH56 系列**（E2 / F2） | URDF 6 关节；CAN FD 扩展帧；寄存器协议同 Modbus Inspire |

不支持：LinkerHand **O7**、Freedom **V2**（V2 请用 `modbus_ros2_control/FreedomRS485Hardware`）、夹爪。

推荐在 `*_description` 中通过 xacro 宏接线（见 [§3](#3-配置参考)）。

## 2. 代码结构

```
can_ros2_control/
├── hands/
│   ├── linkerhand/o6_can_hardware.cpp    # O6CanHardware
│   ├── freedom/freedom_can_hardware.cpp  # FreedomCanHardware
│   └── inspire/
│       ├── inspire_canfd_hardware.cpp    # InspireCanfdHardware
│       └── inspire_can_protocol.h        # 寄存器 / 帧 ID 编解码
└── can_ros2_control.xml
```

协议实现均在包内自包含；与 `marvin_ros2_control` 中 `InspireHandE2Canfd` 等为并行实现（见 [§8 TODO](#8-todo)）。**不依赖** `gripper_hardware_common`。

## 3. 配置参考

关节接口定义在各 description 包；此处仅示插件与关键参数。

### 3.1 LinkerHand O6/L6（`O6CanHardware`）

```xml
<ros2_control name="linkerhand_o6_left_can_system" type="system">
  <hardware>
    <plugin>can_ros2_control/O6CanHardware</plugin>
    <param name="can_interface">can0</param>
    <param name="hand_side">left</param>
    <param name="hand_type">linkerhand_o6</param>
    <!-- 可选: can_id 覆盖默认 0x28(左) / 0x27(右) -->
  </hardware>
  <!-- linkerhand_description: linkerhand_o6_interfaces -->
</ros2_control>
```

单臂 `hand.launch.py`：`linkerhand_description` 的 `hand.xacro` 目前仅接 Modbus（`hardware:=real`），**未**接 `real_can`；CAN 调试需在 URDF 中显式使用上例或扩展 xacro。

### 3.2 Freedom V1（`FreedomCanHardware`）

```xml
<!-- 推荐宏 -->
<xacro:freedom_can_side_system side="left" can_interface="can0" device_id="0"/>
```

宏定义：`freedom_description/xacro/ros2_control/side_systems.xacro`  
单臂：`freedom_description/xacro/ros2_control/hand.xacro`（`ros2_control_hardware_type:=real_can`）

### 3.3 Inspire RH56（`InspireCanfdHardware`）

```xml
<xacro:inspire_canfd_side_system side="left" can_interface="can0" hand_type="inspire_f2"/>
```

宏定义：`inspire_description/xacro/ros2_control/side_systems.xacro`  
单臂：`inspire_description/xacro/ros2_control/hand.xacro`（`hardware:=real_can`）

**m6_ccs** `usb_can` 接线见 `m6_ccs_description/xacro/ros2_control/robot.xacro`（`freedom_v1`、`inspire_e2` / `inspire_f2`）。

## 4. 硬件参数

| 插件 | 参数 | 默认 | 说明 |
|------|------|------|------|
| **O6CanHardware** | `can_interface` | `can0` | SocketCAN 接口名 |
| | `hand_side` | `right` | `left` → CAN ID `0x28`，`right` → `0x27` |
| | `can_id` | — | 可选，覆盖左右默认 ID |
| | `read_feedback` | `true` | |
| | `feedback_timeout_ms` | `1` | |
| | `command_deadband_raw` | `0` | 0–255 |
| **FreedomCanHardware** | `can_interface` | `can0` | |
| | `device_id` | `0` | 左 `0` / 右 `1`（或 xacro `auto`） |
| | `read_feedback` | `true` | |
| | `feedback_timeout_ms` | `1` | |
| | `command_deadband_deg` | `0` | |
| **InspireCanfdHardware** | `can_interface` | `can0` | 需 CAN FD |
| | `hand_side` | `left` | `hand_id:=auto` 时左 `2`、右 `1` |
| | `hand_id` | `auto` | 别名 `slave_id` / `device_id` |
| | `read_feedback` | `true` | |
| | `feedback_timeout_ms` | `2` | |
| | `default_speed` / `default_force` | `4000` / `6000` | 初始化寄存器 |
| | `wait_write_ack` | — | 写寄存器是否等待 ACK |

## 5. 协议与位置单位

| 末端 | 总线 | ROS2 Control | 设备侧 |
|------|------|--------------|--------|
| LinkerHand O6/L6 | CAN2.0 标准帧 | 弧度（rad） | 命令 `0x01` + 6 字节；`255`=伸，`0`=弯 |
| Freedom V1 | CAN2.0 扩展帧 | 弧度（rad） | Freedom 分片运动帧 |
| Inspire RH56 | **CAN FD** 扩展帧 | 弧度（rad） | 寄存器 `1040`/`1064` 等，每寄存器独立帧；限位写死 RH56E2 |

Inspire 寄存器语义与 [`modbus_ros2_control/InspireHandHardware`](../modbus_ros2_control/README.md#13-inspire-rh56inspirehandhardware)、`marvin_ros2_control::InspireHandE2` 一致，仅物理层为 CAN FD。

## 6. SocketCAN 准备

启动 ROS2 Control **之前** 配置接口（按末端选比特率）：

```bash
# LinkerHand O6 / Freedom V1 — 500 kbps
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Inspire RH56 — CAN FD：仲裁 1 Mbps，数据 5 Mbps
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

检查：`ip -details link show can0`

## 7. 编译与启动示例

```bash
cd ~/ros2_ws
colcon build --packages-up-to can_ros2_control --symlink-install
source install/setup.bash
```

单臂调试（`basic_joint_controller` + 对应 `*_description`）：

```bash
# Freedom V1（左 hand，device_id=0）
ros2 launch basic_joint_controller hand.launch.py \
  hand:=freedom type:=freedomv1 hardware:=real_can direction:=1

# Inspire RH56E2（左 hand_id=2）
ros2 launch basic_joint_controller hand.launch.py \
  hand:=inspire type:=RH56E2 hardware:=real_can direction:=1

# LinkerHand O6 CAN — 需在 URDF/xacro 中接 O6CanHardware（见 §3.1）
```

双臂整机：`ocs2_arm_controller` + `hardware:=real_usb`，并设 `left_ee_drive_type:=usb_can`（或 per-side 默认），`type` 见各机器人 README（如 m6_ccs `freedom_v1` / `inspire_f2`）。

## 8. TODO

- [ ] **抽取 Inspire CAN FD 协议公共层** — 与 `marvin_ros2_control::InspireHandE2Canfd`、`modbus_ros2_control/InspireHandHardware` 去重。
- [ ] **RH56F2 限位** — `InspireCanfdHardware` 关节上限仍为 RH56E2。
- [ ] **`linkerhand_description` 接 `real_can`** — `hand.xacro` 增加 `O6CanHardware` 分支，使单臂 launch 与文档一致。

## 9. 依赖

- ROS2：`hardware_interface`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle`
- 系统：Linux SocketCAN（`PF_CAN`），无额外第三方库

```bash
colcon build --packages-up-to can_ros2_control --symlink-install
```
