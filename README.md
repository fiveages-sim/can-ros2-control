# CAN ROS2 Control

灵巧手 SocketCAN / CAN FD 的 ROS2 Control 硬件接口。

## 1. 支持的末端执行器

本包注册 **3 个** `hardware_interface::SystemInterface` 插件（见 `can_ros2_control.xml`）：

| 插件 | 产品 | 识别 / 配置 |
|------|------|-------------|
| **`O6CanHardware`** | LinkerHand **O6** / **L6** | URDF 6 关节；CAN2.0 标准帧 |
| **`FreedomCanHardware`** | Freedom **V1**（6-DOF） | URDF 6 关节；CAN2.0 扩展帧 |
| **`InspireCanfdHardware`** | Inspire **RH56 系列**（E2 / F2） | URDF 6 关节；CAN FD 扩展帧 |

不支持：LinkerHand **O7**、Freedom **V2**、夹爪。

关节接口与 xacro 宏由各 `*_description` 包提供；此处仅示硬件插件用法（见 [§3](#3-配置参考)）。

## 2. 代码结构

```
can_ros2_control/
├── hands/
│   ├── linkerhand/o6_can_hardware.cpp
│   ├── freedom/freedom_can_hardware.cpp
│   └── inspire/
│       ├── inspire_canfd_hardware.cpp
│       └── inspire_can_protocol.h
└── can_ros2_control.xml
```

## 3. 配置参考

### 3.1 LinkerHand O6/L6（`O6CanHardware`）

```xml
<ros2_control name="linkerhand_o6_left_can_system" type="system">
  <hardware>
    <plugin>can_ros2_control/O6CanHardware</plugin>
    <param name="can_interface">can0</param>
    <param name="hand_side">left</param>
    <param name="hand_type">linkerhand_o6</param>
  </hardware>
  <!-- include 对应 description 包的 6-DOF 关节接口宏 -->
</ros2_control>
```

### 3.2 Freedom V1（`FreedomCanHardware`）

```xml
<ros2_control name="freedom_v1_left_can_system" type="system">
  <hardware>
    <plugin>can_ros2_control/FreedomCanHardware</plugin>
    <param name="can_interface">can0</param>
    <param name="device_id">0</param>
    <param name="read_feedback">true</param>
  </hardware>
  <!-- include freedom_description 6-DOF 关节接口宏 -->
</ros2_control>
```

### 3.3 Inspire RH56（`InspireCanfdHardware`）

```xml
<ros2_control name="inspire_e2_left_canfd_system" type="system">
  <hardware>
    <plugin>can_ros2_control/InspireCanfdHardware</plugin>
    <param name="can_interface">can0</param>
    <param name="hand_side">left</param>
    <param name="hand_id">auto</param>
    <param name="read_feedback">true</param>
  </hardware>
  <!-- include inspire_description RH56E2 / RH56F2 关节接口宏 -->
</ros2_control>
```

## 4. 硬件参数

| 插件 | 参数 | 默认 | 说明 |
|------|------|------|------|
| **O6CanHardware** | `can_interface` | `can0` | SocketCAN 接口名 |
| | `hand_side` | `right` | `left` → `0x28`，`right` → `0x27` |
| | `can_id` | — | 可选，覆盖默认 ID |
| | `read_feedback` | `true` | |
| | `feedback_timeout_ms` | `1` | |
| | `command_deadband_raw` | `0` | 0–255 |
| **FreedomCanHardware** | `can_interface` | `can0` | |
| | `device_id` | `0` | 设备 ID（0–31） |
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
| LinkerHand O6/L6 | CAN2.0 标准帧 | 弧度（rad） | `0x01` + 6 字节；`255`=伸，`0`=弯 |
| Freedom V1 | CAN2.0 扩展帧 | 弧度（rad） | Freedom 分片运动帧 |
| Inspire RH56 | CAN FD 扩展帧 | 弧度（rad） | 寄存器 `1040`/`1064` 等；限位当前为 RH56E2 |

Inspire 寄存器语义与 [modbus_ros2_control `InspireHandHardware`](https://github.com/fiveages-sim/modbus-ros2-control#33-inspire-rh56inspirehandhardware) 一致，物理层为 CAN FD。

## 6. SocketCAN 准备

```bash
# O6 / Freedom V1 — 500 kbps
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Inspire RH56 — CAN FD：1 Mbps / 5 Mbps
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

ip -details link show can0
```

## 7. 编译与启动

```bash
cd ~/ros2_ws
colcon build --packages-up-to can_ros2_control --symlink-install
source install/setup.bash
```

单臂调试示例（需对应 description 包中 `hardware:=real_can` 已接本插件）：

```bash
ros2 launch basic_joint_controller hand.launch.py \
  hand:=freedom type:=freedomv1 hardware:=real_can direction:=1

ros2 launch basic_joint_controller hand.launch.py \
  hand:=inspire type:=RH56E2 hardware:=real_can direction:=1
```

## 8. TODO

- [ ] **RH56F2 限位** — `InspireCanfdHardware` 关节上限仍为 RH56E2。
- [ ] **抽取 Inspire CAN FD 协议公共层** — 与 RS485 / 其他栈内实现去重。

## 9. 依赖

- ROS2：`hardware_interface`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle`
- 系统：Linux SocketCAN（`PF_CAN`），无额外第三方库
