# CAN ROS2 Control

灵巧手 SocketCAN / CAN FD 的 ROS2 Control 硬件接口。

## 1. 支持的末端执行器

本包注册以下 `hardware_interface::SystemInterface` 插件（见 `can_ros2_control.xml`）：

| 插件 | 产品 | 识别 / 配置 |
|------|------|-------------|
| **`O6CanHardware`** | LinkerHand **O6** / **L6** | URDF 6 关节；CAN2.0 标准帧 |
| **`O7CanHardware`** | LinkerHand **O7** | URDF 7 关节；CAN2.0 标准帧，1 Mbps |
| **`FreedomCanHardware`** | Freedom **V1**（6-DOF） | URDF 6 关节；CAN2.0 扩展帧 |
| **`InspireCanfdHardware`** | Inspire **RH56 系列**（E2 / F2） | URDF 6 关节；CAN FD 扩展帧 |

不支持：Freedom **V2**、夹爪。

关节接口与 xacro 宏由各 `*_description` 包提供；此处仅示硬件插件用法（见 [§3](#3-配置参考)）。

## 2. 代码结构

```
can_ros2_control/
├── include/can_ros2_control/hands/
│   └── linkerhand/o7_can_protocol.h
├── src/hands/
│   ├── linkerhand/o6_can_hardware.cpp
│   ├── linkerhand/o7_can_hardware.cpp
│   ├── linkerhand/o7_can_protocol.cpp
│   ├── freedom/freedom_can_hardware.cpp
│   └── inspire/inspire_canfd_hardware.cpp
├── scripts/o7_tactile_visualizer
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

### 3.3 LinkerHand O7（`O7CanHardware`）

O7 使用位置命令接口以及位置、速度状态接口。速度由位置反馈差分计算，不使用
协议中的无符号速度数据；无符号力矩不导出为 `effort`。

```bash
ros2 launch basic_joint_controller hand.launch.py \
  hand:=linkerhand type:=o7 hardware:=real_can direction:=-1 \
  can_interface:=can0 read_tactile:=false
```

`direction:=-1` 为右手（CAN ID `0x27`），`direction:=1` 为左手（CAN ID
`0x28`）。SocketCAN 接口默认使用 `can0`。

#### O7 触觉反馈

带触觉版本的 O7 可通过同一条 CAN 总线读取五指触觉阵列。启动时必须显式传入
`read_tactile:=true`：

```bash
# 右手（CAN ID 0x27）
ros2 launch basic_joint_controller hand.launch.py \
  hand:=linkerhand type:=o7 hardware:=real_can direction:=-1 \
  can_interface:=can0 read_tactile:=true

# 左手（CAN ID 0x28）
ros2 launch basic_joint_controller hand.launch.py \
  hand:=linkerhand type:=o7 hardware:=real_can direction:=1 \
  can_interface:=can0 read_tactile:=true
```

硬件初始化日志应显示 `tactile=true`。如果显示 `tactile=false`，说明启动参数
没有传入生成后的 ros2_control 描述，此时驱动不会创建触觉发布器，也不会发送
触觉读取请求。

每根手指发布一个 `std_msgs/msg/UInt8MultiArray`：

| 手指 | 话题后缀 |
|------|----------|
| 拇指 | `tactile/thumb` |
| 食指 | `tactile/index` |
| 中指 | `tactile/middle` |
| 无名指 | `tactile/ring` |
| 小指 | `tactile/pinky` |

完整话题名称为 `/o7_hand/<left|right>/tactile/<finger>`。每条消息是一张
`12 × 6` 的无符号 8 位触觉矩阵，共 72 个值；`layout.dim` 顺序为 `row`、
`column`，`data` 按行优先排列。只有收齐一根手指的 12 个响应帧后才发布该指
的完整矩阵。

检查话题和采样率：

```bash
ros2 topic list | grep '/o7_hand/.*/tactile'
ros2 topic hz /o7_hand/right/tactile/thumb
ros2 topic echo /o7_hand/right/tactile/thumb --once
```

本包提供五指实时热力图工具：

```bash
# 右手
ros2 run can_ros2_control o7_tactile_visualizer \
  --ros-args -p hand_side:=right

# 左手
ros2 run can_ros2_control o7_tactile_visualizer \
  --ros-args -p hand_side:=left
```

可选参数：`topic_prefix`（默认 `/o7_hand`）、`refresh_rate`（默认 20 Hz）和
`color_max`（默认 255）。该工具依赖 `python3-matplotlib` 和 `python3-numpy`。

触觉协议使用 `0xB1`～`0xB5` 分别请求拇指到小指，`0xC6` 选择 12×6
矩阵。一次单指请求应返回 12 个 DLC=8 的响应帧。安装 `can-utils` 后可绕过
ROS 进行原始总线自检：

```bash
sudo apt install can-utils

# 终端 1：右手；左手将 027 改为 028
candump -tz can0,027:7FF

# 终端 2：请求右手拇指触觉
cansend can0 027#B1C6
```

正常响应的命令字为 `B1`，坐标字节依次为 `00`、`10`、...、`B0`。
其他手指将请求命令改为 `B2`、`B3`、`B4`、`B5`。

### 3.4 Inspire RH56（`InspireCanfdHardware`）

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
| **O7CanHardware** | `can_interface` | `can0` | SocketCAN 接口名 |
| | `hand_side` | `right` | `left` → `0x28`，`right` → `0x27` |
| | `can_id` | — | 可选，覆盖由 `hand_side` 推导的 ID |
| | `read_feedback` | `true` | 读取位置反馈并差分计算有符号速度 |
| | `read_tactile` | `false` | 启用五指 12×6 触觉读取与 ROS 话题 |
| | `feedback_timeout_ms` | `2` | 每个控制周期接收 CAN 帧的等待窗口 |
| | `velocity_timeout_ms` | `100` | 位置反馈超时后将速度状态归零 |
| | `tactile_timeout_ms` | `100` | 触觉批次超时后重新发起五指请求 |
| | `command_deadband_raw` | `0` | 0–255；抑制小幅重复位置命令 |
| | `configure_motion_limits` | `true` | 激活时写入速度和加速度上限 |
| | `max_velocity_raw` | `255` | 各关节原始速度上限 |
| | `max_acceleration_raw` | `254` | 各关节原始加速度上限，最大 254 |
| | `velocity_filter_alpha` | `0.25` | 差分速度低通滤波系数，范围 0–1 |
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
| LinkerHand O7 | CAN2.0 标准帧 | 弧度（rad）；触觉为 `UInt8MultiArray` | `0x01` + 7 字节；触觉 `0xB1`～`0xB5` |
| Freedom V1 | CAN2.0 扩展帧 | 弧度（rad） | Freedom 分片运动帧 |
| Inspire RH56 | CAN FD 扩展帧 | 弧度（rad） | 寄存器 `1040`/`1064` 等；限位当前为 RH56E2 |

Inspire 寄存器语义与 [modbus_ros2_control `InspireHandHardware`](https://github.com/fiveages-sim/modbus-ros2-control#33-inspire-rh56inspirehandhardware) 一致，物理层为 CAN FD。

## 6. SocketCAN 准备

```bash
# O6 / Freedom V1 — 500 kbps
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# LinkerHand O7 — 1 Mbps
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
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

### 7.1 O7 触觉故障排查

1. 启动日志必须包含 `feedback=true, tactile=true`。
2. `/joint_states` 有数据但没有触觉话题：检查 `read_tactile` 是否传入 Xacro。
3. 有触觉话题但没有消息：用 `candump` 检查是否发出 `B1 C6`～`B5 C6`，
   以及每根手指是否返回完整 12 帧。
4. 手动 `cansend` 有响应但 ROS 无消息：核对响应 DLC 是否为 8、命令字是否为
   `B1`～`B5`、坐标是否为 `00`～`B0`。当前解析器会拒绝格式不符的帧。
5. 检查 CAN 状态和错误计数：

```bash
ip -details -statistics link show can0
```

O7 应使用经典 CAN 1 Mbps，接口通常应处于 `ERROR-ACTIVE`；大量 RX 丢包、
`ERROR-PASSIVE` 或 `BUS-OFF` 通常表示波特率、终端电阻或物理链路问题。

## 8. TODO

- [ ] **RH56F2 限位** — `InspireCanfdHardware` 关节上限仍为 RH56E2。
- [ ] **抽取 Inspire CAN FD 协议公共层** — 与 RS485 / 其他栈内实现去重。

## 9. 依赖

- ROS2：`hardware_interface`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle`
- 系统：Linux SocketCAN（`PF_CAN`），无额外第三方库
