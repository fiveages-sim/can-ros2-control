#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <linux/can.h>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "can_ros2_control/hands/linkerhand/o7_can_protocol.h"

namespace can_ros2_control
{

class O6CanHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

private:
  using Finger = O7CanProtocol::Finger;
  using SteadyTime = std::chrono::steady_clock::time_point;

  static constexpr std::size_t kJointCount = 6;
  static constexpr std::size_t kFingerCount = O7CanProtocol::kFingerCount;
  static constexpr uint32_t kRightHandCanId = 0x27;
  static constexpr uint32_t kLeftHandCanId = 0x28;
  static constexpr uint8_t kAngleCommand = 0x01;
  static constexpr uint8_t kMaxTorqueCommand = 0x02;
  static constexpr uint8_t kMaxVelocityCommand = 0x05;
  // O6 selector 0xA4 requests a 10-row x 4-column tactile matrix.
  static constexpr uint8_t kTactileLayout10x4 = 0xA4;
  static constexpr std::size_t kTactileRows = 10;
  static constexpr std::size_t kTactileColumns = 4;
  static constexpr std::size_t kTactileFramesPerRequest = 10;
  static constexpr std::size_t kTactileValuesPerFrame = 4;
  static constexpr std::size_t kTactilePointCount =
    kTactileRows * kTactileColumns;
  static constexpr uint8_t kTactileResponseDlc = 6;
  using TactileMatrix =
    std::array<std::array<uint8_t, kTactileColumns>, kTactileRows>;

  void load_parameters();
  void declare_tool_parameters();
  rcl_interfaces::msg::SetParametersResult on_tool_parameters(
    const std::vector<rclcpp::Parameter>& parameters);
  bool validate_joint_interfaces() const;
  bool open_socket();
  void close_socket();
  bool receive_feedback(const rclcpp::Duration& period);
  bool send_position_read_request();
  bool send_command(const std::array<uint8_t, kJointCount>& raw_command);
  bool send_joint_setting(
    uint8_t command, const std::array<uint8_t, kJointCount>& values);
  bool send_pending_tool_settings();
  bool send_tactile_request(Finger finger);
  void schedule_tactile_request();
  bool process_tactile_frame(const struct can_frame& frame);
  void reset_tactile(Finger finger);
  void publish_tactile(Finger finger);

  uint8_t radians_to_raw(double radians, std::size_t joint_index) const;
  double raw_to_radians(uint8_t raw, std::size_t joint_index) const;
  bool raw_command_changed(const std::array<uint8_t, kJointCount>& raw_command) const;

  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo& joint);
  static std::array<double, kJointCount> default_upper_limits(const std::vector<std::string>& joint_names);
  static const char* finger_name(Finger finger);

  struct TactileState
  {
    TactileMatrix matrix{};
    std::array<bool, kTactileRows> received_rows{};
    std::size_t received_frame_count{0};
  };

  std::string can_interface_ = "can0";
  std::string hand_side_ = "right";
  std::string hand_type_;
  uint32_t can_id_ = kRightHandCanId;
  bool read_feedback_ = true;
  bool read_tactile_ = false;
  bool send_initial_command_ = false;
  int feedback_timeout_ms_ = 1;
  int tactile_timeout_ms_ = 100;
  int command_deadband_raw_ = 0;
  std::string tool_torque_parameter_name_;
  std::string tool_velocity_parameter_name_;
  std::atomic<double> tool_torque_scale_{1.0};
  std::atomic<double> tool_velocity_scale_{1.0};
  std::atomic_bool tool_settings_pending_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  int socket_fd_ = -1;
  bool command_sent_ = false;
  bool position_feedback_initialized_ = false;
  bool tactile_request_pending_ = false;
  std::array<bool, kFingerCount> tactile_batch_complete_{};
  SteadyTime tactile_request_time_{};
  std::array<TactileState, kFingerCount> tactile_states_{};

  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> lower_limits_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointCount> upper_limits_{0.58, 1.30, 1.60, 1.60, 1.60, 1.60};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_efforts_{};
  std::array<double, kJointCount> hw_commands_{};
  std::array<double, kJointCount> previous_positions_{};
  std::array<uint8_t, kJointCount> last_raw_command_{255, 255, 255, 255, 255, 255};
  std::array<
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr,
    kFingerCount> tactile_publishers_{};
};

}  // namespace can_ros2_control
