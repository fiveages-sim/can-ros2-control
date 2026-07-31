#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "can_ros2_control/hands/linkerhand/o7_can_protocol.h"

namespace can_ros2_control
{

class O7CanHardware : public hardware_interface::SystemInterface
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
  using Frame = O7CanProtocol::Frame;
  using Finger = O7CanProtocol::Finger;
  using JointValues = O7CanProtocol::JointValues;
  using SteadyTime = std::chrono::steady_clock::time_point;

  static constexpr std::size_t kJointCount = O7CanProtocol::kJointCount;
  static constexpr std::size_t kFingerCount = O7CanProtocol::kFingerCount;

  // Protocol order:
  // thumb bend, thumb swing, index, middle, ring, pinky, thumb rotation.
  // ROS order:
  // thumb_joint1, thumb_joint2, thumb_joint3, index, middle, ring, pinky.
  static constexpr std::array<std::size_t, kJointCount> kProtocolToRos{
    0, 1, 3, 4, 5, 6, 2};

  void load_parameters();
  bool validate_joint_interfaces() const;
  bool open_socket();
  void close_socket();
  bool send_frame(const Frame& frame);
  bool receive_frames();
  bool send_motion_limits();
  void schedule_tactile_request();
  void handle_dispatch_result(
    const O7CanReceiveDispatcher::DispatchResult& result,
    const SteadyTime& received_time);
  void update_position_and_velocity(
    const JointValues& raw_positions,
    const SteadyTime& received_time);
  void publish_tactile(Finger finger);

  uint8_t radians_to_raw(double radians, std::size_t ros_joint_index) const;
  double raw_to_radians(uint8_t raw, std::size_t ros_joint_index) const;
  bool command_changed(const JointValues& command) const;

  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double parse_double(const std::string& value, double default_value);
  static double initial_value_for_joint(
    const hardware_interface::ComponentInfo& joint);
  static std::array<double, kJointCount> default_upper_limits(
    const std::vector<std::string>& joint_names);
  static const char* finger_name(Finger finger);

  std::string can_interface_{"can0"};
  std::string hand_side_{"right"};
  uint32_t can_id_{O7CanProtocol::kRightHandCanId};
  bool read_feedback_{true};
  bool send_initial_command_{false};
  bool configure_motion_limits_{true};
  bool read_tactile_{false};
  int feedback_timeout_ms_{2};
  int velocity_timeout_ms_{100};
  int tactile_timeout_ms_{100};
  int command_deadband_raw_{0};
  uint8_t max_velocity_raw_{O7CanProtocol::kMaxJointRaw};
  uint8_t max_acceleration_raw_{O7CanProtocol::kMaxAccelerationRaw};
  double velocity_filter_alpha_{0.25};

  int socket_fd_{-1};
  bool command_sent_{false};
  bool position_feedback_initialized_{false};
  bool tactile_request_pending_{false};
  std::array<bool, kFingerCount> tactile_batch_complete_{};
  SteadyTime last_position_feedback_time_{};
  SteadyTime tactile_request_time_{};

  std::unique_ptr<O7CanReceiveDispatcher> dispatcher_;
  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> lower_limits_{};
  std::array<double, kJointCount> upper_limits_{};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_commands_{};
  JointValues last_raw_command_{};

  std::array<
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr,
    kFingerCount>
    tactile_publishers_{};
};

}  // namespace can_ros2_control
