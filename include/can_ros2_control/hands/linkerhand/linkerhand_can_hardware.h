#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "can_ros2_control/hands/linkerhand/linkerhand_can_protocol.h"

namespace can_ros2_control
{

struct LinkerHandModelConfig
{
  const char* model_name;
  std::size_t joint_count;
  std::array<std::size_t, 7> protocol_to_ros;
  uint8_t tactile_layout;
  std::size_t tactile_frame_count;
  std::size_t tactile_values_per_frame;
  std::array<double, 7> upper_limits;
};

class LinkerHandCanHardware : public hardware_interface::SystemInterface
{
public:
  explicit LinkerHandCanHardware(LinkerHandModelConfig config);
  ~LinkerHandCanHardware() override;

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
  using Finger = LinkerHandCanProtocol::Finger;
  using SteadyTime = std::chrono::steady_clock::time_point;

  static constexpr std::size_t kMaxJointCount = 7;
  static constexpr std::size_t kFingerCount =
    LinkerHandCanProtocol::kFingerCount;
  static constexpr uint32_t kRightHandCanId =
    LinkerHandCanProtocol::kRightHandCanId;
  static constexpr uint32_t kLeftHandCanId =
    LinkerHandCanProtocol::kLeftHandCanId;
  static constexpr uint8_t kAngleCommand =
    LinkerHandCanProtocol::kPositionCommand;
  static constexpr uint8_t kMaxTorqueCommand = 0x02;
  static constexpr uint8_t kMaxVelocityCommand = 0x05;
  static constexpr std::size_t kMaxTactileRows = 12;
  static constexpr std::size_t kMaxTactileColumns = 6;
  using TactileMatrix =
    std::array<
      std::array<uint8_t, kMaxTactileColumns>, kMaxTactileRows>;

  void load_parameters();
  void declare_tool_parameters();
  rcl_interfaces::msg::SetParametersResult on_tool_parameters(
    const std::vector<rclcpp::Parameter>& parameters);
  bool validate_joint_interfaces() const;
  bool open_socket();
  void close_socket();
  void start_io_thread();
  void stop_io_thread();
  void io_loop();
  bool drain_feedback();
  bool send_position_read_request();
  bool send_command(const std::array<uint8_t, kMaxJointCount>& raw_command);
  bool send_joint_setting(
    uint8_t command, const std::array<uint8_t, kMaxJointCount>& values);
  bool write_can_frame(const struct can_frame& frame, const char* operation);
  void process_pending_tool_settings(SteadyTime now);
  bool send_tactile_request(Finger finger);
  void schedule_tactile_request();
  bool process_tactile_frame(const struct can_frame& frame);
  void reset_tactile(Finger finger);
  void publish_tactile(Finger finger);

  uint8_t radians_to_raw(double radians, std::size_t joint_index) const;
  double raw_to_radians(uint8_t raw, std::size_t joint_index) const;
  bool raw_command_changed(const std::array<uint8_t, kMaxJointCount>& raw_command) const;

  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo& joint);
  static const char* finger_name(Finger finger);

  struct TactileState
  {
    TactileMatrix matrix{};
    std::array<bool, kMaxTactileRows> received_rows{};
    std::size_t received_frame_count{0};
  };

  struct StateSnapshot
  {
    std::array<double, kMaxJointCount> positions{};
    SteadyTime timestamp{};
    uint64_t sequence{0};
    bool valid{false};
  };

  struct CommandSnapshot
  {
    std::array<uint8_t, kMaxJointCount> raw{};
    uint64_t sequence{0};
  };

  enum class ToolSettingPhase
  {
    kIdle,
    kWaitTorqueSecond,
    kWaitVelocitySecond,
  };

  std::string can_interface_ = "can0";
  std::string hand_side_ = "right";
  std::string hand_type_;
  const LinkerHandModelConfig config_;
  uint32_t can_id_ = kRightHandCanId;
  bool read_feedback_ = true;
  bool read_tactile_ = false;
  bool send_initial_command_ = false;
  bool export_effort_ = false;
  int tactile_timeout_ms_ = 100;
  int tactile_period_ms_ = 20;
  int command_deadband_raw_ = 0;
  std::string tool_torque_parameter_name_;
  std::string tool_velocity_parameter_name_;
  std::atomic<double> tool_torque_scale_{1.0};
  std::atomic<double> tool_velocity_scale_{1.0};
  std::atomic_bool tool_settings_pending_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  int socket_fd_ = -1;
  std::atomic_bool stop_io_{false};
  std::thread io_thread_;
  std::mutex state_mutex_;
  StateSnapshot latest_state_;
  uint64_t consumed_state_sequence_ = 0;
  SteadyTime last_consumed_state_time_{};
  std::mutex command_mutex_;
  CommandSnapshot latest_command_;
  uint64_t sent_command_sequence_ = 0;
  bool command_sent_ = false;
  SteadyTime can_write_backoff_until_{};
  SteadyTime last_can_write_error_log_{};
  uint64_t suppressed_can_write_errors_ = 0;
  bool can_write_failed_ = false;
  std::atomic_bool position_feedback_initialized_{false};
  bool io_position_initialized_ = false;
  bool position_request_pending_ = false;
  SteadyTime position_request_time_{};
  ToolSettingPhase tool_setting_phase_ = ToolSettingPhase::kIdle;
  SteadyTime next_tool_setting_time_{};
  std::array<uint8_t, kMaxJointCount> pending_tool_torques_{};
  std::array<uint8_t, kMaxJointCount> pending_tool_velocities_{};
  bool tactile_request_pending_ = false;
  SteadyTime next_tactile_request_time_{};
  std::array<bool, kFingerCount> tactile_batch_complete_{};
  SteadyTime tactile_request_time_{};
  std::array<TactileState, kFingerCount> tactile_states_{};

  std::vector<std::string> joint_names_;
  std::array<double, kMaxJointCount> lower_limits_{};
  std::array<double, kMaxJointCount> upper_limits_{};
  std::array<double, kMaxJointCount> hw_positions_{};
  std::array<double, kMaxJointCount> hw_velocities_{};
  std::array<double, kMaxJointCount> hw_efforts_{};
  std::array<double, kMaxJointCount> hw_commands_{};
  std::array<double, kMaxJointCount> previous_positions_{};
  std::array<uint8_t, kMaxJointCount> last_raw_command_{};
  std::array<
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr,
    kFingerCount> tactile_publishers_{};
};

}  // namespace can_ros2_control
