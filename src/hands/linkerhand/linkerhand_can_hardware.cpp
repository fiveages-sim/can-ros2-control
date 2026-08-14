#include "can_ros2_control/hands/linkerhand/linkerhand_can_hardware.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

namespace can_ros2_control
{
namespace
{
constexpr auto kLoggerName = "LinkerHandCanHardware";

}  // namespace

LinkerHandCanHardware::LinkerHandCanHardware(
  LinkerHandModelConfig config)
: config_(std::move(config))
{
}

hardware_interface::CallbackReturn LinkerHandCanHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != config_.joint_count)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "%s CAN hardware expects exactly %zu joints, got %zu",
      config_.model_name,
      config_.joint_count,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.reserve(config_.joint_count);
  for (const auto& joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  if (!validate_joint_interfaces())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  load_parameters();
  declare_tool_parameters();
  upper_limits_ = config_.upper_limits;

  for (std::size_t i = 0; i < config_.joint_count; ++i)
  {
    hw_positions_[i] = initial_value_for_joint(info_.joints[i]);
    hw_positions_[i] = std::clamp(hw_positions_[i], lower_limits_[i], upper_limits_[i]);
    previous_positions_[i] = hw_positions_[i];
    hw_commands_[i] = hw_positions_[i];
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    last_raw_command_[i] = radians_to_raw(hw_commands_[i], i);
  }

  if (read_tactile_)
  {
    for (std::size_t index = 0; index < kFingerCount; ++index)
    {
      const auto finger = static_cast<Finger>(index);
      const std::string topic =
        "/" + std::string(config_.model_name) + "_hand/" + hand_side_ +
        "/tactile/" +
        finger_name(finger);
      tactile_publishers_[index] =
        get_node()->create_publisher<std_msgs::msg::UInt8MultiArray>(topic, 10);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured %s CAN hardware: interface=%s, side=%s, can_id=0x%X, "
    "feedback=%s, tactile=%s",
    config_.model_name,
    can_interface_.c_str(),
    hand_side_.c_str(),
    can_id_,
    read_feedback_ ? "true" : "false",
    read_tactile_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LinkerHandCanHardware::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  if (!open_socket())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_sent_ = false;
  tool_settings_pending_.store(true);
  position_feedback_initialized_ = false;
  tactile_request_pending_ = false;
  tactile_batch_complete_.fill(false);
  for (std::size_t index = 0; index < kFingerCount; ++index)
  {
    reset_tactile(static_cast<Finger>(index));
  }
  for (std::size_t i = 0; i < config_.joint_count; ++i)
  {
    hw_commands_[i] = hw_positions_[i];
    last_raw_command_[i] = radians_to_raw(hw_commands_[i], i);
  }

  if (send_initial_command_ && !read_feedback_)
  {
    std::array<uint8_t, kMaxJointCount> raw_command{};
    for (std::size_t protocol_index = 0;
         protocol_index < config_.joint_count; ++protocol_index)
    {
      const auto ros_index = config_.protocol_to_ros[protocol_index];
      raw_command[protocol_index] =
        radians_to_raw(hw_commands_[ros_index], ros_index);
    }
    if (!send_command(raw_command))
    {
      close_socket();
      return hardware_interface::CallbackReturn::ERROR;
    }
    last_raw_command_ = raw_command;
    command_sent_ = true;
  }
  else if (send_initial_command_)
  {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "Ignoring send_initial_command because position feedback is enabled");
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "%s CAN hardware activated on %s",
    config_.model_name,
    can_interface_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LinkerHandCanHardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  close_socket();
  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName), "%s CAN hardware deactivated",
    config_.model_name);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
LinkerHandCanHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.reserve(config_.joint_count * (config_.export_effort ? 3 : 2));

  for (std::size_t i = 0; i < config_.joint_count; ++i)
  {
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    if (config_.export_effort)
    {
      state_interfaces.push_back(
        std::make_shared<hardware_interface::StateInterface>(
          joint_names_[i], hardware_interface::HW_IF_EFFORT,
          &hw_efforts_[i]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
LinkerHandCanHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.reserve(config_.joint_count);

  for (std::size_t i = 0; i < config_.joint_count; ++i)
  {
    command_interfaces.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type LinkerHandCanHardware::read(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& period)
{
  if (socket_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  previous_positions_ = hw_positions_;

  if (read_feedback_ && !send_position_read_request())
  {
    return hardware_interface::return_type::ERROR;
  }

  if (read_feedback_ || read_tactile_)
  {
    if (!receive_feedback(period))
    {
      return hardware_interface::return_type::ERROR;
    }
  }
  if (!read_feedback_)
  {
    hw_positions_ = hw_commands_;
  }

  // On startup, position must be initialized before applying torque and
  // velocity limits or starting tactile traffic. Each pending setting is
  // sent twice by send_pending_tool_settings().
  if ((!read_feedback_ || position_feedback_initialized_) &&
      !send_pending_tool_settings())
  {
    return hardware_interface::return_type::ERROR;
  }

  if (read_tactile_ &&
      (!read_feedback_ || position_feedback_initialized_))
  {
    schedule_tactile_request();
  }

  const double dt = period.seconds();
  for (std::size_t i = 0; i < config_.joint_count; ++i)
  {
    hw_efforts_[i] = 0.0;
    hw_velocities_[i] = dt > std::numeric_limits<double>::epsilon()
                          ? (hw_positions_[i] - previous_positions_[i]) / dt
                          : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LinkerHandCanHardware::write(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& /* period */)
{
  if (socket_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  // Wait for the first real position sample before allowing ros2_control to
  // transmit. Otherwise the URDF's zero initial values make the hand jump to
  // zero during startup.
  if (read_feedback_ && !position_feedback_initialized_)
  {
    return hardware_interface::return_type::OK;
  }

  std::array<uint8_t, kMaxJointCount> raw_command{};
  for (std::size_t protocol_index = 0;
       protocol_index < config_.joint_count; ++protocol_index)
  {
    const auto ros_index = config_.protocol_to_ros[protocol_index];
    hw_commands_[ros_index] = std::clamp(
      hw_commands_[ros_index], lower_limits_[ros_index],
      upper_limits_[ros_index]);
    raw_command[protocol_index] =
      radians_to_raw(hw_commands_[ros_index], ros_index);
  }

  if (command_sent_ && !raw_command_changed(raw_command))
  {
    return hardware_interface::return_type::OK;
  }

  if (!send_command(raw_command))
  {
    return hardware_interface::return_type::ERROR;
  }

  last_raw_command_ = raw_command;
  command_sent_ = true;

  if (!read_feedback_)
  {
    hw_positions_ = hw_commands_;
  }

  return hardware_interface::return_type::OK;
}

void LinkerHandCanHardware::load_parameters()
{
  const auto get_parameter = [this](const std::string& name, const std::string& fallback) {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };

  can_interface_ = get_parameter("can_interface", can_interface_);
  hand_side_ = get_parameter("hand_side", hand_side_);
  hand_type_ = get_parameter("hand_type", hand_type_);
  read_feedback_ = parse_bool(get_parameter("read_feedback", read_feedback_ ? "true" : "false"), read_feedback_);
  read_tactile_ = parse_bool(
    get_parameter("read_tactile", read_tactile_ ? "true" : "false"),
    read_tactile_);
  send_initial_command_ = parse_bool(
    get_parameter("send_initial_command", send_initial_command_ ? "true" : "false"),
    send_initial_command_);
  feedback_timeout_ms_ = std::max(0, parse_int(get_parameter("feedback_timeout_ms", "1"), feedback_timeout_ms_));
  tactile_timeout_ms_ = std::max(
    1, parse_int(get_parameter("tactile_timeout_ms", "100"), tactile_timeout_ms_));
  command_deadband_raw_ = std::clamp(parse_int(get_parameter("command_deadband_raw", "0"), 0), 0, 255);

  tool_torque_parameter_name_ =
    hand_side_ == "left" ? "left_tool_torque" : "right_tool_torque";
  tool_velocity_parameter_name_ =
    hand_side_ == "left" ? "left_tool_velocity" : "right_tool_velocity";
  const auto parse_scale = [&get_parameter](const std::string& name) {
    try
    {
      return std::clamp(std::stod(get_parameter(name, "1.0")), 0.0, 1.0);
    }
    catch (const std::exception&)
    {
      return 1.0;
    }
  };
  tool_torque_scale_.store(parse_scale(tool_torque_parameter_name_));
  tool_velocity_scale_.store(parse_scale(tool_velocity_parameter_name_));

  const auto can_id_parameter = info_.hardware_parameters.find("can_id");
  if (can_id_parameter != info_.hardware_parameters.end())
  {
    can_id_ = static_cast<uint32_t>(parse_int(can_id_parameter->second, static_cast<int>(can_id_)));
  }
  else if (hand_side_ == "left")
  {
    can_id_ = kLeftHandCanId;
  }
  else
  {
    can_id_ = kRightHandCanId;
  }
}

void LinkerHandCanHardware::declare_tool_parameters()
{
  const auto node = get_node();
  if (!node)
  {
    return;
  }

  if (!node->has_parameter(tool_torque_parameter_name_))
  {
    node->declare_parameter<double>(
      tool_torque_parameter_name_, tool_torque_scale_.load());
  }
  if (!node->has_parameter(tool_velocity_parameter_name_))
  {
    node->declare_parameter<double>(
      tool_velocity_parameter_name_, tool_velocity_scale_.load());
  }

  parameter_callback_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& parameters) {
      return on_tool_parameters(parameters);
    });
}

rcl_interfaces::msg::SetParametersResult LinkerHandCanHardware::on_tool_parameters(
  const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  double next_torque = tool_torque_scale_.load();
  double next_velocity = tool_velocity_scale_.load();
  bool changed = false;

  for (const auto& parameter : parameters)
  {
    const auto& name = parameter.get_name();
    if (name != tool_torque_parameter_name_ &&
        name != tool_velocity_parameter_name_)
    {
      continue;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
        parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      result.successful = false;
      result.reason = name + " must be numeric";
      return result;
    }
    const double value =
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
        ? static_cast<double>(parameter.as_int())
        : parameter.as_double();
    if (value < 0.0 || value > 1.0)
    {
      result.successful = false;
      result.reason = name + " must be in [0.0, 1.0]";
      return result;
    }

    if (name == tool_torque_parameter_name_)
    {
      next_torque = value;
    }
    else
    {
      next_velocity = value;
    }
    changed = true;
  }
  if (changed)
  {
    tool_torque_scale_.store(next_torque);
    tool_velocity_scale_.store(next_velocity);
    tool_settings_pending_.store(true);
  }
  return result;
}

bool LinkerHandCanHardware::validate_joint_interfaces() const
{
  for (const auto& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose exactly one position command interface",
        joint.name.c_str());
      return false;
    }

    const auto has_state_interface = [&joint](const std::string& interface_name) {
      return std::any_of(
        joint.state_interfaces.begin(),
        joint.state_interfaces.end(),
        [&interface_name](const auto& interface) { return interface.name == interface_name; });
    };

    if (!has_state_interface(hardware_interface::HW_IF_POSITION) ||
        !has_state_interface(hardware_interface::HW_IF_VELOCITY) ||
        (config_.export_effort &&
         !has_state_interface(hardware_interface::HW_IF_EFFORT)) ||
        (!config_.export_effort &&
         has_state_interface(hardware_interface::HW_IF_EFFORT)))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' has state interfaces incompatible with %s",
        joint.name.c_str(), config_.model_name);
      return false;
    }
  }

  return true;
}

bool LinkerHandCanHardware::open_socket()
{
  close_socket();

  socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to create CAN socket: %s",
      std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to find CAN interface '%s': %s",
      can_interface_.c_str(),
      std::strerror(errno));
    close_socket();
    return false;
  }

  struct sockaddr_can address;
  std::memset(&address, 0, sizeof(address));
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to bind CAN interface '%s': %s",
      can_interface_.c_str(),
      std::strerror(errno));
    close_socket();
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Opened SocketCAN interface %s for CAN ID 0x%X",
    can_interface_.c_str(),
    can_id_);

  return true;
}

void LinkerHandCanHardware::close_socket()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool LinkerHandCanHardware::receive_feedback(const rclcpp::Duration& period)
{
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(feedback_timeout_ms_);
  bool processed_position_frame = false;

  while (true)
  {
    struct can_frame frame;
    const auto bytes_read = ::read(socket_fd_, &frame, sizeof(frame));

    if (bytes_read == static_cast<ssize_t>(sizeof(frame)))
    {
      if ((frame.can_id & CAN_EFF_FLAG) != 0)
      {
        continue;
      }

      const auto frame_id = frame.can_id & CAN_SFF_MASK;
      if (frame_id == can_id_ && read_tactile_ && process_tactile_frame(frame))
      {
        continue;
      }
      if (frame_id == can_id_ &&
          frame.can_dlc >= config_.joint_count + 1 &&
          frame.data[0] == kAngleCommand)
      {
        for (std::size_t protocol_index = 0;
             protocol_index < config_.joint_count; ++protocol_index)
        {
          const auto ros_index = config_.protocol_to_ros[protocol_index];
          hw_positions_[ros_index] = raw_to_radians(
            frame.data[protocol_index + 1], ros_index);
        }
        if (!position_feedback_initialized_)
        {
          // Seed both the command and velocity baseline from hardware. The
          // controller therefore starts by holding the measured pose.
          hw_commands_ = hw_positions_;
          previous_positions_ = hw_positions_;
          hw_velocities_.fill(0.0);
          for (std::size_t protocol_index = 0;
               protocol_index < config_.joint_count; ++protocol_index)
          {
            last_raw_command_[protocol_index] =
              frame.data[protocol_index + 1];
          }
          position_feedback_initialized_ = true;
          RCLCPP_INFO(
            rclcpp::get_logger(kLoggerName),
            "Initialized %s commands from the first position feedback frame",
            config_.model_name);
        }
        processed_position_frame = true;
      }
      continue;
    }

    if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      if (feedback_timeout_ms_ == 0 || std::chrono::steady_clock::now() >= deadline)
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (bytes_read < 0 && errno == EINTR)
    {
      continue;
    }

    if (bytes_read < 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Failed to read CAN frame: %s",
        std::strerror(errno));
      return false;
    }

    break;
  }

  if (!processed_position_frame && period.seconds() > 0.0)
  {
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger(kLoggerName),
      *get_node()->get_clock(),
      2000,
      "No %s CAN feedback frame received for CAN ID 0x%X",
      config_.model_name, can_id_);
  }

  return true;
}

bool LinkerHandCanHardware::send_position_read_request()
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id_;
  frame.can_dlc = 1;
  frame.data[0] = kAngleCommand;

  if (::write(socket_fd_, &frame, sizeof(frame)) !=
      static_cast<ssize_t>(sizeof(frame)))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to write %s position request to ID 0x%X: %s",
      config_.model_name, can_id_, std::strerror(errno));
    return false;
  }
  return true;
}

bool LinkerHandCanHardware::send_tactile_request(Finger finger)
{
  const auto request = LinkerHandCanProtocol::make_tactile_read_request(
    can_id_, finger, config_.tactile_layout);
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = request.can_id;
  frame.can_dlc = request.dlc;
  std::copy_n(request.data.begin(), request.dlc, frame.data);

  if (::write(socket_fd_, &frame, sizeof(frame)) !=
      static_cast<ssize_t>(sizeof(frame)))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to write %s tactile request 0x%02X to ID 0x%X: %s",
      config_.model_name, request.data[0], request.can_id,
      std::strerror(errno));
    return false;
  }
  return true;
}

void LinkerHandCanHardware::schedule_tactile_request()
{
  const auto now = std::chrono::steady_clock::now();
  if (tactile_request_pending_)
  {
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - tactile_request_time_);
    if (age.count() <= tactile_timeout_ms_)
    {
      return;
    }
    tactile_request_pending_ = false;
  }

  tactile_batch_complete_.fill(false);
  for (std::size_t index = 0; index < kFingerCount; ++index)
  {
    reset_tactile(static_cast<Finger>(index));
  }
  for (std::size_t index = 0; index < kFingerCount; ++index)
  {
    if (!send_tactile_request(static_cast<Finger>(index)))
    {
      return;
    }
  }
  tactile_request_time_ = now;
  tactile_request_pending_ = true;
}

bool LinkerHandCanHardware::process_tactile_frame(const struct can_frame& socket_frame)
{
  LinkerHandCanProtocol::Frame frame;
  frame.can_id = socket_frame.can_id & CAN_SFF_MASK;
  frame.dlc = std::min<uint8_t>(socket_frame.can_dlc, 8);
  std::copy_n(socket_frame.data, frame.dlc, frame.data.begin());

  Finger finger;
  if (!LinkerHandCanProtocol::finger_from_tactile_command(
        frame.data[0], finger))
  {
    return false;
  }

  const auto expected_dlc = static_cast<uint8_t>(
    config_.tactile_values_per_frame + 2);
  if (frame.can_id != can_id_ || frame.dlc != expected_dlc)
  {
    return false;
  }

  const uint8_t coordinate = frame.data[1];
  const uint8_t row = static_cast<uint8_t>((coordinate >> 4) & 0x0F);
  const uint8_t column = static_cast<uint8_t>(coordinate & 0x0F);
  if (row >= config_.tactile_frame_count ||
      column >= config_.tactile_values_per_frame ||
      static_cast<std::size_t>(column) + config_.tactile_values_per_frame >
        kMaxTactileColumns)
  {
    return false;
  }

  const auto finger_index = static_cast<std::size_t>(finger);
  auto& state = tactile_states_[finger_index];
  std::copy_n(
    frame.data.begin() + 2, config_.tactile_values_per_frame,
    state.matrix[row].begin() + column);
  if (!state.received_rows[row])
  {
    state.received_rows[row] = true;
    ++state.received_frame_count;
  }

  if (state.received_frame_count == config_.tactile_frame_count &&
      !tactile_batch_complete_[finger_index])
  {
    tactile_batch_complete_[finger_index] = true;
    publish_tactile(finger);
    if (std::all_of(
          tactile_batch_complete_.begin(), tactile_batch_complete_.end(),
          [](bool complete) { return complete; }))
    {
      tactile_request_pending_ = false;
    }
  }
  return true;
}

void LinkerHandCanHardware::reset_tactile(Finger finger)
{
  auto& state = tactile_states_[static_cast<std::size_t>(finger)];
  for (auto& row : state.matrix)
  {
    row.fill(0);
  }
  state.received_rows.fill(false);
  state.received_frame_count = 0;
}

void LinkerHandCanHardware::publish_tactile(Finger finger)
{
  const auto index = static_cast<std::size_t>(finger);
  if (!tactile_publishers_[index])
  {
    return;
  }

  std_msgs::msg::UInt8MultiArray message;
  message.layout.dim.resize(2);
  message.layout.dim[0].label = "row";
  const std::size_t point_count =
    config_.tactile_frame_count * config_.tactile_values_per_frame;
  message.layout.dim[0].size = config_.tactile_frame_count;
  message.layout.dim[0].stride = point_count;
  message.layout.dim[1].label = "column";
  message.layout.dim[1].size = config_.tactile_values_per_frame;
  message.layout.dim[1].stride = config_.tactile_values_per_frame;
  message.data.reserve(point_count);
  for (std::size_t row = 0; row < config_.tactile_frame_count; ++row)
  {
    message.data.insert(
      message.data.end(), tactile_states_[index].matrix[row].begin(),
      tactile_states_[index].matrix[row].begin() +
        config_.tactile_values_per_frame);
  }
  tactile_publishers_[index]->publish(message);
}

const char* LinkerHandCanHardware::finger_name(Finger finger)
{
  switch (finger)
  {
    case Finger::kThumb: return "thumb";
    case Finger::kIndex: return "index";
    case Finger::kMiddle: return "middle";
    case Finger::kRing: return "ring";
    case Finger::kPinky: return "pinky";
  }
  return "unknown";
}

bool LinkerHandCanHardware::send_command(const std::array<uint8_t, kMaxJointCount>& raw_command)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id_;
  frame.can_dlc = static_cast<uint8_t>(config_.joint_count + 1);
  frame.data[0] = kAngleCommand;

  for (std::size_t i = 0; i < config_.joint_count; ++i)
  {
    frame.data[i + 1] = raw_command[i];
  }

  const auto bytes_written = ::write(socket_fd_, &frame, sizeof(frame));
  if (bytes_written != static_cast<ssize_t>(sizeof(frame)))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to write %s CAN command to ID 0x%X: %s",
      config_.model_name,
      can_id_,
      std::strerror(errno));
    return false;
  }

  return true;
}

bool LinkerHandCanHardware::send_joint_setting(
  uint8_t command, const std::array<uint8_t, kMaxJointCount>& values)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id_;
  frame.can_dlc = static_cast<uint8_t>(config_.joint_count + 1);
  frame.data[0] = command;
  std::copy_n(values.begin(), config_.joint_count, frame.data + 1);

  const auto bytes_written = ::write(socket_fd_, &frame, sizeof(frame));
  if (bytes_written != static_cast<ssize_t>(sizeof(frame)))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to write %s setting 0x%02X to ID 0x%X: %s",
      config_.model_name, command, can_id_, std::strerror(errno));
    return false;
  }
  return true;
}

bool LinkerHandCanHardware::send_pending_tool_settings()
{
  if (!tool_settings_pending_.exchange(false))
  {
    return true;
  }

  const auto to_raw = [](double scale) {
    return static_cast<uint8_t>(
      std::lround(std::clamp(scale, 0.0, 1.0) * 255.0));
  };
  std::array<uint8_t, kMaxJointCount> torques{};
  std::array<uint8_t, kMaxJointCount> velocities{};
  torques.fill(to_raw(tool_torque_scale_.load()));
  velocities.fill(to_raw(tool_velocity_scale_.load()));

  if (!send_joint_setting(kMaxTorqueCommand, torques))
  {
    tool_settings_pending_.store(true);
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(6));
  if (!send_joint_setting(kMaxTorqueCommand, torques))
  {
    tool_settings_pending_.store(true);
    return false;
  }

  // The LinkerHand vendor implementation sends each setting twice. A single
  // speed frame can be lost when it is adjacent to another request, even
  // though SocketCAN accepted the write. The vendor loop has a 1 ms pre-send
  // delay plus send_frame()'s 5 ms delay, so preserve the effective 6 ms gap.
  if (!send_joint_setting(kMaxVelocityCommand, velocities))
  {
    tool_settings_pending_.store(true);
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(6));
  if (!send_joint_setting(kMaxVelocityCommand, velocities))
  {
    tool_settings_pending_.store(true);
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Applied %s %s tool limits: torque=%u, velocity=%u",
    config_.model_name, hand_side_.c_str(),
    torques.front(), velocities.front());
  return true;
}

uint8_t LinkerHandCanHardware::radians_to_raw(double radians, std::size_t joint_index) const
{
  const auto lower = lower_limits_[joint_index];
  const auto upper = upper_limits_[joint_index];
  if (upper <= lower)
  {
    return 255;
  }

  const auto normalized = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
  const auto raw = std::lround(255.0 * (1.0 - normalized));
  return static_cast<uint8_t>(std::clamp<long>(raw, 0, 255));
}

double LinkerHandCanHardware::raw_to_radians(uint8_t raw, std::size_t joint_index) const
{
  const auto lower = lower_limits_[joint_index];
  const auto upper = upper_limits_[joint_index];
  if (upper <= lower)
  {
    return lower;
  }

  const auto normalized = 1.0 - static_cast<double>(raw) / 255.0;
  return lower + normalized * (upper - lower);
}

bool LinkerHandCanHardware::raw_command_changed(const std::array<uint8_t, kMaxJointCount>& raw_command) const
{
  for (std::size_t i = 0; i < config_.joint_count; ++i)
  {
    if (std::abs(static_cast<int>(raw_command[i]) - static_cast<int>(last_raw_command_[i])) >
        command_deadband_raw_)
    {
      return true;
    }
  }
  return false;
}

bool LinkerHandCanHardware::parse_bool(const std::string& value, bool default_value)
{
  if (value == "true" || value == "1" || value == "True" || value == "TRUE")
  {
    return true;
  }
  if (value == "false" || value == "0" || value == "False" || value == "FALSE")
  {
    return false;
  }
  return default_value;
}

int LinkerHandCanHardware::parse_int(const std::string& value, int default_value)
{
  try
  {
    return std::stoi(value, nullptr, 0);
  }
  catch (const std::exception&)
  {
    return default_value;
  }
}

double LinkerHandCanHardware::initial_value_for_joint(const hardware_interface::ComponentInfo& joint)
{
  for (const auto& interface : joint.state_interfaces)
  {
    if (interface.name == hardware_interface::HW_IF_POSITION && !interface.initial_value.empty())
    {
      try
      {
        return std::stod(interface.initial_value);
      }
      catch (const std::exception&)
      {
        return 0.0;
      }
    }
  }
  return 0.0;
}

}  // namespace can_ros2_control
