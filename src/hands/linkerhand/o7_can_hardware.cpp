#include "can_ros2_control/hands/linkerhand/o7_can_hardware.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pluginlib/class_list_macros.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

namespace can_ros2_control
{
namespace
{
constexpr auto kLoggerName = "O7CanHardware";

bool has_suffix(const std::string& value, const std::string& suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

hardware_interface::CallbackReturn O7CanHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "O7 CAN hardware expects exactly %zu joints, got %zu",
      kJointCount, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.reserve(kJointCount);
  for (const auto& joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }
  if (!validate_joint_interfaces())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  load_parameters();
  dispatcher_ = std::make_unique<O7CanReceiveDispatcher>(can_id_);
  upper_limits_ = default_upper_limits(joint_names_);
  lower_limits_.fill(0.0);
  last_raw_command_.fill(0);

  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    hw_positions_[index] = std::clamp(
      initial_value_for_joint(info_.joints[index]),
      lower_limits_[index], upper_limits_[index]);
    hw_commands_[index] = hw_positions_[index];
    hw_velocities_[index] = 0.0;
    last_raw_command_[index] = radians_to_raw(hw_commands_[index], index);
  }

  if (read_tactile_)
  {
    for (std::size_t index = 0; index < kFingerCount; ++index)
    {
      const auto finger = static_cast<Finger>(index);
      const std::string topic =
        "/o7_hand/" + hand_side_ + "/tactile/" + finger_name(finger);
      tactile_publishers_[index] =
        get_node()->create_publisher<std_msgs::msg::UInt8MultiArray>(topic, 10);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured O7 CAN hardware: interface=%s, side=%s, can_id=0x%X, "
    "feedback=%s, tactile=%s",
    can_interface_.c_str(), hand_side_.c_str(), can_id_,
    read_feedback_ ? "true" : "false",
    read_tactile_ ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn O7CanHardware::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  if (!open_socket())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  dispatcher_->reset();
  command_sent_ = false;
  position_feedback_initialized_ = false;
  tactile_request_pending_ = false;
  tactile_batch_complete_.fill(false);
  hw_velocities_.fill(0.0);

  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    hw_commands_[index] = hw_positions_[index];
    last_raw_command_[index] = radians_to_raw(hw_commands_[index], index);
  }

  if (configure_motion_limits_ && !send_motion_limits())
  {
    close_socket();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (send_initial_command_)
  {
    JointValues raw_command{};
    for (std::size_t protocol_index = 0;
         protocol_index < kJointCount; ++protocol_index)
    {
      const auto ros_index = kProtocolToRos[protocol_index];
      raw_command[protocol_index] =
        radians_to_raw(hw_commands_[ros_index], ros_index);
    }
    if (!send_frame(
          O7CanProtocol::make_position_command(can_id_, raw_command)))
    {
      close_socket();
      return hardware_interface::CallbackReturn::ERROR;
    }
    last_raw_command_ = raw_command;
    command_sent_ = true;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "O7 CAN hardware activated on %s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn O7CanHardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  close_socket();
  hw_velocities_.fill(0.0);
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "O7 CAN hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
O7CanHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
  interfaces.reserve(kJointCount * 2);
  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[index], hardware_interface::HW_IF_POSITION,
        &hw_positions_[index]));
    interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[index], hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[index]));
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
O7CanHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
  interfaces.reserve(kJointCount);
  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    interfaces.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        joint_names_[index], hardware_interface::HW_IF_POSITION,
        &hw_commands_[index]));
  }
  return interfaces;
}

hardware_interface::return_type O7CanHardware::read(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& /* period */)
{
  if (socket_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  if (read_feedback_ &&
      !send_frame(O7CanProtocol::make_position_read_request(can_id_)))
  {
    return hardware_interface::return_type::ERROR;
  }

  if (read_tactile_)
  {
    schedule_tactile_request();
  }

  if (!receive_frames())
  {
    return hardware_interface::return_type::ERROR;
  }

  // If the previous five-finger batch completed while draining CAN frames,
  // immediately start the next batch instead of waiting for another cycle.
  if (read_tactile_)
  {
    schedule_tactile_request();
  }

  if (position_feedback_initialized_)
  {
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_position_feedback_time_);
    if (age.count() > velocity_timeout_ms_)
    {
      hw_velocities_.fill(0.0);
    }
  }
  else if (!read_feedback_)
  {
    hw_positions_ = hw_commands_;
    hw_velocities_.fill(0.0);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type O7CanHardware::write(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& /* period */)
{
  if (socket_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  // Do not transmit a position command until the first real position frame
  // initializes both state and command. This prevents a startup jump to the
  // URDF initial value.
  if (read_feedback_ && !position_feedback_initialized_)
  {
    return hardware_interface::return_type::OK;
  }

  JointValues raw_command{};
  for (std::size_t protocol_index = 0;
       protocol_index < kJointCount; ++protocol_index)
  {
    const auto ros_index = kProtocolToRos[protocol_index];
    hw_commands_[ros_index] = std::clamp(
      hw_commands_[ros_index],
      lower_limits_[ros_index], upper_limits_[ros_index]);
    raw_command[protocol_index] =
      radians_to_raw(hw_commands_[ros_index], ros_index);
  }

  if (command_sent_ && !command_changed(raw_command))
  {
    return hardware_interface::return_type::OK;
  }

  if (!send_frame(O7CanProtocol::make_position_command(can_id_, raw_command)))
  {
    return hardware_interface::return_type::ERROR;
  }
  last_raw_command_ = raw_command;
  command_sent_ = true;
  return hardware_interface::return_type::OK;
}

void O7CanHardware::load_parameters()
{
  const auto get = [this](const std::string& name, const std::string& fallback) {
    const auto iterator = info_.hardware_parameters.find(name);
    return iterator == info_.hardware_parameters.end()
             ? fallback : iterator->second;
  };

  can_interface_ = get("can_interface", can_interface_);
  hand_side_ = get("hand_side", hand_side_);
  read_feedback_ = parse_bool(
    get("read_feedback", read_feedback_ ? "true" : "false"), read_feedback_);
  send_initial_command_ = parse_bool(
    get("send_initial_command", send_initial_command_ ? "true" : "false"),
    send_initial_command_);
  configure_motion_limits_ = parse_bool(
    get(
      "configure_motion_limits",
      configure_motion_limits_ ? "true" : "false"),
    configure_motion_limits_);
  read_tactile_ = parse_bool(
    get("read_tactile", read_tactile_ ? "true" : "false"), read_tactile_);
  feedback_timeout_ms_ = std::max(
    0, parse_int(get("feedback_timeout_ms", "2"), feedback_timeout_ms_));
  velocity_timeout_ms_ = std::max(
    1, parse_int(get("velocity_timeout_ms", "100"), velocity_timeout_ms_));
  tactile_timeout_ms_ = std::max(
    1, parse_int(get("tactile_timeout_ms", "100"), tactile_timeout_ms_));
  command_deadband_raw_ = std::clamp(
    parse_int(get("command_deadband_raw", "0"), command_deadband_raw_),
    0, 255);
  max_velocity_raw_ = static_cast<uint8_t>(std::clamp(
    parse_int(get("max_velocity_raw", "255"), 255), 0, 255));
  max_acceleration_raw_ = static_cast<uint8_t>(std::clamp(
    parse_int(get("max_acceleration_raw", "254"), 254), 0, 254));
  velocity_filter_alpha_ = std::clamp(
    parse_double(
      get("velocity_filter_alpha", "0.25"), velocity_filter_alpha_),
    0.0, 1.0);

  const auto can_id = info_.hardware_parameters.find("can_id");
  if (can_id != info_.hardware_parameters.end())
  {
    can_id_ = static_cast<uint32_t>(
      parse_int(can_id->second, static_cast<int>(can_id_)));
  }
  else
  {
    can_id_ = hand_side_ == "left"
                ? O7CanProtocol::kLeftHandCanId
                : O7CanProtocol::kRightHandCanId;
  }
}

bool O7CanHardware::validate_joint_interfaces() const
{
  for (const auto& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name !=
          hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose exactly one position command interface",
        joint.name.c_str());
      return false;
    }

    const auto has_state = [&joint](const std::string& name) {
      return std::any_of(
        joint.state_interfaces.begin(), joint.state_interfaces.end(),
        [&name](const auto& interface) {
          return interface.name == name;
        });
    };
    if (!has_state(hardware_interface::HW_IF_POSITION) ||
        !has_state(hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose position and velocity state interfaces",
        joint.name.c_str());
      return false;
    }
    if (has_state(hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must not expose effort: O7 reports unsigned magnitude only",
        joint.name.c_str());
      return false;
    }
  }
  return true;
}

bool O7CanHardware::open_socket()
{
  close_socket();
  socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to create CAN socket: %s", std::strerror(errno));
    return false;
  }

  struct ifreq interface_request;
  std::memset(&interface_request, 0, sizeof(interface_request));
  std::strncpy(
    interface_request.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &interface_request) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to find CAN interface '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    close_socket();
    return false;
  }

  struct can_filter filter;
  filter.can_id = can_id_;
  filter.can_mask = CAN_SFF_MASK;
  if (setsockopt(
        socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER,
        &filter, sizeof(filter)) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to configure CAN filter: %s", std::strerror(errno));
    close_socket();
    return false;
  }

  struct sockaddr_can address;
  std::memset(&address, 0, sizeof(address));
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;
  if (bind(
        socket_fd_, reinterpret_cast<struct sockaddr*>(&address),
        sizeof(address)) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to bind CAN interface '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
    close_socket();
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Opened %s for O7 standard CAN ID 0x%X",
    can_interface_.c_str(), can_id_);
  return true;
}

void O7CanHardware::close_socket()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool O7CanHardware::send_frame(const Frame& source)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = source.can_id;
  frame.can_dlc = source.dlc;
  std::copy_n(source.data.begin(), source.dlc, frame.data);

  const auto written = ::write(socket_fd_, &frame, sizeof(frame));
  if (written != static_cast<ssize_t>(sizeof(frame)))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to write O7 CAN frame 0x%02X to ID 0x%X: %s",
      source.data[0], source.can_id, std::strerror(errno));
    return false;
  }
  return true;
}

bool O7CanHardware::receive_frames()
{
  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(feedback_timeout_ms_);

  while (true)
  {
    struct can_frame socket_frame;
    const auto bytes = ::read(socket_fd_, &socket_frame, sizeof(socket_frame));
    if (bytes == static_cast<ssize_t>(sizeof(socket_frame)))
    {
      if ((socket_frame.can_id & CAN_EFF_FLAG) != 0 ||
          (socket_frame.can_id & CAN_SFF_MASK) != can_id_)
      {
        continue;
      }

      Frame frame;
      frame.can_id = socket_frame.can_id & CAN_SFF_MASK;
      frame.dlc = std::min<uint8_t>(socket_frame.can_dlc, 8);
      std::copy_n(socket_frame.data, frame.dlc, frame.data.begin());
      const auto received_time = std::chrono::steady_clock::now();
      handle_dispatch_result(dispatcher_->process(frame), received_time);
      continue;
    }

    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      if (feedback_timeout_ms_ == 0 ||
          std::chrono::steady_clock::now() >= deadline)
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (bytes < 0 && errno == EINTR)
    {
      continue;
    }
    if (bytes < 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Failed to read O7 CAN frame: %s", std::strerror(errno));
      return false;
    }
    break;
  }
  return true;
}

bool O7CanHardware::send_motion_limits()
{
  JointValues velocities;
  velocities.fill(max_velocity_raw_);
  if (!send_frame(
        O7CanProtocol::make_max_velocity_command(can_id_, velocities)))
  {
    return false;
  }

  JointValues accelerations;
  accelerations.fill(max_acceleration_raw_);
  return send_frame(
    O7CanProtocol::make_max_acceleration_command(can_id_, accelerations));
}

void O7CanHardware::schedule_tactile_request()
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
    const auto finger = static_cast<Finger>(index);
    dispatcher_->reset_tactile(finger);
  }

  for (std::size_t index = 0; index < kFingerCount; ++index)
  {
    const auto finger = static_cast<Finger>(index);
    if (!send_frame(
          O7CanProtocol::make_tactile_read_request(can_id_, finger)))
    {
      return;
    }
  }
  tactile_request_time_ = now;
  tactile_request_pending_ = true;
}

void O7CanHardware::handle_dispatch_result(
  const O7CanReceiveDispatcher::DispatchResult& result,
  const SteadyTime& received_time)
{
  if (!result.accepted)
  {
    return;
  }

  if (result.type == O7CanProtocol::MessageType::kPosition)
  {
    update_position_and_velocity(dispatcher_->position(), received_time);
  }
  if (result.tactile_matrix_complete)
  {
    const auto index = static_cast<std::size_t>(result.tactile_finger);
    if (index < tactile_batch_complete_.size() &&
        !tactile_batch_complete_[index])
    {
      tactile_batch_complete_[index] = true;
      publish_tactile(result.tactile_finger);
    }
    if (std::all_of(
          tactile_batch_complete_.begin(),
          tactile_batch_complete_.end(),
          [](bool complete) { return complete; }))
    {
      tactile_request_pending_ = false;
    }
  }
}

void O7CanHardware::update_position_and_velocity(
  const JointValues& raw_positions,
  const SteadyTime& received_time)
{
  std::array<double, kJointCount> positions{};
  for (std::size_t protocol_index = 0;
       protocol_index < kJointCount; ++protocol_index)
  {
    const auto ros_index = kProtocolToRos[protocol_index];
    positions[ros_index] =
      raw_to_radians(raw_positions[protocol_index], ros_index);
  }

  if (!position_feedback_initialized_)
  {
    hw_positions_ = positions;
    hw_commands_ = positions;
    hw_velocities_.fill(0.0);
    position_feedback_initialized_ = true;
    last_position_feedback_time_ = received_time;
    return;
  }

  const double dt = std::chrono::duration<double>(
    received_time - last_position_feedback_time_).count();
  if (dt > std::numeric_limits<double>::epsilon())
  {
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
      const double measured =
        (positions[index] - hw_positions_[index]) / dt;
      hw_velocities_[index] =
        velocity_filter_alpha_ * measured +
        (1.0 - velocity_filter_alpha_) * hw_velocities_[index];
    }
  }
  hw_positions_ = positions;
  last_position_feedback_time_ = received_time;
}

void O7CanHardware::publish_tactile(Finger finger)
{
  const auto index = static_cast<std::size_t>(finger);
  if (index >= tactile_publishers_.size() || !tactile_publishers_[index])
  {
    return;
  }

  std_msgs::msg::UInt8MultiArray message;
  message.layout.dim.resize(2);
  message.layout.dim[0].label = "row";
  message.layout.dim[0].size = O7CanProtocol::kTactileRows;
  message.layout.dim[0].stride = O7CanProtocol::kTactilePointCount;
  message.layout.dim[1].label = "column";
  message.layout.dim[1].size = O7CanProtocol::kTactileColumns;
  message.layout.dim[1].stride = O7CanProtocol::kTactileColumns;
  message.data.reserve(O7CanProtocol::kTactilePointCount);
  for (const auto& row : dispatcher_->tactile_matrix(finger))
  {
    message.data.insert(message.data.end(), row.begin(), row.end());
  }
  tactile_publishers_[index]->publish(message);
}

uint8_t O7CanHardware::radians_to_raw(
  double radians,
  std::size_t ros_joint_index) const
{
  const double lower = lower_limits_[ros_joint_index];
  const double upper = upper_limits_[ros_joint_index];
  if (upper <= lower)
  {
    return O7CanProtocol::kMaxJointRaw;
  }
  const double normalized = std::clamp(
    (radians - lower) / (upper - lower), 0.0, 1.0);
  const long raw = std::lround(255.0 * (1.0 - normalized));
  return static_cast<uint8_t>(std::clamp<long>(raw, 0, 255));
}

double O7CanHardware::raw_to_radians(
  uint8_t raw,
  std::size_t ros_joint_index) const
{
  const double lower = lower_limits_[ros_joint_index];
  const double upper = upper_limits_[ros_joint_index];
  if (upper <= lower)
  {
    return lower;
  }
  const double normalized = 1.0 - static_cast<double>(raw) / 255.0;
  return lower + normalized * (upper - lower);
}

bool O7CanHardware::command_changed(const JointValues& command) const
{
  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    if (std::abs(
          static_cast<int>(command[index]) -
          static_cast<int>(last_raw_command_[index])) >
        command_deadband_raw_)
    {
      return true;
    }
  }
  return false;
}

bool O7CanHardware::parse_bool(
  const std::string& value,
  bool default_value)
{
  if (value == "true" || value == "1" ||
      value == "True" || value == "TRUE")
  {
    return true;
  }
  if (value == "false" || value == "0" ||
      value == "False" || value == "FALSE")
  {
    return false;
  }
  return default_value;
}

int O7CanHardware::parse_int(
  const std::string& value,
  int default_value)
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

double O7CanHardware::parse_double(
  const std::string& value,
  double default_value)
{
  try
  {
    return std::stod(value);
  }
  catch (const std::exception&)
  {
    return default_value;
  }
}

double O7CanHardware::initial_value_for_joint(
  const hardware_interface::ComponentInfo& joint)
{
  for (const auto& interface : joint.state_interfaces)
  {
    if (interface.name == hardware_interface::HW_IF_POSITION &&
        !interface.initial_value.empty())
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

std::array<double, O7CanHardware::kJointCount>
O7CanHardware::default_upper_limits(
  const std::vector<std::string>& joint_names)
{
  std::array<double, kJointCount> limits{
    0.5146, 1.9189, 1.1339, 1.3607, 1.3607, 1.3607, 1.3607};
  for (std::size_t index = 0;
       index < std::min(kJointCount, joint_names.size()); ++index)
  {
    const auto& name = joint_names[index];
    if (has_suffix(name, "thumb_joint1"))
    {
      limits[index] = 0.5146;
    }
    else if (has_suffix(name, "thumb_joint2"))
    {
      limits[index] = 1.9189;
    }
    else if (has_suffix(name, "thumb_joint3"))
    {
      limits[index] = 1.1339;
    }
    else
    {
      limits[index] = 1.3607;
    }
  }
  return limits;
}

const char* O7CanHardware::finger_name(Finger finger)
{
  switch (finger)
  {
    case Finger::kThumb:
      return "thumb";
    case Finger::kIndex:
      return "index";
    case Finger::kMiddle:
      return "middle";
    case Finger::kRing:
      return "ring";
    case Finger::kPinky:
      return "pinky";
  }
  return "unknown";
}

}  // namespace can_ros2_control

PLUGINLIB_EXPORT_CLASS(
  can_ros2_control::O7CanHardware,
  hardware_interface::SystemInterface)
