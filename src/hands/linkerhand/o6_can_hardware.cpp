#include "can_ros2_control/hands/linkerhand/o6_can_hardware.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
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
constexpr auto kLoggerName = "O6CanHardware";

bool has_suffix(const std::string& value, const std::string& suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

hardware_interface::CallbackReturn O6CanHardware::on_init(
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
      "O6 CAN hardware expects exactly %zu joints, got %zu",
      kJointCount,
      info_.joints.size());
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
  upper_limits_ = default_upper_limits(joint_names_);

  for (std::size_t i = 0; i < kJointCount; ++i)
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
        "/o6_hand/" + hand_side_ + "/tactile/" + finger_name(finger);
      tactile_publishers_[index] =
        get_node()->create_publisher<std_msgs::msg::UInt8MultiArray>(topic, 10);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured O6 CAN hardware: interface=%s, side=%s, can_id=0x%X, "
    "feedback=%s, tactile=%s",
    can_interface_.c_str(),
    hand_side_.c_str(),
    can_id_,
    read_feedback_ ? "true" : "false",
    read_tactile_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn O6CanHardware::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  if (!open_socket())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_sent_ = false;
  position_feedback_initialized_ = false;
  tactile_request_pending_ = false;
  tactile_batch_complete_.fill(false);
  for (std::size_t index = 0; index < kFingerCount; ++index)
  {
    reset_tactile(static_cast<Finger>(index));
  }
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_commands_[i] = hw_positions_[i];
    last_raw_command_[i] = radians_to_raw(hw_commands_[i], i);
  }

  if (send_initial_command_)
  {
    std::array<uint8_t, kJointCount> raw_command{};
    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      raw_command[i] = radians_to_raw(hw_commands_[i], i);
    }
    if (!send_command(raw_command))
    {
      close_socket();
      return hardware_interface::CallbackReturn::ERROR;
    }
    last_raw_command_ = raw_command;
    command_sent_ = true;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "O6 CAN hardware activated on %s",
    can_interface_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn O6CanHardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  close_socket();
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "O6 CAN hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
O6CanHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.reserve(kJointCount * 3);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
O6CanHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.reserve(kJointCount);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    command_interfaces.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type O6CanHardware::read(
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

  if (read_tactile_)
  {
    schedule_tactile_request();
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

  if (read_tactile_)
  {
    schedule_tactile_request();
  }

  const double dt = period.seconds();
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_efforts_[i] = 0.0;
    hw_velocities_[i] = dt > std::numeric_limits<double>::epsilon()
                          ? (hw_positions_[i] - previous_positions_[i]) / dt
                          : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type O6CanHardware::write(
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

  std::array<uint8_t, kJointCount> raw_command{};
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_commands_[i] = std::clamp(hw_commands_[i], lower_limits_[i], upper_limits_[i]);
    raw_command[i] = radians_to_raw(hw_commands_[i], i);
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

void O6CanHardware::load_parameters()
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

bool O6CanHardware::validate_joint_interfaces() const
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
        !has_state_interface(hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose position, velocity, and effort state interfaces",
        joint.name.c_str());
      return false;
    }
  }

  return true;
}

bool O6CanHardware::open_socket()
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

void O6CanHardware::close_socket()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool O6CanHardware::receive_feedback(const rclcpp::Duration& period)
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
      if (frame_id == can_id_ && frame.can_dlc >= kJointCount + 1 && frame.data[0] == kAngleCommand)
      {
        for (std::size_t i = 0; i < kJointCount; ++i)
        {
          hw_positions_[i] = raw_to_radians(frame.data[i + 1], i);
        }
        if (!position_feedback_initialized_)
        {
          // Seed both the command and velocity baseline from hardware. The
          // controller therefore starts by holding the measured pose.
          hw_commands_ = hw_positions_;
          previous_positions_ = hw_positions_;
          hw_velocities_.fill(0.0);
          for (std::size_t i = 0; i < kJointCount; ++i)
          {
            last_raw_command_[i] = frame.data[i + 1];
          }
          position_feedback_initialized_ = true;
          RCLCPP_INFO(
            rclcpp::get_logger(kLoggerName),
            "Initialized O6 commands from the first position feedback frame");
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
      "No O6 CAN feedback frame received for CAN ID 0x%X",
      can_id_);
  }

  return true;
}

bool O6CanHardware::send_position_read_request()
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
      "Failed to write O6 position request to ID 0x%X: %s",
      can_id_, std::strerror(errno));
    return false;
  }
  return true;
}

bool O6CanHardware::send_tactile_request(Finger finger)
{
  auto request = O7CanProtocol::make_tactile_read_request(can_id_, finger);
  request.data[1] = kTactileLayout10x4;
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
      "Failed to write O6 tactile request 0x%02X to ID 0x%X: %s",
      request.data[0], request.can_id, std::strerror(errno));
    return false;
  }
  return true;
}

void O6CanHardware::schedule_tactile_request()
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

bool O6CanHardware::process_tactile_frame(const struct can_frame& socket_frame)
{
  O7CanProtocol::Frame frame;
  frame.can_id = socket_frame.can_id & CAN_SFF_MASK;
  frame.dlc = std::min<uint8_t>(socket_frame.can_dlc, 8);
  std::copy_n(socket_frame.data, frame.dlc, frame.data.begin());

  Finger finger;
  if (!O7CanProtocol::finger_from_tactile_command(frame.data[0], finger))
  {
    return false;
  }

  if (frame.can_id != can_id_ || frame.dlc != kTactileResponseDlc)
  {
    return false;
  }

  const uint8_t coordinate = frame.data[1];
  const uint8_t row = static_cast<uint8_t>((coordinate >> 4) & 0x0F);
  const uint8_t column = static_cast<uint8_t>(coordinate & 0x0F);
  if (row >= kTactileRows || column >= kTactileColumns ||
      static_cast<std::size_t>(column) + kTactileValuesPerFrame >
        kTactileColumns)
  {
    return false;
  }

  const auto finger_index = static_cast<std::size_t>(finger);
  auto& state = tactile_states_[finger_index];
  std::copy_n(
    frame.data.begin() + 2, kTactileValuesPerFrame,
    state.matrix[row].begin() + column);
  if (!state.received_rows[row])
  {
    state.received_rows[row] = true;
    ++state.received_frame_count;
  }

  if (state.received_frame_count == kTactileFramesPerRequest &&
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

void O6CanHardware::reset_tactile(Finger finger)
{
  auto& state = tactile_states_[static_cast<std::size_t>(finger)];
  for (auto& row : state.matrix)
  {
    row.fill(0);
  }
  state.received_rows.fill(false);
  state.received_frame_count = 0;
}

void O6CanHardware::publish_tactile(Finger finger)
{
  const auto index = static_cast<std::size_t>(finger);
  if (!tactile_publishers_[index])
  {
    return;
  }

  std_msgs::msg::UInt8MultiArray message;
  message.layout.dim.resize(2);
  message.layout.dim[0].label = "row";
  message.layout.dim[0].size = kTactileRows;
  message.layout.dim[0].stride = kTactilePointCount;
  message.layout.dim[1].label = "column";
  message.layout.dim[1].size = kTactileColumns;
  message.layout.dim[1].stride = kTactileColumns;
  message.data.reserve(kTactilePointCount);
  for (const auto& row : tactile_states_[index].matrix)
  {
    message.data.insert(message.data.end(), row.begin(), row.end());
  }
  tactile_publishers_[index]->publish(message);
}

const char* O6CanHardware::finger_name(Finger finger)
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

bool O6CanHardware::send_command(const std::array<uint8_t, kJointCount>& raw_command)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id_;
  frame.can_dlc = kJointCount + 1;
  frame.data[0] = kAngleCommand;

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    frame.data[i + 1] = raw_command[i];
  }

  const auto bytes_written = ::write(socket_fd_, &frame, sizeof(frame));
  if (bytes_written != static_cast<ssize_t>(sizeof(frame)))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to write O6 CAN command to ID 0x%X: %s",
      can_id_,
      std::strerror(errno));
    return false;
  }

  return true;
}

uint8_t O6CanHardware::radians_to_raw(double radians, std::size_t joint_index) const
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

double O6CanHardware::raw_to_radians(uint8_t raw, std::size_t joint_index) const
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

bool O6CanHardware::raw_command_changed(const std::array<uint8_t, kJointCount>& raw_command) const
{
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    if (std::abs(static_cast<int>(raw_command[i]) - static_cast<int>(last_raw_command_[i])) >
        command_deadband_raw_)
    {
      return true;
    }
  }
  return false;
}

bool O6CanHardware::parse_bool(const std::string& value, bool default_value)
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

int O6CanHardware::parse_int(const std::string& value, int default_value)
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

double O6CanHardware::initial_value_for_joint(const hardware_interface::ComponentInfo& joint)
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

std::array<double, O6CanHardware::kJointCount> O6CanHardware::default_upper_limits(
  const std::vector<std::string>& joint_names)
{
  std::array<double, kJointCount> limits{0.58, 1.30, 1.60, 1.60, 1.60, 1.60};

  for (std::size_t i = 0; i < std::min(kJointCount, joint_names.size()); ++i)
  {
    const auto& name = joint_names[i];
    if (has_suffix(name, "thumb_joint1"))
    {
      limits[i] = 0.58;
    }
    else if (has_suffix(name, "thumb_joint2"))
    {
      limits[i] = 1.30;
    }
    else
    {
      limits[i] = 1.60;
    }
  }

  return limits;
}

}  // namespace can_ros2_control

PLUGINLIB_EXPORT_CLASS(can_ros2_control::O6CanHardware, hardware_interface::SystemInterface)
