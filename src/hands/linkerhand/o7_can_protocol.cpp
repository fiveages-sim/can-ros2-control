#include "can_ros2_control/hands/linkerhand/o7_can_protocol.h"

#include <algorithm>
#include <stdexcept>

namespace can_ros2_control
{
namespace
{

bool has_expected_header(
  const O7CanProtocol::Frame& frame,
  uint32_t expected_can_id,
  uint8_t expected_command,
  uint8_t expected_dlc)
{
  return frame.can_id == expected_can_id &&
         frame.dlc == expected_dlc &&
         frame.data[0] == expected_command;
}

O7CanProtocol::Frame make_joint_frame(
  uint32_t can_id,
  uint8_t command,
  const O7CanProtocol::JointValues& values)
{
  O7CanProtocol::Frame frame;
  frame.can_id = can_id;
  frame.dlc = O7CanProtocol::kPositionWriteDlc;
  frame.data[0] = command;
  std::copy(values.begin(), values.end(), frame.data.begin() + 1);
  return frame;
}

bool parse_joint_frame(
  const O7CanProtocol::Frame& frame,
  uint32_t expected_can_id,
  uint8_t expected_command,
  O7CanProtocol::JointValues& values)
{
  if (!has_expected_header(
        frame, expected_can_id, expected_command,
        O7CanProtocol::kPositionWriteDlc))
  {
    return false;
  }

  std::copy(frame.data.begin() + 1, frame.data.end(), values.begin());
  return true;
}

}  // namespace

O7CanProtocol::Frame O7CanProtocol::make_position_command(
  uint32_t can_id,
  const JointValues& positions)
{
  return make_joint_frame(can_id, kPositionCommand, positions);
}

O7CanProtocol::Frame O7CanProtocol::make_position_read_request(uint32_t can_id)
{
  Frame frame;
  frame.can_id = can_id;
  frame.dlc = kPositionReadDlc;
  frame.data[0] = kPositionCommand;
  return frame;
}

O7CanProtocol::Frame O7CanProtocol::make_max_velocity_command(
  uint32_t can_id,
  const JointValues& max_velocities)
{
  return make_joint_frame(can_id, kMaxVelocityCommand, max_velocities);
}

O7CanProtocol::Frame O7CanProtocol::make_max_acceleration_command(
  uint32_t can_id,
  const JointValues& max_accelerations)
{
  JointValues clamped = max_accelerations;
  for (uint8_t& value : clamped)
  {
    value = std::min(value, kMaxAccelerationRaw);
  }
  return make_joint_frame(can_id, kMaxAccelerationCommand, clamped);
}

O7CanProtocol::Frame O7CanProtocol::make_tactile_read_request(
  uint32_t can_id,
  Finger finger)
{
  Frame frame;
  frame.can_id = can_id;
  frame.dlc = kTactileRequestDlc;
  frame.data[0] = tactile_command(finger);
  frame.data[1] = kTactileLayout12x6;
  return frame;
}

O7CanProtocol::MessageType O7CanProtocol::classify(const Frame& frame)
{
  if (frame.dlc == 0)
  {
    return MessageType::kUnknown;
  }

  switch (frame.data[0])
  {
    case kPositionCommand:
      return MessageType::kPosition;
    case kMaxVelocityCommand:
      return MessageType::kMaxVelocityReply;
    case kMaxAccelerationCommand:
      return MessageType::kMaxAcceleration;
    case kThumbTactileCommand:
      return MessageType::kThumbTactile;
    case kIndexTactileCommand:
      return MessageType::kIndexTactile;
    case kMiddleTactileCommand:
      return MessageType::kMiddleTactile;
    case kRingTactileCommand:
      return MessageType::kRingTactile;
    case kPinkyTactileCommand:
      return MessageType::kPinkyTactile;
    default:
      return MessageType::kUnknown;
  }
}

bool O7CanProtocol::parse_position_feedback(
  const Frame& frame,
  uint32_t expected_can_id,
  JointValues& positions)
{
  return parse_joint_frame(
    frame, expected_can_id, kPositionCommand, positions);
}

bool O7CanProtocol::parse_max_velocity_reply(
  const Frame& frame,
  uint32_t expected_can_id,
  JointValues& max_velocities)
{
  return parse_joint_frame(
    frame, expected_can_id, kMaxVelocityCommand, max_velocities);
}

bool O7CanProtocol::parse_max_acceleration_reply(
  const Frame& frame,
  uint32_t expected_can_id,
  JointValues& max_accelerations)
{
  return parse_joint_frame(
    frame, expected_can_id, kMaxAccelerationCommand,
    max_accelerations);
}

bool O7CanProtocol::parse_tactile_reply(
  const Frame& frame,
  uint32_t expected_can_id,
  Finger expected_finger,
  TactileReply& reply)
{
  if (!has_expected_header(
        frame, expected_can_id, tactile_command(expected_finger),
        kTactileResponseDlc))
  {
    return false;
  }

  const uint8_t coordinate = frame.data[1];
  const uint8_t row = static_cast<uint8_t>((coordinate >> 4) & 0x0F);
  const uint8_t column = static_cast<uint8_t>(coordinate & 0x0F);
  if (row >= kTactileRows ||
      column >= kTactileColumns ||
      static_cast<std::size_t>(column) + kTactileValuesPerFrame >
        kTactileColumns)
  {
    return false;
  }

  reply.finger = expected_finger;
  reply.row = row;
  reply.column = column;
  std::copy(
    frame.data.begin() + 2,
    frame.data.end(),
    reply.values.begin());
  return true;
}

bool O7CanProtocol::finger_from_tactile_command(
  uint8_t command,
  Finger& finger)
{
  const auto it = std::find(
    kTactileCommands.begin(), kTactileCommands.end(), command);
  if (it == kTactileCommands.end())
  {
    return false;
  }

  finger = static_cast<Finger>(
    std::distance(kTactileCommands.begin(), it));
  return true;
}

O7CanReceiveDispatcher::O7CanReceiveDispatcher(uint32_t expected_can_id)
: expected_can_id_(expected_can_id)
{
  reset();
}

void O7CanReceiveDispatcher::reset()
{
  position_.fill(0);
  max_velocity_reply_.fill(0);
  max_acceleration_reply_.fill(0);
  has_position_ = false;
  has_max_velocity_reply_ = false;
  has_max_acceleration_reply_ = false;

  for (std::size_t index = 0;
       index < O7CanProtocol::kFingerCount;
       ++index)
  {
    reset_tactile(static_cast<Finger>(index));
  }
}

void O7CanReceiveDispatcher::reset_tactile(Finger finger)
{
  TactileState& state = tactile_states_.at(finger_index(finger));
  for (auto& row : state.matrix)
  {
    row.fill(0);
  }
  state.received_rows.fill(false);
  state.received_frame_count = 0;
}

O7CanReceiveDispatcher::DispatchResult O7CanReceiveDispatcher::process(
  const Frame& frame)
{
  DispatchResult result;
  result.type = O7CanProtocol::classify(frame);

  if (frame.can_id != expected_can_id_)
  {
    return result;
  }

  switch (result.type)
  {
    case MessageType::kPosition:
      result.accepted = O7CanProtocol::parse_position_feedback(
        frame, expected_can_id_, position_);
      if (result.accepted)
      {
        has_position_ = true;
      }
      break;

    case MessageType::kMaxVelocityReply:
      result.accepted = O7CanProtocol::parse_max_velocity_reply(
        frame, expected_can_id_, max_velocity_reply_);
      if (result.accepted)
      {
        has_max_velocity_reply_ = true;
      }
      break;

    case MessageType::kMaxAcceleration:
      result.accepted = O7CanProtocol::parse_max_acceleration_reply(
        frame, expected_can_id_, max_acceleration_reply_);
      if (result.accepted)
      {
        has_max_acceleration_reply_ = true;
      }
      break;

    case MessageType::kThumbTactile:
    case MessageType::kIndexTactile:
    case MessageType::kMiddleTactile:
    case MessageType::kRingTactile:
    case MessageType::kPinkyTactile:
    {
      Finger finger;
      if (!O7CanProtocol::finger_from_tactile_command(
            frame.data[0], finger))
      {
        break;
      }

      O7CanProtocol::TactileReply reply;
      if (!O7CanProtocol::parse_tactile_reply(
            frame, expected_can_id_, finger, reply))
      {
        break;
      }

      result.tactile_finger = finger;
      result.accepted = store_tactile_reply(
        reply, result.tactile_matrix_complete);
      break;
    }

    case MessageType::kUnknown:
      break;
  }

  return result;
}

bool O7CanReceiveDispatcher::tactile_complete(Finger finger) const
{
  return tactile_states_.at(finger_index(finger)).received_frame_count ==
         O7CanProtocol::kTactileFramesPerRequest;
}

std::size_t O7CanReceiveDispatcher::tactile_frame_count(Finger finger) const
{
  return tactile_states_.at(finger_index(finger)).received_frame_count;
}

const O7CanReceiveDispatcher::TactileMatrix&
O7CanReceiveDispatcher::tactile_matrix(Finger finger) const
{
  return tactile_states_.at(finger_index(finger)).matrix;
}

bool O7CanReceiveDispatcher::store_tactile_reply(
  const O7CanProtocol::TactileReply& reply,
  bool& matrix_complete)
{
  TactileState& state = tactile_states_.at(finger_index(reply.finger));
  if (reply.row >= O7CanProtocol::kTactileRows ||
      reply.column >= O7CanProtocol::kTactileColumns ||
      static_cast<std::size_t>(reply.column) +
        O7CanProtocol::kTactileValuesPerFrame >
        O7CanProtocol::kTactileColumns)
  {
    matrix_complete = false;
    return false;
  }

  std::copy(
    reply.values.begin(),
    reply.values.end(),
    state.matrix[reply.row].begin() + reply.column);

  if (!state.received_rows[reply.row])
  {
    state.received_rows[reply.row] = true;
    ++state.received_frame_count;
  }

  matrix_complete =
    state.received_frame_count ==
    O7CanProtocol::kTactileFramesPerRequest;
  return true;
}

}  // namespace can_ros2_control
