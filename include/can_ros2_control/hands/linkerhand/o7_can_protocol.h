#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace can_ros2_control
{

/**
 * @brief LinkerHand O7 CAN protocol (protocol version 3.3).
 *
 * CAN bus:
 *   - bitrate: 1 Mbit/s
 *   - right hand CAN ID: 0x27
 *   - left hand CAN ID: 0x28
 *
 * A position read request contains only the command byte (DLC = 1).
 * A position read request contains only the command byte (DLC = 1).
 * Velocity and acceleration are write-only limits in this interface. The
 * hardware layer derives signed joint velocity from consecutive position
 * samples instead of requesting the protocol's unsigned velocity values.
 *
 * Transmission and reception are asynchronous. The receive loop must inspect
 * data[0] of every incoming frame and dispatch it to the matching parser; no
 * send-and-wait-for-one-reply transaction is assumed.
 */
class O7CanProtocol
{
public:
  static constexpr std::size_t kJointCount = 7;
  static constexpr std::size_t kFingerCount = 5;

  static constexpr uint32_t kRightHandCanId = 0x27;
  static constexpr uint32_t kLeftHandCanId = 0x28;
  static constexpr int kCanBitrate = 1000000;

  static constexpr uint8_t kPositionCommand = 0x01;
  static constexpr uint8_t kMaxVelocityCommand = 0x05;
  static constexpr uint8_t kMaxAccelerationCommand = 0x87;

  static constexpr uint8_t kThumbTactileCommand = 0xB1;
  static constexpr uint8_t kIndexTactileCommand = 0xB2;
  static constexpr uint8_t kMiddleTactileCommand = 0xB3;
  static constexpr uint8_t kRingTactileCommand = 0xB4;
  static constexpr uint8_t kPinkyTactileCommand = 0xB5;

  static constexpr uint8_t kPositionReadDlc = 1;
  static constexpr uint8_t kPositionWriteDlc = 8;
  static constexpr uint8_t kLimitWriteDlc = 8;

  // 0xC6 selects a 12-row x 6-column tactile matrix.
  static constexpr uint8_t kTactileLayout12x6 = 0xC6;
  static constexpr uint8_t kTactileRequestDlc = 2;
  static constexpr uint8_t kTactileResponseDlc = 8;
  static constexpr std::size_t kTactileRows = 12;
  static constexpr std::size_t kTactileColumns = 6;
  static constexpr std::size_t kTactileFramesPerRequest = 12;
  static constexpr std::size_t kTactileValuesPerFrame = 6;
  static constexpr std::size_t kTactilePointCount =
    kTactileRows * kTactileColumns;

  static constexpr uint8_t kMaxJointRaw = 0xFF;
  static constexpr uint8_t kMaxAccelerationRaw = 0xFE;

  enum class Finger : uint8_t
  {
    kThumb = 0,
    kIndex,
    kMiddle,
    kRing,
    kPinky,
  };

  enum class MessageType : uint8_t
  {
    kPosition,
    kMaxVelocityReply,
    kMaxAcceleration,
    kThumbTactile,
    kIndexTactile,
    kMiddleTactile,
    kRingTactile,
    kPinkyTactile,
    kUnknown,
  };

  struct Frame
  {
    uint32_t can_id{0};
    uint8_t dlc{0};
    std::array<uint8_t, 8> data{};
  };

  using JointValues = std::array<uint8_t, kJointCount>;
  using TactileMatrix =
    std::array<std::array<uint8_t, kTactileColumns>, kTactileRows>;

  struct TactileReply
  {
    Finger finger{Finger::kThumb};
    uint8_t row{0};
    uint8_t column{0};
    std::array<uint8_t, kTactileValuesPerFrame> values{};
  };

  /**
   * @brief Build a seven-joint position command.
   *
   * Payload: [0x01, joint1, ..., joint7], DLC = 8.
   */
  static Frame make_position_command(
    uint32_t can_id,
    const JointValues& positions);

  /**
   * @brief Build a current-position read request.
   *
   * Payload: [0x01], DLC = 1. No padding bytes are transmitted.
   */
  static Frame make_position_read_request(uint32_t can_id);

  /**
   * @brief Build the per-joint maximum-velocity setting.
   *
   * Payload: [0x05, joint1, ..., joint7], DLC = 8.
   */
  static Frame make_max_velocity_command(
    uint32_t can_id,
    const JointValues& max_velocities);

  /**
   * @brief Build the per-joint maximum-acceleration setting.
   *
   * Payload: [0x87, joint1, ..., joint7], DLC = 8.
   * Each acceleration value must be in [0x00, 0xFE].
   */
  static Frame make_max_acceleration_command(
    uint32_t can_id,
    const JointValues& max_accelerations);

  /**
   * @brief Build a 12x6 tactile-matrix request.
   *
   * Payload: [0xB1..0xB5, 0xC6], DLC = 2.
   * One request is expected to produce exactly 12 response frames.
   */
  static Frame make_tactile_read_request(
    uint32_t can_id,
    Finger finger);

  /**
   * @brief Classify an incoming frame solely from data[0].
   *
   * CAN ID and DLC validation is performed by the corresponding parser.
   */
  static MessageType classify(const Frame& frame);

  /**
   * @brief Parse [0x01, joint1, ..., joint7] position feedback.
   */
  static bool parse_position_feedback(
    const Frame& frame,
    uint32_t expected_can_id,
    JointValues& positions);

  /**
   * @brief Parse the echo of a maximum-velocity setting.
   *
   * This acknowledgement is not exposed as real-time joint velocity.
   */
  static bool parse_max_velocity_reply(
    const Frame& frame,
    uint32_t expected_can_id,
    JointValues& max_velocities);

  /**
   * @brief Parse the echoed values from a maximum-acceleration setting reply.
   *
   * This is an acknowledgement parser, not an acceleration read command.
   */
  static bool parse_max_acceleration_reply(
    const Frame& frame,
    uint32_t expected_can_id,
    JointValues& max_accelerations);

  /**
   * @brief Parse one tactile response frame.
   *
   * Response payload:
   *   [finger command, start coordinate, value1, ..., value6]
   *
   * The coordinate high nibble is the row and the low nibble is the column.
   */
  static bool parse_tactile_reply(
    const Frame& frame,
    uint32_t expected_can_id,
    Finger expected_finger,
    TactileReply& reply);

  static constexpr uint8_t tactile_command(Finger finger)
  {
    return kTactileCommands[static_cast<std::size_t>(finger)];
  }

  static bool finger_from_tactile_command(
    uint8_t command,
    Finger& finger);

private:
  static constexpr std::array<uint8_t, kFingerCount> kTactileCommands{
    kThumbTactileCommand,
    kIndexTactileCommand,
    kMiddleTactileCommand,
    kRingTactileCommand,
    kPinkyTactileCommand,
  };
};

/**
 * @brief Dispatch and store asynchronously received O7 CAN frames.
 *
 * process() is called once for every frame read from the CAN socket. It uses
 * data[0] to select the parser and updates the corresponding cached data.
 * Position, setting acknowledgements, and tactile frames may arrive in any
 * order. Maximum-velocity replies are acknowledged but are never exposed as
 * real-time joint velocity.
 */
class O7CanReceiveDispatcher
{
public:
  using Finger = O7CanProtocol::Finger;
  using Frame = O7CanProtocol::Frame;
  using JointValues = O7CanProtocol::JointValues;
  using MessageType = O7CanProtocol::MessageType;
  using TactileMatrix = O7CanProtocol::TactileMatrix;

  struct DispatchResult
  {
    MessageType type{MessageType::kUnknown};
    bool accepted{false};
    bool tactile_matrix_complete{false};
    Finger tactile_finger{Finger::kThumb};
  };

  explicit O7CanReceiveDispatcher(
    uint32_t expected_can_id = O7CanProtocol::kRightHandCanId);

  void reset();
  void reset_tactile(Finger finger);

  /**
   * @brief Dispatch one received frame according to frame.data[0].
   */
  DispatchResult process(const Frame& frame);

  bool has_position() const
  {
    return has_position_;
  }

  const JointValues& position() const
  {
    return position_;
  }

  bool has_max_velocity_reply() const
  {
    return has_max_velocity_reply_;
  }

  const JointValues& max_velocity_reply() const
  {
    return max_velocity_reply_;
  }

  bool has_max_acceleration_reply() const
  {
    return has_max_acceleration_reply_;
  }

  const JointValues& max_acceleration_reply() const
  {
    return max_acceleration_reply_;
  }

  bool tactile_complete(Finger finger) const;
  std::size_t tactile_frame_count(Finger finger) const;
  const TactileMatrix& tactile_matrix(Finger finger) const;

private:
  struct TactileState
  {
    TactileMatrix matrix{};
    std::array<bool, O7CanProtocol::kTactileRows> received_rows{};
    std::size_t received_frame_count{0};
  };

  static constexpr std::size_t finger_index(Finger finger)
  {
    return static_cast<std::size_t>(finger);
  }

  bool store_tactile_reply(
    const O7CanProtocol::TactileReply& reply,
    bool& matrix_complete);

  uint32_t expected_can_id_{O7CanProtocol::kRightHandCanId};
  JointValues position_{};
  JointValues max_velocity_reply_{};
  JointValues max_acceleration_reply_{};
  bool has_position_{false};
  bool has_max_velocity_reply_{false};
  bool has_max_acceleration_reply_{false};
  std::array<TactileState, O7CanProtocol::kFingerCount> tactile_states_{};
};

}  // namespace can_ros2_control
