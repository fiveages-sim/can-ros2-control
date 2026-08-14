#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>

namespace can_ros2_control
{

/** Common CAN identifiers and tactile commands shared by LinkerHand models. */
struct LinkerHandCanProtocol
{
  static constexpr std::size_t kFingerCount = 5;
  static constexpr uint32_t kRightHandCanId = 0x27;
  static constexpr uint32_t kLeftHandCanId = 0x28;
  static constexpr int kCanBitrate = 1000000;

  static constexpr uint8_t kPositionCommand = 0x01;
  static constexpr uint8_t kThumbTactileCommand = 0xB1;
  static constexpr uint8_t kIndexTactileCommand = 0xB2;
  static constexpr uint8_t kMiddleTactileCommand = 0xB3;
  static constexpr uint8_t kRingTactileCommand = 0xB4;
  static constexpr uint8_t kPinkyTactileCommand = 0xB5;
  static constexpr uint8_t kTactileRequestDlc = 2;

  enum class Finger : uint8_t
  {
    kThumb = 0,
    kIndex,
    kMiddle,
    kRing,
    kPinky,
  };

  struct Frame
  {
    uint32_t can_id{0};
    uint8_t dlc{0};
    std::array<uint8_t, 8> data{};
  };

  static constexpr std::array<uint8_t, kFingerCount> kTactileCommands{
    kThumbTactileCommand,
    kIndexTactileCommand,
    kMiddleTactileCommand,
    kRingTactileCommand,
    kPinkyTactileCommand,
  };

  static constexpr uint8_t tactile_command(Finger finger)
  {
    return kTactileCommands[static_cast<std::size_t>(finger)];
  }

  static bool finger_from_tactile_command(uint8_t command, Finger& finger)
  {
    const auto iterator = std::find(
      kTactileCommands.begin(), kTactileCommands.end(), command);
    if (iterator == kTactileCommands.end())
    {
      return false;
    }
    finger = static_cast<Finger>(
      std::distance(kTactileCommands.begin(), iterator));
    return true;
  }

  static Frame make_tactile_read_request(
    uint32_t can_id, Finger finger, uint8_t layout)
  {
    Frame frame;
    frame.can_id = can_id;
    frame.dlc = kTactileRequestDlc;
    frame.data[0] = tactile_command(finger);
    frame.data[1] = layout;
    return frame;
  }
};

}  // namespace can_ros2_control
