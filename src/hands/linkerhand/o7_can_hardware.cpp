#include "can_ros2_control/hands/linkerhand/o7_can_hardware.h"

#include <pluginlib/class_list_macros.hpp>

namespace can_ros2_control
{
namespace
{
constexpr LinkerHandModelConfig kO7Config{
  "o7",
  7,
  // Wire order: thumb bend, thumb swing, index, middle, ring, pinky,
  // thumb rotation. ROS order places thumb rotation at index 2.
  {0, 1, 3, 4, 5, 6, 2},
  0xC6,
  12,
  6,
  {0.58, 1.92, 1.13, 1.36, 1.36, 1.36, 1.36},
};
}  // namespace

O7CanHardware::O7CanHardware()
: LinkerHandCanHardware(kO7Config)
{
}

}  // namespace can_ros2_control

PLUGINLIB_EXPORT_CLASS(
  can_ros2_control::O7CanHardware,
  hardware_interface::SystemInterface)
