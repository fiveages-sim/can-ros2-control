#include "can_ros2_control/hands/linkerhand/o6_can_hardware.h"

#include <pluginlib/class_list_macros.hpp>

namespace can_ros2_control
{
namespace
{
constexpr LinkerHandModelConfig kO6Config{
  "o6",
  6,
  {0, 1, 2, 3, 4, 5, 6},
  0xA4,
  10,
  4,
  {0.58, 1.30, 1.60, 1.60, 1.60, 1.60, 0.0},
  true,
};
}  // namespace

O6CanHardware::O6CanHardware()
: LinkerHandCanHardware(kO6Config)
{
}

}  // namespace can_ros2_control

PLUGINLIB_EXPORT_CLASS(
  can_ros2_control::O6CanHardware,
  hardware_interface::SystemInterface)
