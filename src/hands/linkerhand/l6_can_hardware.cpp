#include "can_ros2_control/hands/linkerhand/l6_can_hardware.h"

#include <pluginlib/class_list_macros.hpp>

namespace can_ros2_control
{
namespace
{
constexpr LinkerHandModelConfig kL6Config{
  "l6",
  6,
  {0, 1, 2, 3, 4, 5, 6},
  0xC6,
  12,
  6,
  {0.99, 1.39, 1.26, 1.26, 1.26, 1.26, 0.0},
};
}  // namespace

L6CanHardware::L6CanHardware()
: LinkerHandCanHardware(kL6Config)
{
}

}  // namespace can_ros2_control

PLUGINLIB_EXPORT_CLASS(
  can_ros2_control::L6CanHardware,
  hardware_interface::SystemInterface)
