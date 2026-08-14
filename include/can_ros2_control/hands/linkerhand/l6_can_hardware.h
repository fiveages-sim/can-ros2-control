#pragma once

#include "can_ros2_control/hands/linkerhand/linkerhand_can_hardware.h"

namespace can_ros2_control
{

class L6CanHardware final : public LinkerHandCanHardware
{
public:
  L6CanHardware();
};

}  // namespace can_ros2_control
