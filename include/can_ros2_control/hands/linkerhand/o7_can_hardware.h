#pragma once

#include "can_ros2_control/hands/linkerhand/linkerhand_can_hardware.h"

namespace can_ros2_control
{

class O7CanHardware final : public LinkerHandCanHardware
{
public:
  O7CanHardware();
};

}  // namespace can_ros2_control
