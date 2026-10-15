#pragma once

#include "rokae_ros2_control/linkerhand.h"

namespace rokae_ros2_control {
// Compatibility for existing users of the L6 driver. Config still defaults to L6.
using LinkerHandL6 = LinkerHand;
}
