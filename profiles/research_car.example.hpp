#pragma once
#include "rc_car/control.hpp"

// Unqualified starting values. Deployment must validate the installed harness and vehicle.
namespace board {
constexpr unsigned drive_input = 12, steer_input = 13;
constexpr unsigned drive_output = 14, steer_output = 15, pan_output = 7;
constexpr rc_car::Profile vehicle{
    {1000, 1500, 2000, 20, false}, {1000, 1500, 2000, 10, false},
    {1100, 1500, 1750, false}, {1000, 1500, 2000, false}, {600, 1500, 2400, false},
    750, 2250, 10000, 35000, 1000, 2000, 500,
};
static_assert(rc_car::valid(vehicle));
}
