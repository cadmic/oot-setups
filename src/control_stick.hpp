#pragma once

#include "global.hpp"

f32 controlStickMagnitude(int x, int y);
u16 controlStickAngle(int x, int y);

enum SpeedMode { SPEED_MODE_CURVED, SPEED_MODE_LINEAR };

f32 controlStickSpeed(int x, int y, u16 floorPitch, SpeedMode speedMode);
