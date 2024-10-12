#pragma once

#include "camera.hpp"
#include "global.hpp"

struct CullZone {
    f32 forward; // actor + 0xF4
    f32 scale; // actor + 0xF8
    f32 downward; // actor + 0xFC
};

bool isCulled(Camera* camera, CullZone* zone, Vec3f pos, f32 fovy, f32 near, f32 far);
