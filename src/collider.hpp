#pragma once

#include "global.hpp"

// Compute Link's displacement from a bomb push.
Vec3f bombPush(Vec3f linkPos, Vec3f bombPos);

// Compute Link's displacement from colliding with an immovable object.
Vec3f immovablePush(Vec3f linkPos, Vec3f objectPos, s16 objectRadius);
