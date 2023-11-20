#pragma once

#include "global.hpp"
#include "sys_math3d.hpp"

// Compute Link's displacement from a bomb push.
Vec3f bombPush(Vec3f linkPos, Vec3f bombPos);

// Compute Link's displacement from colliding with an immovable object.
Vec3f immovablePush(Vec3f linkPos, Vec3f objectPos, s16 objectRadius);

// Test whether a sphere collides with a quad.
bool colliderSphVsQuad(Sphere16* sph, Vec3f* quad);
