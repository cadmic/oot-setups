#pragma once

#include "global.hpp"
#include "sys_math3d.hpp"

// Compute Link's displacement from a push.
Vec3f calculatePush(Vec3s linkPos, Vec3s objectPos, s16 objectRadius, f32 dispRatio);

// Compute Link's displacement from a bomb push (radius 6, displacement ratio 0.8).
Vec3f bombPush(Vec3f linkPos, Vec3f bombPos);

// Compute Link's displacement from colliding with an immovable object (displacement ratio 1.0).
Vec3f immovablePush(Vec3f linkPos, Vec3f objectPos, s16 objectRadius);

// Test whether a sphere collides with a quad.
bool colliderSphVsQuad(Sphere16* sph, Vec3f* quad);

// Test whether a sphere collides with a cylinder.
bool colliderSphVsCyl(Sphere16* sph, Cylinder16* cyl);
