#pragma once

#include "global.hpp"

#define CAM_DEG_TO_BINANG(degrees) (s16)(s32)((degrees)*182.04167f + .5f)
#define RAD_TO_DEG(radians) ((radians) * (180.0f / M_PI))

struct VecSphGeo {
  f32 r;      // radius
  s16 pitch;  // depends on coordinate system. See below.
  s16 yaw;    // azimuthal angle
};

// Defines a point in the spherical coordinate system.
// Pitch is 0 along the positive y-axis (up)
typedef VecSphGeo VecSph;

// Defines a point in the geographic coordinate system.
// Pitch is 0 along the xz-plane (horizon)
typedef VecSphGeo VecGeo;

f32 OLib_Vec3fDist(Vec3f* a, Vec3f* b);
f32 OLib_Vec3fDistXZ(Vec3f* a, Vec3f* b);
f32 OLib_ClampMinDist(f32 val, f32 min);
f32 OLib_ClampMaxDist(f32 val, f32 max);
Vec3f* OLib_Vec3fDistNormalize(Vec3f* dest, Vec3f* a, Vec3f* b);
Vec3f* OLib_VecGeoToVec3f(Vec3f* dest, VecGeo* geo);
VecSph* OLib_Vec3fToVecSph(VecSph* dest, Vec3f* vec);
VecGeo* OLib_Vec3fToVecGeo(VecGeo* dest, Vec3f* vec);
VecGeo* OLib_Vec3fDiffToVecGeo(VecGeo* dest, Vec3f* a, Vec3f* b);
Vec3f* OLib_Vec3fDiffRad(Vec3f* dest, Vec3f* a, Vec3f* b);
