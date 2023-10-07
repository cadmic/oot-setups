#pragma once

#include "global.hpp"

f32 Math_SinS(s16 angle);
f32 Math_CosS(s16 angle);
u16 Math_Atan2S(f32 x, f32 y);

f32 Math_FAtan2F(f32 y, f32 x);
f32 Math_FAsinF(f32 x);
f32 Math_FAcosF(f32 x);

f32 Math_Vec3f_DistXZ(Vec3f* a, Vec3f* b);
u16 Math_Vec3f_Yaw(Vec3f* a, Vec3f* b);

bool Math_ScaledStepToS(u16* pValue, u16 target, s16 step);
bool Math_StepToF(f32* pValue, f32 target, f32 step);
