#pragma once

#include "global.hpp"

struct CameraNormalSettings {
  s16 yOffset;
  s16 distMin;
  s16 distMax;
  s16 pitchTarget;
  s16 yawUpdateRateTarget;
  s16 pitchUpdateRateTarget;
  s16 maxYawUpdate;
  s16 fov;
  s16 atLerpStepScale;
  s16 interfaceField;
};

struct CameraZParallelSettings {
  s16 yOffset;
  s16 dist;
  s16 pitchTarget;
  s16 yawTarget;
  s16 yawUpdateRateTarget;
  s16 xzUpdateRateTarget;
  s16 fov;
  s16 atLerpStepScale;
  s16 interfaceField;
  s16 groundYOffset;
  s16 groundAtLerpStepScale;
};

struct CameraJumpSettings {
  s16 yOffset;
  s16 eyeDist;
  s16 eyeDistNext;
  s16 yawUpdateRateTarget;
  s16 maxYawUpdate;
  s16 fov;
  s16 atLerpStepScale;
  s16 interfaceField;
};

// Normal mode data
extern CameraNormalSettings cameraNormalSettings[];

// Z-target mode data
extern CameraZParallelSettings cameraZParallelSettings[];

// Jump mode data
extern CameraJumpSettings cameraJumpSettings[];
