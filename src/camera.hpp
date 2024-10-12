#pragma once

#include "collision.hpp"
#include "global.hpp"

struct Camera {
  int setting;
  int mode;
  bool modeChangeDisallowed;  // CAM_STATE_5
  int frames;                 // Global frame counter (used by pitch adjustment)
  Collision* col;             // Collision data
  f32 playerHeight;           // Adult or child height (constant)

  Vec3f prevPlayerPos;  // Previous player position
  Vec3f playerPos;      // Last player position
  Vec3f atOffset;       // Offset from player
  Vec3f at;             // Where the camera's looking at
  Vec3f eyeNext;        // Where the camera wants to be
  Vec3f eye;            // Where the camera is
  f32 xzSpeed;
  f32 speedRatio;

  f32 dist;
  f32 rUpdateRateInv;
  f32 pitchUpdateRateInv;
  f32 yawUpdateRateInv;
  f32 atLERPStepScale;

  // Normal mode variables
  f32 normalPrevXZSpeed;          // rwData->unk_20 in decomp
  f32 normalYawUpdateRateTarget;  // rwData->swing.swingUpdateRate in decomp
  s16 normalRUpdateRateTimer;     // rwData->unk_28 in decomp
  s16 normalSlopePitchAdj;        // rwData->slopePitchAdj in decomp

  // Parallel mode variables
  int parallelAnimTimer;  // rwData->animTimer in decomp
  f32 parallelYawTarget;  // rwData->yawTarget in decomp

  // Jump mode variables
  f32 jumpStartY;  // rwData->unk_1C in decomp

  // In the game, these variables are static and only sometimes updated
  // depending on the global frame counter
  Vec3f pitchTestPos;
  Vec3f pitchTestNormal;
  f32 floorYNear;
  f32 floorYFar;

  // Collision polygons (informational only)
  CollisionPoly* wallPoly;
  CollisionPoly* floorPoly;

  Camera(Collision* col);

  // Initialize the camera as if the player has been targeting for a while and
  // moving around. This usually won't be exactly correct but it's often close.
  void initParallel(Vec3f pos, u16 angle, int setting);

  // Update the camera.
  void update(Vec3f pos, u16 angle, int setting, int mode);

  // Alias for mode 0
  void updateNormal(Vec3f pos, u16 angle, int setting);
  // Alias for mode 1
  void updateParallel(Vec3f pos, u16 angle, int setting);
  // Alias for mode 13
  void updateJump(Vec3f pos, u16 angle, int setting);

  // Get the current camera angle.
  u16 yaw();
};
