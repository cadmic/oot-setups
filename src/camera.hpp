#pragma once

#include "collision.hpp"
#include "global.hpp"

struct Camera {
  int setting;
  int mode;
  int frames;        // Global frame counter (used by pitch adjustment)
  Collision* col;    // Collision data
  f32 playerHeight;  // Adult or child height (constant)

  Vec3f playerPos;  // Last player position
  Vec3f atOffset;   // Offset from player
  Vec3f at;         // Where the camera's looking at
  Vec3f eyeNext;    // Where the camera wants to be
  Vec3f eye;        // Where the camera is

  f32 xzSpeed;
  f32 dist;
  f32 speedRatio;
  f32 rUpdateRateInv;
  f32 pitchUpdateRateInv;
  f32 yawUpdateRateInv;
  f32 atLERPStepScale;

  // Normal mode variables
  f32 prevXZSpeed;          // rwData->unk_20 in decomp
  f32 yawUpdateRateTarget;  // rwData->swing.swingUpdateRate in decomp
  s16 rUpdateRateTimer;     // rwData->unk_28 in decomp
  s16 slopePitchAdj;        // rwData->slopePitchAdj in decomp

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

  u16 yaw();
  // Initialize the camera as if we've been targeting for a while.
  void initParallel1(Vec3f pos, u16 angle, int setting);
  // Update untargeted camera.
  void updateNormal1(Vec3f pos, u16 angle, int setting);
};
