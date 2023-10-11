#pragma once

#include "collision.hpp"
#include "global.hpp"

// Camera approximation. Assumes adult link only, flat ground, no camera swing,
// Link has moved around a bit before while targeted, and probably more.
struct Camera {
  int setting;
  int mode;
  int frames;  // global frame counter (used by pitch adjustment)

  Vec3f playerPos;  // Last player position
  Vec3f atOffset;   // Offset from player
  Vec3f at;         // Where the camera's looking at
  Vec3f eyeNext;    // Where the camera wants to be
  Vec3f eye;        // Where the camera is

  f32 playerHeight;
  f32 xzSpeed;
  f32 dist;
  f32 speedRatio;
  f32 yawUpdateRateTarget;  // rwData->swing.swingUpdateRate in decomp
  s16 rUpdateRateTimer;     // rwData->unk_28 in decomp
  f32 rUpdateRateInv;
  f32 pitchUpdateRateInv;
  f32 yawUpdateRateInv;
  f32 atLERPStepScale;

  Vec3f pitchTestPos;
  Vec3f pitchTestNormal;
  f32 floorYNear;
  f32 floorYFar;
  s16 slopePitchAdj;

  CollisionPoly* wallPoly;
  CollisionPoly* floorPoly;

  Camera(PlayerAge age);

  u16 yaw();
  // Initialize the camera as if we've been targeting for a while.
  void initParallel1(Vec3f pos, u16 angle, int setting);
  // Update untargeted camera.
  void updateNormal1(Collision* col, Vec3f pos, u16 angle, int setting);
};
