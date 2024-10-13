#pragma once

#include "collision.hpp"
#include "global.hpp"
#include "skin_matrix.hpp"

struct JointIndex {
  u16 x;
  u16 y;
  u16 z;
};

struct Limb {
  Vec3s jointPos;
  u8 child;
  u8 sibling;
};

struct AnimationHeader {
  s16 frameCount;
  u16* frameData;
  JointIndex* jointIndices;
  u16 staticIndexMax;
};

#define LIMB_DONE 0xFF

enum PlayerLimb {
  PLAYER_LIMB_ROOT,
  PLAYER_LIMB_WAIST,
  PLAYER_LIMB_LOWER,
  PLAYER_LIMB_R_THIGH,
  PLAYER_LIMB_R_SHIN,
  PLAYER_LIMB_R_FOOT,
  PLAYER_LIMB_L_THIGH,
  PLAYER_LIMB_L_SHIN,
  PLAYER_LIMB_L_FOOT,
  PLAYER_LIMB_UPPER,
  PLAYER_LIMB_HEAD,
  PLAYER_LIMB_HAT,
  PLAYER_LIMB_COLLAR,
  PLAYER_LIMB_L_SHOULDER,
  PLAYER_LIMB_L_FOREARM,
  PLAYER_LIMB_L_HAND,
  PLAYER_LIMB_R_SHOULDER,
  PLAYER_LIMB_R_FOREARM,
  PLAYER_LIMB_R_HAND,
  PLAYER_LIMB_SHEATH,
  PLAYER_LIMB_TORSO,
  PLAYER_LIMB_MAX,
};

struct AnimFrame {
  Vec3s rootPos;
  // Entry 0 is actually root limb position
  Vec3s jointTable[PLAYER_LIMB_MAX];
  s16 face;
};

// Advance to the next frame in the animation. Returns false if the animation is
// finished.
bool nextAnimFrame(f32* curFrame, int endFrame, f32 updateRate);

// Load an animation frame from the animation data.
void loadAnimFrame(AnimationHeader* animHeader, int limbCount, int frame,
                   Vec3f* rootPos, Vec3s* jointTable);

// Load a Link animation frame from the animation data.
void loadAnimFrame(u16* animData, int frame, AnimFrame* animFrame);

// Load only the upper body part of animation frame from the animation data.
void loadUpperBodyAnimFrame(u16* animData, int frame, AnimFrame* animFrame);

// Apply animation frame for a skeleton, outputting matrices for each limb.
void applySkeleton(Limb* skeleton, Vec3s* jointTable, Vec3f pos, u16 angle,
                   Vec3f rootPos, MtxF* outLimbMatrices);

// Apply animation frame for Link, outputting matrices for each limb.
void applyAnimFrame(AnimFrame* animFrame, PlayerAge age, Vec3f pos, u16 angle,
                    MtxF* outLimbMatrices);

// Apply animation frame for a "skin" skeleton, outputting matrices for each limb.
void applySkinSkeleton(Limb* skeleton, int limbCount, Vec3s* jointTable, Vec3f pos, u16 angle,
                       Vec3f rootPos, MtxF* outMatrix, MtxF* outLimbMatrices);

// Default root translation for Link's skeleton.
Vec3f baseRootTranslation(PlayerAge age, u16 angle);

// Update Link's position, update the previous root translation, and reset the
// xz root translation in the animation based on the animation and previous root
// translation.
void updateRootTranslation(AnimFrame* animFrame, Vec3f* pos, u16 angle,
                           Vec3f* prevRootTranslation);

// Compute held actor position, halfway between Link's hands.
Vec3f heldActorPosition(AnimFrame* animFrame, PlayerAge age, Vec3f pos,
                        u16 angle);

// Compute weapon base and tip positions.
void getWeaponPosition(AnimFrame* animFrame, PlayerAge age, f32 weaponLength, Vec3f pos, u16 angle,
                       Vec3f* outBase, Vec3f* outTip);

// Tests if weapon collides with a wall.
bool weaponRecoil(Collision* col, AnimFrame* animFrame, PlayerAge age, f32 weaponLength, Vec3f pos,
                  u16 angle);

// Compute shield corner positions. The order is DR, UR, DL, UL.
void getShieldPosition(AnimFrame* animFrame, PlayerAge age, Vec3f pos,
                       u16 angle, Vec3f* outCorners);
