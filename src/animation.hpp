#pragma once

#include "collision.hpp"
#include "global.hpp"
#include "skin_matrix.hpp"

enum PlayerLimb {
  /* 0x00 */ PLAYER_LIMB_NONE,
  /* 0x01 */ PLAYER_LIMB_ROOT,
  /* 0x02 */ PLAYER_LIMB_WAIST,
  /* 0x03 */ PLAYER_LIMB_LOWER,
  /* 0x04 */ PLAYER_LIMB_R_THIGH,
  /* 0x05 */ PLAYER_LIMB_R_SHIN,
  /* 0x06 */ PLAYER_LIMB_R_FOOT,
  /* 0x07 */ PLAYER_LIMB_L_THIGH,
  /* 0x08 */ PLAYER_LIMB_L_SHIN,
  /* 0x09 */ PLAYER_LIMB_L_FOOT,
  /* 0x0A */ PLAYER_LIMB_UPPER,
  /* 0x0B */ PLAYER_LIMB_HEAD,
  /* 0x0C */ PLAYER_LIMB_HAT,
  /* 0x0D */ PLAYER_LIMB_COLLAR,
  /* 0x0E */ PLAYER_LIMB_L_SHOULDER,
  /* 0x0F */ PLAYER_LIMB_L_FOREARM,
  /* 0x10 */ PLAYER_LIMB_L_HAND,
  /* 0x11 */ PLAYER_LIMB_R_SHOULDER,
  /* 0x12 */ PLAYER_LIMB_R_FOREARM,
  /* 0x13 */ PLAYER_LIMB_R_HAND,
  /* 0x14 */ PLAYER_LIMB_SHEATH,
  /* 0x15 */ PLAYER_LIMB_TORSO,
  /* 0x16 */ PLAYER_LIMB_MAX
};

// Apply animation frame overriding root translation, outputting matrices for
// each limb
void applyAnimation(u16* animData, int frame, PlayerAge age, Vec3f pos,
                    u16 angle, Vec3f rootTranslation, MtxF* outLimbMatrices);

// Apply animation frame, outputting matrices for each limb
void applyAnimation(u16* animData, int frame, PlayerAge age, Vec3f pos,
                    u16 angle, MtxF* outLimbMatrices);

// Advance to the next frame in the animation
bool nextAnimationFrame(f32* curFrame, int endFrame, f32 updateRate);

// Default root translation for Link's skeleton
Vec3f baseRootTranslation(u16 angle);

// Update Link's position based on the animation
void updateRootTranslation(u16* animData, int frame, PlayerAge age, Vec3f* pos,
                           u16 angle, Vec3f* prevRootTranslation);

// Compute held actor position, halfway between Link's hands
Vec3f heldActorPosition(u16* animData, int frame, PlayerAge age, Vec3f pos,
                        u16 angle);

// Compute sword base and tip positions
void getSwordPosition(u16* animData, int frame, PlayerAge age, Vec3f pos,
                      u16 angle, Vec3f* outBase, Vec3f* outTip);

// Tests if sword collides with a wall
bool swordRecoil(Collision* col, u16* animData, int frame, PlayerAge age,
                 Vec3f pos, u16 angle);
