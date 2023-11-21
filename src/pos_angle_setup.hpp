#pragma once

#include <vector>

#include "collision.hpp"
#include "global.hpp"

// Helper for brute forcing position and angle setups.

#define ACTIONS                      \
  X(ROLL)                            \
  X(LONG_ROLL)                       \
  X(SHIELD_SCOOT)                    \
  X(SIDEHOP_LEFT)                    \
  X(SIDEHOP_LEFT_SIDEROLL)           \
  X(SIDEHOP_LEFT_SIDEROLL_RETARGET)  \
  X(SIDEHOP_RIGHT)                   \
  X(SIDEHOP_RIGHT_SIDEROLL)          \
  X(SIDEHOP_RIGHT_SIDEROLL_RETARGET) \
  X(BACKFLIP)                        \
  X(BACKFLIP_SIDEROLL)               \
  X(BACKFLIP_SIDEROLL_RETARGET)      \
  X(HORIZONTAL_SLASH)                \
  X(HORIZONTAL_SLASH_SHIELD)         \
  X(DIAGONAL_SLASH)                  \
  X(DIAGONAL_SLASH_SHIELD)           \
  X(VERTICAL_SLASH)                  \
  X(VERTICAL_SLASH_SHIELD)           \
  X(FORWARD_STAB)                    \
  X(FORWARD_STAB_SHIELD)             \
  X(JUMPSLASH)                       \
  X(JUMPSLASH_SHIELD)                \
  X(LONG_JUMPSLASH_SHIELD)           \
  X(CROUCH_STAB)                     \
  X(TURN_ESS_LEFT)                   \
  X(TURN_2_ESS_LEFT)                 \
  X(TURN_3_ESS_LEFT)                 \
  X(TURN_4_ESS_LEFT)                 \
  X(TURN_5_ESS_LEFT)                 \
  X(TURN_6_ESS_LEFT)                 \
  X(TURN_7_ESS_LEFT)                 \
  X(TURN_ESS_RIGHT)                  \
  X(TURN_2_ESS_RIGHT)                \
  X(TURN_3_ESS_RIGHT)                \
  X(TURN_4_ESS_RIGHT)                \
  X(TURN_5_ESS_RIGHT)                \
  X(TURN_6_ESS_RIGHT)                \
  X(TURN_7_ESS_RIGHT)                \
  X(TURN_ESS_UP)                     \
  X(TURN_LEFT)                       \
  X(TURN_RIGHT)                      \
  X(TURN_DOWN)

enum Action {
#define X(name) name,
  ACTIONS
#undef X
};

const char* actionName(Action action);
int actionCost(Action action);
int actionsCost(const std::vector<Action>& action);

struct SwordSlash;

struct PosAngleSetup {
  // Parameters
  Collision* col;
  Vec3f minBounds;
  Vec3f maxBounds;
  // Current position and angle
  Vec3f pos;
  u16 angle;
  // Cached collision check data
  CollisionPoly* floorPoly;
  int dynaId;
  // Cached camera angle
  u16 cameraAngle;
  bool cameraStable;

  PosAngleSetup(Collision* col, Vec3f initialPos, u16 initialAngle,
                Vec3f minBounds, Vec3f maxBounds);
  PosAngleSetup(Collision* col, Vec3f initialPos, u16 initialAngle);

  // Returns true if the action was performed successfully.
  bool performAction(Action action);
  bool performActions(const std::vector<Action>& actions);

 private:
  bool essLeft(int n);
  bool essRight(int n);
  bool cameraTurn(u16 offset);
  bool settle();
  bool move(Vec3f prevPos, u16 movementAngle, f32 xzSpeed, f32 ySpeed,
            bool* onGround);
  bool moveOnGround(Vec3f prevPos, u16 movementAngle, f32 xzSpeed, f32 ySpeed);
  bool roll(u16 movementAngle, bool retarget);
  bool longRoll();
  bool shieldScoot();
  bool jump(u16 movementAngle, f32 xzSpeed, f32 ySpeed);
  bool swordSlash(const SwordSlash& slash, bool lunge, bool shield);
  bool jumpslash(bool holdUp, bool shield);
  bool crouchStab();

  bool doAction(Action action);
  void updateCameraAngle();
};
