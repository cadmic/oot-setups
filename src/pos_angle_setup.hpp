#pragma once

#include <string>
#include <vector>

#include "collision.hpp"
#include "global.hpp"

// Helper for brute forcing position and angle setups.

#define ACTIONS                      \
  X(TARGET_WALL)                     \
  X(ROLL)                            \
  X(LONG_ROLL)                       \
  X(SHIELD_SCOOT)                    \
  X(SIDEHOP_LEFT)                    \
  X(SIDEHOP_LEFT_SIDEROLL)           \
  X(SIDEHOP_LEFT_SIDEROLL_UNTARGET)  \
  X(SIDEHOP_RIGHT)                   \
  X(SIDEHOP_RIGHT_SIDEROLL)          \
  X(SIDEHOP_RIGHT_SIDEROLL_UNTARGET) \
  X(BACKFLIP)                        \
  X(BACKFLIP_SIDEROLL)               \
  X(BACKFLIP_SIDEROLL_UNTARGET)      \
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
  X(ROTATE_ESS_LEFT)                 \
  X(ROTATE_ESS_RIGHT)                \
  X(ESS_TURN_UP)                     \
  X(ESS_TURN_LEFT)                   \
  X(ESS_TURN_RIGHT)                  \
  X(ESS_TURN_DOWN)                   \
  X(SHIELD_TURN_LEFT)                \
  X(SHIELD_TURN_RIGHT)               \
  X(SHIELD_TURN_DOWN)

enum Action {
#define X(name) name,
  ACTIONS
#undef X
};

// Returns the name of the action.
const char* actionName(Action action);

// Returns the names of the actions as a comma-separated list.
std::string actionNames(const std::vector<Action>& actions);

// Returns the appoximate cost (in frames) of the action in isolation.
int actionCost(Action action);

// Returns the approximate cost (in frames) of the action given a previous
// action.
int actionCost(Action prevAction, Action action);

// Returns the approximate cost (in frames) of the actions in sequence.
int actionsCost(const std::vector<Action>& action);

struct SwordSlash;

struct Collider {
  Vec3s pos;
  s16 objectRadius;
  f32 dispRatio;
};

struct PosAngleSetup {
  // Parameters
  Collision* col;
  Vec3f minBounds;
  Vec3f maxBounds;
  // Current state
  Vec3f pos;
  u16 angle;
  bool targeted;
  // Collision check data
  CollisionPoly* wallPoly;
  CollisionPoly* floorPoly;
  int dynaId;
  // Camera data
  u16 cameraSetting;
  bool cameraStable;
  u16 cameraAngle;
  // Wall interaction for targeting
  bool canTargetWall;
  u16 targetWallAngle;
  // Collider data
  std::vector<Collider> colliders;

  PosAngleSetup(Collision* col, Vec3f initialPos, u16 initialAngle,
                Vec3f minBounds, Vec3f maxBounds);
  PosAngleSetup(Collision* col, Vec3f initialPos, u16 initialAngle);

  void addCollider(Vec3s pos, s16 objectRadius, f32 dispRatio);

  // Returns true if the action was performed successfully.
  bool performAction(Action action);
  bool performActions(const std::vector<Action>& actions);

 private:
  bool ensureTargeted();
  bool targetWall();
  bool rotateEss(int dir);
  bool cameraTurn(u16 offset);
  bool settle();
  bool move(Vec3f prevPos, u16 movementAngle, f32 xzSpeed, f32 ySpeed,
            bool* onGround);
  bool moveOnGround(Vec3f prevPos, u16 movementAngle, f32 xzSpeed, f32 ySpeed);
  bool roll(u16 movementAngle, bool retarget);
  bool longRoll();
  bool shieldScoot();
  bool jump(u16 movementAngle, f32 xzSpeed, f32 ySpeed);
  bool swordSlash(const SwordSlash& slash, bool requiresTarget, bool lunge,
                  bool shield);
  bool jumpslash(bool holdUp, bool shield);
  bool crouchStab();

  bool doAction(Action action);
  void updateCameraAngle();
  void updateTargetWall();
};
