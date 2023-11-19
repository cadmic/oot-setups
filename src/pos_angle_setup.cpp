#include "pos_angle_setup.hpp"

#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "camera.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"
#include "sys_matrix.hpp"

const char* actionNames[] = {
#define X(name) #name,
    ACTIONS
#undef X
};

const char* actionName(Action action) { return actionNames[action]; }

int essCost(int turns) { return turns + 6; }

// TODO: make these numbers more principled
int actionCost(Action action) {
  switch (action) {
    case ROLL:
      return 13;
    case LONG_ROLL:
      return 20;
    case SHIELD_SCOOT:
      return 10;
    case SIDEHOP_LEFT:
    case SIDEHOP_RIGHT:
      return 8;
    case SIDEHOP_LEFT_SIDEROLL:
    case SIDEHOP_LEFT_SIDEROLL_RETARGET:
    case SIDEHOP_RIGHT_SIDEROLL:
    case SIDEHOP_RIGHT_SIDEROLL_RETARGET:
      return 21;
    case BACKFLIP:
      return 13;
    case BACKFLIP_SIDEROLL:
    case BACKFLIP_SIDEROLL_RETARGET:
      return 26;
    // TODO: these slash timings are a bit optimistic
    case HORIZONTAL_SLASH:
      return 13;
    case HORIZONTAL_SLASH_SHIELD:
      return 10;
    case DIAGONAL_SLASH:
      return 15;
    case DIAGONAL_SLASH_SHIELD:
      return 10;
    case VERTICAL_SLASH:
      return 12;
    case VERTICAL_SLASH_SHIELD:
      return 10;
    case FORWARD_STAB:
      return 12;
    case FORWARD_STAB_SHIELD:
      return 9;
    case JUMPSLASH:
      return 32;
    case JUMPSLASH_SHIELD:
      return 20;
    case LONG_JUMPSLASH_SHIELD:
      return 24;
    case CROUCH_STAB:
      return 8;
    case TURN_ESS_UP:
    case TURN_ESS_LEFT:
    case TURN_ESS_RIGHT:
      return essCost(1);
    case TURN_2_ESS_LEFT:
    case TURN_2_ESS_RIGHT:
      return essCost(2);
    case TURN_3_ESS_LEFT:
    case TURN_3_ESS_RIGHT:
      return essCost(3);
    case TURN_4_ESS_LEFT:
    case TURN_4_ESS_RIGHT:
      return essCost(4);
    case TURN_5_ESS_LEFT:
    case TURN_5_ESS_RIGHT:
      return essCost(5);
    case TURN_6_ESS_LEFT:
    case TURN_6_ESS_RIGHT:
      return essCost(6);
    case TURN_7_ESS_LEFT:
    case TURN_7_ESS_RIGHT:
      return essCost(7);
    case TURN_LEFT:
    case TURN_RIGHT:
    case TURN_DOWN:
      return 8;
  }

  return 0;
}

int actionsCost(const std::vector<Action>& actions) {
  int cost = 0;
  for (Action action : actions) {
    cost += actionCost(action);
  }
  return cost;
}

struct SwordSlash {
  u16* startAnimData;
  int startAnimFrames;
  u16* endAnimData;
  int endAnimFrames;
};

SwordSlash horizontalSlash = {
    .startAnimData = gPlayerAnim_link_fighter_Lside_kiru_Data,
    .startAnimFrames = 5,
    .endAnimData = gPlayerAnim_link_fighter_Lside_kiru_end_Data,
    .endAnimFrames = 9,
};

SwordSlash diagonalSlash = {
    .startAnimData = gPlayerAnim_link_fighter_Rside_kiru_Data,
    .startAnimFrames = 5,
    .endAnimData = gPlayerAnim_link_fighter_Rside_kiru_end_Data,
    .endAnimFrames = 12,
};

SwordSlash verticalSlash = {
    .startAnimData = gPlayerAnim_link_fighter_normal_kiru_Data,
    .startAnimFrames = 5,
    .endAnimData = gPlayerAnim_link_fighter_normal_kiru_end_Data,
    .endAnimFrames = 8,
};

SwordSlash forwardStab = {
    .startAnimData = gPlayerAnim_link_fighter_pierce_kiru_Data,
    .startAnimFrames = 4,
    .endAnimData = gPlayerAnim_link_fighter_pierce_kiru_end_Data,
    .endAnimFrames = 9,
};

SwordSlash jumpslashLanding = {
    .startAnimData = gPlayerAnim_link_fighter_Lpower_jump_kiru_hit_Data,
    .startAnimFrames = 9,
    .endAnimData = gPlayerAnim_link_fighter_power_jump_kiru_end_Data,
    .endAnimFrames = 20,
};

PosAngleSetup::PosAngleSetup(Collision* col, Vec3f initialPos, u16 initialAngle,
                             Vec3f minBounds, Vec3f maxBounds)
    : col(col),
      minBounds(minBounds),
      maxBounds(maxBounds),
      pos(initialPos),
      angle(initialAngle),
      floorPoly(NULL),
      dynaId(-1) {
  this->col->findFloor({this->pos.x, this->pos.y + 50.0f, this->pos.z},
                       &this->floorPoly, &this->dynaId);
}

PosAngleSetup::PosAngleSetup(Collision* col, Vec3f initialPos, u16 initialAngle)
    : PosAngleSetup(col, initialPos, initialAngle,
                    Vec3f(-10000, -10000, -10000), Vec3f(10000, 10000, 10000)) {
}

bool PosAngleSetup::essLeft(int n) {
  this->angle += n * 0x708;
  return true;
}

bool PosAngleSetup::essRight(int n) {
  this->angle -= n * 0x708;
  return true;
}

bool PosAngleSetup::cameraTurn(u16 offset) {
  if (!this->floorPoly) {
    return false;
  }

  // TODO: cache camera angle somehow?
  int setting = this->col->getCameraSetting(this->floorPoly, this->dynaId);
  Camera camera(this->col);
  camera.initParallel(this->pos, this->angle, setting);
  camera.updateNormal(this->pos, this->angle, setting);

  // TODO: he camera might not be stable after only 1 frame of movement.
  // Sometimes this makes turns unreliable, but sometimes it's fine because the
  // camera angle only varies slightly. We need some principled way to determine
  // whether the camera is stable.
  this->angle = camera.yaw() + offset;
  return true;
}

bool PosAngleSetup::move(Vec3f prevPos, u16 movementAngle, f32 xzSpeed,
                         f32 ySpeed, bool* onGround) {
  // printf(
  //     "move x=%.9g y=%.9g z=%.9g movementAngle=%04x xzSpeed=%.9g "
  //     "ySpeed=%.9g\n", this->pos.x, this->pos.y, this->pos.z, this->angle,
  //     xzSpeed, ySpeed);

  Vec3f intendedPos = translate(pos, movementAngle, xzSpeed, ySpeed);

  CollisionPoly* wallPoly;
  f32 floorHeight;
  this->pos =
      this->col->runChecks(prevPos, intendedPos, &wallPoly, &this->floorPoly,
                           &this->dynaId, &floorHeight);
  *onGround = (this->pos.y <= floorHeight);

  // Check bounds
  if (this->pos.x < this->minBounds.x || this->pos.x > this->maxBounds.x ||
      this->pos.y < this->minBounds.y || this->pos.y > this->maxBounds.y ||
      this->pos.z < this->minBounds.z || this->pos.z > this->maxBounds.z) {
    return false;
  }

  return true;
}

bool PosAngleSetup::moveOnGround(Vec3f prevPos, u16 movementAngle, f32 xzSpeed,
                                 f32 ySpeed) {
  bool onGround;
  if (!move(prevPos, movementAngle, xzSpeed, ySpeed, &onGround)) {
    return false;
  }

  if (!onGround) {
    return false;
  }

  return true;
}

bool PosAngleSetup::settle() {
  Vec3f prevPos;
  do {
    prevPos = this->pos;
    if (!moveOnGround(prevPos, this->angle, 0, -5.0f)) {
      return false;
    }
  } while (pos != prevPos);

  return true;
}

bool PosAngleSetup::roll(u16 movementAngle, bool retarget) {
  u16 facingAngle = this->angle;
  for (int i = 0; i < 11; i++) {
    Math_ScaledStepToS(&movementAngle, facingAngle, 2000);
    Math_ScaledStepToS(&facingAngle, movementAngle, 2000);

    // 1 frame of 2 speed, 10 frames of 3 speed, 1 frame of 0 speed
    f32 xzSpeed;
    if (i < 1) {
      xzSpeed = 2.0f;
    } else {
      xzSpeed = 3.0f;
    }

    if (!moveOnGround(this->pos, movementAngle, xzSpeed, -5.0f)) {
      return false;
    }
  }

  if (!settle()) {
    return false;
  }

  if (retarget) {
    this->angle = facingAngle;
  }
  return true;
}

bool PosAngleSetup::longRoll() {
  for (int i = 0; i < 12; i++) {
    // 11 frames of roll up to 9 speed, 1 frame of 1 speed
    f32 xzSpeed = 0.0f;
    if (i < 11) {
      xzSpeed = std::min(xzSpeed + 2.0f, 9.0f);
    } else {
      xzSpeed = 1.0f;
    }

    // TODO: don't bonk
    if (!moveOnGround(this->pos, this->angle, xzSpeed, -5.0f)) {
      return false;
    }
  }

  if (!settle()) {
    return false;
  }

  return true;
}

bool PosAngleSetup::shieldScoot() {
  for (int i = 0; i < 2; i++) {
    if (!moveOnGround(this->pos, this->angle, 2.0f, -5.0f)) {
      return false;
    }
  }

  if (!moveOnGround(this->pos, this->angle, 0.0f, -5.0f)) {
    return false;
  }

  return true;
}

bool PosAngleSetup::jump(u16 movementAngle, f32 xzSpeed, f32 ySpeed) {
  bool onGround;
  for (int i = 0; i < 20; i++) {
    ySpeed -= 1.0f;
    if (!move(this->pos, movementAngle, xzSpeed, ySpeed, &onGround)) {
      return false;
    }

    if (onGround) {
      break;
    }
  }

  xzSpeed -= 1.0f;
  ySpeed -= 1.0f;
  if (!moveOnGround(this->pos, movementAngle, xzSpeed, ySpeed)) {
    return false;
  }

  if (!settle()) {
    return false;
  }

  return true;
}

bool PosAngleSetup::swordSlash(const SwordSlash& slash, bool lunge,
                               bool shield) {
  PlayerAge age = this->col->age;

  Vec3f prevRoot = baseRootTranslation(this->angle);
  Vec3f prevPos = this->pos;
  f32 speed = 0.0f;

  AnimFrame animFrame;
  f32 curFrame = 0;
  do {
    loadAnimFrame(slash.startAnimData, curFrame, &animFrame);
    updateRootTranslation(&animFrame, age, &this->pos, this->angle, &prevRoot);

    bool swordHit = false;
    if (curFrame >= 2) {
      swordHit =
          swordRecoil(this->col, &animFrame, age, this->pos, this->angle);
    }

    bool onGround;
    if (!move(prevPos, this->angle, speed, -5.0f, &onGround)) {
      return false;
    }

    if (!onGround) {
      if (curFrame >= 2) {
        this->pos = prevPos;
        speed = 0.0f;
      } else {
        return false;
      }
    }

    prevPos = this->pos;

    if (lunge && curFrame == 0) {
      speed = 15.0f;
    }

    if (swordHit && speed >= 0.0f) {
      speed = -14.0f;
    }

    Math_StepToF(&speed, 0.0f, 5.0f);
  } while (nextAnimFrame(&curFrame, slash.startAnimFrames - 1, 1.0f));

  curFrame = 0.0f;
  do {
    loadAnimFrame(slash.endAnimData, curFrame, &animFrame);
    updateRootTranslation(&animFrame, age, &this->pos, this->angle, &prevRoot);

    if (!moveOnGround(prevPos, this->angle, speed, -5.0f)) {
      return false;
    }
    prevPos = this->pos;

    Math_StepToF(&speed, 0.0f, 8.0f);
    if (shield) {
      break;
    }
  } while (nextAnimFrame(&curFrame, slash.endAnimFrames - 1, 1.5f));

  if (!moveOnGround(prevPos, this->angle, speed, -5.0f)) {
    return false;
  }

  if (!settle()) {
    return false;
  }

  return true;
}

bool PosAngleSetup::jumpslash(bool holdUp, bool shield) {
  f32 xzSpeed = 5.0f;
  f32 ySpeed = 5.0f;
  f32 gravity = 1.0f;
  f32 accel = holdUp ? 0.05f : -0.1f;

  bool onGround;
  for (int i = 0; i < 20; i++) {
    // TODO: model sword recoil while in the air

    ySpeed -= gravity;
    if (!move(this->pos, this->angle, xzSpeed, ySpeed, &onGround)) {
      return false;
    }

    xzSpeed += accel;
    gravity = 1.2f;

    if (onGround) {
      break;
    }
  }

  return swordSlash(jumpslashLanding, false, shield);
}

bool PosAngleSetup::crouchStab() {
  PlayerAge age = this->col->age;
  f32 speed = 0.0f;

  AnimFrame animFrame;
  f32 curFrame = 0.0f;
  do {
    bool swordHit = false;
    if (curFrame >= 2) {
      loadAnimFrame(gPlayerAnim_link_normal_defense_kiru_Data, curFrame,
                    &animFrame);
      swordHit =
          swordRecoil(this->col, &animFrame, age, this->pos, this->angle);
    }

    Vec3f prevPos = this->pos;
    bool onGround;
    if (!move(this->pos, this->angle, speed, -5.0f, &onGround)) {
      return false;
    }

    if (!onGround) {
      if (curFrame >= 2) {
        this->pos = prevPos;
        speed = 0.0f;
      } else {
        return false;
      }
    }

    Math_StepToF(&speed, 0.0f, 8.0f);
    if (swordHit && speed >= 0.0f) {
      speed = -14.0f;
    }
  } while (nextAnimFrame(&curFrame, 4, 1.5f));

  if (!moveOnGround(this->pos, this->angle, speed, -5.0f)) {
    return false;
  }

  if (!settle()) {
    return false;
  }

  return true;
}

bool PosAngleSetup::performAction(Action action) {
  switch (action) {
    case ROLL:
      return roll(this->angle, false);
    case LONG_ROLL:
      return longRoll();
    case SHIELD_SCOOT:
      return shieldScoot();
    case SIDEHOP_LEFT:
      return jump(this->angle + 0x4000, 8.5f, 3.5f);
    case SIDEHOP_LEFT_SIDEROLL:
      if (!jump(this->angle + 0x4000, 8.5f, 3.5f)) {
        return false;
      }
      return roll(this->angle + 0x4000, false);
    case SIDEHOP_LEFT_SIDEROLL_RETARGET:
      if (!jump(this->angle + 0x4000, 8.5f, 3.5f)) {
        return false;
      }
      return roll(this->angle + 0x4000, true);
    case SIDEHOP_RIGHT:
      return jump(this->angle - 0x4000, 8.5f, 3.5f);
    case SIDEHOP_RIGHT_SIDEROLL:
      if (!jump(this->angle - 0x4000, 8.5f, 3.5f)) {
        return false;
      }
      return roll(this->angle - 0x4000, false);
    case SIDEHOP_RIGHT_SIDEROLL_RETARGET:
      if (!jump(this->angle - 0x4000, 8.5f, 3.5f)) {
        return false;
      }
      return roll(this->angle - 0x4000, true);
    case BACKFLIP:
      return jump(this->angle + 0x8000, 6.0f, 5.8f);
    case BACKFLIP_SIDEROLL:
      if (!jump(this->angle + 0x8000, 6.0f, 5.8f)) {
        return false;
      }
      return roll(this->angle + 0x8000, false);
    case BACKFLIP_SIDEROLL_RETARGET:
      if (!jump(this->angle + 0x8000, 6.0f, 5.8f)) {
        return false;
      }
      return roll(this->angle + 0x8000, true);
    case HORIZONTAL_SLASH:
      return swordSlash(horizontalSlash, false, false);
    case HORIZONTAL_SLASH_SHIELD:
      return swordSlash(horizontalSlash, false, true);
    case DIAGONAL_SLASH:
      return swordSlash(diagonalSlash, false, false);
    case DIAGONAL_SLASH_SHIELD:
      return swordSlash(diagonalSlash, false, true);
    case VERTICAL_SLASH:
      return swordSlash(verticalSlash, false, false);
    case VERTICAL_SLASH_SHIELD:
      return swordSlash(verticalSlash, false, true);
    case FORWARD_STAB:
      return swordSlash(forwardStab, true, false);
    case FORWARD_STAB_SHIELD:
      return swordSlash(forwardStab, true, true);
    case JUMPSLASH:
      return jumpslash(false, false);
    case JUMPSLASH_SHIELD:
      return jumpslash(false, true);
    case LONG_JUMPSLASH_SHIELD:
      return jumpslash(true, true);
    case CROUCH_STAB:
      return crouchStab();
    case TURN_ESS_LEFT:
      return essLeft(1);
    case TURN_2_ESS_LEFT:
      return essLeft(2);
    case TURN_3_ESS_LEFT:
      return essLeft(3);
    case TURN_4_ESS_LEFT:
      return essLeft(4);
    case TURN_5_ESS_LEFT:
      return essLeft(5);
    case TURN_6_ESS_LEFT:
      return essLeft(6);
    case TURN_7_ESS_LEFT:
      return essLeft(7);
    case TURN_ESS_RIGHT:
      return essRight(1);
    case TURN_2_ESS_RIGHT:
      return essRight(2);
    case TURN_3_ESS_RIGHT:
      return essRight(3);
    case TURN_4_ESS_RIGHT:
      return essRight(4);
    case TURN_5_ESS_RIGHT:
      return essRight(5);
    case TURN_6_ESS_RIGHT:
      return essRight(6);
    case TURN_7_ESS_RIGHT:
      return essRight(7);
    case TURN_ESS_UP:
      return cameraTurn(0x0000);
    case TURN_LEFT:
      return cameraTurn(0x4000);
    case TURN_RIGHT:
      return cameraTurn(0xc000);
    case TURN_DOWN:
      return cameraTurn(0x8000);
  }
  return false;
}

bool PosAngleSetup::performActions(const std::vector<Action>& actions) {
  for (Action action : actions) {
    if (!performAction(action)) {
      return false;
    }
  }
  return true;
}
