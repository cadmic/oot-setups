#include "pos_angle_setup.hpp"

#include "camera_angles.hpp"
#include "sys_math.hpp"

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
    case LONGROLL:
      return 20;
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
}

int actionsCost(const std::vector<Action>& actions) {
  int cost = 0;
  for (Action action : actions) {
    cost += actionCost(action);
  }
  return cost;
}

bool PosAngleSetup::essLeft(int n) {
  this->angle += n * 0x708;
  return true;
}

bool PosAngleSetup::essRight(int n) {
  this->angle -= n * 0x708;
  return true;
}

bool PosAngleSetup::move(u16 movementAngle, f32 xzSpeed, f32 ySpeed,
                         bool* onGround) {
  // printf(
  //     "move x=%.9g y=%.9g z=%.9g movementAngle=%04x xzSpeed=%.9g
  //     ySpeed=%.9g\n", this->pos.x, this->pos.y, this->pos.z, this->angle,
  //     xzSpeed, ySpeed);

  // TODO: move this math somewhere common
  Vec3f velocity = {Math_SinS(movementAngle) * xzSpeed, ySpeed - 1.0f,
                    Math_CosS(movementAngle) * xzSpeed};
  Vec3f intendedPos = pos + velocity * 1.5f;

  CollisionPoly* wallPoly;
  CollisionPoly* floorPoly;
  f32 floorHeight;
  this->pos = this->col->runChecks(this->pos, intendedPos, &wallPoly,
                                   &floorPoly, &floorHeight);
  *onGround = (this->pos.y <= floorHeight);

  // Check bounds
  if (this->pos.x < this->minBounds.x || this->pos.x > this->maxBounds.x ||
      this->pos.y < this->minBounds.y || this->pos.y > this->maxBounds.y ||
      this->pos.z < this->minBounds.z || this->pos.z > this->maxBounds.z) {
    return false;
  }

  return true;
}

bool PosAngleSetup::roll(u16 movementAngle, bool retarget) {
  u16 facingAngle = this->angle;
  for (int i = 0; i < 12; i++) {
    Math_ScaledStepToS(&movementAngle, facingAngle, 2000);
    Math_ScaledStepToS(&facingAngle, movementAngle, 2000);

    // 1 frame of 2 speed, 10 frames of 3 speed, 1 frame of 0 speed
    f32 xzSpeed;
    if (i < 1) {
      xzSpeed = 2.0f;
    } else if (i < 11) {
      xzSpeed = 3.0f;
    } else {
      xzSpeed = 0.0f;
    }

    bool onGround;
    if (!move(movementAngle, xzSpeed, -4.0f, &onGround)) {
      return false;
    }

    if (!onGround) {
      return false;
    }
  }

  if (retarget) {
    this->angle = facingAngle;
  }
  return true;
}

bool PosAngleSetup::longroll() {
  for (int i = 0; i < 13; i++) {
    // 11 frames of roll up to 9 speed, 1 frame of 1 speed, 1 frame of 0 speed
    f32 xzSpeed = 0.0f;
    if (i < 11) {
      xzSpeed = std::min(xzSpeed + 2.0f, 9.0f);
    } else if (i < 12) {
      xzSpeed = 1.0f;
    } else {
      xzSpeed = 0.0f;
    }

    bool onGround;
    // TODO: don't bonk
    if (!move(this->angle, xzSpeed, -4.0f, &onGround)) {
      return false;
    }

    if (!onGround) {
      return false;
    }
  }

  return true;
}

bool PosAngleSetup::jump(u16 movementAngle, f32 xzSpeed, f32 ySpeed) {
  bool onGround;
  for (int i = 0; i < 20; i++) {
    if (!move(movementAngle, xzSpeed, ySpeed, &onGround)) {
      return false;
    }

    ySpeed -= 1.0f;

    if (onGround) {
      break;
    }
  }

  xzSpeed -= 1.0f;
  if (!move(movementAngle, xzSpeed, ySpeed, &onGround)) {
    return false;
  }
  if (!onGround) {
    return false;
  }

  if (!move(movementAngle, 0, -4.0f, &onGround)) {
    return false;
  }
  if (!onGround) {
    return false;
  }

  return true;
}

bool PosAngleSetup::performAction(Action action) {
  switch (action) {
    case ROLL:
      return roll(this->angle, false);
    case LONGROLL:
      return longroll();
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
      this->angle = cameraAngles[this->angle];
      return true;
    case TURN_LEFT:
      this->angle = cameraAngles[this->angle] + 0x4000;
      return true;
    case TURN_RIGHT:
      this->angle = cameraAngles[this->angle] - 0x4000;
      return true;
    case TURN_DOWN:
      this->angle = cameraAngles[this->angle] + 0x8000;
      return true;
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
