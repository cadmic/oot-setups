#include "actor.hpp"
#include "camera_angles.hpp"
#include "collision_data.hpp"
#include "pos_angle_setup.hpp"
#include "sys_math.hpp"

Vec3f calculatePush(Vec3f pos) {
  Vec3s linkPos = pos.toVec3s();
  Vec3s switchPos = {-3160, 760, -540};

  f32 xDelta = linkPos.x - switchPos.x;
  f32 zDelta = linkPos.z - switchPos.z;

  f32 xzDist = sqrtf(SQ(xDelta) + SQ(zDelta));
  f32 overlap = 32 - xzDist;

  if (overlap <= 0) {
    return Vec3f();
  }

  if (xzDist != 0.0f) {
    xDelta *= overlap / xzDist;
    zDelta *= overlap / xzDist;
    return Vec3f(xDelta, 0, zDelta);
  } else {
    return Vec3f(-overlap, 0, 0);
  }
}

Vec3f translateWithPush(Vec3f pos, u16 angle, f32 xzSpeed, f32 ySpeed) {
  return translate(pos, angle, xzSpeed, ySpeed, calculatePush(pos));
}

bool onPlatform(Vec3f pos) {
  return -3181 < pos.x && pos.x < -3139 && -561 < pos.z && pos.z < -519;
}

bool platformPath(Vec3f pos, u16 angle, Vec3f* result) {
  pos = translateWithPush(pos, angle, -18.0f, 0.0f);
  if (!onPlatform(pos)) {
    return false;
  }

  pos = translateWithPush(pos, angle, -17.0f, 0.0f);
  if (!onPlatform(pos)) {
    return false;
  }

  *result = pos;
  return true;
}

void findPlatformPaths() {
  u16 angle = 0x1000;

  for (f32 x = -3181; x < -3139; x += 0.1f) {
    for (f32 z = -540; z < -519; z += 0.1f) {
      Vec3f pos = {x, 0, z};
      Vec3f result;
      if (platformPath(pos, angle, &result)) {
        printf(
            "angle=%04x startx=%.9g (%08x) startz=%.9g (%08x) endx=%.9g (%08x) "
            "endz=%.9g (%08x)\n",
            angle, pos.x, floatToInt(pos.x), pos.z, floatToInt(pos.z), result.x,
            floatToInt(result.x), result.z, floatToInt(result.z));
      }
    }
  }
}

bool testBackflip(Vec3f pos, u16 angle, bool debug) {
  for (int i = 0; i < 8; i++) {
    pos = translateWithPush(pos, angle, -6.0f, 0.0f);
  }

  if (pos.z < -633) {
    if (debug) {
      printf("backflip works\n");
    }
    return true;
  }

  return false;
}

void findBackflips() {
  for (int angle = -0x2000; angle <= 0x2000; angle += 0x10) {
    for (f32 x = -3181; x < -3139; x += 0.1f) {
      for (f32 z = -561; z < -550; z += 0.1f) {
        Vec3f pos = {x, 0, z};
        if (testBackflip(pos, angle, false)) {
          printf("angle=%04x x=%.9g (%08x) z=%.9g (%08x)\n", (u16)angle, pos.x,
                 floatToInt(pos.x), pos.z, floatToInt(pos.z));
        }
      }
    }
  }
}

void findTwistedMegaLandings() {
  for (int angle = -0x2000; angle <= 0x2000; angle += 0x08) {
    for (f32 x = -3181; x < -3139; x += 0.1f) {
      for (f32 z = -540; z < -519; z += 0.1f) {
        Vec3f pos = {x, 0, z};
        Vec3f result;
        if (platformPath(pos, angle + 0xbb8, &result) &&
            testBackflip(result, angle, false)) {
          printf(
              "angle=%04x startx=%.9g startx_raw=%08x startz=%.9g "
              "startz_raw=%08x endx=%.9g "
              "endx_raw=%08x endz=%.9g endz_raw=%08x\n",
              (u16)angle, pos.x, floatToInt(pos.x), pos.z, floatToInt(pos.z),
              result.x, floatToInt(result.x), result.z, floatToInt(result.z));
        }
      }
    }
  }
}

bool testMegaflip(Vec3f pos, u16 angle, bool debug) {
  u16 facingAngle = angle;
  u16 movementAngle = cameraAngles[angle] + 0x8000;

  pos = translate(pos, movementAngle, 1.5f, 0.0f);
  pos = translate(pos, movementAngle, 1.5f, 0.0f);

  for (int i = 0; i < 11; i++) {
    Math_ScaledStepToS(&movementAngle, facingAngle, 2000);
    Math_ScaledStepToS(&facingAngle, movementAngle, 2000);

    if (debug) {
      printf("i=%d x=%.9g (%08x) z=%.9g (%08x) movementAngle=%04x\n", i, pos.x,
             floatToInt(pos.x), pos.z, floatToInt(pos.z), movementAngle);
    }

    pos = translate(pos, movementAngle, 3.0f, 0.0f);
    if (pos.z < -281) {
      return false;
    }
  }

  movementAngle -= 2 * 0xbb8;
  f32 ySpeed = 4.8f;
  pos = translate(pos, movementAngle, -6.0f, 5.8f);

  movementAngle -= 2 * 0xbb8;
  for (int i = 0;; i++) {
    if (debug) {
      printf(
          "i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f "
          "movementAngle=%04x\n",
          i, pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
          floatToInt(pos.z), ySpeed, movementAngle);
    }

    ySpeed -= 1.0f;
    pos = translate(pos, movementAngle, -18.0f, ySpeed);

    if (onPlatform(pos)) {
      pos.y = 760;
      break;
    }

    if (pos.y < 735) {
      if (debug) {
        printf("fell too far\n");
      }
      return false;
    }
  }

  pos = translateWithPush(pos, movementAngle, -17.0f, 0.0f);
  if (!onPlatform(pos)) {
    if (debug) {
      printf("pushed off platform\n");
    }
    return false;
  }

  return testBackflip(pos, angle, debug);
}

void findMegaflips() {
  unsigned long long tested = 0;
  int found = 0;
  for (u16 angle = 0x0000; angle <= 0x1000; angle += 0x8) {
    for (f32 x = -3061; x < -3000; x += 0.1f) {
      for (f32 z = -281; z < -190; z += 0.1f) {
        if (tested % 1000000 == 0) {
          fprintf(stderr,
                  "tested=%llu found=%d angle=%04x x=%.2f z=%.2f ... \r",
                  tested, found, angle, x, z);
        }
        tested++;

        Vec3f pos = {x, 760, z};
        if (testMegaflip(pos, angle, false)) {
          found++;
          printf(
              "angle=%04x x=%.9g x_raw=%08x z=%.9g "
              "z_raw=%08x\n",
              angle, pos.x, floatToInt(pos.x), pos.z, floatToInt(pos.z));
        }
      }
    }
  }
}

bool nextCombination(std::vector<int>& v, int n) {
  int k = v.size();
  for (int i = k - 1; i >= 0; i--) {
    if (v[i] < n - 1) {
      v[i]++;
      for (int j = i + 1; j < k; j++) {
        v[j] = v[j - 1] + 1;
      }
      return true;
    }
  }
  return false;
}

void findPosAngleSetups(Collision* col) {
  // door opening
  Vec3f initialPos = {intToFloat(0xc534084f), 760, intToFloat(0xc33368f6)};
  u16 initialAngle = 0xc000;
  std::vector<Action> angleSetup = {TURN_ESS_LEFT, TURN_LEFT};

  // corner
  // Vec3f initialPos = {-2838, 760, -98};
  // u16 initialAngle = 0x0000;
  // std::vector<Action> angleSetup = {TURN_ESS_LEFT};

  // corner
  // Vec3f initialPos = {-2838, 760, -98};
  // u16 initialAngle = 0x4000;
  // std::vector<Action> angleSetup = {TURN_RIGHT, TURN_ESS_LEFT};

  std::vector<Action> addlActions = {
      ROLL,
      LONG_ROLL,
      SHIELD_SCOOT,
      BACKFLIP,
      BACKFLIP_SIDEROLL,
      SIDEHOP_LEFT,
      SIDEHOP_LEFT_SIDEROLL,
      SIDEHOP_RIGHT,
      SIDEHOP_RIGHT_SIDEROLL,

      HORIZONTAL_SLASH,
      HORIZONTAL_SLASH_SHIELD,
      DIAGONAL_SLASH,
      DIAGONAL_SLASH_SHIELD,
      VERTICAL_SLASH,
      VERTICAL_SLASH_SHIELD,
      FORWARD_STAB,
      FORWARD_STAB_SHIELD,
      CROUCH_STAB,
  };

  int tested = 0;
  int found = 0;

  std::vector<Action> actions;
  actions.reserve(10);
  for (int k = 2; k <= 4; k++) {
    std::vector<int> indices(k, 0);
    do {
      actions = angleSetup;
      for (int i = 0; i < k; i++) {
        actions.push_back(addlActions[indices[i]]);
      }
      std::sort(actions.begin(), actions.end());

      do {
        if (tested % 1000 == 0) {
          fprintf(stderr, "tested=%d found=%d k=%d ", tested, found, k);
          for (int i = 0; i < k; i++) {
            fprintf(stderr, "%d ", indices[i]);
          }

          fprintf(stderr, "... \r");
        }
        tested++;

        PosAngleSetup setup(col, initialPos, initialAngle,
                            {-10000, 760, -10000}, {10000, 10000, 10000});
        if (!setup.performActions(actions)) {
          continue;
        }

        if (!setup.performAction(BACKFLIP_SIDEROLL)) {
          continue;
        }

        if (testMegaflip(setup.pos, setup.angle, false)) {
          found++;
          printf("cost=%d angle=%04x x=%.9g (%08x) z=%.9g (%08x) actions=",
                 actionsCost(actions), setup.angle, setup.pos.x,
                 floatToInt(setup.pos.x), setup.pos.z, floatToInt(setup.pos.z));
          for (Action action : actions) {
            printf("%s,", actionName(action));
          }
          printf("\n");
          fflush(stdout);
        }
      } while (std::next_permutation(actions.begin(), actions.end()));
    } while (nextCombination(indices, addlActions.size()));
  }
}

int main(int argc, char* argv[]) {
  Collision col(&MIZUsin_sceneCollisionHeader_013C04, PLAYER_AGE_ADULT,
                {-3060, 760, -280}, {-2820, 760, -80});

  // findPlatformPaths();

  // testBackflip({-3160, 0, -561}, 0x0000);
  // findBackflips();

  // findTwistedMegaLandings();

  // testMegaflip({intToFloat(0xc53ea4c3), 760, intToFloat(0xc37d2996)}, 0x0708,
  //              true);

  // findMegaflips();

  findPosAngleSetups(&col);

  return 0;
}
