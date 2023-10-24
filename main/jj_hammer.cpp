#include "actor.hpp"
#include "camera.hpp"
#include "collision_data.hpp"
#include "global.hpp"
#include "pos_angle_setup.hpp"
#include "sys_math.hpp"

Vec3f instantBombOffset = {-8.56731033f, 0.0f, 3.83789444f};

Vec3s dropBomb(Vec3f pos, u16 angle) {
  return (pos + rotate(instantBombOffset, angle)).toVec3s();
}

Vec3f bombPushDisplacement(Vec3f linkPos, Vec3s bombPosTrunc) {
  Vec3s linkPosTrunc = linkPos.toVec3s();

  f32 xDelta = linkPosTrunc.x - bombPosTrunc.x;
  f32 zDelta = linkPosTrunc.z - bombPosTrunc.z;

  f32 xzDist = sqrtf(SQ(xDelta) + SQ(zDelta));
  f32 overlap = 18 - xzDist;

  if (overlap <= 0) {
    return Vec3f();
  }

  f32 dispRatio = 0.8f;

  if (xzDist != 0.0f) {
    xDelta *= overlap / xzDist;
    zDelta *= overlap / xzDist;
    return Vec3f(xDelta * dispRatio, 0, zDelta * dispRatio);
  } else {
    return Vec3f(-overlap * dispRatio, 0, 0);
  }
}

bool simulateRecoil(Vec3f pos, u16 angle, Vec3s bomb1Pos, Vec3s bomb2Pos,
                    Vec3s bomb3Pos, bool debug) {
  // hammer recoil
  pos = translate(pos, angle, -18.0f, 0.0f);

  if (debug) {
    printf("hammer recoil: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  // ensure can open door
  // door pos: (119, -3010)
  // < 20 units in x direction and < 50 units in z direction
  if (!(pos.x > 99.0f && pos.x < 139.0f && pos.z < -2960.0f)) {
    return false;
  }

  // hammer recoil + bomb push
  Vec3f displacement = bombPushDisplacement(pos, bomb1Pos) +
                       bombPushDisplacement(pos, bomb2Pos) +
                       bombPushDisplacement(pos, bomb3Pos);
  pos = translate(pos, angle, -10.0f, 0.0f, displacement);

  if (debug) {
    printf("bomb push: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  // walk through door
  pos = translate(pos, 0x8000, 0.1f, 0.0f);
  pos = translate(pos, 0x8bb8, 2.1f, 0.0f);
  pos = translate(pos, 0x8000, 4.1f, 0.0f);
  for (int i = 0; i < 6; i++) {
    pos = translate(pos, 0x7faf, 5.0f, 0.0f);
  }
  for (int i = 0; i < 6; i++) {
    pos = translate(pos, 0x7fa4, 5.0f, 0.0f);
  }
  pos = translate(pos, 0x7f9a, 5.0f, 0.0f);
  pos = translate(pos, 0x7f9a, 3.5f, 0.0f);
  pos = translate(pos, 0x7f9a, 2.0f, 0.0f);
  pos = translate(pos, 0x7f9a, 0.5f, 0.0f);

  if (debug) {
    printf("final position: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  // ensure inside door frame
  return ((pos.x < 85.0f || pos.x > 153.0f) && pos.z > -3026.0f);
}

void findRecoils() {
  Vec3s bomb1Pos = {99, 467, -2961};
  Vec3s bomb2Pos = {99, 467, -2961};
  Vec3s bomb3Pos = {100, 467, -2962};
  f32 z = -2982.0f;

  for (u16 angle = 0x6000; angle <= 0x7000; angle += 0x8) {
    f32 xmin = 1000.0f;
    f32 xmax = 0.0f;
    bool found = false;
    for (f32 x = 110.0f; x <= 120.0f; x += 0.001f) {
      Vec3f pos = {x, 467, z};
      if (simulateRecoil(pos, angle, bomb1Pos, bomb2Pos, bomb3Pos, false)) {
        found = true;
        xmin = std::min(xmin, x);
        xmax = std::max(xmax, x);
      }
    }

    if (found) {
      printf("angle=%04x xmin=%.9g xmax=%.9g range=%.9g\n", angle, xmin, xmax,
             xmax - xmin);
    }
  }
}

void findBombDrops() {
  Vec3s bomb1TargetPos = {99, 467, -2961};
  Vec3s bomb2TargetPos = {100, 467, -2962};
  int i = 0;
  for (u16 angle = 0x2000; angle < 0x7000; angle += 0x8) {
    for (f32 x = 90.0; x <= 100.0f; x += 0.01f) {
      for (f32 z = -2972.0; z <= -2960.0f; z += 0.01f) {
        if (i % 10000000 == 0) {
          fprintf(stderr, "i=%d angle=%04x x=%.9g z=%.9g ...\r", i, angle, x,
                  z);
        }
        i++;

        Vec3f pos = {x, 467, z};
        Vec3s bomb1Pos = dropBomb(pos, angle);
        Vec3s bomb2Pos = dropBomb(pos, angle + ESS);

        if (bomb1Pos == bomb1TargetPos && bomb2Pos == bomb2TargetPos) {
          printf("angle=%04x x=%.9g z=%.9g\n", angle, x, z);
        }
      }
    }
  }
}

u16 targetUpAngle(Collision* col, Vec3f pos, u16 angle) {
  int setting = 4;
  Camera camera(col);
  camera.initParallel(pos, angle, setting);
  return camera.yaw();
}

u16 essUpAngle(Collision* col, Vec3f pos, u16 angle) {
  int setting = 4;
  Camera camera(col);
  camera.initParallel(pos, angle, setting);
  camera.updateNormal(pos, angle, setting);
  return camera.yaw();
}

void walkToDoor(Collision* col, Vec3f pos, u16 angle, int numFrames,
                bool releaseTarget, bool shield, Vec3f* outPos, u16* outAngle,
                bool debug) {
  f32 speed = 0.0f;
  u16 movementAngle = targetUpAngle(col, pos, angle);

  for (int i = 1; i < numFrames + 6; i++) {
    if (debug) {
      printf(
          "frame %d: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) speed=%.9g "
          "angle=%04x\n",
          i, pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
          floatToInt(pos.z), speed, movementAngle);
    }

    CollisionPoly* wallPoly;
    CollisionPoly* floorPoly;
    f32 floorHeight;
    pos = col->runChecks(pos, translate(pos, movementAngle, speed, 0.0f),
                         &wallPoly, &floorPoly, &floorHeight);

    f32 targetSpeed;
    if (i < numFrames) {
      targetSpeed = 6.0f;
      if (wallPoly != nullptr) {
        u16 angleDiff = movementAngle - 0x8000;
        targetSpeed = targetSpeed * (std::abs((s16)angleDiff) * 0.00008f);
      }
    } else {
      targetSpeed = 0.0f;
    }

    if (i > numFrames && shield) {
      speed = 0.0f;
    } else if (!(i == numFrames && releaseTarget)) {
      // when releasing target, speed remains the same
      if (speed < targetSpeed) {
        speed = std::min(speed + 2.0f, targetSpeed);
      } else {
        speed = std::max(speed - 1.5f, targetSpeed);
      }
    }

    if (i == numFrames && !releaseTarget) {
      movementAngle = angle;
    }
  }

  if (debug) {
    printf("result: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) angle=%04x\n",
           pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z), movementAngle);
  }

  *outPos = pos;
  *outAngle = movementAngle;
}

bool simulateSetup(Collision* col, Vec3f pos, u16 walkAngle, u16 bomb1Angle,
                   u16 bomb2Angle, int walkFrames, bool releaseTarget,
                   bool shield, bool debug) {
  Vec3s bomb1TargetPos = {99, 467, -2961};
  Vec3s bomb2TargetPos = {100, 467, -2962};

  Vec3s bomb1Pos = dropBomb(pos, bomb1Angle);
  Vec3s bomb2Pos = dropBomb(pos, bomb2Angle);

  if (!(bomb1Pos == bomb1TargetPos && bomb2Pos == bomb2TargetPos)) {
    return false;
  }
  if (debug) {
    printf(
        "-- testing setup: walkAngle=%04x bomb1Angle=%04x bomb2Angle=%04x "
        "x=%.9g z=%.9g "
        "walkFrames=%d --\n",
        walkAngle, bomb1Angle, bomb2Angle, pos.x, pos.z, walkFrames);
  }

  Vec3f recoilPos;
  u16 recoilAngle;
  walkToDoor(col, pos, walkAngle, walkFrames, releaseTarget, shield, &recoilPos,
             &recoilAngle, debug);

  return simulateRecoil(recoilPos, recoilAngle, bomb1TargetPos, bomb1TargetPos,
                        bomb2TargetPos, debug);
}

void findSetups(Collision* col) {
  int i = 0;
  for (u16 angle = 0x6480; angle <= 0x6680; angle++) {
    for (f32 x = 90.0f; x <= 100.0f; x += 0.05f) {
      for (f32 z = -2971.5f; z <= -2961.5f; z += 0.05f) {
        Vec3f pos = {x, 467, z};
        for (int essTurns = -5; essTurns <= 5; essTurns++) {
          if (essTurns == 0) {
            continue;
          }

          u16 walkAngle = essUpAngle(col, pos, angle);
          u16 bomb1Angle;
          u16 bomb2Angle;

          if (essTurns < 0) {
            bomb1Angle = angle + essTurns * ESS;
            bomb2Angle = walkAngle;
          } else {
            bomb1Angle = walkAngle;
            bomb2Angle = angle + essTurns * ESS;
          }

          for (int walkFrames = 2; walkFrames <= 8; walkFrames++) {
            for (int releaseTarget = 0; releaseTarget <= 1; releaseTarget++) {
              for (int shield = 0; shield <= 1; shield++) {
                if (shield && !releaseTarget) {
                  continue;
                }

                if (i % 1000000 == 0) {
                  fprintf(stderr, "i=%d angle=%04x x=%.9g z=%.9g ...\r", i,
                          angle, x, z);
                }
                i++;

                if (simulateSetup(col, pos, walkAngle, bomb1Angle, bomb2Angle,
                                  walkFrames, releaseTarget, shield, false)) {
                  printf(
                      "angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x "
                      "essTurns=%d walkFrames=%d releaseTarget=%d "
                      "shield=%d\n",
                      angle, x, z, floatToInt(x), floatToInt(z), essTurns,
                      walkFrames, releaseTarget, shield);
                }
              }
            }
          }
        }
      }
    }
  }
}

bool testPosAngleSetup(Collision* col, const std::vector<Action>& actions,
                       Vec3f initialPos, u16 initialAngle, bool debug) {
  PosAngleSetup setup(col, initialPos, initialAngle);
  for (int i = 0; i < actions.size(); i++) {
    Action action = actions[i];
    if (setup.pos.z <= -2972 && ((i > 0 && (actions[i - 1] == TURN_ESS_RIGHT ||
                                            actions[i - 1] == TURN_ESS_LEFT)) ||
                                 (i == 0 && setup.angle != 0x8000))) {
      // against wall; only allow actions that don't require targeting
      switch (action) {
        case TURN_ESS_LEFT:
        case TURN_ESS_RIGHT:
        case HORIZONTAL_SLASH:
        case HORIZONTAL_SLASH_SHIELD:
        case CROUCH_STAB:
          break;
        default:
          return false;
      }
    }

    if (!setup.performAction(action)) {
      return false;
    }

    if (debug) {
      printf("%s: x=%.9g z=%.9g x_raw=%08x z_raw=%08x\n", actionName(action),
             setup.pos.x, setup.pos.z, floatToInt(setup.pos.x),
             floatToInt(setup.pos.z));
    }
  }

  Vec3f pos = setup.pos;
  u16 angle = essUpAngle(col, pos, setup.angle);

  if (debug) {
    printf("angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x\n", angle, pos.x,
           pos.z, floatToInt(pos.x), floatToInt(pos.z));
  }

  u16 cameraAngle = essUpAngle(col, pos, angle);
  u16 walkAngle = targetUpAngle(col, pos, cameraAngle);
  u16 bomb1Angle = angle - ESS;
  u16 bomb2Angle = cameraAngle;
  int walkFrames = 5;
  bool releaseTarget = false;
  bool shield = false;

  return simulateSetup(col, pos, walkAngle, bomb1Angle, bomb2Angle, walkFrames,
                       releaseTarget, shield, debug);
}

// TODO: put this somewhere
bool nextCombinationWithRepetition(std::vector<int>& v, int n) {
  int k = v.size();
  if (k == 0) {
    return false;
  }
  v[k - 1]++;
  for (int i = k - 1; i >= 0; i--) {
    if (v[i] >= n) {
      if (i == 0) {
        return false;
      }
      v[i - 1]++;
      for (int j = i; j < k; j++) {
        v[j] = v[j - 1];
      }
    }
  }
  return true;
}

void findPosAngleSetups(Collision* col) {
  std::vector<Vec3f> initialPositions = {
      {53, 467, -2972},
      {185, 467, -2972},
  };

  // face door, target
  u16 initialAngle = 0x8000;
  std::vector<Action> angleSetup = {TURN_ESS_RIGHT, TURN_ESS_RIGHT,
                                    TURN_ESS_RIGHT, TURN_ESS_RIGHT};

  // face right wall, sidehop sideroll left
  // u16 initialAngle = 0x4000;
  // std::vector<Action> angleSetup = {SIDEHOP_SIDEROLL_LEFT, TURN_ESS_LEFT};

  std::vector<Action> addlActions = {
      ROLL,
      SIDEHOP_LEFT,
      SIDEHOP_RIGHT,
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

  for (int k = 1; k <= 4; k++) {
    std::vector<int> indices(k, 0);
    std::vector<Action> actions;
    actions.reserve(20);

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
            fprintf(stderr, "%i ", indices[i]);
          }
          fprintf(stderr, "... \r");
        }
        tested++;

        for (const Vec3f& initialPos : initialPositions) {
          if (testPosAngleSetup(col, actions, initialPos, initialAngle,
                                false)) {
            found++;

            int cost = 0;
            for (int i = 0; i < actions.size(); i++) {
              if (i > 0 && actions[i] == TURN_ESS_RIGHT &&
                  actions[i - 1] == TURN_ESS_RIGHT) {
                cost += 1;
              } else {
                cost += actionCost(actions[i]);
              }
            }

            printf("cost=%d startx=%.0f startAngle=%04x actions=", cost,
                   initialPos.x, initialAngle);
            for (Action action : actions) {
              printf("%s,", actionName(action));
            }
            printf("\n");
            fflush(stdout);
          }
        }
      } while (std::next_permutation(actions.begin(), actions.end()));
    } while (nextCombinationWithRepetition(indices, addlActions.size()));
  }
}

int main(int argc, char* argv[]) {
  Collision col(&Bmori1_sceneCollisionHeader_014054, PLAYER_AGE_ADULT,
                {20, 467, -3020}, {220, 467, -2990});

  // simulateRecoil({intToFloat(0x42e61486), 467, intToFloat(0xc53a5bc7)},
  // 0x6619,
  //                {99.1602f, 467, -2961.02f}, {99.1602f, 467, -2961.02f},
  //                {100.076f, 467, -2962.81f});

  // findRecoils();
  // findBombDrops();

  // Vec3f pos;
  // u16 angle;
  // walkToDoor(&col, {intToFloat(0x42b605c2), 467, intToFloat(0xc5394e1b)},
  //            0x6561, 7, false, false, &pos, &angle, true);

  // simulateSetup(&col, {intToFloat(0x42b619a2), 467, intToFloat(0xc5394fe6)},
  //               0x6561, 0x5d9a, 0x6561, 4, true, false, true);

  // findSetups(&col);

  findPosAngleSetups(&col);

  return 0;
}
