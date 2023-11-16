#include <string>
#include <vector>

#include "actor.hpp"
#include "collider.hpp"
#include "collision_data.hpp"
#include "global.hpp"
#include "pos_angle_setup.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"

Vec3f move(Collision* col, Vec3f pos, u16 angle, f32 xzSpeed) {
  return col->runChecks(pos, translate(pos, angle, xzSpeed, -5.0f));
}

// y offset for explosion is 9.8f (starts at 7, scaled by 1.4 for after big blue
// / big red)
Vec3f overheadBombOffset = {-0.981463f, 9.8f, -0.510888f};
Vec3f instantBombOffset = {-8.56731033f, 9.8f, 3.83789444f};
// starts frame 8
Vec3f superslideShieldCorners[8][4] = {
    {
        {-32.38439f, 52.01130f, -7.214777f},
        {-28.87611f, -3.823368f, -28.87302f},
        {27.50389f, 55.41977f, -6.304045f},
        {31.01217f, -0.4148884f, -27.96229},
    },
    {
        {-32.38439f, 50.86442f, 15.27922f},
        {-28.87611f, 16.05066f, -33.44962f},
        {27.50389f, 53.21526f, 17.90988f},
        {31.01217f, 18.40151f, -30.81896f},
    },
    {
        {-32.37439f, 45.81742f, 24.51334f},
        {-28.86611f, 25.10391f, -31.67852f},
        {27.51390f, 47.39033f, 27.67136f},
        {31.02217f, 26.67682f, -28.52050f},
    },
    {
        {-32.37439f, 32.04693f, 37.17230f},
        {-28.86611f, 39.20173f, -22.28581f},
        {27.51390f, 32.00875f, 40.70009f},
        {31.02217f, 39.16355f, -18.75803f},
    },
    {
        {-32.36439f, 18.14089f, 42.02077f},
        {-28.85611f, 47.12396f, -10.38762f},
        {27.52390f, 16.77926f, 45.27552f},
        {31.03218f, 45.76233f, -7.132877f},
    },
    {
        {-32.43439f, 5.302599f, 42.65164f},
        {-28.92611f, 48.51745f, 1.191589f},
        {27.45390f, 3.037409f, 45.35638f},
        {30.96218f, 46.25226f, 3.896332f},
    },
    {
        {-32.45439f, -3.501917f, 41.67535f},
        {-28.94611f, 47.75484f, 10.70161f},
        {27.43390f, -6.305208f, 43.81750f},
        {30.94217f, 44.95155f, 12.84377f},
    },
    {
        {-31.97439f, -4.826283f, 39.41304f},
        {-28.46611f, 50.31640f, 16.04967f},
        {27.91389f, -7.905888f, 41.13441f},
        {31.42217f, 47.23680f, 17.77105f},
    },
};

bool testCompactSuperslideWorks(Vec3f pos, Vec3f bombPos, u16 angle,
                                int bombTimer, int grabFrame) {
  if (grabFrame > bombTimer) {
    return false;  // bomb to grab explodes too
  }

  bool hitShield = false;
  for (int f = grabFrame + 3; f <= 15; f++) {
    f32 xzSpeed = hitShield ? -18.0f : 0.0f;

    int explosionFrame = f - bombTimer;
    if (explosionFrame > 0) {
      Sphere16 sphere = {bombPos.toVec3s(), (s16)(8 * explosionFrame)};
      Cylinder16 cylinder = {12, 53, 5, pos.toVec3s()};

      Vec3f quad[4];
      for (int i = 0; i < 4; i++) {
        quad[i] = pos + rotate(superslideShieldCorners[f - 8][i], angle);
      }

      TriNorm tri1, tri2;
      Vec3f hitPos;
      f32 overlapSize;
      Math3D_TriNorm(&tri1, &quad[2], &quad[3], &quad[1]);
      Math3D_TriNorm(&tri2, &quad[2], &quad[1], &quad[0]);
      if (Math3D_TriVsSphIntersect(&sphere, &tri1, &hitPos) ||
          Math3D_TriVsSphIntersect(&sphere, &tri2, &hitPos)) {
        if (f == 15) {
          return false;  // recoil
        }

        hitShield = true;
      } else if (Math3D_SphVsCylOverlap(&sphere, &cylinder, &overlapSize)) {
        return false;  // hits Link
      }
    }

    pos = translate(pos, angle, xzSpeed, 0.0f);
  }

  return hitShield;
}

void compactSuperslideFrameData() {
  u16 angle = 0x0000;
  Vec3f linkPos = rotate({0, 0, -58.5f}, angle);
  Vec3f bombPos = rotate(instantBombOffset, angle);

  for (int bombTimer = 15; bombTimer > 0; bombTimer--) {
    printf("bombTimer=%d\n", bombTimer);
    for (int grabFrame = 5; grabFrame <= 11; grabFrame++) {
      Vec3f pos = linkPos;
      pos = translate(pos, angle, 2.0f, 0.0f);
      for (int i = 2; i < grabFrame; i++) {
        pos = translate(pos, angle, 3.0f, 0.0f);
      }

      if (testCompactSuperslideWorks(pos, bombPos, angle, bombTimer,
                                     grabFrame)) {
        printf("  grabFrame=%d\n", grabFrame);
      }
    }
  }
}

bool testClip(Collision* col, Vec3f pos, u16 angle, bool debug) {
  if (debug) {
    printf("x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) angle=%04x\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z), angle);
  }

  Vec3f posResult = move(col, pos, angle, -18);

  if (debug) {
    printf("posResult: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n",
           posResult.x, floatToInt(posResult.x), posResult.y,
           floatToInt(posResult.y), posResult.z, floatToInt(posResult.z));
  }

  Vec3f posAfter = move(col, posResult, angle, -18);
  bool clipped = (posAfter.z < -90.0f);

  if (debug) {
    printf("posAfter: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) clipped=%d\n",
           posAfter.x, floatToInt(posAfter.x), posAfter.y,
           floatToInt(posAfter.y), posAfter.z, floatToInt(posAfter.z), clipped);
  }

  return clipped;
}

void findClipRegion() {
  Collision col(&ydan_sceneCollisionHeader_00B618, PLAYER_AGE_ADULT,
                {-1355, -820, -80}, {-1355, -800, -60});
  int i = 0;

  for (u16 angle = 0xf920; angle <= 0xff30; angle += 0x10) {
    for (f32 x = -1360.5f; x < -1357.0f; x += 0.01f) {
      for (f32 z = -54.0f; z < -52.0f; z += 0.01f) {
        Vec3f pos = col.findFloor({x, -800, z});

        if (i % 100000 == 0) {
          fprintf(stderr, "angle=%04x x=%.9g y=%.9g z=%.9g\r", angle, pos.x,
                  pos.y, pos.z);
        }
        i++;

        if (testClip(&col, pos, angle, false)) {
          printf(
              "angle=%04x x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x "
              "z_raw=%08x\n",
              angle, pos.x, pos.y, pos.z, floatToInt(pos.x), floatToInt(pos.y),
              floatToInt(pos.z));
        }
      }
    }
  }
}

#define BOMB_SETUPS               \
  X(SETUP_ROLL_BACKFLIP)          \
  X(SETUP_ROLL_BACKFLIP_INSTANT)  \
  X(SETUP_ROLL_BACKFLIP_OVERHEAD) \
  X(SETUP_BACKFLIP_ROLL)          \
  X(SETUP_BACKFLIP_INSTANT_ROLL)  \
  X(SETUP_BACKFLIP_OVERHEAD_ROLL) \
  X(SETUP_BACKFLIP_ROLL_INSTANT)  \
  X(SETUP_BACKFLIP_ROLL_OVERHEAD) \
  X(SETUP_BACKFLIP)               \
  X(SETUP_BACKFLIP_INSTANT)       \
  X(SETUP_BACKFLIP_OVERHEAD)      \
  X(BOMB_SETUPS_MAX)

enum BombSetup {
#define X(name) name,
  BOMB_SETUPS
#undef X
};

const char* bombSetupNames[] = {
#define X(name) #name,
    BOMB_SETUPS
#undef X
};

bool testSuperslide(Collision* col, Vec3f startPos, u16 angle,
                    BombSetup bombSetup, bool* outHoldUp, int* outGrabFrame,
                    int* outMinBombTimer, int* outMaxBombTimer, bool debug) {
  PosAngleSetup setup(col, startPos, angle);

  bool useBombPush = false;
  Vec3f bombPos;
  switch (bombSetup) {
    case SETUP_ROLL_BACKFLIP:
      setup.performAction(ROLL);
      setup.performAction(BACKFLIP);
      break;
    case SETUP_ROLL_BACKFLIP_INSTANT:
      setup.performAction(ROLL);
      setup.performAction(BACKFLIP);
      bombPos = setup.pos + rotate(instantBombOffset, angle);
      useBombPush = true;
      break;
    case SETUP_ROLL_BACKFLIP_OVERHEAD:
      setup.performAction(ROLL);
      setup.performAction(BACKFLIP);
      bombPos = setup.pos + rotate(overheadBombOffset, angle);
      useBombPush = true;
      break;
    case SETUP_BACKFLIP_ROLL:
      setup.performAction(BACKFLIP);
      setup.performAction(ROLL);
      break;
    case SETUP_BACKFLIP_INSTANT_ROLL:
      setup.performAction(BACKFLIP);
      bombPos = setup.pos + rotate(instantBombOffset, angle);
      setup.performAction(ROLL);
      useBombPush = true;
      break;
    case SETUP_BACKFLIP_OVERHEAD_ROLL:
      setup.performAction(BACKFLIP);
      bombPos = setup.pos + rotate(overheadBombOffset, angle);
      setup.performAction(ROLL);
      useBombPush = true;
      break;
    case SETUP_BACKFLIP_ROLL_INSTANT:
      setup.performAction(BACKFLIP);
      setup.performAction(ROLL);
      bombPos = setup.pos + rotate(instantBombOffset, angle);
      useBombPush = true;
      break;
    case SETUP_BACKFLIP_ROLL_OVERHEAD:
      setup.performAction(BACKFLIP);
      setup.performAction(ROLL);
      bombPos = setup.pos + rotate(overheadBombOffset, angle);
      useBombPush = true;
      break;
    case SETUP_BACKFLIP:
      setup.performAction(BACKFLIP);
      break;
    case SETUP_BACKFLIP_INSTANT:
      setup.performAction(BACKFLIP);
      bombPos = setup.pos + rotate(instantBombOffset, angle);
      useBombPush = true;
      break;
    case SETUP_BACKFLIP_OVERHEAD:
      setup.performAction(BACKFLIP);
      bombPos = setup.pos + rotate(overheadBombOffset, angle);
      useBombPush = true;
      break;
    case BOMB_SETUPS_MAX:
      assert(false);
      break;
  }

  Vec3f rollPos = setup.pos;

  if (debug) {
    printf("after setup: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n",
           rollPos.x, floatToInt(rollPos.x), rollPos.y, floatToInt(rollPos.y),
           rollPos.z, floatToInt(rollPos.z));
    printf("bombPos: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", bombPos.x,
           floatToInt(bombPos.x), bombPos.y, floatToInt(bombPos.y), bombPos.z,
           floatToInt(bombPos.z));
  }

  // Ensure not in grab range for roll
  if (Math_Vec3f_DistXZ(&rollPos, &startPos) < 50.0f) {
    return false;
  }

  for (int holdUp = 0; holdUp <= 1; holdUp++) {
    f32 maxSpeed = holdUp ? 9.0f : 3.0f;

    for (int grabFrame = 5; grabFrame <= 11; grabFrame++) {
      Vec3f grabPos = rollPos;
      f32 rollSpeed = 2.0f;
      for (int i = 0; i < grabFrame - 2; i++) {
        grabPos = move(col, grabPos, angle, rollSpeed);
        rollSpeed = std::min(rollSpeed + 2.0f, maxSpeed);
      }

      // Ensure in range to grab
      if (Math_Vec3f_DistXZ(&grabPos, &startPos) >= 50.0f) {
        continue;
      }

      grabPos = move(col, grabPos, angle, rollSpeed);

      // If bomb push, make sure bomb push box activates
      if (useBombPush && Math_Vec3f_DistXZ(&grabPos, &bombPos) <= 20.0f) {
        continue;
      }

      // Test explosion frames
      Vec3f superslideBombPos = startPos + rotate(instantBombOffset, angle);
      bool foundBombTimer = false;
      // TODO: this counts frame sandwiches
      int minBombTimer;
      int maxBombTimer;
      for (int bombTimer = 5; bombTimer <= 15; bombTimer++) {
        if (testCompactSuperslideWorks(grabPos, superslideBombPos, angle,
                                       bombTimer, grabFrame)) {
          if (!foundBombTimer) {
            foundBombTimer = true;
            minBombTimer = bombTimer;
          }
          maxBombTimer = bombTimer;
        }
      }

      if (!foundBombTimer) {
        continue;
      }

      // Test clip
      Vec3f pos = grabPos;
      for (int i = 0; i < 6; i++) {
        if (debug) {
          printf(
              "superslide: grabFrame=%d i=%d x=%.9g (%08x) y=%.9g (%08x) "
              "z=%.9g "
              "(%08x)\n",
              grabFrame, i, pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y),
              pos.z, floatToInt(pos.z));
        }

        Vec3f intendedPos = translate(pos, angle, -18.0f, -5.0f);
        if (useBombPush) {
          intendedPos = intendedPos + bombPush(pos, bombPos);
        }
        pos = col->runChecks(pos, intendedPos);

        if (pos.z < -90.0f) {
          *outHoldUp = holdUp;
          *outGrabFrame = grabFrame;
          *outMinBombTimer = minBombTimer;
          *outMaxBombTimer = maxBombTimer;
          return true;
        }
      }
    }
  }

  return false;
}

void findSuperslides(Collision* col, u16 angle) {
  bool debug = false;
  int i = 0;
  for (f32 x = -1382.0f; x < -1359.0f; x += 0.1f) {
    for (f32 z = 5.0f; z < 65.0f; z += 0.1f) {
      if (i % 1000 == 0) {
        fprintf(stderr, "i=%d angle=%04x x=%.9g z=%.9g    \r", i, angle, x, z);
      }
      i++;

      Vec3f pos = {x, -820, z};
      pos = col->runChecks(pos, pos);
      if (pos.y < -820 || pos.x != x || pos.z != z) {
        continue;
      }

      for (int bombSetup = 0; bombSetup < BOMB_SETUPS_MAX; bombSetup++) {
        bool holdUp;
        int grabFrame, minBombTimer, maxBombTimer;
        if (testSuperslide(col, pos, angle, static_cast<BombSetup>(bombSetup),
                           &holdUp, &grabFrame, &minBombTimer, &maxBombTimer,
                           debug)) {
          printf(
              "angle=%04x x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x "
              "z_raw=%08x bombSetup=%s holdUp=%d grabFrame=%d minBombTimer=%d "
              "maxBombTimer=%d bombTimerWindow=%d\n",
              angle, pos.x, pos.y, pos.z, floatToInt(pos.x), floatToInt(pos.y),
              floatToInt(pos.z), bombSetupNames[bombSetup], holdUp, grabFrame,
              minBombTimer, maxBombTimer, maxBombTimer - minBombTimer + 1);
        }
      }
    }
  }
}

void findWallSuperslides(Collision* col, u16 angle) {
  bool debug = false;
  int i = 0;
  for (f32 x = -1382.0f; x < -1359.0f; x += 0.1f) {
    if (i % 1000 == 0) {
      fprintf(stderr, "angle=%04x x=%.9g    \r", angle, x);
    }
    i++;

    Vec3f pos = {x, -820, 79};
    pos = col->runChecks(pos, pos);

    for (int bombSetup = 0; bombSetup < BOMB_SETUPS_MAX; bombSetup++) {
      bool holdUp;
      int grabFrame, minBombTimer, maxBombTimer;
      if (testSuperslide(col, pos, angle, static_cast<BombSetup>(bombSetup),
                         &holdUp, &grabFrame, &minBombTimer, &maxBombTimer,
                         debug)) {
        printf(
            "angle=%04x x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x "
            "z_raw=%08x bombSetup=%s holdUp=%d grabFrame=%d minBombTimer=%d "
            "maxBombTimer=%d bombTimerWindow=%d\n",
            angle, pos.x, pos.y, pos.z, floatToInt(pos.x), floatToInt(pos.y),
            floatToInt(pos.z), bombSetupNames[bombSetup], holdUp, grabFrame,
            minBombTimer, maxBombTimer, maxBombTimer - minBombTimer + 1);
      }
    }
  }
}

Vec3f setupMinBound = {-2200, -820, -350};
Vec3f setupMaxBound = {-1220, -730, 200};

int setupCost(std::vector<Action> actions) {
  int cost = 0;
  for (Action action : actions) {
    cost += actionCost(action);
  }
  return cost;
}

void testPosAngleSetup(Collision* col, Collision* setupCol,
                       const std::vector<Action>& actions) {
  Vec3f initialPos = {intToFloat(0xc503f666), -760, intToFloat(0xc3a31437)};
  u16 initialAngle = 0x9f8a;
  bool debug = false;

  PosAngleSetup setup(setupCol, initialPos, initialAngle, setupMinBound,
                      setupMaxBound);
  if (!setup.performActions(actions)) {
    return;
  }

  Vec3f pos = setup.pos;
  u16 angle = setup.angle;

  if (pos.x < -1380.0f || pos.x > -1360.0f || pos.z < 5.0f) {
    return;
  }

  for (int bombSetup = 0; bombSetup < BOMB_SETUPS_MAX; bombSetup++) {
    bool holdUp;
    int grabFrame, minBombTimer, maxBombTimer;
    if (testSuperslide(col, pos, angle, static_cast<BombSetup>(bombSetup),
                       &holdUp, &grabFrame, &minBombTimer, &maxBombTimer,
                       debug)) {
      printf(
          "cost=%d angle=%04x x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x "
          "z_raw=%08x bombSetup=%s holdUp=%d grabFrame=%d "
          "minBombTimer=%d "
          "maxBombTimer=%d bombTimerWindow=%d actions=",
          setupCost(actions), angle, pos.x, pos.y, pos.z, floatToInt(pos.x),
          floatToInt(pos.y), floatToInt(pos.z), bombSetupNames[bombSetup],
          holdUp, grabFrame, minBombTimer, maxBombTimer,
          maxBombTimer - minBombTimer + 1);
      for (Action action : actions) {
        printf("%s,", actionName(action));
      }
      printf("\n");
    }
  }
}

bool nextIndices(std::vector<int>& indices, int max) {
  int i = indices.size() - 1;
  while (i >= 0 && indices[i] == max) {
    indices[i] = 0;
    i--;
  }
  if (i < 0) {
    return false;
  }
  indices[i]++;
  return true;
}

void findPosAngleSetups(Collision* col) {
  Collision setupCol(&ydan_sceneCollisionHeader_00B618, PLAYER_AGE_ADULT,
                     setupMinBound, setupMaxBound);

  std::vector<Action> addlActions = {
      ROLL,
      BACKFLIP,
      // BACKFLIP_SIDEROLL,
      SIDEHOP_LEFT,
      SIDEHOP_LEFT_SIDEROLL,
      SIDEHOP_RIGHT,
      SIDEHOP_RIGHT_SIDEROLL,
  };

  std::vector<int> indices;
  indices.reserve(10);

  std::vector<Action> actions;
  actions.reserve(20);

  std::string bitmask;
  bitmask.reserve(20);

  int numSetups = 0;
  for (int n = 1; n <= 6; n++) {
    indices.clear();
    indices.resize(n, 0);

    do {
      // https://rosettacode.org/wiki/Combinations#C.2B.2B
      bitmask.clear();
      bitmask.resize(n, 1);
      bitmask.resize(n + 3, 0);
      do {
        for (int angleSetup = 0; angleSetup < 2; angleSetup++) {
          if (numSetups % 10000 == 0) {
            fprintf(stderr, "numSetups=%d n=%d ", numSetups, n);
            for (int i = 0; i < n; i++) {
              fprintf(stderr, "%d ", indices[i]);
            }
            fprintf(stderr, "        \r");
          }
          numSetups++;

          actions.clear();
          int j = 0;
          int k = 0;
          for (int i = 0; i < bitmask.size(); i++) {
            if (bitmask[i]) {
              actions.push_back(addlActions[indices[j]]);
              j++;
            } else {
              switch (k) {
                case 0:
                  actions.push_back(angleSetup ? TURN_LEFT : TURN_4_ESS_LEFT);
                  break;
                case 1:
                  actions.push_back(angleSetup ? TURN_4_ESS_LEFT : TURN_LEFT);
                  break;
                case 2:
                  actions.push_back(SIDEHOP_LEFT);
                  actions.push_back(SIDEHOP_LEFT);
                  actions.push_back(SIDEHOP_LEFT);
                  break;
              }
              k++;
            }
          }

          // Ban some first actions
          if (actions[0] == ROLL || actions[0] == SIDEHOP_LEFT_SIDEROLL) {
            continue;
          }

          testPosAngleSetup(col, &setupCol, actions);
        }
      } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
    } while (nextIndices(indices, addlActions.size() - 1));
  }
}

int main(int argc, char* argv[]) {
  Collision col(&ydan_sceneCollisionHeader_00B618, PLAYER_AGE_ADULT,
                {-1355, -820, -80}, {-1355, -800, 80});

  // findClipRegion();
  // compactSuperslideFrameData();

  // int grabFrame, minBombTimer, maxBombTimer;
  // testSuperslide(&col, {-1372.3f, -815.516f, 60.0f}, 0xfba0,
  //                SETUP_BACKFLIP_ROLL, &grabFrame, &minBombTimer,
  //                &maxBombTimer, true);

  // if (argc > 1) {
  //   u16 angle = strtoul(argv[1], NULL, 16);
  //   findSuperslides(&col, angle);
  //   // findWallSuperslides(&col, angle);
  // }

  // angle=fb00 x=-1375.802 y=-814.6407 z=55.69973 x_raw=c4abf9a6
  // y_raw=c44ba901 z_raw=425ecc85 rollFirst=0 bomb1=OVERHEAD_NEAR
  // bomb2=OVERHEAD_NEAR grabFrame=5

  // testSuperslide(
  //     &col,
  //     {intToFloat(0xc4abf9a6), intToFloat(0xc44ba901),
  //     intToFloat(0x425ecc85)}, 0xfb00, false, OVERHEAD_NEAR, OVERHEAD_NEAR,
  //     5, true);

  findPosAngleSetups(&col);

  return 0;
}
