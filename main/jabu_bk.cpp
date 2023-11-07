#include <algorithm>

#include "actor.hpp"
#include "camera.hpp"
#include "collision.hpp"
#include "collision_data.hpp"
#include "global.hpp"
#include "pos_angle_setup.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"

bool simulateHess(Collision* col, Vec3f pos, f32 speed, u16 startAngle,
                  bool debug) {
  u16 rot = startAngle + ESS;            // actor.world.rot.y
  u16 facingAngle = startAngle;          // yaw
  u16 movementAngle = startAngle + ESS;  // actor.shape.rot.y
  f32 yvel = -4.0f;                      // y velocity

  bool target = true;
  bool foundGap = false;

  for (int i = 0; i < 100; i++) {
    if (debug) {
      printf(
          "i=%03i x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x z_raw=%08x "
          "facing=%04x movement=%04x rot=%04x speed=%.9g yvel=%.9g\n",
          i, pos.x, pos.y, pos.z, floatToInt(pos.x), floatToInt(pos.y),
          floatToInt(pos.z), facingAngle, movementAngle, rot, speed, yvel);
    }

    if (pos.x > 300 && pos.y > -320 && pos.z < -1783) {
      return true;
    }

    if (pos.x >= 592) {
      return false;
    }

    Math_StepToF(&speed, -18, 0.35f);
    Math_ScaledStepToS(&rot, movementAngle, 1350);

    movementAngle = facingAngle + ESS;
    if (!target) {
      facingAngle = facingAngle + ESS;
    }

    yvel -= 1.0f;
    Vec3f posNext = translate(pos, rot, speed, yvel);

    // Collision check
    CollisionPoly* wallPoly;
    CollisionPoly* floorPoly;
    int dynaId;
    f32 floorHeight;
    posNext = col->runChecks(pos, posNext, &wallPoly, &floorPoly, &dynaId,
                             &floorHeight);

    if (posNext.y <= floorHeight) {  // on ground
      // Not technically correct but close enough. There's a frame of
      // 0 yvel when landing with hover boots.
      yvel = -4.0f;
      if (foundGap) {
        target = false;
      }
    } else if (posNext.y - floorHeight > 11.0f) {  // in air
      foundGap = true;
      yvel = 1.0f;
    }

    pos = posNext;
  }

  return false;
}

void findHessPositions(Collision* col, u16 angle, f32 speed) {
  int i = 0;
  u16 startAngle = angle - 7 * ESS;

  for (f32 x = -80; x < 80; x += 2.0f) {
    for (f32 z = -2180; z < -1980; z += 0.005f) {
      if (i % 10000 == 0) {
        fprintf(stderr, "angle=%04x x=%.9g z=%.9g\n", angle, x, z);
      }
      i++;

      Vec3f pos = {x, -340, z};
      pos = col->runChecks(pos, pos);
      if (pos.y < -340 || pos.x != x || pos.z != z) {
        continue;
      }

      // camera consistency check
      Camera camera(col);
      int setting = 3;  // TODO: use floor?
      u16 facingAngle = angle - (7 * 0x708) + 0x48;
      camera.initParallel(pos, facingAngle, setting);
      camera.updateNormal(pos, facingAngle, setting);
      if (camera.wallPoly) {
        continue;
      }

      if (simulateHess(col, pos, speed, startAngle, false)) {
        printf(
            "angle=%04x startAngle=%04x x=%.9g y=%.9g z=%.9g x_raw=%08x "
            "y_raw=%08x z_raw=%08x\n",
            angle, startAngle, pos.x, pos.y, pos.z, floatToInt(pos.x),
            floatToInt(pos.y), floatToInt(pos.z));
      }
    }
  }
}

bool testClip(Collision* col, f32 x, f32 z, u16 angle, bool debug) {
  Vec3f pos = col->findFloor({x, -300, z});

  if (debug) {
    printf("x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) angle=%04x\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z), angle);
  }

  Vec3f posNext = translate(pos, angle, -18.0f, -5.0f);

  if (debug) {
    printf("posNext: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", posNext.x,
           floatToInt(posNext.x), posNext.y, floatToInt(posNext.y), posNext.z,
           floatToInt(posNext.z));
  }

  Vec3f posResult = col->runChecks(pos, posNext);

  bool clipped = (posResult.y > -320.0f && posResult.z < -1783.0f);

  if (debug) {
    printf("posResult: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) clipped=%d\n",
           posResult.x, floatToInt(posResult.x), posResult.y,
           floatToInt(posResult.y), posResult.z, floatToInt(posResult.z),
           clipped);
  }

  return clipped;
}

void findClipRegion(Collision* col, f32 x) {
  int i = 0;

  for (u16 angle = 0xd800; angle < 0xe800; angle += 0x10) {
    bool found = false;
    f32 zmin;
    f32 zmax;

    int start = floatToInt(1764.0f);
    int end = floatToInt(1765.0f);

    for (int zi = start; zi < end; zi++) {
      f32 z = -intToFloat(zi);

      if (i % 100000 == 0) {
        fprintf(stderr, "angle=%04x z=%.9g\r", angle, z);
      }
      i++;

      if (!testClip(col, x, z, angle, false)) {
        continue;
      }

      if (!found) {
        found = true;
        zmax = z;
      } else {
        zmin = z;
      }
    }

    if (found) {
      printf("x: %.9g angle: %04x zmin: %.9g (%08x) zmax: %.9g (%08x)\n", x,
             angle, zmin, floatToInt(zmin), zmax, floatToInt(zmax));
    }
  }
}

u16 predictHessAngle(u16 angle) {
  Vec3f bombPos = rotate(Vec3f(-8.56731033f, 0.0f, 3.83789444f), angle);
  Vec3f linkPos = rotate(Vec3f(0, 0, -33), angle);
  return Math_Vec3f_Yaw(&linkPos, &bombPos);
}

void printHessAngles() {
  for (int angle = 0; angle < 0x10000; angle += 0x10) {
    u16 hessAngle = predictHessAngle(angle);
    printf("angle: %04x hessAngle: %04x\n", angle, hessAngle);
  }
}

Vec3f move(Collision* col, Vec3f pos, u16 angle, f32 speed) {
  return col->runChecks(pos, translate(pos, angle, speed, -5.0f));
}

bool testHessSetup(Collision* col, Vec3f pos, u16 angle,
                   std::vector<Action> actions, u16 essAngle, bool debug) {
  if (debug) {
    printf("setup pos: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) angle=%04x\n",
           pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z), angle);
  }

  if (!(-2082 <= pos.z && pos.z <= -1980)) {
    if (debug) {
      printf("z coordinate out of range\n");
    }
    return false;
  }

  // Drop bomb
  Vec3f bombPos = pos + rotate(Vec3f(-8.56731033f, 0.0f, 3.83789444f), angle);

  // Hess setup
  PosAngleSetup setup(col, pos, angle, {-140, -340, -2415}, {80, -200, -1840});
  setup.performActions(actions);
  pos = setup.pos;

  Camera camera(col);
  int setting = 3;
  camera.initParallel(pos, angle, setting);
  camera.updateNormal(pos, angle, setting);

  // Dry roll (interrupted)
  pos = move(col, pos, angle, 2.0f);
  camera.updateNormal(pos, angle, setting);
  for (int i = 0; i < 4; i++) {
    pos = move(col, pos, angle, 3.0f);
    camera.updateNormal(pos, angle, setting);
  }

  // 1st damage frame
  pos = move(col, pos, angle, 3.0f);
  u16 hessAngle = Math_Vec3f_Yaw(&pos, &bombPos);
  camera.updateNormal(pos, hessAngle, setting);
  u16 damageAngle = camera.yaw();

  if (debug) {
    printf(
        "damage frame: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) "
        "hessAngle=%04x damageAngle=%04x\n",
        pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
        floatToInt(pos.z), hessAngle, damageAngle);
  }

  if (!(0xad48 <= hessAngle && hessAngle < 0xad78)) {
    if (debug) {
      printf("hess angle out of range: %04x\n", hessAngle);
    }
    return false;
  }

  // Need camera off the wall for consistency
  // experimentally: b6a0 -> b60f, b6b0 -> b61f
  u16 targetDamageAngle = (u16)(angle & 0xFFF0) - 0xa0;
  s16 diff = damageAngle - targetDamageAngle;
  if (!(0 <= diff && diff < 0x10)) {
    if (debug) {
      printf("damage angle out of range: %04x\n", damageAngle);
    }
    return false;
  }

  pos = move(col, pos, hessAngle, 15.0f);
  pos = move(col, pos, damageAngle + essAngle, 7.0f);
  pos = move(col, pos, hessAngle + ESS, 7.0f);
  pos = move(col, pos, hessAngle + ESS, 7.0f);

  return simulateHess(col, pos, -18, hessAngle, debug);
}

bool testHbHessSetup(Collision* col, Vec3f pos, u16 angle, u16 essDir,
                     bool debug) {
  Camera camera(col);
  int setting = 3;
  camera.initParallel(pos, angle, setting);
  camera.updateNormal(pos, angle, setting);

  if (camera.wallPoly) {
    return false;
  }

  // Drop bomb (big blue)
  Vec3f bombPos = pos + rotate(Vec3f(-1.03482, 0.0f, -0.610465f), angle);

  // Dry roll (interrupted)
  f32 speed = 0;
  for (int i = 0; i < 5; i++) {
    speed += 0.35f;
    pos = move(col, pos, angle, speed);
    camera.updateNormal(pos, angle, setting);
  }

  // 1st damage frame
  speed += 0.35f;
  pos = move(col, pos, angle, speed);
  u16 hessAngle = 0x8000 + Math_Vec3f_Yaw(&pos, &bombPos);

  if (!(0xad48 <= hessAngle && hessAngle < 0xad88)) {
    return false;
  }

  camera.updateNormal(pos, hessAngle, setting);
  u16 damageAngle = camera.yaw();

  speed += 0.35f;
  pos = move(col, pos, hessAngle, speed);

  if (essDir == 0x0000) {  // ess up
    speed += 0.35f;
    pos = move(col, pos, damageAngle, speed);
    speed += 0.35f;
    pos = move(col, pos, damageAngle + 0x7E9, speed);
    speed += 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
    speed -= 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
    speed -= 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
  } else if (essDir == 0x4000) {  // ess left
    speed += 0.35f;
    pos = move(col, pos, hessAngle + 0x7E9, speed);
    speed += 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
    speed += 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
    speed -= 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
    speed -= 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
  } else if (essDir == 0x8000) {  // ess down
    speed -= 0.35f;
    pos = move(col, pos, damageAngle, speed);
    speed += 0.35f;
    pos = move(col, pos, damageAngle + 0x7E9, speed);
    speed += 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
  } else if (essDir == 0xc000) {  // ess right
    speed += 0.35f;
    pos = move(col, pos, hessAngle - 0x7E9, speed);
    speed += 0.35f;
    pos = move(col, pos, hessAngle, speed);
    speed += 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
    speed -= 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
    speed -= 0.35f;
    pos = move(col, pos, hessAngle + ESS, speed);
  } else {
    return false;
  }

  return simulateHess(col, pos, speed, hessAngle, debug);
}

void findHessSetupPositions(Collision* col) {
  f32 xmin = -100.0f;
  f32 xmax = 100.0f;
  f32 zmin = -2082;
  f32 zmax = -1980;
  f32 step = 0.5f;

  int i = 0;
  for (f32 x = xmin; x < xmax; x += step) {
    for (f32 z = zmin; z < zmax; z += step) {
      if (i % 1000 == 0) {
        fprintf(stderr, "x=%.9g\n", x);
      }
      i++;

      Vec3f pos = col->findFloor({x, -340, z});
      u16 angle = 0xb6a0;
      if (testHessSetup(col, pos, angle, {BACKFLIP, ROLL}, 0x4000, false)) {
        printf(
            "x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x z_raw=%08x "
            "angle=%04x\n",
            pos.x, pos.y, pos.z, floatToInt(pos.x), floatToInt(pos.y),
            floatToInt(pos.z), angle);
      }
    }
  }
}

void testPosAngleSetup(Collision* col, Vec3f initialPos,
                       const std::vector<Action>& actions, bool debug) {
  PosAngleSetup setup(col, initialPos, 0x8000, {-140, -340, -2415},
                      {80, -200, -1840});
  if (!setup.performActions(actions)) {
    return;
  }

  Vec3f pos = setup.pos;
  u16 angle = setup.angle;

  std::vector<std::vector<Action>> setups = {
      {BACKFLIP, ROLL},
      {ROLL, BACKFLIP},
  };

  for (int i = 0; i < 4; i++) {
    u16 essAngle = 0x4000 * i;

    for (int j = 0; j < setups.size(); j++) {
      if (testHessSetup(col, pos, angle, setups[j], essAngle, debug)) {
        printf(
            "startx=%.9g x=%.9g z=%.9g x_raw=%08x z_raw=%08x essAngle=%04x "
            "hessSetup=%d actions=",
            initialPos.x, pos.x, pos.z, floatToInt(pos.x), floatToInt(pos.z),
            essAngle, j);
        for (Action action : actions) {
          printf("%s,", actionName(action));
        }
        printf("\n");
      }
    }
  }
}

void findPosAngleSetups(Collision* col) {
  std::vector<Vec3f> initialPositions = {
      col->findFloor({62, -300, -2415}),
      col->findFloor({-62, -300, -2415}),
  };

  // Idea: try permutations of angle setup, adding angle-preserving actions
  // in between
  std::vector<Action> angleSetup = {BACKFLIP_SIDEROLL_RETARGET, TURN_7_ESS_LEFT,
                                    TURN_LEFT};

  std::vector<Action> addlActions = {
      ROLL, BACKFLIP,
      SIDEHOP_LEFT,   // SIDEHOP_LEFT_SIDEROLL,
      SIDEHOP_RIGHT,  // SIDEHOP_RIGHT_SIDEROLL,
  };

  int numSetups = 0;
  // TODO: generalize to arbitrary number of actions
  for (int i = 0; i < addlActions.size(); i++) {
    for (int j = i; j < addlActions.size(); j++) {
      for (int k = j; k < addlActions.size(); k++) {
        for (int l = k; l < addlActions.size(); l++) {
          std::vector<Action> actions = angleSetup;
          actions.push_back(addlActions[i]);
          actions.push_back(addlActions[j]);
          actions.push_back(addlActions[k]);
          actions.push_back(addlActions[l]);
          std::sort(actions.begin(), actions.end());
          do {
            if (numSetups % 1000 == 0) {
              fprintf(stderr, "numSetups=%d i=%d j=%d k=%d l=%d\r", numSetups,
                      i, j, k, l);
            }
            numSetups++;

            // Ban some first actions
            if (actions[0] == ROLL || actions[0] == TURN_ESS_LEFT ||
                actions[0] == TURN_2_ESS_LEFT ||
                actions[0] == SIDEHOP_RIGHT_SIDEROLL_RETARGET) {
              continue;
            }

            for (const Vec3f& initialPos : initialPositions) {
              testPosAngleSetup(col, initialPos, actions, false);
            }
          } while (std::next_permutation(actions.begin(), actions.end()));
        }
      }
    }
  }

  fprintf(stderr, "Searched %d setups\n", numSetups);
}

int main(int argc, char* argv[]) {
  Collision col(&bdan_sceneCollisionHeader_013074, PLAYER_AGE_ADULT,
                {-120, -360, -2440}, {620, -280, -1620});

  // printHessAngles();

  // testClip(&col, intToFloat(0x43c32c3d), intToFloat(0xc4dc9183), 0xdeb0,
  // true);

  // findClipRegion(&col, 400);
  // findClipRegion(&col, 592);

  // simulateHess(
  //     &col,
  //     {intToFloat(0x422ce0c8), intToFloat(0xc3aa0000),
  //     intToFloat(0xc50d5f36)}, -18, 0xad78, true);

  // simulateHess(&col, {40, -340, intToFloat(0xc50d4000)}, -18, 0xad48, true);

  // simulateHess(&col, {intToFloat(0xc27a1d73), -340,
  // intToFloat(0xc501d783)},
  //              intToFloat(0xb4400000), 0xad55, true);

  // u16 angle = strtol(argv[1], NULL, 16);
  // findHessPositions(&col, angle, intToFloat(0xb4400000));
  // findHessPositions(&col, angle, -18);

  // findHessSetupPositions(&col);

  // testHessSetup(&col, {intToFloat(0xc2767f80), -340,
  // intToFloat(0xc4fe93ce)},
  //               0xb6bd, {BACKFLIP, ROLL}, 0x0000, true);

  // testHessSetup(&col, {intToFloat(0xc242cea0), -340, intToFloat(0xc4fe8942)},
  //               0xb6ad, {BACKFLIP, ROLL}, 0x0000, true);

  findPosAngleSetups(&col);

  return 0;
}
