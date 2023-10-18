#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "camera.hpp"
#include "collision.hpp"
#include "collision_data.hpp"
#include "global.hpp"
#include "pos_angle_setup.hpp"
#include "sys_math.hpp"

Vec3f throwPosition(Vec3f pos, u16 angle) {
  return getHeldActorPosition(gPlayerAnim_link_normal_throw_free_Data, 1,
                              PLAYER_AGE_CHILD, pos, angle);
}

Vec3s guardPosition(Vec3f pos) {
  Vec3f home = {-1472, -80, -294};
  u16 yaw = Math_Vec3f_Yaw(&home, &pos);
  return (home + Vec3f(Math_SinS(yaw), 0, Math_CosS(yaw)) * 80.0f).toVec3s();
}

bool guardPush(Vec3f pos, Vec3f guardPos, int combinedRadius,
               Vec3f* displacement) {
  Vec3s posTrunc = pos.toVec3s();
  Vec3f guardPosTrunc = guardPos.toVec3s();

  int diff = posTrunc.x - guardPosTrunc.x;
  if (diff >= combinedRadius) {
    return true;
  }

  if (posTrunc.z != guardPosTrunc.z) {
    return false;
  }

  f32 push = (f32)diff * ((f32)(combinedRadius - diff) / (f32)diff);
  *displacement = {push, 0.0f, 0.0f};
  return true;
}

bool testThrowPosition(f32 x, f32 z, u16 throwAngle, u16 cameraAngle,
                       bool debug) {
  f32 nx = (s16)0xf237 * SHT_MINV;
  f32 ny = (s16)0x7f40 * SHT_MINV;
  f32 nz = (s16)0x000a * SHT_MINV;
  f32 originDist = (s16)0xffb1;
  f32 y = (f32)((((-nx * x) - (nz * z)) - originDist) / ny);
  Vec3f linkPos = {x, y, z};

  Vec3f rockPos = throwPosition(linkPos, throwAngle);
  Vec3s guardPos = guardPosition(linkPos);
  f32 backwalkSpeed = 0.0f;
  for (int f = 2;; f++) {
    if (debug) {
      printf(
          "f=%d linkx=%.9g (%08x) linkz=%.9g (%08x) rockx=%.9g (%08x) "
          "rockz=%.9g (%08x)\n",
          f, linkPos.x, floatToInt(linkPos.x), linkPos.z, floatToInt(linkPos.z),
          rockPos.x, floatToInt(rockPos.x), rockPos.z, floatToInt(rockPos.z));
    }

    if (f == 21) {
      break;
    }

    if (f >= 9 && f < 12) {
      linkPos = translate(linkPos, cameraAngle + 0x8000, 2.0f, 0.0f, Vec3f());

      if (guardPos != guardPosition(linkPos)) {
        if (debug) {
          printf("guard position changed\n");
        }
        return false;
      }
    } else if (f >= 15) {
      Vec3f linkDisplacement;
      if (!guardPush(linkPos.toVec3s(), guardPos, 32, &linkDisplacement)) {
        if (debug) {
          printf("link push failed\n");
        }
        return false;
      }

      backwalkSpeed = std::min(backwalkSpeed + 1.5f, 8.25f);
      linkPos = translate(linkPos, cameraAngle, backwalkSpeed, 0.0f,
                          linkDisplacement);

      if (guardPos != guardPosition(linkPos)) {
        if (debug) {
          printf("guard position changed\n");
        }
        return false;
      }
    }

    Vec3f rockDisplacement;
    if (!guardPush(rockPos.toVec3s(), guardPos, 30, &rockDisplacement)) {
      if (debug) {
        printf("rock push failed\n");
      }
      return false;
    }

    rockPos = translate(rockPos, throwAngle, 8.0f, 0.0f, rockDisplacement);
  }

  Vec3s linkPosTrunc = linkPos.toVec3s();
  Vec3s rockPosTrunc = rockPos.toVec3s();

  if (linkPosTrunc.x != rockPosTrunc.x || linkPosTrunc.z != rockPosTrunc.z) {
    if (debug) {
      printf("link and rock not aligned\n");
    }
    return false;
  }

  return true;
}

void findThrowPositions(u16 throwAngle, u16 cameraAngle) {
  int tested = 0;
  int found = 0;

  for (u32 zi = floatToInt(-292.0f); zi <= floatToInt(-294.4f); zi++) {
    f32 z = intToFloat(zi);
    for (int xi = -1359; xi < -1335; xi++) {
      for (int xf = 0x2e7; xf <= 0x2fd; xf++) {
        f32 x = intToFloat(floatToInt((f32)xi) | xf);
        if (tested % 100000 == 0) {
          fprintf(stderr, "tested=%d found=%d z=%.9g x=%.9g ...\r", tested,
                  found, z, x);
        }
        tested++;

        if (testThrowPosition(x, z, throwAngle, cameraAngle, false)) {
          found++;
          printf(
              "throwAngle=%04x cameraAngle=%04x x=%.9g z=%.9g x_raw=%08x "
              "z_raw=%08x\n",
              throwAngle, cameraAngle, x, z, floatToInt(x), floatToInt(z));
        }
      }
    }
  }
}

bool simulateWalk(f32 x, f32 z, u16 walkAngle, u16 throwAngle, u16 cameraAngle,
                  bool debug) {
  Vec3f walkDir = {Math_SinS(walkAngle), 0, Math_CosS(walkAngle)};

  Vec3f pos = {x, 0, z};
  pos = pos + walkDir * 2.0f * 1.5f;
  pos = pos + walkDir * 4.0f * 1.5f;

  Vec3f displacement = walkDir * 5.5f * 1.5f;
  for (int i = 0;; i++) {
    if (debug) {
      printf("i=%d x=%.9g (%08x) z=%.9g (%08x)\n", i, x, floatToInt(x), z,
             floatToInt(z));
    }

    if (pos.x <= -1360.0f) {
      break;
    }

    Vec3f throwPos = pos;
    throwPos = throwPos + walkDir * 4.0f * 1.5f;
    throwPos = throwPos + walkDir * 2.5f * 1.5f;
    throwPos = throwPos + walkDir * 1.0f * 1.5f;
    if (throwPos.x > -1360.0f) {
      // quickly filter by x coordinate
      u32 xi = floatToInt(throwPos.x);
      u32 xf = xi & 0x1fff;
      if (xf >= 0x2f1 && xf <= 0x2fd &&
          testThrowPosition(throwPos.x, throwPos.z, throwAngle, cameraAngle,
                            false)) {
        return true;
      }
    }

    pos = pos + displacement;
  }

  return false;
}

void findZRange(f32 x, u16 angle) {
  bool found = false;
  printf("angle=%04x x=%.9g (%08x)\n", angle, x, floatToInt(x));
  for (int zi = floatToInt(-290.0f); zi <= floatToInt(-300.0f); zi++) {
    f32 z = intToFloat(zi);
    if (simulateWalk(x, z, angle, angle, angle, false)) {
      if (!found) {
        printf("  z=%.9g (%08x) to ", z, floatToInt(z));
        found = true;
      }
    } else {
      if (found) {
        printf("z=%.9g (%08x)\n", z, floatToInt(z));
        found = false;
      }
    }
  }
}

const int ACTION_JUMP_SIZE = 12;
const int ACTION_ROLL_SIZE = 12;
const int ACTION_SIZE = ACTION_JUMP_SIZE + ACTION_ROLL_SIZE;

struct SetupAction {
  Action action;
  u16 angle;
  f32 x[ACTION_SIZE];
  f32 z[ACTION_SIZE];
};

struct Crumb {
  Action action;
  u16 angle;
};

Action possibleActions[] = {
    ROLL,
    SIDEHOP_LEFT,
    SIDEHOP_LEFT_SIDEROLL,
    SIDEHOP_RIGHT,
    SIDEHOP_RIGHT_SIDEROLL,
    BACKFLIP,
    BACKFLIP_SIDEROLL,
};

void computeJump(SetupAction* action, u16 dir, f32 speed, int frames) {
  u16 movementAngle = action->angle + dir;
  for (int i = 0; i < frames; i++) {
    action->x[i] = (Math_SinS(movementAngle) * speed) * 1.5f;
    action->z[i] = (Math_CosS(movementAngle) * speed) * 1.5f;
  }

  action->x[frames] = (Math_SinS(movementAngle) * (speed - 1.0f)) * 1.5f;
  action->z[frames] = (Math_CosS(movementAngle) * (speed - 1.0f)) * 1.5f;
}

void computeRoll(SetupAction* action, u16 dir) {
  u16 facingAngle = action->angle;
  u16 movementAngle = action->angle + dir;

  for (int i = 0; i < 11; i++) {
    Math_ScaledStepToS(&movementAngle, facingAngle, 2000);
    Math_ScaledStepToS(&facingAngle, movementAngle, 2000);

    // 1 frame of 2 speed, 10 frames of 3 speed
    f32 speed;
    if (i < 1) {
      speed = 2.0f;
    } else {
      speed = 3.0f;
    }

    action->x[ACTION_JUMP_SIZE + i] = (Math_SinS(movementAngle) * speed) * 1.5f;
    action->z[ACTION_JUMP_SIZE + i] = (Math_CosS(movementAngle) * speed) * 1.5f;
  }
}

void computeSetupAction(SetupAction* action) {
  switch (action->action) {
    case ROLL:
      computeRoll(action, 0x0000);
      break;
    case SIDEHOP_LEFT:
      computeJump(action, 0x4000, 8.5f, 6);
      break;
    case SIDEHOP_LEFT_SIDEROLL:
      computeJump(action, 0x4000, 8.5f, 6);
      computeRoll(action, 0x4000);
      break;
    case SIDEHOP_RIGHT:
      computeJump(action, 0xc000, 8.5f, 6);
      break;
    case SIDEHOP_RIGHT_SIDEROLL:
      computeJump(action, 0xc000, 8.5f, 6);
      computeRoll(action, 0xc000);
      break;
    case BACKFLIP:
      computeJump(action, 0x8000, 6.0f, 11);
      break;
    case BACKFLIP_SIDEROLL:
      computeJump(action, 0x8000, 6.0f, 11);
      computeRoll(action, 0x8000);
      break;
    default:
      assert(false);
  }
}

std::vector<SetupAction> computeSetupActions() {
  std::vector<SetupAction> setupActions;

  for (int i = 0; i < ARRAY_SIZE(possibleActions); i++) {
    for (int angle = 0; angle < 0x10000; angle += 0x4000) {
      SetupAction action;
      action.action = possibleActions[i];
      action.angle = angle;
      memset(action.x, 0, sizeof(action.x));
      memset(action.z, 0, sizeof(action.z));
      computeSetupAction(&action);

      setupActions.push_back(action);
    }
  }

  return setupActions;
}

bool inBounds(Vec3f pos) {
  return pos.x >= -640.0f && pos.x <= -55.0f && pos.z >= -328.0f &&
         pos.z <= 350.0f;
}

bool inOpenArea(Vec3f pos) {
  return !(pos.x <= -540.0f && pos.z >= -117.0f) &&  // slope
         !(pos.x >= -385.0f && pos.z >= 80.0f);      // grass patch
}

Vec3f rockPos = {-292, 0, -350};

u64 tested = 0;
u64 found = 0;

void testSetup(std::vector<Crumb>* path, int runningCost, Vec3f pos) {
  if (tested % 10000 == 0) {
    fprintf(stderr, "tested=%llu found=%llu depth=%lu ", tested, found,
            path->size());
    for (int i = 0; i < std::min((int)path->size(), 5); i++) {
      fprintf(stderr, "%s %04x, ", actionName((*path)[i].action),
              (*path)[i].angle);
    }
    fprintf(stderr, "...\r");
  }
  tested++;

  for (u16 walkAngle = 0xc010; walkAngle <= 0xc020; walkAngle += 0x10) {
    for (int taps = 1; taps <= 2; taps++) {
      if (simulateWalk(pos.x, pos.z + 4.5f * taps, walkAngle, walkAngle,
                       walkAngle, false)) {
        found++;

        int cost = runningCost;

        u16 prevAngle = path->back().angle;
        if (prevAngle != 0x8000) {
          cost += 20;
        }

        cost += taps * 20;
        if (walkAngle == 0xc020) {
          cost += 12;
        }

        printf("cost=%d x=%.9g (%08x) z=%.9g (%08x) taps=%d angle=%04x ", cost,
               pos.x, floatToInt(pos.x), pos.z, floatToInt(pos.z), taps,
               walkAngle);
        for (const auto& crumb : *path) {
          printf("%s %04x, ", actionName(crumb.action), crumb.angle);
        }
        printf("\n");
        fflush(stdout);
      }
    }
  }
}

void doSearch(Collision* col, const std::vector<SetupAction>& actions,
              int maxCost, std::vector<Crumb>* path, int runningCost, Vec3f pos,
              int depth) {
  if (SQ(pos.x - rockPos.x) + SQ(pos.z - rockPos.z) >
      SQ(50.0f + depth * 130.0f)) {
    return;  // too far
  }

  if (runningCost + 8 * depth > maxCost) {
    return;
  }

  if (depth == 0) {
    testSetup(path, runningCost, pos);
    return;
  }

  for (const auto& action : actions) {
    Vec3f newPos = pos;
    for (int i = 0; i < ACTION_SIZE; i++) {
      newPos.x += action.x[i];
      newPos.z += action.z[i];
    }

    if (!inBounds(newPos)) {
      continue;
    }

    u16 prevAngle = path->empty() ? 0xc000 : path->back().angle;

    if (!(inOpenArea(pos) && inOpenArea(newPos))) {
      if (action.angle != prevAngle) {
        // check that camera turn works
        int setting = 1;
        Camera camera(col);
        camera.initParallel(pos, prevAngle, setting);
        camera.updateNormal(pos, prevAngle, setting);
        if ((camera.yaw() & 0xfff0) != prevAngle) {
          continue;
        }
      }

      // slow collision check
      PosAngleSetup setup(col, pos, action.angle, {-10000, 0, -10000},
                          {10000, 10000, 10000});
      if (!setup.performAction(action.action)) {
        continue;
      }
      newPos = setup.pos;
    }

    int cost = runningCost;
    cost += actionCost(action.action);
    if (action.angle != prevAngle) {
      if (action.angle - prevAngle == 0x8000) {
        cost += 35;
      } else {
        cost += 20;
      }
    }

    // printf(
    //     "depth=%d action=%s angle=%04x x=%.9g (%08x) z=%.9g (%08x) x=%.9g "
    //     "(%08x) z=%.9g (%08x)\n",
    //     depth, actionName(action.action), action.angle, pos.x,
    //     floatToInt(pos.x), pos.z, floatToInt(pos.z), newPos.x,
    //     floatToInt(newPos.x), newPos.z, floatToInt(newPos.z));

    path->push_back({action.action, action.angle});
    doSearch(col, actions, maxCost, path, cost, newPos, depth - 1);
    path->pop_back();
  }
}

void searchSetups(Collision* col) {
  std::vector<SetupAction> actions = computeSetupActions();
  std::vector<Crumb> path;

  for (int k = 8; k <= 10; k++) {
    path.reserve(k);
    doSearch(col, actions, 200, &path, 0,
             {intToFloat(0xc4084d4a), 1, intToFloat(0x433f0847)}, k);
  }
}

int main(int argc, char* argv[]) {
  Collision col(&spot04_sceneCollisionHeader_008918, PLAYER_AGE_CHILD);
  col.addPoly(0x140);
  col.addPoly(0x141);
  col.addPoly(0x1f9);
  col.addPoly(0x1fa);
  col.addPoly(0x2bc);
  col.addPoly(0x2bd);

  // testThrowPosition(intToFloat(0xc4a8c2fd), intToFloat(0xc3931ae1),
  //                   0xc010, 0xc010, true);

  // findThrowPositions(0xc010, 0xc010);
  // findThrowPositions(0xc010, 0xc040);

  // findZRange(intToFloat(0xc3908fd5), 0xc020);
  // findZRange(intToFloat(0xc3964fc5), 0xc020);
  // findZRange(intToFloat(0xc396efb9), 0xc020);
  // findZRange(intToFloat(0xc38e4fe7), 0xc010);

  searchSetups(&col);

  return 0;
}
