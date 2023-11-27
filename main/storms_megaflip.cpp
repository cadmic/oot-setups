#include "actor.hpp"
#include "collision_data.hpp"
#include "pos_angle_setup.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"

bool testMegaflip(Collision* corridorCol, Collision* platformCol, Vec3f pos,
                  u16 angle, bool debug) {
  // Roll
  pos = translate(pos, angle, 2.0f, 0.0f);
  for (int i = 0; i < 10; i++) {
    pos = translate(pos, angle, 3.0f, 0.0f);
    if (platformCol->runChecks(pos, pos) != pos) {
      return false;
    }
  }

  f32 ySpeed = 4.8f;
  pos = translate(pos, angle + 0x8000, 6.0f, ySpeed);

  for (int i = 0; i < 24; i++) {
    if (debug) {
      printf("i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n", i,
             pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
             floatToInt(pos.z), ySpeed);
    }

    Vec3f prevPos = pos;

    ySpeed -= 1.0f;
    pos = translate(pos, angle, -18.0f, ySpeed);

    if (pos.z > -792 && pos.z < -742 && pos.y > -260) {
      if (debug) {
        printf("hit loading plane\n");
      }
      return false;
    }

    if (pos.x >= 2842 && pos.x <= 2942) {
      if (pos.z > -792) {
        if (debug) {
          printf("in bounds before loading zone\n");
        }
        return false;
      }

      CollisionPoly* wallPoly;
      CollisionPoly* floorPoly;
      int dynaId;
      f32 floorHeight;
      pos = corridorCol->runChecks(prevPos, pos, &wallPoly, &floorPoly, &dynaId,
                                   &floorHeight);
      if (pos.y <= floorHeight) {
        return true;
      } else {
        if (debug) {
          printf("missed floor\n");
        }
        return false;
      }
    }
  }

  return false;
}

void findMegaflips(Collision* corridorCol, Collision* platformCol) {
  int tested = 0;
  int found = 0;
  for (int dir = -0x100; dir <= 0x100; dir += 0x10) {
    u16 angle = dir;
    for (f32 x = 2834; x <= 2950; x += 0.01f) {
      if (x >= 2842 && x <= 2942) {
        continue;
      }

      for (f32 z = -293; z <= -290; z += 0.01f) {
        if (tested % 10000 == 0) {
          fprintf(stderr, "tested=%d found=%d angle=%04x x=%.2f z=%.2f ...\r",
                  tested, found, angle, x, z);
        }
        tested++;

        Vec3f pos = {x, -150, z};
        if (testMegaflip(corridorCol, platformCol, pos, angle, false)) {
          found++;
          printf("angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x\n", angle, x,
                 z, floatToInt(x), floatToInt(z));
          fflush(stdout);
        }
      }
    }
  }
}

bool testLedgeClip(Collision* col, Vec3f pos, u16 angle, f32 speed,
                   u16 movementAngle, bool debug, Vec3f* outPos) {
  Vec3f prevPos = pos;

  CollisionPoly* wallPoly;
  CollisionPoly* floorPoly;
  int dynaId;
  f32 floorHeight;
  pos = col->runChecks(prevPos, translate(pos, movementAngle, speed, -5.0f),
                       &wallPoly, &floorPoly, &dynaId, &floorHeight);

  if (debug) {
    printf("prevPos: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", prevPos.x,
           floatToInt(prevPos.x), prevPos.y, floatToInt(prevPos.y), prevPos.z,
           floatToInt(prevPos.z));
    printf("pos: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  if (pos.y <= floorHeight) {
    if (debug) {
      printf("on ground\n");
    }
    return false;
  }

  if (!wallPoly) {
    if (debug) {
      printf("not touching wall\n");
    }
    return false;
  }

  Vec3f posA = pos + Vec3f(0.0f, 18.0f, 0.0f);
  Vec3f posB = pos + rotate({0.0f, 18.0f, 28.0f}, angle);
  col->entityLineTest(posA, posB, true, false, false, &wallPoly);
  if (!wallPoly) {
    if (debug) {
      printf("not facing wall\n");
    }
    return false;
  }

  posA = pos;
  posB = prevPos + (prevPos - pos) * (5.0f / Math_Vec3f_DistXZ(&prevPos, &pos));
  col->entityLineTest(posA, posB, true, false, false, &wallPoly);
  if (!wallPoly) {
    if (debug) {
      printf("no ledge\n");
    }
    return false;
  }

  Vec3f normal = CollisionPoly_GetNormalF(wallPoly);
  f32 dist = Math3D_UDistPlaneToPos(normal.x, normal.y, normal.z,
                                    (s16)wallPoly->dist, &pos);

  outPos->x = pos.x - (dist + 1.0f) * normal.x;
  outPos->y = prevPos.y;
  outPos->z = pos.z - (dist + 1.0f) * normal.z;

  if (debug) {
    printf("ledge clip pos: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n",
           outPos->x, floatToInt(outPos->x), outPos->y, floatToInt(outPos->y),
           outPos->z, floatToInt(outPos->z));
  }

  return true;
}

void findSidehopLedgeClips(Collision* col) {
  int tested = 0;
  int found = 0;
  for (u16 angle = 0x5000; angle <= 0x8000; angle += 0x10) {
    for (f32 x = 2850; x <= 2920; x += 1.0f) {
      for (f32 z = -290; z <= -200; z += 1.0f) {
        if (tested % 100000 == 0) {
          fprintf(stderr, "tested=%d found=%d angle=%04x x=%.2f z=%.2f ...\r",
                  tested, found, angle, x, z);
        }
        tested++;

        Vec3f pos = {x, -10, z};
        for (int i = 0; i < 6; i++) {
          pos = col->runChecks(pos, translate(pos, angle + 0x4000, 8.5f, 0.0f));
        }

        Vec3f outPos;
        if (testLedgeClip(col, pos, angle, 7.5f, angle + 0x4000, false,
                          &outPos)) {
          found++;
          printf(
              "angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x outx=%.9g "
              "outx_raw=%08x\n",
              angle, x, z, floatToInt(x), floatToInt(z), outPos.x,
              floatToInt(outPos.x));
        }
      }
    }
  }
}

std::vector<Action> addlActions = {
    // HORIZONTAL_SLASH,
    HORIZONTAL_SLASH_SHIELD,
    // DIAGONAL_SLASH,
    DIAGONAL_SLASH_SHIELD,
    // VERTICAL_SLASH,
    VERTICAL_SLASH_SHIELD,
    // FORWARD_STAB,
    FORWARD_STAB_SHIELD,
    JUMPSLASH_SHIELD,
    LONG_JUMPSLASH_SHIELD,
    CROUCH_STAB,
    SIDEHOP_LEFT,
    SIDEHOP_RIGHT,
    BACKFLIP,
    ESS_TURN_UP,
    ROTATE_ESS_LEFT,
    ROTATE_ESS_RIGHT,
    SHIELD_TURN_LEFT,
    SHIELD_TURN_DOWN,
    SHIELD_TURN_RIGHT,
};

unsigned long long tested = 0;
int close = 0;
int found = 0;
int maxCost = 66;

bool inRange(Vec3f pos, u16 angle) {
  if (pos.z > -291) {
    return false;
  }

  switch (angle & 0xfff0) {
    case 0x0010:
      return pos.x >= 2942.71f && pos.x <= 2942.77f;
    case 0x0020:
      return pos.x >= 2943.44f && pos.x <= 2943.54f;
    case 0x0030:
      return pos.x >= 2944.16f && pos.x <= 2944.30f;
    case 0x0040:
      return pos.x >= 2944.89f && pos.x <= 2945.08f;
    case 0x0050:
      return pos.x >= 2945.62f && pos.x <= 2945.84f;
    case 0x0060:
      return pos.x >= 2946.34f && pos.x <= 2946.61f;
    case 0x0070:
      return pos.x >= 2947.09f && pos.x <= 2947.40f;
    case 0x0080:
      return pos.x >= 2947.81f && pos.x <= 2948.16f;
    case 0x0090:
      return pos.x >= 2948.53f && pos.x <= 2948.78f;
    case 0xffe0:
      return pos.x >= 2841.22f && pos.x <= 2841.28f;
    case 0xffd0:
      return pos.x >= 2840.46f && pos.x <= 2840.56f;
    case 0xffc0:
      return pos.x >= 2839.70f && pos.x <= 2839.84f;
    case 0xffb0:
      return pos.x >= 2838.92f && pos.x <= 2839.10f;
    case 0xffa0:
      return pos.x >= 2838.16f && pos.x <= 2838.38f;
    case 0xff90:
      return pos.x >= 2837.39f && pos.x <= 2837.66f;
    case 0xff80:
      return pos.x >= 2836.61f && pos.x <= 2836.92f;
    case 0xff70:
      return pos.x >= 2835.89f && pos.x <= 2836.20f;
    default:
      return false;
  }
}

// Take into account we can do ESS turns while waiting for the chu timer
int actualCost(int prefixCost, int suffixCost) {
  return prefixCost + std::max(suffixCost - 24, 0);
}

bool isEssAction(Action action) {
  switch (action) {
    case ESS_TURN_UP:
    case ROTATE_ESS_LEFT:
    case ROTATE_ESS_RIGHT:
    case SHIELD_TURN_LEFT:
    case SHIELD_TURN_DOWN:
    case SHIELD_TURN_RIGHT:
      return true;
    default:
      return false;
  }
}

void search(Collision* corridorCol, const PosAngleSetup& setup,
            std::vector<Action>* actions, int prefixCost, int suffixCost) {
  int k = actions->size();
  if (k == 5) {
    fprintf(stderr, "tested=%llu found=%d", tested, found);
    for (int i = 0; i < 5; i++) {
      fprintf(stderr, " %s", actionName((*actions)[i]));
    }
    fprintf(stderr, " ... \r");
  }

  tested++;
  if (inRange(setup.pos, setup.angle)) {
    close++;
    if (testMegaflip(corridorCol, setup.col, setup.pos, setup.angle, false)) {
      found++;

      printf("cost=%d angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x actions=",
             actualCost(prefixCost, suffixCost), setup.angle, setup.pos.x,
             setup.pos.z, floatToInt(setup.pos.x), floatToInt(setup.pos.z));
      for (int i = 0; i < k; i++) {
        printf("%s,", actionName((*actions)[i]));
      }
      printf("\n");
      fflush(stdout);
      return;
    }
  }

  for (const Action action : addlActions) {
    if (k > 0 &&
        ((action == ROTATE_ESS_LEFT && actions->back() == ROTATE_ESS_RIGHT) ||
         (action == ROTATE_ESS_RIGHT && actions->back() == ROTATE_ESS_LEFT))) {
      continue;
    }

    int newPrefixCost = prefixCost;
    int newSuffixCost = suffixCost;
    if (k > 0 &&
        ((action == ROTATE_ESS_RIGHT && actions->back() == ROTATE_ESS_RIGHT) ||
         (action == ROTATE_ESS_LEFT && actions->back() == ROTATE_ESS_LEFT))) {
      newSuffixCost += 3;
    } else if (isEssAction(action)) {
      newSuffixCost += actionCost(action);
    } else {
      newPrefixCost += suffixCost + actionCost(action);
      newSuffixCost = 0;
    }

    int newCost = actualCost(newPrefixCost, newSuffixCost);
    if (newCost > maxCost) {
      continue;
    }

    PosAngleSetup newSetup(setup);
    if (!newSetup.performAction(action)) {
      continue;
    }
    if (newSetup.pos == setup.pos && newSetup.angle == setup.angle) {
      continue;
    }

    // Check if there's no hope of reaching required angle with ESS turns and
    // cardinal turns (with leeway for angle range and ess up shenanigans)
    int angleDiff = newSetup.angle % 0x4000;
    if (angleDiff < 0) {
      angleDiff += 0x4000;
    }
    int maxTurn = ((maxCost - newCost + 6) / 7) * ESS + 0x200;
    if (angleDiff > maxTurn && angleDiff < 0x4000 - maxTurn) {
      continue;
    }

    actions->push_back(action);
    search(corridorCol, newSetup, actions, newPrefixCost, newSuffixCost);
    actions->pop_back();
  }
}

void findSetups(Collision* corridorCol, Collision* platformCol) {
  std::vector<Action> actions;
  actions.reserve(20);

  // 2 vertical slash, hold target
  // Vec3f initialPos = {intToFloat(0x45312caa), -150, intToFloat(0xc3899e6c)};
  // u16 initialAngle = 0x8000;

  //  2 vertical slash, don't hold target
  // Vec3f initialPos = {intToFloat(0x453126ed), -150, intToFloat(0xc3899bc5)};
  // u16 initialAngle = 0xa027;

  // 2 ess right, hold target
  Vec3f initialPos = {intToFloat(0x45312d73), -150, intToFloat(0xc3899e6c)};
  u16 initialAngle = 0x8000;

  PosAngleSetup setup(platformCol, initialPos, initialAngle, {2827, -150, -300},
                      {2957, 0, -182});
  search(corridorCol, setup, &actions, 0, 0);
}

int main(int argc, char* argv[]) {
  Collision corridorCol(&hakasitarelay_sceneCollisionHeader_00C04C,
                        PLAYER_AGE_ADULT, {2846, -210, -692},
                        {2938, -210, -692});
  // corridorCol.printPolys();

  Collision platformCol(&hakasitarelay_sceneCollisionHeader_00C04C,
                        PLAYER_AGE_ADULT, {2754, -150, -292},
                        {3030, -150, -178});
  // platformCol.printPolys();

  Collision ledgeCol(&hakasitarelay_sceneCollisionHeader_00C04C,
                     PLAYER_AGE_ADULT, {2823, -10, -292}, {2846, -10, -280});
  // ledgeCol.printPolys();

  // testMegaflip(&corridorCol,
  //              {intToFloat(0x45316b86), -150, intToFloat(0xc3744354)},
  //              0xffb0, true);

  // findMegaflips(&corridorCol, &platformCol);

  // Vec3f outPos;
  // testLedgeClip(&ledgeCol, {2833.5f, -10, -262}, 0x8000, 7.5f, 0xc000, true,
  //               &outPos);

  // findSidehopLedgeClips(&ledgeCol);
  findSetups(&corridorCol, &platformCol);

  return 0;
}
