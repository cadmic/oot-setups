#include "actor.hpp"
#include "collision_data.hpp"
#include "search.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"

void printSeamHeights(Collision* col) {
  f32 cx = -1822;
  f32 cz = 1923;
  for (f32 x = cx - 1.0f; x <= cx + 1.0f; x += 0.01f) {
    for (f32 z = cz - 1.0f; z <= cz + 1.0f; z += 0.01f) {
      Vec3f pos = col->findFloor({x, 1000, z});
      if (pos.y >= 95.3f) {
        printf("x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x z_raw=%08x\n", pos.x,
               pos.y, pos.z, floatToInt(pos.x), floatToInt(pos.y),
               floatToInt(pos.z));
      }
    }
  }
}

bool testJumpToSeam(Collision* col, Vec3f pos, u16 angle, f32 xzSpeed,
                    f32 ySpeed, f32* finalHeight) {
  for (int i = 0; i < 20; i++) {
    ySpeed -= 1.0f;

    CollisionPoly* wallPoly;
    CollisionPoly* floorPoly;
    int dynaId;
    f32 floorHeight;
    pos = col->runChecks(pos, translate(pos, angle, xzSpeed, ySpeed), &wallPoly,
                         &floorPoly, &dynaId, &floorHeight);

    if (ySpeed <= 0.0f && pos.y <= floorHeight) {
      break;
    }

    if (pos.y < 45.0f) {
      return false;
    }
  }

  if (pos.y < 95.3f) {
    return false;
  }

  *finalHeight = pos.y;
  return true;
}

void printSeamJumps(Collision* col, f32 xzSpeed, f32 ySpeed) {
  int tested = 0;
  int found = 0;
  for (u16 angle = 0xcc00; angle <= 0xea00; angle += 0x10) {
    for (f32 x = -1790; x < -1710; x += 0.1f) {
      for (f32 z = 1800; z < 1910; z += 0.1f) {
        if (tested % 100000 == 0) {
          fprintf(stderr, "tested=%d found=%d angle=%04x x=%.2f z=%.2f ...\r",
                  tested, found, angle, x, z);
        }
        tested++;

        Vec3f pos = col->findFloor({x, 1000, z});
        if (pos.y == BGCHECK_Y_MIN) {
          continue;
        }

        f32 height;
        if (testJumpToSeam(col, pos, angle, xzSpeed, ySpeed, &height)) {
          found++;
          printf(
              "angle=%04x x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x "
              "z_raw=%08x height=%.9g xzSpeed=%.1f ySpeed=%.1f\n",
              angle, pos.x, pos.y, pos.z, floatToInt(pos.x), floatToInt(pos.y),
              floatToInt(pos.z), height, xzSpeed, ySpeed);
        }
      }
    }
  }
}

void findSetups(Collision* col, int argc, char* argv[]) {
  SearchParams params = {
      .col = col,
      .minBounds = {-10000, -12, -10000},
      .maxBounds = {10000, 10000, 10000},
      .starts =
          {
              // sidehop sideroll left into corner
              {{intToFloat(0xc523e646), intToFloat(0x419de8fe),
                intToFloat(0x440a21b3)},
               0xc05c},
              {{intToFloat(0xc523e646), intToFloat(0x419de8fe),
                intToFloat(0x440a21b3)},
               0xdd34},
              // sidehop sideroll right into corner
              {{intToFloat(0xc523e646), intToFloat(0x419dea62),
                intToFloat(0x440a22a0)},
               0xfb76},
              {{intToFloat(0xc523e646), intToFloat(0x419dea62),
                intToFloat(0x440a22a0)},
               0xdea4},
          },
      .maxCost = 240,
      .angleMin = 0x0000,
      .angleMax = 0xffff,
      .xMin = -1787,
      .xMax = -1714,
      .zMin = 1811,
      .zMax = 1896,
      .actions =
          {
              TARGET_WALL,
              ROLL,
              BACKFLIP,
              BACKFLIP_SIDEROLL,
              BACKFLIP_SIDEROLL_UNTARGET,
              SIDEHOP_LEFT,
              SIDEHOP_LEFT_SIDEROLL,
              SIDEHOP_LEFT_SIDEROLL_UNTARGET,
              SIDEHOP_RIGHT,
              SIDEHOP_RIGHT_SIDEROLL,
              SIDEHOP_RIGHT_SIDEROLL_UNTARGET,
              ROTATE_ESS_LEFT,
              ROTATE_ESS_RIGHT,
              ESS_TURN_UP,
              ESS_TURN_LEFT,
              ESS_TURN_RIGHT,
              ESS_TURN_DOWN,
          },
  };

  auto filter = [&](const PosAngleSetup& setup) {
    Vec3f signPos = {-2557, 3, 486};
    Vec3f linkPos = setup.pos;
    f32 distToSign = Math_Vec3f_DistXZ(&signPos, &linkPos);
    u16 yaw = Math_Vec3f_Yaw(&signPos, &linkPos);

    // Don't read sign
    s16 relYawTowardsPlayer = yaw - 0xcaab;
    if (distToSign <= 50.0f && (std::abs(relYawTowardsPlayer) < 0x2800 ||
                                std::abs(relYawTowardsPlayer) > 0x5800)) {
      return false;
    }

    // Don't target sign
    s16 yawFromLink = yaw - 0x8000 - setup.angle;
    if (distToSign <= 70.0f && std::abs(yawFromLink) <= 0x2aaa) {
      return false;
    }

    return true;
  };

  auto output = [&](Vec3f initialPos, u16 initialAngle, Vec3f finalPos,
                    u16 finalAngle, const std::vector<Action>& path, int cost) {
    // On top of log toward seam is in the 0xc000-0xffff range
    u16 diff = finalAngle - 0xc000;
    int dir = (diff & 0xc000) >> 14;

    u16 angle;
    f32 xzSpeed;
    f32 ySpeed;
    if (dir == 1) {
      angle = finalAngle - 0x4000;
      xzSpeed = 8.5f;
      ySpeed = 3.5f;
    } else if (dir == 2) {
      angle = finalAngle + 0x8000;
      xzSpeed = 6.0f;
      ySpeed = 5.8f;
    } else if (dir == 3) {
      angle = finalAngle + 0x4000;
      xzSpeed = 8.5f;
      ySpeed = 3.5f;
    } else {
      return false;
    }

    f32 finalHeight;
    if (testJumpToSeam(col, finalPos, angle, xzSpeed, ySpeed, &finalHeight)) {
      printf(
          "cost=%d startAngle=%04x startx=%.9g startz=%.9g angle=%04x x=%.9g "
          "(%08x) z=%.9g (%08x) finalHeight=%.9g actions=%s\n",
          cost, initialAngle, initialPos.x, initialPos.z, finalAngle,
          finalPos.x, floatToInt(finalPos.x), finalPos.z,
          floatToInt(finalPos.z), finalHeight, actionNames(path).c_str());
      fflush(stdout);
      return true;
    }
    return false;
  };

  if (argc > 1) {
    int shard = atoi(argv[1]);
    searchSetupsShard(params, 2, shard, filter, output);
  } else {
    searchSetups(params, filter, output);
  }
}

int main(int argc, char* argv[]) {
  Collision col(&spot08_sceneCollisionHeader_002CE0, PLAYER_AGE_CHILD);
  std::vector<int> polys = {
      // log front walls
      489, 530,
      // log lower part floors
      519, 523, 524,
      // log lower part walls
      520, 528, 529,
      // log upper part floors
      521, 522,
      // log upper part walls
      518, 526, 527,
      // walkway floors
      289, 290, 300, 301, 302, 303, 304,
      // walkway walls
      30, 31, 32,
      // entrance floors
      305, 306, 307, 308, 309, 317, 318, 320, 321,
      // entrance walls (near walkway)
      10, 11, 97,
      // entrance walls (near fence)
      12, 13, 94,
      // fence
      431,
      //
  };
  for (int poly : polys) {
    col.addPoly(poly);
  }
  // col.printPolys();

  // printSeamHeights(&col);

  // printSeamJumps(&col, 8.5f, 3.5f);  // sidehop
  // printSeamJumps(&col, 6.0f, 5.8f);  // backflip

  findSetups(&col, argc, argv);

  return 0;
}
