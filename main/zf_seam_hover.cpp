#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "collider.hpp"
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
      .maxCost = 239,
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

  auto filter = [&](Vec3f initialPos, u16 initialAngle,
                    const PosAngleSetup& setup, const std::vector<Action>& path,
                    int cost) {
    Vec3f pos = setup.pos;
    if (pos.x < -2040.0f || pos.z < 620.0f) {
      // near entrance
      Vec3f corner = {-2040, 0, 620};
      f32 distToCorner = Math_Vec3f_DistXZ(&corner, &pos);
      int minCostToCorner = ceilf(distToCorner / 12.75f);
      if (cost + minCostToCorner > params.maxCost - 140) {
        return false;
      }

      Vec3f signPos = {-2557, 3, 486};
      f32 distToSign = Math_Vec3f_DistXZ(&signPos, &pos);
      u16 yaw = Math_Vec3f_Yaw(&signPos, &pos);

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
    } else if (pos.z < 1800.0f) {
      // not yet near foot of log
      f32 distToLog = 1800.0f - pos.z;
      int minCostToLog = ceilf(distToLog / 12.75f);
      if (cost + minCostToLog > params.maxCost - 30) {
        return false;
      }
    }

    return true;
  };

  auto output = [&](Vec3f initialPos, u16 initialAngle,
                    const PosAngleSetup& setup, const std::vector<Action>& path,
                    int cost) {
    // On top of log toward seam is in the 0xc000-0xffff range
    u16 diff = setup.angle - 0xc000;
    int dir = (diff & 0xc000) >> 14;

    u16 angle;
    f32 xzSpeed;
    f32 ySpeed;
    if (dir == 1) {
      angle = setup.angle - 0x4000;
      xzSpeed = 8.5f;
      ySpeed = 3.5f;
    } else if (dir == 2) {
      angle = setup.angle + 0x8000;
      xzSpeed = 6.0f;
      ySpeed = 5.8f;
    } else if (dir == 3) {
      angle = setup.angle + 0x4000;
      xzSpeed = 8.5f;
      ySpeed = 3.5f;
    } else {
      return false;
    }

    f32 finalHeight;
    if (testJumpToSeam(col, setup.pos, angle, xzSpeed, ySpeed, &finalHeight)) {
      printf(
          "cost=%d startAngle=%04x startx=%.9g startz=%.9g angle=%04x x=%.9g "
          "(%08x) z=%.9g (%08x) finalHeight=%.9g actions=%s\n",
          cost, initialAngle, initialPos.x, initialPos.z, setup.angle,
          setup.pos.x, floatToInt(setup.pos.x), setup.pos.z,
          floatToInt(setup.pos.z), finalHeight, actionNames(path).c_str());
      fflush(stdout);
      return true;
    }
    return false;
  };

  if (argc > 1) {
    int shard = atoi(argv[1]);
    searchSetupsShard(params, 1, shard, filter, output);
  } else {
    searchSetups(params, filter, output);
  }
}

// Experimentally-determined skulltula hitbox positions
std::vector<Vec3s> skullPositions = {
    {-1886, 200, 1909}, {-1886, 199, 1909}, {-1886, 199, 1910},
    {-1886, 199, 1911}, {-1886, 200, 1911}, {-1887, 200, 1911},
    {-1887, 201, 1911}, {-1887, 201, 1910}, {-1887, 201, 1909},
    {-1887, 200, 1909},
};

Vec3f simulateHover(Collision* col, Vec3f pos, u16 angle, Vec3s skullPos,
                    int shieldFrame, bool debug) {
  f32 ySpeed = 5.8f;
  for (int i = 0; i < 10; i++) {
    Vec3f shieldCorners[4];
    bool hitShield = false;

    if (i >= shieldFrame) {
      AnimFrame animFrame;
      loadAnimFrame(gPlayerAnim_link_fighter_backturn_jump_Data, i, &animFrame);
      // left stance
      loadUpperBodyAnimFrame(gPlayerAnim_link_anchor_waitR2defense_Data, 2,
                             &animFrame);

      getShieldPosition(&animFrame, PLAYER_AGE_CHILD, pos, angle,
                        shieldCorners);

      Sphere16 skull = {skullPos, 21};
      hitShield = colliderSphVsQuad(&skull, shieldCorners);
    }

    if (debug) {
      printf(
          "i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) "
          "ySpeed=%.1f hitShield=%d\n",
          i, pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
          floatToInt(pos.z), ySpeed, hitShield);
      for (int j = 0; j < 4; j++) {
        printf("  shieldCorners[%d]: x=%.9g y=%.9g z=%.9g\n", j,
               shieldCorners[j].x, shieldCorners[j].y, shieldCorners[j].z);
      }
    }

    if (hitShield) {
      return pos;
    }

    ySpeed -= 1.0f;
    pos = col->runChecks(pos, translate(pos, angle + 0x8000, 6.0f, ySpeed));
  }

  return Vec3f();
}

void simulateHovers(Collision* col, Vec3f pos, u16 angle, int shieldFrame) {
  for (Vec3s skullPos : skullPositions) {
    Vec3f result = simulateHover(col, pos, angle, skullPos, shieldFrame, false);
    printf(
        "  shieldFrame=%d skullPos: x=%d y=%d z=%d result: x=%.9g (%08x) "
        "y=%.9g (%08x) z=%.9g (%08x)\n",
        shieldFrame, skullPos.x, skullPos.y, skullPos.z, result.x,
        floatToInt(result.x), result.y, floatToInt(result.y), result.z,
        floatToInt(result.z));
  }
}

void testHovers(Collision* col) {
  printf("hover 1\n");
  simulateHovers(
      col,
      {intToFloat(0xc4e3acd8), intToFloat(0x42bf8bfe), intToFloat(0x44f04acd)},
      0x48d8, 0);

  printf("hover 2\n");
  simulateHovers(
      col,
      {intToFloat(0xc4e793a2), intToFloat(0x42e72596), intToFloat(0x44f13def)},
      0x48d8, 0);

  printf("hover 3\n");
  simulateHovers(
      col,
      {intToFloat(0xc4e80c96), intToFloat(0x43075f97), intToFloat(0x44f1d6c8)},
      0x48d8, 0);

  printf("hover 4 (lower height)\n");
  for (int shieldFrame = 0; shieldFrame <= 5; shieldFrame++) {
    simulateHovers(col,
                   {intToFloat(0xc4e86b96), intToFloat(0x430e92ca),
                    intToFloat(0x44f1ff4d)},
                   0x48d8, shieldFrame);
  }

  printf("hover 4 (upper height)\n");
  for (int shieldFrame = 0; shieldFrame <= 5; shieldFrame++) {
    simulateHovers(col,
                   {intToFloat(0xc4e8bc0b), intToFloat(0x431445fd),
                    intToFloat(0x44f22628)},
                   0x48d8, shieldFrame);
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

  // findSetups(&col, argc, argv);

  testHovers(&col);

  return 0;
}
