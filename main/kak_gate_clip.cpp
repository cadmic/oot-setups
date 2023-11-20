#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "collider.hpp"
#include "collision_data.hpp"
#include "pos_angle_setup.hpp"
#include "search.hpp"
#include "sys_math.hpp"

CamData gKakarikoGuardGateColCamDataList[] = {
    {0x0000, 0, NULL},
};

SurfaceType gKakarikoGuardGateColSurfaceType[] = {
    {0x00000000, 0x000007C0},
};

CollisionPoly gKakarikoGuardGateColPolygons[] = {
    {0x0000, 0x0000, 0x0001, 0x0002, 0x0000, 0x0000, 0x7FFF, 0x0000},
    {0x0000, 0x0000, 0x0002, 0x0003, 0x0000, 0x0000, 0x7FFF, 0x0000},
};

Vec3s gKakarikoGuardGateColVertices[] = {
    {100, 0, 0},
    {100, 120, 0},
    {-100, 120, 0},
    {-100, 0, 0},
};

CollisionHeader gKakarikoGuardGateCol = {
    {-100, 0, 0},
    {100, 120, 0},
    4,
    gKakarikoGuardGateColVertices,
    2,
    gKakarikoGuardGateColPolygons,
    gKakarikoGuardGateColSurfaceType,
    gKakarikoGuardGateColCamDataList,
    0,
    NULL,
};

bool testHessClip(Collision* col, Vec3f pos, u16 angle) {
  for (int i = 0; i < 100; i++) {
    pos = col->runChecks(pos, translate(pos, angle, -18.0f, -5.0f));
    if (pos.z < -1365) {
      return true;
    }
    if (pos.x < -150) {
      return false;
    }
  }
  return false;
}

void findHessClips(Collision* col) {
  PosAngleRange range = {
      .angleMin = 0x2200,
      .angleMax = 0x2400,
      .xMin = 40.0f,
      .xMax = 120.0f,
      .xStep = 0.1f,
      .zMin = -1320.0f,
      .zMax = -1320.0f,
      .zStep = 0.1f,
  };

  searchPosAngleRange(range, [=](u16 angle, f32 x, f32 z) {
    Vec3f pos = {x, 400.0f, z};
    if (testHessClip(col, pos, angle)) {
      printf("angle=%04x x=%.9g (%08x) z=%.9g (%08x)\n", angle, pos.x,
             floatToInt(pos.x), pos.z, floatToInt(pos.z));
      return true;
    }
    return false;
  });
}

bool testJumpslashClip(Collision* col, Vec3f pos, u16 angle, bool holdUp,
                       bool debug) {
  // Quick filter by distance to corner
  Vec3f corner = {-7, 400, -1365};
  f32 cornerDist = Math_Vec3f_DistXZ(&pos, &corner);
  if (holdUp) {
    if (cornerDist < 87.5f || cornerDist > 89.5f) {
      if (debug) {
        printf("wrong distance to corner: cornerDist=%.9f\n", cornerDist);
      }
      return false;
    }
  } else {
    if (cornerDist < 81.0f || cornerDist > 83.0f) {
      if (debug) {
        printf("wrong distance to corner: cornerDist=%.9f\n", cornerDist);
      }
      return false;
    }
  }

  Vec3f guardPos = {-30, 400, -1298};
  f32 guardRadius = 15.0f;

  // Don't start inside guard
  if (Math_Vec3f_DistXZ(&pos, &guardPos) < 12 + guardRadius) {
    if (debug) {
      printf("inside guard\n");
    }
    return false;
  }

  f32 xzSpeed = 5.0f;
  f32 ySpeed = 5.0f;
  f32 gravity = 1.0f;
  f32 accel = holdUp ? 0.05f : -0.1f;
  AnimFrame animFrame;

  for (int i = 0; i < 8; i++) {
    if (debug) {
      printf("i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n", i,
             pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
             floatToInt(pos.z), ySpeed);
    }

    if (i == 7) {
      // Check for deku stick break
      loadAnimFrame(gPlayerAnim_link_fighter_Lpower_jump_kiru_Data, 7, &animFrame);
      bool stickHit = weaponRecoil(col, &animFrame, PLAYER_AGE_CHILD, 5000.0f, pos, angle);
      if (stickHit) {
        if (debug) {
          printf("deku stick breaks in air\n");
        }
        return false;
      }
    }

    Vec3f displacement = immovablePush(pos, guardPos, guardRadius);
    ySpeed -= gravity;
    pos = col->runChecks(pos, translate(pos, angle, xzSpeed, ySpeed, displacement));
    xzSpeed += accel;
    gravity = 1.2f;
  }

  Vec3f prevRoot = baseRootTranslation(PLAYER_AGE_CHILD, angle);
  Vec3f prevPos = pos;

  loadAnimFrame(gPlayerAnim_link_fighter_Lpower_jump_kiru_hit_Data, 0, &animFrame);
  updateRootTranslation(&animFrame, &pos, angle, &prevRoot);

  if (debug) {
    printf("jumpslash: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n",
           pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z), ySpeed);
  }

  pos = col->runChecks(prevPos, translate(pos, angle, 0.0f, -5.0f));
  prevPos = pos;

  loadAnimFrame(gPlayerAnim_link_fighter_Lpower_jump_kiru_hit_Data, 1, &animFrame);
  updateRootTranslation(&animFrame, &pos, angle, &prevRoot);

  if (debug) {
    printf("lunge: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n",
           pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z), ySpeed);
  }

  pos = col->runChecks(prevPos, translate(pos, angle, 10.0f, -5.0f));
  prevPos = pos;

  loadAnimFrame(gPlayerAnim_link_fighter_Lpower_jump_kiru_hit_Data, 2, &animFrame);
  updateRootTranslation(&animFrame, &pos, angle, &prevRoot);

  if (debug) {
    printf("result: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  bool stickHit = weaponRecoil(col, &animFrame, PLAYER_AGE_CHILD, 5000.0f, pos, angle);
  if (stickHit) {
    if (debug) {
      printf("deku stick breaks on ground\n");
    }
    return false;
  }

  return pos.z < -1365;
}

void findJumpslashClips(Collision* col) {
  PosAngleRange range = {
      .angleMin = 0x8a80,
      .angleMax = 0xa080,
      .xMin = 15.0f,
      .xMax = 56.0f,
      .xStep = 0.1f,
      .zMin = -1308.0f,
      .zMax = -1277.0f,
      .zStep = 0.1f,
  };

  searchPosAngleRange(range, [=](u16 angle, f32 x, f32 z) {
    Vec3f pos = {x, 400.0f, z};
    bool found = false;
    for (bool holdUp : {false, true}) {
      if (testJumpslashClip(col, pos, angle, holdUp, false)) {
        printf("angle=%04x x=%.9g x_raw=%08x z=%.9g z_raw=%08x holdUp=%d\n", angle, pos.x,
              floatToInt(pos.x), pos.z, floatToInt(pos.z), holdUp);
        fflush(stdout);
        found = true;
      }
    }
    return found;
  });
}

bool withinRange(Vec3f pos, u16 angle, Vec3f obj, f32 radius, u16 yawRange) {
  if (SQ(pos.x - obj.x) + SQ(pos.z - obj.z) >= SQ(radius)) {
    return false;
  }

  u16 yawDiff = Math_Vec3f_Yaw(&obj, &pos) - 0x8000 - angle;
  if (yawDiff > yawRange && yawDiff < 0x10000 - yawRange) {
    return false;
  }

  return true;
}

void findSetups(Collision* col) {
  SearchParams params = {
      .col = col,
      .minBounds = {-10000, 400, -10000},
      .maxBounds = {10000, 10000, 10000},
      .starts =
          {
              // right gate corner
              {{intToFloat(0x432d1146), 400, intToFloat(0xc4a5469e)}, 0x799b},
              {{intToFloat(0x432d1122), 400, intToFloat(0xc4a54680)}, 0x39eb},
              // far right corner
              // {{intToFloat(0x43709c4d), 400, intToFloat(0xc4a15a4e)},
              // 0x7992},
              // {{intToFloat(0x43709c42), 400, intToFloat(0xc4a15a0d)},
              // 0x3f34}, left gate corner
              // {{intToFloat(0x404a1ed2), 400, intToFloat(0xc4a8a330)}, 0x799b},
              // {{intToFloat(0x404a1ea2), 400, intToFloat(0xc4a8a32f)}, 0xb9c3},
          },
      .maxCost = 90,
      .angleMin = 0x8a80,
      .angleMax = 0xa080,
      .xMin = 15.0f,
      .xMax = 56.0f,
      .zMin = -1308.0f,
      .zMax = -1277.0f,
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

  auto output = [=](Vec3f initialPos, u16 initialAngle,
                    const PosAngleSetup& setup,
                    const std::vector<Action>& actions, int cost) {
    bool found = false;
    Vec3f pos = setup.pos;
    u16 angle = setup.angle;
    for (bool holdUp : {false, true}) {
      if (testJumpslashClip(col, pos, angle, holdUp, false)) {
        printf(
            "cost=%d initialx=%.9g (%08x) initialz=%.9g (%08x) "
            "initialAngle=%04x x=%.9g (%08x) z=%.9g (%08x) angle=%04x "
            "holdUp=%d actions=%s\n",
            cost, initialPos.x, floatToInt(initialPos.x), initialPos.z,
            floatToInt(initialPos.z), initialAngle, pos.x, floatToInt(pos.x),
            pos.z, floatToInt(pos.z), angle, holdUp,
            actionNames(actions).c_str());
        fflush(stdout);
        found = true;
      }
    }

    return found;
  };

  searchSetups(params, output);
}

void testSetup(Collision* col) {
  // Position in corner can vary so make sure it's lenient enough
  PosAngleRange range = {
      .angleMin = 0x799b,
      .angleMax = 0x799b,
      .xMin = 172.0f,
      .xMax = 174.0f,
      .xStep = 0.01f,
      .zMin = -1323.0f,
      .zMax = -1321.0f,
      .zStep = 0.01f,
  };

  searchPosAngleRange(range, [=](u16 angle, f32 x, f32 z) {
    Vec3f pos = {x, 400, z};
    if (col->runChecks(pos, pos) != pos) {
      return false;
    }

    PosAngleSetup setup(col, pos, angle);
    setup.performActions({
      ESS_TURN_LEFT,
      ROTATE_ESS_RIGHT,
      SIDEHOP_LEFT_SIDEROLL_UNTARGET,
      ROLL,
      SIDEHOP_RIGHT,
      ESS_TURN_RIGHT,
    });

    if (testJumpslashClip(col, setup.pos, setup.angle, true, false)) {
        printf("angle=%04x x=%.9g x_raw=%08x z=%.9g z_raw=%08x\n", angle, pos.x,
            floatToInt(pos.x), pos.z, floatToInt(pos.z));
      fflush(stdout);
      return true;
    }
    return false;
  });
}

int main(int argc, char* argv[]) {
  Collision col(&spot01_sceneCollisionHeader_004A1C, PLAYER_AGE_CHILD);

  std::vector<u32> polyAddrs = {
      0x8036c1dc,  // floor
      0x8036c1ec,  // floor
      0x8036c1fc,  // floor
      0x8036c24c,  // far right wall
      0x8036bb4c,  // right front wall
      0x8036bb5c,  // right front wall
      0x8036bb8c,  // right side wall
      0x8036bb9c,  // right side wall
      0x8036bbac,  // left front wall
      0x8036bbbc,  // left front wall
      0x8036bbec,  // left side wall
      0x8036bbfc,  // left side wall
  };
  for (u32 polyAddr : polyAddrs) {
    col.addPoly((polyAddr - 0x803695cc) / 0x10);
  }

  col.addDynapoly(&gKakarikoGuardGateCol, {1, 1, 1}, {0, (s16)0xf99a, 0},
                  {91, 400, -1350});
  // col.printPolys();

  // findHessClips(&col);

  // testJumpslashClip(&col, {intToFloat(0x420e0cfa), 400, intToFloat(0xc4a1cb27)}, 0x9677, false, true);
  // testJumpslashClip(&col, {intToFloat(0x41de6bce), 400, intToFloat(0xc4a149c7)}, 0x9232, false, true);
  // testJumpslashClip(&col, {intToFloat(0x425b3358), 400, intToFloat(0xc4a2c010)}, 0xa000, true, true);
  // testJumpslashClip(&col, {intToFloat(0xc0ecccd2), 400, intToFloat(0xc4a06698)}, 0x82a0, false, true);
  // findJumpslashClips(&col);

  // findSetups(&col);

  testSetup(&col);

  return 0;
}
