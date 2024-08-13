#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "camera_angles.hpp"
#include "collision.hpp"
#include "collision_data.hpp"
#include "control_stick.hpp"
#include "search.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"

// Tomb collision data

BgCamInfo object_spot02_objects_Col_0133ECCamDataList[] = {
    {0x0000, 0, NULL},
};

SurfaceType object_spot02_objects_Col_0133ECSurfaceType[] = {
    {0x00000000, 0x000007C2},
    {0x00200000, 0x000007C2},
};

CollisionPoly object_spot02_objects_Col_0133ECPolygons[] = {
    {0x0000, 0x0000, 0x0001, 0x0002, 0x8613, 0x242A, 0xF189, 0xFCC8},
    {0x0000, 0x0003, 0x0000, 0x0002, 0x8089, 0x0000, 0xF456, 0xFCBA},
    {0x0000, 0x0004, 0x0005, 0x0006, 0x79ED, 0x242A, 0xF189, 0xFCC8},
    {0x0000, 0x0004, 0x0006, 0x0007, 0x7F77, 0x0000, 0xF456, 0xFCBA},
    {0x0001, 0x0008, 0x0009, 0x0005, 0x7FFF, 0x0000, 0x0000, 0xFD06},
    {0x0001, 0x0008, 0x0005, 0x0004, 0x7FFF, 0x0000, 0x0000, 0xFD06},
    {0x0000, 0x000A, 0x0009, 0x0008, 0x5580, 0x5F40, 0x0000, 0xFADE},
    {0x0000, 0x000B, 0x000C, 0x000A, 0x0000, 0x7877, 0xD4BF, 0xFAE0},
    {0x0000, 0x000B, 0x000A, 0x0008, 0x0000, 0x7877, 0xD4BF, 0xFAE0},
    {0x0000, 0x000B, 0x000D, 0x000C, 0xAA80, 0x5F40, 0x0000, 0xFADE},
    {0x0001, 0x0001, 0x000D, 0x000B, 0x8001, 0x0000, 0x0000, 0xFD06},
    {0x0001, 0x0002, 0x0001, 0x000B, 0x8001, 0x0000, 0x0000, 0xFD06},
    {0x0001, 0x0002, 0x000B, 0x0008, 0x0000, 0x0000, 0x8001, 0xFC9A},
    {0x0001, 0x0002, 0x0008, 0x0004, 0x0000, 0x0000, 0x8001, 0xFC9A},
    {0x0000, 0x0007, 0x0006, 0x0000, 0x0000, 0x0000, 0x7FFF, 0xFDFA},
    {0x0000, 0x0007, 0x0000, 0x0003, 0x0000, 0x0000, 0x7FFF, 0xFDFA},
    {0x0000, 0x0000, 0x0006, 0x0005, 0x0000, 0x7E37, 0x1549, 0xFF2D},
    {0x0000, 0x0000, 0x0005, 0x0001, 0x0000, 0x7E37, 0x1549, 0xFF2D},
    {0x0001, 0x0009, 0x000A, 0x000C, 0x0000, 0x0000, 0x7FFF, 0x00EB},
    {0x0001, 0x0009, 0x000C, 0x000D, 0x0000, 0x0000, 0x7FFF, 0x00EB},
    {0x0001, 0x0005, 0x0009, 0x000D, 0x0000, 0x0000, 0x7FFF, 0x00EB},
    {0x0001, 0x0005, 0x000D, 0x0001, 0x0000, 0x0000, 0x7FFF, 0x00EB},
};

Vec3s object_spot02_objects_Col_0133ECVertices[] = {
    {-889, 127, 518},   {-762, 254, -235},  {-762, 0, -870},
    {-889, 0, 518},     {762, 0, -870},     {762, 254, -235},
    {889, 127, 518},    {889, 0, 518},      {762, 1082, -870},
    {762, 1082, -235},  {508, 1310, -235},  {-762, 1082, -870},
    {-508, 1310, -235}, {-762, 1082, -235},
};

CollisionHeader object_spot02_objects_Col_0133EC = {
    {-889, 0, -870},
    {889, 1310, 518},
    14,
    object_spot02_objects_Col_0133ECVertices,
    22,
    object_spot02_objects_Col_0133ECPolygons,
    object_spot02_objects_Col_0133ECSurfaceType,
    object_spot02_objects_Col_0133ECCamDataList,
    0,
    NULL};

// Invisible seam poly
void findSeamHeights() {
  Vec3f v1 = {629, 194, -73};
  Vec3f v2 = {694, 180, -72};
  Vec3f v3 = {694, 228, -76};
  Vec3f normal = {0x0054 * SHT_MINV, 0x0aa1 * SHT_MINV, 0x7f8e * SHT_MINV};
  s16 dist = 0x0037;

  for (f32 x = 693.0f; x <= 695.0f; x += 0.01) {
    for (f32 z = -77.0f; z <= -75.0f; z += 0.01) {
      f32 yIntersect;
      if (Math3D_TriChkPointParaYIntersectInsideTri(&v1, &v2, &v3, normal.x,
                                                    normal.y, normal.z, dist, z,
                                                    x, &yIntersect, 1.0f)) {
        printf("x=%.7g y=%.7g z=%.7g\n", x, yIntersect, z);
      }
    }
  }
}

bool simulateJumpRoll(Collision* col, Vec3f pos, u16 angle, Vec3f* outPos, bool debug) {
  f32 xzSpeed = 0.0f;
  bool jump = false;

  Vec3f normal = {
      (s16)0xc4ad * COLPOLY_NORMAL_FRAC,
      (s16)0x716b * COLPOLY_NORMAL_FRAC,
      (s16)0x0000 * COLPOLY_NORMAL_FRAC,
  };
  u16 floorPitch = computeFloorPitch(normal, angle);
  f32 maxSpeed = std::min(controlStickSpeed(0, 127, floorPitch, SPEED_MODE_CURVED) * 1.5f, 8.25f);

  printf("normal: x=%.9g y=%.9g z=%.9g\n", normal.x, normal.y, normal.z);
  printf("floorPitch=%04x maxSpeed=%.9g\n", floorPitch, maxSpeed);

  for (int i = 0; i < 11; i++) {
    xzSpeed = std::min(xzSpeed + 2.0f, maxSpeed);

    if (debug) {
      printf(
          "i=%02d angle=%04x x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) "
          "xzSpeed=%.2f\n",
          i, angle, pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
          floatToInt(pos.z), xzSpeed);
    }

    pos = translate(pos, angle, xzSpeed, -5.0f);

    Vec3f floorPos = col->findFloor({pos.x, pos.y + 50.0f, pos.z});
    if (floorPos.y < 200) {
      jump = true;
      break;
    }
    pos = floorPos;
  }

  if (!jump) {
    if (debug) {
      printf("no jump\n");
    }
    return false;
  }

  if (debug) {
    printf("jump x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n",
           pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  if (xzSpeed < 6.0f) {
    if (debug) {
      printf("too slow\n");
    }
    return false;
  }

  if (pos.y < 222.5f) {
    if (debug) {
      printf("too low\n");
    }
    return false;
  }

  if (debug) {
    printf("success\n");
  }
  *outPos = pos;
  return true;

}

bool simulateJump(Collision* col, Vec3f pos, u16 angle, int jsFrame, bool holdUp, bool debug) {
  f32 xzSpeed = 5.5f;
  f32 ySpeed = 7.5f;
  for (int i = 0; i < 20; i++) {
    if (debug) {
      printf(
          "i=%02d angle=%04x x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) xzSpeed=%.2f ySpeed=%.1f\n",
          i, angle, pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
          floatToInt(pos.z), xzSpeed, ySpeed);
    }

    if (i > jsFrame + 1) {
      ySpeed -= 1.2f;
    } else {
      ySpeed -= 1.0f;
    }
    Vec3f velocity = {Math_SinS(angle) * xzSpeed, ySpeed,
                      Math_CosS(angle) * xzSpeed};
    Vec3f posNext = pos + velocity * 1.5f;

    CollisionPoly* wallPoly;
    CollisionPoly* floorPoly;
    int dynaId;
    f32 floorHeight;
    // TODO: no need to run every frame
    pos = col->runChecks(pos, posNext, &wallPoly, &floorPoly, &dynaId, &floorHeight);

    if (pos.y <= floorHeight && pos.y > 280) {
      if (debug) {
        printf("success\n");
      }
      return true;
    }

    if (i == jsFrame) {
      // jumpslash
      xzSpeed = 3.0f;
      ySpeed = 4.5f;
    } else if (i > jsFrame) {
      if (holdUp) {
        xzSpeed += 0.05f;
      }
    }
  }

  return false;
}

void findJumps(Collision* col) {
  PosAngleRange range = {
      .angleMin = 0x22f0,
      .angleMax = 0x24a0,
      .angleStep = 0x10,
      .xMin = 640.0f,
      .xMax = 681.0f,
      .xStep = 0.05f,
      .zMin = -121.0f,
      .zMax = -87.0f,
      .zStep = 0.05f,
  };

  searchPosAngleRange(range, [=](u16 angle, f32 x, f32 z) {
    Vec3f startPos = col->findFloor({x, 500.0f, z});
    if (startPos.y <= 180) {
      return false;
    }

    Vec3f pos;
    if (!simulateJumpRoll(col, startPos, angle, &pos, false)) {
      return false;
    }

    if (!simulateJump(col, pos, angle, 10, true, false)) {
      return false;
    }

    printf("angle=%04x x=%.9g x_raw=%08x y=%.9g y_raw=%08x z=%.9g z_raw=%08x\n",
        angle, startPos.x, floatToInt(startPos.x), startPos.y, floatToInt(startPos.y), startPos.z, floatToInt(startPos.z));
    fflush(stdout);
    return true;
  });
}


void findJumpSetups(Collision* col) {
  SearchParams params = {
      .col = col,
      .minBounds = {-10000, 180, -10000},
      .maxBounds = {10000, 10000, 10000},
      .starts =
          {
              // target grave, sidehop right to wall
              {{708, 180, -306}, 0xc000},
              // target wall, turn left, sidehop left x2, walk forward to grave
              {{708, 180, -130.5f}, 0xc000},
              // target wall, turn left, sidehop left x2, sideroll, walk forward to grave
              {{708, 180, intToFloat(0xc2c3060f)}, 0xc000},
              // target tomb, backflip, sidehop right to grave,
              {{708, 180, -117.5f}, 0x0000},
              // target tomb, turn right, sidehop right, walk forward to grave
              {{708, 180, -98.75f}, 0xc000},
          },
      .maxCost = 85,
      .angleMin = 0x2310,
      .angleMax = 0x2480,
      .xMin = 640.8f,
      .xMax = 680.9f,
      .zMin = -120.6f,
      .zMax = -88.0f,
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
              HORIZONTAL_SLASH_SHIELD,
              DIAGONAL_SLASH_SHIELD,
              VERTICAL_SLASH_SHIELD,
              FORWARD_STAB_SHIELD,
              JUMPSLASH_SHIELD,
          },
  };

  auto filter = [&](Vec3f initialPos, u16 initialAngle,
                    const PosAngleSetup& setup, const std::vector<Action>& path,
                    int cost) {
    Vec3f pos = setup.pos;

    // Don't talk to Sharp
    Vec3f wonderPos = {-630, 199, -100};
    f32 distToPlayer = Math_Vec3f_DistXZ(&wonderPos, &pos);
    u16 yawTowardPlayer = Math_Vec3f_Yaw(&wonderPos, &pos);

    if (distToPlayer <= 110.0f && yawTowardPlayer > 0x8000) {
      return false;
    }

    return true;
  };

  auto output = [=](Vec3f initialPos, u16 initialAngle,
                    const PosAngleSetup& setup,
                    const std::vector<Action>& actions, int cost) {

    Vec3f startPos = setup.pos;
    u16 angle = cameraAngles[setup.angle];

    Vec3f pos;
    if (!simulateJumpRoll(col, startPos, angle, &pos, false)) {
      return false;
    }

    if (!simulateJump(col, pos, angle, 10, true, false)) {
      return false;
    }

    printf(
        "cost=%d initialx=%.9g (%08x) initialz=%.9g (%08x) "
        "initialAngle=%04x x=%.9g (%08x) z=%.9g (%08x) angle=%04x actions=%s\n",
        cost, initialPos.x, floatToInt(initialPos.x), initialPos.z,
        floatToInt(initialPos.z), initialAngle, startPos.x, floatToInt(startPos.x),
        startPos.z, floatToInt(startPos.z), angle, actionNames(actions).c_str());
    fflush(stdout);
    return true;
  };

  searchSetups(params, filter, output);
}

bool simulateRoll(Collision* col, Vec3f pos, u16 facingAngle, Vec3f* outPos, bool debug) {
  f32 xzSpeed = 0.0f;
  u16 angle = cameraAngles[facingAngle] + 0x8000;
  bool jump = false;
  Math_ScaledStepToS(&angle, facingAngle, 2000);

  for (int i = 0; i < 11; i++) {
    xzSpeed = std::min(xzSpeed + 2.0f, 8.25f);

    if (debug) {
      printf(
          "i=%02d angle=%04x x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) "
          "xzSpeed=%.2f\n",
          i, angle, pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
          floatToInt(pos.z), xzSpeed);
    }

    pos = translate(pos, angle, xzSpeed, -5.0f);

    Vec3f floorPos = col->findFloor({pos.x, pos.y + 50.0f, pos.z});
    if (floorPos.y < 250) {
      jump = true;
      break;
    }
    pos = floorPos;
  }

  if (!jump) {
    if (debug) {
      printf("no jump\n");
    }
    return false;
  }

  if (debug) {
    printf("jump x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n",
           pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  if (xzSpeed < 6.0f) {
    if (debug) {
      printf("too slow\n");
    }
    return false;
  }

  if (debug) {
    printf("success\n");
  }
  *outPos = pos;
  return true;
}

bool simulateClip(Collision* col, Vec3f pos, u16 facingAngle, int releaseDownFrame, int jsFrame, bool debug) {
  f32 xzSpeed = 5.5f;
  f32 ySpeed = 7.5f;
  u16 angle = cameraAngles[facingAngle] + 0x8000;
  f32 animFrameNum = 0.0f;
  Math_ScaledStepToS(&angle, facingAngle, 2000);
  Math_ScaledStepToS(&angle, facingAngle + 0x8000, 200);

  AnimFrame animFrame;
  CollisionPoly* wallPoly;
  CollisionPoly* floorPoly;
  int dynaId;
  f32 floorHeight;

  for (int i = 0; i < 40; i++) {
    if (debug) {
      printf(
          "i=%02d angle=%04x x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) xzSpeed=%.2f ySpeed=%.1f animFrameNum=%.0f\n",
          i, angle, pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z, floatToInt(pos.z), xzSpeed, ySpeed, animFrameNum);
    }

    if (animFrameNum >= 7.0f) {
      // Check for recoil
      loadAnimFrame(gPlayerAnim_link_fighter_Lpower_jump_kiru_Data, animFrameNum, &animFrame);
      if (weaponRecoil(col, &animFrame, PLAYER_AGE_CHILD, 5000.0f, pos, angle)) {
        if (debug) {
          printf("recoil\n");
        }
        return false;
      }
    }

    if (i > jsFrame) {
      animFrameNum = std::min(animFrameNum + 1.0f, 9.0f);
    }

    if (i > jsFrame + 1) {
      ySpeed -= 1.2f;
    } else {
      ySpeed -= 1.0f;
    }

    pos = col->runChecks(pos, translate(pos, angle, xzSpeed, ySpeed), &wallPoly, &floorPoly, &dynaId, &floorHeight);

    if (pos.y < 180) {
      if (debug) {
        printf("too low\n");
      }
      return false;
    }

    if (pos.y <= floorHeight) {
      if (pos.y > 250) {
        if (debug) {
          printf("no jump\n");
        }
        return false;
      }

      break;
    }

    if (i < releaseDownFrame) {
      // hold down
      Math_ScaledStepToS(&angle, facingAngle + 0x8000, 200);
    } else if (i < jsFrame) {
      // release down
      xzSpeed = std::max(0.0f, xzSpeed - 1.0f);
    } else if (i == jsFrame) {
      // jumpslash
      xzSpeed = 3.0f;
      ySpeed = 4.5f;
      angle = facingAngle;
    } else {
      xzSpeed -= 0.10f;
    }
  }

  xzSpeed = 0.0f;
  if (debug) {
    printf(
        "landing x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) xzSpeed=%.2f ySpeed=%.1f\n",
        pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z, floatToInt(pos.z), xzSpeed, ySpeed);
  }

  if (pos.y == 180) {
    if (debug) {
      printf("hit ground\n");
    }
    return false;
  }

  if (pos.x == 771) {
    if (debug) {
      printf("hit front wall\n");
    }
    return false;
  }

  // Simulate ground clip
  // pos.y -= 0.0001f;

  Vec3f prevRoot = baseRootTranslation(PLAYER_AGE_CHILD, angle);
  Vec3f prevPos = pos;

  loadAnimFrame(gPlayerAnim_link_fighter_Lpower_jump_kiru_hit_Data, 0, &animFrame);
  updateRootTranslation(&animFrame, &pos, angle, &prevRoot);

  ySpeed -= 1.2f;
  pos = col->runChecks(prevPos, translate(pos, angle, 0, ySpeed), &wallPoly, &floorPoly, &dynaId, &floorHeight);

  if (debug) {
    printf(
        "jumpslash x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) xzSpeed=%.2f ySpeed=%.1f\n",
        pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z, floatToInt(pos.z), xzSpeed, ySpeed);
  }

  if (pos.z <= 7 || pos.z >= 152 || pos.x <= 789) {
    if (debug) {
      printf("no clip\n");
    }
    return false;
  }

  if (debug) {
    printf("success\n");
  }
  return true;
}

void findClips(Collision* col) {
  PosAngleRange range = {
      .angleMin = 0x3000,
      .angleMax = 0x3200,
      .angleStep = 0x1,
      .xMin = 820.0f, // edge: 786
      .xMax = 850.0f, // edge: 850
      .xStep = 0.01f,
      .zMin = 2.0f, // edge: 2
      .zMax = 7.0f, // edge: 157
      .zStep = 0.1f,
  };

  searchPosAngleRange(range, [=](u16 angle, f32 x, f32 z) {
    if (angle != cameraAngles[angle]) {
      return false;
    }

    Vec3f startPos = col->findFloor({x, 500.0f, z});
    if (startPos.y < 250) {
      return false;
    }

    Vec3f pos;
    if (!simulateRoll(col, startPos, angle, &pos, false)) {
      return false;
    }
    if (!simulateClip(col, pos, angle, 7, 10, false)) {
      return false;
    }

    printf("angle=%04x x=%.9g x_raw=%08x y=%.9g y_raw=%08x z=%.9g z_raw=%08x\n",
        angle, startPos.x, floatToInt(startPos.x), startPos.y, floatToInt(startPos.y), startPos.z, floatToInt(startPos.z));
    fflush(stdout);
    return true;
  });
}

int main(int argc, char* argv[]) {
  // jump to grave
  Collision graveCol(&spot02_sceneCollisionHeader_003C54, PLAYER_AGE_CHILD, {620, 180, -320}, {850, 230, 156});
  graveCol.addDynapoly(&object_spot02_objects_Col_0133EC, {0.1f, 0.1f, 0.1f},
                  {0, (s16)0xc000, 0}, {762, 180, 80});
  // graveCol.printPolys();

  // floor under tomb
  Collision tombCol(&spot02_sceneCollisionHeader_003C54, PLAYER_AGE_CHILD, {744, 180, 56}, {792, 180, 104});
  tombCol.addDynapoly(&object_spot02_objects_Col_0133EC, {0.1f, 0.1f, 0.1f},
                  {0, (s16)0xc000, 0}, {762, 180, 80});
  // tombCol.printPolys();

  // findSeamHeights();

  // Vec3f pos = {intToFloat(0x4420a610), intToFloat(0x4348b0bf), intToFloat(0xc2ef3cbc)};
  // u16 angle = 0x23e9;
  // simulateJumpRoll(&graveCol, pos, angle, &pos, true);
  // simulateJump(&graveCol, pos, angle, 10, true, true);

  // findJumps(&graveCol);
  findJumpSetups(&graveCol);

  // Vec3f pos = {intToFloat(0x444e0000), intToFloat(0x438f84f1), intToFloat(0x40400000)};
  // u16 angle = 0x310c;
  // simulateRoll(&tombCol, pos, angle, &pos, true);
  // simulateClip(&tombCol, pos, angle, 7, 10, true);

  // findClips(&tombCol);
}
