#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "collision.hpp"
#include "collision_data.hpp"
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

bool simulateJump(Collision* col, Vec3f pos, u16 angle, f32 initialSpeed,
                  int jsFrame, bool holdUp, bool debug) {
  f32 xzSpeed = initialSpeed;
  f32 ySpeed = -4.0f;
  for (int i = 0; i < 20; i++) {
    if (debug) {
      printf(
          "i=%02d x=%.7g y=%.7g z=%.7g x_raw=%08x y_raw=%08x z_raw=%08x "
          "angle=%04x "
          "xzSpeed=%.7g ySpeed=%.7g\n",
          i, pos.x, pos.y, pos.z, floatToInt(pos.x), floatToInt(pos.y),
          floatToInt(pos.z), angle, xzSpeed, ySpeed);
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
      return true;
    }

    if (i == 0) {
      // jump
      xzSpeed = 6.0f;
      ySpeed = 7.5f;
    } else if (i == jsFrame) {
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

void findJumpSpots(Collision* col) {
  int i = 0;
  for (f32 speed = 6.0f; speed <= 9.0f; speed += 0.25f) {
    for (int angle = 0x2200; angle < 0x2600; angle += 0x10) {
      for (f32 x = 693.0f; x <= 695.0f; x += 0.05) {
        for (f32 z = -77.0f; z <= -76.0f; z += 0.05) {
          for (int js = 9; js <= 11; js++) {
            if (i % 10000 == 0) {
              fprintf(stderr, "i=%d speed=%g angle=%04x x=%.7g z=%.7g\r", i,
                      speed, angle, x, z);
            }
            i++;

            Vec3f pos = col->findFloor({x, 250, z});
            if (pos.y < 230.0f) {
              continue;
            }

            if (simulateJump(col, pos, angle, speed, js, true, false)) {
              printf(
                  "speed=%.7g angle=%04x x=%.7g y=%.7g z=%.7g x_raw=%08x "
                  "y_raw=%08x z_raw=%08x js=%d\n",
                  speed, angle, pos.x, pos.y, pos.z, floatToInt(pos.x),
                  floatToInt(pos.y), floatToInt(pos.z), js);
            }
          }
        }
      }
    }
  }
}

bool simulateRoll(Collision* col, Vec3f pos, u16 facingAngle, int rollDir, Vec3f* outPos, bool debug) {
  f32 xzSpeed = 0.0f;
  u16 angle = facingAngle + 0x8000 - 0xbb8 * rollDir;
  bool jump = false;

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

bool simulateClip(Collision* col, Vec3f pos, u16 facingAngle, int rollDir, int releaseDownFrame, int jsFrame, bool debug) {
  f32 xzSpeed = 5.5f;
  f32 ySpeed = 7.5f;
  u16 angle = facingAngle + 0x8000 - 0xbb8 * rollDir;
  f32 animFrameNum = 0.0f;
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
  pos.y -= 0.0001f;

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
      .angleMin = 0x1800,
      .angleMax = 0x6800,
      .angleStep = 0x80,
      .xMin = 786.0f, // edge: 786
      .xMax = 850.0f, // edge: 850
      .xStep = 1.0f,
      .zMin = 2.0f, // edge: 2
      .zMax = 157.0f, // edge: 157
      .zStep = 1.0f,
  };

  searchPosAngleRange(range, [=](u16 angle, f32 x, f32 z) {
    Vec3f startPos = col->findFloor({x, 500.0f, z});
    if (startPos.y < 250) {
      return false;
    }

    bool found = false;
    for (int rollDir : {-1, 1}) {
      for (int releaseDownFrame = 2; releaseDownFrame <= 7; releaseDownFrame++) {
        int minJsFrame = -1;
        int maxJsFrame = -1;

        for (int jsFrame = 8; jsFrame <= 14; jsFrame++) {
          Vec3f pos;
          if (!simulateRoll(col, startPos, angle, rollDir, &pos, false)) {
            continue;
          }
          if (!simulateClip(col, pos, angle, rollDir, releaseDownFrame, jsFrame, false)) {
            continue;
          }

          if (minJsFrame == -1) {
            minJsFrame = jsFrame;
          }
          maxJsFrame = jsFrame;
        }

        if (minJsFrame != -1) {
          printf("angle=%04x x=%.9g x_raw=%08x y=%.9g y_raw=%08x z=%.9g z_raw=%08x rollDir=%d releaseDown=%d minJsFrame=%d maxJsFrame=%d jsFrames=%d\n",
              angle, startPos.x, floatToInt(startPos.x), startPos.y, floatToInt(startPos.y), startPos.z, floatToInt(startPos.z),
              rollDir, releaseDownFrame, minJsFrame, maxJsFrame, maxJsFrame - minJsFrame + 1);
          fflush(stdout);
          found = true;
          break;
        }
      }
    }
    return found;
  });
}

int main(int argc, char* argv[]) {
  // grave to jump
  // Collision col(&spot02_sceneCollisionHeader_003C54, PLAYER_AGE_CHILD, {620, 180, -320}, {850, 230, 156});

  // floor under tomb
  Collision col(&spot02_sceneCollisionHeader_003C54, PLAYER_AGE_CHILD, {744, 180, 56}, {792, 180, 104});
  col.addDynapoly(&object_spot02_objects_Col_0133EC, {0.1f, 0.1f, 0.1f},
                  {0, (s16)0xc000, 0}, {762, 180, 80});
  // col.printPolys();

  // findSeamHeights();

  // Jolin's demo
  // simulateJump(
  //     &col,
  //     {intToFloat(0x442d9005), intToFloat(0x436aaf8a), intToFloat(0xc299107a)},
  //     0x23f0, 9.0f, 10, true);

  // findJumpSpots(&col);

  // angle=5b80 x=848 x_raw=44540000 y=288.211761 y_raw=43901b1b z=46 z_raw=42380000 rollDir=-1 releaseDown=7 minJsFrame=10 maxJsFrame=10 jsFrames=1
  // Vec3f pos = {intToFloat(0x44540000), intToFloat(0x43901b1b), intToFloat(0x42380000)};
  // u16 angle = 0x5b80;
  // simulateRoll(&col, pos, angle, -1, &pos, true);
  // simulateClip(&col, pos, angle, -1, 7, 10, true);

  findClips(&col);
}
