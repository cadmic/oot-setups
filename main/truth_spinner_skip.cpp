#include "actor.hpp"
#include "collision_data.hpp"
#include "sys_math.hpp"

extern CollisionHeader object_haka_objects_Col_00A938;

bool testClip(Collision* col, Vec3f pos, u16 angle, bool debug) {
  CollisionPoly* wallPoly;
  CollisionPoly* floorPoly;
  int dynaId;
  f32 floorHeight = BGCHECK_Y_MIN;

  f32 yvel = 4.8f;
  for (int i = 0; i < 15; i++) {
    if (debug) {
      printf(
          "i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) yvel=%.1f "
          "angle=%04x\n",
          i, pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
          floatToInt(pos.z), yvel, angle);
    }

    if (pos.y <= floorHeight) {
      if (debug) {
        printf("hit floor\n");
      }
      return false;
    }

    yvel -= 1.0f;
    pos = col->runChecks(pos, translate(pos, angle, -18.0f, yvel), &wallPoly,
                         &floorPoly, &dynaId, &floorHeight);
  }

  if (debug) {
    printf(
        "x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) yvel=%.1f "
        "angle=%04x\n",
        pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
        floatToInt(pos.z), yvel, angle);
  }

  if (pos.y < -63.0f) {
    if (debug) {
      printf("fell too far\n");
    }
    return false;
  }

  pos = col->runChecks(pos, translate(pos, angle, -17.0f, -11.2f));

  if (debug) {
    printf(
        "x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) yvel=%.1f "
        "angle=%04x\n",
        pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
        floatToInt(pos.z), yvel, angle);
  }

  if (pos.x <= 1403.0f) {
    if (debug) {
      printf("hit wall\n");
    }
    return false;
  }

  return true;
}

void findClipRegion(Collision* col) {
  int tested = 0;
  int found = 0;
  for (u16 angle = 0xaf00; angle < 0xc400; angle += 0x10) {
    for (f32 x = 970.0f; x < 1006.0f; x += 0.5f) {
      for (f32 z = -175.0f; z < 45.0f; z += 0.5f) {
        if (tested % 1000 == 0) {
          fprintf(stderr, "tested=%d found=%d angle=%04x x=%.1f z=%.1f ... \r",
                  tested, found, angle, x, z);
        }
        tested++;

        Vec3f pos = {x, -63.0f, z};
        pos = translate(pos, angle + 0x8000, 6.0f, 4.8f);
        if (testClip(col, pos, angle, false)) {
          found++;
          printf("angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x\n", angle, x,
                 z, floatToInt(x), floatToInt(z));
        }
      }
    }
  }
}

bool testSidehopSiderollMegaflip(Collision* col, f32 x, f32 z, u16 angle) {
  Vec3f pos = {x, -63.0f, z};
  // backflip
  for (int i = 0; i < 11; i++) {
    pos = translate(pos, angle + 0x8000, 6.0f, 0.0f);
  }

  pos = translate(pos, angle + 0x8000, 5.0f, 0.0f);
  if (pos.x > 1006.0f) {
    return false;
  }

  // sideroll
  pos = translate(pos, angle - 0x4000 + 0xbb8, 2.0f, 0.0f);
  pos = translate(pos, angle - 0x4000 + 2 * 0xbb8, 3.0f, 0.0f);
  for (int i = 0; i < 9; i++) {
    pos = translate(pos, angle - 0x4000 + 3 * 0xbb8, 3.0f, 0.0f);
  }

  // start megaflip
  pos = translate(pos, angle + 0x8000 - 0x568, 6.0f, 4.8f);
  return testClip(col, pos, angle, false);
}

void findSidehopSiderollMegaflipRegion(Collision* col) {
  int tested = 0;
  int found = 0;
  u16 angle = 0xc000;
  for (f32 x = 880.0f; x < 920.0f; x += 0.1f) {
    for (f32 z = 0.0f; z < 40.0f; z += 0.1f) {
      if (tested % 1000 == 0) {
        fprintf(stderr, "tested=%d found=%d angle=%04x x=%.1f z=%.1f ... \r",
                tested, found, angle, x, z);
      }
      tested++;

      if (testSidehopSiderollMegaflip(col, x, z, angle)) {
        found++;
        printf("angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x\n", angle, x, z,
               floatToInt(x), floatToInt(z));
      }
    }
  }
}

bool testBackflipSiderollMegaflip(Collision* col, f32 x, f32 z, u16 angle,
                                  bool debug) {
  Vec3f pos = {x, -63.0f, z};
  // backflip
  for (int i = 0; i < 11; i++) {
    pos = translate(pos, angle + 0x8000, 6.0f, 0.0f);
  }

  pos = translate(pos, angle + 0x8000, 5.0f, 0.0f);
  if (pos.x > 1006.0f) {
    return false;
  }

  // sideroll
  u16 facingAngle = angle;
  u16 movementAngle = angle + 0x8000;
  for (int i = 0; i < 11; i++) {
    Math_ScaledStepToS(&movementAngle, facingAngle, 2000);
    Math_ScaledStepToS(&facingAngle, movementAngle, 2000);

    if (debug) {
      printf("i=%d x=%.9g (%08x) z=%.9g (%08x) movementAngle=%04x\n", i, pos.x,
             floatToInt(pos.x), pos.z, floatToInt(pos.z), movementAngle);
    }

    pos = translate(pos, movementAngle, i == 0 ? 2.0f : 3.0f, 0.0f);
    if (pos.x > 1006.0f) {
      return false;
    }
  }

  movementAngle += 2 * 0xbb8;
  if (debug) {
    printf("x=%.9g (%08x) z=%.9g (%08x) movementAngle=%04x\n", pos.x,
           floatToInt(pos.x), pos.z, floatToInt(pos.z), movementAngle);
  }
  pos = translate(pos, movementAngle, -6.0f, 5.8f);

  movementAngle += 2 * 0xbb8;
  return testClip(col, pos, movementAngle, debug);
}

void findBackflipSiderollMegaflipRegion(Collision* col) {
  int tested = 0;
  int found = 0;
  u16 angle = 0xc000;
  for (f32 x = 870.0f; x < 890.0f; x += 0.1f) {
    for (f32 z = -95.0f; z < -55.0f; z += 0.1f) {
      if (tested % 1000 == 0) {
        fprintf(stderr, "tested=%d found=%d angle=%04x x=%.1f z=%.1f ... \r",
                tested, found, angle, x, z);
      }
      tested++;

      if (testBackflipSiderollMegaflip(col, x, z, angle, false)) {
        found++;
        printf("angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x\n", angle, x, z,
               floatToInt(x), floatToInt(z));
      }
    }
  }
}

int main(int argc, char* argv[]) {
  // TODO: pare this down
  Collision col(&HAKAdan_sceneCollisionHeader_016394, PLAYER_AGE_ADULT,
                {1200, -120, -100}, {1421, -50, 200});
  col.addDynapoly(&object_haka_objects_Col_00A938, {0.1f, 0.1f, 0.1f},
                  {0, 0, 0}, {1421.0f, -63.0f, 55});

  // testClip(&col, intToFloat(0x4479ed7f), intToFloat(0xc3127718), 0xb1f0,
  // true);

  // findClipRegion(&col);
  // findSidehopSiderollMegaflipRegion(&col);

  // testBackflipSiderollMegaflip(&col, intToFloat(0x445da8a8),
  //                              intToFloat(0xc2a7f5d9), 0xc000, true);
  findBackflipSiderollMegaflipRegion(&col);

  return 0;
}

CamData object_haka_objects_Col_00A938CamDataList[] = {
    {0x0000, 0, NULL},
};

SurfaceType object_haka_objects_Col_00A938SurfaceType[] = {
    {0x00000000, 0x000007C0},
};

CollisionPoly object_haka_objects_Col_00A938Polygons[] = {
    {0x0000, 0x0000, 0x0001, 0x0002, 0x8001, 0x0000, 0x0000, 0x0000},
    {0x0000, 0x0000, 0x0002, 0x0003, 0x8001, 0x0000, 0x0000, 0x0000},
};

Vec3s object_haka_objects_Col_00A938Vertices[] = {
    {0, 0, 400},
    {0, 1000, 400},
    {0, 1000, -400},
    {0, 0, -400},
};

CollisionHeader object_haka_objects_Col_00A938 = {
    {0, 0, -400},
    {0, 1000, 400},
    4,
    object_haka_objects_Col_00A938Vertices,
    2,
    object_haka_objects_Col_00A938Polygons,
    object_haka_objects_Col_00A938SurfaceType,
    object_haka_objects_Col_00A938CamDataList,
    0,
    NULL};
