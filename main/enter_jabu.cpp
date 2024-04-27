#include <vector>

#include "bombchu.hpp"
#include "collision.hpp"
#include "collision_data.hpp"
#include "global.hpp"
#include "search.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"
#include "sys_matrix.hpp"

extern CollisionHeader gZorasFountainIceRampCol;

bool simulateChu(Collision* col, f32 x, f32 z, u16 angle, bool debug,
                 Vec3f* result) {
  Vec3f pos = col->findFloor({x, 1000, z});
  if (pos.y == BGCHECK_Y_MIN) {
    if (debug) {
      printf("No floor\n");
    }
    return false;
  }

  Bombchu chu(col, pos, angle);

  // Start frame 3 to simulate instant chu drop
  for (int i = 3; i < 120; i++) {
    if (debug) {
      printf(
          "i=%3i x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x z_raw=%08x "
          "xrot=%04x yrot=%04x zrot=%04x\n",
          i, chu.pos.x, chu.pos.y, chu.pos.z, floatToInt(chu.pos.x),
          floatToInt(chu.pos.y), floatToInt(chu.pos.z), (u16)chu.rot.x,
          (u16)chu.rot.y, (u16)chu.rot.z);
    }

    if (!chu.move()) {
      return false;
    }
  }

  if (debug) {
    printf("x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x z_raw=%08x\n", chu.pos.x,
           chu.pos.y, chu.pos.z, floatToInt(chu.pos.x), floatToInt(chu.pos.y),
           floatToInt(chu.pos.z));
  }

  *result = chu.pos;
  return true;
}

bool testPosition(Collision* col, f32 x, f32 z, u16 angle, bool debug) {
  // Assume y=2

  Vec3f result;
  if (!simulateChu(col, x, z, angle, debug, &result)) {
    return false;
  }
  Vec3s explosionCenter = result.toVec3s();

  Vec3f stances[2] = {
      {-1316.32, 59.35973, 141.602},  // default stance
      {-1318.92, 64.4887, 145.583},   // alternate stance
  };

  Vec3f centerf = Vec3f(explosionCenter);
  bool stanceFound = false;
  int stance;
  int sidehopFrame;

  for (int j = 0; j < 2 && !stanceFound; j++) {
    Vec3f sidehopShieldCornerLow = stances[j];
    Vec3f sidehopShieldCornerHigh = sidehopShieldCornerLow + Vec3f(0, 14.25, 0);

    f32 dhigh = Math3D_Vec3f_DistXYZ(&centerf, &sidehopShieldCornerHigh);
    f32 dlow = Math3D_Vec3f_DistXYZ(&centerf, &sidehopShieldCornerLow);
    for (int i = 7; i <= 9 && !stanceFound; i++) {
      if (dhigh > 8.0f * (i - 1) && dlow < 8.0f * i) {
        stanceFound = true;
        sidehopFrame = i;
        stance = j;
      }
    }
  }

  if (!stanceFound) {
    return false;
  }

  Vec3f megaShieldCorner = {-1308.555, 61.53977, 130.6067};  // for y=2
  Cylinder16 linkCylinder = {12, 32, 5, {-1351, 2, 154}};
  Sphere16 sphere = {explosionCenter, 64};
  f32 overlapSize;
  if (Math3D_SphVsCylOverlap(&sphere, &linkCylinder, &overlapSize) ||
      Math3D_Vec3f_DistXYZ(&centerf, &megaShieldCorner) > 72.0f) {
    return false;
  }

  printf(
      "x=%.9g z=%.9g x_raw=%08x z_raw=%08x chux=%d chuz=%d stance=%d "
      "sidehop=%d\n",
      x, z, floatToInt(x), floatToInt(z), explosionCenter.x, explosionCenter.z,
      stance, sidehopFrame);
  return true;
}

void searchPositions(Collision* col, u16 angle) {
  PosAngleRange range = {
      .angleMin = angle,
      .angleMax = angle,
      .xMin = -1470,
      .xMax = -1380,
      .xStep = 0.05f,
      .zMin = 0,
      .zMax = 90,
      .zStep = 0.05f,
  };
  searchPosAngleRange(range, [&](u16 angle, f32 x, f32 z) {
    return testPosition(col, x, z, angle, false);
  });
}

int main(int argc, char* argv[]) {
  Collision col(&spot08_sceneCollisionHeader_002CE0, PLAYER_AGE_ADULT);
  col.addDynapoly(&gZorasFountainIceRampCol, {0.1f, 0.1f, 0.1f}, {0, 0, 0},
                  {-1200.0f, 0.0f, 0.0f});

  // testPosition(&col, intToFloat(0xc4b2c146), intToFloat(0x4214b9e0), 0x2240,
  //              true);

  searchPositions(&col, 0x2240);
  // searchPositions(&collision, 0x2188);

  return 0;
}

// Collision
BgCamInfo gZorasFountainIceRampColCamDataList[] = {
    {0x0000, 0, NULL},
};

SurfaceType gZorasFountainIceRampColSurfaceType[] = {
    {0x0000A000, 0x000007CC},
    {0x00000000, 0x000007CC},
};

CollisionPoly gZorasFountainIceRampColPolygons[] = {
    {0x0000, 0x0000, 0x0001, 0x0002, 0x2D40, 0x773B, 0xF50F, 0xFE0A},
    {0x0000, 0x0000, 0x0002, 0x0003, 0x30D0, 0x747B, 0xEB33, 0xFD89},
    {0x0000, 0x0004, 0x0005, 0x0006, 0x0000, 0x7FFF, 0x0000, 0xFA72},
    {0x0000, 0x0004, 0x0006, 0x0007, 0x0000, 0x7FFF, 0x0000, 0xFA72},
    {0x0000, 0x0006, 0x0008, 0x0009, 0xEA10, 0x7E1A, 0xFF94, 0xFC0D},
    {0x0000, 0x0006, 0x0009, 0x0007, 0xE7AD, 0x7D9D, 0xFC71, 0xFC36},
    {0x0001, 0x0005, 0x0000, 0x0006, 0xEF60, 0x0ABF, 0x818B, 0xFFA3},
    {0x0001, 0x0000, 0x0003, 0x0006, 0x0F8C, 0x1313, 0x8264, 0xFD0B},
    {0x0000, 0x0003, 0x0008, 0x0006, 0xEF43, 0x507B, 0x9DE4, 0xFCDB},
    {0x0000, 0x0002, 0x000A, 0x0008, 0x035A, 0x7D30, 0xE58B, 0xFC66},
    {0x0000, 0x000A, 0x000B, 0x0008, 0xFA37, 0x7E76, 0xED18, 0xFC3C},
    {0x0000, 0x000B, 0x000C, 0x0008, 0xF1DF, 0x7F14, 0xFA24, 0xFC23},
    {0x0000, 0x000C, 0x000D, 0x0008, 0xF0E9, 0x7EF3, 0x0642, 0xFC31},
    {0x0000, 0x000D, 0x000E, 0x0008, 0xF762, 0x7E01, 0x14C9, 0xFC6D},
    {0x0001, 0x000F, 0x0010, 0x000E, 0xA6DD, 0xE6B7, 0x584F, 0xF8E5},
    {0x0001, 0x0010, 0x0009, 0x000E, 0xD386, 0x22D9, 0x72D9, 0xF8F6},
    {0x0001, 0x0011, 0x000F, 0x000E, 0x9F68, 0xE2E8, 0x4EC8, 0xF8F3},
    {0x0001, 0x0011, 0x000E, 0x000D, 0xD267, 0xD0F6, 0x6DF6, 0xFAA4},
    {0x0001, 0x0011, 0x000D, 0x000C, 0x909F, 0xD50E, 0x2E30, 0xF680},
    {0x0001, 0x0011, 0x000C, 0x000B, 0x909F, 0xD50E, 0xD1D0, 0xF560},
    {0x0001, 0x0011, 0x000B, 0x000A, 0xC1C7, 0xCFB7, 0x9B1B, 0xF704},
    {0x0001, 0x000A, 0x0012, 0x0011, 0x9DE6, 0xD9A0, 0xB74B, 0xF634},
    {0x0001, 0x0012, 0x000A, 0x0013, 0xBDE3, 0xE75C, 0x9535, 0xF5FF},
    {0x0001, 0x000A, 0x0002, 0x0013, 0xFB7A, 0x2D05, 0x8845, 0xF7E5},
    {0x0001, 0x0007, 0x0009, 0x0014, 0x12DC, 0x36D0, 0x721E, 0xF8B0},
    {0x0001, 0x0009, 0x0010, 0x0014, 0x0000, 0x1C47, 0x7CD5, 0xF8E0},
    {0x0001, 0x0010, 0x000F, 0x0014, 0x0000, 0xEDF1, 0x7EB7, 0xF7F3},
    {0x0001, 0x000F, 0x0015, 0x0014, 0x2D94, 0xCADA, 0x6B26, 0xF562},
    {0x0001, 0x0004, 0x0014, 0x0016, 0x30A8, 0x2071, 0x71DB, 0xF641},
    {0x0001, 0x0004, 0x0007, 0x0014, 0x158E, 0x36D5, 0x71A1, 0xF888},
    {0x0001, 0x0014, 0x0015, 0x0016, 0x2DC3, 0xCAE1, 0x6B16, 0xF560},
    {0x0000, 0x0001, 0x0017, 0x0002, 0x19B1, 0x5110, 0xA055, 0xF936},
    {0x0001, 0x0017, 0x0013, 0x0002, 0xF81F, 0x2E08, 0x88D4, 0xF7DD},
    {0x0001, 0x0001, 0x0018, 0x0017, 0x210D, 0xF406, 0x84ED, 0xF53D},
    {0x0001, 0x0018, 0x0012, 0x0017, 0x0409, 0xE649, 0x82AE, 0xF5C2},
    {0x0001, 0x0012, 0x0013, 0x0017, 0xF7A7, 0xECD1, 0x81BA, 0xF632},
    {0x0001, 0x0018, 0x0001, 0x0000, 0x7A7F, 0xE9A1, 0xE260, 0xF4AE},
    {0x0001, 0x0019, 0x0004, 0x0016, 0x78DA, 0xE0B3, 0x1C41, 0xF246},
    {0x0001, 0x0019, 0x0005, 0x0004, 0x7E5E, 0xEBA6, 0x0000, 0xF1C3},
    {0x0001, 0x0019, 0x0000, 0x0005, 0x37A4, 0xE1EF, 0x90B8, 0xF8FC},
    {0x0001, 0x0015, 0x0019, 0x0016, 0x736A, 0xCFAE, 0x1AFB, 0xF292},
    {0x0001, 0x0015, 0x0000, 0x0019, 0x3450, 0xCC0B, 0x9760, 0xF8FE},
    {0x0001, 0x0000, 0x0015, 0x0018, 0x7AE8, 0xE10B, 0xEE24, 0xF4AE},
    {0x0000, 0x0008, 0x0003, 0x0002, 0x30FE, 0x7463, 0xEB1C, 0xFD88},
    {0x0000, 0x0008, 0x000E, 0x0009, 0xDE72, 0x7B7E, 0x029B, 0xFBEC},
    {0x0001, 0x0011, 0x0012, 0x000F, 0xD8B1, 0x8645, 0x045E, 0xF328},
    {0x0001, 0x0012, 0x0015, 0x000F, 0x0000, 0x8001, 0x0000, 0xF2CE},
    {0x0001, 0x0012, 0x0018, 0x0015, 0x1EC4, 0x87FA, 0xDFE5, 0xF21B},
};

Vec3s gZorasFountainIceRampColVertices[] = {
    {2800, -571, -511},   {2400, -571, -2165}, {-469, 533, -2000},
    {180, 498, -673},     {3921, 1422, 724},   {3921, 1422, -489},
    {2278, 1422, -273},   {2081, 1422, 1073},  {-615, 919, -192},
    {-156, 1005, 1643},   {-2085, 594, -1916}, {-2731, 622, -1531},
    {-3200, 622, -400},   {-2731, 622, 731},   {-1600, 622, 1200},
    {-69, -3378, 1600},   {-469, -571, 2000},  {-1852, -2874, -400},
    {-469, -3378, -2000}, {-869, -571, -2400}, {2264, -571, 2000},
    {2256, -3378, 611},   {3200, -571, 1600},  {1052, -571, -2527},
    {2034, -2722, -2054}, {3600, -571, -111},
};

CollisionHeader gZorasFountainIceRampCol = {
    {-3200, -3378, -2527},
    {3921, 1422, 2000},
    26,
    gZorasFountainIceRampColVertices,
    48,
    gZorasFountainIceRampColPolygons,
    gZorasFountainIceRampColSurfaceType,
    gZorasFountainIceRampColCamDataList,
    0,
    NULL,
};
