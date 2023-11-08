#include "actor.hpp"
#include "collision_data.hpp"

CamData gIngoGateColCamDataList[] = {
    {0x0000, 0, NULL},
};

SurfaceType gIngoGateColSurfaceType[] = {
    {0x00200000, 0x000007C2},
    {0x00000000, 0x000007C2},
};

CollisionPoly gIngoGateColPolygons[] = {
    {0x0000, 0xA000, 0x0001, 0x0002, 0x8001, 0x0000, 0x0000, 0x0000},
    {0x0000, 0xA000, 0x0002, 0x0003, 0x8001, 0x0000, 0x0000, 0x0000},
    {0x0000, 0xA004, 0x0005, 0x0006, 0x7FFF, 0x0000, 0x0000, 0xFCE0},
    {0x0000, 0xA004, 0x0006, 0x0007, 0x7FFF, 0x0000, 0x0000, 0xFCE0},
    {0x0001, 0xA007, 0x0006, 0x0003, 0x0000, 0x7FFF, 0x0000, 0xFB50},
    {0x0001, 0xA007, 0x0003, 0x0002, 0x0000, 0x7FFF, 0x0000, 0xFB50},
    {0x0000, 0xA003, 0x0006, 0x0005, 0x0000, 0x0000, 0x8001, 0xFF9C},
    {0x0000, 0xA003, 0x0005, 0x0000, 0x0000, 0x0000, 0x8001, 0xFF9C},
    {0x0000, 0xA001, 0x0004, 0x0007, 0x0000, 0x0000, 0x7FFF, 0xFF9C},
    {0x0000, 0xA001, 0x0007, 0x0002, 0x0000, 0x0000, 0x7FFF, 0xFF9C},
};

Vec3s gIngoGateColVertices[] = {
    {0, 0, -100},  {0, 0, 100},    {0, 1200, 100},    {0, 1200, -100},
    {800, 0, 100}, {800, 0, -100}, {800, 1200, -100}, {800, 1200, 100},
};

CollisionHeader gIngoGateCol = {
    {0, 0, -100},
    {800, 1200, 100},
    8,
    gIngoGateColVertices,
    10,
    gIngoGateColPolygons,
    gIngoGateColSurfaceType,
    gIngoGateColCamDataList,
    0,
    NULL,
};

bool testSidehop(Collision* col, u16 angle, bool debug) {
  Vec3f pos = {818, 0, -2518};
  f32 ySpeed = 3.5f;
  for (int i = 0; i < 6; i++) {
    ySpeed -= 1.0f;
    pos = col->runChecks(pos, translate(pos, angle, 8.5f, ySpeed));

    if (debug) {
      printf("i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", i, pos.x,
             floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
             floatToInt(pos.z));
    }
  }

  ySpeed -= 1.0f;
  pos = col->runChecks(pos, translate(pos, angle, 7.5f, ySpeed));

  if (debug) {
    printf("x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  return pos.x < 796;
}

void findSidehopAngles(Collision* col) {
  for (u16 angle = 0xc800; angle <= 0xf800; angle += 0x10) {
    if (testSidehop(col, angle, false)) {
      printf("angle=%04x\n", angle);
    }
  }
}

int main(int argc, char* argv[]) {
  Collision col(&spot20_sceneCollisionHeader_002948, PLAYER_AGE_ADULT,
                {800, 0, -2880}, {800, 0, -2880});
  col.addDynapoly(&gIngoGateCol, {0.1f, 0.1f, 0.1f}, {0, 0x4000, 0},
                  {820, -40, -2420});

  // col.printPolys();

  // testSidehop(&col, 0xdf00, true);
  findSidehopAngles(&col);
}
