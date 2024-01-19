#include "actor.hpp"
#include "collision.hpp"
#include "collision_data.hpp"

bool testMegaflip(Collision* col, Vec3f pos, u16 angle, bool debug) {
  pos = translate(pos, angle, 2.0f, 0.0f);
  for (int i = 0; i < 10; i++) {
    pos = translate(pos, angle, 3.0f, 0.0f);
  }

  f32 ySpeed = 4.8f;
  pos = translate(pos, angle + 0x8000, 6.0f, ySpeed);

  for (int i = 0; i < 20; i++) {
    if (debug) {
      printf("i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n", i,
             pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
             floatToInt(pos.z), ySpeed);
    }

    ySpeed -= 1.0f;
    pos = translate(pos, angle, -18.0f, ySpeed);
  }

  if (debug) {
    printf("x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z), ySpeed);
  }

  ySpeed -= 1.0f;
  pos = col->runChecks(pos, translate(pos, 0xc000, -18.0f, ySpeed));

  if (debug) {
    printf("x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z), ySpeed);
  }

  ySpeed -= 1.0f;
  pos = col->runChecks(pos, translate(pos, 0xc000, -17.0f, ySpeed));

  if (debug) {
    printf("x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z), ySpeed);
  }
  return (pos.x > 600.0f);
}

void findMegaflips(Collision* col) {
  for (u32 xi = floatToInt(70.0f); xi <= floatToInt(75.0f); xi++) {
    Vec3f pos = {intToFloat(xi), 0.0f, -1010.0f};

    if (testMegaflip(col, pos, 0xc000, false)) {
      printf("x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", pos.x,
             floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
             floatToInt(pos.z));
    }
  }
}

int main(int argc, char* argv[]) {
  Collision col(&ddan_sceneCollisionHeader_011D40, PLAYER_AGE_CHILD);

  std::vector<u32> polys = {
      0x80370ee8,  // sloped floor
      0x80375058,  // wall
  };

  for (u32 poly : polys) {
    col.addPoly((poly - 0x803704B8) / 0x10);
  }

  // col.printPolys();

  findMegaflips(&col);

  // testMegaflip(&col, {intToFloat(0x428d74e0), 0, -1010.0f}, 0xc000, true);

  return 0;
}
