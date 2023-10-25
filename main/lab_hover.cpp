#include "actor.hpp"
#include "collision_data.hpp"

f32 popupHeight(Collision* col, Vec3f pos) {
  Vec3f resultPos = col->runChecks(pos, pos);
  return resultPos.y - pos.y;
}

void printPopupRegion(Collision* col) {
  int tested = 0;
  int found = 0;

  for (f32 y = -930; y <= -902; y += 0.1f) {
    for (f32 x = -2410; x <= -2400; x += 0.1f) {
      for (f32 z = 4032; z <= 4042; z += 0.1f) {
        if (tested % 100000 == 0) {
          fprintf(stderr, "tested=%d found=%d y=%.1f x=%.1f z=%.1f ...\r",
                  tested, found, y, x, z);
        }
        tested++;

        Vec3f pos = {x, y, z};
        f32 h = popupHeight(col, pos);
        if (h > 0) {
          found++;
          printf(
              "h=%.9g x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x z_raw=%08x\n",
              h, x, y, z, floatToInt(x), floatToInt(y), floatToInt(z));
        }
      }
    }
  }
}

Vec3f backflip(Collision* col, Vec3f pos, u16 angle, int frames, bool debug) {
  f32 ySpeed = 5.8f;
  for (int i = 1; i < frames; i++) {
    if (debug) {
      printf("i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n", i,
             pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
             floatToInt(pos.z), ySpeed);
    }
    ySpeed -= 1.0f;
    pos = col->runChecks(pos, translate(pos, angle, 6.0f, ySpeed));
  }

  if (debug) {
    printf("i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", frames, pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  return pos;
}

bool jumpToHouse(Collision* col, Vec3f pos, u16 angle, f32 xzSpeed, f32 ySpeed,
                 bool debug) {
  for (int i = 1;; i++) {
    if (debug) {
      printf("i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n", i,
             pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
             floatToInt(pos.z), ySpeed);
    }
    ySpeed -= 1.0f;
    pos = col->runChecks(pos, translate(pos, angle, xzSpeed, ySpeed));
    if (pos.y > -883.0f) {
      return true;
    }
    if (pos.y < -950.0f) {
      return false;
    }
  }

  return false;
}

bool test4ChuHover(Collision* col, Vec3f pos, u16 angle, bool debug) {
  pos = backflip(col, pos, angle + 0xa890, 10, debug);
  pos = backflip(col, pos, angle + 0x1770, 8, debug);
  pos = backflip(col, pos, angle + 0x1770, 8, debug);
  pos = backflip(col, pos, angle + 0x1770, 8, debug);
  return jumpToHouse(col, pos, angle + 0xc000, 8.5f, 3.5f, debug);
}

void find4ChuHovers(Collision* col) {
  int tested = 0;
  int found = 0;

  for (u16 angle = 0xc180; angle <= 0xc280; angle += 0x10) {
    for (f32 x = -2410; x <= -2324; x += 5.0f) {
      for (f32 z = 3976; z <= 3998; z += 0.001f) {
        if (tested % 10000 == 0) {
          fprintf(stderr, "tested=%d found=%d angle=%04x x=%.1f z=%.1f ...\r",
                  tested, found, angle, x, z);
        }
        tested++;

        Vec3f pos = {x, -993, z};
        if (test4ChuHover(col, pos, angle, false)) {
          found++;
          printf("angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x\n", angle, x,
                 z, floatToInt(x), floatToInt(z));
          fflush(stdout);
        }
      }
    }
  }
}

int main(int argc, char* argv[]) {
  Collision col(&spot06_sceneCollisionHeader_0055AC, PLAYER_AGE_ADULT,
                {-2405, -873, 4037}, {-2405, -873, 4037});

  // testPopup(&col, {-2405, -922.5f, 4037});
  // printPopupRegion(&col);

  // backflip(&col, {-2324.62f, -993, 3997.51f}, 0x6890, 10, true);
  find4ChuHovers(&col);

  return 0;
}
