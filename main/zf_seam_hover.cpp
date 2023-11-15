#include "actor.hpp"
#include "collision_data.hpp"

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

    if (pos.y <= floorHeight) {
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
  col.printPolys();

  // printSeamHeights(&col);

  // printSeamJumps(&col, 8.5f, 3.5f);  // sidehop
  // printSeamJumps(&col, 6.0f, 5.8f);  // backflip

  return 0;
}
