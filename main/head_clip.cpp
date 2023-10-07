#include <iostream>

#include "collision.hpp"
#include "global.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"

// Spirit temple head clip

// floor polygon data
bool startFloorHeight(f32 x, f32 z, f32* yIntersect) {
  Vec3f v0 = {137, 1038, -1630};
  Vec3f v1 = {112, 1021, -1604};
  Vec3f v2 = {175, 995, -1594};
  return Math3D_TriChkPointParaYIntersectInsideTri(
      &v0, &v1, &v2, 0x18DE * (1.0f / 32767.0f), 0x5CCB * (1.0f / 32767.0f),
      0x5495 * (1.0f / 32767.0f), 0x012A, z, x, yIntersect, 1.0f);
}

bool lowerFloorHeight(f32 x, f32 z, f32* yIntersect) {
  Vec3f v0 = {169, 960, -1630};
  Vec3f v1 = {104, 979, -1630};
  Vec3f v2 = {150, 940, -1557};
  return Math3D_TriChkPointParaYIntersectInsideTri(
      &v0, &v1, &v2, 0x220B * (1.0f / 32767.0f), 0x7475 * (1.0f / 32767.0f),
      0x28C4 * (1.0f / 32767.0f), 0xFE71, z, x, yIntersect, 1.0f);
}

bool simulate(f32 x, f32 y, f32 z, u16 sidehopAngle, u16 jumpslashAngle,
              int jumpslashFrame, bool debug) {
  f32 prevY = y;
  u16 angle = sidehopAngle;
  f32 hSpeed = 8.5f;
  f32 vSpeed = 3.5f;
  bool isJumpslashing = false;

  for (int i = 0;; i++) {
    if (debug) {
      printf(
          "i=%i x=%.7f (%08x) y=%.7f (%08x) z=%.7f (%08x) "
          "angle=%04x hSpeed=%g "
          "vSpeed=%g\n",
          i, x, floatToInt(x), y, floatToInt(y), z, floatToInt(z), angle,
          hSpeed, vSpeed);
    }

    if (x <= 124 && y >= 981 && z >= -1808) {
      // collides with wall
      return false;
    }

    if (x <= 121 && prevY >= 957) {
      // lands on floor (maybe)
      if (z <= -1804) {
        if (debug) {
          printf("success!\n");
        }
        return true;
      } else {
        return false;
      }
    }

    if (y < 940) {
      // will void out
      return false;
    }

    f32 dx = (Math_SinS(angle) * hSpeed) * 1.5f;
    f32 dz = (Math_CosS(angle) * hSpeed) * 1.5f;
    if (i == 0) {
      // Need to make sure we sidehop over a non-sticky floor
      f32 yIntersect;
      if (!lowerFloorHeight(x + dx, z + dz, &yIntersect)) {
        return false;
      }
    } else {
      x += dx;
      z += dz;
    }

    if (isJumpslashing && vSpeed < 4.5f) {
      vSpeed -= 1.2f;
    } else {
      vSpeed -= 1.0f;
    }

    prevY = y;
    y += vSpeed * 1.5f;

    if (isJumpslashing) {
      hSpeed += 0.05f;
    }

    // start jumpslash
    if (i == jumpslashFrame) {
      angle = jumpslashAngle;
      hSpeed = 3.0f;
      vSpeed = 4.5f;
      isJumpslashing = true;
    }
  }

  return false;
}

bool findAngleRange(f32 x, f32 y, f32 z, u16* minAngle, u16* maxAngle) {
  // Perfect diagonal
  // u16 jumpslashOffset = 0x892;
  // u16 startAngle = 0x8800;
  // u16 endAngle = 0xa000;

  // Straight left
  u16 jumpslashOffset = 0x1770;
  u16 startAngle = 0x9300;
  u16 endAngle = 0x9600;

  // Down 3 frames
  // u16 jumpslashOffset = 0x111A;
  // u16 startAngle = 0x8800;
  // u16 endAngle = 0xa000;

  bool found = false;

  // TODO: narrow bounds
  for (u16 jumpslashAngle = startAngle; jumpslashAngle < endAngle;
       jumpslashAngle += 4) {
    // TODO: more accurate sidehop angle?
    u16 sidehopAngle = jumpslashAngle - jumpslashOffset;
    int jumpslashFrame = 12;
    if (simulate(x, y, z, sidehopAngle, jumpslashAngle, jumpslashFrame,
                 false)) {
      if (!found) {
        found = true;
        *minAngle = jumpslashAngle;
      }
      *maxAngle = jumpslashAngle;
    }
  }

  return found;
}

void findAngleRanges() {
  u16 minAngle, maxAngle;
  // TODO: expand search out? not sure how much the wall matters
  for (f32 x = 111.0f; x <= 132.0f; x += 0.1f) {
    for (f32 z = -1631.0f; z <= -1593.0f; z += 0.01f) {
      f32 y;
      if (!startFloorHeight(x, z, &y)) {
        continue;
      }

      if (findAngleRange(x, y, z, &minAngle, &maxAngle)) {
        printf(
            "x=%.7f (%08x) y=%.7f (%08x) z=%.7f (%08x) "
            "minAngle=%04x "
            "maxAngle=%04x range=%04x\n",
            x, floatToInt(x), y, floatToInt(y), z, floatToInt(z), minAngle,
            maxAngle, maxAngle - minAngle);
      }
    }
  }
}

void findPositions() {
  for (f32 x = 111.0f; x <= 132.0f; x += 0.1f) {
    for (f32 z = -1631.0f; z <= -1593.0f; z += 0.01f) {
      f32 y;
      if (!startFloorHeight(x, z, &y)) {
        continue;
      }

      // 1 ess right, turn down
      // u16 sidehopAngle = 0x7c76;
      // u16 jumpslashAngle = 0x93e5;

      // target face, 5 ess left, turn down
      u16 jumpslashAngle =
          (u16)Math_Atan2S(-1439.0f - z, 59.0f - x) + 0x708 * 5 + 0x8000;
      u16 sidehopAngle = jumpslashAngle - 0x1770;

      int jumpslashFrame = 12;
      if (simulate(x, y, z, sidehopAngle, jumpslashAngle, jumpslashFrame,
                   false)) {
        printf("x=%.7f (%08x) y=%.7f (%08x) z=%.7f (%08x) angle=%04x\n", x,
               floatToInt(x), y, floatToInt(y), z, floatToInt(z),
               jumpslashAngle);
      }
    }
  }
}

int main(int argc, char* argv[]) {
  // CollisionHeader* col = &jyasinzou_sceneCollisionHeader_01680C;
  // printCollisionObj(col);

  // Danny's setup
  // simulate(intToFloat(0x42fdd6b1), intToFloat(0x4480b4c3),
  //          intToFloat(0xc4ca3bfd), 0x7de5, 0x8eff, 12, true);

  // simulate(intToFloat(0x42fd9563), intToFloat(0x4480baee),
  //          intToFloat(0xc4ca418f), 0x7cf6, 0x9465, 12, true);

  findPositions();

  return 0;
}
