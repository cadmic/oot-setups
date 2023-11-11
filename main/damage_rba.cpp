#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "bombchu.hpp"
#include "collision.hpp"
#include "collision_data.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"

// TODO: put somewhere common and use everywhere
#define dprintf(debug, ...) \
  do {                      \
    if (debug) {            \
      printf(__VA_ARGS__);  \
    }                       \
  } while (0)

// TODO: move somewhere common?
bool testSphVsQuad(Sphere16* sph, Vec3f* quad) {
  TriNorm tri1;
  TriNorm tri2;
  Vec3f hitPos;

  Math3D_TriNorm(&tri1, &quad[2], &quad[3], &quad[1]);
  Math3D_TriNorm(&tri2, &quad[1], &quad[0], &quad[2]);
  return Math3D_TriVsSphIntersect(sph, &tri1, &hitPos) ||
         Math3D_TriVsSphIntersect(sph, &tri2, &hitPos);
}

// TODO: colliders?
bool testDamageRba(Collision* col, Cylinder16 object, Vec3f pos, u16 angle,
                   int swordFrame, bool debug) {
  // Explode chu
  Bombchu chu(col, pos, angle);
  bool chuExploded = false;
  for (int i = 0; i < 8; i++) {
    Sphere16 chuSph = {chu.pos.toVec3s(), 12};

    f32 overlapSize;
    if (Math3D_SphVsCylOverlap(&chuSph, &object, &overlapSize)) {
      chuExploded = true;
      break;
    }

    if (!chu.move()) {
      // TODO: allow this?
      dprintf(debug, "chu ran out of floor\n");
      return false;
    }

    dprintf(debug, "chu: i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", i,
            chu.pos.x, floatToInt(chu.pos.x), chu.pos.y, floatToInt(chu.pos.y),
            chu.pos.z, floatToInt(chu.pos.z));
  }

  if (!chuExploded) {
    dprintf(debug, "chu did not explode\n");
    return false;
  }

  Vec3s explosionPos = chu.pos.toVec3s();
  int f = 0;
  dprintf(debug, "explosionPos: x=%d y=%d z=%d\n", explosionPos.x,
          explosionPos.y, explosionPos.z);

  // Take damage
  bool tookDamage = false;
  for (; f <= 10; f++) {
    // In reality Link's cylinder height/yoffset can vary but hopefully it
    // doesn't matter too much
    Cylinder16 link = {12, 50, 0, pos.toVec3s()};
    Sphere16 explosion = {explosionPos, (s16)(f * 8)};

    f32 overlapSize;
    if (Math3D_SphVsCylOverlap(&explosion, &link, &overlapSize)) {
      tookDamage = true;
      break;
    }
  }

  if (!tookDamage) {
    dprintf(debug, "did not take damage\n");
    return false;
  }

  u16 movementAngle = Math_Vec3f_Yaw(&pos, &chu.pos) + 0x8000;
  pos = col->runChecks(pos, translate(pos, movementAngle, 15.0f, -5.0f));
  f32 speed = 7.0f;
  f += 2;

  if (swordFrame < f) {
    dprintf(debug, "sword press happened too early\n");
    return false;
  }

  // Take knockback
  int knockbackState = 0;
  for (; f < 10; f++) {
    Sphere16 explosion = {explosionPos, (s16)(f * 8)};

    AnimFrame animFrame;
    loadAnimFrame(gPlayerAnim_link_normal_front_hit_Data, 2, &animFrame);
    if (f == swordFrame + 1) {
      loadUpperBodyAnimFrame(gPlayerAnim_link_normal_fighter2free_Data, 15,
                             &animFrame);
    } else if (knockbackState > 0) {
      loadUpperBodyAnimFrame(gPlayerAnim_link_anchor_defense_hit_Data,
                             knockbackState * 3 - 2, &animFrame);
    } else {
      loadUpperBodyAnimFrame(gPlayerAnim_link_anchor_waitR2defense_Data, 2,
                             &animFrame);
    }

    Vec3f shieldCorners[4];
    getShieldPosition(&animFrame, PLAYER_AGE_ADULT, pos, angle, shieldCorners);
    bool hitShield = testSphVsQuad(&explosion, shieldCorners);

    dprintf(debug, "f=%d x=%.9g y=%.9g z=%.9g speed=%.0f knockbackState=%d\n",
            f, pos.x, pos.y, pos.z, speed, knockbackState);
    dprintf(debug, " c[0]: x=%.9g y=%.9g z=%.9g\n", shieldCorners[0].x,
            shieldCorners[0].y, shieldCorners[0].z);
    dprintf(debug, " c[1]: x=%.9g y=%.9g z=%.9g\n", shieldCorners[1].x,
            shieldCorners[1].y, shieldCorners[1].z);
    dprintf(debug, " c[2]: x=%.9g y=%.9g z=%.9g\n", shieldCorners[2].x,
            shieldCorners[2].y, shieldCorners[2].z);
    dprintf(debug, " c[3]: x=%.9g y=%.9g z=%.9g\n", shieldCorners[3].x,
            shieldCorners[3].y, shieldCorners[3].z);

    if (f == swordFrame) {
      if (knockbackState == 0) {
        dprintf(debug, "not in knockback state during sword press\n");
        return false;
      }

      if (hitShield) {
        dprintf(debug, "not in knockback state during sword press\n");
        return false;
      }
    } else if (f == swordFrame + 1) {
      if (!hitShield) {
        dprintf(debug, "not in knockback state after sword press\n");
        return false;
      }

      return true;
    }

    CollisionPoly* wallPoly;
    CollisionPoly* floorPoly;
    int dynaId;
    f32 floorHeight;
    pos = col->runChecks(pos, translate(pos, movementAngle, speed, -5.0f),
                         &wallPoly, &floorPoly, &dynaId, &floorHeight);
    if (pos.y > floorHeight) {
      dprintf(debug, "fell off ground\n");
      return false;
    }

    if (hitShield) {
      knockbackState = 1;
      movementAngle = angle;
      speed = -18.0f;
    } else if (knockbackState > 0) {
      knockbackState++;
      if (knockbackState > 3) {
        dprintf(debug, "knockback ended\n");
        return false;
      }
    }

    Math_StepToF(&speed, 0.0f, 8.0f);
  }

  dprintf(debug, "explosion ended\n");
  return false;
}

void findSwordFrames(Collision* col, Cylinder16 object, Vec3f pos, u16 angle,
                     std::vector<int>* swordFrames) {
  swordFrames->clear();
  for (int swordFrame = 3; swordFrame <= 9; swordFrame++) {
    if (testDamageRba(col, object, pos, angle, swordFrame, false)) {
      swordFrames->push_back(swordFrame);
    }
  }
}

void printSwordFrames(Vec3f pos, u16 angle,
                      const std::vector<int>& swordFrames) {
  printf(
      "angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x "
      "numSwordFrames=%d "
      "swordFrames=",
      angle, pos.x, pos.z, floatToInt(pos.x), floatToInt(pos.z),
      (int)swordFrames.size());
  for (int swordFrame : swordFrames) {
    printf("%d,", swordFrame);
  }
  printf("\n");
}

Cylinder16 shadowTorch = {12, 45, 0, {2343, 95, -39}};

void findOutsideShadowPositions(Collision* col) {
  std::vector<int> swordFrames;
  swordFrames.reserve(10);

  int tested = 0;
  int found = 0;
  for (u16 angle = 0xc000; angle <= 0xc000; angle += 0x10) {
    for (f32 x = 2360.0f; x <= 2410.0f; x += 0.1f) {
      for (f32 z = -70.0f; z <= -30.0f; z += 0.1f) {
        if (tested % 1000 == 0) {
          fprintf(stderr, "tested=%d found=%d angle=%04x x=%.2f z=%.2f ...\r",
                  tested, found, angle, x, z);
        }
        tested++;

        Vec3f pos = {x, 100, z};
        findSwordFrames(col, shadowTorch, pos, angle, &swordFrames);
        if (!swordFrames.empty()) {
          found++;
          printSwordFrames(pos, angle, swordFrames);
        }
      }
    }
  }
}

void findMidoPositions(Collision* col) {
  std::vector<int> swordFrames;
  swordFrames.reserve(10);

  u16 angle = 0x4000;

  int tested = 0;
  int found = 0;
  for (f32 x = 1418.0f; x <= 1518.0f; x += 0.1f) {
    for (f32 z = -982.0f; z <= -882.0f; z += 0.1f) {
      if (tested % 1000 == 0) {
        fprintf(stderr, "tested=%d found=%d angle=%04x x=%.2f z=%.2f ...\r",
                tested, found, angle, x, z);
      }
      tested++;

      Vec3f pos = {x, 0, z};

      Vec3f midoHome = {1599, 0, -980};
      u16 midoYaw = Math_Vec3f_Yaw(&midoHome, &pos);
      Vec3f midoPos = midoHome + Vec3f(60.0f * Math_SinS(midoYaw), 0.0f,
                                       60.0f * Math_CosS(midoYaw));
      Cylinder16 mido = {36, 46, 0, midoPos.toVec3s()};

      Cylinder16 link = {12, 50, 0, pos.toVec3s()};
      f32 overlapSize;
      if (Math3D_CylVsCylOverlap(&link, &mido, &overlapSize)) {
        continue;
      }

      findSwordFrames(col, mido, pos, angle, &swordFrames);
      if (!swordFrames.empty()) {
        found++;
        printSwordFrames(pos, angle, swordFrames);
      }
    }
  }
}

int main(int argc, char* argv[]) {
  // Collision col(&spot02_sceneCollisionHeader_003C54, PLAYER_AGE_ADULT,
  //               {2456, 100, -40}, {2456, 100, -40});
  // testDamageRba(&col, shadowTorch,
  //               {intToFloat(0x4515ac20), 100, intToFloat(0xc242f00e)},
  //               0xc000, 7, true);
  // findOutsideShadowPositions(&col);

  Collision col(&spot10_sceneCollisionHeader_00AC98, PLAYER_AGE_ADULT,
                {1400, 0, -1000}, {1400, 0, -850});
  findMidoPositions(&col);

  return 0;
}
