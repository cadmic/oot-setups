#include "collision.hpp"
#include "collision_data.hpp"
#include "control_stick.hpp"
#include "global.hpp"
#include "sys_math.hpp"

f32 simulateFwess(Collision* col, Vec3f pos, u16 angle, u16 storedPushedYaw,
                  bool debug) {
  f32 vx = 0.0f;
  f32 vy = 3.5f;
  f32 vz = 0.0f;
  f32 gravity = -1.0f;
  f32 moveSpeed = 8.5f;
  u16 moveYaw = angle - 0x4000;
  f32 pushedSpeed = 0.0f;
  u16 pushedYaw = storedPushedYaw;
  bool onGround = false;     // BGCHECKFLAG_GROUND
  bool isSwimming = false;   // PLAYER_STATE1_27
  bool isSurfacing = false;  // actionFunc == Player_Action_8084E1EC

  for (int f = 1; f < 30; f++) {
    if (debug) {
      printf(
          "f=%2d x=%11.9g y=%11.9g z=%11.9g vx=%11.9g vy=%6.2f vz=%11.9g "
          "gravity=%2.0f moveSpeed=%3.1f moveYaw=%04x pushedSpeed=%4.2f "
          "pushedYaw=%04x onGround=%d isSwimming=%d isSurfacing=%d\n",
          f, pos.x, pos.y, pos.z, vx, vy, vz, gravity, moveSpeed, moveYaw,
          pushedSpeed, pushedYaw, onGround, isSwimming, isSurfacing);
    }

    vx = Math_SinS(moveYaw) * moveSpeed + Math_SinS(pushedYaw) * pushedSpeed;
    vy = std::max(vy + gravity, -20.0f);
    vz = Math_CosS(moveYaw) * moveSpeed + Math_CosS(pushedYaw) * pushedSpeed;

    CollisionPoly* wallPoly;
    CollisionPoly* floorPoly;
    int dynaId;
    f32 floorHeight;
    pos = col->runChecks(pos, pos + Vec3f(vx, vy, vz) * 1.5f, &wallPoly,
                         &floorPoly, &dynaId, &floorHeight);

    if (pos.y <= floorHeight) {
      if (!onGround) {
        onGround = true;
      } else if (vy <= 0.0f) {
        vy = gravity < 0.0f ? -4.0f : 0.0f;
      }
    } else {
      onGround = false;
    }

    f32 yDistToWater = -60.0f - pos.y;
    if (yDistToWater > 20.0f) {
      f32 conveyorSpeed = isSwimming ? 2.0f : 0.5f;
      u16 conveyorYaw = 0xb000;
      Math_StepToF(&pushedSpeed, conveyorSpeed, conveyorSpeed * 0.1f);
      Math_ScaledStepToS(&pushedYaw, conveyorYaw,
                         (isSwimming ? 400.0f : 800.0f) * conveyorSpeed);
    } else {
      Math_StepToF(&pushedSpeed, 0.0f, isSwimming ? 0.5f : 1.0f);
    }

    if (yDistToWater > 32.0f) {
      isSwimming = true;
    } else if (isSwimming && yDistToWater < 22.0f) {
      if (debug) {
        printf("fwess with moveSpeed=%.1f\n", moveSpeed);
      }
      return moveSpeed;
    }

    if (isSurfacing) {
      if (debug) {
        printf("slow fwess: surfacing\n");
      }
      // TODO: calculate slow fwess speed?
      return 0.0f;
    }

    // Player_Action_8084D610
    if (isSwimming) {
      // func_8084B000
      f32 targetDist = vy < 0.0f ? 30.6f : 29.6f;
      if (yDistToWater < targetDist) {
        f32 delta = -0.1f - std::max(vy * 0.5f, 0.0f);
        vy = std::max(vy + delta, -5.0f);
      } else {
        f32 delta = 0.1f + std::max(vy * -0.3f, 0.0f);
        vy = std::min(vy + delta, 2.0f);
      }
      gravity = 0.0f;

      // func_8083D12C
      if (vy > 0.0f && yDistToWater < 48.0f) {
        isSurfacing = true;
      } else {
        // assume holding back on control stick
        Math_StepToF(&moveSpeed, 0.0f, 1.0f);
      }
    }
  }

  return 0.0f;
}

void findFwesses(Collision* col) {
  u16 pushedYaw = 0x0000;
  for (u16 angle = 0x9c00; angle <= 0xb000; angle += 0x10) {
    fprintf(stderr, "angle=%04x ...\r", angle);
    for (f32 x = 3130.0f; x < 3530.0f; x += 0.1f) {
      // clean up graph a little
      if (x < 3140.0f && angle >= 0xab00) {
        continue;
      }

      // Approximate fence wall
      f32 nx = (s16)0xd462 * (1.0f / SHT_MAX);
      f32 nz = (s16)0x7856 * (1.0f / SHT_MAX);
      f32 z = (629.0f - x * nx) / nz;

      Vec3f pos = col->findFloor({x, 200.0f, z});
      if (pos.y == BGCHECK_Y_MIN) {
        continue;
      }

      f32 fwessSpeed = simulateFwess(col, pos, angle, pushedYaw, false);
      if (fwessSpeed > 4.0f) {
        printf(
            "angle=%04x x=%.9g y=%.9g z=%.9g x_raw=%08x y_raw=%08x "
            "z_raw=%08x pushedYaw=%04x speed=%.1f\n",
            angle, pos.x, pos.y, pos.z, floatToInt(pos.x), floatToInt(pos.y),
            floatToInt(pos.z), pushedYaw, fwessSpeed);
      }
    }
  }
}

f32 slopeFloorPitch(u16 angle) {
  f32 nx = (s16)0xe995 * (1.0f / SHT_MAX);
  f32 ny = (s16)0x5551 * (1.0f / SHT_MAX);
  f32 nz = (s16)0x5cbe * (1.0f / SHT_MAX);

  f32 sin = Math_SinS(angle);
  f32 cos = Math_CosS(angle);

  return Math_Atan2S(1.0f, (-(nx * sin) - (nz * cos)) * (1.0f / ny));
}

void printControlStick(u16 angle) {
  f32 sidehopFloorPitch = slopeFloorPitch(angle - 0x4000);
  f32 essFloorPitch = slopeFloorPitch(angle);

  for (int x = -67; x <= 67; x++) {
    for (int y = -67; y <= 67; y++) {
      u16 angle = controlStickAngle(x, y);
      f32 waterSpeed =
          controlStickSpeed(x, y, sidehopFloorPitch, SPEED_MODE_LINEAR);
      f32 slopeEssSpeed =
          controlStickSpeed(x, y, essFloorPitch, SPEED_MODE_CURVED);
      f32 flatEssSpeed = controlStickSpeed(x, y, 0x000, SPEED_MODE_CURVED);

      bool water = angle > 0x6000 && angle < 0xa000 && waterSpeed != 0.0f;
      bool slopeEss = slopeEssSpeed == 0.0f;
      bool flatEss = flatEssSpeed == 0.0f;

      printf("x=%d y=%d water=%d slopeEss=%d flatEss=%d\n", x, y, water,
             slopeEss, flatEss);
    }
  }
}

int main(int argc, char* argv[]) {
  Collision col(&spot00_sceneCollisionHeader_008464, PLAYER_AGE_CHILD,
                {3530, 100, 1830}, {3670, 200, 1960});

  // water floor
  col.addPoly(0x1db);
  col.addPoly(0x1dc);

  // col.printPolys();

  // Nalle's example: https://www.youtube.com/watch?v=xNxbIMgPS68
  // simulateFwess(
  //     &col,
  //     {intToFloat(0x4550330b), intToFloat(0x428a9d83),
  //     intToFloat(0x44ea8f0b)}, 0xa2cd, 0x0000, true);

  // Danny's example:
  // https://discord.com/channels/82938430371139584/82991320754294784/994085263049498707
  // simulateFwess(
  //     &col,
  //     {intToFloat(0x45568a61), intToFloat(0x42cba07c),
  //     intToFloat(0x44ef27d1)}, 0xa156, 0x8400, true);

  // simulateFwess(&col, {3480.0f, 117.15f, 1930.59f}, 0xa0c0, 0x0000, true);

  // findFwesses(&col);

  printControlStick(0xa200);

  return 0;
}
