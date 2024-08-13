#include "control_stick.hpp"

#include "sys_math.hpp"

int subtractDeadZone(int x) {
  if (x > 7) {
    return std::min(x - 7, 60);
  } else if (x < -7) {
    return std::max(x + 7, -60);
  } else {
    return 0;
  }
}

f32 controlStickMagnitude(int x, int y) {
  int relX = subtractDeadZone(x);
  int relY = subtractDeadZone(y);
  return std::min(sqrtf(SQ(relX) + SQ(relY)), 60.0f);
}

u16 controlStickAngle(int x, int y) {
  int relX = subtractDeadZone(x);
  int relY = subtractDeadZone(y);
  return Math_Atan2S(relY, -relX);
}

u16 computeFloorPitch(Vec3f normal, u16 angle) {
  f32 sin = Math_SinS(angle);
  f32 cos = Math_CosS(angle);
  return Math_Atan2S(1.0f, (-(normal.x * sin) - (normal.z * cos)) * (1.0f / normal.y));
}

f32 controlStickSpeed(int x, int y, u16 floorPitch, SpeedMode speedMode) {
  f32 magnitude = controlStickMagnitude(x, y);

  f32 speedTarget = magnitude;
  switch (speedMode) {
    case SPEED_MODE_CURVED:
      speedTarget -= 20.0f;

      if (speedTarget < 0.0f) {
        speedTarget = 0.0f;
      } else {
        f32 t = 1.0f - Math_CosS(speedTarget * 450.0f);
        speedTarget = (SQ(t) * 30.0f) + 7.0f;
      }
      break;
    case SPEED_MODE_LINEAR:
      speedTarget *= 0.8f;
      break;
  }

  if (magnitude != 0.0f) {
    f32 sinFloorPitch = Math_SinS(floorPitch);
    f32 floorPitchInfluence = std::min(std::max(sinFloorPitch, 0.0f), 0.6f);

    // TODO: speedCap and unk_6C4?

    speedTarget =
        speedTarget * 0.14f - 8.0f * floorPitchInfluence * floorPitchInfluence;
    speedTarget = std::max(speedTarget, 0.0f);
  }

  return speedTarget;
}
