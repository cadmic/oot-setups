#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#undef M_PI
#define M_PI 3.14159265358979323846f
#undef M_SQRT2
#define M_SQRT2 1.41421356237309504880f

#define SHT_MAX 32767.0f
#define SHT_MINV (1.0f / SHT_MAX)

#define IS_ZERO(f) (fabsf(f) < 0.008f)
#define SQ(x) ((x) * (x))
#define DOTXYZ(vec1, vec2) \
  ((vec1).x * (vec2).x + (vec1).y * (vec2).y + (vec1).z * (vec2).z)

#define ESS 0x708

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef float f32;

struct MtxF {
  // Note: The order displayed here is the transpose of the order in which
  // matrices are typically written. For example, [xw, yw, zw] is the
  // translation part of the matrix, not [wx, wy, wz].
  float xx, yx, zx, wx, xy, yy, zy, wy, xz, yz, zz, wz, xw, yw, zw, ww;
};

struct Vec3s {
  s16 x, y, z;

  Vec3s() : x(0), y(0), z(0) {}
  Vec3s(s16 x, s16 y, s16 z) : x(x), y(y), z(z) {}

  Vec3s operator+(const Vec3s& rhs) const {
    return Vec3s(x + rhs.x, y + rhs.y, z + rhs.z);
  }

  Vec3s operator-(const Vec3s& rhs) const {
    return Vec3s(x - rhs.x, y - rhs.y, z - rhs.z);
  }

  bool operator==(const Vec3s& rhs) const {
    return x == rhs.x && y == rhs.y && z == rhs.z;
  }

  bool operator!=(const Vec3s& rhs) const { return !(*this == rhs); }
};

struct Vec3f {
  f32 x, y, z;

  Vec3f() : x(0), y(0), z(0) {}
  Vec3f(f32 x, f32 y, f32 z) : x(x), y(y), z(z) {}
  Vec3f(const Vec3s& v) : x((f32)v.x), y((f32)v.y), z((f32)v.z) {}

  Vec3s toVec3s() const { return Vec3s{(s16)x, (s16)y, (s16)z}; }

  Vec3f operator+(const Vec3f& rhs) const {
    return Vec3f(x + rhs.x, y + rhs.y, z + rhs.z);
  }

  Vec3f operator-(const Vec3f& rhs) const {
    return Vec3f(x - rhs.x, y - rhs.y, z - rhs.z);
  }

  Vec3f operator*(const f32& rhs) const {
    return Vec3f(x * rhs, y * rhs, z * rhs);
  }

  bool operator==(const Vec3f& rhs) const {
    return x == rhs.x && y == rhs.y && z == rhs.z;
  }

  bool operator!=(const Vec3f& rhs) const { return !(*this == rhs); }
};

inline const f32 intToFloat(u32 i) {
  f32 out;
  memcpy(&out, &i, 4);
  return out;
}

inline u32 floatToInt(f32 x) {
  u32 out;
  memcpy(&out, &x, 4);
  return out;
}

enum PlayerAge {
  PLAYER_AGE_CHILD,
  PLAYER_AGE_ADULT,
};
