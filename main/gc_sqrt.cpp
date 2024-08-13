#include "global.hpp"

#include <array>

struct BaseAndDec {
  int m_base;
  int m_dec;
};

const std::array<BaseAndDec, 32> frsqrte_expected = {{
    {0x1a7e800, -0x568}, {0x17cb800, -0x4f3}, {0x1552800, -0x48d}, {0x130c000, -0x435},
    {0x10f2000, -0x3e7}, {0x0eff000, -0x3a2}, {0x0d2e000, -0x365}, {0x0b7c000, -0x32e},
    {0x09e5000, -0x2fc}, {0x0867000, -0x2d0}, {0x06ff000, -0x2a8}, {0x05ab800, -0x283},
    {0x046a000, -0x261}, {0x0339800, -0x243}, {0x0218800, -0x226}, {0x0105800, -0x20b},
    {0x3ffa000, -0x7a4}, {0x3c29000, -0x700}, {0x38aa000, -0x670}, {0x3572000, -0x5f2},
    {0x3279000, -0x584}, {0x2fb7000, -0x524}, {0x2d26000, -0x4cc}, {0x2ac0000, -0x47e},
    {0x2881000, -0x43a}, {0x2665000, -0x3fa}, {0x2468000, -0x3c2}, {0x2287000, -0x38e},
    {0x20c1000, -0x35e}, {0x1f12000, -0x332}, {0x1d79000, -0x30a}, {0x1bf4000, -0x2e6},
}};

// https://github.com/dolphin-emu/dolphin/blob/d4ec524f218f83bf3646f0dd666236307a727bc5/Source/Core/Common/FloatUtils.cpp#L86
f64 ApproximateReciprocalSquareRoot(f64 val) {
  s64 integral = std::bit_cast<s64>(val);
  s64 mantissa = integral & ((1LL << 52) - 1);
  const s64 sign = integral & (1ULL << 63);
  s64 exponent = integral & (0x7FFLL << 52);

  // Special case 0
  if (mantissa == 0 && exponent == 0) {
    return sign ? -std::numeric_limits<f64>::infinity() :
                  std::numeric_limits<f64>::infinity();
  }

  // Special case NaN-ish numbers
  if (exponent == (0x7FFLL << 52)) {
    if (mantissa == 0) {
      if (sign)
        return std::numeric_limits<f64>::quiet_NaN();

      return 0.0;
    }

    return 0.0 + val;
  }


  // Negative numbers return NaN
  if (sign)
    return std::numeric_limits<f64>::quiet_NaN();

  if (!exponent) {
    // "Normalize" denormal values
    do {
      exponent -= 1LL << 52;
      mantissa <<= 1;
    } while (!(mantissa & (1LL << 52)));
    mantissa &= (1LL << 52) - 1;
    exponent += 1LL << 52;
  }

  const s64 exponent_lsb = exponent & (1LL << 52);
  exponent = ((0x3FFLL << 52) - ((exponent - (0x3FELL << 52)) / 2)) & (0x7FFLL << 52);
  integral = sign | exponent;

  const int i = static_cast<int>((exponent_lsb | mantissa) >> 37);
  const auto& entry = frsqrte_expected[i / 2048];
  integral |= static_cast<s64>(entry.m_base + entry.m_dec * (i % 2048)) << 26;

  return std::bit_cast<f64>(integral);
}

// https://github.com/dolphin-emu/dolphin/blob/d4ec524f218f83bf3646f0dd666236307a727bc5/Source/Core/Core/PowerPC/Interpreter/Interpreter_FPUtils.h#L344

u32 ConvertToSingle(u64 x) {
  static constexpr u64 DOUBLE_SIGN = 0x8000000000000000ULL;
  static constexpr u64 DOUBLE_FRAC = 0x000FFFFFFFFFFFFFULL;

  u32 exp = (u32)((x >> 52) & 0x7ff);
  if (exp > 896 || (x & ~DOUBLE_SIGN) == 0) {
    return (u32)(((x >> 32) & 0xc0000000) | ((x >> 29) & 0x3fffffff));
  } else if (exp >= 874) {
    u32 t = (u32)(0x80000000 | ((x & DOUBLE_FRAC) >> 21));
    t = t >> (905 - exp);
    t |= (u32)((x >> 32) & 0x80000000);
    return t;
  } else {
    // This is said to be undefined.
    // The code is based on hardware tests.
    return (u32)((x >> 32) & 0xc0000000) | ((x >> 29) & 0x3fffffff);
  }
}

f32 truncate_double(f64 x) {
  u64 i = std::bit_cast<u64>(x);
  return std::bit_cast<f32>(ConvertToSingle(i));
}

// https://github.com/zeldaret/oot-gc/blob/37adc3d14cd7ed88c6d04564c92600c6c859909d/src/emulator/cpu.c#L1168
// Seems identical to https://github.com/zeldaret/oot-gc/blob/37adc3d14cd7ed88c6d04564c92600c6c859909d/libc/math.h#L60
f64 gc_sqrt(f64 x) {
  if (x > 0.0) {
    f64 guess = ApproximateReciprocalSquareRoot(x);
    guess = 0.5 * guess * (3.0 - guess * guess * x);
    guess = 0.5 * guess * (3.0 - guess * guess * x);
    guess = 0.5 * guess * (3.0 - guess * guess * x);
    guess = 0.5 * guess * (3.0 - guess * guess * x);
    return x * guess;
  } else if (x == 0.0) {
    return 0.0;
  } else if (x) {
    return NAN;
  }
  return INFINITY;
}

// https://github.com/zeldaret/oot-gc/blob/37adc3d14cd7ed88c6d04564c92600c6c859909d/src/emulator/cpu.c#L1114
// Seems identical to https://github.com/zeldaret/oot-gc/blob/37adc3d14cd7ed88c6d04564c92600c6c859909d/libc/math.h#L77
// except with an extra iteration
f32 gc_sqrtf(f32 x) {
  if (x > 0.0f) {
    f64 guess = ApproximateReciprocalSquareRoot((f64)x);
    guess = 0.5 * guess * (3.0 - guess * guess * x);
    guess = 0.5 * guess * (3.0 - guess * guess * x);
    guess = 0.5 * guess * (3.0 - guess * guess * x);
    guess = 0.5 * guess * (3.0 - guess * guess * x);
    return (f32)(x * guess);
  }
  return x;
}

int main(int argc, char* argv[]) {
  // Enumerate all positive floats
  for (u32 i = 0; i < 0x7F800000; i++) {
    f32 value = intToFloat(i);
    if ((i & 0xFFFFF) == 0) {
      fprintf(stderr, "testing %08x %.9g ...\r", i, value);
    }

    // sqrtf
    // f32 expected = sqrtf(value);
    // f32 actual = gc_sqrtf(value);

    // sqrt (correctly rounded)
    f32 expected = (f32)sqrt((f64)value);
    f32 actual = (f32)gc_sqrt((f64)value);

    // sqrt (incorrectly rounded)
    // f32 expected = (f32)sqrt((f64)value);
    // f32 actual = truncate_double(gc_sqrt((f64)value));

    if (expected != actual) {
      printf("input=%.9g (%08x) expected=%.9g (%08x) actual=%.9g (%08x)\n",
             value, i, expected, floatToInt(expected), actual, floatToInt(actual));
    }
  }

  return 0;
}
