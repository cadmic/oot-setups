#include "rand.hpp"

#define RAND_MULTIPLIER 1664525
#define RAND_INCREMENT 1013904223

f32 Rand_ZeroOne(u32* seed) {
    *seed = *seed * RAND_MULTIPLIER + RAND_INCREMENT;
    return intToFloat((*seed >> 9) | 0x3F800000) - 1.0f;
}

f32 Rand_ZeroFloat(f32 f, u32* seed) {
    return Rand_ZeroOne(seed) * f;
}
