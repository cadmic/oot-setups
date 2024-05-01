#include "rand.hpp"

#include <vector>

int rupeeType(u32* seed) {
    switch ((s16)Rand_ZeroFloat(30.99f, seed)) {
        case 0:
            return 2;
        case 10:
        case 20:
        case 30:
            return 1;
    }
    return 0;
}

int countNumRed(u32 start) {
    u32 seed = start;

    int total = 0;
    for (int i = 0; i < 8; i++) {
        if (rupeeType(&seed) == 2) {
            total++;
        }
    }

    return total;
}

int main(int argc, char* argv[]) {
    u32 start = 0;

    std::vector<u32> histogram = std::vector<u32>(9, 0);

    while (true) {
        if (start % 0x100000 == 0) {
            fprintf(stderr, "testing seed 0x%08x ...\r", start);
        }

        int numRed = countNumRed(start);
        histogram[numRed]++;

        if (start == std::numeric_limits<u32>::max()) {
            break;
        }
        start++;
    }

    for (int i = 0; i < 9; i++) {
        printf("%d reds: %10u seeds (%.4g%%)\n", i, histogram[i], (f64)(histogram[i]) * 100 / (f64)std::numeric_limits<u32>::max());
    }
}
