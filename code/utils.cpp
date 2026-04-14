#include "utils.h"
#include "math.h"
#include "consts.h"

float AngleMod(float a) {
    a = fmod(a, 2  * PI);
    if (a < - PI) {
        return a + 2 * PI;
    }
    if (a >= PI) {
        return a - 2 * PI;
    }
    return a;
}