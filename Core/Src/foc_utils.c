#include "foc_utils.h"
#include <math.h>

float _normalizeAngle(float angle)
{
    float a = fmodf(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

float _sin(float a)
{
    return sinf(a);
}

float _cos(float a)
{
    return cosf(a);
}