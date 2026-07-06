#ifndef UTILS_H
#define UTILS_H

static inline float sgn(float x)
{
    return (x > 0.0f) - (x < 0.0f);
}

#endif // UTILS_H