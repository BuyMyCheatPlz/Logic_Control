#include "soft_filter.h"
#include <math.h>

float SoftFilterEMA_Update(SoftFilterEMA_t *f, float input)
{
    if (f == NULL) return input;

    if (!f->initialized)
    {
        f->state = input;
        f->initialized = 1;
        return f->state;
    }

    /* Clamp single-sample spikes */
    float diff = input - f->state;
    if (f->spike_limit > 0.0f && fabsf(diff) > f->spike_limit)
    {
        input = f->state + ((diff > 0.0f) ? f->spike_limit : -f->spike_limit);
    }

    /* EMA update */
    float a = f->alpha;
    if (a <= 0.0f)
    {
        /* no update */
        return f->state;
    }
    if (a >= 1.0f)
    {
        f->state = input;
        return f->state;
    }

    f->state = a * input + (1.0f - a) * f->state;
    return f->state;
}
