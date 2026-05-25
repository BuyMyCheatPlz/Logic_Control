/* soft_filter.h - simple software filtering utilities
 * Provides EMA (exponential moving average) with optional spike clamp.
 */
#ifndef SOFT_FILTER_H
#define SOFT_FILTER_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    float alpha;        /* EMA coefficient (0..1) */
    float spike_limit;  /* maximum allowed single-sample jump */
    float state;        /* current filtered value */
    uint8_t initialized;
} SoftFilterEMA_t;

static inline void SoftFilterEMA_Init(SoftFilterEMA_t *f, float alpha, float spike_limit)
{
    if (f == NULL) return;
    f->alpha = alpha;
    f->spike_limit = spike_limit;
    f->initialized = 0;
    f->state = 0.0f;
}

/* Update EMA with input value, returns filtered value. */
float SoftFilterEMA_Update(SoftFilterEMA_t *f, float input);

#endif /* SOFT_FILTER_H */
