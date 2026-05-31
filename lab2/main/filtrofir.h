#ifndef FILTRO_FIR
#define FILTRO_FIR

#include "config.h"
#include <stdint.h>

extern const float fir_coeffs[FIR_TAPS];

void apply_fir(int16_t *input, int16_t *output, int num_samples);


#endif