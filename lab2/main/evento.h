#ifndef EVENTO_H
#define EVENTO_H

#include <stdbool.h>
#include <stdint.h>

#include "nota.h"
#include "config.h"

extern uint32_t block_count;

bool detecta_evento(float energy);

void imprime_evento(
    bool flag_evento,
    float energy,
    float freq_dominante,
    float freq_suave,
    NotaInfo nota);



#endif