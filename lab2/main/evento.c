#include "evento.h"

#include <stdio.h>
#include <stdint.h>

uint32_t block_count = 0;

bool detecta_evento(float energy)
{
    static int64_t ultimo_bloco_evento = -1000;

    if (energy >= LIMIAR)
    {
        if ((block_count - ultimo_bloco_evento) > REFRACTORY_BLOCKS)
        {
            ultimo_bloco_evento = block_count;
            return true;
        }
    }

    return false;
}

void imprime_evento(bool flag_evento,
                    float energy,
                    float freq_dominante,
                    float freq_suave,
                    NotaInfo nota)
{
    if (!flag_evento)
    {
        return;
    }

    float t_s = block_count * ((float)BLOCK_SIZE / SAMPLE_RATE); // tempo em segundos

    printf("*** EVENTO DETECTADO ***\n");
    printf("Bloco #%lu | t = %.3f s\n", block_count, t_s);
    printf("Energia = %.3f\n", energy);
    printf("Freq. dominante = %.1f Hz\n", freq_dominante);
    printf("Freq. dominante Suavizada = %.1f Hz\n", freq_suave);
    printf("Nota = %s\n", nota.nome);
    printf("Erro = %+.1f cents\n", nota.cents);
    printf("Status = %s\n", nota.status);
    printf("------------------------\n");
}