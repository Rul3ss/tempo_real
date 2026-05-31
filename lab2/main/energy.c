#include "energy.h"

#include <math.h>


float system_energy(int16_t *data, int num_samples)
{
    float soma_quadrados = 0.0f;

    for (int i = 0; i < num_samples; i++)
    {
        float sample = (float)data[i];
        // Normaliza a amostra individualmente para o intervalo [-1.0, 1.0]
        // antes de multiplicar, evitando estourar a capacidade do float (overflow)
        float sample_norm = sample / 32768.0f;
        soma_quadrados += (sample_norm * sample_norm);
    }

    // Calcula a energia média do bloco (agora garantida entre 0.0 e 1.0)
    float energia_media = soma_quadrados / num_samples;

    // Retorna o valor RMS (Root Mean Square)
    // Deixa a escala de energia linear e muito mais fácil de ler no console
    return sqrtf(energia_media);
}