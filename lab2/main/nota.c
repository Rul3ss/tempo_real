#include "nota.h"
#include <math.h>
#include <stdio.h>


static const char *nomes_notas[] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};


NotaInfo identificar_nota_musical(float freq_suavizada)
{
    NotaInfo info = {.nome = "--", .cents = 0.0f, .status = "silencio"};

    if (freq_suavizada < 20.0f)
        return info;

    int midi = (int)roundf(69.0f + 12.0f * log2f(freq_suavizada / 440.0f));
    float f_ref = 440.0f * powf(2.0f, (midi - 69) / 12.0f);
    info.cents = 1200.0f * log2f(freq_suavizada / f_ref);

    int nota_indice = midi % 12;
    int oitava = (midi / 12) - 1;
    if (nota_indice < 0)
        nota_indice += 12;

    snprintf(info.nome, sizeof(info.nome), "%s%d", nomes_notas[nota_indice], oitava);

    if (fabsf(info.cents) <= 10.0f)
    {
        info.status = "afinado";
    }
    else if (info.cents < 0.0f)
    {
        info.status = "abaixo";
    }
    else
    {
        info.status = "acima";
    }

    return info;
}