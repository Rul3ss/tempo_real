#include "mediamovel.h"

float calcular_media_movel(circular_buffer *buf, float *soma_atual, float nova_amostra)
{
    if (cb_get_filled(buf) == buf->size)
    {
        float valor_antigo = 0.0f;
        cb_pop(buf, &valor_antigo);
        *soma_atual -= valor_antigo;
    }
    cb_push(buf, nova_amostra);
    *soma_atual += nova_amostra;
    int preenchidos = cb_get_filled(buf);
    if (preenchidos == 0)
        return 0.0f;

    return *soma_atual / preenchidos;
}