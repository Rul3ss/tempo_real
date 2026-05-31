#include "bufferpp.h"

void pp_init(pingpong_buffer *pp)
{
    pp->active = 0;

    pp->write_index[0] = 0;
    pp->block_ready[0] = false;
    pp->block_in_use[0] = false;

    pp->write_index[1] = 0;
    pp->block_ready[1] = false;
    pp->block_in_use[1] = false;
}

int pp_write(pingpong_buffer *pp, int sample)
{
    int a = pp->active;

    if (pp->block_ready[a] || pp->block_in_use[a])
    {
        return 0;
    }

    pp->buffer[a][pp->write_index[a]++] = sample;

    if (pp->write_index[a] == BLOCK_SIZE)
    {
        pp->block_ready[a] = true;
        pp->write_index[a] = 0;
        pp->active = !a;
    }

    return 1;
}

int pp_get_ready_block(pingpong_buffer *pp)
{
    for (int i = 0; i < 2; i++)
    {
        if (pp->block_ready[i] && !pp->block_in_use[i])
        {
            pp->block_in_use[i] = true;
            return i;
        }
    }

    return -1;
}

int16_t *pp_get_block_data(pingpong_buffer *pp, int block_index)
{
    return pp->buffer[block_index];
}

void pp_release_block(pingpong_buffer *pp, int block_index)
{
    pp->block_ready[block_index] = false;
    pp->block_in_use[block_index] = false;
}