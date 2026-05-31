#ifndef BUFFERPP_H
#define BUFFERPP_H

#include <stdint.h>
#include <stdbool.h>

#include "config.h"

typedef struct
{
    int16_t buffer[2][BLOCK_SIZE];
    int write_index[2];
    volatile int active;
    volatile bool block_ready[2];
    volatile bool block_in_use[2];
} pingpong_buffer;

void pp_init(pingpong_buffer *pp);
int pp_write(pingpong_buffer *pp, int sample);
int pp_get_ready_block(pingpong_buffer *pp);
int16_t *pp_get_block_data(pingpong_buffer *pp, int block_index);
void pp_release_block(pingpong_buffer *pp, int block_index);

#endif
