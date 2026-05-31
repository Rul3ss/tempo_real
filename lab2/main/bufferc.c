#include "bufferc.h"
#include <stdlib.h>


void cb_init(circular_buffer *buf, int size)
{
    buf->buffer = (float *)malloc(size * sizeof(float));
    buf->head = 0;
    buf->tail = 0;
    buf->size = size;
    buf->count = 0;
}

int cb_get_avail(circular_buffer *buf)
{
    return buf->size - buf->count;
}

int cb_get_filled(circular_buffer *buf)
{
    return buf->count;
}

int cb_push(circular_buffer *buf, float data)
{
    if (cb_get_avail(buf) < 1)
    {
        return 0;
    }

    buf->buffer[buf->head] = data;
    buf->head = (buf->head + 1) % buf->size;
    buf->count++;
    return 1;
}

int cb_pop(circular_buffer *buf, float *data)
{
    if (cb_get_filled(buf) < 1)
    {
        return 0; // Buffer vazio
    }

    *data = buf->buffer[buf->tail];
    buf->tail = (buf->tail + 1) % buf->size;
    buf->count--;
    return 1;
}