#ifndef BUFFERC_H
#define BUFFERC_H

typedef struct
{
    float *buffer;
    int size;
    unsigned int head;
    unsigned int tail;
    int count;
} circular_buffer;

void cb_init(circular_buffer *buf, int size);
int cb_get_avail(circular_buffer *buf);
int cb_get_filled(circular_buffer *buf);
int cb_push(circular_buffer *buf, float data);
int cb_pop(circular_buffer *buf, float *data);


#endif