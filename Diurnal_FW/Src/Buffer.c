#include "Buffer.h"

void Ring_buffer_init(struct ring_buffer *rb)
{
	rb->head = 0;
	rb->tail = 0;	
}

int Ring_buffer_full(struct ring_buffer *rb)
{
	return ((rb->head - rb->tail) >= BUFFER_SIZE) ? 1 : 0;
}

int Ring_buffer_empty(struct ring_buffer *rb)
{
	return ((rb->head - rb->tail) == 0) ? 1 : 0;
}

int Ring_buffer_put(struct ring_buffer *rb, uint8_t data)
{
	if(Ring_buffer_full(rb) == 1) return -1;
	rb->buf[rb->head & (BUFFER_SIZE - 1)] = data;
	(rb->head)++;

	return 0;
}

int Ring_buffer_get(struct ring_buffer *rb, uint8_t *data)
{
	if(Ring_buffer_empty(rb) == 1) return -1;
	*data = rb->buf[rb->tail & (BUFFER_SIZE - 1)];
	(rb->tail)++;
	
	return 0;
}

void Ring_buffer_flush(struct ring_buffer *rb)
{
    rb->head = 0;
    rb->tail = 0;
}