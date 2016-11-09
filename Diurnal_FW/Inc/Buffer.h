#ifndef _BUFFER_H  
#define _BUFFER_H  
     
#include "define.h"
#include "stm32l0xx_hal.h"

struct ring_buffer
{
	uint8_t buf[BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
};

#endif  

void Ring_buffer_init(struct ring_buffer *rb);
int Ring_buffer_full(struct ring_buffer *rb);
int Ring_buffer_empty(struct ring_buffer *rb);
int Ring_buffer_put(struct ring_buffer *rb, uint8_t data);
int Ring_buffer_get(struct ring_buffer *rb, uint8_t *data);
void Ring_buffer_flush(struct ring_buffer *rb);
