#include <stdint.h>
#include <string.h>

typedef struct{
	uint8_t *buffer;
	int front;
	int rear;
	int maxlen;
}ring_buffer_t;

void ring_buf_init(ring_buffer_t *rbuf,uint8_t *buffer, int size);
void ring_buf_clr(ring_buffer_t *rbuf);
void buf_clear(ring_buffer_t *rbuf);
uint8_t ring_buf_push(ring_buffer_t *rbuf, uint8_t data);
uint8_t ring_buf_pop(ring_buffer_t *rbuf, uint8_t *data);
uint8_t ring_buf_is_full(ring_buffer_t *rbuf);
uint8_t ring_buf_is_empty(ring_buffer_t *rbuf);



