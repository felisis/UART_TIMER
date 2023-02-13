#include <stdint.h>
#include <string.h>
#include "ring_buffer.h"

void ring_buf_init(ring_buffer_t *rbuf, uint8_t *buffer, int size) {
	memset(rbuf, 0x00, size);
	rbuf->buffer = buffer;
	rbuf->maxlen = size;
}

void ring_buf_clr(ring_buffer_t *rbuf) {
	rbuf->front = 0;
	rbuf->rear = 0;
}

void buf_clear(ring_buffer_t *rbuf) {
	memset(rbuf->buffer, 0x00, 30);
}

uint8_t ring_buf_is_full(ring_buffer_t *rbuf) {
	if (rbuf->rear == (rbuf->front + 1) % rbuf->maxlen)
		return 1;
	else
		return 0;
}
uint8_t ring_buf_is_empty(ring_buffer_t *rbuf) {
	if (rbuf->rear == rbuf->front)
		return 1;
	else
		return 0;
}

uint8_t ring_buf_push(ring_buffer_t *rbuf, uint8_t data) {
	if (ring_buf_is_full(rbuf))
		return 0;
	rbuf->buffer[rbuf->front] = data;
	rbuf->front = (rbuf->front + 1) % rbuf->maxlen;
	return 1;

}
uint8_t ring_buf_pop(ring_buffer_t *rbuf, uint8_t *data) {
	if (ring_buf_is_empty(rbuf))
		return 0;
	*data = rbuf->buffer[rbuf->rear];
	rbuf->rear = (rbuf->rear + 1) % rbuf->maxlen;
	return 1;
}

