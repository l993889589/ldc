#ifndef LDC_PACKET_H
#define LDC_PACKET_H

#include <stdint.h>
#include <stdbool.h>

/* Packet descriptor. It points to bytes already stored in ldc_ring_t. */
typedef struct
{
    uint32_t start;
    uint32_t len;
} ldc_packet_t;

/*
 * FIFO queue of packet descriptors.
 *
 * Important rule: packets must be popped in FIFO order because ring bytes are
 * released with ldc_ring_drop() only after the oldest packet is read.
 */
typedef struct
{
    ldc_packet_t *nodes;
    uint16_t capacity;
    uint16_t in;
    uint16_t out;
    uint16_t count;
} ldc_packet_queue_t;

void ldc_packet_init(ldc_packet_queue_t *queue, ldc_packet_t *nodes, uint16_t capacity);
bool ldc_packet_push(ldc_packet_queue_t *queue, uint32_t start, uint32_t len);
bool ldc_packet_pop(ldc_packet_queue_t *queue, ldc_packet_t *packet);
bool ldc_packet_peek(const ldc_packet_queue_t *queue, ldc_packet_t *packet);
uint16_t ldc_packet_count(const ldc_packet_queue_t *queue);
bool ldc_packet_is_empty(const ldc_packet_queue_t *queue);
bool ldc_packet_is_full(const ldc_packet_queue_t *queue);

#endif
