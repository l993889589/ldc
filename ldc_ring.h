#ifndef LDC_RING_H
#define LDC_RING_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Single-producer/single-consumer style ring buffer used by LDC.
 * LDC protects access with the injected lock/unlock callbacks, so this module
 * only focuses on index arithmetic and byte storage.
 *
 * One byte is intentionally left unused. This makes head == tail mean empty.
 */
typedef struct
{
    uint8_t *buf;
    uint32_t size;
    volatile uint32_t head;
    volatile uint32_t tail;
} ldc_ring_t;

void ldc_ring_init(ldc_ring_t *ring, uint8_t *buffer, uint32_t size);
uint32_t ldc_ring_used(const ldc_ring_t *ring);
uint32_t ldc_ring_free(const ldc_ring_t *ring);
bool ldc_ring_is_empty(const ldc_ring_t *ring);
bool ldc_ring_is_full(const ldc_ring_t *ring);

/* Write/read/drop return or consume byte counts only. They do not create packets. */
uint32_t ldc_ring_write(ldc_ring_t *ring, const uint8_t *data, uint32_t len);
uint32_t ldc_ring_read(ldc_ring_t *ring, uint8_t *data, uint32_t len);
void ldc_ring_drop(ldc_ring_t *ring, uint32_t len);

#endif
