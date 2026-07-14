#include "ldc_ring.h"

#include <string.h>

void ldc_ring_init(ldc_ring_t *ring, uint8_t *buffer, uint32_t size)
{
    ring->buf = buffer;
    ring->size = size;
    ring->head = 0U;
    ring->tail = 0U;
}

uint32_t ldc_ring_used(const ldc_ring_t *ring)
{
    if(ring->head >= ring->tail)
        return ring->head - ring->tail;

    return ring->size - ring->tail + ring->head;
}

uint32_t ldc_ring_free(const ldc_ring_t *ring)
{
    return ring->size - ldc_ring_used(ring) - 1U;
}

bool ldc_ring_is_empty(const ldc_ring_t *ring)
{
    return ring->head == ring->tail;
}

bool ldc_ring_is_full(const ldc_ring_t *ring)
{
    return ((ring->head + 1U) % ring->size) == ring->tail;
}

uint32_t ldc_ring_write(ldc_ring_t *ring, const uint8_t *data, uint32_t len)
{
    uint32_t free_space = ldc_ring_free(ring);
    uint32_t first;

    if(len > free_space)
        len = free_space;

    first = ring->size - ring->head;
    if(first > len)
        first = len;

    memcpy(&ring->buf[ring->head], data, first);
    if(len > first)
        memcpy(&ring->buf[0], data + first, len - first);

    ring->head = (ring->head + len) % ring->size;

    return len;
}

uint32_t ldc_ring_read(ldc_ring_t *ring, uint8_t *data, uint32_t len)
{
    uint32_t used = ldc_ring_used(ring);
    uint32_t first;

    if(len > used)
        len = used;

    first = ring->size - ring->tail;
    if(first > len)
        first = len;

    memcpy(data, &ring->buf[ring->tail], first);
    if(len > first)
        memcpy(data + first, &ring->buf[0], len - first);

    ring->tail = (ring->tail + len) % ring->size;

    return len;
}

void ldc_ring_drop(ldc_ring_t *ring, uint32_t len)
{
    uint32_t used = ldc_ring_used(ring);

    if(len > used)
        len = used;

    ring->tail = (ring->tail + len) % ring->size;
}
