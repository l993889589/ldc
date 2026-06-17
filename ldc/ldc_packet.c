#include "ldc_packet.h"

void ldc_packet_init(ldc_packet_queue_t *queue, ldc_packet_t *nodes, uint16_t capacity)
{
    queue->nodes = nodes;
    queue->capacity = capacity;
    queue->in = 0U;
    queue->out = 0U;
    queue->count = 0U;
}

bool ldc_packet_push(ldc_packet_queue_t *queue, uint32_t start, uint32_t len)
{
    if(queue->count >= queue->capacity)
        return false;

    queue->nodes[queue->in].start = start;
    queue->nodes[queue->in].len = len;
    queue->in = (uint16_t)((queue->in + 1U) % queue->capacity);
    queue->count++;

    return true;
}

bool ldc_packet_pop(ldc_packet_queue_t *queue, ldc_packet_t *packet)
{
    if(queue->count == 0U)
        return false;

    *packet = queue->nodes[queue->out];
    queue->out = (uint16_t)((queue->out + 1U) % queue->capacity);
    queue->count--;

    return true;
}

bool ldc_packet_peek(const ldc_packet_queue_t *queue, ldc_packet_t *packet)
{
    if(queue->count == 0U)
        return false;

    *packet = queue->nodes[queue->out];
    return true;
}

uint16_t ldc_packet_count(const ldc_packet_queue_t *queue)
{
    return queue->count;
}

bool ldc_packet_is_empty(const ldc_packet_queue_t *queue)
{
    return queue->count == 0U;
}

bool ldc_packet_is_full(const ldc_packet_queue_t *queue)
{
    return queue->count >= queue->capacity;
}
