#include "ldc_easy.h"

#include <string.h>

static ldc_easy_t *s_tick_list;

static ldc_easy_event_t ldc_easy_map_event(ldc_event_t event)
{
    if(event == LDC_EVT_PACKET)
        return LDC_EASY_EVT_PACKET;
    if(event == LDC_EVT_OVERFLOW)
        return LDC_EASY_EVT_OVERFLOW;
    return LDC_EASY_EVT_DROP;
}

static void ldc_easy_on_core_event(void *arg, ldc_event_t event)
{
    ldc_easy_t *queue = (ldc_easy_t *)arg;

    if(queue == NULL || queue->event_cb == NULL)
        return;

    queue->event_cb(queue, ldc_easy_map_event(event), queue->event_arg);
}

static bool ldc_easy_config_valid(const ldc_easy_config_t *config)
{
    return config != NULL &&
           config->ring_buffer != NULL &&
           config->ring_size >= 2U &&
           config->packet_pool != NULL &&
           config->packet_count != 0U;
}

static bool ldc_easy_tick_register_with_config_lock(ldc_easy_t *queue,
                                                    const ldc_easy_config_t *config)
{
    uint32_t lock_state = 0U;
    bool locked = false;
    bool registered;

    if(config->lock != NULL && config->unlock != NULL)
    {
        lock_state = config->lock(config->lock_arg);
        locked = true;
    }

    registered = ldc_easy_tick_register(queue);

    if(locked)
        config->unlock(config->lock_arg, lock_state);

    return registered;
}

bool ldc_easy_init(ldc_easy_t *queue, const ldc_easy_config_t *config)
{
    int delimiter;

    if(queue == NULL || !ldc_easy_config_valid(config))
        return false;

    memset(queue, 0, sizeof(*queue));

    if(!ldc_init(&queue->ldc,
                 config->ring_buffer,
                 config->ring_size,
                 config->packet_pool,
                 config->packet_count))
        return false;

    queue->event_cb = config->event_cb;
    queue->event_arg = config->event_arg;

    ldc_set_lock(&queue->ldc, config->lock, config->unlock, config->lock_arg);
    ldc_set_mode(&queue->ldc, config->mode);

    delimiter = config->delimiter_enabled ? (int)config->delimiter : -1;
    if(config->timeout_us != 0U)
    {
        ldc_set_frame_config_us(&queue->ldc,
                                config->max_frame,
                                config->timeout_us,
                                delimiter);
    }
    else
    {
        ldc_set_frame_config(&queue->ldc,
                             config->max_frame,
                             config->timeout_ms,
                             delimiter);
    }

    ldc_set_callback(&queue->ldc, ldc_easy_on_core_event, queue);
    queue->initialized = 1U;

    if(config->auto_tick && !ldc_easy_tick_register_with_config_lock(queue, config))
        return false;

    return true;
}

uint32_t ldc_easy_add(ldc_easy_t *queue, const uint8_t *data, uint32_t len)
{
    if(queue == NULL || queue->initialized == 0U)
        return 0U;

    return ldc_write(&queue->ldc, data, len);
}

bool ldc_easy_putc(ldc_easy_t *queue, uint8_t byte)
{
    return ldc_easy_add(queue, &byte, 1U) == 1U;
}

uint32_t ldc_easy_rx_idle(ldc_easy_t *queue, const uint8_t *data, uint32_t len)
{
    uint32_t written;

    if(queue == NULL || queue->initialized == 0U)
        return 0U;

    written = ldc_write(&queue->ldc, data, len);
    if(written == len)
    {
        (void)ldc_flush(&queue->ldc);
    }
    else if(written != 0U)
    {
        (void)ldc_discard_frame(&queue->ldc);
    }

    return written;
}

void ldc_easy_tick(ldc_easy_t *queue, uint32_t elapsed_ms)
{
    if(queue == NULL || queue->initialized == 0U)
        return;

    ldc_tick(&queue->ldc, elapsed_ms);
}

void ldc_easy_tick_us(ldc_easy_t *queue, uint32_t elapsed_us)
{
    if(queue == NULL || queue->initialized == 0U)
        return;

    ldc_tick_us(&queue->ldc, elapsed_us);
}

bool ldc_easy_tick_register(ldc_easy_t *queue)
{
    if(queue == NULL || queue->initialized == 0U)
        return false;

    if(queue->tick_registered != 0U)
        return true;

    queue->tick_next = s_tick_list;
    s_tick_list = queue;
    queue->tick_registered = 1U;
    return true;
}

void ldc_easy_tick_unregister(ldc_easy_t *queue)
{
    ldc_easy_t **link = &s_tick_list;

    if(queue == NULL || queue->tick_registered == 0U)
        return;

    while(*link != NULL)
    {
        if(*link == queue)
        {
            *link = queue->tick_next;
            queue->tick_next = NULL;
            queue->tick_registered = 0U;
            return;
        }
        link = &(*link)->tick_next;
    }

    queue->tick_next = NULL;
    queue->tick_registered = 0U;
}

void ldc_easy_tick_all(uint32_t elapsed_ms)
{
    ldc_easy_t *queue = s_tick_list;

    while(queue != NULL)
    {
        ldc_easy_tick(queue, elapsed_ms);
        queue = queue->tick_next;
    }
}

void ldc_easy_tick_all_us(uint32_t elapsed_us)
{
    ldc_easy_t *queue = s_tick_list;

    while(queue != NULL)
    {
        ldc_easy_tick_us(queue, elapsed_us);
        queue = queue->tick_next;
    }
}

bool ldc_easy_settle(ldc_easy_t *queue)
{
    if(queue == NULL || queue->initialized == 0U)
        return false;

    return ldc_flush(&queue->ldc);
}

bool ldc_easy_abort(ldc_easy_t *queue)
{
    if(queue == NULL || queue->initialized == 0U)
        return false;

    return ldc_discard_frame(&queue->ldc);
}

int ldc_easy_pop(ldc_easy_t *queue, uint8_t *buf, uint32_t size)
{
    if(queue == NULL || queue->initialized == 0U)
        return -1;

    return ldc_read_packet(&queue->ldc, buf, size);
}

uint16_t ldc_easy_available(ldc_easy_t *queue)
{
    if(queue == NULL || queue->initialized == 0U)
        return 0U;

    return ldc_packet_available(&queue->ldc);
}

bool ldc_easy_frame_pending(ldc_easy_t *queue)
{
    if(queue == NULL || queue->initialized == 0U)
        return false;

    return ldc_frame_pending(&queue->ldc);
}

bool ldc_easy_get_stats(ldc_easy_t *queue, ldc_stats_t *stats)
{
    if(queue == NULL || queue->initialized == 0U || stats == NULL)
        return false;

    return ldc_get_stats(&queue->ldc, stats);
}

ldc_t *ldc_easy_core(ldc_easy_t *queue)
{
    if(queue == NULL || queue->initialized == 0U)
        return NULL;

    return &queue->ldc;
}
