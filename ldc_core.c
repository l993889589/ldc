/*
 * LDC core: transport-independent byte buffering and frame extraction.
 *
 * How to use:
 * 1. Allocate one ldc_t, one ring buffer and one packet descriptor pool.
 * 2. Call ldc_init(), then configure framing with ldc_set_frame_config()
 *    for millisecond timeouts or ldc_set_frame_config_us() for microsecond
 *    timeouts such as Modbus RTU T3.5.
 * 3. Feed received bytes with ldc_write() or ldc_putc().
 * 4. Drive idle-time framing with ldc_tick() or ldc_tick_us().
 * 5. Read complete frames with ldc_read_packet().
 *
 * This file deliberately has no HAL, UART, ThreadX, USBX or malloc dependency.
 * Platform locking and wakeup are provided by callbacks in upper layers.
 */
#include "ldc_core.h"

#include <string.h>
#include <stdio.h>


static uint32_t ldc_enter(ldc_t *ldc)
{
    if(ldc->lock && ldc->unlock)
        return ldc->lock(ldc->lock_arg);

    return 0U;
}

static void ldc_leave(ldc_t *ldc, uint32_t state)
{
    if(ldc->lock && ldc->unlock)
        ldc->unlock(ldc->lock_arg, state);
}

typedef struct
{
    uint32_t packet;
    uint32_t overflow;
    uint32_t drop;
} ldc_pending_events_t;

static void ldc_notify_pending(ldc_t *ldc, const ldc_pending_events_t *events)
{
    if(!ldc->callback || !events)
        return;

    for(uint32_t i = 0U; i < events->drop; i++)
        ldc->callback(ldc->callback_arg, LDC_EVT_DROP);
    for(uint32_t i = 0U; i < events->overflow; i++)
        ldc->callback(ldc->callback_arg, LDC_EVT_OVERFLOW);
    for(uint32_t i = 0U; i < events->packet; i++)
        ldc->callback(ldc->callback_arg, LDC_EVT_PACKET);
}

static void ldc_refresh_usage(ldc_t *ldc)
{
    ldc->stats.cur_used = ldc_ring_used(&ldc->ring);
    ldc->stats.packet_used = ldc_packet_count(&ldc->packet);

    if(ldc->stats.cur_used > ldc->stats.max_used)
        ldc->stats.max_used = ldc->stats.cur_used;

    if(ldc->stats.packet_used > ldc->stats.packet_peak)
        ldc->stats.packet_peak = ldc->stats.packet_used;
}

static bool ldc_drop_oldest_packet(ldc_t *ldc, ldc_pending_events_t *events)
{
    ldc_packet_t old;

    if(!ldc_packet_pop(&ldc->packet, &old))
        return false;

    ldc_ring_drop(&ldc->ring, old.len);
    ldc->stats.drop++;
    ldc->stats.overwrite_count++;
    ldc_refresh_usage(ldc);
    events->drop++;

    return true;
}

static void ldc_reject_open_frame_unlocked(ldc_t *ldc)
{
    ldc->ring.head = ldc->frame_start;
    ldc->frame_len = 0U;
    ldc->frame_timer_us = 0U;
    ldc_refresh_usage(ldc);
}

static bool ldc_commit_frame_unlocked(ldc_t *ldc, ldc_pending_events_t *events)
{
    if(ldc->frame_len == 0U)
        return false;

    /* Packet queue stores only descriptors. The bytes remain in the ring until read_packet drops them. */
    while(ldc_packet_is_full(&ldc->packet))
    {
        if(ldc->mode != LDC_MODE_OVERWRITE || !ldc_drop_oldest_packet(ldc, events))
        {
            ldc->stats.overflow++;
            events->overflow++;
            ldc_reject_open_frame_unlocked(ldc);
            return false;
        }
    }

    if(!ldc_packet_push(&ldc->packet, ldc->frame_start, ldc->frame_len))
    {
        ldc->stats.overflow++;
        events->overflow++;
        ldc_reject_open_frame_unlocked(ldc);
        return false;
    }

    ldc->stats.packets++;
    ldc->frame_start = ldc->ring.head;
    ldc->frame_len = 0U;
    ldc->frame_timer_us = 0U;

    ldc_refresh_usage(ldc);
    events->packet++;

    return true;
}

static uint32_t ldc_write_unlocked(ldc_t *ldc,
                                   const uint8_t *data,
                                   uint32_t len,
                                   ldc_pending_events_t *events,
                                   bool *stop)
{
    uint32_t written = 0U;

    while(written < len)
    {
        uint32_t chunk;
        uint32_t free_space;
        const uint8_t *delimiter;

        /* Keep one byte empty in the ring so head == tail can mean empty. */
        while((free_space = ldc_ring_free(&ldc->ring)) == 0U)
        {
            if(ldc->mode != LDC_MODE_OVERWRITE || !ldc_drop_oldest_packet(ldc, events))
            {
                ldc->stats.overflow++;
                events->overflow++;
                *stop = true;
                return written;
            }
        }

        if(ldc->frame_len == 0U)
            ldc->frame_start = ldc->ring.head;

        chunk = len - written;
        if(chunk > free_space)
            chunk = free_space;

        if(ldc->max_len != 0U)
        {
            uint32_t frame_room = (ldc->frame_len < ldc->max_len) ?
                                  ldc->max_len - ldc->frame_len : 0U;

            if(frame_room == 0U)
            {
                (void)ldc_commit_frame_unlocked(ldc, events);
                continue;
            }
            if(chunk > frame_room)
                chunk = frame_room;
        }

        delimiter = NULL;
        if(ldc->delimiter >= 0)
        {
            delimiter = (const uint8_t *)memchr(data + written,
                                                (uint8_t)ldc->delimiter,
                                                chunk);
            if(delimiter)
                chunk = (uint32_t)(delimiter - (data + written)) + 1U;
        }

        chunk = ldc_ring_write(&ldc->ring, data + written, chunk);
        if(chunk == 0U)
        {
            *stop = true;
            return written;
        }

        written += chunk;
        ldc->stats.rx_bytes += chunk;
        ldc->frame_len += chunk;
        ldc->frame_timer_us = 0U;
        ldc_refresh_usage(ldc);

        /* Three supported frame-ending strategies: delimiter, fixed max length, or timeout via ldc_tick. */
        if(delimiter != NULL)
        {
            (void)ldc_commit_frame_unlocked(ldc, events);
        }
        else if(ldc->max_len != 0U && ldc->frame_len >= ldc->max_len)
        {
            (void)ldc_commit_frame_unlocked(ldc, events);
        }
    }

    return written;
}

bool ldc_init(ldc_t *ldc, uint8_t *ring_buffer, uint32_t ring_size,
              ldc_packet_t *packet_pool, uint16_t packet_count)
{
    if(!ldc)
        return false;

    memset(ldc, 0, sizeof(*ldc));
    if(!ring_buffer || ring_size < 2U || !packet_pool || packet_count == 0U)
        return false;

    /* All memory is provided by the caller. LDC itself never malloc/free's memory. */
    ldc_ring_init(&ldc->ring, ring_buffer, ring_size);
    ldc_packet_init(&ldc->packet, packet_pool, packet_count);

    ldc->frame_start = 0U;
    ldc->frame_len = 0U;
    ldc->frame_timer_us = 0U;

    ldc->max_len = 0U;
    ldc->timeout_us = 0U;
    ldc->atomic_write_bytes = LDC_DEFAULT_ATOMIC_WRITE_BYTES;
    ldc->delimiter = -1;
    ldc->write_active = 0U;
    ldc->mode = LDC_MODE_PROTECT;

    ldc->callback = NULL;
    ldc->callback_arg = NULL;
    ldc->lock = NULL;
    ldc->unlock = NULL;
    ldc->lock_arg = NULL;

    return true;
}

void ldc_set_frame_config(ldc_t *ldc, uint32_t max_len, uint32_t timeout_ms, int delimiter)
{
    uint32_t timeout_us = 0U;

    if(timeout_ms > (UINT32_MAX / 1000U))
        timeout_us = UINT32_MAX;
    else
        timeout_us = timeout_ms * 1000U;

    ldc_set_frame_config_us(ldc, max_len, timeout_us, delimiter);
}

void ldc_set_frame_config_us(ldc_t *ldc, uint32_t max_len, uint32_t timeout_us, int delimiter)
{
    if(!ldc)
        return;

    uint32_t state = ldc_enter(ldc);

    ldc->max_len = max_len;
    ldc->timeout_us = timeout_us;
    ldc->delimiter = delimiter;

    ldc_leave(ldc, state);
}

void ldc_set_mode(ldc_t *ldc, ldc_mode_t mode)
{
    if(!ldc)
        return;

    uint32_t state = ldc_enter(ldc);

    ldc->mode = mode;

    ldc_leave(ldc, state);
}

void ldc_set_callback(ldc_t *ldc, ldc_event_cb_t cb, void *arg)
{
    if(!ldc)
        return;

    uint32_t state = ldc_enter(ldc);

    ldc->callback = cb;
    ldc->callback_arg = arg;

    ldc_leave(ldc, state);
}

void ldc_set_atomic_write_bytes(ldc_t *ldc, uint32_t bytes)
{
    if(!ldc)
        return;

    uint32_t state = ldc_enter(ldc);

    ldc->atomic_write_bytes = bytes;

    ldc_leave(ldc, state);
}

void ldc_set_lock(ldc_t *ldc, ldc_lock_cb_t lock, ldc_unlock_cb_t unlock, void *arg)
{
    if(!ldc)
        return;

    ldc->lock = lock;
    ldc->unlock = unlock;
    ldc->lock_arg = arg;
}

uint32_t ldc_write(ldc_t *ldc, const uint8_t *data, uint32_t len)
{
    uint32_t state;
    uint32_t written = 0U;
    uint32_t atomic_write_bytes = 0U;
    bool split_active = false;
    bool config_loaded = false;
    bool stop = false;
    ldc_pending_events_t events = {0U, 0U, 0U};

    if(!ldc || !data || len == 0U)
        return 0U;

    while(written < len && !stop)
    {
        uint32_t step_len;
        uint32_t step_written;

        state = ldc_enter(ldc);

        if(!config_loaded)
        {
            atomic_write_bytes = ldc->atomic_write_bytes;
            if(atomic_write_bytes != 0U && len > atomic_write_bytes)
            {
                ldc->write_active = 1U;
                split_active = true;
            }
            config_loaded = true;
        }

        step_len = len - written;
        if(atomic_write_bytes != 0U && step_len > atomic_write_bytes)
            step_len = atomic_write_bytes;

        step_written = ldc_write_unlocked(ldc, data + written, step_len, &events, &stop);
        written += step_written;

        ldc_leave(ldc, state);

        if(step_written == 0U)
            break;
    }

    if(split_active)
    {
        state = ldc_enter(ldc);
        ldc->write_active = 0U;
        ldc_leave(ldc, state);
    }

    ldc_notify_pending(ldc, &events);
    return written;
}

bool ldc_putc(ldc_t *ldc, uint8_t ch)
{
    return ldc_write(ldc, &ch, 1U) == 1U;
}

void ldc_tick(ldc_t *ldc, uint32_t elapsed_ms)
{
    uint32_t elapsed_us = 0U;

    if(elapsed_ms > (UINT32_MAX / 1000U))
        elapsed_us = UINT32_MAX;
    else
        elapsed_us = elapsed_ms * 1000U;

    ldc_tick_us(ldc, elapsed_us);
}

void ldc_tick_us(ldc_t *ldc, uint32_t elapsed_us)
{
    uint32_t state;
    ldc_pending_events_t events = {0U, 0U, 0U};

    if(!ldc || elapsed_us == 0U)
        return;

    state = ldc_enter(ldc);

    if(ldc->write_active != 0U)
    {
        ldc_leave(ldc, state);
        return;
    }

    if(ldc->frame_len != 0U && ldc->timeout_us != 0U)
    {
        uint32_t remain;

        /* Use remaining-time comparison so elapsed_us can cross the boundary safely. */
        if(ldc->frame_timer_us >= ldc->timeout_us)
            remain = 0U;
        else
            remain = ldc->timeout_us - ldc->frame_timer_us;

        if(elapsed_us >= remain)
        {
            (void)ldc_commit_frame_unlocked(ldc, &events);
        }
        else
        {
            ldc->frame_timer_us += elapsed_us;
        }
    }

    ldc_leave(ldc, state);
    ldc_notify_pending(ldc, &events);
}

void ldc_rx_activity(ldc_t *ldc)
{
    uint32_t state;

    if(ldc == NULL)
        return;
    state = ldc_enter(ldc);
    if(ldc->frame_len != 0U)
        ldc->frame_timer_us = 0U;
    ldc_leave(ldc, state);
}

bool ldc_flush(ldc_t *ldc)
{
    bool ret;
    uint32_t state;
    ldc_pending_events_t events = {0U, 0U, 0U};

    if(!ldc)
        return false;

    state = ldc_enter(ldc);
    ret = ldc_commit_frame_unlocked(ldc, &events);
    ldc_leave(ldc, state);
    ldc_notify_pending(ldc, &events);

    return ret;
}

bool ldc_discard_frame(ldc_t *ldc)
{
    uint32_t state;
    ldc_pending_events_t events = {0U, 0U, 0U};

    if(!ldc)
        return false;

    state = ldc_enter(ldc);
    if(ldc->frame_len == 0U)
    {
        ldc_leave(ldc, state);
        return false;
    }

    ldc->ring.head = ldc->frame_start;
    ldc->frame_len = 0U;
    ldc->frame_timer_us = 0U;
    ldc->stats.drop++;
    ldc_refresh_usage(ldc);
    events.drop++;
    ldc_leave(ldc, state);
    ldc_notify_pending(ldc, &events);
    return true;
}

int ldc_read_packet(ldc_t *ldc, uint8_t *buf, uint32_t size)
{
    ldc_packet_t pkt;
    uint32_t first;
    uint32_t state;

    if(!ldc || !buf)
        return -1;

    state = ldc_enter(ldc);

    if(!ldc_packet_peek(&ldc->packet, &pkt))
    {
        ldc_leave(ldc, state);
        return 0;
    }

    if(size < pkt.len)
    {
        /* Keep the packet queued. The caller can retry with a larger buffer. */
        ldc_leave(ldc, state);
        return -1;
    }

    /* The packet may wrap at the end of the ring, so copy it in up to two pieces. */
    first = ldc->ring.size - pkt.start;
    if(first > pkt.len)
        first = pkt.len;

    memcpy(buf, &ldc->ring.buf[pkt.start], first);

    if(pkt.len > first)
        memcpy(buf + first, &ldc->ring.buf[0], pkt.len - first);

    (void)ldc_packet_pop(&ldc->packet, &pkt);
    ldc_ring_drop(&ldc->ring, pkt.len);
    ldc_refresh_usage(ldc);

    ldc_leave(ldc, state);
    return (int)pkt.len;
}

uint16_t ldc_packet_available(ldc_t *ldc)
{
    uint16_t count;
    uint32_t state;

    if(!ldc)
        return 0U;

    state = ldc_enter(ldc);
    count = ldc_packet_count(&ldc->packet);
    ldc_leave(ldc, state);
    return count;
}

bool ldc_frame_pending(ldc_t *ldc)
{
    bool pending;
    uint32_t state;

    if(!ldc)
        return false;

    state = ldc_enter(ldc);
    pending = (ldc->frame_len != 0U);
    ldc_leave(ldc, state);
    return pending;
}

bool ldc_get_stats(ldc_t *ldc, ldc_stats_t *stats)
{
    uint32_t state;

    if(ldc == NULL || stats == NULL)
        return false;

    state = ldc_enter(ldc);
    *stats = ldc->stats;
    ldc_leave(ldc, state);
    return true;
}

void ldc_dump_stats(ldc_t *ldc)
{
    ldc_stats_t stats;
    const ldc_stats_t *s = &stats;

    if(!ldc_get_stats(ldc, &stats))
        return;

    printf("\r\n===== LDC Stats =====\r\n");
    printf("RX Bytes      : %llu\r\n", s->rx_bytes);
    printf("Packets       : %llu\r\n", s->packets);
    printf("Overflow      : %llu\r\n", s->overflow);
    printf("Drop          : %llu\r\n", s->drop);
    printf("Overwrite     : %llu\r\n", s->overwrite_count);
    printf("Ring Used     : %llu/%lu\r\n", s->cur_used, (unsigned long)ldc->ring.size);
    printf("Ring Peak     : %llu/%lu\r\n", s->max_used, (unsigned long)ldc->ring.size);
    printf("Packet Used   : %llu/%u\r\n", s->packet_used, ldc->packet.capacity);
    printf("Packet Peak   : %llu/%u\r\n", s->packet_peak, ldc->packet.capacity);
    printf("=====================\r\n");
}
