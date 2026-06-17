#include "ldc_core.h"

#include <string.h>
#include <stdio.h>


//hello
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

static void ldc_notify(ldc_t *ldc, ldc_event_t evt)
{
    if(ldc->callback)
        ldc->callback(ldc->callback_arg, evt);
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

static bool ldc_drop_oldest_packet(ldc_t *ldc)
{
    ldc_packet_t old;

    if(!ldc_packet_pop(&ldc->packet, &old))
        return false;

    ldc_ring_drop(&ldc->ring, old.len);
    ldc->stats.drop++;
    ldc->stats.overwrite_count++;
    ldc_refresh_usage(ldc);
    ldc_notify(ldc, LDC_EVT_DROP);

    return true;
}

static bool ldc_commit_frame_unlocked(ldc_t *ldc)
{
    if(ldc->frame_len == 0U)
        return false;

    /* Packet queue stores only descriptors. The bytes remain in the ring until read_packet drops them. */
    while(ldc_packet_is_full(&ldc->packet))
    {
        if(ldc->mode != LDC_MODE_OVERWRITE || !ldc_drop_oldest_packet(ldc))
        {
            ldc->stats.overflow++;
            ldc_notify(ldc, LDC_EVT_OVERFLOW);
            return false;
        }
    }

    if(!ldc_packet_push(&ldc->packet, ldc->frame_start, ldc->frame_len))
    {
        ldc->stats.overflow++;
        ldc_notify(ldc, LDC_EVT_OVERFLOW);
        return false;
    }

    ldc->stats.packets++;
    ldc->frame_start = ldc->ring.head;
    ldc->frame_len = 0U;
    ldc->frame_timer = 0U;

    ldc_refresh_usage(ldc);
    ldc_notify(ldc, LDC_EVT_PACKET);

    return true;
}

void ldc_init(ldc_t *ldc, uint8_t *ring_buffer, uint32_t ring_size,
              ldc_packet_t *packet_pool, uint16_t packet_count)
{
    /* All memory is provided by the caller. LDC itself never malloc/free's memory. */
    ldc_ring_init(&ldc->ring, ring_buffer, ring_size);
    ldc_packet_init(&ldc->packet, packet_pool, packet_count);

    ldc->frame_start = 0U;
    ldc->frame_len = 0U;
    ldc->frame_timer = 0U;

    ldc->max_len = 0U;
    ldc->timeout_ms = 0U;
    ldc->delimiter = -1;
    ldc->mode = LDC_MODE_PROTECT;

    ldc->callback = NULL;
    ldc->callback_arg = NULL;
    ldc->lock = NULL;
    ldc->unlock = NULL;
    ldc->lock_arg = NULL;

    memset(&ldc->stats, 0, sizeof(ldc->stats));
}

void ldc_set_frame_config(ldc_t *ldc, uint32_t max_len, uint32_t timeout_ms, int delimiter)
{
    uint32_t state = ldc_enter(ldc);

    ldc->max_len = max_len;
    ldc->timeout_ms = timeout_ms;
    ldc->delimiter = delimiter;

    ldc_leave(ldc, state);
}

void ldc_set_mode(ldc_t *ldc, ldc_mode_t mode)
{
    uint32_t state = ldc_enter(ldc);

    ldc->mode = mode;

    ldc_leave(ldc, state);
}

void ldc_set_callback(ldc_t *ldc, ldc_event_cb_t cb, void *arg)
{
    uint32_t state = ldc_enter(ldc);

    ldc->callback = cb;
    ldc->callback_arg = arg;

    ldc_leave(ldc, state);
}

void ldc_set_lock(ldc_t *ldc, ldc_lock_cb_t lock, ldc_unlock_cb_t unlock, void *arg)
{
    ldc->lock = lock;
    ldc->unlock = unlock;
    ldc->lock_arg = arg;
}

uint32_t ldc_write(ldc_t *ldc, const uint8_t *data, uint32_t len)
{
    uint32_t state;
    uint32_t written = 0U;

    if(!ldc || !data || len == 0U)
        return 0U;

    state = ldc_enter(ldc);

    for(uint32_t i = 0U; i < len; i++)
    {
        uint8_t ch = data[i];

        /* Keep one byte empty in the ring so head == tail can mean empty. */
        while(ldc_ring_free(&ldc->ring) == 0U)
        {
            if(ldc->mode != LDC_MODE_OVERWRITE || !ldc_drop_oldest_packet(ldc))
            {
                ldc->stats.overflow++;
                ldc_notify(ldc, LDC_EVT_OVERFLOW);
                ldc_leave(ldc, state);
                return written;
            }
        }

        if(ldc->frame_len == 0U)
            ldc->frame_start = ldc->ring.head;

        if(ldc_ring_write(&ldc->ring, &ch, 1U) != 1U)
            break;

        written++;
        ldc->stats.rx_bytes++;
        ldc->frame_len++;
        ldc->frame_timer = 0U;
        ldc_refresh_usage(ldc);

        /* Three supported frame-ending strategies: delimiter, fixed max length, or timeout via ldc_tick. */
        if(ldc->delimiter >= 0 && ch == (uint8_t)ldc->delimiter)
        {
            (void)ldc_commit_frame_unlocked(ldc);
        }
        else if(ldc->max_len != 0U && ldc->frame_len >= ldc->max_len)
        {
            (void)ldc_commit_frame_unlocked(ldc);
        }
    }

    ldc_leave(ldc, state);
    return written;
}

bool ldc_putc(ldc_t *ldc, uint8_t ch)
{
    return ldc_write(ldc, &ch, 1U) == 1U;
}

void ldc_tick(ldc_t *ldc, uint32_t elapsed_ms)
{
    uint32_t state;

    if(!ldc || elapsed_ms == 0U)
        return;

    state = ldc_enter(ldc);

    if(ldc->frame_len != 0U && ldc->timeout_ms != 0U)
    {
        uint32_t remain;

        /* Use remaining-time comparison so elapsed_ms can cross the boundary safely. */
        if(ldc->frame_timer >= ldc->timeout_ms)
            remain = 0U;
        else
            remain = ldc->timeout_ms - ldc->frame_timer;

        if(elapsed_ms >= remain)
        {
            (void)ldc_commit_frame_unlocked(ldc);
        }
        else
        {
            ldc->frame_timer += elapsed_ms;
        }
    }

    ldc_leave(ldc, state);
}

bool ldc_flush(ldc_t *ldc)
{
    bool ret;
    uint32_t state;

    if(!ldc)
        return false;

    state = ldc_enter(ldc);
    ret = ldc_commit_frame_unlocked(ldc);
    ldc_leave(ldc, state);

    return ret;
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

uint16_t ldc_packet_available(const ldc_t *ldc)
{
    return ldc_packet_count(&ldc->packet);
}

const ldc_stats_t *ldc_get_stats(const ldc_t *ldc)
{
    return &ldc->stats;
}

void ldc_dump_stats(ldc_t *ldc)
{
    const ldc_stats_t *s = &ldc->stats;

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
