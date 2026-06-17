#ifndef LDC_CORE_H
#define LDC_CORE_H

#include <stdint.h>
#include <stdbool.h>

#include "ldc_ring.h"
#include "ldc_packet.h"

/* LDC event notification. Keep callbacks short: set a flag or wake a task. */
typedef enum
{
    LDC_EVT_PACKET = 0,
    LDC_EVT_OVERFLOW,
    LDC_EVT_DROP
} ldc_event_t;

/* Buffer policy when the ring/packet queue has no room. */
typedef enum
{
    LDC_MODE_PROTECT = 0,   /* Reject new bytes and count overflow. */
    LDC_MODE_OVERWRITE      /* Drop the oldest complete packet for new bytes. */
} ldc_mode_t;

typedef struct
{
    uint64_t rx_bytes;        /* Total received bytes. */
    uint64_t packets;         /* Complete packets committed. */
    uint64_t overflow;        /* New data rejected or packet commit failed. */
    uint64_t drop;            /* Old packets dropped in overwrite mode. */
    uint64_t overwrite_count; /* Overwrite events. */
    uint64_t max_used;        /* Ring peak usage. */
    uint64_t cur_used;        /* Current ring usage. */
    uint64_t packet_used;     /* Current packet queue usage. */
    uint64_t packet_peak;     /* Packet queue peak usage. */
} ldc_stats_t;

typedef void (*ldc_event_cb_t)(void *user, ldc_event_t evt);
typedef uint32_t (*ldc_lock_cb_t)(void *user);
typedef void (*ldc_unlock_cb_t)(void *user, uint32_t state);

typedef struct
{
    ldc_ring_t ring;             /* Raw byte storage. Packet data stays here. */
    ldc_packet_queue_t packet;   /* FIFO of packet descriptors: start + length. */

    uint32_t frame_start;        /* Current open frame start index in ring. */
    uint32_t frame_len;          /* Current open frame length. */
    uint32_t frame_timer;        /* Silent time accumulated after last byte. */

    uint32_t max_len;            /* Commit when frame_len reaches max_len; 0 disables it. */
    uint32_t timeout_ms;         /* Commit after silent timeout; 0 disables it. */
    int delimiter;               /* Commit when delimiter byte is seen; -1 disables it. */

    ldc_mode_t mode;

    ldc_event_cb_t callback;     /* Called on packet/overflow/drop. Do not parse protocol here. */
    void *callback_arg;

    ldc_lock_cb_t lock;          /* Bare metal: disable IRQ. RTOS: enter critical section. */
    ldc_unlock_cb_t unlock;
    void *lock_arg;

    ldc_stats_t stats;
} ldc_t;

void ldc_init(ldc_t *ldc,
              uint8_t *ring_buffer,
              uint32_t ring_size,
              ldc_packet_t *packet_pool,
              uint16_t packet_count);

void ldc_set_frame_config(ldc_t *ldc,
                          uint32_t max_len,
                          uint32_t timeout_ms,
                          int delimiter);

void ldc_set_mode(ldc_t *ldc, ldc_mode_t mode);
void ldc_set_callback(ldc_t *ldc, ldc_event_cb_t cb, void *arg);
void ldc_set_lock(ldc_t *ldc,
                  ldc_lock_cb_t lock,
                  ldc_unlock_cb_t unlock,
                  void *arg);

/* Block input path. Use this for DMA ReceiveToIdle chunks. */
uint32_t ldc_write(ldc_t *ldc, const uint8_t *data, uint32_t len);

/* Byte input path. Use this for RXNE/byte interrupt reception. */
bool ldc_putc(ldc_t *ldc, uint8_t ch);

/* Periodic timeout accounting. Call from SysTick, timer ISR, or an RTOS timer. */
void ldc_tick(ldc_t *ldc, uint32_t elapsed_ms);

/* Manually commit the current open frame. Useful when a hardware timer expires. */
bool ldc_flush(ldc_t *ldc);

/* Read one committed packet. Returns packet length, 0 for none, -1 for error/buffer too small. */
int ldc_read_packet(ldc_t *ldc, uint8_t *buf, uint32_t size);

uint16_t ldc_packet_available(const ldc_t *ldc);
const ldc_stats_t *ldc_get_stats(const ldc_t *ldc);
void ldc_dump_stats(ldc_t *ldc);

#endif
