#ifndef LDC_EASY_H
#define LDC_EASY_H

#include <stdbool.h>
#include <stdint.h>

#include "ldc_core.h"

/*
 * Thin convenience wrapper over ldc_core.
 *
 * Common byte interrupt model:
 *   ldc_easy_putc(&q, byte);       // RX interrupt/callback
 *   ldc_easy_tick(&q, 1U);         // 1 ms SysTick or timer
 *   len = ldc_easy_pop(&q, buf, sizeof(buf));
 *
 * UART ReceiveToIdle DMA/IT model:
 *   ldc_easy_rx_idle(&q, data, len);   // RX idle callback; commits the frame
 *   len = ldc_easy_pop(&q, buf, sizeof(buf));
 *
 * This layer has no HAL, RTOS, malloc, or global tick dependency.
 */

#define LDC_EASY_RING_BYTES(max_frame, packet_count) \
    (((uint32_t)(max_frame) * (uint32_t)(packet_count)) + 1U)

typedef enum
{
    LDC_EASY_EVT_PACKET = 0,
    LDC_EASY_EVT_OVERFLOW,
    LDC_EASY_EVT_DROP
} ldc_easy_event_t;

typedef struct ldc_easy ldc_easy_t;

typedef void (*ldc_easy_event_cb_t)(ldc_easy_t *queue,
                                    ldc_easy_event_t event,
                                    void *arg);

typedef struct
{
    uint8_t *ring_buffer;
    uint32_t ring_size;
    ldc_packet_t *packet_pool;
    uint16_t packet_count;

    uint32_t max_frame;       /* Commit when this length is reached; 0 disables it. */
    uint32_t timeout_ms;      /* Idle timeout in ms; ignored when timeout_us != 0. */
    uint32_t timeout_us;      /* Idle timeout in us; 0 means use timeout_ms. */

    bool delimiter_enabled;   /* Zero-initialized config disables delimiter framing. */
    uint8_t delimiter;

    ldc_mode_t mode;

    ldc_lock_cb_t lock;       /* Optional: disable IRQ or enter critical section. */
    ldc_unlock_cb_t unlock;
    void *lock_arg;

    ldc_easy_event_cb_t event_cb;
    void *event_arg;

    bool auto_tick;            /* If true, ldc_easy_tick_all() will tick this queue. */
} ldc_easy_config_t;

struct ldc_easy
{
    ldc_t ldc;
    ldc_easy_event_cb_t event_cb;
    void *event_arg;
    struct ldc_easy *tick_next;
    uint8_t initialized;
    uint8_t tick_registered;
};

bool ldc_easy_init(ldc_easy_t *queue, const ldc_easy_config_t *config);

/* Add bytes without committing a frame. Use with tick timeout, delimiter, or max_frame. */
uint32_t ldc_easy_add(ldc_easy_t *queue, const uint8_t *data, uint32_t len);
bool ldc_easy_putc(ldc_easy_t *queue, uint8_t byte);

/*
 * Add bytes from a hardware idle event and commit the open frame immediately.
 * If only part of the block was accepted, the partial open frame is discarded.
 */
uint32_t ldc_easy_rx_idle(ldc_easy_t *queue, const uint8_t *data, uint32_t len);

void ldc_easy_tick(ldc_easy_t *queue, uint32_t elapsed_ms);
void ldc_easy_tick_us(ldc_easy_t *queue, uint32_t elapsed_us);
bool ldc_easy_tick_register(ldc_easy_t *queue);
void ldc_easy_tick_unregister(ldc_easy_t *queue);
void ldc_easy_tick_all(uint32_t elapsed_ms);
void ldc_easy_tick_all_us(uint32_t elapsed_us);
bool ldc_easy_settle(ldc_easy_t *queue);
bool ldc_easy_abort(ldc_easy_t *queue);

/* Return frame length, 0 for no packet, -1 for invalid args or buffer too small. */
int ldc_easy_pop(ldc_easy_t *queue, uint8_t *buf, uint32_t size);

uint16_t ldc_easy_available(ldc_easy_t *queue);
bool ldc_easy_frame_pending(ldc_easy_t *queue);
bool ldc_easy_get_stats(ldc_easy_t *queue, ldc_stats_t *stats);
ldc_t *ldc_easy_core(ldc_easy_t *queue);

#endif /* LDC_EASY_H */
