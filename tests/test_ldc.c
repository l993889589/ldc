#include "ldc_easy.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    uint32_t packet;
    uint32_t overflow;
    uint32_t drop;
} test_events_t;

static void test_event(ldc_easy_t *queue, ldc_easy_event_t event, void *argument)
{
    test_events_t *events = (test_events_t *)argument;

    (void)queue;
    if(event == LDC_EASY_EVT_PACKET)
        events->packet++;
    else if(event == LDC_EASY_EVT_OVERFLOW)
        events->overflow++;
    else
        events->drop++;
}

static void test_timeout_framing(void)
{
    ldc_easy_t queue;
    uint8_t ring[LDC_EASY_RING_BYTES(256U, 4U)];
    ldc_packet_t packets[4];
    ldc_easy_config_t config = {0};
    test_events_t events = {0};
    const uint8_t input[] = {0x01U, 0x03U, 0x00U, 0x00U};
    uint8_t output[16];

    config.ring_buffer = ring;
    config.ring_size = sizeof(ring);
    config.packet_pool = packets;
    config.packet_count = 4U;
    config.max_frame = 256U;
    config.timeout_us = 1750U;
    config.mode = LDC_MODE_PROTECT;
    config.event_cb = test_event;
    config.event_arg = &events;

    assert(ldc_easy_init(&queue, &config));
    assert(ldc_easy_add(&queue, input, sizeof(input)) == sizeof(input));
    ldc_easy_tick_us(&queue, 1749U);
    assert(ldc_easy_available(&queue) == 0U);
    ldc_easy_tick_us(&queue, 1U);
    assert(ldc_easy_available(&queue) == 1U);
    assert(events.packet == 1U);
    assert(ldc_easy_pop(&queue, output, sizeof(output)) == (int)sizeof(input));
    assert(memcmp(input, output, sizeof(input)) == 0);
}

static void test_receive_idle_commit(void)
{
    ldc_easy_t queue;
    uint8_t ring[33];
    ldc_packet_t packets[2];
    ldc_easy_config_t config = {0};
    const uint8_t input[] = {0x10U, 0x20U, 0x30U};
    uint8_t output[8];

    config.ring_buffer = ring;
    config.ring_size = sizeof(ring);
    config.packet_pool = packets;
    config.packet_count = 2U;
    config.mode = LDC_MODE_PROTECT;

    assert(ldc_easy_init(&queue, &config));
    assert(ldc_easy_rx_idle(&queue, input, sizeof(input)) == sizeof(input));
    assert(ldc_easy_pop(&queue, output, sizeof(output)) == (int)sizeof(input));
    assert(memcmp(input, output, sizeof(input)) == 0);
}

static void test_delimiter_and_max_length(void)
{
    ldc_easy_t delimiter_queue;
    ldc_easy_t length_queue;
    uint8_t delimiter_ring[33];
    uint8_t length_ring[33];
    ldc_packet_t delimiter_packets[4];
    ldc_packet_t length_packets[4];
    ldc_easy_config_t config = {0};
    const uint8_t lines[] = {'A', '\n', 'B', '\n'};
    const uint8_t fixed[] = {1U, 2U, 3U, 4U, 5U, 6U};
    uint8_t output[8];

    config.ring_buffer = delimiter_ring;
    config.ring_size = sizeof(delimiter_ring);
    config.packet_pool = delimiter_packets;
    config.packet_count = 4U;
    config.delimiter_enabled = true;
    config.delimiter = (uint8_t)'\n';
    config.mode = LDC_MODE_PROTECT;
    assert(ldc_easy_init(&delimiter_queue, &config));
    assert(ldc_easy_add(&delimiter_queue, lines, sizeof(lines)) == sizeof(lines));
    assert(ldc_easy_available(&delimiter_queue) == 2U);
    assert(ldc_easy_pop(&delimiter_queue, output, sizeof(output)) == 2);
    assert(output[0] == 'A' && output[1] == '\n');
    assert(ldc_easy_pop(&delimiter_queue, output, sizeof(output)) == 2);
    assert(output[0] == 'B' && output[1] == '\n');

    memset(&config, 0, sizeof(config));
    config.ring_buffer = length_ring;
    config.ring_size = sizeof(length_ring);
    config.packet_pool = length_packets;
    config.packet_count = 4U;
    config.max_frame = 3U;
    config.mode = LDC_MODE_PROTECT;
    assert(ldc_easy_init(&length_queue, &config));
    assert(ldc_easy_add(&length_queue, fixed, sizeof(fixed)) == sizeof(fixed));
    assert(ldc_easy_available(&length_queue) == 2U);
    assert(ldc_easy_pop(&length_queue, output, sizeof(output)) == 3);
    assert(memcmp(output, fixed, 3U) == 0);
    assert(ldc_easy_pop(&length_queue, output, sizeof(output)) == 3);
    assert(memcmp(output, fixed + 3U, 3U) == 0);
}

static void test_overwrite_policy(void)
{
    ldc_easy_t queue;
    uint8_t ring[9];
    ldc_packet_t packets[1];
    ldc_easy_config_t config = {0};
    test_events_t events = {0};
    const uint8_t first[] = {1U, 1U, 1U, 1U};
    const uint8_t second[] = {2U, 2U, 2U, 2U};
    uint8_t output[8];
    ldc_stats_t stats;

    config.ring_buffer = ring;
    config.ring_size = sizeof(ring);
    config.packet_pool = packets;
    config.packet_count = 1U;
    config.max_frame = 4U;
    config.mode = LDC_MODE_OVERWRITE;
    config.event_cb = test_event;
    config.event_arg = &events;
    assert(ldc_easy_init(&queue, &config));

    assert(ldc_easy_add(&queue, first, sizeof(first)) == sizeof(first));
    assert(ldc_easy_add(&queue, second, sizeof(second)) == sizeof(second));
    assert(ldc_easy_available(&queue) == 1U);
    assert(ldc_easy_pop(&queue, output, sizeof(output)) == (int)sizeof(second));
    assert(memcmp(output, second, sizeof(second)) == 0);
    assert(events.drop == 1U);
    assert(ldc_easy_get_stats(&queue, &stats));
    assert(stats.drop == 1U);
    assert(stats.overwrite_count == 1U);
}

int main(void)
{
    test_timeout_framing();
    test_receive_idle_commit();
    test_delimiter_and_max_length();
    test_overwrite_policy();
    puts("LDC host tests passed");
    return 0;
}

