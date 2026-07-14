# LDC

LDC is a small static-memory byte-to-frame queue for embedded transports.
It has no HAL, RTOS, heap, or global clock dependency.

The caller owns the ring buffer and packet descriptors. Received bytes can be
framed by an idle timeout, delimiter, fixed maximum length, or a hardware idle
event. Optional lock callbacks make one queue safe between an RX interrupt and
one application task.

## Files

- `ldc_core.*`: transport-independent ring and frame ownership.
- `ldc_easy.*`: convenience API for byte IRQ and ReceiveToIdle integrations.
- `ldc_ring.*`: byte ring implementation.
- `ldc_packet.*`: complete-frame descriptor queue.

## Byte-interrupt example

```c
static ldc_easy_t rx_queue;
static uint8_t rx_ring[LDC_EASY_RING_BYTES(256U, 8U)];
static ldc_packet_t rx_packets[8];

ldc_easy_config_t config = {0};
config.ring_buffer = rx_ring;
config.ring_size = sizeof(rx_ring);
config.packet_pool = rx_packets;
config.packet_count = 8U;
config.max_frame = 256U;
config.timeout_us = 1750U;
config.mode = LDC_MODE_PROTECT;

ldc_easy_init(&rx_queue, &config);
```

Feed bytes with `ldc_easy_add()`, advance silent time with
`ldc_easy_tick_us()`, and consume complete frames with `ldc_easy_pop()`.
Protocol parsing belongs to the application task, never the UART interrupt.

## Host test

```powershell
cmake -S . -B build-mingw -G 'MinGW Makefiles' -DCMAKE_C_COMPILER=C:/MinGW/bin/gcc.exe
cmake --build build-mingw
ctest --test-dir build-mingw --output-on-failure
```

This GitHub repository preserves the history of the earlier Gitee-hosted LDC
core. GitHub is the active publication target for current STM32 work.
