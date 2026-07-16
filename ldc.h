/**
 * @file ldc.h
 * @brief Static-memory UART frame collector with bounded critical sections.
 *
 * LDC is a transport-neutral byte-to-frame collector intended for UART, AT
 * command and small private protocols.  It does not allocate memory, own a
 * timer, register global objects, call an RTOS, or include an MCU HAL.
 *
 * Concurrency contract:
 * - exactly one producer calls the ldc_rx_* functions;
 * - exactly one consumer calls the ldc_frame_* functions;
 * - producer and consumer may run concurrently when a matching lock/unlock
 *   pair is supplied;
 * - lifecycle functions require producer and consumer to be quiescent.
 *
 * A received block is transactional: ldc_rx_write() consumes all bytes or no
 * bytes.  Complete frames never become visible until the whole block has been
 * copied.  Frame payload copies performed by ldc_frame_read() occur outside
 * the caller-supplied critical section.
 */

#ifndef LDC_H
#define LDC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LDC_VERSION_MAJOR 2U
#define LDC_VERSION_MINOR 0U
#define LDC_VERSION_PATCH 2U

#define LDC_DELIMITER_MAX 8U
#define LDC_SLOT_COUNT_MAX 64U
#define LDC_CONTEXT_INITIALIZER {0}

/*
 * A migration target may define LDC_SYMBOL_PREFIX (for example ldc_vnext_)
 * before including this header.  The default build keeps the concise ldc_*
 * public symbols; the prefix exists only so old and new generations can be
 * linked during a controlled migration.
 */
#define LDC_JOIN_INNER(left, right) left##right
#define LDC_JOIN(left, right) LDC_JOIN_INNER(left, right)
#if defined(LDC_SYMBOL_PREFIX)
#define LDC_PUBLIC_SYMBOL(name) LDC_JOIN(LDC_SYMBOL_PREFIX, name)
#define ldc_init LDC_PUBLIC_SYMBOL(init)
#define ldc_reset LDC_PUBLIC_SYMBOL(reset)
#define ldc_deinit LDC_PUBLIC_SYMBOL(deinit)
#define ldc_rx_write LDC_PUBLIC_SYMBOL(rx_write)
#define ldc_rx_byte LDC_PUBLIC_SYMBOL(rx_byte)
#define ldc_rx_commit LDC_PUBLIC_SYMBOL(rx_commit)
#define ldc_rx_idle LDC_PUBLIC_SYMBOL(rx_idle)
#define ldc_rx_abort LDC_PUBLIC_SYMBOL(rx_abort)
#define ldc_frame_claim LDC_PUBLIC_SYMBOL(frame_claim)
#define ldc_frame_release LDC_PUBLIC_SYMBOL(frame_release)
#define ldc_frame_read LDC_PUBLIC_SYMBOL(frame_read)
#define ldc_get_stats LDC_PUBLIC_SYMBOL(get_stats)
#endif

/** @brief Status returned by LDC operations. */
typedef enum
{
    LDC_STATUS_OK = 0,
    LDC_STATUS_EMPTY,
    LDC_STATUS_BUSY,
    LDC_STATUS_FULL,
    LDC_STATUS_BUFFER_TOO_SMALL,
    LDC_STATUS_INVALID_ARGUMENT,
    LDC_STATUS_NOT_INITIALIZED,
    LDC_STATUS_ALREADY_INITIALIZED,
    LDC_STATUS_STALE_VIEW,
    LDC_STATUS_DATA_LOSS
} ldc_status_t;

/** @brief Automatic frame boundary mode. */
typedef enum
{
    LDC_FRAME_MODE_MANUAL = 0,
    LDC_FRAME_MODE_DELIMITER,
    LDC_FRAME_MODE_FIXED_LENGTH
} ldc_frame_mode_t;

/** @brief Behavior when no free frame slot is available. */
typedef enum
{
    LDC_FULL_REJECT_NEW = 0,
    LDC_FULL_DROP_OLDEST
} ldc_full_policy_t;

/** @brief Opaque interrupt/lock state returned by the application lock hook. */
typedef uintptr_t ldc_lock_state_t;

/**
 * @brief Enter the application-owned producer/consumer critical section.
 * @param argument Application context from ldc_config_t.
 * @return State passed unchanged to ldc_unlock_fn.
 */
typedef ldc_lock_state_t (*ldc_lock_fn)(void *argument);

/**
 * @brief Leave the application-owned producer/consumer critical section.
 * @param argument Application context from ldc_config_t.
 * @param state State returned by the matching lock call.
 */
typedef void (*ldc_unlock_fn)(void *argument, ldc_lock_state_t state);

/**
 * @brief Per-frame metadata supplied by the caller.
 *
 * Treat every member as private.  The complete definition is public only so
 * applications can allocate a fixed array without dynamic memory.
 */
typedef struct
{
    uint8_t *data;
    size_t length;
    uint32_t sequence;
    uint32_t generation;
    uint64_t claim_token;
    uint16_t reservation_order;
    uint16_t pending_order;
    uint8_t state;
    uint8_t pending_state;
} ldc_slot_t;

/** @brief Immutable initialization parameters. */
typedef struct
{
    uint8_t *storage;
    size_t storage_size;
    ldc_slot_t *slots;
    uint16_t slot_count;
    size_t frame_capacity;
    ldc_frame_mode_t frame_mode;
    ldc_full_policy_t full_policy;
    const uint8_t *delimiter;
    uint8_t delimiter_length;
    uint8_t include_delimiter;
    uint8_t emit_empty_frames;
    size_t fixed_length;
    ldc_lock_fn lock;
    ldc_unlock_fn unlock;
    void *lock_argument;
} ldc_config_t;

/** @brief Result of a producer-side write or boundary operation. */
typedef struct
{
    ldc_status_t status;
    size_t accepted_bytes;
    size_t rejected_bytes;
    uint16_t committed_frames;
    uint16_t dropped_frames;
    size_t dropped_bytes;
    uint16_t overflow_frames;
    size_t overflow_bytes;
} ldc_write_result_t;

/** @brief Read-only zero-copy frame view returned to the consumer. */
typedef struct
{
    const uint8_t *data;
    size_t length;
    uint32_t sequence;
    uint32_t generation;
    uint64_t claim_token;
    uint16_t slot_index;
} ldc_frame_view_t;

/** @brief Monotonic diagnostics snapshot; counters wrap naturally. */
typedef struct
{
    uint64_t accepted_bytes;
    uint64_t rejected_bytes;
    uint64_t committed_frames;
    uint64_t consumed_frames;
    uint64_t consumed_bytes;
    uint64_t dropped_frames;
    uint64_t dropped_bytes;
    uint64_t overflow_frames;
    uint64_t overflow_bytes;
    uint64_t aborted_frames;
    uint64_t aborted_bytes;
    uint64_t busy_calls;
    uint16_t ready_frames;
    uint16_t claimed_frames;
    uint16_t ready_high_watermark;
    uint16_t used_high_watermark;
    uint8_t discarding_overflow_frame;
} ldc_stats_t;

/**
 * @brief LDC object allocated by the caller.
 *
 * Initialize static objects with LDC_CONTEXT_INITIALIZER.  Treat members as
 * private; their definition is visible solely to permit static allocation.
 */
typedef struct
{
    uint32_t magic;
    uint8_t *storage;
    size_t storage_size;
    ldc_slot_t *slots;
    uint16_t slot_count;
    size_t frame_capacity;
    ldc_frame_mode_t frame_mode;
    ldc_full_policy_t full_policy;
    uint8_t delimiter[LDC_DELIMITER_MAX];
    uint8_t delimiter_prefix[LDC_DELIMITER_MAX];
    uint8_t delimiter_length;
    uint8_t include_delimiter;
    uint8_t emit_empty_frames;
    size_t fixed_length;
    ldc_lock_fn lock;
    ldc_unlock_fn unlock;
    void *lock_argument;
    uint16_t current_slot;
    uint16_t ready_count;
    uint16_t claimed_count;
    size_t delimiter_match;
    uint32_t next_sequence;
    uint64_t next_claim_token;
    uint8_t discarding;
    uint8_t write_active;
    ldc_stats_t stats;
} ldc_t;

/**
 * @brief Initialize an LDC object and bind caller-owned storage.
 * @param ldc Initially zero-initialized object that remains valid across
 * deinit/reinit cycles.
 * @param config Configuration and storage that remain valid until deinit.
 * @return OK, ALREADY_INITIALIZED, or INVALID_ARGUMENT.
 * @note Context, slot metadata, and payload storage are exclusive LDC-owned
 * regions and must not overlap. Lifecycle calls require quiescence. Do not
 * clear or rewrite the context after deinit; LDC retains a private token seed
 * so stale views cannot match a later lifecycle.
 */
ldc_status_t ldc_init(ldc_t *ldc, const ldc_config_t *config);

/**
 * @brief Clear queued data while retaining configuration.
 * @param ldc Initialized object.
 * @return OK, BUSY when a write/view is active, or NOT_INITIALIZED.
 * @note Lifecycle call; producer and consumer must be quiescent.
 */
ldc_status_t ldc_reset(ldc_t *ldc);

/**
 * @brief Release the object's configuration without freeing caller memory.
 * @param ldc Initialized object.
 * @return OK, BUSY when a write/view is active, or NOT_INITIALIZED.
 * @note Lifecycle call; producer and consumer must be quiescent.
 */
ldc_status_t ldc_deinit(ldc_t *ldc);

/**
 * @brief Transactionally append one producer block.
 * @param ldc Initialized object.
 * @param data Bytes valid for the duration of this call. The input must not
 * overlap the LDC context, slot metadata, or payload storage.
 * @param length Number of bytes; zero is accepted with no side effects.
 * @return Detailed consumption, publication, drop, and overflow result.
 * @note Single-producer API.  ISR safety depends on the supplied lock hooks
 * and on the caller accepting work proportional to length.
 */
ldc_write_result_t ldc_rx_write(ldc_t *ldc,
                                const uint8_t *data,
                                size_t length);

/**
 * @brief Transactionally append one byte.
 * @param ldc Initialized object.
 * @param byte Byte received by the producer.
 * @return Detailed write result.
 * @note Single-producer API and suitable for bounded byte-at-a-time ISR use.
 */
ldc_write_result_t ldc_rx_byte(ldc_t *ldc, uint8_t byte);

/**
 * @brief Publish the current partial frame at an explicit boundary.
 * @param ldc Initialized object.
 * @return Result with committed_frames set to one on publication.
 * @note Single-producer API.  An overflow-discard state is cleared without
 * publishing a truncated frame.
 */
ldc_write_result_t ldc_rx_commit(ldc_t *ldc);

/**
 * @brief Publish a partial frame when the UART reports an idle boundary.
 * @param ldc Initialized object.
 * @return Same contract as ldc_rx_commit().
 * @note LDC does not measure time; the UART/timer owner decides when idle is
 * a valid protocol boundary.
 */
ldc_write_result_t ldc_rx_idle(ldc_t *ldc);

/**
 * @brief Discard the current partial or overflowed frame.
 * @param ldc Initialized object.
 * @return OK, BUSY, or NOT_INITIALIZED.
 * @note Single-producer API.
 */
ldc_status_t ldc_rx_abort(ldc_t *ldc);

/**
 * @brief Claim the oldest frame without copying it.
 * @param ldc Initialized object.
 * @param view Receives a view valid until release.
 * @return OK, EMPTY, INVALID_ARGUMENT, or NOT_INITIALIZED.
 * @note Single-consumer API.  A claimed slot is pinned and cannot be dropped
 * by the producer.
 */
ldc_status_t ldc_frame_claim(ldc_t *ldc, ldc_frame_view_t *view);

/**
 * @brief Consume and release a previously claimed frame.
 * @param ldc Initialized object.
 * @param view View returned by ldc_frame_claim(); cleared on success.
 * @return OK, STALE_VIEW, INVALID_ARGUMENT, or NOT_INITIALIZED.
 * @note Single-consumer API.
 */
ldc_status_t ldc_frame_release(ldc_t *ldc, ldc_frame_view_t *view);

/**
 * @brief Copy and consume the oldest frame with memcpy outside the lock.
 * @param ldc Initialized object.
 * @param destination Output buffer valid for the duration of this call. It
 * must not overlap the LDC context, slot metadata, or payload storage.
 * @param capacity Output capacity in bytes.
 * @param length Receives required/actual frame length. It must not overlap
 * destination or any memory exclusively owned by LDC.
 * @return OK, EMPTY, BUFFER_TOO_SMALL, or an argument/lifecycle error.
 * @note On BUFFER_TOO_SMALL the frame remains queued and *length reports the
 * required capacity. On INVALID_ARGUMENT caused by output aliasing, the frame
 * remains queued and *length is not modified.
 */
ldc_status_t ldc_frame_read(ldc_t *ldc,
                            uint8_t *destination,
                            size_t capacity,
                            size_t *length);

/**
 * @brief Snapshot diagnostics under the caller-owned critical section.
 * @param ldc Initialized object.
 * @param stats Receives the snapshot.
 * @return OK, INVALID_ARGUMENT, or NOT_INITIALIZED.
 */
ldc_status_t ldc_get_stats(ldc_t *ldc, ldc_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif
