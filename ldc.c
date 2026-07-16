/**
 * @file ldc.c
 * @brief Two-file static-memory implementation of the LDC frame collector.
 */

#include "ldc.h"

#include <limits.h>
#include <string.h>

#define LDC_MAGIC 0x4C444332UL
#define LDC_DEINIT_MAGIC 0x4C444344UL
#define LDC_SLOT_NONE UINT16_MAX
#define LDC_ORDER_NONE UINT16_MAX

#define LDC_SLOT_FREE 0U
#define LDC_SLOT_OPEN 1U
#define LDC_SLOT_READY 2U
#define LDC_SLOT_CLAIMED 3U
#define LDC_SLOT_RESERVED 4U

typedef struct
{
    size_t committed_frames;
    size_t overflow_frames;
    size_t overflow_bytes;
    size_t live_slots;
    size_t additional_slots;
    size_t final_match;
    uint8_t final_open;
    uint8_t final_discarding;
} ldc_plan_t;

/** @brief Convert one non-empty memory region to checked integer bounds. */
static uint8_t ldc_region_bounds(const void *address,
                                 size_t length,
                                 uintptr_t *begin,
                                 uintptr_t *end)
{
    uintptr_t first;

    if ((address == NULL) || (length == 0U) ||
        (begin == NULL) || (end == NULL))
    {
        return 0U;
    }

    first = (uintptr_t)address;
    if (length > (size_t)(UINTPTR_MAX - first))
    {
        return 0U;
    }

    *begin = first;
    *end = first + (uintptr_t)length;
    return 1U;
}

/** @brief Validate that a non-empty region has representable bounds. */
static uint8_t ldc_region_is_valid(const void *address, size_t length)
{
    uintptr_t begin;
    uintptr_t end;

    return ldc_region_bounds(address, length, &begin, &end);
}

/** @brief Test two checked, non-empty memory regions for overlap. */
static uint8_t ldc_regions_overlap(const void *first_address,
                                   size_t first_length,
                                   const void *second_address,
                                   size_t second_length)
{
    uintptr_t first_begin;
    uintptr_t first_end;
    uintptr_t second_begin;
    uintptr_t second_end;

    if ((ldc_region_bounds(first_address, first_length,
                           &first_begin, &first_end) == 0U) ||
        (ldc_region_bounds(second_address, second_length,
                           &second_begin, &second_end) == 0U))
    {
        return 1U;
    }

    return (uint8_t)((first_begin < second_end) &&
                     (second_begin < first_end));
}

/** @brief Reject buffers that alias any memory exclusively owned by LDC. */
static uint8_t ldc_buffer_overlaps_owned_memory(const ldc_t *ldc,
                                                const void *buffer,
                                                size_t length)
{
    size_t slot_bytes = sizeof(ldc->slots[0]) * ldc->slot_count;
    size_t payload_bytes = ldc->frame_capacity * ldc->slot_count;

    if (length == 0U)
    {
        return 0U;
    }

    return (uint8_t)(
        (ldc_regions_overlap(buffer, length, ldc, sizeof(*ldc)) != 0U) ||
        (ldc_regions_overlap(buffer, length,
                             ldc->slots, slot_bytes) != 0U) ||
        (ldc_regions_overlap(buffer, length,
                             ldc->storage, payload_bytes) != 0U));
}

/** @brief Allocate a practically non-repeating identity for one publication. */
static uint64_t ldc_allocate_claim_token_unlocked(ldc_t *ldc)
{
    uint64_t token = ldc->next_claim_token;

    if (token == 0U)
    {
        token = 1U;
    }
    ldc->next_claim_token = token + 1U;
    if (ldc->next_claim_token == 0U)
    {
        ldc->next_claim_token = 1U;
    }
    return token;
}

/** @brief Return an all-zero successful producer result. */
static ldc_write_result_t ldc_write_result_ok(void)
{
    ldc_write_result_t result;

    (void)memset(&result, 0, sizeof(result));
    result.status = LDC_STATUS_OK;
    return result;
}

/** @brief Enter the caller-supplied lock when one is configured. */
static ldc_lock_state_t ldc_lock(ldc_t *ldc)
{
    if (ldc->lock != NULL)
    {
        return ldc->lock(ldc->lock_argument);
    }

    return (ldc_lock_state_t)0U;
}

/** @brief Leave the caller-supplied lock when one is configured. */
static void ldc_unlock(ldc_t *ldc, ldc_lock_state_t state)
{
    if (ldc->unlock != NULL)
    {
        ldc->unlock(ldc->lock_argument, state);
    }
}

/** @brief Test whether an object has completed initialization. */
static uint8_t ldc_is_initialized(const ldc_t *ldc)
{
    return (uint8_t)((ldc != NULL) && (ldc->magic == LDC_MAGIC));
}

/** @brief Build the KMP prefix table for the copied delimiter. */
static void ldc_build_delimiter_prefix(ldc_t *ldc)
{
    size_t index;
    size_t matched = 0U;

    ldc->delimiter_prefix[0] = 0U;
    for (index = 1U; index < ldc->delimiter_length; index++)
    {
        while ((matched > 0U) &&
               (ldc->delimiter[index] != ldc->delimiter[matched]))
        {
            matched = ldc->delimiter_prefix[matched - 1U];
        }

        if (ldc->delimiter[index] == ldc->delimiter[matched])
        {
            matched++;
        }
        ldc->delimiter_prefix[index] = (uint8_t)matched;
    }
}

/** @brief Advance delimiter matching by one byte, including overlap support. */
static size_t ldc_delimiter_advance(const ldc_t *ldc,
                                    size_t matched,
                                    uint8_t byte,
                                    uint8_t *complete)
{
    *complete = 0U;

    while ((matched > 0U) && (byte != ldc->delimiter[matched]))
    {
        matched = ldc->delimiter_prefix[matched - 1U];
    }

    if (byte == ldc->delimiter[matched])
    {
        matched++;
    }

    if (matched == ldc->delimiter_length)
    {
        *complete = 1U;
        matched = ldc->delimiter_prefix[matched - 1U];
    }

    return matched;
}

/** @brief Compare bounded in-flight sequence numbers across wrap-around. */
static uint8_t ldc_sequence_is_older(uint32_t candidate, uint32_t reference)
{
    return (uint8_t)(((int32_t)(candidate - reference)) < 0);
}

/** @brief Find the oldest ready slot; caller holds the configured lock. */
static uint16_t ldc_find_oldest_ready_unlocked(const ldc_t *ldc)
{
    uint16_t index;
    uint16_t oldest = LDC_SLOT_NONE;

    for (index = 0U; index < ldc->slot_count; index++)
    {
        const ldc_slot_t *slot = &ldc->slots[index];

        if (slot->state != LDC_SLOT_READY)
        {
            continue;
        }

        if ((oldest == LDC_SLOT_NONE) ||
            (ldc_sequence_is_older(slot->sequence,
                                   ldc->slots[oldest].sequence) != 0U))
        {
            oldest = index;
        }
    }

    return oldest;
}

/** @brief Count slots in a requested internal state. */
static uint16_t ldc_count_slots_unlocked(const ldc_t *ldc, uint8_t state)
{
    uint16_t index;
    uint16_t count = 0U;

    for (index = 0U; index < ldc->slot_count; index++)
    {
        if (ldc->slots[index].state == state)
        {
            count++;
        }
    }

    return count;
}

/** @brief Update queue watermarks after a state transition. */
static void ldc_update_watermarks_unlocked(ldc_t *ldc)
{
    uint16_t used = (uint16_t)(ldc->ready_count + ldc->claimed_count);

    if (ldc->current_slot != LDC_SLOT_NONE)
    {
        used++;
    }

    if (ldc->ready_count > ldc->stats.ready_high_watermark)
    {
        ldc->stats.ready_high_watermark = ldc->ready_count;
    }
    if (used > ldc->stats.used_high_watermark)
    {
        ldc->stats.used_high_watermark = used;
    }
}

/** @brief Simulate one complete write without modifying payload or queue state. */
static ldc_plan_t ldc_plan_write(const ldc_t *ldc,
                                 const uint8_t *data,
                                 size_t length)
{
    ldc_plan_t plan;
    size_t open_length = 0U;
    size_t matched = ldc->delimiter_match;
    size_t index;
    uint8_t has_initial_slot =
        (uint8_t)(ldc->current_slot != LDC_SLOT_NONE);
    uint8_t has_slot = has_initial_slot;
    uint8_t discarding = ldc->discarding;

    (void)memset(&plan, 0, sizeof(plan));
    if (has_initial_slot != 0U)
    {
        open_length = ldc->slots[ldc->current_slot].length;
    }

    for (index = 0U; index < length; index++)
    {
        uint8_t delimiter_complete = 0U;

        if (discarding != 0U)
        {
            plan.overflow_bytes++;
            if (ldc->frame_mode == LDC_FRAME_MODE_DELIMITER)
            {
                matched = ldc_delimiter_advance(ldc,
                                                matched,
                                                data[index],
                                                &delimiter_complete);
                if (delimiter_complete != 0U)
                {
                    discarding = 0U;
                    matched = 0U;
                }
            }
            continue;
        }

        if (has_slot == 0U)
        {
            has_slot = 1U;
            open_length = 0U;
        }

        if (open_length >= ldc->frame_capacity)
        {
            plan.overflow_frames++;
            plan.overflow_bytes += open_length + 1U;
            open_length = 0U;
            discarding = 1U;

            if (ldc->frame_mode == LDC_FRAME_MODE_DELIMITER)
            {
                matched = ldc_delimiter_advance(ldc,
                                                matched,
                                                data[index],
                                                &delimiter_complete);
                if (delimiter_complete != 0U)
                {
                    discarding = 0U;
                    matched = 0U;
                }
            }
            else
            {
                matched = 0U;
            }
            continue;
        }

        open_length++;
        if (ldc->frame_mode == LDC_FRAME_MODE_DELIMITER)
        {
            size_t output_length;

            matched = ldc_delimiter_advance(ldc,
                                            matched,
                                            data[index],
                                            &delimiter_complete);
            if (delimiter_complete == 0U)
            {
                continue;
            }

            output_length = open_length;
            if (ldc->include_delimiter == 0U)
            {
                output_length -= ldc->delimiter_length;
            }

            if ((output_length > 0U) || (ldc->emit_empty_frames != 0U))
            {
                plan.committed_frames++;
                has_slot = 0U;
            }
            open_length = 0U;
            matched = 0U;
        }
        else if ((ldc->frame_mode == LDC_FRAME_MODE_FIXED_LENGTH) &&
                 (open_length == ldc->fixed_length))
        {
            plan.committed_frames++;
            open_length = 0U;
            has_slot = 0U;
        }
    }

    plan.final_discarding = discarding;
    plan.final_match = matched;
    plan.final_open = (uint8_t)((discarding == 0U) &&
                                (has_slot != 0U) &&
                                (open_length > 0U));
    plan.live_slots = plan.committed_frames + (size_t)plan.final_open;
    if (plan.live_slots > (size_t)has_initial_slot)
    {
        plan.additional_slots =
            plan.live_slots - (size_t)has_initial_slot;
    }

    return plan;
}

/** @brief Find a producer-reserved slot by transaction-local order. */
static uint16_t ldc_find_reserved_slot(const ldc_t *ldc, uint16_t order)
{
    uint16_t index;

    for (index = 0U; index < ldc->slot_count; index++)
    {
        if ((ldc->slots[index].state == LDC_SLOT_RESERVED) &&
            (ldc->slots[index].reservation_order == order))
        {
            return index;
        }
    }

    return LDC_SLOT_NONE;
}

/** @brief Drop the oldest ready frame and account the visible loss. */
static uint8_t ldc_drop_oldest_unlocked(ldc_t *ldc,
                                        ldc_write_result_t *result)
{
    uint16_t index = ldc_find_oldest_ready_unlocked(ldc);
    ldc_slot_t *slot;

    if (index == LDC_SLOT_NONE)
    {
        return 0U;
    }

    slot = &ldc->slots[index];
    result->dropped_frames++;
    result->dropped_bytes += slot->length;
    ldc->stats.dropped_frames++;
    ldc->stats.dropped_bytes += slot->length;
    slot->length = 0U;
    slot->state = LDC_SLOT_FREE;
    slot->pending_state = LDC_SLOT_FREE;
    slot->reservation_order = LDC_ORDER_NONE;
    slot->pending_order = LDC_ORDER_NONE;
    ldc->ready_count--;
    ldc->stats.ready_frames = ldc->ready_count;
    return 1U;
}

/** @brief Reserve all additional slots required by a planned write. */
static ldc_status_t ldc_reserve_write_unlocked(ldc_t *ldc,
                                               size_t required,
                                               ldc_write_result_t *result)
{
    uint16_t free_count = ldc_count_slots_unlocked(ldc, LDC_SLOT_FREE);
    size_t available = free_count;
    size_t order = 0U;
    uint16_t index;

    if (ldc->full_policy == LDC_FULL_DROP_OLDEST)
    {
        available += ldc->ready_count;
    }

    if (required > available)
    {
        return LDC_STATUS_FULL;
    }

    while ((size_t)free_count < required)
    {
        if (ldc_drop_oldest_unlocked(ldc, result) == 0U)
        {
            return LDC_STATUS_FULL;
        }
        free_count++;
    }

    for (index = 0U; (index < ldc->slot_count) && (order < required); index++)
    {
        ldc_slot_t *slot = &ldc->slots[index];

        if (slot->state == LDC_SLOT_FREE)
        {
            slot->state = LDC_SLOT_RESERVED;
            slot->reservation_order = (uint16_t)order;
            slot->pending_order = LDC_ORDER_NONE;
            slot->pending_state = LDC_SLOT_FREE;
            slot->length = 0U;
            order++;
        }
    }

    return (order == required) ? LDC_STATUS_OK : LDC_STATUS_FULL;
}

/** @brief Apply a previously validated plan into open/reserved payload slots. */
static void ldc_apply_write(ldc_t *ldc,
                            const uint8_t *data,
                            size_t length,
                            const ldc_plan_t *plan)
{
    uint16_t current = ldc->current_slot;
    uint16_t next_reservation = 0U;
    uint16_t next_commit = 0U;
    size_t open_length = 0U;
    size_t matched = ldc->delimiter_match;
    size_t index;
    uint8_t discarding = ldc->discarding;

    if (current != LDC_SLOT_NONE)
    {
        ldc->slots[current].pending_state = LDC_SLOT_OPEN;
        ldc->slots[current].pending_order = LDC_ORDER_NONE;
        open_length = ldc->slots[current].length;
    }

    for (index = 0U; index < length; index++)
    {
        uint8_t delimiter_complete = 0U;

        if (discarding != 0U)
        {
            if (ldc->frame_mode == LDC_FRAME_MODE_DELIMITER)
            {
                matched = ldc_delimiter_advance(ldc,
                                                matched,
                                                data[index],
                                                &delimiter_complete);
                if (delimiter_complete != 0U)
                {
                    discarding = 0U;
                    matched = 0U;
                }
            }
            continue;
        }

        if (current == LDC_SLOT_NONE)
        {
            current = ldc_find_reserved_slot(ldc, next_reservation);
            next_reservation++;
            if (current == LDC_SLOT_NONE)
            {
                break;
            }
            ldc->slots[current].pending_state = LDC_SLOT_OPEN;
            ldc->slots[current].pending_order = LDC_ORDER_NONE;
            open_length = 0U;
        }

        if (open_length >= ldc->frame_capacity)
        {
            open_length = 0U;
            ldc->slots[current].length = 0U;
            discarding = 1U;

            if (ldc->frame_mode == LDC_FRAME_MODE_DELIMITER)
            {
                matched = ldc_delimiter_advance(ldc,
                                                matched,
                                                data[index],
                                                &delimiter_complete);
                if (delimiter_complete != 0U)
                {
                    discarding = 0U;
                    matched = 0U;
                }
            }
            else
            {
                matched = 0U;
            }
            continue;
        }

        ldc->slots[current].data[open_length] = data[index];
        open_length++;
        ldc->slots[current].length = open_length;

        if (ldc->frame_mode == LDC_FRAME_MODE_DELIMITER)
        {
            size_t output_length;

            matched = ldc_delimiter_advance(ldc,
                                            matched,
                                            data[index],
                                            &delimiter_complete);
            if (delimiter_complete == 0U)
            {
                continue;
            }

            output_length = open_length;
            if (ldc->include_delimiter == 0U)
            {
                output_length -= ldc->delimiter_length;
            }

            if ((output_length > 0U) || (ldc->emit_empty_frames != 0U))
            {
                ldc->slots[current].length = output_length;
                ldc->slots[current].pending_state = LDC_SLOT_READY;
                ldc->slots[current].pending_order = next_commit;
                next_commit++;
                current = LDC_SLOT_NONE;
            }
            else
            {
                ldc->slots[current].length = 0U;
            }
            open_length = 0U;
            matched = 0U;
        }
        else if ((ldc->frame_mode == LDC_FRAME_MODE_FIXED_LENGTH) &&
                 (open_length == ldc->fixed_length))
        {
            ldc->slots[current].pending_state = LDC_SLOT_READY;
            ldc->slots[current].pending_order = next_commit;
            next_commit++;
            current = LDC_SLOT_NONE;
            open_length = 0U;
        }
    }

    if (current != LDC_SLOT_NONE)
    {
        if ((discarding != 0U) || (open_length == 0U))
        {
            ldc->slots[current].length = 0U;
            ldc->slots[current].pending_state = LDC_SLOT_FREE;
            current = LDC_SLOT_NONE;
        }
        else
        {
            ldc->slots[current].length = open_length;
            ldc->slots[current].pending_state = LDC_SLOT_OPEN;
        }
    }

    ldc->delimiter_match = plan->final_match;
    ldc->discarding = plan->final_discarding;
}

/** @brief Publish all transaction-pending slot states atomically. */
static void ldc_publish_write_unlocked(ldc_t *ldc,
                                       const ldc_plan_t *plan)
{
    uint32_t first_sequence = ldc->next_sequence;
    uint16_t index;

    ldc->current_slot = LDC_SLOT_NONE;
    ldc->next_sequence += (uint32_t)plan->committed_frames;
    for (index = 0U; index < ldc->slot_count; index++)
    {
        ldc_slot_t *slot = &ldc->slots[index];

        if (slot->pending_state == LDC_SLOT_READY)
        {
            slot->state = LDC_SLOT_READY;
            slot->sequence = first_sequence + slot->pending_order;
            slot->generation++;
            if (slot->generation == 0U)
            {
                slot->generation++;
            }
            slot->claim_token = ldc_allocate_claim_token_unlocked(ldc);
            ldc->ready_count++;
        }
    }

    for (index = 0U; index < ldc->slot_count; index++)
    {
        ldc_slot_t *slot = &ldc->slots[index];

        if (slot->pending_state == LDC_SLOT_OPEN)
        {
            slot->state = LDC_SLOT_OPEN;
            ldc->current_slot = index;
        }
        else if (slot->pending_state == LDC_SLOT_FREE)
        {
            if ((slot->state == LDC_SLOT_RESERVED) ||
                (slot->state == LDC_SLOT_OPEN))
            {
                slot->state = LDC_SLOT_FREE;
                slot->length = 0U;
            }
        }

        slot->reservation_order = LDC_ORDER_NONE;
        slot->pending_order = LDC_ORDER_NONE;
        slot->pending_state = LDC_SLOT_FREE;
    }

    ldc->stats.ready_frames = ldc->ready_count;
    ldc->stats.claimed_frames = ldc->claimed_count;
    ldc->stats.discarding_overflow_frame = ldc->discarding;
    ldc_update_watermarks_unlocked(ldc);
}

/** @brief Initialize an LDC object and bind static storage. */
ldc_status_t ldc_init(ldc_t *ldc, const ldc_config_t *config)
{
    ldc_config_t snapshot;
    uint8_t delimiter_snapshot[LDC_DELIMITER_MAX];
    size_t required_storage;
    size_t slot_bytes;
    uint64_t retained_claim_token = 1U;
    uint16_t index;

    if ((ldc == NULL) || (config == NULL))
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }
    (void)memcpy(&snapshot, config, sizeof(snapshot));
    if (ldc->magic == LDC_MAGIC)
    {
        return LDC_STATUS_ALREADY_INITIALIZED;
    }
    if (ldc->magic == LDC_DEINIT_MAGIC)
    {
        retained_claim_token = ldc->next_claim_token;
        if (retained_claim_token == 0U)
        {
            retained_claim_token = 1U;
        }
    }
    if ((snapshot.storage == NULL) || (snapshot.slots == NULL) ||
        (snapshot.slot_count == 0U) ||
        (snapshot.slot_count > LDC_SLOT_COUNT_MAX) ||
        (snapshot.frame_capacity == 0U) ||
        ((snapshot.lock == NULL) != (snapshot.unlock == NULL)) ||
        ((snapshot.frame_mode != LDC_FRAME_MODE_MANUAL) &&
         (snapshot.frame_mode != LDC_FRAME_MODE_DELIMITER) &&
         (snapshot.frame_mode != LDC_FRAME_MODE_FIXED_LENGTH)) ||
        ((snapshot.full_policy != LDC_FULL_REJECT_NEW) &&
         (snapshot.full_policy != LDC_FULL_DROP_OLDEST)))
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }
    if (snapshot.frame_capacity > (SIZE_MAX / snapshot.slot_count))
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }

    required_storage = snapshot.frame_capacity * snapshot.slot_count;
    slot_bytes = sizeof(snapshot.slots[0]) * snapshot.slot_count;
    if ((snapshot.storage_size < required_storage) ||
        (ldc_region_is_valid(ldc, sizeof(*ldc)) == 0U) ||
        (ldc_region_is_valid(snapshot.storage, required_storage) == 0U) ||
        (ldc_region_is_valid(snapshot.slots, slot_bytes) == 0U) ||
        (ldc_regions_overlap(ldc, sizeof(*ldc),
                             snapshot.storage, required_storage) != 0U) ||
        (ldc_regions_overlap(ldc, sizeof(*ldc),
                             snapshot.slots, slot_bytes) != 0U) ||
        (ldc_regions_overlap(snapshot.storage, required_storage,
                             snapshot.slots, slot_bytes) != 0U))
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }
    if ((snapshot.frame_mode == LDC_FRAME_MODE_DELIMITER) &&
        ((snapshot.delimiter == NULL) ||
         (snapshot.delimiter_length == 0U) ||
         (snapshot.delimiter_length > LDC_DELIMITER_MAX)))
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }
    if ((snapshot.frame_mode == LDC_FRAME_MODE_FIXED_LENGTH) &&
        ((snapshot.fixed_length == 0U) ||
         (snapshot.fixed_length > snapshot.frame_capacity)))
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }

    if (snapshot.frame_mode == LDC_FRAME_MODE_DELIMITER)
    {
        (void)memcpy(delimiter_snapshot,
                     snapshot.delimiter,
                     snapshot.delimiter_length);
    }

    (void)memset(ldc, 0, sizeof(*ldc));
    (void)memset(snapshot.slots, 0, slot_bytes);
    ldc->storage = snapshot.storage;
    ldc->storage_size = snapshot.storage_size;
    ldc->slots = snapshot.slots;
    ldc->slot_count = snapshot.slot_count;
    ldc->frame_capacity = snapshot.frame_capacity;
    ldc->frame_mode = snapshot.frame_mode;
    ldc->full_policy = snapshot.full_policy;
    ldc->delimiter_length = snapshot.delimiter_length;
    ldc->include_delimiter = snapshot.include_delimiter;
    ldc->emit_empty_frames = snapshot.emit_empty_frames;
    ldc->fixed_length = snapshot.fixed_length;
    ldc->lock = snapshot.lock;
    ldc->unlock = snapshot.unlock;
    ldc->lock_argument = snapshot.lock_argument;
    ldc->current_slot = LDC_SLOT_NONE;
    ldc->next_sequence = 1U;
    ldc->next_claim_token = retained_claim_token;

    if (snapshot.frame_mode == LDC_FRAME_MODE_DELIMITER)
    {
        (void)memcpy(ldc->delimiter,
                     delimiter_snapshot,
                     snapshot.delimiter_length);
        ldc_build_delimiter_prefix(ldc);
    }

    for (index = 0U; index < ldc->slot_count; index++)
    {
        ldc->slots[index].data =
            &ldc->storage[(size_t)index * ldc->frame_capacity];
        ldc->slots[index].reservation_order = LDC_ORDER_NONE;
        ldc->slots[index].pending_order = LDC_ORDER_NONE;
        ldc->slots[index].state = LDC_SLOT_FREE;
        ldc->slots[index].pending_state = LDC_SLOT_FREE;
    }

    ldc->magic = LDC_MAGIC;
    return LDC_STATUS_OK;
}

/** @brief Clear queued data while retaining configuration and diagnostics. */
ldc_status_t ldc_reset(ldc_t *ldc)
{
    ldc_lock_state_t state;
    uint16_t index;

    if (ldc_is_initialized(ldc) == 0U)
    {
        return LDC_STATUS_NOT_INITIALIZED;
    }

    state = ldc_lock(ldc);
    if ((ldc->write_active != 0U) || (ldc->claimed_count != 0U))
    {
        ldc->stats.busy_calls++;
        ldc_unlock(ldc, state);
        return LDC_STATUS_BUSY;
    }

    for (index = 0U; index < ldc->slot_count; index++)
    {
        ldc->slots[index].length = 0U;
        ldc->slots[index].state = LDC_SLOT_FREE;
        ldc->slots[index].pending_state = LDC_SLOT_FREE;
        ldc->slots[index].reservation_order = LDC_ORDER_NONE;
        ldc->slots[index].pending_order = LDC_ORDER_NONE;
    }
    ldc->current_slot = LDC_SLOT_NONE;
    ldc->ready_count = 0U;
    ldc->delimiter_match = 0U;
    ldc->discarding = 0U;
    ldc->stats.ready_frames = 0U;
    ldc->stats.claimed_frames = 0U;
    ldc->stats.discarding_overflow_frame = 0U;
    ldc_unlock(ldc, state);
    return LDC_STATUS_OK;
}

/** @brief Deinitialize without freeing caller-owned memory. */
ldc_status_t ldc_deinit(ldc_t *ldc)
{
    ldc_lock_fn lock_fn;
    ldc_unlock_fn unlock_fn;
    void *argument;
    ldc_lock_state_t state;
    uint64_t retained_claim_token;
    uint16_t index;

    if (ldc_is_initialized(ldc) == 0U)
    {
        return LDC_STATUS_NOT_INITIALIZED;
    }

    lock_fn = ldc->lock;
    unlock_fn = ldc->unlock;
    argument = ldc->lock_argument;
    retained_claim_token = ldc->next_claim_token;
    state = (lock_fn != NULL) ? lock_fn(argument) : (ldc_lock_state_t)0U;
    if ((ldc->write_active != 0U) || (ldc->claimed_count != 0U))
    {
        ldc->stats.busy_calls++;
        if (unlock_fn != NULL)
        {
            unlock_fn(argument, state);
        }
        return LDC_STATUS_BUSY;
    }

    for (index = 0U; index < ldc->slot_count; index++)
    {
        (void)memset(&ldc->slots[index], 0, sizeof(ldc->slots[index]));
    }
    if (unlock_fn != NULL)
    {
        unlock_fn(argument, state);
    }
    (void)memset(ldc, 0, sizeof(*ldc));
    ldc->next_claim_token = (retained_claim_token != 0U) ?
                                retained_claim_token : 1U;
    ldc->magic = LDC_DEINIT_MAGIC;
    return LDC_STATUS_OK;
}

/** @brief Transactionally append a producer block. */
ldc_write_result_t ldc_rx_write(ldc_t *ldc,
                                const uint8_t *data,
                                size_t length)
{
    ldc_write_result_t result = ldc_write_result_ok();
    ldc_plan_t plan;
    ldc_lock_state_t state;
    ldc_status_t reserve_status;

    if (ldc_is_initialized(ldc) == 0U)
    {
        result.status = LDC_STATUS_NOT_INITIALIZED;
        result.rejected_bytes = length;
        return result;
    }
    if ((data == NULL) && (length != 0U))
    {
        result.status = LDC_STATUS_INVALID_ARGUMENT;
        result.rejected_bytes = length;
        return result;
    }
    if ((length != 0U) &&
        (ldc_buffer_overlaps_owned_memory(ldc, data, length) != 0U))
    {
        result.status = LDC_STATUS_INVALID_ARGUMENT;
        result.rejected_bytes = length;
        return result;
    }
    if (length == 0U)
    {
        return result;
    }

    state = ldc_lock(ldc);
    if (ldc->write_active != 0U)
    {
        ldc->stats.busy_calls++;
        ldc->stats.rejected_bytes += length;
        ldc_unlock(ldc, state);
        result.status = LDC_STATUS_BUSY;
        result.rejected_bytes = length;
        return result;
    }
    ldc->write_active = 1U;
    ldc_unlock(ldc, state);

    plan = ldc_plan_write(ldc, data, length);
    if ((plan.live_slots > ldc->slot_count) ||
        (plan.additional_slots > ldc->slot_count))
    {
        state = ldc_lock(ldc);
        ldc->write_active = 0U;
        ldc->stats.rejected_bytes += length;
        ldc_unlock(ldc, state);
        result.status = LDC_STATUS_FULL;
        result.rejected_bytes = length;
        return result;
    }

    state = ldc_lock(ldc);
    reserve_status = ldc_reserve_write_unlocked(ldc,
                                                plan.additional_slots,
                                                &result);
    if (reserve_status != LDC_STATUS_OK)
    {
        ldc->write_active = 0U;
        ldc->stats.rejected_bytes += length;
        ldc_unlock(ldc, state);
        result.status = reserve_status;
        result.rejected_bytes = length;
        return result;
    }
    ldc_unlock(ldc, state);

    ldc_apply_write(ldc, data, length, &plan);

    state = ldc_lock(ldc);
    ldc_publish_write_unlocked(ldc, &plan);
    ldc->stats.accepted_bytes += length;
    ldc->stats.committed_frames += plan.committed_frames;
    ldc->stats.overflow_frames += plan.overflow_frames;
    ldc->stats.overflow_bytes += plan.overflow_bytes;
    ldc->write_active = 0U;
    ldc_unlock(ldc, state);

    result.accepted_bytes = length;
    result.committed_frames = (uint16_t)plan.committed_frames;
    result.overflow_frames = (uint16_t)plan.overflow_frames;
    result.overflow_bytes = plan.overflow_bytes;
    if ((result.dropped_frames != 0U) || (result.overflow_frames != 0U))
    {
        result.status = LDC_STATUS_DATA_LOSS;
    }
    return result;
}

/** @brief Transactionally append one producer byte. */
ldc_write_result_t ldc_rx_byte(ldc_t *ldc, uint8_t byte)
{
    return ldc_rx_write(ldc, &byte, 1U);
}

/** @brief Publish the current partial frame at an explicit boundary. */
ldc_write_result_t ldc_rx_commit(ldc_t *ldc)
{
    ldc_write_result_t result = ldc_write_result_ok();
    ldc_lock_state_t state;
    ldc_slot_t *slot;

    if (ldc_is_initialized(ldc) == 0U)
    {
        result.status = LDC_STATUS_NOT_INITIALIZED;
        return result;
    }

    state = ldc_lock(ldc);
    if (ldc->write_active != 0U)
    {
        ldc->stats.busy_calls++;
        ldc_unlock(ldc, state);
        result.status = LDC_STATUS_BUSY;
        return result;
    }
    if (ldc->discarding != 0U)
    {
        ldc->discarding = 0U;
        ldc->delimiter_match = 0U;
        ldc->stats.discarding_overflow_frame = 0U;
        ldc_unlock(ldc, state);
        result.status = LDC_STATUS_DATA_LOSS;
        return result;
    }
    if (ldc->current_slot == LDC_SLOT_NONE)
    {
        ldc_unlock(ldc, state);
        result.status = LDC_STATUS_EMPTY;
        return result;
    }

    slot = &ldc->slots[ldc->current_slot];
    if (slot->length == 0U)
    {
        slot->state = LDC_SLOT_FREE;
        ldc->current_slot = LDC_SLOT_NONE;
        ldc->delimiter_match = 0U;
        ldc_unlock(ldc, state);
        result.status = LDC_STATUS_EMPTY;
        return result;
    }

    slot->state = LDC_SLOT_READY;
    slot->sequence = ldc->next_sequence++;
    slot->generation++;
    if (slot->generation == 0U)
    {
        slot->generation++;
    }
    slot->claim_token = ldc_allocate_claim_token_unlocked(ldc);
    ldc->current_slot = LDC_SLOT_NONE;
    ldc->delimiter_match = 0U;
    ldc->ready_count++;
    ldc->stats.committed_frames++;
    ldc->stats.ready_frames = ldc->ready_count;
    ldc_update_watermarks_unlocked(ldc);
    ldc_unlock(ldc, state);

    result.committed_frames = 1U;
    return result;
}

/** @brief Publish a partial frame on a caller-detected UART idle boundary. */
ldc_write_result_t ldc_rx_idle(ldc_t *ldc)
{
    return ldc_rx_commit(ldc);
}

/** @brief Discard the current partial or overflowed frame. */
ldc_status_t ldc_rx_abort(ldc_t *ldc)
{
    ldc_lock_state_t state;
    size_t aborted_bytes = 0U;
    uint8_t had_frame;

    if (ldc_is_initialized(ldc) == 0U)
    {
        return LDC_STATUS_NOT_INITIALIZED;
    }

    state = ldc_lock(ldc);
    if (ldc->write_active != 0U)
    {
        ldc->stats.busy_calls++;
        ldc_unlock(ldc, state);
        return LDC_STATUS_BUSY;
    }

    had_frame = ldc->discarding;
    if (ldc->current_slot != LDC_SLOT_NONE)
    {
        ldc_slot_t *slot = &ldc->slots[ldc->current_slot];

        had_frame = 1U;
        aborted_bytes = slot->length;
        slot->length = 0U;
        slot->state = LDC_SLOT_FREE;
        ldc->current_slot = LDC_SLOT_NONE;
    }
    ldc->delimiter_match = 0U;
    ldc->discarding = 0U;
    ldc->stats.discarding_overflow_frame = 0U;
    if (had_frame != 0U)
    {
        ldc->stats.aborted_frames++;
        ldc->stats.aborted_bytes += aborted_bytes;
    }
    ldc_unlock(ldc, state);
    return LDC_STATUS_OK;
}

/** @brief Claim the oldest ready frame without copying. */
ldc_status_t ldc_frame_claim(ldc_t *ldc, ldc_frame_view_t *view)
{
    ldc_lock_state_t state;
    uint16_t index;
    ldc_slot_t *slot;

    if (view == NULL)
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(view, 0, sizeof(*view));
    view->slot_index = LDC_SLOT_NONE;
    if (ldc_is_initialized(ldc) == 0U)
    {
        return LDC_STATUS_NOT_INITIALIZED;
    }

    state = ldc_lock(ldc);
    index = ldc_find_oldest_ready_unlocked(ldc);
    if (index == LDC_SLOT_NONE)
    {
        ldc_unlock(ldc, state);
        return LDC_STATUS_EMPTY;
    }

    slot = &ldc->slots[index];
    slot->state = LDC_SLOT_CLAIMED;
    ldc->ready_count--;
    ldc->claimed_count++;
    ldc->stats.ready_frames = ldc->ready_count;
    ldc->stats.claimed_frames = ldc->claimed_count;
    view->data = slot->data;
    view->length = slot->length;
    view->sequence = slot->sequence;
    view->generation = slot->generation;
    view->claim_token = slot->claim_token;
    view->slot_index = index;
    ldc_unlock(ldc, state);
    return LDC_STATUS_OK;
}

/** @brief Consume and release a claimed frame. */
ldc_status_t ldc_frame_release(ldc_t *ldc, ldc_frame_view_t *view)
{
    ldc_lock_state_t state;
    ldc_slot_t *slot;

    if ((view == NULL) || (view->slot_index == LDC_SLOT_NONE))
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }
    if (ldc_is_initialized(ldc) == 0U)
    {
        return LDC_STATUS_NOT_INITIALIZED;
    }

    state = ldc_lock(ldc);
    if (view->slot_index >= ldc->slot_count)
    {
        ldc_unlock(ldc, state);
        return LDC_STATUS_STALE_VIEW;
    }

    slot = &ldc->slots[view->slot_index];
    if ((slot->state != LDC_SLOT_CLAIMED) ||
        (slot->claim_token != view->claim_token))
    {
        ldc_unlock(ldc, state);
        return LDC_STATUS_STALE_VIEW;
    }

    ldc->stats.consumed_frames++;
    ldc->stats.consumed_bytes += slot->length;
    slot->length = 0U;
    slot->state = LDC_SLOT_FREE;
    ldc->claimed_count--;
    ldc->stats.claimed_frames = ldc->claimed_count;
    ldc_unlock(ldc, state);

    (void)memset(view, 0, sizeof(*view));
    view->slot_index = LDC_SLOT_NONE;
    return LDC_STATUS_OK;
}

/** @brief Copy and consume the oldest frame outside the critical section. */
ldc_status_t ldc_frame_read(ldc_t *ldc,
                            uint8_t *destination,
                            size_t capacity,
                            size_t *length)
{
    ldc_frame_view_t view;
    ldc_lock_state_t state;
    uint16_t index;
    ldc_slot_t *slot;
    ldc_status_t status;
    size_t frame_length;

    if (length == NULL)
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }
    if (ldc_is_initialized(ldc) == 0U)
    {
        return LDC_STATUS_NOT_INITIALIZED;
    }
    if (ldc_buffer_overlaps_owned_memory(ldc,
                                          length,
                                          sizeof(*length)) != 0U)
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }

    state = ldc_lock(ldc);
    index = ldc_find_oldest_ready_unlocked(ldc);
    if (index == LDC_SLOT_NONE)
    {
        *length = 0U;
        ldc_unlock(ldc, state);
        return LDC_STATUS_EMPTY;
    }

    slot = &ldc->slots[index];
    frame_length = slot->length;
    if ((destination != NULL) && (frame_length != 0U) &&
        (ldc_buffer_overlaps_owned_memory(ldc,
                                          destination,
                                          frame_length) != 0U))
    {
        ldc_unlock(ldc, state);
        return LDC_STATUS_INVALID_ARGUMENT;
    }
    if ((destination != NULL) && (frame_length != 0U) &&
        (ldc_regions_overlap(destination,
                             frame_length,
                             length,
                             sizeof(*length)) != 0U))
    {
        ldc_unlock(ldc, state);
        return LDC_STATUS_INVALID_ARGUMENT;
    }
    if (capacity < frame_length)
    {
        *length = frame_length;
        ldc_unlock(ldc, state);
        return LDC_STATUS_BUFFER_TOO_SMALL;
    }
    if ((destination == NULL) && (frame_length != 0U))
    {
        ldc_unlock(ldc, state);
        return LDC_STATUS_INVALID_ARGUMENT;
    }

    slot->state = LDC_SLOT_CLAIMED;
    ldc->ready_count--;
    ldc->claimed_count++;
    ldc->stats.ready_frames = ldc->ready_count;
    ldc->stats.claimed_frames = ldc->claimed_count;
    view.data = slot->data;
    view.length = slot->length;
    view.sequence = slot->sequence;
    view.generation = slot->generation;
    view.claim_token = slot->claim_token;
    view.slot_index = index;
    ldc_unlock(ldc, state);

    if (view.length != 0U)
    {
        (void)memcpy(destination, view.data, view.length);
    }
    status = ldc_frame_release(ldc, &view);
    if (status == LDC_STATUS_OK)
    {
        *length = frame_length;
    }
    return status;
}

/** @brief Snapshot diagnostics under the configured lock. */
ldc_status_t ldc_get_stats(ldc_t *ldc, ldc_stats_t *stats)
{
    ldc_lock_state_t state;

    if (stats == NULL)
    {
        return LDC_STATUS_INVALID_ARGUMENT;
    }
    if (ldc_is_initialized(ldc) == 0U)
    {
        return LDC_STATUS_NOT_INITIALIZED;
    }

    state = ldc_lock(ldc);
    *stats = ldc->stats;
    ldc_unlock(ldc, state);
    return LDC_STATUS_OK;
}
