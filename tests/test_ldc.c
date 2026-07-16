/**
 * @file test_ldc.c
 * @brief Host regression tests for LDC transactional framing and ownership.
 */

#include "ldc.h"

#include <stdio.h>
#include <string.h>

#define TEST_SLOT_COUNT 4U
#define TEST_FRAME_CAPACITY 32U

#define CHECK(expression)                                                     \
    do                                                                        \
    {                                                                         \
        if (!(expression))                                                    \
        {                                                                     \
            (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expression); \
            return 0;                                                         \
        }                                                                     \
    } while (0)

typedef struct
{
    unsigned int depth;
    unsigned int maximum_depth;
    unsigned int enter_count;
    unsigned int leave_count;
} test_lock_t;

typedef struct
{
    ldc_t ldc;
    ldc_slot_t slots[TEST_SLOT_COUNT];
    uint8_t storage[TEST_SLOT_COUNT * TEST_FRAME_CAPACITY];
    test_lock_t lock;
} test_fixture_t;

/** @brief Instrument a host-only critical section. */
static ldc_lock_state_t test_lock_enter(void *argument)
{
    test_lock_t *lock = (test_lock_t *)argument;

    lock->depth++;
    lock->enter_count++;
    if (lock->depth > lock->maximum_depth)
    {
        lock->maximum_depth = lock->depth;
    }
    return (ldc_lock_state_t)lock->depth;
}

/** @brief Instrument exit and verify balanced lock hooks. */
static void test_lock_leave(void *argument, ldc_lock_state_t state)
{
    test_lock_t *lock = (test_lock_t *)argument;

    (void)state;
    lock->leave_count++;
    if (lock->depth > 0U)
    {
        lock->depth--;
    }
}

/** @brief Initialize a fixture with common static storage. */
static ldc_status_t fixture_init(test_fixture_t *fixture,
                                 uint16_t slot_count,
                                 size_t frame_capacity,
                                 ldc_frame_mode_t mode,
                                 ldc_full_policy_t full_policy,
                                 const uint8_t *delimiter,
                                 uint8_t delimiter_length,
                                 uint8_t include_delimiter,
                                 size_t fixed_length)
{
    ldc_config_t config;

    (void)memset(fixture, 0, sizeof(*fixture));
    (void)memset(&config, 0, sizeof(config));
    config.storage = fixture->storage;
    config.storage_size = sizeof(fixture->storage);
    config.slots = fixture->slots;
    config.slot_count = slot_count;
    config.frame_capacity = frame_capacity;
    config.frame_mode = mode;
    config.full_policy = full_policy;
    config.delimiter = delimiter;
    config.delimiter_length = delimiter_length;
    config.include_delimiter = include_delimiter;
    config.fixed_length = fixed_length;
    config.lock = test_lock_enter;
    config.unlock = test_lock_leave;
    config.lock_argument = &fixture->lock;
    return ldc_init(&fixture->ldc, &config);
}

/** @brief Read and compare one queued frame. */
static int expect_frame(test_fixture_t *fixture,
                        const uint8_t *expected,
                        size_t expected_length)
{
    uint8_t output[TEST_FRAME_CAPACITY];
    size_t length = 0U;

    CHECK(ldc_frame_read(&fixture->ldc,
                         output,
                         sizeof(output),
                         &length) == LDC_STATUS_OK);
    CHECK(length == expected_length);
    CHECK(memcmp(output, expected, length) == 0);
    return 1;
}

/** @brief Verify fragmented and batched CRLF delimiter handling. */
static int test_delimiter_fragmentation(void)
{
    static const uint8_t delimiter[] = {'\r', '\n'};
    static const uint8_t first[] = "AT+G";
    static const uint8_t second[] = "MR\r\nOK\r\n";
    static const uint8_t expected_first[] = "AT+GMR";
    static const uint8_t expected_second[] = "OK";
    test_fixture_t fixture;
    ldc_write_result_t result;

    CHECK(fixture_init(&fixture, 4U, 16U,
                       LDC_FRAME_MODE_DELIMITER,
                       LDC_FULL_REJECT_NEW,
                       delimiter, 2U, 0U, 0U) == LDC_STATUS_OK);
    result = ldc_rx_write(&fixture.ldc, first, sizeof(first) - 1U);
    CHECK(result.status == LDC_STATUS_OK);
    CHECK(result.accepted_bytes == sizeof(first) - 1U);
    CHECK(result.committed_frames == 0U);
    result = ldc_rx_write(&fixture.ldc, second, sizeof(second) - 1U);
    CHECK(result.status == LDC_STATUS_OK);
    CHECK(result.committed_frames == 2U);
    CHECK(expect_frame(&fixture, expected_first, sizeof(expected_first) - 1U));
    CHECK(expect_frame(&fixture, expected_second, sizeof(expected_second) - 1U));
    CHECK(fixture.lock.depth == 0U);
    CHECK(fixture.lock.maximum_depth == 1U);
    CHECK(fixture.lock.enter_count == fixture.lock.leave_count);
    return 1;
}

/** @brief Verify exact fixed-length batching. */
static int test_fixed_length(void)
{
    static const uint8_t input[] = "ABCDEFGH";
    test_fixture_t fixture;
    ldc_write_result_t result;

    CHECK(fixture_init(&fixture, 3U, 8U,
                       LDC_FRAME_MODE_FIXED_LENGTH,
                       LDC_FULL_REJECT_NEW,
                       NULL, 0U, 0U, 4U) == LDC_STATUS_OK);
    result = ldc_rx_write(&fixture.ldc, input, sizeof(input) - 1U);
    CHECK(result.status == LDC_STATUS_OK);
    CHECK(result.committed_frames == 2U);
    CHECK(expect_frame(&fixture, input, 4U));
    CHECK(expect_frame(&fixture, &input[4], 4U));
    return 1;
}

/** @brief Verify explicit idle commit and copy capacity reporting. */
static int test_idle_and_small_buffer(void)
{
    static const uint8_t input[] = "hello";
    uint8_t output[8];
    size_t length = 0U;
    test_fixture_t fixture;

    CHECK(fixture_init(&fixture, 2U, 8U,
                       LDC_FRAME_MODE_MANUAL,
                       LDC_FULL_REJECT_NEW,
                       NULL, 0U, 0U, 0U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, input, sizeof(input) - 1U).status ==
          LDC_STATUS_OK);
    CHECK(ldc_rx_idle(&fixture.ldc).committed_frames == 1U);
    CHECK(ldc_frame_read(&fixture.ldc, output, 3U, &length) ==
          LDC_STATUS_BUFFER_TOO_SMALL);
    CHECK(length == sizeof(input) - 1U);
    CHECK(ldc_frame_read(&fixture.ldc,
                         output,
                         sizeof(output),
                         &length) == LDC_STATUS_OK);
    CHECK(length == sizeof(input) - 1U);
    CHECK(memcmp(output, input, length) == 0);
    return 1;
}

/** @brief Verify a rejected block cannot publish a valid-looking prefix. */
static int test_transactional_reject(void)
{
    static const uint8_t delimiter[] = {'\n'};
    static const uint8_t old_frame[] = "old\n";
    static const uint8_t partial[] = "pre";
    static const uint8_t rejected[] = "\nnew\n";
    static const uint8_t expected_old[] = "old";
    static const uint8_t expected_partial[] = "pre";
    test_fixture_t fixture;
    ldc_write_result_t result;

    CHECK(fixture_init(&fixture, 2U, 8U,
                       LDC_FRAME_MODE_DELIMITER,
                       LDC_FULL_REJECT_NEW,
                       delimiter, 1U, 0U, 0U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc,
                       old_frame,
                       sizeof(old_frame) - 1U).committed_frames == 1U);
    CHECK(ldc_rx_write(&fixture.ldc,
                       partial,
                       sizeof(partial) - 1U).accepted_bytes ==
          sizeof(partial) - 1U);

    result = ldc_rx_write(&fixture.ldc,
                          rejected,
                          sizeof(rejected) - 1U);
    CHECK(result.status == LDC_STATUS_FULL);
    CHECK(result.accepted_bytes == 0U);
    CHECK(result.rejected_bytes == sizeof(rejected) - 1U);
    CHECK(expect_frame(&fixture, expected_old, sizeof(expected_old) - 1U));
    CHECK(ldc_rx_commit(&fixture.ldc).committed_frames == 1U);
    CHECK(expect_frame(&fixture,
                       expected_partial,
                       sizeof(expected_partial) - 1U));
    return 1;
}

/** @brief Verify DROP_OLDEST reports exactly what was removed. */
static int test_drop_oldest_visible(void)
{
    static const uint8_t delimiter[] = {'\n'};
    static const uint8_t first[] = "one\n";
    static const uint8_t second[] = "two\n";
    static const uint8_t third[] = "three\n";
    static const uint8_t expected_second[] = "two";
    static const uint8_t expected_third[] = "three";
    test_fixture_t fixture;
    ldc_write_result_t result;
    ldc_stats_t stats;

    CHECK(fixture_init(&fixture, 2U, 8U,
                       LDC_FRAME_MODE_DELIMITER,
                       LDC_FULL_DROP_OLDEST,
                       delimiter, 1U, 0U, 0U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, first, sizeof(first) - 1U).status ==
          LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, second, sizeof(second) - 1U).status ==
          LDC_STATUS_OK);
    result = ldc_rx_write(&fixture.ldc, third, sizeof(third) - 1U);
    CHECK(result.status == LDC_STATUS_DATA_LOSS);
    CHECK(result.accepted_bytes == sizeof(third) - 1U);
    CHECK(result.dropped_frames == 1U);
    CHECK(result.dropped_bytes == 3U);
    CHECK(expect_frame(&fixture, expected_second, sizeof(expected_second) - 1U));
    CHECK(expect_frame(&fixture, expected_third, sizeof(expected_third) - 1U));
    CHECK(ldc_get_stats(&fixture.ldc, &stats) == LDC_STATUS_OK);
    CHECK(stats.dropped_frames == 1U);
    CHECK(stats.dropped_bytes == 3U);
    return 1;
}

/** @brief Verify a claimed frame is pinned against overwrite. */
static int test_claim_pins_slot(void)
{
    static const uint8_t delimiter[] = {'\n'};
    static const uint8_t first[] = "A\n";
    static const uint8_t second[] = "B\n";
    test_fixture_t fixture;
    ldc_frame_view_t view;
    ldc_write_result_t result;

    CHECK(fixture_init(&fixture, 1U, 4U,
                       LDC_FRAME_MODE_DELIMITER,
                       LDC_FULL_DROP_OLDEST,
                       delimiter, 1U, 0U, 0U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, first, sizeof(first) - 1U).status ==
          LDC_STATUS_OK);
    CHECK(ldc_frame_claim(&fixture.ldc, &view) == LDC_STATUS_OK);
    CHECK(view.length == 1U);
    CHECK(view.data[0] == (uint8_t)'A');
    result = ldc_rx_write(&fixture.ldc, second, sizeof(second) - 1U);
    CHECK(result.status == LDC_STATUS_FULL);
    CHECK(result.rejected_bytes == sizeof(second) - 1U);
    CHECK(view.data[0] == (uint8_t)'A');
    CHECK(ldc_frame_release(&fixture.ldc, &view) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, second, sizeof(second) - 1U).status ==
          LDC_STATUS_OK);
    return 1;
}

/** @brief Verify an oversized frame is discarded through its boundary. */
static int test_overflow_discards_whole_frame(void)
{
    static const uint8_t delimiter[] = {'\n'};
    static const uint8_t input[] = "ABCDE\nOK\n";
    static const uint8_t expected[] = "OK";
    test_fixture_t fixture;
    ldc_write_result_t result;

    CHECK(fixture_init(&fixture, 2U, 4U,
                       LDC_FRAME_MODE_DELIMITER,
                       LDC_FULL_REJECT_NEW,
                       delimiter, 1U, 0U, 0U) == LDC_STATUS_OK);
    result = ldc_rx_write(&fixture.ldc, input, sizeof(input) - 1U);
    CHECK(result.status == LDC_STATUS_DATA_LOSS);
    CHECK(result.accepted_bytes == sizeof(input) - 1U);
    CHECK(result.overflow_frames == 1U);
    CHECK(result.overflow_bytes == 6U);
    CHECK(result.committed_frames == 1U);
    CHECK(expect_frame(&fixture, expected, sizeof(expected) - 1U));
    return 1;
}

/** @brief Verify delimiter overlap uses streaming KMP state. */
static int test_overlapping_delimiter(void)
{
    static const uint8_t delimiter[] = {'a', 'b', 'a', 'b'};
    static const uint8_t input[] = "xxababyyabab";
    static const uint8_t first[] = "xx";
    static const uint8_t second[] = "yy";
    test_fixture_t fixture;

    CHECK(fixture_init(&fixture, 3U, 16U,
                       LDC_FRAME_MODE_DELIMITER,
                       LDC_FULL_REJECT_NEW,
                       delimiter, 4U, 0U, 0U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, input, sizeof(input) - 1U).committed_frames ==
          2U);
    CHECK(expect_frame(&fixture, first, sizeof(first) - 1U));
    CHECK(expect_frame(&fixture, second, sizeof(second) - 1U));
    return 1;
}

/** @brief Verify lifecycle guards and stale view protection. */
static int test_lifecycle_and_stale_view(void)
{
    static const uint8_t delimiter[] = {'\n'};
    static const uint8_t input[] = "X\n";
    test_fixture_t fixture;
    ldc_frame_view_t view;
    ldc_frame_view_t stale;

    CHECK(fixture_init(&fixture, 2U, 4U,
                       LDC_FRAME_MODE_DELIMITER,
                       LDC_FULL_REJECT_NEW,
                       delimiter, 1U, 0U, 0U) == LDC_STATUS_OK);
    CHECK(ldc_init(&fixture.ldc, NULL) == LDC_STATUS_INVALID_ARGUMENT);
    CHECK(ldc_rx_write(&fixture.ldc, input, sizeof(input) - 1U).status ==
          LDC_STATUS_OK);
    CHECK(ldc_frame_claim(&fixture.ldc, &view) == LDC_STATUS_OK);
    CHECK(ldc_reset(&fixture.ldc) == LDC_STATUS_BUSY);
    CHECK(ldc_deinit(&fixture.ldc) == LDC_STATUS_BUSY);
    stale = view;
    CHECK(ldc_frame_release(&fixture.ldc, &view) == LDC_STATUS_OK);
    CHECK(ldc_frame_release(&fixture.ldc, &stale) == LDC_STATUS_STALE_VIEW);
    CHECK(ldc_deinit(&fixture.ldc) == LDC_STATUS_OK);
    CHECK(ldc_rx_abort(&fixture.ldc) == LDC_STATUS_NOT_INITIALIZED);
    return 1;
}

/** @brief Reject overlapping context, metadata, and payload ownership. */
static int test_init_rejects_overlapping_regions(void)
{
    test_fixture_t fixture;
    ldc_config_t config;

    (void)memset(&fixture, 0, sizeof(fixture));
    (void)memset(&config, 0, sizeof(config));
    config.storage = (uint8_t *)fixture.slots;
    config.storage_size = sizeof(fixture.slots);
    config.slots = fixture.slots;
    config.slot_count = 2U;
    config.frame_capacity = 8U;
    config.frame_mode = LDC_FRAME_MODE_MANUAL;
    config.full_policy = LDC_FULL_REJECT_NEW;
    CHECK(ldc_init(&fixture.ldc, &config) == LDC_STATUS_INVALID_ARGUMENT);

    config.storage = (uint8_t *)&fixture.ldc;
    config.storage_size = sizeof(fixture.ldc);
    config.slots = fixture.slots;
    config.slot_count = 1U;
    config.frame_capacity = 8U;
    CHECK(ldc_init(&fixture.ldc, &config) == LDC_STATUS_INVALID_ARGUMENT);
    return 1;
}

/** @brief Reject producer input that aliases LDC-owned payload memory. */
static int test_rx_rejects_owned_input(void)
{
    static const uint8_t input[] = "ABCD";
    test_fixture_t fixture;
    ldc_write_result_t result;

    CHECK(fixture_init(&fixture, 2U, 8U,
                       LDC_FRAME_MODE_FIXED_LENGTH,
                       LDC_FULL_DROP_OLDEST,
                       NULL, 0U, 0U, 4U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, input, 4U).committed_frames == 1U);
    result = ldc_rx_write(&fixture.ldc, fixture.storage, 4U);
    CHECK(result.status == LDC_STATUS_INVALID_ARGUMENT);
    CHECK(result.accepted_bytes == 0U);
    CHECK(result.rejected_bytes == 4U);
    CHECK(expect_frame(&fixture, input, 4U));
    return 1;
}

/** @brief Reject consumer output that aliases LDC-owned payload memory. */
static int test_read_rejects_owned_destination(void)
{
    static const uint8_t input[] = "WXYZ";
    test_fixture_t fixture;
    size_t length = SIZE_MAX;

    CHECK(fixture_init(&fixture, 2U, 8U,
                       LDC_FRAME_MODE_FIXED_LENGTH,
                       LDC_FULL_REJECT_NEW,
                       NULL, 0U, 0U, 4U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, input, 4U).committed_frames == 1U);
    CHECK(ldc_frame_read(&fixture.ldc,
                         &fixture.storage[1],
                         4U,
                         &length) == LDC_STATUS_INVALID_ARGUMENT);
    CHECK(length == SIZE_MAX);
    CHECK(expect_frame(&fixture, input, 4U));
    return 1;
}

/** @brief Reject overlapping external output objects without consuming data. */
static int test_read_rejects_overlapping_outputs(void)
{
    static const uint8_t input[] = {0x78U, 0x56U, 0x34U, 0x12U};
    test_fixture_t fixture;
    size_t output_and_length = SIZE_MAX;

    CHECK(fixture_init(&fixture, 2U, 8U,
                       LDC_FRAME_MODE_FIXED_LENGTH,
                       LDC_FULL_REJECT_NEW,
                       NULL, 0U, 0U, 4U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, input, 4U).committed_frames == 1U);
    CHECK(ldc_frame_read(&fixture.ldc,
                         (uint8_t *)&output_and_length,
                         1U,
                         &output_and_length) == LDC_STATUS_INVALID_ARGUMENT);
    CHECK(output_and_length == SIZE_MAX);
    CHECK(ldc_frame_read(&fixture.ldc,
                         (uint8_t *)&output_and_length,
                         4U,
                         &output_and_length) == LDC_STATUS_INVALID_ARGUMENT);
    CHECK(output_and_length == SIZE_MAX);
    CHECK(expect_frame(&fixture, input, 4U));
    return 1;
}

/** @brief Reject a length output that aliases LDC-owned metadata. */
static int test_read_rejects_owned_length(void)
{
    static const uint8_t input[] = "ABCD";
    test_fixture_t fixture;
    uint8_t output[4];

    CHECK(fixture_init(&fixture, 2U, 8U,
                       LDC_FRAME_MODE_FIXED_LENGTH,
                       LDC_FULL_REJECT_NEW,
                       NULL, 0U, 0U, 4U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, input, 4U).committed_frames == 1U);
    CHECK(ldc_frame_read(&fixture.ldc,
                         output,
                         sizeof(output),
                         &fixture.slots[0].length) ==
          LDC_STATUS_INVALID_ARGUMENT);
    CHECK(expect_frame(&fixture, input, 4U));
    return 1;
}

/** @brief Keep stale views invalid across synchronized 32-bit wrap values. */
static int test_claim_token_prevents_32_bit_aba(void)
{
    static const uint8_t first[] = {'A'};
    static const uint8_t second[] = {'B'};
    static const uint8_t third[] = {'C'};
    test_fixture_t fixture;
    ldc_frame_view_t first_view;
    ldc_frame_view_t stale_view;
    ldc_frame_view_t second_view;
    ldc_frame_view_t live_view;

    CHECK(fixture_init(&fixture, 1U, 1U,
                       LDC_FRAME_MODE_FIXED_LENGTH,
                       LDC_FULL_REJECT_NEW,
                       NULL, 0U, 0U, 1U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, first, 1U).committed_frames == 1U);
    CHECK(ldc_frame_claim(&fixture.ldc, &first_view) == LDC_STATUS_OK);
    stale_view = first_view;
    CHECK(ldc_frame_release(&fixture.ldc, &first_view) == LDC_STATUS_OK);

    fixture.ldc.next_sequence = UINT32_MAX;
    fixture.slots[0].generation = UINT32_MAX - 1U;
    CHECK(ldc_rx_write(&fixture.ldc, second, 1U).committed_frames == 1U);
    CHECK(ldc_frame_claim(&fixture.ldc, &second_view) == LDC_STATUS_OK);
    CHECK(ldc_frame_release(&fixture.ldc, &second_view) == LDC_STATUS_OK);

    fixture.ldc.next_sequence = stale_view.sequence;
    CHECK(ldc_rx_write(&fixture.ldc, third, 1U).committed_frames == 1U);
    CHECK(ldc_frame_claim(&fixture.ldc, &live_view) == LDC_STATUS_OK);
    CHECK(live_view.slot_index == stale_view.slot_index);
    CHECK(live_view.sequence == stale_view.sequence);
    CHECK(live_view.generation == stale_view.generation);
    CHECK(live_view.claim_token != stale_view.claim_token);
    CHECK(ldc_frame_release(&fixture.ldc, &stale_view) ==
          LDC_STATUS_STALE_VIEW);
    CHECK(ldc_frame_release(&fixture.ldc, &live_view) == LDC_STATUS_OK);
    return 1;
}

/** @brief Keep stale views invalid across deinit/reinit of one object. */
static int test_claim_token_survives_reinit(void)
{
    static const uint8_t first[] = {'A'};
    static const uint8_t second[] = {'B'};
    test_fixture_t fixture;
    ldc_config_t config;
    ldc_frame_view_t first_view;
    ldc_frame_view_t stale_view;
    ldc_frame_view_t live_view;

    CHECK(fixture_init(&fixture, 1U, 1U,
                       LDC_FRAME_MODE_FIXED_LENGTH,
                       LDC_FULL_REJECT_NEW,
                       NULL, 0U, 0U, 1U) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, first, 1U).committed_frames == 1U);
    CHECK(ldc_frame_claim(&fixture.ldc, &first_view) == LDC_STATUS_OK);
    stale_view = first_view;
    CHECK(ldc_frame_release(&fixture.ldc, &first_view) == LDC_STATUS_OK);
    CHECK(ldc_deinit(&fixture.ldc) == LDC_STATUS_OK);

    (void)memset(&config, 0, sizeof(config));
    config.storage = fixture.storage;
    config.storage_size = sizeof(fixture.storage);
    config.slots = fixture.slots;
    config.slot_count = 1U;
    config.frame_capacity = 1U;
    config.frame_mode = LDC_FRAME_MODE_FIXED_LENGTH;
    config.full_policy = LDC_FULL_REJECT_NEW;
    config.fixed_length = 1U;
    config.lock = test_lock_enter;
    config.unlock = test_lock_leave;
    config.lock_argument = &fixture.lock;
    CHECK(ldc_init(&fixture.ldc, &config) == LDC_STATUS_OK);
    CHECK(ldc_rx_write(&fixture.ldc, second, 1U).committed_frames == 1U);
    CHECK(ldc_frame_claim(&fixture.ldc, &live_view) == LDC_STATUS_OK);
    CHECK(live_view.slot_index == stale_view.slot_index);
    CHECK(live_view.claim_token != stale_view.claim_token);
    CHECK(ldc_frame_release(&fixture.ldc, &stale_view) ==
          LDC_STATUS_STALE_VIEW);
    CHECK(ldc_frame_release(&fixture.ldc, &live_view) == LDC_STATUS_OK);
    return 1;
}

/** @brief Execute every host regression case. */
int main(void)
{
    if ((!test_delimiter_fragmentation()) ||
        (!test_fixed_length()) ||
        (!test_idle_and_small_buffer()) ||
        (!test_transactional_reject()) ||
        (!test_drop_oldest_visible()) ||
        (!test_claim_pins_slot()) ||
        (!test_overflow_discards_whole_frame()) ||
        (!test_overlapping_delimiter()) ||
        (!test_lifecycle_and_stale_view()) ||
        (!test_init_rejects_overlapping_regions()) ||
        (!test_rx_rejects_owned_input()) ||
        (!test_read_rejects_owned_destination()) ||
        (!test_read_rejects_overlapping_outputs()) ||
        (!test_read_rejects_owned_length()) ||
        (!test_claim_token_prevents_32_bit_aba()) ||
        (!test_claim_token_survives_reinit()))
    {
        return 1;
    }
    (void)printf("PASS: LDC vNext host regression suite\n");
    return 0;
}
