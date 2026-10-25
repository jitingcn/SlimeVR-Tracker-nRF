#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

static unsigned raw_calls, batch_calls, mag_calls, mag_clears, mag_calibrations, online_calls;
static bool raw_enabled, mag_enabled, online_enabled, last_batch_enabled;
static uint16_t last_batch_rate;
static void connection_set_data_collection(bool enable) { raw_calls++; raw_enabled = enable; }
static int sensor_set_mag_enabled(bool enable) { mag_calls++; mag_enabled = enable; return 0; }
static int sensor_calibration_clear_mag(void *matrix, bool persist, bool feedback)
{
    assert(matrix == NULL && persist && feedback);
    mag_clears++;
    return 0;
}
static int sensor_request_calibration_mag(void) { mag_calibrations++; return 0; }
static int sensor_calibration_set_online_mag_enabled(bool enable)
{
    online_calls++;
    online_enabled = enable;
    return 0;
}
static unsigned retransmit_calls;
static uint16_t retransmit_sequence;
static int connection_request_raw_retransmit(uint16_t sequence)
{
    retransmit_calls++;
    retransmit_sequence = sequence;
    return 0;
}
#define RAW_ARQ_MARKER 0xAA
#define OTA_RX_QUEUE_SIZE 16
#define __DMB() ((void)0)
static struct { uint8_t data[64], length; } ota_rx_queue[OTA_RX_QUEUE_SIZE];
static uint8_t ota_rx_head, ota_rx_tail;
#define main enabled_regression_main
#include "test_commands.c"
#undef main

static void execute_owner(void)
{
    now_ms += 100;
    feeds = 0;
    stop_after = 3;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(feeds == 3);
}

static unsigned mutations(void)
{
    return raw_calls + batch_calls + mag_calls + mag_clears + mag_calibrations + online_calls;
}

static void command(uint8_t flag, bool supported, uint8_t event_kind)
{
    receive_control(ESB_PONG_FLAG_NORMAL);
    assert(acked_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(received_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(!remote_command_rejected);
    unsigned before = mutations();
    unsigned rejects = led_test_events[LED_REJECTED];
    unsigned warnings = warning_count;
    unsigned events = rejected_events;
    rx_payload.data[8] = 100;
    receive_control(flag);
    assert(mutations() == before);
    assert(acked_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(esb_get_ping_ack_flag() == (supported ? flag : ESB_PONG_FLAG_NORMAL));
    feeds = 0;
    stop_after = 1;
    if (setjmp(thread_stop) == 0) esb_thread();
    assert(mutations() == before && warning_count == warnings);
    assert(acked_remote_command == ESB_PONG_FLAG_NORMAL && !remote_command_rejected);
    if (supported) {
        receive_control(ESB_PONG_FLAG_NORMAL);
        assert(received_remote_command == flag && acked_remote_command == ESB_PONG_FLAG_NORMAL);
        assert(esb_get_ping_ack_flag() == flag);
    }
    if (!supported) {
        int status = esb_remote_command_execute(flag);
        assert(status == -ENOTSUP);
        if (getenv("CAPABILITY_SMOKE")) printf("remote_execute raw=%d mag=%d command=0x%02x result=%d\n",
            CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER, flag, status);
        assert(mutations() == before);
        assert(led_test_events[LED_REJECTED] == rejects + 1);
        assert(warning_count == warnings + 1);
        rejects++;
        warnings++;
        if (event_kind) {
            assert(rejected_events == events + 1);
            assert(rejected_kind == event_kind && rejected_reason == CAL_REASON_UNSUPPORTED);
            assert(event_notifications == rejected_events);
            events++;
        }
    }
    execute_owner();
    assert(acked_remote_command == (supported ? flag : ESB_PONG_FLAG_NORMAL));
    assert(esb_get_ping_ack_flag() == (supported ? flag : ESB_PONG_FLAG_NORMAL));
    assert(mutations() == before + (supported ? 1U : 0U));
    uint8_t executed_echo = esb_get_ping_ack_flag();
    uint8_t echo_data[4];
    esb_get_ping_request_data(echo_data);
    const uint8_t expected_echo_data[4] = {
        supported && flag == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON ? 100 : 0, 0, 0, 0,
    };
    assert(memcmp(echo_data, expected_echo_data, sizeof(echo_data)) == 0);
    if (getenv("CAPABILITY_SMOKE")) {
        printf("remote raw=%d mag=%d command=0x%02x supported=%d echo=0x%02x data_hex=%02x%02x%02x%02x mutations=%u rejects=%u warnings=%u raw_active=%d batch_active=%d batch_rate=%u mag_active=%d online_active=%d rejected=%d\n",
               CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER, flag,
               supported, executed_echo, echo_data[0], echo_data[1], echo_data[2], echo_data[3],
               mutations() - before, led_test_events[LED_REJECTED] - rejects, warning_count - warnings,
               raw_enabled, last_batch_enabled, last_batch_rate, mag_enabled, online_enabled,
               remote_command_rejected);
    }
    if (!supported) {
        assert(remote_command_rejected);
        assert(led_test_events[LED_REJECTED] == rejects + 1);
        assert(warning_count == warnings + 1);
        assert(rejected_events == events + (event_kind ? 1U : 0U));
    }
    unsigned completed = mutations();
    unsigned final_rejects = led_test_events[LED_REJECTED];
    unsigned final_warnings = warning_count;
    uint32_t generation = remote_command_generation;
    receive_control(flag);
    execute_owner();
    assert(remote_command_generation == generation);
    assert(mutations() == completed);
    assert(led_test_events[LED_REJECTED] == final_rejects);
    assert(warning_count == final_warnings);
    receive_control(ESB_PONG_FLAG_NORMAL);
    assert(received_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(acked_remote_command == ESB_PONG_FLAG_NORMAL && !remote_command_rejected);
    if (!supported) {
        receive_control(flag);
        execute_owner();
        assert(remote_command_rejected && acked_remote_command == ESB_PONG_FLAG_NORMAL);
        receive_control(ESB_PONG_FLAG_PING);
        assert(received_remote_command == ESB_PONG_FLAG_PING && !remote_command_rejected);
        execute_owner();
        assert(acked_remote_command == ESB_PONG_FLAG_PING);
        receive_control(flag);
        assert(acked_remote_command == ESB_PONG_FLAG_PING);
        assert(esb_get_ping_ack_flag() == ESB_PONG_FLAG_NORMAL);
        execute_owner();
        assert(remote_command_rejected && acked_remote_command == ESB_PONG_FLAG_PING);
        receive_control(ESB_PONG_FLAG_NORMAL);
        assert(received_remote_command == ESB_PONG_FLAG_NORMAL && acked_remote_command == ESB_PONG_FLAG_NORMAL);
    }
    receive_control(ESB_PONG_FLAG_PING);
    unsigned identifies = led_test_events[LED_IDENTIFY];
    execute_owner();
    assert(acked_remote_command == ESB_PONG_FLAG_PING);
    assert(esb_get_ping_ack_flag() == ESB_PONG_FLAG_PING);
    assert(led_test_events[LED_IDENTIFY] == identifies + 1);
}

int main(void)
{
    now_ms = 1000;
    batch_result = 0;
    remote_command_rejected = false;
    received_remote_command = acked_remote_command = ESB_PONG_FLAG_NORMAL;
    int identify_result = esb_remote_command_execute(ESB_PONG_FLAG_PING);
    assert(identify_result == 0 && led_test_events[LED_IDENTIFY] == 1);
    if (getenv("CAPABILITY_SMOKE")) printf("remote_execute raw=%d mag=%d command=0x%02x result=%d identify=%u\n",
        CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER,
        ESB_PONG_FLAG_PING, identify_result, led_test_events[LED_IDENTIFY]);
    command(ESB_PONG_FLAG_DATA_COLLECT_ON, CONFIG_CONNECTION_RAW_COLLECTION, 0);
    assert(raw_enabled == !!CONFIG_CONNECTION_RAW_COLLECTION);
    command(ESB_PONG_FLAG_DATA_COLLECT_OFF, CONFIG_CONNECTION_RAW_COLLECTION, 0);
    assert(!raw_enabled);
    command(ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON, CONFIG_CONNECTION_RAW_COLLECTION, 0);
    if (CONFIG_CONNECTION_RAW_COLLECTION) assert(last_batch_enabled && last_batch_rate == 100);
    command(ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF, CONFIG_CONNECTION_RAW_COLLECTION, 0);
    if (CONFIG_CONNECTION_RAW_COLLECTION) assert(!last_batch_enabled && last_batch_rate == 0);
    command(ESB_PONG_FLAG_MAG_ON, CONFIG_SENSOR_USE_MAGNETOMETER, 0);
    assert(mag_enabled == !!CONFIG_SENSOR_USE_MAGNETOMETER);
    command(ESB_PONG_FLAG_MAG_OFF, CONFIG_SENSOR_USE_MAGNETOMETER, 0);
    assert(!mag_enabled);
    command(ESB_PONG_FLAG_MAG_CLEAR, CONFIG_SENSOR_USE_MAGNETOMETER, 0);
    command(ESB_PONG_FLAG_MAG_CAL, CONFIG_SENSOR_USE_MAGNETOMETER, CAL_KIND_MAG_MANUAL);
    command(ESB_PONG_FLAG_MAG_AUTO_ON, CONFIG_SENSOR_USE_MAGNETOMETER, CAL_KIND_MAG_ONLINE);
    assert(online_enabled == !!CONFIG_SENSOR_USE_MAGNETOMETER);
    command(ESB_PONG_FLAG_MAG_AUTO_OFF, CONFIG_SENSOR_USE_MAGNETOMETER, CAL_KIND_MAG_ONLINE);
    assert(!online_enabled);

    receive_control(ESB_PONG_FLAG_NORMAL);
    unsigned metadata_before = metadata_requests;
    rx_payload.data[8] = 0x3f;
    rx_payload.data[9] = 255;
    rx_payload.data[10] = 0x12;
    rx_payload.data[11] = 0x34;
    receive_control(ESB_PONG_FLAG_DATA_COLLECT_METADATA);
    assert(esb_get_ping_ack_flag() == (CONFIG_CONNECTION_RAW_COLLECTION ? ESB_PONG_FLAG_DATA_COLLECT_METADATA : ESB_PONG_FLAG_NORMAL));
    assert(metadata_requests == metadata_before + CONFIG_CONNECTION_RAW_COLLECTION);
    if (CONFIG_CONNECTION_RAW_COLLECTION) {
        assert(metadata_echo_pending && metadata_mask == 0x3f && metadata_chunk == 255 && metadata_token == 0x1234);
    } else {
        execute_owner();
        assert(remote_command_rejected && acked_remote_command == ESB_PONG_FLAG_NORMAL);
        unsigned rejected = led_test_events[LED_REJECTED];
        receive_control(ESB_PONG_FLAG_DATA_COLLECT_METADATA);
        execute_owner();
        assert(led_test_events[LED_REJECTED] == rejected && metadata_requests == metadata_before);
    }
    uint8_t metadata_echo[4];
    esb_get_ping_request_data(metadata_echo);
    const uint8_t expected_metadata_echo[4] = {0x3f, 0xff, 0x12, 0x34};
    const uint8_t unavailable_metadata_echo[4] = {0};
    assert(memcmp(metadata_echo, CONFIG_CONNECTION_RAW_COLLECTION ? expected_metadata_echo : unavailable_metadata_echo,
                  sizeof(metadata_echo)) == 0);
    if (getenv("CAPABILITY_SMOKE")) printf("remote_metadata raw=%d mag=%d echo=0x%02x data_hex=%02x%02x%02x%02x requests=%u rejected=%d\n",
        CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER, esb_get_ping_ack_flag(),
        metadata_echo[0], metadata_echo[1], metadata_echo[2], metadata_echo[3],
        metadata_requests - metadata_before, remote_command_rejected);
    receive_control(ESB_PONG_FLAG_NORMAL);
    assert(!metadata_echo_pending && received_remote_command == ESB_PONG_FLAG_NORMAL);
    rx_payload.length = 4;
    memcpy(rx_payload.data, (uint8_t[]){RAW_ARQ_MARKER, 1, 0x12, 0x34}, 4);
    receive_auxiliary();
    assert(retransmit_calls == CONFIG_CONNECTION_RAW_COLLECTION);
    if (CONFIG_CONNECTION_RAW_COLLECTION) assert(retransmit_sequence == 0x1234);
    const uint8_t ota[] = {ESB_OTA_DATA_TYPE, 0x31, 0x42, 0x53, 0x64};
    rx_payload.length = sizeof(ota);
    memcpy(rx_payload.data, ota, sizeof(ota));
    receive_auxiliary();
    assert(ota_rx_head == 1 && ota_rx_tail == 0);
    assert(ota_rx_queue[0].length == sizeof(ota));
    assert(memcmp(ota_rx_queue[0].data, ota, sizeof(ota)) == 0);
    if (getenv("CAPABILITY_SMOKE")) {
        printf("remote_auxiliary raw=%d mag=%d retransmit_calls=%u retransmit_sequence=%u ota_head=%u ota_tail=%u ota_hex=",
               CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER,
               retransmit_calls, retransmit_sequence, ota_rx_head, ota_rx_tail);
        for (size_t i = 0; i < ota_rx_queue[0].length; i++) printf("%02x", ota_rx_queue[0].data[i]);
        putchar('\n');
    }
    assert(test_mode_changes == (CONFIG_CONNECTION_RAW_COLLECTION ? 4U : 0U));
    printf("capability command admission raw=%d mag=%d: PASS\n", CONFIG_CONNECTION_RAW_COLLECTION, CONFIG_SENSOR_USE_MAGNETOMETER);
    return 0;
}
