#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define K_FOREVER -1
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define ESB_DEFAULT_CONFIG {0}
#define ESB_PROTOCOL_ESB_DPL 1
#define ESB_BITRATE_2MBPS 2
#define ESB_TXMODE_MANUAL_START 3
#define ESB_MODE_PRX 4
#define CONFIG_RADIO_TX_POWER 0
#define RADIO_RETRANSMIT_DELAY 600
#define RADIO_RF_CHANNEL 80
#define ESB_RF_CHANNEL_DEFAULT 0xff
#define SYS_STATUS_CONNECTION_ERROR 1
struct esb_config {
    int protocol, bitrate, tx_output_power, retransmit_delay, retransmit_count, tx_mode, mode;
    bool selective_auto_ack, use_fast_ramp_up;
    void (*event_handler)(void), (*tx_capture_handler)(void);
};
static void event_handler(void) {}
#ifdef CONFIG_TDMA_DIAGNOSTICS
static void radio_capture_record(void) {}
static unsigned capture_live;
static int radio_capture_init(void) { capture_live = 1; return 0; }
static void radio_capture_deinit(void) { capture_live = 0; }
#endif
static int esb_radio_lock, lock_depth;
static void k_mutex_lock(int *lock, int timeout) { lock_depth++; }
static void k_mutex_unlock(int *lock) { assert(lock_depth > 0); lock_depth--; }
static bool esb_initialized, own_pong_seen, channel_search, channel_wait_normal;
static bool channel_found, channel_heard, ping_pending;
static uint32_t own_pong_time, radio_session_generation;
static uint8_t radio_channel, base_addr_0[4], base_addr_1[4], addr_prefix[8];
static struct { uint8_t rf_channel; } storage, *retained = &storage;
static uint8_t esb_rf_channel_decode(uint8_t channel) { return channel; }
static void retained_update(void) {}
static int fail_step, steps, disable_calls;
static bool driver_live, connection_error;
static int step(void) { return ++steps == fail_step ? -EIO : 0; }
static int esb_init(const struct esb_config *config) { int err = step(); driver_live = !err; return err; }
static int esb_set_rf_channel(uint8_t channel) { return step(); }
static int esb_set_base_address_0(uint8_t *address) { return step(); }
static int esb_set_base_address_1(uint8_t *address) { return step(); }
static int esb_set_prefixes(uint8_t *prefix, size_t count) { return step(); }
static void esb_disable(void) { assert(driver_live); driver_live = false; disable_calls++; }
static void set_status(int mask, bool value) { connection_error = value; }
static void esb_clear_time_sync_state(void) {}
static void tdma_set_enabled(bool enabled) {}
static int64_t now_ms;
static int64_t k_uptime_get(void) { return now_ms; }
static uint32_t next_ping_deadline_ms;
static uint32_t atomic_get(uint32_t *value) { return *value; }
static bool ready, guarded, test_enabled, test_wake_delay_valid;
static uint32_t window_delay, event_deadline;
static uint64_t test_wake_delay_us;
static bool esb_ready(void) { return ready; }
static bool tdma_ping_wake_delay_ms(uint32_t *delay) { *delay = window_delay; return guarded; }
static bool test_mode_get(void) { return test_enabled; }
static uint64_t k_uptime_ticks(void) { return now_ms * 1000; }
static uint64_t k_ticks_to_us_near64(uint64_t ticks) { return ticks; }
static uint64_t test_rate_delay_us(uint64_t now) { return 1000000; }
static bool esb_ota_is_active(void) { return false; }
static int get_status(int mask) { return 0; }
static uint32_t tracker_events_deadline(uint32_t now) { return event_deadline; }
static int sensor_data_snapshot;
static bool sensor_data_snapshot_m_pending(int *snapshot) { return false; }
static bool sensor_ids_set;
static int64_t last_mag_time, last_info_time, last_status_time, last_runtime_time;
#define K_NO_WAIT 0
#define K_USEC(us) (us)
#define K_MSEC(ms) ((ms) * 1000)
static int connection_wake_sem;
static int64_t waited_us;
static int k_sem_take(int *sem, int64_t timeout) { waited_us = timeout; return 0; }
#define OTA_FLASH_PAGE_SIZE 4096
#define BOOTLOADER_SETTINGS_ADDR 0xff000
#define BANK_VALID_APP 1
#define BANK_INVALID_APP 0xff
#define MIN(a,b) ((a) < (b) ? (a) : (b))
static int flash_dev;
static uint8_t image[8200];
static int reads, fail_read;
static int flash_read(int dev, uint32_t offset, uint8_t *buffer, size_t length)
{
    if (++reads == fail_read) return -EIO;
    assert(offset + length <= sizeof(image));
    memcpy(buffer, image + offset, length);
    return 0;
}
#if !LEGACY_CRC16
#define CONFIG_SOC_NRF52840 1
#define BUILD_ASSERT(condition, message) _Static_assert(condition, message)
static struct { unsigned CTRL; } mpu;
#define MPU (&mpu)
static bool irq_disabled;
static void __disable_irq(void) { irq_disabled = true; }
static void __DSB(void) {}
static void __ISB(void) {}
static void k_msleep(unsigned ms) { assert(ms == 500); }
#endif
#include "production.inc"
#if !LEGACY_CRC16
static unsigned copy_calls;
static void ota_flash_copy_from_ram(const struct flash_copy_params *p)
{
    assert(irq_disabled && MPU->CTRL == 0);
    assert(p->src_addr == 0x80000 && p->dst_addr == 0x10000);
    assert(p->size == 4100 && p->page_size == OTA_FLASH_PAGE_SIZE);
    if (bl_settings_prepared) {
        assert(p->settings_addr == BOOTLOADER_SETTINGS_ADDR);
        assert(p->settings_words == sizeof(prepared_bl_settings) / 4);
        assert(memcmp(p->settings_data, &prepared_bl_settings, sizeof(prepared_bl_settings)) == 0);
    } else {
        assert(!p->settings_addr && !p->settings_words);
    }
    copy_calls++;
}
static void test_copy_launch(void)
{
    for (unsigned prepared = 0; prepared < 2; prepared++) {
        bl_settings_prepared = prepared;
        MPU->CTRL = 1; irq_disabled = false;
        esb_ota_flash_copy_and_reset(0x80000, 0x10000, 4100);
    }
    assert(copy_calls == 2);
}
#endif

static void test_initialization(void)
{
    for (unsigned retained_channel = 0; retained_channel < 2; retained_channel++) {
        storage.rf_channel = retained_channel ? 42 : ESB_RF_CHANNEL_DEFAULT;
        for (int failure = 1; failure <= 5; failure++) {
            fail_step = failure; steps = 0; disable_calls = 0;
            uint32_t generation = radio_session_generation;
            assert(esb_initialize(true) == -EIO);
            assert(!esb_initialized && !driver_live && lock_depth == 0);
            assert(connection_error && radio_session_generation == generation);
            assert(steps == failure && disable_calls == (failure != 1));
#ifdef CONFIG_TDMA_DIAGNOSTICS
            assert(!capture_live);
#endif
            fail_step = 0; steps = 0;
            assert(esb_initialize(false) == 0);
            assert(esb_initialized && driver_live && steps == 5 && lock_depth == 0);
            assert(radio_channel == (retained_channel ? 42 : RADIO_RF_CHANNEL));
            assert(radio_session_generation == generation + 1);
            esb_deinitialize();
        }
    }
    steps = 0;
    assert(esb_initialize(true) == 0);
    steps = 0; fail_step = 3;
    assert(esb_initialize(true) == -EIO);
    assert(!driver_live && !esb_initialized && !lock_depth);
}

static void test_deadlines(void)
{
    const int64_t times[] = { 1000, INT64_C(0xffffffff) - 20,
        INT64_C(0x100000000) + 20, INT64_C(0x200000000) + 20 };
    ready = true;
    for (size_t i = 0; i < ARRAY_SIZE(times); i++) {
        now_ms = times[i]; last_status_time = last_runtime_time = now_ms;
        event_deadline = UINT32_MAX;
        next_ping_deadline_ms = (uint32_t)(now_ms + 50);
        assert(connection_next_deadline_ms(now_ms) == now_ms + 50);
        connection_idle_wait(now_ms); assert(waited_us == 50000);
        next_ping_deadline_ms = (uint32_t)(now_ms - 5);
        assert(connection_next_deadline_ms(now_ms) == now_ms - 5);
        connection_idle_wait(now_ms); assert(waited_us == 0);
        next_ping_deadline_ms = (uint32_t)now_ms;
        connection_idle_wait(now_ms); assert(waited_us == 0);
        next_ping_deadline_ms = (uint32_t)(now_ms + 2000);
        assert(connection_next_deadline_ms(now_ms) == now_ms + 1000);
        event_deadline = (uint32_t)(now_ms + 15);
        assert(connection_next_deadline_ms(now_ms) == now_ms + 15);
        event_deadline = UINT32_MAX;
        guarded = true; window_delay = 25;
        assert(connection_next_deadline_ms(now_ms) == now_ms + 25);
        guarded = false;
    }
    /* UINT32_MAX is an event sentinel, not a sentinel for periodic PING. */
    now_ms = INT64_C(0x100000000) - 10;
    last_status_time = last_runtime_time = now_ms;
    next_ping_deadline_ms = UINT32_MAX;
    assert(connection_next_deadline_ms(now_ms) == now_ms + 9);
    ready = false;
    assert(connection_next_deadline_ms(now_ms) == now_ms + 1000);
}

static void test_crc16(void)
{
    uint8_t scratch[OTA_FLASH_PAGE_SIZE];
    /* Find a genuine nonempty FFFF image using an independent bitwise CRC. */
    bool found = false;
    for (unsigned word = 0; word <= 0xffff; word++) {
        uint16_t crc = 0xffff;
        uint8_t bytes[2] = { word >> 8, word };
        for (unsigned b = 0; b < 2; b++) {
            crc ^= (uint16_t)bytes[b] << 8;
            for (unsigned bit = 0; bit < 8; bit++)
                crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
        }
        if (crc == 0xffff) { memcpy(image, bytes, 2); found = true; break; }
    }
    assert(found);
    reads = fail_read = 0;
    assert(esb_ota_flash_prepare_bootloader_settings(0, 2, scratch) == 0);
    assert(bl_settings_prepared && prepared_bl_settings.bank_0_crc == 0xffff);
    assert(prepared_bl_settings.bank_0 == BANK_VALID_APP && prepared_bl_settings.bank_0_size == 2);
    for (int failure = 1; failure <= 2; failure++) {
        reads = 0; fail_read = failure;
        assert(esb_ota_flash_prepare_bootloader_settings(0, 4100, scratch) == -EIO);
        assert(!bl_settings_prepared);
#if !LEGACY_CRC16
        uint16_t result = 0x1234;
        reads = 0;
        assert(esb_ota_flash_compute_crc16_nordic(0, 4100, scratch, &result) == -EIO);
        assert(result == 0x1234);
#endif
    }
    reads = fail_read = 0;
    image[0] = image[1] = 0xff;
    assert(esb_ota_flash_prepare_bootloader_settings(0, 2, scratch) == -EINVAL);
    assert(!bl_settings_prepared);
    memcpy(image, "123456789", 9);
    assert(esb_ota_flash_prepare_bootloader_settings(0, 9, scratch) == 0);
    assert(prepared_bl_settings.bank_0_crc == 0x29b1);
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1], "init")) test_initialization();
    else if (!strcmp(argv[1], "deadline")) test_deadlines();
    else if (!strcmp(argv[1], "crc16")) test_crc16();
    else {
#if !LEGACY_CRC16
        assert(!strcmp(argv[1], "copy"));
        test_copy_launch();
#else
        assert(!"unknown regression");
#endif
    }
    puts("radio/OTA regression passed");
    return 0;
}
