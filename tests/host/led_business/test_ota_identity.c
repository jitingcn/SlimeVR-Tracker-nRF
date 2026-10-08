#include <errno.h>
#include "../led_sync/test_runtime.h"

#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define OTA_SUPPORTED 1
#define OTA_USE_MCUBOOT 1
#define OTA_USE_RAM_ENGINE 0
#define CONFIG_BOARD_HAS_NRF5_BOOTLOADER 0
#define __aligned(n) __attribute__((aligned(n)))
typedef int atomic_t;
static int atomic_get(const atomic_t *value) { return *value; }
static int atomic_set(atomic_t *value, int next) { int old = *value; *value = next; return old; }
#define atomic_clear(value) atomic_set(value, 0)
static bool atomic_cas(atomic_t *value, int expected, int desired)
{ if (*value != expected) return false; *value = desired; return true; }
static void k_msleep(unsigned milliseconds) { host_now_ms += milliseconds; }
static int sys_ota_reboot_reserve(void) { return 0; }
static void sys_ota_reboot_resolve(bool commit) { (void)commit; }
static int esb_ota_flash_request_mcuboot_upgrade(void) { return 0; }
static void ota_update_led(void);
static void ota_send_status(void) { ota_update_led(); }
#include "ota_production.inc"

static void accepted_update(void)
{
    host_reset(LED_CAP_RGB_PWM);
    memset(&ota, 0, sizeof(ota));
    atomic_set(&ota_reboot_pending, 0);
    atomic_set(&ota_abort_requested, 0);
    ota_feedback = host_begin(LED_OWNER_RADIO, LED_OTA_ACTIVE);
    ota_feedback_revision = 1;
    ota_feedback_terminal = false;
    ota_feedback_state = LED_OTA_ACTIVE;
    ota.state = OTA_STATE_RECEIVING;
    ota.session_started = true;
    ota_update_led();
    host_step(0);
    assert(engine.winner.semantic == LED_OTA_ACTIVE);
}
static void reject_new_begin(void)
{
    struct led_token accepted = ota_feedback;
    uint8_t invalid = 0;
    assert(esb_ota_handle_begin(&invalid, sizeof(invalid)) == -EINVAL);
    assert(engine.owners[LED_OWNER_RADIO].session == accepted.session);
    assert(engine.terminal_request_id[LED_OWNER_RADIO] != accepted.request_id);
    assert(engine.owners[LED_OWNER_RADIO].terminal == LED_NONE);
}
int main(void)
{
    accepted_update(); reject_new_begin();
    ota.state = OTA_STATE_ERROR;
    ota.error_code = OTA_STATUS_FLASH_ERROR;
    ota_update_led();
    assert(ota_feedback_terminal && engine.owners[LED_OWNER_RADIO].terminal == LED_FAILED);
    host_step(300);
    assert(engine.winner.semantic == LED_FAILED && host_role == LED_ROLE_NEGATIVE);
    host_step(5000);
    assert(engine.winner.semantic == LED_OTA_ACTIVE); /* Actual ERROR still owns reboot lifecycle. */
    assert(esb_ota_handle_activate() == -EINVAL);
    host_step(5400);
    assert(engine.winner.semantic == LED_REJECTED && host_role == LED_ROLE_NEGATIVE);

    accepted_update(); reject_new_begin();
    host_step(300);
    esb_ota_request_abort();
    esb_ota_service();
    assert(ota_feedback_terminal && engine.owners[LED_OWNER_RADIO].terminal == LED_CANCELLED);
    /* A delayed worker cannot start cancellation unless the complete short
     * receipt still fits its TTL, even though the business session ended. */
    host_step(2600);
    assert(engine.winner.semantic != LED_CANCELLED && host_role != LED_ROLE_POSITIVE);
    host_step(7000);
    assert(engine.winner.semantic == LED_OTA_ACTIVE);

    accepted_update(); reject_new_begin();
    host_step(300);
    /* Expire the visible higher-priority refusal, not its newer request
     * identity: the old cancellation must still be session-valid afterward. */
    host_step(7000);
    esb_ota_request_abort();
    esb_ota_service();
    assert(ota_feedback_terminal && engine.owners[LED_OWNER_RADIO].terminal == LED_CANCELLED);
    assert(ota.state == OTA_STATE_IDLE && atomic_get(&ota_reboot_pending));
    assert(esb_ota_is_active() && engine.owners[LED_OWNER_RADIO].ota_active);
    bool saw_cancellation = false;
    for (unsigned frame = 0; frame < 40 && !saw_cancellation; ++frame) {
        host_step(host_now_ms + 20);
        saw_cancellation = engine.winner.semantic == LED_CANCELLED && host_value > 0;
    }
    assert(saw_cancellation && host_role != LED_ROLE_POSITIVE);
    host_step(10000);
    assert(engine.winner.semantic == LED_OTA_ACTIVE);
    struct led_connection_facts facts = {.healthy = true, .output_ready = true, .radio_required = true, .paired = true};
    led_connection_publish(&facts);
    host_step(10500);
    assert(!engine.ready && engine.winner.semantic == LED_OTA_ACTIVE);
    puts("OTA ownership: newer refused request cannot discard old failure/cancel; ERROR and wire-IDLE reboot-pending retain OTA blocker and never show update-success or READY");
    return 0;
}
