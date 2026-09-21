/* Included after the production core; each binary represents one Kconfig combination. */
static unsigned seen;
static unsigned boot_notices, watchdog_notices, fusion_backends;

static void pump(uint32_t until, bool observe)
{
    for (; now_ms <= until; now_ms += 10) {
        if (observe) {
            tracker_events_observe_sensor(tracker_events_sensor_epoch(), true, true,
                true, true, now_ms < 4000 ? FUSION_BACKEND_VQF : FUSION_BACKEND_EQF, now_ms);
        }
        tracker_events_notify();
        struct tracker_event_tx tx;
        if (tracker_events_select(now_ms, 2, &tx)) {
            struct tracker_event event;
            assert(tracker_event_decode(tx.packet, sizeof(tx.packet), &event));
            unsigned category = tracker_event_kind_filter(event.kind);
            assert(category & (TEST_EVENT_MASK | 16));
            seen |= category;
            if (event.kind == TRACKER_EVENT_KIND_POWER) {
                boot_notices += event.phase == POWER_BOOT;
                watchdog_notices += event.phase == POWER_WATCHDOG_RESET;
            }
            if (event.kind == TRACKER_EVENT_KIND_FUSION_REST && event.phase == FUSION_REST_DETECTED) {
                fusion_backends |= 1U << event.detail;
            }
            tracker_events_complete(tx.token, true, now_ms);
        }
        if (!TEST_EVENT_MASK) {
            assert(tracker_events_deadline(now_ms) == UINT32_MAX);
            assert(wake_calls == 0);
        }
    }
}

int main(void)
{
    assert(host_init() == 0);
    uint16_t op = cal_event_accept(CAL_KIND_IMU_ZRO);
    cal_event_start(op, CAL_PHASE_WAIT_STILL, 0);
    cal_event_step(op, CAL_PHASE_COLLECT, 0);
    uint16_t online = cal_event_begin(CAL_KIND_MAG_ONLINE | CAL_EVENT_ORIGIN_AUTO, CAL_PHASE_FREEZE, 0);
    cal_event_reject(CAL_KIND_ACCEL_POSES, CAL_REASON_BUSY);
    tracker_events_schedule_boot(false, true);
    tracker_event_notice(TRACKER_EVENT_KIND_POWER, POWER_WILL_WOM, POWER_WOM_NORMAL);
    tracker_event_set_state(TRACKER_EVENT_KIND_TRACKER_REST, TRACKER_REST_REST, TRACKER_REST_OBSERVED);
    tracker_event_set_state(TRACKER_EVENT_KIND_FUSION_REST, FUSION_REST_DETECTED, FUSION_BACKEND_VQF);
    pump(7500, true); /* Includes delayed boot, repeats, calibration/state heartbeats, both fusion backends. */
    cal_event_end(op, CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
    cal_event_end(online, CAL_OUTCOME_FAILED, CAL_PHASE_VALIDATE, CAL_REASON_RADIAL);
    tracker_event_notice(TRACKER_EVENT_KIND_POWER, POWER_WOM_CANCELLED, POWER_WOM_NORMAL);
    uint32_t previous_epoch = tracker_events_sensor_epoch();
    tracker_events_sensor_invalidate(TRACKER_REST_SUSPENDED);
    assert(tracker_events_sensor_epoch() != previous_epoch);
    tracker_events_observe_sensor(previous_epoch, true, true, true, true, FUSION_BACKEND_VQF, now_ms);
    pump(25000, false);
    assert(seen == TEST_EVENT_MASK);
    assert(!!boot_notices == !!(TEST_EVENT_MASK & 8));
    assert(!!watchdog_notices == !!(TEST_EVENT_MASK & 8));
    assert(fusion_backends == ((TEST_EVENT_MASK & 4) ? 6U : 0U));
    assert(tracker_events_deadline(now_ms) == UINT32_MAX);

    tracker_events_session_changed();
    tracker_events_observe_sensor(tracker_events_sensor_epoch(), true, false,
        true, false, FUSION_BACKEND_VQF, now_ms);
    pump(45000, false); /* Fresh identity and stale observations must respect the same mask. */
    assert(tracker_events_deadline(now_ms) == UINT32_MAX);

    /* Buttons remain usable even when every configurable class is disabled. */
    tracker_event_notice(TRACKER_EVENT_KIND_BUTTON, BUTTON_CLICK_GROUP, 3);
    struct tracker_event_tx tx;
    assert(tracker_events_select(now_ms, 2, &tx));
    struct tracker_event event;
    assert(tracker_event_decode(tx.packet, sizeof(tx.packet), &event));
    assert(event.kind == TRACKER_EVENT_KIND_BUTTON && event.detail == 3);
    tracker_events_complete(tx.token, true, now_ms);
    printf("PASS event configuration mask=%x: category isolation, lifecycle, deadlines, buttons\n", TEST_EVENT_MASK);
    return 0;
}
