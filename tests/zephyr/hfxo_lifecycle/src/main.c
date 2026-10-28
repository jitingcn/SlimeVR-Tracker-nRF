#include <zephyr/irq.h>
#include <zephyr/irq_offload.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/onoff.h>
#include <zephyr/ztest.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "thread_priority.h"
#include "posix_board_if.h"

/* Clock, startup, radio-disable and writer bodies are extracted from the
 * tracker; pairing admission is followed by a modeled send leaf. Zephyr's
 * onoff manager, notifications, scheduler, locks, and the nrfx reference
 * counter are real. The oscillator delay and peripheral leaves are modeled. */
#define LOG_ERR(...) do { } while (0)
#define LOG_DBG(...) do { } while (0)
#define LOG_INF(...) do { } while (0)
#define NRF_CLOCK_HAS_PLL HFXO_NRF54
#define NRF_CLOCK NULL
#define NRF_CLOCK_TASK_PLLSTART 1
#define NRF_POWER NULL
#define NRF_POWER_TASK_CONSTLAT 1
#define NRF_POWER_TASK_LOWPWR 2
#define NRFX_ASSERT(x) __ASSERT_NO_MSG(x)
#define NRFX_CRITICAL_SECTION_ENTER() unsigned int nrfx_key = irq_lock()
#define NRFX_CRITICAL_SECTION_EXIT() irq_unlock(nrfx_key)
#define CLOCK_CONTROL_NRF_SUBSYS_HF 0
#define ESB_PING_LEN 13
#define TDMA_PING_DEFERRED 1
#define led_result(...) do { } while (0)
#define led_maintenance_publish(...) do { } while (0)
#define led_request_event(...) do { } while (0)

enum { ESB_ST_PAIRING = 1, ESB_ST_PAIRED = 2 };
enum start_mode { START_TIMED, START_MANUAL, START_SYNC_OK, START_SYNC_ERROR };

static enum start_mode start_mode;
static struct onoff_manager managers[32];
static unsigned int next_manager;
static struct onoff_manager *fixture_manager;
static struct onoff_manager *pending_manager;
static onoff_notify_fn pending_notify;
static bool fixture_idle, fixture_collection, wake_sensor;
static bool esb_initialized, radio_user_disabled, radio_payload_queued;
static bool pair_ack_pending, pairing_search_active, pair_provisional;
static bool channel_search, channel_wait_normal, channel_found, channel_heard;
static bool channel_redirect_pending, channel_confirmed, channel_legacy_peer;
static bool ping_channel_confirm_sent, channel_confirm_capable, ping_pending;
static bool pairing_send_attempted, lf_initialized, lf_started_without_hf;
static bool pair_send_without_clock, pair_send_without_lease, pair_step2_seen;
static bool pair_stop_lost_clock;
static uint32_t radio_session_generation;
static uint8_t paired_addr[8], ping_history[32];
static int esb_conn_state;
static struct k_mutex esb_radio_lock;
static struct k_sem request_seen, starter_entered[2], starter_done[2];
static struct k_sem sensor_ready, sensor_go, sensor_done;
static atomic_t completion_fired, sensor_before_completion;
static struct k_thread starters[2], sensor_thread, writer_thread, pair_stopper;
static K_THREAD_STACK_ARRAY_DEFINE(starter_stacks, 2, 4096);
static K_THREAD_STACK_DEFINE(sensor_stack, 4096);
static K_THREAD_STACK_DEFINE(writer_stack, 4096);
static K_THREAD_STACK_DEFINE(pair_stopper_stack, 4096);
static struct k_sem writer_entered, writer_done, pair_step_seen[3], pair_stopper_done;
static int writer_result;
static int starter_result[2];

void clocks_stop(void);
static inline bool esb_is_idle(void) { return fixture_idle; }
static inline bool connection_get_data_collection(void) { return fixture_collection; }
static void esb_wom_reset(void) { }
static void esb_clear_time_sync_state(void) { }
static void tdma_set_enabled(bool enabled) { ARG_UNUSED(enabled); }
static void esb_disable(void) { }
static int esb_reinitialize(void) { return -ENOTSUP; }
static int tdma_wait_for_ping_window(void) { return 0; }
static int esb_send_pair_step(int step);
static int esb_write_clocked(uint8_t *data, bool no_ack, size_t length)
{
    ARG_UNUSED(data);
    ARG_UNUSED(no_ack);
    ARG_UNUSED(length);
    if (!esb_initialized || esb_conn_state == ESB_ST_PAIRING) { return -EACCES; }
    radio_payload_queued = true;
    return -ENODATA;
}
extern bool clock_status;
static void clock_init_external_async(void)
{
    lf_initialized = true;
    if (!clock_status) { lf_started_without_hf = true; }
}
static struct onoff_manager *z_nrf_clock_control_get_onoff(int subsystem)
{
    ARG_UNUSED(subsystem);
    return fixture_manager;
}

#if HFXO_NRF54
static void nrf_power_task_trigger(void *power, int task)
{
    ARG_UNUSED(power);
    ARG_UNUSED(task);
}
static void nrf_clock_task_trigger(void *clock, int task)
{
    ARG_UNUSED(clock);
    ARG_UNUSED(task);
}
#include "nrfx_power.inc"
#endif

/* Observe accepted requests without replacing the SDK manager. native_sim
 * instructions otherwise take no simulated time. Charging 100 us to each
 * production yield makes the pre-fix spin loop terminate at the modeled timer
 * deadline; it does not change scheduler priorities or production logic. */
static int observed_request(struct onoff_manager *mgr, struct onoff_client *client)
{
    int result = onoff_request(mgr, client);
    if (result >= 0) {
        k_sem_give(&request_seen);
    }
    return result;
}
static inline void observed_yield(void)
{
    posix_cpu_hold(100);
    k_yield();
}

/* Test fixture calls the real initializer in before(), after installing the
 * controlled manager, instead of invoking SYS_INIT at native_sim boot. */
#undef SYS_INIT
#define SYS_INIT(...)
#define onoff_request observed_request
#define k_yield observed_yield
#include "tracker_clock.inc"
#undef k_yield
#undef onoff_request
static int esb_send_pair_step(int step)
{
    pairing_send_attempted = true;
    if (!clock_status) { pair_send_without_clock = true; }
    if (!atomic_get(&tx_clock_users)) { pair_send_without_lease = true; }
    if (step == 2) { pair_step2_seen = true; }
    if (step >= 0 && step < 3) { k_sem_give(&pair_step_seen[step]); }
    return 0;
}
#include "tracker_pair_gate.inc"
#include "tracker_boot.inc"
#include "tracker_radio.inc"

static void complete_start(int result)
{
    struct onoff_manager *mgr = pending_manager;
    onoff_notify_fn notify = pending_notify;
    __ASSERT_NO_MSG(notify != NULL);
    pending_manager = NULL;
    pending_notify = NULL;
    atomic_set(&completion_fired, 1);
    notify(mgr, result);
}
static void timer_done(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    complete_start(0);
}
K_TIMER_DEFINE(oscillator_timer, timer_done, NULL);

static void oscillator_start(struct onoff_manager *mgr, onoff_notify_fn notify)
{
    pending_manager = mgr;
    pending_notify = notify;
    if (wake_sensor) {
        k_sem_give(&sensor_go);
    }
    if (start_mode == START_SYNC_OK) {
        complete_start(0);
    } else if (start_mode == START_SYNC_ERROR) {
        complete_start(-ETIMEDOUT);
    } else if (start_mode == START_TIMED) {
        k_timer_start(&oscillator_timer, K_MSEC(20), K_NO_WAIT);
    }
}
static void oscillator_stop(struct onoff_manager *mgr, onoff_notify_fn notify)
{
    notify(mgr, 0);
}
static const struct onoff_transitions transitions =
    ONOFF_TRANSITIONS_INITIALIZER(oscillator_start, oscillator_stop, NULL);

static void before(void *data)
{
    ARG_UNUSED(data);
    zassert_true(next_manager < ARRAY_SIZE(managers));
    fixture_manager = &managers[next_manager++];
    zassert_equal(onoff_manager_init(fixture_manager, &transitions), 0);
    zassert_equal(clocks_init(), 0);
    k_timer_stop(&oscillator_timer);
    k_sem_init(&request_seen, 0, 4);
    for (size_t i = 0; i < ARRAY_SIZE(starter_done); ++i) {
        k_sem_init(&starter_entered[i], 0, 1);
        k_sem_init(&starter_done[i], 0, 1);
        starter_result[i] = -999;
    }
    k_sem_init(&sensor_ready, 0, 1);
    k_sem_init(&sensor_go, 0, 1);
    k_sem_init(&sensor_done, 0, 1);
    k_sem_init(&writer_entered, 0, 1);
    k_sem_init(&writer_done, 0, 1);
    for (size_t i = 0; i < ARRAY_SIZE(pair_step_seen); ++i) {
        k_sem_init(&pair_step_seen[i], 0, 1);
    }
    k_sem_init(&pair_stopper_done, 0, 1);
    k_mutex_init(&esb_radio_lock);
    zassert_false(clock_status, "previous test left the application clock owned");
    zassert_equal(atomic_get(&tx_clock_users), 0, "previous test left a TX user");
    atomic_clear(&completion_fired);
    atomic_clear(&sensor_before_completion);
#if HFXO_NRF54
    zassert_equal(m_power_mode_refs, 0, "previous test left constant latency owned");
#endif
    pending_manager = NULL;
    pending_notify = NULL;
    fixture_idle = true;
    fixture_collection = wake_sensor = false;
    esb_initialized = true;
    radio_user_disabled = radio_payload_queued = false;
    pairing_search_active = pair_provisional = false;
    pairing_send_attempted = lf_initialized = lf_started_without_hf = false;
    pair_send_without_clock = pair_send_without_lease = pair_step2_seen = false;
    pair_stop_lost_clock = false;
    writer_result = -999;
    esb_conn_state = ESB_ST_PAIRED;
    start_mode = START_TIMED;
}

static void take(struct k_sem *sem)
{
    zassert_equal(k_sem_take(sem, K_SECONDS(2)), 0, "test rendezvous timed out");
}
static void join(struct k_thread *thread)
{
    zassert_equal(k_thread_join(thread, K_SECONDS(2)), 0, "test thread did not finish");
}
static void expect_power(unsigned int refs)
{
#if HFXO_NRF54
    zassert_equal(m_power_mode_refs, refs, "nrfx constant-latency ownership");
#else
    ARG_UNUSED(refs);
#endif
}
static void stop_from_isr(const void *unused)
{
    ARG_UNUSED(unused);
    clocks_stop();
}
static void stop_in_isr(void)
{
    irq_offload(stop_from_isr, NULL);
}
/* LF_SYNTH intentionally never releases the app reference in production.
 * Between isolated test boots, release the real SDK reference and matching
 * nrfx ownership first, then reset only the app's bookkeeping. */
static void release_synth_fixture_owner(void)
{
    if (!HFXO_SYNTH || !clock_status) { return; }
    zassert_true(onoff_release(clk_mgr) >= 0, "fixture HFXO release failed");
#if HFXO_NRF54
    zassert_true(m_power_mode_refs != 0, "fixture constant-latency owner missing");
    (void)nrfx_power_constlat_mode_free();
#endif
    clock_status = false;
#if FIXTURE_HAS_HF_OWNED
    clocks_hf_owned = false;
#endif
}

static void after(void *data)
{
    ARG_UNUSED(data);
    release_synth_fixture_owner();
}

static void starter_entry(void *index_arg, void *pair_arg, void *unused)
{
    unsigned int index = (uintptr_t)index_arg;
    bool pairing = (uintptr_t)pair_arg != 0;
    ARG_UNUSED(unused);
    k_sem_give(&starter_entered[index]);
    if (pairing) {
        pairing_clock_gate();
        starter_result[index] = 0;
    } else {
        starter_result[index] = clocks_start();
    }
    k_sem_give(&starter_done[index]);
}
static void spawn_starter(unsigned int index, int priority, bool pairing)
{
    k_thread_create(&starters[index], starter_stacks[index],
                    K_THREAD_STACK_SIZEOF(starter_stacks[index]), starter_entry,
                    (void *)(uintptr_t)index, (void *)(uintptr_t)pairing, NULL,
                    priority, 0, K_NO_WAIT);
}
static void finish_starter(unsigned int index)
{
    take(&starter_done[index]);
    join(&starters[index]);
}

static void sensor_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    k_sem_give(&sensor_ready);
    k_sem_take(&sensor_go, K_FOREVER);
    if (!atomic_get(&completion_fired)) {
        atomic_set(&sensor_before_completion, 1);
    }
    k_sem_give(&sensor_done);
}
static void sensor_progress(int priority)
{
    wake_sensor = true;
    k_thread_create(&sensor_thread, sensor_stack, K_THREAD_STACK_SIZEOF(sensor_stack),
                    sensor_entry, NULL, NULL, NULL, SENSOR_LOOP_THREAD_PRIORITY, 0, K_NO_WAIT);
    take(&sensor_ready);
    spawn_starter(0, priority, false);
    finish_starter(0);
    take(&sensor_done);
    join(&sensor_thread);
    int start_result = starter_result[0];
    int progressed = atomic_get(&sensor_before_completion);
    stop_in_isr();
    zassert_equal(start_result, 0);
    zassert_equal(progressed, 1, "lower-priority sensor must run while startup is pending");
    zassert_equal(clk_mgr->refs, HFXO_SYNTH ? 1 : 0);
    expect_power(HFXO_SYNTH ? 1 : 0);
}

ZTEST(hfxo, test_sensor_progress_during_connection_start)
{
    sensor_progress(CONNECTION_THREAD_PRIORITY);
}
ZTEST(hfxo, test_sensor_progress_during_esb_start)
{
    sensor_progress(ESB_THREAD_PRIORITY);
}

ZTEST(hfxo, test_pairing_and_start_share_one_reference)
{
    start_mode = START_MANUAL;
    esb_conn_state = ESB_ST_PAIRING;
    /* Startup and pairing both run at priority 5. The second uses the actual
     * pairing gate under the same radio mutex as the production pairing path. */
    spawn_starter(0, CLOCKS_START_THREAD_PRIORITY, false);
    take(&request_seen);
    spawn_starter(1, ESB_THREAD_PRIORITY, true);
    take(&starter_entered[1]);
    k_msleep(2);
    stop_in_isr();
    complete_start(0);
    finish_starter(0);
    finish_starter(1);
    unsigned int owned_refs = clk_mgr->refs;
#if HFXO_NRF54
    unsigned int power_refs = m_power_mode_refs;
#endif
    esb_conn_state = ESB_ST_PAIRED;
    stop_in_isr();
    unsigned int remaining_refs = clk_mgr->refs;
#if HFXO_NRF54
    unsigned int remaining_power_refs = m_power_mode_refs;
#endif
    /* Clean leaked references on the pre-fix snapshot, after recording the
     * invariant violation; all releases still use the real SDK manager. */
    release_synth_fixture_owner();
    while (clk_mgr->refs != 0) { (void)onoff_release(clk_mgr); }
#if HFXO_NRF54
    while (m_power_mode_refs != 0) { (void)nrfx_power_constlat_mode_free(); }
#endif
    start_mode = START_SYNC_OK;
    int restart_result = clocks_start();
    unsigned int restart_refs = clk_mgr->refs;
#if HFXO_NRF54
    unsigned int restart_power_refs = m_power_mode_refs;
#endif
    stop_in_isr();
    unsigned int restart_remaining_refs = clk_mgr->refs;
#if HFXO_NRF54
    unsigned int restart_remaining_power_refs = m_power_mode_refs;
#endif
    release_synth_fixture_owner();
    while (clk_mgr->refs != 0) { (void)onoff_release(clk_mgr); }
#if HFXO_NRF54
    while (m_power_mode_refs != 0) { (void)nrfx_power_constlat_mode_free(); }
#endif
    zassert_equal(starter_result[0], 0);
    zassert_equal(owned_refs, 1, "concurrent starts must share one app reference");
    zassert_true(pair_step2_seen, "pairing must reach its final send");
    zassert_false(pair_send_without_clock, "pairing must send with HFXO ready");
    zassert_equal(remaining_refs, HFXO_SYNTH ? 1 : 0, "stop must respect LF ownership");
    zassert_equal(restart_result, 0);
    zassert_equal(restart_refs, 1, "restart must own one app reference");
    zassert_equal(restart_remaining_refs, HFXO_SYNTH ? 1 : 0, "restart stop violated LF policy");
#if HFXO_NRF54
    zassert_equal(power_refs, 1, "constant latency must match HFXO ownership");
    zassert_equal(remaining_power_refs, HFXO_SYNTH ? 1 : 0, "constant-latency ownership mismatch");
    zassert_equal(restart_power_refs, 1, "restart must own one constant-latency reference");
    zassert_equal(restart_remaining_power_refs, HFXO_SYNTH ? 1 : 0, "restart constant-latency mismatch");
#endif
}

static void pairing_stopper_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    /* Priority 6 is lower than the pairing thread: each interrupt runs
     * during the following real 20/2/60+ ms pairing sleep. */
    for (size_t i = 0; i < ARRAY_SIZE(pair_step_seen); ++i) {
        take(&pair_step_seen[i]);
        stop_in_isr();
        if (!clock_status) { pair_stop_lost_clock = true; }
    }
    k_sem_give(&pair_stopper_done);
}

ZTEST(hfxo, test_pairing_lease_covers_interstep_and_response_waits)
{
    start_mode = START_SYNC_OK;
    esb_conn_state = ESB_ST_PAIRING;
    k_thread_create(&pair_stopper, pair_stopper_stack, K_THREAD_STACK_SIZEOF(pair_stopper_stack),
                    pairing_stopper_entry, NULL, NULL, NULL, ESB_THREAD_PRIORITY + 1, 0, K_NO_WAIT);
    spawn_starter(0, ESB_THREAD_PRIORITY, true);
    finish_starter(0);
    take(&pair_stopper_done);
    join(&pair_stopper);
    stop_in_isr();
    zassert_true(pair_step2_seen);
    zassert_false(pair_send_without_clock);
    zassert_false(pair_send_without_lease);
    zassert_false(pair_stop_lost_clock, "ISR stop removed HFXO during a pairing burst");
    zassert_equal(atomic_get(&tx_clock_users), 0);
    zassert_equal(clk_mgr->refs, HFXO_SYNTH ? 1 : 0);
    expect_power(HFXO_SYNTH ? 1 : 0);
}

ZTEST(hfxo, test_pairing_failed_clock_never_reaches_send)
{
    start_mode = START_SYNC_ERROR;
    esb_conn_state = ESB_ST_PAIRING;
    pairing_search_active = true;
    pairing_clock_gate();
    zassert_false(pairing_send_attempted);
    zassert_false(pairing_search_active);
    zassert_equal(atomic_get(&tx_clock_users), 0);
    zassert_equal(clk_mgr->refs, 0);
    expect_power(0);
}

ZTEST(hfxo, test_stop_pending_and_queued_starts_obeys_lf_policy)
{
    start_mode = START_MANUAL;
    spawn_starter(0, ESB_THREAD_PRIORITY, false);
    take(&request_seen);
    spawn_starter(1, ESB_THREAD_PRIORITY, false);
    take(&starter_entered[1]);
    k_msleep(2);
    stop_in_isr();
    zassert_equal(atomic_get(&completion_fired), 0);
    complete_start(0);
    finish_starter(0);
    finish_starter(1);
    unsigned int stale_refs = clk_mgr->refs;
    bool stale_status = clock_status;
#if HFXO_NRF54
    unsigned int stale_power_refs = m_power_mode_refs;
#endif
    /* Clean pre-fix leaked refs before exercising a fresh request. */
    stop_in_isr();
    release_synth_fixture_owner();
    while (clk_mgr->refs != 0) { (void)onoff_release(clk_mgr); }
#if HFXO_NRF54
    while (m_power_mode_refs != 0) { (void)nrfx_power_constlat_mode_free(); }
#endif
    start_mode = START_SYNC_OK;
    int restart_result = clocks_start();
    unsigned int restart_refs = clk_mgr->refs;
    stop_in_isr();
    unsigned int restart_remaining_refs = clk_mgr->refs;
    zassert_equal(starter_result[0], HFXO_SYNTH ? 0 : -ECANCELED);
    zassert_equal(starter_result[1], HFXO_SYNTH ? 0 : -ECANCELED);
    zassert_equal(stale_status, HFXO_SYNTH != 0);
    zassert_equal(stale_refs, HFXO_SYNTH ? 1 : 0, "pending start violated LF policy");
#if HFXO_NRF54
    zassert_equal(stale_power_refs, HFXO_SYNTH ? 1 : 0, "constant latency violated LF policy");
#endif
    zassert_equal(restart_result, 0);
    zassert_equal(restart_refs, 1);
    zassert_equal(restart_remaining_refs, HFXO_SYNTH ? 1 : 0);
    expect_power(HFXO_SYNTH ? 1 : 0);
}
ZTEST(hfxo, test_tx_user_preserves_pending_start)
{
    start_mode = START_MANUAL;
    atomic_set(&tx_clock_users, 1);
    spawn_starter(0, ESB_THREAD_PRIORITY, false);
    take(&request_seen);
    stop_in_isr();
    complete_start(0);
    finish_starter(0);
    int result = starter_result[0];
    unsigned int refs = clk_mgr->refs;
    esb_release_tx_clock(false);
    stop_in_isr();
    zassert_equal(result, 0, "TX ownership must suppress stop generation changes");
    zassert_equal(refs, 1);
    zassert_equal(clk_mgr->refs, HFXO_SYNTH ? 1 : 0);
    expect_power(HFXO_SYNTH ? 1 : 0);
}

ZTEST(hfxo, test_delayed_worker_stop_obeys_lf_policy)
{
    start_mode = START_SYNC_OK;
    clocks_request_start(5000);
    stop_in_isr();
    join(&clocks_thread_id);
    unsigned int stale_refs = clk_mgr->refs;
    bool stale_status = clock_status;
    stop_in_isr();
    release_synth_fixture_owner();
    while (clk_mgr->refs != 0) { (void)onoff_release(clk_mgr); }
#if HFXO_NRF54
    while (m_power_mode_refs != 0) { (void)nrfx_power_constlat_mode_free(); }
#endif
    zassert_equal(stale_status, HFXO_SYNTH != 0);
    zassert_equal(stale_refs, HFXO_SYNTH ? 1 : 0, "delayed worker violated LF policy");
}

ZTEST(hfxo, test_radio_off_pending_worker_obeys_lf_policy)
{
    start_mode = START_MANUAL;
    clocks_request_start(0);
    take(&request_seen);
    zassert_equal(esb_user_set_enabled(false), 0);
    zassert_true(radio_user_disabled);
    complete_start(0);
    join(&clocks_thread_id);
    unsigned int stale_refs = clk_mgr->refs;
    bool stale_status = clock_status;
    stop_in_isr();
    release_synth_fixture_owner();
    while (clk_mgr->refs != 0) { (void)onoff_release(clk_mgr); }
#if HFXO_NRF54
    while (m_power_mode_refs != 0) { (void)nrfx_power_constlat_mode_free(); }
#endif
    zassert_equal(stale_status, HFXO_SYNTH != 0);
    zassert_equal(stale_refs, HFXO_SYNTH ? 1 : 0, "radio-off violated LF policy");
}

ZTEST(hfxo, test_boot_after_radio_off_obeys_lf_policy)
{
    start_mode = START_SYNC_OK;
    zassert_equal(esb_user_set_enabled(false), 0);
    boot_clock_request();
    k_msleep(5);
    unsigned int stale_refs = clk_mgr->refs;
    bool stale_status = clock_status;
    stop_in_isr();
    release_synth_fixture_owner();
    while (clk_mgr->refs != 0) { (void)onoff_release(clk_mgr); }
#if HFXO_NRF54
    while (m_power_mode_refs != 0) { (void)nrfx_power_constlat_mode_free(); }
#endif
    zassert_true(lf_initialized, "LF startup must continue under either policy");
    if (HFXO_SYNTH) {
        zassert_false(lf_started_without_hf, "LF_SYNTH started before HFXO");
    }
    zassert_equal(stale_status, HFXO_SYNTH != 0);
    zassert_equal(stale_refs, HFXO_SYNTH ? 1 : 0, "boot violated LF policy");
}

static void writer_entry(void *ping_arg, void *unused1, void *unused2)
{
    uint8_t payload[ESB_PING_LEN] = {0};
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);
    k_sem_give(&writer_entered);
    writer_result = (uintptr_t)ping_arg
        ? esb_write_ping(payload, true) : esb_write(payload, true, sizeof(payload));
    k_sem_give(&writer_done);
}
static void stale_writer_after_off(bool ping)
{
    start_mode = START_SYNC_OK;
    fixture_idle = false;
    k_mutex_lock(&esb_radio_lock, K_FOREVER);
    k_thread_create(&writer_thread, writer_stack, K_THREAD_STACK_SIZEOF(writer_stack),
                    writer_entry, (void *)(uintptr_t)ping, NULL, NULL,
                    CONNECTION_THREAD_PRIORITY, 0, K_NO_WAIT);
    take(&writer_entered);
    /* The writer passes its initial admission check, then blocks on the held
     * radio mutex while this thread turns the radio off recursively. */
    k_msleep(2);
    int off_result = esb_user_set_enabled(false);
    k_mutex_unlock(&esb_radio_lock);
    take(&writer_done);
    join(&writer_thread);
    unsigned int stale_refs = clk_mgr->refs;
    bool stale_status = clock_status;
    stop_in_isr();
    release_synth_fixture_owner();
    while (clk_mgr->refs != 0) { (void)onoff_release(clk_mgr); }
#if HFXO_NRF54
    while (m_power_mode_refs != 0) { (void)nrfx_power_constlat_mode_free(); }
#endif
    zassert_equal(off_result, 0);
    zassert_equal(writer_result, -EACCES);
    zassert_false(radio_payload_queued);
    zassert_false(stale_status);
    zassert_equal(stale_refs, 0, "stale writer restarted HFXO after radio-off");
    zassert_equal(atomic_get(&tx_clock_users), 0);
}
ZTEST(hfxo, test_stale_data_writer_after_radio_off)
{
    stale_writer_after_off(false);
}
ZTEST(hfxo, test_stale_ping_writer_after_radio_off)
{
    stale_writer_after_off(true);
}


ZTEST(hfxo, test_tx_users_prevent_early_stop)
{
    start_mode = START_SYNC_OK;
    zassert_equal(clocks_start(), 0);
    atomic_set(&tx_clock_users, 2);
    stop_in_isr();
    zassert_true(clock_status);
    zassert_equal(clk_mgr->refs, 1);
    expect_power(1);
    esb_release_tx_clock(false);
    zassert_equal(atomic_get(&tx_clock_users), 1);
    zassert_true(clock_status);
    zassert_equal(clk_mgr->refs, 1);
    expect_power(1);
    esb_release_tx_clock(false);
    stop_in_isr();
    zassert_equal(atomic_get(&tx_clock_users), 0);
    zassert_equal(clock_status, HFXO_SYNTH != 0);
    zassert_equal(clk_mgr->refs, HFXO_SYNTH ? 1 : 0);
    expect_power(HFXO_SYNTH ? 1 : 0);
}

ZTEST(hfxo, test_warm_external_owner_survives_app_stop)
{
    struct onoff_client external = {0};
    int result = -EINPROGRESS;
    start_mode = START_SYNC_OK;
    sys_notify_init_spinwait(&external.notify);
    zassert_true(onoff_request(clk_mgr, &external) >= 0);
    zassert_equal(sys_notify_fetch_result(&external.notify, &result), 0);
    zassert_equal(result, 0);
    zassert_equal(clocks_start(), 0);
    zassert_equal(clk_mgr->refs, 2);
    expect_power(1);
    stop_in_isr();
    zassert_equal(clock_status, HFXO_SYNTH != 0);
    zassert_equal(clk_mgr->refs, HFXO_SYNTH ? 2 : 1);
    expect_power(HFXO_SYNTH ? 1 : 0);
    release_synth_fixture_owner();
    zassert_equal(clk_mgr->refs, 1);
    zassert_equal(clk_mgr->flags & ONOFF_STATE_MASK, ONOFF_STATE_ON);
    expect_power(0);
    zassert_equal(onoff_release(clk_mgr), ONOFF_STATE_ON);
    zassert_equal(clk_mgr->refs, 0);
}

ZTEST(hfxo, test_completion_error_then_rejected_request)
{
    start_mode = START_SYNC_ERROR;
    zassert_equal(clocks_start(), -ETIMEDOUT);
    zassert_false(clock_status);
    zassert_equal(clk_mgr->refs, 0);
    zassert_equal(clk_mgr->flags & ONOFF_STATE_MASK, ONOFF_STATE_ERROR);
    expect_power(0);
    /* The SDK manager itself rejects a later request in its error state. */
    zassert_equal(clocks_start(), -EIO);
    zassert_false(clock_status);
    zassert_equal(clk_mgr->refs, 0);
    expect_power(0);
}

ZTEST(hfxo, test_nrf54_constlat_preserves_other_owner)
{
    if (!HFXO_NRF54) { ztest_test_skip(); return; }
#if HFXO_NRF54
    start_mode = START_SYNC_OK;
    (void)nrfx_power_constlat_mode_request();
    zassert_equal(clocks_start(), 0);
    zassert_equal(clk_mgr->refs, 1);
    expect_power(2);
    stop_in_isr();
    zassert_equal(clk_mgr->refs, HFXO_SYNTH ? 1 : 0);
    expect_power(HFXO_SYNTH ? 2 : 1);
    release_synth_fixture_owner();
    zassert_equal(clk_mgr->refs, 0);
    expect_power(1);
    (void)nrfx_power_constlat_mode_free();
    expect_power(0);
#endif
}

ZTEST(hfxo, test_lf_synth_boot_waits_for_success_and_retains_hf)
{
    if (!HFXO_SYNTH) { ztest_test_skip(); return; }
    /* Run the actual boot request and worker: a failed HF start must not
     * start the synthesized LF clock. */
    start_mode = START_SYNC_ERROR;
    boot_clock_request();
    join(&clocks_thread_id);
    zassert_false(lf_initialized, "LF_SYNTH started after failed HFXO");
    zassert_equal(clk_mgr->refs, 0);
    expect_power(0);
    /* A fresh real manager avoids manufacturing an onoff transition out of
     * ERROR; no request or reference remains on the failed instance. */
    zassert_true(next_manager < ARRAY_SIZE(managers));
    fixture_manager = &managers[next_manager++];
    zassert_equal(onoff_manager_init(fixture_manager, &transitions), 0);
    zassert_equal(clocks_init(), 0);
    k_sem_reset(&request_seen);
    atomic_clear(&completion_fired);
    start_mode = START_MANUAL;
    boot_clock_request();
    take(&request_seen);
    zassert_false(lf_initialized, "LF_SYNTH started while HFXO was pending");
    stop_in_isr();
    complete_start(0);
    join(&clocks_thread_id);
    zassert_true(lf_initialized, "successful HFXO must continue LF startup");
    zassert_false(lf_started_without_hf);
    zassert_true(clock_status);
    zassert_equal(clk_mgr->refs, 1);
    expect_power(1);
    stop_in_isr();
    zassert_true(clock_status);
    zassert_equal(clk_mgr->refs, 1);
    expect_power(1);
    zassert_equal(clocks_start(), 0);
    zassert_equal(clk_mgr->refs, 1);
    expect_power(1);
    /* The suite's after hook releases the modeled test-boot ownership. */
}

ZTEST_SUITE(hfxo, NULL, NULL, before, after, NULL);
