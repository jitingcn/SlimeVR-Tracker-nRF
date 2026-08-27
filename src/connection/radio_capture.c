/*
 * Hardware-only RADIO timing capture for TDMA diagnostics.
 * PPI/DPPI latches events; the RADIO ISR only copies one fixed record.
 */
#include "radio_capture.h"

#include "esb.h"
#include "tdma.h"

#include <errno.h>
#include <hal/nrf_radio.h>
#include <helpers/nrfx_gppi.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_SOC_SERIES_NRF52) && defined(NRF_TIMER3)
#include <hal/nrf_timer.h>
#elif defined(CONFIG_SOC_SERIES_NRF54L)
#include <zephyr/drivers/timer/nrf_grtc_timer.h>
#endif

LOG_MODULE_REGISTER(radio_capture, LOG_LEVEL_INF);

#define CAPTURE_METRICS 3
#define DURATION_BUCKET_US 4
#define DURATION_BUCKETS 64
#define PHASE_MIN_TICKS (-32)
#define PHASE_BUCKETS 64
#define CAPTURE_QUEUE_SIZE 16
#define TDMA_PING_WINDOW_SLOTS 2

BUILD_ASSERT((CAPTURE_QUEUE_SIZE & (CAPTURE_QUEUE_SIZE - 1U)) == 0,
	"capture queue must be power of two");

struct capture_metric {
	uint32_t samples;
	uint32_t address_samples;
	uint32_t ready_to_end[DURATION_BUCKETS];
	uint32_t address_to_end[DURATION_BUCKETS];
	uint32_t ready_phase[PHASE_BUCKETS];
	uint32_t end_phase[PHASE_BUCKETS];
	uint32_t ready_to_end_max_us;
	uint32_t address_to_end_max_us;
	uint32_t crossed_slot_end;
	uint32_t phase_samples;
};

struct ping_capture_record {
	uint64_t start_us;
	uint64_t end_us;
	uint64_t start_server_ticks;
	uint32_t duration_us;
	uint16_t frame_ticks;
	uint8_t slot_index;
	uint8_t slot_ticks;
	uint8_t attempts;
	bool success;
	bool schedule_valid;
};

struct ping_capture_metric {
	uint32_t success[3];
	uint32_t failed;
	uint64_t success_duration_us[3];
	uint32_t success_duration_max_us[3];
	uint64_t failure_duration_us;
	uint32_t failure_duration_max_us;
	uint32_t failure_timed;
	uint32_t start_missing;
	uint32_t phase_samples;
	uint32_t start_phase[PHASE_BUCKETS];
	uint32_t end_phase[PHASE_BUCKETS];
	uint32_t crossed_slot_boundary;
	uint32_t outside_own_slot;
};

struct capture_record {
	uint64_t ready_us;
	uint64_t address_us;
	uint64_t end_us;
	uint8_t payload_length;
};

static struct capture_record capture_queue[CAPTURE_QUEUE_SIZE];
static volatile uint8_t capture_head;
static volatile uint8_t capture_tail;
static atomic_t capture_queue_drops;
static atomic_t capture_incomplete[CAPTURE_METRICS];
static atomic_t capture_enabled = ATOMIC_INIT(1);
static struct capture_metric metrics[CAPTURE_METRICS];
static struct ping_capture_metric ping_metric;
static struct ping_capture_record ping_queue[CAPTURE_QUEUE_SIZE];
static volatile uint8_t ping_head;
static volatile uint8_t ping_tail;
static atomic_t ping_queue_drops;
static uint64_t ping_transaction_start_us;
static uint64_t ping_transaction_server_ticks;
static uint16_t ping_transaction_frame_ticks;
static uint8_t ping_transaction_slot_index;
static uint8_t ping_transaction_slot_ticks;
static bool ping_transaction_schedule_valid;
static atomic_t ping_transaction_pending;
static bool capture_initialized;
K_MUTEX_DEFINE(capture_metrics_lock);

static uint8_t metric_index(uint8_t payload_length)
{
	if (payload_length == ESB_SENSOR_DATA_LEN) {
		return 0;
	}
	return payload_length <= 32 ? 1 : 2;
}

static uint8_t duration_bucket(uint32_t duration_us)
{
	return (uint8_t)MIN(duration_us / DURATION_BUCKET_US, DURATION_BUCKETS - 1);
}

static uint8_t phase_bucket(int32_t phase_ticks)
{
	return (uint8_t)CLAMP(phase_ticks - PHASE_MIN_TICKS, 0, PHASE_BUCKETS - 1);
}

static uint32_t percentile(const uint32_t *histogram, uint32_t count, uint32_t numerator, uint32_t denominator)
{
	if (count == 0) {
		return 0;
	}
	uint32_t target = (uint32_t)(((uint64_t)count * numerator + denominator - 1U) / denominator);
	uint32_t accumulated = 0;
	for (uint32_t i = 0; i < DURATION_BUCKETS; i++) {
		accumulated += histogram[i];
		if (accumulated >= target) {
			return i * DURATION_BUCKET_US;
		}
	}
	return (DURATION_BUCKETS - 1U) * DURATION_BUCKET_US;
}

static int32_t phase_percentile(const uint32_t *histogram, uint32_t count, uint32_t numerator, uint32_t denominator)
{
	if (count == 0) {
		return 0;
	}
	uint32_t target = (uint32_t)(((uint64_t)count * numerator + denominator - 1U) / denominator);
	uint32_t accumulated = 0;
	for (uint32_t i = 0; i < PHASE_BUCKETS; i++) {
		accumulated += histogram[i];
		if (accumulated >= target) {
			return (int32_t)i + PHASE_MIN_TICKS;
		}
	}
	return PHASE_MIN_TICKS + PHASE_BUCKETS - 1;
}

#if defined(CONFIG_SOC_SERIES_NRF52) && defined(NRF_TIMER3)
static nrfx_gppi_handle_t capture_conn[3];
static size_t capture_conn_count;
static const nrf_radio_event_t radio_events[3] = {
	NRF_RADIO_EVENT_READY,
	NRF_RADIO_EVENT_ADDRESS,
	NRF_RADIO_EVENT_END,
};
static const nrf_timer_task_t timer_tasks[3] = {
	NRF_TIMER_TASK_CAPTURE0,
	NRF_TIMER_TASK_CAPTURE1,
	NRF_TIMER_TASK_CAPTURE2,
};

static void capture_platform_deinit(void)
{
	for (size_t i = 0; i < capture_conn_count; i++) {
		uint32_t event = nrf_radio_event_address_get(NRF_RADIO, radio_events[i]);
		uint32_t task = nrf_timer_task_address_get(NRF_TIMER3, timer_tasks[i]);
		nrfx_gppi_conn_disable(capture_conn[i]);
		nrfx_gppi_conn_free(event, task, capture_conn[i]);
	}
	capture_conn_count = 0;
	nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_STOP);
}

static int capture_platform_init(void)
{
	nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_STOP);
	nrf_timer_mode_set(NRF_TIMER3, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(NRF_TIMER3, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_prescaler_set(NRF_TIMER3, NRF_TIMER_FREQ_1MHz);
	nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_CLEAR);
	nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_START);

	for (size_t i = 0; i < ARRAY_SIZE(capture_conn); i++) {
		uint32_t event = nrf_radio_event_address_get(NRF_RADIO, radio_events[i]);
		uint32_t task = nrf_timer_task_address_get(NRF_TIMER3, timer_tasks[i]);
		int err = nrfx_gppi_conn_alloc(event, task, &capture_conn[i]);
		if (err) {
			capture_platform_deinit();
			return err;
		}
		capture_conn_count++;
		nrfx_gppi_conn_enable(capture_conn[i]);
	}
	return 0;
}

static bool capture_platform_read(struct capture_record *record)
{
	record->ready_us = nrf_timer_cc_get(NRF_TIMER3, NRF_TIMER_CC_CHANNEL0);
	record->address_us = nrf_timer_cc_get(NRF_TIMER3, NRF_TIMER_CC_CHANNEL1);
	record->end_us = nrf_timer_cc_get(NRF_TIMER3, NRF_TIMER_CC_CHANNEL2);
	return record->ready_us != 0 && record->address_us != 0 && record->end_us != 0;
}

static uint64_t capture_platform_now(void)
{
	nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_CAPTURE3);
	return nrf_timer_cc_get(NRF_TIMER3, NRF_TIMER_CC_CHANNEL3);
}
#elif defined(CONFIG_SOC_SERIES_NRF54L)
static nrfx_gppi_handle_t capture_conn[2];
static int32_t capture_chan[2] = {-1, -1};
static size_t capture_conn_count;
static const nrf_radio_event_t radio_events[2] = {
	NRF_RADIO_EVENT_READY,
	NRF_RADIO_EVENT_PHYEND,
};

static void capture_platform_rearm(void)
{
	for (size_t i = 0; i < capture_conn_count; i++) {
		uint32_t event = nrf_radio_event_address_get(NRF_RADIO, radio_events[i]);
		uint32_t domain = nrfx_gppi_domain_id_get(event);
		int channel = nrfx_gppi_domain_channel_get(capture_conn[i], domain);
		if (channel >= 0) {
			nrf_radio_publish_set(NRF_RADIO, radio_events[i], (uint8_t)channel);
		}
	}
}

static void capture_platform_deinit(void)
{
	for (size_t i = 0; i < capture_conn_count; i++) {
		uint32_t event = nrf_radio_event_address_get(NRF_RADIO, radio_events[i]);
		uint32_t task = z_nrf_grtc_timer_capture_task_address_get(capture_chan[i]);
		nrfx_gppi_conn_disable(capture_conn[i]);
		nrfx_gppi_conn_free(event, task, capture_conn[i]);
	}
	for (size_t i = 0; i < ARRAY_SIZE(capture_chan); i++) {
		if (capture_chan[i] >= 0) {
			z_nrf_grtc_timer_chan_free(capture_chan[i]);
			capture_chan[i] = -1;
		}
	}
	capture_conn_count = 0;
}

static int capture_platform_init(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(capture_conn); i++) {
		capture_chan[i] = z_nrf_grtc_timer_chan_alloc();
		if (capture_chan[i] < 0 || z_nrf_grtc_timer_capture_prepare(capture_chan[i]) != 0) {
			capture_platform_deinit();
			return -ENOMEM;
		}
		uint32_t event = nrf_radio_event_address_get(NRF_RADIO, radio_events[i]);
		uint32_t task = z_nrf_grtc_timer_capture_task_address_get(capture_chan[i]);
		int err = nrfx_gppi_conn_alloc(event, task, &capture_conn[i]);
		if (err) {
			capture_platform_deinit();
			return err;
		}
		capture_conn_count++;
		nrfx_gppi_conn_enable(capture_conn[i]);
	}
	capture_platform_rearm();
	return 0;
}

static bool capture_platform_read(struct capture_record *record)
{
	int ready_err = z_nrf_grtc_timer_capture_read(capture_chan[0], &record->ready_us);
	int end_err = z_nrf_grtc_timer_capture_read(capture_chan[1], &record->end_us);
	record->address_us = 0;
	for (size_t i = 0; i < ARRAY_SIZE(capture_chan); i++) {
		(void)z_nrf_grtc_timer_capture_prepare(capture_chan[i]);
	}
	capture_platform_rearm();
	return ready_err == 0 && end_err == 0;
}

static uint64_t capture_platform_now(void)
{
	return z_nrf_grtc_timer_read();
}
#else
static int capture_platform_init(void)
{
	return -ENOTSUP;
}

static void capture_platform_deinit(void)
{
}

static bool capture_platform_read(struct capture_record *record)
{
	ARG_UNUSED(record);
	return false;
}

static uint64_t capture_platform_now(void)
{
	return 0;
}
#endif

int radio_capture_init(void)
{
	if (capture_initialized) {
		return 0;
	}
	int err = capture_platform_init();
	if (err == 0) {
		capture_initialized = true;
		atomic_set(&capture_enabled, 1);
	}
	return err;
}

void radio_capture_deinit(void)
{
	if (!capture_initialized) {
		return;
	}
	capture_platform_deinit();
	atomic_set(&capture_enabled, 0);
	capture_initialized = false;
}

void radio_capture_set_enabled(bool enabled)
{
	if (!capture_initialized) {
		return;
	}
	for (size_t i = 0; i < capture_conn_count; i++) {
		if (enabled) {
			nrfx_gppi_conn_enable(capture_conn[i]);
		} else {
			nrfx_gppi_conn_disable(capture_conn[i]);
		}
	}
	atomic_set(&capture_enabled, enabled ? 1 : 0);
#if defined(CONFIG_SOC_SERIES_NRF54L)
	if (enabled) {
		capture_platform_rearm();
	}
#endif
}

bool radio_capture_is_enabled(void)
{
	return capture_initialized && atomic_get(&capture_enabled) != 0;
}

void radio_capture_record(
	uint8_t packet_type,
	uint8_t payload_length,
	bool no_ack,
	uint8_t attempts,
	bool success,
	bool transaction_start
)
{
	if (!radio_capture_is_enabled()) {
		return;
	}
	if (!no_ack && packet_type == ESB_PING_TYPE && transaction_start) {
		ping_transaction_start_us = capture_platform_now();
		ping_transaction_server_ticks = esb_get_server_time_ticks_64();
		ping_transaction_schedule_valid = ping_transaction_server_ticks != 0
			&& tdma_config_get(
				&ping_transaction_slot_index,
				&ping_transaction_slot_ticks,
				&ping_transaction_frame_ticks
			);
		atomic_set(&ping_transaction_pending, 1);
		return;
	}

	struct capture_record record = {
		.payload_length = payload_length,
	};
	bool complete = capture_platform_read(&record);
	if (!no_ack) {
		if (packet_type != ESB_PING_TYPE) {
			return;
		}
		uint64_t completion_us = capture_platform_now();
		uint8_t head = ping_head;
		uint8_t next = (head + 1U) & (CAPTURE_QUEUE_SIZE - 1U);
		if (next == ping_tail) {
			atomic_inc(&ping_queue_drops);
			return;
		}
		bool have_start = atomic_cas(&ping_transaction_pending, 1, 0);
		ping_queue[head] = (struct ping_capture_record){
			.start_us = have_start ? ping_transaction_start_us : 0,
			.end_us = completion_us,
			.start_server_ticks = have_start ? ping_transaction_server_ticks : 0,
			.duration_us = have_start ? (uint32_t)(completion_us - ping_transaction_start_us) : 0,
			.frame_ticks = ping_transaction_frame_ticks,
			.slot_index = ping_transaction_slot_index,
			.slot_ticks = ping_transaction_slot_ticks,
			.attempts = CLAMP(attempts, 1, 3),
			.success = success,
			.schedule_valid = have_start && ping_transaction_schedule_valid,
		};
		__DMB();
		ping_head = next;
		return;
	}
	if (!complete) {
		atomic_inc(&capture_incomplete[metric_index(payload_length)]);
		return;
	}
	uint8_t head = capture_head;
	uint8_t next = (head + 1U) & (CAPTURE_QUEUE_SIZE - 1U);
	if (next == capture_tail) {
		atomic_inc(&capture_queue_drops);
		return;
	}
	capture_queue[head] = record;
	__DMB();
	capture_head = next;
}

void radio_capture_process(void)
{
	while (ping_tail != ping_head) {
		uint8_t tail = ping_tail;
		struct ping_capture_record record = ping_queue[tail];
		__DMB();
		ping_tail = (tail + 1U) & (CAPTURE_QUEUE_SIZE - 1U);
		uint8_t index = record.attempts - 1U;
		if (record.success) {
			ping_metric.success[index]++;
			if (record.duration_us > 0) {
				ping_metric.success_duration_us[index] += record.duration_us;
				ping_metric.success_duration_max_us[index]
					= MAX(ping_metric.success_duration_max_us[index], record.duration_us);
			}
		} else {
			ping_metric.failed++;
			if (record.duration_us > 0) {
				ping_metric.failure_duration_us += record.duration_us;
				ping_metric.failure_duration_max_us
					= MAX(ping_metric.failure_duration_max_us, record.duration_us);
				ping_metric.failure_timed++;
			}
		}
		if (record.start_us == 0) {
			ping_metric.start_missing++;
			continue;
		}
		if (!record.schedule_valid || record.frame_ticks == 0 || record.slot_ticks == 0) {
			continue;
		}
		uint64_t start_ticks = record.start_server_ticks;
		uint64_t duration_ticks
			= ((uint64_t)record.duration_us * 32768U + 999999U) / 1000000U;
		uint64_t end_ticks = start_ticks + duration_ticks;
		int32_t slot_start = (int32_t)record.slot_index * record.slot_ticks;
		int32_t start_phase = (int32_t)(start_ticks % record.frame_ticks) - slot_start;
		int32_t end_phase = start_phase + (int32_t)duration_ticks;
		while (start_phase > (int32_t)(record.frame_ticks / 2)) {
			start_phase -= record.frame_ticks;
		}
		while (start_phase < -(int32_t)(record.frame_ticks / 2)) {
			start_phase += record.frame_ticks;
		}
		ping_metric.phase_samples++;
		ping_metric.start_phase[phase_bucket(start_phase)]++;
		ping_metric.end_phase[phase_bucket(end_phase)]++;
		if (start_phase < 0 || start_phase >= record.slot_ticks) {
			ping_metric.outside_own_slot++;
		}
		uint64_t start_cell = start_ticks / record.slot_ticks;
		uint64_t end_cell = end_ticks / record.slot_ticks;
		if (end_cell - start_cell >= TDMA_PING_WINDOW_SLOTS) {
			ping_metric.crossed_slot_boundary++;
		}
	}

	while (capture_tail != capture_head) {
		uint8_t tail = capture_tail;
		struct capture_record record = capture_queue[tail];
		__DMB();
		capture_tail = (tail + 1U) & (CAPTURE_QUEUE_SIZE - 1U);
		uint64_t now_us = capture_platform_now();
#if defined(CONFIG_SOC_SERIES_NRF52) && defined(NRF_TIMER3)
		uint64_t elapsed_ready = (uint32_t)now_us - (uint32_t)record.ready_us;
		uint64_t elapsed_end = (uint32_t)now_us - (uint32_t)record.end_us;
		uint32_t ready_to_end_us = (uint32_t)record.end_us - (uint32_t)record.ready_us;
		uint32_t address_to_end_us = (uint32_t)record.end_us - (uint32_t)record.address_us;
#else
		uint64_t elapsed_ready = now_us - record.ready_us;
		uint64_t elapsed_end = now_us - record.end_us;
		uint32_t ready_to_end_us = (uint32_t)(record.end_us - record.ready_us);
		uint32_t address_to_end_us = 0;
#endif
		uint8_t index = metric_index(record.payload_length);
		uint8_t slot_index;
		uint8_t slot_ticks;
		uint16_t frame_ticks;
		int32_t ready_phase = 0;
		int32_t end_phase = 0;
		uint64_t server_ticks_now = esb_get_server_time_ticks_64();
		bool phase_valid = server_ticks_now != 0
			&& tdma_config_get(&slot_index, &slot_ticks, &frame_ticks);
		bool crossed = false;
		if (phase_valid) {
			uint64_t ready_ticks
				= server_ticks_now - (elapsed_ready * 32768U + 999999U) / 1000000U;
			uint64_t end_ticks
				= server_ticks_now - (elapsed_end * 32768U + 999999U) / 1000000U;
			uint32_t slot_start = (uint32_t)slot_index * slot_ticks;
			ready_phase = (int32_t)(ready_ticks % frame_ticks) - (int32_t)slot_start;
			end_phase = (int32_t)(end_ticks % frame_ticks) - (int32_t)slot_start;
			if (ready_phase > (int32_t)(frame_ticks / 2)) {
				ready_phase -= frame_ticks;
			}
			if (end_phase > (int32_t)(frame_ticks / 2)) {
				end_phase -= frame_ticks;
			}
			crossed = ready_phase >= 0 && end_phase >= slot_ticks;
		}

		k_mutex_lock(&capture_metrics_lock, K_FOREVER);
		struct capture_metric *metric = &metrics[index];
		metric->samples++;
		metric->ready_to_end[duration_bucket(ready_to_end_us)]++;
		metric->ready_to_end_max_us = MAX(metric->ready_to_end_max_us, ready_to_end_us);
		if (record.address_us != 0) {
			metric->address_samples++;
			metric->address_to_end[duration_bucket(address_to_end_us)]++;
			metric->address_to_end_max_us = MAX(metric->address_to_end_max_us, address_to_end_us);
		}
		if (phase_valid) {
			metric->phase_samples++;
			metric->ready_phase[phase_bucket(ready_phase)]++;
			metric->end_phase[phase_bucket(end_phase)]++;
			metric->crossed_slot_end += crossed;
		}
		k_mutex_unlock(&capture_metrics_lock);
	}
}

void radio_capture_print_stats(void)
{
	static const char *const names[CAPTURE_METRICS] = {"len17", "len18_32", "len33_plus"};
	for (size_t i = 0; i < CAPTURE_METRICS; i++) {
		k_mutex_lock(&capture_metrics_lock, K_FOREVER);
		struct capture_metric *metric = &metrics[i];
		uint32_t ready_p50_us = percentile(metric->ready_to_end, metric->samples, 50, 100);
		uint32_t ready_p99_us = percentile(metric->ready_to_end, metric->samples, 99, 100);
		uint32_t ready_p999_us = percentile(metric->ready_to_end, metric->samples, 999, 1000);
		uint32_t address_p50_us = percentile(metric->address_to_end, metric->address_samples, 50, 100);
		uint32_t address_p99_us = percentile(metric->address_to_end, metric->address_samples, 99, 100);
		int32_t ready_p50 = phase_percentile(metric->ready_phase, metric->phase_samples, 50, 100);
		int32_t ready_p99 = phase_percentile(metric->ready_phase, metric->phase_samples, 99, 100);
		int32_t end_p99 = phase_percentile(metric->end_phase, metric->phase_samples, 99, 100);
		LOG_INF(
			"RADIO CAP %s n=%u incomplete=%u queue_drop=%u phase_n=%u ready_end_us=%u/%u/%u/%u address_n=%u address_end_us=%u/%u/%u ready_tick=%d/%d end_tick_p99=%d cross=%u",
			names[i], metric->samples, (uint32_t)atomic_get(&capture_incomplete[i]),
			(uint32_t)atomic_get(&capture_queue_drops), metric->phase_samples,
			ready_p50_us, ready_p99_us, ready_p999_us, metric->ready_to_end_max_us,
			metric->address_samples, address_p50_us, address_p99_us, metric->address_to_end_max_us,
			ready_p50, ready_p99, end_p99, metric->crossed_slot_end
		);
		k_mutex_unlock(&capture_metrics_lock);
	}
	int32_t ping_start_p50
		= phase_percentile(ping_metric.start_phase, ping_metric.phase_samples, 50, 100);
	int32_t ping_start_p99
		= phase_percentile(ping_metric.start_phase, ping_metric.phase_samples, 99, 100);
	int32_t ping_end_p99
		= phase_percentile(ping_metric.end_phase, ping_metric.phase_samples, 99, 100);
	LOG_INF(
		"PING CAP window=2slots ok=%u/%u/%u fail=%u success_us=%llu/%llu/%llu success_max=%u/%u/%u fail_timed=%u fail_us=%llu fail_max=%u missing=%u queue_drop=%u phase_n=%u start_tick=%d/%d end_tick_p99=%d outside=%u overshoot=%u",
		ping_metric.success[0], ping_metric.success[1], ping_metric.success[2], ping_metric.failed,
		ping_metric.success[0] ? ping_metric.success_duration_us[0] / ping_metric.success[0] : 0,
		ping_metric.success[1] ? ping_metric.success_duration_us[1] / ping_metric.success[1] : 0,
		ping_metric.success[2] ? ping_metric.success_duration_us[2] / ping_metric.success[2] : 0,
		ping_metric.success_duration_max_us[0], ping_metric.success_duration_max_us[1],
		ping_metric.success_duration_max_us[2], ping_metric.failure_timed,
		ping_metric.failure_timed ? ping_metric.failure_duration_us / ping_metric.failure_timed : 0,
		ping_metric.failure_duration_max_us, ping_metric.start_missing,
		(uint32_t)atomic_get(&ping_queue_drops), ping_metric.phase_samples,
		ping_start_p50, ping_start_p99, ping_end_p99,
		ping_metric.outside_own_slot, ping_metric.crossed_slot_boundary
	);
}
