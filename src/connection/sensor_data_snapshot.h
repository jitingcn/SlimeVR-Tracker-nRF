#ifndef SLIMENRF_SENSOR_DATA_SNAPSHOT_H
#define SLIMENRF_SENSOR_DATA_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

enum sensor_stream {
	SENSOR_STREAM_QA = 1u << 0,
	SENSOR_STREAM_M = 1u << 1,
};

typedef struct {
	float q[4];
	float a[3];
#if CONFIG_SENSOR_USE_MAGNETOMETER
	float m[3];
#endif
	uint64_t qa_generation;
#if CONFIG_SENSOR_USE_MAGNETOMETER
	uint64_t m_generation;
#endif
	bool precise;
	uint8_t pending;
} sensor_data_capture_t;

typedef struct {
	struct k_spinlock lock;
	float q[4];
	float a[3];
#if CONFIG_SENSOR_USE_MAGNETOMETER
	float m[3];
#endif
	uint64_t qa_generation;
#if CONFIG_SENSOR_USE_MAGNETOMETER
	uint64_t m_generation;
#endif
	bool precise;
	atomic_t qa_pending;
#if CONFIG_SENSOR_USE_MAGNETOMETER
	atomic_t m_pending;
#endif
} sensor_data_snapshot_t;

static inline bool sensor_data_snapshot_qa_pending(const sensor_data_snapshot_t *state)
{
	return atomic_get(&state->qa_pending) != 0;
}

#if CONFIG_SENSOR_USE_MAGNETOMETER
static inline bool sensor_data_snapshot_m_pending(const sensor_data_snapshot_t *state)
{
	return atomic_get(&state->m_pending) != 0;
}
#endif

static inline void sensor_data_snapshot_publish_qa(
	sensor_data_snapshot_t *state,
	const float q[4],
	const float a[3],
	bool precise
)
{
	k_spinlock_key_t key = k_spin_lock(&state->lock);
	memcpy(state->q, q, sizeof(state->q));
	memcpy(state->a, a, sizeof(state->a));
	state->precise = precise;
	state->qa_generation++;
	atomic_set(&state->qa_pending, 1);
	k_spin_unlock(&state->lock, key);
}

#if CONFIG_SENSOR_USE_MAGNETOMETER
static inline void sensor_data_snapshot_publish_m(
	sensor_data_snapshot_t *state,
	const float m[3]
)
{
	k_spinlock_key_t key = k_spin_lock(&state->lock);
	memcpy(state->m, m, sizeof(state->m));
	state->m_generation++;
	atomic_set(&state->m_pending, 1);
	k_spin_unlock(&state->lock, key);
}
#endif

static inline void sensor_data_snapshot_capture(
	sensor_data_snapshot_t *state,
	sensor_data_capture_t *capture
)
{
	k_spinlock_key_t key = k_spin_lock(&state->lock);
	memcpy(capture->q, state->q, sizeof(capture->q));
	memcpy(capture->a, state->a, sizeof(capture->a));
#if CONFIG_SENSOR_USE_MAGNETOMETER
	memcpy(capture->m, state->m, sizeof(capture->m));
#endif
	capture->qa_generation = state->qa_generation;
#if CONFIG_SENSOR_USE_MAGNETOMETER
	capture->m_generation = state->m_generation;
#endif
	capture->precise = state->precise;
	capture->pending = sensor_data_snapshot_qa_pending(state) ? SENSOR_STREAM_QA : 0;
#if CONFIG_SENSOR_USE_MAGNETOMETER
	capture->pending |= sensor_data_snapshot_m_pending(state) ? SENSOR_STREAM_M : 0;
#endif
	k_spin_unlock(&state->lock, key);
}

static inline void sensor_data_snapshot_commit(
	sensor_data_snapshot_t *state,
	const sensor_data_capture_t *capture,
	uint8_t accepted_streams
)
{
	k_spinlock_key_t key = k_spin_lock(&state->lock);
	if ((accepted_streams & SENSOR_STREAM_QA) && state->qa_generation == capture->qa_generation) {
		atomic_set(&state->qa_pending, 0);
	}
#if CONFIG_SENSOR_USE_MAGNETOMETER
	if ((accepted_streams & SENSOR_STREAM_M) && state->m_generation == capture->m_generation) {
		atomic_set(&state->m_pending, 0);
	}
#endif
	k_spin_unlock(&state->lock, key);
}

#endif
