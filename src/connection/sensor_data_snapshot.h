#ifndef SLIMENRF_SENSOR_DATA_SNAPSHOT_H
#define SLIMENRF_SENSOR_DATA_SNAPSHOT_H

#include <stdbool.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

typedef struct {
	struct k_spinlock lock;
	float q[4];
	float a[3];
	float m[3];
	bool precise;
	atomic_t qa_pending;
	atomic_t m_pending;
} sensor_data_snapshot_t;

static inline bool sensor_data_snapshot_qa_pending(const sensor_data_snapshot_t *state)
{
	return atomic_get(&state->qa_pending) != 0;
}

static inline bool sensor_data_snapshot_m_pending(const sensor_data_snapshot_t *state)
{
	return atomic_get(&state->m_pending) != 0;
}

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
	atomic_set(&state->qa_pending, 1);
	k_spin_unlock(&state->lock, key);
}

static inline void sensor_data_snapshot_publish_m(
	sensor_data_snapshot_t *state,
	const float m[3]
)
{
	k_spinlock_key_t key = k_spin_lock(&state->lock);
	memcpy(state->m, m, sizeof(state->m));
	atomic_set(&state->m_pending, 1);
	k_spin_unlock(&state->lock, key);
}

static inline void sensor_data_snapshot_read_qa(
	sensor_data_snapshot_t *state,
	float q_out[4],
	float a_out[3]
)
{
	k_spinlock_key_t key = k_spin_lock(&state->lock);
	memcpy(q_out, state->q, sizeof(state->q));
	memcpy(a_out, state->a, sizeof(state->a));
	atomic_set(&state->qa_pending, 0);
	k_spin_unlock(&state->lock, key);
}

static inline void sensor_data_snapshot_read_qm(
	sensor_data_snapshot_t *state,
	float q_out[4],
	float m_out[3]
)
{
	k_spinlock_key_t key = k_spin_lock(&state->lock);
	memcpy(q_out, state->q, sizeof(state->q));
	memcpy(m_out, state->m, sizeof(state->m));
	atomic_set(&state->qa_pending, 0);
	atomic_set(&state->m_pending, 0);
	k_spin_unlock(&state->lock, key);
}

static inline bool sensor_data_snapshot_is_precise(sensor_data_snapshot_t *state)
{
	k_spinlock_key_t key = k_spin_lock(&state->lock);
	bool precise = state->precise;
	k_spin_unlock(&state->lock, key);
	return precise;
}

#endif
