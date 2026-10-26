#include <stdbool.h>
#include <stdint.h>
#ifdef HOST_CONCURRENCY_STRESS
#include <pthread.h>
#define RUN_HOST_CONCURRENCY_STRESS
#endif
#include <stdio.h>
#include <string.h>

#include "../../../src/connection/sensor_data_snapshot.h"

static bool vector_is(const float *vector, size_t count, float expected)
{
	for (size_t i = 0; i < count; i++) {
		if (vector[i] != expected) {
			return false;
		}
	}
	return true;
}

#ifdef RUN_HOST_CONCURRENCY_STRESS
#define STRESS_ITERATIONS 500000U

struct stress_context {
	sensor_data_snapshot_t snapshot;
	bool producer_done;
	bool failed;
	uint32_t last_observed;
	uint32_t last_mag_observed;
};

static void *stress_producer(void *arg)
{
	struct stress_context *ctx = arg;
	for (uint32_t value = 1; value <= STRESS_ITERATIONS; value++) {
		float q[4] = {value, value, value, value};
		float a[3] = {value, value, value};
		sensor_data_snapshot_publish_qa(&ctx->snapshot, q, a, (value & 1U) != 0);
		float m[3] = {value, value, value};
		sensor_data_snapshot_publish_m(&ctx->snapshot, m);
	}
	__atomic_store_n(&ctx->producer_done, true, __ATOMIC_RELEASE);
	return NULL;
}

static void *stress_consumer(void *arg)
{
	struct stress_context *ctx = arg;
	for (;;) {
		if (sensor_data_snapshot_qa_pending(&ctx->snapshot)
		    || sensor_data_snapshot_m_pending(&ctx->snapshot)) {
			sensor_data_capture_t capture;
			sensor_data_snapshot_capture(&ctx->snapshot, &capture);
			uint32_t value = (uint32_t)capture.q[0];
			uint32_t mag_value = (uint32_t)capture.m[0];
			if (!vector_is(capture.q, 4, capture.q[0])
			    || !vector_is(capture.a, 3, capture.q[0])
			    || !vector_is(capture.m, 3, capture.m[0])
			    || capture.precise != ((value & 1U) != 0)
			    || capture.qa_generation != value || capture.m_generation != mag_value
			    || value < ctx->last_observed || mag_value < ctx->last_mag_observed) {
				__atomic_store_n(&ctx->failed, true, __ATOMIC_RELAXED);
				return NULL;
			}
			ctx->last_observed = value;
			ctx->last_mag_observed = mag_value;
			sensor_data_snapshot_commit(
				&ctx->snapshot, &capture, SENSOR_STREAM_QA | SENSOR_STREAM_M
			);
		}
		if (__atomic_load_n(&ctx->producer_done, __ATOMIC_ACQUIRE)
		    && !sensor_data_snapshot_qa_pending(&ctx->snapshot)
		    && !sensor_data_snapshot_m_pending(&ctx->snapshot)) {
			return NULL;
		}
	}
}

static bool concurrent_stress_remained_coherent(void)
{
	struct stress_context ctx;
	memset(&ctx, 0, sizeof(ctx));
	pthread_t producer;
	pthread_t consumer;
	if (pthread_create(&producer, NULL, stress_producer, &ctx) != 0
	    || pthread_create(&consumer, NULL, stress_consumer, &ctx) != 0) {
		return false;
	}
	if (pthread_join(producer, NULL) != 0 || pthread_join(consumer, NULL) != 0) {
		return false;
	}
	return !__atomic_load_n(&ctx.failed, __ATOMIC_RELAXED)
		&& ctx.last_observed == STRESS_ITERATIONS
		&& ctx.last_mag_observed == STRESS_ITERATIONS;
}
#endif

int main(void)
{
	sensor_data_snapshot_t snapshot;
	memset(&snapshot, 0, sizeof(snapshot));
	float q[4] = {1, 1, 1, 1};
	float a[3] = {2, 2, 2};
	float m[3] = {3, 3, 3};
	sensor_data_capture_t first, latest;

#define CHECK(condition, message) do { \
	if (!(condition)) { fputs("FAIL: " message "\n", stderr); return 1; } \
} while (0)

	sensor_data_snapshot_publish_qa(&snapshot, q, a, true);
	sensor_data_snapshot_publish_m(&snapshot, m);
	sensor_data_snapshot_capture(&snapshot, &first);
	CHECK(vector_is(first.q, 4, 1) && vector_is(first.a, 3, 2)
	      && vector_is(first.m, 3, 3) && first.precise
	      && first.pending == (SENSOR_STREAM_QA | SENSOR_STREAM_M),
	      "capture did not contain the coherent publication");
	CHECK(sensor_data_snapshot_qa_pending(&snapshot) && sensor_data_snapshot_m_pending(&snapshot),
	      "capture consumed unpublished transport work");
	sensor_data_snapshot_commit(&snapshot, &first, 0);
	CHECK(sensor_data_snapshot_qa_pending(&snapshot) && sensor_data_snapshot_m_pending(&snapshot),
	      "refusal consumed a stream");
	sensor_data_snapshot_commit(&snapshot, &first, SENSOR_STREAM_M);
	CHECK(sensor_data_snapshot_qa_pending(&snapshot) && !sensor_data_snapshot_m_pending(&snapshot),
	      "M acceptance consumed QA");
	sensor_data_snapshot_commit(&snapshot, &first, SENSOR_STREAM_QA);
	CHECK(!sensor_data_snapshot_qa_pending(&snapshot) && !sensor_data_snapshot_m_pending(&snapshot),
	      "QA acceptance did not retire its captured generation");

	sensor_data_snapshot_publish_qa(&snapshot, q, a, true);
	sensor_data_snapshot_publish_m(&snapshot, m);
	sensor_data_snapshot_capture(&snapshot, &first);
	q[0] = 4; a[0] = 5;
	sensor_data_snapshot_publish_qa(&snapshot, q, a, false);
	sensor_data_snapshot_commit(&snapshot, &first, SENSOR_STREAM_QA | SENSOR_STREAM_M);
	CHECK(sensor_data_snapshot_qa_pending(&snapshot) && !sensor_data_snapshot_m_pending(&snapshot),
	      "A acceptance consumed newer QA B or failed independent M retirement");
	CHECK(first.q[0] == 1 && first.a[0] == 2 && first.m[0] == 3 && first.precise,
	      "publication mutated a captured transaction");
	sensor_data_snapshot_capture(&snapshot, &latest);
	CHECK(latest.q[0] == 4 && latest.a[0] == 5 && !latest.precise
	      && latest.qa_generation == first.qa_generation + 1
	      && latest.m_generation == first.m_generation && latest.pending == SENSOR_STREAM_QA,
	      "QA publication changed the wrong generation or precision");

	m[0] = 6;
	sensor_data_snapshot_publish_m(&snapshot, m);
	sensor_data_snapshot_commit(&snapshot, &latest, SENSOR_STREAM_QA | SENSOR_STREAM_M);
	CHECK(!sensor_data_snapshot_qa_pending(&snapshot) && sensor_data_snapshot_m_pending(&snapshot),
	      "new M prevented independent QA retirement or was retired by stale M");
	sensor_data_snapshot_capture(&snapshot, &latest);
	CHECK(latest.m[0] == 6 && latest.pending == SENSOR_STREAM_M,
	      "new M publication was lost");
	sensor_data_snapshot_commit(&snapshot, &latest, SENSOR_STREAM_M);
	CHECK(!sensor_data_snapshot_qa_pending(&snapshot) && !sensor_data_snapshot_m_pending(&snapshot),
	      "latest M was not retired");

	snapshot.qa_generation = UINT32_MAX;
	snapshot.m_generation = UINT32_MAX;
	sensor_data_snapshot_publish_qa(&snapshot, q, a, false);
	sensor_data_snapshot_publish_m(&snapshot, m);
	sensor_data_snapshot_capture(&snapshot, &latest);
	CHECK(latest.qa_generation == UINT64_C(4294967296)
	      && latest.m_generation == UINT64_C(4294967296),
	      "publication identities truncated to 32 bits");
	sensor_data_snapshot_commit(&snapshot, &first, SENSOR_STREAM_QA | SENSOR_STREAM_M);
	CHECK(sensor_data_snapshot_qa_pending(&snapshot) && sensor_data_snapshot_m_pending(&snapshot),
	      "old acceptance retired a later generation");
#ifdef RUN_HOST_CONCURRENCY_STRESS
	CHECK(concurrent_stress_remained_coherent(), "concurrent capture/commit observed torn or lost data");
#endif
#undef CHECK
	puts("PASS: coherent immutable capture and independent generation-checked acceptance");
	return 0;
}
