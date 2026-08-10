#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

struct persistence_state {
	unsigned generation;
	unsigned nvs_matrix;
	unsigned retained_matrix;
	bool dirty;
	bool enabled;
	bool commits_suspended;
	bool storage_locked;
};

struct dirty_slot {
	unsigned id;
	uintptr_t ptr;
	size_t len;
};

struct dirty_table {
	struct dirty_slot slots[2];
	size_t count;
	bool locked;
};

struct norm_state {
	float mean;
	float variance;
	uint32_t count;
	bool locked;
};

static bool storage_try_lock(struct persistence_state *state)
{
	if (state->storage_locked) {
		return false;
	}
	state->storage_locked = true;
	return true;
}

static void storage_unlock(struct persistence_state *state)
{
	state->storage_locked = false;
}

static bool commit_begin(struct persistence_state *state, unsigned matrix)
{
	if (!state->enabled || state->commits_suspended || !storage_try_lock(state)) {
		return false;
	}
	state->retained_matrix = matrix;
	return true;
}

static void commit_end(struct persistence_state *state)
{
	state->generation++;
	state->dirty = true;
	storage_unlock(state);
}

static bool flush(struct persistence_state *state)
{
	if (!storage_try_lock(state)) {
		return false;
	}
	if (state->dirty) {
		state->nvs_matrix = state->retained_matrix;
		state->dirty = false;
	}
	storage_unlock(state);
	return true;
}

static void prepare_power_down(struct persistence_state *state)
{
	state->commits_suspended = true;
	state->generation++;
}

static bool commit_and_flush_are_serialized(void)
{
	struct persistence_state state = {
		.generation = 10,
		.nvs_matrix = 1,
		.retained_matrix = 1,
		.enabled = true,
	};

	if (!commit_begin(&state, 2) || flush(&state)) {
		return false;
	}
	commit_end(&state);
	if (!flush(&state) || state.nvs_matrix != state.retained_matrix) {
		return false;
	}

	state.enabled = false;
	state.generation++;
	if (commit_begin(&state, 3)) {
		return false;
	}

	state.enabled = true;
	prepare_power_down(&state);
	if (!flush(&state) || commit_begin(&state, 4)) {
		return false;
	}
	return state.nvs_matrix == state.retained_matrix;
}

static bool dirty_try_lock(struct dirty_table *table)
{
	if (table->locked) {
		return false;
	}
	table->locked = true;
	return true;
}

static bool dirty_clear_id(struct dirty_table *table, unsigned id)
{
	if (!dirty_try_lock(table)) {
		return false;
	}
	for (size_t i = 0; i < table->count; i++) {
		if (table->slots[i].id == id) {
			table->slots[i] = table->slots[table->count - 1];
			table->count--;
			break;
		}
	}
	table->locked = false;
	return true;
}

static bool dirty_slot_update_is_serialized(void)
{
	struct dirty_table table = {
		.slots = {
			{.id = 5, .ptr = 0x1000, .len = 48},
			{.id = 33, .ptr = 0x2000, .len = 4},
		},
		.count = 2,
	};
	if (!dirty_try_lock(&table)) {
		return false;
	}
	table.slots[0].ptr = 0x3000;
	if (dirty_clear_id(&table, 5)) {
		return false;
	}
	table.slots[0].len = 48;
	table.locked = false;
	if (!dirty_clear_id(&table, 5)) {
		return false;
	}
	return table.count == 1 && table.slots[0].id == 33
		&& table.slots[0].ptr == 0x2000 && table.slots[0].len == 4;
}

static bool norm_track(struct norm_state *state, float sample)
{
	if (state->locked) {
		return false;
	}
	state->locked = true;
	if (state->count == 0) {
		state->mean = sample;
		state->variance = 0.0f;
	}
	state->count++;
	state->locked = false;
	return true;
}

static bool norm_reset_is_serialized(void)
{
	struct norm_state state = {
		.mean = 1.0f,
		.variance = 0.01f,
		.count = 120,
		.locked = true,
	};
	state.count = 0;
	if (norm_track(&state, 2.0f)) {
		return false;
	}
	state.mean = 0.0f;
	state.variance = 0.0f;
	state.locked = false;
	if (!norm_track(&state, 2.0f)) {
		return false;
	}
	return state.count == 1 && state.mean == 2.0f && state.variance == 0.0f;
}

int main(void)
{
	int failures = 0;
	if (!commit_and_flush_are_serialized()) {
		fputs("FAIL: commit/flush transaction was not serialized\n", stderr);
		failures++;
	}
	if (!dirty_slot_update_is_serialized()) {
		fputs("FAIL: protected warm-dirty update corrupted a moved slot\n", stderr);
		failures++;
	}
	if (!norm_reset_is_serialized()) {
		fputs("FAIL: protected norm reset mixed state epochs\n", stderr);
		failures++;
	}
	if (failures != 0) {
		return 1;
	}
	puts("PASS: persistence interleavings remained serialized");
	return 0;
}
