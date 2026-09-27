#ifndef SYSTEM_HEATER_H
#define SYSTEM_HEATER_H

#include <stdbool.h>
#include <stdint.h>

#define HEATER_HW_LEASE_MS 2000
#define HEATER_HW_PERIOD_NS 20000000U

struct heater_hw_status {
	bool available;
	bool armed; /* Logical session, not proof that PWM is still running. */
	bool expired;
	bool faulted;
	int last_error;
	uint32_t generation;
	uint16_t duty_pptt; /* Last command; hardware expiry can already have stopped it. */
	int64_t deadline_ms;
};

/* Thread-context only. Arm never energizes; generation must advance per session. */
int heater_hw_arm(uint32_t generation);
int heater_hw_write(uint32_t generation, uint32_t sample_sequence,
		    int64_t sampled_at_ms, float raw_temperature_c, uint16_t duty_pptt);
/* Bounded synchronous off. A negative result is NOT an off confirmation. */
int heater_hw_force_off(void);
bool heater_hw_expired(void);
bool heater_hw_available(void);
void heater_hw_get_status(struct heater_hw_status *out);

#endif
