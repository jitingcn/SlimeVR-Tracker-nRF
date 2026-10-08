#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../led_feedback_stub.h"

#ifndef CONFIG_BOOTLOADER_MCUBOOT
#define CONFIG_BOOTLOADER_MCUBOOT 0
#endif
#ifndef CONFIG_BUILD_OUTPUT_UF2
#define CONFIG_BUILD_OUTPUT_UF2 0
#endif
#ifndef CONFIG_SOC_SERIES_NRF52
#define CONFIG_SOC_SERIES_NRF52 0
#endif
/* MMIO is W1C, not ordinary RAM. This also handles the old x = x clear. */
struct reset_register {
	uint32_t value = 0;
	unsigned writes = 0;
	operator uint32_t() const { return value; }
	reset_register &operator=(uint32_t mask)
	{
		value &= ~mask;
		++writes;
		return *this;
	}
	reset_register &operator=(const reset_register &other)
	{
		return *this = static_cast<uint32_t>(other);
	}
};
struct peripheral { reset_register RESETREAS; unsigned GPREGRET = 0; unsigned GPREGRET2 = 0x5A; } registers;
struct timer { uint32_t CC[4] = {}; } timer2;
#define NRF_TIMER2 (&timer2)
#define NRF_POWER (&registers)
#if MODEL_SOC == 54
#define NRF_RESET (&registers)
/* nRF54L15/LM20A: DOG0 bit 1, DOG1 bit 2, no VBUS field. */
#define RESET_RESETREAS_RESETPIN_Msk 1u
#define RESET_RESETREAS_DOG0_Msk 2u
#define RESET_RESETREAS_DOG1_Msk 4u
#define RESET_RESETREAS_OFF_Msk (1u << 8)
#else
/* nRF52833/840 fields. */
#define POWER_RESETREAS_RESETPIN_Msk 1u
#define POWER_RESETREAS_DOG_Msk 2u
#define POWER_RESETREAS_OFF_Msk (1u << 16)
#define POWER_RESETREAS_VBUS_Msk (1u << 20)
#endif

struct init_entry { int (*run)(); int stage; int priority; };
static std::vector<init_entry> init_entries;
struct init_registration {
	init_registration(int (*run)(), int stage, int priority)
	{
		init_entries.push_back({run, stage, priority});
	}
};
#define PRE_KERNEL_1 0
#define APPLICATION 1
#ifndef CONFIG_APPLICATION_INIT_PRIORITY
#define CONFIG_APPLICATION_INIT_PRIORITY 90
#endif
#define SYS_INIT(fn, stage, priority) static init_registration init_##fn(fn, stage, priority)
static int current_stage = -1;
static int current_priority = -1;
struct boot_schedule {
	const bool wake;
	const bool watchdog;
	const int stage;
	const int priority;
};
static std::vector<boot_schedule> boot_schedules;
static void tracker_events_schedule_boot(bool wake, bool watchdog)
{
	boot_schedules.push_back({wake, watchdog, current_stage, current_priority});
}
#define DT_NODE_HAS_PROP(...) 0
#define BUTTON_EXISTS 1
#define USER_SHUTDOWN_ENABLED 0
#define ADAFRUIT_BOOTLOADER 0
#ifndef CONFIG_USER_EXTRA_ACTIONS
#define CONFIG_USER_EXTRA_ACTIONS 0
#endif
#define LOG_INF(...) ((void)0)
#define BIT(n) (1u << (n))
#define GPIO_INPUT 0
#define GPIO_INT_EDGE_BOTH 0

struct gpio_spec { int pin; int port; };
static gpio_spec button0 = {0, 0};
static int button_cb_data;
static bool button_held_from_init;
static void button_interrupt_handler() {}
static void gpio_pin_configure_dt(const gpio_spec *, int) {}
static void gpio_pin_interrupt_configure_dt(const gpio_spec *, int) {}
static void gpio_init_callback(int *, void (*)(), unsigned) {}
static void gpio_add_callback(int, int *) {}
static int gpio_pin_get_dt(const gpio_spec *) { return 1; }
static unsigned nrf_power_gpregret_get(peripheral *p, int) { return p->GPREGRET; }
static void nrf_power_gpregret_set(peripheral *p, int, unsigned value) { p->GPREGRET = value; }
static uint8_t reboot_counter_read() { return 100; }
static std::vector<uint8_t> counter_writes;
static void reboot_counter_write(uint8_t value) { counter_writes.push_back(value); }
static bool button_read() { return false; }
static bool dock_read() { return false; }
static int system_uptime_since_boot_ms() { return 0; }
static void esb_reset_pair() {}
static void k_msleep(int) {}
static void k_usleep(int) {}
static uint8_t applied_reset_mode;
static void sys_reset_mode(uint8_t mode) { applied_reset_mode = mode; }
#define CUSTOMER_INFO_REPORT_LOG_SUMMARY 0
static void customer_info_report(int) {}

static struct { struct { uint8_t reset_count; } watchdog_state; } retained_storage, *retained = &retained_storage;
static void retained_update() {}
static bool locked;
static unsigned inject_marker;
static unsigned int irq_lock()
{
	assert(!locked);
	if (inject_marker) registers.GPREGRET = inject_marker;
	locked = true;
	return 0;
}
static void irq_unlock(unsigned int) { assert(locked); locked = false; }
struct arch_esf {};
struct rebooted {};
static unsigned barriers, panics;
#define ARG_UNUSED(x) (void)(x)
#define CODE_UNREACHABLE __builtin_unreachable()
#define LOG_PANIC() (++panics)
static unsigned expected_fatal;
static void __DSB() { assert(registers.GPREGRET == expected_fatal); ++barriers; }
[[noreturn]] static void sys_arch_reboot(int)
{
	assert(!locked);
#if MODEL_SOC == 52 && CONFIG_BUILD_OUTPUT_UF2 && !CONFIG_BOOTLOADER_MCUBOOT
	assert(registers.GPREGRET == expected_fatal && barriers == 1 && panics == 0);
#else
	assert(barriers == 0 && panics == 1);
#endif
	throw rebooted{};
}
#include "production.inc"

int main(int argc, char **argv)
{
	assert(argc == 4);
	const uint32_t reason = static_cast<uint32_t>(strtoul(argv[1], nullptr, 0));
	const unsigned marker = static_cast<unsigned>(strtoul(argv[2], nullptr, 0));
	const uint32_t token = static_cast<uint32_t>(strtoul(argv[3], nullptr, 0));
	const bool supported = ADAFRUIT_FAULT_RECOVERY && token == ADAFRUIT_RECOVERY_CAPABILITY;
	assert(!sys_bootloader_supports_recovery());
	/* Even a valid unread token cannot authorize E3 before early init. */
	timer2.CC[0] = 0x01020304;
	timer2.CC[1] = token;
	expected_fatal = 0x57;
	try {
		k_sys_fatal_error_handler(42, nullptr);
		assert(false);
	} catch (const rebooted &) {}
	assert(registers.GPREGRET2 == 0x5A);
	barriers = panics = 0;
	expected_fatal = supported ? 0xE3 : 0x57;
	registers.GPREGRET = marker;
	registers.RESETREAS.value = reason;
	uint32_t expected_reason = reason;
#if MODEL_SOC == 52 && CONFIG_BUILD_OUTPUT_UF2 && !CONFIG_BOOTLOADER_MCUBOOT
	if (supported && (marker == 0xE1 || marker == 0xE2)) expected_reason |= POWER_RESETREAS_DOG_Msk;
#endif
#if MODEL_SOC == 54
	const bool expected_wake = reason & RESET_RESETREAS_OFF_Msk;
	const bool expected_watchdog = reason & (RESET_RESETREAS_DOG0_Msk | RESET_RESETREAS_DOG1_Msk);
#else
	const bool expected_wake = reason & POWER_RESETREAS_OFF_Msk;
	const bool expected_watchdog = expected_reason & POWER_RESETREAS_DOG_Msk;
#endif
	std::stable_sort(init_entries.begin(), init_entries.end(), [](const auto &a, const auto &b) {
		return a.stage != b.stage ? a.stage < b.stage : a.priority < b.priority;
	});
	for (const auto &entry : init_entries) {
		current_stage = entry.stage;
		current_priority = entry.priority;
		assert(entry.run() == 0);
	}
	assert(sys_bootloader_supports_recovery() == supported);
	assert(timer2.CC[0] == 0x01020304);
	assert(timer2.CC[1] == (ADAFRUIT_FAULT_RECOVERY ? 0 : token));
	/* Reused peripheral state must neither grant nor revoke cached support. */
	timer2.CC[1] = supported ? 0 : ADAFRUIT_RECOVERY_CAPABILITY;
	assert(sys_bootloader_supports_recovery() == supported);
	/* Scheduling cannot wait for main's button handling or require TASK_WDT. */
	assert(boot_schedules.size() == 1);
	assert(boot_schedules[0].wake == expected_wake);
	assert(boot_schedules[0].watchdog == expected_watchdog);
	assert(sys_boot_woke_from_off() == expected_wake);

	/* The decision immediately before retained validation must still see pin. */
	assert(ram_retention_valid == !(reason & 1u));
#if MODEL_SOC == 52
	/* A high button at USB wake must remain visible, not treated as wake hold. */
	assert(button_read_filtered() == !!(reason & (1u << 20)));
#else
	assert(!button_read_filtered());
#endif
	current_stage = APPLICATION + 1;
	current_priority = -1;
	assert(tracker_main() == 0);
	assert(boot_schedules.size() == 1);
	const bool counted_pin = (reason & 1u) && !IGNORE_RESET;
	const std::vector<uint8_t> expected = counted_pin ? std::vector<uint8_t>{101, 100}
						       : std::vector<uint8_t>{100};
	assert(counter_writes == expected);
	assert(applied_reset_mode == (counted_pin ? 0 : 255));
	assert(registers.RESETREAS.value == 0);
	assert(registers.RESETREAS.writes == 1);
	assert(watchdog_caused_reset() == expected_watchdog);
#ifdef HAS_RESET_SNAPSHOT
	assert(sys_get_reset_reason() == expected_reason);
	/* Later hardware changes cannot rewrite the identity of this boot. */
	registers.RESETREAS.value = ~reason;
	assert(sys_get_reset_reason() == expected_reason);
	assert(watchdog_caused_reset() == expected_watchdog);
	assert(sys_boot_woke_from_off() == expected_wake);
	assert(boot_schedules.size() == 1);
	assert(boot_schedules[0].wake == expected_wake);
	assert(boot_schedules[0].watchdog == expected_watchdog);
#endif
#if CONFIG_TASK_WDT
	assert(registers.GPREGRET == (marker >= 0xD0 && marker <= 0xDE ? 0 : marker));
#else
	assert(registers.GPREGRET == marker);
#endif
#if ADAFRUIT_FAULT_RECOVERY
	/* Real public API remains functional with CONFIG_TASK_WDT disabled. */
	registers.GPREGRET = marker;
	assert(watchdog_get_reset_count() == (supported ? (marker == 0xE1 ? 1 : marker == 0xE2 ? 2 : 0) : 0));
	watchdog_mark_boot_success();
	assert(registers.GPREGRET == (supported && (marker == 0xE1 || marker == 0xE2) ? 0 : marker));
	if (supported) for (unsigned pending : {0xE3u, 0x57u, 0xA8u, 0x4Eu, 0xD3u}) {
		registers.GPREGRET = 0xE2;
		inject_marker = pending;
		watchdog_clear_reset_count();
		assert(registers.GPREGRET == pending && !locked);
	}
	inject_marker = 0;
#endif
	registers.GPREGRET = marker;
	try {
		k_sys_fatal_error_handler(42, nullptr);
		assert(false);
	} catch (const rebooted &) {}
#if ADAFRUIT_FAULT_RECOVERY
	assert(registers.GPREGRET == expected_fatal && barriers == 1 && panics == 0);
#else
	assert(registers.GPREGRET == marker && barriers == 0 && panics == 1);
#endif
	assert(registers.GPREGRET2 == 0x5A);
	printf("reset model passed: soc=%d reason=0x%x ignore=%d\n", MODEL_SOC, reason, IGNORE_RESET);
}
