#include <zephyr/irq_offload.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "system/power_request.h"

ZTEST(power_request, test_first_request_owns_execution_and_retry)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t generation = 0;
	struct k_sem wake;
	k_sem_init(&wake, 0, 1);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), 0);
	zassert_equal(k_sem_take(&wake, K_NO_WAIT), 0);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), 0);
	zassert_equal(k_sem_count_get(&wake), 0, "Duplicate must not accelerate retries");
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM_FORCE, &wake), -EBUSY);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_WOM);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_REBOOT, &wake), -EBUSY);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_NONE,
		      "A claimed request must not execute twice");
	power_request_finish(&mailbox, SYS_POWER_REQ_WOM, generation, false);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, &wake), -EBUSY);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), 0);
	zassert_equal(k_sem_count_get(&wake), 0, "Retry remains paced by owner loop");
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_WOM);
	power_request_finish(&mailbox, SYS_POWER_REQ_WOM, generation, true);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_REBOOT, &wake), 0);
	zassert_equal(k_sem_take(&wake, K_NO_WAIT), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_REBOOT);
	power_request_finish(&mailbox, SYS_POWER_REQ_REBOOT, generation, true);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_NONE);
}

ZTEST(power_request, test_idle_completion_does_not_erase_new_request)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t generation = 0;
	struct k_sem wake;
	k_sem_init(&wake, 0, 1);

	enum sys_power_request idle = power_request_begin(&mailbox, &generation);
	zassert_equal(idle, SYS_POWER_REQ_NONE);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, &wake), 0);
	power_request_finish(&mailbox, idle, generation, true);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_SYSTEM_OFF);
	power_request_finish(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, generation, true);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, &wake), 0);
	/* Even the same kind newly queued after completion is not the old claim. */
	power_request_finish(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, generation, true);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_SYSTEM_OFF);
	power_request_finish(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, generation, true);
}

ZTEST(power_request, test_invalid_request_does_not_reserve_or_wake)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t generation = 0;
	struct k_sem wake;
	k_sem_init(&wake, 0, 1);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_NONE, &wake), -EINVAL);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_REBOOT + 1, &wake), -EINVAL);
	zassert_equal(k_sem_count_get(&wake), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_NONE);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, &wake), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_SYSTEM_OFF);
	power_request_finish(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, generation, true);
}

struct submit_context {
	struct power_request_mailbox *mailbox;
	struct k_sem *wake;
	struct k_sem *start;
	enum sys_power_request request;
	int result;
};

static void submit_from_thread(void *arg, void *unused1, void *unused2)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	struct submit_context *context = arg;
	k_sem_take(context->start, K_FOREVER);
	context->result = power_request_submit(context->mailbox, context->request, context->wake);
}

K_THREAD_STACK_ARRAY_DEFINE(submit_stacks, 2, 1024);
static struct k_thread submit_threads[2];

ZTEST(power_request, test_competing_producers_accept_exactly_one)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t generation = 0;
	struct k_sem wake;
	struct k_sem start;
	k_sem_init(&wake, 0, 1);
	k_sem_init(&start, 0, 2);
	struct submit_context contexts[2] = {
		{&mailbox, &wake, &start, SYS_POWER_REQ_WOM, -EINPROGRESS},
		{&mailbox, &wake, &start, SYS_POWER_REQ_REBOOT, -EINPROGRESS},
	};

	for (int i = 0; i < 2; i++) {
		k_thread_create(&submit_threads[i], submit_stacks[i],
				K_THREAD_STACK_SIZEOF(submit_stacks[i]), submit_from_thread,
				&contexts[i], NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	}
	k_sem_give(&start);
	k_sem_give(&start);
	for (int i = 0; i < 2; i++) {
		zassert_equal(k_thread_join(&submit_threads[i], K_FOREVER), 0);
	}
	zassert_true((contexts[0].result == 0 && contexts[1].result == -EBUSY) ||
		     (contexts[1].result == 0 && contexts[0].result == -EBUSY));
	int winner = contexts[0].result == 0 ? 0 : 1;
	zassert_equal(k_sem_take(&wake, K_NO_WAIT), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), contexts[winner].request);
	power_request_finish(&mailbox, contexts[winner].request, generation, true);
}

static void submit_from_isr(const void *arg)
{
	struct submit_context *context = (struct submit_context *)arg;
	context->result = power_request_submit(context->mailbox, context->request, context->wake);
}

ZTEST(power_request, test_isr_submission_and_execution_reservation)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t generation = 0;
	struct k_sem wake;
	k_sem_init(&wake, 0, 1);
	struct submit_context context = {&mailbox, &wake, NULL, SYS_POWER_REQ_WOM, -EINPROGRESS};

	irq_offload(submit_from_isr, &context);
	zassert_equal(context.result, 0);
	zassert_equal(k_sem_take(&wake, K_NO_WAIT), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_WOM);
	context.request = SYS_POWER_REQ_REBOOT;
	irq_offload(submit_from_isr, &context);
	zassert_equal(context.result, -EBUSY);
	context.request = SYS_POWER_REQ_WOM;
	irq_offload(submit_from_isr, &context);
	zassert_equal(context.result, 0);
	zassert_equal(k_sem_count_get(&wake), 0);
	power_request_finish(&mailbox, SYS_POWER_REQ_WOM, generation, true);
}

ZTEST(power_request, test_ota_reservation_survives_executing_off_completion)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t generation = 0;
	struct k_sem wake;
	k_sem_init(&wake, 0, 1);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, &wake), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_SYSTEM_OFF);
	/* Producer races after claim, before irreversible shutdown prep. */
	zassert_equal(power_request_ota_reserve(&mailbox), 0);
	zassert_false(power_request_start_physical(&mailbox, false));
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), -EBUSY);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_REBOOT, &wake), -EBUSY);
	power_request_ota_resolve(&mailbox, true, &wake);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_NONE,
		      "Never replace an owner call before it returns");
	power_request_finish(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, generation, false);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_REBOOT);
	zassert_true(power_request_start_physical(&mailbox, true));
	power_request_finish(&mailbox, SYS_POWER_REQ_REBOOT, generation, true);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_NONE);
}

ZTEST(power_request, test_ota_reservation_survives_consumed_wom_and_idle_finish)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t generation = 0;
	struct k_sem wake;
	k_sem_init(&wake, 0, 1);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_WOM);
	zassert_equal(power_request_ota_reserve(&mailbox), 0);
	/* OTA-blocked WOM is consumed, unlike OFF, which retries. */
	power_request_finish(&mailbox, SYS_POWER_REQ_WOM, generation, true);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_NONE,
		      "Preparation must finish before reboot can run");
	power_request_ota_resolve(&mailbox, true, &wake);
	power_request_finish(&mailbox, SYS_POWER_REQ_NONE, generation, true);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_REBOOT);
}

ZTEST(power_request, test_ota_failure_preserves_deferred_request)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t generation = 0;
	struct k_sem wake;
	k_sem_init(&wake, 0, 1);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, &wake), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_SYSTEM_OFF);
	power_request_finish(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, generation, false);
	zassert_equal(power_request_ota_reserve(&mailbox), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_NONE);
	zassert_false(power_request_start_physical(&mailbox, true),
		      "Ordinary reboot cannot race unfinished bootloader writes");
	power_request_ota_resolve(&mailbox, false, &wake);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_SYSTEM_OFF);
	zassert_true(power_request_start_physical(&mailbox, false));
}

ZTEST(power_request, test_ota_cannot_preempt_physical_shutdown)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t generation = 0;
	struct k_sem wake;
	k_sem_init(&wake, 0, 1);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, &wake), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_SYSTEM_OFF);
	zassert_true(power_request_start_physical(&mailbox, false));
	zassert_equal(power_request_ota_reserve(&mailbox), -EBUSY);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_REBOOT, &wake), -EBUSY);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_NONE);

	/* Battery/dock transitions also commit without claiming the mailbox. */
	struct power_request_mailbox private_mailbox = {0};
	zassert_true(power_request_start_physical(&private_mailbox, false));
	zassert_equal(power_request_ota_reserve(&private_mailbox), -EBUSY);
}

ZTEST(power_request, test_cancelled_wom_cannot_commit_or_finish_same_kind_replacement)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t old_generation = 0;
	uint32_t generation = 0;
	struct k_sem wake;
	k_sem_init(&wake, 0, 1);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), 0);
	zassert_equal(power_request_begin(&mailbox, &old_generation), SYS_POWER_REQ_WOM);
	zassert_true(power_request_cancel_wom(&mailbox));
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), 0);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_WOM);

	zassert_false(power_request_wom_claim_current(&mailbox, old_generation));
	zassert_true(power_request_wom_claim_current(&mailbox, generation));
	zassert_false(power_request_start_wom(&mailbox, old_generation));
	power_request_finish(&mailbox, SYS_POWER_REQ_WOM, old_generation, true);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_REBOOT, &wake), -EBUSY,
		      "Stale completion must not erase the replacement");
	power_request_finish(&mailbox, SYS_POWER_REQ_WOM, old_generation, false);
	zassert_equal(power_request_begin(&mailbox, &old_generation), SYS_POWER_REQ_NONE,
		      "Stale retry must not release the replacement owner");
	zassert_true(power_request_start_wom(&mailbox, generation));

	power_request_finish(&mailbox, SYS_POWER_REQ_WOM, old_generation, true);
	zassert_false(power_request_cancel_wom(&mailbox),
		      "Stale completion must not undo the replacement's physical commit");
	zassert_equal(power_request_ota_reserve(&mailbox), -EBUSY);
	power_request_finish(&mailbox, SYS_POWER_REQ_WOM, generation, true);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_REBOOT, &wake), 0);
}

ZTEST(power_request, test_cancelled_wom_preserves_ota_reservation)
{
	struct power_request_mailbox mailbox = {0};
	uint32_t old_generation = 0;
	uint32_t generation = 0;
	struct k_sem wake;
	k_sem_init(&wake, 0, 1);

	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), 0);
	zassert_equal(power_request_begin(&mailbox, &old_generation), SYS_POWER_REQ_WOM);
	zassert_equal(power_request_ota_reserve(&mailbox), 0);
	zassert_true(power_request_cancel_wom(&mailbox));
	zassert_false(power_request_start_wom(&mailbox, old_generation));
	power_request_finish(&mailbox, SYS_POWER_REQ_WOM, old_generation, true);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), -EBUSY);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_NONE);
	zassert_false(power_request_start_physical(&mailbox, true),
		      "Cancellation must not release unfinished bootloader preparation");

	power_request_ota_resolve(&mailbox, true, &wake);
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_REBOOT);
	power_request_finish(&mailbox, SYS_POWER_REQ_WOM, old_generation, true);
	zassert_true(power_request_start_physical(&mailbox, true));
	power_request_finish(&mailbox, SYS_POWER_REQ_REBOOT, generation, true);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_WOM, &wake), 0);
}

ZTEST(power_request, test_intent_withdrawal_preserves_other_mailbox_owners)
{
	struct power_request_mailbox mailbox = {0};
	struct k_sem wake;
	uint32_t generation;
	k_sem_init(&wake, 0, 1);
	zassert_true(power_request_cancel_wom(&mailbox));
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, &wake), 0);
	zassert_true(power_request_cancel_wom(&mailbox));
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_SYSTEM_OFF);
	zassert_true(power_request_cancel_wom(&mailbox));
	zassert_true(power_request_start_physical(&mailbox, false));
	zassert_false(power_request_cancel_wom(&mailbox));
	power_request_finish(&mailbox, SYS_POWER_REQ_SYSTEM_OFF, generation, true);
	zassert_equal(power_request_submit(&mailbox, SYS_POWER_REQ_REBOOT, &wake), 0);
	zassert_true(power_request_cancel_wom(&mailbox));
	zassert_equal(power_request_begin(&mailbox, &generation), SYS_POWER_REQ_REBOOT);
}

ZTEST_SUITE(power_request, NULL, NULL, NULL, NULL, NULL);
