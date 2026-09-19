#ifndef SLIMENRF_SYSTEM_POWER_REQUEST_H
#define SLIMENRF_SYSTEM_POWER_REQUEST_H

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

/* Private to the power owner; zero initialization creates an empty mailbox. */
enum sys_power_request {
	SYS_POWER_REQ_NONE = 0,
	SYS_POWER_REQ_WOM,
	SYS_POWER_REQ_WOM_FORCE,
	SYS_POWER_REQ_SYSTEM_OFF,
	SYS_POWER_REQ_REBOOT,
};

enum power_request_state {
	POWER_REQUEST_EMPTY = 0,
	POWER_REQUEST_QUEUED,
	POWER_REQUEST_EXECUTING,
	POWER_REQUEST_RETRY,
};

/* OTA holds off physical transitions while preparing the bootloader, then
 * supersedes deferred ordinary work with its required reboot. */
enum power_ota_reboot_state {
	POWER_OTA_REBOOT_NONE = 0,
	POWER_OTA_REBOOT_RESERVED,
	POWER_OTA_REBOOT_READY,
};

struct power_request_mailbox {
	struct k_spinlock lock;
	enum sys_power_request request;
	enum power_request_state state;
	enum power_ota_reboot_state ota_reboot;
	uint32_t generation;
	bool physical_started;
};

/* Ordinary requests are first-winner through execution and retry, except for
 * the explicit OTA reservation below. Ordinary duplicates do not re-wake. */
static inline int power_request_submit(struct power_request_mailbox *mailbox,
				      enum sys_power_request request, struct k_sem *wake)
{
	if (request <= SYS_POWER_REQ_NONE || request > SYS_POWER_REQ_REBOOT) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&mailbox->lock);
	if (mailbox->physical_started) {
		k_spin_unlock(&mailbox->lock, key);
		return -EBUSY;
	}
	if (mailbox->ota_reboot != POWER_OTA_REBOOT_NONE) {
		int err = request == SYS_POWER_REQ_REBOOT &&
			mailbox->ota_reboot == POWER_OTA_REBOOT_READY ? 0 : -EBUSY;
		k_spin_unlock(&mailbox->lock, key);
		return err;
	}
	if (mailbox->state != POWER_REQUEST_EMPTY) {
		int err = mailbox->request == request ? 0 : -EBUSY;
		k_spin_unlock(&mailbox->lock, key);
		return err;
	}
	mailbox->request = request;
	mailbox->generation++;
	mailbox->state = POWER_REQUEST_QUEUED;
	k_spin_unlock(&mailbox->lock, key);
	k_sem_give(wake);
	return 0;
}

/* Only the power thread claims/finishes requests. Never remove a request at
 * claim time: a producer cannot replace the transition while it is executing.
 */
static inline enum sys_power_request power_request_begin(struct power_request_mailbox *mailbox,
							 uint32_t *generation)
{
	k_spinlock_key_t key = k_spin_lock(&mailbox->lock);
	enum sys_power_request request = SYS_POWER_REQ_NONE;
	if (mailbox->state != POWER_REQUEST_EXECUTING &&
	    mailbox->ota_reboot == POWER_OTA_REBOOT_READY) {
		mailbox->request = SYS_POWER_REQ_REBOOT;
		mailbox->generation++;
		mailbox->state = POWER_REQUEST_QUEUED;
	}
	if (mailbox->ota_reboot != POWER_OTA_REBOOT_RESERVED &&
	    (mailbox->state == POWER_REQUEST_QUEUED || mailbox->state == POWER_REQUEST_RETRY)) {
		request = mailbox->request;
		mailbox->state = POWER_REQUEST_EXECUTING;
		*generation = mailbox->generation;
	}
	k_spin_unlock(&mailbox->lock, key);
	return request;
}

static inline void power_request_finish(struct power_request_mailbox *mailbox,
					enum sys_power_request request, uint32_t generation, bool consumed)
{
	k_spinlock_key_t key = k_spin_lock(&mailbox->lock);
	if (mailbox->state == POWER_REQUEST_EXECUTING && mailbox->request == request &&
	    mailbox->generation == generation) {
		mailbox->physical_started = false;
		if (consumed) {
			if (request == SYS_POWER_REQ_REBOOT) {
				mailbox->ota_reboot = POWER_OTA_REBOOT_NONE;
			}
			mailbox->request = SYS_POWER_REQ_NONE;
			mailbox->state = POWER_REQUEST_EMPTY;
		} else {
			mailbox->state = POWER_REQUEST_RETRY;
		}
	}
	k_spin_unlock(&mailbox->lock, key);
}

/* Revoke only reversible WOM, leaving any other mailbox owner untouched.
 * Return whether intent may be withdrawn (also true if the mailbox no longer
 * contains WOM); physical shutdown is irreversible and returns false. */
static inline bool power_request_cancel_wom(struct power_request_mailbox *mailbox)
{
	k_spinlock_key_t key = k_spin_lock(&mailbox->lock);
	bool reversible = !mailbox->physical_started;
	if (reversible &&
	    (mailbox->request == SYS_POWER_REQ_WOM || mailbox->request == SYS_POWER_REQ_WOM_FORCE)) {
		mailbox->generation++;
		mailbox->request = SYS_POWER_REQ_NONE;
		mailbox->state = POWER_REQUEST_EMPTY;
	}
	k_spin_unlock(&mailbox->lock, key);
	return reversible;
}

/* The policy mutex excludes re-planning, not mailbox owner completion. Check
 * the claim under the mailbox lock before allowing a stale owner to cancel
 * policy belonging to a replacement. The physical gate still rechecks it. */
static inline bool power_request_wom_claim_current(struct power_request_mailbox *mailbox,
						   uint32_t generation)
{
	k_spinlock_key_t key = k_spin_lock(&mailbox->lock);
	bool current = mailbox->state == POWER_REQUEST_EXECUTING &&
		mailbox->generation == generation && !mailbox->physical_started &&
		(mailbox->request == SYS_POWER_REQ_WOM || mailbox->request == SYS_POWER_REQ_WOM_FORCE);
	k_spin_unlock(&mailbox->lock, key);
	return current;
}

static inline bool power_request_start_wom(struct power_request_mailbox *mailbox,
					   uint32_t generation)
{
	k_spinlock_key_t key = k_spin_lock(&mailbox->lock);
	bool allowed = mailbox->state == POWER_REQUEST_EXECUTING &&
		mailbox->generation == generation && !mailbox->physical_started &&
		mailbox->ota_reboot == POWER_OTA_REBOOT_NONE &&
		(mailbox->request == SYS_POWER_REQ_WOM || mailbox->request == SYS_POWER_REQ_WOM_FORCE);
	if (allowed) {
		mailbox->physical_started = true;
	}
	k_spin_unlock(&mailbox->lock, key);
	return allowed;
}

/* Reserve before touching bootloader state. An EXECUTING request may still be
 * in its reversible checks: its physical gate below will defer to us. Once
 * physical shutdown has started, OTA must not prepare or replace that action. */
static inline int power_request_ota_reserve(struct power_request_mailbox *mailbox)
{
	k_spinlock_key_t key = k_spin_lock(&mailbox->lock);
	int err = 0;
	if (mailbox->physical_started || mailbox->ota_reboot != POWER_OTA_REBOOT_NONE) {
		err = -EBUSY;
	} else {
		mailbox->ota_reboot = POWER_OTA_REBOOT_RESERVED;
	}
	k_spin_unlock(&mailbox->lock, key);
	return err;
}

/* Only the caller whose OTA reservation succeeded may resolve it, exactly once.
 * Failed preparation preserves the ordinary slot; successful preparation
 * guarantees a reboot after any in-flight reversible owner call returns. */
static inline void power_request_ota_resolve(struct power_request_mailbox *mailbox,
					   bool prepared, struct k_sem *wake)
{
	k_spinlock_key_t key = k_spin_lock(&mailbox->lock);
	if (mailbox->ota_reboot == POWER_OTA_REBOOT_RESERVED) {
		mailbox->ota_reboot = prepared ? POWER_OTA_REBOOT_READY : POWER_OTA_REBOOT_NONE;
	}
	k_spin_unlock(&mailbox->lock, key);
	k_sem_give(wake);
}

/* Owner-only, immediately before irreversible shutdown prep, including private
 * battery/dock paths. No lock is held while hardware or persistence runs. */
static inline bool power_request_start_physical(struct power_request_mailbox *mailbox,
					       bool reboot)
{
	k_spinlock_key_t key = k_spin_lock(&mailbox->lock);
	bool allowed = mailbox->ota_reboot == POWER_OTA_REBOOT_NONE ||
		(mailbox->ota_reboot == POWER_OTA_REBOOT_READY && reboot);
	if (allowed) {
		mailbox->physical_started = true;
	}
	k_spin_unlock(&mailbox->lock, key);
	return allowed;
}

#endif
