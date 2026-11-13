/* Scalar three-pass cache builder retained for independent equivalence. */
size_t reference_build_repair(uint8_t *buf, uint8_t slot)
{
	struct relay_repair repair
		= {.helper = control_helper, .epoch = control_plan.epoch, .session = control_plan.session};
	unsigned key = irq_lock();
	unsigned grant = slot - RELAY_V3_REPAIR_FIRST_SLOT;
	repair_pending_mask = 0;
	pending_report.count = 0;
#ifdef CONFIG_RELAY_FRAME_LEDGER
	ledger_pending_count = 0;
#endif
	memset(repair_pending, 0, sizeof(repair_pending));
	repair_attempted |= 1U << grant;
	if (!control_plan.grants[grant].children) {
		struct relay_peer_report report
			= {.helper = control_helper, .epoch = control_plan.epoch, .session = control_plan.session};
		uint16_t selected = 0;
		for (unsigned n = 0; n < RELAY_PEER_REPORT_ENTRIES; n++) {
			int best = -1;
			for (unsigned c = 0; c < RELAY_MAX_TRACKERS; c++) {
				if (!(selected & (1U << c)) && peer_counts[c].expected
					&& (best < 0 || peer_counts[c].expected > peer_counts[best].expected)) {
					best = c;
				}
			}
			if (best < 0) {
				break;
			}
			report.entries[report.count++] = (struct relay_peer_observation){
				.child = best,
				.window_end = control_plan.frame,
				.expected = peer_counts[best].expected,
				.heard = peer_counts[best].heard,
				.rssi = peer_counts[best].rssi_x8 / 8U
			};
			selected |= 1U << best;
		}
		pending_report = report;
		size_t n = report.count ? relay_peer_report_encode(buf, &report) : 0;
		irq_unlock(key);
		return n;
	}
	size_t length = RELAY_REPAIR_HEADER_LEN;
	/* A newer pose must not obsolete an older repair held by another helper.
	 * Fill consecutive missing ages in one packet; a gap waits for the next
	 * B confirmation or the existing cache expiry, never a longer deadline. */
	uint16_t blocked = 0;
	for (int age = RELAY_V3_CACHE_FRAMES - 1; age >= 0; age--) {
		uint16_t included = 0;
		for (unsigned i = 0; i < RELAY_CACHE_RECORDS && repair.count < RELAY_MAX_CHILDREN; i++) {
			struct relay_record *r = &records[i];
			if (!r->valid || (repair_selected & (1ULL << i)) || (blocked & (1U << r->child))
				|| (uint16_t)(control_plan.frame - r->capture_frame) != age
				|| !(control_plan.grants[grant].children & (1U << r->child))
				|| length + RELAY_REPAIR_RECORD_META_LEN + r->length > RELAY_FRAME_MAX) {
				continue;
			}
			repair.records[repair.count++] = (struct relay_repair_record){
				.capture_frame = r->capture_frame,
				.source = {.frame = r->data, .length = r->length, .rssi = r->rssi}
			};
			length += RELAY_REPAIR_RECORD_META_LEN + r->length;
			included |= 1U << r->child;
			repair_selected |= 1ULL << i;
			repair_pending_mask |= 1ULL << i;
			repair_diag[r->child].selected++;
			ledger_note(
				control_plan.session,
				control_plan.frame - age,
				r->child,
				LEDGER_SELECTED,
				-(int16_t)r->rssi,
				control_helper,
				age
			);
			repair_pending[r->child]++;
#ifdef CONFIG_RELAY_FRAME_LEDGER
			ledger_pending[ledger_pending_count].frame = control_plan.frame - age;
			ledger_pending[ledger_pending_count].child = r->child;
			ledger_pending[ledger_pending_count++].age = age;
#endif
		}
		/* R2 is the last opportunity in the narrow pool. An uncaptured
		 * age-two record cannot arrive after it within the cache deadline,
		 * so it must not suppress a younger copy held by this final owner.
		 * Earlier grants and age-one gaps retain chronological blocking. */
		bool final_expiring_gap = control_plan.repair_count >= 1 && control_plan.repair_count <= 3
							   && grant == control_plan.repair_count - 1U && age == RELAY_V3_CACHE_FRAMES - 1;
		if (!final_expiring_gap) {
			blocked |= control_plan.missing[age] & ~included;
		}
	}
	size_t n = repair.count ? relay_repair_encode(buf, &repair) : 0;
	irq_unlock(key);
	return n;
}
