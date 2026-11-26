/* Included after the extracted TDMA code (see run_tdma.py). Layout: own slot
 * 0, 8 slots of 16 ticks (frame 128 ticks), PING period 256 frames. */

static void frame_start_exact(void)
{
	uint32_t n = 0;
	for (unsigned i = 0; i < 100000; i++) {
		assert(relay_frame_start(0xfffffff0U, n) == 0xfffffff0U + (uint32_t)((uint64_t)n * 4096U / 15U));
		n = n * 1664525U + 1013904223U;
	}
	assert(relay_frame_start(0, UINT32_MAX) == (uint32_t)((uint64_t)UINT32_MAX * 4096U / 15U));
	/* Fast skew projection must preserve signed truncation at its boundary. */
	for (int32_t skew = -1000001; skew <= 1000001; skew += 100000) {
		for (uint32_t elapsed = 0; elapsed < 10000; elapsed += 17) {
			assert(esb_project_network_ticks(100 + elapsed, 100, 7, skew)
			       == 107 + elapsed + (uint32_t)((int64_t)skew * elapsed / 1000000000LL));
		}
	}
}

static void configure(uint8_t own, uint8_t total)
{
	tdma_recent_activation_valid = false;
	tdma_cfg_pack = 0;
	relay_timeline_clear(&relay_timeline_active);
	relay_direct_anchor.valid = false;
	assert(tdma_update_config(own, total, 16, 7));
	tdma_last_admitted_frame = tdma_repair_admitted_frame = UINT32_MAX;
}

static void data_windows(void)
{
	assert(TDMA_UNPACK_SLOT(TDMA_PACK(16, 16, 17, 255)) == 16);
	configure(0, 8);
	uint32_t start;
	/* Ordinary frame: next start of slot 3 strictly after `after`. */
	set_ticks(10 * 128 + 5);
	assert(tdma_slot_data_window(3, now_ticks(), &start));
	assert(start == 10 * 128 + 3 * 16);
	assert(tdma_slot_data_window(3, 10 * 128 + 3 * 16, &start));
	assert(start == 11 * 128 + 3 * 16);
	/* Slot 2 PINGs in frame 64 (phase 2*256/8) and spills into slot 3. */
	assert(tdma_slot_data_window(3, 63 * 128 + 3 * 16, &start));
	assert(start == 65 * 128 + 3 * 16);
	/* Slot 3's own PING frame (96) is skipped too. */
	assert(tdma_slot_data_window(3, 95 * 128 + 3 * 16, &start));
	assert(start == 97 * 128 + 3 * 16);
	/* Frame 0 (the 32-bit rollover frame) is never a data frame. */
	assert(tdma_slot_data_window(5, 0, &start) && start == 128 + 5 * 16);
	assert(!tdma_slot_data_window(8, 0, &start)); /* outside the layout */
	puts("PASS relay data windows follow the shared guard rules");
}

static void ping_windows(void)
{
	configure(0, 8);
	uint32_t start;
	uint16_t length;
	assert(tdma_slot_ping_window(5, 1000, &start, &length));
	assert(start == 160 * 128 + 5 * 16); /* phase 5*256/8 = 160 */
	assert(length == 2 * 16 + 2 + 5);    /* both slots, offset, late tolerance */
	assert(tdma_slot_ping_window(5, start, &start, &length));
	assert(start == (160 + 256) * 128 + 5 * 16);
	puts("PASS presence windows land on other trackers' PING exchanges");
}




static void final_admission_check(void)
{
 configure(1,8);set_ticks(20*128+16+1);
 assert(tdma_wait_for_slot(17));assert(tdma_slot_tx_valid(17));
 uint32_t now=now_ticks();set_ticks(now+16);
 assert(!tdma_slot_tx_valid(17));
 set_ticks(now);tdma_cfg_pack^=(1U<<24);
 assert(!tdma_slot_tx_valid(17));
 puts("PASS final TX validation rejects elapsed and replaced admissions");
}

/* Staged layout activation: the staged tuple is promoted by the first
 * snapshot at or after its activation tick, no data or PING window straddles
 * that tick, an unassigned stage withdraws, and a later tuple recovers. */
static void postpone_same_stage(void)
{
 atomic_set(&tdma_staged_activation, atomic_get(&tdma_staged_activation)+100);
 tdma_staged_local += 1000000;
}
static void staged_deadline_race(void)
{
 struct tdma_layout layout;
 configure(0,8);set_ticks(128000);
 assert(tdma_stage_config(1,8,16,8,128100));set_ticks(128100);
 lock_hook=postpone_same_stage;
 assert(tdma_layout_get(&layout) && layout.epoch==7);
 tdma_stage_clear();
}

static void staged_activation(void)
{
	struct tdma_layout layout;
	configure(0, 8);
	set_ticks(1000 * 128 + 20);
	uint32_t act = 1003 * 128 + 2; /* 2 ticks into own slot 0 of frame 1003 */
	assert(tdma_stage_config(3, 9, 16, 8, act));
	assert(!tdma_stage_config(9, 9, 16, 8, act)); /* slot outside layout */
	assert(tdma_layout_get(&layout) && layout.epoch == 7 && layout.total_slots == 8);
	/* Frames 1001 and 1002 are clear of the activation: old layout. */
	assert(tdma_wait_for_slot(17));
	assert(now_ticks() / 128 == 1001 && tdma_get_config_epoch() == 7);
	assert(tdma_wait_for_slot(17));
	assert(now_ticks() / 128 == 1002);
	/* Frame 1003's own slot straddles the activation: never admitted. The
	 * next admission is in the new layout (slot 3 of 9, frame 144 ticks);
	 * the wait that crosses the activation aborts and the caller retries. */
	assert(!tdma_wait_for_slot(17));
	assert(tdma_wait_for_slot(17));
	assert(now_ticks() >= act + TDMA_ACTIVATION_QUIET_TICKS);
	assert(tdma_layout_get(&layout) && layout.epoch == 8 && layout.slot_index == 3
		&& layout.total_slots == 9);
	assert(now_ticks() % 144 >= 3 * 16 && now_ticks() % 144 < 4 * 16);
	assert(!tdma_layout_withdrawn());

	/* Promotion must preserve the exact same quiet window. */
	act = now_ticks() + 1000;
	assert(tdma_stage_config(3, 9, 16, 9, act));
	assert(tdma_window_quiet(act - 2, 16));
	set_ticks(act);
	assert(tdma_layout_get(&layout) && layout.epoch == 9);
	assert(tdma_window_quiet(act - 2, 16));
	assert(!tdma_window_quiet(act + TDMA_ACTIVATION_QUIET_TICKS, 16));
	assert(tdma_update_config(3, 9, 16, 8));

	/* Removal: an unassigned stage disables TDMA at activation. */
	act = now_ticks() + 1000;
	assert(tdma_stage_config(TDMA_SLOT_UNASSIGNED, 8, 16, 9, act));
	assert(tdma_layout_get(&layout) && layout.epoch == 8);
	set_ticks(act);
	assert(!tdma_layout_get(&layout) && tdma_layout_withdrawn());
	assert(!tdma_wait_for_slot(17));
	/* A later authoritative tuple restores transmission rights. */
	assert(tdma_update_config(2, 8, 16, 10));
	assert(!tdma_layout_withdrawn() && tdma_layout_get(&layout) && layout.epoch == 10);

	/* A stage cleared before activation (receiver dropped it) never applies;
	 * a tuple of the staged epoch, heard directly, supersedes the stage. */
	act = now_ticks() + 1000;
	assert(tdma_stage_config(1, 8, 16, 11, act));
	tdma_stage_clear();
	set_ticks(act + 10);
	assert(tdma_layout_get(&layout) && layout.epoch == 10);
	act = now_ticks() + 1000;
	assert(tdma_stage_config(1, 8, 16, 11, act));
	assert(tdma_update_config(4, 8, 16, 11));
	set_ticks(act + 10);
	assert(tdma_layout_get(&layout) && layout.epoch == 11 && layout.slot_index == 4);
	puts("PASS staged layout activation: promotion, quiet zone, withdrawal, recovery");
}

/* Review counterexample (old slot 0 and a new forwarding slot admitting at
 * the same tick): with staging, every admission around an activation, at any
 * phase, lies wholly in the old layout before the quiet zone or wholly in the
 * new one after it, so trackers that switch together cannot mix layouts.
 * Trackers that missed the announcement are held off by the layout grant
 * (esb_layout_granted, run_submission.py / run_presence.py). */
static void staged_no_mixed_layouts(void)
{
	for (uint32_t phase = 0; phase < 2 * 144; phase++) {
		configure(0, 11);   /* old: own slot 0 of 11 x 16 ticks (176/frame) */
		uint32_t act = 2000 * 176 + phase;
		set_ticks(act - 3 * 176);
		assert(tdma_stage_config(1, 12, 16, 8, act)); /* new: slot 1 of 12 */
		for (unsigned n = 0; n < 8; n++) {
			if (!tdma_wait_for_slot(17)) {
				continue; /* aborted at the promotion; retried */
			}
			uint32_t t = now_ticks();
			struct tdma_layout layout;
			assert(tdma_layout_get(&layout));
			uint32_t slot_start = t - (t % layout.frame_ticks) + layout.slot_index * layout.slot_ticks;
			uint32_t slot_end = slot_start + layout.slot_ticks;
			if (layout.epoch == 7) {
				assert(slot_end <= act - TDMA_ACTIVATION_QUIET_TICKS);
			} else {
				assert(layout.epoch == 8 && layout.total_slots == 12);
				assert(slot_start >= act + TDMA_ACTIVATION_QUIET_TICKS);
			}
			set_ticks(t + 1);
		}
		assert(tdma_get_config_epoch() == 8);
	}
	puts("PASS staged activation never mixes layouts at any activation phase");
}

static void stream_profile(void)
{
 const uint32_t origin=123456;
 configure(2,3);set_ticks(origin-1000);
 assert(tdma_stage_config(2, relay_geometry_code(11, 5), 16, 8, origin));
 /* A running profile is announced alongside a same-epoch legacy tuple. */
 assert(tdma_update_config(2,10,16,8));
 set_ticks(origin);assert(tdma_profile_get()==1);
 granted = false;
 uint32_t first; bool control_frame;
 assert(tdma_stream_next(&first, &control_frame) && control_frame);
 assert(first == origin); /* No control frame yet: continuous acquisition, no poses. */
 granted = true;
 struct tdma_layout layout;assert(tdma_layout_get(&layout));
 assert(layout.profile==1 && layout.total_slots==17 && layout.slot_index==2);
 for (unsigned f = 1; f <= 120; f++) {
	 struct relay_control control
		 = {.frame = f - 1,
			.preamble_ticks = origin + (f - 1U) * 276U + 182U,
			.direct_count = 11,
			.repair_count = 5,
			.slot_ticks = 16,
			.tps_max = 120};
	 relay_control_set_next_repairs(&control, 5);
	 tdma_stream_control(&control);
	 uint32_t own = relay_frame_start(origin, f) + 34;
	 set_ticks(own - 1);
	 uint32_t target;
	 bool b;
	 assert(tdma_stream_next(&target, &b) && !b && target == own);
	 set_ticks(own + 1);
	 assert(tdma_stream_next(&target, &b) && !b && target == own + 1);
	 assert(tdma_wait_for_slot(60));
	 assert(tdma_slot_tx_valid(60) && !tdma_wait_for_slot(17));
	 set_ticks(own + 1);
	 assert(tdma_stream_next(&target, &b) && b);
	 assert(target == relay_frame_start(origin, f) + 176);
	 control.frame = f;
	 control.preamble_ticks = origin + f * 276U + 182U;
	 tdma_stream_control(&control);
	 for (unsigned slot = RELAY_REPAIR_FIRST_SLOT; slot < RELAY_TOTAL_SLOTS; slot++) {
		 uint32_t tx_target = relay_frame_start(origin, f) + relay_slot_offset(slot) + RELAY_TX_OFFSET;
		 set_ticks(tx_target - 4U);
		 assert(tdma_wait_for_repair_slot(slot, 72));
		 assert(now_ticks() >= tx_target && now_ticks() <= tx_target + 2U);
		 assert(tdma_repair_tx_valid(slot, 72));
		 assert(!tdma_wait_for_repair_slot(slot, 72));
	 }
 }
 assert(relay_frame_start(origin,120)==origin+120U*276U);
 puts("PASS profile bootstrap, 120 direct admissions and compulsory control frame deadlines");
}

static void stream_d0_boundary(void)
{
 const uint32_t origins[] = {123456U, 0xfffffe00U};
 for (unsigned o=0;o<2;o++) {
  uint32_t origin=origins[o];
  configure(0,3);set_ticks(origin-1000);
  assert(tdma_stage_config(0, relay_geometry_code(11, 5), 16, 8, origin));
  set_ticks(origin);assert(tdma_profile_get()==1);granted=true;
  for (unsigned f=1;f<120;f++) {
	  struct relay_control control
		  = {.frame = f - 1,
			 .preamble_ticks = origin + (f - 1U) * 276U + 182U,
			 .direct_count = 11,
			 .repair_count = 5,
			 .slot_ticks = 16,
			 .tps_max = 120};
	  relay_control_set_next_repairs(&control, 5);
	  tdma_stream_control(&control);
	  uint32_t start = relay_frame_start(origin, f);
	  set_ticks(start - 3);
	  assert(!tdma_wait_for_slot(60)); /* Outside the unchanged early hold. */
	  set_ticks(start - 2);
	  assert(!tdma_slot_tx_valid(60)); /* Never transmit in the preceding frame. */
	  assert(f & 1 ? tdma_admit_stream_source(60) : tdma_wait_for_slot(60));
	  assert(now_ticks() == start + TDMA_SLOT_TARGET_OFFSET);
	  assert(tdma_slot_tx_valid(60));
	  assert(!tdma_wait_for_slot(60));
	  set_ticks(start + RELAY_SLOT_TICKS - TDMA_LEN33_PLUS_RESERVE_TICKS + 1);
	  assert(!tdma_slot_tx_valid(60)); /* Original final deadline remains exact. */
  }
 }
 puts("PASS D0 early handoff across frame and clock rollover, with unchanged deadline");
}

static void coarse_sleep_entry_delay(void)
{
 configure(0, 8); set_ticks(1000);
 sleep_entry_delay = 150;
 assert(tdma_wait_network_tick(1040));
 assert(now_ticks() == 1040);
 sleep_entry_delay = 0;
 puts("PASS coarse wait preserves target despite sleep-entry work");
}

static void stream_only_admission(void)
{
 configure(3,8);set_ticks(1000);unsigned before=sleeps;
 assert(!tdma_admit_stream_source(17));
 assert(sleeps==before);
}

static void layout_full16(void)
{
	configure(10, 16);
	set_ticks(1000);
	assert(tdma_stage_config(10, relay_geometry_code(16, 3), 15, 9, 2000));
	set_ticks(2000);
	assert(tdma_profile_get() == 1);
	assert(RELAY_DIRECT_SLOTS == 16 && RELAY_REPAIR_SLOTS == 3);
	for (unsigned f = 0; f < 10000; f++) {
		uint32_t start = relay_frame_start(2000, f);
		assert(start == 2000 + f * 304);
		assert(relay_frame_at_tick(2000, start + 303) == f);
	}
	assert(!tdma_stage_config(10, relay_geometry_code(16, 6), 16, 10, 3000));
	assert(!tdma_stage_config(10, relay_geometry_code(16, 3), 14, 10, 3000));
	assert(tdma_update_config(10, 16, 16, 10));
	assert(tdma_profile_get() == 0);
	puts("PASS 304-tick receiver bootstrap, exact frames and invalid layouts");
}

static void variable_repair_timeline(void)
{
	const uint32_t origin = 10000;
	configure(0, 16);
	set_ticks(origin - 100);
	assert(tdma_stage_config(0, relay_geometry_code(16, 1), 16, 8, origin));
	set_ticks(origin);
	assert(tdma_profile_get() == 1);
	granted = true;
	struct relay_control c
		= {.frame = 6,
		   .preamble_ticks = origin + 262,
		   .direct_count = 16,
		   .repair_count = 1,
		   .slot_ticks = 16,
		   .tps_max = 120};
	relay_control_set_next_repairs(&c, 1);
	relay_control_set_next2_repairs(&c, 2);
	tdma_stream_control(&c);
	uint32_t second = relay_direct_anchor.next_start;
	uint32_t third = relay_direct_anchor.following_start;
	uint32_t fourth = relay_direct_anchor.third_start;
	assert(second == origin + 292 && third == second + 292 && fourth == third + 308);
	/* Two lost control frames still leave the exact direct schedule, including the
	 * two-slot maintenance bump in the interval between them. */
	for (unsigned start_index = 0; start_index < 3; start_index++) {
		uint32_t start = start_index == 0 ? second : start_index == 1 ? third : fourth;
		set_ticks(start + 2);
		assert(tdma_wait_for_slot(17) && tdma_slot_tx_valid(17));
	}
	uint32_t peer_start;
	assert(tdma_slot_data_window(9, fourth - 1U, &peer_start));
	assert(peer_start == fourth + 9U * 16U);
	/* No fourth future start may be invented after three lost control frames. */
	assert(!tdma_slot_data_window(9, fourth + 9U * 16U, &peer_start));
	uint32_t cutoff = fourth + 256;
	set_ticks(cutoff);
	assert(!tdma_wait_for_slot(17));
	bool control_frame;
	uint32_t target;
	assert(tdma_stream_next(&target, &control_frame) && control_frame && target == now_ticks());
	/* The control frame reacquisition restores two physical repair slots despite the R1 pack. */
	c.frame = 9;
	c.preamble_ticks = fourth + 262;
	c.repair_count = 2;
	relay_control_set_next_repairs(&c, 1);
	relay_control_set_next2_repairs(&c, 1);
	tdma_stream_control(&c);
	set_ticks(fourth + 18 * 16 + RELAY_CONTROL_ARM_GUARD_TICKS + 2);
	assert(tdma_wait_for_repair_slot(18, 72) && tdma_repair_tx_valid(18, 72));
	assert(!tdma_wait_for_repair_slot(19, 72));
	puts("PASS variable R: two missed control frames, third-miss pose fence, acquisition scan and maintenance admission");
}

static void repeat_geometry(void)
{
 for(unsigned mode=0;mode<=2;mode++) {
  configure(2,3);set_ticks(0);
  assert(tdma_stage_config(2,relay_geometry_code(11,3),16,8,1000));
  set_ticks(1000);assert(tdma_profile_get()==1);
  struct relay_control c={.frame=1,.preamble_ticks=1182,.direct_count=11,
   .repair_count=3,.slot_ticks=16,.tps_max=120,.control_repeat_mode=mode};
  relay_control_set_next_repairs(&c,3);relay_control_set_next2_repairs(&c,3);
  tdma_stream_control(&c);
  struct tdma_config cfg;assert(tdma_config_snapshot(&cfg));
  unsigned first=12+(mode==1?2:0),total=15+relay_control_repeat_slots(mode);
  assert(RELAY_REPAIR_FIRST_SLOT==first);
  assert(relay_direct_anchor.next_start-relay_direct_anchor.start>=total*16+4U*(1U+(mode!=0)));
  for(unsigned slot=first;slot<first+3;slot++) {
   uint32_t at=relay_direct_anchor.start+relay_slot_offset(slot)+2;
   set_ticks(at-4);assert(tdma_wait_for_repair_slot(slot,72));
   assert(tdma_repair_tx_valid(slot,72));
  }
  assert(cfg.total_slots==total);
  assert(!tdma_repair_tx_valid(total,72));
 }
 puts("PASS repeat geometry: all repair grants fit within frame for modes 0/1/2");
}

static void session_retirement(void)
{
	struct tdma_layout layout;
	assert(tdma_update_config(2, 8, 16, 19));
	set_ticks(1000);
	uint8_t private = relay_geometry_code(11, 5);
	assert(tdma_stage_config(2, private, 16, 20, 1100));
	tdma_session_reset();
	set_ticks(1100);
	assert(!tdma_profile_get() && !tdma_profile_staged());
	assert(!tdma_layout_get(&layout));
	assert(TDMA_UNPACK_EPOCH(atomic_get(&tdma_cfg_pack)) == 19); /* Native tuple kept disabled. */

	assert(tdma_stage_config(2, private, 16, 20, 1200));
	set_ticks(1200);
	assert(tdma_profile_get() && tdma_layout_get(&layout));
	struct relay_control control = {
		.frame = 1, .preamble_ticks = 1368, .direct_count = 11,
		.repair_count = 5, .slot_ticks = 16, .tps_max = 120,
	};
	tdma_stream_control(&control);
	assert(relay_direct_anchor.valid && relay_timeline_active.valid);

	/* A transient inhibit can leave the same network's profile and stage
	 * in place; a terminal reset must not let either outlive the session. */
	assert(tdma_stage_config(2, private, 16, 21, 1500));
	tdma_set_enabled(false);
	assert(tdma_profile_get() && tdma_profile_staged() && !tdma_layout_get(&layout));
	tdma_session_reset();
	assert(!relay_direct_anchor.valid && !relay_timeline_active.valid);
	assert(!tdma_window_quiet(1496, 8));
	set_ticks(1600);
	assert(!tdma_profile_get() && !tdma_profile_staged() && !tdma_layout_get(&layout));
	assert(!tdma_layout_withdrawn());

	assert(tdma_update_config(6, 8, 22, 20)); /* Reused epoch, new network. */
	assert(tdma_layout_get(&layout) && !layout.profile && layout.epoch == 20 && layout.slot_index == 6);
	puts("PASS terminal session retirement clears private geometry, pending activation and grants");
}

int main(void)
{
	(void)relay_geometry_make(&tdma_active_geometry, 11, 5, 16, 120);
	repeat_geometry();
	coarse_sleep_entry_delay();
	frame_start_exact();
	stream_profile();
	layout_full16();
	stream_d0_boundary();
	variable_repair_timeline();
	stream_only_admission();
	data_windows();
	ping_windows();
	final_admission_check();
	staged_deadline_race();
	staged_activation();
	staged_no_mixed_layouts();
	session_retirement();
	return 0;
}
