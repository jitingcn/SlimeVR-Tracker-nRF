#include "test_runtime.h"

static void invalid_owner_and_fault_inputs(void)
{
	host_reset(LED_CAP_MONO_GPIO);
	struct led_token live = host_begin(LED_OWNER_ACC, LED_WAIT_STILL);
	host_step(0);
	assert(led_request_event(LED_OWNER_SYSTEM, 1, 1, LED_INPUT_ACK) == LED_ADMITTED);
	struct led_engine before;
	memcpy(&before, &engine, sizeof(before));
	led_changed.count = 0;
	const enum led_owner invalid[] = {(enum led_owner)-1, LED_OWNER_COUNT};
	for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
		enum led_owner owner = invalid[i];
		assert(led_begin(owner, 123).session == 0);
		struct led_token token = live;
		token.owner = owner;
		assert(led_state(token, 2, LED_PROCESSING) == LED_INVALID);
		assert(led_result(token, 123, LED_FAILED) == LED_INVALID);
		assert(led_request_event(owner, 123, 123, LED_FAILED) == LED_INVALID);
		led_fault_publish(owner, LED_FAULT_SAFETY, 123);
		led_maintenance_publish(owner, true);
		led_operation_publish(owner, true, true);
		led_engine_fault(&engine, owner, LED_FAULT_SAFETY, 123, 0);
	}
	const enum led_fault_kind faults[] = {(enum led_fault_kind)-1, (enum led_fault_kind)(LED_FAULT_SAFETY + 1)};
	for (unsigned i = 0; i < sizeof(faults) / sizeof(faults[0]); ++i) {
		led_fault_publish(LED_OWNER_ACC, faults[i], 123);
		led_engine_fault(&engine, LED_OWNER_ACC, faults[i], 123, 0);
	}
	assert(memcmp(&before, &engine, sizeof(before)) == 0);
	assert(led_changed.count == 0);
	host_step(100);
	assert(engine.winner.semantic == LED_INPUT_ACK && host_value == 0);
}

static void credited_black_does_not_restart_unlit_event(void)
{
	host_reset(LED_CAP_MONO_GPIO);
	host_step(0);
	host_now_ms = 1000;
	assert(led_request_event(LED_OWNER_SYSTEM, 1, 1, LED_SUCCESS) == LED_ADMITTED);
	/* Select without a driver receipt: a delayed worker must retain the
	 * zero-gap origin rather than restart an unilluminated finite envelope. */
	struct led_selection first = led_engine_select(&engine, 1000);
	struct led_selection delayed = led_engine_select(&engine, 1050);
	assert(first.semantic == LED_SUCCESS && first.origin_ms == 1000);
	assert(delayed.identity == first.identity && delayed.origin_ms == first.origin_ms);
	host_step(1050);
	assert(host_value == 10000);
	host_step(2999);
	assert(engine.winner.semantic == LED_SUCCESS && host_value == 0);
	host_step(3000);
	assert(engine.winner.semantic == LED_NONE && host_value == 0);
}

static void timeline_boundaries(void)
{
	assert(led_timeline(LED_STYLE_BREATHE,0,false,false).value_pptt==0);
	assert(led_timeline(LED_STYLE_BREATHE,1000,false,false).value_pptt==7000);
	assert(led_timeline(LED_STYLE_BREATHE,2000,false,false).value_pptt==10000);
	assert(led_timeline(LED_STYLE_BREATHE,3999,false,false).value_pptt<20);
	assert(led_timeline(LED_STYLE_BREATHE,4000,false,false).value_pptt==0);
	assert(led_timeline(LED_STYLE_BREATHE,3999,true,false).value_pptt==10000);
	assert(led_timeline(LED_STYLE_BREATHE,4000,true,false).value_pptt==0);
	assert(led_timeline(LED_STYLE_BREATHE,5000,false,false).value_pptt==0);
	assert(led_timeline(LED_STYLE_BREATHE,5100,false,false).value_pptt
		==led_timeline(LED_STYLE_BREATHE,100,false,false).value_pptt);
	static const struct { uint32_t age; uint16_t value; } breathing_anchors[] = {
		{0,0}, {800,6000}, {1200,8000}, {1600,9500}, {2000,10000}
	};
	for (unsigned i=0;i<sizeof(breathing_anchors)/sizeof(breathing_anchors[0]);++i) {
		assert(led_timeline(LED_STYLE_BREATHE,breathing_anchors[i].age,false,false).value_pptt
			==breathing_anchors[i].value);
		assert(led_timeline(LED_STYLE_BREATHE,4000-breathing_anchors[i].age,false,false).value_pptt
			==breathing_anchors[i].value);
	}
	uint16_t up=0,down=10000;
	for (uint32_t time=0;time<2000;time++) {
		struct led_envelope rising=led_timeline(LED_STYLE_BREATHE,time,false,false);
		struct led_envelope falling=led_timeline(LED_STYLE_BREATHE,2000+time,false,false);
		assert(rising.fading && falling.fading && rising.next_ms<=LED_FRAME_MS && falling.next_ms<=LED_FRAME_MS);
		assert(rising.value_pptt>=up && falling.value_pptt<=down);
		assert(falling.value_pptt==led_timeline(LED_STYLE_BREATHE,2000-time,false,false).value_pptt);
		up=rising.value_pptt; down=falling.value_pptt;
	}
	assert(led_timeline(LED_STYLE_EXIT,300,false,true).value_pptt==5000);
	assert(led_timeline(LED_STYLE_EXIT,599,true,true).value_pptt==10000);
	assert(led_timeline(LED_STYLE_EXIT,600,true,true).value_pptt==0);
	assert(led_timeline(LED_STYLE_EXIT,800,true,true).complete);
	assert(led_timeline(LED_STYLE_MOVE,0,false,false).value_pptt==10000);
	assert(led_timeline(LED_STYLE_MOVE,200,false,false).value_pptt==0);
	assert(led_timeline(LED_STYLE_MOVE,400,false,false).value_pptt==10000);
	assert(led_timeline(LED_STYLE_MOVE,600,false,false).value_pptt==0);
	assert(led_timeline(LED_STYLE_MOVE,2000,false,false).value_pptt==10000);
	static const struct { uint32_t age; uint16_t value; } hold_cases[] = {
		{0,10000}, {999,10000}, {1000,0}, {1249,0}, {1250,10000},
		{1750,5000}, {2249,10}, {2250,0}, {2499,0}, {2500,10000},
		{2999,10000}, {3000,0}, {3499,0}, {3500,10000}
	};
	for (unsigned i=0;i<sizeof(hold_cases)/sizeof(hold_cases[0]);++i) {
		struct led_envelope sample=led_timeline(LED_STYLE_BUTTON_HOLD,hold_cases[i].age,false,false);
		assert(sample.value_pptt==hold_cases[i].value);
		assert(sample.fading==(hold_cases[i].age>=1250 && hold_cases[i].age<2250));
	}
	assert(led_timeline(LED_STYLE_MANUAL_EXIT,0,false,true).value_pptt==0);
	assert(led_timeline(LED_STYLE_MANUAL_EXIT,249,false,true).value_pptt==0);
	assert(led_timeline(LED_STYLE_MANUAL_EXIT,250,false,true).value_pptt==10000);
	assert(led_timeline(LED_STYLE_MANUAL_EXIT,750,false,true).value_pptt==5000);
	assert(led_timeline(LED_STYLE_MANUAL_EXIT,1249,false,true).value_pptt==10);
	assert(led_timeline(LED_STYLE_MANUAL_EXIT,1250,false,true).value_pptt==0);
	assert(led_timeline(LED_STYLE_MANUAL_EXIT,1500,false,true).complete);
	assert(led_timeline(LED_STYLE_MANUAL_EXIT,1800,false,false).value_pptt==0);
}
/* Observe complete short/success/double glyphs through the actual worker on mono
 * and RGB. Interior gaps and the final actual-black tail remain mandatory. */
static void finite_glyph_recognition(void)
{
	static const struct {
		enum led_semantic semantic;
		unsigned pulses, on_ms, duration_ms;
	} cases[] = {
		{LED_INPUT_ACK,1,200,800}, {LED_ACCEPTED,1,200,800},
		{LED_STAGE_ACK,1,200,800}, {LED_CANCELLED,1,200,800},
		{LED_SUCCESS,4,200,2000}, {LED_REJECTED,2,200,1200},
		{LED_FAILED,2,200,1200}, {LED_PARTIAL,2,200,1200},
		{LED_APPLIED_NOT_SAVED,2,200,1200}
	};
	const enum led_capability capabilities[] = {LED_CAP_MONO_GPIO, LED_CAP_RGB_PWM};
	for (unsigned cap=0;cap<2;++cap) {
		for (unsigned family=0;family<sizeof(cases)/sizeof(cases[0]);++family) {
			host_reset(capabilities[cap]);
			assert(led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),cases[family].semantic)==LED_ADMITTED);
			uint32_t end=600+cases[family].duration_ms;
			unsigned pulses=0;
			bool previous=false;
			uint32_t started=0,last_end=0;
			for (uint32_t time=0;time<=end;++time) {
				host_step(time);
				bool lit=host_value!=0;
				if (lit && !previous) {
					assert(pulses<cases[family].pulses);
					if (pulses) assert(time-last_end==200);
					else assert(time==600);
					started=time;
				}
				if (!lit && previous) {
					assert(time-started==cases[family].on_ms);
					++pulses;
					last_end=time;
				}
				previous=lit;
				if (time>=600 && time<end) assert(engine.winner.semantic==cases[family].semantic);
			}
			assert(pulses==cases[family].pulses && end-last_end==600);
			assert(engine.winner.semantic!=cases[family].semantic && !host_value);
			for (uint32_t time=end+1;time<=end+4000;++time) {
				host_step(time);
				assert(engine.winner.semantic!=cases[family].semantic && !host_value);
			}
		}
	}
}

static void persistent_fault_and_link_glyphs(void)
{
	const enum led_fault_kind faults[] = {
		LED_FAULT_SENSOR_MISSING, LED_FAULT_SENSOR, LED_FAULT_SYSTEM, LED_FAULT_SAFETY
	};
	const enum led_semantic semantics[] = {
		LED_SENSOR_MISSING, LED_SENSOR_FAULT, LED_BLOCKING_FAULT, LED_SAFETY_FAULT
	};
	const enum led_capability capabilities[] = {LED_CAP_MONO_GPIO, LED_CAP_RGB_PWM};
	for (unsigned cap=0;cap<2;++cap) {
		for (unsigned family=0;family<4;++family) {
			host_reset(capabilities[cap]);
			led_fault_publish(LED_OWNER_SENSOR,faults[family],0);
			for (uint32_t time=0;time<10000;++time) {
				host_step(time);
				assert(engine.winner.semantic==semantics[family]);
				uint32_t phase=time%5000;
				bool on=phase<800 || (phase>=1000 && phase<1800);
				assert(host_value==(on ? 10000 : 0));
			}
		}
		for (unsigned link=0;link<3;++link) {
			host_reset(capabilities[cap]);
			struct led_connection_facts facts={
				.radio_required=true,.paired=link==1,.pairing=link==2
			};
			led_connection_publish(&facts);
			enum led_semantic semantic=link==0 ? LED_UNPAIRED_IDLE : link==1 ? LED_RECONNECTING : LED_PAIRING;
			for (uint32_t time=0;time<4000;++time) {
				host_step(time);
				assert(engine.winner.semantic==semantic);
				assert(host_value==(time%2000<200 ? 10000 : 0));
			}
		}
	}
}

static void hold_ack_black_gap(void)
{
	host_reset(LED_CAP_MONO_GPIO);
	struct led_token token=host_begin(LED_OWNER_ACC,LED_WAIT_STILL);
	host_step(0); assert(host_value==10000);
	host_now_ms=50;
	assert(led_result(token,led_event_id(),LED_STAGE_ACK)==LED_ADMITTED);
	assert(host_step(50)==600); assert(host_value==0);
	host_step(649); assert(host_value==0);
	host_step(650); assert(host_value==10000);
	uint32_t end=engine.winner.origin_ms+led_behavior_get(LED_STAGE_ACK)->duration_ms;
	/* A new owner phase cannot clip the pulse or the finite black tail. */
	host_now_ms=700; assert(led_state(token,2,LED_WAIT_MOVE)==LED_ADMITTED);
	for (uint32_t time=701;time<end;++time) {
		host_step(time); assert(engine.winner.semantic==LED_STAGE_ACK);
	}
	host_step(end); assert(engine.winner.semantic==LED_WAIT_MOVE);
}
static void identities_and_truthful_terminal(void)
{
	host_reset(LED_CAP_MONO_GPIO);
	struct led_token old=host_begin(LED_OWNER_IMU,LED_WAIT_STILL);
	host_step(0);
	uint32_t id=led_event_id();
	assert(led_result(old,id,LED_ACCEPTED)==LED_ADMITTED);
	assert(led_result(old,id,LED_ACCEPTED)==LED_DUPLICATE);
	struct led_token newer=host_begin(LED_OWNER_IMU,LED_PROCESSING);
	assert(led_result(old,led_event_id(),LED_SUCCESS)==LED_STALE);
	assert(led_state(old,2,LED_NONE)==LED_STALE);
	host_step(100); assert(engine.winner.semantic==LED_PROCESSING);
	uint32_t origin=engine.winner.origin_ms;
	assert(led_state(newer,2,LED_PROCESSING)==LED_ADMITTED);
	host_step(200); assert(engine.winner.origin_ms==origin);
	assert(led_result(newer,led_event_id(),LED_FAILED)==LED_ADMITTED);
	assert(led_result(newer,led_event_id(),LED_SUCCESS)==LED_CONFLICT);
	host_step(200); assert(engine.winner.semantic==LED_FAILED && host_value==0);
	host_step(800); assert(host_role==LED_ROLE_NEGATIVE && host_value==10000);
	assert(engine.owners[LED_OWNER_IMU].semantic==LED_NONE);
}
static void expired_success_and_interrupted_result(void)
{
	host_reset(LED_CAP_MONO_GPIO);
	struct led_token token=host_begin(LED_OWNER_ACC,LED_PROCESSING);
	assert(led_result(token,led_event_id(),LED_SUCCESS)==LED_ADMITTED);
	led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_SENSOR,0);
	host_step(0); assert(engine.winner.semantic==LED_SENSOR_FAULT);
	const struct led_behavior *success=led_behavior_get(LED_SUCCESS);
	uint32_t too_late=success->ttl_ms-success->duration_ms+1;
	host_step(too_late);
	led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_NONE,0);
	host_step(too_late); assert(engine.winner.semantic!=LED_SUCCESS);
	host_step(success->ttl_ms+1); assert(engine.winner.semantic!=LED_SUCCESS);
	host_reset(LED_CAP_MONO_GPIO); host_step(0);
	uint32_t request=led_request_id();
	led_request_event(LED_OWNER_SYSTEM,request,led_event_id(),LED_FAILED);
	host_step(200); assert(engine.winner.semantic==LED_FAILED && !host_value);
	host_step(600); assert(host_value==10000);
	led_power_publish(LED_POWER_BATTERY,true);
	host_step(700); assert(engine.winner.semantic==LED_LOW_BATTERY);
	host_step(2700); assert(engine.winner.semantic!=LED_FAILED);
}
static void low_guarantee_and_admission(void)
{
	enum led_semantic backgrounds[]={LED_OTA_ACTIVE,LED_WAIT_STILL,LED_PROCESSING};
	for (size_t i=0;i<sizeof(backgrounds)/sizeof(backgrounds[0]);++i) {
		host_reset(LED_CAP_MONO_GPIO); host_begin(LED_OWNER_RADIO,backgrounds[i]);
		led_power_publish(LED_POWER_BATTERY,true);
		host_step(0); assert(engine.winner.semantic==LED_LOW_BATTERY && host_value==10000);
		for (uint32_t time=100;time<10000;time+=100) {
			host_now_ms=time; led_identify();
			led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_INPUT_ACK);
			host_step(time);
			if (time<2000) { assert(engine.winner.semantic==LED_LOW_BATTERY); }
		}
		host_step(10000); assert(engine.winner.semantic==LED_LOW_BATTERY);
	}
	host_reset(LED_CAP_MONO_GPIO); host_begin(LED_OWNER_ACC,LED_WAIT_STILL);
	led_power_publish(LED_POWER_BATTERY,true); host_step(0); host_step(2000);
	/* One ms too late to fit the start gap plus whole double glyph before LOW.
	 * Its TTL still allows the full receipt after the two-second LOW window. */
	host_now_ms=10000-led_behavior_get(LED_FAILED)->duration_ms-LED_FEEDBACK_START_GAP_MS+1;
	led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_FAILED);
	host_step(host_now_ms); assert(engine.winner.semantic==LED_WAIT_STILL);
	host_step(10000); assert(engine.winner.semantic==LED_LOW_BATTERY);
	host_step(10350); assert(host_value==0);
	host_step(12000); assert(engine.winner.semantic==LED_FAILED && host_value==10000);
	/* True low clear / actual charging cancels an admitted warning immediately. */
	led_power_publish(LED_POWER_BATTERY,false); host_step(12010);
	led_power_publish(LED_POWER_BATTERY,true); host_step(12020);
	assert(engine.winner.semantic==LED_LOW_BATTERY);
	led_power_publish(LED_POWER_CHARGING,true); host_step(12021);
	assert(engine.winner.semantic!=LED_LOW_BATTERY && !engine.low);
	/* Exact fit is allowed: the complete black tail ends at the next LOW
	 * boundary, rather than being clipped or needlessly postponed. */
	host_reset(LED_CAP_MONO_GPIO); host_begin(LED_OWNER_ACC,LED_WAIT_STILL);
	led_power_publish(LED_POWER_BATTERY,true); host_step(0); host_step(2000);
	host_now_ms=10000-led_behavior_get(LED_FAILED)->duration_ms-LED_FEEDBACK_START_GAP_MS;
	led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_FAILED);
	host_step(host_now_ms); assert(engine.winner.semantic==LED_FAILED && !host_value);
	uint32_t origin=engine.winner.origin_ms;
	for (uint32_t phase=0;phase<1200;++phase) {
		host_step(origin+phase);
		assert(engine.winner.semantic==LED_FAILED);
		assert(host_value==(phase<200 || (phase>=400 && phase<600) ? 10000 : 0));
	}
	host_step(10000); assert(engine.winner.semantic==LED_LOW_BATTERY && host_value==10000);
}
static void safety_record_and_reject_preserves_ota(void)
{
	host_reset(LED_CAP_MONO_GPIO);
	struct led_token ota=host_begin(LED_OWNER_RADIO,LED_OTA_ACTIVE);
	led_operation_publish(LED_OWNER_RADIO,true,false);
	host_step(0);
	host_now_ms=50;
	led_request_event(LED_OWNER_RADIO,led_request_id(),led_event_id(),LED_REJECTED);
	host_step(50); assert(engine.owners[LED_OWNER_RADIO].session==ota.session);
	host_step(650); assert(engine.winner.semantic==LED_REJECTED && host_value==10000);
	uint32_t refusal_end=engine.winner.origin_ms+led_behavior_get(LED_REJECTED)->duration_ms;
	for (uint32_t time=651;time<refusal_end;++time) {
		host_step(time); assert(engine.winner.semantic==LED_REJECTED);
		assert(engine.owners[LED_OWNER_RADIO].session==ota.session);
	}
	host_step(refusal_end); assert(engine.winner.semantic==LED_OTA_ACTIVE);
	uint32_t first_record=refusal_end+30;
	led_fault_publish(LED_OWNER_TCAL,LED_FAULT_NONE,7);
	host_step(first_record); assert(engine.winner.semantic==LED_SAFETY_FAULT);
	uint32_t record_until=engine.owners[LED_OWNER_TCAL].protection_expires_ms;
	host_now_ms=first_record+(record_until-first_record)/2;
	led_fault_publish(LED_OWNER_TCAL,LED_FAULT_NONE,7);
	host_step(host_now_ms); assert(engine.winner.semantic==LED_SAFETY_FAULT);
	assert(engine.owners[LED_OWNER_TCAL].protection_expires_ms==record_until);
	host_step(record_until); assert(engine.winner.semantic==LED_OTA_ACTIVE);
	host_now_ms=record_until+100; led_fault_publish(LED_OWNER_TCAL,LED_FAULT_SAFETY,8);
	uint32_t latched_until=engine.owners[LED_OWNER_TCAL].protection_expires_ms;
	host_step(latched_until+1); assert(engine.winner.semantic==LED_SAFETY_FAULT);
	led_fault_publish(LED_OWNER_TCAL,LED_FAULT_NONE,8);
	host_step(host_now_ms+1); assert(engine.winner.semantic==LED_OTA_ACTIVE);
}
static void immediate_ready_and_blockers(void)
{
	host_reset(LED_CAP_MONO_GPIO); host_step(0);
	struct led_connection_facts facts={.radio_required=true,.healthy=true,.output_ready=false,.paired=true};
	led_connection_publish(&facts); host_step(200);
	assert(!engine.ready && engine.winner.semantic!=LED_READY);
	facts.output_ready=true; led_connection_publish(&facts);
	host_step(300); assert(engine.winner.semantic==LED_READY && !host_value);
	host_step(10000); assert(engine.winner.semantic==LED_READY && host_value==10000);
	host_step(10199); assert(engine.winner.semantic==LED_READY && host_value==10000);
	host_step(10201); assert(engine.winner.semantic==LED_READY && !host_value);
	facts.healthy=false; led_connection_publish(&facts); host_step(10300);
	assert(engine.winner.semantic==LED_RECONNECTING);
	facts.healthy=true; led_connection_publish(&facts); host_step(10301);
	assert(engine.winner.semantic==LED_READY && !host_value); /* No entry cue or recovery debounce. */
	/* LOW blocks readiness without allowing stale health to survive its clear. */
	host_reset(LED_CAP_MONO_GPIO);
	led_power_publish(LED_POWER_BATTERY,true); led_connection_publish(&facts);
	host_step(0); assert(engine.winner.semantic==LED_LOW_BATTERY);
	facts.healthy=false; host_now_ms=1000; led_connection_publish(&facts);
	host_step(2000); assert(engine.winner.semantic==LED_RECONNECTING);
	/* Healthy PONG and live fusion are insufficient while OTA blocks output. */
	host_reset(LED_CAP_MONO_GPIO); facts.healthy=true; led_connection_publish(&facts);
	led_operation_publish(LED_OWNER_RADIO,true,false);
	assert(!engine.ready);
	host_step(0); assert(engine.winner.semantic==LED_OTA_ACTIVE);
	host_now_ms=1500; led_operation_publish(LED_OWNER_RADIO,false,false);
	host_step(1500); assert(engine.winner.semantic==LED_READY && !host_value);
	/* Protection expiry exposes the ordinary heartbeat, never a fallback. */
	host_reset(LED_CAP_MONO_GPIO); led_connection_publish(&facts);
	led_fault_publish(LED_OWNER_TCAL,LED_FAULT_NONE,1);
	host_step(0); assert(engine.winner.semantic==LED_SAFETY_FAULT);
	host_step(6000); assert(engine.winner.semantic==LED_READY && !host_value);
	host_step(10000); assert(engine.winner.semantic==LED_READY && host_value==10000);
}
static void external_power_priority_and_identify(void)
{
	const enum led_power_state powers[]={LED_POWER_CHARGING,LED_POWER_CHARGED,LED_POWER_EXTERNAL_UNKNOWN};
	const enum led_semantic semantics[]={LED_CHARGING,LED_CHARGED,LED_EXTERNAL_POWER_UNKNOWN};
	const enum led_role roles[]={LED_ROLE_POWER,LED_ROLE_POSITIVE,LED_ROLE_NEUTRAL};
	for (unsigned power=0;power<sizeof(powers)/sizeof(powers[0]);++power) {
		for (unsigned link=0;link<4;++link) {
			host_reset(LED_CAP_RGB_PWM);
			struct led_connection_facts facts={
				.radio_required=true,.paired=link!=0,.pairing=link==2,
				.healthy=link==3,.output_ready=link==3
			};
			led_connection_publish(&facts); led_power_publish(powers[power],false);
			enum led_semantic semantic=semantics[power];
			host_step(0);
			for (uint32_t time=1;time<=12000;++time) {
				host_step(time);
				assert(engine.winner.semantic==semantic);
				if (host_value) assert(host_role==roles[power]);
			}
			assert(engine.ready==(link==3)); /* Power presentation is not link truth. */
			struct led_token hold=host_begin(LED_OWNER_ACC,LED_WAIT_STILL); host_step(12001);
			assert(engine.winner.semantic==LED_WAIT_STILL && host_value==10000);
			host_now_ms=13000; led_state(hold,2,LED_NONE); host_step(13000);
			assert(engine.winner.semantic==semantic);
			led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_SENSOR,0); host_step(13001);
			assert(engine.winner.semantic==LED_SENSOR_FAULT);
			led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_NONE,0); host_step(13002);
			assert(engine.winner.semantic==semantic);
			led_operation_publish(LED_OWNER_RADIO,true,false); host_step(13003);
			assert(engine.winner.semantic==LED_OTA_ACTIVE);
			led_operation_publish(LED_OWNER_RADIO,false,false); host_step(13004);
			assert(engine.winner.semantic==semantic);
			struct led_token heated=host_begin(LED_OWNER_TCAL,LED_HEATED_ACTIVE);
			led_operation_publish(LED_OWNER_TCAL,false,true); host_step(13005);
			assert(engine.winner.semantic==LED_HEATED_ACTIVE);
			host_now_ms=13006; led_state(heated,2,LED_NONE);
			led_operation_publish(LED_OWNER_TCAL,false,false); host_step(13006);
			assert(engine.winner.semantic==semantic);
			led_power_publish(LED_POWER_BATTERY,false); host_step(13007);
			assert(engine.winner.semantic==(link==0 ? LED_UNPAIRED_IDLE : link==1 ? LED_RECONNECTING
				: link==2 ? LED_PAIRING : LED_READY));
		}
	}
	host_reset(LED_CAP_MONO_GPIO); led_identify(); host_step(0);
	host_now_ms=1000; led_identify(); host_step(1000);
	led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_SENSOR,0); host_step(1200);
	host_now_ms=1600; led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_NONE,0); host_step(1600);
	assert(engine.winner.semantic==LED_IDENTIFY && engine.winner.origin_ms==0 && !host_value);
	host_step(6000); assert(engine.winner.semantic!=LED_IDENTIFY);
}
static void foreground_order_exit_and_quiesce(void)
{
	host_reset(LED_CAP_MONO_GPIO);
	struct led_token imu=host_begin(LED_OWNER_IMU,LED_WAIT_MOVE);
	struct led_token acc=host_begin(LED_OWNER_ACC,LED_WAIT_STILL);
	struct led_token ota=host_begin(LED_OWNER_RADIO,LED_OTA_ACTIVE);
	struct led_token exit=host_begin(LED_OWNER_SYSTEM,LED_EXIT_PENDING);
	host_step(0); assert(engine.winner.semantic==LED_EXIT_PENDING);
	led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_SENSOR,0); host_step(100);
	host_now_ms=900; led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_NONE,0); host_step(900);
	assert(engine.winner.semantic==LED_EXIT_PENDING && host_value==0 && engine.winner.origin_ms==0);
	led_state(exit,2,LED_NONE); host_step(901); assert(engine.winner.semantic==LED_OTA_ACTIVE);
	led_state(ota,2,LED_NONE); host_step(902); assert(engine.winner.owner==LED_OWNER_IMU);
	led_state(imu,2,LED_NONE); host_step(903); assert(engine.winner.owner==LED_OWNER_ACC);
	(void)acc;
	host_write_failure=true; led_quiesce(); host_step(904);
	assert(engine.winner.priority==LED_PRIORITY_SHUTDOWN && host_offs>0 && hardware_quiesced);
	assert(led_state(acc,2,LED_WAIT_MOVE)==LED_SHUTDOWN);
	assert(led_result(acc,led_event_id(),LED_SUCCESS)==LED_SHUTDOWN);
	host_step(1000); assert(host_value==0);
}
static void request_flood_tie_break_and_wrap(void)
{
	host_reset(LED_CAP_MONO_GPIO); host_step(0); host_now_ms=200;
	for (int owner=LED_OWNER_SYSTEM;owner>=LED_OWNER_IMU;--owner) {
		led_request_event(owner,led_request_id(),led_event_id(),LED_SUCCESS);
	}
	host_step(200); assert(engine.winner.owner==LED_OWNER_IMU);
	uint32_t first_end=engine.winner.origin_ms+led_behavior_get(LED_SUCCESS)->duration_ms;
	for (uint32_t time=201;time<first_end;++time) {
		host_step(time); assert(engine.winner.owner==LED_OWNER_IMU);
	}
	/* A second full success group no longer fits these same-time receipts'
	 * original TTL. Drop them rather than emit a truncated/replayed group. */
	host_step(first_end);
	assert(engine.winner.semantic==LED_NONE && !host_value);
	host_step(4201); assert(engine.winner.semantic==LED_NONE && !host_value);
	for (uint32_t i=0;i<10000;++i) {
		led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_INPUT_ACK);
	}
	led_power_publish(LED_POWER_BATTERY,true); host_step(4301);
	assert(engine.winner.semantic==LED_LOW_BATTERY);
	host_reset(LED_CAP_MONO_GPIO); host_now_ms=UINT32_MAX-500;
	led_power_publish(LED_POWER_BATTERY,true); host_step(UINT32_MAX-500);
	host_step(1499); assert(engine.winner.semantic!=LED_LOW_BATTERY);
	host_step(9499); assert(engine.winner.semantic==LED_LOW_BATTERY);
	assert(led_sync_kernel_ticks((uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC*7,CONFIG_SYS_CLOCK_TICKS_PER_SEC)==7*LED_SYNC_HZ);
	assert(led_sync_phase_ms(UINT32_MAX,10000)!=led_sync_phase_ms(0,10000));
	assert(led_sync_wrap_ms(UINT32_MAX)==1);
}
static void invisible_effects_and_driver_black_failure(void)
{
	host_reset(LED_CAP_NO_LED);
	led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_SUCCESS);
	assert(host_step(0)==UINT32_MAX && host_writes==0 && host_offs==1);
	host_reset(LED_CAP_MONO_GPIO); host_hardware.global_limit_pptt=0;
	host_begin(LED_OWNER_ACC,LED_WAIT_STILL);
	assert(host_step(0)==UINT32_MAX && host_writes==0 && host_value==0);
	host_reset(LED_CAP_MONO_GPIO); host_begin(LED_OWNER_ACC,LED_WAIT_STILL); host_step(0);
	led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_SUCCESS);
	host_write_failure=true; host_step(50); host_step(170);
	assert(!engine.black_known && host_value==10000);
	host_write_failure=false; host_step(190); assert(host_value==0);
	host_step(789); assert(host_value==0);
	host_step(790); assert(host_value==10000 && host_role==LED_ROLE_POSITIVE);
}
static void same_class_pending_and_input_merge(void)
{
	host_reset(LED_CAP_MONO_GPIO); host_step(0); host_now_ms=200;
	led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_FAILED);
	host_step(200); assert(host_value==0);
	uint32_t active=engine.winner.identity;
	host_now_ms=300;
	led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_FAILED);
	host_step(300); assert(engine.winner.identity==active && host_value==0);
	uint32_t active_end=engine.winner.origin_ms+led_behavior_get(LED_FAILED)->duration_ms;
	for (uint32_t time=301;time<active_end;++time) {
		host_step(time); assert(engine.winner.identity==active);
	}
	host_step(active_end); assert(engine.winner.identity!=active && host_value==10000);
	host_reset(LED_CAP_MONO_GPIO); host_step(0); host_now_ms=200;
	uint32_t request=led_request_id();
	led_request_event(LED_OWNER_SYSTEM,request,led_event_id(),LED_INPUT_ACK);
	struct led_token token=led_begin(LED_OWNER_SYSTEM,request);
	assert(led_result(token,led_event_id(),LED_ACCEPTED)==LED_DUPLICATE);
	assert(led_begin(LED_OWNER_SYSTEM,request).session==token.session);
	host_step(200); assert(engine.winner.semantic==LED_INPUT_ACK && host_value==0);
	host_step(400); assert(engine.winner.semantic!=LED_ACCEPTED);
	assert(led_result(token,led_event_id(),LED_CANCELLED)==LED_ADMITTED);
	assert(led_begin(LED_OWNER_SYSTEM,request).session==token.session);
	assert(led_state(token,1,LED_MAINTENANCE)==LED_STALE);
	host_reset(LED_CAP_MONO_GPIO);
	struct led_connection_facts facts={.radio_required=true};
	led_connection_publish(&facts);
	struct led_token init=host_begin(LED_OWNER_SENSOR,LED_INITIALIZING);
	host_step(0);
	const struct led_behavior *visible=led_behavior_get(engine.winner.semantic);
	assert((!visible || visible->style!=LED_STYLE_BREATHE) && !engine.ready);
	host_step(299);
	visible=led_behavior_get(engine.winner.semantic);
	assert((!visible || visible->style!=LED_STYLE_BREATHE) && !engine.ready);
	host_step(300); assert(engine.winner.semantic==LED_INITIALIZING);
	led_state(init,2,LED_NONE); facts.pairing=true; led_connection_publish(&facts);
	host_step(301); assert(engine.winner.semantic==LED_PAIRING);
}
static void reserved_completion_and_ota_lock(void)
{
	host_reset(LED_CAP_MONO_GPIO);
	struct led_token ota=host_begin(LED_OWNER_RADIO,LED_OTA_ACTIVE);
	led_operation_publish(LED_OWNER_RADIO,true,false);
	uint32_t completion_id=led_event_id();
	/* A newer request rejection and input must not make the old immutable
	 * session's reserved completion ID stale, or terminate its OTA lock. */
	uint32_t other=led_request_id();
	assert(led_request_event(LED_OWNER_RADIO,other,led_event_id(),LED_REJECTED)==LED_ADMITTED);
	host_step(0); host_step(600);
	host_now_ms=700;
	assert(led_result(ota,completion_id,LED_FAILED)==LED_ADMITTED);
	uint32_t refusal_end=600+led_behavior_get(LED_REJECTED)->duration_ms;
	for (uint32_t time=701;time<refusal_end;++time) host_step(time);
	host_step(refusal_end);
	assert(engine.winner.semantic==LED_FAILED && host_value==10000);
	host_step(refusal_end+led_behavior_get(LED_FAILED)->duration_ms);
	assert(engine.winner.semantic==LED_OTA_ACTIVE);
	led_operation_publish(LED_OWNER_RADIO,false,false); host_step(host_now_ms+1);
	assert(engine.winner.semantic==LED_NONE);
	/* An unrelated input must not merge away acceptance of a different request. */
	host_reset(LED_CAP_MONO_GPIO);
	led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_INPUT_ACK);
	struct led_token accepted=led_begin(LED_OWNER_SYSTEM,led_request_id());
	assert(led_result(accepted,led_event_id(),LED_ACCEPTED)==LED_ADMITTED);
	host_step(0); host_step(600);
	assert(engine.winner.semantic==LED_ACCEPTED && host_value==10000);
}
static void actual_clock_wrap(void)
{
	host_reset(LED_CAP_MONO_GPIO); host_step(0);
	struct led_connection_facts facts={.radio_required=true,.healthy=true,.output_ready=true};
	led_connection_publish(&facts); host_step(200);
	assert(engine.winner.semantic==LED_READY);
#if CONFIG_LED_NETWORK_SYNC
	host_network_available=true;
	host_network_raw=0;
	host_step(0x80000064U);
	assert(host_value==10000); /* uptime bit31 is not a future origin */
	host_network_raw=0x7fffffffU;
	host_step(0x80000065U);
	uint32_t phase=led_sync_phase_ms(host_network_raw,10000);
	assert(host_value==(phase<200 ? 10000 : 0));
	host_network_raw=0x80000000U;
	host_step(0x80000066U);
	phase=led_sync_phase_ms(host_network_raw,10000);
	assert(host_value==(phase<200 ? 10000 : 0));
	host_network_raw=UINT32_MAX;
	assert(host_step(0x80000067U)<=1);
	host_network_raw=0;
	host_step(0x80000068U); assert(host_value==10000);
	host_network_available=false;
	host_step(0x80000069U); assert(host_value==10000); /* hold readonly offset */
#else
	for (uint32_t delta=0;delta<10000;++delta) {
		uint32_t time=0x80000000U+delta;
		host_now_ms=time;
		uint32_t raw=led_sync_kernel_ticks(k_uptime_ticks(),CONFIG_SYS_CLOCK_TICKS_PER_SEC);
		if (led_sync_phase_ms(raw,10000)<200) {
			host_step(time); assert(host_value==10000); return;
		}
	}
	assert(false);
#endif
}
static void breathing_grid_and_idle_wakes(void)
{
	host_reset(LED_CAP_RGB_PWM);
	struct led_token token=host_begin(LED_OWNER_IMU,LED_PROCESSING);
	/* Continuous tasks start on first visibility, not admission. Present the
	 * initial ramp before injecting later producer wakes into a lit segment. */
	(void)host_step(0);
	assert(host_step(1000)==LED_FRAME_MS && host_value>0);
	for (uint32_t extra=1;extra<LED_FRAME_MS;extra++) {
		host_now_ms=1000+extra;
		assert(led_state(token,1+extra,LED_PROCESSING)==LED_ADMITTED);
		/* Repeated producer wakes cannot defer the same next physical sample. */
		assert(host_step(host_now_ms)+host_now_ms==1005);
	}
	assert(host_step(1005)==LED_FRAME_MS);
	host_reset(LED_CAP_RGB_PWM);
	host_begin(LED_OWNER_IMU,LED_WAIT_STILL);
	assert(host_step(1000)>LED_FRAME_MS); /* Steady HOLD adds no frame wake train. */
	led_quiesce();
	assert(host_step(1001)==UINT32_MAX && host_value==0);
}

static void replace_selected_hold(void)
{
	uint32_t generation=led_button_input();
	assert(led_button_hold(generation,true,host_now_ms)==LED_ADMITTED);
}
static void release_selected_hold(void)
{
	assert(led_button_hold(engine.button_generation,false,0)==LED_ADMITTED);
}
static void physical_hold_and_manual_handoff(void)
{
	host_reset(LED_CAP_RGB_PWM);
	host_begin(LED_OWNER_ACC,LED_WAIT_STILL);
	host_step(0);
	host_now_ms=100;
	uint32_t generation=led_button_input();
	/* Publication is late; the initial full-on belongs to physical origin. */
	host_now_ms=200;
	assert(led_button_hold(generation,true,100)==LED_ADMITTED);
	led_request_event(LED_OWNER_IMU,led_request_id(),led_event_id(),LED_FAILED);
	led_identify();
	host_step(600);
	assert(engine.winner.semantic==LED_BUTTON_HOLD && host_value==10000 && host_level==LED_LEVEL_NOTICE);
	assert(engine.winner.origin_ms==100);
	uint32_t hold_identity=engine.winner.identity;
	host_now_ms=800;
	assert(led_button_hold(generation,true,700)==LED_DUPLICATE);
	host_step(1100); assert(host_value==0 && engine.winner.identity==hold_identity);
	host_step(1350); assert(host_value==10000);
	host_step(2100); assert(host_value==2500 && engine.winner.origin_ms==100);
	/* Faults and LOW still win, but never restart the physical wall-clock phase. */
	led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_SENSOR_MISSING,0);
	host_step(2100); assert(engine.winner.semantic==LED_SENSOR_MISSING);
	led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_NONE,0);
	host_step(2101); assert(engine.winner.semantic==LED_BUTTON_HOLD && host_value==2490);
	led_power_publish(LED_POWER_BATTERY,true);
	host_step(2102); assert(engine.winner.semantic==LED_LOW_BATTERY);
	led_power_publish(LED_POWER_BATTERY,false);
	host_step(2103); assert(engine.winner.semantic==LED_BUTTON_HOLD && host_value==2470);
	struct led_token exit=led_begin(LED_OWNER_SYSTEM,led_request_id());
	assert(led_button_exit(exit,1,generation)==LED_ADMITTED);
	assert(!engine.button_hold_active);
	uint32_t released=host_now_ms;
	for (uint32_t elapsed=0;elapsed<=1800;++elapsed) {
		host_step(released+elapsed);
		assert(engine.winner.semantic==LED_MANUAL_EXIT && engine.winner.origin_ms==1100);
		uint32_t age=released+elapsed-1100;
		uint16_t expected=age<250 || age>=1250 ? 0 : (1250-age)*10;
		assert(host_value==expected && host_level==LED_LEVEL_NOTICE);
	}
	assert(led_state(exit,2,LED_NONE)==LED_ADMITTED);
	assert(led_button_hold(generation,true,100)==LED_STALE);
	/* A newer physical press during a reversible fade survives old-generation
	 * cleanup. Cancelling the old fade exposes the new press's own origin. */
	host_reset(LED_CAP_RGB_PWM);
	generation=led_button_input();
	assert(led_button_hold(generation,true,0)==LED_ADMITTED);
	host_step(1000);
	exit=led_begin(LED_OWNER_SYSTEM,led_request_id());
	assert(led_button_exit(exit,1,generation)==LED_ADMITTED);
	host_step(1000);
	host_now_ms=1100;
	uint32_t newer=led_button_input();
	assert(led_button_hold(newer,true,1100)==LED_ADMITTED);
	assert(led_button_hold(generation,false,0)==LED_STALE);
	assert(led_state(exit,2,LED_NONE)==LED_ADMITTED);
	assert(led_button_exit(exit,3,generation)==LED_STALE);
	host_step(1600);
	assert(engine.winner.semantic==LED_BUTTON_HOLD && host_value==10000 && engine.winner.origin_ms==1100);
	assert(led_button_hold(newer,false,0)==LED_ADMITTED);
	assert(led_button_hold(newer,true,1100)==LED_STALE);
	host_step(1601); assert(engine.winner.semantic!=LED_BUTTON_HOLD);
	/* GPIO preserves the marker and full-ON fade interval, not a threshold. */
	host_reset(LED_CAP_MONO_GPIO);
	generation=led_button_input();
	assert(led_button_hold(generation,true,0)==LED_ADMITTED);
	for (uint32_t time=0;time<=3000;time+=100) {
		host_step(time);
		assert(host_value==(time<1000 || (time>=1250 && time<2250) || (time>=2500 && time<3000) ? 10000 : 0));
	}
	/* ISR retirement/replacement after selection but before actual driver call
	 * rejects the stale worker frame, just as count cancellation does. */
	void (*hooks[])(void)={release_selected_hold,replace_selected_hold};
	for (unsigned cause=0;cause<2;++cause) {
		host_reset(LED_CAP_RGB_PWM);
		generation=led_button_input();
		assert(led_button_hold(generation,true,0)==LED_ADMITTED);
		host_step(400);
		unsigned old_writes=host_writes;
		host_info_hook=hooks[cause];
		assert(host_step(500)==0 && host_writes==old_writes);
		host_step(500);
		if (cause) assert(engine.winner.semantic==LED_BUTTON_HOLD && engine.winner.origin_ms==500 && host_value==10000);
		else assert(engine.winner.semantic!=LED_BUTTON_HOLD && !host_value);
	}
	/* Release during the marker, within the fade, or after its held-blink
	 * boundary always inherits threshold origin and never replays light. */
	static const uint32_t releases[]={1000,1600,2800};
	for (unsigned i=0;i<sizeof(releases)/sizeof(releases[0]);++i) {
		host_reset(LED_CAP_RGB_PWM);
		generation=led_button_input();
		assert(led_button_hold(generation,true,0)==LED_ADMITTED);
		host_step(releases[i]);
		exit=led_begin(LED_OWNER_SYSTEM,led_request_id());
		assert(led_button_exit(exit,1,generation)==LED_ADMITTED);
		for (unsigned dt=0;dt<=LED_MANUAL_EXIT_MS;++dt) {
			host_step(releases[i]+dt);
			uint32_t age=releases[i]+dt-1000;
			assert(engine.winner.origin_ms==1000);
			assert(host_value==(age<250 || age>=1250 ? 0 : (1250-age)*10));
		}
	}
}

static void terminal_feedback_group_separation(void)
{
	host_reset(LED_CAP_MONO_GPIO);
	struct led_token token=led_begin(LED_OWNER_IMU,led_request_id());
	assert(led_result(token,led_event_id(),LED_ACCEPTED)==LED_ADMITTED);
	for (uint32_t time=0;time<=650;++time) host_step(time);
	assert(host_value==10000);
	/* Applied/persisted terminal replaces an incomplete acknowledgement. A
	 * fresh 600ms observed dark separator precedes its four short pulses. */
	assert(led_result(token,led_event_id(),LED_SUCCESS)==LED_ADMITTED);
	for (uint32_t time=650;time<=3250;++time) {
		host_step(time);
		uint32_t age=time>=1250 ? time-1250 : 2000;
		assert(host_value==(age<1400 && age%400<200 ? 10000 : 0));
		if (time<3250) assert(engine.winner.semantic==LED_SUCCESS);
	}
	assert(engine.winner.semantic!=LED_SUCCESS);
}

static void success_full_window_and_low_conflicts(void)
{
	const enum led_capability capabilities[]={LED_CAP_MONO_GPIO,LED_CAP_RGB_PWM};
	for (unsigned cap=0;cap<2;++cap) {
		for (unsigned late=0;late<2;++late) {
			host_reset(capabilities[cap]);
			assert(led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_SUCCESS)==LED_ADMITTED);
			led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_SENSOR,0);
			host_step(0);
			host_step(1000);
			uint32_t release=1400+late;
			host_step(release);
			assert(host_value==10000 && !engine.black_known); /* No preceding black credit. */
			led_fault_publish(LED_OWNER_SENSOR,LED_FAULT_NONE,0);
			host_step(release);
			if (late) {
				assert(engine.winner.semantic!=LED_SUCCESS);
				for (uint32_t time=release+1;time<=5000;++time) {
					host_step(time);
					assert(engine.winner.semantic!=LED_SUCCESS && !host_value);
				}
			} else {
				assert(engine.winner.semantic==LED_SUCCESS && engine.winner.origin_ms==2000 && !host_value);
				for (uint32_t time=release+1;time<4000;++time) {
					host_step(time);
					uint32_t age=time>=2000 ? time-2000 : 2000;
					assert(engine.winner.semantic==LED_SUCCESS);
					assert(host_value==(age<1400 && age%400<200 ? 10000 : 0));
				}
				host_step(4000);
				assert(engine.winner.semantic!=LED_SUCCESS && !host_value);
			}

			host_reset(capabilities[cap]);
			host_begin(LED_OWNER_ACC,LED_WAIT_STILL);
			led_power_publish(LED_POWER_BATTERY,true);
			host_step(0); host_step(2000);
			host_now_ms=7400+late; /* 600ms separator + 2000ms glyph must fit before LOW at 10000. */
			assert(led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_SUCCESS)==LED_ADMITTED);
			host_step(host_now_ms);
			assert(engine.winner.semantic==(late ? LED_WAIT_STILL : LED_SUCCESS));
			for (uint32_t time=7401+late;time<10000;++time) {
				host_step(time);
				if (!late) {
					uint32_t age=time>=8000 ? time-8000 : 2000;
					assert(engine.winner.semantic==LED_SUCCESS);
					assert(host_value==(age<1400 && age%400<200 ? 10000 : 0));
				} else assert(engine.winner.semantic!=LED_SUCCESS);
			}
			host_step(10000);
			assert(engine.winner.semantic==LED_LOW_BATTERY && host_value==10000);
			host_step(12000); /* The missed success cannot fit after LOW within its occurrence TTL. */
			assert(engine.winner.semantic!=LED_SUCCESS);
		}
		host_reset(capabilities[cap]);
		assert(led_request_event(LED_OWNER_SYSTEM,led_request_id(),led_event_id(),LED_SUCCESS)==LED_ADMITTED);
		host_step(0); host_step(600);
		assert(host_value==10000);
		led_power_publish(LED_POWER_BATTERY,true);
		host_step(700);
		assert(engine.winner.semantic==LED_LOW_BATTERY);
		led_power_publish(LED_POWER_BATTERY,false);
		for (uint32_t time=701;time<=5000;++time) {
			host_step(time);
			assert(engine.winner.semantic!=LED_SUCCESS && !host_value);
		}
	}
}

static void ready_heartbeat_boundaries(void)
{
	const enum led_capability capabilities[]={LED_CAP_MONO_GPIO,LED_CAP_RGB_PWM};
	const struct led_connection_facts facts={.radio_required=true,.healthy=true,.output_ready=true,.paired=true};
	for (unsigned cap=0;cap<2;++cap) {
		host_reset(capabilities[cap]);
		/* A network-enabled worker retains its last offset across loss.
		 * Seed a real zero-offset snapshot before testing local/held time,
		 * rather than inheriting a previous case's synchronization offset. */
		host_network_available=true;
		host_network_raw=0;
		led_connection_publish(&facts);
		host_step(0);
		host_network_available=false;
		for (uint32_t time=0;time<=20200;++time) {
			host_step(time);
			assert(engine.ready && engine.winner.semantic==LED_READY);
			uint32_t raw=led_sync_kernel_ticks(k_uptime_ticks(),CONFIG_SYS_CLOCK_TICKS_PER_SEC);
			assert(host_value==(led_sync_phase_ms(raw,10000)<200 ? 10000 : 0));
		}
		for (uint32_t age=0;age<=20000;++age) {
			struct led_envelope sample=led_timeline(LED_STYLE_READY,age,cap==0,false);
			uint32_t phase=age%10000;
			assert(!sample.complete && !sample.fading);
			assert(sample.value_pptt==(phase<200 ? 10000 : 0));
			assert(sample.next_ms==(phase<200 ? 200-phase : 10000-phase));
		}
#if CONFIG_LED_NETWORK_SYNC
		/* Probe raw 32768Hz ticks immediately around the 200ms edge and
		 * period wrap, rather than assuming integer uptime equals phase. */
		const uint32_t ticks[]={6553,6554,327679,327680,334233,334234};
		const bool on[]={true,false,false,true,true,false};
		host_network_available=true;
		for (unsigned i=0;i<sizeof(ticks)/sizeof(ticks[0]);++i) {
			host_network_raw=ticks[i];
			host_step(20300+i);
			assert(engine.winner.semantic==LED_READY && host_value==(on[i] ? 10000 : 0));
		}
		/* Leave a zero held offset for subsequent local-clock scenarios. */
		host_now_ms=20306;
		host_network_raw=led_sync_kernel_ticks(k_uptime_ticks(),CONFIG_SYS_CLOCK_TICKS_PER_SEC);
		host_step(host_now_ms);
		host_network_available=false;
#endif
	}
}

int main(void)
{
	invalid_owner_and_fault_inputs();
	credited_black_does_not_restart_unlit_event();
	timeline_boundaries(); finite_glyph_recognition(); persistent_fault_and_link_glyphs();
	success_full_window_and_low_conflicts(); ready_heartbeat_boundaries();
	hold_ack_black_gap(); identities_and_truthful_terminal();
	physical_hold_and_manual_handoff(); terminal_feedback_group_separation();
	breathing_grid_and_idle_wakes();
	expired_success_and_interrupted_result(); low_guarantee_and_admission();
	safety_record_and_reject_preserves_ota(); immediate_ready_and_blockers();
	external_power_priority_and_identify(); foreground_order_exit_and_quiesce();
	request_flood_tie_break_and_wrap(); invisible_effects_and_driver_black_failure();
	same_class_pending_and_input_merge(); actual_clock_wrap();
	reserved_completion_and_ota_lock();
	puts("semantic LED renderer: timeline/identity/TTL/LOW/READY/background/P0 scenarios passed");
	return 0;
}
