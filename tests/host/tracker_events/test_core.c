/* Included after the unmodified production module, with hardware/kernel leaves. */
static uint8_t sent[1024][17];
static uint32_t sent_at[1024];
static unsigned nsent;
static bool emit_wire;
static struct tracker_event decode(unsigned index)
{
    struct tracker_event e;
    assert(index < nsent && tracker_event_decode(sent[index], 17, &e));
    return e;
}
static bool send_at(uint32_t at, bool success)
{
    now_ms=at;
    struct tracker_event_tx tx;
    if (!tracker_events_select(at, 2, &tx)) return false;
    if (success) {
        assert(nsent < 1024);
        if (nsent) assert((uint32_t)(at-sent_at[nsent-1]) >= 100);
        memcpy(sent[nsent],tx.packet,17); sent_at[nsent++]=at;
        if (emit_wire) {
            printf("@%u\n",at);
            for(unsigned i=0;i<17;i++) printf("%02x",tx.packet[i]);
            puts("");
        }
    }
    tracker_events_complete(tx.token,success,at);
    return true;
}
static void drain(uint32_t until)
{
    for(uint32_t t=now_ms;t<=until;t++) send_at(t,true);
    now_ms=until;
}
static unsigned count(uint8_t event, uint8_t phase)
{
    unsigned n=0;
    for(unsigned i=0;i<nsent;i++) { struct tracker_event e=decode(i); if(e.event==event && e.phase==phase)n++; }
    return n;
}
static void copies(void)
{
    tracker_event_notice(TRACKER_EVENT_KIND_BUTTON,BUTTON_CLICK_GROUP,3);
    tracker_events_notify();
    assert(send_at(0,false));
    assert(!send_at(9,true));
    assert(send_at(10,true));
    assert(!send_at(109,true));
    drain(1000);
    assert(nsent==3);
    assert(!memcmp(sent[0],sent[1],17) && !memcmp(sent[1],sent[2],17));
    tracker_event_notice(TRACKER_EVENT_KIND_BUTTON,BUTTON_CLICK_GROUP,3);
    drain(2000);
    assert(nsent==6 && decode(0).event_seq != decode(3).event_seq);
    assert(decode(3).detail==3);
    puts("PASS immutable copies, failed admission retry, 100ms spacing, independent button groups");
}
static void lifecycle(void)
{
    uint16_t op=cal_event_accept(CAL_KIND_IMU_ZRO); assert(op);
    cal_event_start(op,CAL_PHASE_WAIT_STILL,0);
    for(unsigned i=0;i<100;i++)cal_event_step(op,CAL_PHASE_COVERAGE,(uint8_t)i);
    cal_event_step(op,CAL_PHASE_APPLY_PENDING,0);
    cal_event_set_completion_reason(op,CAL_REASON_PARTIAL);
    cal_event_end(op,CAL_OUTCOME_SUCCESS,CAL_PHASE_APPLIED,CAL_REASON_NONE);
    cal_event_end(op,CAL_OUTCOME_FAILED,CAL_PHASE_NONE,CAL_REASON_MOTION);
    drain(1900);
    assert(nsent==7);
    assert(decode(0).event==CAL_EVENT_ACCEPTED && decode(1).event==CAL_EVENT_BEGIN && decode(2).event==CAL_EVENT_END);
    assert(count(CAL_EVENT_BEGIN,CAL_PHASE_WAIT_STILL)==1);
    assert(count(CAL_EVENT_STEP,CAL_PHASE_APPLY_PENDING)==0);
    unsigned ends=0;
    for(unsigned i=0;i<nsent;i++) { struct tracker_event e=decode(i); if(e.event==CAL_EVENT_END){ends++;assert(e.outcome==CAL_OUTCOME_SUCCESS && e.detail==CAL_REASON_PARTIAL);} }
    assert(ends==3);
    cal_event_step(op,CAL_PHASE_STORAGE,CAL_REASON_STORAGE_ERROR);
    drain(2900);
    assert(count(CAL_EVENT_STEP,CAL_PHASE_STORAGE)==3);
    unsigned before=nsent; drain(6000); assert(nsent==before);
    puts("PASS terminal idempotence, completion reason, pending deletion and post-END storage");
}
static void test_progress(void)
{
    uint16_t op=cal_event_begin(CAL_KIND_ACCEL_POSES,CAL_PHASE_WAIT_STILL,0);
    assert(send_at(0,true));
    for(unsigned i=0;i<100;i++){now_ms=i;cal_event_step(op,CAL_PHASE_COVERAGE,(uint8_t)i);}
    drain(298); assert(nsent==1);
    drain(1000); assert(nsent==2 && decode(1).detail==99);
    cal_event_step(op,CAL_PHASE_COVERAGE,99); drain(1900); assert(nsent==2);
    for(unsigned t=1901;t<=3300;t++) {
        now_ms=t; cal_event_step(op,CAL_PHASE_WAIT_POSE,(t%18)+1); send_at(t,true);
    }
    assert(nsent==3 && decode(2).phase==CAL_PHASE_WAIT_POSE);
    assert(sent_at[2]-sent_at[1]>=2000);
    puts("PASS latest-only progress, stable dwell, duplicate suppression, heartbeat flood escape");
}
static void online(void)
{
    const uint8_t kind=CAL_KIND_MAG_ONLINE|CAL_EVENT_ORIGIN_AUTO;
    for(unsigned outcome=CAL_OUTCOME_FAILED;outcome<=CAL_OUTCOME_SKIPPED;outcome++){
        uint16_t op=cal_event_begin(kind,CAL_PHASE_FREEZE,0);
        cal_event_step(op,CAL_PHASE_FIT,0);
        cal_event_end(op,outcome,CAL_PHASE_FIT,CAL_REASON_FIT_ERROR);
    }
    drain(999); assert(nsent==0);
    uint16_t op=cal_event_begin(kind,CAL_PHASE_FREEZE,0);
    cal_event_end(op,CAL_OUTCOME_SUCCESS,CAL_PHASE_CONFIRM,CAL_REASON_ENVIRONMENT_ONLY);
    drain(1999); assert(nsent==3 && decode(0).event==CAL_EVENT_END);
    op=cal_event_begin(kind,CAL_PHASE_FREEZE,0);
    cal_event_step(op,CAL_PHASE_PROBATION,0);
    drain(2998); assert(nsent==3);
    drain(3099); assert(nsent==4 && decode(3).event==CAL_EVENT_BEGIN && decode(3).phase==CAL_PHASE_PROBATION);
    cal_event_end(op,CAL_OUTCOME_FAILED,CAL_PHASE_VALIDATE,CAL_REASON_RADIAL);
    drain(3999); assert(nsent==7);
    puts("PASS invisible online failures suppressed, fast success retained, visible failure terminal");
}
static void test_states(void)
{
    uint32_t epoch=tracker_events_sensor_epoch();
    tracker_events_observe_sensor(epoch,true,true,true,true,FUSION_BACKEND_VQF,0);
    drain(499);
    tracker_event_set_state(TRACKER_EVENT_KIND_TRACKER_REST,TRACKER_REST_NOT_REST,TRACKER_REST_OBSERVED);
    tracker_event_set_state(TRACKER_EVENT_KIND_TRACKER_REST,TRACKER_REST_REST,TRACKER_REST_OBSERVED);
    unsigned before=nsent; drain(599);
    for(unsigned i=before;i<nsent;i++)assert(decode(i).phase!=TRACKER_REST_NOT_REST);
    now_ms=900;
    tracker_events_observe_sensor(epoch,true,true,false,false,FUSION_BACKEND_VQF,now_ms);
    drain(1200);
    bool fusion_unknown=false,tracker_unknown=false;
    for(unsigned i=0;i<nsent;i++){struct tracker_event e=decode(i); if(e.phase==2){fusion_unknown|=e.kind==0x21;tracker_unknown|=e.kind==0x20;}}
    assert(fusion_unknown && !tracker_unknown);
    drain(2200);
    for(unsigned i=0;i<nsent;i++){struct tracker_event e=decode(i);if(e.kind==0x20 && e.phase==2)tracker_unknown=true;}
    assert(tracker_unknown);
    tracker_events_sensor_invalidate(TRACKER_REST_SUSPENDED);
    uint32_t newer=tracker_events_sensor_epoch(); assert(newer && newer!=epoch);
    tracker_events_observe_sensor(epoch,true,true,true,true,FUSION_BACKEND_VQF,now_ms);
    before=nsent; drain(3100);
    for(unsigned i=before;i<nsent;i++)assert(decode(i).phase==2);
    tracker_events_observe_sensor(newer,true,true,false,false,FUSION_BACKEND_UNKNOWN,now_ms);
    before=nsent; drain(3900);
    bool rest=false,unavailable=false;
    for(unsigned i=before;i<nsent;i++){struct tracker_event e=decode(i);rest|=e.kind==0x20&&e.phase==1;unavailable|=e.kind==0x21&&e.phase==3&&e.detail==0;}
    assert(rest && unavailable);
    puts("PASS independent freshness, cancelled roundtrip state, stale epoch exclusion, backend unavailable");
}
static void receipts(void)
{
    tracker_event_set_state(0x20,1,0);
    struct tracker_event_tx old;
    assert(tracker_events_select(0,2,&old));
    tracker_event_set_state(0x20,0,0);
    tracker_events_complete(old.token,true,0);
    drain(900);
    assert(nsent>0);
    for(unsigned i=0;i<nsent;i++)assert(decode(i).phase==0);
    /* Saturation must not erase the newest action or allocate unbounded history. */
    for(unsigned i=1;i<=40;i++)tracker_event_notice(0x31,1,(uint8_t)i);
    nsent=0; drain(14000);
    bool newest=false;
    for(unsigned i=0;i<nsent;i++){struct tracker_event e=decode(i);newest|=e.kind==0x31&&e.detail==40;}
    assert(newest && nsent<=96);
    now_ms=15000;tracker_event_notice(0x31,1,255);
    now_ms=31000;nsent=0;drain(32000);assert(nsent==0);
    puts("PASS stale copy receipt isolation, bounded overflow and TTL expiration");
}
static void entropy(void)
{
    assert(cal_event_accept(CAL_KIND_IMU_ZRO)==0);
    cal_event_step(0,CAL_PHASE_COLLECT,0);cal_event_end(0,CAL_OUTCOME_SUCCESS,CAL_PHASE_APPLIED,0);
    tracker_event_notice(0x31,1,1);drain(2000);assert(nsent==0);
    assert(entropy_calls==4 || entropy_error);
    assert(error_logs>0);
    entropy_error=false;entropy_zero=false;
    tracker_events_session_changed();
    assert(cal_event_accept(CAL_KIND_IMU_ZRO)==0);
    tracker_event_notice(0x31,1,1);drain(3000);assert(nsent==0);
    puts("PASS entropy failure disables telemetry without uninitialized session");
}
static void state_heartbeat(void)
{
    uint32_t epoch=tracker_events_sensor_epoch();
    for(unsigned t=0;t<=5700;t++) {
        now_ms=t;
        if(t%100==0)tracker_events_observe_sensor(epoch,true,true,true,false,FUSION_BACKEND_VQF,t);
        send_at(t,true);
    }
    unsigned tracker_count=0,fusion_count=0,tracker_first=0,fusion_first=0;
    for(unsigned i=0;i<nsent;i++){
        struct tracker_event e=decode(i);
        if(e.kind==0x20){if(!tracker_count)tracker_first=i;tracker_count++;assert(!memcmp(sent[i],sent[tracker_first],17));}
        if(e.kind==0x21){if(!fusion_count)fusion_first=i;fusion_count++;assert(!memcmp(sent[i],sent[fusion_first],17));}
    }
    assert(tracker_count==4 && fusion_count==4);
    puts("PASS continuously fresh rest renewals retain immutable original sequence");
}
static void scheduling(void)
{
    assert(tracker_events_deadline(0)==UINT32_MAX);
    uint16_t op=cal_event_accept(CAL_KIND_IMU_ZRO);
    cal_event_start(op,CAL_PHASE_WAIT_STILL,0);
    tracker_event_notice(0x31,1,1);
    tracker_event_notice(0x30,POWER_WILL_SHUTDOWN,0);
    assert(tracker_events_deadline(0)==0);
    assert(send_at(0,true));
    assert(decode(0).kind==TRACKER_EVENT_KIND_POWER);
    assert(tracker_events_deadline(0)==100);
    assert(send_at(100,true) && decode(1).phase==POWER_WILL_SHUTDOWN);
    assert(send_at(200,true) && decode(2).phase==POWER_WILL_SHUTDOWN);
    assert(send_at(300,true) && decode(3).event==CAL_EVENT_ACCEPTED);
    assert(send_at(400,true) && decode(4).event==CAL_EVENT_BEGIN);
    assert(send_at(500,true) && decode(5).kind==TRACKER_EVENT_KIND_BUTTON);
    drain(1400);
    unsigned before=nsent;
    uint32_t heartbeat_at=0;
    for(unsigned i=0;i<nsent;i++)if(decode(i).operation_id==op)heartbeat_at=sent_at[i]+2000;
    drain(heartbeat_at-1);assert(nsent==before);
    drain(heartbeat_at);
    assert(nsent==before+1 && decode(nsent-1).event==CAL_EVENT_BEGIN);
    assert(!memcmp(sent[4],sent[nsent-1],17));
    uint32_t old_nonce=decode(0).nonce;
    tracker_events_session_changed();
    cal_event_end(op,CAL_OUTCOME_SUCCESS,CAL_PHASE_APPLIED,0);
    before=nsent;drain(now_ms+300);assert(nsent==before);
    tracker_event_notice(0x31,1,2);
    drain(now_ms+400);assert(nsent==before+3);
    assert(decode(before).nonce!=old_nonce);
    tracker_events_notify();unsigned woke=wake_calls;
    tracker_events_notify();assert(wake_calls==woke);
    puts("PASS power priority, generation-order first sends, immutable heartbeat, session reset and coalesced wake");
}
static void startup_delay(bool wake, bool watchdog)
{
    now_ms=137;
    tracker_events_schedule_boot(wake,watchdog);
    assert(wake_calls==1);
    uint32_t at=now_ms+TRACKER_EVENT_BOOT_DELAY_MS;
    assert(tracker_events_deadline(now_ms)==at);
    assert(!send_at(at-1,true));
    assert(send_at(at,false));
    assert(tracker_events_deadline(at)==at+10);
    assert(!send_at(at+9,true));
    drain(at+700);
    assert(count(CAL_EVENT_NOTICE,wake?POWER_WAKE:POWER_BOOT)==3);
    assert(count(CAL_EVENT_NOTICE,wake?POWER_BOOT:POWER_WAKE)==0);
    assert(count(CAL_EVENT_NOTICE,POWER_WATCHDOG_RESET)==(watchdog?3:0));
    for(unsigned i=0;i<nsent;i++) {
        struct tracker_event e=decode(i);
        assert(sent_at[i]>=at+10 && e.kind==TRACKER_EVENT_KIND_POWER && e.detail==0);
        if(i%3)assert(!memcmp(sent[i],sent[i-1],17));
    }
    if(watchdog)assert((uint16_t)(decode(3).event_seq-decode(0).event_seq)==1);
    unsigned before=nsent;
    tracker_events_schedule_boot(!wake,!watchdog);
    drain(now_ms+4000);
    assert(nsent==before && tracker_events_deadline(now_ms)==UINT32_MAX);
    puts("PASS deferred boot classification, optional watchdog, failed admission and one-shot scheduling");
}
static void startup_late(void)
{
    tracker_events_schedule_boot(false,false);
    /* Connection owner first polls after both the delay and an ordinary TTL. */
    now_ms=TRACKER_EVENT_BOOT_DELAY_MS+TRACKER_EVENT_TTL_MS+1;
    assert(tracker_events_deadline(now_ms)==now_ms);
    drain(now_ms+300);
    assert(nsent==3 && count(CAL_EVENT_NOTICE,POWER_BOOT)==3);
    puts("PASS startup TTL begins at deferred publication, not boot classification");
}
static int64_t future_event_wake(int64_t now, int32_t wait)
{
    uint32_t deadline=tracker_events_deadline((uint32_t)now);
    assert(deadline!=UINT32_MAX);
    int32_t delta=(int32_t)(deadline-(uint32_t)now);
    assert(delta==wait && delta>0);
    /* Exact connection_next_deadline_ms conversion, beyond the first epoch. */
    int64_t wake=now+(int32_t)(deadline-(uint32_t)now);
    assert(wake==now+wait && wake>now);
    return wake;
}
static void startup_wrap(void)
{
    now_ms=UINT32_MAX-999;
    tracker_events_schedule_boot(true,true);
    assert(tracker_events_deadline(now_ms)==2000);
    int64_t now64=(INT64_C(3)<<32)+now_ms;
    future_event_wake(now64,TRACKER_EVENT_BOOT_DELAY_MS);
    future_event_wake((INT64_C(4)<<32),2000);
    assert(!send_at(UINT32_MAX,true));
    assert(!send_at(1999,true));
    drain(2600);
    assert(nsent==6 && sent_at[0]==2000);
    assert(count(CAL_EVENT_NOTICE,POWER_WAKE)==3 && count(CAL_EVENT_NOTICE,POWER_WATCHDOG_RESET)==3);
    puts("PASS wrap-safe startup deadline across uptime rollover");
}
static void startup_wrap_sentinel(void)
{
    now_ms=UINT32_MAX-TRACKER_EVENT_BOOT_DELAY_MS;
    tracker_events_schedule_boot(false,false);
    int64_t now64=(INT64_C(3)<<32)+now_ms;
    const int64_t rollover=(INT64_C(4)<<32);
    /* A sentinel-valued future deadline sleeps to rollover, not now+1.
     * Follow the connection owner's 1000ms bounded fallback to that wake. */
    unsigned wakes=0;
    while(now64<rollover-1) {
        int64_t event_wake=future_event_wake(now64,(int32_t)(rollover-now64));
        assert(event_wake==rollover);
        assert(!send_at((uint32_t)now64,true));
        now64=event_wake<now64+1000?event_wake:now64+1000;
        wakes++;
    }
    assert(wakes==3 && now64==rollover-1);
    /* now+1 itself collides with the sentinel here; the wake is two ms away. */
    future_event_wake(rollover-2,2);
    future_event_wake(rollover-1,1);
    assert(tracker_events_deadline((uint32_t)rollover)==(uint32_t)rollover);
    assert(send_at((uint32_t)rollover,true));
    assert(!send_at(99,true));
    assert(send_at(100,true) && send_at(200,true));
    assert(nsent==3 && count(CAL_EVENT_NOTICE,POWER_BOOT)==3);
    assert(tracker_events_deadline(200)==UINT32_MAX);
    puts("PASS sentinel startup sleeps to a future 64-bit wake and sends after rollover without busy polling");
}
static void next_send_wrap_sentinel(void)
{
    now_ms=UINT32_MAX-TRACKER_EVENT_SPACING_MS;
    tracker_event_notice(TRACKER_EVENT_KIND_BUTTON,BUTTON_CLICK_GROUP,1);
    assert(send_at(now_ms,true));
    int64_t now64=(INT64_C(3)<<32)+now_ms;
    const int64_t rollover=(INT64_C(4)<<32);
    assert(future_event_wake(now64,TRACKER_EVENT_SPACING_MS+1)==rollover);
    assert(!send_at(UINT32_MAX-1,true));
    future_event_wake(rollover-2,2);
    future_event_wake(rollover-1,1);
    assert(tracker_events_deadline((uint32_t)rollover)==(uint32_t)rollover);
    assert(send_at((uint32_t)rollover,true));
    assert(!send_at(99,true));
    assert(send_at(100,true));
    assert(nsent==3 && !memcmp(sent[0],sent[1],17) && !memcmp(sent[1],sent[2],17));
    assert(tracker_events_deadline(100)==UINT32_MAX);
    puts("PASS sentinel send gate preserves spacing with a future 64-bit wake across rollover");
}
static void startup_session(void)
{
    tracker_events_schedule_boot(false,true);
    now_ms=1000;
    tracker_events_session_changed(); /* Initial pairing before due time. */
    assert(tracker_events_deadline(now_ms)==3000);
    assert(!send_at(2999,true));
    struct tracker_event_tx old;
    assert(tracker_events_select(3000,2,&old));
    struct tracker_event old_event;
    assert(tracker_event_decode(old.packet,17,&old_event));
    now_ms=3010;
    tracker_events_session_changed(); /* Materialized, but not yet admitted. */
    tracker_events_complete(old.token,true,3010); /* Stale admission cannot consume new identity. */
    assert(tracker_events_deadline(now_ms)==now_ms);
    assert(send_at(3010,true));
    assert(decode(0).phase==POWER_BOOT && decode(0).nonce!=old_event.nonce);
    now_ms=3110;
    tracker_events_session_changed(); /* BOOT admitted, watchdog still pending. */
    drain(3410);
    assert(nsent==4 && count(CAL_EVENT_NOTICE,POWER_BOOT)==1);
    assert(count(CAL_EVENT_NOTICE,POWER_WATCHDOG_RESET)==3);
    assert(decode(1).nonce!=decode(0).nonce);
    tracker_events_session_changed();
    tracker_events_schedule_boot(true,true);
    drain(8000);
    assert(nsent==4 && tracker_events_deadline(now_ms)==UINT32_MAX);
    puts("PASS pending startup survives pairing and stale receipt; admitted observations never replay on re-pair");
}
static void power_priority(void)
{
    tracker_events_schedule_boot(false,false);
    /* An already admitted ordinary packet still owns the global send gate. */
    tracker_event_notice(TRACKER_EVENT_KIND_BUTTON,BUTTON_CLICK_GROUP,1);
    assert(send_at(2950,true));
    for(unsigned i=0;i<40;i++)tracker_event_notice(TRACKER_EVENT_KIND_BUTTON,BUTTON_CLICK_GROUP,2);
    assert(!send_at(2999,true) && !send_at(3000,true));
    assert(tracker_events_deadline(3000)==3050);
    assert(send_at(3050,true) && decode(1).phase==POWER_BOOT);
    /* Ordinary producers saturating the queue cannot evict power repeats. */
    for(unsigned i=0;i<40;i++)tracker_event_notice(TRACKER_EVENT_KIND_BUTTON,BUTTON_CLICK_GROUP,3);
    assert(!send_at(3149,true));
    assert(send_at(3150,true) && decode(2).phase==POWER_BOOT);
    assert(send_at(3250,true) && decode(3).phase==POWER_BOOT);
    assert(!memcmp(sent[1],sent[2],17) && !memcmp(sent[2],sent[3],17));
    now_ms=3260;
    tracker_event_notice(TRACKER_EVENT_KIND_POWER,POWER_WILL_SHUTDOWN,0);
    drain(3260+TRACKER_EVENT_POWER_FLUSH_MS-1);
    assert(count(CAL_EVENT_NOTICE,POWER_WILL_SHUTDOWN)==3);
    puts("PASS power repeats outrank saturated ordinary backlog without bypassing deadline or global spacing");
}
static void terminal_priority(void)
{
    tracker_events_schedule_boot(false,true);
    assert(send_at(3000,true) && decode(0).phase==POWER_BOOT);
    now_ms=3001;
    tracker_event_notice(TRACKER_EVENT_KIND_POWER,POWER_WILL_SHUTDOWN,0);
    drain(now_ms+TRACKER_EVENT_POWER_FLUSH_MS-1);
    assert(count(CAL_EVENT_NOTICE,POWER_WILL_SHUTDOWN)==3);
    for(unsigned i=1;i<=3;i++)assert(decode(i).phase==POWER_WILL_SHUTDOWN);
    now_ms=3501;
    tracker_event_notice(TRACKER_EVENT_KIND_POWER,POWER_WILL_REBOOT,0);
    unsigned before=nsent;
    drain(now_ms+TRACKER_EVENT_POWER_FLUSH_MS-1);
    assert(count(CAL_EVENT_NOTICE,POWER_WILL_REBOOT)==3);
    for(unsigned i=before;i<before+3;i++)assert(decode(i).phase==POWER_WILL_REBOOT);
    /* Watchdog evidence already materialized retains its older sequence. */
    drain(4300);
    assert(count(CAL_EVENT_NOTICE,POWER_WATCHDOG_RESET)==3);
    puts("PASS shutdown and reboot repeat within 500ms ahead of boot/watchdog backlog");
}
static void terminal_before_startup(bool scheduled)
{
    if(scheduled)tracker_events_schedule_boot(false,true);
    now_ms=2900;
    tracker_event_notice(TRACKER_EVENT_KIND_POWER,POWER_WILL_SHUTDOWN,0);
    tracker_events_schedule_boot(false,true); /* Cannot revive retired startup. */
    drain(7000);
    assert(nsent==3 && count(CAL_EVENT_NOTICE,POWER_WILL_SHUTDOWN)==3);
    tracker_events_session_changed();
    tracker_events_schedule_boot(false,true);
    drain(11000);
    assert(nsent==3 && tracker_events_deadline(now_ms)==UINT32_MAX);
    puts("PASS terminal intent suppresses not-yet-materialized startup even across session changes");
}
static void startup_disconnected(void)
{
    tracker_events_schedule_boot(false,true);
    assert(tracker_events_deadline(3000)==3000); /* Materialize while disconnected. */
    struct tracker_event_tx first, later_tx;
    assert(tracker_events_select(3000,2,&first));
    tracker_events_complete(first.token,false,3000);
    now_ms=20000;
    assert(tracker_events_select(now_ms,2,&later_tx));
    assert(first.token==later_tx.token && !memcmp(first.packet,later_tx.packet,17));
    tracker_events_complete(later_tx.token,false,now_ms);
    /* Even another power producer cannot evict the unadmitted boot pair. */
    for(unsigned i=0;i<40;i++)tracker_event_notice(TRACKER_EVENT_KIND_POWER,POWER_WOM_CANCELLED,POWER_WOM_NORMAL);
    drain(20700);
    assert(count(CAL_EVENT_NOTICE,POWER_BOOT)==3 && count(CAL_EVENT_NOTICE,POWER_WATCHDOG_RESET)==3);
    assert(!memcmp(first.packet,sent[0],17));
    puts("PASS unadmitted startup survives queue TTL and pressure without changing its ordering identity");
}
static unsigned count_power(uint8_t phase)
{
    unsigned n=0;
    for(unsigned i=0;i<nsent;i++) {
        struct tracker_event e=decode(i);
        if(e.kind==TRACKER_EVENT_KIND_POWER && e.phase==phase)n++;
    }
    return n;
}
static void wom_cancellation(void)
{
    tracker_event_notice(TRACKER_EVENT_KIND_POWER,POWER_WILL_WOM,POWER_WOM_NORMAL);
    tracker_event_notice(TRACKER_EVENT_KIND_BUTTON,BUTTON_CLICK_GROUP,1);
    tracker_event_notice(TRACKER_EVENT_KIND_POWER,POWER_WOM_CANCELLED,POWER_WOM_NORMAL);
    drain(900);
    assert(count_power(POWER_WILL_WOM)==0);
    assert(count_power(POWER_WOM_CANCELLED)==3);
    assert(count(CAL_EVENT_NOTICE,BUTTON_CLICK_GROUP)==3);
    now_ms=1000;
    tracker_event_notice(TRACKER_EVENT_KIND_POWER,POWER_WILL_WOM,POWER_WOM_FORCED);
    assert(send_at(1000,true));
    struct tracker_event_tx inflight;
    assert(tracker_events_select(1100,2,&inflight));
    struct tracker_event old_event;
    assert(tracker_event_decode(inflight.packet,17,&old_event) && old_event.phase==POWER_WILL_WOM);
    now_ms=1101;
    tracker_event_notice(TRACKER_EVENT_KIND_POWER,POWER_WOM_CANCELLED,POWER_WOM_FORCED);
    tracker_events_complete(inflight.token,true,1101);
    unsigned before=nsent;
    assert(!send_at(1200,true));
    drain(1600);
    assert(nsent==before+3);
    for(unsigned i=before;i<nsent;i++) {
        struct tracker_event e=decode(i);
        assert(e.phase==POWER_WOM_CANCELLED && e.detail==POWER_WOM_FORCED);
        assert((int16_t)(e.event_seq-old_event.event_seq)>0);
    }
    /* A new sleep attempt after cancellation is independent, not retired. */
    tracker_event_notice(TRACKER_EVENT_KIND_POWER,POWER_WILL_WOM,POWER_WOM_NORMAL);
    drain(2000);
    assert(count_power(POWER_WILL_WOM)==4);
    assert((int16_t)(decode(nsent-1).event_seq-decode(before).event_seq)>0);
    puts("PASS WOM cancellation retires unsent and repeated intent, preserves order and tolerates an in-flight copy");
}
static void wire(void)
{
    emit_wire=true;
    uint16_t op=cal_event_accept(CAL_KIND_IMU_ZRO);cal_event_start(op,CAL_PHASE_WAIT_STILL,0);
    for(unsigned i=0;i<100;i++)cal_event_step(op,CAL_PHASE_COVERAGE,(uint8_t)i);
    drain(1000);cal_event_end(op,CAL_OUTCOME_SUCCESS,CAL_PHASE_APPLIED,0);drain(1900);
    op=cal_event_begin(CAL_KIND_MAG_MANUAL,CAL_PHASE_COLLECT,0);
    cal_event_end(op,CAL_OUTCOME_FAILED,CAL_PHASE_COLLECT,CAL_REASON_SAMPLE_TIMEOUT);drain(2900);
    tracker_event_notice(0x31,1,3);tracker_event_notice(0x30,POWER_WILL_WOM,POWER_WOM_FORCED);
    tracker_events_observe_sensor(tracker_events_sensor_epoch(),true,true,true,true,FUSION_BACKEND_VQF,now_ms);
    drain(4500);
    cal_event_begin(CAL_KIND_GYRO_SENS,CAL_PHASE_WAIT_ROTATION,2);drain(5000);
    tracker_events_observe_sensor(tracker_events_sensor_epoch(),true,true,true,true,FUSION_BACKEND_VQF,now_ms);
    drain(5500);
    puts("!"); /* New subscription: current rest only, no action/result replay. */
    puts("@21000"); /* Receiver observes transport silence, not tracker failure. */
    unsigned last_rest=0;
    for(unsigned i=0;i<nsent;i++)if(decode(i).kind==TRACKER_EVENT_KIND_TRACKER_REST)last_rest=i;
    puts("@22000");
    /* A late immutable copy is received after the cached value became stale. */
    for(unsigned i=0;i<17;i++)printf("%02x",sent[last_rest][i]);
    puts("");
}
int main(int argc,char **argv)
{
    assert(argc==2);
    entropy_error=!strcmp(argv[1],"entropy-error");
    entropy_zero=!strcmp(argv[1],"entropy-zero");
    assert(host_init()==0);
    if(!strcmp(argv[1],"copies"))copies();
    else if(!strcmp(argv[1],"lifecycle"))lifecycle();
    else if(!strcmp(argv[1],"progress"))test_progress();
    else if(!strcmp(argv[1],"online"))online();
    else if(!strcmp(argv[1],"states"))test_states();
    else if(!strcmp(argv[1],"receipts"))receipts();
    else if(!strcmp(argv[1],"scheduling"))scheduling();
    else if(!strcmp(argv[1],"state-heartbeat"))state_heartbeat();
    else if(!strcmp(argv[1],"startup-boot"))startup_delay(false,false);
    else if(!strcmp(argv[1],"startup-wake"))startup_delay(true,false);
    else if(!strcmp(argv[1],"startup-watchdog"))startup_delay(false,true);
    else if(!strcmp(argv[1],"startup-wake-watchdog"))startup_delay(true,true);
    else if(!strcmp(argv[1],"startup-late"))startup_late();
    else if(!strcmp(argv[1],"startup-wrap"))startup_wrap();
    else if(!strcmp(argv[1],"startup-session"))startup_session();
    else if(!strcmp(argv[1],"power-priority"))power_priority();
    else if(!strcmp(argv[1],"wom-cancellation"))wom_cancellation();
    else if(!strcmp(argv[1],"terminal-priority"))terminal_priority();
    else if(!strcmp(argv[1],"terminal-before-startup"))terminal_before_startup(true);
    else if(!strcmp(argv[1],"terminal-before-schedule"))terminal_before_startup(false);
    else if(!strcmp(argv[1],"startup-disconnected"))startup_disconnected();
    else if(!strcmp(argv[1],"startup-wrap-sentinel"))startup_wrap_sentinel();
    else if(!strcmp(argv[1],"next-send-wrap-sentinel"))next_send_wrap_sentinel();
    else if(!strcmp(argv[1],"wire"))wire();
    else if(entropy_error||entropy_zero)entropy();
    else return 2;
}
