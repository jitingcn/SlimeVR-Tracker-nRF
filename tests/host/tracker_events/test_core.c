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
    assert(send_at(100,true) && decode(1).event==CAL_EVENT_ACCEPTED);
    assert(send_at(200,true) && decode(2).event==CAL_EVENT_BEGIN);
    assert(send_at(300,true) && decode(3).kind==TRACKER_EVENT_KIND_BUTTON);
    drain(1400);
    unsigned before=nsent;
    uint32_t heartbeat_at=0;
    for(unsigned i=0;i<nsent;i++)if(decode(i).operation_id==op)heartbeat_at=sent_at[i]+2000;
    drain(heartbeat_at-1);assert(nsent==before);
    drain(heartbeat_at);
    assert(nsent==before+1 && decode(nsent-1).event==CAL_EVENT_BEGIN);
    assert(!memcmp(sent[2],sent[nsent-1],17));
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
    else if(!strcmp(argv[1],"wire"))wire();
    else if(entropy_error||entropy_zero)entropy();
    else return 2;
}
