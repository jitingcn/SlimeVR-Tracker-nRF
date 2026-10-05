/* Real console IRQ/editor/worker, packet codec and MLS; hardware/NVS leaves modeled. */
#include "transport.h"
static void clear_output(void) { output[0] = 0; output_size = 0; }
static void command(const char *line) {
    char buffer[128];
    assert(strlen(line) + 1 < sizeof(buffer));
    snprintf(buffer, sizeof(buffer), "%s\n", line);
    receive(buffer); run_worker();
    assert(lock_depth == 0 && !maintenance && !warm_transaction);
}
static void unchanged(const struct retained_fixture *before, unsigned writes) {
    assert(!memcmp(before, retained, sizeof(*before)));
    assert(persist_calls == writes);
}
static void seed(unsigned count, bool enabled) {
    sensor_tcal_backup_input_lost(); sensor_tcal_backup_process();
    assert(live_allocations == 0);
    memset(retained, 0, sizeof(*retained));
    retained->tcal_enabled = enabled;
    retained->gyroTemp = 25.123456f;
    retained->gyroBias[0] = 17.5f;
    retained->accelBias[2] = -8.5f;
    retained->magBias[3][1] = 1.25f;
    retained->gyroSensScale = 1.01f;
    retained->fusion_id = 42;
    for (unsigned j = 0; j < count; j++) {
        unsigned i = count == TCAL_BUFFER_SIZE ? j : j * (TCAL_BUFFER_SIZE - 1) / (count > 1 ? count - 1 : 1);
        retained->tempCalPoints[i].temp = 10.125f + (float)i / 2;
        for (unsigned axis = 0; axis < 3; axis++)
            retained->tempCalPoints[i].bias[axis] = 0.1234567f * (float)(axis + 1) + (float)i / 1024;
    }
    sensor_tcal_cache_invalidate();
    sensor_tcal_runtime_init_from_retained(); sensor_tcal_refresh_model();
    flash_data = *retained; clear_output();
}
static void finish_model(void) {
    for (int i = 0; i < 200; i++) if (sensor_tcal_build_lut_continue()) return;
    assert(!"model build did not complete");
}
#define PACKET_BYTES (24 + TCAL_BUFFER_SIZE * 16)
#define PAYLOAD_CHARS (((PACKET_BYTES + 2) / 3) * 4)
static char saved[PAYLOAD_CHARS + 128];
static void export_model(void) {
    struct retained_fixture before = *retained;
    unsigned writes = persist_calls, outcomes = outcome_calls, alloc = allocations;
    clear_output(); unsigned messages = printk_calls; background_logs = true;
    command("tcal export"); background_logs = false; unchanged(&before, writes);
    assert(outcome_calls == outcomes && allocations == alloc + 1 && !live_allocations);
    assert(printk_calls == messages + 1); /* One atomic logger package, not interleavable chunks. */
    const char *payload = strchr(output, '\n'); assert(payload); payload++;
    assert(strcspn(payload, "\n") == PAYLOAD_CHARS);
    memcpy(saved, payload, PAYLOAD_CHARS); saved[PAYLOAD_CHARS] = '\n'; saved[PAYLOAD_CHARS + 1] = 0;
    assert(strstr(payload + PAYLOAD_CHARS + 1, "[background log]")); clear_output();
}
static void replay(const char *text, bool check_unpublished) {
    struct retained_fixture before = *retained;
    unsigned writes = persist_calls;
    command("tcal import");
    if (!sensor_tcal_backup_active()) return; /* busy/ENOMEM admission */
    assert(live_allocations == 1 && live_bytes <= PACKET_BYTES + 64);
    receive(text); /* Entire >128-byte paste arrives before the worker runs. */
    if (check_unpublished) unchanged(&before, writes);
    assert(console_line_msgq.count == 0); /* No dependence on four queued lines. */
    run_worker(); assert(!live_allocations);
}
static void compare_model(const struct retained_fixture *expected) {
    assert(!memcmp(expected->tempCalPoints, retained->tempCalPoints, sizeof(expected->tempCalPoints)));
    assert(!memcmp(&expected->gyroTemp, &retained->gyroTemp, sizeof(float)));
    assert(expected->tcal_enabled == retained->tcal_enabled);
    assert(expected->tempCalState.count == retained->tempCalState.count);
    assert(tcal_compensation_enabled == expected->tcal_enabled);
    assert(tcal_curve_apply_ready == (expected->tcal_enabled && expected->tempCalState.count >= 4));
}
static void unrelated(const struct retained_fixture *before) {
    assert(!memcmp(before->gyroBias, retained->gyroBias, sizeof(before->gyroBias)));
    assert(!memcmp(before->accelBias, retained->accelBias, sizeof(before->accelBias)));
    assert(!memcmp(before->magBias, retained->magBias, sizeof(before->magBias)));
    assert(before->gyroSensScale == retained->gyroSensScale);
}
static void cold_restore(void) {
    memset(retained, 0, sizeof(*retained));
    tcal_compensation_enabled = false; tcal_curve_apply_ready = false;
    sensor_tcal_backup_input_lost(); sensor_tcal_backup_process(); sensor_tcal_cache_invalidate();
    *retained = flash_data; sensor_tcal_runtime_init_from_retained(); sensor_tcal_build_lut_priority(25.0f);
}
static void roundtrip(unsigned count, bool destination_enabled) {
    seed(count, !destination_enabled);
    if (count) {
        uint32_t words[] = {0x80000000u, 0x00000001u, 0x3eaaaaabu};
        memcpy(retained->tempCalPoints[0].bias, words, sizeof(words));
        retained->tempCalPoints[0].temp = 9.75f;
        sensor_tcal_refresh_model();
    }
    if (count < TCAL_BUFFER_SIZE) {
        const uint32_t empty_words[] = {0x80000000u, 0x00000001u, 0x80000000u, 0x3eaaaaabu};
        memcpy(&retained->tempCalPoints[1], empty_words, sizeof(empty_words));
    }
    struct retained_fixture expected = *retained;
    expected.tcal_enabled = destination_enabled;
    float before_bias[3] = {0};
    if (count >= 4) { finish_model(); assert(sensor_tcal_lut_lookup(25, before_bias) == 0); }
    export_model();
    seed(1, destination_enabled);
    struct retained_fixture unrelated_before = *retained;
    unsigned generation = sensor_tcal_model_generation(), old_reference = reference_generation;
    unsigned writes = persist_calls;
    replay(saved, true);
    assert(strstr(output, "saved successfully") && outcome_result == 0 && outcome_applied);
    assert(persist_calls == writes + 4);
    if (destination_enabled && count >= 4) {
        assert(reference_generation > old_reference && retained->fusion_id == 0);
    } else {
        assert(reference_generation == old_reference && retained->fusion_id == unrelated_before.fusion_id);
    }
    compare_model(&expected); unrelated(&unrelated_before);
    assert(sensor_tcal_model_generation() > generation && !retained->bootCalState.doffset_valid);
    float other[3];
    if (count >= 4) {
        finish_model(); assert(sensor_tcal_lut_lookup(25, other) == 0);
        for (unsigned a = 0; a < 3; a++) assert(fabsf(other[a] - before_bias[a]) < 1e-6f);
    }
    cold_restore(); compare_model(&expected); unrelated(&unrelated_before);
    if (count >= 4) {
        finish_model(); assert(sensor_tcal_lut_lookup(25, other) == 0);
        for (unsigned a = 0; a < 3; a++) assert(fabsf(other[a] - before_bias[a]) < 1e-6f);
    }
    printf("UART full-burst roundtrip count=%u destination-enabled=%u exact float32/cold restore/MLS OK\n", count, destination_enabled);
}
static const char base64_alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static unsigned base64_value(char c) {
    const char *p = strchr(base64_alphabet, c); assert(p && c); return (unsigned)(p-base64_alphabet);
}
static void fixture_decode(const char *text, uint8_t *bytes) {
    unsigned bits=0, count=0, n=0;
    for (unsigned i=0;i<PAYLOAD_CHARS && text[i]!='=';i++) {
        bits=(bits<<6)|base64_value(text[i]);count+=6;
        if(count>=8){count-=8;assert(n<PACKET_BYTES);bytes[n++]=(uint8_t)(bits>>count);}
    }
    assert(n==PACKET_BYTES);
}
static void fixture_encode(const uint8_t *bytes, char *text) {
    unsigned out=0;
    for(unsigned i=0;i<PACKET_BYTES;i+=3) {
        unsigned remain=PACKET_BYTES-i;
        uint32_t v=(uint32_t)bytes[i]<<16;
        if(remain>1)v|=(uint32_t)bytes[i+1]<<8;
        if(remain>2)v|=bytes[i+2];
        text[out++]=base64_alphabet[v>>18];text[out++]=base64_alphabet[(v>>12)&63];
        text[out++]=remain>1?base64_alphabet[(v>>6)&63]:'=';
        text[out++]=remain>2?base64_alphabet[v&63]:'=';
    }
    assert(out==PAYLOAD_CHARS);text[out++]='\n';text[out]=0;
}
static void replace_word(char *text, unsigned word, uint32_t value, bool repair_crc) {
    uint8_t bytes[PACKET_BYTES];fixture_decode(text,bytes);
    sys_put_le32(value,bytes+word*4);
    if(repair_crc)sys_put_le32(crc32_ieee(bytes,PACKET_BYTES-4),bytes+PACKET_BYTES-4);
    fixture_encode(bytes,text);
}
static void reject_payload(const char *bad) {
    seed(4, true);
    struct retained_fixture before = *retained, flash_before = flash_data;
    unsigned writes = persist_calls;
    replay(bad, true); unchanged(&before, writes);
    assert(!memcmp(&flash_before, &flash_data, sizeof(flash_data)));
    assert(!strstr(output, "saved successfully") && !live_allocations);
}
static void invalid_cases(void) {
    seed(4, true); export_model();
    char bad[sizeof(saved)];
    const struct { unsigned word; uint32_t value; } malformed[] = {
        {0, 0x314c4354}, {1, 11}, {2, 46}, {3, 1}, {4, 0x447a0000},
        {5, 0x41100000}, /* 9C is outside legacy producer edge */
        {5, 0x42340000}, /* 45C cannot live in slot zero */
    };
    for (unsigned i = 0; i < ARRAY_SIZE(malformed); i++) { strcpy(bad,saved); replace_word(bad,malformed[i].word,malformed[i].value,true); reject_payload(bad); }
    uint32_t nonfinite[] = {0x7fc00000, 0x7f800000, 0xff800000, 0x7f800001};
    for (unsigned i=0;i<ARRAY_SIZE(nonfinite);i++) for(unsigned field=4;field<13;field++) {
        strcpy(bad,saved);replace_word(bad,field,nonfinite[i],true);reject_payload(bad);
    }
    strcpy(bad,saved);bad[40]='!';reject_payload(bad);
    strcpy(bad,saved);bad[40]=bad[40]=='0'?'1':'0';reject_payload(bad);
    strcpy(bad,saved);bad[PAYLOAD_CHARS-1]='\n';bad[PAYLOAD_CHARS]=0;reject_payload(bad); /* odd */
    strcpy(bad,saved);bad[100]='\n';bad[101]=0;reject_payload(bad); /* short */
    strcpy(bad,saved);strcpy(bad+PAYLOAD_CHARS,"00\n");reject_payload(bad); /* valid CRC prefix, extra bytes */
    strcpy(bad,saved);strcpy(bad+PAYLOAD_CHARS," suffix\n");reject_payload(bad);
    strcpy(bad,saved);bad[10]='=';reject_payload(bad); /* early padding */
    strcpy(bad,saved);bad[PAYLOAD_CHARS-1]='A';reject_payload(bad); /* missing second pad */
    strcpy(bad,saved);bad[PAYLOAD_CHARS-2]='A';reject_payload(bad); /* wrong pad count */
    strcpy(bad,saved);bad[PAYLOAD_CHARS-3]=base64_alphabet[base64_value(bad[PAYLOAD_CHARS-3])|1];reject_payload(bad); /* noncanonical discarded bits */
    strcpy(bad,saved);
    for(unsigned i=0;i<PAYLOAD_CHARS;i++)if(isalpha((unsigned char)bad[i])){bad[i]=islower((unsigned char)bad[i])?(char)toupper((unsigned char)bad[i]):(char)tolower((unsigned char)bad[i]);break;}
    reject_payload(bad); /* Case must not be normalized by the console tokenizer. */
    reject_payload("\n");
    puts("wrong-version/geometry/range/nonfinite/Base64/CRC/padding/case/short/CRC-valid-prefix-plus-suffix rejected");
}
static void busy_and_storage(void) {
    seed(4,true);export_model();seed(1,false);
    struct retained_fixture before=*retained;unsigned writes=persist_calls;
#if !ACTUAL_OWNERSHIP
    busy=true;command("tcal export");replay(saved,true);unchanged(&before,writes);busy=false;
    command("tcal import");receive(saved);busy=true;run_worker();busy=false;
    unchanged(&before,writes);assert(!live_allocations);
#endif
    allocation_fail=true;command("tcal export");replay(saved,true);unchanged(&before,writes);allocation_fail=false;
    assert(!live_allocations);
    const int ids[]={MAIN_GYRO_TEMP_ID,MAIN_GYRO_TCAL_POINTS_ID,MAIN_GYRO_TCAL_COEFFS_ID,MAIN_GYRO_TCAL_STATE_ID};
    for(unsigned i=0;i<ARRAY_SIZE(ids);i++) {
        seed(1,false);tcal_compensation_storage_error=-ENOSPC;fail_id=ids[i];replay(saved,true);
        assert(strstr(output,"SAVE FAILED") && !strstr(output,"saved successfully"));
        assert(outcome_result==-EIO && outcome_applied && !live_allocations && !retained->tcal_enabled);
        assert(tcal_compensation_storage_error==-ENOSPC);
        fail_id=-1;clear_output();replay(saved,true);assert(strstr(output,"saved successfully"));
        assert(tcal_compensation_storage_error==-ENOSPC);
        cold_restore();assert(retained->tempCalState.count==4 && !retained->tcal_enabled);
    }
}
static void transport_loss(void) {
    seed(4,true);export_model();
    for(unsigned mode=0;mode<5;mode++) {
        seed(1,false);struct retained_fixture before=*retained;unsigned writes=persist_calls;
        command("tcal import");assert(live_allocations==1);receive("54434c32");
        switch(mode) {
        case 0:receive("\003");break;
        case 1:console_serial_close();break;
        case 2:console_serial_stop();break;
        case 3:now_ms+=120000;break;
        case 4:sensor_tcal_backup_input_lost();break;
        }
        run_worker();unchanged(&before,writes);assert(!live_allocations);
        if(mode==1||mode==2)assert(console_serial_start()==0);
        clear_output();replay(saved,true);assert(strstr(output,"saved successfully"));
    }
    seed(1,false);struct retained_fixture before=*retained;unsigned writes=persist_calls;
    before_storage_acquire=sensor_tcal_backup_input_lost;replay(saved,true);unchanged(&before,writes);assert(!live_allocations);
    command("tcal import");receive(saved);console_serial_close();run_worker();
    unchanged(&before,writes);assert(!live_allocations);assert(console_serial_start()==0);
    receive("tcal import\n");console_serial_close();run_worker();
    unchanged(&before,writes);assert(!live_allocations);assert(console_serial_start()==0);
    /* CRLF command terminator split across arming; payload CRLF likewise. */
    clear_output();receive("tcal import\r");run_worker();receive("\n");
    char crlf[sizeof(saved)];strcpy(crlf,saved);strcpy(crlf+PAYLOAD_CHARS,"\r\n");receive(crlf);run_worker();
    assert(strstr(output,"saved successfully") && !live_allocations);
    /* Full ordinary queue cannot hide import completion. */
    seed(1,false);command("tcal import");
    struct console_line_message noise={.epoch=console_input.epoch,.session=console_input.session};
    for(unsigned i=0;i<CONSOLE_LINE_QUEUE_DEPTH;i++)assert(k_msgq_put(&console_line_msgq,&noise,K_NO_WAIT)==0);
    receive(saved);run_worker();assert(strstr(output,"saved successfully") && !live_allocations);
    /* Retiring the physical console while allocation sleeps must never arm a stale session. */
    seed(1,false);before=*retained;writes=persist_calls;
    during_allocation=sensor_tcal_backup_input_lost;command("tcal import");
    unchanged(&before,writes);assert(!sensor_tcal_backup_active() && !live_allocations);
    during_allocation=console_serial_close;command("tcal import");
    unchanged(&before,writes);assert(!sensor_tcal_backup_active() && !live_allocations);
    assert(console_serial_start()==0);
    /* Even a complete CRC-valid packet stays unpublished until physical Enter. */
    seed(1,false);before=*retained;writes=persist_calls;command("tcal import");
    char unterminated[sizeof(saved)];strcpy(unterminated,saved);unterminated[PAYLOAD_CHARS]=0;
    receive(unterminated);run_worker();unchanged(&before,writes);assert(live_allocations==1);
    receive("AA\n");run_worker();unchanged(&before,writes);assert(!live_allocations);
    clear_output();command("tcal import");receive(unterminated);run_worker();unchanged(&before,writes);
    receive("\n");run_worker();assert(strstr(output,"saved successfully") && !live_allocations);
    puts("cancel/close/hard-stop/timeout/loss/storage-wait loss/CRLF/queue-full wake and free lifetimes OK");
}
int main(int argc,char **argv) {
    assert(console_serial_start()==0);
    if(argc==3 && !strcmp(argv[1],"--export")) {
        seed(TCAL_BUFFER_SIZE,true);export_model();FILE *f=fopen(argv[2],"w");assert(f);assert(fputs(saved,f)>=0);assert(!fclose(f));return 0;
    }
    if(argc==3 && !strcmp(argv[1],"--smoke")) {
        FILE *f=fopen(argv[2],"r");assert(f);size_t n=fread(saved,1,sizeof(saved)-1,f);assert(!ferror(f)&&feof(f));saved[n]=0;fclose(f);
        seed(0,false);memset(&flash_data,0,sizeof(flash_data));replay(saved,true);assert(strstr(output,"saved successfully"));
        struct retained_fixture expected=*retained;cold_restore();compare_model(&expected);
        clear_output();command("tcal export");assert(!strcmp(saved,output));fputs(output,stdout);
        if(retained->tempCalState.count>=4) {float bias[3];finish_model();assert(sensor_tcal_lut_lookup(25,bias)==0);fprintf(stderr,"cold restored real MLS bias: %.9g %.9g %.9g\n",(double)bias[0],(double)bias[1],(double)bias[2]);}
        fprintf(stderr,"external-file full-burst UART import/save/cold restore exact; points=%u enabled=%u\n",retained->tempCalState.count,retained->tcal_enabled);return 0;
    }
    assert(argc==1);
    for(unsigned n=0;n<=4;n++)for(unsigned enabled=0;enabled<2;enabled++)roundtrip(n,enabled);
    roundtrip(TCAL_BUFFER_SIZE,true);roundtrip(TCAL_BUFFER_SIZE,false);
    invalid_cases();busy_and_storage();transport_loss();assert(allocations==frees);
    puts("T-Cal compact backup production tests OK");return 0;
}
