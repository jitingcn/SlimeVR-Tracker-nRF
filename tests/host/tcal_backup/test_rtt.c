#define main backup_suite_main
#include "test_backup.c"
#undef main
/* SEGGER input leaves only; both RTT reader and console worker are production. */
static const char *rtt_input = "";
static bool rtt_expire;
static int SEGGER_RTT_HasKey(void) { return *rtt_input != '\0'; }
static int SEGGER_RTT_GetKey(void) { return *rtt_input ? (unsigned char)*rtt_input++ : -1; }
static void k_usleep(int us) {
    (void)us;
    if (rtt_expire) { now_ms += TCAL_BACKUP_TIMEOUT_MS; rtt_expire = false; return; }
    longjmp(worker_idle, 1);
}
#include "rtt_production.h"
#undef USB_EXISTS
#define USB_EXISTS 0
#include "rtt_worker.inc"
#undef USB_EXISTS
#define USB_EXISTS 1
static void rtt_receive(const char *text) {
    during_sleep = k_usleep;
    rtt_input = text;
    if (setjmp(worker_idle) == 0) console_rtt_thread();
    during_sleep = NULL;
    assert(!*rtt_input && !lock_depth && !warm_transaction && !maintenance);
}
int main(void) {
    assert(console_serial_start() == 0);
    seed(TCAL_BUFFER_SIZE,true); export_model();
    struct retained_fixture expected = *retained;
    expected.tcal_enabled = false;
    seed(1,false);
    rtt_receive("tcal import\r"); assert(live_allocations == 1);
    rtt_receive("\n"); assert(live_allocations == 1); /* delayed LF after ready */
    char full[sizeof(saved)]; strcpy(full,saved); strcpy(full+PAYLOAD_CHARS,"\r\n");
    rtt_receive(full); assert(strstr(output,"saved successfully") && !live_allocations);
    compare_model(&expected); cold_restore(); compare_model(&expected);
    float bias[3]; finish_model(); assert(sensor_tcal_lut_lookup(25,bias) == 0);
    /* A valid CRC prefix followed by extra data on that physical line is not a commit. */
    for(unsigned bad=0;bad<4;bad++) {
        seed(1,false); struct retained_fixture before=*retained; unsigned writes=persist_calls;
        rtt_receive("tcal import\n"); assert(live_allocations==1);
        strcpy(full,saved);
        if(bad==0)strcpy(full+PAYLOAD_CHARS,"00\r\n");
        if(bad==1)strcpy(full+PAYLOAD_CHARS," invalid_suffix\n");
        if(bad==2){full[PAYLOAD_CHARS-1]='\n';full[PAYLOAD_CHARS]=0;}
        if(bad==3){full[64]='\n';full[65]=0;}
        rtt_receive(full); unchanged(&before,writes); assert(!live_allocations && !strstr(output,"saved successfully"));
    }
    seed(1,false); struct retained_fixture before=*retained; unsigned writes=persist_calls;
    rtt_receive("tcal import\n"); rtt_receive("54434c32\003"); unchanged(&before,writes); assert(!live_allocations);
    rtt_receive("tcal import\n"); rtt_receive("54434c32"); rtt_expire=true; rtt_receive(""); unchanged(&before,writes); assert(!live_allocations);
    /* Ordinary RTT editor still accepts 63 and discards 64, without prefix dispatch. */
    char line_text[100];
    for(unsigned n=63;n<=64;n++) {
        memset(line_text,' ',n); memcpy(line_text,"tcal export",11); strcpy(line_text+n,"\n");
        clear_output(); unsigned before_alloc=allocations;
        rtt_receive(line_text);
        assert(allocations == before_alloc + (n==63));
        if(n==64)assert(strstr(output,"Input line too long"));
    }
    clear_output();rtt_receive("tcal import\r\n");rtt_receive(full); /* malformed session is safely reusable */
    sensor_tcal_backup_input_lost();sensor_tcal_backup_process();
    clear_output();rtt_receive("tcal import\r\n");rtt_receive(saved);assert(strstr(output,"saved successfully") && !live_allocations);
    assert(allocations==frees);
    puts("actual RTT reader+worker: 1528-byte Base64 burst, CRLF, malformed/prefix suffix, Ctrl-C, timeout, cold MLS, 63/64 normal editor boundary OK");
    return 0;
}
