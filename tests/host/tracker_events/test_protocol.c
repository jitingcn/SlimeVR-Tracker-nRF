#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "connection/tracker_event_protocol.h"

int main(void)
{
    static const uint8_t golden[17] = {0xe0,2,1,0x14,0x78,0x56,0x34,0x12,0xff,0xff,7,0,3,16,0,0x43,0x45};
    static const uint8_t hid_golden[16] = {0xfb,0,0xe1,0x14,2,0x78,0x56,0x34,0x12,0xff,0xff,7,0,3,16,0};
    struct tracker_event event = {.nonce=0x12345678,.event_seq=65535,.operation_id=7,
        .tracker_id=2,.kind=CAL_KIND_MAG_MANUAL,.event=CAL_EVENT_END,
        .outcome=CAL_OUTCOME_SUCCESS,.phase=CAL_PHASE_APPLIED};
    uint8_t packet[17], hid[16];
    assert(tracker_event_encode(packet, &event));
    assert(memcmp(packet, golden, sizeof(packet)) == 0);
    assert(tracker_event_decode(packet, sizeof(packet), &event));
    tracker_event_encode_hid(hid, &event, RCV_HID_OP_TRACKER_EVENT);
    assert(memcmp(hid, hid_golden, sizeof(hid)) == 0);
    assert(tracker_event_decode_body(2, packet + 2, 15, &event));
    assert(event.event_seq == 65535 && event.operation_id == 7);
    event.event_seq = 0;
    event.phase = 250; /* Future calibration phases are opaque, not array indices. */
    assert(tracker_event_encode(packet, &event));
    assert(tracker_event_decode(packet, 17, &event) && event.phase == 250 && event.event_seq == 0);
    for (size_t n = 0; n < 20; n++) if (n != 17) assert(!tracker_event_decode(packet, n, &event));
    const unsigned offsets[] = {0, 2, 15, 16};
    for (unsigned i = 0; i < sizeof(offsets)/sizeof(offsets[0]); i++) {
        memcpy(packet, golden, 17); packet[offsets[i]] ^= 0x40;
        assert(!tracker_event_decode(packet, 17, &event));
    }
    memcpy(packet, golden, 17); packet[1]=16;
    assert(!tracker_event_decode(packet, 17, &event));
    memcpy(packet, golden, 17); packet[10]=0;
    assert(!tracker_event_decode(packet, 17, &event));
    memcpy(packet, golden, 17); packet[3]=CAL_EVENT_END|(CAL_OUTCOME_UNKNOWN<<4);
    assert(!tracker_event_decode(packet, 17, &event));
    struct { uint8_t kind, phase, detail, event; } values[] = {
        {TRACKER_EVENT_KIND_BUTTON, BUTTON_CLICK_GROUP, 3, CAL_EVENT_NOTICE},
        {TRACKER_EVENT_KIND_TRACKER_REST, TRACKER_REST_REST, TRACKER_REST_OBSERVED, CAL_EVENT_STATE},
        {TRACKER_EVENT_KIND_FUSION_REST, FUSION_REST_DETECTED, FUSION_BACKEND_VQF, CAL_EVENT_STATE},
        {TRACKER_EVENT_KIND_POWER, POWER_WILL_WOM, POWER_WOM_FORCED, CAL_EVENT_NOTICE},
    };
    const uint8_t vectors[][17] = {
        {0xe0,2,1,7,0x78,0x56,0x34,0x12,8,0,0,0,0x31,1,3,0x43,0x45},
        {0xe0,2,1,6,0x78,0x56,0x34,0x12,9,0,0,0,0x20,1,0,0x43,0x45},
        {0xe0,2,1,6,0x78,0x56,0x34,0x12,10,0,0,0,0x21,1,1,0x43,0x45},
        {0xe0,2,1,7,0x78,0x56,0x34,0x12,11,0,0,0,0x30,1,2,0x43,0x45},
    };
    for (unsigned i=0;i<4;i++) {
        event=(struct tracker_event){.nonce=0x12345678,.tracker_id=2,.event_seq=8+i,
            .kind=values[i].kind,.phase=values[i].phase,.detail=values[i].detail,.event=values[i].event};
        assert(tracker_event_encode(packet,&event)); assert(memcmp(packet,vectors[i],17)==0);
        assert(tracker_event_decode_body(2,packet+2,15,&event));
        event.kind |= CAL_EVENT_ORIGIN_AUTO; assert(!tracker_event_encode(packet,&event));
        event.kind &= TRACKER_EVENT_KIND_MASK; event.operation_id=1;
        assert(!tracker_event_encode(packet,&event));
        event.operation_id=0; event.outcome=CAL_OUTCOME_SUCCESS;
        assert(!tracker_event_encode(packet,&event));
    }
    event=(struct tracker_event){.nonce=1,.kind=TRACKER_EVENT_KIND_BUTTON,.event=CAL_EVENT_NOTICE,.phase=BUTTON_CLICK_GROUP};
    assert(!tracker_event_encode(packet,&event));
    event.detail=255; assert(tracker_event_encode(packet,&event));
    puts("PASS production protocol golden, bounds, future phase, kind-domain validation");
}
