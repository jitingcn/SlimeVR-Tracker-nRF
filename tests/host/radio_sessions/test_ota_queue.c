#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../harness/include/atomic.h"

#define CONFIG_CONNECTION_RAW_COLLECTION 0
#define CONFIG_ESB_MAX_PAYLOAD_LENGTH 64
#define LOG_WRN(...) ((void)0)

static struct { uint8_t data[64], length; } rx_payload;
static void esb_ota_process_rx_packet(const uint8_t *data, size_t length);
#include "ota_queue.inc"

#define PACKET_COUNT 100000
static uint32_t accepted[PACKET_COUNT], received[PACKET_COUNT];
static unsigned accepted_count, received_count;
static bool inject_full_queue;
static atomic_t producer_done;

static void enqueue(uint32_t sequence)
{
    rx_payload.length = 5;
    rx_payload.data[0] = ESB_OTA_DATA_TYPE;
    memcpy(&rx_payload.data[1], &sequence, sizeof(sequence));
    receive_auxiliary();
}

static void esb_ota_process_rx_packet(const uint8_t *data, size_t length)
{
    assert(length == 5 && data[0] == ESB_OTA_DATA_TYPE);
    uint32_t sequence;
    memcpy(&sequence, &data[1], sizeof(sequence));
    if (inject_full_queue) {
        inject_full_queue = false;
        assert(sequence == 0 && atomic_get(&ota_rx_head) == 15);
        assert(atomic_get(&ota_rx_tail) == 0);
        enqueue(9999);
        assert(atomic_get(&ota_rx_head) == 15);
        uint32_t after;
        memcpy(&after, &data[1], sizeof(after));
        assert(after == sequence);
    }
    assert(received_count < PACKET_COUNT);
    received[received_count++] = sequence;
}

static void *producer(void *unused)
{
    (void)unused;
    for (uint32_t sequence = 0; sequence < PACKET_COUNT; ++sequence) {
        int previous = atomic_get(&ota_rx_head);
        enqueue(sequence);
        if (atomic_get(&ota_rx_head) != previous) {
            accepted[accepted_count++] = sequence;
        }
    }
    atomic_set(&producer_done, true);
    return NULL;
}

static void *consumer(void *unused)
{
    (void)unused;
    do {
        esb_process_ota_rx_queue();
    } while (!atomic_get(&producer_done) || atomic_get(&ota_rx_head) != atomic_get(&ota_rx_tail));
    return NULL;
}

int main(void)
{
    for (uint32_t i = 0; i < 15; ++i) enqueue(i);
    assert(atomic_get(&ota_rx_head) == 15 && atomic_get(&ota_rx_tail) == 0);
    enqueue(9998);
    assert(atomic_get(&ota_rx_head) == 15);
    inject_full_queue = true;
    esb_process_ota_rx_queue();
    assert(!inject_full_queue && received_count == 15);
    for (unsigned i = 0; i < 15; ++i) assert(received[i] == i);
    assert(atomic_get(&ota_rx_head) == 15 && atomic_get(&ota_rx_tail) == 15);
    for (uint32_t i = 15; i < 30; ++i) enqueue(i);
    esb_process_ota_rx_queue();
    assert(received_count == 30);
    for (unsigned i = 0; i < 30; ++i) assert(received[i] == i);

    atomic_set(&ota_rx_head, 0);
    atomic_set(&ota_rx_tail, 0);
    atomic_set(&producer_done, false);
    accepted_count = received_count = 0;
    pthread_t writer, reader;
    assert(pthread_create(&writer, NULL, producer, NULL) == 0);
    assert(pthread_create(&reader, NULL, consumer, NULL) == 0);
    assert(pthread_join(writer, NULL) == 0);
    assert(pthread_join(reader, NULL) == 0);
    assert(received_count == accepted_count && received_count > 0);
    for (unsigned i = 0; i < received_count; ++i) assert(received[i] == accepted[i]);
    puts("OTA queue publication, capacity, wrap and processing ownership: PASS");
    return 0;
}
