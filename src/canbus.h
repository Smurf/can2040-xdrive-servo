#include "can2040/src/can2040.h"

// Simple example of irq safe queue (this is not multi-core safe)
#define QUEUE_SIZE 128 // Must be power of 2
typedef struct MessageQueue_t{
    uint32_t pull_pos;
    volatile uint32_t push_pos;
    struct can2040_msg queue[QUEUE_SIZE];
} MessageQueue_t;

extern struct MessageQueue_t MessageQueue;
static struct can2040 cbus;


bool send_can_message(uint32_t can_id, uint8_t dlc, uint8_t* data, const char* description, bool is_rtr);

void canbus_setup(void);
