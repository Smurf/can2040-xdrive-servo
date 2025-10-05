#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include "canbus.h"
#include "can2040/src/can2040.h"

MessageQueue_t MessageQueue;
// Internal storage for can2040 module
static struct can2040 cbus;

// Main canbus callback (called from irq handler)
static void
can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if (notify == CAN2040_NOTIFY_RX) {
        // Add to queue
        uint32_t push_pos = MessageQueue.push_pos;
        uint32_t pull_pos = MessageQueue.pull_pos;
        if ((push_pos + 1) % QUEUE_SIZE == pull_pos)
            // No space in queue
            return;
        MessageQueue.queue[push_pos % QUEUE_SIZE] = *msg;
        MessageQueue.push_pos = (push_pos + 1) % QUEUE_SIZE;
    }
}

// PIO interrupt handler
static void
PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

// Initialize the can2040 module
void
canbus_setup(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = SYS_CLK_HZ, bitrate = 250000;
    uint32_t gpio_rx = 20, gpio_tx = 21;
    //uint32_t gpio_rx = 26, gpio_tx = 27;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, 1);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
    
    printf("CAN bus initialized: RX=GPIO%d, TX=GPIO%d, Baudrate=%d\n", 
           gpio_rx, gpio_tx, bitrate);
}

// Send a CAN message with error checking
bool send_can_message(uint32_t can_id, uint8_t dlc, uint8_t* data, const char* description, bool is_rtr)
{
    struct can2040_msg msg;
    msg.id = can_id;

    if (is_rtr){
        msg.id |= CAN2040_ID_RTR;
    }

    memset(msg.data, 0, 8);
    msg.dlc = dlc;
    if (data && dlc > 0) {
        memcpy(msg.data, data, dlc);
    }
    
    int status = can2040_transmit(&cbus, &msg);
    
    printf("%s: CAN_ID=0x%03X, DLC=%d, Status=%d\n", 
           description, can_id, dlc, status);
    
    if (dlc > 0 && data) {
        printf("  Data: ");
        for (int i = 0; i < dlc; i++) {
            printf("%02X ", data[i]);
        }
        printf("\n");
    }
    
    if (status != 0) {
        printf("  ERROR: Transmission failed!\n");
        return false;
    }
    
    return true;
}