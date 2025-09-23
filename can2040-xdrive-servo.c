// This is a simple example program using can2040 and the PICO SDK.
//
// See the CMakeLists.txt file for information on compiling.

#include <pico/stdlib.h>
#include "hardware/adc.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "can2040/src/can2040.h"

// Define the ADC channels for the joystick axes
#define JOYSTICK_X_ADC 0
#define JOYSTICK_Y_ADC 1

// Define the GPIO pins connected to the joystick axes
// ADC0 is on GPIO26, ADC1 is on GPIO27
#define JOYSTICK_X_PIN 26
#define JOYSTICK_Y_PIN 27

// ODrive CAN Commands
#define ODR_CMD_HEARTBEAT           0x001
#define ODR_CMD_ESTOP               0x002
#define ODR_CMD_GET_MOTOR_ERROR     0x003
#define ODR_CMD_GET_ENCODER_ERROR   0x004
#define ODR_CMD_GET_SENSORLESS_ERROR 0x005
#define ODR_CMD_SET_AXIS_NODE_ID    0x006
#define ODR_CMD_SET_AXIS_STATE      0x007
#define ODR_CMD_GET_ENCODER_ESTIMATES 0x009
#define ODR_CMD_GET_ENCODER_COUNT   0x00A
#define ODR_CMD_SET_CONTROLLER_MODES 0x00B
#define ODR_CMD_SET_INPUT_POS       0x00C
#define ODR_CMD_SET_INPUT_VEL       0x00D
#define ODR_CMD_SET_INPUT_TORQUE    0x00E
#define ODR_CMD_SET_VEL_LIMIT       0x00F
#define ODR_CMD_START_ANTICOGGING   0x010
#define ODR_CMD_SET_TRAJ_VEL_LIMIT  0x011
#define ODR_CMD_SET_TRAJ_ACCEL_LIMITS 0x012
#define ODR_CMD_SET_TRAJ_INERTIA    0x013
#define ODR_CMD_GET_IQ              0x014
#define ODR_CMD_GET_SENSORLESS_ESTIMATES 0x015
#define ODR_CMD_REBOOT              0x016
#define ODR_CMD_GET_VBUS_VOLTAGE    0x017
#define ODR_CMD_CLEAR_ERRORS        0x018
#define ODR_CMD_SET_LINEAR_COUNT    0x019
#define ODR_CMD_SET_POS_GAIN        0x01A
#define ODR_CMD_SET_VEL_GAINS       0x01B

// ODrive Axis States
#define AXIS_STATE_UNDEFINED                    0
#define AXIS_STATE_IDLE                         1
#define AXIS_STATE_STARTUP_SEQUENCE             2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE    3
#define AXIS_STATE_MOTOR_CALIBRATION            4
#define AXIS_STATE_SENSORLESS_CONTROL           5
#define AXIS_STATE_ENCODER_INDEX_SEARCH         6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION   7
#define AXIS_STATE_CLOSED_LOOP_CONTROL          8
#define AXIS_STATE_LOCKIN_SPIN                  9
#define AXIS_STATE_ENCODER_DIR_FIND             10

// Simple example of irq safe queue (this is not multi-core safe)
#define QUEUE_SIZE 128 // Must be power of 2
static struct {
    uint32_t pull_pos;
    volatile uint32_t push_pos;
    struct can2040_msg queue[QUEUE_SIZE];
} MessageQueue;

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

// Set ODrive axis state
bool set_axis_state(uint8_t node_id, uint32_t axis_state)
{
    uint32_t can_id = (node_id << 5) | ODR_CMD_SET_AXIS_STATE;
    uint8_t data[4];
    
    // Pack as little endian
    data[0] = (axis_state) & 0xFF;
    data[1] = (axis_state >> 8) & 0xFF;
    data[2] = (axis_state >> 16) & 0xFF;
    data[3] = (axis_state >> 24) & 0xFF;
    
    char description[64];
    snprintf(description, sizeof(description), "Set_Axis_State (Node %d -> State %d)", 
             node_id, axis_state);
    
    return send_can_message(can_id, 4, data, description, false);
}

// Request heartbeat/state from ODrive
bool request_heartbeat(uint8_t node_id)
{
    uint32_t can_id = (node_id << 5) | ODR_CMD_HEARTBEAT;
    return send_can_message(can_id, 0, NULL, "Request_Heartbeat", false);
}

// Send velocity command
bool send_velocity(uint8_t node_id, float velocity, float torque_ff)
{
    uint32_t can_id = (node_id << 5) | ODR_CMD_SET_INPUT_VEL;
    uint8_t data[8];
    
    memcpy(&data[0], &velocity, 4);
    memcpy(&data[4], &torque_ff, 4);
    
    char description[64];
    snprintf(description, sizeof(description), "Set_Input_Vel (Node %d, Vel %.2f)", 
             node_id, velocity);
    
    return send_can_message(can_id, 8, data, description, false);
}

int
main(void)
{
    stdio_init_all();
    sleep_ms(3000);  // Give time for USB serial to connect
    
    printf("\n=== ODrive CAN Control Test ===\n");
    
    canbus_setup();
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);
    
    // Try multiple node IDs - ODrives often default to 0
    uint8_t test_node_ids[] = {0, 1, 2, 3};
    uint8_t node_id = 0;  // Will be set based on which one responds
    bool odrive_found = false;
    
    printf("\nSearching for ODrive nodes...\n");
    
    // TODO: Test different node IDs to find the ODrive.
    //for (int i = 0; i < sizeof(test_node_ids); i++) {
    //    printf("\nTesting Node ID %d:\n", test_node_ids[i]);
    //    
    //    // Clear any existing messages
    //    MessageQueue.pull_pos = MessageQueue.push_pos;
    //    
    //    // Request heartbeat
    //    request_heartbeat(test_node_ids[i]);
    //    
    //    // Wait for response
    //    sleep_ms(100);
    //    
    //    // Check for any response
    //    if (MessageQueue.push_pos != MessageQueue.pull_pos) {
    //        printf("  -> ODrive found at Node ID %d!\n", test_node_ids[i]);
    //        node_id = test_node_ids[i];
    //        odrive_found = true;
    //        node_id = 1;
    //        break;
    //    } else {
    //        printf("  -> No response from Node ID %d\n", test_node_ids[i]);
    //    }
    //}
    
    //if (!odrive_found) {
    //    printf("\nWARNING: No ODrive responded to heartbeat requests.\n");
    //    return 1;
    //}
    sleep_ms(1000);
    node_id = 2;
    
    // Clear errors first
    printf("\nClearing ODrive errors...\n");
    uint32_t clear_errors_id = (node_id << 5) | ODR_CMD_CLEAR_ERRORS;
    send_can_message(clear_errors_id, 0, NULL, "Clear_Errors", true);
    sleep_ms(500);
    
    // Set to IDLE first
    printf("\nSetting axis to IDLE state...\n");
    set_axis_state(node_id, AXIS_STATE_IDLE);
    sleep_ms(1000);
    
    printf("\nSetting axis to FULL_CALIBRATION_SEQUENCE state...\n");
    set_axis_state(node_id, AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
    sleep_ms(10000);
    // Now try to set CLOSED_LOOP_CONTROL
    printf("\nAttempting to set CLOSED_LOOP_CONTROL...\n");
    set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL);
    sleep_ms(1000);
    
    // Request status to verify
    printf("\nRequesting current state...\n");
    request_heartbeat(node_id);
    sleep_ms(200);
    
    // Variables for velocity control
    float last_clamped_vel = 0.0f;
    uint32_t last_send_time = 0;
    uint32_t send_interval_ms = 100;  // Send velocity commands every 100ms
    
    printf("\nStarting main control loop...\n");
    printf("Move joystick X-axis to control motor velocity.\n");
    printf("Monitor serial output for CAN messages.\n\n");
    
    // Main loop
    for (;;) {
        // Handle incoming CAN messages
        while (MessageQueue.push_pos != MessageQueue.pull_pos) {
            struct can2040_msg *qmsg = &MessageQueue.queue[MessageQueue.pull_pos % QUEUE_SIZE];
            struct can2040_msg msg = *qmsg;
            MessageQueue.pull_pos = (MessageQueue.pull_pos + 1) % QUEUE_SIZE;

            // Decode the message
            uint8_t msg_node_id = (msg.id >> 5) & 0x3F;
            uint8_t cmd_id = msg.id & 0x1F;
            
            printf("RX: Node=%d, CMD=0x%02X, DLC=%d, Data=", 
                   msg_node_id, cmd_id, msg.dlc);
            for (int i = 0; i < msg.dlc; i++) {
                printf("%02X ", msg.data[i]);
            }
            
            // Decode specific message types
            if (cmd_id == ODR_CMD_HEARTBEAT && msg.dlc >= 8) {
                uint32_t axis_error, axis_state, controller_flags, encoder_flags;
                memcpy(&axis_error, &msg.data[0], 4);
                axis_state = msg.data[4];
                controller_flags = msg.data[5];
                encoder_flags = msg.data[6];
                
                printf(" [Heartbeat: Error=0x%08X, State=%d", axis_error, axis_state);
                
                // Decode axis state
                switch(axis_state) {
                    case AXIS_STATE_IDLE:
                        printf("(IDLE)");
                        break;
                    case AXIS_STATE_CLOSED_LOOP_CONTROL:
                        printf("(CLOSED_LOOP)");
                        break;
                    case AXIS_STATE_FULL_CALIBRATION_SEQUENCE:
                        printf("(CALIBRATING)");
                        break;
                    default:
                        printf("(STATE_%d)", axis_state);
                        break;
                }
                
                printf(", Ctrl=0x%02X, Enc=0x%02X]", controller_flags, encoder_flags);
                
                if (axis_error != 0) {
                    printf(" *** AXIS ERROR DETECTED ***");
                }
            }
            
            printf("\n");
        }
        
        // Read joystick and send velocity commands
        adc_select_input(JOYSTICK_X_ADC);
        uint16_t x_pos = adc_read();

        // Convert joystick X position to velocity (-5 to +5 for safety)
        float clamped_vel = (((float)x_pos - 2047.5f) / 2047.5f) * 5.0f;
        
        // Apply deadband
        if (fabs(clamped_vel) < 0.2f) {
            clamped_vel = 0.0f;
        }
        
        // Send velocity command periodically
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if ((current_time - last_send_time >= send_interval_ms) || 
            (fabs(clamped_vel - last_clamped_vel) > 0.1f)) {
            
            send_velocity(node_id, clamped_vel, 0.0f);
            last_clamped_vel = clamped_vel;
            last_send_time = current_time;
        }
        
        // Request periodic status updates
        static uint32_t last_heartbeat = 0;
        if (current_time - last_heartbeat > 2000) {  // Every 2 seconds
            request_heartbeat(node_id);
            last_heartbeat = current_time;
        }
        
        sleep_ms(10);
    }

    return 0;
}