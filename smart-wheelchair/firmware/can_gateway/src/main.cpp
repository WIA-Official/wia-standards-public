/**
 * @file main.cpp
 * @brief WIA Smart Wheelchair CAN Gateway Main Application
 * @version 1.0.0
 *
 * This firmware provides a bridge between the CAN bus and ROS2
 * via USB/Serial communication using a simple protocol.
 */

#include <Arduino.h>
#include "wia_can_protocol.h"

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define SERIAL_BAUD         115200
#define HEARTBEAT_INTERVAL  100     // ms (10 Hz)
#define CAN_TIMEOUT         500     // ms
#define LED_PIN             LED_BUILTIN

/*******************************************************************************
 * Serial Protocol
 * Format: [START][LEN][CMD][ID_H][ID_L][DLC][DATA...][CHECKSUM][END]
 * START = 0xAA, END = 0x55
 ******************************************************************************/

#define PROTO_START         0xAA
#define PROTO_END           0x55

// Commands
#define CMD_CAN_SEND        0x01    // Send CAN message
#define CMD_CAN_RECV        0x02    // Received CAN message
#define CMD_HEARTBEAT       0x10    // Gateway heartbeat
#define CMD_SET_MODE        0x20    // Set gateway mode
#define CMD_GET_STATUS      0x21    // Get gateway status
#define CMD_ERROR           0xF0    // Error response

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

static uint8_t g_node_id = NODE_ID_MAIN_CTRL;
static wia_node_state_t g_node_state = NODE_STATE_BOOT;
static uint32_t g_last_heartbeat = 0;
static uint32_t g_uptime_seconds = 0;
static uint8_t g_error_count = 0;
static uint8_t g_rx_buffer[64];
static uint8_t g_rx_index = 0;

/*******************************************************************************
 * CAN Interface (Platform-specific)
 ******************************************************************************/

#if defined(HAL_CAN_MODULE_ENABLED)
// STM32 native CAN
#include <STM32_CAN.h>
STM32_CAN Can(CAN1, DEF);

void can_init() {
    Can.begin();
    Can.setBaudRate(WIA_CAN_BITRATE);
}

bool can_send(uint32_t id, uint8_t *data, uint8_t len) {
    CAN_message_t msg;
    msg.id = id;
    msg.len = len;
    memcpy(msg.buf, data, len);
    return Can.write(msg) > 0;
}

bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    CAN_message_t msg;
    if (Can.read(msg)) {
        *id = msg.id;
        *len = msg.len;
        memcpy(data, msg.buf, msg.len);
        return true;
    }
    return false;
}

#elif defined(USE_MCP2515)
// ESP32 with MCP2515
#include <mcp2515.h>
MCP2515 mcp2515(5);  // CS pin

void can_init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
}

bool can_send(uint32_t id, uint8_t *data, uint8_t len) {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = len;
    memcpy(frame.data, data, len);
    return mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK;
}

bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    struct can_frame frame;
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        *id = frame.can_id;
        *len = frame.can_dlc;
        memcpy(data, frame.data, frame.can_dlc);
        return true;
    }
    return false;
}

#elif defined(USE_FLEXCAN)
// Teensy with FlexCAN
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can;

void can_init() {
    Can.begin();
    Can.setBaudRate(WIA_CAN_BITRATE);
}

bool can_send(uint32_t id, uint8_t *data, uint8_t len) {
    CAN_message_t msg;
    msg.id = id;
    msg.len = len;
    memcpy(msg.buf, data, len);
    return Can.write(msg) > 0;
}

bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    CAN_message_t msg;
    if (Can.read(msg)) {
        *id = msg.id;
        *len = msg.len;
        memcpy(data, msg.buf, msg.len);
        return true;
    }
    return false;
}

#else
// Stub implementation for testing
void can_init() {}
bool can_send(uint32_t id, uint8_t *data, uint8_t len) {
    (void)id; (void)data; (void)len;
    return true;
}
bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    (void)id; (void)data; (void)len;
    return false;
}
#endif

/*******************************************************************************
 * Serial Protocol Functions
 ******************************************************************************/

uint8_t calculate_checksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

void send_serial_packet(uint8_t cmd, uint16_t can_id, uint8_t *data, uint8_t dlc) {
    uint8_t packet[16];
    uint8_t idx = 0;

    packet[idx++] = PROTO_START;
    packet[idx++] = dlc + 4;        // Length: cmd + id(2) + dlc + data
    packet[idx++] = cmd;
    packet[idx++] = (can_id >> 8) & 0xFF;
    packet[idx++] = can_id & 0xFF;
    packet[idx++] = dlc;

    for (uint8_t i = 0; i < dlc; i++) {
        packet[idx++] = data[i];
    }

    // Checksum over cmd, id, dlc, data
    packet[idx++] = calculate_checksum(&packet[2], dlc + 4);
    packet[idx++] = PROTO_END;

    Serial.write(packet, idx);
}

void send_error(uint8_t error_code) {
    uint8_t data[1] = {error_code};
    send_serial_packet(CMD_ERROR, 0, data, 1);
    g_error_count++;
}

void process_serial_command(uint8_t *packet, uint8_t len) {
    if (len < 5) {
        send_error(0x01);  // Invalid length
        return;
    }

    uint8_t cmd = packet[0];
    uint16_t can_id = (packet[1] << 8) | packet[2];
    uint8_t dlc = packet[3];

    if (dlc > WIA_CAN_MAX_DLC) {
        send_error(0x02);  // Invalid DLC
        return;
    }

    switch (cmd) {
        case CMD_CAN_SEND:
            if (!can_send(can_id, &packet[4], dlc)) {
                send_error(0x10);  // CAN send failed
            }
            break;

        case CMD_SET_MODE:
            // Handle mode change
            if (dlc >= 1) {
                g_node_state = (wia_node_state_t)packet[4];
            }
            break;

        case CMD_GET_STATUS:
            {
                uint8_t status[4];
                status[0] = g_node_id;
                status[1] = g_node_state;
                status[2] = g_error_count;
                status[3] = 0;  // Reserved
                send_serial_packet(CMD_GET_STATUS, 0, status, 4);
            }
            break;

        default:
            send_error(0x03);  // Unknown command
            break;
    }
}

void handle_serial_input() {
    while (Serial.available()) {
        uint8_t byte = Serial.read();

        if (g_rx_index == 0) {
            // Waiting for start byte
            if (byte == PROTO_START) {
                g_rx_buffer[g_rx_index++] = byte;
            }
        } else if (g_rx_index == 1) {
            // Length byte
            g_rx_buffer[g_rx_index++] = byte;
            if (byte > sizeof(g_rx_buffer) - 4) {
                g_rx_index = 0;  // Invalid length
            }
        } else {
            g_rx_buffer[g_rx_index++] = byte;

            uint8_t expected_len = g_rx_buffer[1] + 4;  // START + LEN + data + CHECKSUM + END

            if (g_rx_index >= expected_len) {
                // Check end byte
                if (g_rx_buffer[g_rx_index - 1] == PROTO_END) {
                    // Verify checksum
                    uint8_t calc_cs = calculate_checksum(&g_rx_buffer[2], g_rx_buffer[1]);
                    uint8_t recv_cs = g_rx_buffer[g_rx_index - 2];

                    if (calc_cs == recv_cs) {
                        process_serial_command(&g_rx_buffer[2], g_rx_buffer[1]);
                    } else {
                        send_error(0x04);  // Checksum error
                    }
                }
                g_rx_index = 0;
            }
        }
    }
}

/*******************************************************************************
 * Heartbeat
 ******************************************************************************/

void send_can_heartbeat() {
    wia_can_heartbeat_t hb;
    hb.node_id = g_node_id;
    hb.state = g_node_state;
    hb.uptime = g_uptime_seconds;
    hb.reserved = 0;
    hb.error_count = g_error_count;
    hb.checksum = wia_can_checksum((uint8_t*)&hb, 7);

    can_send(CAN_ID_HEARTBEAT, (uint8_t*)&hb, sizeof(hb));
}

/*******************************************************************************
 * Main Application
 ******************************************************************************/

void setup() {
    // Initialize serial
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000);  // Wait for serial (max 3s)

    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize CAN
    can_init();

    // Set initial state
    g_node_state = NODE_STATE_READY;

    // Send startup message
    Serial.println(F("WIA CAN Gateway v" WIA_WHEELCHAIR_VERSION));
}

void loop() {
    uint32_t now = millis();
    static uint32_t last_second = 0;

    // Process serial commands
    handle_serial_input();

    // Process CAN messages
    uint32_t can_id;
    uint8_t can_data[8];
    uint8_t can_len;

    while (can_receive(&can_id, can_data, &can_len)) {
        // Forward CAN message to serial
        send_serial_packet(CMD_CAN_RECV, can_id, can_data, can_len);

        // Handle emergency stop locally
        if (can_id == CAN_ID_ESTOP) {
            wia_can_estop_t *estop = (wia_can_estop_t*)can_data;
            if (estop->state == 1) {
                digitalWrite(LED_PIN, HIGH);  // LED on for E-stop
            } else {
                digitalWrite(LED_PIN, LOW);
            }
        }
    }

    // Heartbeat (10 Hz)
    if (now - g_last_heartbeat >= HEARTBEAT_INTERVAL) {
        g_last_heartbeat = now;
        send_can_heartbeat();

        // Blink LED to show activity (if not in E-stop)
        static bool led_state = false;
        if (g_node_state == NODE_STATE_OPERATIONAL) {
            led_state = !led_state;
            digitalWrite(LED_PIN, led_state);
        }
    }

    // Update uptime
    if (now - last_second >= 1000) {
        last_second = now;
        g_uptime_seconds++;

        // Transition to operational after 2 seconds
        if (g_uptime_seconds >= 2 && g_node_state == NODE_STATE_READY) {
            g_node_state = NODE_STATE_OPERATIONAL;
        }
    }
}
