/**
 * @file ble_service.h
 * @brief BLE Service Definitions for WIA EMG
 * @version 1.0.0
 */

#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <Arduino.h>

// WIA EMG BLE Service UUIDs
// Using custom 128-bit UUIDs for the service

// Base UUID: 12345678-1234-5678-1234-56789abcdef0
// This should be replaced with properly generated UUIDs in production

namespace WIA_BLE {
    // Service UUID
    const char* SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0";

    // Characteristic UUIDs
    const char* CHAR_EMG_DATA_UUID    = "12345678-1234-5678-1234-56789abcdef1";
    const char* CHAR_COMMAND_UUID     = "12345678-1234-5678-1234-56789abcdef2";
    const char* CHAR_STATUS_UUID      = "12345678-1234-5678-1234-56789abcdef3";
    const char* CHAR_CONFIG_UUID      = "12345678-1234-5678-1234-56789abcdef4";

    // Device name
    const char* DEVICE_NAME = "WIA-EMG-2CH";

    // MTU size (maximum transmission unit)
    const uint16_t MTU_SIZE = 185;

    // Connection parameters
    const uint16_t MIN_CONN_INTERVAL = 6;   // 7.5ms (units of 1.25ms)
    const uint16_t MAX_CONN_INTERVAL = 12;  // 15ms
    const uint16_t SLAVE_LATENCY = 0;
    const uint16_t CONN_TIMEOUT = 100;      // 1000ms (units of 10ms)
}

// BLE packet format
struct BLEEMGPacket {
    uint8_t header;             // 0xA5 = data packet
    uint8_t packetId;           // Rolling packet ID
    uint32_t timestamp;         // Timestamp in ms
    uint8_t channelCount;       // Number of channels
    uint8_t sampleCount;        // Samples per channel in this packet
    uint16_t samples[];         // Interleaved samples [ch1_s1, ch2_s1, ch1_s2, ...]
} __attribute__((packed));

#define BLE_PACKET_HEADER 0xA5

// Status packet format
struct BLEStatusPacket {
    uint8_t header;             // 0xB5 = status packet
    uint8_t batteryLevel;       // 0-100%
    uint8_t connectionQuality;  // 0-100%
    uint8_t flags;              // Bit flags for status
    uint16_t sampleRate;        // Current sample rate
    uint16_t gain;              // Current gain setting
} __attribute__((packed));

#define BLE_STATUS_HEADER 0xB5

// Status flags
#define STATUS_FLAG_STREAMING   (1 << 0)
#define STATUS_FLAG_CALIBRATED  (1 << 1)
#define STATUS_FLAG_CHARGING    (1 << 2)
#define STATUS_FLAG_LOW_BATTERY (1 << 3)

// Command packet format
struct BLECommandPacket {
    uint8_t command;            // Command code
    uint8_t data[];             // Command-specific data
} __attribute__((packed));

#endif // BLE_SERVICE_H
