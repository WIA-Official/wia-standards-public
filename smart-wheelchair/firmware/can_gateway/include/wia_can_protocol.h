/**
 * @file wia_can_protocol.h
 * @brief WIA Smart Wheelchair CAN Protocol Definitions
 * @version 1.0.0
 *
 * This header defines all CAN message IDs, structures, and constants
 * for the WIA Smart Wheelchair communication protocol.
 */

#ifndef WIA_CAN_PROTOCOL_H
#define WIA_CAN_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * CAN Configuration
 ******************************************************************************/

#define WIA_CAN_BITRATE         500000      // 500 kbps
#define WIA_CAN_BITRATE_LOW     250000      // 250 kbps (low-speed mode)
#define WIA_CAN_MAX_DLC         8           // Maximum data length

/*******************************************************************************
 * Message ID Definitions
 ******************************************************************************/

// System Messages (0x000-0x0FF)
#define CAN_ID_HEARTBEAT        0x001
#define CAN_ID_SYNC             0x002
#define CAN_ID_TIMESTAMP        0x010

// Motor Control (0x100-0x1FF)
#define CAN_ID_MOTOR_LEFT_CMD   0x100
#define CAN_ID_MOTOR_RIGHT_CMD  0x101
#define CAN_ID_MOTOR_LEFT_STS   0x110
#define CAN_ID_MOTOR_RIGHT_STS  0x111
#define CAN_ID_MOTOR_CONFIG     0x120
#define CAN_ID_MOTOR_LIMIT      0x121

// Input Devices (0x200-0x2FF)
#define CAN_ID_JOYSTICK_XY      0x200
#define CAN_ID_JOYSTICK_BTN     0x201
#define CAN_ID_SIPSUFF          0x210
#define CAN_ID_HEAD_ARRAY       0x220

// Sensors (0x300-0x3FF)
#define CAN_ID_IMU_ACCEL        0x300
#define CAN_ID_IMU_GYRO         0x301
#define CAN_ID_IMU_MAG          0x302
#define CAN_ID_ENCODER_LEFT     0x310
#define CAN_ID_ENCODER_RIGHT    0x311
#define CAN_ID_ULTRASONIC_0     0x320
#define CAN_ID_ULTRASONIC_1     0x321
#define CAN_ID_ULTRASONIC_2     0x322
#define CAN_ID_ULTRASONIC_3     0x323
#define CAN_ID_LIDAR_ZONE       0x330
#define CAN_ID_SEAT_PRESSURE    0x340

// Status (0x400-0x4FF)
#define CAN_ID_BATTERY_STATUS   0x400
#define CAN_ID_BATTERY_DETAIL   0x401
#define CAN_ID_SYSTEM_STATUS    0x410
#define CAN_ID_TEMP_MONITOR     0x420
#define CAN_ID_ERROR_CODE       0x4F0

// Navigation (0x500-0x5FF)
#define CAN_ID_NAV_GOAL         0x500
#define CAN_ID_NAV_PATH         0x501
#define CAN_ID_NAV_STATUS       0x502
#define CAN_ID_OBSTACLE_ALERT   0x510
#define CAN_ID_LOCALIZATION     0x520

// Safety (0x600-0x6FF)
#define CAN_ID_ESTOP            0x600
#define CAN_ID_SAFETY_STATUS    0x610
#define CAN_ID_COLLISION_WARN   0x620
#define CAN_ID_TILT_ALERT       0x630

// Diagnostic (0x700-0x7FF)
#define CAN_ID_DIAG_REQUEST     0x700
#define CAN_ID_DIAG_RESPONSE    0x701
#define CAN_ID_FIRMWARE_INFO    0x710

/*******************************************************************************
 * Node ID Definitions
 ******************************************************************************/

#define NODE_ID_MAIN_CTRL       0x01
#define NODE_ID_MOTOR_LEFT      0x10
#define NODE_ID_MOTOR_RIGHT     0x11
#define NODE_ID_INPUT_MODULE    0x20
#define NODE_ID_SENSOR_HUB      0x30
#define NODE_ID_POWER_MGMT      0x40
#define NODE_ID_NAV_MODULE      0x50
#define NODE_ID_SAFETY_CTRL     0x60

/*******************************************************************************
 * Motor Control Mode
 ******************************************************************************/

typedef enum {
    MOTOR_MODE_COAST    = 0,    // Free wheel (no current)
    MOTOR_MODE_VELOCITY = 1,    // Velocity control (rad/s)
    MOTOR_MODE_POSITION = 2,    // Position control (rad)
    MOTOR_MODE_TORQUE   = 3,    // Torque control (Nm)
    MOTOR_MODE_BRAKE    = 4     // Active brake
} wia_motor_mode_t;

/*******************************************************************************
 * Node State
 ******************************************************************************/

typedef enum {
    NODE_STATE_BOOT         = 0,
    NODE_STATE_READY        = 1,
    NODE_STATE_OPERATIONAL  = 2,
    NODE_STATE_ERROR        = 3
} wia_node_state_t;

/*******************************************************************************
 * Emergency Stop Source
 ******************************************************************************/

typedef enum {
    ESTOP_SOURCE_NONE       = 0,
    ESTOP_SOURCE_BUTTON     = 1,
    ESTOP_SOURCE_REMOTE     = 2,
    ESTOP_SOURCE_SOFTWARE   = 3,
    ESTOP_SOURCE_SENSOR     = 4
} wia_estop_source_t;

/*******************************************************************************
 * Message Structures
 ******************************************************************************/

#pragma pack(push, 1)

// Heartbeat message (8 bytes)
typedef struct {
    uint8_t  node_id;           // Node identifier
    uint8_t  state;             // Node state (wia_node_state_t)
    uint16_t uptime;            // Uptime in seconds
    uint16_t reserved;          // Reserved
    uint8_t  error_count;       // Error count since boot
    uint8_t  checksum;          // XOR checksum
} wia_can_heartbeat_t;

// Motor command message (8 bytes)
typedef struct {
    uint8_t  mode;              // Control mode (wia_motor_mode_t)
    int16_t  setpoint;          // Setpoint value
    uint16_t acceleration;      // Acceleration limit
    uint8_t  flags;             // Bit0=enable, Bit1=direction_lock
    uint16_t reserved;          // Reserved
} wia_can_motor_cmd_t;

// Motor status message (8 bytes)
typedef struct {
    uint8_t  mode;              // Current mode
    int16_t  speed;             // Actual speed (rpm)
    int16_t  current;           // Motor current (mA)
    int8_t   temperature;       // Temperature (°C)
    uint8_t  status;            // Status flags
    uint8_t  error;             // Error code
} wia_can_motor_status_t;

// Joystick XY message (8 bytes)
typedef struct {
    int16_t  x;                 // X-axis (-32768 to +32767)
    int16_t  y;                 // Y-axis (-32768 to +32767)
    uint16_t buttons;           // Button bitmask
    uint8_t  mode;              // Input mode
    uint8_t  profile;           // User profile ID
} wia_can_joystick_t;

// IMU accelerometer message (8 bytes)
typedef struct {
    int16_t  accel_x;           // X acceleration (mg)
    int16_t  accel_y;           // Y acceleration (mg)
    int16_t  accel_z;           // Z acceleration (mg)
    uint16_t reserved;          // Reserved
} wia_can_imu_accel_t;

// IMU gyroscope message (8 bytes)
typedef struct {
    int16_t  gyro_x;            // X angular rate (0.1 °/s)
    int16_t  gyro_y;            // Y angular rate (0.1 °/s)
    int16_t  gyro_z;            // Z angular rate (0.1 °/s)
    uint16_t reserved;          // Reserved
} wia_can_imu_gyro_t;

// Battery status message (8 bytes)
typedef struct {
    uint16_t voltage;           // Battery voltage (mV)
    int16_t  current;           // Current (mA, +discharge, -charge)
    uint8_t  soc;               // State of charge (0-100%)
    int8_t   temperature;       // Temperature (°C)
    uint8_t  status;            // Status flags
    uint8_t  health;            // Battery health (0-100%)
} wia_can_battery_t;

// Emergency stop message (8 bytes)
typedef struct {
    uint8_t  source;            // E-stop source
    uint8_t  state;             // 0=released, 1=engaged
    uint16_t timestamp;         // Time since engagement (ms)
    uint32_t reserved;          // Reserved
} wia_can_estop_t;

// Ultrasonic sensor message (8 bytes)
typedef struct {
    uint16_t distance;          // Distance (mm)
    uint8_t  status;            // Sensor status
    uint8_t  confidence;        // Measurement confidence (0-100%)
    uint32_t timestamp;         // Timestamp (us)
} wia_can_ultrasonic_t;

// Encoder message (8 bytes)
typedef struct {
    int32_t  position;          // Encoder position (ticks)
    int16_t  velocity;          // Velocity (ticks/s)
    uint16_t status;            // Status flags
} wia_can_encoder_t;

#pragma pack(pop)

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/**
 * @brief Calculate XOR checksum for CAN message
 * @param data Pointer to data buffer
 * @param len Data length (excluding checksum byte)
 * @return XOR checksum
 */
static inline uint8_t wia_can_checksum(const uint8_t *data, uint8_t len) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief Check if CAN ID is in valid range for category
 */
static inline int wia_can_id_is_motor(uint32_t id) {
    return (id >= 0x100 && id <= 0x1FF);
}

static inline int wia_can_id_is_sensor(uint32_t id) {
    return (id >= 0x300 && id <= 0x3FF);
}

static inline int wia_can_id_is_safety(uint32_t id) {
    return (id >= 0x600 && id <= 0x6FF);
}

#ifdef __cplusplus
}
#endif

#endif // WIA_CAN_PROTOCOL_H
