# WIA Smart Wheelchair CAN Protocol Specification

## Version
- **Version**: 1.0.0
- **Date**: 2025-01-01
- **Status**: Draft

## 1. Overview

본 문서는 WIA Smart Wheelchair 시스템의 CAN(Controller Area Network) 버스 통신 프로토콜을 정의합니다.

### 1.1 Physical Layer

| Parameter | Value |
|-----------|-------|
| Standard | CAN 2.0B |
| Baud Rate | 500 kbps (default), 250 kbps (low-speed mode) |
| Identifier | 11-bit (standard), 29-bit (extended) |
| Max Data Length | 8 bytes |
| Termination | 120Ω at both ends |

### 1.2 Bus Topology

```
┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐
│  Main    │───│  Motor   │───│  Sensor  │───│  Safety  │
│Controller│   │Controller│   │  Hub     │   │  Module  │
└──────────┘   └──────────┘   └──────────┘   └──────────┘
     │              │              │              │
     └──────────────┴──────────────┴──────────────┘
                     CAN Bus (500 kbps)
```

## 2. Message ID Allocation

### 2.1 ID Range Assignment

| Range | Category | Description |
|-------|----------|-------------|
| 0x000-0x0FF | System | 시스템 제어 및 동기화 |
| 0x100-0x1FF | Motor | 모터 제어 및 상태 |
| 0x200-0x2FF | Input | 조이스틱, 버튼 입력 |
| 0x300-0x3FF | Sensor | IMU, 엔코더, 초음파 센서 |
| 0x400-0x4FF | Status | 배터리, 시스템 상태 |
| 0x500-0x5FF | Navigation | 자율주행 관련 |
| 0x600-0x6FF | Safety | 안전 시스템 |
| 0x700-0x7FF | Diagnostic | 진단 및 디버그 |

### 2.2 Message ID Definitions

```c
// System Messages (0x000-0x0FF)
#define CAN_ID_HEARTBEAT        0x001   // 10Hz, node alive signal
#define CAN_ID_SYNC             0x002   // Synchronization signal
#define CAN_ID_TIME_STAMP       0x010   // System timestamp

// Motor Control (0x100-0x1FF)
#define CAN_ID_MOTOR_LEFT_CMD   0x100   // Left motor command
#define CAN_ID_MOTOR_RIGHT_CMD  0x101   // Right motor command
#define CAN_ID_MOTOR_LEFT_STS   0x110   // Left motor status
#define CAN_ID_MOTOR_RIGHT_STS  0x111   // Right motor status
#define CAN_ID_MOTOR_CONFIG     0x120   // Motor configuration
#define CAN_ID_MOTOR_LIMIT      0x121   // Speed/torque limits

// Input Devices (0x200-0x2FF)
#define CAN_ID_JOYSTICK_XY      0x200   // Joystick X/Y position
#define CAN_ID_JOYSTICK_BTN     0x201   // Joystick buttons
#define CAN_ID_SIPSUFF          0x210   // Sip-and-puff input
#define CAN_ID_HEAD_ARRAY       0x220   // Head array input

// Sensors (0x300-0x3FF)
#define CAN_ID_IMU_ACCEL        0x300   // IMU accelerometer
#define CAN_ID_IMU_GYRO         0x301   // IMU gyroscope
#define CAN_ID_IMU_MAG          0x302   // IMU magnetometer
#define CAN_ID_ENCODER_LEFT     0x310   // Left wheel encoder
#define CAN_ID_ENCODER_RIGHT    0x311   // Right wheel encoder
#define CAN_ID_ULTRASONIC_0     0x320   // Ultrasonic sensor 0 (front)
#define CAN_ID_ULTRASONIC_1     0x321   // Ultrasonic sensor 1 (left)
#define CAN_ID_ULTRASONIC_2     0x322   // Ultrasonic sensor 2 (right)
#define CAN_ID_ULTRASONIC_3     0x323   // Ultrasonic sensor 3 (rear)
#define CAN_ID_LIDAR_ZONE       0x330   // LiDAR zone detection
#define CAN_ID_SEAT_PRESSURE    0x340   // Seat pressure sensors

// Status (0x400-0x4FF)
#define CAN_ID_BATTERY_STATUS   0x400   // Battery state
#define CAN_ID_BATTERY_DETAIL   0x401   // Detailed battery info
#define CAN_ID_SYSTEM_STATUS    0x410   // Overall system status
#define CAN_ID_TEMP_MONITOR     0x420   // Temperature monitoring
#define CAN_ID_ERROR_CODE       0x4F0   // Error codes

// Navigation (0x500-0x5FF)
#define CAN_ID_NAV_GOAL         0x500   // Navigation goal
#define CAN_ID_NAV_PATH         0x501   // Path segment
#define CAN_ID_NAV_STATUS       0x502   // Navigation status
#define CAN_ID_OBSTACLE_ALERT   0x510   // Obstacle detection
#define CAN_ID_LOCALIZATION     0x520   // Position estimate

// Safety (0x600-0x6FF)
#define CAN_ID_ESTOP            0x600   // Emergency stop
#define CAN_ID_SAFETY_STATUS    0x610   // Safety system status
#define CAN_ID_COLLISION_WARN   0x620   // Collision warning
#define CAN_ID_TILT_ALERT       0x630   // Tilt/tip-over alert

// Diagnostic (0x700-0x7FF)
#define CAN_ID_DIAG_REQUEST     0x700   // Diagnostic request
#define CAN_ID_DIAG_RESPONSE    0x701   // Diagnostic response
#define CAN_ID_FIRMWARE_INFO    0x710   // Firmware version
```

## 3. Message Formats

### 3.1 Heartbeat Message (0x001)

주기: 100ms (10Hz)

| Byte | Field | Description |
|------|-------|-------------|
| 0 | node_id | Node identifier (0-255) |
| 1 | state | Node state (0=boot, 1=ready, 2=operational, 3=error) |
| 2-3 | uptime | Uptime in seconds (uint16) |
| 4-5 | reserved | Reserved for future use |
| 6 | error_count | Error count since boot |
| 7 | checksum | XOR checksum |

### 3.2 Motor Command (0x100, 0x101)

주기: 50Hz (명령 전송 시)

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | mode | uint8 | 0=coast, 1=velocity, 2=position, 3=torque, 4=brake |
| 1-2 | setpoint | int16 | Speed (rpm) or Position (0.1°) or Torque (mNm) |
| 3-4 | acceleration | uint16 | Acceleration limit (rpm/s) |
| 5 | flags | uint8 | Bit0=enable, Bit1=direction_lock |
| 6-7 | reserved | - | Reserved |

### 3.3 Motor Status (0x110, 0x111)

주기: 50Hz

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | mode | uint8 | Current operating mode |
| 1-2 | speed | int16 | Actual speed (rpm) |
| 3-4 | current | int16 | Motor current (mA) |
| 5 | temperature | int8 | Motor temperature (°C) |
| 6 | status | uint8 | Status flags |
| 7 | error | uint8 | Error code |

### 3.4 Joystick XY (0x200)

주기: 50Hz

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0-1 | x | int16 | X-axis (-32768 to +32767) |
| 2-3 | y | int16 | Y-axis (-32768 to +32767) |
| 4-5 | buttons | uint16 | Button bitmask |
| 6 | mode | uint8 | Input mode (0=standard, 1=fine, 2=turbo) |
| 7 | profile | uint8 | User profile ID |

### 3.5 IMU Data (0x300, 0x301)

주기: 100Hz

**Accelerometer (0x300):**
| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0-1 | accel_x | int16 | X acceleration (mg) |
| 2-3 | accel_y | int16 | Y acceleration (mg) |
| 4-5 | accel_z | int16 | Z acceleration (mg) |
| 6-7 | reserved | - | Reserved |

**Gyroscope (0x301):**
| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0-1 | gyro_x | int16 | X angular rate (0.1 °/s) |
| 2-3 | gyro_y | int16 | Y angular rate (0.1 °/s) |
| 4-5 | gyro_z | int16 | Z angular rate (0.1 °/s) |
| 6-7 | reserved | - | Reserved |

### 3.6 Battery Status (0x400)

주기: 1Hz

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0-1 | voltage | uint16 | Battery voltage (mV) |
| 2-3 | current | int16 | Current draw (mA, + discharge, - charge) |
| 4 | soc | uint8 | State of charge (0-100%) |
| 5 | temperature | int8 | Battery temperature (°C) |
| 6 | status | uint8 | Bit0=charging, Bit1=low, Bit2=critical |
| 7 | health | uint8 | Battery health (0-100%) |

### 3.7 Emergency Stop (0x600)

주기: 이벤트 기반 (변경 시 즉시 + 100ms 주기 반복)

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | source | uint8 | E-stop source (0=button, 1=remote, 2=software, 3=sensor) |
| 1 | state | uint8 | 0=released, 1=engaged |
| 2-3 | timestamp | uint16 | Time since engagement (ms) |
| 4-7 | reserved | - | Reserved |

## 4. Node Addressing

### 4.1 Node ID Assignment

| Node ID | Module | Description |
|---------|--------|-------------|
| 0x01 | Main Controller | 중앙 제어 유닛 |
| 0x10 | Motor Controller Left | 좌측 모터 컨트롤러 |
| 0x11 | Motor Controller Right | 우측 모터 컨트롤러 |
| 0x20 | Input Module | 조이스틱/입력 모듈 |
| 0x30 | Sensor Hub | 센서 통합 모듈 |
| 0x40 | Power Management | 전원 관리 모듈 |
| 0x50 | Navigation Module | 자율주행 모듈 |
| 0x60 | Safety Controller | 안전 시스템 컨트롤러 |

## 5. Error Handling

### 5.1 Error Codes (0x4F0)

| Code | Category | Description |
|------|----------|-------------|
| 0x00 | - | No error |
| 0x01-0x0F | Communication | CAN bus errors |
| 0x10-0x1F | Motor | Motor controller errors |
| 0x20-0x2F | Sensor | Sensor failures |
| 0x30-0x3F | Battery | Power system errors |
| 0x40-0x4F | Safety | Safety system alerts |
| 0xF0-0xFF | Critical | Critical system failures |

### 5.2 Bus Error Recovery

1. **Bus-off Recovery**: 자동 복구 시도 (128번 11 recessive bits 후)
2. **Message Timeout**: 500ms 내 heartbeat 미수신 시 노드 오프라인 처리
3. **Error Frame**: 연속 5회 오류 시 해당 메시지 비활성화

## 6. Timing Requirements

### 6.1 Message Priorities

| Priority | ID Range | Max Latency | Description |
|----------|----------|-------------|-------------|
| Highest | 0x000-0x0FF | < 1ms | System critical |
| High | 0x100-0x1FF | < 5ms | Motor control |
| Medium | 0x200-0x3FF | < 10ms | Input/Sensors |
| Low | 0x400-0x7FF | < 50ms | Status/Diagnostic |

### 6.2 Update Rates

| Message | Rate | Notes |
|---------|------|-------|
| Motor Command | 50 Hz | Real-time control |
| Motor Status | 50 Hz | Feedback loop |
| Joystick | 50 Hz | User input |
| IMU | 100 Hz | Inertial data |
| Encoders | 100 Hz | Odometry |
| Battery | 1 Hz | Slow-changing |
| Heartbeat | 10 Hz | Node monitoring |

## 7. Safety Considerations

### 7.1 Fail-Safe Behaviors

1. **Communication Loss**: 200ms 명령 미수신 시 감속 정지
2. **E-Stop**: 최우선 순위, 즉시 모터 브레이크
3. **Low Battery**: 10% 미만 시 속도 제한, 5% 미만 시 운행 불가
4. **Tilt Warning**: 15° 이상 기울기 감지 시 경고

### 7.2 Redundancy

- Dual CAN bus option for safety-critical applications
- Watchdog timeout on all nodes
- Checksum verification for critical messages

## Appendix A: TypeScript Interface

```typescript
export interface CANMessage {
  id: number;          // 11-bit or 29-bit identifier
  dlc: number;         // Data Length Code (0-8)
  data: Uint8Array;    // Payload (max 8 bytes)
  timestamp: number;   // Microseconds since epoch
  extended: boolean;   // Extended frame flag
}

export enum CANMessageID {
  // System
  HEARTBEAT = 0x001,
  SYNC = 0x002,

  // Motor Control
  MOTOR_LEFT_CMD = 0x100,
  MOTOR_RIGHT_CMD = 0x101,
  MOTOR_LEFT_STATUS = 0x110,
  MOTOR_RIGHT_STATUS = 0x111,

  // Input
  JOYSTICK_XY = 0x200,
  JOYSTICK_BUTTON = 0x201,

  // Sensors
  IMU_ACCEL = 0x300,
  IMU_GYRO = 0x301,
  ENCODER_LEFT = 0x310,
  ENCODER_RIGHT = 0x311,
  ULTRASONIC_FRONT = 0x320,

  // Status
  BATTERY_STATUS = 0x400,
  SYSTEM_STATUS = 0x410,
  ERROR_CODE = 0x4F0,

  // Navigation
  NAV_GOAL = 0x500,
  NAV_PATH = 0x501,
  OBSTACLE_ALERT = 0x510,

  // Safety
  ESTOP = 0x600,
  SAFETY_STATUS = 0x610,
}
```

## Appendix B: Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-01-01 | WIA | Initial release |
