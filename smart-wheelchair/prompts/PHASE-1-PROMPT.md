# WIA Smart Wheelchair - Phase 1: Communication Protocol

## 목표
휠체어 모듈 간 통신 표준을 정의합니다.

## 1.1 CAN 버스 메시지 표준

```typescript
interface CANMessage {
  id: number;                  // 11-bit or 29-bit
  dlc: number;                 // Data Length Code (0-8)
  data: Uint8Array;            // Payload
  timestamp: number;
}

// 메시지 ID 할당
enum CANMessageID {
  // 모터 제어 (0x100-0x1FF)
  MOTOR_LEFT_CMD = 0x100,
  MOTOR_RIGHT_CMD = 0x101,
  MOTOR_LEFT_STATUS = 0x110,
  MOTOR_RIGHT_STATUS = 0x111,

  // 조이스틱 입력 (0x200-0x2FF)
  JOYSTICK_XY = 0x200,
  JOYSTICK_BUTTON = 0x201,

  // 센서 (0x300-0x3FF)
  IMU_ACCEL = 0x300,
  IMU_GYRO = 0x301,
  ENCODER_LEFT = 0x310,
  ENCODER_RIGHT = 0x311,
  ULTRASONIC_FRONT = 0x320,

  // 상태 (0x400-0x4FF)
  BATTERY_STATUS = 0x400,
  SYSTEM_STATUS = 0x410,
  ERROR_CODE = 0x4F0,

  // 자율주행 (0x500-0x5FF)
  NAV_GOAL = 0x500,
  NAV_PATH = 0x501,
  OBSTACLE_ALERT = 0x510,
}
```

## 1.2 메시지 페이로드 정의

```typescript
// 모터 명령
interface MotorCommandPayload {
  mode: MotorMode;             // 1 byte
  speed: number;               // 2 bytes (signed, -32768 to 32767)
  acceleration: number;        // 2 bytes
  reserved: number;            // 3 bytes
}

enum MotorMode {
  COAST = 0,
  VELOCITY = 1,
  POSITION = 2,
  TORQUE = 3,
  BRAKE = 4,
}

// 조이스틱 입력
interface JoystickPayload {
  x: number;                   // 2 bytes (-32768 to 32767)
  y: number;                   // 2 bytes (-32768 to 32767)
  buttons: number;             // 2 bytes (bitmask)
  mode: number;                // 1 byte
  reserved: number;            // 1 byte
}

// 배터리 상태
interface BatteryStatusPayload {
  voltage: number;             // 2 bytes (mV)
  current: number;             // 2 bytes (mA, signed)
  soc: number;                 // 1 byte (0-100%)
  temperature: number;         // 1 byte (°C, signed)
  status: number;              // 1 byte (flags)
  reserved: number;            // 1 byte
}
```

## 1.3 ROS2 인터페이스

```python
# ROS2 메시지 정의
# wia_wheelchair_msgs/msg/WheelchairState.msg

std_msgs/Header header

# 운동 상태
geometry_msgs/Twist velocity
geometry_msgs/Pose pose

# 모터 상태
float32 left_motor_speed
float32 right_motor_speed
float32 left_motor_current
float32 right_motor_current

# 배터리
float32 battery_voltage
float32 battery_soc

# 상태 플래그
uint8 mode                     # 0=manual, 1=assisted, 2=autonomous
uint8 status                   # bitmask
```

## 1.4 토픽/서비스 구조

```yaml
# ROS2 토픽 네임스페이스: /wia_wheelchair/

# 상태 퍼블리시
/wia_wheelchair/state           # WheelchairState
/wia_wheelchair/odom            # nav_msgs/Odometry
/wia_wheelchair/battery         # sensor_msgs/BatteryState

# 명령 구독
/wia_wheelchair/cmd_vel         # geometry_msgs/Twist
/wia_wheelchair/cmd_mode        # std_msgs/UInt8

# 서비스
/wia_wheelchair/set_mode        # SetMode.srv
/wia_wheelchair/emergency_stop  # Trigger.srv
/wia_wheelchair/go_to_pose      # GoToPose.srv
```

---

## 산출물

```
smart-wheelchair/
├── spec/
│   ├── CAN-PROTOCOL.md
│   ├── ROS2-INTERFACE.md
│   └── MESSAGE-DEFINITIONS.md
├── ros2_ws/
│   └── src/
│       └── wia_wheelchair_msgs/
│           ├── msg/
│           │   ├── WheelchairState.msg
│           │   └── MotorCommand.msg
│           ├── srv/
│           │   └── SetMode.srv
│           └── CMakeLists.txt
├── firmware/
│   └── can_gateway/
```

---

## 다음: Phase 2 (센서 인터페이스)
