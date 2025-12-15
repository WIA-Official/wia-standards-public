# WIA Smart Wheelchair Message Definitions

## Version
- **Version**: 1.0.0
- **Date**: 2025-01-01
- **Status**: Draft

## 1. Overview

본 문서는 WIA Smart Wheelchair 시스템에서 사용하는 모든 커스텀 메시지, 서비스, 액션을 정의합니다.

## 2. Custom Messages

### 2.1 WheelchairState.msg

휠체어의 통합 상태를 나타내는 메시지입니다.

```
# WheelchairState.msg
# 휠체어 통합 상태 메시지

std_msgs/Header header

# 운동 상태
geometry_msgs/Twist velocity          # 현재 속도 (linear.x, angular.z)
geometry_msgs/Pose pose               # 현재 위치/자세

# 모터 상태
float32 left_motor_speed              # 좌측 모터 속도 (rad/s)
float32 right_motor_speed             # 우측 모터 속도 (rad/s)
float32 left_motor_current            # 좌측 모터 전류 (A)
float32 right_motor_current           # 우측 모터 전류 (A)
int8 left_motor_temperature           # 좌측 모터 온도 (°C)
int8 right_motor_temperature          # 우측 모터 온도 (°C)

# 배터리 상태
float32 battery_voltage               # 배터리 전압 (V)
float32 battery_current               # 배터리 전류 (A)
float32 battery_soc                   # 충전 상태 (0.0-1.0)
int8 battery_temperature              # 배터리 온도 (°C)

# 운행 모드
uint8 mode                            # 운행 모드
uint8 MODE_MANUAL = 0                 # 수동 조작
uint8 MODE_ASSISTED = 1               # 보조 주행
uint8 MODE_AUTONOMOUS = 2             # 자율 주행

# 상태 플래그
uint8 status                          # 상태 비트마스크
uint8 STATUS_MOTORS_ENABLED = 1       # Bit 0: 모터 활성화
uint8 STATUS_JOYSTICK_CONNECTED = 2   # Bit 1: 조이스틱 연결
uint8 STATUS_EMERGENCY_STOP = 4       # Bit 2: 비상 정지
uint8 STATUS_LOW_BATTERY = 8          # Bit 3: 배터리 부족
uint8 STATUS_OBSTACLE_DETECTED = 16   # Bit 4: 장애물 감지
uint8 STATUS_TILT_WARNING = 32        # Bit 5: 기울기 경고

# 에러 코드
uint16 error_code                     # 에러 코드 (0 = 에러 없음)
```

### 2.2 MotorCommand.msg

모터 직접 제어를 위한 명령 메시지입니다.

```
# MotorCommand.msg
# 모터 직접 제어 명령

std_msgs/Header header

# 제어 모드
uint8 mode
uint8 MODE_COAST = 0                  # 프리휠
uint8 MODE_VELOCITY = 1               # 속도 제어
uint8 MODE_POSITION = 2               # 위치 제어
uint8 MODE_TORQUE = 3                 # 토크 제어
uint8 MODE_BRAKE = 4                  # 브레이크

# 좌측 모터
float32 left_setpoint                 # 설정값 (모드에 따라 해석)
float32 left_acceleration             # 가속도 제한 (rad/s^2)

# 우측 모터
float32 right_setpoint                # 설정값 (모드에 따라 해석)
float32 right_acceleration            # 가속도 제한 (rad/s^2)

# 플래그
bool enable                           # 모터 활성화
bool sync                             # 양쪽 모터 동기화
```

### 2.3 MotorStatus.msg

개별 모터의 상태 메시지입니다.

```
# MotorStatus.msg
# 개별 모터 상태

std_msgs/Header header

# 식별
string motor_id                       # "left" 또는 "right"

# 현재 상태
uint8 mode                            # 현재 제어 모드
float32 velocity                      # 현재 속도 (rad/s)
float32 position                      # 현재 위치 (rad)
float32 torque                        # 현재 토크 (Nm)
float32 current                       # 전류 (A)

# 환경
int8 temperature                      # 온도 (°C)

# 상태
bool enabled                          # 활성화 여부
bool fault                            # 폴트 상태
uint8 error_code                      # 에러 코드
```

### 2.4 JoystickInput.msg

처리된 조이스틱 입력 메시지입니다.

```
# JoystickInput.msg
# 처리된 조이스틱 입력

std_msgs/Header header

# 위치 (정규화: -1.0 ~ 1.0)
float32 x                             # 좌우 방향
float32 y                             # 전후 방향

# 버튼 상태 (비트마스크)
uint16 buttons
uint16 BUTTON_1 = 1                   # 버튼 1 (보통 속도 부스트)
uint16 BUTTON_2 = 2                   # 버튼 2 (호른/경고음)
uint16 BUTTON_3 = 4                   # 버튼 3 (모드 변경)
uint16 BUTTON_4 = 8                   # 버튼 4 (비상 정지)
uint16 BUTTON_MODE_UP = 16            # 모드 업
uint16 BUTTON_MODE_DOWN = 32          # 모드 다운
uint16 BUTTON_PROFILE = 64            # 프로파일 선택

# 입력 모드
uint8 input_mode
uint8 INPUT_MODE_STANDARD = 0         # 표준
uint8 INPUT_MODE_FINE = 1             # 정밀 (감속)
uint8 INPUT_MODE_TURBO = 2            # 터보 (증속)

# 프로파일
uint8 profile_id                      # 사용자 프로파일 ID
```

### 2.5 SafetyStatus.msg

안전 시스템 상태 메시지입니다.

```
# SafetyStatus.msg
# 안전 시스템 상태

std_msgs/Header header

# 비상 정지
bool emergency_stop_active            # 비상 정지 활성화 여부
uint8 emergency_stop_source           # 비상 정지 원인
uint8 ESTOP_SOURCE_NONE = 0
uint8 ESTOP_SOURCE_BUTTON = 1         # 물리 버튼
uint8 ESTOP_SOURCE_REMOTE = 2         # 원격 명령
uint8 ESTOP_SOURCE_SOFTWARE = 3       # 소프트웨어 감지
uint8 ESTOP_SOURCE_SENSOR = 4         # 센서 이상

# 장애물 감지
bool obstacle_front                   # 전방 장애물
bool obstacle_left                    # 좌측 장애물
bool obstacle_right                   # 우측 장애물
bool obstacle_rear                    # 후방 장애물
float32 min_obstacle_distance         # 최소 장애물 거리 (m)

# 기울기 상태
float32 tilt_angle                    # 현재 기울기 (degrees)
bool tilt_warning                     # 기울기 경고
bool tilt_critical                    # 기울기 위험

# 시스템 상태
bool system_healthy                   # 전체 시스템 정상
uint8 safety_level                    # 안전 수준
uint8 SAFETY_LEVEL_NORMAL = 0         # 정상
uint8 SAFETY_LEVEL_CAUTION = 1        # 주의
uint8 SAFETY_LEVEL_WARNING = 2        # 경고
uint8 SAFETY_LEVEL_CRITICAL = 3       # 위험
```

### 2.6 CollisionWarning.msg

충돌 경고 메시지입니다.

```
# CollisionWarning.msg
# 충돌 경고

std_msgs/Header header

# 충돌 위험
bool collision_imminent               # 즉각적인 충돌 위험
float32 time_to_collision             # 충돌까지 예상 시간 (seconds)
float32 distance_to_obstacle          # 장애물까지 거리 (m)

# 위치 정보
geometry_msgs/Point obstacle_position # 장애물 위치 (base_link 기준)
float32 obstacle_angle                # 장애물 방향 (rad, 전방=0)

# 권장 동작
uint8 recommended_action
uint8 ACTION_NONE = 0                 # 조치 불필요
uint8 ACTION_SLOW_DOWN = 1            # 감속 권장
uint8 ACTION_STOP = 2                 # 정지 권장
uint8 ACTION_REVERSE = 3              # 후진 권장

# 경고 레벨
uint8 warning_level
uint8 LEVEL_INFO = 0                  # 정보
uint8 LEVEL_WARNING = 1               # 경고
uint8 LEVEL_DANGER = 2                # 위험
uint8 LEVEL_CRITICAL = 3              # 긴급
```

### 2.7 SeatPressure.msg

좌석 압력 센서 메시지입니다.

```
# SeatPressure.msg
# 좌석 압력 센서 데이터

std_msgs/Header header

# 착석 감지
bool occupied                         # 착석 여부
float32 total_pressure                # 총 압력 (N)

# 압력 분포 (4x4 매트릭스)
float32[16] pressure_map              # 압력 맵 (row-major)

# 무게 중심
geometry_msgs/Point center_of_pressure # 압력 중심 위치

# 착석 상태
uint8 posture_status
uint8 POSTURE_UNKNOWN = 0
uint8 POSTURE_NORMAL = 1              # 정상 착석
uint8 POSTURE_LEANING_LEFT = 2        # 좌측 쏠림
uint8 POSTURE_LEANING_RIGHT = 3       # 우측 쏠림
uint8 POSTURE_LEANING_FORWARD = 4     # 전방 쏠림
uint8 POSTURE_LEANING_BACK = 5        # 후방 쏠림
```

### 2.8 NavigationStatus.msg

자율 주행 상태 메시지입니다.

```
# NavigationStatus.msg
# 자율 주행 상태

std_msgs/Header header

# 현재 상태
uint8 status
uint8 STATUS_IDLE = 0                 # 대기
uint8 STATUS_NAVIGATING = 1           # 주행 중
uint8 STATUS_GOAL_REACHED = 2         # 목표 도달
uint8 STATUS_STUCK = 3                # 이동 불가
uint8 STATUS_RECOVERING = 4           # 복구 중
uint8 STATUS_PAUSED = 5               # 일시 정지

# 목표 정보
geometry_msgs/PoseStamped current_goal # 현재 목표
float32 distance_to_goal              # 목표까지 거리 (m)
float32 estimated_time                # 예상 도착 시간 (s)

# 경로 정보
uint32 waypoints_total                # 전체 웨이포인트 수
uint32 waypoints_completed            # 완료된 웨이포인트 수
float32 path_length                   # 경로 총 길이 (m)

# 진행률
float32 progress                      # 진행률 (0.0-1.0)
```

## 3. Services

### 3.1 SetMode.srv

운행 모드 설정 서비스입니다.

```
# SetMode.srv
# 운행 모드 설정

# Request
uint8 mode
uint8 MODE_MANUAL = 0
uint8 MODE_ASSISTED = 1
uint8 MODE_AUTONOMOUS = 2

---

# Response
bool success                          # 성공 여부
string message                        # 결과 메시지
uint8 current_mode                    # 현재 모드
```

### 3.2 GetMode.srv

현재 모드 조회 서비스입니다.

```
# GetMode.srv
# 현재 모드 조회

# Request
# (empty)

---

# Response
uint8 mode                            # 현재 모드
string mode_name                      # 모드 이름 문자열
bool motors_enabled                   # 모터 활성화 상태
```

### 3.3 SetSpeedLimit.srv

속도 제한 설정 서비스입니다.

```
# SetSpeedLimit.srv
# 속도 제한 설정

# Request
float32 max_linear_velocity           # 최대 선속도 (m/s)
float32 max_angular_velocity          # 최대 각속도 (rad/s)
float32 max_acceleration              # 최대 가속도 (m/s^2)

---

# Response
bool success
string message
float32 actual_max_linear             # 적용된 선속도 제한
float32 actual_max_angular            # 적용된 각속도 제한
float32 actual_max_accel              # 적용된 가속도 제한
```

### 3.4 SetProfile.srv

사용자 프로파일 적용 서비스입니다.

```
# SetProfile.srv
# 사용자 프로파일 적용

# Request
uint8 profile_id                      # 프로파일 ID (0-255)
string profile_name                   # 프로파일 이름 (빈 문자열이면 ID 사용)

---

# Response
bool success
string message
uint8 active_profile_id               # 활성화된 프로파일 ID
string active_profile_name            # 활성화된 프로파일 이름
```

### 3.5 GetDiagnostics.srv

진단 정보 조회 서비스입니다.

```
# GetDiagnostics.srv
# 진단 정보 조회

# Request
bool include_details                  # 상세 정보 포함 여부

---

# Response
bool system_ok                        # 시스템 정상 여부
diagnostic_msgs/DiagnosticStatus[] diagnostics  # 진단 상태 목록
string[] warnings                     # 경고 메시지
string[] errors                       # 에러 메시지
```

## 4. Actions

### 4.1 GoToPose.action

목표 위치로 이동하는 액션입니다.

```
# GoToPose.action
# 목표 위치 이동

# Goal
geometry_msgs/PoseStamped target_pose # 목표 위치
float32 speed_factor                  # 속도 계수 (0.0-1.0)
bool precise_arrival                  # 정밀 도착 여부

---

# Result
bool success                          # 성공 여부
string message                        # 결과 메시지
geometry_msgs/PoseStamped final_pose  # 최종 위치
float32 travel_time                   # 소요 시간 (s)
float32 travel_distance               # 이동 거리 (m)

---

# Feedback
geometry_msgs/PoseStamped current_pose # 현재 위치
float32 distance_remaining            # 남은 거리 (m)
float32 estimated_time_remaining      # 예상 남은 시간 (s)
uint8 status                          # 현재 상태
uint8 STATUS_MOVING = 0               # 이동 중
uint8 STATUS_ROTATING = 1             # 회전 중
uint8 STATUS_AVOIDING = 2             # 장애물 회피 중
uint8 STATUS_WAITING = 3              # 대기 중
```

### 4.2 Dock.action

도킹 스테이션으로 이동하는 액션입니다.

```
# Dock.action
# 도킹 스테이션 이동

# Goal
uint8 dock_id                         # 도킹 스테이션 ID
bool auto_detect                      # 자동 감지 여부

---

# Result
bool success                          # 성공 여부
string message                        # 결과 메시지
uint8 final_dock_id                   # 도킹된 스테이션 ID
bool charging                         # 충전 시작 여부

---

# Feedback
uint8 status
uint8 STATUS_SEARCHING = 0            # 스테이션 탐색 중
uint8 STATUS_APPROACHING = 1          # 접근 중
uint8 STATUS_ALIGNING = 2             # 정렬 중
uint8 STATUS_DOCKING = 3              # 도킹 중
float32 distance_to_dock              # 도킹까지 거리 (m)
```

## 5. Constants and Enumerations

### 5.1 Error Codes

```
# 에러 코드 정의

# No Error
ERROR_NONE = 0

# Communication Errors (0x01 - 0x0F)
ERROR_CAN_BUS_OFF = 0x01
ERROR_CAN_TIMEOUT = 0x02
ERROR_CAN_CHECKSUM = 0x03
ERROR_ROS_COMM = 0x04

# Motor Errors (0x10 - 0x1F)
ERROR_MOTOR_OVERCURRENT = 0x10
ERROR_MOTOR_OVERHEAT = 0x11
ERROR_MOTOR_ENCODER = 0x12
ERROR_MOTOR_STALL = 0x13

# Sensor Errors (0x20 - 0x2F)
ERROR_IMU_FAULT = 0x20
ERROR_ULTRASONIC_FAULT = 0x21
ERROR_LIDAR_FAULT = 0x22
ERROR_ENCODER_FAULT = 0x23

# Battery Errors (0x30 - 0x3F)
ERROR_BATTERY_LOW = 0x30
ERROR_BATTERY_CRITICAL = 0x31
ERROR_BATTERY_OVERVOLT = 0x32
ERROR_BATTERY_OVERCURRENT = 0x33
ERROR_BATTERY_OVERHEAT = 0x34

# Safety Errors (0x40 - 0x4F)
ERROR_ESTOP_ACTIVE = 0x40
ERROR_TILT_EXCEEDED = 0x41
ERROR_COLLISION_DETECTED = 0x42
ERROR_WATCHDOG_TIMEOUT = 0x43

# System Errors (0xF0 - 0xFF)
ERROR_SYSTEM_FAULT = 0xF0
ERROR_CONFIG_INVALID = 0xF1
ERROR_FIRMWARE_MISMATCH = 0xF2
```

## 6. TypeScript Interface Definitions

CAN-ROS2 브리지 구현을 위한 TypeScript 인터페이스입니다.

```typescript
// WheelchairState interface
export interface WheelchairState {
  header: Header;
  velocity: Twist;
  pose: Pose;
  leftMotorSpeed: number;
  rightMotorSpeed: number;
  leftMotorCurrent: number;
  rightMotorCurrent: number;
  leftMotorTemperature: number;
  rightMotorTemperature: number;
  batteryVoltage: number;
  batteryCurrent: number;
  batterySoc: number;
  batteryTemperature: number;
  mode: WheelchairMode;
  status: number;
  errorCode: number;
}

export enum WheelchairMode {
  MANUAL = 0,
  ASSISTED = 1,
  AUTONOMOUS = 2,
}

export enum MotorControlMode {
  COAST = 0,
  VELOCITY = 1,
  POSITION = 2,
  TORQUE = 3,
  BRAKE = 4,
}

// MotorCommand interface
export interface MotorCommand {
  header: Header;
  mode: MotorControlMode;
  leftSetpoint: number;
  leftAcceleration: number;
  rightSetpoint: number;
  rightAcceleration: number;
  enable: boolean;
  sync: boolean;
}

// SafetyStatus interface
export interface SafetyStatus {
  header: Header;
  emergencyStopActive: boolean;
  emergencyStopSource: EmergencyStopSource;
  obstacleFront: boolean;
  obstacleLeft: boolean;
  obstacleRight: boolean;
  obstacleRear: boolean;
  minObstacleDistance: number;
  tiltAngle: number;
  tiltWarning: boolean;
  tiltCritical: boolean;
  systemHealthy: boolean;
  safetyLevel: SafetyLevel;
}

export enum EmergencyStopSource {
  NONE = 0,
  BUTTON = 1,
  REMOTE = 2,
  SOFTWARE = 3,
  SENSOR = 4,
}

export enum SafetyLevel {
  NORMAL = 0,
  CAUTION = 1,
  WARNING = 2,
  CRITICAL = 3,
}
```

## Appendix A: Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-01-01 | WIA | Initial release |
