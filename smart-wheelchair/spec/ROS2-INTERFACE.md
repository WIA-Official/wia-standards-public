# WIA Smart Wheelchair ROS2 Interface Specification

## Version
- **Version**: 1.0.0
- **Date**: 2025-01-01
- **ROS2 Distribution**: Humble Hawksbill (LTS)
- **Status**: Draft

## 1. Overview

본 문서는 WIA Smart Wheelchair 시스템의 ROS2 인터페이스를 정의합니다.

### 1.1 Package Structure

```
wia_wheelchair/
├── wia_wheelchair_msgs/        # Custom messages and services
├── wia_wheelchair_driver/      # Hardware driver nodes
├── wia_wheelchair_control/     # Control algorithms
├── wia_wheelchair_navigation/  # Navigation stack integration
├── wia_wheelchair_description/ # URDF and robot description
└── wia_wheelchair_bringup/     # Launch files and configs
```

### 1.2 Namespace Convention

모든 WIA 휠체어 관련 토픽, 서비스, 액션은 `/wia_wheelchair/` 네임스페이스를 사용합니다.

## 2. Topics

### 2.1 State Topics (Published by Driver)

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/wia_wheelchair/state` | `wia_wheelchair_msgs/WheelchairState` | 50 Hz | 통합 휠체어 상태 |
| `/wia_wheelchair/odom` | `nav_msgs/Odometry` | 50 Hz | 오도메트리 데이터 |
| `/wia_wheelchair/battery` | `sensor_msgs/BatteryState` | 1 Hz | 배터리 상태 |
| `/wia_wheelchair/imu` | `sensor_msgs/Imu` | 100 Hz | IMU 데이터 |
| `/wia_wheelchair/joint_states` | `sensor_msgs/JointState` | 50 Hz | 휠 조인트 상태 |

### 2.2 Command Topics (Subscribed by Driver)

| Topic | Type | Description |
|-------|------|-------------|
| `/wia_wheelchair/cmd_vel` | `geometry_msgs/Twist` | 속도 명령 |
| `/wia_wheelchair/cmd_mode` | `std_msgs/UInt8` | 운행 모드 변경 |
| `/wia_wheelchair/motor_cmd` | `wia_wheelchair_msgs/MotorCommand` | 직접 모터 제어 |

### 2.3 Sensor Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/wia_wheelchair/ultrasonic/front` | `sensor_msgs/Range` | 20 Hz | 전방 초음파 |
| `/wia_wheelchair/ultrasonic/left` | `sensor_msgs/Range` | 20 Hz | 좌측 초음파 |
| `/wia_wheelchair/ultrasonic/right` | `sensor_msgs/Range` | 20 Hz | 우측 초음파 |
| `/wia_wheelchair/ultrasonic/rear` | `sensor_msgs/Range` | 20 Hz | 후방 초음파 |
| `/wia_wheelchair/scan` | `sensor_msgs/LaserScan` | 10 Hz | LiDAR 스캔 (옵션) |
| `/wia_wheelchair/seat_pressure` | `wia_wheelchair_msgs/SeatPressure` | 10 Hz | 착석 압력 |

### 2.4 Input Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/wia_wheelchair/joystick/raw` | `sensor_msgs/Joy` | 50 Hz | 원시 조이스틱 데이터 |
| `/wia_wheelchair/joystick/processed` | `geometry_msgs/Twist` | 50 Hz | 처리된 조이스틱 명령 |

### 2.5 Safety Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/wia_wheelchair/emergency_stop` | `std_msgs/Bool` | 비상 정지 상태 |
| `/wia_wheelchair/safety_status` | `wia_wheelchair_msgs/SafetyStatus` | 안전 시스템 상태 |
| `/wia_wheelchair/collision_warning` | `wia_wheelchair_msgs/CollisionWarning` | 충돌 경고 |

## 3. Services

### 3.1 Mode Control Services

| Service | Type | Description |
|---------|------|-------------|
| `/wia_wheelchair/set_mode` | `wia_wheelchair_msgs/SetMode` | 운행 모드 설정 |
| `/wia_wheelchair/get_mode` | `wia_wheelchair_msgs/GetMode` | 현재 모드 조회 |

**SetMode.srv:**
```
# Request
uint8 mode
# 0 = MANUAL
# 1 = ASSISTED
# 2 = AUTONOMOUS

---

# Response
bool success
string message
uint8 current_mode
```

### 3.2 Safety Services

| Service | Type | Description |
|---------|------|-------------|
| `/wia_wheelchair/emergency_stop` | `std_srvs/Trigger` | 비상 정지 활성화 |
| `/wia_wheelchair/reset_emergency` | `std_srvs/Trigger` | 비상 정지 해제 |
| `/wia_wheelchair/enable_motors` | `std_srvs/SetBool` | 모터 활성화/비활성화 |

### 3.3 Configuration Services

| Service | Type | Description |
|---------|------|-------------|
| `/wia_wheelchair/set_speed_limit` | `wia_wheelchair_msgs/SetSpeedLimit` | 속도 제한 설정 |
| `/wia_wheelchair/set_profile` | `wia_wheelchair_msgs/SetProfile` | 사용자 프로파일 적용 |
| `/wia_wheelchair/calibrate_joystick` | `std_srvs/Trigger` | 조이스틱 캘리브레이션 |

### 3.4 Diagnostic Services

| Service | Type | Description |
|---------|------|-------------|
| `/wia_wheelchair/get_diagnostics` | `wia_wheelchair_msgs/GetDiagnostics` | 진단 정보 조회 |
| `/wia_wheelchair/run_self_test` | `std_srvs/Trigger` | 자가 진단 실행 |

## 4. Actions

### 4.1 Navigation Actions

| Action | Type | Description |
|--------|------|-------------|
| `/wia_wheelchair/go_to_pose` | `wia_wheelchair_msgs/GoToPose` | 목표 위치 이동 |
| `/wia_wheelchair/follow_path` | `nav2_msgs/FollowPath` | 경로 추종 |
| `/wia_wheelchair/dock` | `wia_wheelchair_msgs/Dock` | 도킹 스테이션 이동 |

**GoToPose.action:**
```
# Goal
geometry_msgs/PoseStamped target_pose
float32 speed_factor   # 0.0-1.0, relative speed

---

# Result
bool success
string message
geometry_msgs/PoseStamped final_pose
float32 travel_time
float32 travel_distance

---

# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
float32 estimated_time_remaining
uint8 status   # 0=moving, 1=rotating, 2=avoiding_obstacle
```

## 5. Parameters

### 5.1 Driver Parameters

```yaml
/wia_wheelchair_driver:
  ros__parameters:
    # CAN Interface
    can_interface: "can0"
    can_bitrate: 500000

    # Wheel Configuration
    wheel_separation: 0.55          # meters
    wheel_radius: 0.15              # meters

    # Control Limits
    max_linear_velocity: 1.5        # m/s
    max_angular_velocity: 1.0       # rad/s
    max_linear_acceleration: 0.5    # m/s^2
    max_angular_acceleration: 1.0   # rad/s^2

    # Timeout Settings
    cmd_vel_timeout: 0.2            # seconds
    motor_timeout: 0.1              # seconds

    # Safety
    emergency_deceleration: 2.0     # m/s^2
```

### 5.2 Controller Parameters

```yaml
/wia_wheelchair_controller:
  ros__parameters:
    # PID Gains (velocity control)
    pid_linear:
      p: 1.0
      i: 0.1
      d: 0.05
    pid_angular:
      p: 2.0
      i: 0.2
      d: 0.1

    # Joystick Mapping
    joystick:
      deadzone: 0.1
      curve_exponent: 2.0           # response curve
      invert_x: false
      invert_y: false
```

### 5.3 Safety Parameters

```yaml
/wia_wheelchair_safety:
  ros__parameters:
    # Obstacle Avoidance
    min_obstacle_distance: 0.3      # meters
    slowdown_distance: 1.0          # meters
    emergency_stop_distance: 0.15   # meters

    # Tilt Protection
    max_tilt_angle: 15.0            # degrees
    tilt_warning_angle: 10.0        # degrees

    # Battery Protection
    low_battery_threshold: 20       # percent
    critical_battery_threshold: 10  # percent
```

## 6. TF Frames

### 6.1 Frame Hierarchy

```
map
└── odom
    └── base_footprint
        └── base_link
            ├── left_wheel_link
            ├── right_wheel_link
            ├── imu_link
            ├── joystick_link
            ├── ultrasonic_front_link
            ├── ultrasonic_left_link
            ├── ultrasonic_right_link
            ├── ultrasonic_rear_link
            ├── lidar_link (optional)
            └── seat_link
```

### 6.2 Frame Definitions

| Frame | Parent | Description |
|-------|--------|-------------|
| `base_footprint` | `odom` | 지면 투영점 |
| `base_link` | `base_footprint` | 휠체어 기준점 |
| `left_wheel_link` | `base_link` | 좌측 구동휠 |
| `right_wheel_link` | `base_link` | 우측 구동휠 |
| `imu_link` | `base_link` | IMU 센서 위치 |
| `seat_link` | `base_link` | 좌석 중심점 |

## 7. Launch Files

### 7.1 Bringup Launch

```python
# wia_wheelchair_bringup/launch/bringup.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Driver Node
        Node(
            package='wia_wheelchair_driver',
            executable='wheelchair_driver_node',
            name='wia_wheelchair_driver',
            namespace='wia_wheelchair',
            parameters=['config/driver_params.yaml'],
            remappings=[
                ('cmd_vel', '/wia_wheelchair/cmd_vel'),
                ('odom', '/wia_wheelchair/odom'),
            ]
        ),

        # Controller Node
        Node(
            package='wia_wheelchair_control',
            executable='wheelchair_controller_node',
            name='wia_wheelchair_controller',
            namespace='wia_wheelchair',
            parameters=['config/controller_params.yaml'],
        ),

        # Safety Node
        Node(
            package='wia_wheelchair_safety',
            executable='safety_node',
            name='wia_wheelchair_safety',
            namespace='wia_wheelchair',
            parameters=['config/safety_params.yaml'],
        ),
    ])
```

## 8. Quality of Service (QoS) Profiles

### 8.1 Recommended QoS Settings

| Topic Category | Reliability | Durability | History | Depth |
|----------------|-------------|------------|---------|-------|
| cmd_vel | RELIABLE | VOLATILE | KEEP_LAST | 1 |
| odom | RELIABLE | VOLATILE | KEEP_LAST | 10 |
| sensor data | BEST_EFFORT | VOLATILE | KEEP_LAST | 5 |
| safety topics | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 |

## 9. Message Definitions

모든 커스텀 메시지는 `wia_wheelchair_msgs` 패키지에 정의됩니다.
상세한 메시지 정의는 [MESSAGE-DEFINITIONS.md](./MESSAGE-DEFINITIONS.md)를 참조하세요.

## Appendix A: Standard Message Types Used

| Package | Message/Service | Usage |
|---------|-----------------|-------|
| geometry_msgs | Twist | 속도 명령 |
| geometry_msgs | PoseStamped | 목표 위치 |
| nav_msgs | Odometry | 오도메트리 |
| sensor_msgs | Imu | IMU 데이터 |
| sensor_msgs | Joy | 조이스틱 입력 |
| sensor_msgs | Range | 초음파 거리 |
| sensor_msgs | LaserScan | LiDAR 데이터 |
| sensor_msgs | BatteryState | 배터리 상태 |
| sensor_msgs | JointState | 조인트 상태 |
| std_msgs | Bool | 불린 상태 |
| std_msgs | UInt8 | 모드 값 |
| std_srvs | Trigger | 트리거 서비스 |
| std_srvs | SetBool | 활성화/비활성화 |

## Appendix B: Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-01-01 | WIA | Initial release |
