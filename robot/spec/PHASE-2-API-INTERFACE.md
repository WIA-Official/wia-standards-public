# WIA Robot - Phase 2: API Interface

## Version
- **Version**: 1.0.0
- **Date**: 2025-12-16
- **Status**: Final

## Overview

본 문서는 WIA Robot 접근성 표준의 API 인터페이스를 정의합니다.

## Rust SDK

완전한 Rust 구현이 제공됩니다:

**위치**: `api/rust/`

### 주요 컴포넌트

#### 1. 로봇 제어 접근성
- **음성 명령**: 자연어 로봇 제어
- **BCI 제어**: 뇌파로 로봇 조종
- **Eye Gaze**: 시선 추적 제어
- **AAC 통합**: AAC 디바이스 제어

#### 2. 안전 시스템
- **충돌 회피**: 장애물 감지 및 회피
- **긴급 정지**: 즉시 정지 기능
- **안전 경계**: 작동 범위 제한
- **모니터링**: 실시간 안전 감시

#### 3. 센서 피드백
- **햅틱**: 촉각 피드백
- **오디오**: 음성 안내
- **시각**: LED, 화면 표시
- **진동**: 진동 알림

#### 4. WIA 생태계 통합
- **Smart Wheelchair**: 휠체어 로봇 협업
- **Smart Home**: 홈 자동화 연동
- **AAC**: 의사소통 지원
- **Emergency**: 긴급 상황 대응

## API 구조

```rust
// 로봇 접근성 인터페이스
pub trait RobotAccessibility {
    fn voice_command(&mut self, command: &str) -> Result<Action>;
    fn bci_control(&mut self, signal: BCISignal) -> Result<Action>;
    fn eye_gaze_control(&mut self, point: Point2D) -> Result<Action>;
}

// 안전 시스템
pub trait SafetySystem {
    fn emergency_stop(&mut self) -> Result<()>;
    fn detect_obstacles(&self) -> Result<Vec<Obstacle>>;
    fn check_boundaries(&self, position: Position) -> Result<bool>;
}

// 피드백 시스템
pub trait FeedbackSystem {
    fn haptic_feedback(&mut self, pattern: HapticPattern) -> Result<()>;
    fn audio_feedback(&mut self, message: &str) -> Result<()>;
    fn visual_indicator(&mut self, status: Status) -> Result<()>;
}

// 로봇 제어
pub trait RobotControl {
    fn move_to(&mut self, target: Position) -> Result<()>;
    fn pick_object(&mut self, object_id: &str) -> Result<()>;
    fn release_object(&mut self) -> Result<()>;
    fn rotate(&mut self, angle: f32) -> Result<()>;
}
```

## 통신 프로토콜

### gRPC
```protobuf
service Robot {
  rpc SendCommand(CommandRequest) returns (CommandResponse);
  rpc GetStatus(StatusRequest) returns (StatusResponse);
  rpc EmergencyStop(EmptyRequest) returns (EmptyResponse);
}
```

### REST API
```
POST   /robot/command
GET    /robot/status
POST   /robot/stop
GET    /robot/sensors
PUT    /robot/accessibility/settings
```

### WebSocket
```
ws://robot.server/control
- 실시간 제어
- 센서 데이터 스트림
- 안전 알림
```

### MQTT
```
robot/command/#
robot/status/#
robot/sensor/#
robot/emergency/#
```

## 제어 모드

### 1. 음성 제어
```rust
let mut robot = Robot::new();
robot.voice_command("앞으로 이동")?;
robot.voice_command("물건 집어")?;
robot.voice_command("정지")?;
```

### 2. BCI 제어
```rust
let bci = BCIAdapter::new();
let signal = bci.read_signal()?;
robot.bci_control(signal)?;
```

### 3. Eye Gaze 제어
```rust
let eye_tracker = EyeTracker::new();
let point = eye_tracker.get_gaze_point()?;
robot.eye_gaze_control(point)?;
```

### 4. AAC 제어
```rust
let aac = AACDevice::new();
let command = aac.get_command()?;
robot.execute_command(command)?;
```

## 안전 기능

### 충돌 회피
```rust
impl SafetySystem for Robot {
    fn detect_obstacles(&self) -> Result<Vec<Obstacle>> {
        let obstacles = self.sensors.scan()?;
        Ok(obstacles.into_iter()
            .filter(|o| o.distance < SAFE_DISTANCE)
            .collect())
    }
}
```

### 긴급 정지
```rust
robot.emergency_stop()?;
// 모든 동작 즉시 중단
// 안전 상태로 전환
```

### 안전 경계
```rust
let boundary = SafeBoundary::new(
    min_x: -10.0,
    max_x: 10.0,
    min_y: -10.0,
    max_y: 10.0,
);
robot.set_boundary(boundary)?;
```

## 센서 통합

### 거리 센서
```rust
let distance = robot.sensors.distance()?;
if distance < SAFE_DISTANCE {
    robot.slow_down()?;
}
```

### 카메라
```rust
let image = robot.camera.capture()?;
let objects = robot.vision.detect_objects(image)?;
```

### 라이다
```rust
let scan = robot.lidar.scan()?;
let map = robot.mapping.create_map(scan)?;
```

## 에러 처리

```rust
pub enum RobotError {
    CommandFailed,
    ObstacleDetected,
    BoundaryViolation,
    SafetyViolation,
    CommunicationError,
}
```

## 예제

```rust
use wia_robot::*;

// 로봇 초기화
let mut robot = Robot::new();

// 접근성 프로필 적용
let profile = AccessibilityProfile::for_motor_impairment();
robot.apply_profile(profile)?;

// 음성 제어
robot.voice_command("앞으로 1미터 이동")?;

// 안전 확인
if robot.is_safe()? {
    robot.pick_object("bottle")?;
}

// 긴급 정지
robot.emergency_stop()?;
```

---

**Author**: Yeon Sam-Heum, Ph.D.  
**License**: MIT  
**弘益人間** - Benefit All Humanity
