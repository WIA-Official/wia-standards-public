# Phase 2: API Interface Standard (Rust)
## Claude Code 작업 프롬프트

---

**Standard**: WIA Robot (Robotics Accessibility)
**Phase**: 2 of 4
**Language**: **Rust** (Primary)
**목표**: Rust 기반 고성능 보조 로봇 API 구현
**난이도**: ★★★★★
**예상 작업량**: Rust 라이브러리 + 테스트 + 예제

---

## 🎯 Phase 2 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의했다.

 이제 이 데이터를 프로그래밍 방식으로 어떻게 다룰 것인가?

 - 외골격 로봇의 보행 패턴 계산?
 - 의수의 EMG 신호 처리 및 그립 제어?
 - 재활 로봇의 운동 궤적 계획?
 - 돌봄 로봇의 감정 인식 및 응답?

 모든 계산과 제어를 표준 API로 제공할 수 있을까?"
```

### 목표
```
보조 로봇 데이터를 처리하는 Rust API 구현

- 데이터 타입 정의 (Phase 1 스키마 기반)
- 핵심 제어 알고리즘 구현
- 센서 데이터 처리
- 안전 검증 함수
- 실시간 처리 보장
- ROS2 연동 준비
```

---

## 🦀 Rust 선택 이유

```
1. 성능: C++ 수준 속도 (실시간 로봇 제어 필수)
2. 안전: 메모리 안전 보장 (의료/보조 기기의 필수 요구사항)
3. 동시성: 멀티스레드 안전 (센서 다중 처리)
4. 정밀도: f64 고정밀 연산 (로봇 제어)
5. 크로스 플랫폼: 임베디드부터 클라우드까지
6. 일관성: WIA 표준 전체에서 Rust 사용
```

---

## 📦 프로젝트 구조

```
/api/rust/
├── Cargo.toml
├── src/
│   ├── lib.rs               # 메인 라이브러리
│   ├── types.rs             # 타입 정의
│   ├── error.rs             # 에러 타입
│   ├── safety.rs            # 안전 검증
│   ├── core/
│   │   ├── mod.rs
│   │   ├── device.rs        # 디바이스 관리
│   │   ├── control.rs       # 제어 시스템
│   │   ├── sensor.rs        # 센서 처리
│   │   └── actuator.rs      # 액추에이터 제어
│   ├── adapters/
│   │   ├── mod.rs
│   │   ├── exoskeleton.rs   # 외골격 로봇
│   │   ├── prosthetics.rs   # 의수/의족
│   │   ├── rehabilitation.rs # 재활 로봇
│   │   ├── care.rs          # 돌봄 로봇
│   │   ├── surgical.rs      # 수술 보조 로봇
│   │   └── mobility.rs      # 이동 보조 로봇
│   ├── algorithms/
│   │   ├── mod.rs
│   │   ├── gait.rs          # 보행 패턴
│   │   ├── trajectory.rs    # 궤적 계획
│   │   ├── emg.rs           # EMG 신호 처리
│   │   └── kinematics.rs    # 역기구학
│   └── prelude.rs           # 편의 re-exports
├── tests/
│   └── integration_test.rs
└── examples/
    ├── basic_usage.rs
    ├── exoskeleton_control.rs
    ├── prosthetic_emg.rs
    └── rehabilitation_session.rs
```

---

## 🔧 핵심 구현 코드

### Error 타입 (error.rs)
```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum RobotError {
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    #[error("Safety violation: {0}")]
    SafetyViolation(String),

    #[error("Control error: {0}")]
    ControlError(String),

    #[error("Sensor error: {0}")]
    SensorError(String),

    #[error("Actuator error: {0}")]
    ActuatorError(String),

    #[error("Communication error: {0}")]
    CommunicationError(String),

    #[error("Calibration required: {0}")]
    CalibrationRequired(String),

    #[error("Emergency stop activated")]
    EmergencyStop,

    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

pub type RobotResult<T> = std::result::Result<T, RobotError>;
```

### 안전 시스템 (safety.rs)
```rust
use crate::{RobotResult, RobotError};
use serde::{Deserialize, Serialize};

/// 안전 상태
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyState {
    pub emergency_stop: bool,
    pub fall_detection: bool,
    pub collision_avoidance: bool,
    pub vital_signs_ok: bool,
    pub workspace_boundary_ok: bool,
    pub force_limit_ok: bool,
}

impl SafetyState {
    /// 안전 검사
    pub fn is_safe(&self) -> bool {
        !self.emergency_stop
            && !self.fall_detection
            && self.vital_signs_ok
            && self.workspace_boundary_ok
            && self.force_limit_ok
    }

    /// 안전 상태 검증
    pub fn validate(&self) -> RobotResult<()> {
        if self.emergency_stop {
            return Err(RobotError::EmergencyStop);
        }

        if self.fall_detection {
            return Err(RobotError::SafetyViolation(
                "Fall detected - operation halted".into()
            ));
        }

        if !self.vital_signs_ok {
            return Err(RobotError::SafetyViolation(
                "Vital signs abnormal".into()
            ));
        }

        if !self.workspace_boundary_ok {
            return Err(RobotError::SafetyViolation(
                "Workspace boundary exceeded".into()
            ));
        }

        if !self.force_limit_ok {
            return Err(RobotError::SafetyViolation(
                "Force limit exceeded".into()
            ));
        }

        Ok(())
    }
}

/// 안전 제약 조건
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyConstraints {
    pub max_velocity_m_s: f64,
    pub max_acceleration_m_s2: f64,
    pub max_force_n: f64,
    pub max_torque_nm: f64,
    pub workspace_limits: WorkspaceLimits,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkspaceLimits {
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
    pub min_z: f64,
    pub max_z: f64,
}

impl WorkspaceLimits {
    pub fn contains(&self, x: f64, y: f64, z: f64) -> bool {
        x >= self.min_x && x <= self.max_x
            && y >= self.min_y && y <= self.max_y
            && z >= self.min_z && z <= self.max_z
    }
}
```

### 기본 타입 정의 (types.rs)
```rust
use serde::{Deserialize, Serialize};
use std::time::SystemTime;

/// 로봇 유형
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum RobotType {
    Exoskeleton,
    Prosthetics,
    Rehabilitation,
    CareRobot,
    SurgicalAssistant,
    MobilityAid,
}

/// 디바이스 상태
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum DeviceStatus {
    Operational,
    Standby,
    Error,
    Maintenance,
    Calibrating,
}

/// 3D 위치
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Position3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// 두 점 사이의 유클리드 거리
    pub fn distance_to(&self, other: &Position3D) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// 벡터 정규화
    pub fn normalize(&self) -> Position3D {
        let magnitude = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if magnitude == 0.0 {
            return *self;
        }
        Position3D {
            x: self.x / magnitude,
            y: self.y / magnitude,
            z: self.z / magnitude,
        }
    }
}

/// 오리엔테이션 (Roll-Pitch-Yaw)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Orientation {
    pub roll: f64,   // degrees
    pub pitch: f64,  // degrees
    pub yaw: f64,    // degrees
}

/// 관절 정보
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    pub angle_deg: f64,
    pub velocity_deg_s: f64,
    pub torque_nm: f64,
    pub target_angle_deg: f64,
}

impl Joint {
    /// 목표 각도까지의 에러
    pub fn angle_error(&self) -> f64 {
        self.target_angle_deg - self.angle_deg
    }

    /// PID 제어 계산
    pub fn compute_pid(&self, kp: f64, ki: f64, kd: f64, integral: f64) -> f64 {
        let error = self.angle_error();
        let derivative = -self.velocity_deg_s;  // 각속도의 음수
        kp * error + ki * integral + kd * derivative
    }
}

/// IMU 센서 데이터
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ImuData {
    pub acceleration: Position3D,
    pub gyroscope: Position3D,
    pub orientation: Orientation,
}
```

### 외골격 로봇 (adapters/exoskeleton.rs)
```rust
use crate::{RobotResult, RobotError, Position3D, Joint, ImuData, SafetyState};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExoskeletonSpec {
    pub exo_type: ExoskeletonType,
    pub joints: Vec<Joint>,
    pub gait: GaitData,
    pub imu: ImuData,
    pub control_mode: ControlMode,
    pub assist_level: f64,  // 0.0 ~ 1.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExoskeletonType {
    LowerBody,
    UpperBody,
    FullBody,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlMode {
    Assist,    // 보조 모드
    Resist,    // 저항 모드 (재활용)
    Passive,   // 수동 모드
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GaitData {
    pub phase: GaitPhase,
    pub step_count: u32,
    pub cadence_steps_min: f64,
    pub stride_length_cm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum GaitPhase {
    Stance,         // 지지기
    Swing,          // 유각기
    DoubleSupport,  // 양발 지지기
}

impl ExoskeletonSpec {
    /// 보행 주기 계산 (초)
    pub fn gait_cycle_duration(&self) -> f64 {
        if self.gait.cadence_steps_min <= 0.0 {
            return 0.0;
        }
        60.0 / self.gait.cadence_steps_min
    }

    /// 보행 속도 계산 (m/s)
    pub fn walking_speed_m_s(&self) -> f64 {
        let stride_m = self.gait.stride_length_cm / 100.0;
        let cadence_hz = self.gait.cadence_steps_min / 60.0;
        stride_m * cadence_hz
    }

    /// 관절 보조 토크 계산
    pub fn calculate_assist_torque(&self, joint_name: &str) -> RobotResult<f64> {
        let joint = self.joints.iter()
            .find(|j| j.name == joint_name)
            .ok_or_else(|| RobotError::InvalidParameter(
                format!("Joint not found: {}", joint_name)
            ))?;

        // 간단한 비례 제어
        let base_torque = joint.torque_nm;
        let assist_torque = base_torque * self.assist_level;

        Ok(assist_torque)
    }

    /// 낙상 위험 감지
    pub fn detect_fall_risk(&self) -> bool {
        // IMU 데이터 기반 낙상 위험 감지
        let pitch_threshold = 30.0;  // degrees
        let roll_threshold = 25.0;

        self.imu.orientation.pitch.abs() > pitch_threshold
            || self.imu.orientation.roll.abs() > roll_threshold
    }
}
```

### 의수/의족 (adapters/prosthetics.rs)
```rust
use crate::{RobotResult, RobotError};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProstheticSpec {
    pub prosthetic_type: ProstheticType,
    pub side: Side,
    pub dof: u8,
    pub fingers: Vec<Finger>,
    pub emg_sensors: Vec<EmgSensor>,
    pub grip_type: GripType,
    pub grip_force_n: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProstheticType {
    ProstheticHand,
    ProstheticArm,
    ProstheticLeg,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Side {
    Left,
    Right,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Finger {
    pub name: String,
    pub position: f64,         // 0.0 (open) ~ 1.0 (closed)
    pub force_n: f64,
    pub target_position: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmgSensor {
    pub channel: u8,
    pub muscle_site: String,
    pub signal_mv: f64,
    pub activation_level: f64,  // 0.0 ~ 1.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GripType {
    Power,      // 파워 그립
    Precision,  // 정밀 그립
    Lateral,    // 측면 그립
    Hook,       // 후크 그립
}

impl ProstheticSpec {
    /// EMG 신호 처리 및 의도 분류
    pub fn classify_intent(&self) -> RobotResult<GripIntent> {
        if self.emg_sensors.is_empty() {
            return Err(RobotError::SensorError(
                "No EMG sensors available".into()
            ));
        }

        // 평균 활성화 레벨 계산
        let avg_activation: f64 = self.emg_sensors.iter()
            .map(|s| s.activation_level)
            .sum::<f64>() / self.emg_sensors.len() as f64;

        // EMG 신호 분산 계산 (그립 유형 결정에 사용)
        let variance: f64 = self.emg_sensors.iter()
            .map(|s| (s.activation_level - avg_activation).powi(2))
            .sum::<f64>() / self.emg_sensors.len() as f64;

        // 간단한 의도 분류
        if avg_activation < 0.2 {
            Ok(GripIntent::Rest)
        } else if avg_activation > 0.7 {
            Ok(GripIntent::Close)
        } else if variance > 0.1 {
            Ok(GripIntent::Adjust)
        } else {
            Ok(GripIntent::Hold)
        }
    }

    /// 손가락 위치 제어
    pub fn control_finger(&mut self, finger_name: &str, target: f64) -> RobotResult<()> {
        if target < 0.0 || target > 1.0 {
            return Err(RobotError::InvalidParameter(
                "Target position must be between 0.0 and 1.0".into()
            ));
        }

        let finger = self.fingers.iter_mut()
            .find(|f| f.name == finger_name)
            .ok_or_else(|| RobotError::InvalidParameter(
                format!("Finger not found: {}", finger_name)
            ))?;

        finger.target_position = target;
        Ok(())
    }

    /// 그립 힘 조절
    pub fn set_grip_force(&mut self, force_n: f64) -> RobotResult<()> {
        if force_n < 0.0 {
            return Err(RobotError::InvalidParameter(
                "Grip force must be non-negative".into()
            ));
        }

        // 최대 힘 제한 (안전)
        const MAX_GRIP_FORCE: f64 = 100.0;  // Newton
        if force_n > MAX_GRIP_FORCE {
            return Err(RobotError::SafetyViolation(
                format!("Grip force {} N exceeds maximum {} N", force_n, MAX_GRIP_FORCE)
            ));
        }

        self.grip_force_n = force_n;
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum GripIntent {
    Rest,    // 휴식
    Open,    // 열기
    Close,   // 닫기
    Hold,    // 유지
    Adjust,  // 조정
}
```

### 재활 로봇 (adapters/rehabilitation.rs)
```rust
use crate::{RobotResult, RobotError, Position3D};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RehabilitationSpec {
    pub therapy_type: TherapyType,
    pub exercise: Exercise,
    pub trajectory: Trajectory,
    pub patient_effort: PatientEffort,
    pub performance: PerformanceMetrics,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TherapyType {
    UpperLimb,
    LowerLimb,
    Gait,
    Balance,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Exercise {
    pub name: String,
    pub repetition: u32,
    pub total_repetitions: u32,
    pub duration_seconds: u32,
    pub difficulty_level: u8,  // 1~5
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Trajectory {
    pub current_position: Position3D,
    pub target_position: Position3D,
    pub velocity_m_s: f64,
    pub path_completion: f64,  // 0.0 ~ 1.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PatientEffort {
    pub active_participation: f64,  // 0.0 ~ 1.0
    pub assist_as_needed: f64,      // 0.0 ~ 1.0
    pub resistance_nm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    pub rom_achieved_deg: f64,    // Range of Motion
    pub rom_target_deg: f64,
    pub smoothness_score: f64,    // 0.0 ~ 1.0
    pub accuracy_cm: f64,
}

impl RehabilitationSpec {
    /// 운동 진행률 계산
    pub fn exercise_progress(&self) -> f64 {
        if self.exercise.total_repetitions == 0 {
            return 0.0;
        }
        self.exercise.repetition as f64 / self.exercise.total_repetitions as f64
    }

    /// ROM 달성률
    pub fn rom_achievement_rate(&self) -> f64 {
        if self.performance.rom_target_deg == 0.0 {
            return 0.0;
        }
        (self.performance.rom_achieved_deg / self.performance.rom_target_deg)
            .min(1.0)
    }

    /// Assist-As-Needed 제어 계산
    pub fn calculate_assistance(&self) -> f64 {
        // 환자의 능동 참여도에 반비례하여 보조
        let base_assist = 1.0 - self.patient_effort.active_participation;

        // 난이도에 따라 조정
        let difficulty_factor = self.exercise.difficulty_level as f64 / 5.0;

        (base_assist * difficulty_factor).clamp(0.0, 1.0)
    }

    /// 궤적 에러 계산
    pub fn trajectory_error(&self) -> f64 {
        self.trajectory.current_position.distance_to(
            &self.trajectory.target_position
        )
    }

    /// 다음 목표 위치 생성 (선형 보간)
    pub fn interpolate_next_target(&self, dt: f64) -> Position3D {
        let distance = self.trajectory_error();
        let direction = Position3D {
            x: self.trajectory.target_position.x - self.trajectory.current_position.x,
            y: self.trajectory.target_position.y - self.trajectory.current_position.y,
            z: self.trajectory.target_position.z - self.trajectory.current_position.z,
        }.normalize();

        let step = self.trajectory.velocity_m_s * dt;
        let step = step.min(distance);  // 목표를 넘어가지 않도록

        Position3D {
            x: self.trajectory.current_position.x + direction.x * step,
            y: self.trajectory.current_position.y + direction.y * step,
            z: self.trajectory.current_position.z + direction.z * step,
        }
    }
}
```

### 돌봄 로봇 (adapters/care.rs)
```rust
use crate::{RobotResult, RobotError};
use serde::{Deserialize, Serialize};
use std::time::SystemTime;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CareRobotSpec {
    pub care_type: CareType,
    pub interaction: Interaction,
    pub emotion: EmotionRecognition,
    pub vital_monitoring: VitalSigns,
    pub tasks: Vec<CareTask>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CareType {
    ElderlyCompanion,
    Pediatric,
    DementiaCare,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Interaction {
    pub mode: InteractionMode,
    pub active_duration_s: u32,
    pub engagement_level: f64,  // 0.0 ~ 1.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InteractionMode {
    Conversation,
    Entertainment,
    Reminder,
    Monitoring,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmotionRecognition {
    pub detected_emotion: String,
    pub confidence: f64,
    pub valence: f64,   // -1.0 (negative) ~ 1.0 (positive)
    pub arousal: f64,   // 0.0 (calm) ~ 1.0 (excited)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VitalSigns {
    pub heart_rate_bpm: u16,
    pub respiratory_rate_bpm: u16,
    pub body_temp_c: f64,
    pub fall_detected: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CareTask {
    pub task_type: String,
    pub scheduled_time: Option<String>,
    pub status: TaskStatus,
    pub description: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TaskStatus {
    Pending,
    InProgress,
    Completed,
    Cancelled,
}

impl VitalSigns {
    /// 바이탈 사인 정상 여부 확인
    pub fn is_normal(&self) -> bool {
        const HEART_RATE_MIN: u16 = 60;
        const HEART_RATE_MAX: u16 = 100;
        const RESP_RATE_MIN: u16 = 12;
        const RESP_RATE_MAX: u16 = 20;
        const TEMP_MIN: f64 = 36.0;
        const TEMP_MAX: f64 = 37.5;

        !self.fall_detected
            && self.heart_rate_bpm >= HEART_RATE_MIN
            && self.heart_rate_bpm <= HEART_RATE_MAX
            && self.respiratory_rate_bpm >= RESP_RATE_MIN
            && self.respiratory_rate_bpm <= RESP_RATE_MAX
            && self.body_temp_c >= TEMP_MIN
            && self.body_temp_c <= TEMP_MAX
    }

    /// 경고 생성
    pub fn generate_alerts(&self) -> Vec<String> {
        let mut alerts = Vec::new();

        if self.fall_detected {
            alerts.push("CRITICAL: Fall detected!".to_string());
        }

        if self.heart_rate_bpm < 60 {
            alerts.push(format!("WARNING: Low heart rate ({} bpm)", self.heart_rate_bpm));
        } else if self.heart_rate_bpm > 100 {
            alerts.push(format!("WARNING: High heart rate ({} bpm)", self.heart_rate_bpm));
        }

        if self.body_temp_c > 38.0 {
            alerts.push(format!("WARNING: Fever ({:.1}°C)", self.body_temp_c));
        } else if self.body_temp_c < 35.0 {
            alerts.push(format!("WARNING: Hypothermia ({:.1}°C)", self.body_temp_c));
        }

        alerts
    }
}

impl EmotionRecognition {
    /// 감정 상태 평가
    pub fn emotional_state(&self) -> EmotionalState {
        // 2차원 감정 모델 (Valence-Arousal)
        if self.valence > 0.3 && self.arousal > 0.5 {
            EmotionalState::Happy
        } else if self.valence > 0.3 && self.arousal < 0.5 {
            EmotionalState::Calm
        } else if self.valence < -0.3 && self.arousal > 0.5 {
            EmotionalState::Anxious
        } else if self.valence < -0.3 && self.arousal < 0.5 {
            EmotionalState::Sad
        } else {
            EmotionalState::Neutral
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum EmotionalState {
    Happy,
    Calm,
    Anxious,
    Sad,
    Neutral,
}
```

---

## 📋 Cargo.toml

```toml
[package]
name = "wia-robot"
version = "1.0.0"
edition = "2021"
description = "WIA Robot Accessibility Standard - Rust SDK"
license = "MIT"
repository = "https://github.com/WIA-Official/wia-standards"
keywords = ["robotics", "accessibility", "exoskeleton", "prosthetics", "rehabilitation"]
categories = ["science", "embedded", "hardware-support"]

[dependencies]
tokio = { version = "1", features = ["full"] }
serde = { version = "1", features = ["derive"] }
serde_json = "1"
thiserror = "1"
async-trait = "0.1"
chrono = { version = "0.4", features = ["serde"] }
uuid = { version = "1", features = ["v4", "serde"] }

# 수치 계산
nalgebra = "0.32"  # 선형대수
ndarray = "0.15"   # 배열 연산

# 신호 처리
rustfft = "6.0"    # FFT (EMG 신호 처리)

# WebAssembly 지원
wasm-bindgen = { version = "0.2", optional = true }

# Python 바인딩
pyo3 = { version = "0.20", optional = true }

# ROS2 연동
r2r = { version = "0.8", optional = true }

[features]
default = []
wasm = ["wasm-bindgen"]
python = ["pyo3"]
ros2 = ["r2r"]

[dev-dependencies]
tokio-test = "0.4"
approx = "0.5"  # 부동소수점 비교
criterion = "0.5"  # 벤치마크

[[bench]]
name = "control_benchmark"
harness = false
```

---

## 🚀 사용 예시

### Basic Usage
```rust
use wia_robot::prelude::*;

#[tokio::main]
async fn main() -> RobotResult<()> {
    // 외골격 로봇 제어
    let mut exo = ExoskeletonSpec {
        exo_type: ExoskeletonType::LowerBody,
        joints: vec![
            Joint {
                name: "hip_left".to_string(),
                angle_deg: 15.0,
                velocity_deg_s: 2.0,
                torque_nm: 40.0,
                target_angle_deg: 20.0,
            }
        ],
        gait: GaitData {
            phase: GaitPhase::Swing,
            step_count: 100,
            cadence_steps_min: 60.0,
            stride_length_cm: 65.0,
        },
        // ... 기타 필드
    };

    // 보행 속도 계산
    let speed = exo.walking_speed_m_s();
    println!("Walking speed: {:.2} m/s", speed);

    // 낙상 위험 감지
    if exo.detect_fall_risk() {
        println!("WARNING: Fall risk detected!");
    }

    // 의수 제어
    let mut prosthetic = ProstheticSpec {
        // ... 초기화
    };

    // EMG 신호 기반 의도 분류
    let intent = prosthetic.classify_intent()?;
    println!("Detected intent: {:?}", intent);

    // 손가락 제어
    prosthetic.control_finger("thumb", 0.8)?;
    prosthetic.set_grip_force(25.0)?;

    Ok(())
}
```

---

## 📁 산출물 목록

```
/api/rust/Cargo.toml
/api/rust/src/lib.rs
/api/rust/src/types.rs
/api/rust/src/error.rs
/api/rust/src/safety.rs
/api/rust/src/prelude.rs
/api/rust/src/core/mod.rs
/api/rust/src/core/device.rs
/api/rust/src/core/control.rs
/api/rust/src/core/sensor.rs
/api/rust/src/core/actuator.rs
/api/rust/src/adapters/mod.rs
/api/rust/src/adapters/exoskeleton.rs
/api/rust/src/adapters/prosthetics.rs
/api/rust/src/adapters/rehabilitation.rs
/api/rust/src/adapters/care.rs
/api/rust/src/adapters/surgical.rs
/api/rust/src/adapters/mobility.rs
/api/rust/src/algorithms/mod.rs
/api/rust/src/algorithms/gait.rs
/api/rust/src/algorithms/trajectory.rs
/api/rust/src/algorithms/emg.rs
/api/rust/src/algorithms/kinematics.rs
/api/rust/tests/integration_test.rs
/api/rust/examples/basic_usage.rs
/api/rust/examples/exoskeleton_control.rs
/api/rust/examples/prosthetic_emg.rs
/api/rust/examples/rehabilitation_session.rs
/api/rust/README.md
```

---

## ✅ 완료 체크리스트

```
□ Cargo.toml 생성
□ Error 타입 정의
□ Safety 시스템 구현
□ 기본 타입 정의 (Position3D, Joint, IMU 등)
□ 6개 로봇 어댑터 구현
  □ Exoskeleton (보행 제어, 낙상 감지)
  □ Prosthetics (EMG 처리, 그립 제어)
  □ Rehabilitation (궤적 계획, 진행률)
  □ CareRobot (감정 인식, 바이탈 모니터링)
  □ SurgicalAssistant (원격 제어, 안전)
  □ MobilityAid (자율 내비게이션)
□ 알고리즘 모듈 구현
  □ Gait analysis
  □ Trajectory planning
  □ EMG signal processing
  □ Kinematics
□ 단위 테스트 작성
□ 통합 테스트 작성
□ 예제 코드 작성
□ cargo test 통과
□ cargo clippy 경고 없음
□ README 업데이트
```

---

## 🔄 작업 순서

```
1. Cargo.toml 생성
   ↓
2. error.rs - 에러 타입 정의
   ↓
3. safety.rs - 안전 시스템 구현
   ↓
4. types.rs - 기본 타입 정의
   ↓
5. adapters/exoskeleton.rs - 외골격 로봇
   ↓
6. adapters/prosthetics.rs - 의수/의족
   ↓
7. adapters/rehabilitation.rs - 재활 로봇
   ↓
8. adapters/care.rs - 돌봄 로봇
   ↓
9. adapters/surgical.rs - 수술 보조 로봇
   ↓
10. adapters/mobility.rs - 이동 보조 로봇
   ↓
11. algorithms/ 모듈 구현
   ↓
12. core/ 모듈 통합
   ↓
13. 테스트 작성 및 실행
   ↓
14. 예제 코드 작성
   ↓
15. 완료 체크리스트 확인
   ↓
16. Phase 3 시작 가능
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1 스키마와 1:1 대응되는 타입 정의
✅ 모든 제어 함수에 안전 검증 포함
✅ 실시간 처리 고려 (비동기, 무잠금 알고리즘)
✅ 모든 물리량에 단위 명시
✅ 의료 기기 표준 준수 (IEC 62304)
✅ f64 사용 (제어 정밀도)
✅ serde 지원으로 JSON 변환 가능
✅ ROS2 메시지와 호환 가능한 구조
```

### DON'T (하지 말 것)

```
❌ 안전 검증 없는 제어 함수
❌ panic! 사용 (Result 반환)
❌ unwrap() 남용 (? 연산자 사용)
❌ 하드코딩된 물리 상수
❌ 부동소수점 직접 비교
❌ 블로킹 I/O (비동기 사용)
```

---

## 🚀 작업 시작

이제 Phase 2 작업을 시작하세요.

첫 번째 단계: **Cargo.toml 생성 후 error.rs 구현**

```bash
cargo new --lib wia-robot
```

화이팅! 🤖🦀

---

<div align="center">

**Phase 2 of 4**

Rust API Implementation

🦀 Safe, Fast, Accessible 🦀

弘益人間 - Benefit All Humanity

</div>
