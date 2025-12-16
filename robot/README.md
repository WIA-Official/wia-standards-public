# WIA Robot Standard

**Robotics Accessibility Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20ROBOT-orange.svg)](https://robot.wia.live)

---

<div align="center">

🦾 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**弘益人間** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA Robot is an open standard for assistive robotics data formats, enabling interoperability between diverse robot types and manufacturers.

### Supported Robot Types

| Type | Description | Examples |
|------|-------------|----------|
| **Exoskeleton** | 착용형 보행 보조 로봇 | ReWalk, Ekso, H-MEX |
| **Prosthetics** | 의수/의족 | Hero Arm, C-Leg, LUKE Arm |
| **Rehabilitation** | 재활 치료 로봇 | InMotion ARM, Lokomat |
| **Care Robot** | 돌봄/서비스 로봇 | Pepper, PARO |
| **Surgical** | 수술 보조 로봇 | da Vinci, Hugo RAS |
| **Mobility Aid** | 이동 보조 로봇 | Smart Wheelchair, Walker |

### Design Principles

- **Interoperability**: 제조사/플랫폼 독립적 데이터 형식
- **Safety First**: 안전 관련 데이터 필수 포함
- **ROS Compatibility**: ROS/ROS2 메시지 타입 호환
- **Extensibility**: 미래 로봇 유형 확장 가능
- **Accessibility**: 장애 유형별 맞춤 지원

---

## 📋 Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | Standard data format | ✅ Complete |
| **2** | API Interface | Rust SDK for developers | ✅ Complete |
| **3** | Communication Protocol | Device protocols | ✅ Complete |
| **4** | Ecosystem Integration | WIA integration | ⏳ Planned |

### Phase 1: Data Format (Complete)

- **Research**: 6개 로봇 유형 데이터 형식 조사
- **Base Schema**: 공통 디바이스 스키마 정의
- **Robot Schemas**: 유형별 확장 스키마 (6종)
- **Sample Data**: 실제 제품 기반 예제 데이터 (6종)
- **Safety Fields**: 비상정지, 낙상감지, 충돌회피 등

### Phase 2: Rust API (Complete)

- **Core SDK**: `wia-robot` crate (97 tests passing)
- **Safety System**: SafetyStatus, EStopSource, WorkspaceLimits
- **Control Systems**: PID, Impedance, Trajectory generators
- **6 Robot Adapters**: Exoskeleton, Prosthetics, Rehabilitation, Care, Surgical, Mobility
- **Device Management**: RobotDevice, DeviceRegistry

### Phase 3: Communication Protocol (Complete)

- **WRP Protocol**: WIA Robot Protocol for device communication
- **Message Types**: Handshake, Telemetry, Control, EmergencyStop, SafetyAlert
- **Safety Protocol**: Emergency stop sequence, Safety watchdog
- **Transport Abstraction**: WebSocket, MQTT, ROS2 DDS support
- **QoS Profiles**: Emergency (< 10ms), Control (< 50ms), Telemetry (< 100ms)

---

## 🚀 Quick Start

```json
{
  "$schema": "https://wia.live/schemas/robot/v1/exoskeleton.schema.json",
  "version": "1.0.0",
  "device": {
    "id": "exo-rewalk-001",
    "type": "exoskeleton",
    "name": "ReWalk Personal 6.0",
    "manufacturer": "ReWalk Robotics",
    "model": "Personal 6.0",
    "firmware_version": "6.2.1",
    "capabilities": ["lower_body", "gait_assist", "sit_stand"]
  },
  "state": {
    "status": "operational",
    "battery_percent": 78,
    "uptime_seconds": 3600
  },
  "spec": {
    "type": "lower_body",
    "joints": [
      {
        "name": "hip_left",
        "angle_deg": 15.5,
        "velocity_deg_s": 2.3,
        "torque_nm": 45.2,
        "min_angle_deg": -20,
        "max_angle_deg": 100
      }
    ],
    "gait": {
      "phase": "swing",
      "step_count": 1523,
      "cadence_steps_min": 65,
      "velocity_m_s": 0.5
    },
    "control": {
      "mode": "assist",
      "assist_level": 0.75
    }
  },
  "safety": {
    "emergency_stop": false,
    "fall_detection": false,
    "safety_score": 95
  },
  "meta": {
    "timestamp": "2025-01-15T10:30:00Z",
    "sequence": 12345
  }
}
```

### Rust SDK

```rust
use wia_robot::prelude::*;

// Create an exoskeleton
let mut exo = ExoskeletonSpec::new_lower_body();
exo.assist_level = 0.75;

// Configure gait parameters
exo.gait.cadence_steps_min = 60.0;
exo.gait.stride_length_cm = 65.0;

// Calculate walking speed
let speed = exo.walking_speed_m_s();
println!("Walking speed: {:.2} m/s", speed);

// Check for fall risk
if exo.detect_fall_risk() {
    println!("Warning: Fall risk detected!");
}

// Safety system
let mut safety = SafetyStatus::new_safe();
if let Err(e) = safety.validate() {
    safety.trigger_estop(EStopSource::UserButton);
}
```

---

## 📁 Structure

```
robot/
├── spec/                    # Specifications
│   ├── RESEARCH-PHASE-1.md
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── RESEARCH-PHASE-3.md
│   ├── PHASE-3-PROTOCOL.md
│   └── schemas/
│       ├── device.schema.json          # Base schema
│       ├── exoskeleton.schema.json
│       ├── prosthetics.schema.json
│       ├── rehabilitation.schema.json
│       ├── care-robot.schema.json
│       ├── surgical.schema.json
│       ├── mobility-aid.schema.json
│       ├── wrp-message.schema.json     # Protocol message
│       ├── wrp-safety.schema.json      # Safety messages
│       └── wrp-error.schema.json       # Error messages
├── api/
│   └── rust/                # Rust SDK (wia-robot crate)
│       ├── Cargo.toml
│       └── src/
│           ├── lib.rs
│           ├── error.rs
│           ├── safety.rs
│           ├── types.rs
│           ├── adapters/    # 6 robot adapters
│           │   ├── exoskeleton.rs
│           │   ├── prosthetics.rs
│           │   ├── rehabilitation.rs
│           │   ├── care.rs
│           │   ├── surgical.rs
│           │   └── mobility.rs
│           ├── core/        # Device & control
│           │   ├── device.rs
│           │   └── control.rs
│           ├── protocol/    # WRP Protocol (Phase 3)
│           │   ├── message.rs
│           │   ├── builder.rs
│           │   ├── handler.rs
│           │   └── error.rs
│           └── transport/   # Transport layer
│               ├── base.rs
│               └── mock.rs
├── examples/
│   └── sample-data/
│       ├── exoskeleton-rewalk.json
│       ├── prosthetic-hand-openbionics.json
│       ├── rehabilitation-inmotion.json
│       ├── care-robot-pepper.json
│       ├── surgical-davinci.json
│       └── wheelchair-autonomous.json
├── prompts/                 # Claude Code prompts
│   ├── PHASE-1-PROMPT.md
│   ├── PHASE-2-RUST-PROMPT.md
│   ├── PHASE-3-PROMPT.md
│   └── PHASE-4-PROMPT.md
└── docs/
```

---

## 🤖 Robot Type Details

### Exoskeleton (외골격)

```typescript
interface ExoskeletonSpec {
  type: "lower_body" | "upper_body" | "full_body";
  joints: ExoJoint[];        // 관절 상태 (각도, 속도, 토크)
  gait: GaitData;            // 보행 데이터 (단계, 속도, 보폭)
  control: ControlParams;    // 보조 모드 및 수준
}
```

### Prosthetics (의수/의족)

```typescript
interface ProstheticsSpec {
  type: "hand" | "arm" | "leg" | "foot";
  side: "left" | "right";
  fingers?: FingerData[];    // 손가락 위치/힘
  emg_sensors?: EMGSensor[]; // 근전도 센서
  grip?: GripData;           // 그립 유형/힘
}
```

### Rehabilitation (재활 로봇)

```typescript
interface RehabilitationSpec {
  therapy_type: "upper_limb" | "lower_limb" | "hand" | "gait";
  exercise: ExerciseData;    // 운동 정보
  trajectory: TrajectoryData; // 궤적 추적
  performance: PerformanceMetrics; // ROM, 정확도
  session: SessionData;      // 세션 정보
}
```

### Care Robot (돌봄 로봇)

```typescript
interface CareRobotSpec {
  care_type: "elderly_companion" | "dementia_care" | ...;
  interaction: InteractionData;   // 상호작용 모드
  emotion_recognition?: EmotionData; // 감정 인식
  vital_monitoring?: VitalMonitoring; // 생체 신호
  tasks: CareTask[];         // 알림/케어 태스크
}
```

### Surgical (수술 보조 로봇)

```typescript
interface SurgicalSpec {
  surgical_type: "minimally_invasive" | "orthopedic" | ...;
  instruments: SurgicalInstrument[]; // 기구 위치/상태
  teleoperation: Teleoperation;  // 원격 제어
  camera: CameraData;        // 카메라 정보
  workspace: WorkspaceData;  // 작업 영역
}
```

### Mobility Aid (이동 보조)

```typescript
interface MobilityAidSpec {
  mobility_type: "powered_wheelchair" | "smart_walker" | ...;
  navigation: NavigationData; // 위치/경로
  motion: MotionData;        // 속도/가속도
  sensors: SensorData;       // LiDAR, 카메라
  seating: SeatingData;      // 좌석 조절
}
```

---

## 🔒 Safety Requirements

모든 로봇 데이터에 필수 포함:

```json
{
  "safety": {
    "emergency_stop": false,    // 비상 정지 상태 (필수)
    "fall_detection": false,    // 낙상 감지
    "collision_avoidance": true, // 충돌 회피 활성화
    "force_limit_ok": true,     // 힘 한계 이내
    "safety_score": 95          // 종합 안전 점수 (필수)
  }
}
```

---

## ♿ Accessibility

사용자 접근성 요구사항 지원:

```json
{
  "user": {
    "accessibility_needs": {
      "visual": "low_vision",
      "hearing": "hard_of_hearing",
      "motor": "significant",
      "preferred_feedback": ["audio", "haptic"],
      "preferred_input": ["voice", "switch", "eye_gaze"]
    }
  }
}
```

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://robot.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/robot |
| **JSON Schema** | https://wia.live/schemas/robot/v1/ |

---

## 📚 References

- [ROS (Robot Operating System)](https://www.ros.org/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [ISO 13482:2014](https://www.iso.org/standard/53820.html) - Personal Care Robot Safety
- [ISO 13485:2016](https://www.iso.org/standard/59752.html) - Medical Device QMS
- [IEEE 1872](https://standards.ieee.org/standard/1872-2015.html) - Ontology for Robotics

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **弘益人間** - Benefit All Humanity

*다시 걸을 수 있는 기회를 더 많은 사람들에게*

© 2025 SmileStory Inc. / WIA

</div>
