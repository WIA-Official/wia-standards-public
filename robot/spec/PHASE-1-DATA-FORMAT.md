# WIA Robot Standard - Phase 1: Data Format Specification

## Version Information
- **Document Version**: 1.0.0
- **Last Updated**: 2025-01-15
- **Status**: Phase 1 - Complete
- **Standard**: WIA-ROBOT-DATA-001

---

## 1. Overview

본 문서는 보조 로봇 기술의 통합 데이터 형식 표준을 정의합니다.

### 1.1 Scope

이 표준은 다음 보조 로봇 유형을 포함합니다:

| Robot Type | Description | Examples |
|------------|-------------|----------|
| **Exoskeleton** | 착용형 보행 보조 로봇 | ReWalk, Ekso, H-MEX |
| **Prosthetics** | 의수/의족 | Hero Arm, C-Leg, LUKE |
| **Rehabilitation** | 재활 치료 로봇 | InMotion, Lokomat |
| **Care Robot** | 돌봄/서비스 로봇 | Pepper, PARO |
| **Surgical** | 수술 보조 로봇 | da Vinci, Hugo |
| **Mobility Aid** | 이동 보조 로봇 | Smart Wheelchair |

### 1.2 Design Principles

1. **Interoperability**: 제조사 및 플랫폼 독립적
2. **Safety First**: 안전 관련 데이터 필수
3. **Extensibility**: 미래 로봇 유형 확장 가능
4. **Accessibility**: 장애 유형별 맞춤 지원
5. **ROS Compatibility**: ROS/ROS2 메시지 타입 호환

---

## 2. Terminology

| Term | Definition |
|------|------------|
| **DOF** | Degrees of Freedom (자유도) |
| **ROM** | Range of Motion (운동 범위) |
| **EMG** | Electromyography (근전도) |
| **IMU** | Inertial Measurement Unit (관성 측정 장치) |
| **E-Stop** | Emergency Stop (비상 정지) |
| **Haptic** | Tactile feedback (촉각 피드백) |
| **Teleoperation** | Remote control (원격 제어) |

---

## 3. Base Structure

모든 보조 로봇 데이터는 다음 기본 구조를 따릅니다.

### 3.1 Root Schema

```typescript
interface WIARobotData {
  $schema: string;              // Schema URL
  version: string;              // "1.0.0"

  device: DeviceInfo;           // Device identification
  user?: UserInfo;              // User information (optional)
  state: DeviceState;           // Current state
  spec: RobotTypeSpec;          // Type-specific data
  safety: SafetyStatus;         // Safety information
  meta: Metadata;               // Timestamps, location
}
```

### 3.2 Device Information

```typescript
interface DeviceInfo {
  id: string;                   // UUID format
  type: RobotType;              // Robot type enum
  name: string;                 // Human-readable name
  manufacturer: string;         // Manufacturer name
  model: string;                // Model name
  firmware_version: string;     // Firmware version
  serial_number?: string;       // Serial number (optional)
  capabilities: string[];       // Supported features
}

enum RobotType {
  EXOSKELETON = "exoskeleton",
  PROSTHETICS = "prosthetics",
  REHABILITATION = "rehabilitation",
  CARE_ROBOT = "care_robot",
  SURGICAL = "surgical",
  MOBILITY_AID = "mobility_aid"
}
```

### 3.3 User Information

```typescript
interface UserInfo {
  id: string;                   // User ID
  profile_id?: string;          // WIA Profile ID (optional)
  medical_clearance: boolean;   // Medical clearance status
  accessibility_needs?: AccessibilityNeeds;
}

interface AccessibilityNeeds {
  visual?: "none" | "low_vision" | "blind";
  hearing?: "none" | "hard_of_hearing" | "deaf";
  motor?: "none" | "limited" | "significant";
  cognitive?: "none" | "mild" | "moderate";
  preferred_feedback: ("visual" | "audio" | "haptic")[];
  preferred_input: ("joystick" | "voice" | "switch" | "eye_gaze" | "emg")[];
}
```

### 3.4 Device State

```typescript
interface DeviceState {
  status: "operational" | "standby" | "error" | "maintenance" | "charging";
  battery_percent: number;      // 0-100
  battery_voltage?: number;     // Volts
  battery_temp_c?: number;      // Celsius
  uptime_seconds: number;       // Time since boot
  last_calibration?: string;    // ISO 8601
  error_codes?: string[];       // Active error codes
}
```

### 3.5 Safety Status

```typescript
interface SafetyStatus {
  emergency_stop: boolean;      // E-Stop active
  emergency_stop_source?: EStopSource;
  fall_detection: boolean;      // Fall detected
  collision_avoidance: boolean; // Collision avoidance enabled
  force_limit_ok: boolean;      // Within force limits
  workspace_boundary_ok: boolean; // Within workspace
  vital_signs_ok?: boolean;     // User vitals normal
  safety_score: number;         // 0-100, overall safety
}

enum EStopSource {
  USER_BUTTON = "user_button",
  REMOTE = "remote",
  SOFTWARE = "software",
  OVERLOAD = "overload",
  FALL = "fall",
  OBSTACLE = "obstacle"
}
```

### 3.6 Metadata

```typescript
interface Metadata {
  timestamp: string;            // ISO 8601
  sequence: number;             // Message sequence number
  sample_rate_hz?: number;      // Data sample rate
  location?: {
    latitude: number;
    longitude: number;
    altitude_m?: number;
    accuracy_m?: number;
  };
  session_id?: string;          // Current session ID
}
```

---

## 4. Robot-Specific Data Formats

### 4.1 Exoskeleton (외골격)

```typescript
interface ExoskeletonSpec {
  type: "lower_body" | "upper_body" | "full_body";

  joints: ExoJoint[];

  gait?: GaitData;

  sensors: {
    imu?: IMUData[];
    force_sensors?: ForceSensor[];
    encoders?: EncoderData[];
  };

  control: {
    mode: "assist" | "resist" | "passive" | "transparent";
    assist_level: number;       // 0.0-1.0
    impedance?: ImpedanceParams;
  };

  training?: {
    program_id: string;
    step_count: number;
    distance_m: number;
    calories_burned: number;
  };
}

interface ExoJoint {
  name: "hip_left" | "hip_right" | "knee_left" | "knee_right" |
        "ankle_left" | "ankle_right" | string;
  angle_deg: number;            // Current angle
  velocity_deg_s: number;       // Angular velocity
  acceleration_deg_s2?: number; // Angular acceleration
  torque_nm: number;            // Measured torque
  assist_torque_nm?: number;    // Assistance torque
  target_angle_deg?: number;    // Target angle
  min_angle_deg: number;        // ROM min
  max_angle_deg: number;        // ROM max
}

interface GaitData {
  phase: "stance" | "swing" | "double_support" | "heel_strike" |
         "toe_off" | "mid_stance";
  side?: "left" | "right";
  cycle_percent: number;        // 0-100%
  step_count: number;
  cadence_steps_min: number;
  stride_length_cm: number;
  step_width_cm?: number;
  velocity_m_s: number;
  symmetry_index?: number;      // 0-100% (100 = perfect)
}

interface IMUData {
  location: string;             // Sensor location
  acceleration: Vector3D;       // m/s^2
  gyroscope: Vector3D;          // deg/s
  orientation?: {
    roll: number;               // degrees
    pitch: number;
    yaw: number;
  };
  quaternion?: {
    x: number; y: number; z: number; w: number;
  };
}

interface Vector3D {
  x: number;
  y: number;
  z: number;
}
```

### 4.2 Prosthetics (의수/의족)

```typescript
interface ProstheticsSpec {
  type: "hand" | "arm" | "leg" | "foot";
  side: "left" | "right";
  dof: number;                  // Degrees of freedom

  // Hand/Arm specific
  fingers?: FingerData[];
  wrist?: WristData;
  elbow?: JointData;
  shoulder?: JointData;

  // Leg/Foot specific
  knee?: KneeData;
  ankle?: AnkleData;

  // EMG control
  emg_sensors?: EMGSensor[];

  // Grip (hand only)
  grip?: {
    type: "power" | "precision" | "lateral" | "hook" | "tripod" | "custom";
    force_n: number;
    opening_percent: number;    // 0-100%
  };

  // Sensory feedback
  sensory_feedback?: {
    tactile_enabled: boolean;
    vibration_intensity: number; // 0-1
    temperature_c?: number;
    pressure_zones?: PressureZone[];
  };

  // Adaptation
  adaptation: {
    terrain_mode?: "flat" | "stairs" | "ramp" | "uneven";
    speed_mode?: "slow" | "normal" | "fast";
    activity_mode?: "walking" | "running" | "sitting" | "cycling";
  };
}

interface FingerData {
  name: "thumb" | "index" | "middle" | "ring" | "pinky";
  position: number;             // 0.0 (open) - 1.0 (closed)
  velocity: number;             // position/s
  force_n: number;              // Grip force
  target_position?: number;
  tactile_pressure?: number;    // 0-1
}

interface EMGSensor {
  channel: number;
  muscle_site: string;          // Anatomical location
  signal_mv: number;            // Raw signal
  signal_filtered_mv?: number;  // Filtered signal
  activation_level: number;     // 0-1 normalized
  contraction_detected: boolean;
}

interface KneeData {
  angle_deg: number;
  velocity_deg_s: number;
  resistance_level: number;     // 0-1
  stance_phase: boolean;
  microprocessor_mode: string;
}
```

### 4.3 Rehabilitation Robot (재활 로봇)

```typescript
interface RehabilitationSpec {
  therapy_type: "upper_limb" | "lower_limb" | "hand" | "gait" | "balance";
  device_type: "end_effector" | "exoskeleton" | "treadmill" | "platform";

  exercise: {
    name: string;
    exercise_type: string;
    repetition: number;
    total_repetitions: number;
    set: number;
    total_sets: number;
    duration_seconds: number;
    difficulty_level: number;   // 1-10
    game_id?: string;           // Gamification
  };

  trajectory: {
    current_position: Position3D;
    target_position: Position3D;
    start_position: Position3D;
    velocity_m_s: number;
    path_completion: number;    // 0-1
    path_type?: "linear" | "circular" | "custom";
  };

  patient_effort: {
    active_participation: number; // 0-1
    assist_as_needed: number;   // 0-1 (robot assistance)
    resistance_nm?: number;
    force_applied: Vector3D;
    emg_activation?: number;
  };

  performance: {
    rom_achieved_deg: number;
    rom_target_deg: number;
    rom_baseline_deg: number;
    smoothness_score: number;   // 0-1
    accuracy_cm: number;
    speed_score: number;        // 0-1
    error_rate: number;         // 0-1
  };

  session: {
    session_id: string;
    therapist_id?: string;
    start_time: string;
    elapsed_seconds: number;
    progress_percent: number;
    notes?: string;
  };

  biofeedback?: {
    heart_rate_bpm?: number;
    fatigue_level?: number;     // 0-1
    pain_reported?: number;     // 0-10 scale
  };
}

interface Position3D {
  x: number;                    // meters
  y: number;
  z: number;
}
```

### 4.4 Care Robot (돌봄 로봇)

```typescript
interface CareRobotSpec {
  care_type: "elderly_companion" | "pediatric" | "dementia_care" |
             "hospital_assistant" | "service_robot";

  interaction: {
    mode: "conversation" | "entertainment" | "reminder" |
          "monitoring" | "assistance" | "idle";
    active_duration_s: number;
    engagement_level: number;   // 0-1
    last_interaction?: string;  // ISO 8601
    language: string;           // "ko-KR", "en-US", etc.
  };

  emotion_recognition?: {
    detected_emotion: "happy" | "sad" | "angry" | "fearful" |
                      "surprised" | "neutral" | "content";
    confidence: number;         // 0-1
    valence: number;            // -1 (negative) to 1 (positive)
    arousal: number;            // 0 (calm) to 1 (excited)
    face_detected: boolean;
  };

  vital_monitoring?: {
    heart_rate_bpm?: number;
    respiratory_rate?: number;
    body_temp_c?: number;
    blood_pressure?: { systolic: number; diastolic: number };
    oxygen_saturation?: number;
    fall_detected: boolean;
    activity_level: "resting" | "light" | "moderate" | "vigorous";
    sleep_quality?: number;     // 0-1
  };

  tasks: CareTask[];

  navigation: {
    current_room?: string;
    current_pose?: Pose2D;
    following_user: boolean;
    distance_to_user_m?: number;
    map_id?: string;
    navigation_status: "idle" | "navigating" | "arrived" | "blocked";
  };

  voice_interaction?: {
    last_utterance?: string;
    asr_confidence?: number;
    tts_active: boolean;
    wake_word_detected: boolean;
  };
}

interface CareTask {
  task_id: string;
  type: "medication_reminder" | "appointment_reminder" | "activity_prompt" |
        "meal_reminder" | "hydration_reminder" | "exercise_prompt" |
        "social_interaction" | "emergency_alert";
  description: string;
  scheduled_time?: string;
  status: "pending" | "in_progress" | "completed" | "skipped" | "failed";
  confirmation: boolean;
  notes?: string;
}

interface Pose2D {
  x: number;                    // meters
  y: number;
  theta: number;                // radians
}
```

### 4.5 Surgical Assistant (수술 보조 로봇)

```typescript
interface SurgicalSpec {
  surgical_type: "minimally_invasive" | "orthopedic" | "neurosurgical" |
                 "cardiac" | "urological" | "general";
  procedure_phase: "setup" | "docking" | "operating" | "undocking" | "complete";

  instruments: SurgicalInstrument[];

  teleoperation: {
    surgeon_console_id: string;
    connected: boolean;
    latency_ms: number;
    motion_scaling: number;     // e.g., 5.0 means 5:1 reduction
    tremor_filtering: boolean;
    clutch_engaged: boolean;
  };

  camera: {
    arm_id: number;
    type: "endoscope" | "microscope" | "external";
    zoom_level: number;
    focus_distance_mm?: number;
    stereo_enabled: boolean;
    resolution: string;         // "4K", "1080p"
    frame_rate_fps: number;
    field_of_view_deg?: number;
  };

  workspace: {
    boundaries: WorkspaceBoundary;
    reference_frame: string;
    registration_accuracy_mm?: number;
  };

  safety: {
    workspace_boundary_ok: boolean;
    collision_detection: boolean;
    force_limit_exceeded: boolean;
    instrument_count_verified: boolean;
    all_arms_visible: boolean;
  };

  recording?: {
    active: boolean;
    duration_s: number;
    storage_available_gb: number;
  };
}

interface SurgicalInstrument {
  arm_id: number;
  instrument_type: string;      // "grasper", "scissors", "cautery", etc.
  instrument_id?: string;

  position: Position3D;         // mm
  orientation: Orientation;
  velocity?: Vector3D;          // mm/s

  state: "open" | "closed" | "active" | "inactive";
  articulation_deg?: number;    // Wrist articulation

  force_n?: number;
  power_w?: number;             // For cautery

  insertion_depth_mm?: number;
  usage_count?: number;
}

interface Orientation {
  roll: number;                 // degrees
  pitch: number;
  yaw: number;
}

interface WorkspaceBoundary {
  type: "sphere" | "cylinder" | "box";
  center: Position3D;
  dimensions: { [key: string]: number };
}
```

### 4.6 Mobility Aid (이동 보조 로봇)

```typescript
interface MobilityAidSpec {
  mobility_type: "powered_wheelchair" | "smart_walker" |
                 "mobility_scooter" | "stair_climber";
  autonomous_mode: "manual" | "semi_autonomous" | "fully_autonomous";

  navigation: {
    current_pose: Pose2D;
    destination?: {
      x: number;
      y: number;
      name?: string;
    };
    path_status: "idle" | "planning" | "navigating" | "arrived" | "blocked";
    distance_to_goal_m?: number;
    eta_seconds?: number;
    map_id?: string;
    localization_confidence: number; // 0-1
  };

  motion: {
    velocity: {
      linear: number;           // m/s
      angular: number;          // rad/s
    };
    max_velocity: {
      linear: number;
      angular: number;
    };
    acceleration: {
      linear: number;           // m/s^2
      angular: number;          // rad/s^2
    };
    odometry: {
      distance_traveled_m: number;
      heading_deg: number;
    };
  };

  sensors: {
    lidar?: {
      range_m: number;
      fov_deg: number;
      obstacle_detected: boolean;
      closest_obstacle_m?: number;
      obstacle_direction_deg?: number;
    };
    ultrasonic?: UltrasonicSensor[];
    cameras?: CameraSensor[];
    cliff_sensors?: boolean[];
    bump_sensors?: boolean[];
  };

  user_interface: {
    input_method: "joystick" | "head_tracker" | "voice" |
                  "eye_gaze" | "sip_puff" | "switch";
    command?: {
      type: string;
      value: number;
    };
    override_active: boolean;
    input_sensitivity: number;  // 0-1
  };

  seating: {
    tilt_angle_deg: number;
    recline_angle_deg: number;
    seat_elevation_cm: number;
    leg_rest_angle_deg: number;
    armrest_position: "up" | "down" | "adjustable";
    pressure_relief_active?: boolean;
  };

  environment: {
    indoor: boolean;
    surface_type?: "smooth" | "carpet" | "outdoor" | "gravel";
    incline_deg?: number;
    detected_obstacles: number;
  };
}

interface UltrasonicSensor {
  id: string;
  location: string;             // "front_left", "rear", etc.
  distance_m: number;
  obstacle_detected: boolean;
}

interface CameraSensor {
  id: string;
  location: string;
  resolution: string;
  frame_rate_fps: number;
  depth_enabled: boolean;
  objects_detected?: number;
}
```

---

## 5. Common Data Types

### 5.1 Shared Interfaces

```typescript
interface ImpedanceParams {
  stiffness: number;            // Nm/rad
  damping: number;              // Nms/rad
  inertia?: number;             // kgm^2
  equilibrium_position?: number;
}

interface ForceSensor {
  location: string;
  force_n: number;
  force_vector?: Vector3D;
}

interface EncoderData {
  joint: string;
  raw_counts: number;
  resolution_cpr: number;       // Counts per revolution
}

interface PressureZone {
  zone_id: string;
  location: string;
  pressure: number;             // 0-1 normalized
}

interface JointData {
  angle_deg: number;
  velocity_deg_s?: number;
  torque_nm?: number;
  min_angle_deg: number;
  max_angle_deg: number;
}

interface WristData extends JointData {
  pronation_deg?: number;
  deviation_deg?: number;
}

interface AnkleData extends JointData {
  dorsiflexion_deg: number;
  plantarflexion_deg: number;
  inversion_deg?: number;
  eversion_deg?: number;
}
```

---

## 6. Safety Requirements

### 6.1 Mandatory Safety Fields

모든 로봇 데이터에는 다음 안전 필드가 필수입니다:

| Field | Type | Description |
|-------|------|-------------|
| emergency_stop | boolean | 비상 정지 활성화 여부 |
| safety_score | number | 0-100 종합 안전 점수 |

### 6.2 Safety Levels

```typescript
enum SafetyLevel {
  NORMAL = "normal",           // 정상 운영
  CAUTION = "caution",         // 주의 필요
  WARNING = "warning",         // 경고 상태
  CRITICAL = "critical",       // 위험 상태
  EMERGENCY = "emergency"      // 비상 상태
}
```

### 6.3 E-Stop Requirements

- 모든 로봇은 E-Stop 기능 필수
- E-Stop 트리거 시 200ms 이내 정지
- E-Stop 상태는 항상 데이터에 포함
- E-Stop 해제는 의도적 사용자 동작 필요

---

## 7. Accessibility Requirements

### 7.1 Multi-Modal Feedback

```typescript
interface AccessibilityFeedback {
  visual?: {
    enabled: boolean;
    high_contrast: boolean;
    large_text: boolean;
    screen_reader_compatible: boolean;
  };

  audio?: {
    enabled: boolean;
    volume: number;             // 0-1
    speech_rate: number;        // 0.5-2.0
    language: string;
  };

  haptic?: {
    enabled: boolean;
    intensity: number;          // 0-1
    patterns_supported: string[];
  };
}
```

### 7.2 Input Adaptations

지원 입력 방식:
- **Joystick**: 표준 조이스틱/조그다이얼
- **Voice**: 음성 명령
- **Eye Gaze**: 시선 추적
- **Switch**: 단일/다중 스위치
- **Sip-Puff**: 호흡 조절
- **EMG**: 근전도 신호
- **BCI**: 뇌-컴퓨터 인터페이스 (미래)

---

## 8. Extensibility

### 8.1 Custom Extensions

제조사별 확장 데이터:

```json
{
  "spec": {
    "type": "exoskeleton",
    "joints": [...],
    "extensions": {
      "manufacturer": "ExoTech",
      "proprietary_data": {
        "ai_prediction_score": 0.95,
        "custom_algorithm_version": "2.3.1"
      }
    }
  }
}
```

### 8.2 Future Robot Types

예약된 확장 유형:
- `companion_robot`: 반려 로봇
- `delivery_robot`: 배달 로봇
- `educational_robot`: 교육 로봇
- `industrial_exo`: 산업용 외골격
- `space_robot`: 우주 보조 로봇

---

## 9. Versioning

### 9.1 Semantic Versioning

- **Major**: 호환되지 않는 변경
- **Minor**: 하위 호환 기능 추가
- **Patch**: 하위 호환 버그 수정

### 9.2 Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-15 | Initial release |

---

## 10. Examples

### 10.1 Exoskeleton Example

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
      "side": "left",
      "cycle_percent": 65,
      "step_count": 1523,
      "cadence_steps_min": 65,
      "stride_length_cm": 58.5,
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
    "collision_avoidance": true,
    "force_limit_ok": true,
    "workspace_boundary_ok": true,
    "safety_score": 95
  },
  "meta": {
    "timestamp": "2025-01-15T10:30:00Z",
    "sequence": 12345
  }
}
```

### 10.2 Prosthetic Hand Example

```json
{
  "$schema": "https://wia.live/schemas/robot/v1/prosthetics.schema.json",
  "version": "1.0.0",
  "device": {
    "id": "pros-hero-001",
    "type": "prosthetics",
    "name": "Hero Arm",
    "manufacturer": "Open Bionics",
    "model": "Hero Arm v2",
    "firmware_version": "3.1.0",
    "capabilities": ["myoelectric", "multi_grip", "haptic_feedback"]
  },
  "state": {
    "status": "operational",
    "battery_percent": 85,
    "uptime_seconds": 7200
  },
  "spec": {
    "type": "hand",
    "side": "right",
    "dof": 6,
    "fingers": [
      {"name": "thumb", "position": 0.7, "velocity": 0.1, "force_n": 12.5},
      {"name": "index", "position": 0.8, "velocity": 0.1, "force_n": 15.2}
    ],
    "emg_sensors": [
      {"channel": 1, "muscle_site": "flexor_carpi_radialis",
       "signal_mv": 125.5, "activation_level": 0.72, "contraction_detected": true}
    ],
    "grip": {
      "type": "precision",
      "force_n": 45.0,
      "opening_percent": 20
    },
    "sensory_feedback": {
      "tactile_enabled": true,
      "vibration_intensity": 0.6
    },
    "adaptation": {
      "activity_mode": "walking"
    }
  },
  "safety": {
    "emergency_stop": false,
    "fall_detection": false,
    "collision_avoidance": false,
    "force_limit_ok": true,
    "workspace_boundary_ok": true,
    "safety_score": 98
  },
  "meta": {
    "timestamp": "2025-01-15T11:00:00Z",
    "sequence": 5678
  }
}
```

---

## 11. References

- ROS Standard Message Types: https://docs.ros.org/
- ISO 13482:2014 - Personal Care Robot Safety
- ISO 13485:2016 - Medical Devices QMS
- IEC 62304 - Medical Device Software
- IEEE 1872 - Ontology for Robotics

---

## Document Information

- **Document ID**: WIA-ROBOT-DATA-001
- **Classification**: Public Standard
- **License**: Open Standard (CC BY 4.0)

---

弘益人間 - Accessible Robotics for All Humanity
