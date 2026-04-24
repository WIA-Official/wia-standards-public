# Phase 1: Data Format Standard
## Claude Code 작업 프롬프트

---

**Standard**: WIA Robot (Robotics Accessibility)
**Phase**: 1 of 4
**목표**: 보조 로봇 기술 데이터의 표준 형식 정의
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + JSON Schema + 예제 파일

---

## 🎯 Phase 1 목표

### 핵심 질문
```
"외골격 로봇, 의수/의족, 재활 로봇, 돌봄 로봇, 수술 보조 로봇...

 각각 다른 제조사, 다른 프로토콜, 다른 데이터 형식을 사용한다.

 장애인이 다양한 보조 로봇을 함께 사용하려면?
 재활 로봇의 데이터를 외골격에 전달하려면?
 의수의 센서 데이터를 AI 모델과 연동하려면?

 이걸 하나의 표준 형식으로 통일할 수 있을까?"
```

### 목표
```
보조 로봇 기술 유형에 관계없이
모든 로봇이 동일한 JSON 형식으로 데이터를 표현하도록
Data Format Standard를 정의한다.
```

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 보조 로봇 기술 조사

아래 기술 유형별로 웹서치하여 실제 데이터 형식을 조사하세요:

| 로봇 유형 | 조사 대상 | 웹서치 키워드 |
|----------|----------|--------------|
| **Exoskeleton** | 보행 보조 외골격 | "exoskeleton robot ReWalk Ekso data format API" |
| **Prosthetics** | 의수/의족 | "robotic prosthetic arm hand sensor data open bionics" |
| **Rehabilitation Robot** | 재활 치료 로봇 | "rehabilitation robot InMotion data protocol" |
| **Care Robot** | 돌봄 로봇 | "elderly care robot Pepper Paro telemetry" |
| **Surgical Assistant** | 수술 보조 로봇 | "surgical robot da Vinci API data format" |
| **Mobility Aid** | 이동 보조 로봇 | "wheelchair robot autonomous navigation data" |

### 2단계: 기존 표준/프로토콜 조사

| 표준/프로젝트 | 조사 내용 | 웹서치 키워드 |
|-------------|----------|--------------|
| **ROS (Robot Operating System)** | 로봇 미들웨어 | "ROS message format robot data types" |
| **ROS 2** | 차세대 ROS | "ROS2 DDS data format IDL" |
| **OpenRobotics** | 오픈 로봇 표준 | "OpenRobotics standard interface" |
| **ISO 13482** | 서비스 로봇 안전 | "ISO 13482 personal care robot standard" |
| **IEEE P2751** | 3D 맵핑 데이터 | "IEEE P2751 robot map data format" |
| **DICOM** | 의료 영상 표준 | "DICOM medical device integration" |

### 3단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-1.md`에 다음을 정리:

```markdown
# Phase 1 사전 조사 결과

## 1. Exoskeleton (외골격 로봇)

### ReWalk
- 데이터 형식: [조사 내용]
- 센서 데이터: 관절 각도, 힘 센서, IMU
- 제어 인터페이스: [조사 내용]

### Ekso Bionics
- 데이터 형식: [조사 내용]
- 필요 데이터 필드: [분석]
...

## 2. Prosthetics (의수/의족)

### Open Bionics
- 센서 유형: [조사 내용]
- EMG 신호 처리: [조사 내용]
...

## 3. Rehabilitation Robot (재활 로봇)

### InMotion ARM
- 운동 데이터: [조사 내용]
- 치료 프로토콜: [조사 내용]
...

## 4. Care Robot (돌봄 로봇)

### Pepper / Paro
- 상호작용 데이터: [조사 내용]
- 감정 인식: [조사 내용]
...

## 5. Surgical Assistant (수술 보조 로봇)

### da Vinci Surgical System
- 원격 수술 데이터: [조사 내용]
- 정밀도 요구사항: [조사 내용]
...

## 6. Mobility Aid (이동 보조 로봇)

### Autonomous Wheelchair
- 내비게이션 데이터: [조사 내용]
- 장애물 회피: [조사 내용]
...

## 7. 공통점 분석
- 모든 로봇에 공통으로 필요한 필드: [분석]
- 로봇별 고유 필드: [분석]
- 안전 관련 필수 데이터: [분석]

## 8. 결론
- 표준 형식 설계 방향: [제안]
- 접근성 고려사항: [제안]
```

---

## 🏗️ 표준 설계

### 기본 구조 (제안)

```json
{
  "$schema": "https://wia.live/schemas/robot/device.schema.json",
  "version": "1.0.0",
  "device": {
    "id": "고유 디바이스 ID",
    "type": "로봇 유형",
    "name": "디바이스명",
    "manufacturer": "제조사",
    "model": "모델명",
    "firmware_version": "펌웨어 버전",
    "serial_number": "시리얼 번호"
  },
  "user": {
    "id": "사용자 ID",
    "profile": "사용자 프로필 (선택)",
    "medical_clearance": true
  },
  "state": {
    "status": "operational|standby|error|maintenance",
    "battery_percent": 85,
    "uptime_seconds": 3600,
    "last_calibration": "ISO 8601 timestamp"
  },
  "spec": {
    "로봇 유형별 고유 데이터"
  },
  "safety": {
    "emergency_stop": false,
    "fall_detection": false,
    "collision_avoidance": true,
    "vital_signs_ok": true
  },
  "meta": {
    "timestamp": "ISO 8601 timestamp",
    "sequence": 12345,
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitude_m": 38
    }
  }
}
```

### 로봇별 `spec` 필드 정의

#### Exoskeleton (외골격)
```json
{
  "spec": {
    "type": "lower_body",        // "lower_body", "upper_body", "full_body"
    "joints": [
      {
        "name": "hip_left",
        "angle_deg": 15.5,
        "velocity_deg_s": 2.3,
        "torque_nm": 45.2,
        "target_angle_deg": 18.0
      },
      {
        "name": "knee_left",
        "angle_deg": 30.1,
        "velocity_deg_s": 3.1,
        "torque_nm": 38.5,
        "target_angle_deg": 35.0
      }
    ],
    "gait": {
      "phase": "swing",          // "stance", "swing", "double_support"
      "step_count": 1523,
      "cadence_steps_min": 65,
      "stride_length_cm": 58.5
    },
    "sensors": {
      "imu": {
        "acceleration": {"x": 0.12, "y": 9.81, "z": 0.05},
        "gyroscope": {"x": 0.01, "y": 0.02, "z": -0.01},
        "orientation": {"roll": 1.2, "pitch": -2.5, "yaw": 85.3}
      },
      "force_sensors": [
        {"location": "foot_left", "force_n": 650},
        {"location": "foot_right", "force_n": 200}
      ]
    },
    "control_mode": "assist",    // "assist", "resist", "passive"
    "assist_level": 0.75         // 0.0 ~ 1.0
  }
}
```

#### Prosthetics (의수/의족)
```json
{
  "spec": {
    "type": "prosthetic_hand",   // "prosthetic_hand", "prosthetic_arm", "prosthetic_leg"
    "side": "left",              // "left", "right"
    "dof": 6,                    // Degrees of freedom
    "fingers": [
      {
        "name": "thumb",
        "position": 0.65,        // 0.0 (open) ~ 1.0 (closed)
        "force_n": 12.5,
        "target_position": 0.70
      },
      {
        "name": "index",
        "position": 0.80,
        "force_n": 15.2,
        "target_position": 0.85
      }
    ],
    "emg_sensors": [
      {
        "channel": 1,
        "muscle_site": "flexor_carpi_radialis",
        "signal_mv": 125.5,
        "activation_level": 0.72
      },
      {
        "channel": 2,
        "muscle_site": "extensor_carpi_radialis",
        "signal_mv": 85.3,
        "activation_level": 0.45
      }
    ],
    "grip_type": "precision",    // "power", "precision", "lateral", "hook"
    "grip_force_n": 45.0,
    "sensory_feedback": {
      "tactile_enabled": true,
      "vibration_intensity": 0.6,
      "temperature_c": 28.5
    }
  }
}
```

#### Rehabilitation Robot (재활 로봇)
```json
{
  "spec": {
    "therapy_type": "upper_limb", // "upper_limb", "lower_limb", "gait", "balance"
    "exercise": {
      "name": "shoulder_flexion",
      "repetition": 15,
      "total_repetitions": 30,
      "duration_seconds": 180,
      "difficulty_level": 3       // 1~5
    },
    "trajectory": {
      "current_position": {"x": 0.35, "y": 0.25, "z": 0.15},
      "target_position": {"x": 0.40, "y": 0.30, "z": 0.20},
      "velocity_m_s": 0.05,
      "path_completion": 0.65     // 0.0 ~ 1.0
    },
    "patient_effort": {
      "active_participation": 0.75, // 0.0 ~ 1.0
      "assist_as_needed": 0.30,
      "resistance_nm": 5.2
    },
    "performance_metrics": {
      "rom_achieved_deg": 85,     // Range of Motion
      "rom_target_deg": 90,
      "smoothness_score": 0.82,   // 0.0 ~ 1.0
      "accuracy_cm": 2.5
    },
    "session": {
      "session_id": "session_2024_001",
      "therapist_id": "therapist_kim",
      "start_time": "2024-12-14T10:00:00Z",
      "progress_percent": 50
    }
  }
}
```

#### Care Robot (돌봄 로봇)
```json
{
  "spec": {
    "care_type": "elderly_companion", // "elderly_companion", "pediatric", "dementia_care"
    "interaction": {
      "mode": "conversation",     // "conversation", "entertainment", "reminder", "monitoring"
      "active_duration_s": 300,
      "engagement_level": 0.75    // 0.0 ~ 1.0
    },
    "emotion_recognition": {
      "detected_emotion": "content",
      "confidence": 0.82,
      "valence": 0.65,            // -1.0 (negative) ~ 1.0 (positive)
      "arousal": 0.45             // 0.0 (calm) ~ 1.0 (excited)
    },
    "vital_monitoring": {
      "heart_rate_bpm": 72,
      "respiratory_rate_bpm": 16,
      "body_temp_c": 36.8,
      "fall_detected": false
    },
    "tasks": [
      {
        "type": "medication_reminder",
        "scheduled_time": "2024-12-14T14:00:00Z",
        "status": "completed",
        "confirmation": true
      },
      {
        "type": "activity_prompt",
        "description": "Time for gentle exercise",
        "status": "pending"
      }
    ],
    "navigation": {
      "current_room": "living_room",
      "following_user": true,
      "distance_to_user_m": 2.5
    }
  }
}
```

#### Surgical Assistant (수술 보조 로봇)
```json
{
  "spec": {
    "surgical_type": "minimally_invasive",
    "instruments": [
      {
        "arm_id": 1,
        "instrument_type": "grasper",
        "position": {"x": 125.5, "y": 85.3, "z": 200.1},
        "orientation": {"roll": 15, "pitch": 30, "yaw": 45},
        "state": "open",
        "force_n": 0.5
      },
      {
        "arm_id": 2,
        "instrument_type": "cautery",
        "position": {"x": 130.2, "y": 90.1, "z": 198.5},
        "orientation": {"roll": 10, "pitch": 25, "yaw": 50},
        "state": "inactive",
        "power_w": 0
      }
    ],
    "teleoperation": {
      "surgeon_console": "console_1",
      "latency_ms": 12,
      "motion_scaling": 5.0,       // 1:5 surgeon:robot motion
      "tremor_filtering": true
    },
    "camera": {
      "zoom_level": 10,
      "stereo_enabled": true,
      "resolution": "4K",
      "frame_rate_fps": 60
    },
    "safety": {
      "workspace_boundary_ok": true,
      "collision_detection": true,
      "force_limit_exceeded": false,
      "emergency_stop_active": false
    }
  }
}
```

#### Mobility Aid (이동 보조 로봇)
```json
{
  "spec": {
    "mobility_type": "powered_wheelchair", // "powered_wheelchair", "walker_robot", "carrier_robot"
    "autonomous_mode": "semi_autonomous",  // "manual", "semi_autonomous", "fully_autonomous"
    "navigation": {
      "current_pose": {
        "x": 5.25,
        "y": 3.80,
        "theta": 1.57
      },
      "destination": {
        "x": 10.0,
        "y": 8.5,
        "name": "bedroom"
      },
      "path_status": "navigating",  // "idle", "planning", "navigating", "arrived"
      "distance_to_goal_m": 7.5
    },
    "motion": {
      "velocity": {"linear": 0.5, "angular": 0.1},
      "max_velocity": {"linear": 1.2, "angular": 0.5},
      "acceleration": {"linear": 0.2, "angular": 0.05}
    },
    "sensors": {
      "lidar": {
        "range_m": 10.0,
        "fov_deg": 270,
        "obstacle_detected": true,
        "closest_obstacle_m": 1.2
      },
      "cameras": [
        {
          "id": "front_camera",
          "resolution": "1920x1080",
          "obstacles_detected": 2
        }
      ]
    },
    "user_interface": {
      "input_method": "joystick",  // "joystick", "head_tracker", "voice", "eye_gaze"
      "command": {"type": "move_forward", "value": 0.7},
      "override_active": false
    },
    "accessibility": {
      "tilt_angle_deg": 0,
      "seat_elevation_cm": 0,
      "armrest_position": "down"
    }
  }
}
```

---

## 📁 산출물 목록

Phase 1 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-1.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-1-DATA-FORMAT.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 기본 구조 (Base Structure)
4. 로봇별 데이터 형식 (Robot-Specific Data)
   - Exoskeleton
   - Prosthetics
   - Rehabilitation Robot
   - Care Robot
   - Surgical Assistant
   - Mobility Aid
5. 안전 표준 (Safety Standards)
6. 접근성 요구사항 (Accessibility Requirements)
7. 확장성 (Extensibility)
8. 버전 관리 (Versioning)
9. 예제 (Examples)
10. 참고문헌 (References)
```

### 3. JSON Schema 파일
```
/spec/schemas/
├── device.schema.json           (기본 디바이스 스키마)
├── robot-type.schema.json       (로봇 유형 정의)
├── exoskeleton.schema.json
├── prosthetics.schema.json
├── rehabilitation.schema.json
├── care-robot.schema.json
├── surgical.schema.json
└── mobility-aid.schema.json
```

### 4. 예제 데이터 파일
```
/examples/sample-data/
├── exoskeleton-rewalk.json
├── prosthetic-hand-openbionics.json
├── rehabilitation-inmotion.json
├── care-robot-pepper.json
├── surgical-davinci.json
└── wheelchair-autonomous.json
```

---

## ✅ 완료 체크리스트

Phase 1 완료 전 확인:

```
□ 웹서치로 6개 이상 보조 로봇 데이터 형식 조사 완료
□ ROS/ROS2 메시지 형식 조사 완료
□ 안전 표준 (ISO 13482 등) 조사 완료
□ /spec/RESEARCH-PHASE-1.md 작성 완료
□ /spec/PHASE-1-DATA-FORMAT.md 작성 완료
□ JSON Schema 파일 생성 완료 (기본 + 로봇별 6개)
□ 예제 데이터 파일 생성 완료 (6개)
□ JSON Schema로 예제 데이터 검증 통과
□ 접근성 요구사항 문서화 완료
□ 안전 관련 필드 정의 완료
□ README 업데이트 (Phase 1 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 보조 로봇 기술 및 기존 표준 조사
   ↓
2. /spec/RESEARCH-PHASE-1.md 작성
   ↓
3. 조사 결과 바탕으로 표준 설계
   ↓
4. /spec/PHASE-1-DATA-FORMAT.md 작성
   ↓
5. JSON Schema 파일 생성
   ↓
6. 예제 데이터 파일 생성
   ↓
7. 스키마 검증 테스트
   ↓
8. 완료 체크리스트 확인
   ↓
9. Phase 2 시작 가능
```

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ 실제 제품의 SDK/API 문서를 웹서치로 확인
✅ ROS/ROS2 메시지 타입과 호환성 고려
✅ 모든 필드에 명확한 단위와 범위 명시
✅ 안전 관련 필드 (emergency_stop, fall_detection 등) 필수 포함
✅ 의료기기 표준 (ISO 13485, IEC 62304) 참고
✅ 접근성 관련 필드 명시 (사용자 맞춤 설정)
✅ 확장 가능한 구조로 설계 (미래 로봇 유형 고려)
✅ JSON Schema는 draft-07 표준 사용
```

### DON'T (하지 말 것)

```
❌ 추측으로 데이터 형식 정의 (반드시 조사 후)
❌ 특정 제조사 형식에 종속되는 설계
❌ 안전 관련 필드 누락
❌ 필수 필드와 선택 필드 구분 없이 작성
❌ 단위 없는 수치 데이터
❌ 사용자 개인정보 보호 고려 없이 설계
```

---

## 🔗 참고 자료

- **ROS**: Robot Operating System - http://www.ros.org/
- **ROS 2**: Next Generation ROS - https://docs.ros.org/
- **ISO 13482**: Safety requirements for personal care robots
- **ISO 13485**: Medical devices quality management
- **IEC 62304**: Medical device software lifecycle
- **OpenRobotics**: Open-source robotics foundation

---

## 🚀 작업 시작

이제 Phase 1 작업을 시작하세요.

첫 번째 단계: **웹서치로 Exoskeleton 로봇 데이터 형식 조사**

```
검색 키워드: "ReWalk exoskeleton API data format sensor"
```

화이팅! 🤖

---

<div align="center">

**Phase 1 of 4**

Data Format Standard

🤖 Robotics Accessibility for All 🤖

弘益人間 - Benefit All Humanity

</div>
