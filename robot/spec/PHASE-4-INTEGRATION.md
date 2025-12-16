# WIA Robot Ecosystem Integration Specification

**Version 1.0.0**

---

## 1. 개요 (Overview)

WIA Robot Ecosystem Integration은 보조 로봇 데이터를 외부 시스템과 연동하기 위한 표준입니다.

### 목적

- **시각화**: RViz2, Gazebo, Unity를 통한 로봇 상태 시각화
- **의료 연동**: HL7 FHIR를 통한 EMR 시스템 통합
- **AI/ML**: 학습 데이터셋 생성 및 모델 연동
- **대시보드**: 실시간 웹 대시보드 스트리밍
- **데이터 내보내기**: JSON, CSV, URDF/SDF 형식 지원

### 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA Robot Data                            │
│     (Phase 1: Data Format + Phase 2: API + Phase 3: WRP)    │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    OutputManager                             │
│                  (Central Hub)                               │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │Visualization│ │   Medical   │ │   AI/ML     │           │
│  │   Adapter   │ │   Adapter   │ │   Adapter   │           │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘           │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │  Dashboard  │ │   Export    │ │   Logger    │           │
│  │   Adapter   │ │   Adapter   │ │   Adapter   │           │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘           │
└─────────┼───────────────┼───────────────┼───────────────────┘
          │               │               │
          ▼               ▼               ▼
    ┌──────────┐   ┌──────────┐   ┌──────────┐
    │  RViz2   │   │ HL7 FHIR │   │TensorFlow│
    │  Gazebo  │   │   EMR    │   │ PyTorch  │
    │  Unity   │   │  DICOM   │   │   CSV    │
    └──────────┘   └──────────┘   └──────────┘
```

---

## 2. 출력 인터페이스 (Output Interface)

### 2.1 OutputAdapter Trait

모든 출력 어댑터가 구현해야 하는 공통 인터페이스:

```rust
pub trait OutputAdapter: Send + Sync {
    /// 출력 유형
    fn output_type(&self) -> OutputType;

    /// 어댑터 이름
    fn name(&self) -> &str;

    /// 초기화
    fn initialize(&mut self, config: &OutputConfig) -> RobotResult<()>;

    /// 출력
    fn output(&self, data: &OutputData) -> RobotResult<OutputResult>;

    /// 사용 가능 여부
    fn is_available(&self) -> bool;

    /// 정리
    fn dispose(&mut self) -> RobotResult<()>;
}
```

### 2.2 출력 유형 (OutputType)

| Type | 설명 | 예시 |
|------|------|------|
| Visualization | 3D 시각화 | RViz2, Gazebo, Unity |
| Medical | 의료 시스템 | HL7 FHIR, DICOM |
| AiMl | AI/ML 데이터 | TensorFlow, PyTorch |
| Dashboard | 대시보드 | WebSocket, REST |
| Export | 데이터 내보내기 | JSON, CSV, URDF |
| Logger | 로깅 | File, Database |
| Alert | 알림 | Webhook, Email |

### 2.3 OutputData 구조

```rust
pub struct OutputData {
    /// 디바이스 ID
    pub device_id: String,
    /// 타임스탬프
    pub timestamp: DateTime<Utc>,
    /// 로봇 유형
    pub robot_type: String,
    /// 실제 데이터 (Phase 1 형식)
    pub data: serde_json::Value,
    /// 메타데이터
    pub metadata: Option<serde_json::Value>,
}
```

### 2.4 OutputResult 구조

```rust
pub struct OutputResult {
    /// 성공 여부
    pub success: bool,
    /// 결과 메시지
    pub message: String,
    /// 추가 메타데이터
    pub metadata: Option<serde_json::Value>,
    /// 처리 시간 (ms)
    pub duration_ms: u64,
}
```

---

## 3. 로봇 시각화 (Robot Visualization)

### 3.1 RViz2 Marker 매핑

| WIA Robot 데이터 | RViz2 Marker |
|-----------------|--------------|
| Joint position | SPHERE |
| Joint trajectory | LINE_STRIP |
| Robot body | CUBE_LIST |
| Workspace boundary | LINE_LIST |
| Safety zone | CYLINDER |
| Force vector | ARROW |
| Text status | TEXT_VIEW_FACING |

### 3.2 Marker 메시지 구조

```json
{
  "header": {
    "frame_id": "base_link",
    "stamp": { "sec": 0, "nanosec": 0 }
  },
  "ns": "wia_robot",
  "id": 0,
  "type": 2,
  "action": 0,
  "pose": {
    "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  },
  "scale": { "x": 0.05, "y": 0.05, "z": 0.05 },
  "color": { "r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0 },
  "lifetime": { "sec": 0, "nanosec": 0 }
}
```

### 3.3 URDF 생성

외골격 로봇의 기본 URDF 템플릿:

```xml
<?xml version="1.0"?>
<robot name="wia_exoskeleton">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="thigh_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- Additional joints and links -->
</robot>
```

### 3.4 Gazebo SDF 생성

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="wia_robot_world">
    <model name="wia_exoskeleton">
      <link name="base_link">
        <pose>0 0 0.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.2 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.2 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

---

## 4. 의료 시스템 연동 (Medical System Integration)

### 4.1 HL7 FHIR 리소스 매핑

| WIA Robot | FHIR Resource | Code System |
|-----------|---------------|-------------|
| ROM measurement | Observation | LOINC 89255-4 |
| Vital signs | Observation | LOINC vital-signs |
| EMG signal | DeviceMetric | IEEE 11073 |
| Safety event | DeviceAlert | Custom |
| Treatment session | Procedure | SNOMED CT |

### 4.2 FHIR Observation 예시

재활 로봇 ROM 측정 데이터:

```json
{
  "resourceType": "Observation",
  "id": "rom-001",
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/observation-category",
      "code": "therapy",
      "display": "Therapy"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "89255-4",
      "display": "Range of motion"
    }]
  },
  "subject": {
    "reference": "Patient/patient-001"
  },
  "effectiveDateTime": "2025-01-15T10:30:00Z",
  "valueQuantity": {
    "value": 85.5,
    "unit": "degrees",
    "system": "http://unitsofmeasure.org",
    "code": "deg"
  },
  "device": {
    "reference": "Device/rehab-robot-001"
  }
}
```

### 4.3 FHIR DeviceMetric 예시

EMG 센서 측정 특성:

```json
{
  "resourceType": "DeviceMetric",
  "id": "emg-metric-001",
  "type": {
    "coding": [{
      "system": "urn:iso:std:iso:11073:10101",
      "code": "150456",
      "display": "EMG signal"
    }]
  },
  "unit": {
    "coding": [{
      "system": "http://unitsofmeasure.org",
      "code": "mV",
      "display": "millivolt"
    }]
  },
  "source": {
    "reference": "Device/prosthetic-001"
  },
  "parent": {
    "reference": "Device/prosthetic-001"
  },
  "operationalStatus": "on",
  "category": "measurement",
  "measurementPeriod": {
    "repeat": {
      "frequency": 1000,
      "period": 1,
      "periodUnit": "s"
    }
  }
}
```

---

## 5. AI/ML 데이터 준비 (AI/ML Data Preparation)

### 5.1 학습 데이터 형식

**JSON Lines (.jsonl)**
```json
{"timestamp":1702483200,"device_id":"prosthetic-001","features":[0.5,0.3,0.8,0.2],"label":"grip"}
{"timestamp":1702483201,"device_id":"prosthetic-001","features":[0.1,0.1,0.1,0.1],"label":"rest"}
```

**CSV**
```csv
timestamp,device_id,emg_0,emg_1,emg_2,emg_3,accel_x,accel_y,accel_z,label
1702483200,prosthetic-001,0.5,0.3,0.8,0.2,0.1,-0.2,9.8,grip
1702483201,prosthetic-001,0.1,0.1,0.1,0.1,0.0,0.0,9.8,rest
```

### 5.2 특징 벡터 구성

| 데이터 소스 | 특징 |
|------------|------|
| EMG 센서 | signal_mv, activation_level |
| IMU | accel_xyz, gyro_xyz, orientation |
| 관절 | angle_deg, velocity_deg_s, torque_nm |
| 압력 센서 | pressure_kpa |
| 바이탈 | heart_rate, spo2, body_temp |

### 5.3 데이터셋 메타데이터

```json
{
  "dataset_name": "wia_emg_gestures",
  "version": "1.0.0",
  "created_at": "2025-01-15T10:00:00Z",
  "description": "EMG gesture recognition dataset from WIA prosthetic devices",
  "features": [
    {"name": "emg_0", "type": "float", "unit": "mV"},
    {"name": "emg_1", "type": "float", "unit": "mV"},
    {"name": "emg_2", "type": "float", "unit": "mV"},
    {"name": "emg_3", "type": "float", "unit": "mV"}
  ],
  "labels": ["rest", "grip", "pinch", "point", "open"],
  "sample_rate_hz": 1000,
  "total_samples": 100000,
  "participants": 10
}
```

---

## 6. 대시보드 연동 (Dashboard Integration)

### 6.1 WebSocket 메시지 형식

```json
{
  "type": "telemetry",
  "deviceId": "exo-001",
  "timestamp": "2025-01-15T10:30:00.123Z",
  "data": {
    "status": "operational",
    "battery": 85,
    "joints": [
      {"name": "hip_left", "angle": 15.5, "torque": 45.2}
    ],
    "gait": {
      "phase": "swing",
      "velocity": 0.5
    },
    "safety": {
      "level": "normal",
      "score": 95
    }
  }
}
```

### 6.2 대시보드 이벤트 유형

| Event | 용도 |
|-------|-----|
| telemetry | 실시간 센서 데이터 |
| status | 상태 업데이트 |
| alert | 경고/알림 |
| command | 제어 명령 응답 |
| config | 설정 변경 |

---

## 7. 데이터 내보내기 (Data Export)

### 7.1 지원 형식

| 형식 | 확장자 | 용도 |
|------|--------|------|
| JSON | .json | 범용 |
| JSON Lines | .jsonl | 스트리밍 |
| CSV | .csv | 스프레드시트, ML |
| URDF | .urdf | ROS 로봇 모델 |
| SDF | .sdf | Gazebo 시뮬레이션 |

### 7.2 JSON 내보내기 구조

```json
{
  "export_info": {
    "format": "wia-robot-export",
    "version": "1.0.0",
    "exported_at": "2025-01-15T10:30:00Z",
    "device_id": "exo-001"
  },
  "device": {
    "// Phase 1 device schema"
  },
  "state": {
    "// Phase 1 state schema"
  },
  "spec": {
    "// Phase 1 spec schema"
  },
  "safety": {
    "// Phase 1 safety schema"
  },
  "sessions": [
    {
      "session_id": "session-001",
      "start_time": "2025-01-15T09:00:00Z",
      "end_time": "2025-01-15T10:00:00Z",
      "data_points": 3600
    }
  ]
}
```

---

## 8. 통합 출력 매니저 (Output Manager)

### 8.1 매니저 API

```rust
impl OutputManager {
    /// 새 매니저 생성
    pub fn new() -> Self;

    /// 어댑터 등록
    pub fn register(&mut self, name: &str, adapter: Box<dyn OutputAdapter>);

    /// 특정 어댑터로 출력
    pub fn output_to(&self, name: &str, data: &OutputData) -> RobotResult<OutputResult>;

    /// 타입별 브로드캐스트
    pub fn broadcast(&self, output_type: OutputType, data: &OutputData) -> Vec<RobotResult<OutputResult>>;

    /// 활성 어댑터 조회
    pub fn get_available_adapters(&self) -> Vec<String>;

    /// 타입별 어댑터 조회
    pub fn get_by_type(&self, output_type: OutputType) -> Vec<String>;
}
```

### 8.2 사용 예시

```rust
use wia_robot::output::*;

// 매니저 생성
let mut manager = OutputManager::new();

// 어댑터 등록
manager.register("fhir", Box::new(FhirExporter::new("https://fhir.example.com")));
manager.register("csv", Box::new(CsvExporter::new("./data")));
manager.register("dashboard", Box::new(WebSocketDashboard::new("ws://dashboard.example.com")));

// 데이터 출력
let data = OutputData {
    device_id: "exo-001".to_string(),
    timestamp: Utc::now(),
    robot_type: "exoskeleton".to_string(),
    data: serde_json::json!({"joints": [...]}),
    metadata: None,
};

// 특정 어댑터로 출력
manager.output_to("fhir", &data)?;

// 모든 Medical 어댑터로 브로드캐스트
manager.broadcast(OutputType::Medical, &data);
```

---

## 9. 보안 고려사항 (Security Considerations)

### 9.1 의료 데이터 보호

- **HIPAA**: 미국 의료정보 보호법 준수
- **GDPR**: EU 개인정보 보호법 준수
- **암호화**: TLS 1.3 전송 암호화 필수
- **접근 제어**: Role-based 접근 제어

### 9.2 데이터 익명화

```rust
pub struct AnonymizationConfig {
    /// 환자 ID 해시화
    pub hash_patient_id: bool,
    /// 위치 정보 제거
    pub remove_location: bool,
    /// 타임스탬프 일반화
    pub generalize_timestamp: bool,
    /// 민감 필드 제거
    pub remove_sensitive_fields: Vec<String>,
}
```

---

## 10. 참고문헌 (References)

1. [ROS2 RViz Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/)
2. [Gazebo Simulation](https://gazebosim.org/docs/)
3. [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
4. [HL7 FHIR R4](https://www.hl7.org/fhir/)
5. [TensorFlow Datasets](https://www.tensorflow.org/datasets)
6. [URDF Specification](http://wiki.ros.org/urdf/XML)
7. [SDF Specification](http://sdformat.org/spec)

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-15
**Status**: Draft

---

<div align="center">

**WIA Robot Ecosystem Integration**

Connect Assistive Robots to the World

弘益人間 - Benefit All Humanity

</div>
