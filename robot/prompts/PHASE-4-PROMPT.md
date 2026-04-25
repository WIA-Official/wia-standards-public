# Phase 4: Ecosystem Integration
## Claude Code 작업 프롬프트

---

**Standard**: WIA Robot (Robotics Accessibility)
**Phase**: 4 of 4
**목표**: 보조 로봇 데이터를 외부 시스템과 연동
**난이도**: ★★★★★
**예상 작업량**: 스펙 문서 1개 + 출력 모듈 구현 + 예제

---

## 🎯 Phase 4 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의하고,
 Phase 2에서 API Interface를 만들고,
 Phase 3에서 Communication Protocol을 정의했다.

 이제 WIA Robot 데이터를 외부 시스템과 어떻게 연동할 것인가?

 - ROS2/RViz로 로봇 상태 시각화?
 - Gazebo로 재활 로봇 시뮬레이션?
 - 의료 시스템(EMR)과 치료 데이터 공유?
 - Unity/Unreal로 VR 재활 훈련?
 - 클라우드 AI 모델과 연동?

 모든 출력 방식에서 동일한 인터페이스를 사용할 수 있을까?"
```

### 목표
```
WIA Robot 데이터 → 외부 시스템 연동

출력 경로:
├─ Visualization: RViz, Gazebo, Unity
├─ Export: ROS2 Bags, URDF, STL
├─ Medical: HL7 FHIR, DICOM
├─ Dashboard: Web Dashboard, Mobile App
├─ AI/ML: TensorFlow, PyTorch 데이터셋
└─ Alert: Webhook, SMS, Email

단일 API로 모든 출력 방식 지원
```

---

## 📋 Phase 1-3 결과물 활용

| 이전 Phase 산출물 | Phase 4 활용 |
|-----------------|-------------|
| Phase 1: Data Format | 내보내기 데이터 소스 |
| Phase 2: Rust API | 데이터 처리 API |
| Phase 3: Protocol | 실시간 데이터 스트리밍 |
| Safety System | 안전 데이터 로깅 |

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 로봇 시각화 도구 조사

| 도구 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **RViz2** | ROS2 3D 시각화 | "RViz2 robot visualization marker topic" |
| **Gazebo** | 로봇 시뮬레이터 | "Gazebo robot simulation SDF URDF" |
| **Unity Robotics** | Unity 로봇 시뮬레이션 | "Unity robotics hub ROS integration" |
| **MuJoCo** | 물리 시뮬레이터 | "MuJoCo robot simulation biomechanics" |
| **Webots** | 로봇 시뮬레이터 | "Webots robot simulation rehabilitation" |

### 2단계: 의료 데이터 통합 조사

| 형식/표준 | 조사 대상 | 웹서치 키워드 |
|----------|----------|--------------|
| **HL7 FHIR** | 의료 데이터 교환 | "HL7 FHIR DeviceMetric Observation" |
| **DICOM** | 의료 영상 표준 | "DICOM structured report medical device" |
| **OpenEHR** | 전자건강기록 | "OpenEHR archetypes rehabilitation data" |
| **OMOP CDM** | 임상 데이터 모델 | "OMOP common data model device exposure" |

### 3단계: AI/ML 연동 조사

| 프레임워크 | 조사 대상 | 웹서치 키워드 |
|----------|----------|--------------|
| **TensorFlow** | 머신러닝 | "TensorFlow dataset robot telemetry" |
| **PyTorch** | 딥러닝 | "PyTorch time series EMG signal" |
| **ROS2 ML** | ROS2 머신러닝 | "ROS2 machine learning inference" |
| **ONNX** | 모델 교환 형식 | "ONNX robot control neural network" |

### 4단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-4.md`에 다음을 정리:

```markdown
# Phase 4 사전 조사 결과

## 1. 로봇 시각화 도구 비교

### RViz2
- 기능: [조사 내용]
- Marker 메시지: [조사 내용]
- WIA Robot 적용: [분석]

### Gazebo
- 기능: [조사 내용]
- SDF/URDF 형식: [조사 내용]
- WIA Robot 적용: [분석]

## 2. 의료 데이터 통합

### HL7 FHIR
- DeviceMetric 리소스: [조사 내용]
- Observation 리소스: [조사 내용]
- WIA Robot 매핑: [분석]

### DICOM
- Structured Report: [조사 내용]
- 의료 기기 통합: [조사 내용]

## 3. AI/ML 연동

### TensorFlow Dataset
- 데이터 파이프라인: [조사 내용]
- 시계열 데이터: [조사 내용]

## 4. 결론
- 권장 시각화 방식: [제안]
- 의료 데이터 매핑: [제안]
- AI 학습 데이터 형식: [제안]
```

---

## 🏗️ 출력 연동 설계

### 1. 출력 인터페이스 (Output Interface)

#### 기본 출력 인터페이스
```rust
use crate::{RobotResult, RobotError};
use async_trait::async_trait;
use serde_json::Value;

#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// 출력 유형
    fn output_type(&self) -> OutputType;

    /// 어댑터 이름
    fn name(&self) -> &str;

    /// 초기화
    async fn initialize(&mut self, config: &OutputConfig) -> RobotResult<()>;

    /// 출력
    async fn output(&self, data: &OutputData) -> RobotResult<OutputResult>;

    /// 사용 가능 여부
    fn is_available(&self) -> bool;

    /// 정리
    async fn dispose(&mut self) -> RobotResult<()>;
}

#[derive(Debug, Clone)]
pub enum OutputType {
    Visualization,  // 3D 시각화
    Export,         // 데이터 내보내기
    Medical,        // 의료 시스템
    Dashboard,      // 대시보드
    AiMl,           // AI/ML 데이터셋
    Alert,          // 알림
    Logger,         // 로깅
    Custom(String), // 사용자 정의
}

#[derive(Debug, Clone)]
pub struct OutputConfig {
    pub endpoint: Option<String>,
    pub format: String,
    pub options: serde_json::Map<String, Value>,
}

#[derive(Debug, Clone)]
pub struct OutputData {
    pub device_id: String,
    pub timestamp: chrono::DateTime<chrono::Utc>,
    pub robot_type: String,
    pub data: Value,
}

#[derive(Debug, Clone)]
pub struct OutputResult {
    pub success: bool,
    pub message: String,
    pub metadata: Option<Value>,
}
```

### 2. ROS2 연동

#### RViz2 Marker Exporter
```rust
#[cfg(feature = "ros2")]
use r2r::visualization_msgs::msg::Marker;

pub struct RVizMarkerExporter {
    name: String,
    namespace: String,
}

impl RVizMarkerExporter {
    /// 외골격 로봇을 RViz Marker로 변환
    pub fn exoskeleton_to_marker(
        &self,
        exo_data: &ExoskeletonSpec,
    ) -> RobotResult<Vec<Marker>> {
        let mut markers = Vec::new();

        // 관절 위치 마커
        for (idx, joint) in exo_data.joints.iter().enumerate() {
            let marker = Marker {
                header: r2r::std_msgs::msg::Header {
                    frame_id: "base_link".to_string(),
                    stamp: r2r::builtin_interfaces::msg::Time::default(),
                },
                ns: self.namespace.clone(),
                id: idx as i32,
                marker_type: 2, // SPHERE
                action: 0,      // ADD
                pose: r2r::geometry_msgs::msg::Pose {
                    position: r2r::geometry_msgs::msg::Point {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    orientation: r2r::geometry_msgs::msg::Quaternion {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0,
                    },
                },
                scale: r2r::geometry_msgs::msg::Vector3 {
                    x: 0.05,
                    y: 0.05,
                    z: 0.05,
                },
                color: r2r::std_msgs::msg::ColorRGBA {
                    r: 0.0,
                    g: 1.0,
                    b: 0.0,
                    a: 1.0,
                },
                lifetime: r2r::builtin_interfaces::msg::Duration {
                    sec: 0,
                    nanosec: 0,
                },
                ..Default::default()
            };
            markers.push(marker);
        }

        Ok(markers)
    }

    /// URDF 생성
    pub fn generate_urdf(&self, robot_data: &Value) -> RobotResult<String> {
        // URDF XML 생성
        let urdf = format!(
            r#"<?xml version="1.0"?>
<robot name="wia_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>"#
        );
        Ok(urdf)
    }
}
```

#### Gazebo SDF Exporter
```rust
pub struct GazeboSdfExporter {
    name: String,
    world_name: String,
}

impl GazeboSdfExporter {
    /// SDF (Simulation Description Format) 생성
    pub fn generate_sdf(&self, robot_data: &Value) -> RobotResult<String> {
        let sdf = format!(
            r#"<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="{}">
    <model name="wia_robot">
      <link name="base_link">
        <pose>0 0 0 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>"#,
            self.world_name
        );
        Ok(sdf)
    }
}
```

### 3. 의료 시스템 연동

#### HL7 FHIR Exporter
```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct FhirObservation {
    #[serde(rename = "resourceType")]
    pub resource_type: String,
    pub id: String,
    pub status: String,
    pub category: Vec<FhirCodeableConcept>,
    pub code: FhirCodeableConcept,
    pub subject: FhirReference,
    pub effective: FhirDateTime,
    pub value: FhirValue,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct FhirCodeableConcept {
    pub coding: Vec<FhirCoding>,
    pub text: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct FhirCoding {
    pub system: String,
    pub code: String,
    pub display: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct FhirReference {
    pub reference: String,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(untagged)]
pub enum FhirDateTime {
    DateTime(String),
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(untagged)]
pub enum FhirValue {
    Quantity(FhirQuantity),
}

#[derive(Debug, Serialize, Deserialize)]
pub struct FhirQuantity {
    pub value: f64,
    pub unit: String,
    pub system: String,
    pub code: String,
}

pub struct FhirExporter {
    server_url: String,
}

impl FhirExporter {
    /// 재활 로봇 데이터를 FHIR Observation으로 변환
    pub fn rehabilitation_to_fhir(
        &self,
        rehab_data: &RehabilitationSpec,
        patient_id: &str,
    ) -> RobotResult<FhirObservation> {
        Ok(FhirObservation {
            resource_type: "Observation".to_string(),
            id: uuid::Uuid::new_v4().to_string(),
            status: "final".to_string(),
            category: vec![FhirCodeableConcept {
                coding: vec![FhirCoding {
                    system: "http://terminology.hl7.org/CodeSystem/observation-category".to_string(),
                    code: "therapy".to_string(),
                    display: "Therapy".to_string(),
                }],
                text: Some("Rehabilitation Therapy".to_string()),
            }],
            code: FhirCodeableConcept {
                coding: vec![FhirCoding {
                    system: "http://loinc.org".to_string(),
                    code: "89255-4".to_string(),  // ROM measurement
                    display: "Range of motion".to_string(),
                }],
                text: Some("Range of Motion".to_string()),
            },
            subject: FhirReference {
                reference: format!("Patient/{}", patient_id),
            },
            effective: FhirDateTime::DateTime(
                chrono::Utc::now().to_rfc3339()
            ),
            value: FhirValue::Quantity(FhirQuantity {
                value: rehab_data.performance.rom_achieved_deg,
                unit: "degrees".to_string(),
                system: "http://unitsofmeasure.org".to_string(),
                code: "deg".to_string(),
            }),
        })
    }

    /// FHIR 서버로 전송
    pub async fn post_to_fhir_server(
        &self,
        observation: &FhirObservation,
    ) -> RobotResult<()> {
        let client = reqwest::Client::new();
        let response = client
            .post(format!("{}/Observation", self.server_url))
            .json(observation)
            .send()
            .await
            .map_err(|e| RobotError::CommunicationError(e.to_string()))?;

        if response.status().is_success() {
            Ok(())
        } else {
            Err(RobotError::CommunicationError(
                format!("FHIR server error: {}", response.status())
            ))
        }
    }
}
```

### 4. AI/ML 데이터 준비

#### TensorFlow Dataset Exporter
```rust
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::Write;

#[derive(Debug, Serialize, Deserialize)]
pub struct TrainingDatapoint {
    pub timestamp: i64,
    pub device_id: String,
    pub features: Vec<f64>,
    pub label: String,
}

pub struct TensorFlowExporter {
    output_dir: String,
}

impl TensorFlowExporter {
    /// EMG 신호를 학습 데이터로 변환
    pub fn emg_to_training_data(
        &self,
        prosthetic_data: &ProstheticSpec,
        label: &str,
    ) -> RobotResult<TrainingDatapoint> {
        // EMG 센서 데이터를 특징 벡터로 변환
        let features: Vec<f64> = prosthetic_data
            .emg_sensors
            .iter()
            .flat_map(|sensor| {
                vec![
                    sensor.signal_mv,
                    sensor.activation_level,
                ]
            })
            .collect();

        Ok(TrainingDatapoint {
            timestamp: chrono::Utc::now().timestamp(),
            device_id: "prosthetic_001".to_string(),
            features,
            label: label.to_string(),
        })
    }

    /// TFRecord 형식으로 저장
    pub fn save_as_tfrecord(
        &self,
        datapoints: &[TrainingDatapoint],
    ) -> RobotResult<()> {
        // TFRecord는 protobuf 기반이므로 실제로는 prost 등 사용
        // 여기서는 간단히 JSON Lines 형식으로 저장
        let file_path = format!("{}/training_data.jsonl", self.output_dir);
        let mut file = File::create(file_path)
            .map_err(|e| RobotError::IoError(e))?;

        for dp in datapoints {
            let json = serde_json::to_string(dp)?;
            writeln!(file, "{}", json)?;
        }

        Ok(())
    }

    /// CSV 형식으로 저장 (Pandas 호환)
    pub fn save_as_csv(
        &self,
        datapoints: &[TrainingDatapoint],
    ) -> RobotResult<()> {
        let file_path = format!("{}/training_data.csv", self.output_dir);
        let mut file = File::create(file_path)
            .map_err(|e| RobotError::IoError(e))?;

        // 헤더
        writeln!(file, "timestamp,device_id,features,label")?;

        // 데이터
        for dp in datapoints {
            let features_str = dp
                .features
                .iter()
                .map(|f| f.to_string())
                .collect::<Vec<_>>()
                .join(";");
            writeln!(
                file,
                "{},{},{},{}",
                dp.timestamp,
                dp.device_id,
                features_str,
                dp.label
            )?;
        }

        Ok(())
    }
}
```

### 5. Web Dashboard 연동

#### WebSocket Dashboard Exporter
```rust
use tokio_tungstenite::tungstenite::Message;

pub struct WebDashboardExporter {
    ws_url: String,
}

impl WebDashboardExporter {
    /// 실시간 텔레메트리 전송
    pub async fn stream_telemetry(
        &self,
        robot_data: &OutputData,
    ) -> RobotResult<()> {
        // WebSocket 연결 (실제 구현에서는 연결 풀 사용)
        let (mut ws_stream, _) = tokio_tungstenite::connect_async(&self.ws_url)
            .await
            .map_err(|e| RobotError::CommunicationError(e.to_string()))?;

        // JSON으로 직렬화
        let json = serde_json::to_string(&robot_data)?;
        let message = Message::Text(json);

        // 전송
        use futures_util::SinkExt;
        ws_stream
            .send(message)
            .await
            .map_err(|e| RobotError::CommunicationError(e.to_string()))?;

        Ok(())
    }

    /// 대시보드 데이터 형식
    pub fn format_for_dashboard(
        &self,
        robot_data: &OutputData,
    ) -> RobotResult<Value> {
        let dashboard_data = serde_json::json!({
            "deviceId": robot_data.device_id,
            "timestamp": robot_data.timestamp.to_rfc3339(),
            "type": robot_data.robot_type,
            "metrics": {
                "status": "operational",
                "battery": 85,
                "uptime": 3600,
            },
            "visualization": {
                "chart_type": "line",
                "data_points": [],
            }
        });

        Ok(dashboard_data)
    }
}
```

### 6. 통합 출력 매니저

```rust
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

pub struct OutputManager {
    adapters: Arc<RwLock<HashMap<String, Box<dyn OutputAdapter>>>>,
}

impl OutputManager {
    pub fn new() -> Self {
        Self {
            adapters: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// 어댑터 등록
    pub async fn register(
        &self,
        name: &str,
        adapter: Box<dyn OutputAdapter>,
    ) {
        let mut adapters = self.adapters.write().await;
        adapters.insert(name.to_string(), adapter);
    }

    /// 특정 어댑터로 출력
    pub async fn output_to(
        &self,
        name: &str,
        data: &OutputData,
    ) -> RobotResult<OutputResult> {
        let adapters = self.adapters.read().await;
        let adapter = adapters
            .get(name)
            .ok_or_else(|| RobotError::InvalidParameter(
                format!("Adapter not found: {}", name)
            ))?;

        adapter.output(data).await
    }

    /// 특정 타입의 모든 어댑터로 브로드캐스트
    pub async fn broadcast(
        &self,
        output_type: OutputType,
        data: &OutputData,
    ) -> Vec<RobotResult<OutputResult>> {
        let adapters = self.adapters.read().await;
        let mut results = Vec::new();

        for adapter in adapters.values() {
            if std::mem::discriminant(&adapter.output_type())
                == std::mem::discriminant(&output_type)
            {
                results.push(adapter.output(data).await);
            }
        }

        results
    }

    /// 활성 어댑터 목록
    pub async fn get_available_adapters(&self) -> Vec<String> {
        let adapters = self.adapters.read().await;
        adapters
            .iter()
            .filter(|(_, adapter)| adapter.is_available())
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// 타입별 어댑터 조회
    pub async fn get_by_type(
        &self,
        output_type: OutputType,
    ) -> Vec<String> {
        let adapters = self.adapters.read().await;
        adapters
            .iter()
            .filter(|(_, adapter)| {
                std::mem::discriminant(&adapter.output_type())
                    == std::mem::discriminant(&output_type)
            })
            .map(|(name, _)| name.clone())
            .collect()
    }
}
```

---

## 📁 산출물 목록

Phase 4 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-4.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-4-INTEGRATION.md

내용:
1. 개요 (Overview)
2. 출력 계층 아키텍처 (Output Layer Architecture)
3. 출력 인터페이스 (Output Interface)
4. 로봇 시각화 (Robot Visualization)
   - RViz2 (ROS2)
   - Gazebo Simulation
   - Unity/Unreal Engine
5. 의료 시스템 연동 (Medical System Integration)
   - HL7 FHIR
   - DICOM
   - OpenEHR
6. AI/ML 데이터 준비 (AI/ML Data Preparation)
   - TensorFlow Dataset
   - PyTorch Dataset
   - CSV Export
7. 대시보드 연동 (Dashboard Integration)
   - Web Dashboard
   - Mobile App
8. 데이터 내보내기 (Data Export)
   - ROS2 Bags
   - URDF/SDF
   - JSON/CSV
9. 알림 시스템 (Alert System)
   - Webhook
   - Email/SMS
10. 통합 출력 매니저 (Output Manager)
11. 예제 (Examples)
12. 참고문헌 (References)
```

### 3. Rust 출력 모듈
```
/api/rust/src/
├── output/
│   ├── mod.rs
│   ├── adapter.rs           # 출력 인터페이스
│   ├── manager.rs           # 통합 매니저
│   ├── visualization/
│   │   ├── mod.rs
│   │   ├── rviz.rs          # RViz2 연동
│   │   ├── gazebo.rs        # Gazebo SDF
│   │   └── urdf.rs          # URDF 생성
│   ├── medical/
│   │   ├── mod.rs
│   │   ├── fhir.rs          # HL7 FHIR
│   │   └── dicom.rs         # DICOM (선택)
│   ├── aiml/
│   │   ├── mod.rs
│   │   ├── tensorflow.rs    # TensorFlow Dataset
│   │   └── pytorch.rs       # PyTorch Dataset
│   ├── dashboard/
│   │   ├── mod.rs
│   │   └── websocket.rs     # WebSocket 스트리밍
│   ├── export/
│   │   ├── mod.rs
│   │   ├── rosbag.rs        # ROS2 Bags
│   │   ├── json.rs          # JSON 내보내기
│   │   └── csv.rs           # CSV 내보내기
│   └── error.rs             # 에러 타입
└── ...
```

### 4. 예제 코드
```
/api/rust/examples/
├── output_demo.rs           # 출력 계층 데모
├── rviz_visualization.rs    # RViz2 시각화
├── fhir_export.rs           # FHIR 내보내기
├── ml_dataset.rs            # ML 학습 데이터 생성
└── dashboard_streaming.rs   # 대시보드 스트리밍
```

---

## ✅ 완료 체크리스트

Phase 4 완료 전 확인:

```
□ 웹서치로 시각화/의료/AI 기술 조사 완료
□ /spec/RESEARCH-PHASE-4.md 작성 완료
□ /spec/PHASE-4-INTEGRATION.md 작성 완료
□ Rust output 모듈 구현 완료
□ 시각화 어댑터 구현
  □ RViz2 Marker Exporter
  □ Gazebo SDF Exporter
  □ URDF Generator
□ 의료 어댑터 구현
  □ HL7 FHIR Exporter
  □ FHIR 서버 연동
□ AI/ML 어댑터 구현
  □ TensorFlow Dataset Exporter
  □ CSV Exporter
□ 대시보드 어댑터 구현
  □ WebSocket Dashboard
□ 데이터 내보내기 구현
  □ JSON Exporter
  □ CSV Exporter
  □ ROS2 Bag (선택)
□ OutputManager 구현 완료
□ 단위 테스트 작성 완료
□ 테스트 통과
□ 예제 코드 완료
□ README 업데이트 (Phase 4 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 시각화/의료/AI 기술 조사
   ↓
2. /spec/RESEARCH-PHASE-4.md 작성
   ↓
3. 출력 인터페이스 설계
   ↓
4. /spec/PHASE-4-INTEGRATION.md 작성
   ↓
5. Rust OutputAdapter trait 정의
   ↓
6. RViz2/Gazebo 시각화 구현
   ↓
7. HL7 FHIR 의료 연동 구현
   ↓
8. TensorFlow/CSV AI 데이터 구현
   ↓
9. WebSocket 대시보드 구현
   ↓
10. JSON/CSV 내보내기 구현
   ↓
11. OutputManager 구현
   ↓
12. 테스트 작성 및 실행
   ↓
13. 예제 코드 작성
   ↓
14. 완료 체크리스트 확인
   ↓
15. WIA Robot Standard 완료! 🎉
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1-3 결과물과 연동 가능하도록 설계
✅ 출력 어댑터 추상화 (새로운 출력 방식 쉽게 추가)
✅ 표준 형식 지원 (FHIR, URDF, SDF)
✅ 비동기 처리 (async/await)
✅ 의료 데이터 보안 고려 (HIPAA, GDPR)
✅ AI 학습에 적합한 데이터 형식
✅ ROS2와 완벽한 호환
✅ 에러 처리 포함
```

### DON'T (하지 말 것)

```
❌ 특정 시각화 도구에만 종속
❌ 의료 데이터 보안 미고려
❌ 동기 블로킹 처리
❌ Phase 1-3 형식과 불일치
❌ 개인정보 평문 저장
❌ 에러 처리 누락
```

---

## 🔗 WIA Robot 전체 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                     보조 로봇 데이터                         │
│     (Exoskeleton, Prosthetics, Rehabilitation, Care...)    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 1: Data Format Standard                   │
│                    표준 JSON 스키마                          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 2: API Interface Standard                 │
│                 Rust API + Safety System                     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 3: Communication Protocol                 │
│                 WIA Robot Protocol (WRP)                     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 4: Ecosystem Integration                  │
│                     OutputManager                            │
├──────────┬──────────┬──────────┬──────────┬─────────────────┤
│ RViz/    │ HL7 FHIR │TensorFlow│Dashboard │ JSON/CSV        │
│ Gazebo   │ Medical  │ AI/ML    │ WebSocket│ Export          │
└────┬─────┴────┬─────┴────┬─────┴────┬─────┴────┬────────────┘
     │          │          │          │          │
     ▼          ▼          ▼          ▼          ▼
┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐
│ ROS2   │ │ EMR    │ │ AI     │ │  Web   │ │ Data   │
│Ecosystem│ │ System │ │ Model  │ │  App   │ │Analysis│
└────────┘ └────────┘ └────────┘ └────────┘ └────────┘
```

---

## 🚀 작업 시작

이제 Phase 4 작업을 시작하세요.

첫 번째 단계: **웹서치로 로봇 시각화 및 의료 연동 기술 조사**

```
검색 키워드: "RViz2 marker visualization robot ROS2"
```

화이팅! 🤖🔗

WIA Robot Standard의 마지막 Phase입니다.
완료되면 데이터 정의부터 외부 시스템 연동까지 전체 파이프라인이 완성됩니다!

---

<div align="center">

**Phase 4 of 4**

Ecosystem Integration

🎯 최종 목표: 데이터 → 시각화/의료/AI/대시보드

🌟 Robotics Accessibility for All 🌟

弘益人間 - Benefit All Humanity

</div>
