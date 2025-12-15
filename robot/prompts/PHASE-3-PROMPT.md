# Phase 3: Communication Protocol
## Claude Code 작업 프롬프트

---

**Standard**: WIA Robot (Robotics Accessibility)
**Phase**: 3 of 4
**목표**: 보조 로봇 시스템 간 통신 프로토콜 표준화
**난이도**: ★★★★★
**예상 작업량**: 스펙 문서 1개 + Protocol 구현 + 예제

---

## 🎯 Phase 3 목표

### 핵심 질문
```
"Phase 1에서 Data Format을 정의하고,
 Phase 2에서 API Interface를 만들었다.

 이제 보조 로봇들이 실제로 어떻게 통신할 것인가?

 - 외골격 로봇과 재활 로봇의 데이터 공유?
 - 의수와 AI 모델의 실시간 제어 통신?
 - 돌봄 로봇과 병원 시스템의 바이탈 데이터 전송?
 - 수술 로봇의 저지연 원격 제어?
 - ROS2 네트워크와의 호환?

 모든 통신 방식에서 동일한 메시지 형식을 사용할 수 있을까?"
```

### 목표
```
보조 로봇 시스템 간 통신을 위한
WIA Robot Protocol (WRP)을 정의한다.

- 메시지 형식 (Message Format)
- 연결 관리 (Connection Management)
- 에러 처리 (Error Handling)
- 실시간 제어 (Real-time Control)
- ROS2 호환성 (ROS2 Compatibility)
- 안전 프로토콜 (Safety Protocol)
```

---

## 📋 Phase 1 & 2 결과물 활용

| 이전 Phase 산출물 | Phase 3 활용 |
|-----------------|-------------|
| Phase 1: Data Format | 메시지 페이로드 (payload) |
| Phase 2: Rust API | 메시지 핸들러 연동 |
| JSON Schema | 메시지 검증 |
| Safety System | 안전 통신 프로토콜 |

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 로봇 통신 프로토콜 조사

| 프로토콜 | 조사 대상 | 웹서치 키워드 |
|---------|----------|--------------|
| **ROS2 DDS** | 로봇 운영체제 통신 | "ROS2 DDS middleware communication protocol" |
| **MQTT** | IoT 메시지 브로커 | "MQTT protocol robotics medical device" |
| **WebSocket** | 실시간 양방향 통신 | "WebSocket robot control low latency" |
| **OPC UA** | 산업 자동화 표준 | "OPC UA robot communication standard" |
| **EtherCAT** | 실시간 이더넷 | "EtherCAT real-time robot control" |

### 2단계: 의료/보조 기기 통신 표준 조사

| 표준 | 조사 내용 | 웹서치 키워드 |
|------|----------|--------------|
| **HL7 FHIR** | 의료 데이터 교환 | "HL7 FHIR medical device integration" |
| **IEEE 11073** | 의료 기기 통신 | "IEEE 11073 point-of-care medical device" |
| **DICOM** | 의료 영상 통신 | "DICOM medical device communication" |
| **ISO 13482** | 서비스 로봇 안전 | "ISO 13482 robot safety communication" |

### 3단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-3.md`에 다음을 정리:

```markdown
# Phase 3 사전 조사 결과

## 1. 로봇 통신 프로토콜 비교

### ROS2 DDS (Data Distribution Service)
- 개요: [조사 내용]
- 특징: Pub/Sub 패턴, QoS 설정
- WIA Robot 적용: [분석]

### MQTT
- 개요: [조사 내용]
- 특징: 경량, Broker 기반
- WIA Robot 적용: [분석]

### WebSocket
- 개요: [조사 내용]
- 특징: 양방향, 저지연
- WIA Robot 적용: [분석]

## 2. 의료 기기 통신 표준

### HL7 FHIR
- 데이터 모델: [조사 내용]
- REST API: [조사 내용]
- WIA Robot 적용: [분석]

### IEEE 11073
- PHD (Personal Health Device): [조사 내용]
- 의료 기기 통신: [조사 내용]

## 3. 결론
- 권장 프로토콜 아키텍처: [제안]
- 메시지 형식 설계 방향: [제안]
- ROS2 호환 전략: [제안]
```

---

## 🏗️ 프로토콜 설계

### 1. 메시지 형식 (Message Format)

#### 기본 메시지 구조
```json
{
  "protocol": "wia-robot",
  "version": "1.0.0",
  "messageId": "uuid-v4",
  "timestamp": 1702483200000,
  "type": "메시지 유형",
  "priority": "normal|high|critical|emergency",
  "source": {
    "deviceId": "송신 로봇 ID",
    "deviceType": "로봇 유형",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780
    }
  },
  "destination": {
    "deviceId": "수신 로봇/시스템 ID",
    "deviceType": "목적지 유형"
  },
  "safety": {
    "emergencyStop": false,
    "safetyLevel": "normal",
    "requiresAck": true
  },
  "payload": {
    "메시지 데이터 (Phase 1 형식)"
  },
  "checksum": "CRC32 or SHA256"
}
```

#### 메시지 유형 (Message Types)

| Type | 방향 | 설명 | 우선순위 |
|------|-----|------|---------|
| `handshake` | Both | 연결 설정 | High |
| `heartbeat` | Both | 연결 유지 | Normal |
| `telemetry` | Device → Server | 센서 데이터 | Normal |
| `control` | Server → Device | 제어 명령 | High |
| `control_ack` | Device → Server | 제어 응답 | High |
| `emergency_stop` | Both | 긴급 정지 | Emergency |
| `safety_alert` | Both | 안전 경보 | Critical |
| `status` | Device → Server | 상태 업데이트 | Normal |
| `config` | Server → Device | 설정 변경 | High |
| `error` | Both | 에러 메시지 | High |
| `log` | Device → Server | 로그 데이터 | Low |

### 2. 연결 상태 관리 (Connection State Machine)

```
┌─────────────┐
│ DISCONNECTED│
└──────┬──────┘
       │ handshake()
       ▼
┌─────────────┐
│  CONNECTING │
└──────┬──────┘
       │ handshake_ack received
       ▼
┌─────────────┐
│   ACTIVE    │◄──────┐
└──────┬──────┘       │
       │              │ reconnect
       │ timeout/     │
       │ error        │
       ▼              │
┌─────────────┐       │
│ RECONNECTING├───────┘
└──────┬──────┘
       │ max retries exceeded
       │ or emergency_stop
       ▼
┌─────────────┐
│   STOPPED   │
└─────────────┘
```

### 3. 안전 프로토콜 (Safety Protocol)

#### Emergency Stop Sequence
```
1. 디바이스가 비상 상황 감지
   ↓
2. 즉시 emergency_stop 메시지 브로드캐스트
   ↓
3. 모든 수신 디바이스는 즉시 정지
   ↓
4. 정지 확인 응답 (Ack)
   ↓
5. 상태 진단 및 복구 대기
```

#### Safety Level
```rust
pub enum SafetyLevel {
    Normal,      // 정상 동작
    Warning,     // 경고 상태
    Caution,     // 주의 상태
    Critical,    // 위험 상태
    Emergency,   // 긴급 상태
}
```

### 4. QoS (Quality of Service) 설정

| 메시지 유형 | Reliability | Durability | Latency |
|-----------|-------------|------------|---------|
| Emergency Stop | Reliable | Transient | < 10ms |
| Control | Reliable | Volatile | < 50ms |
| Telemetry | Best Effort | Volatile | < 100ms |
| Status | Reliable | Transient | < 200ms |
| Log | Best Effort | Persistent | N/A |

---

## 🔧 Rust Protocol 구현

### Protocol 메시지 (protocol/message.rs)
```rust
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WrpMessage {
    pub protocol: String,
    pub version: String,
    pub message_id: Uuid,
    pub timestamp: DateTime<Utc>,
    pub message_type: MessageType,
    pub priority: Priority,
    pub source: Endpoint,
    pub destination: Endpoint,
    pub safety: SafetyInfo,
    pub payload: serde_json::Value,
    pub checksum: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    Handshake,
    Heartbeat,
    Telemetry,
    Control,
    ControlAck,
    EmergencyStop,
    SafetyAlert,
    Status,
    Config,
    Error,
    Log,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Priority {
    Low,
    Normal,
    High,
    Critical,
    Emergency,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Endpoint {
    pub device_id: String,
    pub device_type: String,
    pub location: Option<Location>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Location {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyInfo {
    pub emergency_stop: bool,
    pub safety_level: SafetyLevel,
    pub requires_ack: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SafetyLevel {
    Normal,
    Warning,
    Caution,
    Critical,
    Emergency,
}

impl WrpMessage {
    /// 새 메시지 생성
    pub fn new(
        message_type: MessageType,
        source: Endpoint,
        destination: Endpoint,
        payload: serde_json::Value,
    ) -> Self {
        Self {
            protocol: "wia-robot".to_string(),
            version: "1.0.0".to_string(),
            message_id: Uuid::new_v4(),
            timestamp: Utc::now(),
            message_type,
            priority: Priority::Normal,
            source,
            destination,
            safety: SafetyInfo {
                emergency_stop: false,
                safety_level: SafetyLevel::Normal,
                requires_ack: false,
            },
            payload,
            checksum: None,
        }
    }

    /// 긴급 정지 메시지 생성
    pub fn emergency_stop(source: Endpoint) -> Self {
        let mut msg = Self::new(
            MessageType::EmergencyStop,
            source,
            Endpoint {
                device_id: "broadcast".to_string(),
                device_type: "all".to_string(),
                location: None,
            },
            serde_json::json!({"reason": "Emergency stop activated"}),
        );
        msg.priority = Priority::Emergency;
        msg.safety.emergency_stop = true;
        msg.safety.safety_level = SafetyLevel::Emergency;
        msg.safety.requires_ack = true;
        msg
    }

    /// 체크섬 계산
    pub fn calculate_checksum(&mut self) {
        let serialized = serde_json::to_string(&self).unwrap_or_default();
        let checksum = format!("{:x}", crc32fast::hash(serialized.as_bytes()));
        self.checksum = Some(checksum);
    }

    /// 체크섬 검증
    pub fn verify_checksum(&self) -> bool {
        if let Some(expected) = &self.checksum {
            let mut msg = self.clone();
            msg.checksum = None;
            let serialized = serde_json::to_string(&msg).unwrap_or_default();
            let actual = format!("{:x}", crc32fast::hash(serialized.as_bytes()));
            &actual == expected
        } else {
            true  // 체크섬 없으면 검증 통과
        }
    }
}
```

### Protocol Handler (protocol/handler.rs)
```rust
use crate::{RobotResult, RobotError};
use crate::protocol::message::{WrpMessage, MessageType, Priority};
use tokio::sync::{mpsc, RwLock};
use std::collections::HashMap;
use std::sync::Arc;

pub type MessageCallback = Arc<dyn Fn(WrpMessage) -> RobotResult<()> + Send + Sync>;

pub struct ProtocolHandler {
    callbacks: Arc<RwLock<HashMap<MessageType, Vec<MessageCallback>>>>,
    tx: mpsc::UnboundedSender<WrpMessage>,
    rx: Arc<RwLock<mpsc::UnboundedReceiver<WrpMessage>>>,
}

impl ProtocolHandler {
    pub fn new() -> Self {
        let (tx, rx) = mpsc::unbounded_channel();
        Self {
            callbacks: Arc::new(RwLock::new(HashMap::new())),
            tx,
            rx: Arc::new(RwLock::new(rx)),
        }
    }

    /// 메시지 타입별 콜백 등록
    pub async fn register_callback<F>(&self, msg_type: MessageType, callback: F)
    where
        F: Fn(WrpMessage) -> RobotResult<()> + Send + Sync + 'static,
    {
        let mut callbacks = self.callbacks.write().await;
        callbacks
            .entry(msg_type)
            .or_insert_with(Vec::new)
            .push(Arc::new(callback));
    }

    /// 메시지 처리 루프 시작
    pub async fn start(&self) -> RobotResult<()> {
        loop {
            let msg = {
                let mut rx = self.rx.write().await;
                rx.recv().await
            };

            match msg {
                Some(message) => {
                    self.handle_message(message).await?;
                }
                None => {
                    return Err(RobotError::CommunicationError(
                        "Message channel closed".into()
                    ));
                }
            }
        }
    }

    /// 메시지 수신 및 처리
    async fn handle_message(&self, message: WrpMessage) -> RobotResult<()> {
        // 체크섬 검증
        if !message.verify_checksum() {
            return Err(RobotError::CommunicationError(
                "Checksum verification failed".into()
            ));
        }

        // 긴급 정지 메시지는 최우선 처리
        if message.safety.emergency_stop {
            println!("EMERGENCY STOP received!");
            // 긴급 정지 처리 로직
        }

        // 콜백 실행
        let callbacks = self.callbacks.read().await;
        if let Some(cbs) = callbacks.get(&message.message_type) {
            for callback in cbs {
                callback(message.clone())?;
            }
        }

        Ok(())
    }

    /// 메시지 전송
    pub fn send(&self, message: WrpMessage) -> RobotResult<()> {
        self.tx.send(message).map_err(|e| {
            RobotError::CommunicationError(format!("Failed to send message: {}", e))
        })
    }
}
```

### Transport 추상화 (transport/base.rs)
```rust
use crate::{RobotResult, RobotError};
use crate::protocol::message::WrpMessage;
use async_trait::async_trait;

#[async_trait]
pub trait Transport: Send + Sync {
    /// 전송 유형
    fn transport_type(&self) -> TransportType;

    /// 연결
    async fn connect(&mut self, config: &TransportConfig) -> RobotResult<()>;

    /// 연결 해제
    async fn disconnect(&mut self) -> RobotResult<()>;

    /// 메시지 전송
    async fn send(&self, message: &WrpMessage) -> RobotResult<()>;

    /// 메시지 수신
    async fn receive(&self) -> RobotResult<WrpMessage>;

    /// 연결 상태
    fn is_connected(&self) -> bool;

    /// 지연 시간 (ms)
    fn latency_ms(&self) -> u64;
}

#[derive(Debug, Clone)]
pub enum TransportType {
    WebSocket,
    Mqtt,
    Ros2Dds,
    Mock,  // 테스트용
}

#[derive(Debug, Clone)]
pub struct TransportConfig {
    pub endpoint: String,
    pub port: u16,
    pub timeout_ms: u64,
    pub retry_count: u32,
    pub use_tls: bool,
}
```

### Mock Transport (테스트용)
```rust
use crate::{RobotResult, RobotError};
use crate::protocol::message::WrpMessage;
use crate::transport::base::{Transport, TransportType, TransportConfig};
use async_trait::async_trait;
use tokio::sync::RwLock;
use std::collections::VecDeque;

pub struct MockTransport {
    connected: RwLock<bool>,
    send_queue: RwLock<VecDeque<WrpMessage>>,
    receive_queue: RwLock<VecDeque<WrpMessage>>,
    latency_ms: u64,
}

impl MockTransport {
    pub fn new() -> Self {
        Self {
            connected: RwLock::new(false),
            send_queue: RwLock::new(VecDeque::new()),
            receive_queue: RwLock::new(VecDeque::new()),
            latency_ms: 0,
        }
    }

    /// 테스트용: 수신 큐에 메시지 추가
    pub async fn enqueue_receive(&self, message: WrpMessage) {
        let mut queue = self.receive_queue.write().await;
        queue.push_back(message);
    }

    /// 테스트용: 전송 큐에서 메시지 가져오기
    pub async fn dequeue_sent(&self) -> Option<WrpMessage> {
        let mut queue = self.send_queue.write().await;
        queue.pop_front()
    }
}

#[async_trait]
impl Transport for MockTransport {
    fn transport_type(&self) -> TransportType {
        TransportType::Mock
    }

    async fn connect(&mut self, _config: &TransportConfig) -> RobotResult<()> {
        let mut connected = self.connected.write().await;
        *connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> RobotResult<()> {
        let mut connected = self.connected.write().await;
        *connected = false;
        Ok(())
    }

    async fn send(&self, message: &WrpMessage) -> RobotResult<()> {
        if !self.is_connected() {
            return Err(RobotError::CommunicationError(
                "Not connected".into()
            ));
        }

        // 지연 시뮬레이션
        if self.latency_ms > 0 {
            tokio::time::sleep(tokio::time::Duration::from_millis(self.latency_ms)).await;
        }

        let mut queue = self.send_queue.write().await;
        queue.push_back(message.clone());
        Ok(())
    }

    async fn receive(&self) -> RobotResult<WrpMessage> {
        if !self.is_connected() {
            return Err(RobotError::CommunicationError(
                "Not connected".into()
            ));
        }

        let mut queue = self.receive_queue.write().await;
        queue.pop_front().ok_or_else(|| {
            RobotError::CommunicationError("No message available".into())
        })
    }

    fn is_connected(&self) -> bool {
        // RwLock을 동기적으로 읽을 수 없으므로 간단히 처리
        // 실제 구현에서는 AtomicBool 사용 권장
        true
    }

    fn latency_ms(&self) -> u64 {
        self.latency_ms
    }
}
```

### ROS2 브릿지 (선택)
```rust
// ROS2 DDS 연동을 위한 브릿지
// r2r 크레이트 사용

#[cfg(feature = "ros2")]
use r2r::QosProfile;

#[cfg(feature = "ros2")]
pub struct Ros2Bridge {
    // ROS2 노드 및 퍼블리셔/서브스크라이버
}

#[cfg(feature = "ros2")]
impl Ros2Bridge {
    /// WRP 메시지를 ROS2 메시지로 변환
    pub fn wrp_to_ros2(&self, message: &WrpMessage) -> RobotResult<()> {
        // 변환 로직
        Ok(())
    }

    /// ROS2 메시지를 WRP 메시지로 변환
    pub fn ros2_to_wrp(&self) -> RobotResult<WrpMessage> {
        // 변환 로직
        unimplemented!()
    }
}
```

---

## 📁 산출물 목록

Phase 3 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-3.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-3-PROTOCOL.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 메시지 형식 (Message Format)
4. 메시지 유형 (Message Types)
5. 연결 관리 (Connection Management)
6. 안전 프로토콜 (Safety Protocol)
7. QoS 설정 (Quality of Service)
8. 에러 처리 (Error Handling)
9. 전송 계층 (Transport Layer)
   - WebSocket
   - MQTT
   - ROS2 DDS
   - Mock (테스트용)
10. ROS2 호환성 (ROS2 Compatibility)
11. 보안 (Security)
12. 예제 (Examples)
13. 참고문헌 (References)
```

### 3. JSON Schema
```
/spec/schemas/
├── wrp-message.schema.json     # 프로토콜 메시지 스키마
├── wrp-safety.schema.json      # 안전 메시지 스키마
└── wrp-error.schema.json       # 에러 메시지 스키마
```

### 4. Rust Protocol 구현
```
/api/rust/src/
├── protocol/
│   ├── mod.rs
│   ├── message.rs           # 메시지 타입 정의
│   ├── builder.rs           # 메시지 생성
│   ├── handler.rs           # 프로토콜 처리
│   └── error.rs             # 프로토콜 에러
├── transport/
│   ├── mod.rs
│   ├── base.rs              # 전송 인터페이스
│   ├── mock.rs              # 테스트용
│   ├── websocket.rs         # WebSocket (선택)
│   └── ros2.rs              # ROS2 브릿지 (선택)
└── ...
```

### 5. 예제 코드
```
/api/rust/examples/
├── protocol_demo.rs         # 프로토콜 데모
├── emergency_stop.rs        # 긴급 정지 시나리오
└── ros2_bridge.rs           # ROS2 연동 예제
```

---

## ✅ 완료 체크리스트

Phase 3 완료 전 확인:

```
□ 웹서치로 로봇 통신 프로토콜 조사 완료
□ 의료 기기 통신 표준 조사 완료
□ /spec/RESEARCH-PHASE-3.md 작성 완료
□ /spec/PHASE-3-PROTOCOL.md 작성 완료
□ 메시지 형식 JSON Schema 정의 완료
□ Rust protocol 모듈 구현 완료
  □ WrpMessage 타입 정의
  □ ProtocolHandler 구현
  □ Safety Protocol 구현
  □ 체크섬 검증 구현
□ Rust transport 모듈 구현 완료
  □ Transport trait 정의
  □ MockTransport 테스트용 구현
□ 긴급 정지 프로토콜 구현 완료
□ 단위 테스트 작성 완료
□ 테스트 통과
□ 예제 코드 완료
□ README 업데이트 (Phase 3 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 로봇 통신 프로토콜 조사
   ↓
2. /spec/RESEARCH-PHASE-3.md 작성
   ↓
3. 프로토콜 설계
   ↓
4. /spec/PHASE-3-PROTOCOL.md 작성
   ↓
5. 메시지 형식 JSON Schema 작성
   ↓
6. Rust protocol/message.rs 구현
   ↓
7. Rust protocol/handler.rs 구현
   ↓
8. Rust transport/base.rs 구현
   ↓
9. Rust transport/mock.rs 구현
   ↓
10. 긴급 정지 프로토콜 구현
   ↓
11. 테스트 작성 및 실행
   ↓
12. 예제 코드 작성
   ↓
13. 완료 체크리스트 확인
   ↓
14. Phase 4 시작 가능
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1 Data Format을 메시지 payload로 사용
✅ Phase 2 Rust API와 연동 가능하도록 설계
✅ 전송 계층 추상화 (다양한 전송 방식 지원)
✅ 안전 프로토콜 최우선 (Emergency Stop)
✅ 메시지 체크섬 검증
✅ 비동기 처리 (tokio async/await)
✅ ROS2 DDS와 호환 가능한 QoS 설계
✅ 의료 기기 표준 고려 (HL7 FHIR, IEEE 11073)
```

### DON'T (하지 말 것)

```
❌ 특정 전송 방식에만 종속되는 설계
❌ 안전 검증 없는 긴급 제어
❌ Phase 1/2 형식과 불일치
❌ 동기 블로킹 통신
❌ 체크섬 없는 중요 메시지
❌ 에러 처리 없는 통신
```

---

## 🔗 참고 자료

- **ROS2**: Robot Operating System 2 - https://docs.ros.org/
- **DDS**: Data Distribution Service - https://www.omg.org/spec/DDS/
- **MQTT**: Message Queuing Telemetry Transport - https://mqtt.org/
- **HL7 FHIR**: Fast Healthcare Interoperability Resources - https://www.hl7.org/fhir/
- **IEEE 11073**: Personal Health Device Communication - https://11073.org/
- **ISO 13482**: Robots and robotic devices - Safety requirements for personal care robots

---

## 🚀 작업 시작

이제 Phase 3 작업을 시작하세요.

첫 번째 단계: **웹서치로 로봇 통신 프로토콜 조사**

```
검색 키워드: "ROS2 DDS middleware communication protocol robotics"
```

화이팅! 🤖📡

---

<div align="center">

**Phase 3 of 4**

WIA Robot Protocol (WRP)

🔗 Safe, Real-time, Accessible Communication 🔗

弘益人間 - Benefit All Humanity

</div>
