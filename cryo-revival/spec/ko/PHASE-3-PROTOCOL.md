# WIA 냉동인간 소생 통신 프로토콜
## 3단계 명세

---

**버전**: 1.0.0
**상태**: Draft
**작성일**: 2025-01
**작성자**: WIA 표준 위원회
**라이선스**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## 목차

1. [개요](#개요)
2. [프로토콜 아키텍처](#프로토콜-아키텍처)
3. [전송 계층](#전송-계층)
4. [메시지 형식](#메시지-형식)
5. [연결 생명주기](#연결-생명주기)
6. [메시지 유형](#메시지-유형)
7. [보안](#보안)
8. [오류 처리](#오류-처리)
9. [예제](#예제)

---

## 개요

### 1.1 목적

WIA 냉동인간 소생 통신 프로토콜은 소생 절차 중 생체 징후 모니터링, 의료진 조정, 중요 경고 전달 및 통합 의료 시스템으로의 환자 데이터 스트리밍을 위한 실시간 통신 표준을 정의합니다.

**핵심 목표**:
- 소생 절차 중 실시간 생체 징후 모니터링
- 의료진에게 즉각적인 중요 경고 알림 전달
- 의료 기기와 모니터링 시스템 간 안전한 양방향 통신
- 확인 및 재시도 메커니즘을 통한 신뢰할 수 있는 메시지 전달
- 병원 모니터링 및 경고 인프라와의 통합

### 1.2 프로토콜 선택

| 사용 사례 | 권장 프로토콜 | 근거 |
|----------|--------------|------|
| 실시간 생체 징후 | WebSocket | 낮은 대기 시간, 양방향 |
| 의료 기기 데이터 | MQTT | IoT 최적화, QoS 지원 |
| 시설 간 동기화 | gRPC | 높은 처리량, 타입 안전 |
| 경고 알림 | Server-Sent Events (SSE) | 단방향 푸시, 간단함 |
| 응급 방송 | WebSocket + Redis PubSub | 다중 수신자, 즉각적 |

### 1.3 설계 원칙

1. **의료급 신뢰성**: 확인을 통한 최소 한 번 전달 보장
2. **초저 지연**: 중요 경고의 100ms 미만 메시지 전달
3. **HIPAA 준수**: 종단간 암호화 및 감사 로깅
4. **장애 허용**: 자동 재연결 및 메시지 버퍼링
5. **확장성**: 여러 동시 소생 절차 지원

---

## 프로토콜 아키텍처

### 2.1 계층 모델

```
┌──────────────────────────────────────────────────────────────┐
│                    애플리케이션 계층                           │
│         (소생 모니터링, 팀 조정)                              │
├──────────────────────────────────────────────────────────────┤
│                    프로토콜 계층                              │
│        (메시지 형식, 핸들러, 확인)                            │
├──────────────────────────────────────────────────────────────┤
│                    보안 계층                                  │
│         (TLS 1.3, JWT 인증, 메시지 암호화)                   │
├──────────────────────────────────────────────────────────────┤
│                    전송 계층                                  │
│           (WebSocket / MQTT / gRPC / SSE)                    │
├──────────────────────────────────────────────────────────────┤
│                    네트워크 계층                              │
│                     (TCP/IP)                                  │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 구성 요소 아키텍처

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  생체 징후      │     │  의료진         │     │  경고 시스템     │
│    모니터       │     │  대시보드       │     │   (모바일)      │
└────────┬────────┘     └────────┬────────┘     └────────┬────────┘
         │                       │                       │
         │ MQTT                  │ WebSocket             │ WebSocket
         │                       │                       │
         ▼                       ▼                       ▼
┌──────────────────────────────────────────────────────────────────┐
│              메시지 브로커 / 게이트웨이 (Redis PubSub)            │
│       (인증, 라우팅, 버퍼링, 팬아웃)                              │
└──────────────────────────────────────────────────────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  생체 징후      │     │  경고 서비스    │     │  감사 로그      │
│    데이터       │     │  (응급)         │     │   서비스        │
│    서비스       │     │                 │     │                 │
└─────────────────┘     └─────────────────┘     └─────────────────┘
         │                                               │
         ▼                                               ▼
┌─────────────────┐                             ┌─────────────────┐
│  EHR/FHIR       │                             │  규정 준수      │
│  통합           │                             │  스토리지       │
└─────────────────┘                             └─────────────────┘
```

---

## 전송 계층

### 3.1 WebSocket (주요)

**연결 URL:**
```
wss://ws.wia.live/cryo-revival/v1
```

**기본 포트:** 443 (WSS)

**서브프로토콜:** `wia-revival-v1`

**연결 예제:**
```javascript
const ws = new WebSocket(
  'wss://ws.wia.live/cryo-revival/v1',
  'wia-revival-v1',
  {
    headers: {
      'Authorization': `Bearer ${jwt_token}`,
      'X-Facility-ID': 'FAC-KR-REVIVAL-001'
    }
  }
);
```

**사용 사례:**
- 실시간 생체 징후 스트리밍
- 의료진 채팅 및 조정
- 응급 경고 방송
- 절차 상태 업데이트

### 3.2 MQTT (의료 기기)

**브로커 URL:**
```
mqtts://mqtt.wia.live:8883
```

**토픽 구조:**
```
wia/revival/{facility_id}/{revival_id}/{data_type}

예시:
wia/revival/FAC-KR-REVIVAL-001/REV-2025-001/vitals
wia/revival/FAC-KR-REVIVAL-001/REV-2025-001/heart_rate
wia/revival/FAC-KR-REVIVAL-001/REV-2025-001/temperature
wia/revival/FAC-KR-REVIVAL-001/REV-2025-001/alerts
wia/revival/FAC-KR-REVIVAL-001/+/critical_alerts
```

**QoS 수준:**
| QoS | 사용 사례 | 예시 |
|-----|----------|------|
| 0 | 일상적인 생체 징후 (고빈도) | 5초마다 온도 측정 |
| 1 | 중요한 상태 업데이트 | 단계 전환 |
| 2 | 중요 경고 및 의료 결정 | 심정지, 심각한 생체 징후 |

**MQTT 메시지 형식:**
```json
{
  "deviceId": "MONITOR-001",
  "revivalId": "REV-2025-001",
  "timestamp": "2025-01-15T19:00:00.123Z",
  "data": {
    "heart_rate": 72,
    "source": "cardiac_monitor",
    "quality": "good"
  }
}
```

### 3.3 gRPC (서비스 간)

**Proto 정의:**
```protobuf
syntax = "proto3";

package wia.revival.v1;

service RevivalMonitoring {
  // 소생 절차의 생체 징후 스트리밍
  rpc StreamVitals(RevivalRequest) returns (stream VitalSigns);

  // 중요 경고 전송
  rpc SendCriticalAlert(Alert) returns (AlertAck);

  // 현재 소생 상태 가져오기
  rpc GetRevivalStatus(StatusRequest) returns (RevivalStatus);

  // 양방향 팀 조정
  rpc CoordinateTeam(stream TeamMessage) returns (stream TeamMessage);
}

message VitalSigns {
  string revival_id = 1;
  int64 timestamp = 2;
  int32 heart_rate = 3;
  BloodPressure blood_pressure = 4;
  double body_temperature = 5;
  int32 oxygen_saturation = 6;
  int32 respiratory_rate = 7;
  NeurologicalStatus neurological_status = 8;
}

message BloodPressure {
  int32 systolic = 1;
  int32 diastolic = 2;
}

message NeurologicalStatus {
  int32 glasgow_coma_scale = 1;
  string pupil_response = 2;
  string eeg_activity = 3;
}

message Alert {
  string revival_id = 1;
  string alert_type = 2;
  string severity = 3;
  string message = 4;
  int64 timestamp = 5;
  map<string, string> metadata = 6;
}

message AlertAck {
  string alert_id = 1;
  bool acknowledged = 2;
  int64 ack_timestamp = 3;
}
```

**gRPC 사용 예제:**
```go
import (
    pb "wia.live/cryo-revival/v1"
    "google.golang.org/grpc"
)

conn, _ := grpc.Dial("grpc.wia.live:443", grpc.WithTransportCredentials(...))
client := pb.NewRevivalMonitoringClient(conn)

stream, _ := client.StreamVitals(context.Background(), &pb.RevivalRequest{
    RevivalId: "REV-2025-001",
})

for {
    vitals, err := stream.Recv()
    if err != nil {
        break
    }

    fmt.Printf("심박수: %d\n", vitals.HeartRate)

    if vitals.HeartRate > 150 {
        client.SendCriticalAlert(context.Background(), &pb.Alert{
            RevivalId: vitals.RevivalId,
            AlertType: "tachycardia",
            Severity: "critical",
            Message: "심박수가 임계값을 초과했습니다",
        })
    }
}
```

### 3.4 Server-Sent Events (경고)

**엔드포인트:**
```
GET /api/v1/revivals/{revivalId}/events/stream
Accept: text/event-stream
Authorization: Bearer <token>
```

**이벤트 형식:**
```
event: vital_signs_update
data: {"revivalId":"REV-2025-001","heart_rate":72,"timestamp":"2025-01-15T19:00:00Z"}

event: critical_alert
data: {"revivalId":"REV-2025-001","type":"tachycardia","severity":"critical"}

event: status_change
data: {"revivalId":"REV-2025-001","previousStatus":"warming","newStatus":"perfusion_reversal"}

event: heartbeat
data: {"timestamp":1704067200}
```

**SSE 클라이언트 예제:**
```javascript
const eventSource = new EventSource(
  'https://api.wia.live/cryo-revival/v1/revivals/REV-2025-001/events/stream',
  {
    headers: {
      'Authorization': `Bearer ${token}`
    }
  }
);

eventSource.addEventListener('critical_alert', (event) => {
  const alert = JSON.parse(event.data);
  displayEmergencyAlert(alert);
  notifyMedicalTeam(alert);
});

eventSource.addEventListener('vital_signs_update', (event) => {
  const vitals = JSON.parse(event.data);
  updateDashboard(vitals);
});
```

---

## 메시지 형식

### 4.1 기본 메시지 구조

모든 WIA 소생 프로토콜 메시지는 다음 구조를 따릅니다:

```json
{
  "protocol": "wia-revival",
  "version": "1.0.0",
  "messageId": "msg-uuid-v4",
  "messageType": "vital_signs_update",
  "timestamp": "2025-01-15T19:00:00.123Z",
  "revivalId": "REV-2025-001",
  "facilityId": "FAC-KR-REVIVAL-001",
  "priority": "normal",
  "requiresAck": true,
  "payload": {
    // 메시지별 데이터
  },
  "meta": {
    "deviceId": "MONITOR-001",
    "sequenceNumber": 12345
  }
}
```

### 4.2 메시지 유형

#### 생체 징후 업데이트

```json
{
  "messageType": "vital_signs_update",
  "priority": "normal",
  "requiresAck": false,
  "payload": {
    "heart_rate": 72,
    "blood_pressure": {
      "systolic": 120,
      "diastolic": 80
    },
    "respiratory_rate": 16,
    "body_temperature": 37.0,
    "oxygen_saturation": 98,
    "timestamp": "2025-01-15T19:00:00.123Z"
  }
}
```

#### 중요 경고

```json
{
  "messageType": "critical_alert",
  "priority": "critical",
  "requiresAck": true,
  "payload": {
    "alertType": "cardiac_arrest",
    "severity": "critical",
    "message": "심장 활동이 중단되었습니다",
    "vitalSigns": {
      "heart_rate": 0,
      "blood_pressure": { "systolic": 0, "diastolic": 0 }
    },
    "recommendedAction": "응급 심장 지원 시작",
    "autoEscalated": true
  }
}
```

#### 상태 변경

```json
{
  "messageType": "status_change",
  "priority": "high",
  "requiresAck": true,
  "payload": {
    "previousStatus": "warming",
    "newStatus": "perfusion_reversal",
    "changedBy": "DR-001",
    "reason": "가온 프로토콜이 성공적으로 완료되었습니다",
    "timestamp": "2025-01-15T14:00:00Z"
  }
}
```

#### 팀 조정

```json
{
  "messageType": "team_message",
  "priority": "normal",
  "requiresAck": false,
  "payload": {
    "sender": "DR-001",
    "recipients": ["DR-002", "RN-001", "RN-002"],
    "messageContent": "관류 역전을 시작할 준비를 하고 있습니다. 팀 대기하세요.",
    "messageType": "announcement"
  }
}
```

#### 확인

```json
{
  "messageType": "acknowledgment",
  "priority": "normal",
  "requiresAck": false,
  "payload": {
    "originalMessageId": "msg-550e8400-e29b-41d4",
    "acknowledged": true,
    "acknowledgedBy": "system",
    "acknowledgedAt": "2025-01-15T19:00:00.150Z",
    "processingStatus": "received_and_stored"
  }
}
```

---

## 연결 생명주기

### 5.1 WebSocket 연결 플로우

```
┌────────────┐                                    ┌────────────┐
│  클라이언트 │                                    │   서버     │
└─────┬──────┘                                    └─────┬──────┘
      │                                                 │
      │  1. WebSocket 핸드셰이크 + JWT                   │
      │  ──────────────────────────────────────────►   │
      │                                                 │
      │  2. 101 프로토콜 전환                            │
      │  ◄──────────────────────────────────────────   │
      │                                                 │
      │  3. 인증 성공                                   │
      │  ◄──────────────────────────────────────────   │
      │                                                 │
      │  4. 소생 스트림 구독                             │
      │  ──────────────────────────────────────────►   │
      │                                                 │
      │  5. 구독 확인                                   │
      │  ◄──────────────────────────────────────────   │
      │                                                 │
      │  6. 생체 징후 스트리밍 (연속)                     │
      │  ◄──────────────────────────────────────────   │
      │  ◄──────────────────────────────────────────   │
      │                                                 │
      │  7. 하트비트 (30초마다)                          │
      │  ──────────────────────────────────────────►   │
      │  ◄──────────────────────────────────────────   │
      │                                                 │
      │  8. 중요 경고                                   │
      │  ◄──────────────────────────────────────────   │
      │                                                 │
      │  9. 경고 확인                                   │
      │  ──────────────────────────────────────────►   │
      │                                                 │
```

### 5.2 연결 설정

**클라이언트 요청:**
```javascript
const ws = new WebSocket('wss://ws.wia.live/cryo-revival/v1');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'eyJhbGciOiJSUzI1NiIs...',
    facilityId: 'FAC-KR-REVIVAL-001'
  }));
};
```

**서버 응답:**
```json
{
  "type": "auth_success",
  "sessionId": "session-550e8400",
  "userId": "DR-001",
  "permissions": ["revival:read", "revival:write", "monitoring:stream"],
  "heartbeatInterval": 30000
}
```

### 5.3 구독 관리

**소생 스트림 구독:**
```json
{
  "type": "subscribe",
  "revivalId": "REV-2025-001",
  "streams": ["vital_signs", "alerts", "status_changes"],
  "updateInterval": 1000
}
```

**구독 확인:**
```json
{
  "type": "subscribed",
  "revivalId": "REV-2025-001",
  "subscriptionId": "sub-550e8400",
  "activeStreams": ["vital_signs", "alerts", "status_changes"]
}
```

### 5.4 하트비트 및 킵얼라이브

**클라이언트 하트비트 (30초마다):**
```json
{
  "type": "ping",
  "timestamp": "2025-01-15T19:00:00Z"
}
```

**서버 응답:**
```json
{
  "type": "pong",
  "timestamp": "2025-01-15T19:00:00.005Z",
  "serverTime": "2025-01-15T19:00:00.005Z"
}
```

### 5.5 재연결 전략

```javascript
class RevivalStreamClient {
  constructor(revivalId, token) {
    this.revivalId = revivalId;
    this.token = token;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 10;
    this.reconnectDelay = 1000; // 1초로 시작
  }

  connect() {
    this.ws = new WebSocket('wss://ws.wia.live/cryo-revival/v1');

    this.ws.onopen = () => {
      this.reconnectAttempts = 0;
      this.reconnectDelay = 1000;
      this.authenticate();
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket 오류:', error);
    };

    this.ws.onclose = () => {
      this.handleReconnect();
    };
  }

  handleReconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = Math.min(
        this.reconnectDelay * Math.pow(2, this.reconnectAttempts),
        30000 // 최대 30초
      );

      console.log(`${delay}ms 후 재연결 중 (시도 ${this.reconnectAttempts})`);
      setTimeout(() => this.connect(), delay);
    } else {
      console.error('최대 재연결 시도 횟수에 도달했습니다');
      this.notifyConnectionFailure();
    }
  }
}
```

---

## 메시지 유형

### 6.1 생체 징후 스트림

**메시지 플로우:**
```
기기 → MQTT → 브로커 → WebSocket → 대시보드
```

**MQTT 발행:**
```json
{
  "protocol": "wia-revival",
  "version": "1.0.0",
  "messageId": "msg-001",
  "messageType": "vital_signs_update",
  "timestamp": "2025-01-15T19:00:00.123Z",
  "revivalId": "REV-2025-001",
  "payload": {
    "heart_rate": 72,
    "ecg_rhythm": "normal_sinus",
    "blood_pressure": { "systolic": 120, "diastolic": 80 },
    "respiratory_rate": 16,
    "body_temperature": 37.0,
    "oxygen_saturation": 98,
    "capnography": 38
  },
  "meta": {
    "deviceId": "CARDIAC-MONITOR-001",
    "deviceType": "multi_parameter_monitor",
    "calibrationDate": "2025-01-10"
  }
}
```

### 6.2 경고 우선순위 수준

| 우선순위 | 응답 시간 | 사용 사례 | 예시 |
|---------|----------|----------|------|
| `critical` | 즉시 | 생명을 위협하는 상황 | 심정지, 심한 출혈 |
| `high` | < 30초 | 긴급 의료 조치 필요 | 비정상적인 생체 징후, 의식 변화 |
| `medium` | < 2분 | 중요하지만 긴급하지 않음 | 프로토콜 이탈 |
| `low` | < 10분 | 정보 제공 | 일상적인 상태 업데이트 |
| `info` | 요구사항 없음 | 일반 정보 | 절차 이정표 |

### 6.3 의료 경고 유형

```typescript
enum AlertType {
  // 심장
  CARDIAC_ARREST = 'cardiac_arrest',              // 심정지
  VENTRICULAR_FIBRILLATION = 'ventricular_fibrillation', // 심실세동
  TACHYCARDIA = 'tachycardia',                   // 빈맥
  BRADYCARDIA = 'bradycardia',                   // 서맥

  // 호흡
  APNEA = 'apnea',                              // 무호흡
  HYPOXIA = 'hypoxia',                          // 저산소증
  RESPIRATORY_FAILURE = 'respiratory_failure',   // 호흡 부전

  // 신경학적
  SEIZURE = 'seizure',                          // 발작
  DECREASED_CONSCIOUSNESS = 'decreased_consciousness', // 의식 감소
  INTRACRANIAL_PRESSURE = 'intracranial_pressure',    // 두개내압

  // 혈역학적
  HYPOTENSION = 'hypotension',                  // 저혈압
  HYPERTENSION = 'hypertension',                // 고혈압
  HEMORRHAGE = 'hemorrhage',                    // 출혈

  // 체온
  HYPERTHERMIA = 'hyperthermia',                // 고체온증
  HYPOTHERMIA = 'hypothermia',                  // 저체온증

  // 장비
  DEVICE_MALFUNCTION = 'device_malfunction',    // 기기 오작동
  POWER_FAILURE = 'power_failure',              // 전원 장애

  // 프로토콜
  PROTOCOL_DEVIATION = 'protocol_deviation',    // 프로토콜 이탈
  TIMEFRAME_EXCEEDED = 'timeframe_exceeded'     // 시간 초과
}
```

---

## 보안

### 7.1 전송 계층 보안

**TLS 구성:**
```
프로토콜: TLS 1.3
암호화 스위트:
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256
  - TLS_AES_128_GCM_SHA256

인증서 피닝: 활성화
HSTS: max-age=31536000; includeSubDomains
```

### 7.2 메시지 수준 암호화

**민감 데이터 암호화:**
```json
{
  "messageType": "vital_signs_update",
  "payload": {
    "heart_rate": 72,
    "patient_identifiers": {
      "encrypted": true,
      "algorithm": "AES-256-GCM",
      "ciphertext": "8f7a9b2c3d4e5f6a7b8c9d0e1f2a3b4c...",
      "iv": "a1b2c3d4e5f6a7b8",
      "tag": "f1e2d3c4b5a69788"
    }
  }
}
```

### 7.3 감사 로깅

모든 메시지는 다음과 함께 기록됩니다:

```json
{
  "auditId": "audit-550e8400",
  "timestamp": "2025-01-15T19:00:00.123Z",
  "messageId": "msg-001",
  "messageType": "vital_signs_update",
  "sender": {
    "type": "device",
    "id": "CARDIAC-MONITOR-001"
  },
  "recipients": ["DR-001", "RN-001"],
  "revivalId": "REV-2025-001",
  "facilityId": "FAC-KR-REVIVAL-001",
  "dataClassification": "PHI",
  "accessJustification": "active_medical_care",
  "ipAddress": "192.168.1.100",
  "userAgent": "WiaRevivalClient/1.0.0"
}
```

---

## 오류 처리

### 8.1 오류 메시지 형식

```json
{
  "type": "error",
  "errorCode": "ERR_INVALID_MESSAGE_FORMAT",
  "errorMessage": "잘못된 생체 징후 데이터 형식",
  "severity": "warning",
  "timestamp": "2025-01-15T19:00:00Z",
  "originalMessageId": "msg-001",
  "details": {
    "field": "payload.heart_rate",
    "expectedType": "integer",
    "receivedType": "string",
    "receivedValue": "seventy-two"
  },
  "suggestedAction": "데이터 타입을 수정하고 다시 전송하세요"
}
```

### 8.2 오류 코드

| 코드 | 설명 | 조치 |
|------|------|------|
| `ERR_AUTH_FAILED` | 인증 실패 | 재인증 |
| `ERR_INVALID_MESSAGE_FORMAT` | 잘못된 메시지 형식 | 형식을 수정하고 재전송 |
| `ERR_REVIVAL_NOT_FOUND` | 소생 ID를 찾을 수 없음 | 소생 ID 확인 |
| `ERR_PERMISSION_DENIED` | 권한 부족 | 높은 액세스 권한 요청 |
| `ERR_RATE_LIMIT_EXCEEDED` | 메시지 과다 | 빈도 감소 |
| `ERR_DEVICE_OFFLINE` | 의료 기기 오프라인 | 기기 연결 확인 |
| `ERR_CRITICAL_THRESHOLD` | 생체 징후 중요 | 의료 개입 필요 |

---

## 예제

### 9.1 완전한 실시간 모니터링 세션

```javascript
// 클라이언트 초기화
const client = new WiaRevivalStreamClient({
  revivalId: 'REV-2025-001',
  token: 'eyJhbGciOiJSUzI1NiIs...',
  facilityId: 'FAC-KR-REVIVAL-001'
});

// 연결 및 구독
await client.connect();
await client.subscribe(['vital_signs', 'alerts', 'status_changes']);

// 생체 징후 업데이트 처리
client.on('vital_signs_update', (data) => {
  console.log('심박수:', data.heart_rate);
  console.log('체온:', data.body_temperature);

  updateDashboard({
    heartRate: data.heart_rate,
    bloodPressure: data.blood_pressure,
    temperature: data.body_temperature,
    oxygenSaturation: data.oxygen_saturation
  });

  // 임계값 확인
  if (data.heart_rate > 150) {
    triggerAlert('빈맥이 감지되었습니다');
  }
});

// 중요 경고 처리
client.on('critical_alert', (alert) => {
  console.error('중요 경고:', alert.message);

  // 응급 알림 표시
  showEmergencyModal({
    title: alert.alertType,
    message: alert.message,
    severity: alert.severity,
    recommendedAction: alert.recommendedAction
  });

  // 의료진 알림
  notifyMedicalTeam({
    revivalId: alert.revivalId,
    alertType: alert.alertType,
    urgency: 'immediate'
  });

  // 수신 확인
  client.acknowledgeAlert(alert.messageId);
});

// 상태 변경 처리
client.on('status_change', (change) => {
  console.log(`상태 변경: ${change.previousStatus} → ${change.newStatus}`);
  updateProcedureStatus(change.newStatus);
});

// 연결 해제 처리
client.on('disconnect', () => {
  console.warn('연결이 끊어졌습니다. 재연결 중...');
  showConnectionWarning();
});

client.on('reconnect', () => {
  console.log('재연결 성공');
  hideConnectionWarning();
});
```

### 9.2 MQTT 의료 기기 통합

```python
import paho.mqtt.client as mqtt
import json
import time

class MedicalDeviceMQTT:
    def __init__(self, device_id, revival_id, facility_id):
        self.device_id = device_id
        self.revival_id = revival_id
        self.facility_id = facility_id
        self.client = mqtt.Client()

        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish

        # TLS 구성
        self.client.tls_set(
            ca_certs="/path/to/ca.crt",
            certfile="/path/to/client.crt",
            keyfile="/path/to/client.key"
        )

        self.client.username_pw_set("device_id", "device_secret")
        self.client.connect("mqtt.wia.live", 8883, 60)

    def on_connect(self, client, userdata, flags, rc):
        print(f"연결됨, 결과 코드 {rc}")

    def publish_vitals(self, vitals_data):
        topic = f"wia/revival/{self.facility_id}/{self.revival_id}/vitals"

        message = {
            "protocol": "wia-revival",
            "version": "1.0.0",
            "messageId": f"msg-{int(time.time()*1000)}",
            "messageType": "vital_signs_update",
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime()),
            "revivalId": self.revival_id,
            "payload": vitals_data,
            "meta": {
                "deviceId": self.device_id
            }
        }

        # 생체 징후에 대한 QoS 1
        result = self.client.publish(
            topic,
            json.dumps(message),
            qos=1
        )

        return result

    def publish_critical_alert(self, alert_data):
        topic = f"wia/revival/{self.facility_id}/{self.revival_id}/alerts"

        message = {
            "protocol": "wia-revival",
            "version": "1.0.0",
            "messageId": f"msg-alert-{int(time.time()*1000)}",
            "messageType": "critical_alert",
            "priority": "critical",
            "requiresAck": True,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime()),
            "revivalId": self.revival_id,
            "payload": alert_data,
            "meta": {
                "deviceId": self.device_id
            }
        }

        # 중요 경고에 대한 QoS 2
        result = self.client.publish(
            topic,
            json.dumps(message),
            qos=2
        )

        return result

# 사용
device = MedicalDeviceMQTT(
    device_id="CARDIAC-MONITOR-001",
    revival_id="REV-2025-001",
    facility_id="FAC-KR-REVIVAL-001"
)

# 매초마다 생체 징후 발행
while True:
    vitals = {
        "heart_rate": get_heart_rate(),
        "blood_pressure": get_blood_pressure(),
        "body_temperature": get_temperature(),
        "oxygen_saturation": get_spo2()
    }

    device.publish_vitals(vitals)

    # 중요 조건 확인
    if vitals["heart_rate"] == 0:
        device.publish_critical_alert({
            "alertType": "cardiac_arrest",
            "severity": "critical",
            "message": "심장 활동이 중단되었습니다",
            "vitalSigns": vitals
        })

    time.sleep(1)
```

---

<div align="center">

**WIA 냉동인간 소생 통신 프로토콜 v1.0.0**

**弘益人間 (홍익인간)** - 모든 인류에게 이익을

---

**© 2025 WIA 표준 위원회**

**MIT 라이선스**

</div>
