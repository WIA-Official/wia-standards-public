# WIA Pet Emotion Communication Protocol
## Phase 3 사양 (Specification)

**버전 (Version)**: 1.0.0
**상태 (Status)**: Draft
**날짜 (Date)**: 2025-12-18
**주요 색상 (Primary Color)**: #F59E0B (Amber)

---

## 목차 (Table of Contents)

1. [개요](#개요-overview)
2. [용어 정의](#용어-정의-terminology)
3. [프로토콜 아키텍처](#프로토콜-아키텍처-protocol-architecture)
4. [메시지 형식](#메시지-형식-message-format)
5. [메시지 유형](#메시지-유형-message-types)
6. [연결 관리](#연결-관리-connection-management)
7. [스트리밍 프로토콜](#스트리밍-프로토콜-streaming-protocol)
8. [보안](#보안-security)
9. [전송 계층](#전송-계층-transport-layers)
10. [오류 처리](#오류-처리-error-handling)

---

## 1. 개요 (Overview)

### 1.1 목적 (Purpose)

본 문서는 WIA Pet Emotion Standard의 Phase 3으로, 감정 감지 시스템, 센서, 애플리케이션 및 클라우드 서비스 간의 표준화된 통신을 가능하게 하는 통신 프로토콜을 정의합니다.

**핵심 목표 (Core Objectives)**:
- 실시간 감정 데이터 스트리밍
- 다중 장치 동기화
- 클라우드-엣지 조정
- 저지연 통신
- 안전한 데이터 전송

### 1.2 적용 범위 (Scope)

| 컴포넌트 (Component) | 설명 (Description) |
|---------------------|-------------------|
| **Message Format** | JSON 기반 프로토콜 메시지 |
| **Transport** | WebSocket, MQTT, HTTP, BLE |
| **Security** | TLS, 인증, 암호화 |
| **Streaming** | 실시간 감정 이벤트 스트리밍 |
| **Sync** | 다중 장치 상태 동기화 |

### 1.3 관련 문서 (Related Documents)

| 문서 (Document) | 설명 (Description) |
|----------------|-------------------|
| PHASE-1-DATA-FORMAT.md | 감정 데이터 구조 |
| PHASE-2-API-INTERFACE.md | 프로그래밍 인터페이스 |
| PHASE-4-INTEGRATION.md | 생태계 통합 |

---

## 2. 용어 정의 (Terminology)

| 용어 (Term) | 정의 (Definition) |
|-----------|------------------|
| **Client** | 감정 감지를 요청하는 장치/앱 |
| **Server** | 감정 감지를 제공하는 서비스 |
| **Edge Device** | 로컬 처리 장치 (카메라, 웨어러블) |
| **Cloud Service** | 원격 처리/저장 서비스 |
| **Message** | 프로토콜 데이터 단위 |
| **Stream** | 연속적인 메시지 흐름 |
| **Session** | 연결된 클라이언트-서버 상호작용 |

---

## 3. 프로토콜 아키텍처 (Protocol Architecture)

### 3.1 시스템 아키텍처 (System Architecture)

```
┌─────────────────────────────────────────────────────────────┐
│                     클라우드 계층 (Cloud Layer)              │
│            (저장소, 분석, ML 학습)                            │
└────────────────────┬────────────────────────────────────────┘
                     │ HTTPS/MQTT
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                 게이트웨이/허브 계층 (Gateway/Hub Layer)      │
│           (집계, 라우팅, 전처리)                              │
└────────────────────┬────────────────────────────────────────┘
                     │ WebSocket/MQTT
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                   엣지 계층 (Edge Layer)                     │
│        (카메라, 웨어러블, 로컬 처리)                          │
└────────────────────┬────────────────────────────────────────┘
                     │ BLE/USB/Local
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                  센서 계층 (Sensor Layer)                    │
│              (가속도계, 심박수, 마이크)                        │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 프로토콜 스택 (Protocol Stack)

```
┌─────────────────────────────────────────┐
│      애플리케이션 계층                     │
│    (감정 감지 로직)                       │
├─────────────────────────────────────────┤
│      프로토콜 계층                        │
│  (WIA Pet Emotion Messages)             │
├─────────────────────────────────────────┤
│       전송 계층                          │
│  (WebSocket / MQTT / HTTP / BLE)        │
├─────────────────────────────────────────┤
│       보안 계층                          │
│      (TLS / 암호화)                     │
├─────────────────────────────────────────┤
│      네트워크 계층                        │
│         (TCP/IP / BLE)                  │
└─────────────────────────────────────────┘
```

---

## 4. 메시지 형식 (Message Format)

### 4.1 기본 메시지 구조 (Base Message Structure)

모든 WIA Pet Emotion 프로토콜 메시지는 다음 구조를 따릅니다:

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "uuid-v4-string",
  "timestamp": 1702915200000,
  "type": "message_type",
  "payload": {},
  "metadata": {}
}
```

### 4.2 필드 정의 (Field Definitions)

| 필드 (Field) | 타입 (Type) | 필수 (Required) | 설명 (Description) |
|------------|-----------|---------------|------------------|
| `protocol` | string | Yes | 프로토콜 식별자 ("wia-pet-emotion") |
| `version` | string | Yes | 프로토콜 버전 (SemVer) |
| `messageId` | string | Yes | 고유 메시지 ID (UUID v4) |
| `timestamp` | number | Yes | Unix timestamp (milliseconds) |
| `type` | string | Yes | 메시지 유형 식별자 |
| `payload` | object | Yes | 메시지별 데이터 |
| `metadata` | object | No | 선택적 메타데이터 |

### 4.3 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wiastandards.com/schemas/pet-emotion-protocol/v1/message.schema.json",
  "title": "WIA Pet Emotion Protocol Message",
  "type": "object",
  "required": ["protocol", "version", "messageId", "timestamp", "type", "payload"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-pet-emotion"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "messageId": {
      "type": "string",
      "format": "uuid"
    },
    "timestamp": {
      "type": "integer",
      "minimum": 0
    },
    "type": {
      "type": "string",
      "enum": [
        "connect", "connect_ack", "disconnect", "disconnect_ack",
        "emotion_data", "emotion_request", "emotion_response",
        "sensor_data", "sensor_command", "sensor_status",
        "stream_start", "stream_stop", "stream_data",
        "sync_request", "sync_response",
        "alert", "event",
        "error", "ping", "pong"
      ]
    },
    "payload": {
      "type": "object"
    }
  }
}
```

---

## 5. 메시지 유형 (Message Types)

### 5.1 메시지 유형 개요 (Message Type Overview)

| 범주 (Category) | 메시지 유형 (Message Types) | 방향 (Direction) |
|----------------|---------------------------|-----------------|
| 연결 (Connection) | `connect`, `connect_ack`, `disconnect`, `disconnect_ack` | 양방향 |
| 감정 데이터 (Emotion Data) | `emotion_data`, `emotion_request`, `emotion_response` | 양방향 |
| 센서 (Sensor) | `sensor_data`, `sensor_command`, `sensor_status` | 양방향 |
| 스트리밍 (Streaming) | `stream_start`, `stream_stop`, `stream_data` | 양방향 |
| 동기화 (Sync) | `sync_request`, `sync_response` | 양방향 |
| 알림 (Alerts) | `alert`, `event` | Server → Client |
| 제어 (Control) | `ping`, `pong`, `error` | 양방향 |

### 5.2 연결 메시지 (Connection Messages)

#### 5.2.1 connect

감정 감지 서비스에 연결을 설정합니다.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702915200000,
  "type": "connect",
  "payload": {
    "clientId": "mobile-app-12345",
    "clientType": "mobile_app",
    "clientVersion": "2.1.0",
    "capabilities": {
      "sensors": ["camera", "microphone"],
      "models": ["emotion-detect-v2"],
      "streaming": true,
      "offline_mode": true
    },
    "pets": [
      {
        "petId": "PET-001",
        "species": "dog",
        "name": "맥스"
      }
    ],
    "preferences": {
      "detection_frequency": 30,
      "data_compression": true,
      "alert_enabled": true
    }
  }
}
```

**Payload 필드 (Payload Fields)**:

| 필드 (Field) | 타입 (Type) | 필수 | 설명 (Description) |
|------------|-----------|-----|------------------|
| `clientId` | string | Yes | 고유 클라이언트 식별자 |
| `clientType` | string | Yes | 클라이언트 유형 (mobile_app, web_app, edge_device) |
| `clientVersion` | string | No | 클라이언트 소프트웨어 버전 |
| `capabilities` | object | Yes | 클라이언트 기능 |
| `pets` | array | Yes | 모니터링할 반려동물 목록 |
| `preferences` | object | No | 연결 설정 |

#### 5.2.2 connect_ack

서버의 연결 승인입니다.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1702915200050,
  "type": "connect_ack",
  "payload": {
    "success": true,
    "sessionId": "SESSION-abc123",
    "serverId": "emotion-server-01",
    "serverVersion": "1.5.2",
    "capabilities": {
      "max_pets": 10,
      "streaming": true,
      "models_available": ["emotion-detect-v2", "emotion-detect-v3"],
      "cloud_storage": true
    },
    "config": {
      "heartbeat_interval": 30000,
      "max_message_size": 1048576,
      "compression_enabled": true
    }
  }
}
```

### 5.3 감정 데이터 메시지 (Emotion Data Messages)

#### 5.3.1 emotion_request

감정 감지를 요청합니다.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440020",
  "timestamp": 1702915210000,
  "type": "emotion_request",
  "payload": {
    "petId": "PET-001",
    "requestId": "REQ-001",
    "inputs": {
      "image": {
        "encoding": "base64",
        "format": "jpeg",
        "data": "base64-encoded-image-data",
        "width": 1920,
        "height": 1080
      },
      "audio": {
        "encoding": "base64",
        "format": "wav",
        "data": "base64-encoded-audio-data",
        "sample_rate": 44100,
        "channels": 1,
        "duration": 1000
      },
      "sensors": {
        "heart_rate": 95,
        "activity_level": 0.75,
        "temperature": 38.5
      }
    },
    "options": {
      "model": "emotion-detect-v2",
      "return_raw_predictions": true,
      "min_confidence": 0.7
    }
  }
}
```

#### 5.3.2 emotion_response

감지된 감정과 함께 응답합니다.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440021",
  "timestamp": 1702915210145,
  "type": "emotion_response",
  "payload": {
    "petId": "PET-001",
    "requestId": "REQ-001",
    "success": true,
    "result": {
      "emotion": "happy",
      "intensity": {
        "intensity": 0.85,
        "level": "high",
        "confidence": 0.92
      },
      "dimensions": {
        "valence": 0.8,
        "arousal": 0.7,
        "dominance": 0.6
      },
      "confidence": 0.92,
      "predictions": [
        { "emotion": "happy", "probability": 0.89, "confidence": 0.92 },
        { "emotion": "excited", "probability": 0.08, "confidence": 0.85 }
      ]
    }
  }
}
```

### 5.4 스트리밍 메시지 (Streaming Messages)

#### 5.4.1 stream_start

연속적인 감정 스트리밍을 시작합니다.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440050",
  "timestamp": 1702915260000,
  "type": "stream_start",
  "payload": {
    "petId": "PET-001",
    "streamId": "STREAM-001",
    "stream_type": "emotion_continuous",
    "options": {
      "frequency": 30,
      "sources": ["camera", "wearable"],
      "min_confidence": 0.7,
      "compression": true
    }
  }
}
```

#### 5.4.2 stream_data

스트리밍 감정 데이터 패킷입니다.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440051",
  "timestamp": 1702915260033,
  "type": "stream_data",
  "payload": {
    "streamId": "STREAM-001",
    "sequence": 1,
    "petId": "PET-001",
    "data": {
      "emotion": "happy",
      "intensity": 0.85,
      "confidence": 0.92,
      "valence": 0.8,
      "arousal": 0.7
    }
  }
}
```

### 5.5 알림 및 이벤트 메시지 (Alert and Event Messages)

#### 5.5.1 alert

높은 우선순위 알림 알림입니다.

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440070",
  "timestamp": 1702915340000,
  "type": "alert",
  "payload": {
    "alertId": "ALERT-001",
    "petId": "PET-001",
    "alert_type": "high_stress",
    "severity": "high",
    "description": "반려동물이 장시간 높은 스트레스 징후를 보이고 있습니다",
    "details": {
      "stress_level": 0.88,
      "duration": 180000,
      "current_emotion": "anxious",
      "triggers": ["loud_noise", "unfamiliar_person"]
    },
    "recommended_action": "환경을 확인하고 위안을 제공하세요",
    "requires_acknowledgment": true
  },
  "metadata": {
    "priority": 9
  }
}
```

---

## 6. 연결 관리 (Connection Management)

### 6.1 연결 생명주기 (Connection Lifecycle)

```
Client                                  Server
  │                                       │
  ├────── connect ──────────────────────►│
  │                                       │
  │◄────── connect_ack ───────────────────┤
  │                                       │
  │                                       │
  ├────── emotion_request ──────────────►│
  │                                       │
  │◄────── emotion_response ──────────────┤
  │                                       │
  │                                       │
  ├────── ping ──────────────────────────►│
  │                                       │
  │◄────── pong ───────────────────────────┤
  │                                       │
  │                                       │
  ├────── disconnect ────────────────────►│
  │                                       │
  │◄────── disconnect_ack ─────────────────┤
  │                                       │
```

### 6.2 하트비트 메커니즘 (Heartbeat Mechanism)

```typescript
interface HeartbeatConfig {
  enabled: boolean;
  interval: number;          // milliseconds
  timeout: number;           // milliseconds
  max_missed: number;        // 연결 해제 전 연속 누락 횟수
}

// 기본 설정 (Default configuration)
const DEFAULT_HEARTBEAT: HeartbeatConfig = {
  enabled: true,
  interval: 30000,          // 30초
  timeout: 5000,            // 5초
  max_missed: 3             // 3회 연속 누락
};
```

### 6.3 재연결 전략 (Reconnection Strategy)

```typescript
interface ReconnectionConfig {
  enabled: boolean;
  initial_delay: number;     // milliseconds
  max_delay: number;         // milliseconds
  backoff_multiplier: number;
  max_attempts: number;
  jitter: boolean;           // 지터 사용
}

// 지터를 포함한 지수 백오프 (Exponential backoff with jitter)
function calculateReconnectDelay(attempt: number, config: ReconnectionConfig): number {
  let delay = Math.min(
    config.initial_delay * Math.pow(config.backoff_multiplier, attempt),
    config.max_delay
  );

  if (config.jitter) {
    delay = delay * (0.5 + Math.random() * 0.5);
  }

  return delay;
}
```

---

## 7. 스트리밍 프로토콜 (Streaming Protocol)

### 7.1 스트림 유형 (Stream Types)

| 스트림 유형 (Stream Type) | 설명 (Description) | 빈도 (Frequency) | Payload |
|-------------------------|-------------------|-----------------|---------|
| `emotion_continuous` | 연속 감정 상태 | 1-60 Hz | EmotionState |
| `sensor_raw` | 원시 센서 데이터 | 가변 | SensorData |
| `sensor_processed` | 처리된 센서 데이터 | 1-30 Hz | ProcessedSensorData |
| `event_stream` | 이벤트 알림 | 이벤트 기반 | Event |
| `alert_stream` | 알림 알림 | 이벤트 기반 | Alert |

### 7.2 스트림 서비스 품질 (Stream Quality of Service)

```typescript
enum StreamQoS {
  AT_MOST_ONCE = 0,      // 발사 후 망각 (Fire and forget)
  AT_LEAST_ONCE = 1,     // 전달 보장, 중복 가능
  EXACTLY_ONCE = 2       // 전달 보장, 중복 없음
}

interface StreamOptions {
  qos: StreamQoS;
  buffer_size: number;
  compression: boolean;
  encryption: boolean;
  priority: number;       // 0-9
}
```

### 7.3 흐름 제어 (Flow Control)

```typescript
interface FlowControl {
  enabled: boolean;
  window_size: number;    // 최대 미확인 메시지 수
  rate_limit: number;     // 초당 메시지 수
  burst_size: number;     // 최대 버스트 크기
}

// 흐름 제어 메시지 (Flow control message)
interface FlowControlMessage {
  type: 'flow_control';
  action: 'pause' | 'resume' | 'adjust';
  parameters?: {
    new_rate?: number;
    window_size?: number;
  };
}
```

---

## 8. 보안 (Security)

### 8.1 인증 (Authentication)

#### 8.1.1 API Key 인증

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702915400000,
  "type": "connect",
  "payload": {
    "clientId": "mobile-app-12345",
    "auth": {
      "method": "api_key",
      "api_key": "wpe_1234567890abcdef",
      "api_secret": "secret_abcdefghijklmnop"
    }
  }
}
```

#### 8.1.2 OAuth 2.0 인증

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702915400000,
  "type": "connect",
  "payload": {
    "clientId": "mobile-app-12345",
    "auth": {
      "method": "oauth2",
      "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
      "token_type": "Bearer",
      "expires_in": 3600
    }
  }
}
```

### 8.2 암호화 (Encryption)

#### 8.2.1 전송 계층 보안 (Transport Layer Security)

```
모든 연결은 TLS 1.2 이상을 사용해야 합니다:
- WebSocket: wss://
- HTTPS: https://
- MQTT: mqtts://

권장 암호 스위트 (Recommended cipher suites):
- TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
- TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
```

#### 8.2.2 종단 간 암호화 (End-to-End Encryption)

```typescript
interface E2EEncryption {
  enabled: boolean;
  algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
  key_exchange: 'ECDH' | 'RSA';

  // 암호화된 payload
  encrypted_data: string;    // Base64 인코딩
  iv: string;                // 초기화 벡터
  auth_tag: string;          // 인증 태그
  key_id: string;            // 키 식별자
}
```

---

## 9. 전송 계층 (Transport Layers)

### 9.1 WebSocket 전송

```typescript
// WebSocket 엔드포인트
const WS_ENDPOINT = 'wss://api.wia-pet-emotion.com/v1/ws';

// 연결 (Connection)
const ws = new WebSocket(WS_ENDPOINT);

ws.on('open', () => {
  // connect 메시지 전송
  ws.send(JSON.stringify(connectMessage));
});

ws.on('message', (data) => {
  const message = JSON.parse(data);
  handleMessage(message);
});

ws.on('close', (code, reason) => {
  console.log('연결 종료:', code, reason);
  attemptReconnect();
});

// 하트비트 (Heartbeat)
setInterval(() => {
  ws.send(JSON.stringify(pingMessage));
}, 30000);
```

### 9.2 MQTT 전송

```typescript
// MQTT 브로커
const MQTT_BROKER = 'mqtts://mqtt.wia-pet-emotion.com:8883';

// 토픽 (Topics)
const TOPICS = {
  // 발행 (Publishing)
  emotion_request: `pet-emotion/pet/{petId}/request`,
  sensor_data: `pet-emotion/pet/{petId}/sensor`,

  // 구독 (Subscribing)
  emotion_response: `pet-emotion/pet/{petId}/response`,
  emotion_stream: `pet-emotion/pet/{petId}/stream`,
  alerts: `pet-emotion/pet/{petId}/alerts`,
  events: `pet-emotion/pet/{petId}/events`
};

// 연결 (Connect)
const client = mqtt.connect(MQTT_BROKER, {
  clientId: 'mobile-app-12345',
  username: 'user',
  password: 'pass',
  keepalive: 60,
  clean: true,
  reconnectPeriod: 1000
});

// 구독 (Subscribe)
client.subscribe(TOPICS.emotion_response);
client.subscribe(TOPICS.alerts);

// 발행 (Publish)
client.publish(
  TOPICS.emotion_request,
  JSON.stringify(emotionRequestMessage),
  { qos: 1, retain: false }
);
```

---

## 10. 오류 처리 (Error Handling)

### 10.1 오류 코드 (Error Codes)

| 코드 (Code) | 범주 (Category) | 설명 (Description) |
|-----------|----------------|------------------|
| 1000 | Protocol | 잘못된 프로토콜 버전 |
| 1001 | Protocol | 잘못된 형식의 메시지 |
| 2000 | Authentication | 인증 실패 |
| 2001 | Authentication | 잘못된 자격 증명 |
| 2002 | Authentication | 토큰 만료 |
| 3000 | Connection | 연결 시간 초과 |
| 3001 | Connection | 연결 거부 |
| 4000 | Detection | 감지 실패 |
| 4001 | Detection | 불충분한 데이터 품질 |
| 5000 | Sensor | 센서 오류 |
| 5001 | Sensor | 센서를 찾을 수 없음 |
| 9000 | Internal | 내부 서버 오류 |

### 10.2 오류 메시지 형식 (Error Message Format)

```json
{
  "protocol": "wia-pet-emotion",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1702915500000,
  "type": "error",
  "payload": {
    "error_code": 4001,
    "error_category": "Detection",
    "error_message": "감정 감지를 위한 데이터 품질이 불충분합니다",
    "details": {
      "image_quality_score": 0.35,
      "min_required": 0.60,
      "issues": ["low_resolution", "poor_lighting"]
    },
    "recoverable": true,
    "suggested_action": "조명을 개선하고 반려동물이 명확하게 보이도록 하세요",
    "help_url": "https://docs.wia-pet-emotion.com/errors/4001"
  }
}
```

---

## 11. 성능 사양 (Performance Specifications)

| 지표 (Metric) | 요구사항 (Requirement) | 목표 (Target) |
|-------------|----------------------|--------------|
| 메시지 지연시간 (p50) | < 100ms | < 50ms |
| 메시지 지연시간 (p99) | < 500ms | < 200ms |
| 처리량 (Throughput) | > 1000 msg/s | > 5000 msg/s |
| 연결 수립 | < 2s | < 1s |
| 재연결 시간 | < 5s | < 2s |
| 스트림 지연시간 | < 100ms | < 33ms (30Hz) |
| 하트비트 오버헤드 | < 1% 대역폭 | < 0.5% 대역폭 |

---

## 12. 준수 및 표준 (Compliance and Standards)

| 표준 (Standard) | 준수 (Compliance) |
|---------------|-----------------|
| RFC 6455 | WebSocket Protocol |
| RFC 7692 | WebSocket Compression |
| MQTT 5.0 | MQTT Protocol |
| TLS 1.3 | Transport Security |
| OAuth 2.0 | Authentication |
| JSON Schema | Message Validation |

---

**문서 ID (Document ID)**: WIA-PET-EMOTION-PHASE3-001
**버전 (Version)**: 1.0.0
**최종 업데이트 (Last Updated)**: 2025-12-18
**저작권 (Copyright)**: © 2025 WIA - MIT License

---
**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
