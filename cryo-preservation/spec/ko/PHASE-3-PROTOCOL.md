# WIA 냉동보존 통신 프로토콜
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
8. [에러 처리](#에러-처리)
9. [예제](#예제)

---

## 개요

### 1.1 목적

WIA 냉동보존 통신 프로토콜은 저장 조건 모니터링, 알림 수신, 시설 운영 조정을 위한 실시간 통신 표준을 정의합니다.

**핵심 목표**:
- 실시간 저장 조건 모니터링
- 즉각적인 알림 전달
- 안전한 시설 간 통신
- 확인 응답을 통한 신뢰성 있는 메시지 전달

### 1.2 프로토콜 선택

| 사용 사례 | 권장 프로토콜 |
|----------|--------------|
| 실시간 모니터링 | WebSocket |
| IoT 센서 데이터 | MQTT |
| 고처리량 데이터 | gRPC |
| 이벤트 알림 | Server-Sent Events (SSE) |

### 1.3 설계 원칙

1. **신뢰성**: 최소 1회 전달 보장
2. **저지연**: 1초 미만 메시지 전달
3. **보안**: 종단간 암호화
4. **복원력**: 자동 재연결
5. **확장성**: 수평 확장 지원

---

## 프로토콜 아키텍처

### 2.1 계층 모델

```
┌──────────────────────────────────────────────────────────────┐
│                    애플리케이션 계층                           │
│              (WIA 냉동보존 서비스)                             │
├──────────────────────────────────────────────────────────────┤
│                    프로토콜 계층                              │
│              (메시지 형식, 핸들러)                             │
├──────────────────────────────────────────────────────────────┤
│                    보안 계층                                  │
│                (TLS 1.3, JWT 인증)                           │
├──────────────────────────────────────────────────────────────┤
│                    전송 계층                                  │
│          (WebSocket / MQTT / gRPC / SSE)                     │
├──────────────────────────────────────────────────────────────┤
│                    네트워크 계층                              │
│                     (TCP/IP)                                 │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 컴포넌트 아키텍처

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   저장 센서     │     │   제어 패널     │     │   모바일 앱     │
│     장치       │     │    대시보드     │     │    (알림)       │
└────────┬────────┘     └────────┬────────┘     └────────┬────────┘
         │                       │                       │
         │ MQTT                  │ WebSocket             │ WebSocket
         │                       │                       │
         ▼                       ▼                       ▼
┌──────────────────────────────────────────────────────────────────┐
│                    메시지 브로커 / 게이트웨이                      │
│               (인증, 라우팅, 버퍼링)                              │
└──────────────────────────────────────────────────────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   저장 데이터   │     │   알림 서비스   │     │   감사 로그     │
│     서비스     │     │                 │     │    서비스       │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

---

## 전송 계층

### 3.1 WebSocket (주요)

**연결 URL:**
```
wss://ws.wia.live/cryo-preservation/v1
```

**기본 포트:** 443 (WSS)

**서브프로토콜:** `wia-cryo-v1`

**연결 예시:**
```javascript
const ws = new WebSocket('wss://ws.wia.live/cryo-preservation/v1', 'wia-cryo-v1');
```

### 3.2 MQTT (IoT 센서)

**브로커 URL:**
```
mqtts://mqtt.wia.live:8883
```

**토픽 구조:**
```
wia/cryo/{facility_id}/{container_id}/{metric_type}

예시:
wia/cryo/FAC-KR-001/DEW-001/temperature
wia/cryo/FAC-KR-001/DEW-001/nitrogen_level
wia/cryo/FAC-KR-001/+/alerts
```

**QoS 레벨:**
| QoS | 사용 사례 |
|-----|----------|
| 0 | 일반 센서 측정값 |
| 1 | 상태 업데이트 |
| 2 | 중요 알림 |

### 3.3 gRPC (서비스 간)

**Proto 정의:**
```protobuf
syntax = "proto3";

package wia.cryo.v1;

service CryoMonitoring {
  rpc StreamConditions(ContainerRequest) returns (stream StorageCondition);
  rpc SendAlert(Alert) returns (AlertAck);
  rpc GetStatus(StatusRequest) returns (StatusResponse);
}

message StorageCondition {
  string container_id = 1;
  int64 timestamp = 2;
  double temperature = 3;
  double nitrogen_level = 4;
  double vacuum_pressure = 5;
}
```

### 3.4 Server-Sent Events (알림)

**엔드포인트:**
```
GET /api/v1/events/stream
Accept: text/event-stream
```

**이벤트 형식:**
```
event: storage.alert
data: {"containerId":"DEW-001","type":"temperature","value":-195.0}

event: heartbeat
data: {"timestamp":1704067200}
```

---

## 메시지 형식

### 4.1 기본 메시지 구조

```json
{
  "protocol": "wia-cryo",
  "version": "1.0.0",
  "messageId": "msg-uuid-v4",
  "timestamp": 1704067200000,
  "type": "message_type",
  "source": {
    "type": "sensor|service|client",
    "id": "source-identifier"
  },
  "payload": {}
}
```

### 4.2 필드 정의

| 필드 | 타입 | 필수 | 설명 |
|------|------|:----:|------|
| `protocol` | string | 예 | 프로토콜 식별자 `"wia-cryo"` |
| `version` | string | 예 | 프로토콜 버전 |
| `messageId` | string | 예 | 고유 메시지 ID (UUID v4) |
| `timestamp` | integer | 예 | Unix 타임스탬프 (ms) |
| `type` | string | 예 | 메시지 유형 |
| `source` | object | 예 | 메시지 소스 정보 |
| `payload` | object | 예 | 유형별 데이터 |

---

## 연결 생명주기

### 5.1 연결 상태

```
                          ┌─────────────────┐
                          │    연결 끊김     │
                          └────────┬────────┘
                                   │ connect()
                                   ▼
                          ┌─────────────────┐
                          │    연결 중      │
                          └────────┬────────┘
                                   │ connect_ack
                    ┌──────────────┼──────────────┐
                    │              ▼              │
         오류/      │     ┌─────────────────┐     │ 인증 실패
         타임아웃    │     │     연결됨      │     │
                    │     └────────┬────────┘     │
                    │              │              │
                    │         연결 끊김/          │
                    │         오류               │
                    │              ▼              │
                    │     ┌─────────────────┐     │
                    │     │   재연결 중     │     │
                    │     └────────┬────────┘     │
                    └──────────────┴──────────────┘
                                   │
                                   ▼
                          ┌─────────────────┐
                          │    연결 끊김     │
                          └─────────────────┘
```

### 5.2 연결 시퀀스

```
클라이언트                                        서버
   │                                                │
   │ ─────────── TLS 핸드셰이크 ────────────────► │
   │                                                │
   │ ◄─────────── TLS 수립 ───────────────────── │
   │                                                │
   │ ─────────── WebSocket 업그레이드 ──────────► │
   │                                                │
   │ ◄─────────── 101 Switching ──────────────── │
   │                                                │
   │ ─────────────── connect ────────────────────► │
   │     { "type": "connect",                      │
   │       "payload": { "token": "jwt..." } }      │
   │                                                │
   │ ◄────────────── connect_ack ────────────────  │
   │     { "type": "connect_ack",                  │
   │       "payload": { "sessionId": "..." } }     │
   │                                                │
   │ ─────────────── subscribe ──────────────────► │
   │                                                │
   │ ◄────────────── subscribe_ack ──────────────  │
   │                                                │
   │ ◄────────────── sensor_data ────────────────  │
   │                  ...                           │
```

### 5.3 재연결 정책

| 매개변수 | 기본값 | 설명 |
|----------|--------|------|
| `autoReconnect` | `true` | 자동 재연결 활성화 |
| `initialDelay` | `1000ms` | 첫 번째 재시도 지연 |
| `maxDelay` | `30000ms` | 최대 재시도 지연 |
| `maxAttempts` | `10` | 최대 재시도 횟수 |
| `backoffMultiplier` | `2.0` | 지수 백오프 |

**재연결 스케줄:**
```
시도 1:  1,000ms
시도 2:  2,000ms
시도 3:  4,000ms
시도 4:  8,000ms
시도 5: 16,000ms
시도 6+: 30,000ms (상한)
```

### 5.4 하트비트

- **간격:** 30초
- **타임아웃:** 10초
- **최대 누락:** 연결 끊기 전 3회 ping

```json
// Ping
{
  "type": "ping",
  "messageId": "ping-001",
  "timestamp": 1704067200000,
  "payload": { "sequence": 1 }
}

// Pong
{
  "type": "pong",
  "messageId": "pong-001",
  "timestamp": 1704067200005,
  "payload": { "sequence": 1, "latency_ms": 5 }
}
```

---

## 메시지 유형

### 6.1 메시지 유형 요약

| 유형 | 방향 | 설명 |
|------|------|------|
| `connect` | C → S | 연결 요청 |
| `connect_ack` | S → C | 연결 응답 |
| `disconnect` | 양방향 | 연결 종료 알림 |
| `subscribe` | C → S | 토픽 구독 |
| `subscribe_ack` | S → C | 구독 확인 |
| `unsubscribe` | C → S | 토픽 구독 해제 |
| `sensor_data` | S → C | 센서 측정값 |
| `alert` | S → C | 알림 통지 |
| `alert_ack` | C → S | 알림 확인 |
| `command` | C → S | 장치 명령 |
| `command_ack` | S → C | 명령 응답 |
| `ping` | C → S | 하트비트 핑 |
| `pong` | S → C | 하트비트 퐁 |
| `error` | 양방향 | 에러 메시지 |

### 6.2 센서 데이터 메시지

```json
{
  "protocol": "wia-cryo",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440100",
  "timestamp": 1704067200200,
  "type": "sensor_data",
  "source": {
    "type": "sensor",
    "id": "SENSOR-DEW-KR-001-TEMP"
  },
  "payload": {
    "containerId": "DEW-KR-001",
    "metricType": "temperature",
    "value": -196.2,
    "unit": "celsius",
    "quality": {
      "accuracy": 0.1,
      "status": "normal"
    }
  }
}
```

### 6.3 알림 메시지

```json
{
  "protocol": "wia-cryo",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440200",
  "timestamp": 1704067200300,
  "type": "alert",
  "source": {
    "type": "service",
    "id": "alert-service"
  },
  "payload": {
    "alertId": "ALERT-2025-001",
    "severity": "warning",
    "category": "temperature",
    "containerId": "DEW-KR-001",
    "message": "온도 편차 감지됨",
    "details": {
      "currentValue": -195.0,
      "expectedValue": -196.0,
      "threshold": 0.5
    },
    "actions": ["acknowledge", "dismiss", "escalate"]
  }
}
```

---

## 보안

### 7.1 전송 보안

- **TLS 버전:** 1.3 필수
- **암호화 스위트:** TLS_AES_256_GCM_SHA384, TLS_CHACHA20_POLY1305_SHA256
- **인증서:** 유효한 TLS 인증서 필수

### 7.2 인증

모든 연결에 JWT 인증 필요:

```json
{
  "type": "connect",
  "payload": {
    "token": "eyJhbGciOiJSUzI1NiIs..."
  }
}
```

### 7.3 권한 부여

토픽 기반 접근 제어:

| 역할 | 허용 토픽 |
|------|----------|
| `viewer` | `containers/+/temperature`, `containers/+/nitrogen` |
| `operator` | 모든 viewer + `commands/*` |
| `admin` | 모든 토픽 |

---

## 에러 처리

### 8.1 에러 코드

| 코드 | 이름 | 설명 |
|------|------|------|
| 1000 | `CONNECTION_CLOSED` | 정상 종료 |
| 1001 | `CONNECTION_LOST` | 예기치 않은 연결 끊김 |
| 1002 | `PROTOCOL_ERROR` | 프로토콜 위반 |
| 2001 | `AUTH_FAILED` | 인증 실패 |
| 2002 | `AUTH_EXPIRED` | 토큰 만료 |
| 2003 | `PERMISSION_DENIED` | 권한 부족 |
| 3001 | `INVALID_MESSAGE` | 잘못된 형식의 메시지 |
| 3002 | `INVALID_TOPIC` | 알 수 없는 토픽 |
| 3003 | `RATE_LIMITED` | 메시지가 너무 많음 |
| 4001 | `CONTAINER_NOT_FOUND` | 컨테이너가 존재하지 않음 |
| 4002 | `SENSOR_OFFLINE` | 센서가 응답하지 않음 |
| 5001 | `INTERNAL_ERROR` | 서버 오류 |

### 8.2 에러 메시지 형식

```json
{
  "protocol": "wia-cryo",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440999",
  "timestamp": 1704067200600,
  "type": "error",
  "payload": {
    "code": 2001,
    "name": "AUTH_FAILED",
    "message": "유효하지 않거나 만료된 토큰",
    "recoverable": true,
    "retryAfter": 0,
    "relatedMessageId": "550e8400-e29b-41d4-a716-446655440000"
  }
}
```

---

## 예제

### 9.1 전체 세션 예제 (TypeScript)

```typescript
import { WiaCryoProtocol, ConnectionState } from '@wia/cryo-protocol';

const protocol = new WiaCryoProtocol({
  url: 'wss://ws.wia.live/cryo-preservation/v1',
  token: 'your-jwt-token',
  facilityId: 'FAC-KR-001',
  autoReconnect: true
});

// 연결 이벤트
protocol.on('connected', (session) => {
  console.log('연결됨:', session.sessionId);
});

protocol.on('disconnected', (reason) => {
  console.log('연결 끊김:', reason);
});

// 센서 데이터 구독
await protocol.subscribe([
  'containers/DEW-KR-001/temperature',
  'containers/DEW-KR-001/nitrogen',
  'containers/+/alerts'
]);

// 센서 데이터 처리
protocol.on('sensor_data', (data) => {
  console.log(`${data.containerId} ${data.metricType}: ${data.value}${data.unit}`);
});

// 알림 처리
protocol.on('alert', async (alert) => {
  console.log(`알림 [${alert.severity}]: ${alert.message}`);

  // 알림 확인
  await protocol.acknowledgeAlert(alert.alertId, {
    action: 'acknowledge',
    notes: '조사 중'
  });
});

// 명령 전송
await protocol.sendCommand({
  command: 'refill_nitrogen',
  target: 'DEW-KR-001',
  parameters: { targetLevel: 0.95 }
});

// 정리
await protocol.disconnect();
```

### 9.2 Python 예제

```python
import asyncio
from wia_cryo import WiaCryoProtocol

async def main():
    protocol = WiaCryoProtocol(
        url='wss://ws.wia.live/cryo-preservation/v1',
        token='your-jwt-token',
        facility_id='FAC-KR-001'
    )

    await protocol.connect()

    # 구독
    await protocol.subscribe([
        'containers/DEW-KR-001/temperature',
        'containers/+/alerts'
    ])

    # 메시지 처리
    @protocol.on('sensor_data')
    async def on_sensor_data(data):
        print(f"{data['containerId']}: {data['value']}")

    @protocol.on('alert')
    async def on_alert(alert):
        print(f"알림: {alert['message']}")
        await protocol.acknowledge_alert(alert['alertId'])

    # 실행 유지
    await asyncio.sleep(3600)

    await protocol.disconnect()

asyncio.run(main())
```

---

<div align="center">

**WIA 냉동보존 통신 프로토콜 v1.0.0**

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

---

**© 2025 WIA 표준 위원회**

**MIT License**

</div>
