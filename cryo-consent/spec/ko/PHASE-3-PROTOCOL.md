# WIA Cryo-Consent Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## 목차

1. [개요](#개요)
2. [프로토콜 아키텍처](#프로토콜-아키텍처)
3. [Message 타입](#message-타입)
4. [실시간 알림](#실시간-알림)
5. [Event Streaming](#event-streaming)
6. [보안 및 인증](#보안-및-인증)
7. [구현 예제](#구현-예제)
8. [버전 이력](#버전-이력)

---

## 개요

### 1.1 목적

WIA Cryo-Consent Protocol Standard는 동의 변경 알림, event streaming 및 다자간 동기화를 위한 실시간 통신 프로토콜을 정의합니다. 이 표준은 냉동보존 생태계의 모든 이해관계자에게 동의 수정, 철회 및 법적 상태 변경에 대한 즉각적인 알림을 가능하게 합니다.

**핵심 목표**:
- 모든 권한 있는 당사자에게 실시간 동의 변경 알림 제공
- 실시간 업데이트를 위한 WebSocket 및 Server-Sent Events (SSE) 지원
- 다자간 동의 검증 워크플로 촉진
- 안전하고 암호화된 알림 전달 보장
- Event replay 및 감사 추적 재구성 지원
- 고가용성 및 장애 허용 아키텍처 지원

### 1.2 사용 사례

| Use Case | 설명 | Stakeholders |
|----------|------|--------------|
| Consent Modification | 동의 변경의 실시간 알림 | 시설, 법률, 대리인 |
| Emergency Revocation | 즉각적인 동의 철회 알림 | 모든 당사자 |
| Proxy Authorization | 새로운 대리인 위임 알림 | Grantor, 시설, 법률 |
| Legal Status Change | 관할권 준수 업데이트 | 법률, 규정 준수 |
| Multi-Party Signing | 협업 증인 워크플로 | 증인, 공증인 |
| Audit Monitoring | 실시간 감사 추적 업데이트 | 감사자, 규제 기관 |

### 1.3 프로토콜 스택

| Layer | Technology | 목적 |
|-------|-----------|------|
| Application | JSON Messages | 동의 event payload |
| Transport | WebSocket / SSE / HTTP/2 | 실시간 양방향 통신 |
| Security | TLS 1.3 / mTLS | 암호화 및 인증 |
| Infrastructure | Load Balancer / Message Queue | 확장성 및 안정성 |

---

## 프로토콜 아키텍처

### 2.1 시스템 아키텍처

```
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│   Client A   │         │   Client B   │         │   Client C   │
│  (Grantor)   │         │ (Healthcare) │         │   (Legal)    │
└──────┬───────┘         └──────┬───────┘         └──────┬───────┘
       │                        │                        │
       │ WebSocket              │ WebSocket              │ SSE
       │                        │                        │
       └────────────────────────┼────────────────────────┘
                                │
                    ┌───────────▼───────────┐
                    │   WIA Consent Hub     │
                    │  - Connection Manager │
                    │  - Event Router       │
                    │  - Auth Gateway       │
                    └───────────┬───────────┘
                                │
                    ┌───────────▼───────────┐
                    │   Message Broker      │
                    │  (Kafka / RabbitMQ)   │
                    └───────────┬───────────┘
                                │
                ┌───────────────┼───────────────┐
                │               │               │
        ┌───────▼──────┐ ┌─────▼──────┐ ┌─────▼──────┐
        │   Consent    │ │   Legal    │ │ Blockchain │
        │   Service    │ │  Validation│ │   Anchor   │
        └──────────────┘ └────────────┘ └────────────┘
```

### 2.2 연결 유형

| Type | Protocol | Use Case | Latency |
|------|----------|----------|---------|
| WebSocket | WSS | 양방향 실시간 | <100ms |
| Server-Sent Events | HTTPS/SSE | 단방향 streaming | <200ms |
| HTTP/2 Server Push | HTTPS | Push 알림 | <300ms |
| Webhook | HTTPS POST | 비동기 callback | 가변적 |

### 2.3 서비스 품질

| Metric | Target | 설명 |
|--------|--------|------|
| Latency (p95) | <500ms | 95번째 백분위수 메시지 전달 |
| Throughput | 10,000 msg/sec | 노드당 초당 메시지 수 |
| Availability | 99.95% | 서비스 가동 시간 |
| Connection Limit | 100,000 | 인스턴스당 동시 연결 |
| Message Retention | 7일 | Event replay 기능 |

---

## Message 타입

### 3.1 Event Message 구조

모든 프로토콜 메시지는 다음 구조를 따릅니다:

```json
{
  "eventId": "evt_550e8400-e29b-41d4-a716-446655440001",
  "eventType": "consent.modified",
  "timestamp": "2025-01-15T10:30:00.123Z",
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "version": 2,
  "actor": {
    "id": "PERSON-001",
    "type": "grantor"
  },
  "payload": {
    // Event별 데이터
  },
  "meta": {
    "signature": "0xabcdef...",
    "hash": "sha256:...",
    "previousEventId": "evt_prev..."
  }
}
```

### 3.2 Event 타입

| Event Type | 설명 | Priority |
|------------|------|----------|
| `consent.created` | 새로운 동의 기록 생성됨 | NORMAL |
| `consent.modified` | 동의 수정됨 | HIGH |
| `consent.revoked` | 동의가 영구적으로 철회됨 | CRITICAL |
| `consent.verified` | 제3자 검증 완료 | NORMAL |
| `proxy.added` | 새로운 대리인 승인됨 | HIGH |
| `proxy.revoked` | 대리인 권한 제거됨 | HIGH |
| `legal.witnessed` | 증인 서명 추가됨 | NORMAL |
| `legal.notarized` | 공증 완료 | HIGH |
| `jurisdiction.updated` | 법적 관할권 변경됨 | NORMAL |
| `blockchain.anchored` | Blockchain anchoring 완료 | NORMAL |
| `compliance.alert` | 규정 준수 문제 감지됨 | CRITICAL |
| `audit.requested` | 감사 추적 요청됨 | NORMAL |

### 3.3 Event 우선순위 수준

| Priority | Delivery SLA | Retry Policy | 설명 |
|----------|--------------|--------------|------|
| CRITICAL | 즉시 | 10회 시도, 지수 백오프 | 생명 중요 event |
| HIGH | <5초 | 5회 시도 | 중요한 상태 변경 |
| NORMAL | <30초 | 3회 시도 | 표준 알림 |
| LOW | Best effort | 1회 시도 | 정보성 업데이트 |

---

## 실시간 알림

### 4.1 WebSocket 연결

#### 4.1.1 연결 설정

```typescript
const ws = new WebSocket('wss://events.wia.live/cryo-consent/v1/stream');

// 인증
ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'eyJhbGciOiJSUzI1NiIs...',
    subscriptions: [
      'consent.modified',
      'consent.revoked',
      'proxy.added'
    ],
    filters: {
      consentId: '550e8400-e29b-41d4-a716-446655440001'
    }
  }));
};

// 인증 응답 처리
ws.onmessage = (event) => {
  const message = JSON.parse(event.data);

  if (message.type === 'auth.success') {
    console.log('연결되고 인증됨');
  } else if (message.type === 'event') {
    handleConsentEvent(message.data);
  }
};

// 오류 처리
ws.onerror = (error) => {
  console.error('WebSocket 오류:', error);
};

// 재연결 로직
ws.onclose = (event) => {
  if (!event.wasClean) {
    setTimeout(reconnect, 5000);
  }
};
```

#### 4.1.2 Subscription 관리

```json
{
  "type": "subscribe",
  "subscriptions": [
    {
      "eventType": "consent.modified",
      "filters": {
        "grantorId": "PERSON-001",
        "jurisdiction": ["US-CA"]
      }
    },
    {
      "eventType": "consent.revoked",
      "filters": {
        "facilityId": "FAC-001"
      }
    }
  ]
}
```

**Response:**
```json
{
  "type": "subscription.confirmed",
  "subscriptionId": "sub_1234567890",
  "subscriptions": [
    "consent.modified",
    "consent.revoked"
  ],
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 4.2 Server-Sent Events (SSE)

```typescript
const eventSource = new EventSource(
  'https://events.wia.live/cryo-consent/v1/stream/sse?' +
  'token=eyJhbGciOiJSUzI1NiIs...' +
  '&events=consent.modified,consent.revoked'
);

eventSource.addEventListener('consent.modified', (event) => {
  const data = JSON.parse(event.data);
  console.log('동의 수정됨:', data);
});

eventSource.addEventListener('consent.revoked', (event) => {
  const data = JSON.parse(event.data);
  console.log('동의 철회됨:', data);
  // 긴급 절차 트리거
  handleEmergencyRevocation(data);
});

eventSource.onerror = (error) => {
  console.error('SSE 오류:', error);
  // 브라우저에서 자동 재연결 처리
};
```

### 4.3 Webhook 알림

#### 4.3.1 Webhook 등록

```http
POST /api/v1/webhooks
Content-Type: application/json
Authorization: Bearer eyJhbGciOiJSUzI1NiIs...

{
  "url": "https://facility.example.com/webhooks/consent",
  "events": [
    "consent.modified",
    "consent.revoked",
    "proxy.added"
  ],
  "filters": {
    "facilityId": "FAC-001"
  },
  "secret": "whsec_1234567890abcdef"
}
```

**Response:**
```json
{
  "webhookId": "wh_550e8400-e29b-41d4-a716-446655440001",
  "url": "https://facility.example.com/webhooks/consent",
  "events": ["consent.modified", "consent.revoked", "proxy.added"],
  "status": "active",
  "createdAt": "2025-01-15T10:30:00Z"
}
```

#### 4.3.2 Webhook Payload

```http
POST /webhooks/consent HTTP/1.1
Host: facility.example.com
Content-Type: application/json
X-WIA-Event-Type: consent.modified
X-WIA-Event-ID: evt_550e8400-e29b-41d4-a716-446655440001
X-WIA-Signature: sha256=a5b9c3d4e5f6...
X-WIA-Timestamp: 2025-01-15T10:30:00Z

{
  "eventId": "evt_550e8400-e29b-41d4-a716-446655440001",
  "eventType": "consent.modified",
  "timestamp": "2025-01-15T10:30:00Z",
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "version": 2,
  "actor": {
    "id": "PERSON-001",
    "type": "grantor"
  },
  "payload": {
    "modifications": {
      "consent.scope.research.categories": ["medical", "scientific", "commercial"]
    },
    "previousVersion": 1
  },
  "meta": {
    "signature": "0xabcdef...",
    "hash": "sha256:..."
  }
}
```

#### 4.3.3 Webhook 서명 검증

```python
import hmac
import hashlib

def verify_webhook_signature(
    payload: bytes,
    signature: str,
    secret: str,
    timestamp: str
) -> bool:
    """
    HMAC-SHA256을 사용하여 webhook 서명 검증.

    Args:
        payload: 원시 요청 본문
        signature: X-WIA-Signature 헤더 값
        secret: Webhook 비밀
        timestamp: X-WIA-Timestamp 헤더 값

    Returns:
        서명이 유효하면 True
    """
    # Replay 공격 방지 (5분 허용)
    from datetime import datetime, timedelta
    event_time = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
    if abs((datetime.utcnow() - event_time)) > timedelta(minutes=5):
        return False

    # 예상 서명 계산
    signed_payload = f"{timestamp}.{payload.decode('utf-8')}"
    expected_signature = hmac.new(
        secret.encode('utf-8'),
        signed_payload.encode('utf-8'),
        hashlib.sha256
    ).hexdigest()

    # 서명 비교
    signature_value = signature.split('=')[1] if '=' in signature else signature
    return hmac.compare_digest(expected_signature, signature_value)
```

---

## Event Streaming

### 5.1 Event Stream Subscription

```typescript
interface EventStreamOptions {
  apiKey: string;
  events: string[];
  filters?: Record<string, any>;
  fromTimestamp?: string;
  maxRetries?: number;
}

class ConsentEventStream {
  private ws: WebSocket | null = null;
  private reconnectAttempts = 0;
  private readonly maxReconnectAttempts = 10;

  constructor(private options: EventStreamOptions) {}

  connect(): void {
    const url = new URL('wss://events.wia.live/cryo-consent/v1/stream');

    this.ws = new WebSocket(url.toString());

    this.ws.onopen = () => {
      console.log('Event stream에 연결됨');
      this.reconnectAttempts = 0;
      this.authenticate();
    };

    this.ws.onmessage = (event) => {
      this.handleMessage(JSON.parse(event.data));
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket 오류:', error);
    };

    this.ws.onclose = (event) => {
      console.log('연결 종료:', event.code);
      this.reconnect();
    };
  }

  private authenticate(): void {
    if (!this.ws) return;

    this.ws.send(JSON.stringify({
      type: 'auth',
      token: this.options.apiKey,
      subscriptions: this.options.events,
      filters: this.options.filters,
      fromTimestamp: this.options.fromTimestamp
    }));
  }

  private handleMessage(message: any): void {
    switch (message.type) {
      case 'auth.success':
        console.log('인증 성공');
        break;
      case 'auth.error':
        console.error('인증 실패:', message.error);
        break;
      case 'event':
        this.onEvent(message.data);
        break;
      case 'heartbeat':
        this.onHeartbeat();
        break;
      default:
        console.warn('알 수 없는 메시지 유형:', message.type);
    }
  }

  private reconnect(): void {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error('최대 재연결 시도 횟수 도달');
      return;
    }

    const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 30000);
    this.reconnectAttempts++;

    console.log(`${delay}ms 후 재연결 (시도 ${this.reconnectAttempts})`);
    setTimeout(() => this.connect(), delay);
  }

  onEvent(event: any): void {
    // 구현에서 재정의
  }

  onHeartbeat(): void {
    // 구현에서 재정의
  }

  disconnect(): void {
    if (this.ws) {
      this.ws.close(1000, 'Client disconnect');
      this.ws = null;
    }
  }
}
```

### 5.2 Event Replay

```http
GET /api/v1/events/replay
```

**Query Parameters:**
- `consentId` (string, required): Consent ID
- `fromTimestamp` (string, required): ISO 8601 timestamp
- `toTimestamp` (string, optional): ISO 8601 timestamp
- `eventTypes` (string[], optional): Event type으로 필터링

**Response:**
```json
{
  "events": [
    {
      "eventId": "evt_001",
      "eventType": "consent.created",
      "timestamp": "2025-01-15T10:30:00Z",
      "consentId": "550e8400-e29b-41d4-a716-446655440001",
      "payload": { /* ... */ }
    },
    {
      "eventId": "evt_002",
      "eventType": "consent.modified",
      "timestamp": "2025-02-20T14:00:00Z",
      "consentId": "550e8400-e29b-41d4-a716-446655440001",
      "payload": { /* ... */ }
    }
  ],
  "meta": {
    "total": 2,
    "fromTimestamp": "2025-01-15T10:30:00Z",
    "toTimestamp": "2025-02-20T14:00:00Z"
  }
}
```

### 5.3 Event 필터링

| Filter | Type | 설명 | 예제 |
|--------|------|------|------|
| `consentId` | string | Consent ID로 필터링 | `550e8400-e29b-41d4-a716-446655440001` |
| `grantorId` | string | Grantor로 필터링 | `PERSON-001` |
| `facilityId` | string | 시설로 필터링 | `FAC-001` |
| `jurisdiction` | string[] | 관할권으로 필터링 | `["US-CA", "US-NY"]` |
| `eventType` | string[] | Event type으로 필터링 | `["consent.modified", "consent.revoked"]` |
| `priority` | string | 우선순위로 필터링 | `CRITICAL` |
| `fromTimestamp` | timestamp | Timestamp 이후의 event | `2025-01-15T00:00:00Z` |
| `toTimestamp` | timestamp | Timestamp 이전의 event | `2025-12-31T23:59:59Z` |

---

## 보안 및 인증

### 6.1 인증 방법

| Method | Transport | Use Case |
|--------|-----------|----------|
| JWT Bearer Token | WebSocket/SSE | 표준 인증 |
| OAuth 2.0 | WebSocket/SSE | 타사 통합 |
| API Key | Webhook callback | Server-to-server |
| mTLS | WebSocket | 높은 보안 환경 |

### 6.2 연결 인증

```json
{
  "type": "auth",
  "method": "bearer",
  "token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "subscriptions": ["consent.modified"],
  "filters": {
    "consentId": "550e8400-e29b-41d4-a716-446655440001"
  }
}
```

**성공 응답:**
```json
{
  "type": "auth.success",
  "subscriptionId": "sub_1234567890",
  "expiresAt": "2025-01-15T12:30:00Z",
  "capabilities": [
    "subscribe",
    "replay",
    "filter"
  ]
}
```

**오류 응답:**
```json
{
  "type": "auth.error",
  "error": {
    "code": "INVALID_TOKEN",
    "message": "JWT token이 만료되었습니다"
  }
}
```

### 6.3 암호화

| Layer | Protocol | Key Size |
|-------|----------|----------|
| Transport | TLS 1.3 | 256-bit |
| Message | AES-GCM | 256-bit |
| Signature | ECDSA | P-256 |

---

## 구현 예제

### 7.1 TypeScript - 완전한 Event Listener

```typescript
import WebSocket from 'ws';

interface ConsentEvent {
  eventId: string;
  eventType: string;
  timestamp: string;
  consentId: string;
  version: number;
  actor: {
    id: string;
    type: string;
  };
  payload: any;
  meta: {
    signature: string;
    hash: string;
  };
}

class ConsentEventListener {
  private ws: WebSocket | null = null;
  private heartbeatInterval: NodeJS.Timeout | null = null;

  constructor(
    private apiKey: string,
    private eventHandlers: Map<string, (event: ConsentEvent) => void>
  ) {}

  connect(): void {
    this.ws = new WebSocket('wss://events.wia.live/cryo-consent/v1/stream');

    this.ws.on('open', () => {
      console.log('동의 event stream에 연결됨');
      this.authenticate();
      this.startHeartbeat();
    });

    this.ws.on('message', (data: string) => {
      const message = JSON.parse(data);
      this.handleMessage(message);
    });

    this.ws.on('error', (error) => {
      console.error('WebSocket 오류:', error);
    });

    this.ws.on('close', () => {
      console.log('연결 종료');
      this.stopHeartbeat();
      this.reconnect();
    });
  }

  private authenticate(): void {
    if (!this.ws) return;

    const subscriptions = Array.from(this.eventHandlers.keys());

    this.ws.send(JSON.stringify({
      type: 'auth',
      token: this.apiKey,
      subscriptions: subscriptions
    }));
  }

  private handleMessage(message: any): void {
    switch (message.type) {
      case 'auth.success':
        console.log('인증 성공');
        break;

      case 'event':
        this.handleEvent(message.data);
        break;

      case 'heartbeat':
        // 서버 heartbeat 수신됨
        break;

      default:
        console.warn('알 수 없는 메시지 유형:', message.type);
    }
  }

  private handleEvent(event: ConsentEvent): void {
    console.log(`Event 수신됨: ${event.eventType} for consent ${event.consentId}`);

    const handler = this.eventHandlers.get(event.eventType);
    if (handler) {
      try {
        handler(event);
      } catch (error) {
        console.error(`Event ${event.eventType} 처리 오류:`, error);
      }
    }
  }

  private startHeartbeat(): void {
    this.heartbeatInterval = setInterval(() => {
      if (this.ws?.readyState === WebSocket.OPEN) {
        this.ws.send(JSON.stringify({ type: 'ping' }));
      }
    }, 30000); // 30초
  }

  private stopHeartbeat(): void {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }
  }

  private reconnect(): void {
    setTimeout(() => {
      console.log('재연결 시도 중...');
      this.connect();
    }, 5000);
  }

  disconnect(): void {
    this.stopHeartbeat();
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }
}

// 사용 예
const eventHandlers = new Map<string, (event: ConsentEvent) => void>([
  ['consent.modified', (event) => {
    console.log(`동의 ${event.consentId}가 ${event.actor.id}에 의해 수정되었습니다`);
    // 로컬 캐시 업데이트, 이해관계자에게 알림 등
  }],

  ['consent.revoked', (event) => {
    console.log(`긴급: 동의 ${event.consentId}가 철회되었습니다!`);
    // 긴급 절차 트리거
    handleEmergencyRevocation(event);
  }],

  ['proxy.added', (event) => {
    console.log(`동의 ${event.consentId}에 새 대리인 추가됨`);
    // 접근 제어 목록 업데이트
  }]
]);

const listener = new ConsentEventListener(
  'wia_live_sk_1234567890',
  eventHandlers
);

listener.connect();

// 우아한 종료
process.on('SIGINT', () => {
  console.log('종료 중...');
  listener.disconnect();
  process.exit(0);
});

function handleEmergencyRevocation(event: ConsentEvent): void {
  // 긴급 철회 처리 구현
  console.log('긴급 철회 절차 시작됨');

  // 1. 관련 당사자 모두에게 알림
  // 2. 보존 기록 업데이트
  // 3. 철회 문서화
  // 4. 감사 추적 생성
}
```

### 7.2 Python - Webhook 서버

```python
from flask import Flask, request, jsonify
import hmac
import hashlib
from datetime import datetime, timedelta

app = Flask(__name__)

WEBHOOK_SECRET = "whsec_1234567890abcdef"

def verify_webhook_signature(request) -> bool:
    """Webhook 서명 검증."""
    signature = request.headers.get('X-WIA-Signature', '')
    timestamp = request.headers.get('X-WIA-Timestamp', '')

    # Timestamp 확인하여 replay 공격 방지
    try:
        event_time = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
        if abs((datetime.utcnow() - event_time).total_seconds()) > 300:
            return False
    except ValueError:
        return False

    # 예상 서명 계산
    payload = request.get_data()
    signed_payload = f"{timestamp}.{payload.decode('utf-8')}"
    expected_signature = hmac.new(
        WEBHOOK_SECRET.encode('utf-8'),
        signed_payload.encode('utf-8'),
        hashlib.sha256
    ).hexdigest()

    # 서명 값 추출
    signature_value = signature.split('=')[1] if '=' in signature else signature

    return hmac.compare_digest(expected_signature, signature_value)

@app.route('/webhooks/consent', methods=['POST'])
def handle_webhook():
    """Webhook 수신 처리."""

    # 서명 검증
    if not verify_webhook_signature(request):
        return jsonify({'error': '유효하지 않은 서명'}), 401

    # Event 파싱
    event = request.get_json()
    event_type = request.headers.get('X-WIA-Event-Type')

    print(f"Webhook 수신됨: {event_type}")

    # 유형에 따라 event 처리
    if event_type == 'consent.modified':
        handle_consent_modified(event)
    elif event_type == 'consent.revoked':
        handle_consent_revoked(event)
    elif event_type == 'proxy.added':
        handle_proxy_added(event)
    else:
        print(f"알 수 없는 event 유형: {event_type}")

    # 수신 확인을 위해 200 반환
    return jsonify({'received': True}), 200

def handle_consent_modified(event):
    """동의 수정 event 처리."""
    consent_id = event['consentId']
    version = event['version']

    print(f"동의 수정 처리 중: {consent_id} v{version}")

    # 로컬 데이터베이스 업데이트
    # 관련 당사자에게 알림
    # 비즈니스 로직 트리거

def handle_consent_revoked(event):
    """동의 철회 event 처리."""
    consent_id = event['consentId']

    print(f"긴급: 동의 철회 처리 중: {consent_id}")

    # 긴급 절차
    # 보존 기록 업데이트
    # 모든 이해관계자에게 알림

def handle_proxy_added(event):
    """대리인 추가 event 처리."""
    consent_id = event['consentId']
    proxy_id = event['payload']['proxyId']

    print(f"대리인 추가 처리 중: {consent_id}의 {proxy_id}")

    # 접근 제어 업데이트
    # 대리인에게 알림

if __name__ == '__main__':
    app.run(port=8080)
```

---

## 버전 이력

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | 초기 릴리스 |

---

## 부록 A: 관련 표준

| Standard | 관계 |
|----------|------|
| WIA Cryo-Consent Phase 1 | Event의 데이터 형식 |
| WIA Cryo-Consent Phase 2 | 쿼리를 위한 API endpoint |
| WebSocket Protocol RFC 6455 | 실시간 전송 |
| Server-Sent Events W3C | 단방향 streaming |
| CloudEvents CNCF | Event 형식 상호운용성 |

---

<div align="center">

**WIA Cryo-Consent Protocol Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
