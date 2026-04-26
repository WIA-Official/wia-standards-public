# WIA 디지털 장례 표준 - Phase 3: Protocol 명세서

**버전:** 1.0.0
**상태:** Draft
**날짜:** 2025-12-18
**Primary Color:** #64748B (Slate)
**시리즈:** Digital Death Services

---

## 목차

1. [소개](#소개)
2. [WebSocket Protocol](#websocket-protocol)
3. [Webhook 시스템](#webhook-시스템)
4. [Event Streaming](#event-streaming)
5. [실시간 스트리밍 Protocol](#실시간-스트리밍-protocol)
6. [채팅 및 상호작용 Protocol](#채팅-및-상호작용-protocol)
7. [동기화 Protocol](#동기화-protocol)
8. [보안 및 암호화](#보안-및-암호화)
9. [메시지 형식 명세](#메시지-형식-명세)
10. [Protocol 상태 관리](#protocol-상태-관리)
11. [구현 예제](#구현-예제)

---

## 1. 소개

### 1.1 목적

본 명세서는 디지털 장례 서비스의 실시간 상호작용을 위한 통신 protocol을 정의합니다. 이러한 protocol은 라이브 스트리밍, 실시간 채팅, 이벤트 알림 및 클라이언트와 서버 간의 데이터 동기화를 가능하게 하여 원활한 가상 및 하이브리드 장례 경험을 보장합니다.

### 1.2 Protocol 개요

| Protocol | 사용 사례 | Transport | 지속성 |
|----------|---------|-----------|-------|
| WebSocket | 실시간 양방향 통신 | WSS (WebSocket Secure) | 연결 기반 |
| Webhook | 외부 시스템으로 이벤트 알림 | HTTPS | 이벤트 기반 |
| Server-Sent Events (SSE) | 단방향 서버-클라이언트 업데이트 | HTTPS | 연결 기반 |
| WebRTC | Peer-to-peer 오디오/비디오 스트리밍 | UDP/TCP | 세션 기반 |
| MQTT | IoT 장치 통합 | TCP/TLS | 구독 기반 |

### 1.3 연결 요구사항

```
WebSocket URL: wss://ws.funeral-service.example.com/v1
Webhook Endpoint: https://your-server.com/webhooks/funeral-events
SSE Endpoint: https://api.funeral-service.example.com/v1/events/stream
MQTT Broker: mqtts://mqtt.funeral-service.example.com:8883
```

---

## 2. WebSocket Protocol

### 2.1 연결 설정

```javascript
// 인증이 포함된 클라이언트 연결
const ws = new WebSocket('wss://ws.funeral-service.example.com/v1');

ws.onopen = () => {
  // 연결 후 인증
  ws.send(JSON.stringify({
    type: 'auth',
    payload: {
      token: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...',
      clientId: 'client-12345',
      userAgent: 'Mozilla/5.0...'
    }
  }));
};
```

### 2.2 인증 메시지

```json
{
  "type": "auth",
  "payload": {
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "clientId": "client-12345",
    "userAgent": "Mozilla/5.0...",
    "capabilities": ["video", "audio", "chat"]
  },
  "timestamp": "2025-12-18T14:00:00Z",
  "messageId": "msg_1234567890"
}
```

**서버 응답:**
```json
{
  "type": "auth_success",
  "payload": {
    "sessionId": "session_abc123",
    "expiresAt": "2025-12-18T18:00:00Z",
    "permissions": ["stream.view", "chat.send", "react.send"]
  },
  "timestamp": "2025-12-18T14:00:01Z",
  "messageId": "msg_1234567891"
}
```

### 2.3 구독 관리

```json
{
  "type": "subscribe",
  "payload": {
    "channels": [
      "service:660e8400-e29b-41d4-a716-446655440001",
      "chat:660e8400-e29b-41d4-a716-446655440001",
      "reactions:660e8400-e29b-41d4-a716-446655440001"
    ]
  },
  "timestamp": "2025-12-18T14:00:02Z",
  "messageId": "msg_1234567892"
}
```

**서버 확인:**
```json
{
  "type": "subscribed",
  "payload": {
    "channels": [
      "service:660e8400-e29b-41d4-a716-446655440001",
      "chat:660e8400-e29b-41d4-a716-446655440001",
      "reactions:660e8400-e29b-41d4-a716-446655440001"
    ],
    "subscriptionId": "sub_xyz789"
  },
  "timestamp": "2025-12-18T14:00:03Z",
  "messageId": "msg_1234567893"
}
```

### 2.4 Heartbeat 및 Keep-Alive

```json
{
  "type": "ping",
  "timestamp": "2025-12-18T14:05:00Z",
  "messageId": "msg_1234567894"
}
```

**서버 응답:**
```json
{
  "type": "pong",
  "timestamp": "2025-12-18T14:05:00Z",
  "messageId": "msg_1234567895"
}
```

**Heartbeat 구성:**
| 매개변수 | 값 | 설명 |
|---------|---|------|
| Interval | 30초 | 클라이언트가 30초마다 ping 전송 |
| Timeout | 60초 | ping이 수신되지 않으면 서버가 연결 종료 |
| Reconnect Delay | 5초 | 초기 재연결 지연 |
| Max Reconnect Attempts | 10 | 최대 재시도 횟수 |

### 2.5 메시지 유형

| 유형 | 방향 | 설명 |
|-----|------|------|
| auth | Client → Server | 인증 요청 |
| auth_success | Server → Client | 인증 확인 |
| auth_error | Server → Client | 인증 실패 |
| subscribe | Client → Server | Channel 구독 |
| subscribed | Server → Client | 구독 확인 |
| unsubscribe | Client → Server | Channel 구독 취소 |
| message | 양방향 | 일반 메시지 |
| chat | 양방향 | 채팅 메시지 |
| reaction | Client → Server | 이모지 반응 |
| stream_update | Server → Client | 스트림 상태 업데이트 |
| viewer_count | Server → Client | 현재 시청자 수 |
| service_update | Server → Client | 서비스 정보 업데이트 |
| error | Server → Client | 오류 알림 |
| ping | Client → Server | Heartbeat ping |
| pong | Server → Client | Heartbeat pong |

---

## 3. Webhook 시스템

### 3.1 Webhook 등록

```http
POST /v1/webhooks
Authorization: Bearer {token}
Content-Type: application/json

{
  "url": "https://your-server.com/webhooks/funeral-events",
  "events": [
    "service.created",
    "service.updated",
    "service.started",
    "service.ended",
    "rsvp.submitted",
    "condolence.posted",
    "donation.received",
    "stream.started",
    "stream.ended"
  ],
  "secret": "whsec_1234567890abcdef",
  "active": true
}
```

**응답:**
```json
{
  "success": true,
  "data": {
    "id": "webhook_abc123",
    "url": "https://your-server.com/webhooks/funeral-events",
    "events": ["service.created", "service.updated", "..."],
    "secret": "whsec_1234567890abcdef",
    "active": true,
    "createdAt": "2025-12-18T10:00:00Z"
  }
}
```

### 3.2 Webhook 이벤트 Payload

```json
{
  "id": "evt_1234567890",
  "type": "service.started",
  "created": "2025-12-18T14:00:00Z",
  "data": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
    "serviceType": "memorial",
    "startDateTime": "2025-12-18T14:00:00-08:00",
    "virtualService": {
      "enabled": true,
      "streamingUrl": "https://stream.example.com/service/660e8400",
      "viewerUrl": "https://watch.example.com/660e8400"
    }
  },
  "metadata": {
    "webhookId": "webhook_abc123",
    "attempt": 1,
    "signature": "t=1734529200,v1=5257a869e7ecebeda32affa62cdca3fa51cad7e77a0e56ff536d0ce8e108d8bd"
  }
}
```

### 3.3 Webhook 서명 검증

```javascript
const crypto = require('crypto');

function verifyWebhookSignature(payload, signature, secret) {
  const [timestampPart, signaturePart] = signature.split(',');
  const timestamp = timestampPart.split('=')[1];
  const expectedSignature = signaturePart.split('=')[1];

  const signedPayload = `${timestamp}.${JSON.stringify(payload)}`;
  const computedSignature = crypto
    .createHmac('sha256', secret)
    .update(signedPayload)
    .digest('hex');

  return crypto.timingSafeEqual(
    Buffer.from(expectedSignature),
    Buffer.from(computedSignature)
  );
}

// 사용법
app.post('/webhooks/funeral-events', (req, res) => {
  const signature = req.headers['x-webhook-signature'];
  const payload = req.body;
  const secret = 'whsec_1234567890abcdef';

  if (!verifyWebhookSignature(payload, signature, secret)) {
    return res.status(401).json({ error: '잘못된 서명' });
  }

  // Webhook 이벤트 처리
  handleWebhookEvent(payload);

  res.status(200).json({ received: true });
});
```

### 3.4 Webhook 이벤트 유형

| 이벤트 유형 | 트리거 | Payload 포함 내용 |
|-----------|-------|-----------------|
| service.created | 새 서비스 예약됨 | 서비스 세부 정보 |
| service.updated | 서비스 정보 변경됨 | 업데이트된 필드 |
| service.started | 서비스 시작됨 | 시작 시간, 스트림 URL |
| service.ended | 서비스 종료됨 | 종료 시간, 소요 시간, 통계 |
| service.cancelled | 서비스 취소됨 | 취소 사유 |
| rsvp.submitted | 손님 RSVP 수신됨 | RSVP 세부 정보 |
| rsvp.updated | 손님이 RSVP 업데이트 | 변경된 필드 |
| condolence.posted | 새 조문 메시지 | 메시지 내용 |
| donation.received | 기부 처리됨 | 금액, 기부자 정보 |
| stream.started | 라이브 스트림 시작됨 | 스트림 URL, 시청자 URL |
| stream.ended | 라이브 스트림 종료됨 | 통계, 녹화본 URL |
| tribute.created | 새 추모 헌사 | 헌사 세부 정보 |

### 3.5 Webhook 재시도 정책

| 시도 | 지연 | 누적 경과 시간 |
|-----|-----|-------------|
| 1 | 즉시 | 0초 |
| 2 | 5초 | 5초 |
| 3 | 15초 | 20초 |
| 4 | 1분 | 1분 20초 |
| 5 | 5분 | 6분 20초 |
| 6 | 15분 | 21분 20초 |
| 7 | 1시간 | 1시간 21분 20초 |
| 8 | 3시간 | 4시간 21분 20초 |

**재시도 기준:**
- HTTP 5xx 오류: 재시도
- HTTP 4xx 오류 (429 제외): 재시도 안 함
- HTTP 429 (요청 제한): 지수 백오프로 재시도
- 연결 시간 초과: 재시도
- DNS 오류: 재시도

---

## 4. Event Streaming

### 4.1 Server-Sent Events (SSE) 연결

```javascript
const eventSource = new EventSource(
  'https://api.funeral-service.example.com/v1/events/stream?token=eyJhbG...',
  { withCredentials: true }
);

eventSource.addEventListener('service_update', (event) => {
  const data = JSON.parse(event.data);
  console.log('서비스 업데이트:', data);
});

eventSource.addEventListener('viewer_count', (event) => {
  const data = JSON.parse(event.data);
  console.log('현재 시청자:', data.count);
});

eventSource.onerror = (error) => {
  console.error('SSE 오류:', error);
  // 재연결 로직 구현
};
```

### 4.2 SSE 이벤트 형식

```
event: service_update
id: evt_1234567890
data: {"serviceId":"660e8400","status":"in-progress","currentSegment":"eulogy"}

event: viewer_count
id: evt_1234567891
data: {"serviceId":"660e8400","count":87,"peak":92}

event: chat_message
id: evt_1234567892
data: {"serviceId":"660e8400","author":"김민수","message":"아름다운 예식입니다","timestamp":"2025-12-18T14:30:00Z"}
```

### 4.3 Event Stream 필터링

```http
GET /v1/events/stream?types=service_update,viewer_count&serviceId=660e8400
Authorization: Bearer {token}
```

### 4.4 이벤트 재생

```http
GET /v1/events/stream?lastEventId=evt_1234567890
Authorization: Bearer {token}
```

**서버 응답:**
```
: 연결 설정됨
id: evt_1234567890
retry: 10000

event: service_update
id: evt_1234567891
data: {"serviceId":"660e8400","status":"in-progress"}

event: viewer_count
id: evt_1234567892
data: {"count":87}
```

---

## 5. 실시간 스트리밍 Protocol

### 5.1 WebRTC Signaling

```json
{
  "type": "offer",
  "payload": {
    "sessionId": "session_abc123",
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "sdp": "v=0\r\no=- 1234567890 1234567890 IN IP4 127.0.0.1\r\n...",
    "ice_candidates": [
      {
        "candidate": "candidate:1 1 UDP 2130706431 192.168.1.100 54321 typ host",
        "sdpMid": "0",
        "sdpMLineIndex": 0
      }
    ]
  },
  "timestamp": "2025-12-18T14:00:00Z"
}
```

### 5.2 스트림 품질 수준

| 품질 | 해상도 | Bitrate | Frame Rate | 사용 사례 |
|-----|-------|---------|------------|----------|
| low | 640x360 | 500 Kbps | 24 fps | 모바일, 느린 연결 |
| medium | 1280x720 | 2 Mbps | 30 fps | 표준 시청 |
| high | 1920x1080 | 5 Mbps | 30 fps | 데스크톱, 빠른 연결 |
| ultra | 3840x2160 | 15 Mbps | 60 fps | 프리미엄 시청 |

### 5.3 적응형 Bitrate Streaming

```json
{
  "type": "stream_quality_change",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "previousQuality": "high",
    "newQuality": "medium",
    "reason": "bandwidth_reduction",
    "detectedBandwidth": 1500000,
    "requiredBandwidth": 2000000
  },
  "timestamp": "2025-12-18T14:15:00Z"
}
```

### 5.4 스트림 상태 모니터링

```json
{
  "type": "stream_health",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "metrics": {
      "bitrate": 2100000,
      "framerate": 29.97,
      "packetLoss": 0.02,
      "latency": 150,
      "bufferHealth": 95,
      "qualityScore": 8.7
    },
    "issues": []
  },
  "timestamp": "2025-12-18T14:20:00Z"
}
```

### 5.5 HLS/DASH Manifest

**HLS Master Playlist (m3u8):**
```
#EXTM3U
#EXT-X-VERSION:6
#EXT-X-STREAM-INF:BANDWIDTH=500000,RESOLUTION=640x360,FRAME-RATE=24.000
low/index.m3u8
#EXT-X-STREAM-INF:BANDWIDTH=2000000,RESOLUTION=1280x720,FRAME-RATE=30.000
medium/index.m3u8
#EXT-X-STREAM-INF:BANDWIDTH=5000000,RESOLUTION=1920x1080,FRAME-RATE=30.000
high/index.m3u8
```

**Media Playlist:**
```
#EXTM3U
#EXT-X-VERSION:6
#EXT-X-TARGETDURATION:6
#EXT-X-MEDIA-SEQUENCE:0
#EXTINF:6.0,
segment_0.ts
#EXTINF:6.0,
segment_1.ts
#EXTINF:6.0,
segment_2.ts
```

---

## 6. 채팅 및 상호작용 Protocol

### 6.1 채팅 메시지

```json
{
  "type": "chat",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "author": {
      "id": "user_12345",
      "name": "김민수",
      "relationship": "friend"
    },
    "message": "존슨 박사님의 아름다운 추억을 공유해 주셔서 감사합니다.",
    "timestamp": "2025-12-18T14:25:00Z",
    "metadata": {
      "clientId": "client-12345",
      "userAgent": "Mozilla/5.0..."
    }
  },
  "messageId": "msg_chat_1234567890"
}
```

**서버 브로드캐스트:**
```json
{
  "type": "chat",
  "payload": {
    "id": "chat_msg_abc123",
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "author": {
      "name": "김민수",
      "relationship": "friend"
    },
    "message": "존슨 박사님의 아름다운 추억을 공유해 주셔서 감사합니다.",
    "timestamp": "2025-12-18T14:25:00Z",
    "moderationStatus": "approved"
  },
  "messageId": "msg_chat_1234567891"
}
```

### 6.2 반응 시스템

```json
{
  "type": "reaction",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "targetType": "service",
    "targetId": "660e8400-e29b-41d4-a716-446655440001",
    "reactionType": "heart",
    "userId": "user_12345"
  },
  "messageId": "msg_reaction_1234567892"
}
```

**사용 가능한 반응:**
| 반응 | Unicode | 의미 |
|-----|---------|------|
| heart | ❤️ | 사랑, 지지 |
| pray | 🙏 | 기도, 감사 |
| dove | 🕊️ | 평화 |
| candle | 🕯️ | 추모 |
| rose | 🌹 | 헌사 |
| hug | 🤗 | 위로, 동정 |

**반응 집계:**
```json
{
  "type": "reaction_summary",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "reactions": {
      "heart": 156,
      "pray": 89,
      "dove": 45,
      "candle": 123,
      "rose": 67,
      "hug": 78
    },
    "totalReactions": 558
  },
  "timestamp": "2025-12-18T14:30:00Z"
}
```

### 6.3 채팅 검토

```json
{
  "type": "chat_moderate",
  "payload": {
    "messageId": "chat_msg_abc123",
    "action": "approve",
    "moderatorId": "moderator_12345",
    "reason": "적절한 콘텐츠"
  },
  "messageId": "msg_moderate_1234567893"
}
```

**검토 작업:**
| 작업 | 효과 | 알림 |
|-----|------|------|
| approve | 모두에게 메시지 표시 | 없음 |
| hide | 보기에서 메시지 숨김 | 검토자만 |
| delete | 메시지 영구 삭제 | 작성자에게 알림 |
| flag | 검토 표시 | 검토 팀 |
| timeout_user | 사용자를 일시적으로 음소거 | 사용자에게 알림 |
| ban_user | 사용자를 영구적으로 차단 | 사용자에게 알림 |

### 6.4 비공개 메시지

```json
{
  "type": "private_message",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "recipientId": "user_67890",
    "message": "존슨 박사님에 대해 몇 말씀 나누시겠습니까?",
    "senderName": "Sarah Martinez"
  },
  "messageId": "msg_private_1234567894"
}
```

---

## 7. 동기화 Protocol

### 7.1 상태 동기화

```json
{
  "type": "sync_request",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "lastSyncTimestamp": "2025-12-18T14:00:00Z",
    "clientState": {
      "currentSegment": "eulogy",
      "chatScrollPosition": 150,
      "reactions": {"heart": 120}
    }
  },
  "messageId": "msg_sync_1234567895"
}
```

**서버 응답:**
```json
{
  "type": "sync_response",
  "payload": {
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "serverState": {
      "status": "in-progress",
      "currentSegment": "video-tribute",
      "segmentStartedAt": "2025-12-18T14:25:00Z",
      "viewerCount": 87,
      "reactions": {"heart": 156, "pray": 89, "candle": 123}
    },
    "updates": [
      {
        "type": "segment_change",
        "from": "eulogy",
        "to": "video-tribute",
        "timestamp": "2025-12-18T14:25:00Z"
      }
    ],
    "syncTimestamp": "2025-12-18T14:30:00Z"
  },
  "messageId": "msg_sync_1234567896"
}
```

### 7.2 충돌 해결

| 충돌 유형 | 해결 전략 | 우선순위 |
|---------|---------|---------|
| 동시 편집 | Last-write-wins | 서버 타임스탬프 |
| 채팅 메시지 순서 | 서버 할당 시퀀스 | 서버 시퀀스 |
| 반응 수 | 서버 집계 | 서버 합계 |
| 시청자 수 | 서버 권한 | 서버만 |
| 스트림 상태 | 서버 source-of-truth | 서버 상태 |

### 7.3 오프라인 지원

```json
{
  "type": "offline_sync",
  "payload": {
    "clientId": "client-12345",
    "offlineSince": "2025-12-18T14:20:00Z",
    "onlineAt": "2025-12-18T14:35:00Z",
    "queuedActions": [
      {
        "type": "chat",
        "payload": {...},
        "timestamp": "2025-12-18T14:22:00Z"
      },
      {
        "type": "reaction",
        "payload": {...},
        "timestamp": "2025-12-18T14:28:00Z"
      }
    ]
  },
  "messageId": "msg_offline_1234567897"
}
```

---

## 8. 보안 및 암호화

### 8.1 전송 계층 보안

| Protocol | 암호화 | 인증서 요구사항 |
|----------|-------|-------------|
| WebSocket | WSS (TLS 1.3) | 유효한 SSL 인증서 |
| HTTPS | TLS 1.3 | 유효한 SSL 인증서 |
| WebRTC | DTLS-SRTP | 자체 서명 허용 |
| MQTT | TLS 1.3 | 유효한 SSL 인증서 |

### 8.2 비공개 메시지용 End-to-End 암호화

```json
{
  "type": "encrypted_message",
  "payload": {
    "recipientId": "user_67890",
    "encryptedContent": "AES256:iv:ciphertext",
    "algorithm": "AES-256-GCM",
    "keyId": "key_abc123"
  },
  "messageId": "msg_encrypted_1234567898"
}
```

### 8.3 액세스 제어

```json
{
  "type": "access_check",
  "payload": {
    "resourceType": "service",
    "resourceId": "660e8400-e29b-41d4-a716-446655440001",
    "requestedPermission": "stream.view",
    "userId": "user_12345"
  },
  "messageId": "msg_access_1234567899"
}
```

**서버 응답:**
```json
{
  "type": "access_result",
  "payload": {
    "allowed": true,
    "permissions": ["stream.view", "chat.send", "react.send"],
    "expiresAt": "2025-12-18T18:00:00Z"
  },
  "messageId": "msg_access_1234567900"
}
```

### 8.4 요청 제한

| 작업 | 제한 | 시간 창 | 페널티 |
|-----|-----|--------|-------|
| 채팅 메시지 | 10개 메시지 | 1분 | 5분 음소거 |
| 반응 | 30개 반응 | 1분 | 1분 쿨다운 |
| API 요청 | 100개 요청 | 1분 | 429 오류 |
| WebSocket 메시지 | 60개 메시지 | 1분 | 연결 조절 |

---

## 9. 메시지 형식 명세

### 9.1 기본 메시지 구조

```json
{
  "type": "string",
  "payload": {},
  "messageId": "msg_unique_identifier",
  "timestamp": "2025-12-18T14:00:00Z",
  "version": "1.0.0",
  "metadata": {
    "clientId": "client-12345",
    "requestId": "req_1234567890"
  }
}
```

### 9.2 메시지 확인

```json
{
  "type": "ack",
  "payload": {
    "acknowledgedMessageId": "msg_1234567890",
    "status": "received"
  },
  "messageId": "msg_ack_1234567891",
  "timestamp": "2025-12-18T14:00:01Z"
}
```

### 9.3 오류 메시지

```json
{
  "type": "error",
  "payload": {
    "code": "PERMISSION_DENIED",
    "message": "채팅 메시지를 보낼 권한이 없습니다",
    "originalMessageId": "msg_1234567890",
    "retryable": false
  },
  "messageId": "msg_error_1234567892",
  "timestamp": "2025-12-18T14:00:02Z"
}
```

### 9.4 일괄 메시지

```json
{
  "type": "batch",
  "payload": {
    "messages": [
      {
        "type": "viewer_count",
        "payload": {"count": 87}
      },
      {
        "type": "reaction_summary",
        "payload": {"reactions": {"heart": 156}}
      },
      {
        "type": "stream_health",
        "payload": {"metrics": {...}}
      }
    ]
  },
  "messageId": "msg_batch_1234567893",
  "timestamp": "2025-12-18T14:00:03Z"
}
```

---

## 10. Protocol 상태 관리

### 10.1 연결 상태

```
┌──────────┐
│  CLOSED  │
└────┬─────┘
     │ connect()
     ▼
┌──────────────┐
│ CONNECTING   │
└────┬─────────┘
     │ auth_success
     ▼
┌──────────────┐
│  CONNECTED   │◄──┐
└────┬─────────┘   │
     │             │ reconnect
     │ subscribe   │
     ▼             │
┌──────────────┐   │
│ SUBSCRIBED   │───┘
└────┬─────────┘
     │ error / disconnect
     ▼
┌──────────────┐
│ RECONNECTING │
└────┬─────────┘
     │ max_retries
     ▼
┌──────────────┐
│   CLOSED     │
└──────────────┘
```

### 10.2 서비스 상태

| 상태 | 설명 | 허용된 전환 |
|-----|------|-----------|
| planned | 서비스 예약됨 | → confirmed, cancelled |
| confirmed | 서비스 확정됨 | → in-progress, postponed, cancelled |
| in-progress | 서비스 진행 중 | → completed, interrupted |
| completed | 서비스 완료됨 | 없음 |
| interrupted | 서비스 일시 중단됨 | → in-progress, cancelled |
| postponed | 서비스 연기됨 | → confirmed, cancelled |
| cancelled | 서비스 취소됨 | 없음 |

### 10.3 스트림 상태

| 상태 | 설명 | 소요 시간 |
|-----|------|---------|
| idle | 스트림 시작 안 됨 | 무기한 |
| preparing | 스트림 초기화 중 | 30-60초 |
| live | 스트림 활성 | 서비스 소요 시간 |
| paused | 일시 중지됨 | 최대 5분 |
| ended | 스트림 종료됨 | 해당 없음 |
| error | 스트림 실패 | 해당 없음 |

---

## 11. 구현 예제

### 11.1 완전한 WebSocket 클라이언트

```javascript
class FuneralServiceWebSocket {
  constructor(serviceId, token) {
    this.serviceId = serviceId;
    this.token = token;
    this.ws = null;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 10;
    this.heartbeatInterval = null;
  }

  connect() {
    this.ws = new WebSocket('wss://ws.funeral-service.example.com/v1');

    this.ws.onopen = () => {
      console.log('WebSocket 연결됨');
      this.authenticate();
      this.startHeartbeat();
      this.reconnectAttempts = 0;
    };

    this.ws.onmessage = (event) => {
      const message = JSON.parse(event.data);
      this.handleMessage(message);
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket 오류:', error);
    };

    this.ws.onclose = () => {
      console.log('WebSocket 종료됨');
      this.stopHeartbeat();
      this.reconnect();
    };
  }

  authenticate() {
    this.send({
      type: 'auth',
      payload: {
        token: this.token,
        clientId: this.generateClientId(),
        userAgent: navigator.userAgent
      }
    });
  }

  subscribe() {
    this.send({
      type: 'subscribe',
      payload: {
        channels: [
          `service:${this.serviceId}`,
          `chat:${this.serviceId}`,
          `reactions:${this.serviceId}`
        ]
      }
    });
  }

  sendChatMessage(message) {
    this.send({
      type: 'chat',
      payload: {
        serviceId: this.serviceId,
        message: message,
        timestamp: new Date().toISOString()
      }
    });
  }

  sendReaction(reactionType) {
    this.send({
      type: 'reaction',
      payload: {
        serviceId: this.serviceId,
        targetType: 'service',
        targetId: this.serviceId,
        reactionType: reactionType
      }
    });
  }

  handleMessage(message) {
    switch (message.type) {
      case 'auth_success':
        console.log('인증 성공');
        this.subscribe();
        break;

      case 'subscribed':
        console.log('채널 구독됨');
        break;

      case 'chat':
        this.onChatMessage(message.payload);
        break;

      case 'reaction_summary':
        this.onReactionUpdate(message.payload);
        break;

      case 'viewer_count':
        this.onViewerCountUpdate(message.payload);
        break;

      case 'service_update':
        this.onServiceUpdate(message.payload);
        break;

      case 'pong':
        // Heartbeat 확인됨
        break;

      case 'error':
        console.error('서버 오류:', message.payload);
        break;
    }
  }

  send(data) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      data.messageId = this.generateMessageId();
      data.timestamp = new Date().toISOString();
      this.ws.send(JSON.stringify(data));
    }
  }

  startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      this.send({ type: 'ping' });
    }, 30000); // 30초
  }

  stopHeartbeat() {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }
  }

  reconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 30000);
      console.log(`${delay}ms 후 재연결 (시도 ${this.reconnectAttempts})`);

      setTimeout(() => {
        this.connect();
      }, delay);
    } else {
      console.error('최대 재연결 시도 횟수 도달');
    }
  }

  generateClientId() {
    return 'client-' + Math.random().toString(36).substr(2, 9);
  }

  generateMessageId() {
    return 'msg_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
  }

  // 이벤트 핸들러 (사용자가 구현)
  onChatMessage(data) {}
  onReactionUpdate(data) {}
  onViewerCountUpdate(data) {}
  onServiceUpdate(data) {}
}

// 사용법
const wsClient = new FuneralServiceWebSocket(
  '660e8400-e29b-41d4-a716-446655440001',
  'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
);

wsClient.onChatMessage = (data) => {
  console.log(`${data.author.name}: ${data.message}`);
  // 채팅 메시지로 UI 업데이트
};

wsClient.onViewerCountUpdate = (data) => {
  console.log(`현재 시청자: ${data.count}`);
  // 시청자 수 표시 업데이트
};

wsClient.connect();
```

---

## 결론

이 Phase 3 명세서는 디지털 장례 서비스를 위한 포괄적인 실시간 통신 protocol을 정의합니다. Protocol은 원활한 라이브 스트리밍, 대화형 채팅, 이벤트 알림 및 데이터 동기화를 가능하게 하여 매력적이고 의미 있는 가상 장례 경험을 만듭니다.

**다음 단계:** Phase 4는 프로덕션 시스템에서 이러한 protocol을 구현하기 위한 통합 패턴 및 모범 사례를 정의합니다.

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
