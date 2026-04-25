# WIA Cryo-Legal 표준 - 3단계: 통신 프로토콜

**버전**: 1.0.0
**상태**: 초안
**날짜**: 2025-01
**작성자**: WIA 표준 위원회
**라이선스**: MIT
**주요 색상**: #06B6D4 (Cyan)

---

## 1. 개요

### 1.1 목적

이 문서는 WIA Cryo-Legal 표준의 통신 프로토콜을 정의하며, 분산 시스템 전반에 걸쳐 법적 문서, 서명 및 규정 준수 상태의 안전한 실시간 동기화를 가능하게 합니다.

### 1.2 프로토콜 선택 근거

| 프로토콜 | 사용 사례 | 정당성 |
|----------|----------|--------|
| WebSocket | 실시간 업데이트 | 양방향, 낮은 지연 시간 |
| HTTPS | API 호출 | 표준, 안전, 캐시 가능 |
| gRPC | 서비스 간 통신 | 고성능, 타입 지정 |

### 1.3 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    클라이언트 애플리케이션                    │
│           (웹, 모바일, 데스크톱, 시설 시스템)                │
└─────────────────────┬───────────────────────────────────────┘
                      │ WebSocket / HTTPS
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                    API 게이트웨이                            │
│              (인증, 요청 제한)                              │
└─────────────────────┬───────────────────────────────────────┘
                      │ gRPC
                      ▼
┌─────────────────────────────────────────────────────────────┐
│              Cryo-Legal 서비스 클러스터                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │  문서      │  │  서명      │  │ 규정 준수   │        │
│  │  서비스    │  │  서비스    │  │  서비스    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 전송 계층

### 2.1 WebSocket 연결

**엔드포인트:**
```
wss://ws.wia.live/cryo-legal/v1
```

**연결 매개변수:**
```
wss://ws.wia.live/cryo-legal/v1?token={jwt_token}&client_id={client_id}
```

### 2.2 연결 핸드셰이크

```
클라이언트                                  서버
   │                                        │
   │ ────── WebSocket 업그레이드 요청 ────► │
   │         (JWT 토큰 포함)                │
   │                                        │
   │ ◄───── 101 Switching Protocols ─────── │
   │                                        │
   │ ────────── auth 메시지 ─────────────► │
   │                                        │
   │ ◄──────── auth_ack 메시지 ────────── │
   │                                        │
   │ ────────── subscribe ────────────────► │
   │                                        │
   │ ◄──────── subscribe_ack ───────────── │
   │                                        │
   │ ◄─────── 실시간 이벤트 ──────────────  │
   │                                        │
```

### 2.3 TLS 요구사항

| 요구사항 | 값 |
|----------|-----|
| TLS 버전 | 1.3 (최소 1.2) |
| 암호화 스위트 | TLS_AES_256_GCM_SHA384, TLS_CHACHA20_POLY1305_SHA256 |
| 인증서 | 유효한 CA 서명 인증서 |
| HSTS | 활성화, max-age=31536000 |

---

## 3. 메시지 형식

### 3.1 기본 메시지 구조

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "msg-uuid-v4",
  "timestamp": 1705312200000,
  "type": "message_type",
  "payload": {}
}
```

### 3.2 메시지 유형

| 유형 | 방향 | 설명 |
|------|------|------|
| `auth` | C → S | 인증 요청 |
| `auth_ack` | S → C | 인증 응답 |
| `subscribe` | C → S | 이벤트 구독 |
| `subscribe_ack` | S → C | 구독 확인 |
| `unsubscribe` | C → S | 구독 취소 |
| `document.created` | S → C | 새 문서 이벤트 |
| `document.updated` | S → C | 문서 수정됨 |
| `document.signed` | S → C | 서명 추가됨 |
| `signature.request` | S → C | 서명 요청됨 |
| `compliance.alert` | S → C | 규정 준수 문제 |
| `ping` | C → S | 연결 유지 |
| `pong` | S → C | 연결 유지 응답 |
| `error` | S → C | 오류 알림 |

### 3.3 JSON 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-legal/protocol/v1/message.schema.json",
  "type": "object",
  "required": ["protocol", "version", "messageId", "timestamp", "type", "payload"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-cryo-legal"
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
      "description": "밀리초 단위 Unix 타임스탬프"
    },
    "type": {
      "type": "string",
      "enum": [
        "auth", "auth_ack",
        "subscribe", "subscribe_ack", "unsubscribe",
        "document.created", "document.updated", "document.signed",
        "signature.request", "compliance.alert",
        "ping", "pong", "error"
      ]
    },
    "payload": {
      "type": "object"
    }
  }
}
```

---

## 4. 연결 생명주기

### 4.1 인증 메시지

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1705312200000,
  "type": "auth",
  "payload": {
    "token": "eyJhbGciOiJSUzI1NiIs...",
    "clientId": "facility-001",
    "clientType": "facility_system",
    "capabilities": ["documents", "signatures", "compliance"]
  }
}
```

### 4.2 인증 응답

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440002",
  "timestamp": 1705312200050,
  "type": "auth_ack",
  "payload": {
    "success": true,
    "sessionId": "session-abc123",
    "expiresAt": 1705398600000,
    "permissions": ["documents:read", "documents:write", "signatures:create"],
    "serverInfo": {
      "version": "1.0.0",
      "region": "ap-northeast-2"
    }
  }
}
```

### 4.3 구독

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440003",
  "timestamp": 1705312200100,
  "type": "subscribe",
  "payload": {
    "channels": [
      {
        "channel": "documents",
        "filter": {
          "partyId": "party-uuid-123",
          "documentTypes": ["cryopreservation_contract", "consent_form"]
        }
      },
      {
        "channel": "signatures",
        "filter": {
          "pendingOnly": true
        }
      },
      {
        "channel": "compliance",
        "filter": {
          "jurisdictions": ["KR", "US"]
        }
      }
    ]
  }
}
```

### 4.4 연결 유지

**Ping (클라이언트):**
```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440010",
  "timestamp": 1705312230000,
  "type": "ping",
  "payload": {
    "sequence": 1
  }
}
```

**Pong (서버):**
```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440011",
  "timestamp": 1705312230005,
  "type": "pong",
  "payload": {
    "sequence": 1,
    "latency": 5
  }
}
```

### 4.5 재연결 정책

| 시도 | 지연 | 최대 지연 |
|------|------|----------|
| 1 | 1초 | - |
| 2 | 2초 | - |
| 3 | 4초 | - |
| 4 | 8초 | - |
| 5+ | 16초 | 30초 |

```javascript
const delay = Math.min(Math.pow(2, attempt) * 1000, 30000);
```

---

## 5. 이벤트 메시지

### 5.1 문서 생성됨

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440020",
  "timestamp": 1705312300000,
  "type": "document.created",
  "payload": {
    "documentId": "doc-uuid-123",
    "documentType": "cryopreservation_contract",
    "status": "draft",
    "createdBy": "user-123",
    "parties": [
      {
        "partyId": "party-uuid-1",
        "role": "subject",
        "legalName": "홍길동"
      }
    ],
    "jurisdiction": {
      "primaryCountry": "KR",
      "governingLaw": "대한민국 민법"
    },
    "requiredActions": [
      {
        "action": "sign",
        "partyId": "party-uuid-1",
        "deadline": "2025-02-15T00:00:00Z"
      }
    ]
  }
}
```

### 5.2 문서 서명됨

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440021",
  "timestamp": 1705312400000,
  "type": "document.signed",
  "payload": {
    "documentId": "doc-uuid-123",
    "signatureId": "sig-uuid-456",
    "signerId": "party-uuid-1",
    "signerName": "홍길동",
    "signedAt": "2025-01-15T14:30:00Z",
    "documentStatus": "pending",
    "signatureProgress": {
      "completed": 1,
      "required": 2
    },
    "nextSigner": {
      "partyId": "party-uuid-2",
      "role": "facility",
      "legalName": "크라이오라이프 코리아"
    }
  }
}
```

### 5.3 서명 요청

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440022",
  "timestamp": 1705312500000,
  "type": "signature.request",
  "payload": {
    "requestId": "req-uuid-789",
    "documentId": "doc-uuid-123",
    "documentType": "cryopreservation_contract",
    "documentTitle": "냉동보존 서비스 계약서",
    "requestedSigner": {
      "partyId": "party-uuid-2",
      "role": "facility"
    },
    "requestedBy": {
      "partyId": "party-uuid-1",
      "legalName": "홍길동"
    },
    "urgency": "normal",
    "deadline": "2025-02-01T00:00:00Z",
    "signUrl": "https://sign.wia.live/doc-uuid-123?token=xyz"
  }
}
```

### 5.4 규정 준수 알림

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440023",
  "timestamp": 1705312600000,
  "type": "compliance.alert",
  "payload": {
    "alertId": "alert-uuid-101",
    "severity": "warning",
    "documentId": "doc-uuid-123",
    "jurisdiction": "KR",
    "issue": {
      "code": "WITNESS_COUNT",
      "message": "한국법은 사전의료지시서에 2명의 증인을 요구합니다",
      "currentValue": 1,
      "requiredValue": 2
    },
    "remediation": {
      "action": "add_witness",
      "deadline": "2025-02-01T00:00:00Z",
      "instructions": "한국 요구사항을 준수하려면 증인 1명을 추가하세요"
    }
  }
}
```

---

## 6. 보안

### 6.1 메시지 암호화

모든 WebSocket 메시지는 TLS로 암호화됩니다. 민감한 페이로드에 대한 추가 보안:

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1705312200000,
  "type": "document.signed",
  "encrypted": true,
  "payload": {
    "algorithm": "AES-256-GCM",
    "iv": "base64-iv",
    "data": "base64-encrypted-payload",
    "tag": "base64-auth-tag"
  }
}
```

### 6.2 메시지 서명

중요한 메시지에는 디지털 서명이 포함됩니다:

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "...",
  "timestamp": 1705312200000,
  "type": "document.signed",
  "payload": {...},
  "signature": {
    "algorithm": "RS256",
    "keyId": "server-key-001",
    "value": "base64-signature"
  }
}
```

### 6.3 서명 검증

```python
import jwt
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding

def verify_message_signature(message: dict, public_key) -> bool:
    signature_info = message.get('signature')
    if not signature_info:
        return False

    # 정규 페이로드 생성
    payload = json.dumps({
        k: v for k, v in message.items()
        if k != 'signature'
    }, sort_keys=True, separators=(',', ':'))

    signature = base64.b64decode(signature_info['value'])

    try:
        public_key.verify(
            signature,
            payload.encode(),
            padding.PKCS1v15(),
            hashes.SHA256()
        )
        return True
    except Exception:
        return False
```

---

## 7. 오류 처리

### 7.1 오류 메시지 형식

```json
{
  "protocol": "wia-cryo-legal",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440099",
  "timestamp": 1705312200000,
  "type": "error",
  "payload": {
    "code": 4001,
    "name": "SUBSCRIPTION_FAILED",
    "message": "잘못된 채널이 지정되었습니다",
    "details": {
      "channel": "invalid_channel",
      "validChannels": ["documents", "signatures", "compliance"]
    },
    "recoverable": true,
    "relatedMessageId": "550e8400-e29b-41d4-a716-446655440003"
  }
}
```

### 7.2 오류 코드

| 코드 | 이름 | 설명 | 복구 가능 |
|------|------|------|----------|
| 1001 | `AUTH_FAILED` | 인증 실패 | 아니오 |
| 1002 | `AUTH_EXPIRED` | 토큰 만료 | 예 (갱신) |
| 1003 | `PERMISSION_DENIED` | 권한 부족 | 아니오 |
| 2001 | `DOCUMENT_NOT_FOUND` | 문서가 존재하지 않음 | 아니오 |
| 2002 | `DOCUMENT_LOCKED` | 문서가 잠김 | 나중에 재시도 |
| 3001 | `SIGNATURE_INVALID` | 잘못된 서명 | 아니오 |
| 3002 | `CERTIFICATE_EXPIRED` | 인증서 만료 | 예 (갱신) |
| 4001 | `SUBSCRIPTION_FAILED` | 잘못된 구독 | 예 (매개변수 수정) |
| 4002 | `CHANNEL_UNAVAILABLE` | 채널 사용 불가 | 나중에 재시도 |
| 5001 | `RATE_LIMITED` | 너무 많은 메시지 | 예 (백오프) |
| 5002 | `SERVER_ERROR` | 내부 오류 | 나중에 재시도 |

### 7.3 복구 절차

```typescript
async function handleError(error: ProtocolError): Promise<void> {
  switch (error.code) {
    case 1002: // AUTH_EXPIRED
      await refreshToken();
      await reconnect();
      break;

    case 2002: // DOCUMENT_LOCKED
    case 4002: // CHANNEL_UNAVAILABLE
    case 5002: // SERVER_ERROR
      await delay(exponentialBackoff(retryCount));
      await retry();
      break;

    case 5001: // RATE_LIMITED
      await delay(error.details.retryAfter || 60000);
      await retry();
      break;

    default:
      logError(error);
      notifyUser(error);
  }
}
```

---

## 8. 구현 예제

### 8.1 TypeScript 클라이언트

```typescript
import { CryoLegalWebSocket } from '@wia/cryo-legal';

const ws = new CryoLegalWebSocket({
  url: 'wss://ws.wia.live/cryo-legal/v1',
  token: 'jwt-token',
  clientId: 'my-app',
  autoReconnect: true
});

// 연결 및 인증
await ws.connect();

// 채널 구독
await ws.subscribe({
  channels: [
    { channel: 'documents', filter: { partyId: 'my-party-id' } },
    { channel: 'signatures', filter: { pendingOnly: true } }
  ]
});

// 이벤트 처리
ws.on('document.created', (event) => {
  console.log('새 문서:', event.payload.documentId);
});

ws.on('signature.request', (event) => {
  console.log('서명 요청됨:', event.payload.documentTitle);
  showSignatureModal(event.payload);
});

ws.on('compliance.alert', (event) => {
  console.log('규정 준수 문제:', event.payload.issue.message);
  handleComplianceAlert(event.payload);
});

ws.on('error', (error) => {
  console.error('프로토콜 오류:', error);
});

// 정상 종료
await ws.disconnect();
```

### 8.2 Python 클라이언트

```python
import asyncio
from wia_cryo_legal import CryoLegalWebSocket

async def main():
    ws = CryoLegalWebSocket(
        url='wss://ws.wia.live/cryo-legal/v1',
        token='jwt-token',
        client_id='my-app'
    )

    await ws.connect()

    # 구독
    await ws.subscribe(
        channels=[
            {'channel': 'documents', 'filter': {'partyId': 'my-party-id'}},
            {'channel': 'signatures', 'filter': {'pendingOnly': True}}
        ]
    )

    # 이벤트 처리
    @ws.on('document.created')
    async def on_document_created(event):
        print(f"새 문서: {event['payload']['documentId']}")

    @ws.on('signature.request')
    async def on_signature_request(event):
        print(f"서명 요청됨: {event['payload']['documentTitle']}")

    # 취소될 때까지 실행
    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        await ws.disconnect()

asyncio.run(main())
```

---

## 9. 버전 이력

| 버전 | 날짜 | 변경사항 |
|------|------|----------|
| 1.0.0 | 2025-01 | 최초 릴리스 |

---

<div align="center">

**WIA Cryo-Legal 표준 v1.0.0**

3단계: 통신 프로토콜

**弘益人間 (홍익인간)** · 널리 인간을 이롭게

---

© 2025 WIA 표준 위원회

MIT 라이선스

</div>
