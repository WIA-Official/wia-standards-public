# WIA Cryo-Identity 통신 프로토콜
## Phase 3 사양

---

**버전**: 1.0.0
**상태**: Draft
**날짜**: 2025-01
**작성자**: WIA Standards Committee
**라이선스**: MIT
**대표 색상**: #06B6D4 (Cyan)

---

## 목차

1. [개요](#개요)
2. [용어 정의](#용어-정의)
3. [프로토콜 아키텍처](#프로토콜-아키텍처)
4. [메시지 형식](#메시지-형식)
5. [메시지 유형](#메시지-유형)
6. [연결 관리](#연결-관리)
7. [보안](#보안)
8. [전송 계층](#전송-계층)
9. [오류 처리](#오류-처리)
10. [사용 예제](#사용-예제)
11. [참고문헌](#참고문헌)

---

## 개요

### 1.1 목적

WIA Cryo-Identity 통신 프로토콜은 보존 시설, 검증 시스템, 법적 기관, 미래 소생 센터 간의 안전한 신원 데이터 교환을 위한 표준화된 메시지 형식과 통신 패턴을 정의합니다. 이 Phase 3 사양은 데이터 무결성과 진위성에 대한 암호화 보증을 통해 지리적으로 분산된 시스템 간의 상호운용 가능한 신원 관리를 가능하게 합니다.

**핵심 목표**:
- 안전한 P2P 신원 검증 지원
- 분산 신원 저장 및 검색 지원
- 실시간 신원 이벤트 동기화 제공
- 모든 통신의 암호화 무결성 보장
- 시설 간 신원 이식성 지원

### 1.2 적용 범위

본 문서는 다음을 정의합니다:

| 구성요소 | 설명 |
|----------|------|
| Message Format | JSON 기반 프로토콜 메시지 |
| Message Type | 신원 작업 및 이벤트 |
| Connection Management | 세션 생애주기 및 재연결 |
| Security | 암호화, 인증, 권한 부여 |
| Transport Layer | WebSocket, HTTPS, P2P 전송 |

### 1.3 관련 문서

| 문서 | 설명 |
|------|------|
| PHASE-1-DATA-FORMAT.md | 신원 레코드 구조 |
| PHASE-2-API-INTERFACE.md | API 인터페이스 사양 |
| PHASE-4-INTEGRATION.md | 생태계 통합 |

---

## 용어 정의

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **Client** | 신원 관리 시스템 (시설, 법률, 의료) |
| **Server** | 신원 검증 서비스 또는 레지스트리 |
| **Peer** | 분산 신원 네트워크의 동등한 참여자 |
| **Message** | 엔티티 간 교환되는 프로토콜 데이터 단위 |
| **Session** | Client와 Server 간의 인증된 연결 |
| **Channel** | 특정 신원을 위한 논리적 통신 경로 |

### 2.2 메시지 카테고리

| 카테고리 | 설명 |
|----------|------|
| **Identity** | 생성, 읽기, 업데이트, 삭제 작업 |
| **Verification** | 생체인식 및 암호화 검증 |
| **Sync** | 분산 신원 동기화 |
| **Event** | 실시간 신원 생애주기 이벤트 |
| **Control** | 연결 및 세션 관리 |

---

## 프로토콜 아키텍처

### 3.1 계층 아키텍처

```
┌─────────────────────────────────────────────────────────┐
│              Application Layer                           │
│         (Identity Management Systems)                    │
├─────────────────────────────────────────────────────────┤
│              Protocol Layer                              │
│    (Message Format, Handler, Validator)                  │
├─────────────────────────────────────────────────────────┤
│              Security Layer                              │
│         (TLS, Encryption, Signature)                     │
├─────────────────────────────────────────────────────────┤
│              Transport Layer                             │
│    (WebSocket / HTTPS / IPFS / Blockchain)               │
└─────────────────────────────────────────────────────────┘
```

### 3.2 통신 패턴

| 패턴 | 사용 사례 | 예시 |
|------|-----------|------|
| Request-Response | 신원 쿼리 | ID로 신원 가져오기 |
| Publish-Subscribe | 이벤트 알림 | 신원 상태 변경 |
| Peer-to-Peer | 분산 검증 | 시설 간 검증 |
| Streaming | 생체 데이터 캡처 | 실시간 지문 스트림 |

### 3.3 네트워크 토폴로지

```
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│  Facility A  │◄───────►│Central Registry│◄──────►│  Facility B  │
│  (Client)    │         │   (Server)    │         │  (Client)    │
└──────────────┘         └──────────────┘         └──────────────┘
       │                        │                         │
       │                        │                         │
       ▼                        ▼                         ▼
┌──────────────┐         ┌──────────────┐         ┌──────────────┐
│  Blockchain  │         │  IPFS Node   │         │Legal Archive │
│   Anchor     │         │  (Storage)   │         │  (Backup)    │
└──────────────┘         └──────────────┘         └──────────────┘
```

---

## 메시지 형식

### 4.1 기본 메시지 구조

모든 프로토콜 메시지는 다음 구조를 따릅니다:

```json
{
  "protocol": "wia-cryo-identity",
  "version": "1.0.0",
  "messageId": "uuid-v4-string",
  "timestamp": 1702483200000,
  "type": "message_type",
  "sender": {
    "entityId": "facility-001",
    "entityType": "preservation_facility",
    "publicKey": "ed25519:abc123..."
  },
  "payload": {},
  "signature": "ed25519:signature..."
}
```

### 4.2 필드 정의

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `protocol` | string | Y | 프로토콜 식별자 ("wia-cryo-identity") |
| `version` | string | Y | 프로토콜 버전 (SemVer) |
| `messageId` | string | Y | 고유 메시지 ID (UUID v4) |
| `timestamp` | number | Y | Unix 타임스탬프 (밀리초) |
| `type` | string | Y | 메시지 유형 식별자 |
| `sender` | object | Y | 발신자 식별 |
| `payload` | object | Y | 메시지별 데이터 |
| `signature` | string | Y | 메시지의 Ed25519 서명 |

### 4.3 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-identity/protocol/v1/message.schema.json",
  "title": "WIA Cryo-Identity Protocol Message",
  "type": "object",
  "required": ["protocol", "version", "messageId", "timestamp", "type", "sender", "payload", "signature"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-cryo-identity",
      "description": "프로토콜 식별자"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "프로토콜 버전"
    },
    "messageId": {
      "type": "string",
      "format": "uuid",
      "description": "고유 메시지 ID"
    },
    "timestamp": {
      "type": "integer",
      "minimum": 0,
      "description": "Unix 타임스탬프 (밀리초)"
    },
    "type": {
      "type": "string",
      "enum": [
        "connect", "connect_ack", "disconnect",
        "identity_create", "identity_create_ack",
        "identity_get", "identity_get_ack",
        "identity_update", "identity_update_ack",
        "identity_delete", "identity_delete_ack",
        "verify_biometric", "verify_biometric_ack",
        "anchor_blockchain", "anchor_blockchain_ack",
        "subscribe", "subscribe_ack", "unsubscribe",
        "event", "error", "ping", "pong"
      ],
      "description": "메시지 유형"
    },
    "sender": {
      "type": "object",
      "required": ["entityId", "entityType", "publicKey"],
      "description": "발신자 정보",
      "properties": {
        "entityId": { "type": "string", "description": "엔티티 ID" },
        "entityType": { "type": "string", "description": "엔티티 유형" },
        "publicKey": { "type": "string", "description": "공개 키" }
      }
    },
    "payload": {
      "type": "object",
      "description": "메시지 데이터"
    },
    "signature": {
      "type": "string",
      "description": "Ed25519 서명"
    }
  }
}
```

---

## 메시지 유형

### 5.1 연결 메시지

#### 5.1.1 connect

인증된 연결을 수립합니다.

```json
{
  "protocol": "wia-cryo-identity",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702483200000,
  "type": "connect",
  "sender": {
    "entityId": "facility-001",
    "entityType": "preservation_facility",
    "publicKey": "ed25519:abc123..."
  },
  "payload": {
    "capabilities": ["identity_create", "biometric_verify", "blockchain_anchor"],
    "supportedBiometrics": ["fingerprint", "dna", "facial", "retinal"],
    "blockchainNetworks": ["ethereum", "polygon"],
    "maxMessageSize": 10485760,
    "compressionSupported": true
  },
  "signature": "ed25519:sig..."
}
```

#### 5.1.2 connect_ack

연결 확인 응답입니다.

```json
{
  "protocol": "wia-cryo-identity",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1702483200050,
  "type": "connect_ack",
  "sender": {
    "entityId": "registry-central",
    "entityType": "identity_registry",
    "publicKey": "ed25519:def456..."
  },
  "payload": {
    "success": true,
    "sessionId": "session-xyz789",
    "sessionExpiry": 1702569600000,
    "serverCapabilities": ["identity_storage", "verification", "search"],
    "rateLimit": {
      "requestsPerMinute": 100,
      "burstSize": 20
    }
  },
  "signature": "ed25519:sig..."
}
```

### 5.2 신원 메시지

#### 5.2.1 identity_create

새 신원 레코드를 생성합니다.

```json
{
  "protocol": "wia-cryo-identity",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440010",
  "timestamp": 1702483200100,
  "type": "identity_create",
  "sender": {
    "entityId": "facility-001",
    "entityType": "preservation_facility",
    "publicKey": "ed25519:abc123..."
  },
  "payload": {
    "identity": {
      "$schema": "https://wia.live/cryo-identity/v1/schema.json",
      "version": "1.0.0",
      "personal": {
        "legalName": {
          "given": "encrypted:aes256:...",
          "family": "encrypted:aes256:...",
          "hash": "sha256:..."
        },
        "dateOfBirth": "encrypted:aes256:...",
        "nationality": ["US"]
      },
      "biometrics": {
        "fingerprints": [...],
        "dna": {...}
      }
    },
    "encryptionKey": "encrypted-with-server-public-key",
    "requestBlockchainAnchor": true
  },
  "signature": "ed25519:sig..."
}
```

#### 5.2.2 identity_create_ack

신원 생성 확인 응답입니다.

```json
{
  "protocol": "wia-cryo-identity",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440011",
  "timestamp": 1702483200150,
  "type": "identity_create_ack",
  "sender": {
    "entityId": "registry-central",
    "entityType": "identity_registry",
    "publicKey": "ed25519:def456..."
  },
  "payload": {
    "success": true,
    "identityId": "ID-2025-000001",
    "recordHash": "sha256:abc123...",
    "storageLocation": "ipfs://QmXyz...",
    "blockchainAnchor": {
      "network": "ethereum",
      "transactionHash": "0xabc123...",
      "blockNumber": 12345678
    }
  },
  "signature": "ed25519:sig..."
}
```

#### 5.2.3 identity_get

신원 레코드를 조회합니다.

```json
{
  "type": "identity_get",
  "payload": {
    "identityId": "ID-2025-000001",
    "fields": ["personal", "biometrics.fingerprints", "cryptographic"],
    "includeHistory": false,
    "decryptWith": "requestor-public-key"
  }
}
```

#### 5.2.4 identity_update

신원 레코드를 업데이트합니다.

```json
{
  "type": "identity_update",
  "payload": {
    "identityId": "ID-2025-000001",
    "updates": {
      "status": "preserved",
      "biometrics.retinal": {...}
    },
    "updateReason": "preservation_completed",
    "updateAuthorization": "signed-authorization-token"
  }
}
```

#### 5.2.5 identity_delete

신원 레코드를 삭제(보관)합니다.

```json
{
  "type": "identity_delete",
  "payload": {
    "identityId": "ID-2025-000001",
    "deleteReason": "legal_request",
    "deleteAuthorization": "signed-legal-order",
    "archiveLocation": "secure-archive-id",
    "permanentDelete": false
  }
}
```

### 5.3 검증 메시지

#### 5.3.1 verify_biometric

생체 데이터를 검증합니다.

```json
{
  "type": "verify_biometric",
  "payload": {
    "identityId": "ID-2025-000001",
    "biometricType": "fingerprint",
    "biometricData": {
      "template": "base64-encoded-template",
      "quality": 0.95,
      "capturedAt": "2025-01-15T10:00:00Z"
    },
    "requiredConfidence": 0.95,
    "verificationContext": "pre_revival_verification"
  }
}
```

#### 5.3.2 verify_biometric_ack

생체 검증 결과입니다.

```json
{
  "type": "verify_biometric_ack",
  "payload": {
    "success": true,
    "verified": true,
    "confidence": 0.98,
    "matchedBiometric": {
      "type": "fingerprint",
      "finger": "right_index",
      "registeredAt": "2024-06-15T10:00:00Z"
    },
    "verificationId": "verify-abc123",
    "verificationTimestamp": "2025-01-15T10:00:00Z"
  }
}
```

### 5.4 Blockchain 메시지

#### 5.4.1 anchor_blockchain

Blockchain에 신원을 앵커합니다.

```json
{
  "type": "anchor_blockchain",
  "payload": {
    "identityId": "ID-2025-000001",
    "network": "ethereum",
    "dataHash": "sha256:abc123...",
    "contractAddress": "0x...",
    "gasLimit": 100000,
    "priority": "standard"
  }
}
```

#### 5.4.2 anchor_blockchain_ack

Blockchain 앵커 확인입니다.

```json
{
  "type": "anchor_blockchain_ack",
  "payload": {
    "success": true,
    "network": "ethereum",
    "transactionHash": "0xabc123...",
    "blockNumber": 12345678,
    "confirmations": 12,
    "gasUsed": 21000,
    "timestamp": "2025-01-15T10:00:00Z"
  }
}
```

### 5.5 구독 메시지

#### 5.5.1 subscribe

신원 이벤트를 구독합니다.

```json
{
  "type": "subscribe",
  "payload": {
    "events": ["identity:updated", "identity:verified", "blockchain:anchored"],
    "filters": {
      "identityIds": ["ID-2025-000001", "ID-2025-000002"],
      "facilities": ["facility-001"]
    },
    "deliveryMode": "websocket"
  }
}
```

#### 5.5.2 event

이벤트 알림입니다.

```json
{
  "type": "event",
  "payload": {
    "eventType": "identity:verified",
    "eventId": "evt-abc123",
    "identityId": "ID-2025-000001",
    "eventData": {
      "verificationLevel": "cryptographic",
      "confidence": 0.99,
      "verifiedBy": "facility-002"
    },
    "eventTimestamp": "2025-01-15T10:00:00Z"
  }
}
```

### 5.6 제어 메시지

#### 5.6.1 ping/pong

하트비트 메시지입니다.

```json
{
  "type": "ping",
  "payload": {
    "sequence": 1,
    "clientTimestamp": 1702483200000
  }
}
```

```json
{
  "type": "pong",
  "payload": {
    "sequence": 1,
    "serverTimestamp": 1702483200005,
    "latency": 5
  }
}
```

### 5.7 오류 메시지

```json
{
  "type": "error",
  "payload": {
    "code": 2003,
    "name": "BIOMETRIC_MATCH_FAILED",
    "message": "생체 검증 실패 - 신뢰도가 너무 낮음",
    "recoverable": true,
    "details": {
      "confidence": 0.65,
      "required": 0.95,
      "biometricType": "fingerprint"
    },
    "relatedMessageId": "550e8400-e29b-41d4-a716-446655440020"
  }
}
```

---

## 연결 관리

### 6.1 연결 상태

```
                    ┌─────────────────┐
                    │  DISCONNECTED   │
                    └────────┬────────┘
                             │ connect()
                             ▼
                    ┌─────────────────┐
                    │   CONNECTING    │
                    └────────┬────────┘
                             │ connect_ack (성공)
              ┌──────────────┼──────────────┐
              │              ▼              │
              │     ┌─────────────────┐     │
   오류/      │     │   CONNECTED     │     │ connect_ack
   타임아웃   │     └────────┬────────┘     │ (실패)
              │              │              │
              │              │ 오류/        │
              │              │ disconnect   │
              │              ▼              │
              │     ┌─────────────────┐     │
              │     │  RECONNECTING   │     │
              │     └────────┬────────┘     │
              │              │              │
              │   성공       │   최대 재시도│
              │   ┌──────────┘   초과       │
              │   │              │          │
              │   │              ▼          │
              │   │     ┌─────────────────┐ │
              │   │     │     ERROR       │ │
              │   │     └────────┬────────┘ │
              │   │              │          │
              └───┼──────────────┼──────────┘
                  │              │
                  │              ▼
                  │     ┌─────────────────┐
                  └────►│  DISCONNECTED   │
                        └─────────────────┘
```

### 6.2 연결 시퀀스

```
Client                                           Server
  │                                                │
  │ ──────────── TCP/WebSocket 연결 ───────────► │
  │                                                │
  │ ◄────────── 연결 수립됨 ────────────────────  │
  │                                                │
  │ ──────────────── connect ──────────────────► │
  │                                                │
  │ ◄───────────── connect_ack ─────────────────  │
  │                                                │
  │ ──────────────── subscribe ─────────────────► │
  │                                                │
  │ ◄──────────── subscribe_ack ────────────────  │
  │                                                │
  │ ◄────────────── event ──────────────────────  │
  │ ◄────────────── event ──────────────────────  │
  │                  ...                           │
```

### 6.3 세션 관리

| 설정 | 기본값 | 설명 |
|------|--------|------|
| `sessionTimeout` | 3600000ms | 세션 만료 시간 |
| `heartbeatInterval` | 30000ms | Ping 간격 |
| `heartbeatTimeout` | 10000ms | Pong 타임아웃 |
| `maxReconnectAttempts` | 5 | 최대 재연결 시도 |
| `reconnectBackoff` | exponential | 재연결 지연 전략 |

### 6.4 재연결 정책

**지수 백오프**:
```
시도 1: 1000ms
시도 2: 2000ms
시도 3: 4000ms
시도 4: 8000ms
시도 5: 16000ms (최대 30000ms)
```

---

## 보안

### 7.1 전송 보안

| 계층 | 기술 | 목적 |
|------|------|------|
| Transport | TLS 1.3 | 암호화된 채널 |
| Message | Ed25519 | 메시지 서명 |
| Payload | AES-256-GCM | 데이터 암호화 |
| Identity | JWT | 인증 |

### 7.2 인증 플로우

```
Client                                Server
  │                                     │
  │ ─── connect (publicKey) ─────────► │
  │                                     │
  │                                     │ 서명 검증
  │                                     │ 챌린지 생성
  │ ◄── connect_ack (challenge) ──────  │
  │                                     │
  │ 챌린지 서명                          │
  │ ─── auth_response (signature) ───► │
  │                                     │
  │                                     │ 서명 검증
  │                                     │ 세션 생성
  │ ◄── auth_complete (sessionId) ────  │
```

### 7.3 메시지 서명

모든 메시지는 Ed25519를 사용하여 서명해야 합니다:

```typescript
// 메시지 서명 생성
const messageData = JSON.stringify({
  protocol: message.protocol,
  version: message.version,
  messageId: message.messageId,
  timestamp: message.timestamp,
  type: message.type,
  sender: message.sender,
  payload: message.payload
});

const signature = ed25519.sign(messageData, privateKey);
message.signature = `ed25519:${base64url(signature)}`;
```

### 7.4 Payload 암호화

Payload의 민감한 데이터는 암호화되어야 합니다:

```typescript
// 민감한 필드 암호화
const encryptedData = aes256gcm.encrypt(
  JSON.stringify(sensitiveData),
  sharedSecret,
  nonce
);

payload.personal.legalName.given = `encrypted:aes256:${base64url(encryptedData)}`;
```

### 7.5 접근 제어

| 역할 | 권한 |
|------|------|
| `preservation_facility` | 신원 생성, 업데이트, 검증 |
| `legal_representative` | 신원 읽기, 감사 |
| `medical_professional` | 생체 데이터 읽기, 상태 업데이트 |
| `revival_center` | 읽기, 검증, 업데이트 (소생 상태) |
| `registry_admin` | 모든 작업 |

---

## 전송 계층

### 8.1 WebSocket 전송 (주)

**연결 URL**:
```
wss://identity.wia.live/v1/ws
```

**서브프로토콜**: `wia-cryo-identity-v1`

**예시**:
```javascript
const ws = new WebSocket(
  'wss://identity.wia.live/v1/ws',
  'wia-cryo-identity-v1'
);

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  handleMessage(message);
};
```

### 8.2 HTTPS 전송 (보조)

**Base URL**: `https://identity.wia.live/v1/api`

**REST Endpoint**:
```http
POST   /identities
GET    /identities/:id
PATCH  /identities/:id
DELETE /identities/:id
POST   /verify/biometric
POST   /blockchain/anchor
```

### 8.3 IPFS 전송 (저장소)

**목적**: 분산 신원 저장

**형식**:
```
ipfs://QmXyz.../identity-ID-2025-000001.json
```

**메타데이터**:
```json
{
  "identityId": "ID-2025-000001",
  "ipfsHash": "QmXyz...",
  "encryptionKey": "encrypted-with-facility-key",
  "timestamp": "2025-01-15T10:00:00Z"
}
```

### 8.4 Blockchain 전송 (앵커링)

**네트워크**: Ethereum, Polygon, Avalanche

**스마트 계약**:
```solidity
contract CryoIdentityRegistry {
  struct IdentityAnchor {
    bytes32 identityHash;
    uint256 timestamp;
    address facility;
  }

  mapping(bytes32 => IdentityAnchor) public anchors;

  function anchorIdentity(bytes32 identityHash) public {
    anchors[identityHash] = IdentityAnchor({
      identityHash: identityHash,
      timestamp: block.timestamp,
      facility: msg.sender
    });
  }
}
```

---

## 오류 처리

### 9.1 에러 코드

#### 연결 오류 (1xxx)

| 코드 | 이름 | 설명 |
|------|------|------|
| 1000 | `CONNECTION_CLOSED` | 정상 연결 종료 |
| 1001 | `CONNECTION_LOST` | 예상치 못한 연결 끊김 |
| 1002 | `CONNECTION_TIMEOUT` | 연결 타임아웃 |
| 1003 | `AUTHENTICATION_FAILED` | 인증 실패 |
| 1004 | `SESSION_EXPIRED` | 세션 타임아웃 |

#### 신원 오류 (2xxx)

| 코드 | 이름 | 설명 |
|------|------|------|
| 2001 | `IDENTITY_NOT_FOUND` | 신원이 존재하지 않음 |
| 2002 | `IDENTITY_ALREADY_EXISTS` | 중복 신원 |
| 2003 | `BIOMETRIC_MATCH_FAILED` | 검증 실패 |
| 2004 | `INVALID_IDENTITY_DATA` | 데이터 검증 오류 |
| 2005 | `PERMISSION_DENIED` | 권한 부족 |

#### Blockchain 오류 (3xxx)

| 코드 | 이름 | 설명 |
|------|------|------|
| 3001 | `BLOCKCHAIN_UNAVAILABLE` | 네트워크 사용 불가 |
| 3002 | `ANCHOR_FAILED` | 앵커링 실패 |
| 3003 | `TRANSACTION_FAILED` | TX 실패 |
| 3004 | `INSUFFICIENT_FUNDS` | 가스 부족 |

### 9.2 오류 복구 전략

| 오류 유형 | 복구 전략 |
|-----------|-----------|
| `CONNECTION_LOST` | 백오프를 사용한 자동 재연결 |
| `SESSION_EXPIRED` | 재인증 |
| `BIOMETRIC_MATCH_FAILED` | 재캡처 요청 |
| `BLOCKCHAIN_UNAVAILABLE` | 나중에 대기열에 추가 |

---

## 사용 예제

### 10.1 완전한 연결 플로우

```typescript
import { CryoIdentityClient } from 'wia-cryo-identity/protocol';

async function connect() {
  const client = new CryoIdentityClient({
    serverUrl: 'wss://identity.wia.live/v1/ws',
    privateKey: facilityPrivateKey,
    entityId: 'facility-001',
    entityType: 'preservation_facility'
  });

  // 연결
  await client.connect();

  // 이벤트 구독
  await client.subscribe({
    events: ['identity:verified', 'blockchain:anchored']
  });

  // 이벤트 핸들러
  client.on('event', (event) => {
    console.log('이벤트 수신됨:', event.eventType);
  });

  return client;
}
```

### 10.2 프로토콜을 통한 신원 생성

```typescript
async function createIdentity(client: CryoIdentityClient, identityData: any) {
  const message = {
    type: 'identity_create',
    payload: {
      identity: identityData,
      requestBlockchainAnchor: true
    }
  };

  const response = await client.send(message);

  if (response.payload.success) {
    console.log('신원 생성됨:', response.payload.identityId);
    console.log('Blockchain TX:', response.payload.blockchainAnchor.transactionHash);
  }

  return response.payload;
}
```

### 10.3 생체 데이터 검증

```typescript
async function verifyBiometric(
  client: CryoIdentityClient,
  identityId: string,
  biometricData: any
) {
  const message = {
    type: 'verify_biometric',
    payload: {
      identityId,
      biometricType: 'fingerprint',
      biometricData,
      requiredConfidence: 0.95
    }
  };

  const response = await client.send(message);

  if (response.payload.verified) {
    console.log('검증 성공');
    console.log('신뢰도:', response.payload.confidence);
    return true;
  } else {
    console.log('검증 실패');
    return false;
  }
}
```

### 10.4 이벤트 구독

```typescript
async function subscribeToIdentity(
  client: CryoIdentityClient,
  identityId: string
) {
  await client.subscribe({
    events: ['identity:updated', 'identity:verified'],
    filters: { identityIds: [identityId] }
  });

  client.on('event', (event) => {
    if (event.payload.identityId === identityId) {
      console.log(`${identityId}에 대한 이벤트:`, event.payload.eventType);

      switch (event.payload.eventType) {
        case 'identity:updated':
          handleUpdate(event.payload);
          break;
        case 'identity:verified':
          handleVerification(event.payload);
          break;
      }
    }
  });
}
```

### 10.5 Blockchain 검증

```typescript
async function verifyBlockchainAnchor(
  client: CryoIdentityClient,
  identityId: string,
  txHash: string
) {
  // Blockchain에서 신원 가져오기
  const anchor = await client.getBlockchainAnchor(txHash);

  // 해시 일치 확인
  const identity = await client.getIdentity(identityId);
  const computedHash = sha256(JSON.stringify(identity));

  if (anchor.identityHash === computedHash) {
    console.log('Blockchain 앵커 검증됨');
    console.log('블록:', anchor.blockNumber);
    console.log('확인:', anchor.confirmations);
    return true;
  }

  return false;
}
```

---

## 참고문헌

### 프로토콜 표준

- [RFC 6455 - The WebSocket Protocol](https://tools.ietf.org/html/rfc6455)
- [RFC 7519 - JSON Web Token (JWT)](https://tools.ietf.org/html/rfc7519)
- [RFC 8032 - Edwards-Curve Digital Signature Algorithm](https://tools.ietf.org/html/rfc8032)

### 보안 표준

- [NIST SP 800-63 - Digital Identity Guidelines](https://pages.nist.gov/800-63-3/)
- [TLS 1.3 - RFC 8446](https://tools.ietf.org/html/rfc8446)

### 관련 WIA 표준

- [WIA Cryo-Identity 데이터 형식 (Phase 1)](/cryo-identity/spec/PHASE-1-DATA-FORMAT.md)
- [WIA Cryo-Identity API Interface (Phase 2)](/cryo-identity/spec/PHASE-2-API-INTERFACE.md)

---

<div align="center">

**WIA Cryo-Identity 통신 프로토콜 v1.0.0**

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
