# WIA Cryo-Legal 표준 - 2단계: API 인터페이스 명세

**버전**: 1.0.0
**상태**: 초안
**날짜**: 2025-01
**작성자**: WIA 표준 위원회
**라이선스**: MIT
**주요 색상**: #06B6D4 (Cyan)

---

## 1. 개요

### 1.1 목적

WIA Cryo-Legal API는 냉동보존 시스템에서 법적 문서, 동의 검증, 관할권 규정 준수 및 감사 추적을 관리하기 위한 표준 인터페이스를 제공합니다.

### 1.2 설계 원칙

| 원칙 | 설명 |
|------|------|
| RESTful | 리소스 중심 API 설계 |
| 보안 우선 | 종단간 암호화, 감사 로깅 |
| 관할권 인식 | 다중 관할권 지원 |
| 버전 관리 | URL 경로에 API 버전 포함 |
| 멱등성 | 안전한 재시도 작업 |

### 1.3 기본 URL

```
프로덕션: https://api.wia.live/cryo-legal/v1
스테이징: https://staging-api.wia.live/cryo-legal/v1
```

---

## 2. 인증

### 2.1 API 키 인증

```http
Authorization: Bearer {api_key}
X-WIA-Client-ID: {client_id}
```

### 2.2 OAuth 2.0 흐름

```
┌──────────────────────────────────────────────────────────────┐
│                     OAuth 2.0 흐름                            │
├──────────────────────────────────────────────────────────────┤
│  1. 클라이언트가 인가 요청                                    │
│     GET /oauth/authorize?client_id=...&scope=...             │
│                                                               │
│  2. 사용자가 인증 및 접근 허가                                │
│                                                               │
│  3. 인가 서버가 코드 반환                                     │
│     redirect_uri?code={authorization_code}                   │
│                                                               │
│  4. 클라이언트가 코드를 토큰으로 교환                         │
│     POST /oauth/token                                        │
│                                                               │
│  5. API 호출에 액세스 토큰 사용                              │
│     Authorization: Bearer {access_token}                      │
└──────────────────────────────────────────────────────────────┘
```

### 2.3 토큰 요청

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code={authorization_code}
&client_id={client_id}
&client_secret={client_secret}
&redirect_uri={redirect_uri}
```

**응답:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g...",
  "scope": "documents:read documents:write signatures:create"
}
```

### 2.4 스코프

| 스코프 | 설명 |
|--------|------|
| `documents:read` | 문서 메타데이터 및 내용 읽기 |
| `documents:write` | 문서 생성 및 수정 |
| `documents:delete` | 문서 삭제 |
| `signatures:create` | 문서 서명 |
| `signatures:verify` | 서명 검증 |
| `audit:read` | 감사 로그 접근 |
| `parties:manage` | 당사자 정보 관리 |
| `jurisdictions:read` | 관할권 요구사항 조회 |

---

## 3. 엔드포인트

### 3.1 문서

#### 3.1.1 문서 생성

**POST** `/documents`

새 법적 문서를 생성합니다.

**요청 헤더:**
```http
Authorization: Bearer {token}
Content-Type: application/json
X-Jurisdiction: KR
X-Idempotency-Key: {unique_key}
```

**요청 본문:**
```json
{
  "documentType": "cryopreservation_contract",
  "jurisdiction": {
    "primaryCountry": "KR",
    "governingLaw": "대한민국 민법"
  },
  "parties": [
    {
      "role": "subject",
      "identity": {
        "type": "individual",
        "legalName": "홍길동",
        "email": "hong@example.com"
      }
    }
  ],
  "content": {
    "format": "markdown",
    "body": "# 계약서\n\n계약 조건...",
    "templateId": "cryo-contract-v2"
  },
  "effectiveDate": "2025-02-01T00:00:00Z",
  "metadata": {
    "language": "ko-KR",
    "tags": ["전신", "한국"]
  }
}
```

**응답 (201 Created):**
```json
{
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "documentType": "cryopreservation_contract",
  "status": "draft",
  "createdAt": "2025-01-15T10:30:00Z",
  "jurisdiction": {
    "primaryCountry": "KR",
    "governingLaw": "대한민국 민법"
  },
  "requiredSignatures": [
    {
      "partyId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "role": "subject",
      "status": "pending"
    }
  ],
  "links": {
    "self": "/documents/550e8400-e29b-41d4-a716-446655440000",
    "sign": "/documents/550e8400-e29b-41d4-a716-446655440000/signatures",
    "pdf": "/documents/550e8400-e29b-41d4-a716-446655440000/export/pdf"
  }
}
```

#### 3.1.2 문서 조회

**GET** `/documents/{documentId}`

**요청:**
```http
GET /documents/550e8400-e29b-41d4-a716-446655440000
Authorization: Bearer {token}
Accept: application/json
```

**응답 (200 OK):**
```json
{
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "documentType": "cryopreservation_contract",
  "status": "executed",
  "createdAt": "2025-01-15T10:30:00Z",
  "effectiveDate": "2025-02-01T00:00:00Z",
  "jurisdiction": {
    "primaryCountry": "KR",
    "governingLaw": "대한민국 민법"
  },
  "parties": [...],
  "content": {...},
  "signatures": [...],
  "notarization": {...},
  "auditLog": [...]
}
```

#### 3.1.3 문서 목록 조회

**GET** `/documents`

**쿼리 매개변수:**
| 매개변수 | 타입 | 설명 |
|----------|------|------|
| `type` | string | 문서 유형으로 필터 |
| `status` | string | 상태로 필터 (draft, pending, executed) |
| `partyId` | string | 당사자 참여로 필터 |
| `jurisdiction` | string | 관할권으로 필터 |
| `createdAfter` | datetime | 생성일로 필터 |
| `createdBefore` | datetime | 생성일로 필터 |
| `page` | integer | 페이지 번호 (기본값: 1) |
| `limit` | integer | 페이지당 항목 수 (기본값: 20, 최대: 100) |

**요청:**
```http
GET /documents?type=cryopreservation_contract&status=executed&limit=10
Authorization: Bearer {token}
```

**응답 (200 OK):**
```json
{
  "documents": [
    {
      "documentId": "550e8400-e29b-41d4-a716-446655440000",
      "documentType": "cryopreservation_contract",
      "status": "executed",
      "createdAt": "2025-01-15T10:30:00Z",
      "parties": [{"role": "subject", "legalName": "홍길동"}]
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 10,
    "total": 45,
    "totalPages": 5
  },
  "links": {
    "self": "/documents?page=1&limit=10",
    "next": "/documents?page=2&limit=10",
    "last": "/documents?page=5&limit=10"
  }
}
```

#### 3.1.4 문서 수정

**PATCH** `/documents/{documentId}`

**요청:**
```json
{
  "content": {
    "body": "수정된 내용..."
  },
  "metadata": {
    "tags": ["수정됨", "한국"]
  }
}
```

**응답 (200 OK):**
```json
{
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.1",
  "status": "draft",
  "updatedAt": "2025-01-16T09:00:00Z"
}
```

#### 3.1.5 문서 삭제

**DELETE** `/documents/{documentId}`

**응답 (204 No Content)**

---

### 3.2 서명

#### 3.2.1 문서 서명

**POST** `/documents/{documentId}/signatures`

**요청:**
```json
{
  "signerId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "signatureType": "digital",
  "signatureData": "MEUCIQDx...",
  "certificate": "-----BEGIN CERTIFICATE-----...",
  "consent": {
    "acknowledged": true,
    "timestamp": "2025-01-15T14:30:00Z"
  }
}
```

**응답 (201 Created):**
```json
{
  "signatureId": "sig-123456",
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "signerId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "timestamp": "2025-01-15T14:30:00Z",
  "verificationStatus": "verified",
  "documentStatus": "pending",
  "remainingSignatures": 1
}
```

#### 3.2.2 서명 검증

**POST** `/signatures/verify`

**요청:**
```json
{
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "signatureId": "sig-123456"
}
```

**응답:**
```json
{
  "valid": true,
  "signerId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "signedAt": "2025-01-15T14:30:00Z",
  "certificateInfo": {
    "issuer": "WIA 인증 기관",
    "validFrom": "2024-01-01T00:00:00Z",
    "validTo": "2026-01-01T00:00:00Z",
    "status": "valid"
  },
  "documentIntegrity": "intact"
}
```

#### 3.2.3 서명 목록 조회

**GET** `/documents/{documentId}/signatures`

**응답:**
```json
{
  "signatures": [
    {
      "signatureId": "sig-123456",
      "signerId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "signerName": "홍길동",
      "role": "subject",
      "timestamp": "2025-01-15T14:30:00Z",
      "verificationStatus": "verified"
    }
  ]
}
```

---

### 3.3 당사자

#### 3.3.1 당사자 생성

**POST** `/parties`

**요청:**
```json
{
  "role": "subject",
  "identity": {
    "type": "individual",
    "legalName": "홍길동",
    "dateOfBirth": "1990-01-23",
    "nationality": "KR",
    "identificationNumber": "901234-1234567",
    "identificationType": "resident_registration"
  },
  "contact": {
    "email": "hong@example.com",
    "phone": "+82-10-1234-5678"
  },
  "address": {
    "street": "테헤란로 123",
    "city": "서울특별시",
    "state": "강남구",
    "postalCode": "06100",
    "country": "KR"
  }
}
```

**응답 (201 Created):**
```json
{
  "partyId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "role": "subject",
  "identity": {...},
  "verificationStatus": "pending",
  "createdAt": "2025-01-15T09:00:00Z"
}
```

#### 3.3.2 당사자 신원 확인

**POST** `/parties/{partyId}/verify`

**요청:**
```json
{
  "verificationType": "document",
  "documents": [
    {
      "type": "resident_card",
      "frontImage": "base64...",
      "backImage": "base64..."
    }
  ],
  "selfie": "base64..."
}
```

**응답:**
```json
{
  "verificationId": "ver-123456",
  "status": "verified",
  "confidence": 0.98,
  "checks": [
    {"check": "document_authenticity", "passed": true},
    {"check": "face_match", "passed": true, "confidence": 0.97},
    {"check": "liveness", "passed": true}
  ],
  "verifiedAt": "2025-01-15T10:00:00Z"
}
```

---

### 3.4 관할권

#### 3.4.1 관할권 요구사항 조회

**GET** `/jurisdictions/{countryCode}`

**응답:**
```json
{
  "countryCode": "KR",
  "countryName": "대한민국",
  "cryopreservationLegal": true,
  "requirements": {
    "documents": [
      {
        "type": "cryopreservation_contract",
        "required": true,
        "witnessesRequired": 2,
        "notarizationRequired": true
      },
      {
        "type": "advance_directive",
        "required": true,
        "witnessesRequired": 2
      }
    ],
    "minAge": 19,
    "waitingPeriod": null,
    "restrictions": []
  },
  "lastUpdated": "2025-01-01T00:00:00Z"
}
```

---

### 3.5 감사 로그

#### 3.5.1 문서 감사 로그 조회

**GET** `/documents/{documentId}/audit`

**응답:**
```json
{
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "entries": [
    {
      "entryId": "audit-001",
      "timestamp": "2025-01-15T10:30:00Z",
      "action": "created",
      "actorId": "user-123",
      "actorType": "user",
      "details": "템플릿에서 문서 생성됨",
      "ipAddress": "192.168.1.1"
    },
    {
      "entryId": "audit-002",
      "timestamp": "2025-01-15T14:30:00Z",
      "action": "signed",
      "actorId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "actorType": "party",
      "details": "디지털 서명 적용됨",
      "signatureId": "sig-123456"
    }
  ],
  "pagination": {...}
}
```

---

## 4. 오류 처리

### 4.1 오류 응답 형식

```json
{
  "error": {
    "code": "DOCUMENT_NOT_FOUND",
    "message": "지정된 ID의 문서가 존재하지 않습니다",
    "details": {
      "documentId": "550e8400-e29b-41d4-a716-446655440000"
    },
    "requestId": "req-abc123",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 4.2 오류 코드

| 코드 | HTTP 상태 | 메시지 | 설명 |
|------|-----------|--------|------|
| `UNAUTHORIZED` | 401 | 인증 필요 | 토큰 누락 또는 무효 |
| `FORBIDDEN` | 403 | 접근 거부 | 권한 부족 |
| `DOCUMENT_NOT_FOUND` | 404 | 문서를 찾을 수 없음 | 문서 ID가 존재하지 않음 |
| `PARTY_NOT_FOUND` | 404 | 당사자를 찾을 수 없음 | 당사자 ID가 존재하지 않음 |
| `VALIDATION_ERROR` | 400 | 검증 실패 | 잘못된 요청 데이터 |
| `INVALID_SIGNATURE` | 400 | 서명 무효 | 서명 검증 실패 |
| `DOCUMENT_LOCKED` | 409 | 문서가 잠김 | 서명된 문서 수정 불가 |
| `JURISDICTION_INVALID` | 400 | 관할권 무효 | 지원되지 않는 관할권 |
| `SIGNATURE_EXPIRED` | 400 | 인증서 만료 | 서명 인증서 만료됨 |
| `RATE_LIMITED` | 429 | 요청 제한 초과 | 너무 많은 요청 |
| `INTERNAL_ERROR` | 500 | 내부 서버 오류 | 서버 측 오류 |

---

## 5. 요청 제한

### 5.1 제한

| 등급 | 시간당 요청 | 일일 요청 |
|------|------------|----------|
| 무료 | 100 | 1,000 |
| 표준 | 1,000 | 10,000 |
| 엔터프라이즈 | 10,000 | 100,000 |

### 5.2 요청 제한 헤더

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1642288800
```

---

## 6. SDK 예제

### 6.1 Python SDK

```python
from wia_cryo_legal import CryoLegalClient

# 클라이언트 초기화
client = CryoLegalClient(
    api_key="your-api-key",
    environment="production"
)

# 문서 생성
document = client.documents.create(
    document_type="cryopreservation_contract",
    jurisdiction={
        "primary_country": "KR",
        "governing_law": "대한민국 민법"
    },
    parties=[
        {
            "role": "subject",
            "identity": {
                "type": "individual",
                "legal_name": "홍길동"
            }
        }
    ],
    content={
        "template_id": "cryo-contract-v2",
        "variables": {"service_type": "whole_body"}
    }
)

print(f"생성된 문서: {document.document_id}")

# 문서 서명
signature = client.signatures.create(
    document_id=document.document_id,
    signer_id="party-uuid",
    signature_type="digital",
    private_key=private_key
)

# 서명 검증
verification = client.signatures.verify(
    document_id=document.document_id,
    signature_id=signature.signature_id
)

print(f"서명 유효: {verification.valid}")

# 문서 목록 조회
documents = client.documents.list(
    type="cryopreservation_contract",
    status="executed",
    limit=10
)

for doc in documents:
    print(f"- {doc.document_id}: {doc.status}")
```

### 6.2 TypeScript SDK

```typescript
import { CryoLegalClient } from '@wia/cryo-legal';

// 클라이언트 초기화
const client = new CryoLegalClient({
  apiKey: 'your-api-key',
  environment: 'production'
});

// 문서 생성
const document = await client.documents.create({
  documentType: 'cryopreservation_contract',
  jurisdiction: {
    primaryCountry: 'KR',
    governingLaw: '대한민국 민법'
  },
  parties: [
    {
      role: 'subject',
      identity: {
        type: 'individual',
        legalName: '홍길동'
      }
    }
  ],
  content: {
    templateId: 'cryo-contract-v2',
    variables: { serviceType: 'whole_body' }
  }
});

console.log(`생성된 문서: ${document.documentId}`);

// 문서 서명
const signature = await client.signatures.create({
  documentId: document.documentId,
  signerId: 'party-uuid',
  signatureType: 'digital',
  privateKey: privateKey
});

// 검증
const verification = await client.signatures.verify({
  documentId: document.documentId,
  signatureId: signature.signatureId
});

console.log(`서명 유효: ${verification.valid}`);

// 비동기 반복으로 목록 조회
for await (const doc of client.documents.list({ status: 'executed' })) {
  console.log(`- ${doc.documentId}: ${doc.status}`);
}
```

### 6.3 cURL 예제

```bash
# 문서 생성
curl -X POST https://api.wia.live/cryo-legal/v1/documents \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "documentType": "cryopreservation_contract",
    "jurisdiction": {"primaryCountry": "KR", "governingLaw": "대한민국 민법"},
    "parties": [{"role": "subject", "identity": {"legalName": "홍길동"}}],
    "content": {"format": "markdown", "body": "# 계약서"}
  }'

# 문서 조회
curl https://api.wia.live/cryo-legal/v1/documents/550e8400-... \
  -H "Authorization: Bearer $TOKEN"

# 문서 목록 조회
curl "https://api.wia.live/cryo-legal/v1/documents?status=executed&limit=10" \
  -H "Authorization: Bearer $TOKEN"

# 문서 서명
curl -X POST https://api.wia.live/cryo-legal/v1/documents/550e8400-.../signatures \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "signerId": "party-uuid",
    "signatureType": "digital",
    "signatureData": "MEUCIQDx..."
  }'
```

---

## 7. 웹훅

### 7.1 웹훅 이벤트

| 이벤트 | 설명 |
|--------|------|
| `document.created` | 새 문서 생성됨 |
| `document.updated` | 문서 수정됨 |
| `document.signed` | 서명 추가됨 |
| `document.executed` | 모든 서명 완료 |
| `document.expired` | 문서 만료됨 |
| `party.verified` | 당사자 신원 확인됨 |

### 7.2 웹훅 페이로드

```json
{
  "event": "document.signed",
  "timestamp": "2025-01-15T14:30:00Z",
  "data": {
    "documentId": "550e8400-e29b-41d4-a716-446655440000",
    "signatureId": "sig-123456",
    "signerId": "party-uuid",
    "documentStatus": "pending",
    "remainingSignatures": 1
  }
}
```

### 7.3 웹훅 서명 검증

```python
import hmac
import hashlib

def verify_webhook(payload: bytes, signature: str, secret: str) -> bool:
    expected = hmac.new(
        secret.encode(),
        payload,
        hashlib.sha256
    ).hexdigest()
    return hmac.compare_digest(f"sha256={expected}", signature)
```

---

<div align="center">

**WIA Cryo-Legal 표준 v1.0.0**

2단계: API 인터페이스 명세

**弘益人間 (홍익인간)** · 널리 인간을 이롭게

---

© 2025 WIA 표준 위원회

MIT 라이선스

</div>
