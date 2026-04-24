# WIA Cryo-Consent API Interface Standard
## Phase 2 Specification

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
2. [API 아키텍처](#api-아키텍처)
3. [인증 및 권한 부여](#인증-및-권한-부여)
4. [Endpoints](#endpoints)
5. [요청/응답 형식](#요청응답-형식)
6. [오류 처리](#오류-처리)
7. [Rate Limiting](#rate-limiting)
8. [코드 예제](#코드-예제)
9. [버전 이력](#버전-이력)

---

## 개요

### 1.1 목적

WIA Cryo-Consent API Interface Standard는 법적 동의 기록의 전체 생명주기를 관리하기 위한 RESTful API endpoint를 정의합니다. 이 표준은 의료 시설, 법적 시스템 및 냉동보존 조직 전반에 걸쳐 안전하고 규정을 준수하며 상호운용 가능한 동의 관리를 가능하게 합니다.

**핵심 목표**:
- 동의 생명주기 관리를 위한 표준화된 REST API 제공
- 안전한 동의 생성, 수정 및 철회 지원
- 다중 관할권 동의 검증 및 준수 지원
- 대리인 및 후견인 동의 위임 워크플로 촉진
- 암호화 검증 및 감사 추적 무결성 보장
- 법적, 의료 및 blockchain 시스템과 통합

### 1.2 설계 원칙

1. **RESTful 아키텍처**: 표준 HTTP 메서드 및 상태 코드
2. **보안 우선**: OAuth 2.0, JWT token 및 종단 간 암호화
3. **멱등성**: 중요한 작업을 위한 안전한 재시도 메커니즘
4. **버전 관리**: Header를 통한 API 버전 협상
5. **Pagination**: 대규모 결과 집합을 위한 커서 기반 pagination
6. **Filtering**: 데이터 검색을 위한 풍부한 쿼리 매개변수

### 1.3 API 버전 관리

| Version | Base URL | Status |
|---------|----------|--------|
| v1 | `https://api.wia.live/cryo-consent/v1` | Current |

---

## API 아키텍처

### 2.1 아키텍처 개요

```
┌─────────────┐
│   Clients   │
│ (Web, CLI)  │
└──────┬──────┘
       │
       │ HTTPS/REST
       │
┌──────▼──────────────────────────────┐
│     API Gateway                      │
│  - Authentication                    │
│  - Rate Limiting                     │
│  - Request Validation               │
└──────┬──────────────────────────────┘
       │
       │
┌──────▼──────────────────────────────┐
│  Consent Management Service          │
│  - Consent CRUD Operations          │
│  - Validation & Verification        │
│  - Event Publishing                 │
└──────┬──────────────────────────────┘
       │
       │
┌──────▼──────────────────────────────┐
│   Data Layer                         │
│  - PostgreSQL (Primary)             │
│  - Blockchain (Audit)               │
│  - Document Storage                 │
└─────────────────────────────────────┘
```

### 2.2 서비스 기능

| Service | 설명 | SLA |
|---------|------|-----|
| Consent Management | 동의 기록에 대한 CRUD 작업 | 99.9% |
| Validation Service | 법적 및 비즈니스 규칙 검증 | 99.95% |
| Notification Service | 실시간 동의 변경 알림 | 99.5% |
| Audit Service | 불변 감사 추적 관리 | 99.99% |
| Integration Service | 타사 시스템 통합 | 99.5% |

### 2.3 데이터 흐름

| Operation | Flow | Validation Points |
|-----------|------|-------------------|
| Create Consent | Client → API → Validation → Storage → Blockchain | Identity, Legal, Business Rules |
| Modify Consent | Client → API → Verification → New Version → Notification | Authority, Versioning, Legal |
| Revoke Consent | Client → API → Finality Check → Archive → Notification | Authority, Irreversibility |
| Query Consent | Client → API → Authorization → Fetch → Response | Access Rights, Privacy |

---

## 인증 및 권한 부여

### 3.1 인증 방법

| Method | Use Case | 설명 |
|--------|----------|------|
| OAuth 2.0 | Web Applications | PKCE를 사용한 표준 OAuth 2.0 flow |
| JWT Bearer Token | API Clients | 단기 access token |
| API Key | Service-to-Service | 신뢰할 수 있는 서비스를 위한 장기 자격 증명 |
| mTLS | High Security | 중요한 작업을 위한 상호 TLS |

### 3.2 Authorization Scope

| Scope | 설명 | Access Level |
|-------|------|--------------|
| `consent:read` | 동의 기록 읽기 | 동의 세부 정보 보기 |
| `consent:write` | 동의 생성 및 수정 | 삭제를 제외한 전체 CRUD |
| `consent:revoke` | 동의 철회 | 영구 철회 |
| `consent:admin` | 관리 작업 | 감사를 포함한 모든 작업 |
| `proxy:manage` | 대리인 위임 관리 | 대리인 추가/제거 |
| `legal:verify` | 법적 검증 | 공증 및 증인 |

### 3.3 인증 Header

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
X-API-Key: wia_live_sk_1234567890abcdef
X-Request-ID: uuid-v4-string
X-API-Version: 1.0.0
```

---

## Endpoints

### 4.1 동의 관리 Endpoint

#### 4.1.1 동의 기록 생성

```http
POST /api/v1/consent
```

완전한 법적 검증으로 새로운 동의 기록을 생성합니다.

**Request Body:**
```json
{
  "grantor": {
    "id": "PERSON-001",
    "identityVerification": {
      "method": "government_id",
      "documentId": "DL-CA-D1234567"
    }
  },
  "consent": {
    "scope": {
      "preservation": {
        "authorized": true,
        "types": ["whole_body"]
      },
      "research": {
        "authorized": true,
        "categories": ["medical", "scientific"]
      },
      "revival": {
        "authorized": true,
        "minimumViabilityThreshold": 0.7
      }
    },
    "jurisdiction": ["US-CA"]
  },
  "legal": {
    "witnesses": [
      {
        "name": "Jane Doe",
        "signature": "0x1234abcd..."
      }
    ],
    "jurisdiction": "US-CA"
  }
}
```

**Response (201 Created):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "status": "active",
  "version": 1,
  "createdAt": "2025-01-15T10:30:00Z",
  "effectiveDate": "2025-01-15T00:00:00Z",
  "meta": {
    "hash": "sha256:a5b9c3d4e5f6...",
    "signature": "0xabcdef123456...",
    "blockchainAnchor": "ethereum:0x1234567890abcdef"
  }
}
```

**검증 규칙:**
- Grantor는 검증되어야 함
- 최소 1명의 증인 또는 공증 필요
- 관할권별 규칙 적용
- 모든 필수 scope 필드 존재

---

#### 4.1.2 동의 기록 조회

```http
GET /api/v1/consent/{consentId}
```

ID로 특정 동의 기록을 검색합니다.

**Path Parameters:**
- `consentId` (string, required): 동의 기록의 UUID

**Query Parameters:**
- `version` (integer, optional): 특정 버전 번호 (기본값: 최신)
- `include` (string, optional): 관련 데이터 포함 (`proxies`, `history`, `audit`)

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "version": 1,
  "messageType": "consent_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z",
    "effectiveDate": "2025-01-15T00:00:00Z"
  },
  "grantor": {
    "id": "PERSON-001"
  },
  "consent": {
    "status": "active",
    "scope": { /* ... */ }
  },
  "legal": { /* ... */ },
  "meta": { /* ... */ }
}
```

---

#### 4.1.3 동의 기록 업데이트

```http
PUT /api/v1/consent/{consentId}
```

기존 동의 기록의 새 버전을 생성합니다. 원래 동의는 감사 추적을 위해 보존됩니다.

**Request Body:**
```json
{
  "modifications": {
    "consent": {
      "scope": {
        "research": {
          "authorized": true,
          "categories": ["medical", "scientific", "commercial"]
        }
      }
    }
  },
  "verification": {
    "method": "video",
    "verifierId": "LAWYER-001"
  },
  "legal": {
    "witnesses": [
      {
        "name": "Attorney Sarah Lee",
        "signature": "0x9999aaaa..."
      }
    ]
  }
}
```

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "version": 2,
  "previousVersion": 1,
  "status": "active",
  "modifiedAt": "2025-02-20T14:00:00Z",
  "meta": {
    "hash": "sha256:b6c8d5e7f8...",
    "previousHash": "sha256:a5b9c3d4e5f6..."
  }
}
```

---

#### 4.1.4 동의 철회

```http
POST /api/v1/consent/{consentId}/revoke
```

동의를 영구적으로 철회합니다. 이 작업은 되돌릴 수 없습니다.

**Request Body:**
```json
{
  "reason": "결정 변경 - 더 이상 보존을 원하지 않음",
  "verification": {
    "method": "notary",
    "verifierId": "NOTARY-CA-12345"
  },
  "legal": {
    "witnesses": [
      {
        "name": "Michael Chen",
        "signature": "0xbbbbcccc..."
      }
    ],
    "notarization": {
      "notaryId": "NOTARY-CA-12345",
      "sealNumber": "SEAL-2025-045"
    }
  }
}
```

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "status": "revoked",
  "revokedAt": "2025-03-10T09:00:00Z",
  "version": 3,
  "finalHash": "sha256:c7d9e6f8g9..."
}
```

---

#### 4.1.5 동의 기록 목록 조회

```http
GET /api/v1/consent
```

필터링이 가능한 페이지네이션된 동의 기록 목록을 검색합니다.

**Query Parameters:**
- `grantorId` (string, optional): grantor로 필터링
- `status` (string, optional): 상태로 필터링 (`active`, `revoked` 등)
- `jurisdiction` (string, optional): 관할권으로 필터링
- `fromDate` (string, optional): ISO 8601 날짜
- `toDate` (string, optional): ISO 8601 날짜
- `limit` (integer, optional): 페이지 크기 (기본값: 50, 최대: 100)
- `cursor` (string, optional): Pagination cursor

**Response (200 OK):**
```json
{
  "data": [
    {
      "consentId": "550e8400-e29b-41d4-a716-446655440001",
      "status": "active",
      "createdAt": "2025-01-15T10:30:00Z",
      "grantor": { "id": "PERSON-001" }
    }
  ],
  "pagination": {
    "nextCursor": "eyJpZCI6MTIzfQ==",
    "hasMore": true,
    "total": 150
  }
}
```

---

### 4.2 대리인 관리 Endpoint

#### 4.2.1 대리인 추가

```http
POST /api/v1/consent/{consentId}/proxy
```

대리인에게 동의 관리 권한을 위임합니다.

**Request Body:**
```json
{
  "proxy": {
    "id": "PROXY-001",
    "type": "healthcare_proxy",
    "authorizedActions": ["modify_consent", "authorize_revival"],
    "effectiveDate": "2025-01-15T00:00:00Z"
  },
  "verification": {
    "method": "notary",
    "verifierId": "NOTARY-001"
  }
}
```

**Response (201 Created):**
```json
{
  "proxyId": "PROXY-001",
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "status": "active",
  "effectiveDate": "2025-01-15T00:00:00Z",
  "authorizedActions": ["modify_consent", "authorize_revival"]
}
```

---

#### 4.2.2 대리인 목록 조회

```http
GET /api/v1/consent/{consentId}/proxies
```

동의 기록의 모든 대리인을 검색합니다.

**Response (200 OK):**
```json
{
  "proxies": [
    {
      "proxyId": "PROXY-001",
      "type": "healthcare_proxy",
      "status": "active",
      "authorizedActions": ["modify_consent", "authorize_revival"],
      "effectiveDate": "2025-01-15T00:00:00Z"
    }
  ]
}
```

---

#### 4.2.3 대리인 철회

```http
DELETE /api/v1/consent/{consentId}/proxy/{proxyId}
```

대리인 권한을 제거합니다.

**Response (200 OK):**
```json
{
  "proxyId": "PROXY-001",
  "status": "revoked",
  "revokedAt": "2025-04-01T12:00:00Z"
}
```

---

### 4.3 검증 및 확인 Endpoint

#### 4.3.1 동의 검증

```http
POST /api/v1/consent/validate
```

기록을 생성하지 않고 법적 및 비즈니스 규칙에 대해 동의 데이터를 검증합니다.

**Request Body:**
```json
{
  "consent": {
    /* 전체 동의 객체 */
  },
  "jurisdiction": "US-CA"
}
```

**Response (200 OK):**
```json
{
  "valid": true,
  "warnings": [
    "공증이 제공되지 않음 - California에서 whole_body에 필요"
  ],
  "errors": [],
  "compliance": {
    "US-CA": "valid",
    "GDPR": "compliant",
    "HIPAA": "compliant"
  }
}
```

---

#### 4.3.2 동의 서명 검증

```http
POST /api/v1/consent/{consentId}/verify
```

암호화 서명 및 blockchain anchor를 검증합니다.

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "verified": true,
  "checks": {
    "hashIntegrity": true,
    "signatureValid": true,
    "blockchainAnchored": true,
    "witnessSignatures": true
  },
  "blockchainProof": {
    "network": "ethereum",
    "transactionHash": "0x1234567890abcdef",
    "blockNumber": 18234567,
    "timestamp": "2025-01-15T10:35:00Z"
  }
}
```

---

#### 4.3.3 동의 상태 확인

```http
GET /api/v1/consent/{consentId}/status
```

전체 기록 검색 없이 빠른 상태 확인.

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "status": "active",
  "effectiveDate": "2025-01-15T00:00:00Z",
  "expirationDate": null,
  "version": 2,
  "lastModified": "2025-02-20T14:00:00Z"
}
```

---

### 4.4 이력 및 감사 Endpoint

#### 4.4.1 동의 이력 조회

```http
GET /api/v1/consent/{consentId}/history
```

완전한 버전 이력을 검색합니다.

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "versions": [
    {
      "version": 1,
      "timestamp": "2025-01-15T10:30:00Z",
      "messageType": "consent_record",
      "status": "superseded",
      "hash": "sha256:a5b9c3d4e5f6..."
    },
    {
      "version": 2,
      "timestamp": "2025-02-20T14:00:00Z",
      "messageType": "consent_modification",
      "status": "active",
      "hash": "sha256:b6c8d5e7f8...",
      "previousHash": "sha256:a5b9c3d4e5f6..."
    }
  ]
}
```

---

#### 4.4.2 감사 추적 조회

```http
GET /api/v1/consent/{consentId}/audit
```

완전한 감사 로그를 검색합니다.

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "events": [
    {
      "eventId": "EVT-001",
      "timestamp": "2025-01-15T10:30:00Z",
      "eventType": "consent_created",
      "actor": "PERSON-001",
      "details": {
        "version": 1,
        "status": "active"
      }
    },
    {
      "eventId": "EVT-002",
      "timestamp": "2025-02-20T14:00:00Z",
      "eventType": "consent_modified",
      "actor": "PERSON-001",
      "details": {
        "version": 2,
        "modifications": ["research.categories"]
      }
    }
  ]
}
```

---

### 4.5 관할권 및 규정 준수 Endpoint

#### 4.5.1 관할권 요구사항 조회

```http
GET /api/v1/jurisdiction/{jurisdictionCode}
```

특정 관할권에 대한 법적 요구사항을 검색합니다.

**Response (200 OK):**
```json
{
  "jurisdiction": "US-CA",
  "name": "California, United States",
  "requirements": {
    "minimumAge": 18,
    "witnessRequired": true,
    "minimumWitnesses": 2,
    "notarizationRequired": true,
    "waitingPeriod": null,
    "mentalCapacityAssessment": "recommended"
  },
  "governingLaws": [
    "California Health and Safety Code Section 7100-7117",
    "Uniform Anatomical Gift Act"
  ],
  "compliance": ["HIPAA", "California Consumer Privacy Act"]
}
```

---

#### 4.5.2 다중 관할권 준수 확인

```http
POST /api/v1/consent/compliance-check
```

여러 관할권에 대해 동의를 검증합니다.

**Request Body:**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "jurisdictions": ["US-CA", "US-NY", "EU-DE"]
}
```

**Response (200 OK):**
```json
{
  "compliant": false,
  "jurisdictions": {
    "US-CA": {
      "compliant": true,
      "issues": []
    },
    "US-NY": {
      "compliant": true,
      "issues": []
    },
    "EU-DE": {
      "compliant": false,
      "issues": [
        "GDPR 데이터 보호 영향 평가 누락",
        "EU 데이터 대표자 지정되지 않음"
      ]
    }
  }
}
```

---

## 요청/응답 형식

### 5.1 표준 응답 구조

| 필드 | Type | 설명 |
|------|------|------|
| `data` | object/array | 응답 페이로드 |
| `meta` | object | 메타데이터 (pagination, timestamp) |
| `errors` | array | 오류 세부 정보 (해당하는 경우) |

### 5.2 오류 응답 구조

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "동의 검증 실패",
    "details": [
      {
        "field": "legal.witnesses",
        "issue": "US-NY에서 최소 2명의 증인 필요"
      }
    ],
    "requestId": "req_1234567890",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 5.3 HTTP 상태 코드

| Code | 의미 | 사용 |
|------|------|------|
| 200 | OK | 성공적인 GET, PUT, POST (수정) |
| 201 | Created | 성공적인 리소스 생성 |
| 400 | Bad Request | 잘못된 요청 형식 또는 매개변수 |
| 401 | Unauthorized | 인증 누락 또는 유효하지 않음 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스가 존재하지 않음 |
| 409 | Conflict | 리소스 상태 충돌 |
| 422 | Unprocessable Entity | 유효한 형식이지만 비즈니스 로직 실패 |
| 429 | Too Many Requests | Rate limit 초과 |
| 500 | Internal Server Error | 서버 측 오류 |
| 503 | Service Unavailable | 일시적인 서비스 중단 |

---

## 오류 처리

### 6.1 오류 범주

| Category | HTTP Code | 설명 |
|----------|-----------|------|
| Validation Error | 400, 422 | 데이터 검증 실패 |
| Authentication Error | 401 | 유효하지 않은 자격 증명 |
| Authorization Error | 403 | 권한 부족 |
| Not Found Error | 404 | 리소스를 찾을 수 없음 |
| Conflict Error | 409 | 상태 충돌 (예: 이미 철회됨) |
| Rate Limit Error | 429 | 요청이 너무 많음 |
| Server Error | 500, 503 | 내부 시스템 오류 |

### 6.2 오류 코드

| Code | 설명 | 해결 방법 |
|------|------|-----------|
| `INVALID_CONSENT_DATA` | 동의 데이터 형식이 유효하지 않음 | JSON 스키마 확인 |
| `INSUFFICIENT_ATTESTATION` | 증인/공증 누락 | 필요한 증명 추가 |
| `INVALID_JURISDICTION` | 관할권이 지원되지 않음 | 유효한 관할권 코드 사용 |
| `CONSENT_ALREADY_REVOKED` | 철회된 동의를 수정할 수 없음 | 새 동의 생성 |
| `UNAUTHORIZED_PROXY` | 대리인에게 권한 없음 | 대리인 권한 확인 |
| `SIGNATURE_VERIFICATION_FAILED` | 유효하지 않은 암호화 서명 | 유효한 키로 재서명 |
| `BLOCKCHAIN_ANCHOR_FAILED` | Blockchain anchoring 오류 | 작업 재시도 |

---

## Rate Limiting

### 7.1 Rate Limit 계층

| Tier | Requests/Hour | Burst |
|------|---------------|-------|
| Free | 100 | 10 |
| Standard | 1,000 | 50 |
| Professional | 10,000 | 200 |
| Enterprise | Custom | Custom |

### 7.2 Rate Limit Header

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642262400
Retry-After: 3600
```

---

## 코드 예제

### 8.1 TypeScript - 동의 생성

```typescript
import axios from 'axios';

interface ConsentRequest {
  grantor: {
    id: string;
    identityVerification: {
      method: string;
      documentId: string;
    };
  };
  consent: {
    scope: {
      preservation: {
        authorized: boolean;
        types: string[];
      };
      research: {
        authorized: boolean;
        categories: string[];
      };
      revival: {
        authorized: boolean;
        minimumViabilityThreshold: number;
      };
    };
    jurisdiction: string[];
  };
  legal: {
    witnesses: Array<{
      name: string;
      signature: string;
    }>;
    jurisdiction: string;
  };
}

async function createConsent(
  apiKey: string,
  consentData: ConsentRequest
): Promise<any> {
  try {
    const response = await axios.post(
      'https://api.wia.live/cryo-consent/v1/consent',
      consentData,
      {
        headers: {
          'Authorization': `Bearer ${apiKey}`,
          'Content-Type': 'application/json',
          'X-API-Version': '1.0.0'
        }
      }
    );

    console.log('동의 생성됨:', response.data.consentId);
    console.log('Blockchain anchor:', response.data.meta.blockchainAnchor);

    return response.data;
  } catch (error) {
    if (axios.isAxiosError(error)) {
      console.error('API 오류:', error.response?.data);
      throw new Error(`동의 생성 실패: ${error.response?.data.error.message}`);
    }
    throw error;
  }
}

// 사용 예
const consentData: ConsentRequest = {
  grantor: {
    id: 'PERSON-001',
    identityVerification: {
      method: 'government_id',
      documentId: 'DL-CA-D1234567'
    }
  },
  consent: {
    scope: {
      preservation: {
        authorized: true,
        types: ['whole_body']
      },
      research: {
        authorized: true,
        categories: ['medical', 'scientific']
      },
      revival: {
        authorized: true,
        minimumViabilityThreshold: 0.7
      }
    },
    jurisdiction: ['US-CA']
  },
  legal: {
    witnesses: [
      {
        name: 'Jane Doe',
        signature: '0x1234abcd...'
      }
    ],
    jurisdiction: 'US-CA'
  }
};

createConsent('wia_live_sk_1234567890', consentData);
```

### 8.2 Python - 동의 조회

```python
import requests
from typing import Dict, Optional
from datetime import datetime

class ConsentClient:
    def __init__(self, api_key: str, base_url: str = "https://api.wia.live/cryo-consent/v1"):
        self.api_key = api_key
        self.base_url = base_url
        self.session = requests.Session()
        self.session.headers.update({
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json',
            'X-API-Version': '1.0.0'
        })

    def get_consent(
        self,
        consent_id: str,
        version: Optional[int] = None,
        include: Optional[str] = None
    ) -> Dict:
        """
        ID로 동의 기록 검색.

        Args:
            consent_id: 동의 기록의 UUID
            version: 특정 버전 번호 (기본값: 최신)
            include: 관련 데이터 포함 (proxies, history, audit)

        Returns:
            동의 기록을 포함하는 Dict
        """
        url = f"{self.base_url}/consent/{consent_id}"
        params = {}

        if version:
            params['version'] = version
        if include:
            params['include'] = include

        try:
            response = self.session.get(url, params=params)
            response.raise_for_status()

            data = response.json()
            print(f"검색된 동의: {data['consentId']}")
            print(f"상태: {data['consent']['status']}")
            print(f"버전: {data['version']}")

            return data

        except requests.exceptions.HTTPError as e:
            error_data = e.response.json()
            print(f"오류: {error_data['error']['message']}")
            raise

    def revoke_consent(
        self,
        consent_id: str,
        reason: str,
        verification: Dict
    ) -> Dict:
        """
        동의 기록 철회.

        Args:
            consent_id: 동의 기록의 UUID
            reason: 철회 사유
            verification: 검증 세부 정보

        Returns:
            철회 확인을 포함하는 Dict
        """
        url = f"{self.base_url}/consent/{consent_id}/revoke"
        payload = {
            'reason': reason,
            'verification': verification,
            'legal': {
                'witnesses': [
                    {
                        'name': 'Michael Chen',
                        'signature': '0xbbbbcccc...'
                    }
                ]
            }
        }

        try:
            response = self.session.post(url, json=payload)
            response.raise_for_status()

            data = response.json()
            print(f"동의 철회됨: {data['consentId']}")
            print(f"철회 시간: {data['revokedAt']}")

            return data

        except requests.exceptions.HTTPError as e:
            error_data = e.response.json()
            print(f"오류: {error_data['error']['message']}")
            raise

# 사용 예
client = ConsentClient('wia_live_sk_1234567890')

# 이력과 함께 동의 조회
consent = client.get_consent(
    '550e8400-e29b-41d4-a716-446655440001',
    include='history,proxies'
)

# 동의 철회
revocation = client.revoke_consent(
    '550e8400-e29b-41d4-a716-446655440001',
    reason='결정 변경',
    verification={
        'method': 'notary',
        'verifierId': 'NOTARY-CA-12345'
    }
)
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
| WIA Cryo-Consent Phase 1 | 데이터 형식 명세 |
| WIA Cryo-Consent Phase 3 | 실시간 알림 프로토콜 |
| OAuth 2.0 RFC 6749 | 인증 프레임워크 |
| OpenAPI 3.0 | API 문서 표준 |

---

<div align="center">

**WIA Cryo-Consent API Interface Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
