# WIA-CRYO-ASSET Phase 2: API 인터페이스 명세서

**버전:** 1.0.0
**상태:** 초안
**최종 수정일:** 2025-12-18
**대표 색상:** #06B6D4 (Cyan)

## 1. 소개

### 1.1 목적

이 Phase 2 명세서는 WIA 극저온 자산 관리 표준을 위한 API(Application Programming Interface)를 정의합니다. 이러한 API는 극저온 시설, 금융 기관, 수탁자, 법률 대리인 및 소생 검증 시스템 간의 안전하고 표준화된 상호작용을 가능하게 합니다.

### 1.2 API 설계 원칙

- **보안 우선**: 모든 엔드포인트는 인증 및 권한 부여 필요
- **멱등성**: 쓰기 작업은 멱등 요청 지원
- **버전 관리**: URL 경로 및 헤더를 통한 API 버전 관리
- **속도 제한**: 남용 및 DoS 공격 방지
- **감사 로깅**: 모든 작업의 완전한 감사 추적
- **HATEOAS**: 적절한 하이퍼미디어 기반 탐색

### 1.3 기본 URL 구조

```
운영:  https://api.wia.dev/cryo-asset/v1
스테이징:     https://api-staging.wia.dev/cryo-asset/v1
샌드박스:     https://api-sandbox.wia.dev/cryo-asset/v1
```

## 2. 인증 및 권한 부여

### 2.1 인증 방법

API는 클라이언트 유형 및 보안 요구사항에 따라 여러 인증 방법을 지원합니다:

| 방법 | 사용 사례 | 보안 레벨 | 토큰 수명 |
|------|----------|----------|-----------|
| OAuth 2.0 | 웹 애플리케이션, 신뢰할 수 있는 클라이언트 | 높음 | 1시간(액세스), 30일(갱신) |
| JWT Bearer | 서비스 간 통신 | 높음 | 24시간 |
| API Key + Secret | 레거시 시스템 | 중간 | 만료 없음(순환 가능) |
| Multi-Sig | 고가치 트랜잭션 | 중요 | 단일 사용 |
| Biometric + JWT | 소생 검증 | 중요 | 15분 |

### 2.2 OAuth 2.0 인증 흐름

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "cryo_custodian_12345",
  "client_secret": "sk_REPLACE_WITH_YOUR_API_KEY_PLACEHOLDER",
  "scope": "asset:read asset:write custodian:manage"
}
```

**응답:**

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "rt_a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6",
  "scope": "asset:read asset:write custodian:manage"
}
```

### 2.3 권한 범위

| 범위 | 설명 | 필요한 역할 |
|------|-----|------------|
| `asset:read` | 자산 정보 조회 | 모든 인증된 사용자 |
| `asset:write` | 자산 생성 또는 수정 | 수탁자, 법률 대리인 |
| `asset:delete` | 자산 보관 또는 제거 | 주 수탁자 + 법적 승인 |
| `custodian:read` | 수탁자 정보 조회 | 수탁자, 법률 대리인, 가족 |
| `custodian:manage` | 수탁자 추가/제거 | 주 수탁자 + 법원 명령 |
| `transaction:create` | 자산 트랜잭션 시작 | 수탁자 |
| `transaction:approve` | 대기 중인 트랜잭션 승인 | 승인된 서명자 |
| `revival:verify` | 소생 검증 제출 | 의료 책임자, 법률 당국 |
| `revival:execute` | 소생 시 자산 이전 실행 | 모든 필수 당국 |
| `audit:read` | 감사 로그 접근 | 수탁자, 법률 대리인 |

## 3. 핵심 API 엔드포인트

### 3.1 자산 등록부 엔드포인트

#### 3.1.1 자산 등록부 생성

```http
POST /registries
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "subject": {
    "individualId": "CRYO-IND-2025-987654",
    "legalName": "홍길동",
    "dateOfBirth": "1985-06-15",
    "nationality": ["KOR"],
    "identityProof": {
      "type": "biometric_hash",
      "value": "a7f3c9e2b8d4f1a6e9c3b7d2f5a8e1c4b9d6f3a7e2c8b5d1f4a9e6c3b8d7f2a5"
    }
  },
  "preservationDetails": {
    "facility": "Alcor Life Extension Foundation",
    "facilityId": "ALCOR-AZ-001",
    "preservationDate": "2025-12-20T14:00:00Z",
    "preservationType": "vitrification"
  },
  "custodians": [
    {
      "type": "primary",
      "name": "신탁회사 알파",
      "contact": "custodian@trustalpha.com",
      "publicKey": "-----BEGIN PUBLIC KEY-----\nMIICIjANBgkq..."
    }
  ]
}
```

**응답 (201 Created):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "version": "1.0.0",
  "status": "active",
  "created": "2025-12-18T10:30:00Z",
  "subject": {
    "individualId": "CRYO-IND-2025-987654",
    "legalName": "홍길동"
  },
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9",
    "assets": "/registries/AR-2025-1734519000-A7F3C9/assets",
    "custodians": "/registries/AR-2025-1734519000-A7F3C9/custodians"
  }
}
```

#### 3.1.2 자산 등록부 조회

```http
GET /registries/{registryId}
Authorization: Bearer {access_token}
```

**응답 (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "version": "1.0.0",
  "status": "active",
  "created": "2025-12-18T10:30:00Z",
  "lastModified": "2025-12-18T12:45:00Z",
  "subject": {
    "individualId": "CRYO-IND-2025-987654",
    "legalName": "홍길동",
    "dateOfBirth": "1985-06-15",
    "preservationStatus": "preserved",
    "preservationDate": "2025-12-20T14:00:00Z"
  },
  "assets": {
    "financial": 12,
    "realEstate": 3,
    "intellectual": 2,
    "digital": 45,
    "personal": 8,
    "business": 1
  },
  "totalValue": {
    "amount": 5750000.00,
    "currency": "USD",
    "lastAssessed": "2025-12-18T00:00:00Z"
  },
  "custodians": [
    {
      "id": "CUST-001",
      "type": "primary",
      "name": "신탁회사 알파",
      "status": "active"
    }
  ]
}
```

#### 3.1.3 자산 등록부 업데이트

```http
PATCH /registries/{registryId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "operations": [
    {
      "op": "add",
      "path": "/custodians/-",
      "value": {
        "type": "backup",
        "name": "재산 변호사 서비스",
        "contact": "attorney@estatellc.com"
      }
    }
  ]
}
```

### 3.2 자산 관리 엔드포인트

#### 3.2.1 금융 자산 추가

```http
POST /registries/{registryId}/assets/financial
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "category": "cryptocurrency",
  "asset": {
    "name": "비트코인 보유분",
    "cryptocurrency": {
      "symbol": "BTC",
      "network": "bitcoin",
      "wallets": [
        {
          "type": "cold_storage",
          "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
          "balance": {
            "amount": "12.45678901",
            "unit": "BTC"
          },
          "accessMethod": "multisig",
          "requiredSignatures": 3,
          "totalSignatures": 5
        }
      ]
    }
  },
  "custodian": {
    "primary": "신탁회사 알파",
    "backup": ["재산 변호사"]
  }
}
```

**응답 (201 Created):**

```json
{
  "assetId": "FA-2025-BTC-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "assetType": "financial",
  "category": "cryptocurrency",
  "status": "active",
  "registered": "2025-12-18T10:35:00Z",
  "valuation": {
    "totalValue": 523456.78,
    "currency": "USD",
    "lastAssessed": "2025-12-18T10:35:00Z"
  },
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9/assets/FA-2025-BTC-001",
    "registry": "/registries/AR-2025-1734519000-A7F3C9",
    "transactions": "/assets/FA-2025-BTC-001/transactions"
  }
}
```

#### 3.2.2 자산 세부정보 조회

```http
GET /assets/{assetId}
Authorization: Bearer {access_token}
```

#### 3.2.3 자산 목록 조회

```http
GET /registries/{registryId}/assets?type=financial&category=cryptocurrency&limit=20&offset=0
Authorization: Bearer {access_token}
```

**응답 (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "total": 3,
  "limit": 20,
  "offset": 0,
  "assets": [
    {
      "assetId": "FA-2025-BTC-001",
      "assetType": "financial",
      "category": "cryptocurrency",
      "name": "비트코인 보유분",
      "value": 523456.78,
      "currency": "USD",
      "status": "active"
    },
    {
      "assetId": "FA-2025-ETH-001",
      "assetType": "financial",
      "category": "cryptocurrency",
      "name": "이더리움 보유분",
      "value": 367890.12,
      "currency": "USD",
      "status": "active"
    }
  ]
}
```

### 3.3 트랜잭션 엔드포인트

#### 3.3.1 트랜잭션 생성

```http
POST /transactions
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "assetId": "FA-2025-BTC-001",
  "type": "transfer",
  "action": "sell_partial",
  "amount": {
    "value": "2.0",
    "unit": "BTC"
  },
  "purpose": "preservation_expenses",
  "description": "연간 보존 및 수탁 수수료를 위해 2 BTC 청산",
  "requiredSignatures": 3,
  "recipient": {
    "type": "bank_account",
    "accountId": "BANK-2025-001",
    "accountName": "보존 비용 계정"
  }
}
```

**응답 (201 Created):**

```json
{
  "transactionId": "TX-1734519600-A7B8C9D0E1F2",
  "assetId": "FA-2025-BTC-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "type": "transfer",
  "action": "sell_partial",
  "status": "pending_signatures",
  "created": "2025-12-18T11:00:00Z",
  "amount": {
    "value": "2.0",
    "unit": "BTC",
    "estimatedUSD": 84000.00
  },
  "requiredSignatures": 3,
  "currentSignatures": 0,
  "signingDeadline": "2025-12-21T11:00:00Z"
}
```

#### 3.3.2 트랜잭션 서명

```http
POST /transactions/{transactionId}/sign
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "signerId": "CUST-001",
  "signerRole": "primary_custodian",
  "signature": "3045022100f7a9c8b6d4e2f0a8c6b4d2e0f9a7c5b3d1e9f7a5c3b1d9e7f5a3c1b9e7f5a3c10220a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2",
  "publicKey": "04a7b1c3d5e7f9a2b4c6d8e0f1a3b5c7d9e1f3a5b7c9d1e3f5a7b9c1d3e5f7a9..."
}
```

**응답 (200 OK):**

```json
{
  "transactionId": "TX-1734519600-A7B8C9D0E1F2",
  "status": "pending_signatures",
  "currentSignatures": 1,
  "requiredSignatures": 3,
  "signatures": [
    {
      "signerId": "CUST-001",
      "signerRole": "primary_custodian",
      "signerName": "신탁회사 알파",
      "signedAt": "2025-12-18T11:15:00Z",
      "verified": true
    }
  ]
}
```

#### 3.3.3 트랜잭션 상태 조회

```http
GET /transactions/{transactionId}
Authorization: Bearer {access_token}
```

### 3.4 수탁자 관리 엔드포인트

#### 3.4.1 수탁자 추가

```http
POST /registries/{registryId}/custodians
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "type": "backup",
  "name": "가족 신탁 서비스",
  "contact": {
    "email": "trustees@familytrust.com",
    "phone": "+82-2-1234-5678"
  },
  "credentials": {
    "licenseNumber": "SEOUL-TRUST-567890",
    "bondAmount": 1000000000,
    "currency": "KRW"
  }
}
```

#### 3.4.2 수탁자 세부정보 조회

```http
GET /custodians/{custodianId}
Authorization: Bearer {access_token}
```

### 3.5 소생 검증 엔드포인트

#### 3.5.1 의료 검증 제출

```http
POST /registries/{registryId}/revival/medical-verification
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "medicalDirector": {
    "name": "Dr. Sarah Chen, MD",
    "license": "AZ-MED-123456",
    "facility": "Alcor Life Extension Foundation"
  },
  "revivalDate": "2075-06-15T08:30:00Z",
  "vitalSigns": {
    "heartRate": 72,
    "bloodPressure": "120/80",
    "temperature": 36.8,
    "respiratoryRate": 16,
    "oxygenSaturation": 98
  },
  "neurologicalAssessment": {
    "consciousness": "alert_and_oriented",
    "glasgowComaScale": 15,
    "pupilResponse": "normal"
  },
  "medicalCertification": {
    "statement": "극저온 보존으로부터 안정적인 활력 징후와 회복 중인 신경 기능으로 환자가 성공적으로 소생됨",
    "signature": "-----BEGIN SIGNATURE-----\n...",
    "timestamp": "2075-06-15T12:00:00Z"
  }
}
```

**응답 (201 Created):**

```json
{
  "verificationId": "VERIFY-MED-2075-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "type": "medical",
  "status": "verified",
  "submittedBy": "Dr. Sarah Chen, MD",
  "submittedAt": "2075-06-15T12:00:00Z",
  "verifiedAt": "2075-06-15T12:00:00Z"
}
```

#### 3.5.2 법률 검증 제출

```http
POST /registries/{registryId}/revival/legal-verification
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "court": {
    "name": "서울중앙지방법원",
    "jurisdiction": "대한민국",
    "caseNumber": "2075가합12345"
  },
  "judge": {
    "name": "김정의 판사",
    "title": "지방법원 판사"
  },
  "orderDetails": {
    "orderType": "법적 지위 회복",
    "orderDate": "2075-06-20T10:00:00Z",
    "findings": [
      "극저온 보존으로부터 개인이 성공적으로 소생됨",
      "의료 증거가 안정적인 상태를 확인함",
      "생체인식 비교를 통해 신원 확인됨"
    ]
  }
}
```

#### 3.5.3 생체인식 검증 제출

```http
POST /registries/{registryId}/revival/biometric-verification
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "verificationAuthority": {
    "name": "WIA 생체인식 인증센터",
    "certificationId": "WIA-BIO-CERT-001"
  },
  "biometricSamples": [
    {
      "type": "fingerprint",
      "samples": 10,
      "quality": 94,
      "matchScore": 98.7,
      "matched": true
    },
    {
      "type": "iris",
      "samples": 4,
      "quality": 96,
      "matchScore": 99.2,
      "matched": true
    },
    {
      "type": "dna",
      "samples": 1,
      "quality": 99,
      "matchScore": 99.99,
      "matched": true
    }
  ],
  "overallMatchScore": 99.3,
  "verificationResult": "positive_match"
}
```

#### 3.5.4 소생 상태 조회

```http
GET /registries/{registryId}/revival/status
Authorization: Bearer {access_token}
```

**응답 (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "revivalStatus": "verified_pending_transfer",
  "verifications": {
    "medical": {
      "status": "verified",
      "verificationId": "VERIFY-MED-2075-001",
      "verifiedAt": "2075-06-15T12:00:00Z"
    },
    "legal": {
      "status": "verified",
      "verificationId": "VERIFY-LEGAL-2075-001",
      "verifiedAt": "2075-06-20T14:00:00Z"
    },
    "biometric": {
      "status": "verified",
      "verificationId": "VERIFY-BIO-2075-001",
      "verifiedAt": "2075-06-18T10:00:00Z"
    }
  },
  "transferEligibility": {
    "eligible": true,
    "requiredVerifications": 3,
    "completedVerifications": 3
  }
}
```

#### 3.5.5 자산 이전 시작

```http
POST /registries/{registryId}/revival/transfer
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "transferType": "full_revival_transfer",
  "recipient": {
    "individualId": "CRYO-IND-2025-987654",
    "legalName": "홍길동",
    "verifiedIdentity": true
  },
  "authorization": {
    "medicalDirector": "VERIFY-MED-2075-001",
    "legalAuthority": "VERIFY-LEGAL-2075-001",
    "biometricAuthority": "VERIFY-BIO-2075-001",
    "primaryCustodian": "CUST-001"
  }
}
```

**응답 (202 Accepted):**

```json
{
  "transferId": "TRANSFER-2075-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "status": "processing",
  "initiated": "2075-06-25T10:00:00Z",
  "estimatedCompletion": "2075-09-20T23:59:59Z",
  "assets": {
    "total": 71,
    "transferred": 0,
    "pending": 71
  }
}
```

### 3.6 평가 엔드포인트

#### 3.6.1 자산 평가 조회

```http
GET /assets/{assetId}/valuation
Authorization: Bearer {access_token}
```

#### 3.6.2 포트폴리오 평가 조회

```http
GET /registries/{registryId}/valuation
Authorization: Bearer {access_token}
```

### 3.7 감사 및 보고 엔드포인트

#### 3.7.1 감사 로그 조회

```http
GET /registries/{registryId}/audit?startDate=2025-12-01&endDate=2025-12-18&limit=50
Authorization: Bearer {access_token}
```

#### 3.7.2 보고서 생성

```http
POST /registries/{registryId}/reports
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "reportType": "annual_custodian_report",
  "period": {
    "startDate": "2025-01-01",
    "endDate": "2025-12-31"
  },
  "sections": [
    "asset_summary",
    "valuation_changes",
    "transactions",
    "income_and_expenses"
  ],
  "format": "pdf"
}
```

## 4. Webhook

### 4.1 Webhook 이벤트

| 이벤트 유형 | 설명 | 페이로드 |
|-----------|-----|---------|
| `registry.created` | 새 자산 등록부 생성됨 | Registry 객체 |
| `asset.created` | 새 자산 추가됨 | Asset 객체 |
| `transaction.created` | 새 트랜잭션 시작됨 | Transaction 객체 |
| `transaction.executed` | 트랜잭션 완료됨 | Transaction + 결과 |
| `revival.medical_verified` | 의료 검증 제출됨 | Verification 객체 |
| `revival.transfer_initiated` | 자산 이전 시작됨 | Transfer 객체 |

### 4.2 Webhook 구성

```http
POST /webhooks
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "url": "https://custodian.example.com/webhooks/wia-cryo-asset",
  "events": [
    "transaction.created",
    "transaction.executed",
    "revival.medical_verified"
  ],
  "secret": "whsec_a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6"
}
```

## 5. 오류 처리

### 5.1 오류 응답 형식

```json
{
  "error": {
    "code": "insufficient_signatures",
    "message": "트랜잭션에 3개의 서명이 필요하지만 2개만 제공됨",
    "details": {
      "requiredSignatures": 3,
      "currentSignatures": 2,
      "missingRoles": ["legal_representative"]
    },
    "requestId": "req_a1b2c3d4e5f6",
    "timestamp": "2025-12-18T15:00:00Z"
  }
}
```

### 5.2 오류 코드

| HTTP 상태 | 오류 코드 | 설명 |
|----------|---------|------|
| 400 | `invalid_request` | 잘못된 요청 본문 또는 매개변수 |
| 401 | `unauthorized` | 누락되거나 유효하지 않은 인증 |
| 403 | `forbidden` | 권한 부족 |
| 403 | `insufficient_signatures` | 작업에 대한 서명 부족 |
| 404 | `resource_not_found` | 요청한 리소스가 존재하지 않음 |
| 429 | `rate_limit_exceeded` | 요청이 너무 많음 |
| 500 | `internal_error` | 서버 오류 |

## 6. 속도 제한

### 6.1 속도 제한 헤더

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1734523200
```

### 6.2 속도 제한 계층

| 계층 | 시간당 요청 | 버스트 제한 | 사용 사례 |
|-----|-----------|-----------|----------|
| 무료 | 100 | 10 | 테스트, 개발 |
| 표준 | 1,000 | 50 | 소규모 수탁자 |
| 전문가 | 10,000 | 200 | 중규모 수탁자 |
| 기업 | 100,000 | 1,000 | 대형 기관 |

## 7. 코드 예제

### 7.1 Python 클라이언트

```python
import requests
import json

class WIACryoAssetClient:
    """WIA 극저온 자산 API 클라이언트"""

    def __init__(self, api_key, api_secret, base_url="https://api.wia.dev/cryo-asset/v1"):
        self.api_key = api_key
        self.api_secret = api_secret
        self.base_url = base_url
        self.access_token = None

    def authenticate(self):
        """OAuth 2.0 인증"""
        response = requests.post(
            f"{self.base_url}/oauth/token",
            json={
                "grant_type": "client_credentials",
                "client_id": self.api_key,
                "client_secret": self.api_secret,
                "scope": "asset:read asset:write transaction:create"
            }
        )
        response.raise_for_status()
        data = response.json()
        self.access_token = data['access_token']
        return self.access_token

    def _headers(self):
        """요청 헤더 생성"""
        if not self.access_token:
            self.authenticate()
        return {
            "Authorization": f"Bearer {self.access_token}",
            "Content-Type": "application/json"
        }

    def create_registry(self, subject_data, preservation_details):
        """등록부 생성"""
        response = requests.post(
            f"{self.base_url}/registries",
            headers=self._headers(),
            json={
                "subject": subject_data,
                "preservationDetails": preservation_details
            }
        )
        response.raise_for_status()
        return response.json()

    def add_crypto_asset(self, registry_id, crypto_data):
        """암호화폐 자산 추가"""
        response = requests.post(
            f"{self.base_url}/registries/{registry_id}/assets/financial",
            headers=self._headers(),
            json={
                "category": "cryptocurrency",
                "asset": crypto_data
            }
        )
        response.raise_for_status()
        return response.json()

# 사용 예제
client = WIACryoAssetClient(
    api_key="cryo_custodian_12345",
    api_secret="sk_REPLACE_WITH_YOUR_API_KEY_PLACEHOLDER"
)

# 등록부 생성
registry = client.create_registry(
    subject_data={
        "individualId": "CRYO-IND-2025-987654",
        "legalName": "홍길동",
        "dateOfBirth": "1985-06-15"
    },
    preservation_details={
        "facility": "Alcor Life Extension Foundation",
        "facilityId": "ALCOR-AZ-001",
        "preservationDate": "2025-12-20T14:00:00Z"
    }
)

print(f"등록부 생성됨: {registry['registryId']}")
```

### 7.2 JavaScript/Node.js 클라이언트

```javascript
const axios = require('axios');

class WIACryoAssetClient {
    constructor(apiKey, apiSecret, baseURL = 'https://api.wia.dev/cryo-asset/v1') {
        this.apiKey = apiKey;
        this.apiSecret = apiSecret;
        this.baseURL = baseURL;
        this.accessToken = null;
    }

    async authenticate() {
        const response = await axios.post(`${this.baseURL}/oauth/token`, {
            grant_type: 'client_credentials',
            client_id: this.apiKey,
            client_secret: this.apiSecret,
            scope: 'asset:read asset:write transaction:create'
        });

        this.accessToken = response.data.access_token;
        return this.accessToken;
    }

    async _getHeaders() {
        if (!this.accessToken) {
            await this.authenticate();
        }
        return {
            'Authorization': `Bearer ${this.accessToken}`,
            'Content-Type': 'application/json'
        };
    }

    async getAsset(assetId) {
        const headers = await this._getHeaders();
        const response = await axios.get(
            `${this.baseURL}/assets/${assetId}`,
            { headers }
        );
        return response.data;
    }
}

// 사용 예제
(async () => {
    const client = new WIACryoAssetClient(
        'cryo_custodian_12345',
        'sk_REPLACE_WITH_YOUR_API_KEY_PLACEHOLDER'
    );

    const asset = await client.getAsset('FA-2025-BTC-001');
    console.log(`자산: ${asset.asset.name}`);
    console.log(`가치: $${asset.valuation.totalValue}`);
})();
```

---

**弘益人間 (홍익인간)** - 인류에 이로움
© 2025 WIA
MIT 라이선스
