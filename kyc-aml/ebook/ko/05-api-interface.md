# 5장: API 인터페이스 사양

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

- WIA KYC/AML API의 RESTful 설계 원칙 이해
- 인증 메커니즘 (OAuth 2.0 및 API 키) 구현
- 고객 관리, 신원 확인, 스크리닝 엔드포인트 통합
- 거래 모니터링 및 사례 관리 API 활용
- 웹훅을 통한 실시간 이벤트 처리
- 속도 제한 및 오류 처리 적용
- TypeScript 및 Python SDK 사용

---

## 개요

이 장에서는 신원 확인, 스크리닝, 위험 평가, 거래 모니터링 및 사례 관리를 위한 엔드포인트를 제공하는 WIA KYC/AML 표준의 RESTful API 설계를 자세히 설명합니다.

---

## API 설계 원칙

### 1. RESTful 아키텍처

- **리소스 지향 URL**: 각 엔드포인트는 특정 리소스를 나타냄
- **HTTP 메서드 의미론**: GET(조회), POST(생성), PUT(전체 업데이트), PATCH(부분 업데이트), DELETE(삭제)
- **무상태 요청**: 각 요청은 독립적이며 모든 필요한 정보를 포함
- **HATEOAS 링크**: 검색 가능성을 위한 하이퍼미디어 링크

### 2. 일관된 패턴

- **표준 오류 응답**: 모든 오류는 동일한 형식 사용
- **균일한 페이지네이션**: 모든 목록 엔드포인트에서 일관된 페이징
- **공통 필터링 및 정렬**: 예측 가능한 쿼리 매개변수
- **예측 가능한 명명 규칙**: camelCase 일관성

### 3. 보안 우선

- **OAuth 2.0 / API 키 인증**: 안전한 접근 제어
- **TLS 1.3 암호화**: 전송 중 데이터 보호
- **속도 제한**: 남용 방지
- **요청 검증**: 모든 입력 철저히 검증

### 4. 개발자 친화적

- **명확한 문서**: 모든 엔드포인트 상세 설명
- **예제 요청/응답**: 실제 사용 사례 제공
- **여러 언어의 SDK**: TypeScript, Python, Java 등
- **샌드박스 환경**: 위험 없이 테스트

---

## 기본 구성

### 기본 URL

```
프로덕션: https://api.wia-kyc.org/v1
샌드박스: https://sandbox-api.wia-kyc.org/v1
```

### 인증

#### OAuth 2.0 (권장)

OAuth 2.0 클라이언트 자격 증명 플로우:

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET&
scope=kyc:read kyc:write
```

**응답:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "kyc:read kyc:write"
}
```

#### API 키 (대안)

간단한 API 키 인증:

```http
GET /customers/CUST-123456
Authorization: ApiKey YOUR_API_KEY
```

### 요청 헤더

모든 API 요청에 포함해야 하는 표준 헤더:

```http
Authorization: Bearer {access_token}
Content-Type: application/json
Accept: application/json
X-Request-ID: {unique-request-id}
X-Idempotency-Key: {idempotency-key}
```

**헤더 설명:**
- `Authorization`: 인증 토큰 (필수)
- `Content-Type`: 요청 본문 형식 (POST/PUT/PATCH)
- `Accept`: 응답 형식 (일반적으로 application/json)
- `X-Request-ID`: 요청 추적을 위한 고유 ID (권장)
- `X-Idempotency-Key`: 중복 요청 방지 (POST 요청에 권장)

### 표준 응답 형식

#### 성공 응답

```json
{
  "status": "success",
  "data": {
    // 리소스 데이터
  },
  "metadata": {
    "requestId": "req-123456",
    "timestamp": "2025-01-09T10:30:00Z",
    "version": "1.0"
  }
}
```

#### 오류 응답

```json
{
  "status": "error",
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "잘못된 날짜 형식",
    "details": [
      {
        "field": "dateOfBirth",
        "issue": "YYYY-MM-DD 형식이어야 함"
      }
    ]
  },
  "metadata": {
    "requestId": "req-123456",
    "timestamp": "2025-01-09T10:30:00Z"
  }
}
```

---

## 고객 관리 API

### 고객 생성

새 고객 프로필을 생성합니다.

```http
POST /customers
```

#### 요청 본문 (개인 고객)

```json
{
  "type": "individual",
  "personalInfo": {
    "firstName": "John",
    "middleName": "Michael",
    "lastName": "Smith",
    "dateOfBirth": "1985-06-15",
    "nationality": ["USA"]
  },
  "contactInfo": {
    "email": {
      "address": "john.smith@example.com"
    },
    "phone": {
      "number": "+1-555-0123"
    }
  },
  "addresses": [
    {
      "type": "residential",
      "street": "123 Main St",
      "city": "San Francisco",
      "state": "CA",
      "postalCode": "94102",
      "country": "USA",
      "primary": true
    }
  ]
}
```

#### 응답: `201 Created`

```json
{
  "status": "success",
  "data": {
    "customerId": "CUST-789012",
    "type": "individual",
    "status": "pending_verification",
    "createdAt": "2025-01-09T10:30:00Z",
    "personalInfo": {
      "firstName": "John",
      "middleName": "Michael",
      "lastName": "Smith",
      "dateOfBirth": "1985-06-15",
      "nationality": ["USA"]
    },
    "contactInfo": {
      "email": {
        "address": "john.smith@example.com",
        "verified": false
      },
      "phone": {
        "number": "+1-555-0123",
        "verified": false
      }
    },
    "addresses": [
      {
        "type": "residential",
        "street": "123 Main St",
        "city": "San Francisco",
        "state": "CA",
        "postalCode": "94102",
        "country": "USA",
        "primary": true
      }
    ]
  }
}
```

### 고객 조회

기존 고객의 세부 정보를 조회합니다.

```http
GET /customers/{customerId}
```

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": {
    "customerId": "CUST-789012",
    "type": "individual",
    "status": "active",
    "createdAt": "2025-01-09T10:30:00Z",
    "personalInfo": {
      // ... 전체 고객 프로필
    },
    "riskProfile": {
      "category": "low",
      "score": 35,
      "lastAssessment": "2025-01-09T10:35:00Z"
    }
  }
}
```

### 고객 업데이트

고객 정보를 부분적으로 업데이트합니다.

```http
PATCH /customers/{customerId}
```

#### 요청 본문

```json
{
  "contactInfo": {
    "phone": {
      "number": "+1-555-9999"
    }
  }
}
```

#### 응답: `200 OK`

업데이트된 고객 객체가 반환됩니다.

### 고객 목록 조회

필터링 및 페이지네이션과 함께 고객 목록을 조회합니다.

```http
GET /customers?status=active&riskCategory=high&page=1&limit=20
```

#### 쿼리 매개변수

- `status` - 상태별 필터링 (pending_verification, active, suspended, closed)
- `riskCategory` - 위험 범주별 필터링 (low, medium, high, prohibited)
- `createdAfter` - ISO 8601 날짜 형식
- `createdBefore` - ISO 8601 날짜 형식
- `page` - 페이지 번호 (기본값: 1)
- `limit` - 페이지당 항목 수 (기본값: 20, 최대: 100)
- `sort` - 정렬 필드 (예: `createdAt`, `-riskScore`)

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": [
    {
      "customerId": "CUST-789012",
      "type": "individual",
      "status": "active",
      "personalInfo": {
        "firstName": "John",
        "lastName": "Smith"
      },
      "riskProfile": {
        "category": "high",
        "score": 75
      }
    }
  ],
  "pagination": {
    "currentPage": 1,
    "pageSize": 20,
    "totalItems": 150,
    "totalPages": 8,
    "links": {
      "self": "/customers?page=1&limit=20",
      "next": "/customers?page=2&limit=20",
      "last": "/customers?page=8&limit=20"
    }
  }
}
```

---

## 신원 확인 API

### 확인 시작

새 신원 확인 세션을 시작합니다.

```http
POST /identity/verifications
```

#### 요청 본문

```json
{
  "customerId": "CUST-789012",
  "verificationType": "document_and_biometric",
  "returnUrl": "https://yourapp.com/verification/complete",
  "webhookUrl": "https://yourapp.com/webhooks/verification"
}
```

**verificationType 옵션:**
- `email_phone` - 이메일 및 전화 확인만
- `document` - 문서 확인만
- `document_and_biometric` - 문서 및 생체 인식 확인 (권장)
- `enhanced` - 강화된 확인 (비디오 또는 대면 포함)

#### 응답: `201 Created`

```json
{
  "status": "success",
  "data": {
    "verificationId": "IDV-2025-001234",
    "customerId": "CUST-789012",
    "status": "pending",
    "verificationType": "document_and_biometric",
    "createdAt": "2025-01-09T10:10:00Z",
    "sessionUrl": "https://verify.wia-kyc.org/session/abc123def456",
    "expiresAt": "2025-01-09T11:10:00Z"
  }
}
```

### 문서 업로드

신원 확인을 위한 문서를 업로드합니다.

```http
POST /identity/verifications/{verificationId}/documents
Content-Type: multipart/form-data
```

#### 폼 데이터

- `documentType` - passport, drivers_license, national_id, residence_permit
- `documentSide` - front, back
- `file` - 이미지 파일 (JPEG, PNG, 최대 10MB)

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": {
    "documentId": "DOC-001",
    "uploadedAt": "2025-01-09T10:11:00Z",
    "status": "processing"
  }
}
```

### 생체 인식 제출

얼굴 인식 생체 인식 데이터를 제출합니다.

```http
POST /identity/verifications/{verificationId}/biometric
```

#### 요청 본문

```json
{
  "biometricType": "facial",
  "imageData": "base64_encoded_image_data",
  "livenessVideo": "base64_encoded_video_data"
}
```

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": {
    "biometricId": "BIO-FACE-001",
    "status": "processing",
    "submittedAt": "2025-01-09T10:15:00Z"
  }
}
```

### 확인 결과 조회

완료된 확인의 결과를 조회합니다.

```http
GET /identity/verifications/{verificationId}
```

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": {
    "verificationId": "IDV-2025-001234",
    "customerId": "CUST-789012",
    "status": "verified",
    "overallResult": "pass",
    "confidenceScore": 0.94,
    "completedAt": "2025-01-09T10:15:00Z",
    "documentVerification": {
      "documentType": "passport",
      "documentNumber": "P12345678",
      "issueDate": "2020-01-15",
      "expiryDate": "2030-01-15",
      "issuingCountry": "USA",
      "authenticity": {
        "result": "authentic",
        "confidence": 0.96,
        "checks": {
          "hologram": "pass",
          "microtext": "pass",
          "securityFeatures": "pass"
        }
      },
      "dataExtraction": {
        "firstName": "John",
        "lastName": "Smith",
        "dateOfBirth": "1985-06-15",
        "nationality": "USA"
      }
    },
    "biometricVerification": {
      "livenessCheck": {
        "result": "pass",
        "confidence": 0.92
      },
      "faceMatch": {
        "result": "match",
        "confidence": 0.94,
        "comparisonScore": 0.94
      }
    }
  }
}
```

---

## 스크리닝 API

### 종합 스크리닝 수행

제재, PEP, 부정적 미디어에 대한 종합 스크리닝을 수행합니다.

```http
POST /screening/comprehensive
```

#### 요청 본문

```json
{
  "customerId": "CUST-789012",
  "screeningType": ["sanctions", "pep", "adverse_media"],
  "fuzzyMatching": true,
  "threshold": 85,
  "continuousMonitoring": true
}
```

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": {
    "screeningId": "SCR-2025-001234",
    "customerId": "CUST-789012",
    "executedAt": "2025-01-09T10:12:00Z",
    "sanctionsScreening": {
      "status": "no_match",
      "listsChecked": ["OFAC_SDN", "UN_SC", "EU_SANCTIONS"],
      "totalRecordsChecked": 25614,
      "lastUpdated": "2025-01-09T00:00:00Z"
    },
    "pepScreening": {
      "status": "potential_match",
      "matches": [
        {
          "matchId": "PEP-2025-5678",
          "confidence": 78,
          "requiresReview": true,
          "profile": {
            "name": "John M. Smith",
            "position": "City Council Member",
            "country": "USA",
            "tier": 2,
            "source": "Government Registry",
            "dateIdentified": "2023-05-10"
          }
        }
      ]
    },
    "adverseMediaScreening": {
      "status": "clear",
      "articlesReviewed": 0,
      "lastSearchDate": "2025-01-09T10:12:00Z"
    },
    "overallRisk": "low_risk_pending_review",
    "recommendedAction": "manual_review_pep_match"
  }
}
```

### 제재 스크리닝만

제재 목록에 대해서만 스크리닝합니다.

```http
POST /screening/sanctions
```

#### 요청 본문

```json
{
  "name": "John Michael Smith",
  "dateOfBirth": "1985-06-15",
  "nationality": "USA",
  "lists": ["OFAC_SDN", "UN_SC", "EU_SANCTIONS"]
}
```

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": {
    "screeningId": "SCR-SANC-001234",
    "status": "no_match",
    "executedAt": "2025-01-09T10:12:00Z",
    "listsChecked": [
      {
        "listName": "OFAC_SDN",
        "listVersion": "2025-01-09",
        "recordCount": 12458,
        "matches": 0
      },
      {
        "listName": "UN_SC",
        "listVersion": "2025-01-08",
        "recordCount": 7892,
        "matches": 0
      },
      {
        "listName": "EU_SANCTIONS",
        "listVersion": "2025-01-09",
        "recordCount": 5264,
        "matches": 0
      }
    ]
  }
}
```

### PEP 스크리닝만

정치적 노출인물(PEP)에 대해서만 스크리닝합니다.

```http
POST /screening/pep
```

#### 요청 본문

```json
{
  "customerId": "CUST-789012",
  "name": "John Michael Smith",
  "dateOfBirth": "1985-06-15",
  "nationality": "USA"
}
```

#### 응답: `200 OK`

PEP 스크리닝 결과를 포함한 응답.

### 배치 스크리닝

여러 고객에 대한 스크리닝을 배치로 제출합니다.

```http
POST /screening/batch
```

#### 요청 본문

```json
{
  "screeningType": ["sanctions", "pep"],
  "customers": [
    {
      "customerId": "CUST-001",
      "name": "John Smith",
      "dateOfBirth": "1985-06-15"
    },
    {
      "customerId": "CUST-002",
      "name": "Jane Doe",
      "dateOfBirth": "1990-03-20"
    }
  ]
}
```

#### 응답: `202 Accepted`

```json
{
  "status": "success",
  "data": {
    "batchId": "BATCH-2025-001",
    "status": "processing",
    "totalRecords": 2,
    "estimatedCompletionTime": "2025-01-09T10:20:00Z",
    "statusUrl": "/screening/batch/BATCH-2025-001"
  }
}
```

---

## 위험 평가 API

### 고객 위험 평가

고객에 대한 종합적인 위험 평가를 수행합니다.

```http
POST /risk/assess
```

#### 요청 본문

```json
{
  "customerId": "CUST-789012",
  "assessmentType": "comprehensive",
  "includeFactors": true
}
```

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": {
    "assessmentId": "RISK-2025-001234",
    "customerId": "CUST-789012",
    "assessmentDate": "2025-01-09T10:30:00Z",
    "overallRisk": {
      "score": 35,
      "category": "low"
    },
    "riskDimensions": [
      {
        "dimension": "geographic",
        "score": 20,
        "weight": 0.25,
        "factors": [
          {
            "factor": "residence_country",
            "value": "USA",
            "riskLevel": "low",
            "contribution": 10
          },
          {
            "factor": "citizenship",
            "value": "USA",
            "riskLevel": "low",
            "contribution": 10
          }
        ]
      },
      {
        "dimension": "product",
        "score": 30,
        "weight": 0.20,
        "factors": [
          {
            "factor": "account_type",
            "value": "checking",
            "riskLevel": "low",
            "contribution": 15
          },
          {
            "factor": "international_wire",
            "value": "enabled",
            "riskLevel": "medium",
            "contribution": 15
          }
        ]
      },
      {
        "dimension": "customerType",
        "score": 25,
        "weight": 0.20,
        "factors": [
          {
            "factor": "customer_category",
            "value": "individual",
            "riskLevel": "low",
            "contribution": 15
          },
          {
            "factor": "pep_status",
            "value": "not_pep",
            "riskLevel": "low",
            "contribution": 10
          }
        ]
      },
      {
        "dimension": "behavioral",
        "score": 40,
        "weight": 0.20,
        "factors": [
          {
            "factor": "source_of_funds",
            "value": "employment_income",
            "riskLevel": "low",
            "contribution": 20
          },
          {
            "factor": "transaction_patterns",
            "value": "consistent",
            "riskLevel": "low",
            "contribution": 20
          }
        ]
      },
      {
        "dimension": "relationship",
        "score": 15,
        "weight": 0.15,
        "factors": [
          {
            "factor": "screening_results",
            "value": "clear",
            "riskLevel": "low",
            "contribution": 10
          },
          {
            "factor": "adverse_media",
            "value": "none",
            "riskLevel": "low",
            "contribution": 5
          }
        ]
      }
    ],
    "recommendations": {
      "approvalDecision": "approve",
      "cddLevel": "standard",
      "reviewFrequency": "biennial",
      "nextReviewDate": "2027-01-09",
      "transactionLimits": {
        "dailyWithdrawal": 10000,
        "monthlyInternationalWire": 50000
      }
    }
  }
}
```

### 위험 점수 조회

고객의 현재 위험 점수를 조회합니다.

```http
GET /risk/score/{customerId}
```

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": {
    "customerId": "CUST-789012",
    "riskScore": 35,
    "riskCategory": "low",
    "lastAssessment": "2025-01-09T10:30:00Z",
    "nextReview": "2027-01-09"
  }
}
```

### 위험 매개변수 업데이트

위험 평가 매개변수를 업데이트합니다.

```http
PATCH /risk/parameters
```

#### 요청 본문

```json
{
  "dimensionWeights": {
    "geographic": 0.25,
    "product": 0.20,
    "customerType": 0.20,
    "behavioral": 0.20,
    "relationship": 0.15
  },
  "riskThresholds": {
    "low": 39,
    "medium": 69,
    "high": 89
  }
}
```

#### 응답: `200 OK`

업데이트된 매개변수가 반환됩니다.

---

## 거래 모니터링 API

### 거래 제출

모니터링을 위해 거래를 제출합니다.

```http
POST /monitoring/transactions
```

#### 요청 본문

```json
{
  "transactionId": "TXN-001-20250109",
  "customerId": "CUST-789012",
  "accountId": "ACC-CUST-789012-001",
  "timestamp": "2025-01-09T10:00:00Z",
  "type": "wire_credit",
  "amount": 45000,
  "currency": "USD",
  "counterparty": {
    "name": "ABC Trading Ltd",
    "account": "XXXX-5678",
    "bank": "Foreign Bank Corp",
    "country": "SGP"
  },
  "description": "Payment for consulting services"
}
```

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": {
    "transactionId": "TXN-001-20250109",
    "processed": true,
    "alerts": [],
    "riskScore": 25
  }
}
```

### 경보 조회

모니터링 경보를 조회합니다.

```http
GET /monitoring/alerts?status=open&priority=high&page=1&limit=20
```

#### 쿼리 매개변수

- `status` - open, under_investigation, closed
- `priority` - low, medium, high, critical
- `customerId` - 고객별 필터링
- `dateFrom` / `dateTo` - 날짜 범위
- `scenarioCode` - 시나리오별 필터링

#### 응답: `200 OK`

```json
{
  "status": "success",
  "data": [
    {
      "alertId": "ALERT-2025-123456",
      "customerId": "CUST-789012",
      "generatedAt": "2025-01-09T14:30:00Z",
      "status": "open",
      "priority": "high",
      "scenario": "rapid_funds_movement",
      "scenarioDescription": "Large deposit followed by quick withdrawal",
      "riskScore": 0.82,
      "transactions": [
        {
          "transactionId": "TXN-001",
          "amount": 50000,
          "type": "wire_credit",
          "timestamp": "2025-01-09T10:00:00Z"
        },
        {
          "transactionId": "TXN-002",
          "amount": 48000,
          "type": "wire_debit",
          "timestamp": "2025-01-09T14:00:00Z"
        }
      ]
    }
  ],
  "pagination": {
    "currentPage": 1,
    "pageSize": 20,
    "totalItems": 45,
    "totalPages": 3
  }
}
```

### 경보 세부 정보 조회

특정 경보의 세부 정보를 조회합니다.

```http
GET /monitoring/alerts/{alertId}
```

#### 응답: `200 OK`

전체 경보 세부 정보를 포함한 응답.

### 경보 처분

경보에 대한 결정을 기록합니다.

```http
POST /monitoring/alerts/{alertId}/dispose
```

#### 요청 본문

```json
{
  "disposition": "false_positive",
  "notes": "Transaction explained by legitimate business activity. Supporting documentation verified.",
  "disposedBy": "analyst_jdoe"
}
```

**disposition 옵션:**
- `false_positive` - 거짓 양성
- `true_positive` - 참 양성 (SAR 제출)
- `monitoring_continued` - 계속 모니터링
- `escalated` - 상위 단계로 에스컬레이션

#### 응답: `200 OK`

### 모니터링 규칙 구성

새 모니터링 규칙을 구성합니다.

```http
POST /monitoring/rules
```

#### 요청 본문

```json
{
  "ruleName": "High Value Cash Deposits",
  "ruleType": "threshold",
  "enabled": true,
  "condition": {
    "transactionType": "cash_deposit",
    "amount": {
      "greaterThan": 10000
    },
    "currency": "USD"
  },
  "action": "generate_alert",
  "severity": "medium"
}
```

#### 응답: `201 Created`

---

## 사례 관리 API

### 사례 생성

새 조사 사례를 생성합니다.

```http
POST /cases
```

#### 요청 본문

```json
{
  "caseType": "suspicious_activity_investigation",
  "customerId": "CUST-789012",
  "priority": "high",
  "trigger": {
    "type": "transaction_monitoring_alert",
    "sourceId": "ALERT-2025-123456"
  },
  "assignedTo": "analyst_jdoe"
}
```

#### 응답: `201 Created`

```json
{
  "status": "success",
  "data": {
    "caseId": "CASE-2025-001234",
    "caseType": "suspicious_activity_investigation",
    "customerId": "CUST-789012",
    "status": "under_investigation",
    "priority": "high",
    "createdAt": "2025-01-09T14:45:00Z",
    "assignedTo": "analyst_jdoe",
    "trigger": {
      "type": "transaction_monitoring_alert",
      "sourceId": "ALERT-2025-123456"
    }
  }
}
```

### 사례 조회

사례의 세부 정보를 조회합니다.

```http
GET /cases/{caseId}
```

#### 응답: `200 OK`

전체 사례 세부 정보를 포함한 응답.

### 조사 단계 추가

사례에 조사 활동을 추가합니다.

```http
POST /cases/{caseId}/investigation/steps
```

#### 요청 본문

```json
{
  "action": "customer_contact",
  "notes": "Spoke with customer. Provided documentation for transactions. Customer explained business purpose.",
  "performedBy": "analyst_jdoe",
  "duration": 900,
  "contactMethod": "phone",
  "outcome": "successful"
}
```

#### 응답: `201 Created`

### 증거 업로드

사례에 증거 문서를 업로드합니다.

```http
POST /cases/{caseId}/evidence
Content-Type: multipart/form-data
```

#### 폼 데이터

- `evidenceType` - supporting_document, external_search, communication
- `description` - 텍스트 설명
- `file` - 문서 파일

#### 응답: `201 Created`

### SAR 제출

의심스러운 활동 보고서(SAR)를 제출합니다.

```http
POST /cases/{caseId}/sar
```

#### 요청 본문

```json
{
  "suspiciousActivity": {
    "type": ["structuring", "suspected_money_laundering"],
    "description": "Detailed narrative of suspicious activity...",
    "dateRange": {
      "from": "2024-11-15",
      "to": "2024-12-15"
    },
    "totalAmount": 142500,
    "currency": "USD"
  },
  "transactions": [
    {
      "transactionId": "TXN-001",
      "date": "2024-11-15",
      "amount": 9500,
      "type": "cash_deposit"
    },
    {
      "transactionId": "TXN-002",
      "date": "2024-11-18",
      "amount": 9800,
      "type": "cash_deposit"
    }
  ],
  "narrative": "Over a period of 30 days, the customer made multiple cash deposits, each just below the $10,000 reporting threshold. The pattern suggests structuring to avoid CTR filing. Customer could not provide adequate explanation for the source of funds or business purpose. Total amount deposited was $142,500 across 15 transactions.",
  "filedBy": {
    "name": "Jane Investigator",
    "title": "Senior AML Analyst"
  }
}
```

#### 응답: `201 Created`

```json
{
  "status": "success",
  "data": {
    "sarId": "SAR-2025-001234",
    "caseId": "CASE-2025-001234",
    "filedAt": "2025-01-09T18:00:00Z",
    "status": "submitted",
    "confirmationNumber": "FINCEN-20250109-123456"
  }
}
```

### 사례 종료

조사를 완료하고 사례를 종료합니다.

```http
POST /cases/{caseId}/close
```

#### 요청 본문

```json
{
  "disposition": "closed_false_positive",
  "finalNotes": "Investigation determined activity legitimate. Customer provided supporting business documentation. Case closed.",
  "sarFiled": false
}
```

**disposition 옵션:**
- `closed_false_positive` - 거짓 양성으로 종료
- `closed_sar_filed` - SAR 제출 후 종료
- `closed_other_action` - 기타 조치 후 종료

#### 응답: `200 OK`

---

## 웹훅 이벤트

### 웹훅 구성

웹훅 엔드포인트를 구성합니다.

```http
POST /webhooks
```

#### 요청 본문

```json
{
  "url": "https://yourapp.com/webhooks/kyc",
  "events": [
    "verification.completed",
    "screening.match_found",
    "alert.generated",
    "case.status_changed"
  ],
  "secret": "your_webhook_secret"
}
```

#### 응답: `201 Created`

### 이벤트 유형

| 이벤트 | 설명 |
|--------|------|
| `customer.created` | 새 고객 생성 |
| `customer.updated` | 고객 프로필 업데이트 |
| `verification.completed` | 신원 확인 완료 |
| `verification.failed` | 확인 실패 |
| `screening.completed` | 스크리닝 완료 |
| `screening.match_found` | 잠재적 일치 발견 |
| `risk.assessment_completed` | 위험 평가 완료 |
| `risk.category_changed` | 위험 범주 변경 |
| `alert.generated` | 새 모니터링 경보 |
| `alert.escalated` | 경보 에스컬레이션 |
| `case.created` | 새 사례 생성 |
| `case.assigned` | 사례가 조사자에게 할당됨 |
| `case.status_changed` | 사례 상태 변경 |
| `sar.filed` | SAR 제출 |

### 웹훅 페이로드 예제

```json
{
  "eventId": "evt_123456",
  "eventType": "alert.generated",
  "timestamp": "2025-01-09T14:30:00Z",
  "data": {
    "alertId": "ALERT-2025-123456",
    "customerId": "CUST-789012",
    "priority": "high",
    "scenario": "rapid_funds_movement",
    "riskScore": 0.82
  },
  "signature": "sha256_hmac_signature"
}
```

### 웹훅 서명 검증

웹훅 페이로드를 HMAC-SHA256으로 검증:

```typescript
import crypto from 'crypto';

function verifyWebhookSignature(
  payload: string,
  signature: string,
  secret: string
): boolean {
  const expectedSignature = crypto
    .createHmac('sha256', secret)
    .update(payload)
    .digest('hex');

  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(expectedSignature)
  );
}
```

---

## 속도 제한

### 제한

| 등급 | 요청/분 | 버스트 |
|------|---------|-------|
| **무료** | 60 | 100 |
| **기본** | 300 | 500 |
| **프로페셔널** | 1,000 | 2,000 |
| **엔터프라이즈** | 맞춤형 | 맞춤형 |

### 헤더

모든 API 응답에 속도 제한 헤더가 포함됩니다:

```http
X-RateLimit-Limit: 300
X-RateLimit-Remaining: 287
X-RateLimit-Reset: 1704797400
```

### 속도 제한 오류

제한 초과 시:

**응답:** `429 Too Many Requests`
```json
{
  "status": "error",
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please retry after 30 seconds.",
    "retryAfter": 30
  }
}
```

---

## 오류 코드

| 코드 | HTTP 상태 | 설명 |
|------|-----------|------|
| `VALIDATION_ERROR` | 400 | 요청 검증 실패 |
| `AUTHENTICATION_ERROR` | 401 | 잘못되거나 누락된 인증 |
| `AUTHORIZATION_ERROR` | 403 | 권한 부족 |
| `NOT_FOUND` | 404 | 리소스를 찾을 수 없음 |
| `CONFLICT` | 409 | 리소스가 이미 존재함 |
| `RATE_LIMIT_EXCEEDED` | 429 | 요청이 너무 많음 |
| `INTERNAL_ERROR` | 500 | 서버 오류 |
| `SERVICE_UNAVAILABLE` | 503 | 서비스 일시적으로 사용 불가 |

---

## SDK 예제

### TypeScript/Node.js

#### 설치

```bash
npm install @wia/kyc-aml
```

#### 사용법

```typescript
import { WIAKYCClient } from '@wia/kyc-aml';

const client = new WIAKYCClient({
  apiKey: process.env.WIA_API_KEY,
  environment: 'production'
});

// 고객 생성
const customer = await client.customers.create({
  type: 'individual',
  personalInfo: {
    firstName: 'John',
    lastName: 'Smith',
    dateOfBirth: '1985-06-15'
  },
  contactInfo: {
    email: {
      address: 'john.smith@example.com'
    }
  }
});

console.log(`Created customer: ${customer.customerId}`);

// 확인 시작
const verification = await client.identity.verify({
  customerId: customer.customerId,
  verificationType: 'document_and_biometric'
});

console.log(`Verification URL: ${verification.sessionUrl}`);

// 스크리닝 수행
const screening = await client.screening.comprehensive({
  customerId: customer.customerId,
  screeningType: ['sanctions', 'pep', 'adverse_media']
});

console.log(`Screening status: ${screening.overallRisk}`);

// 위험 평가
const risk = await client.risk.assess({
  customerId: customer.customerId,
  assessmentType: 'comprehensive'
});

console.log(`Risk category: ${risk.overallRisk.category}`);
```

### Python

#### 설치

```bash
pip install wia-kyc-aml
```

#### 사용법

```python
from wia_kyc import WIAKYCClient
import os

client = WIAKYCClient(
    api_key=os.environ['WIA_API_KEY'],
    environment='production'
)

# 고객 생성
customer = client.customers.create(
    type='individual',
    personal_info={
        'first_name': 'John',
        'last_name': 'Smith',
        'date_of_birth': '1985-06-15'
    },
    contact_info={
        'email': {
            'address': 'john.smith@example.com'
        }
    }
)

print(f"Created customer: {customer.customer_id}")

# 확인 시작
verification = client.identity.verify(
    customer_id=customer.customer_id,
    verification_type='document_and_biometric'
)

print(f"Verification URL: {verification.session_url}")

# 스크리닝 수행
screening = client.screening.comprehensive(
    customer_id=customer.customer_id,
    screening_type=['sanctions', 'pep', 'adverse_media']
)

print(f"Screening status: {screening.overall_risk}")

# 위험 평가
risk = client.risk.assess(
    customer_id=customer.customer_id,
    assessment_type='comprehensive'
)

print(f"Risk category: {risk.overall_risk.category}")
```

### Java

#### Maven 종속성

```xml
<dependency>
  <groupId>org.wia</groupId>
  <artifactId>kyc-aml-sdk</artifactId>
  <version>1.0.0</version>
</dependency>
```

#### 사용법

```java
import org.wia.kyc.WIAKYCClient;
import org.wia.kyc.models.*;

public class KYCExample {
  public static void main(String[] args) {
    WIAKYCClient client = new WIAKYCClient.Builder()
      .apiKey(System.getenv("WIA_API_KEY"))
      .environment("production")
      .build();

    // 고객 생성
    Customer customer = client.customers().create(
      new CustomerRequest()
        .type("individual")
        .personalInfo(new PersonalInfo()
          .firstName("John")
          .lastName("Smith")
          .dateOfBirth("1985-06-15")
        )
    );

    System.out.println("Created customer: " + customer.getCustomerId());

    // 확인 시작
    Verification verification = client.identity().verify(
      new VerificationRequest()
        .customerId(customer.getCustomerId())
        .verificationType("document_and_biometric")
    );

    System.out.println("Verification URL: " + verification.getSessionUrl());
  }
}
```

---

## 복습 질문

1. WIA KYC/AML API의 네 가지 핵심 설계 원칙은 무엇입니까?
2. OAuth 2.0과 API 키 인증의 차이점은 무엇이며, 각각 언제 사용해야 합니까?
3. `X-Idempotency-Key` 헤더의 목적은 무엇입니까?
4. 고객 온보딩의 전체 워크플로를 설명하십시오 (생성에서 승인까지).
5. 종합 스크리닝에는 어떤 유형의 스크리닝이 포함됩니까?
6. 위험 평가에서 다섯 가지 위험 차원은 무엇입니까?
7. 거래 모니터링 경보는 어떻게 생성되고 처리됩니까?
8. 웹훅의 목적은 무엇이며, 웹훅 페이로드를 어떻게 검증합니까?
9. API 속도 제한을 어떻게 처리해야 합니까?
10. TypeScript SDK를 사용하여 완전한 고객 온보딩 흐름을 어떻게 구현합니까?

---

## 주요 요점

1. 🔌 **RESTful 설계** - 일관된 패턴과 리소스 지향 URL
2. 🔐 **OAuth 2.0 및 API 키** - 프로덕션에는 OAuth 2.0 권장
3. 📋 **포괄적인 엔드포인트** - 모든 KYC/AML 워크플로 커버
4. 🔔 **웹훅 지원** - 실시간 이벤트 알림
5. ⚡ **속도 제한** - 명확한 헤더 및 오류 처리
6. 📚 **다중 언어 SDK** - TypeScript, Python, Java 등
7. 🧪 **샌드박스 환경** - 안전한 테스트 환경

---

**이전**: [← 4장 - 데이터 형식](04-data-format.md) | **다음**: [6장 - 프로토콜 →](06-protocol.md)

---

© 2025 WIA Standards Committee
弘益人間 (홍익인간) - Benefit All Humanity
