# WIA Cryo-Consent Data Format Standard
## Phase 1 Specification

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
2. [용어 정의](#용어-정의)
3. [기본 구조](#기본-구조)
4. [데이터 스키마](#데이터-스키마)
5. [필드 명세](#필드-명세)
6. [데이터 타입](#데이터-타입)
7. [검증 규칙](#검증-규칙)
8. [예제](#예제)
9. [버전 이력](#버전-이력)

---

## 개요

### 1.1 목적

WIA Cryo-Consent Data Format Standard는 냉동보존 절차와 관련된 법적 동의 기록을 관리하기 위한 포괄적인 JSON 기반 형식을 정의합니다. 이 표준은 개인의 자율성과 법적 요구사항을 존중하면서 동의가 적절하게 문서화되고, 검증 가능하며, 여러 관할권에서 법적으로 집행 가능하도록 보장합니다.

**핵심 목표**:
- 전 세계적으로 냉동보존 절차에 대한 동의 문서를 표준화
- 다중 관할권 법적 준수 및 검증 지원
- 동의 수정, 철회 및 위임 워크플로 지원
- 암호화 검증 및 불변 감사 추적 보장
- 후견인 및 대리인 동의 관리 지원
- 기존 법적 및 의료 동의 프레임워크와 통합

### 1.2 범위

이 표준은 다음 동의 영역을 다룹니다:

| Domain | 설명 |
|--------|------|
| Primary Consent | 핵심 냉동보존 승인 및 범위 |
| Consent Scope | 보존, 연구 및 소생에 대한 구체적 권한 |
| Modification & Revocation | 동의 변경 및 철회 절차 |
| Proxy & Guardian Consent | 위임된 권한 및 법적 대리 |
| Multi-Jurisdiction Compliance | 국경 간 법적 프레임워크 정렬 |
| Audit & Verification | 암호화 증명 및 법적 검증 |

### 1.3 설계 원칙

1. **법적 유효성**: 지역 및 국제 동의법 준수
2. **불변성**: blockchain 준비 아키텍처로 변조 방지 동의 기록
3. **추적성**: 모든 동의 수정에 대한 완전한 감사 추적
4. **유연성**: 다양한 법적 프레임워크 및 문화적 요구사항 지원
5. **프라이버시**: GDPR, HIPAA 및 국제 프라이버시법 준수
6. **검증 가능성**: 암호화 서명 및 다자간 증인

---

## 용어 정의

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **Consent Grantor** | 냉동보존에 대한 동의를 제공하는 개인 |
| **Consent Scope** | 부여된 구체적 권한 (보존, 연구, 소생) |
| **Legal Guardian** | 동의 권한을 가진 법원 지정 대리인 |
| **Healthcare Proxy** | 의료 결정을 내릴 권한이 있는 지정된 개인 |
| **Revocation** | 이전에 부여된 동의의 공식적 철회 |
| **Modification** | 기존 동의 조건의 수정 |
| **Witnessing** | 권한 있는 당사자의 법적 증명 |
| **Jurisdiction** | 동의 유효성을 관할하는 법적 영역 |

### 2.2 데이터 타입

| Type | 설명 | 예제 |
|------|------|------|
| `string` | UTF-8 인코딩 텍스트 | `"CONSENT-2025-001"` |
| `number` | IEEE 754 배정밀도 | `1.0`, `0.5` |
| `integer` | 부호 있는 64비트 정수 | `1`, `100` |
| `boolean` | 불리언 값 | `true`, `false` |
| `timestamp` | ISO 8601 날짜시간 | `"2025-01-15T10:30:00Z"` |
| `uuid` | UUID v4 식별자 | `"550e8400-e29b-41d4-a716-446655440000"` |
| `signature` | 암호화 서명 | `"0x1234abcd..."` |

### 2.3 필드 요구사항

| 표시 | 의미 |
|------|------|
| **REQUIRED** | 필드가 반드시 있어야 함 |
| **OPTIONAL** | 필드를 생략할 수 있음 |
| **CONDITIONAL** | 특정 조건에서 필수 |

---

## 기본 구조

### 3.1 Message Format

모든 WIA Cryo-Consent 메시지는 다음 기본 구조를 따릅니다:

```json
{
  "$schema": "https://wia.live/cryo-consent/v1/schema.json",
  "version": "1.0.0",
  "consentId": "uuid-v4-string",
  "messageType": "consent_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z",
    "effectiveDate": "2025-01-15T00:00:00Z",
    "expirationDate": null
  },
  "grantor": {
    "id": "PERSON-001",
    "identityVerification": {
      "method": "biometric",
      "verifiedAt": "2025-01-15T10:00:00Z",
      "verifierId": "NOTARY-001"
    }
  },
  "consent": {
    "status": "active",
    "scope": {},
    "conditions": {},
    "jurisdiction": []
  },
  "legal": {
    "witnesses": [],
    "notarization": {},
    "jurisdiction": "US-CA"
  },
  "meta": {
    "hash": "sha256-hash",
    "signature": "digital-signature",
    "previousHash": "previous-record-hash",
    "version": 1
  }
}
```

### 3.2 필드 상세

#### 3.2.1 `consentId` (REQUIRED)

```
Type: string
Format: UUID v4
설명: 이 동의 기록의 고유 식별자
예제: "550e8400-e29b-41d4-a716-446655440000"
```

#### 3.2.2 `messageType` (REQUIRED)

```
Type: string
설명: 동의 메시지의 유형
유효한 값:
  - "consent_record"      : 초기 동의 문서화
  - "consent_modification": 기존 동의 수정
  - "consent_revocation"  : 동의 철회
  - "proxy_delegation"    : 후견인/대리인 승인
  - "jurisdiction_update" : 법적 관할권 변경
  - "verification_record" : 제3자 검증
```

#### 3.2.3 `status` (REQUIRED)

```
Type: string
설명: 현재 동의 상태
유효한 값:
  - "active"     : 동의가 유효하고 효력 중
  - "pending"    : 검증 또는 승인 대기 중
  - "revoked"    : 동의가 철회됨
  - "expired"    : 동의 유효 기간 종료
  - "suspended"  : 일시적으로 보류 중
  - "superseded" : 새로운 동의로 대체됨
```

---

## 데이터 스키마

### 4.1 완전한 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-consent/v1/schema.json",
  "title": "WIA Cryo-Consent Record",
  "type": "object",
  "required": ["version", "consentId", "messageType", "timestamp", "grantor", "consent", "legal"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "consentId": {
      "type": "string",
      "format": "uuid"
    },
    "messageType": {
      "type": "string",
      "enum": ["consent_record", "consent_modification", "consent_revocation", "proxy_delegation", "jurisdiction_update", "verification_record"]
    },
    "timestamp": {
      "type": "object",
      "required": ["created", "effectiveDate"],
      "properties": {
        "created": { "type": "string", "format": "date-time" },
        "modified": { "type": "string", "format": "date-time" },
        "effectiveDate": { "type": "string", "format": "date-time" },
        "expirationDate": { "type": ["string", "null"], "format": "date-time" }
      }
    },
    "grantor": {
      "type": "object",
      "required": ["id"],
      "properties": {
        "id": { "type": "string" },
        "identityVerification": {
          "type": "object",
          "required": ["method", "verifiedAt"],
          "properties": {
            "method": {
              "type": "string",
              "enum": ["biometric", "government_id", "notary", "witness", "video", "digital_signature"]
            },
            "verifiedAt": { "type": "string", "format": "date-time" },
            "verifierId": { "type": "string" },
            "documentId": { "type": "string" }
          }
        },
        "capacity": {
          "type": "object",
          "properties": {
            "mentalCompetence": { "type": "boolean" },
            "assessedBy": { "type": "string" },
            "assessmentDate": { "type": "string", "format": "date-time" }
          }
        }
      }
    },
    "consent": {
      "type": "object",
      "required": ["status", "scope"],
      "properties": {
        "status": {
          "type": "string",
          "enum": ["active", "pending", "revoked", "expired", "suspended", "superseded"]
        },
        "scope": {
          "type": "object",
          "required": ["preservation", "research", "revival"],
          "properties": {
            "preservation": {
              "type": "object",
              "properties": {
                "authorized": { "type": "boolean" },
                "types": {
                  "type": "array",
                  "items": { "type": "string", "enum": ["whole_body", "neuro", "tissue", "dna"] }
                },
                "conditions": { "type": "array", "items": { "type": "string" } }
              }
            },
            "research": {
              "type": "object",
              "properties": {
                "authorized": { "type": "boolean" },
                "categories": {
                  "type": "array",
                  "items": { "type": "string", "enum": ["medical", "scientific", "commercial", "educational"] }
                },
                "restrictions": { "type": "array", "items": { "type": "string" } }
              }
            },
            "revival": {
              "type": "object",
              "properties": {
                "authorized": { "type": "boolean" },
                "conditions": { "type": "array", "items": { "type": "string" } },
                "minimumViabilityThreshold": { "type": "number", "minimum": 0, "maximum": 1 }
              }
            }
          }
        },
        "conditions": {
          "type": "object",
          "properties": {
            "minimumTechnology": { "type": "string" },
            "minimumSuccessRate": { "type": "number" },
            "financialConditions": { "type": "array", "items": { "type": "string" } }
          }
        },
        "jurisdiction": {
          "type": "array",
          "items": { "type": "string", "pattern": "^[A-Z]{2}(-[A-Z]{2})?$" }
        }
      }
    },
    "proxy": {
      "type": "object",
      "properties": {
        "proxies": {
          "type": "array",
          "items": {
            "type": "object",
            "required": ["id", "type", "authorizedActions"],
            "properties": {
              "id": { "type": "string" },
              "type": { "type": "string", "enum": ["guardian", "healthcare_proxy", "attorney", "family_member"] },
              "authorizedActions": { "type": "array", "items": { "type": "string" } },
              "effectiveDate": { "type": "string", "format": "date-time" },
              "expirationDate": { "type": ["string", "null"], "format": "date-time" }
            }
          }
        }
      }
    },
    "legal": {
      "type": "object",
      "required": ["jurisdiction"],
      "properties": {
        "witnesses": {
          "type": "array",
          "items": {
            "type": "object",
            "required": ["id", "name", "signedAt"],
            "properties": {
              "id": { "type": "string" },
              "name": { "type": "string" },
              "signedAt": { "type": "string", "format": "date-time" },
              "signature": { "type": "string" }
            }
          }
        },
        "notarization": {
          "type": "object",
          "properties": {
            "notaryId": { "type": "string" },
            "notaryName": { "type": "string" },
            "notarizedAt": { "type": "string", "format": "date-time" },
            "sealNumber": { "type": "string" },
            "jurisdiction": { "type": "string" }
          }
        },
        "jurisdiction": { "type": "string", "pattern": "^[A-Z]{2}(-[A-Z]{2})?$" },
        "governingLaw": { "type": "string" },
        "compliance": {
          "type": "array",
          "items": { "type": "string" }
        }
      }
    },
    "meta": {
      "type": "object",
      "required": ["hash", "signature", "version"],
      "properties": {
        "hash": { "type": "string" },
        "signature": { "type": "string" },
        "previousHash": { "type": "string" },
        "version": { "type": "integer", "minimum": 1 },
        "blockchainAnchor": { "type": "string" }
      }
    }
  }
}
```

---

## 필드 명세

### 5.1 Consent Scope 필드

| 필드 | Type | Required | 설명 | 예제 |
|------|------|----------|------|------|
| `preservation.authorized` | boolean | REQUIRED | 냉동보존에 대한 동의 | `true` |
| `preservation.types` | array | REQUIRED | 승인된 보존 유형 | `["whole_body", "neuro"]` |
| `research.authorized` | boolean | REQUIRED | 연구 사용에 대한 동의 | `false` |
| `research.categories` | array | CONDITIONAL | 허용된 연구 유형 | `["medical", "scientific"]` |
| `revival.authorized` | boolean | REQUIRED | 소생 시도에 대한 동의 | `true` |
| `revival.minimumViabilityThreshold` | number | OPTIONAL | 최소 성공 확률 | `0.7` |

### 5.2 Proxy Authorization 필드

| 필드 | Type | Required | 설명 | 예제 |
|------|------|----------|------|------|
| `proxies[].id` | string | REQUIRED | 대리인 식별자 | `"PROXY-001"` |
| `proxies[].type` | string | REQUIRED | 대리인 권한 유형 | `"healthcare_proxy"` |
| `proxies[].authorizedActions` | array | REQUIRED | 허용된 작업 | `["modify_consent", "revoke"]` |
| `proxies[].effectiveDate` | timestamp | REQUIRED | 권한 시작 날짜 | `"2025-01-15T00:00:00Z"` |
| `proxies[].expirationDate` | timestamp | OPTIONAL | 권한 종료 날짜 | `"2030-01-15T00:00:00Z"` |

### 5.3 법적 검증 필드

| 필드 | Type | Required | 설명 | 예제 |
|------|------|----------|------|------|
| `witnesses[].id` | string | REQUIRED | 증인 식별자 | `"WITNESS-001"` |
| `witnesses[].name` | string | REQUIRED | 증인 전체 이름 | `"홍길동"` |
| `witnesses[].signedAt` | timestamp | REQUIRED | 서명 타임스탬프 | `"2025-01-15T10:30:00Z"` |
| `notarization.notaryId` | string | CONDITIONAL | 공증인 식별자 | `"NOTARY-CA-12345"` |
| `notarization.sealNumber` | string | CONDITIONAL | 공증인 인장 번호 | `"SEAL-2025-001"` |

---

## 데이터 타입

### 6.1 사용자 정의 타입

#### ConsentStatus

```typescript
type ConsentStatus =
  | 'active'
  | 'pending'
  | 'revoked'
  | 'expired'
  | 'suspended'
  | 'superseded';
```

#### PreservationType

```typescript
type PreservationType =
  | 'whole_body'
  | 'neuro'
  | 'tissue'
  | 'dna';
```

#### ResearchCategory

```typescript
type ResearchCategory =
  | 'medical'
  | 'scientific'
  | 'commercial'
  | 'educational';
```

#### ProxyType

```typescript
type ProxyType =
  | 'guardian'
  | 'healthcare_proxy'
  | 'attorney'
  | 'family_member';
```

#### VerificationMethod

```typescript
type VerificationMethod =
  | 'biometric'
  | 'government_id'
  | 'notary'
  | 'witness'
  | 'video'
  | 'digital_signature';
```

### 6.2 TypeScript Interface

```typescript
interface ConsentRecord {
  version: string;
  consentId: string;
  messageType: string;
  timestamp: {
    created: string;
    modified?: string;
    effectiveDate: string;
    expirationDate?: string | null;
  };
  grantor: {
    id: string;
    identityVerification: {
      method: VerificationMethod;
      verifiedAt: string;
      verifierId?: string;
      documentId?: string;
    };
    capacity?: {
      mentalCompetence: boolean;
      assessedBy: string;
      assessmentDate: string;
    };
  };
  consent: {
    status: ConsentStatus;
    scope: ConsentScope;
    conditions?: ConsentConditions;
    jurisdiction: string[];
  };
  proxy?: ProxyDelegation;
  legal: LegalAttestation;
  meta: MetaData;
}
```

### 6.3 Python Data Class

```python
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
from enum import Enum

class ConsentStatus(Enum):
    ACTIVE = "active"
    PENDING = "pending"
    REVOKED = "revoked"
    EXPIRED = "expired"
    SUSPENDED = "suspended"
    SUPERSEDED = "superseded"

class PreservationType(Enum):
    WHOLE_BODY = "whole_body"
    NEURO = "neuro"
    TISSUE = "tissue"
    DNA = "dna"

@dataclass
class ConsentScope:
    preservation: dict
    research: dict
    revival: dict

@dataclass
class ConsentRecord:
    version: str
    consent_id: str
    message_type: str
    timestamp: dict
    grantor: dict
    consent: dict
    legal: dict
    meta: dict
    proxy: Optional[dict] = None
```

---

## 검증 규칙

### 7.1 필수 필드 검증

| Rule ID | 필드 | 검증 |
|---------|------|------|
| VAL-001 | `version` | `^\d+\.\d+\.\d+$` 패턴과 일치해야 함 |
| VAL-002 | `consentId` | 유효한 UUID v4여야 함 |
| VAL-003 | `timestamp.created` | 유효한 ISO 8601이어야 함 |
| VAL-004 | `timestamp.effectiveDate` | 유효한 ISO 8601이어야 함 |
| VAL-005 | `grantor.id` | 비어있지 않아야 함 |
| VAL-006 | `consent.status` | 유효한 enum 값이어야 함 |
| VAL-007 | `legal.jurisdiction` | `^[A-Z]{2}(-[A-Z]{2})?$` 패턴과 일치해야 함 |

### 7.2 비즈니스 로직 검증

| Rule ID | 설명 | Error Code |
|---------|------|------------|
| BUS-001 | `effectiveDate`는 미래일 수 없음 | `ERR_INVALID_EFFECTIVE_DATE` |
| BUS-002 | `expirationDate`는 `effectiveDate` 이후여야 함 | `ERR_INVALID_EXPIRATION` |
| BUS-003 | 공증이 없으면 최소 1명의 증인 필요 | `ERR_INSUFFICIENT_ATTESTATION` |
| BUS-004 | 동의자는 18세 이상이거나 후견인이 있어야 함 | `ERR_INSUFFICIENT_CAPACITY` |
| BUS-005 | 철회된 동의는 재활성화할 수 없음 | `ERR_INVALID_STATUS_CHANGE` |
| BUS-006 | `minimumViabilityThreshold`는 0.0-1.0 사이여야 함 | `ERR_INVALID_THRESHOLD` |

### 7.3 관할권별 검증

| Jurisdiction | 규칙 | 요구사항 |
|--------------|------|----------|
| US-CA | Notarization | 전신 보존에는 공증 필요 |
| US-NY | Witnesses | 최소 2명의 증인 필요 |
| EU-* | GDPR Compliance | 데이터 보호 공지 포함 필요 |
| KR | Medical Review | 의사 증명 필요 |

---

## 예제

### 8.1 유효한 동의 기록 - 전체 승인

```json
{
  "$schema": "https://wia.live/cryo-consent/v1/schema.json",
  "version": "1.0.0",
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "messageType": "consent_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z",
    "effectiveDate": "2025-01-15T00:00:00Z",
    "expirationDate": null
  },
  "grantor": {
    "id": "PERSON-2025-001",
    "identityVerification": {
      "method": "government_id",
      "verifiedAt": "2025-01-15T10:00:00Z",
      "verifierId": "NOTARY-CA-12345",
      "documentId": "DL-CA-D1234567"
    },
    "capacity": {
      "mentalCompetence": true,
      "assessedBy": "DR-PSY-001",
      "assessmentDate": "2025-01-14T15:00:00Z"
    }
  },
  "consent": {
    "status": "active",
    "scope": {
      "preservation": {
        "authorized": true,
        "types": ["whole_body"],
        "conditions": [
          "M22 또는 우수한 cryoprotectant 사용",
          "법적 사망 후 15분 이내 시작"
        ]
      },
      "research": {
        "authorized": true,
        "categories": ["medical", "scientific"],
        "restrictions": [
          "가족 승인 없이 상업적 이용 금지",
          "게시된 모든 데이터 익명화"
        ]
      },
      "revival": {
        "authorized": true,
        "conditions": [
          "의식을 회복할 의료 기술 존재",
          "완전한 인지 기능의 최소 70% 확률"
        ],
        "minimumViabilityThreshold": 0.7
      }
    },
    "conditions": {
      "minimumTechnology": "분자 수리 나노기술",
      "minimumSuccessRate": 0.7,
      "financialConditions": [
        "신탁 기금으로 소생 비용 충당",
        "환자에게 부채 의무 없음"
      ]
    },
    "jurisdiction": ["US-CA", "US"]
  },
  "proxy": {
    "proxies": [
      {
        "id": "PROXY-001",
        "type": "healthcare_proxy",
        "authorizedActions": ["modify_consent", "authorize_revival", "receive_updates"],
        "effectiveDate": "2025-01-15T00:00:00Z",
        "expirationDate": null
      }
    ]
  },
  "legal": {
    "witnesses": [
      {
        "id": "WITNESS-001",
        "name": "Jane Doe",
        "signedAt": "2025-01-15T10:30:00Z",
        "signature": "0x1234abcd..."
      },
      {
        "id": "WITNESS-002",
        "name": "Robert Johnson",
        "signedAt": "2025-01-15T10:31:00Z",
        "signature": "0x5678efgh..."
      }
    ],
    "notarization": {
      "notaryId": "NOTARY-CA-12345",
      "notaryName": "Maria Garcia",
      "notarizedAt": "2025-01-15T10:35:00Z",
      "sealNumber": "SEAL-2025-001",
      "jurisdiction": "US-CA"
    },
    "jurisdiction": "US-CA",
    "governingLaw": "California Health and Safety Code Section 7100-7117",
    "compliance": ["UAGA", "HIPAA", "California End of Life Option Act"]
  },
  "meta": {
    "hash": "sha256:a5b9c3d4e5f6...",
    "signature": "0xabcdef123456...",
    "version": 1,
    "blockchainAnchor": "ethereum:0x1234567890abcdef"
  }
}
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
| WIA Cryo-Preservation | 보존 절차에 동의 연결 |
| WIA Cryo-Identity | 주체 식별 및 검증 |
| HL7 FHIR Consent | 의료 동의 상호운용성 |
| GDPR Article 9 | 특수 범주 데이터 처리 |
| UAGA | Uniform Anatomical Gift Act 준수 |

---

<div align="center">

**WIA Cryo-Consent Data Format Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
