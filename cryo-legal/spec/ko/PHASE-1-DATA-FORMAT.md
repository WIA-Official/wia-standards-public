# WIA Cryo-Legal 표준 - 1단계: 데이터 형식 명세

**버전**: 1.0.0
**상태**: 초안
**날짜**: 2025-01
**작성자**: WIA 표준 위원회
**라이선스**: MIT
**주요 색상**: #06B6D4 (Cyan)

---

## 1. 개요

### 1.1 목적

WIA Cryo-Legal 표준은 냉동보존 절차와 관련된 법적 프레임워크, 문서화 및 규정 준수 요구사항을 관리하기 위한 데이터 형식을 정의합니다. 이 명세는 국제 냉동보존 시설 전반에 걸쳐 법적 문서, 동의 기록, 관할권 매핑 및 규정 준수의 표준화된 처리를 보장합니다.

### 1.2 적용 범위

| 범주 | 설명 |
|------|------|
| 법적 문서 | 계약서, 유언장, 사전의료지시서 |
| 관할권 매핑 | 국가별 법적 요구사항 |
| 규정 준수 기록 | 규제 감사 추적 |
| 동의 검증 | 법적 동의 체인 검증 |
| 신탁 관리 | 냉동보존 신탁 구조 |

### 1.3 설계 원칙

1. **법적 무결성**: 모든 문서의 암호화 검증
2. **관할권 인식**: 다중 관할권 지원
3. **시간적 유효성**: 장기 문서 유효성 관리
4. **감사 가능성**: 완전한 법적 감사 추적
5. **개인정보 규정 준수**: GDPR, HIPAA 및 국제 개인정보보호법 지원

---

## 2. 데이터 스키마

### 2.1 CryoLegalDocument 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-legal/v1/document.schema.json",
  "title": "WIA Cryo-Legal Document",
  "type": "object",
  "required": [
    "documentId",
    "version",
    "documentType",
    "jurisdiction",
    "createdAt",
    "parties",
    "content",
    "signatures"
  ],
  "properties": {
    "$schema": {
      "type": "string",
      "format": "uri"
    },
    "documentId": {
      "type": "string",
      "format": "uuid",
      "description": "고유 문서 식별자"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "documentType": {
      "type": "string",
      "enum": [
        "cryopreservation_contract",
        "advance_directive",
        "last_will",
        "trust_document",
        "consent_form",
        "power_of_attorney",
        "medical_directive",
        "revival_instruction",
        "asset_disposition",
        "identity_verification"
      ]
    },
    "jurisdiction": {
      "type": "object",
      "required": ["primaryCountry", "governingLaw"],
      "properties": {
        "primaryCountry": {
          "type": "string",
          "pattern": "^[A-Z]{2}$"
        },
        "secondaryCountries": {
          "type": "array",
          "items": { "type": "string", "pattern": "^[A-Z]{2}$" }
        },
        "governingLaw": {
          "type": "string"
        },
        "disputeResolution": {
          "type": "string",
          "enum": ["arbitration", "litigation", "mediation"]
        },
        "venue": {
          "type": "string"
        }
      }
    },
    "createdAt": {
      "type": "string",
      "format": "date-time"
    },
    "effectiveDate": {
      "type": "string",
      "format": "date-time"
    },
    "expirationDate": {
      "type": "string",
      "format": "date-time"
    },
    "parties": {
      "type": "array",
      "minItems": 1,
      "items": {
        "$ref": "#/definitions/Party"
      }
    },
    "content": {
      "$ref": "#/definitions/DocumentContent"
    },
    "signatures": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Signature"
      }
    },
    "witnesses": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Witness"
      }
    },
    "notarization": {
      "$ref": "#/definitions/Notarization"
    },
    "attachments": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Attachment"
      }
    },
    "auditLog": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/AuditEntry"
      }
    },
    "metadata": {
      "type": "object",
      "properties": {
        "language": { "type": "string" },
        "classification": { "type": "string" },
        "retentionPeriod": { "type": "string" },
        "tags": { "type": "array", "items": { "type": "string" } }
      }
    }
  },
  "definitions": {
    "Party": {
      "type": "object",
      "required": ["partyId", "role", "identity"],
      "properties": {
        "partyId": { "type": "string", "format": "uuid" },
        "role": {
          "type": "string",
          "enum": ["subject", "facility", "trustee", "beneficiary", "executor", "witness", "notary", "legal_representative"]
        },
        "identity": {
          "type": "object",
          "properties": {
            "type": { "type": "string", "enum": ["individual", "organization"] },
            "legalName": { "type": "string" },
            "identificationNumber": { "type": "string" },
            "identificationType": { "type": "string" },
            "dateOfBirth": { "type": "string", "format": "date" },
            "nationality": { "type": "string" },
            "address": { "$ref": "#/definitions/Address" }
          }
        },
        "contact": {
          "type": "object",
          "properties": {
            "email": { "type": "string", "format": "email" },
            "phone": { "type": "string" },
            "preferredMethod": { "type": "string" }
          }
        }
      }
    },
    "Address": {
      "type": "object",
      "properties": {
        "street": { "type": "string" },
        "city": { "type": "string" },
        "state": { "type": "string" },
        "postalCode": { "type": "string" },
        "country": { "type": "string", "pattern": "^[A-Z]{2}$" }
      }
    },
    "DocumentContent": {
      "type": "object",
      "required": ["format", "body"],
      "properties": {
        "format": { "type": "string", "enum": ["plaintext", "markdown", "html", "pdf_base64"] },
        "body": { "type": "string" },
        "sections": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "sectionId": { "type": "string" },
              "title": { "type": "string" },
              "content": { "type": "string" },
              "required": { "type": "boolean" }
            }
          }
        },
        "hash": { "type": "string" },
        "hashAlgorithm": { "type": "string", "enum": ["SHA-256", "SHA-384", "SHA-512"] }
      }
    },
    "Signature": {
      "type": "object",
      "required": ["signerId", "timestamp", "signatureData"],
      "properties": {
        "signerId": { "type": "string", "format": "uuid" },
        "timestamp": { "type": "string", "format": "date-time" },
        "signatureType": {
          "type": "string",
          "enum": ["electronic", "digital", "handwritten_scanned", "biometric"]
        },
        "signatureData": { "type": "string" },
        "certificate": { "type": "string" },
        "ipAddress": { "type": "string" },
        "deviceInfo": { "type": "string" },
        "verificationStatus": {
          "type": "string",
          "enum": ["pending", "verified", "failed", "revoked"]
        }
      }
    },
    "Witness": {
      "type": "object",
      "required": ["witnessId", "identity", "timestamp"],
      "properties": {
        "witnessId": { "type": "string", "format": "uuid" },
        "identity": { "$ref": "#/definitions/Party/properties/identity" },
        "timestamp": { "type": "string", "format": "date-time" },
        "attestation": { "type": "string" },
        "signature": { "$ref": "#/definitions/Signature" }
      }
    },
    "Notarization": {
      "type": "object",
      "properties": {
        "notaryId": { "type": "string" },
        "notaryName": { "type": "string" },
        "commission": { "type": "string" },
        "jurisdiction": { "type": "string" },
        "timestamp": { "type": "string", "format": "date-time" },
        "seal": { "type": "string" },
        "certificate": { "type": "string" }
      }
    },
    "Attachment": {
      "type": "object",
      "required": ["attachmentId", "filename", "mimeType"],
      "properties": {
        "attachmentId": { "type": "string", "format": "uuid" },
        "filename": { "type": "string" },
        "mimeType": { "type": "string" },
        "size": { "type": "integer" },
        "hash": { "type": "string" },
        "url": { "type": "string", "format": "uri" },
        "description": { "type": "string" }
      }
    },
    "AuditEntry": {
      "type": "object",
      "required": ["timestamp", "action", "actorId"],
      "properties": {
        "timestamp": { "type": "string", "format": "date-time" },
        "action": {
          "type": "string",
          "enum": ["created", "viewed", "modified", "signed", "notarized", "revoked", "archived"]
        },
        "actorId": { "type": "string" },
        "details": { "type": "string" },
        "ipAddress": { "type": "string" }
      }
    }
  }
}
```

---

## 3. 필드 명세

### 3.1 핵심 필드

| 필드 | 타입 | 필수 | 설명 | 예시 |
|------|------|------|------|------|
| `documentId` | string (UUID) | 예 | 고유 식별자 | `"550e8400-e29b-41d4-a716-446655440000"` |
| `version` | string | 예 | 시맨틱 버전 | `"1.0.0"` |
| `documentType` | enum | 예 | 법적 문서 유형 | `"cryopreservation_contract"` |
| `jurisdiction.primaryCountry` | string | 예 | ISO 3166-1 alpha-2 코드 | `"KR"` |
| `jurisdiction.governingLaw` | string | 예 | 적용 법률 | `"대한민국 민법"` |
| `createdAt` | datetime | 예 | 생성 타임스탬프 | `"2025-01-15T10:30:00Z"` |
| `effectiveDate` | datetime | 아니오 | 효력 발생일 | `"2025-02-01T00:00:00Z"` |
| `expirationDate` | datetime | 아니오 | 문서 만료일 | `null` (영구) |

### 3.2 당사자 필드

| 필드 | 타입 | 필수 | 설명 | 예시 |
|------|------|------|------|------|
| `parties[].partyId` | string (UUID) | 예 | 당사자 식별자 | `"a1b2c3d4-..."` |
| `parties[].role` | enum | 예 | 문서 내 역할 | `"subject"` |
| `parties[].identity.type` | enum | 예 | 개인 또는 조직 | `"individual"` |
| `parties[].identity.legalName` | string | 예 | 법적 성명 | `"홍길동"` |
| `parties[].identity.identificationNumber` | string | 아니오 | 신분증 번호 | `"901234-1234567"` |
| `parties[].identity.dateOfBirth` | date | 아니오 | 생년월일 | `"1990-01-01"` |
| `parties[].identity.nationality` | string | 아니오 | 국적 | `"KR"` |

### 3.3 서명 필드

| 필드 | 타입 | 필수 | 설명 | 예시 |
|------|------|------|------|------|
| `signatures[].signerId` | string (UUID) | 예 | 서명자 당사자 ID | `"a1b2c3d4-..."` |
| `signatures[].timestamp` | datetime | 예 | 서명 시간 | `"2025-01-15T14:30:00Z"` |
| `signatures[].signatureType` | enum | 예 | 서명 유형 | `"digital"` |
| `signatures[].signatureData` | string | 예 | Base64 인코딩 서명 | `"MEUCIQDx..."` |
| `signatures[].certificate` | string | 아니오 | X.509 인증서 | `"-----BEGIN CERTIFICATE-----..."` |
| `signatures[].verificationStatus` | enum | 아니오 | 검증 상태 | `"verified"` |

---

## 4. 데이터 타입

### 4.1 문서 유형

| 값 | 설명 |
|----|------|
| `cryopreservation_contract` | 시설과의 주 서비스 계약 |
| `advance_directive` | 의료 선호 지시서 |
| `last_will` | 냉동보존 조항이 포함된 유언장 |
| `trust_document` | 냉동보존 자금 신탁 |
| `consent_form` | 고지된 동의 문서 |
| `power_of_attorney` | 법적 대리인 위임장 |
| `medical_directive` | 의료 대리인 지침 |
| `revival_instruction` | 소생 후 돌봄 선호 |
| `asset_disposition` | 자산 관리 지침 |
| `identity_verification` | 신원 증명 문서 |

### 4.2 당사자 역할

| 값 | 설명 |
|----|------|
| `subject` | 냉동보존 대상자 |
| `facility` | 냉동보존 서비스 제공자 |
| `trustee` | 신탁 관리자 |
| `beneficiary` | 수혜자 |
| `executor` | 유언 집행자 |
| `witness` | 문서 증인 |
| `notary` | 공증인 |
| `legal_representative` | 변호사 또는 법적 대리인 |

### 4.3 검증 상태

| 값 | 설명 |
|----|------|
| `pending` | 검증 대기 중 |
| `verified` | 성공적으로 검증됨 |
| `failed` | 검증 실패 |
| `revoked` | 이전에 유효했으나 취소됨 |

---

## 5. 검증 규칙

### 5.1 문서 검증

| 규칙 ID | 필드 | 검증 | 오류 코드 |
|---------|------|------|-----------|
| VAL-001 | `documentId` | 유효한 UUID v4여야 함 | `INVALID_DOCUMENT_ID` |
| VAL-002 | `version` | 시맨틱 버저닝과 일치해야 함 | `INVALID_VERSION` |
| VAL-003 | `jurisdiction.primaryCountry` | 유효한 ISO 3166-1 alpha-2여야 함 | `INVALID_COUNTRY_CODE` |
| VAL-004 | `createdAt` | 유효한 ISO 8601 datetime이어야 함 | `INVALID_DATETIME` |
| VAL-005 | `effectiveDate` | 존재 시 createdAt 이상이어야 함 | `INVALID_EFFECTIVE_DATE` |
| VAL-006 | `parties` | 최소 한 명의 당사자가 있어야 함 | `NO_PARTIES` |
| VAL-007 | `signatures` | 모든 필수 서명자가 서명해야 함 | `MISSING_SIGNATURES` |

### 5.2 서명 검증

| 규칙 ID | 필드 | 검증 | 오류 코드 |
|---------|------|------|-----------|
| SIG-001 | `signerId` | 유효한 당사자를 참조해야 함 | `INVALID_SIGNER` |
| SIG-002 | `timestamp` | 현재 시간 이하여야 함 | `FUTURE_SIGNATURE` |
| SIG-003 | `signatureData` | 유효한 Base64여야 함 | `INVALID_SIGNATURE_DATA` |
| SIG-004 | `certificate` | 디지털 서명 시 유효한 X.509여야 함 | `INVALID_CERTIFICATE` |

### 5.3 오류 코드

| 코드 | 메시지 | 설명 |
|------|--------|------|
| `INVALID_DOCUMENT_ID` | 문서 ID 형식이 잘못됨 | UUID 형식 필요 |
| `INVALID_VERSION` | 버전 형식이 잘못됨 | MAJOR.MINOR.PATCH 사용 |
| `INVALID_COUNTRY_CODE` | 국가 코드가 인식되지 않음 | ISO 3166-1 alpha-2 사용 |
| `INVALID_DATETIME` | DateTime 형식이 잘못됨 | ISO 8601 형식 사용 |
| `INVALID_EFFECTIVE_DATE` | 효력 발생일이 생성일 이전임 | 효력 발생일 >= 생성일 |
| `NO_PARTIES` | 당사자가 정의되지 않음 | 최소 한 명의 당사자 필요 |
| `MISSING_SIGNATURES` | 필수 서명이 누락됨 | 모든 당사자가 서명해야 함 |
| `INVALID_SIGNER` | 서명자가 당사자 목록에 없음 | 서명자는 당사자여야 함 |
| `FUTURE_SIGNATURE` | 서명 타임스탬프가 미래임 | 미래에 서명할 수 없음 |
| `INVALID_SIGNATURE_DATA` | 서명 데이터가 손상됨 | 유효한 Base64 필요 |
| `INVALID_CERTIFICATE` | 인증서 검증 실패 | 유효한 X.509 필요 |

---

## 6. 예제

### 6.1 유효한 냉동보존 계약서

```json
{
  "$schema": "https://wia.live/cryo-legal/v1/document.schema.json",
  "documentId": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "documentType": "cryopreservation_contract",
  "jurisdiction": {
    "primaryCountry": "KR",
    "governingLaw": "대한민국 민법",
    "disputeResolution": "arbitration",
    "venue": "서울"
  },
  "createdAt": "2025-01-15T10:30:00Z",
  "effectiveDate": "2025-02-01T00:00:00Z",
  "parties": [
    {
      "partyId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "role": "subject",
      "identity": {
        "type": "individual",
        "legalName": "홍길동",
        "identificationNumber": "901234-1234567",
        "identificationType": "resident_registration",
        "dateOfBirth": "1990-01-23",
        "nationality": "KR",
        "address": {
          "street": "테헤란로 123",
          "city": "서울특별시",
          "state": "강남구",
          "postalCode": "06100",
          "country": "KR"
        }
      },
      "contact": {
        "email": "hong@email.com",
        "phone": "+82-10-1234-5678",
        "preferredMethod": "email"
      }
    },
    {
      "partyId": "b2c3d4e5-f6a7-8901-bcde-f23456789012",
      "role": "facility",
      "identity": {
        "type": "organization",
        "legalName": "크라이오라이프 코리아 주식회사",
        "identificationNumber": "123-45-67890",
        "identificationType": "business_registration",
        "address": {
          "street": "판교로 456",
          "city": "성남시",
          "state": "경기도",
          "postalCode": "13487",
          "country": "KR"
        }
      }
    }
  ],
  "content": {
    "format": "markdown",
    "body": "# 냉동보존 서비스 계약서\n\n## 제1조: 서비스\n\n시설은 대상자에게 전신 냉동보존 서비스를 제공합니다...",
    "sections": [
      {
        "sectionId": "art-1",
        "title": "서비스",
        "content": "시설은 법적 사망 시 대상자에게 전신 냉동보존 서비스를 제공합니다...",
        "required": true
      },
      {
        "sectionId": "art-2",
        "title": "결제 조건",
        "content": "대상자는 생명보험 양도를 통해 냉동보존 자금을 조달합니다...",
        "required": true
      }
    ],
    "hash": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "hashAlgorithm": "SHA-256"
  },
  "signatures": [
    {
      "signerId": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "timestamp": "2025-01-15T14:30:00Z",
      "signatureType": "digital",
      "signatureData": "MEUCIQDxN2qc8Kj2c5y...",
      "certificate": "-----BEGIN CERTIFICATE-----\nMIIDXTCCAkWgAwIBAgIJAJC1...",
      "verificationStatus": "verified"
    }
  ],
  "witnesses": [
    {
      "witnessId": "c3d4e5f6-a7b8-9012-cdef-345678901234",
      "identity": {
        "type": "individual",
        "legalName": "김증인"
      },
      "timestamp": "2025-01-15T14:35:00Z",
      "attestation": "본인은 대상자가 자발적으로 이 문서에 서명하는 것을 목격했습니다."
    }
  ],
  "notarization": {
    "notaryId": "NP-KR-12345",
    "notaryName": "박공증",
    "commission": "대한민국 공증인 #12345",
    "jurisdiction": "KR",
    "timestamp": "2025-01-15T16:00:00Z"
  },
  "metadata": {
    "language": "ko-KR",
    "classification": "confidential",
    "retentionPeriod": "perpetual",
    "tags": ["냉동보존", "전신", "한국"]
  }
}
```

### 6.2 유효한 신탁 문서

```json
{
  "$schema": "https://wia.live/cryo-legal/v1/document.schema.json",
  "documentId": "660f9511-f30c-52e5-b827-557766551111",
  "version": "1.0.0",
  "documentType": "trust_document",
  "jurisdiction": {
    "primaryCountry": "KR",
    "governingLaw": "대한민국 신탁법"
  },
  "createdAt": "2025-01-20T09:00:00Z",
  "parties": [
    {
      "partyId": "d4e5f6a7-b8c9-0123-def4-567890123456",
      "role": "subject",
      "identity": {
        "type": "individual",
        "legalName": "김위탁",
        "nationality": "KR"
      }
    },
    {
      "partyId": "e5f6a7b8-c9d0-1234-ef56-789012345678",
      "role": "trustee",
      "identity": {
        "type": "organization",
        "legalName": "한국신탁은행"
      }
    }
  ],
  "content": {
    "format": "markdown",
    "body": "# 냉동보존 유지 신탁\n\n이 신탁은 장기 냉동보존 자금 조달을 위해 설립됩니다...",
    "hash": "a1b2c3d4e5f6789012345678901234567890abcdef1234567890abcdef123456",
    "hashAlgorithm": "SHA-256"
  },
  "signatures": [
    {
      "signerId": "d4e5f6a7-b8c9-0123-def4-567890123456",
      "timestamp": "2025-01-20T10:00:00Z",
      "signatureType": "digital",
      "signatureData": "MEUCIQCabc123...",
      "verificationStatus": "verified"
    }
  ],
  "metadata": {
    "language": "ko-KR",
    "classification": "confidential",
    "retentionPeriod": "perpetual",
    "tags": ["신탁", "유지자금", "한국"]
  }
}
```

### 6.3 유효한 사전의료지시서

```json
{
  "$schema": "https://wia.live/cryo-legal/v1/document.schema.json",
  "documentId": "771a0622-a41d-63f6-c938-668877662222",
  "version": "1.0.0",
  "documentType": "advance_directive",
  "jurisdiction": {
    "primaryCountry": "KR",
    "secondaryCountries": ["US", "JP"],
    "governingLaw": "연명의료결정법"
  },
  "createdAt": "2025-01-25T11:00:00Z",
  "effectiveDate": "2025-01-25T11:00:00Z",
  "parties": [
    {
      "partyId": "f6a7b8c9-d0e1-2345-fa67-890123456789",
      "role": "subject",
      "identity": {
        "type": "individual",
        "legalName": "이지시"
      }
    }
  ],
  "content": {
    "format": "markdown",
    "body": "# 냉동보존을 위한 사전의료지시서\n\n법적 사망 판정 시 즉시 냉동보존 절차를 시작할 것을 지시합니다...",
    "hash": "b2c3d4e5f6a789012345678901234567890abcdef1234567890abcdef1234567",
    "hashAlgorithm": "SHA-256"
  },
  "signatures": [
    {
      "signerId": "f6a7b8c9-d0e1-2345-fa67-890123456789",
      "timestamp": "2025-01-25T11:30:00Z",
      "signatureType": "electronic",
      "signatureData": "SGVsbG8gV29ybGQ=",
      "verificationStatus": "verified"
    }
  ],
  "metadata": {
    "language": "ko-KR",
    "classification": "confidential",
    "retentionPeriod": "perpetual"
  }
}
```

### 6.4 잘못된 예제: 필수 필드 누락

```json
{
  "documentId": "invalid-uuid-format",
  "version": "1.0",
  "documentType": "contract",
  "jurisdiction": {
    "primaryCountry": "KOR"
  },
  "parties": []
}
```

**검증 오류:**
- `INVALID_DOCUMENT_ID`: documentId는 UUID v4여야 함
- `INVALID_VERSION`: version은 MAJOR.MINOR.PATCH여야 함
- `INVALID_DOCUMENT_TYPE`: "contract"는 enum에 없음
- `INVALID_COUNTRY_CODE`: "KOR"는 "KR"이어야 함
- `NO_PARTIES`: parties 배열이 비어있음
- `MISSING_REQUIRED_FIELD`: createdAt이 필수임

### 6.5 잘못된 예제: 미래 서명

```json
{
  "documentId": "882b1733-b52e-74a7-da49-779988773333",
  "version": "1.0.0",
  "documentType": "consent_form",
  "jurisdiction": {
    "primaryCountry": "KR",
    "governingLaw": "개인정보보호법"
  },
  "createdAt": "2025-01-01T00:00:00Z",
  "parties": [
    {
      "partyId": "a7b8c9d0-e1f2-3456-ab78-901234567890",
      "role": "subject",
      "identity": {
        "type": "individual",
        "legalName": "테스트 사용자"
      }
    }
  ],
  "content": {
    "format": "plaintext",
    "body": "테스트 내용"
  },
  "signatures": [
    {
      "signerId": "a7b8c9d0-e1f2-3456-ab78-901234567890",
      "timestamp": "2099-12-31T23:59:59Z",
      "signatureType": "electronic",
      "signatureData": "dGVzdA=="
    }
  ]
}
```

**검증 오류:**
- `FUTURE_SIGNATURE`: 서명 타임스탬프가 미래임

---

## 7. 버전 이력

| 버전 | 날짜 | 변경사항 |
|------|------|----------|
| 1.0.0 | 2025-01 | 최초 릴리스 |

---

<div align="center">

**WIA Cryo-Legal 표준 v1.0.0**

1단계: 데이터 형식 명세

**弘益人間 (홍익인간)** · 널리 인간을 이롭게

---

© 2025 WIA 표준 위원회

MIT 라이선스

</div>
