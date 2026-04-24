# WIA Cryo-Identity 데이터 형식 표준
## Phase 1 사양

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
3. [기본 구조](#기본-구조)
4. [데이터 스키마](#데이터-스키마)
5. [필드 사양](#필드-사양)
6. [데이터 타입](#데이터-타입)
7. [검증 규칙](#검증-규칙)
8. [사용 예제](#사용-예제)
9. [버전 히스토리](#버전-히스토리)

---

## 개요

### 1.1 목적

WIA Cryo-Identity 데이터 형식 표준은 동결보존된 개인의 신원을 보존 생애주기 전반에 걸쳐 관리하고 검증하기 위한 통합 형식을 정의합니다. 이는 생전 등록, 보존, 그리고 잠재적인 미래 소생을 포함합니다.

**핵심 목표**:
- 동결보존 대상자의 고유하고 지속적인 식별 보장
- 다양한 검증 방법 지원 (생체인식, 암호화, 생물학적)
- 생전부터 잠재적 소생까지 신원 연속성 유지
- 필요한 데이터 접근을 허용하면서 프라이버시 유지

### 1.2 적용 범위

본 표준은 다음을 다룹니다:

| 도메인 | 설명 |
|--------|------|
| Identity Registration | 초기 신원 캡처 및 검증 |
| Biometric Data | 지문, DNA, 얼굴 특징, 망막 스캔 |
| Cryptographic Identity | 디지털 서명, 키 쌍, blockchain anchor |
| Identity Verification | 다양한 단계에서 신원 확인 방법 |
| Identity Recovery | 소생 후 신원 복원 절차 |

### 1.3 설계 원칙

1. **지속성**: 신원은 수세기에 걸쳐 생존해야 함
2. **중복성**: 다중 백업 식별 방법
3. **프라이버시**: 최소 공개, 최대 보안
4. **검증 가능성**: 신원의 암호화 증명
5. **상호운용성**: 글로벌 신원 표준과 호환

---

## 용어 정의

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **Subject** | 신원이 관리되는 개인 |
| **Identity Record** | Subject의 완전한 신원 데이터 패키지 |
| **Biometric Template** | 인코딩된 생체인식 특징 표현 |
| **Identity Anchor** | 신원 검증을 위한 불변 참조점 |
| **Verification Level** | 신원 확인의 신뢰도 수준 |
| **Recovery Key** | 신원 복원을 위한 암호화 키 |

### 2.2 데이터 타입

| 타입 | 설명 | 예시 |
|------|------|------|
| `string` | UTF-8 인코딩 텍스트 | `"ID-2025-001"` |
| `biometric_hash` | 생체 데이터의 SHA-256 해시 | `"sha256:a1b2c3..."` |
| `public_key` | Ed25519 공개 키 | `"ed25519:abc..."` |
| `dna_sequence` | 인코딩된 DNA 마커 | `"ATCG..."` |
| `timestamp` | ISO 8601 날짜시간 | `"2025-01-15T10:30:00Z"` |

### 2.3 필드 요구사항

| 표시 | 의미 |
|------|------|
| **REQUIRED** | 반드시 존재해야 함 |
| **OPTIONAL** | 생략 가능 |
| **CONDITIONAL** | 특정 조건에서 필수 |

---

## 기본 구조

### 3.1 Identity Record 형식

```json
{
  "$schema": "https://wia.live/cryo-identity/v1/schema.json",
  "version": "1.0.0",
  "identityId": "ID-2025-001",
  "subjectId": "SUBJ-2025-001",
  "status": "active",
  "created": "2024-06-15T10:00:00Z",
  "lastVerified": "2025-01-15T10:30:00Z",
  "personal": {
    "legalName": {
      "given": "encrypted:...",
      "family": "encrypted:...",
      "hash": "sha256:..."
    },
    "dateOfBirth": "encrypted:...",
    "placeOfBirth": "encrypted:...",
    "nationality": ["KR"],
    "governmentIds": []
  },
  "biometrics": {
    "fingerprints": [],
    "facial": {},
    "dna": {},
    "retinal": {},
    "voice": {}
  },
  "cryptographic": {
    "primaryKey": {},
    "recoveryKeys": [],
    "blockchainAnchors": []
  },
  "verification": {
    "level": "biometric",
    "methods": [],
    "history": []
  },
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 필드 상세

#### 3.2.1 `identityId` (REQUIRED)

```
타입: string
형식: ID-YYYY-NNNNNN
설명: 이 신원 레코드의 고유 식별자
예시: "ID-2025-000001"
```

#### 3.2.2 `status` (REQUIRED)

```
타입: string
유효값:
  - "pending"     : 등록 진행 중
  - "active"      : 신원 검증 및 활성화됨
  - "preserved"   : Subject가 동결보존됨
  - "suspended"   : 일시적으로 사용 불가
  - "revived"     : Subject가 소생됨
  - "merged"      : 다른 신원과 병합됨
```

---

## 데이터 스키마

### 4.1 완전한 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-identity/v1/schema.json",
  "title": "WIA Cryo-Identity Record",
  "type": "object",
  "required": ["version", "identityId", "subjectId", "status", "created", "biometrics", "cryptographic"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "스키마 버전"
    },
    "identityId": {
      "type": "string",
      "pattern": "^ID-\\d{4}-\\d{6}$",
      "description": "고유 신원 식별자"
    },
    "subjectId": {
      "type": "string",
      "description": "Subject 식별자"
    },
    "status": {
      "type": "string",
      "enum": ["pending", "active", "preserved", "suspended", "revived", "merged"],
      "description": "신원 상태"
    },
    "created": {
      "type": "string",
      "format": "date-time",
      "description": "생성 타임스탬프"
    },
    "personal": {
      "type": "object",
      "description": "개인 정보 (암호화됨)",
      "properties": {
        "legalName": {
          "type": "object",
          "properties": {
            "given": { "type": "string", "description": "이름" },
            "family": { "type": "string", "description": "성" },
            "hash": { "type": "string", "description": "이름 해시" }
          }
        },
        "dateOfBirth": { "type": "string", "description": "생년월일 (암호화)" },
        "nationality": {
          "type": "array",
          "items": { "type": "string", "pattern": "^[A-Z]{2}$" },
          "description": "국적 코드 (ISO 3166-1 alpha-2)"
        }
      }
    },
    "biometrics": {
      "type": "object",
      "description": "생체인식 데이터",
      "properties": {
        "fingerprints": {
          "type": "array",
          "description": "지문 템플릿 배열"
        },
        "facial": {
          "type": "object",
          "description": "얼굴 인식 데이터"
        },
        "dna": {
          "type": "object",
          "description": "DNA 마커 데이터"
        },
        "retinal": {
          "type": "object",
          "description": "망막 스캔 데이터"
        },
        "voice": {
          "type": "object",
          "description": "음성 템플릿"
        }
      }
    },
    "cryptographic": {
      "type": "object",
      "required": ["primaryKey"],
      "description": "암호화 신원",
      "properties": {
        "primaryKey": {
          "type": "object",
          "description": "주 암호화 키"
        },
        "recoveryKeys": {
          "type": "array",
          "description": "복구 키 배열"
        },
        "blockchainAnchors": {
          "type": "array",
          "description": "Blockchain 앵커 배열"
        }
      }
    },
    "verification": {
      "type": "object",
      "description": "검증 정보",
      "properties": {
        "level": {
          "type": "string",
          "description": "검증 수준"
        },
        "methods": {
          "type": "array",
          "description": "사용된 검증 방법"
        },
        "history": {
          "type": "array",
          "description": "검증 이력"
        }
      }
    }
  }
}
```

### 4.2 Biometrics 스키마

```json
{
  "biometrics": {
    "fingerprints": [
      {
        "finger": "right_index",
        "template": "base64-encoded-template",
        "templateFormat": "ISO-19794-2",
        "quality": 0.95,
        "capturedAt": "2024-06-15T10:00:00Z",
        "hash": "sha256:..."
      }
    ],
    "facial": {
      "template": "base64-encoded-template",
      "templateFormat": "ISO-19794-5",
      "photos": [
        {
          "type": "frontal",
          "hash": "sha256:...",
          "capturedAt": "2024-06-15T10:00:00Z"
        }
      ]
    },
    "dna": {
      "markers": [
        { "locus": "D3S1358", "alleles": [15, 16] },
        { "locus": "vWA", "alleles": [17, 18] },
        { "locus": "FGA", "alleles": [22, 24] }
      ],
      "fullSequenceHash": "sha256:...",
      "sequenceStorage": "ipfs://...",
      "collectedAt": "2024-06-15T10:00:00Z"
    },
    "retinal": {
      "template": "base64-encoded-template",
      "eye": "both",
      "capturedAt": "2024-06-15T10:00:00Z"
    },
    "voice": {
      "template": "base64-encoded-template",
      "sampleHash": "sha256:...",
      "capturedAt": "2024-06-15T10:00:00Z"
    }
  }
}
```

### 4.3 Cryptographic Identity 스키마

```json
{
  "cryptographic": {
    "primaryKey": {
      "algorithm": "Ed25519",
      "publicKey": "ed25519:abc123...",
      "created": "2024-06-15T10:00:00Z",
      "expires": null,
      "status": "active"
    },
    "recoveryKeys": [
      {
        "id": "recovery-1",
        "type": "shamir_share",
        "threshold": 3,
        "totalShares": 5,
        "shareHolders": [
          { "holder": "facility", "shareHash": "sha256:..." },
          { "holder": "family", "shareHash": "sha256:..." },
          { "holder": "legal", "shareHash": "sha256:..." }
        ]
      }
    ],
    "blockchainAnchors": [
      {
        "network": "ethereum",
        "transactionHash": "0x...",
        "blockNumber": 12345678,
        "timestamp": "2024-06-15T10:00:00Z",
        "identityHash": "sha256:..."
      }
    ]
  }
}
```

---

## 필드 사양

### 5.1 Biometric 필드

| 필드 | 타입 | 필수 | 설명 | 예시 |
|------|------|------|------|------|
| `fingerprints[].finger` | string | REQUIRED | 손가락 식별자 | `"right_index"` |
| `fingerprints[].template` | string | REQUIRED | Base64 템플릿 | `"QmFzZTY0..."` |
| `fingerprints[].quality` | number | REQUIRED | 품질 점수 0-1 | `0.95` |
| `dna.markers` | array | REQUIRED | STR 마커 | `[{...}]` |
| `dna.fullSequenceHash` | string | OPTIONAL | 전체 유전체 해시 | `"sha256:..."` |
| `facial.template` | string | REQUIRED | 얼굴 템플릿 | `"QmFzZTY0..."` |

**유효한 finger 값:**

| 값 | 설명 |
|----|------|
| `right_thumb` | 오른손 엄지 |
| `right_index` | 오른손 검지 |
| `right_middle` | 오른손 중지 |
| `right_ring` | 오른손 약지 |
| `right_little` | 오른손 새끼손가락 |
| `left_thumb` | 왼손 엄지 |
| `left_index` | 왼손 검지 |
| `left_middle` | 왼손 중지 |
| `left_ring` | 왼손 약지 |
| `left_little` | 왼손 새끼손가락 |

### 5.2 Verification Level (검증 수준)

| Level | 요구사항 | 신뢰도 |
|-------|----------|--------|
| `basic` | 정부 발급 ID만 | 60% |
| `enhanced` | ID + 단일 생체인식 | 80% |
| `biometric` | 다중 생체인식 | 95% |
| `cryptographic` | 생체인식 + 키 서명 | 99% |
| `full` | 모든 방법 + DNA | 99.9% |

### 5.3 DNA Marker Loci

| Locus | Chromosome | 용도 |
|-------|------------|------|
| D3S1358 | 3 | 핵심 식별 |
| vWA | 12 | 핵심 식별 |
| FGA | 4 | 핵심 식별 |
| D8S1179 | 8 | 확장 프로파일링 |
| D21S11 | 21 | 확장 프로파일링 |
| D18S51 | 18 | 확장 프로파일링 |
| AMEL | X/Y | 성별 결정 |

---

## 데이터 타입

### 6.1 Custom 타입

```typescript
type IdentityStatus =
  | 'pending'      // 대기 중
  | 'active'       // 활성
  | 'preserved'    // 보존됨
  | 'suspended'    // 일시중단
  | 'revived'      // 소생됨
  | 'merged';      // 병합됨

type VerificationLevel =
  | 'basic'          // 기본
  | 'enhanced'       // 향상됨
  | 'biometric'      // 생체인식
  | 'cryptographic'  // 암호화
  | 'full';          // 전체

type FingerType =
  | 'right_thumb' | 'right_index' | 'right_middle' | 'right_ring' | 'right_little'
  | 'left_thumb' | 'left_index' | 'left_middle' | 'left_ring' | 'left_little';

interface BiometricTemplate {
  template: string;        // Base64 인코딩
  templateFormat: string;  // ISO 표준
  quality: number;         // 0.0 - 1.0
  capturedAt: string;      // ISO 8601
  hash: string;            // SHA-256
}
```

### 6.2 Enum 값

#### Template 형식

| 코드 | 표준 | 설명 |
|------|------|------|
| `ISO-19794-2` | ISO/IEC 19794-2 | 지문 minutiae |
| `ISO-19794-4` | ISO/IEC 19794-4 | 지문 이미지 |
| `ISO-19794-5` | ISO/IEC 19794-5 | 얼굴 이미지 |
| `ISO-19794-6` | ISO/IEC 19794-6 | 홍채 이미지 |
| `ANSI-NIST` | ANSI/NIST-ITL | 다중 생체인식 |

---

## 검증 규칙

### 7.1 필수 필드 검증

| 규칙 ID | 필드 | 검증 |
|---------|------|------|
| VAL-001 | `identityId` | `^ID-\d{4}-\d{6}$` 패턴과 일치해야 함 |
| VAL-002 | `biometrics.fingerprints` | 최소 2개 지문 필요 |
| VAL-003 | `biometrics.dna.markers` | 최소 13개 핵심 마커 |
| VAL-004 | `cryptographic.primaryKey` | 유효한 Ed25519 키여야 함 |
| VAL-005 | `verification.level` | 생체인식 증거와 일치해야 함 |

### 7.2 비즈니스 로직 검증

| 규칙 ID | 설명 | 에러 코드 |
|---------|------|-----------|
| BUS-001 | DNA 마커는 시스템 내에서 고유해야 함 | `ERR_DUPLICATE_DNA` |
| BUS-002 | 지문은 품질 > 0.7이어야 함 | `ERR_LOW_QUALITY` |
| BUS-003 | 복구 키는 threshold ≤ total이어야 함 | `ERR_INVALID_THRESHOLD` |
| BUS-004 | Blockchain 앵커는 검증되어야 함 | `ERR_UNVERIFIED_ANCHOR` |

### 7.3 에러 코드

| 코드 | 메시지 | 설명 |
|------|--------|------|
| `ERR_INVALID_IDENTITY` | Invalid identity format | ID 형식 위반 |
| `ERR_DUPLICATE_DNA` | DNA already registered | 중복 감지 |
| `ERR_LOW_QUALITY` | Biometric quality too low | 품질 임계값 실패 |
| `ERR_INVALID_KEY` | Invalid cryptographic key | 키 검증 실패 |
| `ERR_VERIFICATION_FAILED` | Identity verification failed | 매칭 실패 |

---

## 사용 예제

### 8.1 유효한 Identity Record

```json
{
  "$schema": "https://wia.live/cryo-identity/v1/schema.json",
  "version": "1.0.0",
  "identityId": "ID-2025-000001",
  "subjectId": "SUBJ-2025-001",
  "status": "active",
  "created": "2024-06-15T10:00:00Z",
  "lastVerified": "2025-01-15T10:30:00Z",
  "personal": {
    "legalName": {
      "given": "encrypted:aes256:...",
      "family": "encrypted:aes256:...",
      "hash": "sha256:abc123def456..."
    },
    "dateOfBirth": "encrypted:aes256:...",
    "nationality": ["KR"]
  },
  "biometrics": {
    "fingerprints": [
      {
        "finger": "right_index",
        "template": "QmFzZTY0RW5jb2RlZFRlbXBsYXRl...",
        "templateFormat": "ISO-19794-2",
        "quality": 0.95,
        "capturedAt": "2024-06-15T10:00:00Z",
        "hash": "sha256:fingerprint_hash_1..."
      },
      {
        "finger": "left_index",
        "template": "QmFzZTY0RW5jb2RlZFRlbXBsYXRl...",
        "templateFormat": "ISO-19794-2",
        "quality": 0.92,
        "capturedAt": "2024-06-15T10:00:00Z",
        "hash": "sha256:fingerprint_hash_2..."
      }
    ],
    "dna": {
      "markers": [
        { "locus": "D3S1358", "alleles": [15, 16] },
        { "locus": "vWA", "alleles": [17, 18] },
        { "locus": "FGA", "alleles": [22, 24] }
      ],
      "fullSequenceHash": "sha256:dna_full_sequence_hash...",
      "collectedAt": "2024-06-15T10:00:00Z"
    }
  },
  "cryptographic": {
    "primaryKey": {
      "algorithm": "Ed25519",
      "publicKey": "ed25519:abc123def456...",
      "created": "2024-06-15T10:00:00Z",
      "status": "active"
    },
    "blockchainAnchors": [
      {
        "network": "ethereum",
        "transactionHash": "0xabc123...",
        "blockNumber": 12345678,
        "timestamp": "2024-06-15T10:00:00Z"
      }
    ]
  },
  "verification": {
    "level": "biometric",
    "methods": ["fingerprint", "dna", "cryptographic"],
    "history": [
      {
        "timestamp": "2025-01-15T10:30:00Z",
        "method": "fingerprint",
        "result": "verified",
        "confidence": 0.98
      }
    ]
  },
  "meta": {
    "hash": "sha256:record_hash...",
    "signature": "ed25519:signature...",
    "version": 1
  }
}
```

### 8.2 잘못된 예제 - Biometric 누락

```json
{
  "version": "1.0.0",
  "identityId": "ID-2025-000002",
  "subjectId": "SUBJ-2025-002",
  "status": "active",
  "biometrics": {
    "fingerprints": []
  }
}
```

**에러**: `ERR_VALIDATION_FAILED` - 최소 2개의 지문이 필요합니다

---

## 버전 히스토리

| 버전 | 날짜 | 변경사항 |
|------|------|----------|
| 1.0.0 | 2025-01 | 초기 릴리스 |

---

<div align="center">

**WIA Cryo-Identity 데이터 형식 표준 v1.0.0**

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
