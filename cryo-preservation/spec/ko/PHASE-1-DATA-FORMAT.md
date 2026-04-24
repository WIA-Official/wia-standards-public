# WIA 냉동보존 데이터 형식 표준
## 1단계 명세

---

**버전**: 1.0.0
**상태**: Draft
**작성일**: 2025-01
**작성자**: WIA 표준 위원회
**라이선스**: MIT
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

WIA 냉동보존 데이터 형식 표준은 전 세계 의료 시설, 연구 기관 및 냉동보존 기관 간의 냉동보존 데이터를 기록, 전송 및 관리하기 위한 통합 JSON 기반 형식을 정의합니다.

**핵심 목표**:
- 모든 시설에서 냉동보존 기록 표준화
- 서로 다른 냉동보존 제공자 간 상호운용성 확보
- 보존 생애주기 전체에 걸친 데이터 무결성 및 추적성 보장
- 규제 준수 및 감사 요구사항 지원

### 1.2 적용 범위

본 표준은 다음 데이터 도메인을 포함합니다:

| 도메인 | 설명 |
|--------|------|
| 환자 기록 | 냉동보존 대상자의 개인 및 의료 정보 |
| 보존 절차 | 유리화 및 냉각 과정의 상세 정보 |
| 저장 조건 | 환경 모니터링 및 유지보수 데이터 |
| 관리 연속성 | 이송 및 취급 기록 |
| 품질 지표 | 보존 품질 지표 |

### 1.3 설계 원칙

1. **불변성 (Immutability)**: 생성된 기록은 수정 불가, 추가만 가능
2. **추적성 (Traceability)**: 모든 작업에 대한 완전한 감사 추적
3. **상호운용성 (Interoperability)**: 기존 의료 표준(HL7, FHIR)과 호환
4. **보안 (Security)**: 민감 데이터의 종단간 암호화
5. **검증 가능성 (Validation)**: JSON Schema를 통한 형식 검증

---

## 용어 정의

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **대상자 (Subject)** | 냉동보존을 받는 개인 |
| **유리화 (Vitrification)** | 얼음 결정 형성 없이 유리처럼 고체화 |
| **관류 (Perfusion)** | 혈액을 동결보호제로 대체하는 과정 |
| **듀어 (Dewar)** | 액체 질소 저장용 진공 단열 용기 |
| **CPA** | 동결보호제 (Cryoprotective Agent) |
| **냉각 (Cool-down)** | 제어된 온도 감소 과정 |
| **장기 저장** | 극저온(-196°C)에서의 유지 관리 |

### 2.2 데이터 타입

| 타입 | 설명 | 예시 |
|------|------|------|
| `string` | UTF-8 인코딩 텍스트 | `"WIA-CRYO-001"` |
| `number` | IEEE 754 배정밀도 | `-196.0`, `3.14159` |
| `integer` | 부호 있는 64비트 정수 | `1`, `-273` |
| `boolean` | 불리언 값 | `true`, `false` |
| `timestamp` | ISO 8601 날짜시간 | `"2025-01-15T10:30:00Z"` |
| `uuid` | UUID v4 식별자 | `"550e8400-e29b-41d4-a716-446655440000"` |

### 2.3 필드 요구사항

| 표기 | 의미 |
|------|------|
| **REQUIRED** | 필수 포함 |
| **OPTIONAL** | 선택적 포함 가능 |
| **CONDITIONAL** | 특정 조건에서 필수 |

---

## 기본 구조

### 3.1 메시지 형식

모든 WIA 냉동보존 메시지는 다음 기본 구조를 따릅니다:

```json
{
  "$schema": "https://wia.live/cryo-preservation/v1/schema.json",
  "version": "1.0.0",
  "messageId": "uuid-v4-string",
  "messageType": "preservation_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z"
  },
  "facility": {
    "id": "FAC-001",
    "name": "WIA 냉동보존 연구소",
    "location": {
      "country": "KR",
      "city": "서울"
    }
  },
  "subject": {
    "id": "SUBJ-001",
    "anonymizedId": "ANON-550e8400"
  },
  "data": {
    // 도메인별 데이터
  },
  "meta": {
    "hash": "sha256-hash",
    "signature": "digital-signature",
    "previousHash": "previous-record-hash"
  }
}
```

### 3.2 필드 상세

#### 3.2.1 `$schema` (OPTIONAL)

```
타입: string
형식: URI
설명: 검증을 위한 JSON Schema 위치
예시: "https://wia.live/cryo-preservation/v1/schema.json"
```

#### 3.2.2 `version` (REQUIRED)

```
타입: string
형식: 시맨틱 버저닝 (MAJOR.MINOR.PATCH)
설명: 명세 버전
예시: "1.0.0"
```

#### 3.2.3 `messageId` (REQUIRED)

```
타입: string
형식: UUID v4
설명: 이 메시지의 고유 식별자
예시: "550e8400-e29b-41d4-a716-446655440000"
```

#### 3.2.4 `messageType` (REQUIRED)

```
타입: string
설명: 메시지 유형
유효값:
  - "preservation_record"  : 주요 보존 데이터
  - "status_update"        : 상태 변경 알림
  - "quality_report"       : 품질 평가
  - "transfer_record"      : 관리 이전
  - "maintenance_log"      : 시설 유지보수
```

---

## 데이터 스키마

### 4.1 전체 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-preservation/v1/schema.json",
  "title": "WIA 냉동보존 기록",
  "type": "object",
  "required": ["version", "messageId", "messageType", "timestamp", "facility", "subject", "data"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "messageId": {
      "type": "string",
      "format": "uuid"
    },
    "messageType": {
      "type": "string",
      "enum": ["preservation_record", "status_update", "quality_report", "transfer_record", "maintenance_log"]
    },
    "timestamp": {
      "type": "object",
      "required": ["created"],
      "properties": {
        "created": { "type": "string", "format": "date-time" },
        "modified": { "type": "string", "format": "date-time" }
      }
    },
    "facility": {
      "type": "object",
      "required": ["id", "name"],
      "properties": {
        "id": { "type": "string" },
        "name": { "type": "string" },
        "location": {
          "type": "object",
          "properties": {
            "country": { "type": "string", "pattern": "^[A-Z]{2}$" },
            "city": { "type": "string" }
          }
        }
      }
    },
    "subject": {
      "type": "object",
      "required": ["id"],
      "properties": {
        "id": { "type": "string" },
        "anonymizedId": { "type": "string" },
        "consentId": { "type": "string" }
      }
    },
    "data": {
      "type": "object"
    },
    "meta": {
      "type": "object",
      "properties": {
        "hash": { "type": "string" },
        "signature": { "type": "string" },
        "previousHash": { "type": "string" }
      }
    }
  }
}
```

### 4.2 보존 데이터 스키마

```json
{
  "data": {
    "preservationType": "whole_body",
    "status": "long_term_storage",
    "timeline": {
      "pronouncement": "2025-01-15T08:00:00Z",
      "stabilization_start": "2025-01-15T08:15:00Z",
      "perfusion_start": "2025-01-15T10:00:00Z",
      "perfusion_complete": "2025-01-15T14:00:00Z",
      "cooldown_start": "2025-01-15T14:30:00Z",
      "cooldown_complete": "2025-01-16T02:30:00Z",
      "storage_start": "2025-01-16T03:00:00Z"
    },
    "perfusion": {
      "cryoprotectant": "M22",
      "concentration": 0.7,
      "volume_liters": 15.5,
      "duration_minutes": 240,
      "temperature_celsius": 0
    },
    "cooldown": {
      "method": "controlled_rate",
      "rate_celsius_per_minute": -1.0,
      "intermediate_holds": [
        { "temperature": -40, "duration_minutes": 30 },
        { "temperature": -80, "duration_minutes": 60 }
      ],
      "final_temperature": -196
    },
    "storage": {
      "container_type": "dewar",
      "container_id": "DEW-001",
      "position": "A1-05",
      "medium": "liquid_nitrogen",
      "temperature_celsius": -196
    },
    "quality": {
      "vitrification_score": 0.92,
      "tissue_integrity": 0.88,
      "cpa_distribution": 0.95,
      "assessment_date": "2025-01-16T06:00:00Z"
    }
  }
}
```

---

## 필드 명세

### 5.1 보존 유형

| 필드 | 타입 | 필수 | 설명 | 예시 |
|------|------|------|------|------|
| `preservationType` | string | REQUIRED | 보존 유형 | `"whole_body"` |
| `status` | string | REQUIRED | 현재 보존 상태 | `"long_term_storage"` |

**유효한 preservationType 값:**

| 값 | 설명 |
|----|------|
| `whole_body` | 전신 보존 |
| `neuro` | 두부/뇌만 보존 |
| `tissue_sample` | 개별 조직 샘플 |
| `organ` | 특정 장기 보존 |

**유효한 status 값:**

| 값 | 설명 |
|----|------|
| `pending` | 보존 대기 중 |
| `stabilization` | 초기 안정화 진행 중 |
| `perfusion` | 동결보호제 관류 진행 중 |
| `cooldown` | 온도 감소 진행 중 |
| `long_term_storage` | 저장 온도에서 유지 중 |
| `transferred` | 다른 시설로 이송됨 |
| `revived` | 성공적으로 소생됨 |

### 5.2 타임라인 필드

| 필드 | 타입 | 필수 | 설명 | 예시 |
|------|------|------|------|------|
| `pronouncement` | timestamp | REQUIRED | 법적 사망 시간 | `"2025-01-15T08:00:00Z"` |
| `stabilization_start` | timestamp | REQUIRED | 안정화 시작 | `"2025-01-15T08:15:00Z"` |
| `perfusion_start` | timestamp | CONDITIONAL | 관류 시작 | `"2025-01-15T10:00:00Z"` |
| `perfusion_complete` | timestamp | CONDITIONAL | 관류 완료 | `"2025-01-15T14:00:00Z"` |
| `cooldown_start` | timestamp | REQUIRED | 냉각 시작 | `"2025-01-15T14:30:00Z"` |
| `cooldown_complete` | timestamp | REQUIRED | 냉각 완료 | `"2025-01-16T02:30:00Z"` |
| `storage_start` | timestamp | REQUIRED | 장기 저장 시작 | `"2025-01-16T03:00:00Z"` |

### 5.3 관류 필드

| 필드 | 타입 | 필수 | 설명 | 예시 |
|------|------|------|------|------|
| `cryoprotectant` | string | REQUIRED | CPA 화합물명 | `"M22"` |
| `concentration` | number | REQUIRED | 최종 농도 (0.0-1.0) | `0.7` |
| `volume_liters` | number | REQUIRED | 총 사용량 | `15.5` |
| `duration_minutes` | integer | REQUIRED | 총 관류 시간 | `240` |
| `temperature_celsius` | number | REQUIRED | 관류 온도 | `0` |

### 5.4 품질 지표

| 필드 | 타입 | 필수 | 설명 | 범위 |
|------|------|------|------|------|
| `vitrification_score` | number | OPTIONAL | 유리화 품질 | 0.0-1.0 |
| `tissue_integrity` | number | OPTIONAL | 조직 보존 품질 | 0.0-1.0 |
| `cpa_distribution` | number | OPTIONAL | CPA 침투 균일성 | 0.0-1.0 |
| `assessment_date` | timestamp | CONDITIONAL | 품질 평가 일자 | ISO 8601 |

---

## 데이터 타입

### 6.1 사용자 정의 타입

#### PreservationType

```typescript
type PreservationType =
  | 'whole_body'
  | 'neuro'
  | 'tissue_sample'
  | 'organ';
```

#### PreservationStatus

```typescript
type PreservationStatus =
  | 'pending'
  | 'stabilization'
  | 'perfusion'
  | 'cooldown'
  | 'long_term_storage'
  | 'transferred'
  | 'revived';
```

### 6.2 Enum 값

#### 동결보호제 유형

| 코드 | 이름 | 설명 |
|------|------|------|
| `M22` | M22 유리화 용액 | 가장 발전된, 최저 독성 |
| `VM1` | 유리화 혼합물 1 | 표준 용액 |
| `DMSO` | 디메틸 설폭사이드 | 기본 CPA |
| `glycerol` | 글리세롤 | 전통적 CPA |
| `EG` | 에틸렌 글리콜 | 일반 성분 |
| `custom` | 맞춤 혼합물 | 독점 제형 |

#### 컨테이너 유형

| 코드 | 설명 |
|------|------|
| `dewar` | 진공 단열 듀어 플라스크 |
| `cryostat` | 온도 제어 저장소 |
| `transport_container` | 휴대용 극저온 용기 |
| `sample_vial` | 소형 샘플 용기 |

---

## 검증 규칙

### 7.1 필수 필드 검증

| 규칙 ID | 필드 | 검증 |
|---------|------|------|
| VAL-001 | `version` | `^\d+\.\d+\.\d+$` 패턴 일치 필수 |
| VAL-002 | `messageId` | 유효한 UUID v4 필수 |
| VAL-003 | `timestamp.created` | 유효한 ISO 8601 필수 |
| VAL-004 | `facility.id` | 빈 값 불가 |
| VAL-005 | `subject.id` | 빈 값 불가 |

### 7.2 비즈니스 로직 검증

| 규칙 ID | 설명 | 에러 코드 |
|---------|------|-----------|
| BUS-001 | `cooldown_start`는 `perfusion_complete` 이후여야 함 | `ERR_TIMELINE_ORDER` |
| BUS-002 | `storage_start`는 `cooldown_complete` 이후여야 함 | `ERR_TIMELINE_ORDER` |
| BUS-003 | `concentration`은 0.0과 1.0 사이여야 함 | `ERR_INVALID_RANGE` |
| BUS-004 | `final_temperature`는 -196°C 이하여야 함 | `ERR_INVALID_TEMPERATURE` |
| BUS-005 | 품질 점수는 0.0과 1.0 사이여야 함 | `ERR_INVALID_SCORE` |

### 7.3 에러 코드

| 코드 | 메시지 | 설명 |
|------|--------|------|
| `ERR_INVALID_FORMAT` | 잘못된 데이터 형식 | JSON 파싱 실패 |
| `ERR_MISSING_FIELD` | 필수 필드 누락 | 필수 필드 없음 |
| `ERR_INVALID_TYPE` | 잘못된 필드 타입 | 타입 불일치 |
| `ERR_TIMELINE_ORDER` | 잘못된 타임라인 순서 | 이벤트 순서 오류 |
| `ERR_INVALID_RANGE` | 범위 초과 값 | 값이 한계 초과 |
| `ERR_INVALID_TEMPERATURE` | 잘못된 온도 | 온도 제약 위반 |
| `ERR_INVALID_SCORE` | 잘못된 품질 점수 | 점수가 0.0-1.0 외 |

---

## 예제

### 8.1 유효한 보존 기록

```json
{
  "$schema": "https://wia.live/cryo-preservation/v1/schema.json",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "messageType": "preservation_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z"
  },
  "facility": {
    "id": "FAC-KR-001",
    "name": "WIA 냉동보존 연구소 서울",
    "location": {
      "country": "KR",
      "city": "서울",
      "coordinates": {
        "latitude": 37.5665,
        "longitude": 126.9780
      }
    },
    "certification": ["ISO-9001", "WIA-CRYO-CERTIFIED"]
  },
  "subject": {
    "id": "SUBJ-2025-001",
    "anonymizedId": "ANON-550e8400",
    "consentId": "CONSENT-2024-1234",
    "demographics": {
      "birthYear": 1950,
      "biologicalSex": "male",
      "bloodType": "A+"
    }
  },
  "data": {
    "preservationType": "whole_body",
    "status": "long_term_storage",
    "timeline": {
      "pronouncement": "2025-01-15T08:00:00Z",
      "stabilization_start": "2025-01-15T08:15:00Z",
      "perfusion_start": "2025-01-15T10:00:00Z",
      "perfusion_complete": "2025-01-15T14:00:00Z",
      "cooldown_start": "2025-01-15T14:30:00Z",
      "cooldown_complete": "2025-01-16T02:30:00Z",
      "storage_start": "2025-01-16T03:00:00Z"
    },
    "perfusion": {
      "cryoprotectant": "M22",
      "concentration": 0.7,
      "volume_liters": 15.5,
      "duration_minutes": 240,
      "temperature_celsius": 0
    },
    "quality": {
      "vitrification_score": 0.92,
      "tissue_integrity": 0.88,
      "cpa_distribution": 0.95,
      "assessment_date": "2025-01-16T06:00:00Z"
    }
  },
  "meta": {
    "hash": "sha256:a5b9c3d4e5f6...",
    "signature": "eyJhbGciOiJFUzI1NiIs...",
    "version": 1
  }
}
```

### 8.2 유효한 상태 업데이트

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440002",
  "messageType": "status_update",
  "timestamp": {
    "created": "2025-01-20T09:00:00Z"
  },
  "facility": {
    "id": "FAC-KR-001",
    "name": "WIA 냉동보존 연구소 서울"
  },
  "subject": {
    "id": "SUBJ-2025-001"
  },
  "data": {
    "previousStatus": "cooldown",
    "newStatus": "long_term_storage",
    "reason": "냉각 과정 성공적 완료",
    "verifiedBy": "STAFF-001",
    "notes": "모든 품질 지표가 허용 범위 내"
  },
  "meta": {
    "previousHash": "sha256:a5b9c3d4e5f6..."
  }
}
```

### 8.3 잘못된 예제 - 필수 필드 누락

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440004",
  "messageType": "preservation_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z"
  },
  "facility": {
    "name": "WIA 냉동보존 연구소 서울"
  },
  "data": {}
}
```

**에러**: `ERR_MISSING_FIELD` - 필수 필드 누락: `facility.id`, `subject`

### 8.4 잘못된 예제 - 타임라인 순서 위반

```json
{
  "data": {
    "timeline": {
      "pronouncement": "2025-01-15T08:00:00Z",
      "cooldown_start": "2025-01-15T10:00:00Z",
      "perfusion_start": "2025-01-15T14:00:00Z"
    }
  }
}
```

**에러**: `ERR_TIMELINE_ORDER` - `perfusion_start`는 `cooldown_start` 이전이어야 함

---

## 버전 이력

| 버전 | 날짜 | 변경사항 |
|------|------|----------|
| 1.0.0 | 2025-01 | 최초 릴리스 |

---

## 부록 A: 관련 표준

| 표준 | 관계 |
|------|------|
| WIA Cryo-Identity | 대상자 식별 연결 |
| WIA Cryo-Consent | 동의 기록 참조 |
| WIA Cryo-Facility | 시설 인증 데이터 |
| HL7 FHIR | 의료 데이터 상호운용성 |
| ISO 27001 | 정보 보안 관리 |

---

<div align="center">

**WIA 냉동보존 데이터 형식 표준 v1.0.0**

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

---

**© 2025 WIA 표준 위원회**

**MIT License**

</div>
