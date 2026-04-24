# WIA 냉동인간 소생 데이터 형식 표준
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

WIA 냉동인간 소생 데이터 형식 표준은 소생 절차 데이터, 의료 프로토콜, 소생 후 모니터링 및 환자 결과를 전 세계 의료 시설과 냉동보존 기관 간에 기록, 전송 및 관리하기 위한 통합 JSON 기반 형식을 정의합니다.

**핵심 목표**:
- 모든 시설에서 소생 절차 기록 표준화
- 냉동보존 제공자와 의료 시스템 간 상호운용성 확보
- 소생 과정 생애주기 전체의 데이터 무결성 보장
- 증거 기반 소생 프로토콜 최적화 지원
- 규제 준수 및 의료 감독 촉진

### 1.2 적용 범위

본 표준은 다음 데이터 도메인을 포함합니다:

| 도메인 | 설명 |
|--------|------|
| 소생 절차 | 소생을 위한 상세한 프로토콜 및 절차 |
| 의료 모니터링 | 생체 징후, 신경학적 평가, 장기 기능 |
| 성공 기준 | 정량적 및 정성적 소생 결과 지표 |
| 의료 시스템 통합 | EHR, FHIR 및 의료 시스템과의 통합 |
| 소생 후 관리 | 장기 모니터링 및 재활 데이터 |

### 1.3 설계 원칙

1. **임상적 정확성**: 의학적으로 정확하고 임상적으로 검증된 데이터 구조
2. **실시간 기능**: 소생 절차 중 실시간 모니터링 지원
3. **상호운용성**: HL7 FHIR 및 기존 의료 표준과 호환
4. **감사 가능성**: 모든 의료 결정에 대한 완전한 감사 추적
5. **프라이버시**: HIPAA 준수 데이터 보호 및 익명화

---

## 용어 정의

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **소생 대상자 (Revival Subject)** | 냉동보존 상태에서 소생을 받는 개인 |
| **소생 (Reanimation)** | 냉동보존 후 생물학적 기능을 회복하는 과정 |
| **가온 프로토콜 (Warming Protocol)** | 제어된 온도 상승 절차 |
| **관류 역전 (Perfusion Reversal)** | 동결보호제 제거 및 혈류 복원 |
| **신경학적 회복 (Neurological Recovery)** | 뇌 기능 및 의식 회복 |
| **성공 기준 (Success Criteria)** | 성공적인 소생의 측정 가능한 지표 |
| **소생 후 통합 (Post-Revival Integration)** | 의료 및 사회 재활 과정 |

### 2.2 데이터 타입

| 타입 | 설명 | 예시 |
|------|------|------|
| `string` | UTF-8 인코딩 텍스트 | `"WIA-REVIVAL-001"` |
| `number` | IEEE 754 배정밀도 | `37.0`, `98.6` |
| `integer` | 부호 있는 64비트 정수 | `120`, `80` |
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

모든 WIA 냉동인간 소생 메시지는 다음 기본 구조를 따릅니다:

```json
{
  "$schema": "https://wia.live/cryo-revival/v1/schema.json",
  "version": "1.0.0",
  "messageId": "uuid-v4-string",
  "messageType": "revival_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z"
  },
  "facility": {
    "id": "FAC-001",
    "name": "WIA 소생 의료센터",
    "location": {
      "country": "KR",
      "city": "서울"
    },
    "medicalLicense": "MED-KR-2025-001"
  },
  "subject": {
    "id": "SUBJ-001",
    "anonymizedId": "ANON-550e8400",
    "preservationRecordId": "PRES-2024-001"
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
예시: "https://wia.live/cryo-revival/v1/schema.json"
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
  - "revival_record"       : 주요 소생 절차 데이터
  - "monitoring_update"    : 실시간 생체 징후 업데이트
  - "protocol_adherence"   : 프로토콜 준수 보고서
  - "outcome_assessment"   : 소생 결과 평가
  - "integration_status"   : 의료 시스템 통합 상태
```

---

## 데이터 스키마

### 4.1 전체 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/cryo-revival/v1/schema.json",
  "title": "WIA 냉동인간 소생 기록",
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
      "enum": ["revival_record", "monitoring_update", "protocol_adherence", "outcome_assessment", "integration_status"]
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
      "required": ["id", "name", "medicalLicense"],
      "properties": {
        "id": { "type": "string" },
        "name": { "type": "string" },
        "medicalLicense": { "type": "string" },
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
        "preservationRecordId": { "type": "string" }
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

### 4.2 소생 절차 데이터 스키마

```json
{
  "data": {
    "revivalType": "full_body",
    "status": "in_progress",
    "timeline": {
      "revival_initiated": "2025-01-15T08:00:00Z",
      "warming_start": "2025-01-15T08:15:00Z",
      "warming_complete": "2025-01-15T14:00:00Z",
      "perfusion_reversal_start": "2025-01-15T14:30:00Z",
      "perfusion_reversal_complete": "2025-01-15T18:00:00Z",
      "cardiac_activity_detected": "2025-01-15T18:45:00Z",
      "spontaneous_respiration": "2025-01-15T19:30:00Z",
      "neurological_activity_detected": "2025-01-15T20:00:00Z",
      "consciousness_restored": "2025-01-16T06:00:00Z"
    },
    "warming": {
      "method": "controlled_gradient",
      "initial_temperature": -196.0,
      "target_temperature": 37.0,
      "rate_celsius_per_hour": 15.0,
      "intermediate_holds": [
        { "temperature": -80, "duration_minutes": 30 },
        { "temperature": -20, "duration_minutes": 45 },
        { "temperature": 0, "duration_minutes": 60 },
        { "temperature": 20, "duration_minutes": 30 }
      ]
    },
    "perfusionReversal": {
      "cryoprotectant_removed": "M22",
      "replacement_solution": "oxygenated_plasma",
      "flow_rate_ml_per_minute": 500,
      "duration_minutes": 210,
      "temperature_celsius": 37.0
    },
    "vitalSigns": {
      "heart_rate": 72,
      "blood_pressure": {
        "systolic": 120,
        "diastolic": 80
      },
      "respiratory_rate": 16,
      "body_temperature": 37.0,
      "oxygen_saturation": 98
    },
    "neurologicalStatus": {
      "glasgow_coma_scale": 15,
      "pupil_response": "normal",
      "eeg_activity": "normal_alpha_beta",
      "cognitive_function": "responsive",
      "memory_status": "evaluating"
    },
    "successCriteria": {
      "cardiac_function_restored": true,
      "respiratory_function_restored": true,
      "neurological_activity_present": true,
      "consciousness_level": "alert",
      "organ_function_assessment": 0.92,
      "overall_success_score": 0.88
    }
  }
}
```

---

## 필드 명세

### 5.1 소생 유형

| 필드 | 타입 | 필수 | 설명 | 예시 |
|------|------|------|------|------|
| `revivalType` | string | REQUIRED | 소생 절차 유형 | `"full_body"` |
| `status` | string | REQUIRED | 현재 소생 상태 | `"in_progress"` |

**유효한 revivalType 값:**

| 값 | 설명 |
|----|------|
| `full_body` | 전신 소생 |
| `neuro` | 뇌 중심 소생 |
| `partial` | 부분 장기 시스템 소생 |
| `experimental` | 실험적 소생 프로토콜 |

**유효한 status 값:**

| 값 | 설명 |
|----|------|
| `pending` | 소생 시작 대기 중 |
| `warming` | 온도 상승 진행 중 |
| `perfusion_reversal` | 동결보호제 제거 중 |
| `cardiac_restoration` | 심장 기능 회복 중 |
| `neurological_restoration` | 뇌 기능 회복 중 |
| `in_progress` | 소생 절차 진행 중 |
| `stabilization` | 소생 후 안정화 |
| `successful` | 소생 성공적으로 완료 |
| `partial_success` | 부분 기능 회복 |
| `unsuccessful` | 소생 실패 |

### 5.2 타임라인 필드

| 필드 | 타입 | 필수 | 설명 | 예시 |
|------|------|------|------|------|
| `revival_initiated` | timestamp | REQUIRED | 소생 절차 시작 시간 | `"2025-01-15T08:00:00Z"` |
| `warming_start` | timestamp | REQUIRED | 가온 프로토콜 시작 | `"2025-01-15T08:15:00Z"` |
| `warming_complete` | timestamp | CONDITIONAL | 가온 완료 | `"2025-01-15T14:00:00Z"` |
| `perfusion_reversal_start` | timestamp | CONDITIONAL | 관류 역전 시작 | `"2025-01-15T14:30:00Z"` |
| `perfusion_reversal_complete` | timestamp | CONDITIONAL | 관류 역전 종료 | `"2025-01-15T18:00:00Z"` |
| `cardiac_activity_detected` | timestamp | CONDITIONAL | 최초 심장 활동 | `"2025-01-15T18:45:00Z"` |
| `spontaneous_respiration` | timestamp | CONDITIONAL | 최초 자발적 호흡 | `"2025-01-15T19:30:00Z"` |
| `neurological_activity_detected` | timestamp | CONDITIONAL | 최초 뇌 활동 | `"2025-01-15T20:00:00Z"` |
| `consciousness_restored` | timestamp | CONDITIONAL | 의식 회복 | `"2025-01-16T06:00:00Z"` |

### 5.3 생체 징후 필드

| 필드 | 타입 | 필수 | 설명 | 범위 |
|------|------|------|------|------|
| `heart_rate` | integer | REQUIRED | 분당 심박수 | 0-300 |
| `blood_pressure.systolic` | integer | REQUIRED | 수축기 혈압 (mmHg) | 0-300 |
| `blood_pressure.diastolic` | integer | REQUIRED | 이완기 혈압 (mmHg) | 0-200 |
| `respiratory_rate` | integer | REQUIRED | 분당 호흡수 | 0-60 |
| `body_temperature` | number | REQUIRED | 체온 (섭씨) | -196 to 42 |
| `oxygen_saturation` | integer | REQUIRED | 산소 포화도 (%) | 0-100 |

### 5.4 성공 기준 필드

| 필드 | 타입 | 필수 | 설명 | 범위 |
|------|------|------|------|------|
| `cardiac_function_restored` | boolean | REQUIRED | 심장 기능 상태 | true/false |
| `respiratory_function_restored` | boolean | REQUIRED | 호흡 기능 상태 | true/false |
| `neurological_activity_present` | boolean | REQUIRED | 뇌 활동 상태 | true/false |
| `consciousness_level` | string | REQUIRED | 의식 수준 | 열거형 참조 |
| `organ_function_assessment` | number | OPTIONAL | 전체 장기 기능 점수 | 0.0-1.0 |
| `overall_success_score` | number | OPTIONAL | 종합 성공 지표 | 0.0-1.0 |

---

## 데이터 타입

### 6.1 사용자 정의 타입

#### RevivalType

```typescript
type RevivalType =
  | 'full_body'      // 전신
  | 'neuro'          // 신경계
  | 'partial'        // 부분
  | 'experimental';  // 실험적
```

#### RevivalStatus

```typescript
type RevivalStatus =
  | 'pending'                      // 대기
  | 'warming'                      // 가온
  | 'perfusion_reversal'           // 관류 역전
  | 'cardiac_restoration'          // 심장 회복
  | 'neurological_restoration'     // 신경 회복
  | 'in_progress'                  // 진행 중
  | 'stabilization'                // 안정화
  | 'successful'                   // 성공
  | 'partial_success'              // 부분 성공
  | 'unsuccessful';                // 실패
```

#### ConsciousnessLevel

```typescript
type ConsciousnessLevel =
  | 'unconscious'   // 무의식
  | 'minimal'       // 최소
  | 'sedated'       // 진정
  | 'drowsy'        // 졸림
  | 'alert'         // 각성
  | 'oriented';     // 지남력 있음
```

### 6.2 열거형 값

#### 가온 방법

| 코드 | 명칭 | 설명 |
|------|------|------|
| `controlled_gradient` | 제어된 구배 가온 | 정밀한 온도 제어 |
| `rapid_warming` | 급속 가온 프로토콜 | 가속화된 온도 상승 |
| `staged_warming` | 단계적 다상 가온 | 여러 가온 단계 |
| `adaptive_warming` | 적응형 AI 제어 | AI 최적화 가온 |

#### 의식 평가

| 코드 | 글래스고 혼수 척도 | 설명 |
|------|-------------------|------|
| `unconscious` | 3-8 | 반응 없음 |
| `minimal` | 9-12 | 최소 반응 |
| `sedated` | 13 | 진정 상태 |
| `drowsy` | 14 | 음성에 눈 뜸 |
| `alert` | 15 | 완전 각성 |
| `oriented` | 15+ | 각성 및 지남력 있음 |

---

## 검증 규칙

### 7.1 필수 필드 검증

| 규칙 ID | 필드 | 검증 |
|---------|------|------|
| VAL-001 | `version` | `^\d+\.\d+\.\d+$` 패턴과 일치해야 함 |
| VAL-002 | `messageId` | 유효한 UUID v4여야 함 |
| VAL-003 | `timestamp.created` | 유효한 ISO 8601이어야 함 |
| VAL-004 | `facility.medicalLicense` | 비어있지 않아야 함 |
| VAL-005 | `subject.id` | 비어있지 않아야 함 |

### 7.2 비즈니스 로직 검증

| 규칙 ID | 설명 | 오류 코드 |
|---------|------|----------|
| BUS-001 | `warming_start`는 `revival_initiated` 이후여야 함 | `ERR_TIMELINE_ORDER` |
| BUS-002 | `body_temperature`는 가온 중 증가해야 함 | `ERR_INVALID_TEMPERATURE_TREND` |
| BUS-003 | `heart_rate`는 0-300 범위여야 함 | `ERR_INVALID_RANGE` |
| BUS-004 | `glasgow_coma_scale`은 3-15 범위여야 함 | `ERR_INVALID_GCS` |
| BUS-005 | 성공 점수는 0.0-1.0 범위여야 함 | `ERR_INVALID_SCORE` |
| BUS-006 | `consciousness_restored`는 GCS >= 13일 때만 가능 | `ERR_INVALID_CONSCIOUSNESS_STATE` |

### 7.3 오류 코드

| 코드 | 메시지 | 설명 |
|------|--------|------|
| `ERR_INVALID_FORMAT` | 잘못된 데이터 형식 | JSON 파싱 실패 |
| `ERR_MISSING_FIELD` | 필수 필드 누락 | 필수 필드가 없음 |
| `ERR_INVALID_TYPE` | 잘못된 필드 타입 | 타입 불일치 |
| `ERR_TIMELINE_ORDER` | 잘못된 타임라인 순서 | 이벤트 순서 오류 |
| `ERR_INVALID_RANGE` | 범위 초과 값 | 값이 한계 초과 |
| `ERR_INVALID_TEMPERATURE_TREND` | 잘못된 온도 진행 | 온도가 증가하지 않음 |
| `ERR_INVALID_GCS` | 잘못된 글래스고 혼수 척도 | GCS가 3-15 범위 밖 |
| `ERR_INVALID_CONSCIOUSNESS_STATE` | 잘못된 의식 상태 | 생체 징후와 일치하지 않는 의식 주장 |

---

## 예제

### 8.1 유효한 소생 기록 - 진행 중

```json
{
  "$schema": "https://wia.live/cryo-revival/v1/schema.json",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "messageType": "revival_record",
  "timestamp": {
    "created": "2025-01-15T20:00:00Z",
    "modified": "2025-01-15T20:00:00Z"
  },
  "facility": {
    "id": "FAC-KR-REVIVAL-001",
    "name": "WIA 소생 의료센터 서울",
    "medicalLicense": "MED-KR-2025-001",
    "location": {
      "country": "KR",
      "city": "서울",
      "address": "서울특별시 강남구 테헤란로 123"
    },
    "accreditation": ["JCI-인증", "WIA-소생-인증"]
  },
  "subject": {
    "id": "SUBJ-2025-001",
    "anonymizedId": "ANON-550e8400",
    "preservationRecordId": "PRES-2024-001",
    "medicalRecordNumber": "MRN-2025-001"
  },
  "data": {
    "revivalType": "full_body",
    "status": "in_progress",
    "timeline": {
      "revival_initiated": "2025-01-15T08:00:00Z",
      "warming_start": "2025-01-15T08:15:00Z",
      "warming_complete": "2025-01-15T14:00:00Z",
      "perfusion_reversal_start": "2025-01-15T14:30:00Z",
      "perfusion_reversal_complete": "2025-01-15T18:00:00Z",
      "cardiac_activity_detected": "2025-01-15T18:45:00Z",
      "spontaneous_respiration": "2025-01-15T19:30:00Z",
      "neurological_activity_detected": "2025-01-15T20:00:00Z"
    },
    "warming": {
      "method": "controlled_gradient",
      "initial_temperature": -196.0,
      "target_temperature": 37.0,
      "rate_celsius_per_hour": 15.0
    },
    "vitalSigns": {
      "heart_rate": 72,
      "blood_pressure": {
        "systolic": 120,
        "diastolic": 80
      },
      "respiratory_rate": 16,
      "body_temperature": 37.0,
      "oxygen_saturation": 98
    },
    "neurologicalStatus": {
      "glasgow_coma_scale": 10,
      "pupil_response": "sluggish",
      "eeg_activity": "theta_waves_present",
      "cognitive_function": "unresponsive"
    },
    "successCriteria": {
      "cardiac_function_restored": true,
      "respiratory_function_restored": true,
      "neurological_activity_present": true,
      "consciousness_level": "minimal",
      "organ_function_assessment": 0.75,
      "overall_success_score": 0.70
    }
  },
  "meta": {
    "hash": "sha256:b6c8d5e7f9a2...",
    "signature": "eyJhbGciOiJFUzI1NiIs...",
    "version": 1
  }
}
```

### 8.2 유효한 소생 기록 - 성공

```json
{
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440002",
  "messageType": "revival_record",
  "timestamp": {
    "created": "2025-01-16T12:00:00Z"
  },
  "facility": {
    "id": "FAC-KR-REVIVAL-001",
    "name": "WIA 소생 의료센터 서울",
    "medicalLicense": "MED-KR-2025-001"
  },
  "subject": {
    "id": "SUBJ-2025-001"
  },
  "data": {
    "revivalType": "full_body",
    "status": "successful",
    "timeline": {
      "revival_initiated": "2025-01-15T08:00:00Z",
      "consciousness_restored": "2025-01-16T06:00:00Z"
    },
    "vitalSigns": {
      "heart_rate": 75,
      "blood_pressure": {
        "systolic": 118,
        "diastolic": 78
      },
      "respiratory_rate": 14,
      "body_temperature": 36.8,
      "oxygen_saturation": 99
    },
    "neurologicalStatus": {
      "glasgow_coma_scale": 15,
      "pupil_response": "normal",
      "eeg_activity": "normal_alpha_beta",
      "cognitive_function": "responsive",
      "memory_status": "short_term_intact"
    },
    "successCriteria": {
      "cardiac_function_restored": true,
      "respiratory_function_restored": true,
      "neurological_activity_present": true,
      "consciousness_level": "alert",
      "organ_function_assessment": 0.92,
      "overall_success_score": 0.88
    }
  }
}
```

---

## 버전 이력

| 버전 | 날짜 | 변경사항 |
|------|------|---------|
| 1.0.0 | 2025-01 | 최초 릴리스 |

---

## 부록 A: 관련 표준

| 표준 | 관계 |
|------|------|
| WIA 냉동보존 | 소스 보존 기록 연결 |
| WIA 냉동인간 신원 | 대상자 식별 |
| HL7 FHIR | 의료 시스템 통합 |
| HIPAA | 프라이버시 및 보안 준수 |
| ISO 27001 | 정보 보안 관리 |

---

<div align="center">

**WIA 냉동인간 소생 데이터 형식 표준 v1.0.0**

**弘益人間 (홍익인간)** - 모든 인류에게 이익을

---

**© 2025 WIA 표준 위원회**

**MIT 라이선스**

</div>
