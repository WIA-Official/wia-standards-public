# 4장: 데이터 형식 사양 (1단계)

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

1. WIA 표준의 JSON 기반 데이터 형식의 필요성과 이점 이해
2. 센서 데이터 스키마의 모든 필수 및 선택 필드 식별
3. 경보 이벤트 수명 주기와 상태 전환 설명
4. JSON Schema를 사용하여 데이터 유효성 검사 구현
5. 일반적인 유효성 검사 오류 인식 및 수정

---

## 개요

WIA 표준의 1단계는 모든 소방 안전 시스템 구성 요소가 정보를 일관되게 표현하고 교환할 수 있도록 하는 기본 데이터 형식 사양을 설정합니다. 이 장에서는 JSON 스키마, 데이터 유효성 검사 요구 사항 및 구현 지침의 포괄적인 기술 세부 사항을 제공합니다.

---

## 데이터 형식 표준화가 중요한 이유

### 상호 운용성의 기초

표준화된 데이터 형식이 없으면 시스템이 효과적으로 통신할 수 없습니다:

```
문제점: 호환되지 않는 데이터 표현

벤더 A 센서 판독값:
{"dev":12345,"typ":1,"val":3.2,"ts":1640995200}

벤더 B 센서 판독값:
{"deviceIdentifier":"SD-0012345","sensorType":"SMOKE_OPTICAL",
 "measurementValue":{"value":3.2,"unit":"OD_PER_METER"},
 "timestamp":"2021-12-31T12:00:00Z"}

결과: 통합에는 각 벤더에 대한 사용자 정의 파싱이 필요합니다
비용: 벤더 통합당 $15,000-$50,000

해결책: WIA 표준 데이터 형식

모든 벤더가 사용:
{"sensorId":"550e8400-e29b-41d4-a716-446655440000",
 "type":"smoke","status":"normal",
 "readings":{"value":3.2,"unit":"OD_PER_METER",
             "timestamp":"2021-12-31T12:00:00Z"},
 ...}

결과: 보편적 호환성, 통합 비용 제로
```

### JSON 형식의 이점

**사람이 읽을 수 있음:**
```json
// 읽고 이해하기 쉬움
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "status": "alarm"
}
```

**기계 분석 가능:**
- 모든 최신 프로그래밍 언어에서 기본 지원
- 효율적인 파싱 라이브러리 사용 가능
- 자동 직렬화/역직렬화

**유효성 검사됨:**
- JSON Schema를 통한 자동 유효성 검사
- 타입 검사 및 제약 조건
- 문서 자동 생성

**확장 가능:**
- 호환성을 깨지 않고 새 필드 추가 가능
- 선택적 필드가 진화 지원
- 버전 독립적 설계

---

## 핵심 데이터 스키마

### 1. 센서 데이터 스키마

#### 완전한 스키마 정의

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA Fire Safety Sensor Data",
  "type": "object",
  "required": ["sensorId", "type", "location", "status", "readings", "metadata"],
  "properties": {
    "sensorId": {
      "type": "string",
      "format": "uuid",
      "description": "전역적으로 고유한 센서 식별자 (UUID v4)"
    },
    "type": {
      "type": "string",
      "enum": ["smoke", "heat", "flame", "co", "multi"],
      "description": "센서 유형 분류"
    },
    "location": {
      "type": "object",
      "required": ["building", "floor", "zone"],
      "properties": {
        "building": {
          "type": "string",
          "description": "건물 식별자 또는 이름"
        },
        "floor": {
          "type": "integer",
          "description": "층 번호 (0=지상, 음수=지하)"
        },
        "zone": {
          "type": "string",
          "description": "층 내 구역 식별자"
        },
        "coordinates": {
          "type": "object",
          "description": "물리적 좌표 (선택사항)",
          "properties": {
            "x": {"type": "number", "description": "X 좌표 (미터 단위)"},
            "y": {"type": "number", "description": "Y 좌표 (미터 단위)"},
            "z": {"type": "number", "description": "Z 좌표 (미터 단위, 선택사항)"}
          }
        }
      }
    },
    "status": {
      "type": "string",
      "enum": ["normal", "alarm", "trouble", "disabled"],
      "description": "현재 센서 상태"
    },
    "readings": {
      "type": "object",
      "required": ["value", "unit", "timestamp"],
      "properties": {
        "value": {
          "type": "number",
          "description": "센서 판독값"
        },
        "unit": {
          "type": "string",
          "description": "측정 단위"
        },
        "timestamp": {
          "type": "string",
          "format": "date-time",
          "description": "UTC 시간대의 ISO 8601 타임스탬프"
        }
      }
    },
    "metadata": {
      "type": "object",
      "required": ["manufacturer", "model", "firmwareVersion"],
      "properties": {
        "manufacturer": {"type": "string"},
        "model": {"type": "string"},
        "firmwareVersion": {"type": "string"},
        "installationDate": {"type": "string", "format": "date-time"},
        "lastMaintenance": {"type": "string", "format": "date-time"}
      }
    }
  }
}
```

#### 센서 데이터 인스턴스 예제

**연기 감지기 - 정상 상태:**
```json
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "location": {
    "building": "Main Tower",
    "floor": 12,
    "zone": "East Wing E12-A",
    "coordinates": {
      "x": 45.2,
      "y": 23.8,
      "z": 3.5
    }
  },
  "status": "normal",
  "readings": {
    "value": 0.8,
    "unit": "OD/meter",
    "timestamp": "2025-12-27T14:32:15Z"
  },
  "metadata": {
    "manufacturer": "SafetyTech Inc",
    "model": "ST-5000",
    "firmwareVersion": "2.4.1",
    "installationDate": "2024-06-15T10:00:00Z",
    "lastMaintenance": "2025-10-12T09:30:00Z"
  }
}
```

**열 감지기 - 경보 상태:**
```json
{
  "sensorId": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "type": "heat",
  "location": {
    "building": "West Annex",
    "floor": 3,
    "zone": "Storage Room W3-S2"
  },
  "status": "alarm",
  "readings": {
    "value": 72.5,
    "unit": "°C",
    "timestamp": "2025-12-27T14:35:42Z"
  },
  "metadata": {
    "manufacturer": "FireGuard Systems",
    "model": "FG-HT200",
    "firmwareVersion": "1.8.3",
    "installationDate": "2023-03-22T08:15:00Z",
    "lastMaintenance": "2025-09-05T11:20:00Z"
  }
}
```

**CO 감지기 - 고장 상태:**
```json
{
  "sensorId": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
  "type": "co",
  "location": {
    "building": "Parking Structure",
    "floor": -2,
    "zone": "Level B2 North"
  },
  "status": "trouble",
  "readings": {
    "value": 0,
    "unit": "ppm",
    "timestamp": "2025-12-27T14:38:20Z"
  },
  "metadata": {
    "manufacturer": "AirWatch Pro",
    "model": "AW-CO500",
    "firmwareVersion": "3.2.0",
    "installationDate": "2024-11-10T13:45:00Z",
    "lastMaintenance": "2025-11-08T10:30:00Z"
  }
}
```

#### 필드 사양

**sensorId (필수):**
- 형식: UUID v4 (RFC 4122)
- 전역적으로 고유해야 함
- 장치 제조 시 생성
- 장치 수명 동안 절대 변경되지 않음
- 예: `550e8400-e29b-41d4-a716-446655440000`

**type (필수):**
- 유효한 값: `smoke`, `heat`, `flame`, `co`, `multi`
- `multi`는 다중 기준 감지를 나타냄
- 장치 제조 시 고정
- 예상되는 판독값 형식을 결정

**location (필수):**
- `building`: 건물 식별자 (문자열, 최대 100자)
- `floor`: 층 번호 (정수, 0=지상, 음수=지하)
- `zone`: 구역 식별자 (문자열, 최대 50자)
- `coordinates`: 선택적 물리적 위치 (미터 단위)

**status (필수):**
- `normal`: 정상 작동, 경보 없음
- `alarm`: 화재 상태 감지
- `trouble`: 주의가 필요한 고장 상태
- `disabled`: 수동으로 비활성화되었거나 오프라인

**readings (필수):**
- `value`: 숫자 센서 판독값
- `unit`: 측정 단위 (표준화된 단위 필요)
- `timestamp`: UTC 시간대의 ISO 8601 형식

**센서 유형별 일반적인 단위:**
```
연기 감지기:
  - OD/meter (미터당 광학 밀도)
  - %/foot (피트당 차광)

열 감지기:
  - °C (섭씨 온도)
  - °F (화씨 온도)
  - °C/min (상승 속도)

화염 감지기:
  - intensity (0-100 척도)
  - W/m² (적외선 복사)

CO 감지기:
  - ppm (백만분율)
  - mg/m³ (세제곱미터당 밀리그램)
```

**metadata (필수):**
- 장치 식별 및 이력
- `manufacturer`: 회사 이름
- `model`: 모델 번호/이름
- `firmwareVersion`: 현재 펌웨어 버전
- `installationDate`: 장치가 설치된 날짜
- `lastMaintenance`: 마지막 유지보수/테스트 날짜

---

### 2. 경보 이벤트 스키마

#### 완전한 스키마 정의

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA Fire Safety Alarm Event",
  "type": "object",
  "required": ["eventId", "eventType", "priority", "source", "timestamp", "status"],
  "properties": {
    "eventId": {
      "type": "string",
      "format": "uuid",
      "description": "전역적으로 고유한 이벤트 식별자 (UUID v4)"
    },
    "eventType": {
      "type": "string",
      "enum": ["fire", "supervisory", "trouble", "test"],
      "description": "경보 이벤트 유형"
    },
    "priority": {
      "type": "string",
      "enum": ["critical", "high", "medium", "low"],
      "description": "이벤트 우선순위 수준"
    },
    "source": {
      "type": "object",
      "required": ["deviceId", "deviceType", "location"],
      "properties": {
        "deviceId": {"type": "string", "format": "uuid"},
        "deviceType": {"type": "string"},
        "location": {
          "type": "object",
          "properties": {
            "building": {"type": "string"},
            "floor": {"type": "integer"},
            "zone": {"type": "string"}
          }
        }
      }
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "이벤트 발생 시간 (ISO 8601 UTC)"
    },
    "description": {
      "type": "string",
      "description": "사람이 읽을 수 있는 이벤트 설명"
    },
    "notifications": {
      "type": "array",
      "items": {"type": "string"},
      "description": "알림 대상 목록"
    },
    "status": {
      "type": "string",
      "enum": ["active", "acknowledged", "resolved"],
      "description": "현재 이벤트 상태"
    },
    "acknowledgedBy": {
      "type": "string",
      "description": "확인한 사용자 ID (선택사항)"
    },
    "acknowledgedAt": {
      "type": "string",
      "format": "date-time",
      "description": "확인 타임스탬프 (선택사항)"
    },
    "resolvedAt": {
      "type": "string",
      "format": "date-time",
      "description": "해결 타임스탬프 (선택사항)"
    }
  }
}
```

#### 경보 이벤트 인스턴스 예제

**화재 경보 - 활성:**
```json
{
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "eventType": "fire",
  "priority": "critical",
  "source": {
    "deviceId": "550e8400-e29b-41d4-a716-446655440000",
    "deviceType": "smoke_detector",
    "location": {
      "building": "Main Tower",
      "floor": 12,
      "zone": "East Wing E12-A"
    }
  },
  "timestamp": "2025-12-27T14:32:15Z",
  "description": "12층 동쪽 구역 E12-A에서 연기 감지",
  "notifications": [
    "panel",
    "monitoring-center",
    "emergency-services",
    "building-management",
    "security-desk"
  ],
  "status": "active",
  "acknowledgedBy": null,
  "acknowledgedAt": null,
  "resolvedAt": null
}
```

**감시 경보 - 확인됨:**
```json
{
  "eventId": "1a2b3c4d-5e6f-4a5b-8c9d-0e1f2a3b4c5d",
  "eventType": "supervisory",
  "priority": "high",
  "source": {
    "deviceId": "b3c4d5e6-f7a8-4b9c-0d1e-2f3a4b5c6d7e",
    "deviceType": "sprinkler_valve",
    "location": {
      "building": "Main Tower",
      "floor": 10,
      "zone": "Mechanical Room M10"
    }
  },
  "timestamp": "2025-12-27T10:15:30Z",
  "description": "스프링클러 밸브 변조 감지 - 10층 기계실",
  "notifications": [
    "panel",
    "monitoring-center",
    "maintenance-team"
  ],
  "status": "acknowledged",
  "acknowledgedBy": "maintenance-tech-042",
  "acknowledgedAt": "2025-12-27T10:18:45Z",
  "resolvedAt": null
}
```

**고장 이벤트 - 해결됨:**
```json
{
  "eventId": "8e7f6d5c-4b3a-4291-8a7b-6c5d4e3f2a1b",
  "eventType": "trouble",
  "priority": "medium",
  "source": {
    "deviceId": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
    "deviceType": "co_detector",
    "location": {
      "building": "Parking Structure",
      "floor": -2,
      "zone": "Level B2 North"
    }
  },
  "timestamp": "2025-12-27T08:22:10Z",
  "description": "CO 감지기 통신 실패 - 주차장 B2층",
  "notifications": [
    "panel",
    "maintenance-team"
  ],
  "status": "resolved",
  "acknowledgedBy": "tech-156",
  "acknowledgedAt": "2025-12-27T09:05:30Z",
  "resolvedAt": "2025-12-27T11:42:15Z"
}
```

#### 이벤트 수명 주기

```
경보 이벤트 상태 머신:

┌────────────────────────────────────────────────────────┐
│                                                        │
│  [트리거됨] ──────────────────────────┐              │
│       │                                 │              │
│       │ 이벤트 생성                     │              │
│       │                                 │              │
│       ▼                                 │              │
│   [활성] ───────────────────────┐    │              │
│       │                            │    │              │
│       │ 사용자 확인                │    │ 자동 해제  │
│       │                            │    │ (테스트 모드) │
│       ▼                            │    │              │
│ [확인됨]                          │    │              │
│       │                            │    │              │
│       │ 조사 완료                  │    │              │
│       │ 상태 해소                  │    │              │
│       │                            │    │              │
│       ▼                            ▼    ▼              │
│   [해결됨] ◄─────────────────────────────────────────┘
│                                                         │
└─────────────────────────────────────────────────────────┘

타이밍 요구 사항:
- 트리거됨 → 활성: 즉시 (0ms)
- 활성 → 확인됨: 사용자 조치 (일반적으로 30초-5분)
- 확인됨 → 해결됨: 가변적 (조사 + 문제 해결)
```

---

### 3. 장치 메타데이터 스키마

#### 완전한 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA Fire Safety Device Metadata",
  "type": "object",
  "required": ["deviceId", "deviceType", "manufacturer", "model"],
  "properties": {
    "deviceId": {
      "type": "string",
      "format": "uuid"
    },
    "deviceType": {
      "type": "string",
      "enum": [
        "smoke_detector",
        "heat_detector",
        "flame_detector",
        "co_detector",
        "multi_sensor",
        "manual_pull_station",
        "horn",
        "strobe",
        "horn_strobe",
        "speaker",
        "control_panel",
        "annunciator",
        "relay_module",
        "interface_module"
      ]
    },
    "manufacturer": {"type": "string"},
    "model": {"type": "string"},
    "firmwareVersion": {"type": "string"},
    "hardwareRevision": {"type": "string"},
    "serialNumber": {"type": "string"},
    "manufacturingDate": {"type": "string", "format": "date"},
    "installationDate": {"type": "string", "format": "date-time"},
    "warrantyExpiration": {"type": "string", "format": "date"},
    "lastMaintenance": {"type": "string", "format": "date-time"},
    "maintenanceInterval": {
      "type": "integer",
      "description": "유지보수 간격 일수"
    },
    "certifications": {
      "type": "array",
      "items": {"type": "string"},
      "description": "UL, FM, EN54 등"
    }
  }
}
```

---

## 데이터 유효성 검사

### 유효성 검사 요구 사항

모든 데이터는 처리하기 전에 유효성을 검사해야 합니다:

**스키마 유효성 검사:**
```javascript
const Ajv = require('ajv');
const ajv = new Ajv();

// 스키마 로드
const sensorSchema = require('./schemas/sensor-data.json');
const validate = ajv.compile(sensorSchema);

// 데이터 유효성 검사
function validateSensorData(data) {
  const valid = validate(data);
  if (!valid) {
    console.error('유효성 검사 오류:', validate.errors);
    return false;
  }
  return true;
}

// 사용 예제
const sensorData = {
  sensorId: "550e8400-e29b-41d4-a716-446655440000",
  type: "smoke",
  // ... 나머지 데이터
};

if (validateSensorData(sensorData)) {
  processSensorData(sensorData);
}
```

### 일반적인 유효성 검사 오류

**잘못된 UUID 형식:**
```json
// 잘못됨
{"sensorId": "12345"}

// 올바름
{"sensorId": "550e8400-e29b-41d4-a716-446655440000"}
```

**잘못된 열거형 값:**
```json
// 잘못됨
{"type": "smoke-detector"}

// 올바름
{"type": "smoke"}
```

**필수 필드 누락:**
```json
// 잘못됨 (status 누락)
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "location": {...},
  "readings": {...}
}

// 올바름
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "location": {...},
  "status": "normal",
  "readings": {...},
  "metadata": {...}
}
```

**잘못된 타임스탬프 형식:**
```json
// 잘못됨
{"timestamp": "2025-12-27 14:32:15"}

// 올바름
{"timestamp": "2025-12-27T14:32:15Z"}
```

---

## 핵심 요점

1. **표준화된 JSON 스키마**는 모든 벤더 간에 범용 데이터 표현을 보장합니다.

2. **센서 데이터 스키마**는 위치, 상태, 판독값 및 메타데이터를 포함한 완전한 장치 정보를 제공합니다.

3. **경보 이벤트 스키마**는 트리거에서 해결까지 완전한 수명 주기를 추적합니다.

4. **유효성 검사는 필수**이며 JSON Schema를 사용하여 데이터 무결성을 보장합니다.

5. **UUID v4 식별자**는 조정 없이 전역 고유성을 보장합니다.

---

## 복습 문제

1. JSON이 데이터 형식 표준으로 선택된 이유는 무엇인가요?
2. 다섯 가지 유효한 센서 유형은 무엇인가요?
3. 경보 이벤트의 수명 주기는 어떻게 되나요?
4. 센서 ID가 UUID v4 형식이어야 하는 이유는 무엇인가요?
5. 데이터 무결성을 보장하는 유효성 검사 메커니즘은 무엇인가요?

---

## 다음 단계

5장에서는 이러한 데이터 형식을 사용하여 시스템 통신에 사용되는 API 인터페이스를 정의하는 2단계를 탐구합니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
