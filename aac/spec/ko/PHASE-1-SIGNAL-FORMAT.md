# WIA AAC Signal Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-13
**Authors**: WIA (World Industry Authentication Association) / SmileStory Inc.
**License**: MIT

---

## 목차 (Table of Contents)

1. [개요 (Overview)](#1-개요-overview)
2. [용어 정의 (Terminology)](#2-용어-정의-terminology)
3. [기본 구조 (Base Structure)](#3-기본-구조-base-structure)
4. [센서별 데이터 형식 (Sensor-Specific Data)](#4-센서별-데이터-형식-sensor-specific-data)
5. [확장성 (Extensibility)](#5-확장성-extensibility)
6. [버전 관리 (Versioning)](#6-버전-관리-versioning)
7. [예제 (Examples)](#7-예제-examples)
8. [참고문헌 (References)](#8-참고문헌-references)

---

## 1. 개요 (Overview)

### 1.1 목적 (Purpose)

WIA AAC Signal Format Standard는 다양한 AAC(Augmentative and Alternative Communication, 보완대체의사소통) 센서의 출력 형식을 표준화하기 위한 규격입니다.

**핵심 목표**:
- 모든 AAC 센서가 동일한 JSON 형식으로 데이터를 출력
- 센서 제조사에 독립적인 표준 인터페이스 제공
- 소프트웨어 개발자가 다양한 센서를 일관된 방식으로 처리

### 1.2 적용 범위 (Scope)

본 표준은 다음 센서 유형을 포함합니다:

| 센서 유형 | 영문명 | 설명 |
|----------|--------|------|
| 시선 추적기 | Eye Tracker | 눈동자 움직임으로 커서 제어 |
| 스위치 | Switch | 버튼/스위치 입력 |
| 근육 센서 | Muscle Sensor (EMG) | 근전도 신호로 입력 |
| 뇌파 인터페이스 | Brain Interface (EEG/BCI) | 뇌파 신호로 입력 |
| 호흡 센서 | Breath (Sip-and-Puff) | 흡입/불기로 입력 |
| 머리 추적기 | Head Movement | 머리 움직임으로 입력 |

### 1.3 설계 원칙 (Design Principles)

1. **단순성 (Simplicity)**: JSON 기반의 명확한 구조
2. **확장성 (Extensibility)**: 새로운 센서 유형 추가 용이
3. **상호운용성 (Interoperability)**: 모든 플랫폼에서 파싱 가능
4. **정확성 (Precision)**: 고해상도 타임스탬프와 좌표
5. **검증 가능성 (Validation)**: JSON Schema로 형식 검증

---

## 2. 용어 정의 (Terminology)

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **Signal** | 센서에서 생성되는 원시 또는 처리된 데이터 |
| **Message** | 표준 형식을 따르는 하나의 JSON 객체 |
| **Stream** | 시간 순서로 정렬된 Message의 연속 |
| **Sensor** | 물리적 또는 가상의 입력 장치 |
| **Confidence** | 데이터의 신뢰도 (0.0 ~ 1.0) |
| **Validity** | 데이터의 유효성 여부 (boolean) |

### 2.2 데이터 타입

| 타입 | 설명 | 예시 |
|------|------|------|
| `string` | UTF-8 문자열 | `"eye_tracker"` |
| `number` | 64-bit IEEE 754 부동소수점 | `0.45`, `1702468800000` |
| `integer` | 정수 | `1`, `255` |
| `boolean` | 불리언 | `true`, `false` |
| `null` | 널 값 | `null` |
| `object` | JSON 객체 | `{"x": 0.5, "y": 0.3}` |
| `array` | JSON 배열 | `[1, 2, 3]` |

### 2.3 필드 요구사항

| 표기 | 의미 |
|------|------|
| **REQUIRED** | 반드시 포함해야 함 |
| **OPTIONAL** | 선택적으로 포함 가능 |
| **CONDITIONAL** | 특정 조건에서 필수 |

---

## 3. 기본 구조 (Base Structure)

### 3.1 메시지 형식 (Message Format)

모든 WIA AAC Signal Message는 다음 기본 구조를 따릅니다:

```json
{
    "$schema": "https://wia.live/aac/signal/v1/schema.json",
    "version": "1.0.0",
    "type": "<sensor_type>",
    "timestamp": {
        "unix_ms": <milliseconds>,
        "iso8601": "<ISO 8601 string>"
    },
    "sequence": <integer>,
    "device": {
        "manufacturer": "<string>",
        "model": "<string>",
        "firmware": "<string>",
        "serial": "<string>"
    },
    "data": {
        // 센서별 고유 데이터
    },
    "meta": {
        "confidence": <0.0-1.0>,
        "validity": <boolean>,
        "raw": <object>
    }
}
```

### 3.2 필드 상세

#### 3.2.1 `$schema` (OPTIONAL)

```
타입: string
형식: URI
설명: JSON Schema 위치
예시: "https://wia.live/aac/signal/v1/schema.json"
```

#### 3.2.2 `version` (REQUIRED)

```
타입: string
형식: Semantic Versioning (MAJOR.MINOR.PATCH)
설명: 스펙 버전
예시: "1.0.0"
```

#### 3.2.3 `type` (REQUIRED)

```
타입: string
설명: 센서 유형 식별자
유효값:
  - "eye_tracker"     : 시선 추적기
  - "switch"          : 스위치
  - "muscle_sensor"   : 근육 센서 (EMG)
  - "brain_interface" : 뇌파 인터페이스 (EEG/BCI)
  - "breath"          : 호흡 센서
  - "head_movement"   : 머리 추적기
  - "custom"          : 사용자 정의 (확장용)
```

#### 3.2.4 `timestamp` (REQUIRED)

```
타입: object
설명: 메시지 생성 시간

하위 필드:
  - unix_ms (REQUIRED): number
    설명: UNIX 타임스탬프 (밀리초)
    예시: 1702468800000

  - iso8601 (OPTIONAL): string
    설명: ISO 8601 형식 문자열
    예시: "2024-12-13T12:00:00.000Z"
```

#### 3.2.5 `sequence` (OPTIONAL)

```
타입: integer
설명: 메시지 순서 번호 (0부터 시작)
범위: 0 ~ 2^32-1 (32-bit unsigned integer)
용도: 패킷 손실 감지, 순서 보장
```

#### 3.2.6 `device` (REQUIRED)

```
타입: object
설명: 센서 장치 정보

하위 필드:
  - manufacturer (REQUIRED): string
    설명: 제조사명
    예시: "Tobii", "OpenBCI", "AbleNet"

  - model (REQUIRED): string
    설명: 모델명
    예시: "Pro Fusion", "Cyton", "Hitch 2"

  - firmware (OPTIONAL): string
    설명: 펌웨어 버전
    예시: "1.2.3"

  - serial (OPTIONAL): string
    설명: 시리얼 번호
    예시: "ABC123456"
```

#### 3.2.7 `data` (REQUIRED)

```
타입: object
설명: 센서별 고유 데이터 (섹션 4 참조)
```

#### 3.2.8 `meta` (OPTIONAL)

```
타입: object
설명: 메타데이터

하위 필드:
  - confidence (OPTIONAL): number
    설명: 데이터 신뢰도
    범위: 0.0 (최저) ~ 1.0 (최고)

  - validity (OPTIONAL): boolean
    설명: 데이터 유효성

  - raw (OPTIONAL): object
    설명: 원본 센서 데이터 (디버깅/기록용)
```

---

## 4. 센서별 데이터 형식 (Sensor-Specific Data)

### 4.1 Eye Tracker (시선 추적기)

`type: "eye_tracker"`

#### data 구조

```json
{
    "data": {
        "gaze": {
            "x": 0.45,
            "y": 0.32,
            "z": null
        },
        "fixation": {
            "active": true,
            "duration_ms": 850,
            "target_id": "key_A"
        },
        "pupil": {
            "left_diameter_mm": 3.5,
            "right_diameter_mm": 3.4
        },
        "blink": {
            "detected": false,
            "duration_ms": 0
        },
        "eye_validity": {
            "left": true,
            "right": true
        }
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `gaze.x` | number | REQUIRED | 시선 X 좌표 (0.0~1.0 정규화) |
| `gaze.y` | number | REQUIRED | 시선 Y 좌표 (0.0~1.0 정규화) |
| `gaze.z` | number\|null | OPTIONAL | 시선 Z 좌표 (3D 지원 시) |
| `fixation.active` | boolean | OPTIONAL | 응시(fixation) 여부 |
| `fixation.duration_ms` | integer | CONDITIONAL | 응시 지속 시간 (active=true 시) |
| `fixation.target_id` | string | OPTIONAL | 응시 대상 ID |
| `pupil.left_diameter_mm` | number | OPTIONAL | 좌안 동공 크기 (mm) |
| `pupil.right_diameter_mm` | number | OPTIONAL | 우안 동공 크기 (mm) |
| `blink.detected` | boolean | OPTIONAL | 눈 깜빡임 감지 여부 |
| `blink.duration_ms` | integer | CONDITIONAL | 깜빡임 지속 시간 |
| `eye_validity.left` | boolean | OPTIONAL | 좌안 데이터 유효성 |
| `eye_validity.right` | boolean | OPTIONAL | 우안 데이터 유효성 |

---

### 4.2 Switch (스위치)

`type: "switch"`

#### data 구조

```json
{
    "data": {
        "switch_id": 1,
        "channel": "primary",
        "state": "pressed",
        "duration_ms": 150,
        "pressure": null,
        "repeat_count": 1
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `switch_id` | integer | REQUIRED | 스위치 식별자 (1부터 시작) |
| `channel` | string | OPTIONAL | 채널명 (예: "primary", "secondary") |
| `state` | string | REQUIRED | 상태: "pressed", "released", "held" |
| `duration_ms` | integer | OPTIONAL | 누름 지속 시간 (ms) |
| `pressure` | number\|null | OPTIONAL | 압력 (0.0~1.0, 지원 시) |
| `repeat_count` | integer | OPTIONAL | 연속 눌림 횟수 |

#### state 값 정의

| 값 | 설명 |
|----|------|
| `"pressed"` | 스위치가 눌림 |
| `"released"` | 스위치가 해제됨 |
| `"held"` | 스위치가 일정 시간 이상 눌린 상태 유지 |

---

### 4.3 Muscle Sensor / EMG (근육 센서)

`type: "muscle_sensor"`

#### data 구조

```json
{
    "data": {
        "channel_id": 1,
        "muscle_group": "cheek_left",
        "activation": 0.75,
        "raw_uv": 125.5,
        "envelope_uv": 85.2,
        "threshold_exceeded": true,
        "gesture": "single_twitch"
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `channel_id` | integer | REQUIRED | 채널 번호 (1부터 시작) |
| `muscle_group` | string | OPTIONAL | 근육 부위 |
| `activation` | number | REQUIRED | 활성도 (0.0~1.0 정규화) |
| `raw_uv` | number | OPTIONAL | 원시 EMG 값 (µV) |
| `envelope_uv` | number | OPTIONAL | 정류/평활화된 값 (µV) |
| `threshold_exceeded` | boolean | OPTIONAL | 임계값 초과 여부 |
| `gesture` | string | OPTIONAL | 인식된 제스처 |

#### muscle_group 권장 값

```
"cheek_left", "cheek_right"    : 볼
"jaw"                          : 턱
"forehead"                     : 이마
"eyebrow_left", "eyebrow_right": 눈썹
"forearm_left", "forearm_right": 전완
"custom"                       : 사용자 정의
```

#### gesture 예시 값

```
"single_twitch"   : 단일 수축
"double_twitch"   : 이중 수축
"sustained_hold"  : 지속 수축
"release"         : 이완
```

---

### 4.4 Brain Interface / EEG/BCI (뇌파 인터페이스)

`type: "brain_interface"`

#### data 구조

```json
{
    "data": {
        "channel_count": 8,
        "sample_rate_hz": 250,
        "channels": [
            {"id": "Fp1", "value_uv": 12.5},
            {"id": "Fp2", "value_uv": 11.8},
            {"id": "C3", "value_uv": 8.2},
            {"id": "C4", "value_uv": 9.1},
            {"id": "P3", "value_uv": 7.5},
            {"id": "P4", "value_uv": 7.8},
            {"id": "O1", "value_uv": 15.2},
            {"id": "O2", "value_uv": 14.9}
        ],
        "bands": {
            "delta": 0.15,
            "theta": 0.22,
            "alpha": 0.35,
            "beta": 0.18,
            "gamma": 0.10
        },
        "classification": {
            "intent": "select",
            "confidence": 0.82
        },
        "artifacts": {
            "eye_blink": false,
            "muscle": false,
            "movement": false
        }
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `channel_count` | integer | REQUIRED | 채널 수 |
| `sample_rate_hz` | number | OPTIONAL | 샘플 레이트 (Hz) |
| `channels` | array | REQUIRED | 채널별 데이터 배열 |
| `channels[].id` | string | REQUIRED | 채널 ID (10-20 시스템) |
| `channels[].value_uv` | number | REQUIRED | 전압값 (µV) |
| `bands` | object | OPTIONAL | 주파수 대역 파워 |
| `bands.delta` | number | OPTIONAL | Delta (0.5-4 Hz) 상대 파워 |
| `bands.theta` | number | OPTIONAL | Theta (4-8 Hz) 상대 파워 |
| `bands.alpha` | number | OPTIONAL | Alpha (8-13 Hz) 상대 파워 |
| `bands.beta` | number | OPTIONAL | Beta (13-30 Hz) 상대 파워 |
| `bands.gamma` | number | OPTIONAL | Gamma (30+ Hz) 상대 파워 |
| `classification` | object | OPTIONAL | BCI 분류 결과 |
| `classification.intent` | string | CONDITIONAL | 인식된 의도 |
| `classification.confidence` | number | CONDITIONAL | 분류 신뢰도 |
| `artifacts` | object | OPTIONAL | 아티팩트 감지 |

#### 10-20 시스템 채널 ID

```
Fp1, Fp2 : 전두극 (Frontopolar)
F3, F4   : 전두 (Frontal)
C3, C4   : 중심 (Central)
P3, P4   : 두정 (Parietal)
O1, O2   : 후두 (Occipital)
T3, T4   : 측두 (Temporal)
Fz, Cz, Pz: 정중선 (Midline)
```

---

### 4.5 Breath / Sip-and-Puff (호흡 센서)

`type: "breath"`

#### data 구조

```json
{
    "data": {
        "action": "sip",
        "pressure_kpa": 2.5,
        "pressure_normalized": 0.65,
        "duration_ms": 300,
        "intensity": "soft",
        "baseline_kpa": 101.3
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `action` | string | REQUIRED | 동작 유형 |
| `pressure_kpa` | number | OPTIONAL | 절대 압력 (kPa) |
| `pressure_normalized` | number | OPTIONAL | 정규화 압력 (0.0~1.0) |
| `duration_ms` | integer | OPTIONAL | 동작 지속 시간 (ms) |
| `intensity` | string | OPTIONAL | 강도 수준 |
| `baseline_kpa` | number | OPTIONAL | 기준 대기압 (kPa) |

#### action 값 정의

| 값 | 설명 |
|----|------|
| `"sip"` | 부드러운 흡입 |
| `"hard_sip"` | 강한 흡입 |
| `"puff"` | 부드러운 불기 |
| `"hard_puff"` | 강한 불기 |
| `"neutral"` | 중립 (동작 없음) |

#### intensity 값 정의

| 값 | 설명 |
|----|------|
| `"soft"` | 약한 강도 |
| `"medium"` | 중간 강도 |
| `"hard"` | 강한 강도 |

---

### 4.6 Head Movement (머리 움직임)

`type: "head_movement"`

#### data 구조

```json
{
    "data": {
        "position": {
            "x": 0.55,
            "y": 0.48
        },
        "rotation": {
            "pitch": 5.2,
            "yaw": -3.1,
            "roll": 0.5
        },
        "velocity": {
            "x": 0.02,
            "y": -0.01
        },
        "gesture": "dwell",
        "dwell_time_ms": 1200,
        "face_detected": true
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `position.x` | number | REQUIRED | X 위치 (0.0~1.0 정규화) |
| `position.y` | number | REQUIRED | Y 위치 (0.0~1.0 정규화) |
| `rotation.pitch` | number | OPTIONAL | 상하 회전 (도) |
| `rotation.yaw` | number | OPTIONAL | 좌우 회전 (도) |
| `rotation.roll` | number | OPTIONAL | 기울임 (도) |
| `velocity.x` | number | OPTIONAL | X 속도 (정규화/초) |
| `velocity.y` | number | OPTIONAL | Y 속도 (정규화/초) |
| `gesture` | string | OPTIONAL | 인식된 제스처 |
| `dwell_time_ms` | integer | OPTIONAL | 정지 시간 (ms) |
| `face_detected` | boolean | OPTIONAL | 얼굴 인식 여부 |

#### gesture 값 정의

| 값 | 설명 |
|----|------|
| `"dwell"` | 일정 시간 정지 (선택) |
| `"nod"` | 끄덕임 (예) |
| `"shake"` | 가로젓기 (아니오) |
| `"tilt_left"` | 왼쪽 기울임 |
| `"tilt_right"` | 오른쪽 기울임 |
| `"none"` | 제스처 없음 |

---

## 5. 확장성 (Extensibility)

### 5.1 사용자 정의 센서 (Custom Sensors)

`type: "custom"` 을 사용하여 표준에 정의되지 않은 센서를 지원합니다:

```json
{
    "version": "1.0.0",
    "type": "custom",
    "timestamp": { "unix_ms": 1702468800000 },
    "device": {
        "manufacturer": "CustomCorp",
        "model": "MySensor"
    },
    "data": {
        "custom_type": "gesture_sensor",
        "custom_data": {
            "gesture": "wave",
            "hand": "left",
            "confidence": 0.9
        }
    }
}
```

### 5.2 필드 확장

센서별 `data` 객체 내에 추가 필드를 포함할 수 있습니다. 표준에 정의되지 않은 필드는 `x_` 접두사를 권장합니다:

```json
{
    "data": {
        "gaze": { "x": 0.5, "y": 0.5 },
        "x_vendor_specific_field": "custom_value",
        "x_calibration_status": "active"
    }
}
```

### 5.3 하위 호환성 (Backward Compatibility)

- MAJOR 버전 변경 시에만 필드 삭제 가능
- MINOR 버전에서는 새 필드 추가만 가능
- 파서는 알 수 없는 필드를 무시해야 함 (MUST ignore)

---

## 6. 버전 관리 (Versioning)

### 6.1 버전 형식

Semantic Versioning 2.0.0을 따릅니다:

```
MAJOR.MINOR.PATCH

예: 1.2.3
    │ │ └─ PATCH: 버그 수정, 문서 개선
    │ └─── MINOR: 하위 호환 가능한 기능 추가
    └───── MAJOR: 호환되지 않는 변경
```

### 6.2 버전 협상

클라이언트와 서버 간 버전 협상:

1. 클라이언트가 지원하는 버전 목록 전송
2. 서버가 호환 가능한 최신 버전 선택
3. 선택된 버전으로 통신

```json
{
    "supported_versions": ["1.0.0", "1.1.0", "1.2.0"],
    "selected_version": "1.2.0"
}
```

---

## 7. 예제 (Examples)

### 7.1 Eye Tracker 전체 예제

```json
{
    "$schema": "https://wia.live/aac/signal/v1/schema.json",
    "version": "1.0.0",
    "type": "eye_tracker",
    "timestamp": {
        "unix_ms": 1702468800000,
        "iso8601": "2024-12-13T12:00:00.000Z"
    },
    "sequence": 1234,
    "device": {
        "manufacturer": "Tobii",
        "model": "Pro Fusion",
        "firmware": "2.1.0",
        "serial": "TPF-2024-001234"
    },
    "data": {
        "gaze": {
            "x": 0.45,
            "y": 0.32,
            "z": null
        },
        "fixation": {
            "active": true,
            "duration_ms": 850,
            "target_id": "keyboard_key_A"
        },
        "pupil": {
            "left_diameter_mm": 3.5,
            "right_diameter_mm": 3.4
        },
        "blink": {
            "detected": false,
            "duration_ms": 0
        },
        "eye_validity": {
            "left": true,
            "right": true
        }
    },
    "meta": {
        "confidence": 0.95,
        "validity": true
    }
}
```

### 7.2 Switch 전체 예제

```json
{
    "version": "1.0.0",
    "type": "switch",
    "timestamp": {
        "unix_ms": 1702468800150
    },
    "sequence": 42,
    "device": {
        "manufacturer": "AbleNet",
        "model": "Hitch 2"
    },
    "data": {
        "switch_id": 1,
        "channel": "primary",
        "state": "pressed",
        "duration_ms": 150,
        "pressure": null,
        "repeat_count": 1
    },
    "meta": {
        "confidence": 1.0,
        "validity": true
    }
}
```

### 7.3 Brain Interface 전체 예제

```json
{
    "version": "1.0.0",
    "type": "brain_interface",
    "timestamp": {
        "unix_ms": 1702468800004,
        "iso8601": "2024-12-13T12:00:00.004Z"
    },
    "sequence": 5678,
    "device": {
        "manufacturer": "OpenBCI",
        "model": "Cyton",
        "firmware": "3.1.2"
    },
    "data": {
        "channel_count": 8,
        "sample_rate_hz": 250,
        "channels": [
            {"id": "Fp1", "value_uv": 12.5},
            {"id": "Fp2", "value_uv": 11.8},
            {"id": "C3", "value_uv": 8.2},
            {"id": "C4", "value_uv": 9.1},
            {"id": "P3", "value_uv": 7.5},
            {"id": "P4", "value_uv": 7.8},
            {"id": "O1", "value_uv": 15.2},
            {"id": "O2", "value_uv": 14.9}
        ],
        "bands": {
            "delta": 0.15,
            "theta": 0.22,
            "alpha": 0.35,
            "beta": 0.18,
            "gamma": 0.10
        },
        "classification": {
            "intent": "select",
            "confidence": 0.82
        }
    },
    "meta": {
        "confidence": 0.82,
        "validity": true
    }
}
```

---

## 8. 참고문헌 (References)

### 표준 문서

- [JSON (ECMA-404)](https://www.ecma-international.org/publications-and-standards/standards/ecma-404/)
- [JSON Schema draft-07](https://json-schema.org/specification-links.html#draft-7)
- [Semantic Versioning 2.0.0](https://semver.org/)
- [ISO 8601 Date/Time Format](https://www.iso.org/iso-8601-date-and-time-format.html)
- [USB HID Specification 1.11](https://www.usb.org/hid)
- [W3C Pointer Events](https://www.w3.org/TR/pointerevents/)

### 센서 제조사 문서

- [Tobii Pro SDK Documentation](https://developer.tobiipro.com/)
- [OpenBCI Documentation](https://docs.openbci.com/)
- [BrainFlow Documentation](https://brainflow.readthedocs.io/)
- [Emotiv Cortex API](https://emotiv.gitbook.io/cortex-api)

### 관련 프로젝트

- [Intel ACAT](https://github.com/intel/acat)
- [Lab Streaming Layer](https://github.com/sccn/labstreaminglayer)

---

## 부록 A: JSON Schema 파일 목록

| 파일명 | 설명 |
|--------|------|
| `wia-aac-signal-v1.schema.json` | 기본 스키마 |
| `eye-tracker.schema.json` | Eye Tracker data 스키마 |
| `switch.schema.json` | Switch data 스키마 |
| `muscle-sensor.schema.json` | Muscle Sensor data 스키마 |
| `brain-interface.schema.json` | Brain Interface data 스키마 |
| `breath.schema.json` | Breath data 스키마 |
| `head-movement.schema.json` | Head Movement data 스키마 |

---

## 부록 B: 좌표계 규약

### B.1 정규화 좌표 (Normalized Coordinates)

```
원점: 좌상단 (0.0, 0.0)
범위: 0.0 ~ 1.0

    0.0      0.5      1.0  X
     ┌────────┬────────┐
 0.0 │        │        │
     │        │        │
 0.5 ├────────┼────────┤
     │        │        │
 1.0 │        │        │
     └────────┴────────┘
                        Y
```

### B.2 회전 각도 (Rotation Angles)

```
Pitch (상하): + = 위로, - = 아래로
Yaw (좌우):   + = 오른쪽, - = 왼쪽
Roll (기울임): + = 시계방향, - = 반시계방향

단위: 도 (degrees)
범위: -180 ~ +180
```

---

<div align="center">

**WIA AAC Signal Format Standard v1.0.0**

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

🤟

---

**© 2025 SmileStory Inc. / WIA**

**MIT License**

</div>
