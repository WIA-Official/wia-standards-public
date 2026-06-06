# WIA Perception Clarity Standard - Phase 1: Data Format Standard

## 1. Overview

이 문서는 WIA Perception Clarity 표준의 데이터 형식을 정의합니다. 피지컬 AI 에이전트(자율주행차·로봇·드론·AMR)의 광학/인식 센서가 "지금 얼마나 잘 보이는가(인식 명료도)"를 측정·상태화·보고하기 위한 데이터 구조를 표준화합니다.

이 표준이 표준화하는 것: **PCI 지표 · 상태 enum · 오염 분류 · 보고 메시지 스키마**.
이 표준이 표준화하지 않는 것: 세척 하드웨어·노즐·세척액·벤더 알고리즘(특허밭 충돌 회피).

### 1.1 Scope

- 인식 명료도 지수(PCI) 데이터 모델
- 센서 클래스 분류
- 명료도 상태(state) enum
- 오염(contaminant) 분류
- 센서 보고 메시지 스키마
- 적합성 레벨 선언

### 1.2 Compliance

이 표준은 다음과 연동·참조합니다:
- VDA5050 (fleet ↔ agent 메시지 모델 — 보고 규약의 본보기)
- SAE J3016 (자율주행 레벨, ODD·fallback)
- ISO 21448 SOTIF (의도 기능 안전, 센서 성능 한계)
- ISO 12233 / MTF (해상력 측정 — contrast 축 근거)

---

## 2. Core Data Types

### 2.1 Sensor Class

```typescript
enum SensorClass {
  RGB_CAMERA = "rgb_camera",        // 가시광 카메라
  IR_THERMAL = "ir_thermal",        // 적외선·열화상
  LIDAR_WINDOW = "lidar_window",    // LiDAR 투과 윈도우
  RADAR_RADOME = "radar_radome",    // 레이더 레이돔
  ULTRASONIC = "ultrasonic"         // 초음파 송수신면
}
```

### 2.2 Clarity State

```typescript
enum ClarityState {
  CLEAR = "clear",            // PCI 90-100, 정상
  DEGRADED = "degraded",      // PCI 60-89, 모니터링·세척 트리거
  OBSTRUCTED = "obstructed",  // PCI 30-59, 안전 동작(감속·우회·세척)
  BLIND = "blind"             // PCI 0-29, 센서 사용 중단·safe-state
}
```

PCI 구간은 기본값이며, 센서 클래스/운용 환경에 따라 임계를 조정할 수 있다(§6.2). 단, 상태의 의미와 순서(CLEAR > DEGRADED > OBSTRUCTED > BLIND)는 불변이다.

### 2.3 Contaminant Type

```typescript
enum ContaminantType {
  RAIN_FILM = "rain_film",          // 빗물막
  MUD_DUST = "mud_dust",            // 진흙·먼지
  SALT_SPRAY = "salt_spray",        // 염무
  INSECT_STRIKE = "insect_strike",  // 벌레 충돌
  FROST_ICE = "frost_ice",          // 서리·결빙
  CONDENSATION = "condensation",    // 응결(김서림)
  SUN_GLARE = "sun_glare",          // 역광·포화
  SCRATCH_ABRASION = "scratch_abrasion" // 스크래치·마모
}
```

오염 분류는 확장 가능하다. 미분류 오염은 `"other"`로 보고하되 `note` 필드에 설명을 권장한다.

### 2.4 Agent Type

```typescript
enum AgentType {
  VEHICLE = "vehicle",
  ROBOT = "robot",
  DRONE = "drone",
  AMR = "amr",
  OTHER = "other"
}
```

### 2.5 Conformance Level

```typescript
enum ConformanceLevel {
  L_A = "L-A",  // PCI 산출 + 상태 enum 노출
  L_B = "L-B",  // A + 표준 보고 메시지 송출
  L_C = "L-C"   // B + dwell-time SLA + 안전 동작 트리거 연동
}
```

---

## 3. PCI (Perception Clarity Index)

### 3.1 정의

PCI는 0–100 정수로, 센서 단위로 산출하는 인식 명료도 지수다. 값이 높을수록 명료하다(100 = 완벽, 0 = 완전 실명). 3개 축의 가중 합산으로 계산한다:

```
PCI = round( 100 - ( w_occ · OCC + w_dist · DIST + w_mtf · MTF ) )
```

| 축 | 기호 | 정의 | 범위 |
|----|------|------|------|
| Occlusion ratio | OCC | 가려진 유효 시야 비율 | 0.0–1.0 |
| Detection-distance degradation | DIST | (기준거리 - 현재 유효 인식거리) / 기준거리 | 0.0–1.0 |
| Contrast / MTF reduction | MTF | 1 - (현재 MTF50 / 기준 MTF50) | 0.0–1.0 |

각 축의 손상값은 0–100 스케일로 환산되어 가중 합산되며, PCI는 [0,100]로 클램프한다.

### 3.2 센서 클래스별 가중치 (기본값)

| Sensor Class | w_occ | w_dist | w_mtf |
|--------------|-------|--------|-------|
| rgb_camera   | 0.40  | 0.25   | 0.35  |
| ir_thermal   | 0.40  | 0.35   | 0.25  |
| lidar_window | 0.45  | 0.45   | 0.10  |
| radar_radome | 0.30  | 0.65   | 0.05  |
| ultrasonic   | 0.50  | 0.50   | 0.00  |

가중치 합은 1.00이어야 한다. 레이더·초음파처럼 광학 해상력이 무의미한 센서는 w_mtf를 0에 가깝게 둔다. 구현은 사용한 가중치 세트를 보고 메시지의 `pciWeights`(선택)로 노출할 수 있다.

### 3.3 PCI → State 매핑 (기본)

| PCI | State |
|-----|-------|
| 90–100 | clear |
| 60–89  | degraded |
| 30–59  | obstructed |
| 0–29   | blind |

---

## 4. Sensor Clarity Report

### 4.1 Schema

```json
{
  "header": {
    "version": "1.0.0",
    "messageId": "uuid",
    "timestamp": "ISO 8601",
    "agentId": "string",
    "agentType": "vehicle | robot | drone | amr | other",
    "conformanceLevel": "L-A | L-B | L-C"
  },
  "sensors": [
    {
      "sensorId": "string",
      "sensorClass": "rgb_camera | ir_thermal | lidar_window | radar_radome | ultrasonic",
      "pci": "number (0-100)",
      "state": "clear | degraded | obstructed | blind",
      "contaminants": [
        {
          "type": "ContaminantType | other",
          "coverage": "number (0.0-1.0)",
          "note": "string (optional)"
        }
      ],
      "axes": {
        "occlusion": "number (0.0-1.0)",
        "distanceDegradation": "number (0.0-1.0)",
        "mtfReduction": "number (0.0-1.0)"
      },
      "pciWeights": {
        "occlusion": "number",
        "distance": "number",
        "mtf": "number"
      },
      "lastCleanedAt": "ISO 8601 | null",
      "dwellSeconds": "number",
      "confidence": "number (0.0-1.0)"
    }
  ]
}
```

- `dwellSeconds`: 현재 비-CLEAR 상태가 연속 잔존한 시간(초). CLEAR면 0.
- `confidence`: PCI 산출 자체의 신뢰도(자기 진단의 불확실성).
- `axes` / `pciWeights`: 선택 필드. 산출 투명성·감사를 위해 권장.

### 4.2 Example — 강우 중 카메라 DEGRADED

```json
{
  "header": {
    "version": "1.0.0",
    "messageId": "7c1f0a2e-3b9d-4a51-8e2c-91a0b6d4f7e2",
    "timestamp": "2025-06-05T08:14:32Z",
    "agentId": "veh-seoul-0421",
    "agentType": "vehicle",
    "conformanceLevel": "L-C"
  },
  "sensors": [
    {
      "sensorId": "front-cam-main",
      "sensorClass": "rgb_camera",
      "pci": 73,
      "state": "degraded",
      "contaminants": [
        { "type": "rain_film", "coverage": 0.35 }
      ],
      "axes": { "occlusion": 0.18, "distanceDegradation": 0.22, "mtfReduction": 0.40 },
      "pciWeights": { "occlusion": 0.40, "distance": 0.25, "mtf": 0.35 },
      "lastCleanedAt": "2025-06-05T08:02:10Z",
      "dwellSeconds": 47,
      "confidence": 0.92
    },
    {
      "sensorId": "roof-lidar-1",
      "sensorClass": "lidar_window",
      "pci": 95,
      "state": "clear",
      "contaminants": [],
      "lastCleanedAt": "2025-06-05T07:55:00Z",
      "dwellSeconds": 0,
      "confidence": 0.98
    }
  ]
}
```

### 4.3 Example — 진흙 가림 LiDAR BLIND

```json
{
  "header": {
    "version": "1.0.0",
    "messageId": "a90b3c12-77de-4f0a-bb21-2c5e8f9a1d44",
    "timestamp": "2025-06-05T11:03:08Z",
    "agentId": "amr-site-12",
    "agentType": "amr",
    "conformanceLevel": "L-C"
  },
  "sensors": [
    {
      "sensorId": "nav-lidar-front",
      "sensorClass": "lidar_window",
      "pci": 21,
      "state": "blind",
      "contaminants": [
        { "type": "mud_dust", "coverage": 0.81 }
      ],
      "axes": { "occlusion": 0.81, "distanceDegradation": 0.88, "mtfReduction": 0.30 },
      "lastCleanedAt": null,
      "dwellSeconds": 312,
      "confidence": 0.86
    }
  ]
}
```

---

## 5. Conformance Declaration

### 5.1 Schema

```json
{
  "declaration_id": "uuid",
  "version": "1.0.0",
  "vendor": "string",
  "product": "string",
  "agentType": "vehicle | robot | drone | amr | other",
  "conformanceLevel": "L-A | L-B | L-C",
  "sensors": [
    {
      "sensorClass": "SensorClass",
      "count": "number",
      "pciSupported": "boolean",
      "reportingSupported": "boolean",
      "dwellSlaSeconds": "number | null",
      "safeActionTriggers": ["slow_down | reroute | trigger_cleaning | safe_state | disable_sensor"]
    }
  ]
}
```

---

## 6. Validation Rules

### 6.1 Required Fields

| Schema | Required Fields |
|--------|-----------------|
| SensorClarityReport | header.version, header.messageId, header.timestamp, header.agentId, header.agentType, sensors[] |
| Sensor | sensorId, sensorClass, pci, state, dwellSeconds, confidence |
| Contaminant | type, coverage |
| ConformanceDeclaration | declaration_id, version, vendor, conformanceLevel, sensors[] |

### 6.2 Value Constraints

| Field | Constraint |
|-------|------------|
| pci | 0–100 정수 |
| coverage / axes.* | 0.0–1.0 |
| confidence | 0.0–1.0 |
| dwellSeconds | ≥ 0 |
| pciWeights.* 합 | = 1.00 (±0.001) |
| state ↔ pci | 기본 매핑(§3.3)과 모순 금지(임계 조정 시 선언 필요) |

### 6.3 Consistency

- `state`는 보고된 `pci`와 적용 임계에 일치해야 한다.
- `contaminants[].coverage` 합이 `axes.occlusion`보다 클 수 있다(중첩 오염). 단, 개별 coverage는 [0,1].
- `dwellSeconds > 0`이면 `state ≠ clear`이어야 한다.

---

## 7. Versioning

- **Major** (1.x.x): 호환 불가 변경
- **Minor** (x.1.x): 하위호환 기능 추가
- **Patch** (x.x.1): 버그 수정

모든 메시지는 `header.version`을 포함해야 한다.

---

## 8. Security Considerations

- 인식 명료도 보고는 에이전트의 안전 상태를 노출하므로, 위변조 시 안전 위해가 발생할 수 있다.
- 전송 구간 암호화(TLS/DTLS) 및 메시지 서명(서명 페이로드) 권장.
- 보고 채널의 인증·권한(관제만 수신 등) 적용.
- 위치·식별자(agentId)는 추적 PII가 될 수 있으므로 최소 수집·접근 통제.

---

## 9. Schema Files

JSON Schema 파일은 다음 위치에 있습니다:

```
/perception-clarity/spec/schemas/
├── sensor-clarity-report.schema.json
├── sensor.schema.json
├── contaminant.schema.json
├── pci.schema.json
└── conformance-declaration.schema.json
```

OpenAPI 정의: `/perception-clarity/spec/openapi.yaml`

---

*WIA Perception Clarity Standard*
*Phase 1: Data Format Standard v1.0.0*
*June 2025 · 弘益人間 · Benefit All Humanity*
