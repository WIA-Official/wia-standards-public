# WIA Perception Clarity Standard - Phase 4: Ecosystem Integration Specification

## 1. Overview

이 문서는 WIA Perception Clarity 표준이 기존 WIA 피지컬 AI 표준 생태계와 어떻게 결합하는지를 정의합니다. Perception Clarity는 특정 폼팩터에 종속되지 않는 **수평(horizontal) 표준**입니다. 즉 자율주행차·로봇·드론·LiDAR·비전 AI·센서 융합 등 어떤 수직(vertical) 표준이든, 자신의 광학/인식 센서가 "지금 얼마나 잘 보이는가"를 동일한 방식으로 측정·상태화·보고하기 위해 이 표준을 소비(consume)합니다.

이 표준이 제공하는 것: **PCI 지표(0–100) · 명료도 상태 enum · 오염 분류 · 표준 보고 메시지(SensorClarityReport) · 적합성 레벨(L-A/L-B/L-C)**. (정의는 Phase 1 참조.)
이 표준이 제공하지 않는 것: 세척 하드웨어·노즐·세척액·벤더 인식 알고리즘. 수직 표준은 **클램리티 신호를 받아 자신의 의사결정에 반영**할 뿐, 동작 방법은 각 표준의 소관입니다.

### 1.1 Integration Architecture

Perception Clarity는 모든 수직 표준 아래에 깔리는 공통 계층입니다. 각 표준은 자신의 센서에서 PCI를 산출하고, 표준 `SensorClarityReport`를 통해 자신의 안전·자율 의사결정 루프에 명료도를 주입합니다.

```
┌───────────────────────────────────────────────────────────────────────────┐
│                        WIA Vertical Standards                               │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌──────────────┐ ┌─────────┐ ┌────────┐ │
│  │ robot  │ │  auto  │ │ drone  │ │ lidar-sensor │ │vision-ai│ │ fusion │ │
│  └───┬────┘ └───┬────┘ └───┬────┘ └──────┬───────┘ └────┬────┘ └───┬────┘ │
│      │          │          │             │              │          │       │
│      └──────────┴──────────┴──────┬──────┴──────────────┴──────────┘       │
│                                   │  consumes                              │
│                       ┌───────────▼────────────┐                          │
│                       │   SensorClarityReport   │   (Phase 1 schema)       │
│                       │   PCI · State · Contam  │                          │
│                       └───────────┬────────────┘                          │
│                                   │ produced by                            │
│  ┌────────────────────────────────▼─────────────────────────────────────┐ │
│  │             WIA Perception Clarity (horizontal layer)                  │ │
│  │   PCI 0-100  │  ClarityState  │  ContaminantType  │  Conformance L-*   │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                   ▲ measured from                          │
│  ┌────────────────────────────────┴─────────────────────────────────────┐ │
│  │  rgb_camera · ir_thermal · lidar_window · radar_radome · ultrasonic    │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Integration Principles

| Principle | Description |
|-----------|-------------|
| Horizontal Reuse | 명료도 측정·보고를 한 번 정의하고 모든 수직 표준이 재사용 |
| Single Source of Clarity | 센서당 PCI는 단일 산출, 수직 표준은 해석만 다르게 적용 |
| Safety First | BLIND/OBSTRUCTED는 각 표준의 안전 동작을 즉시 트리거 |
| Graceful Degradation | 명료도 저하 시 기능 정지가 아니라 단계적 강등(상태 enum 순서 준수) |
| Non-Prescriptive | 이 표준은 "무엇을 보고하는가"만 규정, "어떻게 대응하는가"는 수직 표준 소관 |
| Auditability | `axes`·`pciWeights`·`confidence`로 산출 근거를 감사 가능하게 노출 |

### 1.3 Decision Mapping (공통)

모든 수직 표준은 동일한 상태 → 의사결정 골격을 공유하되, "안전 동작"의 구체 형태만 도메인별로 치환합니다.

| ClarityState | PCI | 공통 의미 | 수직 표준의 치환 동작 |
|--------------|-----|-----------|------------------------|
| clear | 90–100 | 정상 | 정상 운용 |
| degraded | 60–89 | 모니터링·신뢰도 감쇠 | fusion 가중치 하향, 추론 confidence 게이팅 |
| obstructed | 30–59 | 안전 동작 | 감속·우회·세척 트리거·고도 하강 |
| blind | 0–29 | 센서 사용 중단 | ODD 이탈·return-to-home·safe-state·센서 비활성 |

---

## 2. AgentType Mapping

각 수직 표준은 Phase 1의 `AgentType` enum 중 하나(또는 다수)로 매핑됩니다. `SensorClarityReport.header.agentType`는 이 매핑을 따릅니다.

```typescript
// AgentType ← Vertical Standard
const AGENT_TYPE_MAP = {
  "vehicle": ["auto"],                      // 자율주행차
  "robot":   ["robot"],                     // 서비스·산업 로봇
  "drone":   ["drone"],                     // UAV
  "amr":     ["robot", "auto"],             // 자율이동로봇(물류·실내, 로봇/차량 경계)
  "other":   ["lidar-sensor", "vision-ai", "ai-sensor-fusion"]  // 폼팩터 비종속 컴포넌트 표준
};
```

| AgentType | 주 매핑 표준 | 비고 |
|-----------|--------------|------|
| `vehicle` | auto | 도로 주행, SAE J3016 레벨·ODD 연동 |
| `robot` | robot | 협동·서비스·산업 로봇 |
| `drone` | drone | 공중 플랫폼, 비행 안전 연동 |
| `amr` | robot, auto | 자율이동로봇. 실내 물류는 robot, 옥외/도로 공유는 auto 규약 차용 |
| `other` | lidar-sensor, vision-ai, ai-sensor-fusion | 센서·인지 컴포넌트 표준. 호스트 플랫폼의 실제 `agentType`을 그대로 계승 |

`other`로 보고하는 컴포넌트 표준(lidar-sensor·vision-ai·ai-sensor-fusion)은 독립 폼팩터가 아니라 호스트(차량·로봇·드론) 안에 탑재됩니다. 따라서 실제 배치에서는 호스트의 `agentType`을 계승하고, `agentId`로 호스트를 식별하는 것을 권장합니다.

---

## 3. robot 통합 (AgentType: robot)

표준: https://wiastandards.com/robot/

### 3.1 적용 센서 클래스

| SensorClass | 로봇에서의 역할 |
|-------------|-----------------|
| `rgb_camera` | 객체 인식·사람 감지·매니퓰레이션 비전 |
| `ir_thermal` | 저조도·인체 감지·야간 안전 |
| `lidar_window` | 2D/3D 내비게이션·장애물 회피 |
| `ultrasonic` | 근접 감지·미세 장애물·낙하 방지 |

### 3.2 명료도 → 로봇 의사결정 흐름

- `clear`: 정상 작업 속도·전 자율 동작.
- `degraded`: 작업 속도 하향, 사람 협동 구역에서 안전 여유(separation) 증가.
- `obstructed`: 협동 작업 일시 정지·우회 경로 탐색·세척 트리거(있을 경우). 사람 감지 카메라가 `obstructed`이면 협동 모드를 수동 감독으로 강등.
- `blind`: 해당 센서 의존 동작 정지, 안전 정지(safe-state)로 전이하고 관제·운영자에게 통보.

```
SensorClarityReport ─► RobotSafetyMonitor
   state == blind (사람 감지 센서) ─► COLLABORATIVE_MODE 해제 + protective stop
   state == obstructed (내비 lidar) ─► 감속 + 재계획(replan)
   state == degraded ─► 분리거리(separation distance) +30%
```

### 3.3 데이터 흐름 예시 — 협동 로봇 카메라 OBSTRUCTED

```json
{
  "header": {
    "version": "1.0.0",
    "messageId": "b21c-...-cobot",
    "timestamp": "2025-06-05T09:20:00Z",
    "agentId": "cobot-line-3",
    "agentType": "robot",
    "conformanceLevel": "L-C"
  },
  "sensors": [
    {
      "sensorId": "wrist-cam",
      "sensorClass": "rgb_camera",
      "pci": 41,
      "state": "obstructed",
      "contaminants": [{ "type": "mud_dust", "coverage": 0.55 }],
      "dwellSeconds": 18,
      "confidence": 0.90
    }
  ]
}
// robot 표준 반응: 사람 감지 비전 obstructed → 협동 속도 250mm/s → 보호 정지,
// 운영자 확인 요청. 18초 dwell, L-C dwell-SLA 내 안전 동작 트리거됨.
```

---

## 4. auto 통합 (AgentType: vehicle)

표준: https://wiastandards.com/auto/

### 4.1 적용 센서 클래스

| SensorClass | 차량에서의 역할 |
|-------------|-----------------|
| `rgb_camera` | 차선·신호·표지·객체 인식 |
| `ir_thermal` | 야간 보행자·동물 감지 |
| `lidar_window` | 3D 인지·거리 측정 |
| `radar_radome` | 전천후 거리·속도(악천후 강건) |
| `ultrasonic` | 주차·근접 |

### 4.2 명료도 → SAE J3016 fallback / ODD 흐름

Perception Clarity는 ISO 21448 SOTIF의 "센서 성능 한계"를 정량화하여 SAE J3016의 ODD·fallback 결정에 직접 입력됩니다.

| 집계 상태 | SAE 동작 |
|-----------|----------|
| 전 센서 `clear`/`degraded` | ODD 내 정상 자율 |
| 핵심 센서 `obstructed` | 감속·차간 확대·세척 트리거·운전자 인계 준비 알림 |
| 핵심 센서 `blind` (다중·중복 불가) | **ODD 이탈 선언 → MRM(Minimal Risk Maneuver)**: 안전 정차 또는 운전자 인계 |

ODD 이탈 판정은 단일 센서 PCI가 아니라 **중복(redundancy) 후 잔여 명료도**로 합니다. 예: 전방 카메라 BLIND라도 같은 시야를 radar_radome `clear`가 덮으면 ODD 유지 가능. 융합 판단은 §8(ai-sensor-fusion)으로 위임.

### 4.3 데이터 흐름 예시 — 강우 중 카메라 BLIND, 레이더 보전

```json
{
  "header": { "version": "1.0.0", "agentId": "veh-seoul-0421",
    "agentType": "vehicle", "conformanceLevel": "L-C",
    "messageId": "f0...auto", "timestamp": "2025-06-05T08:31:00Z" },
  "sensors": [
    { "sensorId": "front-cam", "sensorClass": "rgb_camera",
      "pci": 22, "state": "blind",
      "contaminants": [{ "type": "rain_film", "coverage": 0.78 }],
      "dwellSeconds": 6, "confidence": 0.88 },
    { "sensorId": "front-radar", "sensorClass": "radar_radome",
      "pci": 93, "state": "clear",
      "contaminants": [], "dwellSeconds": 0, "confidence": 0.97 }
  ]
}
// auto 결정: 전방 카메라 blind이나 radar clear가 종방향 인지 중복 제공.
// → ODD 유지하되 카메라 의존 기능(차선유지) 강등 + 세척 트리거 + 감속.
// 만약 radar도 obstructed였다면 → MRM(안전 정차) 선언.
```

---

## 5. drone 통합 (AgentType: drone)

표준: https://wiastandards.com/drone/

### 5.1 적용 센서 클래스

| SensorClass | 드론에서의 역할 |
|-------------|-----------------|
| `rgb_camera` | 비주얼 항법·VIO·검사·착륙 표적 |
| `ir_thermal` | 야간·열 탐지·검사 |
| `lidar_window` | 고도·지형·장애물 회피 |
| `ultrasonic` | 저고도·착륙 근접 |

### 5.2 명료도 → 비행 안전 흐름

공중 플랫폼은 "정차" 옵션이 없으므로 명료도 저하의 안전 동작이 차량과 다릅니다.

| 상태 | 드론 동작 |
|------|-----------|
| `clear` | 정상 미션 비행 |
| `degraded` | 속도 하향, 자동 미션 정밀도(검사) 재확인, 호버 안정 마진 증가 |
| `obstructed` | 미션 일시 중단·정지 호버(loiter)·고도 조정으로 오염원 회피(예: 안개층 이탈) |
| `blind` | **Return-to-Home(RTH)** 또는 즉시 안전 착륙. 비주얼 항법 센서 BLIND 시 GNSS/관성 폴백 선언 |

```
SensorClarityReport ─► DroneFlightController
   state == obstructed (nav lidar) ─► loiter + 고도 조정
   state == blind (vio camera) ─► RTH 트리거, 폴백 항법(GNSS/IMU) 활성
   dwellSeconds > rth_dwell_sla ─► 강제 RTH
```

### 5.3 데이터 흐름 예시 — 안개로 비주얼 항법 OBSTRUCTED

```json
{
  "header": { "version": "1.0.0", "agentId": "drone-insp-07",
    "agentType": "drone", "conformanceLevel": "L-C",
    "messageId": "c4...drone", "timestamp": "2025-06-05T06:05:00Z" },
  "sensors": [
    { "sensorId": "down-cam-vio", "sensorClass": "rgb_camera",
      "pci": 38, "state": "obstructed",
      "contaminants": [{ "type": "condensation", "coverage": 0.5,
        "note": "안개층 진입 결로" }],
      "dwellSeconds": 22, "confidence": 0.81 }
  ]
}
// drone 결정: VIO obstructed → loiter, 상승하여 안개층 이탈 시도.
// dwell이 RTH SLA 초과하거나 blind 전이 시 → Return-to-Home.
```

---

## 6. lidar-sensor 통합 (AgentType: other → 호스트 계승)

표준: https://wiastandards.com/lidar-sensor/

### 6.1 적용 센서 클래스

| SensorClass | 비고 |
|-------------|------|
| `lidar_window` | 1차 대상. 투과 윈도우 오염이 측정 명료도를 직접 좌우 |
| `ir_thermal` | 일부 LiDAR가 동축 열화상 동반 시 |

### 6.2 명료도 → 컴포넌트 보고 흐름

lidar-sensor 표준은 점군(point cloud) 품질을 호스트에 보고합니다. Perception Clarity는 그 품질을 PCI로 정규화하여, 호스트(차량·로봇·드론)와 융합 스택이 동일 언어로 해석하게 합니다.

- `lidar_window`의 PCI는 가중치 기본값(`w_occ 0.45 · w_dist 0.45 · w_mtf 0.10`)을 사용 — 윈도우 가림(occlusion)과 유효 인식거리 감쇠(distance)가 지배적.
- 컴포넌트 단위 `confidence`는 자기 진단(반사 강도 히스토그램·return-rate)에서 산출.
- 호스트는 `agentType`을 자신의 폼팩터로 설정하고, lidar-sensor는 `sensorId`로 식별.

```
LiDAR 자기진단(return-rate, 윈도우 오염) ─► PCI(lidar_window)
   ─► SensorClarityReport ─► 호스트 fusion / safety monitor
   blind ─► 호스트가 해당 점군 채널 제외(§8 down-weight 0)
```

### 6.3 데이터 흐름 예시 — 진흙 가림 LiDAR BLIND (Phase 1 §4.3 참조)

```json
{
  "header": { "version": "1.0.0", "agentId": "amr-site-12",
    "agentType": "amr", "conformanceLevel": "L-C",
    "messageId": "a9...lidar", "timestamp": "2025-06-05T11:03:08Z" },
  "sensors": [
    { "sensorId": "nav-lidar-front", "sensorClass": "lidar_window",
      "pci": 14, "state": "blind",
      "contaminants": [{ "type": "mud_dust", "coverage": 0.81 }],
      "axes": { "occlusion": 0.81, "distanceDegradation": 0.88, "mtfReduction": 0.30 },
      "lastCleanedAt": null, "dwellSeconds": 312, "confidence": 0.86 }
  ]
}
// lidar-sensor가 호스트 AMR의 agentType(amr)을 계승.
// blind → 호스트는 이 점군 채널을 융합에서 제외하고 safe-state.
```

---

## 7. vision-ai 통합 (AgentType: other → 호스트 계승)

표준: https://wiastandards.com/vision-ai/

### 7.1 적용 센서 클래스

| SensorClass | 비고 |
|-------------|------|
| `rgb_camera` | 1차 대상. 추론 입력 프레임의 명료도 |
| `ir_thermal` | 열화상 기반 추론 |

### 7.2 명료도 → 추론 confidence 게이팅 흐름

vision-ai의 추론 신뢰도는 입력 프레임의 명료도와 상관합니다. PCI를 추론 confidence의 상한 게이트로 사용하여, 흐린 입력에서 과신(over-confidence)을 차단합니다.

```typescript
// PCI 기반 추론 confidence 게이팅
function gateInference(rawConfidence: number, pci: number, state: ClarityState): number {
  if (state === "blind")      return 0;                    // 추론 무효화
  if (state === "obstructed") return Math.min(rawConfidence, 0.50);
  if (state === "degraded")   return rawConfidence * (pci / 100); // 선형 감쇠
  return rawConfidence;                                    // clear: 그대로
}
```

| 상태 | vision-ai 동작 |
|------|----------------|
| `clear` | 원 추론 신뢰도 사용 |
| `degraded` | confidence × (PCI/100)로 감쇠, 다운스트림에 명료도 메타 전달 |
| `obstructed` | confidence 상한 0.50, 안전 임계 의존 기능은 보류 |
| `blind` | 추론 결과 무효화·미보고, 폴백(타 센서·이전 프레임) 요청 |

### 7.3 데이터 흐름 예시 — 역광 포화 카메라 DEGRADED

```json
{
  "header": { "version": "1.0.0", "agentId": "veh-busan-1180",
    "agentType": "vehicle", "conformanceLevel": "L-B",
    "messageId": "d7...vai", "timestamp": "2025-06-05T17:40:00Z" },
  "sensors": [
    { "sensorId": "front-cam", "sensorClass": "rgb_camera",
      "pci": 64, "state": "degraded",
      "contaminants": [{ "type": "sun_glare", "coverage": 0.4 }],
      "axes": { "occlusion": 0.10, "distanceDegradation": 0.20, "mtfReduction": 0.45 },
      "dwellSeconds": 12, "confidence": 0.83 }
  ]
}
// vision-ai: 보행자 검출 raw confidence 0.88 → gate(0.88, 64, degraded) = 0.56.
// 다운스트림 의사결정은 감쇠된 0.56과 sun_glare 메타로 보수적 판단.
```

---

## 8. ai-sensor-fusion 통합 (AgentType: other → 호스트 계승)

표준: https://wiastandards.com/ai-sensor-fusion/

### 8.1 적용 센서 클래스

전 클래스: `rgb_camera`, `ir_thermal`, `lidar_window`, `radar_radome`, `ultrasonic`. 융합 스택은 다종 센서를 동시에 수용하므로 모든 PCI를 입력으로 받습니다.

### 8.2 명료도 → 융합 가중치 흐름

융합의 핵심은 신뢰할 수 없는 센서를 down-weight하는 것입니다. 각 센서의 PCI·`confidence`를 융합 가중치 산출에 직접 투입합니다.

```typescript
// PCI·confidence 기반 융합 가중치
function fusionWeight(sensor: SensorReport, baseWeight: number): number {
  if (sensor.state === "blind") return 0;          // 완전 배제
  const clarity = sensor.pci / 100;                // 0..1
  return baseWeight * clarity * sensor.confidence; // 명료도·자기신뢰도 동시 반영
}

// 정규화: Σ weight = 1 (blind 배제 후 잔여 센서로 재정규화)
```

| 상태 | fusion 동작 |
|------|-------------|
| `clear` | 기준 가중치 유지 |
| `degraded` | 가중치 × (PCI/100) × confidence로 하향 |
| `obstructed` | 강하향 + 융합 출력 불확실성(공분산) 확대 |
| `blind` | 가중치 0(배제), 잔여 센서로 재정규화. 전 센서 blind면 융합 무효 → 호스트 safe-state |

### 8.3 데이터 흐름 예시 — 다종 융합, LiDAR 강등·레이더 보전

```json
{
  "header": { "version": "1.0.0", "agentId": "amr-port-44",
    "agentType": "amr", "conformanceLevel": "L-C",
    "messageId": "e2...fusion", "timestamp": "2025-06-05T13:12:00Z" },
  "sensors": [
    { "sensorId": "cam-front", "sensorClass": "rgb_camera",
      "pci": 81, "state": "degraded", "contaminants": [],
      "dwellSeconds": 30, "confidence": 0.90 },
    { "sensorId": "lidar-front", "sensorClass": "lidar_window",
      "pci": 44, "state": "obstructed",
      "contaminants": [{ "type": "salt_spray", "coverage": 0.5 }],
      "dwellSeconds": 60, "confidence": 0.85 },
    { "sensorId": "radar-front", "sensorClass": "radar_radome",
      "pci": 96, "state": "clear", "contaminants": [],
      "dwellSeconds": 0, "confidence": 0.98 }
  ]
}
// fusion 가중치(base 동일 가정 0.33):
//   cam:   0.33 × 0.81 × 0.90 = 0.241
//   lidar: 0.33 × 0.44 × 0.85 = 0.123  (obstructed → 강하향)
//   radar: 0.33 × 0.96 × 0.98 = 0.310
// 정규화 → cam 0.357 · lidar 0.182 · radar 0.461.
// 융합은 신뢰 가능한 radar에 무게를 싣고 lidar 기여를 줄여 강건성 유지.
```

이 down-weight 결과는 다시 §4(auto)의 ODD 판정, §3(robot)의 분리거리, §5(drone)의 RTH 결정으로 흘러갑니다. ai-sensor-fusion은 수평 표준(Perception Clarity)과 수직 표준(robot/auto/drone) 사이의 **집계자(aggregator)** 역할을 합니다.

---

## 9. End-to-End Data Flow

여러 센서의 명료도 보고가 융합·집계되어 차량의 최종 의사결정으로 이어지는 전체 흐름.

```
┌──────────────┐  PCI(lidar)   ┌──────────────────┐
│ lidar-sensor │──────────────►│                  │
└──────────────┘               │                  │   fusion weight
┌──────────────┐  PCI(rgb)     │ ai-sensor-fusion │──────────────►┌──────────┐
│  vision-ai   │──────────────►│   (aggregator)   │   (배제/하향) │  auto    │
└──────────────┘               │                  │               │ SAE/ODD  │
┌──────────────┐  PCI(radar)   │                  │               │ decision │
│ radar 자기진단 │──────────────►│                  │               └────┬─────┘
└──────────────┘               └──────────────────┘                    │
                                                                        ▼
                              clear/degraded ─► 정상·감쇠 자율          ODD 유지
                              obstructed ─────► 감속·세척·우회          MRM 준비
                              blind(중복불가) ─► ODD 이탈 ───────────► MRM(안전 정차)
```

각 단계는 동일한 `SensorClarityReport`(Phase 1 스키마)를 메시지로 사용하며, VDA5050 스타일의 fleet ↔ agent 채널로 관제에 전파될 수 있습니다.

---

## 10. Cross-Standard Certification

### 10.1 적합성 상호 인정

Perception Clarity에서 적합성 레벨을 선언한 제품(ConformanceDeclaration, Phase 1 §5)은 그 레벨을 **다른 WIA 수직 표준에서 "clarity-aware safety"로 광고**할 수 있습니다.

| Perception Clarity 레벨 | 수직 표준에서의 의미 |
|-------------------------|----------------------|
| **L-A** | PCI 산출 + 상태 enum 노출. 수직 표준은 "명료도 자가 진단 보유" 표기 가능 |
| **L-B** | L-A + 표준 보고 메시지 송출. "표준 명료도 보고(SensorClarityReport) 호환" 표기 가능 |
| **L-C** | L-B + dwell-time SLA + 안전 동작 트리거 연동. "clarity-aware safety integrated"로 광고 가능 — 즉 명료도 저하 시 도메인 안전 동작이 SLA 내 보장됨 |

### 10.2 광고 예시

- auto: "WIA Perception Clarity **L-C** — 센서 명료도 저하 시 SLA 내 ODD/MRM 연동."
- drone: "WIA Perception Clarity **L-C** — 비주얼 항법 BLIND 시 Return-to-Home 자동 트리거."
- robot: "WIA Perception Clarity **L-C** — 사람 감지 명료도 저하 시 협동 모드 안전 강등."
- ai-sensor-fusion: "WIA Perception Clarity **L-B** — 모든 센서 채널의 표준 명료도 보고를 융합 입력으로 수용."

### 10.3 인증 전제

L-C 광고를 위해서는 ① `dwellSlaSeconds`가 선언되고, ② `safeActionTriggers`(slow_down/reroute/trigger_cleaning/safe_state/disable_sensor)가 해당 수직 표준의 안전 동작에 실제 배선되어야 하며, ③ 보고 메시지가 §6(Phase 1)의 검증 규칙을 통과해야 합니다. 상호 인정은 자가 선언이며, 수직 표준의 별도 안전 인증을 대체하지 않습니다.

---

## 11. Reference Implementation

### 11.1 통합 어댑터 패턴

수직 표준은 얇은 어댑터로 `SensorClarityReport`를 자신의 의사결정 루프에 연결합니다.

```typescript
interface ClarityAdapter {
  // 수평 표준이 산출한 보고를 수직 도메인 동작으로 변환
  onReport(report: SensorClarityReport): DomainAction;
}

// auto 어댑터 예
const autoAdapter: ClarityAdapter = {
  onReport(report) {
    const critical = report.sensors.filter(isPrimaryPerception);
    const survivors = critical.filter(s => s.state !== "blind");
    if (survivors.length === 0) return { type: "MRM" };          // ODD 이탈
    if (critical.some(s => s.state === "obstructed"))
      return { type: "slow_down", triggerCleaning: true };
    return { type: "continue" };
  }
};
```

### 11.2 공개 저장소

스키마·예제·어댑터 참조 구현은 WIA Standards 공개 저장소에 있습니다:

```
https://github.com/WIA-Official/wia-standards-public/tree/main/perception-clarity
```

저장소 구성: Phase 1 데이터 형식, JSON Schema(`schemas/`), OpenAPI(`openapi.yaml`), 본 통합 명세, 수직 표준별 어댑터 예제.

---

## 12. 弘益人間

명료하게 보는 것은 안전의 첫 조건입니다. 모든 피지컬 AI 에이전트에게.

### 12.1 Design Philosophy

- **수평적 재사용**: 명료도 측정·보고를 한 번 표준화해 모든 폼팩터가 공유.
- **정직한 자기 인식**: 에이전트가 "지금 잘 보이지 않는다"를 정량으로 말할 수 있게.
- **안전을 향한 강등**: 보이지 않으면 멈추거나 돌아가거나 양보 — 과신하지 않게.
- **개방과 상호운용**: 벤더·도메인을 넘어 동일 언어로 명료도를 주고받게.

### 12.2 Commitment

WIA Perception Clarity 표준은 어떤 기계도 자신이 무엇을 못 보는지 정직하게 말할 수 있어야 한다는 믿음 위에 서 있습니다. 이 수평 표준이 모든 수직 표준과 결합하여, 사람과 기계가 공유하는 공간이 더 안전해지기를 지향합니다.

---

*WIA Perception Clarity Standard · Phase 4: Integration v1.0.0 · 弘益人間*
