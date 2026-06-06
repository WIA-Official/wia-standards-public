# WIA Perception Clarity Standard
## Phase 3: Reporting & Test Protocol Specification

**Version**: 1.0.0
**License**: MIT
**Status**: Draft
**Date**: 2025-06-05
**Philosophy**: 弘益人間 · Benefit All Humanity

---

## 1. Overview

이 문서는 WIA Perception Clarity 표준의 **프로토콜 계층**을 정의한다. Phase 1(Data Format)이 데이터 모델(PCI 지표·상태 enum·오염 분류·보고 스키마)을 정의했다면, Phase 3은 그 데이터를 **어떻게 주고받고(보고 규약), 어떻게 검증하며(오염 테스트 프로토콜), 어떻게 안전 동작으로 연결하고(SLA·트리거), 어떤 적합성 레벨로 판정하는가**를 정의한다.

이 표준이 표준화하는 것: **메시지 교환 규약 · 전송 바인딩 · 재현 가능한 오염 테스트 절차 · dwell-time SLA · 안전 동작 트리거 매핑 · 적합성 판정 기준**.
이 표준이 표준화하지 않는 것: 세척 하드웨어·노즐·세척액·발수 코팅·벤더 인식 알고리즘(특허밭 충돌 회피).

### 1.1 VDA5050 본보기 — "기계가 아니라 메시지를 표준화한다"

VDA5050은 AGV(무인운반차)와 관제(master control) 사이의 통신 인터페이스만 표준화하고, AGV를 **어떻게 만드는지**(기구·모터·내비게이션 알고리즘)는 건드리지 않음으로써 사실상 산업 표준이 되었다. 어느 제조사의 AGV든 동일한 메시지 어휘로 관제와 대화할 수 있으면 fleet에 섞일 수 있다.

WIA Perception Clarity는 이 철학을 그대로 따른다.

| VDA5050 | WIA Perception Clarity |
|---------|------------------------|
| AGV ↔ master control | physical AI agent ↔ fleet/관제 |
| `order` / `state` / `connection` / `instantActions` 토픽 | `clarity` / `state` / `connection` / `action` 토픽 |
| 주행 명령·상태를 표준화, 모터는 자유 | 인식 명료도 보고를 표준화, 세척 기구는 자유 |
| 5초 주기 + 이벤트 발생 시 즉시 송출 | 주기 보고 + 상태 전이 시 이벤트 송출 (§3.4) |

핵심 원칙: **에이전트가 "지금 얼마나 잘 보이는가"를 공용 메시지로 송출하면, 어느 제조사의 센서든 관제·다른 에이전트가 동일하게 해석한다.** 닦는 방법은 벤더 자유, 보고하는 언어는 표준.

### 1.2 Design Principles

1. **Transparency**: PCI 산출의 축·가중치를 감사 가능하게 노출(선택 필드).
2. **Safety-first**: 명료도 저하는 즉시 안전 동작 트리거로 연결(L-C).
3. **Reproducibility**: 오염 테스트는 인가된 랩에서 동일 결과로 재현 가능해야 한다.
4. **Interoperability**: 전송 계층(MQTT/REST/WebSocket/DDS) 무관하게 동일 페이로드.
5. **Reliability**: 보고 채널 단절·열화 시 안전측 가정(fail-safe).

### 1.3 Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                     Fleet / 관제 Infrastructure                    │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐    │
│  │  Clarity Broker │  │  Audit / Log    │  │  Safety Manager │    │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘    │
│           └────────────────────┼────────────────────┘             │
│                          ┌──────▼──────┐                          │
│                          │  Transport  │  MQTT / REST / WS / DDS  │
│                          └──────┬──────┘                          │
└─────────────────────────────────┼──────────────────────────────────┘
            ┌────────────────────┼────────────────────┐
            │                    │                    │
     ┌──────▼──────┐     ┌──────▼──────┐     ┌──────▼──────┐
     │   vehicle   │     │ robot / amr │     │    drone    │
     │  (sensors)  │     │  (sensors)  │     │  (sensors)  │
     └─────────────┘     └─────────────┘     └─────────────┘
       각 에이전트가 sensor 단위 PCI·state·dwell을 송출
```

---

## 2. Message Envelope

모든 전송 바인딩은 공통 봉투(envelope)로 페이로드를 감싼다. 봉투는 라우팅·상관·우선순위만 담고, 본문은 Phase 1의 `SensorClarityReport`(§4) 또는 본 문서의 액션/응답 메시지다.

```json
{
  "wia_perception": {
    "version": "1.0.0",
    "messageId": "uuid",
    "timestamp": "ISO 8601",
    "source": "agent | fleet | lab",
    "destination": "agent | fleet | broadcast",
    "agentId": "string",
    "messageType": "clarity_report | state_change | safe_action | action_ack | connection",
    "correlationId": "uuid (optional)",
    "priority": "low | normal | high | critical",
    "payload": { }
  }
}
```

| messageType | 방향 | 본문(payload) |
|-------------|------|---------------|
| `clarity_report` | agent → fleet | `SensorClarityReport` (Phase 1 §4) |
| `state_change` | agent → fleet | 전이 이벤트(§3.4.2) |
| `safe_action` | fleet → agent / agent 내부 | 안전 동작 지시(§6) |
| `action_ack` | agent → fleet | 동작 수락/완료 응답 |
| `connection` | agent ↔ fleet | online/offline/heartbeat |

- `priority`: `blind`/`obstructed` 상태 보고와 `safe_action`은 `critical` 또는 `high`로 송출한다.
- `connection`은 VDA5050의 connection 토픽에 대응하며, last-will(비정상 단절)로 `offline`을 게시한다. 관제는 `offline` 에이전트의 마지막 보고를 신뢰하지 않고 안전측으로 가정한다.

---

## 3. Transport Bindings

동일한 봉투·페이로드를 4개 전송 계층에 바인딩한다. 구현은 최소 1개를 지원하면 되고, L-B 이상은 적어도 하나의 **푸시형**(MQTT/WS/DDS) 채널을 권장한다.

### 3.1 MQTT (권장 1순위)

VDA5050과 동일하게 MQTT를 1순위로 둔다. 토픽 구조는 VDA5050 패턴(`interfaceName/majorVersion/manufacturer/serialNumber/topic`)을 따른다.

```
wiapc/v1/{manufacturer}/{agentId}/clarity      # 주기 + 전이 시 SensorClarityReport
wiapc/v1/{manufacturer}/{agentId}/state        # 상태 전이 이벤트
wiapc/v1/{manufacturer}/{agentId}/connection   # online/offline/last-will
wiapc/v1/{manufacturer}/{agentId}/action       # fleet → agent 안전 동작 지시
wiapc/v1/{manufacturer}/{agentId}/action_ack   # agent → fleet 응답
```

| 항목 | 권장값 |
|------|--------|
| QoS (clarity 주기) | 0 또는 1 |
| QoS (state_change, safe_action, connection) | 1 (at-least-once) |
| Retained | `connection`(현재 접속 상태), 최신 `state` |
| Last-Will | `connection` 토픽에 `{"state":"offline"}` |
| Keepalive | 15초 |
| TLS | 필수(8883). DTLS는 비-TCP 환경 |

`clarity` 토픽에 최신 보고를 retained로 두면, 후발 구독자(관제 재기동·다른 에이전트)가 즉시 현재 명료도를 획득한다.

### 3.2 REST (HTTPS)

비실시간 조회·선언 등록·랩 결과 업로드에 사용한다. 푸시가 어려운 환경의 폴링 fallback.

**Base URL**: `https://api.wia-perception.org/v1`

```http
POST   /agents/{agentId}/clarity          # 보고 업로드 (단발/배치)
GET    /agents/{agentId}/clarity/latest    # 최신 보고 조회 (폴링)
GET    /agents/{agentId}/sensors/{sensorId}/history?from=&to=
POST   /agents/{agentId}/actions           # 안전 동작 지시
POST   /conformance/declarations           # 적합성 선언 등록 (Phase 1 §5)
POST   /lab/contamination-tests            # 오염 테스트 결과 업로드 (§5)
```

#### Headers
```http
Authorization: Bearer <jwt_token>
Content-Type: application/json
X-WIA-Version: 1.0.0
X-Request-ID: <uuid>
```

#### Example — 최신 보고 조회
```http
GET /agents/veh-seoul-0421/clarity/latest
```
**Response**: `200 OK` → 본문은 Phase 1 §4 `SensorClarityReport`.

폴링 권장 주기는 §3.4의 cadence를 넘지 않는다(예: 1초 미만 폴링 금지, 서버 부하·과금 회피).

### 3.3 WebSocket (WSS)

관제 대시보드·다른 에이전트가 실시간 명료도 스트림을 구독한다.

**Endpoint**: `wss://ws.wia-perception.org/v1/stream?token=<jwt>&agentId=<id>`

```json
{
  "type": "clarity_report | state_change | safe_action",
  "id": "msg-uuid",
  "timestamp": "ISO 8601",
  "payload": { }
}
```

- 구독: `{"type":"subscribe","payload":{"agents":["veh-seoul-0421"],"events":["state_change"]}}`
- Heartbeat: 클라이언트 `ping` 30초, 3회 누락 시 단절 → 관제는 해당 에이전트를 `offline`으로 간주.

### 3.4 DDS (RTPS)

차량 내부·로봇 내부의 실시간 센서 버스(ROS 2 등)에 적합. 저지연 publish/subscribe, QoS profile로 신뢰성·기한 보장.

| Topic | Type | DDS QoS |
|-------|------|---------|
| `WIAPC/ClarityReport` | SensorClarityReport | RELIABLE, KEEP_LAST(1) |
| `WIAPC/StateChange` | StateChangeEvent | RELIABLE, KEEP_ALL |
| `WIAPC/SafeAction` | SafeActionCommand | RELIABLE, deadline 100ms |

DDS는 에이전트 **내부** 안전 매니저↔센서 모듈 간 트리거 전달에 권장한다(외부 fleet 보고는 MQTT 게이트웨이로 브리지).

### 3.5 Reporting Cadence — 주기 vs 이벤트

VDA5050처럼 **주기 보고 + 이벤트 보고**를 병행한다.

#### 3.4.1 Periodic (heartbeat) reporting

| AgentType | 권장 주기 | 비고 |
|-----------|----------|------|
| vehicle | 1 s (clear), 0.5 s (degraded↓) | 고속·동적 환경 |
| drone | 1 s | 비행 안전 |
| amr | 2 s | 산업 실내 |
| robot | 2–5 s | 서비스·실내 |

- 모든 에이전트는 최소 5초에 1회는 `clarity_report`를 송출한다(staleness 방지).
- 상태가 `clear`가 아니면 주기를 단축(최소 절반)한다.

#### 3.4.2 Event-driven reporting (즉시 송출)

다음 사건은 주기와 무관하게 **즉시** `state_change`를 송출한다:
- 어느 센서든 `state`가 전이(예: clear→degraded, obstructed→blind).
- `dwellSeconds`가 SLA 임계(§5의 windows)를 통과.
- 안전 동작이 트리거되거나 해제됨.
- `lastCleanedAt` 갱신(세척 후 회복 보고).

```json
{
  "type": "state_change",
  "payload": {
    "sensorId": "front-cam-main",
    "sensorClass": "rgb_camera",
    "previousState": "degraded",
    "newState": "obstructed",
    "pci": 48,
    "dwellSeconds": 33,
    "trigger": "pci_threshold_crossed"
  }
}
```

---

## 4. Reporting Protocol Flow

전형적인 에이전트 ↔ fleet 교환:

```
agent                                   fleet/관제
  │  connection: online (retained)        │
  ├───────────────────────────────────────▶
  │                                        │
  │  clarity_report (주기 1s, all CLEAR)   │
  ├───────────────────────────────────────▶  (모니터링)
  │                                        │
  │  [rain_film 누적 → PCI 73 degraded]    │
  │  state_change: clear→degraded (즉시)   │
  ├───────────────────────────────────────▶
  │                                        │  Safety Manager 평가
  │                                        │
  │  clarity_report (주기 0.5s로 단축)     │
  ├───────────────────────────────────────▶
  │                                        │
  │  [dwell 30s 초과, PCI 48 obstructed]   │
  │  state_change + dwell SLA 통과         │
  ├───────────────────────────────────────▶
  │  safe_action: slow_down + trigger_cleaning
  │◀───────────────────────────────────────┤  (또는 agent 내부 자율 트리거)
  │  action_ack: accepted/applied          │
  ├───────────────────────────────────────▶
  │                                        │
  │  [세척 후 PCI 94 clear, lastCleanedAt] │
  │  state_change: obstructed→clear        │
  ├───────────────────────────────────────▶  (안전 동작 해제)
```

L-C 구현에서 안전 동작은 **에이전트 자율**로 트리거되어도 되고(보고는 사후), fleet 지시로 트리거되어도 된다. 어느 경우든 `action_ack`로 결과를 보고한다.

---

## 5. Contamination Test Protocol

PCI 산출과 상태 판정이 **재현 가능**함을 검증하는, 인가된 랩 지향 절차다. 목적은 "센서를 일부러 오염시켜, 그 오염 정도에서 구현이 산출한 PCI·state가 표준 기대와 일치하는가"를 확인하는 것이다. **실차/실로봇이 아닌 통제된 랩 환경에서, 비파괴·가역 오염을 인가된 절차로만 적용한다.**

> 안전·법규 주의: 모든 오염 적용은 격리된 랩 리그에서, 보호 절차에 따라 수행한다. 공도·운용 중 차량/로봇/드론에 오염을 인위 적용하는 행위는 금지한다. 레이저(LiDAR)·RF(레이더) 노출은 해당 안전 등급 절차를 준수한다.

### 5.1 Test Rig 구성

| 구성요소 | 설명 |
|----------|------|
| 고정 지그 | 센서를 기준 자세로 고정. 진동 격리 |
| 타깃 보드 | 가변 거리 레일에 장착(거리 열화 시험용) |
| 조명 | 균일 확산광 + 제어 가능한 글레어 광원 |
| 기준 측정기 | 산출 PCI와 대조할 외부 grand-truth(거리계·MTF 차트 분석기) |
| 환경 챔버(선택) | 결빙·응결·온습도 재현 |

각 시험은 **(1) 기준(clean) 측정 → (2) 오염 적용 → (3) 정량화 → (4) 구현 PCI 캡처 → (5) 대조**의 5단계로 진행한다.

### 5.2 Sensor-class별 오염 적용 매트릭스

오염 적용은 **가역적**이어야 한다(시험 후 원복). ContaminantType enum과 정합한다.

| SensorClass | 적용 오염(예) | 1차 측정 축 |
|-------------|--------------|-------------|
| rgb_camera | rain_film, mud_dust, sun_glare, scratch_abrasion | occlusion + mtf |
| ir_thermal | condensation, frost_ice, rain_film | occlusion + distance |
| lidar_window | mud_dust, rain_film, salt_spray, scratch_abrasion | occlusion + distance |
| radar_radome | mud_dust, frost_ice, salt_spray | distance (RF 감쇠) |
| ultrasonic | rain_film, frost_ice | occlusion + distance |

> 가역 오염 매질 예: 광학용 수성 무라(water-based film), 제거 가능한 분진 페이스트, 탈착식 광학 필터(글레어/감쇠 모사), 결빙은 환경 챔버. 영구 손상(실제 스크래치)은 전용 폐기 검체에만 적용하고 운용 센서에는 금지한다.

### 5.3 Test A — Occlusion 타깃 패턴 시험

목적: occlusion 축(OCC) 산출의 정확성 검증.

1. 센서 시야 정면에 **격자 occlusion 마스크**(예: 10×10 셀, 각 셀 1% 시야)를 단계적으로 적용.
2. 0% → 10% → 25% → 50% → 81% 순으로 가림 비율을 키운다.
3. 각 단계에서 구현이 보고한 `axes.occlusion`과 적용한 실제 가림 비율을 대조.
4. 합격 기준: 각 단계 `|보고 OCC − 실제 OCC| ≤ 0.05`.
5. occlusion이 PCI·state로 올바르게 환산되는지 확인(예: lidar_window 81% 가림 → PCI<30, state=blind).

### 5.4 Test B — 거리 열화 시험

목적: distance-degradation 축(DIST) 산출 검증.

1. clean 상태에서 표준 타깃을 인식 가능한 **기준 인식거리**(baseline)를 측정·고정.
2. 오염 매질(예: rain_film 균일막)을 단계적 두께/밀도로 적용.
3. 각 단계에서 타깃을 레일로 멀리/가까이 이동하며 **유효 인식거리**를 측정.
4. `DIST = (기준거리 − 현재거리) / 기준거리`를 산정, 구현 보고값과 대조.
5. 합격 기준: `|보고 DIST − 측정 DIST| ≤ 0.05`. radar_radome·ultrasonic은 DIST가 주 축이므로 본 시험이 핵심.

### 5.5 Test C — Contrast / MTF 시험 (ISO 12233)

목적: contrast/MTF 축(MTF) 산출 검증. 광학 해상력이 의미 있는 센서(rgb_camera, ir_thermal)에 필수, lidar_window는 선택.

1. **ISO 12233** slanted-edge 차트(또는 등가 SFR 차트)를 기준 거리·조명에서 촬영.
2. clean 상태의 **기준 MTF50**(baseline MTF50)을 산정.
3. 오염 매질(rain_film, mud_dust, scratch_abrasion 모사 필터)을 단계 적용.
4. 각 단계 현재 MTF50을 산정, `MTF = 1 − (현재 MTF50 / 기준 MTF50)`로 환산.
5. 구현 보고 `axes.mtfReduction`과 대조. 합격 기준: `|보고 − 측정| ≤ 0.05`.
6. sun_glare는 균일 차트에 제어 글레어 광원을 더해 포화·블루밍에 의한 contrast 손실로 측정한다.

### 5.6 종합 PCI 검증

세 축을 동시 적용한 **혼합 오염** 시나리오에서, 구현이 산출한 PCI가 Phase 1 §3.1 공식과 §3.2 가중치로 재계산한 기대 PCI와 일치하는지 확인한다.

- 합격 기준: `|보고 PCI − 기대 PCI| ≤ 3` (정수 PCI 기준).
- state 매핑(§Phase 1 3.3)과 보고 `state`가 일치.
- 임계를 조정한 구현은 조정값을 선언하고, 그 선언값 기준으로 대조한다.

### 5.7 결과 리포트 포맷

```json
{
  "testReportId": "uuid",
  "version": "1.0.0",
  "lab": "string",
  "agentType": "vehicle | robot | drone | amr | other",
  "sensorClass": "rgb_camera | ir_thermal | lidar_window | radar_radome | ultrasonic",
  "tests": [
    {
      "test": "A_occlusion | B_distance | C_mtf | composite",
      "steps": [
        {
          "contaminant": "rain_film | mud_dust | salt_spray | insect_strike | frost_ice | condensation | sun_glare | scratch_abrasion | other",
          "appliedAxis": { "occlusion": 0.40, "distanceDegradation": 0.0, "mtfReduction": 0.0 },
          "reportedAxis": { "occlusion": 0.42, "distanceDegradation": 0.0, "mtfReduction": 0.0 },
          "expectedPci": 84,
          "reportedPci": 83,
          "reportedState": "degraded",
          "pass": true
        }
      ]
    }
  ],
  "overallPass": true
}
```

랩 결과는 REST `POST /lab/contamination-tests`로 업로드하여 적합성 선언(Phase 1 §5)의 근거로 첨부한다.

---

## 6. Dwell-Time SLA & Safe-Action Triggers

### 6.1 Max obstruction dwell time

오염은 **시간이 지날수록 회복이 어려워진다.** 빗물막은 초기엔 닦이지만, 진흙·염무·서리는 약 30초를 넘기면 굳거나 누적되어 가역 세척이 실패하기 쉽다. 따라서 표준은 **비-CLEAR 상태의 연속 잔존 시간(`dwellSeconds`)** 에 대한 SLA를 정의하고, 임계 통과 시 **행동 의무**를 부과한다.

핵심 정의 — **max obstruction dwell time**: 한 센서가 비-CLEAR 상태로 연속 잔존할 수 있는 최대 허용 시간. 이를 넘기면 회복 곤란 구간으로 보고 안전 동작이 의무화된다. 기본 권장 임계는 **30초**다(센서 클래스·ODD에 따라 선언 후 조정 가능).

### 6.2 State별 SLA window

| State | SLA window (dwell) | 의무 |
|-------|--------------------|------|
| clear | — | 정상. dwell=0 |
| degraded | 0–30 s 모니터, >30 s 경고 | 모니터링 강화·세척 권장. 주기 보고 단축 |
| obstructed | 즉시 평가, dwell >10 s 행동 의무 | 안전 동작 트리거(감속/우회/세척) |
| blind | 즉시(dwell 무관) | 센서 사용 중단 + safe-state 전이 |

- `degraded`가 **30초 초과 지속**되면 세척을 트리거하거나(가능 시) `obstructed`에 준하는 평가로 격상한다.
- `obstructed`는 **10초 이내**에 적어도 한 가지 안전 동작을 적용해야 한다.
- `blind`는 **즉시** 해당 센서를 신뢰 대상에서 제외하고 safe-state로 전이한다.
- SLA 임계를 통과하는 순간 §3.4.2의 `state_change`(또는 별도 SLA 경보)를 즉시 송출한다.

### 6.3 Safe-Action Triggers

Phase 1의 `safeActionTriggers` enum과 정합한다: `{slow_down, reroute, trigger_cleaning, safe_state, disable_sensor}`.

| 동작 | 의미 | 전형적 발동 상태 |
|------|------|------------------|
| `slow_down` | 속도·동작 속도 저감으로 인식 여유 확보 | degraded(악화)·obstructed |
| `reroute` | 오염 유발 환경 회피·대체 경로 | obstructed (잔존) |
| `trigger_cleaning` | 세척 시퀀스 요청(기구는 벤더 자유) | degraded>30s·obstructed |
| `safe_state` | 안전 정지·최소위험상태(MRC)·착륙 | blind·다중 센서 동시 저하 |
| `disable_sensor` | 해당 센서를 융합에서 제외 | blind |

> `trigger_cleaning`은 **세척을 요청하는 표준 신호**일 뿐, 노즐·세척액·시퀀스는 표준화하지 않는다(특허밭 회피). 에이전트는 자신의 세척 기구를 자유롭게 구현한다.

### 6.4 State → Action 매핑 (규범)

```
degraded (PCI 60–89)
   ├─ dwell ≤ 30s   → monitor (주기 단축, 기록)
   └─ dwell > 30s   → trigger_cleaning (가능 시), 미회복 시 슬로다운 평가

obstructed (PCI 30–59)
   ├─ 즉시          → slow_down
   ├─ dwell > 10s   → trigger_cleaning + (필요 시) reroute
   └─ 미회복 지속    → safe_state 평가

blind (PCI 0–29)
   └─ 즉시          → disable_sensor + safe_state
                      (남은 센서로 안전 운용 불가 시 즉시 MRC/정지/착륙)
```

다중 센서 동시 저하 시(예: 전방 카메라 blind + LiDAR obstructed) 안전 매니저는 **가장 보수적인 동작**을 채택한다(safe_state 우선).

### 6.5 SafeAction 메시지

```json
{
  "type": "safe_action",
  "payload": {
    "agentId": "amr-site-12",
    "sensorId": "nav-lidar-front",
    "state": "blind",
    "dwellSeconds": 312,
    "triggers": ["disable_sensor", "safe_state"],
    "reason": "mud_dust occlusion 0.81, dwell exceeded SLA",
    "issuedBy": "agent | fleet"
  }
}
```

응답 `action_ack`:
```json
{
  "type": "action_ack",
  "payload": {
    "correlationId": "<safe_action messageId>",
    "accepted": true,
    "appliedTriggers": ["disable_sensor", "safe_state"],
    "result": "vehicle_stopped | cleaning_started | rerouted | slowed",
    "timestamp": "ISO 8601"
  }
}
```

---

## 7. Clarity State Machine

센서 단위 명료도 상태 전이. 전이는 PCI 임계(§Phase 1 3.3)와 dwell SLA(§6.2)가 함께 구동한다.

```
                  PCI ≥ 90
        ┌──────────────────────────────────────┐
        │                                       │
        ▼                                        │
   ┌─────────┐  PCI 60–89   ┌──────────┐  PCI 30–59  ┌────────────┐  PCI ≤ 29  ┌────────┐
   │  CLEAR  │─────────────▶│ DEGRADED │────────────▶│ OBSTRUCTED │───────────▶│ BLIND  │
   │ dwell=0 │              │ monitor  │             │ safe-action│            │disable │
   │         │◀─────────────│          │◀────────────│            │◀───────────│ +safe  │
   └─────────┘   PCI ≥ 90   └──────────┘  PCI 60–89  └────────────┘  PCI 30–59 │ -state │
        ▲             (세척/회복)   │                       │                    └────────┘
        │                          │ dwell > 30s           │ dwell > 10s            │
        │                          ▼                       ▼                        │
        │                  [trigger_cleaning]      [slow_down + cleaning]           │
        │                          │                  [reroute if 미회복]           │
        └──────────────────────────┴───────────────────────┴────────── 회복 시 ◀────┘
                          (PCI 상승 → 상위 상태로 복귀, lastCleanedAt 갱신)

  전이 규칙:
   - 하강(악화): PCI가 하위 구간에 진입하면 즉시 전이 + state_change 송출.
   - 상승(회복): PCI가 상위 구간으로 회복되면 전이, dwellSeconds 재설정(CLEAR=0).
   - dwell SLA 통과: 같은 상태 내에서도 §6.2 window 초과 시 행동 의무·SLA 경보.
   - BLIND 진입: 무조건 disable_sensor + safe_state 평가(dwell 무관, 즉시).
   - 이력: 모든 전이는 audit 로그·state 토픽(retained)에 기록.
```

- 상태의 의미·순서(CLEAR > DEGRADED > OBSTRUCTED > BLIND)는 불변이다(Phase 1 §2.2).
- 히스테리시스 권장: 경계(예: PCI 89↔90) 진동(chattering)을 막기 위해 회복 시 +2 PCI 마진을 두어도 된다(선언 권장).

---

## 8. Conformance Levels

적합성은 누적 단계다. 상위 레벨은 하위 레벨의 모든 요건을 포함한다(L-A ⊂ L-B ⊂ L-C).

### 8.1 L-A — Measure (측정)

**요건**
- 센서 단위 PCI를 0–100 정수로 산출(Phase 1 §3 공식·가중치 정합).
- ClarityState enum(clear/degraded/obstructed/blind)을 노출.
- PCI ↔ state 매핑이 §Phase 1 3.3(또는 선언된 임계)과 모순 없음.

**판정 방법**
- §5 오염 테스트 A/B/C 중 해당 센서 클래스 적용 시험을 통과(각 축 오차 ≤ 0.05).
- 종합 PCI 검증(§5.6) 통과(`|보고−기대| ≤ 3`, state 일치).
- 보고는 내부 노출만으로 충분(외부 메시지 송출은 L-B에서 요구).

### 8.2 L-B — Report (보고)

**요건 (L-A +)**
- Phase 1 §4 `SensorClarityReport`를 표준 봉투(§2)로 송출.
- 최소 1개 전송 바인딩(§3) 지원, 적어도 하나의 푸시형 채널 권장.
- 주기 보고(§3.4.1, 최소 5초 1회) + 상태 전이 이벤트(§3.4.2) 모두 구현.
- `connection`(online/offline/last-will) 게시.

**판정 방법**
- 적합성 테스트 하니스가 에이전트를 구독하여, 주기 보고 수신·전이 이벤트 적시 수신·스키마 유효성(Phase 1 §6 validation)을 확인.
- 오염 테스트 중 상태가 전이될 때 `state_change`가 즉시(주기 무관) 도착하는지 확인.
- 봉투·페이로드가 §2·Phase 1 §4 스키마에 적합.

### 8.3 L-C — Act (행동)

**요건 (L-B +)**
- dwell-time SLA(§6.2)를 구현: state별 window·max obstruction dwell time 정의·선언.
- safeActionTriggers(§6.3) 연동: state → action 매핑(§6.4)을 실제 발동.
- SLA 임계 통과 시 행동 의무 이행 + `safe_action`/`action_ack` 보고.

**판정 방법**
- 오염 테스트로 센서를 obstructed/blind로 유도 후, **SLA window 내**에 규정 안전 동작이 발동되는지 측정:
  - obstructed → 10초 이내 `slow_down`(+ 잔존 시 cleaning/reroute).
  - blind → 즉시 `disable_sensor` + `safe_state`.
  - degraded >30s → `trigger_cleaning`(가능 시).
- 다중 센서 동시 저하에서 가장 보수적 동작이 채택되는지 확인.
- 모든 발동이 `action_ack`로 적시 보고되는지 확인.

### 8.4 판정 요약

| 항목 | L-A | L-B | L-C |
|------|-----|-----|-----|
| PCI 산출(±3) | ✔ | ✔ | ✔ |
| state enum 노출 | ✔ | ✔ | ✔ |
| 오염 테스트 A/B/C 통과 | ✔ | ✔ | ✔ |
| 표준 보고 메시지 송출 | | ✔ | ✔ |
| 주기 + 이벤트 보고 | | ✔ | ✔ |
| connection/last-will | | ✔ | ✔ |
| dwell-time SLA | | | ✔ |
| safe-action 트리거 발동 | | | ✔ |
| safe_action/action_ack 보고 | | | ✔ |

적합성 선언은 Phase 1 §5 `ConformanceDeclaration`으로 등록하며, `dwellSlaSeconds`·`safeActionTriggers` 필드는 L-C에서 필수다.

---

## 9. Security Considerations

- 인식 명료도·안전 상태 노출은 위변조 시 직접 안전 위해로 이어진다. 전송 구간 암호화(TLS 1.3 / DTLS) 필수.
- 메시지 서명(HMAC-SHA256 또는 페이로드 서명) 권장. `safe_action`은 서명 필수.
- 보고 채널 인증·권한 분리(관제만 수신, 에이전트 간 broadcast는 읽기 전용).
- `agentId`·위치는 추적 PII가 될 수 있으므로 최소 수집·접근 통제·보관 한도(예: 90일).
- 보고 단절·`offline`·staleness(마지막 보고 5초 초과)는 **안전측으로 가정**한다(명료하다고 가정 금지).

---

## 10. Error Handling

| 상황 | 처리 |
|------|------|
| 보고 staleness(>5s) | 관제는 해당 센서를 `unknown`→보수적(저하 가정) 처리 |
| 스키마 검증 실패 | 메시지 폐기 + `422` 또는 NACK, audit 기록 |
| confidence 낮음(<0.5) | PCI 신뢰 저하 표시, 보수적 상태 채택 권장 |
| safe_action 미수락 | fleet은 재전송(QoS1) 후 미응답 시 상위 안전 매니저로 에스컬레이션 |
| 전송 단절 | last-will `offline` → 안전측 가정, 재접속 시 retained로 현재 상태 복원 |

```json
{
  "error": {
    "code": "SCHEMA_VALIDATION_ERROR | STALE_REPORT | ACTION_REJECTED | UNAUTHORIZED",
    "message": "string",
    "messageId": "uuid",
    "timestamp": "ISO 8601"
  }
}
```

---

## 11. Interoperability

WIA Perception Clarity는 횡단 표준으로, 기존 WIA 표준과 연동한다: `auto`(자율주행 접근성)·`robot`·`drone`·`lidar-sensor`·`vision-ai`·`ai-sensor-fusion`. agentType enum(vehicle/robot/drone/amr/other)으로 도메인을 식별하며, 동일 봉투·페이로드로 모든 도메인이 명료도를 동일하게 해석한다. fleet은 VDA5050 운용 메시지와 본 표준의 명료도 메시지를 같은 MQTT 브로커에서 병행 운용할 수 있다(토픽 네임스페이스 분리).

---

## Appendix A: 빠른 적합성 체크리스트

- [ ] PCI 0–100 정수 산출, 가중치 합 = 1.00 (L-A)
- [ ] clear/degraded/obstructed/blind 매핑 정합 (L-A)
- [ ] 오염 테스트 A/B/C + 종합 PCI 검증 통과 (L-A)
- [ ] SensorClarityReport를 표준 봉투로 송출 (L-B)
- [ ] 주기(≤5s) + 전이 이벤트 + connection/last-will (L-B)
- [ ] MQTT/REST/WS/DDS 중 1개 이상 바인딩 (L-B)
- [ ] dwell-time SLA window 정의·선언 (L-C)
- [ ] state→safe-action 매핑 발동 + action_ack (L-C)
- [ ] ConformanceDeclaration 등록(dwellSlaSeconds·safeActionTriggers) (L-C)

---

*WIA Perception Clarity Standard · Phase 3: Protocol v1.0.0 · 弘益人間*
