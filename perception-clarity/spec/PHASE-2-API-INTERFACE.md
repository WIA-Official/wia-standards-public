# WIA Perception Clarity Standard — Phase 2: API Interface

---

**Version**: 1.0.0
**Status**: Draft → Published
**Date**: 2025-06-05
**Authors**: WIA (World Certification Industry Association) / SmileStory Inc.
**License**: MIT
**Philosophy**: 弘益人間 · Benefit All Humanity

---

## 목차 (Table of Contents)

1. [개요](#1-개요)
2. [설계 원칙](#2-설계-원칙)
3. [API 표면 개요](#3-api-표면-개요)
4. [기본 URL과 버전 관리](#4-기본-url과-버전-관리)
5. [인증 (Bearer)](#5-인증-bearer)
6. [Rust SDK 타입](#6-rust-sdk-타입)
7. [Rust SDK 사용 예제](#7-rust-sdk-사용-예제)
8. [REST 엔드포인트 상세](#8-rest-엔드포인트-상세)
9. [WebSocket 실시간 스트리밍](#9-websocket-실시간-스트리밍)
10. [에러 처리](#10-에러-처리)
11. [속도 제한](#11-속도-제한)
12. [버전 관리 규약](#12-버전-관리-규약)
13. [참고문헌](#13-참고문헌)

---

## 1. 개요

### 1.1 목적

WIA Perception Clarity API는 Phase 1에서 정의한 데이터 형식(PCI 지표·센서 클래스·명료도 상태·오염 분류·보고 메시지 스키마)을 HTTP/WebSocket 기반 인터페이스로 노출하기 위한 표준입니다. 피지컬 AI 에이전트(자율주행차·로봇·드론·AMR)가 센서별 인식 명료도를 **보고(report)** 하고, fleet/관제 시스템이 현재 명료도를 **조회(query)** 하는 계약을 정의합니다.

이 표준이 표준화하는 것: **보고/조회 API 계약 · 메시지 직렬화 · 실시간 스트리밍 규약**.
이 표준이 표준화하지 않는 것: 세척 하드웨어·노즐·세척액·벤더 알고리즘(특허밭 충돌 회피).

본 표준의 정규(canonical) SDK는 Rust로 기술합니다. 생태계 전반의 일관성을 위해 모든 코드 예제는 Rust로 제시하며, 타 언어 바인딩(C FFI·Python·TypeScript)은 동일한 와이어 포맷(JSON)을 따릅니다.

### 1.2 범위

**포함**:

- 센서 클러리티 보고 제출 (`POST /reports`)
- 에이전트 단위 최신 명료도 조회 (`GET /agents/{agentId}/clarity`)
- 단일 센서 명료도 조회 (`GET /agents/{agentId}/sensors/{sensorId}`)
- Rust SDK 타입과 PCI 산출 로직
- Bearer 토큰 인증
- WebSocket 실시간 명료도 변화 스트리밍

**제외**:

- 적합성 선언서 등록 플로우 → 별도 거버넌스 문서
- 세척 액추에이터 제어 → 본 표준 범위 밖(벤더 영역)

### 1.3 레이어 관계

```
┌──────────────────────────────────────────────────────────────┐
│        Fleet Console · Safety Monitor · Telemetry Sink        │
├──────────────────────────────────────────────────────────────┤
│            Phase 2 API Interface (this specification)         │
│      (reporting · query · WebSocket · Bearer auth)            │
├──────────────────────────────────────────────────────────────┤
│               Phase 1 Data Format (Payload)                   │
│   (PciIndex, Sensor, SensorClarityReport, enums, weights)     │
├──────────────────────────────────────────────────────────────┤
│                   HTTP/1.1 · HTTP/2 · WSS                     │
│                    (TLS 1.2+ REQUIRED)                        │
└──────────────────────────────────────────────────────────────┘
```

### 1.4 핵심 용어

| 용어 | 정의 |
|---|---|
| **Agent** | 센서를 탑재한 피지컬 AI 에이전트(차량·로봇·드론·AMR). `agentId`로 식별 |
| **Sensor** | 에이전트에 탑재된 단일 인식 센서. `sensorId` + `sensorClass` |
| **PCI** | Perception Clarity Index. 센서별 0–100 정수 명료도 지수 |
| **Clarity State** | PCI에서 도출된 이산 상태(clear/degraded/obstructed/blind) |
| **Contaminant** | 명료도를 떨어뜨리는 오염 요인(빗물막·진흙·염무 등) |
| **Dwell** | 비-CLEAR 상태가 연속 잔존한 시간(초) |
| **Report** | 특정 시점 한 에이전트의 전 센서 명료도 스냅샷 |

---

## 2. 설계 원칙

1. **Report-First** — 1급 시민은 `SensorClarityReport`이다. 모든 보고는 자기 완결적(self-describing) 스냅샷이며, 수신 측은 상태를 재계산하지 않고 그대로 신뢰할 수 있어야 한다.
2. **Safety-Critical Read Path** — 조회는 안전 판단에 쓰이므로 항상 최신성과 신선도(staleness) 정보를 함께 노출한다.
3. **Schema-Faithful** — 와이어 포맷은 Phase 1 스키마와 100% 동형이다. 필드 추가/생략 없이 직렬화한다.
4. **Stateless Auth** — 서버 세션 없음. 인증은 매 요청 Bearer 토큰으로.
5. **Stream When Live** — 변화 빈도가 높은 명료도는 폴링 대신 WebSocket으로 푸시한다.
6. **Backward-Compatible Versioning** — 메이저는 URL(`/v1`), 마이너는 비파괴적 추가. 모든 메시지에 `header.version` 포함.

---

## 3. API 표면 개요

API 표면은 두 영역으로 나뉩니다: **reporting**(에이전트가 보고를 밀어 넣음)과 **query**(관제가 현재 명료도를 읽음). openapi.yaml과 정확히 일치합니다.

| # | Method | Path | Tag | 설명 |
|:-:|---|---|---|---|
| 1 | `POST` | `/reports` | reporting | 센서 클러리티 보고 제출 |
| 2 | `GET` | `/agents/{agentId}/clarity` | query | 에이전트 최신 보고 조회 |
| 3 | `GET` | `/agents/{agentId}/sensors/{sensorId}` | query | 단일 센서 명료도 조회 |

추가로 실시간 스트리밍을 위한 WebSocket 업그레이드 경로(§9)를 제공합니다. 이는 query 영역의 푸시 변형이며 동일한 페이로드 타입을 사용합니다.

---

## 4. 기본 URL과 버전 관리

```
https://{host}/api/{version}/{resource}

참조 엔드포인트:
https://perception-clarity.wiastandards.com/api/v1/reports
https://perception-clarity.wiastandards.com/api/v1/agents/{agentId}/clarity
https://perception-clarity.wiastandards.com/api/v1/agents/{agentId}/sensors/{sensorId}
```

본 스펙 기준 `{version}=v1`. 메시지 본문의 `header.version`(예: `"1.0.0"`)은 페이로드 스키마 버전을 나타내며, URL의 `v1`은 HTTP 계약 메이저 버전을 나타냅니다. 둘은 독립적으로 진화합니다(§12).

---

## 5. 인증 (Bearer)

openapi.yaml의 `bearerAuth`(HTTP bearer scheme)를 따릅니다. 모든 엔드포인트는 `Authorization: Bearer <token>` 헤더를 요구합니다.

```
Authorization: Bearer eyJhbGciOiJFZERTQSJ9...
```

| 상황 | 응답 |
|---|---|
| 헤더 없음 | `401` + `WWW-Authenticate: Bearer realm="wia-perception-clarity"` |
| 토큰 서명 오류 | `401` + `error.code="invalid_token"` |
| 토큰 만료 | `401` + `error.code="token_expired"` |
| 권한 부족(예: reporting 토큰으로 query) | `403` + `error.code="insufficient_scope"` |

권장 스코프 분리:

| Scope | 의미 |
|---|---|
| `clarity:report` | `POST /reports` 송출 권한(에이전트 측) |
| `clarity:read` | `GET .../clarity`·`GET .../sensors/{id}` + WebSocket 구독(관제 측) |

§8 Security Considerations(Phase 1 §8)에 따라 명료도 보고는 에이전트 안전 상태를 노출하므로, 전송 구간은 TLS 1.2+가 필수이며 메시지 서명을 권장합니다.

---

## 6. Rust SDK 타입

정규 SDK 크레이트 `wia-perception-clarity`는 Phase 1 데이터 모델을 1:1로 미러링합니다. 모든 타입은 `serde`로 직렬화되며 와이어 포맷은 Phase 1 JSON 스키마와 동형입니다.

### 6.1 열거형 (Enums)

enum의 직렬화 값은 Phase 1·openapi.yaml과 정확히 일치합니다. `#[serde(rename_all = "snake_case")]`를 기본으로 하되, `ConformanceLevel`은 하이픈 표기(`L-A`)이므로 변형별 `rename`을 명시합니다.

```rust
use serde::{Deserialize, Serialize};

/// 센서 클래스 — Phase 1 §2.1
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorClass {
    RgbCamera,    // "rgb_camera"   — 가시광 카메라
    IrThermal,    // "ir_thermal"   — 적외선·열화상
    LidarWindow,  // "lidar_window" — LiDAR 투과 윈도우
    RadarRadome,  // "radar_radome" — 레이더 레이돔
    Ultrasonic,   // "ultrasonic"   — 초음파 송수신면
}

/// 명료도 상태 — Phase 1 §2.2 (순서 불변: Clear > Degraded > Obstructed > Blind)
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClarityState {
    Blind,       // "blind"       — PCI 0–29   · 센서 사용 중단·safe-state
    Obstructed,  // "obstructed"  — PCI 30–59  · 안전 동작(감속·우회·세척)
    Degraded,    // "degraded"    — PCI 60–89  · 모니터링·세척 트리거
    Clear,       // "clear"       — PCI 90–100 · 정상
}

/// 오염 분류 — Phase 1 §2.3
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContaminantType {
    RainFilm,        // "rain_film"        — 빗물막
    MudDust,         // "mud_dust"         — 진흙·먼지
    SaltSpray,       // "salt_spray"       — 염무
    InsectStrike,    // "insect_strike"    — 벌레 충돌
    FrostIce,        // "frost_ice"        — 서리·결빙
    Condensation,    // "condensation"     — 응결(김서림)
    SunGlare,        // "sun_glare"        — 역광·포화
    ScratchAbrasion, // "scratch_abrasion" — 스크래치·마모
    Other,           // "other"            — 미분류(note 권장)
}

/// 에이전트 유형 — Phase 1 §2.4
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AgentType {
    Vehicle,
    Robot,
    Drone,
    Amr,
    Other,
}

/// 적합성 레벨 — Phase 1 §2.5
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConformanceLevel {
    #[serde(rename = "L-A")]
    La, // PCI 산출 + 상태 enum 노출
    #[serde(rename = "L-B")]
    Lb, // A + 표준 보고 메시지 송출
    #[serde(rename = "L-C")]
    Lc, // B + dwell-time SLA + 안전 동작 트리거 연동
}
```

`ClarityState`에 `Ord`를 부여한 까닭은 Phase 1 §2.2의 불변 순서(CLEAR > DEGRADED > OBSTRUCTED > BLIND)를 타입 수준에서 보존하기 위함입니다. 변형 선언 순서를 가장 나쁜 상태(`Blind`)부터 두어 `derive(Ord)`가 "값이 클수록 명료" 규약과 일치하도록 했습니다.

### 6.2 PciIndex

PCI는 0–100 정수입니다. newtype으로 감싸 범위를 강제하고 상태 매핑 책임을 한곳에 둡니다.

```rust
/// Perception Clarity Index — Phase 1 §3. 0–100 정수, 클수록 명료.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct PciIndex(u8);

impl PciIndex {
    /// 0–100으로 클램프하여 생성한다. 산출 결과는 항상 이 생성자를 거친다.
    pub fn new(value: i32) -> Self {
        PciIndex(value.clamp(0, 100) as u8)
    }

    pub fn value(self) -> u8 {
        self.0
    }

    /// Phase 1 §3.3 기본 매핑. 임계 조정 시 선언(§6.2 Phase 1) 후 별도 매퍼 사용.
    pub fn state(self) -> ClarityState {
        match self.0 {
            90..=100 => ClarityState::Clear,
            60..=89 => ClarityState::Degraded,
            30..=59 => ClarityState::Obstructed,
            _ => ClarityState::Blind, // 0–29
        }
    }
}
```

### 6.3 PciWeights와 PciAxes

3개 축(occlusion·distance·MTF)과 센서 클래스별 가중치를 구조체로 노출합니다. 가중치 합은 1.00(±0.001)이어야 합니다(Phase 1 §6.2).

```rust
/// PCI 3개 축의 손상값 — 각 0.0–1.0 (Phase 1 §3.1)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PciAxes {
    /// 가려진 유효 시야 비율 (OCC)
    pub occlusion: f64,
    /// (기준거리 - 현재 유효 인식거리)/기준거리 (DIST)
    #[serde(rename = "distanceDegradation")]
    pub distance_degradation: f64,
    /// 1 - (현재 MTF50 / 기준 MTF50) (MTF)
    #[serde(rename = "mtfReduction")]
    pub mtf_reduction: f64,
}

/// 가중치 세트 — 합 = 1.00 (±0.001), Phase 1 §3.2
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PciWeights {
    pub occlusion: f64,
    pub distance: f64,
    pub mtf: f64,
}

impl PciWeights {
    /// 센서 클래스별 기본 가중치 — Phase 1 §3.2 표
    pub fn default_for(class: SensorClass) -> Self {
        match class {
            SensorClass::RgbCamera =>   PciWeights { occlusion: 0.40, distance: 0.25, mtf: 0.35 },
            SensorClass::IrThermal =>   PciWeights { occlusion: 0.40, distance: 0.35, mtf: 0.25 },
            SensorClass::LidarWindow => PciWeights { occlusion: 0.45, distance: 0.45, mtf: 0.10 },
            SensorClass::RadarRadome => PciWeights { occlusion: 0.30, distance: 0.65, mtf: 0.05 },
            SensorClass::Ultrasonic =>  PciWeights { occlusion: 0.50, distance: 0.50, mtf: 0.00 },
        }
    }

    /// 합이 1.00(±0.001)인지 검증 (Phase 1 §6.2)
    pub fn is_valid(&self) -> bool {
        (self.occlusion + self.distance + self.mtf - 1.0).abs() <= 0.001
    }
}
```

### 6.4 Contaminant

```rust
/// 오염 항목 — Phase 1 §4.1. type·coverage 필수.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Contaminant {
    #[serde(rename = "type")]
    pub kind: ContaminantType,
    /// 0.0–1.0
    pub coverage: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}
```

### 6.5 Sensor

Phase 1 §4.1 / openapi.yaml `Sensor` 스키마를 미러링합니다. 필수: `sensorId`, `sensorClass`, `pci`, `state`, `dwellSeconds`, `confidence`.

```rust
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Sensor {
    #[serde(rename = "sensorId")]
    pub sensor_id: String,
    #[serde(rename = "sensorClass")]
    pub sensor_class: SensorClass,
    pub pci: PciIndex,
    pub state: ClarityState,

    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub contaminants: Vec<Contaminant>,

    /// 산출 투명성·감사용 선택 필드 (Phase 1 §4.1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub axes: Option<PciAxes>,
    #[serde(rename = "pciWeights", skip_serializing_if = "Option::is_none")]
    pub pci_weights: Option<PciWeights>,

    #[serde(rename = "lastCleanedAt", skip_serializing_if = "Option::is_none")]
    pub last_cleaned_at: Option<DateTime<Utc>>,

    /// 비-CLEAR 상태 연속 잔존 시간(초). CLEAR면 0. (≥ 0)
    #[serde(rename = "dwellSeconds")]
    pub dwell_seconds: f64,

    /// PCI 산출 자체의 신뢰도. 0.0–1.0
    pub confidence: f64,
}
```

### 6.6 SensorClarityReport

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReportHeader {
    /// 페이로드 스키마 버전. 예: "1.0.0"
    pub version: String,
    #[serde(rename = "messageId")]
    pub message_id: String, // UUID
    pub timestamp: DateTime<Utc>, // ISO 8601
    #[serde(rename = "agentId")]
    pub agent_id: String,
    #[serde(rename = "agentType")]
    pub agent_type: AgentType,
    #[serde(rename = "conformanceLevel", skip_serializing_if = "Option::is_none")]
    pub conformance_level: Option<ConformanceLevel>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorClarityReport {
    pub header: ReportHeader,
    /// minItems: 1 (openapi.yaml)
    pub sensors: Vec<Sensor>,
}
```

---

## 7. Rust SDK 사용 예제

### 7.1 PCI를 3개 축에서 산출

Phase 1 §3.1의 공식을 그대로 구현합니다:

```
PCI = round( 100 - ( w_occ·OCC + w_dist·DIST + w_mtf·MTF ) × 100 )
```

각 축의 손상값(0.0–1.0)을 가중 합산하여 0–100 손상 스케일로 환산한 뒤 100에서 차감하고 클램프합니다.

```rust
/// 3개 축 + 가중치 → PciIndex. Phase 1 §3.1.
pub fn compute_pci(axes: &PciAxes, weights: &PciWeights) -> PciIndex {
    debug_assert!(weights.is_valid(), "가중치 합은 1.00(±0.001)이어야 한다");

    let damage = weights.occlusion * axes.occlusion
        + weights.distance * axes.distance_degradation
        + weights.mtf * axes.mtf_reduction; // 0.0–1.0

    let pci = 100.0 - damage * 100.0;
    PciIndex::new(pci.round() as i32)
}
```

검산 — Phase 1 §4.2의 강우 카메라 예제(OCC 0.18, DIST 0.22, MTF 0.40, 가중치 0.40/0.25/0.35):

```rust
#[test]
fn rain_camera_yields_degraded_73() {
    let axes = PciAxes { occlusion: 0.18, distance_degradation: 0.22, mtf_reduction: 0.40 };
    let weights = PciWeights::default_for(SensorClass::RgbCamera);
    let pci = compute_pci(&axes, &weights);
    assert_eq!(pci.value(), 73);              // 100 - (0.072+0.055+0.140)*100 = 73.3 → 반올림 검증
    assert_eq!(pci.state(), ClarityState::Degraded);
}
```

> 구현 주의: 보고 메시지의 `pci`는 이 산출값과 일치해야 하며, `state`는 §6.2 `PciIndex::state()` 또는 선언된 임계 매퍼와 모순이 없어야 합니다(Phase 1 §6.3).

### 7.2 보고 빌드 (builder)

```rust
use uuid::Uuid;

fn build_report() -> SensorClarityReport {
    // 1) 카메라 센서 — 강우 중 DEGRADED
    let cam_axes = PciAxes { occlusion: 0.18, distance_degradation: 0.22, mtf_reduction: 0.40 };
    let cam_weights = PciWeights::default_for(SensorClass::RgbCamera);
    let cam_pci = compute_pci(&cam_axes, &cam_weights);

    let front_cam = Sensor {
        sensor_id: "front-cam-main".into(),
        sensor_class: SensorClass::RgbCamera,
        pci: cam_pci,
        state: cam_pci.state(),
        contaminants: vec![Contaminant {
            kind: ContaminantType::RainFilm,
            coverage: 0.35,
            note: None,
        }],
        axes: Some(cam_axes),
        pci_weights: Some(cam_weights),
        last_cleaned_at: None,
        dwell_seconds: 47.0,
        confidence: 0.92,
    };

    // 2) LiDAR — CLEAR (dwell 0, 오염 없음)
    let roof_lidar = Sensor {
        sensor_id: "roof-lidar-1".into(),
        sensor_class: SensorClass::LidarWindow,
        pci: PciIndex::new(95),
        state: ClarityState::Clear,
        contaminants: vec![],
        axes: None,
        pci_weights: None,
        last_cleaned_at: None,
        dwell_seconds: 0.0,
        confidence: 0.98,
    };

    SensorClarityReport {
        header: ReportHeader {
            version: "1.0.0".into(),
            message_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now(),
            agent_id: "veh-seoul-0421".into(),
            agent_type: AgentType::Vehicle,
            conformance_level: Some(ConformanceLevel::Lc),
        },
        sensors: vec![front_cam, roof_lidar],
    }
}
```

### 7.3 보고 제출 (`POST /reports`)

크레이트는 `reqwest` 기반 비동기 클라이언트를 제공합니다.

```rust
use reqwest::StatusCode;

pub struct ClarityClient {
    base: String,        // "https://perception-clarity.wiastandards.com/api/v1"
    token: String,       // Bearer
    http: reqwest::Client,
}

#[derive(Debug, Deserialize)]
pub struct ReportAccepted {
    #[serde(rename = "messageId")]
    pub message_id: String,
    #[serde(rename = "receivedAt")]
    pub received_at: DateTime<Utc>,
}

impl ClarityClient {
    pub fn new(base: impl Into<String>, token: impl Into<String>) -> Self {
        Self { base: base.into(), token: token.into(), http: reqwest::Client::new() }
    }

    /// POST /reports → 202 Accepted
    pub async fn submit_report(
        &self,
        report: &SensorClarityReport,
    ) -> Result<ReportAccepted, ClarityError> {
        let res = self.http
            .post(format!("{}/reports", self.base))
            .bearer_auth(&self.token)
            .json(report)
            .send()
            .await?;

        match res.status() {
            StatusCode::ACCEPTED => Ok(res.json().await?),
            StatusCode::BAD_REQUEST => Err(ClarityError::Malformed(res.json().await.ok())),
            StatusCode::UNAUTHORIZED => Err(ClarityError::Unauthorized),
            other => Err(ClarityError::Unexpected(other.as_u16())),
        }
    }
}
```

### 7.4 명료도 조회

```rust
impl ClarityClient {
    /// GET /agents/{agentId}/clarity → 최신 SensorClarityReport
    pub async fn get_agent_clarity(
        &self,
        agent_id: &str,
    ) -> Result<SensorClarityReport, ClarityError> {
        let res = self.http
            .get(format!("{}/agents/{}/clarity", self.base, agent_id))
            .bearer_auth(&self.token)
            .send()
            .await?;

        match res.status() {
            StatusCode::OK => Ok(res.json().await?),
            StatusCode::NOT_FOUND => Err(ClarityError::NotFound),
            StatusCode::UNAUTHORIZED => Err(ClarityError::Unauthorized),
            other => Err(ClarityError::Unexpected(other.as_u16())),
        }
    }

    /// GET /agents/{agentId}/sensors/{sensorId} → 단일 Sensor
    pub async fn get_sensor_clarity(
        &self,
        agent_id: &str,
        sensor_id: &str,
    ) -> Result<Sensor, ClarityError> {
        let res = self.http
            .get(format!("{}/agents/{}/sensors/{}", self.base, agent_id, sensor_id))
            .bearer_auth(&self.token)
            .send()
            .await?;

        match res.status() {
            StatusCode::OK => Ok(res.json().await?),
            StatusCode::NOT_FOUND => Err(ClarityError::NotFound),
            other => Err(ClarityError::Unexpected(other.as_u16())),
        }
    }
}
```

### 7.5 안전 판단 예제 — BLIND 센서 격리

조회 결과를 안전 동작으로 잇는 전형적 소비 패턴입니다. 상태 enum의 `Ord` 덕분에 임계 비교가 명료합니다.

```rust
/// 가장 나쁜 센서 상태를 찾아 안전 동작을 결정한다.
pub fn worst_state(report: &SensorClarityReport) -> ClarityState {
    report.sensors.iter().map(|s| s.state).min().unwrap_or(ClarityState::Clear)
}

pub async fn safety_gate(client: &ClarityClient, agent_id: &str) -> Result<(), ClarityError> {
    let report = client.get_agent_clarity(agent_id).await?;
    match worst_state(&report) {
        ClarityState::Blind => println!("safe-state 진입: 해당 센서 사용 중단"),
        ClarityState::Obstructed => println!("감속·우회·세척 트리거"),
        ClarityState::Degraded => println!("모니터링 강화"),
        ClarityState::Clear => {}
    }
    Ok(())
}
```

---

## 8. REST 엔드포인트 상세

### 8.1 `POST /reports` — 보고 제출

요청 본문은 `SensorClarityReport`(§6.6). `header`와 최소 1개의 `sensors` 항목이 필수입니다.

```
POST /api/v1/reports
Authorization: Bearer <token>
Content-Type: application/json

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
      "contaminants": [ { "type": "rain_film", "coverage": 0.35 } ],
      "axes": { "occlusion": 0.18, "distanceDegradation": 0.22, "mtfReduction": 0.40 },
      "pciWeights": { "occlusion": 0.40, "distance": 0.25, "mtf": 0.35 },
      "lastCleanedAt": "2025-06-05T08:02:10Z",
      "dwellSeconds": 47,
      "confidence": 0.92
    }
  ]
}
```

응답 `202 Accepted`:

```json
{ "messageId": "7c1f0a2e-3b9d-4a51-8e2c-91a0b6d4f7e2", "receivedAt": "2025-06-05T08:14:32Z" }
```

| 상태 | 의미 |
|---|---|
| `202` | 보고 접수 |
| `400` | 형식 오류(스키마·정합성 위반, §10) |
| `401` | 인증 실패 |

### 8.2 `GET /agents/{agentId}/clarity` — 에이전트 최신 명료도

```
GET /api/v1/agents/veh-seoul-0421/clarity
Authorization: Bearer <token>
```

응답 `200 OK`: 가장 최근 `SensorClarityReport`(전 센서 스냅샷). `404`: 에이전트 없음 또는 보고 부재.

신선도(staleness) 안내는 응답 헤더로 노출합니다:

| 헤더 | 값 |
|---|---|
| `Age` | 최신 보고 수신 후 경과 초 |
| `X-Report-Timestamp` | 원 보고의 `header.timestamp` |
| `X-Report-Stale` | `true` 이면 신선도 임계 초과(소비 측 안전 판단 주의) |

### 8.3 `GET /agents/{agentId}/sensors/{sensorId}` — 단일 센서

```
GET /api/v1/agents/veh-seoul-0421/sensors/front-cam-main
Authorization: Bearer <token>
```

응답 `200 OK`: 단일 `Sensor` 객체. `404`: 센서 없음.

```json
{
  "sensorId": "front-cam-main",
  "sensorClass": "rgb_camera",
  "pci": 73,
  "state": "degraded",
  "contaminants": [ { "type": "rain_film", "coverage": 0.35 } ],
  "dwellSeconds": 47,
  "confidence": 0.92
}
```

---

## 9. WebSocket 실시간 스트리밍

명료도는 운행 중 빠르게 변하므로(빗방울·진흙·역광), 폴링 대신 WebSocket으로 변화를 푸시합니다. 이는 query 영역의 푸시 변형이며 페이로드 타입은 §6과 동일합니다.

### 9.1 핸드셰이크

```
GET /api/v1/ws/agents/{agentId}/clarity
Authorization: Bearer <token-with-clarity-read>
Sec-WebSocket-Protocol: wia-perception-clarity-v1
```

연결 직후 서버는 현재 상태를 1회 `clarity.snapshot`으로 보낸 뒤, 이후 변화만 증분 전송합니다.

### 9.2 서버 → 클라이언트 메시지

| `type` | 페이로드 요약 |
|---|---|
| `clarity.snapshot` | 연결 직후 전체 `SensorClarityReport` 1회 |
| `sensor.state_change` | `sensorId`·이전 `state`·새 `state`·`pci`·`dwellSeconds` |
| `sensor.pci_update` | `sensorId`·`pci`·`state`(임계 미교차 미세 변화) |
| `contaminant.detected` | `sensorId`·`Contaminant` |
| `agent.worst_state` | 에이전트 전체 최악 상태 변화(안전 게이트용) |

상태가 임계를 교차할 때(`state_change`)는 반드시 즉시 전송하고, 임계 내 PCI 변동(`pci_update`)은 디바운스(권장 ≥ 1초)할 수 있습니다.

### 9.3 Rust 구독 예제

`tokio-tungstenite` 기반:

```rust
use futures_util::StreamExt;
use tokio_tungstenite::tungstenite::Message;

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ClarityEvent {
    #[serde(rename = "clarity.snapshot")]
    Snapshot { report: SensorClarityReport },
    #[serde(rename = "sensor.state_change")]
    StateChange {
        #[serde(rename = "sensorId")] sensor_id: String,
        from: ClarityState,
        to: ClarityState,
        pci: PciIndex,
    },
    #[serde(rename = "agent.worst_state")]
    WorstState { state: ClarityState },
}

pub async fn subscribe_clarity(url: &str, token: &str) -> Result<(), ClarityError> {
    let req = http::Request::builder()
        .uri(url)
        .header("Authorization", format!("Bearer {token}"))
        .header("Sec-WebSocket-Protocol", "wia-perception-clarity-v1")
        .body(())?;

    let (mut ws, _) = tokio_tungstenite::connect_async(req).await?;
    while let Some(msg) = ws.next().await {
        if let Message::Text(txt) = msg? {
            match serde_json::from_str::<ClarityEvent>(&txt)? {
                ClarityEvent::StateChange { sensor_id, from, to, .. } if to < from => {
                    eprintln!("악화: {sensor_id} {from:?} → {to:?}");
                }
                ClarityEvent::WorstState { state: ClarityState::Blind } => {
                    eprintln!("safe-state 진입");
                }
                _ => {}
            }
        }
    }
    Ok(())
}
```

### 9.4 하트비트

15초 간격 `ping`/`pong`. 30초 내 미응답 시 서버가 `1008`로 연결을 종료합니다. 재연결 시 서버는 다시 `clarity.snapshot`을 선행 전송합니다.

---

## 10. 에러 처리

### 10.1 에러 봉투

실패 응답은 일관된 봉투를 사용합니다:

```json
{
  "error": {
    "code": "validation_failed",
    "message": "state는 pci 및 적용 임계와 모순됩니다.",
    "errors": [
      { "path": "sensors[0].state", "expected": "degraded", "actual": "clear" }
    ],
    "doc_url": "https://perception-clarity.wiastandards.com/errors/validation_failed"
  },
  "meta": { "requestId": "req_01H...", "generatedAt": "2025-06-05T08:14:32Z" }
}
```

### 10.2 에러 코드 표

| HTTP | `error.code` | 의미 |
|---|---|---|
| 400 | `malformed_report` · `invalid_request` | 구문·JSON 오류 |
| 400 | `validation_failed` | Phase 1 §6 정합성 위반 |
| 401 | `invalid_token` · `token_expired` | 인증 실패 |
| 403 | `insufficient_scope` | 스코프 부족(예: report 토큰으로 query) |
| 404 | `agent_not_found` · `sensor_not_found` | 대상 없음 |
| 422 | `weights_sum_invalid` · `state_pci_conflict` | 의미 정합성 |
| 429 | `rate_limit_exceeded` | §11 |
| 500 | `server_error` | 일반 |
| 503 | `service_unavailable` | 가용성 |

### 10.3 정합성 검증(서버 측)

서버는 Phase 1 §6의 규칙을 보고 접수 시 검증합니다:

- `pci`는 0–100 정수. 위반 → `validation_failed`.
- `coverage`·`axes.*`·`confidence`는 0.0–1.0.
- `pciWeights.*` 합 = 1.00(±0.001). 위반 → `weights_sum_invalid`.
- `dwellSeconds ≥ 0`이며, `dwellSeconds > 0`이면 `state ≠ clear`.
- `state`는 보고된 `pci`와 적용 임계에 일치(불일치 → `state_pci_conflict`). 임계를 기본값에서 조정한 경우 별도 선언이 동반되어야 합니다.

### 10.4 SDK 에러 타입

```rust
#[derive(Debug, thiserror::Error)]
pub enum ClarityError {
    #[error("network: {0}")]
    Network(#[from] reqwest::Error),
    #[error("decode: {0}")]
    Decode(#[from] serde_json::Error),
    #[error("malformed report")]
    Malformed(Option<serde_json::Value>),
    #[error("unauthorized")]
    Unauthorized,
    #[error("not found")]
    NotFound,
    #[error("unexpected status {0}")]
    Unexpected(u16),
}
```

---

## 11. 속도 제한

| 인증·영역 | 한도 |
|---|---|
| reporting (`POST /reports`) | 600 req/min/token (고빈도 텔레메트리 허용) |
| query (`GET ...`) | 300 req/min/token |
| WebSocket 구독 | 동시 50 connections/token |

응답 헤더: `X-RateLimit-Limit` / `X-RateLimit-Remaining` / `X-RateLimit-Reset`, 초과 시 `429` + `Retry-After`.

고빈도 보고가 필요한 에이전트는 폴링 대신 단일 WebSocket 구독으로 전환하여 한도 부담을 덜도록 권장합니다.

---

## 12. 버전 관리 규약

Phase 1 §7과 정합합니다.

- **HTTP 계약 메이저**: URL의 `/v1`. 호환 불가 변경 시 `/v2` 신설.
- **페이로드 스키마 버전**: `header.version`(SemVer). 모든 메시지에 필수.
  - **Major** (1.x.x): 호환 불가 변경
  - **Minor** (x.1.x): 하위호환 기능 추가(새 선택 필드·새 enum 변형)
  - **Patch** (x.x.1): 버그 수정

서버는 알 수 없는 선택 필드를 무시(tolerant reader)하고, 알 수 없는 enum 변형은 보고 거부 대신 `400 validation_failed`로 명시 응답해야 합니다. SDK의 `#[serde(skip_serializing_if = ...)]`·`#[non_exhaustive]` 관행은 마이너 추가에 대한 클라이언트 측 전방 호환을 보장합니다.

---

## 13. 참고문헌

- **VDA5050** — Interface for the communication between automated guided vehicles and a master control (보고 규약 본보기)
- **SAE J3016** — Taxonomy and Definitions for Terms Related to Driving Automation Systems
- **ISO 21448 (SOTIF)** — Safety Of The Intended Functionality
- **ISO 12233** — Photography, electronic still-picture imaging, resolution and spatial frequency responses (MTF)
- **RFC 9110** — HTTP Semantics
- **RFC 6750** — OAuth 2.0 Bearer Token Usage
- **RFC 6455** — The WebSocket Protocol
- **RFC 3339** — Date and Time on the Internet
- **OpenAPI 3.0.3** — `spec/openapi.yaml`
- WIA Perception Clarity Standard Phase 1 (`spec/PHASE-1-DATA-FORMAT.md`)

---

*WIA Perception Clarity Standard · Phase 2: API Interface v1.0.0 · 弘益人間*
