# WIA Game Phase 3: Communication Protocol
## 입력 장치 통신 프로토콜 표준

**Version**: 1.0.0
**Date**: 2025-01-15
**Status**: Complete

---

## 1. 개요

게임 접근성 입력 장치와의 표준화된 통신 프로토콜을 정의합니다.

### 1.1 지원 장치

| 카테고리 | 장치 |
|---------|------|
| **적응형 컨트롤러** | Xbox Adaptive Controller, PlayStation Access, HORI Flex |
| **스위치 접근** | Button Arrays, Sip-and-Puff, Switch Interfaces |
| **아이 트래커** | Tobii Eye Tracker 5, Irisbond Duo, EyeWare Beam |
| **헤드 트래커** | TrackIR, Tobii Head Tracking, Webcam-based |
| **보이스 컨트롤** | Voice Attack, Windows Speech Recognition |
| **기타** | QuadStick, Mouth Controllers, Foot Pedals |

---

## 2. HID 프로토콜 계층

### 2.1 USB HID Gamepad

Xbox Adaptive Controller는 USB HID 표준을 따릅니다:

```
USB HID Report Descriptor:
- Usage Page: Generic Desktop (0x01)
- Usage: Gamepad (0x05)
- Input Reports: Buttons, Axes, D-Pad
```

### 2.2 버튼 매핑 (Xbox Adaptive Controller)

| USB 포트 | 버튼 1-8 매핑 |
|----------|---------------|
| **Left Port** | X1, X2, ThumbBtnL, A, B, View, Menu, (ignored) |
| **Right Port** | View+Menu, ThumbBtnR, X, Y, X1, X2, (ignored) |

### 2.3 축 매핑

```rust
pub struct GamepadAxes {
    pub left_stick_x: i16,   // -32768 to 32767
    pub left_stick_y: i16,
    pub right_stick_x: i16,
    pub right_stick_y: i16,
    pub left_trigger: u8,    // 0 to 255
    pub right_trigger: u8,
}
```

---

## 3. 이벤트 시스템

### 3.1 입력 이벤트 타입

```rust
pub enum InputEvent {
    ButtonPressed { button: Button, timestamp: u64 },
    ButtonReleased { button: Button, timestamp: u64 },
    AxisMoved { axis: Axis, value: f32, timestamp: u64 },
    GazePoint { x: f32, y: f32, timestamp: u64 },
    HeadPose { yaw: f32, pitch: f32, roll: f32, timestamp: u64 },
    VoiceCommand { command: String, confidence: f32, timestamp: u64 },
}
```

### 3.2 이벤트 필터링

| 필터 | 설명 | 기본값 |
|------|------|--------|
| **Debounce** | 버튼 바운스 제거 | 50ms |
| **Dead Zone** | 스틱 데드존 | 0.15 |
| **Smoothing** | 아이트래킹 스무딩 | 0.3 |
| **Threshold** | 트리거 활성화 임계값 | 0.1 |

---

## 4. 아이 트래커 통합

### 4.1 Tobii Game Integration

[Tobii Developer Zone](https://developer.tobii.com/)에서 제공하는 TGI SDK 기반:

```rust
pub struct GazeData {
    pub gaze_point: Point2D,      // 정규화된 화면 좌표 (0.0-1.0)
    pub gaze_origin: Point3D,     // 눈 위치 (mm)
    pub head_pose: HeadPose,      // 머리 위치/회전
    pub validity: GazeValidity,   // 유효성 플래그
    pub timestamp: u64,           // 마이크로초
}
```

### 4.2 드웰 선택 (Dwell Selection)

```rust
pub struct DwellConfig {
    pub dwell_time_ms: u32,       // 선택까지 응시 시간 (기본 800ms)
    pub tolerance_radius: f32,    // 허용 반경 (기본 0.05)
    pub progressive_feedback: bool, // 진행률 표시
}
```

---

## 5. 스위치 접근

### 5.1 스캐닝 모드

| 모드 | 설명 |
|------|------|
| **Auto Scan** | 자동으로 옵션 순환 |
| **Step Scan** | 스위치로 다음 항목 이동 |
| **Row-Column** | 행 선택 → 열 선택 |
| **Group Scan** | 그룹 → 항목 선택 |

### 5.2 스위치 설정

```rust
pub struct SwitchConfig {
    pub scan_speed_ms: u32,       // 스캔 속도 (기본 1000ms)
    pub scan_mode: ScanMode,
    pub auto_restart: bool,       // 끝에서 재시작
    pub loops_before_exit: u8,    // 종료까지 반복 횟수
    pub audio_feedback: bool,     // 오디오 피드백
    pub visual_highlight: bool,   // 시각적 강조
}
```

---

## 6. 매크로 시스템

### 6.1 매크로 정의

```rust
pub struct Macro {
    pub name: String,
    pub trigger: MacroTrigger,
    pub actions: Vec<MacroAction>,
    pub repeat_mode: RepeatMode,
    pub enabled: bool,
}

pub enum MacroAction {
    PressButton { button: Button },
    ReleaseButton { button: Button },
    HoldButton { button: Button, duration_ms: u32 },
    MoveAxis { axis: Axis, value: f32 },
    Delay { ms: u32 },
    PlaySound { sound_id: String },
}
```

### 6.2 트리거 타입

```rust
pub enum MacroTrigger {
    Button { button: Button },
    ButtonCombo { buttons: Vec<Button> },
    VoiceCommand { phrase: String },
    GazeRegion { region: Rect },
    Timer { interval_ms: u32 },
}
```

---

## 7. 코파일럿 모드

두 컨트롤러가 하나처럼 동작:

```rust
pub struct CopilotConfig {
    pub enabled: bool,
    pub primary_device: DeviceId,
    pub secondary_device: DeviceId,
    pub merge_mode: MergeMode,
}

pub enum MergeMode {
    FirstInput,      // 먼저 입력된 값 사용
    LastInput,       // 마지막 입력 값 사용
    Sum,             // 합산 (축)
    Override,        // 보조가 주 컨트롤러 덮어쓰기
}
```

---

## 8. 디바이스 핫플러그

```rust
pub enum DeviceEvent {
    Connected { device_id: DeviceId, info: DeviceInfo },
    Disconnected { device_id: DeviceId },
    ConfigChanged { device_id: DeviceId },
    BatteryLow { device_id: DeviceId, level: u8 },
    Error { device_id: DeviceId, error: DeviceError },
}
```

---

## 9. 참고 자료

- [Xbox Adaptive Controller](https://www.xbox.com/en-US/accessories/controllers/xbox-adaptive-controller)
- [Xbox Adaptive Controller HID Descriptor](https://gist.github.com/darthcloud/0e6bb1f14a805f9d32075d2812a06abc)
- [ControllersInfo HID Dumps](https://github.com/DJm00n/ControllersInfo)
- [Tobii Developer Zone](https://developer.tobii.com/)
- [Tobii Game Integration](https://developer.tobii.com/pc-gaming/develop/tobii-game-integration/)

---

弘益人間 - Gaming for Everyone 🎮


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
