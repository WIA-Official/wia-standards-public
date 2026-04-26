# WIA-AIR-POWER PHASE 1 — Data Format Specification

**Standard:** WIA-AIR-POWER (wireless power coordination)
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-AIR-POWER-v1.0.md` §2 (Technical
Foundation) and §3 (Architecture)

This document defines the canonical data structures for wireless-power
deployment: device identification, power-budget envelopes, capability
descriptors, and consent ledger entries. Schemas are expressed in JSON
Schema 2020-12 and stable for the lifetime of this PHASE.

References:
- IEC 63563 (wireless power transfer for consumer products)
- ISO/IEC 27001:2022 (information security management)
- AirFuel Resonant 1.0 (industry alliance specification)
- Qi v1.3 (Wireless Power Consortium)
- OpenAPI 3.1, JSON Schema 2020-12

---

## 2. Technical Foundation

### 2.1 Wireless Power Transfer Methods

#### 2.1.1 RF (Radio Frequency) Energy Harvesting
```yaml
rf_harvesting:
  principle: "라디오파 에너지를 전기로 변환"

  frequencies:
    - band: 900MHz    # 저주파, 장거리
    - band: 2.4GHz    # WiFi 대역
    - band: 5GHz      # WiFi 대역
    - band: 5.8GHz    # ISM 대역

  range:
    near_field: 0 ~ 1m      # 높은 효율
    mid_field: 1m ~ 5m      # 중간 효율
    far_field: 5m ~ 15m     # 낮은 효율, 하지만 충분

  safety:
    compliance: [FCC, CE, KC, TELEC]
    sar_limit: within_standards
```

#### 2.1.2 Magnetic Resonance (자기 공명)
```yaml
magnetic_resonance:
  principle: "공명 주파수로 자기장 에너지 전달"

  frequency: 6.78MHz  # AirFuel 호환

  range: 0 ~ 50cm     # 근거리이지만 비접촉

  efficiency:
    at_10cm: 90%
    at_30cm: 70%
    at_50cm: 50%

  advantage: "높은 효율, 다중 기기 동시 충전"
```

#### 2.1.3 Infrared / Laser
```yaml
infrared_power:
  principle: "적외선 빔으로 에너지 전달"

  wavelength: 850nm ~ 1550nm

  range: 0 ~ 10m

  safety:
    eye_safe: required
    skin_safe: required
    auto_shutoff: when_obstructed

  advantage: "지향성, 높은 효율"

  challenge: "직선 경로 필요, 안전성"
```

#### 2.1.4 Hybrid Approach (WIA 권장)
```yaml
wia_hybrid:
  principle: "상황에 따라 최적의 방식 자동 선택"

  modes:
    - near: magnetic_resonance    # 가까우면 자기공명
    - mid: rf_focused             # 중거리면 RF 집중
    - far: rf_ambient             # 멀면 RF 수집
    - direct: infrared            # 직선경로면 적외선

  auto_switch: true
  seamless: true
```


## 3. Architecture

### 3.1 System Overview

```
┌─────────────────────────────────────────────────────────┐
│                    Power Transmitters                    │
│                        (삼촌들)                          │
│                                                          │
│   WiFi Router    │   Dedicated TX   │   Street Lamp     │
│   + Power TX     │   (전용 송신기)   │   + Power TX      │
└────────┬─────────────────┬─────────────────┬────────────┘
         │                 │                 │
         │    ~~~~~~~~~~~~ AIR ~~~~~~~~~~~~  │
         │         (공기 중 전력 전달)         │
         │                 │                 │
         ▼                 ▼                 ▼
┌─────────────────────────────────────────────────────────┐
│                    Power Receivers                       │
│                        (조카들)                          │
│                                                          │
│   📱 Phone   │   💻 Laptop   │   ⌚ Watch   │   🎧 Buds │
│   + RX       │   + RX        │   + RX       │   + RX    │
└─────────────────────────────────────────────────────────┘
```

### 3.2 Transmitter (TX) - 삼촌

```yaml
transmitter:
  types:
    # 가정용
    home:
      form_factor: "WiFi 라우터 통합 or 독립형"
      coverage: "방 1개 (약 20㎡)"
      power_output: 10W ~ 30W
      devices_supported: 10+

    # 상업용 (카페, 사무실)
    commercial:
      form_factor: "천장 매립 or 벽걸이"
      coverage: "넓은 공간 (약 100㎡)"
      power_output: 50W ~ 100W
      devices_supported: 50+

    # 공공용 (거리, 역)
    public:
      form_factor: "가로등 통합, 기지국 통합"
      coverage: "야외 넓은 영역"
      power_output: 100W+
      devices_supported: 100+

    # 차량용
    vehicle:
      form_factor: "차량 내장"
      coverage: "차량 내부"
      power_output: 20W
      devices_supported: 5+

  features:
    - multi_device: true          # 다중 기기 동시 충전
    - device_tracking: true       # 기기 위치 추적
    - power_focusing: true        # 전력 집중 빔포밍
    - obstacle_detection: true    # 장애물 감지
    - safety_shutoff: true        # 안전 차단
```

### 3.3 Receiver (RX) - 조카

```yaml
receiver:
  types:
    # 스마트폰용
    smartphone:
      form_factor: "내장 안테나 + 칩"
      power_receive: 5W ~ 15W
      battery_impact: minimal

    # 웨어러블용
    wearable:
      form_factor: "초소형 칩"
      power_receive: 0.5W ~ 2W
      always_on: true

    # 이어버드용
    earbud:
      form_factor: "마이크로 칩"
      power_receive: 0.1W ~ 0.5W
      케이스도_충전: true

    # IoT용
    iot:
      form_factor: "모듈"
      power_receive: 0.01W ~ 1W
      무배터리_동작: possible

    # 노트북용
    laptop:
      form_factor: "통합 안테나"
      power_receive: 15W ~ 45W
      케이블_완전_제거: true

  features:
    - multi_source: true          # 여러 TX에서 동시 수신
    - smart_negotiation: true     # 전력 협상
    - battery_management: true    # 배터리 최적 관리
    - trickle_charge: true        # 항상 조금씩 충전
```


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
