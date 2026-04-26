# WIA Exoskeleton Gait Cycle Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14

## 1. Overview

이 문서는 재활 외골격 시스템에서 보행 주기(Gait Cycle) 데이터를 표현하기 위한 표준 형식을
정의합니다. 보행 주기는 한 발의 뒤꿈치 접촉(heel strike)부터 같은 발의 다음 뒤꿈치 접촉까지의
완전한 주기를 나타냅니다.

## 2. Gait Cycle Fundamentals

### 2.1 Normal Gait Cycle Division

정상 보행 주기는 다음과 같이 구분됩니다:

| Phase | Percentage | Description |
|-------|------------|-------------|
| **Stance Phase** | 0-60% | 발이 지면에 접촉한 상태 |
| **Swing Phase** | 60-100% | 발이 공중에 있는 상태 |

### 2.2 Stance Phase Sub-divisions

| Sub-phase | Percentage | Description |
|-----------|------------|-------------|
| Initial Contact | 0-2% | 뒤꿈치 접촉 (Heel Strike) |
| Loading Response | 2-12% | 체중 부하 (Foot Flat) |
| Mid-stance | 12-31% | 중간 입각기 |
| Terminal Stance | 31-50% | 말기 입각기 (Heel Off) |
| Pre-swing | 50-60% | 전유각기 (Toe Off 준비) |

### 2.3 Swing Phase Sub-divisions

| Sub-phase | Percentage | Description |
|-----------|------------|-------------|
| Initial Swing | 60-73% | 초기 유각기 |
| Mid-swing | 73-87% | 중간 유각기 |
| Terminal Swing | 87-100% | 말기 유각기 |

## 3. Data Structure

### 3.1 GaitCycle

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `phase` | GaitPhase | Yes | 현재 보행 단계 |
| `percentComplete` | number | Yes | 주기 진행률 (0-100%) |
| `timing` | GaitTiming | Yes | 시간 파라미터 |
| `spatial` | SpatialParameters | Yes | 공간 파라미터 |

### 3.2 GaitPhase Enumeration

```typescript
enum GaitPhase {
  // Stance Phases (지지기)
  RIGHT_STANCE = 'right_stance',
  LEFT_STANCE = 'left_stance',

  // Double Support Phases (이중 지지기)
  INITIAL_DOUBLE_SUPPORT = 'initial_double_support',
  TERMINAL_DOUBLE_SUPPORT = 'terminal_double_support',

  // Swing Phases (유각기)
  RIGHT_SWING = 'right_swing',
  LEFT_SWING = 'left_swing',

  // Detailed Sub-phases (세부 단계)
  HEEL_STRIKE = 'heel_strike',
  FOOT_FLAT = 'foot_flat',
  MIDSTANCE = 'midstance',
  HEEL_OFF = 'heel_off',
  TOE_OFF = 'toe_off',
}
```

## 4. Timing Parameters

### 4.1 GaitTiming Structure

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `cycleStart` | number | ms (timestamp) | 주기 시작 시간 |
| `stanceStart` | number | ms (timestamp) | 입각기 시작 시간 |
| `swingStart` | number | ms (timestamp) | 유각기 시작 시간 |
| `cycleDuration` | number | ms | 전체 주기 지속 시간 |
| `stanceDuration` | number | ms | 입각기 지속 시간 |
| `swingDuration` | number | ms | 유각기 지속 시간 |

### 4.2 Normal Timing Values

정상 성인 보행 (자유 보행 속도 기준):

| Parameter | Normal Range | Description |
|-----------|--------------|-------------|
| Cycle Duration | 1000-1200 ms | 전체 주기 |
| Stance Duration | 600-720 ms | 입각기 (60%) |
| Swing Duration | 400-480 ms | 유각기 (40%) |
| Double Support | 200-240 ms | 이중 지지기 (총 20%) |
| Single Support | 400-480 ms | 단일 지지기 (40%) |

### 4.3 Timing Ratios

| Ratio | Normal Value | Clinical Significance |
|-------|--------------|----------------------|
| Stance/Swing | 60:40 | 비대칭 시 이상 보행 |
| Double Support % | 20-25% | 증가 시 안정성 저하 |
| Step Time Symmetry | ~1.0 | 1.0에서 벗어날수록 비대칭 |

## 5. Spatial Parameters

### 5.1 SpatialParameters Structure

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `strideLength` | number | cm | 활보장 (한 발 두 번 디딤) |
| `stepLength` | number | cm | 보장 (한 발 한 번 디딤) |
| `stepWidth` | number | cm | 보폭 (좌우 간격) |
| `cadence` | number | steps/min | 분속수 |
| `velocity` | number | m/s | 보행 속도 |

### 5.2 Normal Spatial Values

정상 성인 보행 기준값:

| Parameter | Normal Range | Description |
|-----------|--------------|-------------|
| Stride Length | 130-160 cm | 신장의 약 80% |
| Step Length | 65-80 cm | 활보장의 50% |
| Step Width | 8-12 cm | 골반 너비 기준 |
| Cadence | 100-120 steps/min | 자유 보행 |
| Velocity | 1.2-1.4 m/s | 자유 보행 속도 |

### 5.3 Derived Metrics

```
velocity = (stride_length × cadence) / 120
stride_length = step_length_left + step_length_right
symmetry_index = |left - right| / ((left + right) / 2) × 100
```

## 6. Complete Example

```json
{
  "phase": "midstance",
  "percentComplete": 25.5,
  "timing": {
    "cycleStart": 1702598400000,
    "stanceStart": 1702598400000,
    "swingStart": 1702598400660,
    "cycleDuration": 1100,
    "stanceDuration": 660,
    "swingDuration": 440
  },
  "spatial": {
    "strideLength": 145.2,
    "stepLength": 72.8,
    "stepWidth": 10.5,
    "cadence": 109,
    "velocity": 1.32
  }
}
```

## 7. Phase Transition Detection

### 7.1 Detection Methods

| Transition | Primary Method | Secondary Method |
|------------|----------------|------------------|
| Heel Strike | Vertical GRF > threshold | Foot switch |
| Foot Flat | Foot switch pattern | Ankle angle |
| Heel Off | Heel pressure = 0 | CoP movement |
| Toe Off | Vertical GRF < threshold | Toe switch |

### 7.2 Threshold Values

| Parameter | Typical Threshold |
|-----------|-------------------|
| Heel Strike GRF | > 20 N |
| Toe Off GRF | < 20 N |
| Swing Detection | GRF < 10 N for > 50 ms |

## 8. Gait Events Timeline

```
Time (% of cycle)
0%        10%       20%       30%       40%       50%       60%       70%       80%       90%      100%
|---------|---------|---------|---------|---------|---------|---------|---------|---------|---------|
|←-------- STANCE PHASE (60%) --------→|←------- SWING PHASE (40%) ------→|
|IC |LR   |         MS         |   TS   |PSw|ISw  |    MSw    |  TSw   |

Legend:
IC  = Initial Contact (Heel Strike)
LR  = Loading Response (Foot Flat)
MS  = Mid-stance
TS  = Terminal Stance (Heel Off)
PSw = Pre-swing (Toe Off)
ISw = Initial Swing
MSw = Mid-swing
TSw = Terminal Swing
```

## 9. Validation Rules

1. `percentComplete`는 0 이상 100 이하여야 합니다.
2. `stanceDuration + swingDuration = cycleDuration`
3. `cycleDuration`은 500ms 이상 3000ms 이하 (비정상 보행 포함)
4. `velocity`는 0 이상 3.0 m/s 이하 (달리기 제외)
5. `cadence`는 30 이상 180 이하 steps/min
6. `strideLength`는 30cm 이상 250cm 이하

## 10. Clinical Significance

### 10.1 Common Gait Deviations

| Deviation | Affected Parameter | Typical Pattern |
|-----------|-------------------|-----------------|
| 편마비 (Hemiplegia) | Symmetry | 한쪽 swing 시간 증가 |
| 파킨슨 (Parkinson's) | Stride length | 감소, 동결 현상 |
| 근위축증 (Muscular Dystrophy) | Step width | 증가 (안정성 보상) |
| 관절염 (Arthritis) | Stance time | 통증 측 감소 |

### 10.2 Rehabilitation Goals

| Parameter | Goal | Clinical Meaning |
|-----------|------|------------------|
| Symmetry Index | < 10% | 정상 대칭성 |
| Stance/Swing Ratio | 55-65:35-45 | 정상 비율 회복 |
| Velocity | > 0.8 m/s | 지역사회 보행 능력 |

## 11. Related Specifications

- [JOINT-STATE-SPEC.md](./JOINT-STATE-SPEC.md) - 관절 상태 데이터 명세
- [SESSION-DATA-SPEC.md](./SESSION-DATA-SPEC.md) - 세션 데이터 명세

## 12. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |


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
