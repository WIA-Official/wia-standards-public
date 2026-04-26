# WIA Exoskeleton Joint State Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14

## 1. Overview

이 문서는 재활 외골격 시스템에서 관절 상태 데이터를 표현하기 위한 표준 형식을 정의합니다.
관절 상태 데이터는 외골격의 각 관절에서 실시간으로 수집되는 운동학적(kinematic) 및
운동역학적(kinetic) 정보를 포함합니다.

## 2. Data Structure

### 2.1 JointState

관절의 현재 상태를 나타내는 기본 데이터 구조입니다.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `joint` | JointType | Yes | 관절 유형 (hip, knee, ankle) |
| `side` | string | Yes | 좌/우 구분 ('left' \| 'right') |
| `timestamp` | number | Yes | Unix timestamp (milliseconds) |
| `kinematics` | Kinematics | Yes | 운동학적 데이터 |
| `kinetics` | Kinetics | Yes | 운동역학적 데이터 |
| `sensors` | SensorData | Yes | 센서 원시 데이터 |

### 2.2 JointType Enumeration

지원되는 관절 유형:

| Value | Description | Range of Motion |
|-------|-------------|-----------------|
| `hip` | 고관절 (Hip) | Flexion: 0-120°, Extension: 0-30° |
| `knee` | 슬관절 (Knee) | Flexion: 0-135°, Extension: 0° |
| `ankle` | 족관절 (Ankle) | Dorsiflexion: 0-20°, Plantarflexion: 0-50° |

## 3. Kinematics Data

운동학적 데이터는 관절의 위치, 속도, 가속도 정보를 포함합니다.

### 3.1 Fields

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `angle` | number | degrees | 관절 각도 (해부학적 기준) |
| `angularVelocity` | number | deg/s | 각속도 |
| `angularAcceleration` | number | deg/s² | 각가속도 |

### 3.2 Coordinate System

- **해부학적 기준위치 (Anatomical Reference Position)**: 직립 자세에서 모든 관절은 0°
- **Flexion (굴곡)**: 양수 값 (+)
- **Extension (신전)**: 음수 값 (-)

### 3.3 Example

```json
{
  "angle": 45.2,
  "angularVelocity": 120.5,
  "angularAcceleration": 15.3
}
```

## 4. Kinetics Data

운동역학적 데이터는 관절에 작용하는 힘(토크) 정보를 포함합니다.

### 4.1 Fields

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `torque` | number | Nm | 사용자가 생성한 토크 |
| `assistTorque` | number | Nm | 외골격이 제공하는 보조 토크 |
| `netTorque` | number | Nm | 총 순 토크 (torque + assistTorque) |

### 4.2 Sign Convention

- **양수 (+)**: Flexion 방향 토크
- **음수 (-)**: Extension 방향 토크

### 4.3 Example

```json
{
  "torque": 12.5,
  "assistTorque": 8.3,
  "netTorque": 20.8
}
```

## 5. Sensor Data

원시 센서 데이터를 포함합니다.

### 5.1 Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `encoder` | number | Yes | 인코더 raw counts |
| `imu` | IMUData | No | IMU 센서 데이터 |
| `forceplate` | ForceData | No | 힘판 데이터 |

### 5.2 IMU Data Structure

관성측정장치(IMU) 데이터:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `accelerometer` | Vector3D | m/s² | 3축 가속도 |
| `gyroscope` | Vector3D | deg/s | 3축 각속도 |
| `magnetometer` | Vector3D | μT | 3축 자기장 (optional) |

### 5.3 Vector3D Structure

```json
{
  "x": 0.0,
  "y": 0.0,
  "z": -9.81
}
```

### 5.4 Force Data Structure

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `fx` | number | N | X축 방향 힘 |
| `fy` | number | N | Y축 방향 힘 |
| `fz` | number | N | Z축 방향 힘 (수직 지면 반력) |
| `mx` | number | Nm | X축 모멘트 |
| `my` | number | Nm | Y축 모멘트 |
| `mz` | number | Nm | Z축 모멘트 |
| `cop` | Vector2D | m | 압력 중심 (Center of Pressure) |

## 6. Complete Example

```json
{
  "joint": "knee",
  "side": "right",
  "timestamp": 1702598400000,
  "kinematics": {
    "angle": 45.2,
    "angularVelocity": 120.5,
    "angularAcceleration": 15.3
  },
  "kinetics": {
    "torque": 12.5,
    "assistTorque": 8.3,
    "netTorque": 20.8
  },
  "sensors": {
    "encoder": 32768,
    "imu": {
      "accelerometer": { "x": 0.15, "y": -0.08, "z": -9.78 },
      "gyroscope": { "x": 5.2, "y": -2.1, "z": 0.3 }
    }
  }
}
```

## 7. Sampling Requirements

| Parameter | Minimum | Recommended | Maximum |
|-----------|---------|-------------|---------|
| Sample Rate | 100 Hz | 200 Hz | 1000 Hz |
| Timestamp Resolution | 1 ms | 1 ms | 0.1 ms |
| Angle Resolution | 0.1° | 0.01° | 0.001° |
| Torque Resolution | 0.1 Nm | 0.01 Nm | 0.001 Nm |

## 8. Validation Rules

1. `joint`는 반드시 'hip', 'knee', 'ankle' 중 하나여야 합니다.
2. `side`는 반드시 'left' 또는 'right'여야 합니다.
3. `timestamp`는 양수 정수여야 합니다.
4. `angle`은 관절별 물리적 가동범위 내에 있어야 합니다.
5. `netTorque`는 `torque + assistTorque`와 같아야 합니다.

## 9. Related Specifications

- [GAIT-CYCLE-SPEC.md](./GAIT-CYCLE-SPEC.md) - 보행 주기 데이터 명세
- [SESSION-DATA-SPEC.md](./SESSION-DATA-SPEC.md) - 세션 데이터 명세

## 10. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |


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
