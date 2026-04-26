# WIA-ENE-032 PHASE 2 — API Interface Specification

**Standard:** WIA-ENE-032
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. 데이터 모델

### 5.1 산불 감지 이벤트

```json
{
  "eventId": "FIRE-2025-KR-001234",
  "timestamp": "2025-04-15T14:23:00Z",
  "detectionMethod": "VIIRS",
  "confidenceLevel": 95,

  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "elevation": 450,
    "address": "강원도 속초시 설악산",
    "forestType": "침엽수림",
    "administrativeArea": "강원도"
  },

  "fireCharacteristics": {
    "frp": 125.5,
    "brightness": 345.2,
    "area": 1500,
    "perimeter": 450,
    "fireLineIntensity": 3500,
    "rateOfSpread": 2.5
  },

  "weatherConditions": {
    "temperature": 28.5,
    "humidity": 25,
    "windSpeed": 12.5,
    "windDirection": 225,
    "precipitation24h": 0
  },

  "riskAssessment": {
    "dangerLevel": 4,
    "fwi": 35.2,
    "threatToLife": "high",
    "threatToProperty": "high",
    "evacuationRequired": true
  },

  "metadata": {
    "satellite": "NOAA-20",
    "sensor": "VIIRS-I4",
    "resolution": 375,
    "quality": "high",
    "validated": true
  }
}
```

### 5.2 화재 확산 예측

```json
{
  "predictionId": "PRED-2025-001234",
  "fireEventId": "FIRE-2025-KR-001234",
  "timestamp": "2025-04-15T14:30:00Z",
  "forecastHorizon": 24,

  "spreadPrediction": {
    "currentArea": 1500,
    "predicted6h": 5000,
    "predicted12h": 12000,
    "predicted24h": 30000,
    "confidence": 85
  },

  "firePerimeter": {
    "type": "Polygon",
    "coordinates": [
      [[126.978, 37.566], [126.982, 37.568], [126.980, 37.564], [126.978, 37.566]]
    ]
  },

  "threatenedAssets": [
    {
      "type": "residential",
      "name": "속초시 외곽 주택가",
      "population": 1500,
      "distance": 2.5,
      "arrivalTime": "2025-04-15T18:00:00Z",
      "priority": "critical"
    },
    {
      "type": "infrastructure",
      "name": "고압 송전선",
      "distance": 1.2,
      "arrivalTime": "2025-04-15T16:30:00Z",
      "priority": "high"
    }
  ],

  "model": {
    "name": "FlamMap-FARSITE",
    "version": "5.4",
    "accuracy": 82.5
  }
}
```

---

## 6. 위성 영상 감지

### 6.1 MODIS 위성

**사양:**
- **위성**: Terra (오전 통과), Aqua (오후 통과)
- **공간 해상도**: 1km (열적외선)
- **시간 해상도**: 하루 4회 관측
- **열 밴드**: Band 21 (3.96 μm), Band 31 (11.03 μm)
- **탐지 임계값**: 밝기 온도 > 310K

**데이터 포맷:**

```json
{
  "satellite": "Terra-MODIS",
  "acquisitionTime": "2025-04-15T02:30:00Z",
  "tileId": "h28v05",
  "hotspots": [
    {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "brightness": 345.2,
      "frp": 125.5,
      "confidence": 95,
      "dayNight": "D"
    }
  ]
}
```

### 6.2 VIIRS 위성

**사양:**
- **위성**: Suomi NPP, NOAA-20, NOAA-21
- **공간 해상도**: 375m (I-band), 750m (M-band)
- **시간 해상도**: 하루 2-4회 관측
- **열 밴드**: I4 (3.74 μm), I5 (11.45 μm)
- **탐지 임계값**: 밝기 온도 > 325K

**장점:**
- MODIS 대비 4배 높은 공간 해상도
- 소형 화재 감지 능력 우수 (면적 50m² 이상)
- 야간 감지 성능 향상

### 6.3 Sentinel 위성

**사양:**
- **위성**: Sentinel-2 (광학), Sentinel-3 (열적외선)
- **공간 해상도**: 10-60m (Sentinel-2), 1km (Sentinel-3)
- **시간 해상도**: 5일 (Sentinel-2), 1일 (Sentinel-3)
- **활용**: 화재 피해 면적 평가, 식생 지수 모니터링

---

## 7. 열 감지 시스템

### 7.1 지상 열화상 카메라

**사양:**
- **탐지 범위**: 5-15km (기종별 상이)
- **회전 속도**: 360° / 60초
- **온도 감지 범위**: -40°C ~ 500°C
- **해상도**: 640×480 이상
- **프레임률**: 30fps
- **오탐지 필터**: AI 기반 동물/차량 제외

**설치 기준:**
- 산정상 또는 전망대 (가시거리 최대화)
- 10-15km 간격으로 배치
- 전원 공급 (태양광 패널 + 배터리)
- 통신: LTE/5G 또는 위성통신

### 7.2 드론 열화상 감지

**운용 시나리오:**
- 산불 초기 정밀 위치 확인
- 연기로 인한 가시성 부족 시 투입
- 잔불 수색 (진화 후)
- 인명 구조 지원 (열 신호로 실종자 탐색)

**드론 사양:**
- **비행 시간**: 30분 이상
- **탑재 센서**: 열화상 + 가시광 카메라
- **전송**: 실시간 영상 스트리밍 (5G)
- **자율 비행**: GPS + 장애물 회피

---


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
