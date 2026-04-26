# WIA-ENE-033 PHASE 1 — Data Format Specification

**Standard:** WIA-ENE-033 Flood Prediction
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-ENE-033-v1.0.md` §5 (데이터 모델)

This document defines the canonical data structures for flood
prediction interoperability: gauge identification, river-network
topology, hydrological state envelopes, ensemble-forecast records, and
warning-event schemas. Schemas are expressed in JSON Schema 2020-12
and stable for the lifetime of this PHASE.

References:
- WMO Hydrological Operational Multipurpose System (HOMS)
- OGC WaterML 2.0 (water observation and forecast information)
- ISO 19156:2023 (observations and measurements)
- OpenAPI 3.1, JSON Schema 2020-12

---

## 5. 데이터 모델

### 5.1 홍수 예측 요청

```typescript
interface FloodPredictionRequest {
  // 위치 정보
  location: {
    regionId: string;
    name: string;
    coordinates: {
      latitude: number;
      longitude: number;
    };
    boundary?: GeoJSON;
  };

  // 예측 설정
  timeHorizon: '1h' | '6h' | '24h' | '72h';
  resolution: 'coarse' | 'medium' | 'fine';

  // 데이터 소스 선택
  includePrecipitation: boolean;
  includeRiverLevels: boolean;
  includeDamStatus: boolean;
  includeDrainage: boolean;
  includeTopography: boolean;

  // 시뮬레이션 옵션
  runSimulation: boolean;
  simulationScenarios?: string[];
}
```

### 5.2 홍수 예측 결과

```typescript
interface FloodPredictionResult {
  predictionId: string;
  timestamp: string;
  location: LocationInfo;

  // 위험도 평가
  riskLevel: RiskLevel;
  probability: number;           // 0-100%
  confidence: number;            // 0-100%
  peakTime: string;              // ISO 8601

  // 침수 범위
  inundationArea: {
    estimatedArea: number;       // km²
    maxDepth: number;            // m
    averageDepth: number;        // m
    affectedPopulation: number;
    affectedBuildings: number;
    geoJson: GeoJSON;
  };

  // 기여 요인
  contributingFactors: {
    precipitation: PrecipitationData;
    riverLevels: RiverLevelData[];
    damStatus: DamStatusData[];
    drainage: DrainageData;
    soilSaturation?: number;     // % (토양 포화도)
  };

  // 시나리오 분석
  scenarios?: {
    bestCase: ScenarioResult;
    worstCase: ScenarioResult;
    mostLikely: ScenarioResult;
  };

  // 권고 사항
  recommendations: string[];
  evacuationRequired: boolean;
  affectedAreas: string[];
}
```

### 5.3 강수량 데이터

```typescript
interface PrecipitationData {
  // 관측값
  observed: {
    current: number;             // mm/h (현재 강우 강도)
    last1h: number;              // mm (1시간 누적)
    last3h: number;              // mm (3시간 누적)
    last6h: number;              // mm (6시간 누적)
    last24h: number;             // mm (24시간 누적)
  };

  // 예측값
  forecast: {
    next1h: number;
    next3h: number;
    next6h: number;
    next12h: number;
    next24h: number;
    hourly: number[];            // 시간별 예측 (mm/h)
  };

  // 레이더 데이터
  radar?: {
    imageUrl: string;
    coverage: number;            // km
    lastUpdate: string;
  };

  // 통계
  statistics: {
    returnPeriod: number;        // 년 (재현 주기)
    percentile: number;          // % (백분위수)
    isExtreme: boolean;
  };
}
```

### 5.4 하천 수위 데이터

```typescript
interface RiverLevelData {
  riverId: string;
  name: string;
  stationId: string;
  location: Coordinates;

  // 현재 수위
  current: {
    waterLevel: number;          // m
    flowRate: number;            // m³/s
    velocity: number;            // m/s
    timestamp: string;
  };

  // 기준 수위
  levels: {
    normal: number;              // m (평상시)
    attention: number;           // m (주의)
    warning: number;             // m (경계)
    danger: number;              // m (위험)
    bankHeight: number;          // m (제방 높이)
  };

  // 예측 수위
  forecast: {
    peak: number;                // m (최고 예상 수위)
    peakTime: string;
    hourly: number[];            // 시간별 예측 (m)
  };

  // 변화율
  trend: {
    rateOfRise: number;          // m/h
    direction: 'rising' | 'stable' | 'falling';
    acceleration: number;        // m/h²
  };

  // 상태
  status: {
    condition: 'normal' | 'attention' | 'warning' | 'danger';
    overflowRisk: number;        // 0-100%
    estimatedTimeToOverflow?: number; // hours
  };
}
```

### 5.5 댐/저수지 데이터

```typescript
interface DamStatusData {
  damId: string;
  name: string;
  type: 'multi-purpose' | 'flood-control' | 'hydropower';
  location: Coordinates;

  // 수위 및 용량
  waterLevel: {
    current: number;             // m
    normal: number;              // m (상시 만수위)
    flood: number;               // m (홍수기 제한 수위)
    design: number;              // m (설계 홍수위)
  };

  // 저수량
  storage: {
    current: number;             // 백만 m³
    total: number;               // 백만 m³
    effective: number;           // 백만 m³
    percentage: number;          // %
  };

  // 유입/방류
  flow: {
    inflow: number;              // m³/s (유입량)
    outflow: number;             // m³/s (방류량)
    discharge: number;           // m³/s (여수로 방류)
    powerGeneration: number;     // m³/s (발전 방류)
  };

  // 방류 계획
  dischargePlan: {
    scheduled: boolean;
    startTime?: string;
    duration?: number;           // hours
    maxDischarge?: number;       // m³/s
    affectedArea?: string[];
  };

  // 운영 상태
  operational: {
    gateStatus: 'open' | 'closed' | 'partial';
    gateOpening: number;         // % (수문 개도율)
    emergencyMode: boolean;
  };
}
```

### 5.6 배수 시설 데이터

```typescript
interface DrainageData {
  facilityId: string;
  name: string;
  type: 'pump-station' | 'sewer' | 'detention-basin';
  location: Coordinates;

  // 용량
  capacity: {
    design: number;              // m³/s
    current: number;             // m³/s
    utilization: number;         // %
  };

  // 펌프 상태 (펌프장)
  pumps?: {
    total: number;
    active: number;
    standby: number;
    failed: number;
    efficiency: number;          // %
  };

  // 수위 (저류지)
  waterLevel?: {
    current: number;             // m
    capacity: number;            // m
    available: number;           // m
  };

  // 운영 상태
  operational: {
    status: 'active' | 'standby' | 'maintenance' | 'failure';
    lastMaintenance: string;
    nextMaintenance: string;
    alerts: string[];
  };

  // 성능
  performance: {
    flowRate: number;            // m³/s (실제 배수량)
    energyUsage: number;         // kWh
    reliability: number;         // % (가동률)
  };
}
```

---


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
