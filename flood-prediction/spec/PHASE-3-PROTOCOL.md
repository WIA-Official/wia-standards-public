# WIA-ENE-033 PHASE 3 — Protocol

**Standard:** WIA-ENE-033 Flood Prediction
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-ENE-033-v1.0.md` §6 (예측 알고리즘)
and §7 (조기 경보 시스템)

This document defines the protocols between gauge networks,
forecasting nodes, warning publishers, and downstream consumers.
References WMO standards for the hydrological observation pipeline
itself; protocol schemas reflect the message types defined in those
documents but do not redefine them.

References:
- WMO Hydrological Operational Multipurpose System (HOMS)
- OGC WaterML 2.0 (water observation and forecast information)
- OASIS Common Alerting Protocol (CAP) 1.2
- ISO 22315:2014 (mass evacuation guidance)

---

## 6. 예측 알고리즘

### 6.1 AI 예측 모델

#### 6.1.1 모델 아키텍처

```
입력 레이어:
- 강수량 데이터 (과거 24시간, 미래 예측)
- 하천 수위 데이터 (10개 주요 지점)
- 댐 저수량 및 방류 데이터
- 지형 데이터 (고도, 경사, 토지 이용)
- 배수 시설 현황

히든 레이어:
- LSTM (장단기 메모리): 시계열 패턴 학습
- CNN (합성곱 신경망): 공간 패턴 인식
- Attention 메커니즘: 주요 요인 가중치 부여

출력 레이어:
- 홍수 발생 확률 (0-100%)
- 예상 침수 범위 (GeoJSON)
- 최고 침수 심도 (m)
- 피크 시간 (timestamp)
```

#### 6.1.2 학습 데이터

- **역사적 홍수 데이터**: 과거 30년 이상
- **기상 데이터**: 강수량, 풍속, 기압
- **수문 데이터**: 하천 수위, 유량
- **지형 데이터**: DEM (수치 표고 모델)
- **인프라 데이터**: 배수 시설, 제방

#### 6.1.3 모델 성능 지표

```typescript
interface ModelPerformance {
  accuracy: number;              // 정확도 (%)
  precision: number;             // 정밀도 (%)
  recall: number;                // 재현율 (%)
  f1Score: number;               // F1 점수
  rmse: number;                  // 평균 제곱근 오차
  mae: number;                   // 평균 절대 오차
  leadTime: number;              // 예측 리드 타임 (hours)
  falseAlarmRate: number;        // 허위 경보율 (%)
}

// 목표 성능
const targetPerformance = {
  accuracy: 85,                  // 85% 이상
  precision: 80,                 // 80% 이상
  recall: 90,                    // 90% 이상
  f1Score: 85,                   // 85% 이상
  leadTime: 6,                   // 6시간 이상
  falseAlarmRate: 15             // 15% 이하
};
```

### 6.2 수문 모델링

#### 6.2.1 1D 하천 모델

```
Saint-Venant 방정식:

연속 방정식 (Continuity):
∂A/∂t + ∂Q/∂x = q

운동량 방정식 (Momentum):
∂Q/∂t + ∂(Q²/A)/∂x + gA∂h/∂x + gAS_f = 0

여기서:
A = 단면적 (m²)
Q = 유량 (m³/s)
h = 수위 (m)
g = 중력 가속도 (9.81 m/s²)
S_f = 마찰 경사
q = 측방 유입량 (m³/s/m)
```

#### 6.2.2 2D 침수 모델

```
Shallow Water Equations:

∂h/∂t + ∂(hu)/∂x + ∂(hv)/∂y = R

∂(hu)/∂t + ∂(hu²)/∂x + ∂(huv)/∂y = -gh∂z/∂x - τ_x

∂(hv)/∂t + ∂(huv)/∂x + ∂(hv²)/∂y = -gh∂z/∂y - τ_y

여기서:
h = 수심 (m)
u, v = 유속 성분 (m/s)
z = 지반 고도 (m)
R = 강우 강도 (m/s)
τ_x, τ_y = 바닥 전단 응력
```

### 6.3 앙상블 예측

```typescript
interface EnsemblePrediction {
  // 다중 모델 결과
  models: {
    ai: FloodPredictionResult;
    hydrological: FloodPredictionResult;
    statistical: FloodPredictionResult;
  };

  // 앙상블 결과
  ensemble: {
    meanProbability: number;
    medianProbability: number;
    confidence: number;
    uncertainty: number;
  };

  // 가중 평균 (모델 성능 기반)
  weighted: {
    probability: number;
    inundationArea: number;
    peakTime: string;
  };
}
```

---


## 7. 조기 경보 시스템

### 7.1 경보 발령 기준

```typescript
interface AlertCriteria {
  // 자동 발령 조건
  automatic: {
    riskLevel: RiskLevel;        // LEVEL-2 이상
    probability: number;         // 60% 이상
    leadTime: number;            // 최소 리드 타임 (hours)
    affectedPopulation: number;  // 최소 영향 인구
  };

  // 수동 검토 조건
  manualReview: {
    uncertaintyThreshold: number;  // 불확실성 임계값
    criticalInfrastructure: boolean;
    historicalSignificance: boolean;
  };
}
```

### 7.2 경보 메시지 구조

```typescript
interface FloodAlert {
  alertId: string;
  timestamp: string;
  issuer: {
    organizationId: string;
    name: string;
    authorityLevel: string;
  };

  // 경보 정보
  alert: {
    level: RiskLevel;
    severity: 'minor' | 'moderate' | 'severe' | 'extreme';
    urgency: 'immediate' | 'expected' | 'future';
    certainty: 'observed' | 'likely' | 'possible';
  };

  // 영향 지역
  affectedAreas: {
    regionIds: string[];
    geoJson: GeoJSON;
    population: number;
    buildings: number;
  };

  // 이벤트 정보
  event: {
    type: 'river-flood' | 'urban-flood' | 'coastal-flood' | 'flash-flood';
    onset: string;               // 시작 예상 시각
    expiry: string;              // 종료 예상 시각
    peakTime: string;
  };

  // 권고 사항
  instructions: {
    ko: string;
    en: string;
    actions: string[];
    evacuationRequired: boolean;
    evacuationZones: string[];
  };

  // 대피소 정보
  shelters?: {
    id: string;
    name: string;
    location: Coordinates;
    capacity: number;
    available: number;
    facilities: string[];
  }[];

  // 연락처
  contact: {
    emergency: string;
    information: string;
    website: string;
  };
}
```

### 7.3 경보 전달 채널

```typescript
interface AlertDistribution {
  // 다채널 전송
  channels: {
    sms: boolean;                // 문자 메시지
    push: boolean;               // 앱 푸시 알림
    email: boolean;              // 이메일
    broadcast: boolean;          // 재난 문자 (CBS)
    siren: boolean;              // 경보 사이렌
    radio: boolean;              // 라디오 방송
    tv: boolean;                 // TV 자막
    social: boolean;             // SNS
  };

  // 우선순위 그룹
  recipients: {
    highRisk: string[];          // 고위험 지역 주민
    vulnerable: string[];        // 취약 계층
    firstResponders: string[];   // 재난 대응 인력
    publicOfficials: string[];   // 공무원
    general: string[];           // 일반 주민
  };

  // 전송 상태
  status: {
    sent: number;
    delivered: number;
    failed: number;
    confirmed: number;
  };
}
```

---


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
