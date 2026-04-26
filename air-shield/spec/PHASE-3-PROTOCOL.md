# WIA-AIR-SHIELD PHASE 3 — Protocol

**Standard:** WIA-AIR-SHIELD
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-AIR-SHIELD-v1.0.md` §5 (위협 탐지)
and §6 (프라이버시 정책)

This document defines the protocols for threat detection, consent
revocation propagation, and privacy-preserving event aggregation
across air-shield deployments. References ISO/IEC 27001 and
ISO/IEC 29100 controls; protocol schemas reflect those control
groupings without redefining them.

References:
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 29100:2024 (privacy framework)
- ISO/IEC 27559:2022 (privacy-enhancing data de-identification)
- IETF RFC 8446 (TLS 1.3)

---

## 5. 위협 탐지

### 5.1 Threat Levels

```typescript
type ThreatLevel =
  | 'safe'       // 녹색: 위협 없음
  | 'low'        // 파란색: 경미한 이상 징후
  | 'medium'     // 노란색: 주의 필요
  | 'high'       // 주황색: 위협 감지
  | 'critical';  // 빨간색: 즉각 대응 필요
```

### 5.2 Threat Types

```typescript
type ThreatType =
  | 'evil_twin'           // 가짜 AP/송신기
  | 'mitm'                // 중간자 공격
  | 'eavesdropping'       // 도청
  | 'rf_sniffing'         // RF 스니핑
  | 'power_analysis'      // 전력 분석 공격
  | 'timing_attack'       // 타이밍 공격
  | 'replay_attack'       // 재생 공격
  | 'deauth_attack'       // 인증 해제 공격
  | 'rogue_device'        // 불량 기기
  | 'data_exfiltration'   // 데이터 유출 시도
  | 'fingerprinting'      // 기기 식별 시도
  | 'location_tracking';  // 위치 추적 시도
```

### 5.3 Detection Engine

```typescript
interface DetectionEngine {
  // 실시간 스캔
  scan(): ThreatReport;

  // 지속적 모니터링
  startMonitoring(): void;
  stopMonitoring(): void;

  // 딥 스캔 (정밀 검사)
  deepScan(): Promise<DetailedThreatReport>;

  // ML 기반 이상 탐지
  anomalyDetection: {
    train(normalTraffic: TrafficSample[]): void;
    detect(traffic: TrafficSample): AnomalyScore;
  };
}

interface ThreatReport {
  timestamp: Timestamp;
  threatLevel: ThreatLevel;
  threats: Threat[];
  recommendations: string[];
  autoActions: Action[];
}
```

---


## 6. 프라이버시 정책

### 6.1 데이터 최소화

```typescript
interface DataMinimization {
  // 수집하는 데이터
  collected: {
    // 오직 보안을 위해 필요한 것만
    signalStrength: true;      // 신호 강도 (위협 탐지용)
    connectionMetadata: true;   // 연결 메타데이터 (검증용)
    threatSignatures: true;     // 위협 시그니처 (탐지용)
  };

  // 절대 수집하지 않는 데이터
  neverCollected: {
    messageContent: true;       // 메시지 내용
    personalIdentity: true;     // 개인 신원
    location: true;             // 위치 정보
    browsingHistory: true;      // 브라우징 이력
    contacts: true;             // 연락처
  };
}
```

### 6.2 로컬 처리

```typescript
interface LocalProcessing {
  // 모든 처리는 로컬에서
  allProcessingLocal: true;

  // 서버로 전송되는 것
  serverTransmission: {
    threatSignatures: false;    // 위협 시그니처도 로컬
    analytics: false;           // 분석 데이터도 로컬
    telemetry: false;           // 텔레메트리도 없음

    // 유일한 예외: 사용자가 명시적으로 공유 선택 시
    optInSharing: 'user_explicit_consent_required';
  };
}
```

### 6.3 투명성

```typescript
interface Transparency {
  // 실시간 활동 로그
  getActivityLog(): ActivityEntry[];

  // 차단된 위협 이력
  getBlockedThreats(): BlockedThreat[];

  // 현재 보호 상태
  getProtectionStatus(): DetailedStatus;

  // 수집된 데이터 확인
  getCollectedData(): CollectedDataReport;

  // 모든 데이터 삭제
  deleteAllData(): Promise<void>;
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
