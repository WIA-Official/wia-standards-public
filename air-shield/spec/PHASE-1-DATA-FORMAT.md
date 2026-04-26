# WIA-AIR-SHIELD PHASE 1 — Data Format Specification

**Standard:** WIA-AIR-SHIELD (privacy-respecting threat detection)
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-AIR-SHIELD-v1.0.md` §3 (기술 명세)

This document defines the canonical data structures for air-shield
deployments: device identification, threat-event envelopes,
privacy-preserving telemetry payloads, and consent ledger entries.
Schemas are expressed in JSON Schema 2020-12 and stable for the
lifetime of this PHASE.

References:
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27018:2019 (PII protection in public clouds)
- ISO/IEC 29100:2024 (privacy framework)
- W3C DID 1.0 (decentralized identifiers)
- OpenAPI 3.1, JSON Schema 2020-12

---

## 3. 기술 명세

### 3.1 암호화 계층

#### 3.1.1 Post-Quantum Encryption
```typescript
interface PQEncryption {
  algorithm: 'CRYSTALS-Kyber' | 'NTRU' | 'SABER' | 'Classic-McEliece';
  keySize: 512 | 768 | 1024;

  // WIA-PQ-CRYPTO 연동
  wia_pq_crypto_compatible: true;
}
```

#### 3.1.2 다중 암호화 터널
```
┌──────────────────────────────────────────┐
│  Layer 3: Application Encryption         │
│  ┌────────────────────────────────────┐  │
│  │  Layer 2: Transport Encryption     │  │
│  │  ┌──────────────────────────────┐  │  │
│  │  │  Layer 1: Physical Encryption │  │  │
│  │  │  (RF-level scrambling)        │  │  │
│  │  └──────────────────────────────┘  │  │
│  └────────────────────────────────────┘  │
└──────────────────────────────────────────┘
```

### 3.2 Zero-Knowledge Protocol

```typescript
interface ZeroKnowledgeAuth {
  // 비밀번호를 알려주지 않고 인증
  proveIdentity(secret: never): ZKProof;

  // 위치를 알려주지 않고 범위 증명
  proveLocationInRange(range: GeoRange): ZKProof;

  // 나이를 알려주지 않고 성인 증명
  proveAgeOver(threshold: number): ZKProof;

  // 잔액을 알려주지 않고 충분함 증명
  proveBalanceSufficient(amount: number): ZKProof;
}
```

### 3.3 Anti-Fingerprinting

#### 3.3.1 Device Fingerprint Protection
```typescript
interface AntiFingerprintConfig {
  // MAC 주소 랜덤화
  randomizeMAC: boolean;

  // 기기 특성 은폐
  hideDeviceCharacteristics: boolean;

  // 브라우저/앱 지문 교란
  spoofUserAgent: boolean;

  // 하드웨어 ID 가상화
  virtualizeHardwareId: boolean;

  // 타이밍 지문 교란
  randomizeTimingSignature: boolean;
}
```

#### 3.3.2 Traffic Pattern Obfuscation
```typescript
interface TrafficObfuscation {
  // 패킷 크기 패딩
  padPacketSize: boolean;
  paddingStrategy: 'fixed' | 'random' | 'mtf'; // MTF = Morphing Traffic Flow

  // 전송 타이밍 교란
  randomizeTransmitTiming: boolean;
  timingJitter: Milliseconds;

  // 디코이 트래픽
  decoyTraffic: {
    enabled: boolean;
    percentage: Percentage; // 진짜 대비 가짜 비율
    pattern: 'constant' | 'bursty' | 'mimicry';
  };
}
```

### 3.4 Evil Twin Detection

```typescript
interface EvilTwinDetector {
  // AP/송신기 검증
  verifyAuthenticSource(source: WirelessSource): Promise<VerificationResult>;

  // 신호 특성 분석
  analyzeSignalCharacteristics(signal: SignalInfo): AnomalyScore;

  // 이력 기반 검증
  compareWithHistory(source: WirelessSource): ConsistencyScore;

  // 크라우드소싱 검증
  crowdVerify(source: WirelessSource): Promise<CommunityTrustScore>;
}

interface VerificationResult {
  authentic: boolean;
  confidence: Percentage;
  warnings: Warning[];
  recommendation: 'connect' | 'avoid' | 'proceed_with_caution';
}
```

### 3.5 Side-Channel Protection

#### 3.5.1 Power Analysis Defense
```typescript
interface PowerAnalysisDefense {
  // 일정한 전력 소비 패턴
  constantPowerConsumption: boolean;

  // 노이즈 주입
  powerNoiseInjection: boolean;
  noiseLevel: 'low' | 'medium' | 'high';

  // 연산 마스킹
  operationMasking: boolean;
}
```

#### 3.5.2 Timing Attack Defense
```typescript
interface TimingDefense {
  // 일정 시간 연산
  constantTimeOperations: boolean;

  // 랜덤 지연 삽입
  randomDelayInjection: boolean;
  delayRange: [Milliseconds, Milliseconds];

  // 더미 연산
  dummyOperations: boolean;
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
