# WIA-AIR-POWER PHASE 4 — Integration

**Standard:** WIA-AIR-POWER
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-AIR-POWER-v1.0.md` §5 (Device
Classes), §6 (Integration with WIA Standards), §9 (Safety Certification)

This document defines integration concerns: regulatory crosswalks
(FCC Part 18, ETSI EN 301 489-1, KCC technical regulations), evidence
packaging via CycloneDX 1.5, and signed-attestation submission to a
competent authority. The mapping from this PHASE's object types to
the device classes in §5 of the source spec is defined in Annex G.

References:
- FCC Part 18 (industrial, scientific, and medical equipment)
- ETSI EN 301 489-1 (electromagnetic compatibility)
- IEC 62311:2019 (assessment of electronic and electrical equipment
  related to human exposure)
- CycloneDX 1.5 (SBOM format), Sigstore (DSSE + Rekor)

---

## 5. Device Classes

### 5.1 Power Classes

```yaml
power_classes:
  # Class A: 초저전력 (무배터리 가능)
  class_a:
    name: "Ambient Power"
    power_range: 0 ~ 100mW
    devices: [rfid, sensors, tags]
    battery_required: false
    always_powered: true

  # Class B: 저전력 (웨어러블)
  class_b:
    name: "Wearable Power"
    power_range: 100mW ~ 2W
    devices: [watches, earbuds, rings, glasses]
    battery: small
    charging_time: eliminated

  # Class C: 중전력 (모바일)
  class_c:
    name: "Mobile Power"
    power_range: 2W ~ 15W
    devices: [smartphones, tablets]
    battery: medium
    top_up: continuous

  # Class D: 고전력 (컴퓨팅)
  class_d:
    name: "Computing Power"
    power_range: 15W ~ 100W
    devices: [laptops, monitors]
    battery: large_or_none
    cable_free: finally

  # Class E: 초고전력 (가전)
  class_e:
    name: "Appliance Power"
    power_range: 100W ~ 1kW+
    devices: [tv, vacuum, kitchen]
    vision: "케이블 없는 가전"
    timeline: future
```

### 5.2 Priority System

```yaml
priority_system:
  levels:
    critical:
      description: "생명/안전 관련"
      examples: [medical_devices, emergency_phones]
      guarantee: always_powered
      preemption: can_preempt_others

    high:
      description: "주요 기기"
      examples: [primary_phone, laptop_in_use]
      guarantee: best_effort_high
      preemption: can_preempt_normal

    normal:
      description: "일반 기기"
      examples: [secondary_devices, iot]
      guarantee: fair_share
      preemption: none

    low:
      description: "보조 기기"
      examples: [fully_charged, standby]
      guarantee: when_available
      preemption: yields_to_others
```


## 6. Integration with WIA Standards

### 6.1 WIA-INTENT Integration

```yaml
wia_intent_integration:
  # 의도 기반 전력 요청
  example:
    intent: |
      intent ChargeMy {
        want: phone_charged
        constraints {
          target: 80%
          speed: fast
          cost: free_if_possible
        }
      }

    response:
      found_tx: 3
      selected: "CafeWiFi_TX_01"
      reason: "free, fast, nearby"
      eta_to_80: "12 minutes"
```

### 6.2 WIA-OMNI-API Integration

```yaml
wia_omni_api_integration:
  # API를 통한 전력 관리
  endpoints:
    - intent: "find power sources"
      returns: nearby_transmitters

    - intent: "charge my device"
      action: initiate_charging

    - intent: "power status"
      returns: all_devices_status

    - intent: "optimize power"
      action: rebalance_allocation
```

### 6.3 WIA-LLM-INTEROP Integration

```yaml
wia_llm_integration:
  # AI가 전력 관리
  capabilities:
    - predict_usage: true
    - optimize_distribution: true
    - manage_priorities: true
    - report_anomalies: true

  example:
    ai_action: |
      "사용자가 곧 외출할 것으로 예측됨.
       스마트폰 충전 우선순위 높임.
       노트북은 현재 사용 중이 아니므로 낮춤."
```


## 9. Safety Certification

### 9.1 Regulatory Compliance

```yaml
compliance:
  international:
    - FCC: "미국"
    - CE: "유럽"
    - KC: "한국"
    - TELEC: "일본"
    - CCC: "중국"

  safety_standards:
    - IEC_62311: "인체 전자기장 노출"
    - IEEE_C95.1: "RF 안전"
    - ICNIRP: "비이온화 방사선 가이드라인"

  testing:
    - sar_testing: required
    - thermal_testing: required
    - interference_testing: required
    - long_term_exposure: study_ongoing
```

### 9.2 Health Considerations

```yaml
health:
  # 과학적 근거 기반
  approach: "evidence_based"

  # 보수적 기준 적용
  principle: "precautionary"

  # 지속적 모니터링
  monitoring:
    - population_studies: ongoing
    - incident_reporting: mandatory
    - standard_updates: as_needed

  # 취약 그룹 보호
  vulnerable_protection:
    - children: lower_exposure_limits
    - pregnant: safe_zones_available
    - medical_implants: detection_and_avoidance
```


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
