# WIA-AUG-001 PHASE 2 — API Interface Specification

**Standard:** WIA-AUG-001
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. Baseline Registry System

### 5.1 Baseline Measurement Protocol

Human capability baselines must be measured before augmentation:

```typescript
interface BaselineMeasurement {
  subjectId: string;
  timestamp: Date;
  measurements: {
    physical: PhysicalMetrics;
    sensory: SensoryMetrics;
    cognitive: CognitiveMetrics;
    neural: NeuralMetrics;
  };
  conditions: MeasurementConditions;
}

interface PhysicalMetrics {
  strength: { value: number; unit: string; test: string };
  speed: { value: number; unit: string; test: string };
  endurance: { value: number; unit: string; test: string };
  dexterity: { value: number; unit: string; test: string };
}
```

### 5.2 Standard Test Protocols

#### 5.2.1 Physical Baseline Tests
- **Strength**: 1-rep max, grip strength, isometric force
- **Speed**: 100m sprint, reaction time, movement velocity
- **Endurance**: VO2 max, time to exhaustion, sustained work capacity
- **Dexterity**: Purdue Pegboard, 9-hole peg test, fine motor tasks

#### 5.2.2 Sensory Baseline Tests
- **Visual**: Snellen chart, color perception, contrast sensitivity
- **Auditory**: Pure tone audiometry, speech recognition threshold
- **Tactile**: Two-point discrimination, vibration threshold
- **Proprioception**: Balance tests, position sense

#### 5.2.3 Cognitive Baseline Tests
- **Memory**: Digit span, word recall, working memory capacity
- **Processing Speed**: Reaction time, symbol coding, trail making
- **Pattern Recognition**: Visual search, change detection
- **Decision Making**: Iowa Gambling Task, response selection

### 5.3 Baseline Registry Structure

```json
{
  "registryId": "BR-2025-001234",
  "subjectId": "SUB-123456",
  "registrationDate": "2025-12-26T10:00:00Z",
  "baseline": {
    "physical": {
      "strength_max_kg": 100,
      "speed_100m_sec": 12.5,
      "endurance_vo2max": 45.0
    },
    "sensory": {
      "visual_acuity": 1.0,
      "auditory_range_hz": [20, 20000],
      "tactile_sensitivity_mm": 2.0
    },
    "cognitive": {
      "memory_digit_span": 7,
      "processing_speed_ms": 250,
      "pattern_recognition_accuracy": 0.85
    }
  },
  "populationPercentile": {
    "strength": 50,
    "speed": 60,
    "visual_acuity": 70
  }
}
```

### 5.4 Baseline Update Protocol

```
Baseline Re-measurement Requirements:
- Initial: Before any augmentation
- Annual: For monitoring natural changes
- Pre-upgrade: Before augmentation modification
- Post-removal: After augmentation removal
- Incident: After adverse events
```

---

## 6. Enhancement Ratio Calculation

### 6.1 Basic Enhancement Ratio

```
ER = P_aug / P_base

where:
- ER = Enhancement Ratio
- P_aug = Augmented performance metric
- P_base = Baseline performance metric
```

### 6.2 Normalized Enhancement Ratio

For metrics where higher values aren't always better (e.g., reaction time):

```
NER = |P_optimal - P_base| / |P_optimal - P_aug|

where:
- P_optimal = Optimal performance value
- Inversion applied for "lower is better" metrics
```

### 6.3 Time-Adjusted Enhancement

For capabilities with learning curves:

```
TAER = (P_aug(t) - P_base) / (P_plateau - P_base)

where:
- P_aug(t) = Performance at time t
- P_plateau = Expected plateau performance
- Accounts for adaptation period
```

### 6.4 Multi-Domain Enhancement Score

```
MDES = Σ(ER_i × Impact_i × Weight_i) / Σ(Weight_i)

where:
- ER_i = Enhancement ratio for domain i
- Impact_i = Real-world impact factor (0-1)
- Weight_i = Application priority weight
```

### 6.5 Enhancement Ratio Examples

```
Physical Strength Enhancement:
- Baseline: 100 kg lifting capacity
- Augmented: 350 kg lifting capacity
- ER = 350/100 = 3.5x (MODERATE)

Visual Acuity Enhancement:
- Baseline: 20/20 vision (1.0 decimal)
- Augmented: 20/5 vision (4.0 decimal)
- ER = 4.0/1.0 = 4.0x (MODERATE)

Reaction Time Enhancement (inverse):
- Baseline: 250 ms
- Augmented: 50 ms
- Optimal: 0 ms
- NER = |0-250|/|0-50| = 5.0x (SIGNIFICANT)
```

---

## 7. Compatibility Assessment

### 7.1 Compatibility Dimensions

Three primary dimensions determine compatibility:

| Dimension | Weight | Criteria |
|-----------|--------|----------|
| Technical Interface | 0.40 | Physical/electrical/data compatibility |
| Safety Interaction | 0.35 | Risk of adverse interactions |
| Performance Synergy | 0.25 | Cooperative vs. conflicting enhancement |

### 7.2 Compatibility Score Formula

```
CS = (TI × 0.40) + (SI × 0.35) + (PS × 0.25)

where:
- CS = Compatibility Score (0-1)
- TI = Technical Interface score (0-1)
- SI = Safety Interaction score (0-1)
- PS = Performance Synergy score (0-1)

Classification:
- CS ≥ 0.80: Highly Compatible
- CS 0.60-0.79: Compatible
- CS 0.40-0.59: Conditionally Compatible
- CS < 0.40: Incompatible
```

### 7.3 Technical Interface Assessment

```typescript
interface TechnicalInterface {
  powerCompatibility: number;      // 0-1
  communicationProtocol: number;   // 0-1
  physicalInterference: number;    // 0-1
  dataFormatAlignment: number;     // 0-1
}

function assessTechnicalInterface(aug1: Augmentation, aug2: Augmentation): number {
  const power = checkPowerCompatibility(aug1.power, aug2.power);
  const comm = checkCommunication(aug1.protocol, aug2.protocol);
  const physical = checkPhysicalInterference(aug1.location, aug2.location);
  const data = checkDataAlignment(aug1.dataFormat, aug2.dataFormat);

  return (power + comm + physical + data) / 4;
}
```

### 7.4 Safety Interaction Assessment

```typescript
interface SafetyInteraction {
  biologicalConflict: number;      // 0-1 (1 = no conflict)
  electricalInterference: number;  // 0-1
  thermalInteraction: number;      // 0-1
  mechanicalStress: number;        // 0-1
}

function assessSafetyInteraction(aug1: Augmentation, aug2: Augmentation): number {
  // Check for biological conflicts (immune response, tissue stress)
  const bioConflict = checkBiologicalConflict(aug1, aug2);

  // Check electrical interference
  const elecInterf = checkElectricalInterference(aug1.signals, aug2.signals);

  // Check thermal load
  const thermal = checkThermalLoad(aug1.heat, aug2.heat);

  // Check mechanical stress


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
