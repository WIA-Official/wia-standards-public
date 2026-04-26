# WIA-AUG-005 PHASE 2 — API Interface Specification

**Standard:** WIA-AUG-005
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 4. Performance Metrics

### 4.1 Baseline Metrics

Essential baseline measurements:

#### 4.1.1 IQ Score (General Intelligence)
```
Measurement: WAIS-IV, Raven's Progressive Matrices
Typical Range: 85-115 (mean = 100, SD = 15)
Enhancement Range: +5 to +20 points (0.05-0.20 SD)
Reassessment: Every 6-12 months
```

#### 4.1.2 Working Memory Capacity
```
Measurement: Digit Span, Operation Span, N-back
Typical Capacity: 7±2 items
Enhancement Range: +2 to +4 items
Reassessment: Monthly
```

#### 4.1.3 Processing Speed
```
Measurement: Symbol Digit Modalities Test, Trail Making Test
Typical Speed: 1.5-2.5 items per second
Enhancement Range: +20% to +50%
Reassessment: Bi-weekly
```

#### 4.1.4 Attention Span
```
Measurement: Continuous Performance Test
Typical Duration: 20-45 minutes
Enhancement Range: +30% to +80%
Reassessment: Weekly
```

#### 4.1.5 Creativity Index
```
Measurement: Torrance Tests, Alternative Uses Task
Typical Fluency: 8-15 ideas per prompt
Enhancement Range: +20% to +60%
Reassessment: Monthly
```

### 4.2 Enhancement Metrics

#### 4.2.1 Enhancement Ratio

```
Enhancement Ratio (ER) = (Current Performance - Baseline) / Baseline

Categories:
- Minimal: 0.0 - 0.1 (0-10%)
- Low: 0.1 - 0.2 (10-20%)
- Moderate: 0.2 - 0.4 (20-40%)
- High: 0.4 - 0.6 (40-60%)
- Very High: 0.6 - 0.8 (60-80%)
- Extreme: > 0.8 (>80%) [Caution required]
```

#### 4.2.2 Sustainability Index

```
Sustainability Index (SI) = ER at time T / ER at peak

Interpretation:
- SI > 0.9: Excellent sustainability
- SI 0.7-0.9: Good sustainability
- SI 0.5-0.7: Moderate decay
- SI < 0.5: Poor sustainability, intervention needed
```

#### 4.2.3 Transfer Efficiency

```
Transfer Efficiency = Performance improvement on untrained tasks /
                      Performance improvement on trained tasks

Levels:
- Near Transfer: TE > 0.7 (tasks similar to training)
- Moderate Transfer: TE 0.3-0.7 (related domains)
- Far Transfer: TE < 0.3 (distant domains)
```

### 4.3 Real-time Performance Indicators

```typescript
interface PerformanceIndicators {
  // Primary metrics
  accuracyRate: number;        // 0.0-1.0
  responseTime: number;        // milliseconds
  taskCompletionRate: number;  // 0.0-1.0

  // Secondary metrics
  errorRate: number;           // 0.0-1.0
  perseverationIndex: number;  // repetitive errors
  noveltyScore: number;        // creative/novel responses

  // Physiological correlates
  heartRateVariability: number; // HRV as stress indicator
  eyeTrackingMetrics: {
    fixationDuration: number;
    saccadeVelocity: number;
    blinkRate: number;
  };

  // Subjective metrics
  perceivedDifficulty: number;  // 1-10 scale
  perceivedPerformance: number; // 1-10 scale
  mentalEffort: number;         // 1-10 scale
}
```

---

## 5. Safety Thresholds

### 5.1 Enhancement Limits

#### 5.1.1 Maximum Enhancement Ratios by Domain

```
MEMORY:     0.6 (60% maximum)
ATTENTION:  0.8 (80% maximum)
REASONING:  0.5 (50% maximum)
CREATIVITY: 0.7 (70% maximum)
LANGUAGE:   0.5 (50% maximum)
EXECUTIVE:  0.6 (60% maximum)
SPATIAL:    0.5 (50% maximum)
```

**Rationale**: Higher enhancements may lead to cognitive imbalance, overload, or adverse neuroplastic changes.

#### 5.1.2 Rate of Enhancement

```
Maximum Rate = 0.1 ER per week for sustained enhancement
Maximum Rate = 0.3 ER per session for temporary enhancement

Example:
  Safe: Baseline IQ 100 → 110 over 4 weeks
  Unsafe: Baseline IQ 100 → 120 in 1 week
```

### 5.2 Cognitive Load Thresholds

```
Cognitive Load Index (CLI) = Σ(Task_Demand × Attention_Allocation) /
                             Available_Cognitive_Resources

Safe Range: 0.3 - 0.7
Warning Threshold: CLI > 0.7
Critical Threshold: CLI > 0.9
Emergency Threshold: CLI > 0.95

Actions:
- CLI > 0.7: Suggest break, reduce task complexity
- CLI > 0.9: Mandatory break, reduce enhancement level
- CLI > 0.95: Emergency shutdown, medical consultation
```

### 5.3 Fatigue Detection

```typescript
interface FatigueIndicators {
  // Performance-based
  performanceDecline: number;    // % decrease from baseline
  errorRateIncrease: number;     // % increase in errors
  responseTimeIncrease: number;  // % increase in RT

  // Physiological
  heartRateElevation: number;    // bpm above baseline
  cortisol: number;              // stress hormone level
  pupilDilation: number;         // cognitive effort indicator

  // Subjective
  selfReportedFatigue: number;   // 1-10 scale
  motivationLevel: number;       // 1-10 scale

  // Composite fatigue score
  fatigueScore: number;          // 0-100
}

Thresholds:
- Fatigue Score < 30: Normal, continue
- Fatigue Score 30-60: Monitor, suggest breaks
- Fatigue Score 60-80: Mandatory break (15-30 min)
- Fatigue Score > 80: End session, rest required
```

### 5.4 Long-term Safety Monitoring

Required assessments:

```
Daily:
  - Cognitive load monitoring
  - Fatigue assessment
  - Performance metrics

Weekly:
  - Domain-specific performance tests
  - Subjective well-being survey
  - Sleep quality assessment

Monthly:
  - Comprehensive cognitive battery
  - Baseline reassessment
  - Enhancement ratio recalculation
  - Medical check-in (if pharmacological)

Quarterly:
  - Neuropsychological evaluation
  - MRI/EEG (if electrical methods)
  - Long-term effects assessment

Annually:
  - Full medical and neurological examination
  - Comprehensive cognitive assessment
  - Ethics and quality of life review
```

### 5.5 Contraindications and Warnings

#### Absolute Contraindications:
```
- Active psychosis or severe mental illness
- Epilepsy (for electrical methods)
- Brain tumors or lesions
- Recent stroke or TBI
- Pregnancy (for pharmacological/electrical)
```

#### Relative Contraindications:
```
- Cardiovascular disease (pharmacological)
- Sleep disorders (may exacerbate)
- Substance use disorders (addiction risk)
- Age < 18 or > 75 (limited evidence)
- Concurrent psychotropic medications
```

#### Warning Signs:
```
- Persistent headaches
- Sleep disturbances (insomnia or hypersomnia)
- Mood changes (anxiety, depression, irritability)
- Cognitive rigidity or perseveration
- Physical symptoms (tremor, GI distress, etc.)
- Withdrawal symptoms when not enhancing
```

---

## 6. Cognitive Fatigue Management

### 6.1 Fatigue Mechanisms

```
Primary Mechanisms:
1. Neurotransmitter Depletion
   - Dopamine depletion in prefrontal cortex
   - Serotonin reduction affecting mood and motivation
   - Acetylcholine depletion impairing attention and memory

2. Metabolic Exhaustion
   - Glucose depletion in active brain regions
   - Accumulation of metabolic waste (adenosine)
   - Oxidative stress and free radical damage

3. Attentional Resource Depletion
   - Limited capacity for sustained attention
   - Ego depletion (willpower exhaustion)
   - Motivational fatigue

4. Neuroplastic Stress
   - Excessive synaptic activity
   - Calcium dysregulation
   - Protein synthesis demands
```

### 6.2 Fatigue Prevention Protocol

```
Time-based Management:
- 25-50 minute work intervals (Pomodoro technique)
- 5-10 minute breaks between intervals
- 15-30 minute breaks every 2 hours
- Maximum 4-6 hours of enhanced cognition per day

Task-based Management:
- Alternate between high and low cognitive load tasks
- Interleave different cognitive domains
- Schedule creative tasks after analytical tasks
- End sessions with low-demand activities

Recovery Optimization:
- 7-9 hours of sleep nightly
- Naps (20-30 min) for acute recovery
- Physical exercise (aerobic: 30+ min, 3-5× weekly)
- Mindfulness/meditation (10-20 min daily)
- Adequate nutrition and hydration
```

### 6.3 Recovery Protocols

#### Acute Recovery (5-30 minutes):
```
- Disconnect from enhancement systems
- Physical movement (walk, stretch)
- Mindful breathing exercises
- Hydration and light snacking
- Sensory reset (nature exposure, music)
```

#### Extended Recovery (1-4 hours):
```
- Complete cessation of cognitive enhancement
- Sleep or rest
- Low cognitive load activities
- Social interaction
- Physical exercise
```

#### Long-term Recovery (1-7 days):
```
- Enhancement-free periods
- Vacation from cognitively demanding work
- Baseline reassessment
- Medical consultation if needed
```

### 6.4 Fatigue Monitoring Algorithm

```typescript
function monitorCognitiveFatigue(
  currentMetrics: PerformanceIndicators,
  baseline: PerformanceIndicators,
  sessionDuration: number
): FatigueAssessment {

  // Calculate performance decline
  const accuracyDecline =
    (baseline.accuracyRate - currentMetrics.accuracyRate) / baseline.accuracyRate;
  const rtIncrease =
    (currentMetrics.responseTime - baseline.responseTime) / baseline.responseTime;
  const errorIncrease =
    (currentMetrics.errorRate - baseline.errorRate) / baseline.errorRate;

  // Calculate fatigue score
  const performanceFatigue = (accuracyDecline + rtIncrease + errorIncrease) / 3;
  const temporalFatigue = sessionDuration / 240; // 4 hours = 1.0
  const subjectiveFatigue = currentMetrics.perceivedDifficulty / 10;

  const fatigueScore =
    performanceFatigue * 0.5 +
    temporalFatigue * 0.3 +
    subjectiveFatigue * 0.2;

  // Determine action
  if (fatigueScore > 0.8) {
    return {
      level: 'CRITICAL',
      action: 'END_SESSION',
      recommendedBreak: 240 // 4 hours
    };
  } else if (fatigueScore > 0.6) {
    return {
      level: 'HIGH',
      action: 'MANDATORY_BREAK',
      recommendedBreak: 30 // 30 minutes
    };
  } else if (fatigueScore > 0.3) {
    return {
      level: 'MODERATE',
      action: 'SUGGEST_BREAK',
      recommendedBreak: 10 // 10 minutes
    };
  } else {
    return {
      level: 'LOW',
      action: 'CONTINUE',
      recommendedBreak: 0
    };
  }
}
```

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
