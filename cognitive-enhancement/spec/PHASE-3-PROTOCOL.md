# WIA-AUG-005 PHASE 3 — Protocol Specification

**Standard:** WIA-AUG-005
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 7. Baseline Assessment Protocols

### 7.1 Comprehensive Baseline Battery

Minimum required assessments:

```
1. General Intelligence:
   - WAIS-IV (Wechsler Adult Intelligence Scale)
   - Raven's Progressive Matrices
   Duration: 90-120 minutes

2. Memory:
   - WMS-IV (Wechsler Memory Scale)
   - Rey Auditory Verbal Learning Test
   - Digit Span (forward/backward)
   Duration: 60 minutes

3. Attention:
   - Continuous Performance Test (CPT)
   - Trail Making Test (A & B)
   - Stroop Test
   Duration: 30 minutes

4. Executive Function:
   - Wisconsin Card Sorting Test
   - Tower of Hanoi/London
   - Verbal Fluency Tests
   Duration: 45 minutes

5. Language:
   - Boston Naming Test
   - Token Test
   - Reading comprehension assessment
   Duration: 30 minutes

6. Spatial:
   - Mental Rotation Test
   - Block Design
   - Rey-Osterrieth Complex Figure
   Duration: 30 minutes

7. Creativity:
   - Torrance Tests of Creative Thinking
   - Alternative Uses Task
   - Remote Associates Test
   Duration: 45 minutes

Total Duration: 5-6 hours (can be split across sessions)
```

### 7.2 Rapid Baseline Assessment

For time-constrained scenarios:

```
Duration: 60 minutes total

1. MoCA (Montreal Cognitive Assessment) - 10 min
   - Screens multiple domains quickly

2. Digit Span - 5 min
   - Working memory assessment

3. Trail Making Test A & B - 10 min
   - Processing speed and executive function

4. N-back (2-back, 3-back) - 10 min
   - Working memory and attention

5. Pattern Recognition Test - 10 min
   - Reasoning and processing speed

6. Alternative Uses Task - 10 min
   - Creativity assessment

7. Spatial Memory Test - 5 min
   - Spatial cognition

Accuracy: ~70% of comprehensive battery
Use case: Screening, repeated assessments
```

### 7.3 Baseline Data Structure

```typescript
interface BaselineAssessment {
  assessmentId: string;
  userId: string;
  date: Date;

  // Demographic
  age: number;
  education: number; // years
  occupation: string;

  // General Intelligence
  iq: {
    full: number;
    verbal: number;
    performance: number;
    processing: number;
  };

  // Domain-specific
  domains: {
    MEMORY: DomainScore;
    ATTENTION: DomainScore;
    REASONING: DomainScore;
    CREATIVITY: DomainScore;
    LANGUAGE: DomainScore;
    EXECUTIVE: DomainScore;
    SPATIAL: DomainScore;
  };

  // Composite
  cognitiveIndex: number;

  // Normative data
  percentileRank: number;
  ageNormed: boolean;
  educationNormed: boolean;
}

interface DomainScore {
  rawScore: number;
  standardScore: number; // mean=100, SD=15
  percentile: number;
  confidence95: [number, number];
  subDomains: Record<string, number>;
}
```

---

## 8. Enhancement Protocols

### 8.1 Protocol Selection Flowchart

```
START
  ↓
Baseline Assessment
  ↓
Identify Target Domain(s)
  ↓
Determine Enhancement Goal (ER target)
  ↓
Assess Constraints (time, budget, safety)
  ↓
Select Enhancement Method(s)
  ↓
Design Protocol (duration, frequency, intensity)
  ↓
Safety Review & Medical Clearance
  ↓
Informed Consent
  ↓
Initial Enhancement Session
  ↓
Monitor & Measure Performance
  ↓
Adjust Protocol Based on Response
  ↓
Continue or Terminate?
  ├─ Continue → Return to Monitoring
  └─ Terminate → Final Assessment
```

### 8.2 Protocol Templates

#### 8.2.1 Memory Enhancement (Computational)

```yaml
protocol:
  name: "Working Memory Enhancement - Computational"
  targetDomain: MEMORY
  method: COMPUTATIONAL
  duration: 4 weeks

  phase1_baseline:
    week: 1
    activities:
      - Baseline assessment (full memory battery)
      - System familiarization
      - Baseline cognitive load measurement

  phase2_rampup:
    weeks: 2-3
    schedule:
      frequency: daily
      sessionDuration: 30 minutes
    activities:
      - Memory augmentation system activation
      - Gradual increase in augmentation level
      - Daily performance monitoring
    target: ER = 0.2 by end of phase

  phase3_optimization:
    week: 4
    schedule:
      frequency: daily
      sessionDuration: 45 minutes
    activities:
      - Full augmentation deployment
      - Real-world task integration
      - Performance optimization
    target: ER = 0.3-0.4

  monitoring:
    daily: Cognitive load, fatigue score
    weekly: Memory tests, subjective well-being
    endpoint: Full cognitive battery

  safety:
    cognitiveLoadLimit: 0.7
    fatigueThreshold: 0.6
    mandatoryBreaks: Every 45 minutes
```

#### 8.2.2 Attention Enhancement (Electrical + Training)

```yaml
protocol:
  name: "Sustained Attention - tDCS + Training"
  targetDomain: ATTENTION
  method: HYBRID (ELECTRICAL + TRAINING)
  duration: 3 weeks

  electricalStimulation:
    technique: tDCS
    montage:
      anode: F3 (left DLPFC)
      cathode: Fp2 (right supraorbital)
    parameters:
      current: 2.0 mA
      duration: 20 minutes
      frequency: 5 sessions per week

  training:
    type: Sustained attention task
    schedule:
      frequency: 5 sessions per week
      sessionDuration: 30 minutes
      timing: During and after tDCS
    difficulty: Adaptive (70-80% accuracy)

  schedule:
    week1:
      sessions: 3
      tDCS: Yes
      training: Yes
      target: ER = 0.1

    week2:
      sessions: 5
      tDCS: Yes
      training: Yes
      target: ER = 0.2

    week3:
      sessions: 5
      tDCS: Yes
      training: Yes
      target: ER = 0.3

  monitoring:
    presession: Skin check, questionnaire
    during: Discomfort monitoring
    postsession: Attention test, adverse events
    weekly: Comprehensive attention battery

  safety:
    screening: Epilepsy, metal implants, skin conditions
    adverseEvents: Headache, tingling, redness monitoring
    discontinuationCriteria: Severe adverse events, poor tolerance
```

### 8.3 Personalization Algorithm

```typescript
function personalizeProtocol(
  baseline: BaselineAssessment,
  goals: EnhancementGoals,
  constraints: Constraints
): PersonalizedProtocol {

  // 1. Identify enhancement potential
  const potential = calculateEnhancementPotential(baseline);

  // 2. Select optimal method
  const method = selectOptimalMethod(
    goals.targetDomain,
    constraints.safetyProfile,
    constraints.budget,
    constraints.timeAvailability
  );

  // 3. Determine intensity
  const intensity = calculateOptimalIntensity(
    baseline.domains[goals.targetDomain],
    goals.targetER,
    potential
  );

  // 4. Design schedule
  const schedule = designOptimalSchedule(
    method,
    intensity,
    constraints.timeAvailability
  );

  // 5. Set monitoring parameters
  const monitoring = defineMonitoringProtocol(
    method,
    goals.targetDomain,
    constraints.safetyProfile
  );

  return {
    method,
    intensity,
    schedule,
    monitoring,
    expectedOutcome: predictOutcome(baseline, method, intensity, schedule)
  };
}
```

---

## 9. Monitoring and Measurement

### 9.1 Real-time Monitoring Architecture

```typescript
interface MonitoringSystem {
  // Data collection
  sensors: {
    eeg?: EEGDevice;              // Brain activity
    eyeTracker?: EyeTracker;      // Attention, fatigue
    hrv?: HRVMonitor;             // Stress, arousal
    performance: PerformanceLogger; // Task metrics
  };

  // Processing
  dataProcessor: {
    samplingRate: number;         // Hz
    bufferSize: number;           // samples
    preprocessing: PreprocessingPipeline;
    featureExtraction: FeatureExtractor;
  };

  // Analysis
  analyzer: {
    cognitiveLoad: CognitiveLoadEstimator;
    fatigue: FatigueDetector;
    performance: PerformanceAnalyzer;
    safety: SafetyMonitor;
  };

  // Feedback
  feedback: {
    realtime: boolean;
    visualizations: Dashboard;
    alerts: AlertSystem;
    adaptiveControl: AdaptiveController;
  };
}
```

### 9.2 Key Monitoring Metrics

#### Real-time (continuous):
```
- Cognitive load index (CLI)
- Task performance (accuracy, RT)
- Physiological arousal (HRV, pupil dilation)
- Fatigue indicators
- Safety thresholds
```

#### Session-based (per session):
```
- Enhancement ratio
- Sustainability index
- Domain-specific performance
- Subjective ratings
- Adverse events
```

#### Periodic (weekly/monthly):
```
- Comprehensive cognitive battery
- Baseline drift assessment
- Long-term enhancement trajectory
- Transfer effects
- Quality of life measures
```

### 9.3 Data Storage and Privacy

```typescript
interface EnhancementRecord {
  // Identifiers
  recordId: string;
  userId: string; // Anonymized/encrypted
  timestamp: Date;

  // Session data
  session: {
    method: EnhancementMethod;
    domain: CognitiveDomain;
    duration: number;
    intensity: number;
  };


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
