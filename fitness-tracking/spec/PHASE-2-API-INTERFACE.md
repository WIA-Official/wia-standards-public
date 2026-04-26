# WIA-IND-012 PHASE 2 — API Interface Specification

**Standard:** WIA-IND-012
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. Heart Rate Monitoring

### 5.1 Heart Rate Zones

#### 5.1.1 Maximum Heart Rate Calculation

```
Method 1 (Traditional):
Max HR = 220 - Age

Method 2 (Tanaka):
Max HR = 208 - (0.7 × Age)

Method 3 (Gulati - Women):
Max HR = 206 - (0.88 × Age)

Method 4 (Measured):
Max HR = Actual maximum during incremental test
```

#### 5.1.2 Zone Definitions

```
Resting HR Reserve Method:
HR Reserve = Max HR - Resting HR
Target HR = Resting HR + (HR Reserve × Intensity%)

Zone 1 (Recovery): 50-60% of Max HR
Zone 2 (Aerobic): 60-70% of Max HR
Zone 3 (Tempo): 70-80% of Max HR
Zone 4 (Threshold): 80-90% of Max HR
Zone 5 (Maximum): 90-100% of Max HR
```

#### 5.1.3 Training Benefits by Zone

| Zone | % Max HR | % HRR | Benefit | Duration |
|------|----------|-------|---------|----------|
| 1 | 50-60% | 50-60% | Recovery, warm-up | 20-40 min |
| 2 | 60-70% | 60-70% | Base fitness, fat burning | 40-80 min |
| 3 | 70-80% | 70-80% | Aerobic capacity | 20-40 min |
| 4 | 80-90% | 80-90% | Lactate threshold | 10-20 min |
| 5 | 90-100% | 90-100% | VO2 max, speed | 2-10 min |

### 5.2 Heart Rate Variability (HRV)

#### 5.2.1 HRV Metrics

```
RMSSD (Root Mean Square of Successive Differences):
RMSSD = √(Σ(RR[i+1] - RR[i])² / (N-1))

SDNN (Standard Deviation of NN intervals):
SDNN = √(Σ(RR[i] - mean_RR)² / (N-1))

pNN50 (% of successive RR intervals > 50ms):
pNN50 = (count(|RR[i+1] - RR[i]| > 50ms) / (N-1)) × 100
```

#### 5.2.2 HRV Interpretation

| RMSSD (ms) | Status | Action |
|------------|--------|--------|
| > 50 | Excellent recovery | High intensity OK |
| 30-50 | Good recovery | Moderate intensity |
| 20-30 | Fair recovery | Light training |
| < 20 | Poor recovery | Rest recommended |

### 5.3 Recovery Metrics

#### 5.3.1 Recovery Heart Rate

```
Recovery HR = HR(exercise end) - HR(1 minute later)

Excellent: > 25 BPM drop
Good: 15-25 BPM drop
Fair: 10-15 BPM drop
Poor: < 10 BPM drop
```

#### 5.3.2 Resting Heart Rate Trends

```
Monitor RHR over 7-day rolling average:
- Decrease: Improving fitness
- Stable: Maintenance
- Increase (3+ BPM): Possible overtraining or illness
```

### 5.4 VO2 Max Estimation

#### 5.4.1 Cooper Test Method

```
VO2 max (ml/kg/min) = (Distance(meters) - 504.9) / 44.73

Where distance is covered in 12 minutes
```

#### 5.4.2 Heart Rate-Based Estimation

```
VO2 max = 15.3 × (Max HR / Resting HR)

Fitness Index:
VO2 max = 15 × (Max HR / Resting HR) × Age_factor

Where Age_factor:
- 20-29: 1.0
- 30-39: 0.93
- 40-49: 0.83
- 50-59: 0.74
- 60+: 0.65
```

#### 5.4.3 VO2 Max Categories

| Age | Male (ml/kg/min) | Female (ml/kg/min) | Classification |
|-----|------------------|-------------------|----------------|
| 20-29 | > 52 | > 44 | Excellent |
| 20-29 | 43-52 | 37-44 | Good |
| 20-29 | 35-42 | 30-36 | Average |
| 30-39 | > 49 | > 41 | Excellent |
| 30-39 | 40-49 | 34-41 | Good |
| 40-49 | > 46 | > 38 | Excellent |
| 40-49 | 37-46 | 31-38 | Good |

---

## 6. Calorie Calculation

### 6.1 Basal Metabolic Rate (BMR)

#### 6.1.1 Mifflin-St Jeor Equation

```
Men:
BMR = (10 × weight_kg) + (6.25 × height_cm) - (5 × age) + 5

Women:
BMR = (10 × weight_kg) + (6.25 × height_cm) - (5 × age) - 161
```

#### 6.1.2 Harris-Benedict Equation

```
Men:
BMR = 88.362 + (13.397 × weight_kg) + (4.799 × height_cm) - (5.677 × age)

Women:
BMR = 447.593 + (9.247 × weight_kg) + (3.098 × height_cm) - (4.330 × age)
```

### 6.2 Total Daily Energy Expenditure (TDEE)

```
TDEE = BMR × Activity Factor

Activity Factors:
- Sedentary (little/no exercise): 1.2
- Lightly active (1-3 days/week): 1.375
- Moderately active (3-5 days/week): 1.55
- Very active (6-7 days/week): 1.725
- Extremely active (physical job + training): 1.9
```

### 6.3 Activity Calorie Calculation

#### 6.3.1 MET-Based Calculation

```
Calories = (MET × weight_kg × duration_hours)

Example: 70kg person running (9 MET) for 30 minutes
Calories = 9 × 70 × 0.5 = 315 kcal
```

#### 6.3.2 Heart Rate-Based Calculation

```
Men:
Calories = ((Age × 0.2017) - (weight_kg × 0.09036) + (HR × 0.6309) - 55.0969) × duration_min / 4.184

Women:
Calories = ((Age × 0.074) - (weight_kg × 0.05741) + (HR × 0.4472) - 20.4022) × duration_min / 4.184
```

#### 6.3.3 Advanced Calculation (with VO2)

```
VO2 (ml/kg/min) = ((HR / Max HR) × VO2 max)
Calories/min = (VO2 × weight_kg × 5) / 1000

Total Calories = Calories/min × duration_min
```

### 6.4 Exercise Post-Oxygen Consumption (EPOC)

```
EPOC Calories = Base Calories × EPOC_factor

EPOC Factors:
- Low intensity (< 50% VO2 max): 1.05
- Moderate intensity (50-75% VO2 max): 1.10
- High intensity (> 75% VO2 max): 1.15
- HIIT/Strength training: 1.20-1.25
```

### 6.5 Macronutrient Energy

```
Carbohydrates: 4 kcal/gram
Protein: 4 kcal/gram
Fat: 9 kcal/gram
Alcohol: 7 kcal/gram
```

---

## 7. Workout Logging

### 7.1 Workout Structure

```javascript
interface Workout {
  id: string;
  userId: string;
  type: WorkoutType;
  startTime: Date;
  endTime: Date;
  duration: number;           // seconds

  // Activity metrics
  distance?: number;          // meters
  steps?: number;
  elevation?: {
    gain: number;             // meters
    loss: number;             // meters
  };

  // Cardiovascular metrics
  heartRate?: {
    avg: number;              // BPM
    max: number;              // BPM
    min: number;              // BPM
    zones: HeartRateZones;
  };

  // Energy expenditure
  calories: number;
  caloriesSources?: {
    active: number;
    resting: number;
    epoc: number;
  };

  // Performance metrics
  pace?: {
    avg: number;              // min/km
    max: number;              // min/km (fastest)
  };

  speed?: {
    avg: number;              // km/h
    max: number;              // km/h
  };

  cadence?: {
    avg: number;              // steps/min or RPM
    max: number;
  };

  power?: {
    avg: number;              // watts
    max: number;              // watts
    normalized: number;       // NP
  };

  // GPS data
  route?: GPSPoint[];

  // Intervals
  intervals?: Interval[];

  // User feedback
  perceivedExertion?: number; // 1-10 RPE scale
  notes?: string;

  // Equipment
  equipment?: string[];       // shoe ID, bike ID, etc.

  // Weather conditions
  weather?: WeatherCondition;

  // Training load
  trainingLoad?: number;
  tss?: number;               // Training Stress Score
}
```

### 7.2 Training Load Calculation

#### 7.2.1 TRIMP (Training Impulse)

```
TRIMP = Duration (min) × HR_ratio × e^(k × HR_ratio)

Where:
HR_ratio = (HR_avg - HR_rest) / (HR_max - HR_rest)
k = 1.92 (men), 1.67 (women)
```

#### 7.2.2 Training Stress Score (TSS)

```
For power-based:
TSS = (duration_sec × NP × IF) / (FTP × 3600) × 100

Where:


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
