# WIA-AUG-010 PHASE 2 — API Interface Specification

**Standard:** WIA-AUG-010
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. Function Monitoring System

### 5.1 Real-Time Monitoring Framework

```typescript
interface FunctionMetrics {
  organId: string;
  timestamp: Date;

  output: {
    value: number;              // Organ-specific units
    unit: string;               // L/min, ml/min, units/day
    target: number;             // Expected value
    percentOfTarget: number;    // %
  };

  efficiency: {
    value: number;              // 0-1
    energyInput: number;        // Watts or metabolic equivalent
    usefulOutput: number;       // Functional units
  };

  biomarkers: {
    bloodChemistry: Record<string, number>;
    metabolicIndicators: Record<string, number>;
    organSpecific: Record<string, number>;
  };

  physiology: {
    temperature: number;        // °C
    pressure: number;           // mmHg or cmH₂O
    flow: number;              // ml/min
    resistance: number;        // dynes·s/cm⁵ or equivalent
  };
}
```

### 5.2 Organ-Specific Monitoring

#### 5.2.1 Heart Monitoring
```
Primary Metrics:
- Cardiac Output: 4-6 L/min (rest), up to 25 L/min (peak)
- Ejection Fraction: >55% (if pulsatile)
- Stroke Volume: 60-100 ml/beat
- Heart Rate: 60-100 bpm (if rhythm control)
- Pressure: Systolic 120±20, Diastolic 80±10 mmHg

Secondary Metrics:
- Power Output: 1-2 Watts (rest)
- Efficiency: >75%
- Temperature: 36-38°C
- Thrombosis Markers: D-dimer, platelet count
```

#### 5.2.2 Kidney Monitoring
```
Primary Metrics:
- GFR: >90 ml/min/1.73m² (optimal)
- Urine Output: 800-2000 ml/day
- Creatinine Clearance: >80 ml/min
- BUN: 7-20 mg/dL
- Electrolytes: Na⁺ 135-145, K⁺ 3.5-5.0 mEq/L

Secondary Metrics:
- Fluid Balance: Input vs. output
- Acid-Base: pH 7.35-7.45
- Phosphate Removal: Adequate control
- Albumin Loss: Minimal proteinuria
```

#### 5.2.3 Liver Monitoring
```
Primary Metrics:
- ALT/AST: <40 U/L (normal function)
- Bilirubin: 0.3-1.2 mg/dL
- Albumin: 3.5-5.0 g/dL
- PT/INR: 0.9-1.1 (normal coagulation)
- Ammonia: <50 μg/dL

Secondary Metrics:
- Glucose Production: Maintain 70-140 mg/dL
- Lipid Metabolism: Cholesterol, triglycerides
- Drug Clearance: Appropriate half-lives
- Synthetic Function: Clotting factors
```

#### 5.2.4 Lung Monitoring
```
Primary Metrics:
- O₂ Saturation: >95%
- PaO₂: >80 mmHg
- PaCO₂: 35-45 mmHg
- Tidal Volume: 500 ml (6-8 ml/kg)
- Respiratory Rate: 12-20/min

Secondary Metrics:
- Compliance: >100 ml/cmH₂O
- Resistance: <2.5 cmH₂O/L/s
- Dead Space: <30% of tidal volume
- Shunt Fraction: <5%
```

### 5.3 Alert Thresholds

```
Critical Alerts (Immediate Response):
- Output <50% of target
- Efficiency <60%
- Temperature >39°C or <35°C
- Pressure outside 50-200% of normal
- Biomarker deviation >3 SD from baseline

Warning Alerts (Monitor Closely):
- Output 50-70% of target
- Efficiency 60-75%
- Temperature 38-39°C or 35-36°C
- Pressure outside 75-150% of normal
- Biomarker deviation 2-3 SD from baseline

Information (Trending):
- Output 70-90% of target
- Efficiency 75-85%
- Minor biomarker fluctuations
- Performance degradation trends
```

---

## 6. Rejection Detection Protocol

### 6.1 Rejection Risk Assessment

```
Rejection Risk Score = (Immune_Markers × 0.35) +
                       (Performance_Degradation × 0.30) +
                       (Inflammation_Indicators × 0.20) +
                       (Antibody_Levels × 0.15)

where each factor is scored 0-1 (1 = high risk)

Risk Levels:
  Score > 0.70: High Risk - Immediate intervention
  Score 0.40-0.70: Moderate Risk - Increase monitoring, adjust immunosuppression
  Score < 0.40: Low Risk - Continue standard monitoring
```

### 6.2 Immune Marker Surveillance

```typescript
interface ImmuneMarkers {
  CRP: number;                  // mg/L
  ESR: number;                  // mm/h
  WBC: number;                  // cells/μL
  lymphocytes: number;          // %
  cytokines: {
    IL6: number;                // pg/ml
    TNFalpha: number;           // pg/ml
    IL1beta: number;            // pg/ml
    IFNgamma: number;           // pg/ml
  };
  complement: {
    C3: number;                 // mg/dL
    C4: number;                 // mg/dL
    CH50: number;               // U/ml
  };
}

function assessImmuneMarkers(markers: ImmuneMarkers): number {
  let score = 0;

  // CRP scoring
  if (markers.CRP > 100) score += 0.35;
  else if (markers.CRP > 50) score += 0.20;
  else if (markers.CRP > 10) score += 0.10;

  // ESR scoring
  if (markers.ESR > 50) score += 0.15;
  else if (markers.ESR > 30) score += 0.08;

  // Cytokine elevation
  const cytoScore = calculateCytokineScore(markers.cytokines);
  score += cytoScore * 0.30;

  // Complement activation
  const compScore = calculateComplementScore(markers.complement);
  score += compScore * 0.20;

  return Math.min(score, 1.0);
}
```

### 6.3 Performance Degradation Monitoring

```typescript
interface PerformanceTrend {
  currentOutput: number;
  baselineOutput: number;
  trend7days: number;          // % change
  trend30days: number;         // % change
  variability: number;         // coefficient of variation
  efficiency: number;          // current vs. baseline
}

function assessPerformanceDegradation(trend: PerformanceTrend): number {
  const outputRatio = trend.currentOutput / trend.baselineOutput;
  const trendScore = calculateTrendScore(trend.trend7days, trend.trend30days);
  const variabilityScore = trend.variability; // Higher = worse
  const efficiencyScore = 1 - trend.efficiency;

  return (
    (1 - outputRatio) * 0.40 +
    trendScore * 0.30 +
    variabilityScore * 0.20 +
    efficiencyScore * 0.10
  );
}
```

### 6.4 Antibody Level Monitoring

```
Antibody Testing Schedule:
- Week 1: Every 2-3 days
- Weeks 2-4: Weekly
- Months 2-6: Bi-weekly
- Months 7-12: Monthly
- Year 2+: Quarterly (or as indicated)

Tests:
1. Donor-Specific Antibodies (DSA)
   - Class I HLA (HLA-A, -B, -C)
   - Class II HLA (HLA-DR, -DQ, -DP)
   - MFI (Mean Fluorescence Intensity) >1000 = significant

2. Panel Reactive Antibodies (PRA)
   - % reactivity to panel
   - >80% = highly sensitized

3. Crossmatch Tests
   - T-cell crossmatch
   - B-cell crossmatch
   - Flow cytometry crossmatch
```

### 6.5 Rejection Response Protocol

```
High Risk Detection (Score > 0.70):
1. Immediate notification to medical team
2. Increase immunosuppression (per protocol)
3. Daily monitoring of all markers
4. Biopsy if indicated (for biological organs)
5. Consider rescue therapy (methylprednisolone, thymoglobulin)
6. Evaluate for infection vs. rejection

Moderate Risk (Score 0.40-0.70):
1. Alert medical team
2. Increase monitoring frequency (2x)
3. Review immunosuppression levels
4. Adjust medications if subtherapeutic
5. Screen for infection
6. Trend markers closely

Low Risk (Score < 0.40):
1. Continue standard monitoring
2. Maintain current immunosuppression
3. Routine follow-up
4. Patient education on warning signs
```

---

## 7. Performance Optimization

### 7.1 Adaptive Control Algorithms

```typescript
interface ControlParameters {
  targetOutput: number;
  currentOutput: number;
  energyInput: number;
  physiologicalDemand: number;

  controlMode: 'FIXED' | 'ADAPTIVE' | 'PREDICTIVE';

  constraints: {
    maxOutput: number;
    minOutput: number;
    maxPower: number;
    safetyLimits: SafetyLimits;
  };
}

class AdaptiveController {
  optimize(params: ControlParameters): ControlOutput {
    // PID control for output regulation
    const error = params.targetOutput - params.currentOutput;


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
