# WIA-AUG-007 PHASE 2 — API Interface Specification

**Standard:** WIA-AUG-007
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. Socket Interface Standards

### 5.1 Socket Design Requirements

#### 5.1.1 Fit and Comfort

**Specifications:**
```
Pressure Distribution: <50 kPa max at any point
Contact Area: ≥70% of residual limb surface
Material: Medical-grade silicone or thermoplastic
Liner Thickness: 3-6 mm
Donning Time: <2 minutes
```

#### 5.1.2 Suspension Methods

| Method | Retention Force | Comfort | Adjustability |
|--------|----------------|---------|---------------|
| Suction | High | Excellent | Low |
| Pin/Lock | Very High | Good | Medium |
| Sleeve | Medium | Excellent | High |
| Lanyard | Low | Fair | Very High |
| Osseointegration | Permanent | Variable | None |

#### 5.1.3 Socket Materials

```typescript
interface SocketMaterial {
  name: string;
  hardness: number; // Shore A
  biocompatibility: boolean;
  thermalConductivity: number; // W/m·K
  breathability: 'none' | 'low' | 'medium' | 'high';
  durability: number; // months typical life
}

const approvedMaterials: SocketMaterial[] = [
  {
    name: 'Medical Silicone',
    hardness: 20,
    biocompatibility: true,
    thermalConductivity: 0.2,
    breathability: 'low',
    durability: 12
  },
  {
    name: 'Thermoplastic Elastomer (TPE)',
    hardness: 40,
    biocompatibility: true,
    thermalConductivity: 0.15,
    breathability: 'medium',
    durability: 18
  },
  // ... more materials
];
```

### 5.2 Pressure Mapping

```typescript
interface PressureMap {
  grid: number[][]; // kPa values
  resolution: { rows: number; cols: number };
  timestamp: Date;
  maxPressure: number;
  meanPressure: number;
  hotspots: Location[];
}

function analyzePressureMap(map: PressureMap): SocketFitAssessment {
  const hotspots = map.grid.flatMap((row, i) =>
    row.map((p, j) => p > 50 ? { row: i, col: j, pressure: p } : null)
  ).filter(Boolean);

  const quality = hotspots.length === 0 ? 'Excellent' :
                  hotspots.length <= 3 ? 'Good' :
                  'Needs Adjustment';

  return {
    quality,
    hotspots,
    recommendations: generateFitRecommendations(hotspots)
  };
}
```

### 5.3 Socket Fitting Protocol

```
1. Residual Limb Assessment
   - Measure circumference at 5cm intervals
   - Identify bony prominences
   - Assess tissue quality
   - Document sensitive areas

2. Socket Fabrication
   - 3D scan or cast
   - Digital modification
   - Test socket fabrication
   - Pressure mapping verification

3. Fit Evaluation
   - Static alignment check
   - Pressure distribution analysis
   - Range of motion testing
   - User comfort assessment

4. Adjustment Cycle
   - Identify pressure points
   - Modify socket
   - Re-test
   - Iterate until optimal fit

5. Final Verification
   - Full functional testing
   - Prolonged wear trial (4-8 hours)
   - User satisfaction survey
   - Documentation
```

---

## 6. Grip Patterns and Dexterity

### 6.1 Standard Grip Patterns

#### 6.1.1 Power Grips

```typescript
enum PowerGrip {
  CYLINDRICAL = 'cylindrical',  // Holding tools, bottles
  SPHERICAL = 'spherical',      // Holding balls, fruit
  HOOK = 'hook',                // Carrying bags, heavy items
  LATERAL = 'lateral'           // Key grip
}

interface GripConfig {
  pattern: PowerGrip;
  force: number; // Newtons (0-200)
  speed: number; // 0-1 (percentage of max)
  precision: boolean;
}
```

#### 6.1.2 Precision Grips

```typescript
enum PrecisionGrip {
  TRIPOD_PINCH = 'tripod_pinch',    // Writing, eating
  LATERAL_PINCH = 'lateral_pinch',  // Turning keys
  TIP_PINCH = 'tip_pinch',          // Small objects
  PRECISION_GRIP = 'precision_grip' // Fine manipulation
}
```

### 6.2 Grip Force Control

**Requirements:**
```
Minimum Force: 5 N
Maximum Force: 200 N (adult), 100 N (child)
Force Resolution: 1 N
Force Stability: ±5% during hold
Proportional Control: 10 levels minimum
```

**Algorithm:**
```typescript
interface ForceController {
  currentForce: number;
  targetForce: number;
  maxForce: number;
  kp: number; // Proportional gain
  ki: number; // Integral gain
  kd: number; // Derivative gain
}

function pidForceControl(
  controller: ForceController,
  sensorForce: number,
  dt: number
): number {
  const error = controller.targetForce - sensorForce;
  const integral = controller.integral + error * dt;
  const derivative = (error - controller.prevError) / dt;

  const output =
    controller.kp * error +
    controller.ki * integral +
    controller.kd * derivative;

  return clamp(output, 0, controller.maxForce);
}
```

### 6.3 Grip Pattern Library

```json
{
  "gripPatterns": [
    {
      "id": "power_grip_01",
      "name": "Cylindrical Power Grip",
      "fingerPositions": {
        "thumb": { "flexion": 60, "abduction": 30 },
        "index": { "flexion": 90, "abduction": 0 },
        "middle": { "flexion": 95, "abduction": 0 },
        "ring": { "flexion": 95, "abduction": 0 },
        "pinky": { "flexion": 90, "abduction": 0 }
      },
      "force": { "min": 20, "max": 150 },
      "applications": ["tool_holding", "bottle_grip", "handle_grip"]
    },
    {
      "id": "precision_grip_01",
      "name": "Tripod Pinch",
      "fingerPositions": {
        "thumb": { "flexion": 30, "abduction": 40 },
        "index": { "flexion": 45, "abduction": 20 },
        "middle": { "flexion": 50, "abduction": 15 }
      },
      "force": { "min": 2, "max": 30 },
      "applications": ["writing", "eating", "precision_work"]
    }
  ]
}
```

### 6.4 Dexterity Metrics

```typescript
interface DexterityAssessment {
  gripPatterns: number;        // Number of available grips
  transitionTime: number;      // ms between grips
  forceResolution: number;     // Minimum force increment (N)
  independentFingers: number;  // DOF count
  manipulationScore: number;   // 0-100 composite score
}

function assessDexterity(limb: BionicLimb): DexterityAssessment {
  const score =
    (limb.gripPatterns.length * 5) +
    (100 / limb.averageTransitionTime) +
    (limb.dof * 3) +
    (limb.forceControl.resolution * 2);

  return {
    gripPatterns: limb.gripPatterns.length,
    transitionTime: limb.averageTransitionTime,
    forceResolution: limb.forceControl.resolution,
    independentFingers: limb.dof,
    manipulationScore: Math.min(100, score)
  };
}
```

---

## 7. Gait Analysis (Lower Limbs)

### 7.1 Gait Cycle Phases

```
Stance Phase (60%):
├── Initial Contact (0-2%)
├── Loading Response (2-12%)
├── Mid Stance (12-31%)
├── Terminal Stance (31-50%)
└── Pre-Swing (50-62%)

Swing Phase (40%):
├── Initial Swing (62-75%)
├── Mid Swing (75-87%)
└── Terminal Swing (87-100%)
```

### 7.2 Gait Parameters

```typescript
interface GaitParameters {
  spatiotemporal: {
    strideLength: number;      // cm
    stepLength: number;        // cm
    stepWidth: number;         // cm
    cadence: number;           // steps/min
    velocity: number;          // m/s
    stanceTime: number;        // % gait cycle
    swingTime: number;         // % gait cycle
    doubleSupport: number;     // % gait cycle
  };
  kinematics: {
    hipFlexion: AngleRange;
    kneeFlexion: AngleRange;
    ankleDorsiflexion: AngleRange;
    pelvicTilt: AngleRange;
  };
  kinetics: {
    groundReactionForce: ForceVector[];
    kneeMoment: number[];
    anklePower: number[];
  };
}
```

### 7.3 Normal Gait Ranges

| Parameter | Normal Range | Prosthetic Target |
|-----------|--------------|-------------------|
| Stride Length | 130-150 cm | 120-145 cm |
| Cadence | 110-120 steps/min | 100-115 steps/min |
| Stance/Swing | 60:40 | 58:42 - 62:38 |
| Knee Flexion (swing) | 60-70° | 55-65° |
| Ankle Dorsiflexion | 10-15° | 8-12° |
| Walking Speed | 1.2-1.4 m/s | 1.0-1.3 m/s |

### 7.4 Gait Analysis Algorithm

```typescript
interface GaitAnalysis {
  leftStep: GaitCycle;
  rightStep: GaitCycle;
  symmetry: SymmetryMetrics;
  efficiency: number;
  stability: number;
}

function analyzeGait(
  sensorData: IMUData[],
  duration: number
): GaitAnalysis {
  // Detect gait events
  const heelStrikes = detectHeelStrikes(sensorData);
  const toeOffs = detectToeOffs(sensorData);

  // Segment gait cycles
  const cycles = segmentGaitCycles(heelStrikes, toeOffs);

  // Calculate parameters for each cycle
  const parameters = cycles.map(c => calculateGaitParameters(c));

  // Assess symmetry
  const symmetry = assessSymmetry(parameters);

  // Calculate efficiency and stability
  const efficiency = calculateEfficiency(parameters);
  const stability = calculateStability(sensorData);

  return {
    leftStep: parameters.filter(p => p.side === 'left')[0],
    rightStep: parameters.filter(p => p.side === 'right')[0],
    symmetry,
    efficiency,
    stability
  };
}
```

### 7.5 Symmetry Assessment

```typescript
interface SymmetryMetrics {
  spatialSymmetry: number;    // 0-1 (1 = perfect)
  temporalSymmetry: number;   // 0-1
  forceSymmetry: number;      // 0-1
  overallSymmetry: number;    // 0-1
}

function assessSymmetry(
  left: GaitParameters,
  right: GaitParameters
): SymmetryMetrics {
  const spatial = 1 - Math.abs(
    (left.spatiotemporal.strideLength - right.spatiotemporal.strideLength) /
    ((left.spatiotemporal.strideLength + right.spatiotemporal.strideLength) / 2)
  );

  const temporal = 1 - Math.abs(
    (left.spatiotemporal.stanceTime - right.spatiotemporal.stanceTime) /
    ((left.spatiotemporal.stanceTime + right.spatiotemporal.stanceTime) / 2)
  );

  // Force symmetry calculation...
  const force = 0.9; // placeholder

  const overall = (spatial + temporal + force) / 3;

  return { spatial, temporal, force, overall };
}
```

### 7.6 Microprocessor Knee Control

```typescript
interface KneeControl {
  mode: 'stance' | 'swing' | 'transition';
  resistance: number;     // 0-1


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
