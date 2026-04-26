# WIA-AUG-007 PHASE 3 — Protocol Specification

**Standard:** WIA-AUG-007
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

flexionAngle: number;   // degrees
  extensionStop: number;  // degrees
  swingFlexion: number;   // degrees
  terrain: TerrainType;
}

function controlKnee(
  sensors: SensorData,
  phase: GaitPhase,
  config: KneeControl
): KneeCommand {
  switch (phase) {
    case 'stance':
      // High resistance during weight bearing
      return {
        resistance: 0.9,
        targetAngle: sensors.kneeAngle,
        mode: 'lock'
      };

    case 'swing':
      // Low resistance, natural flexion
      return {
        resistance: 0.1,
        targetAngle: config.swingFlexion,
        mode: 'free'
      };

    case 'pre_swing':
      // Prepare for swing
      return {
        resistance: 0.3,
        targetAngle: 0,
        mode: 'transition'
      };
  }
}
```

---

## 8. Power and Battery Management

### 8.1 Battery Requirements

```typescript
interface BatterySpecification {
  type: 'LiPo' | 'Li-ion' | 'LiFePO4';
  capacity: number;        // mAh
  voltage: number;         // V
  weight: number;          // grams
  cycles: number;          // charge cycles
  runtime: number;         // hours (typical use)
  chargingTime: number;    // hours
  safetyFeatures: string[];
}

const standardBattery: BatterySpecification = {
  type: 'Li-ion',
  capacity: 2000,
  voltage: 7.4,
  weight: 60,
  cycles: 500,
  runtime: 8,
  chargingTime: 2,
  safetyFeatures: [
    'overcharge_protection',
    'short_circuit_protection',
    'thermal_cutoff'
  ]
};
```

### 8.2 Power Consumption

| Component | Idle (mW) | Active (mW) | Peak (mW) |
|-----------|-----------|-------------|-----------|
| Control System | 50 | 200 | 500 |
| Sensors | 20 | 100 | 150 |
| Motors (per joint) | 0 | 1000 | 5000 |
| Feedback System | 10 | 50 | 100 |
| Communication | 5 | 30 | 60 |

### 8.3 Power Management Strategy

```typescript
interface PowerManagement {
  mode: 'performance' | 'balanced' | 'economy';
  batteryLevel: number; // 0-100%
  estimatedRuntime: number; // minutes
  powerSaving: boolean;
}

function managePower(
  battery: BatteryState,
  activity: ActivityLevel
): PowerManagement {
  // Calculate mode based on battery and activity
  let mode: PowerManagement['mode'];

  if (battery.level > 50 && activity === 'high') {
    mode = 'performance';
  } else if (battery.level > 20) {
    mode = 'balanced';
  } else {
    mode = 'economy';
  }

  // Estimate runtime
  const consumption = estimateConsumption(activity, mode);
  const runtime = (battery.level / 100) * (battery.capacity / consumption) * 60;

  // Enable power saving if needed
  const powerSaving = battery.level < 20;

  return {
    mode,
    batteryLevel: battery.level,
    estimatedRuntime: runtime,
    powerSaving
  };
}
```

### 8.4 Charging Protocol

```
1. Pre-charge Check
   - Verify battery health
   - Check temperature (10-40°C)
   - Inspect connections

2. Charging Stages
   - Stage 1: Constant Current (CC) - 1C rate
   - Stage 2: Constant Voltage (CV) - 4.2V per cell
   - Stage 3: Trickle charge until full

3. Safety Monitoring
   - Temperature monitoring
   - Voltage monitoring
   - Current monitoring
   - Auto-cutoff at full charge

4. Post-charge
   - Balance cells (multi-cell)
   - Record charge cycle
   - Update battery health metrics
```

### 8.5 Battery Health Monitoring

```typescript
interface BatteryHealth {
  cycleCount: number;
  capacity: number;        // % of original
  impedance: number;       // mΩ
  health: 'excellent' | 'good' | 'fair' | 'replace';
  estimatedLife: number;   // days
}

function assessBatteryHealth(
  battery: BatteryState,
  history: ChargeHistory[]
): BatteryHealth {
  const degradation = calculateDegradation(history);
  const currentCapacity = 100 - degradation;

  let health: BatteryHealth['health'];
  if (currentCapacity > 80) health = 'excellent';
  else if (currentCapacity > 60) health = 'good';
  else if (currentCapacity > 40) health = 'fair';
  else health = 'replace';

  const estimatedLife = estimateRemainingLife(
    battery.cycleCount,
    currentCapacity
  );

  return {
    cycleCount: battery.cycleCount,
    capacity: currentCapacity,
    impedance: battery.impedance,
    health,
    estimatedLife
  };
}
```

---

## 9. Maintenance and Calibration

### 9.1 Maintenance Schedule

#### 9.1.1 Daily Maintenance

```
User Tasks:
□ Visual inspection for damage
□ Clean socket and liner
□ Check battery level
□ Verify proper fit
□ Test basic functions
□ Charge battery overnight
```

#### 9.1.2 Weekly Maintenance

```
User Tasks:
□ Deep clean all components
□ Inspect cables and connections
□ Check for unusual sounds
□ Verify control responsiveness
□ Test all grip patterns
□ Check electrode contact (EMG systems)

Technician Tasks (if needed):
□ Pressure mapping check
□ Software updates
□ Calibration verification
```

#### 9.1.3 Monthly Maintenance

```
Technician Tasks:
□ Full system diagnostic
□ Recalibrate control system
□ Inspect mechanical components
□ Test safety systems
□ Update firmware
□ Document performance metrics
□ Adjust socket fit if needed
```

#### 9.1.4 Annual Maintenance

```
Comprehensive Service:
□ Complete disassembly and inspection
□ Replace wear components
□ Full recalibration
□ Battery health assessment
□ Structural integrity test
□ User training refresh
□ Performance benchmarking
□ Documentation update
```

### 9.2 Calibration Procedures

#### 9.2.1 Control System Calibration

```typescript
interface CalibrationProcedure {
  type: 'myoelectric' | 'neural' | 'pattern';
  duration: number;      // minutes
  exercises: Exercise[];
  validation: ValidationTest[];
  targetAccuracy: number; // 0-1
}

async function calibrateControl(
  limbId: string,
  userId: string,
  config: CalibrationProcedure
): Promise<CalibrationResult> {
  // Step 1: Baseline recording
  const baseline = await recordBaseline(limbId, 30);

  // Step 2: Training exercises
  const trainingData = [];
  for (const exercise of config.exercises) {
    const data = await recordExercise(exercise, userId);
    trainingData.push(data);
  }

  // Step 3: Model training
  const model = await trainModel(trainingData);

  // Step 4: Validation
  const accuracy = await validateModel(model, config.validation);

  // Step 5: Deploy if accurate enough
  if (accuracy >= config.targetAccuracy) {
    await deployModel(limbId, model);
    return {
      success: true,
      accuracy,
      timestamp: new Date()
    };
  } else {
    return {
      success: false,
      accuracy,
      message: 'Accuracy below threshold, recalibration needed'
    };
  }
}
```

#### 9.2.2 Force Calibration

```
Procedure:
1. Zero calibration (no load)
2. Span calibration (known loads)
   - 10N reference
   - 50N reference
   - 100N reference
   - 200N reference
3. Linearity verification
4. Hysteresis test
5. Repeatability test (10 cycles)
6. Generate calibration curve
7. Apply calibration to system
8. Verification test
```

#### 9.2.3 Sensor Calibration

```typescript
interface SensorCalibration {
  sensorId: string;
  type: SensorType;
  calibrationPoints: CalibrationPoint[];
  calibrationCurve: (input: number) => number;
  lastCalibrated: Date;
  nextCalibration: Date;
}

function calibrateSensor(
  sensor: Sensor,
  referenceInputs: number[],
  referenceOutputs: number[]
): SensorCalibration {
  // Fit calibration curve
  const curve = fitPolynomial(referenceInputs, referenceOutputs, 2);

  // Generate calibration function
  const calibrationCurve = (raw: number) => evaluatePolynomial(curve, raw);

  // Schedule next calibration
  const nextCalibration = new Date();
  nextCalibration.setMonth(nextCalibration.getMonth() + 3);

  return {
    sensorId: sensor.id,
    type: sensor.type,
    calibrationPoints: referenceInputs.map((input, i) => ({
      reference: input,
      measured: referenceOutputs[i]
    })),
    calibrationCurve,
    lastCalibrated: new Date(),
    nextCalibration
  };
}
```

### 9.3 Diagnostic Tests

```typescript
interface DiagnosticTest {
  testId: string;
  name: string;
  category: 'mechanical' | 'electrical' | 'control' | 'safety';
  procedure: TestProcedure;
  passCriteria: PassCriteria;
}

const diagnosticTests: DiagnosticTest[] = [
  {
    testId: 'MECH-001',
    name: 'Joint Range of Motion',
    category: 'mechanical',
    procedure: {
      steps: [
        'Position limb in starting position',
        'Move each joint through full range',
        'Record min and max angles',
        'Compare to specifications'
      ]


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
