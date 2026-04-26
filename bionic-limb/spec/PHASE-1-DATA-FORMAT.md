# WIA-AUG-007 PHASE 1 — Data Format Specification

**Standard:** WIA-AUG-007
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-AUG-007: Bionic Limb Specification v1.0

> **Standard ID:** WIA-AUG-007
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Human Augmentation Bionics Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Limb Classification System](#2-limb-classification-system)
3. [Control Methods](#3-control-methods)
4. [Sensory Feedback Systems](#4-sensory-feedback-systems)
5. [Socket Interface Standards](#5-socket-interface-standards)
6. [Grip Patterns and Dexterity](#6-grip-patterns-and-dexterity)
7. [Gait Analysis (Lower Limbs)](#7-gait-analysis-lower-limbs)
8. [Power and Battery Management](#8-power-and-battery-management)
9. [Maintenance and Calibration](#9-maintenance-and-calibration)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for bionic limb prosthetics, including upper and lower limb replacements. The standard ensures interoperability, safety, and optimal functionality across different manufacturers and control systems.

### 1.2 Scope

The standard covers:
- Classification of bionic limb types
- Control method specifications
- Sensory feedback requirements
- Socket interface standards
- Grip pattern definitions
- Gait analysis for lower limbs
- Power management protocols
- Maintenance and calibration procedures

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Bionic limb technologies should restore not just function, but dignity and independence to amputees. This specification ensures that prosthetic systems are standardized, accessible, and continuously improving to serve all who need them.

### 1.4 Terminology

- **Residual Limb**: The remaining biological limb after amputation
- **Socket**: The interface between residual limb and prosthetic device
- **DOF (Degrees of Freedom)**: Number of independent movements
- **EMG**: Electromyography - electrical signals from muscles
- **Proprioception**: Sense of limb position and movement
- **Myoelectric**: Control via muscle electrical signals
- **TMR**: Targeted Muscle Reinnervation
- **Osseointegration**: Direct skeletal attachment

---

## 2. Limb Classification System

### 2.1 Upper Limb Categories

| Category | Amputation Level | DOF Range | Typical Components |
|----------|------------------|-----------|-------------------|
| FINGER | Partial hand | 1-3 | Individual digit prosthetics |
| HAND | Wrist disarticulation | 3-6 | Hand only, passive wrist |
| FOREARM | Below elbow | 6-12 | Hand + powered wrist |
| UPPER_ARM | Above elbow | 12-18 | Hand + wrist + powered elbow |

### 2.2 Lower Limb Categories

| Category | Amputation Level | DOF Range | Typical Components |
|----------|------------------|-----------|-------------------|
| FOOT | Partial foot/ankle | 2-4 | Forefoot or ankle unit |
| LOWER_LEG | Below knee | 4-8 | Foot + powered ankle |
| THIGH | Above knee | 8-12 | Foot + ankle + microprocessor knee |

### 2.3 Classification Algorithm

```typescript
interface LimbClassification {
  type: LimbType;
  amputationLevel: string;
  residualLength: number; // cm
  dof: number;
  category: 'Minimal' | 'Basic' | 'Moderate' | 'Advanced';
}

function classifyLimb(input: {
  type: LimbType;
  dof: number;
  controlComplexity: number;
}): LimbClassification {
  const score = input.dof * 0.6 + input.controlComplexity * 0.4;

  let category: string;
  if (score <= 5) category = 'Minimal';
  else if (score <= 10) category = 'Basic';
  else if (score <= 15) category = 'Moderate';
  else category = 'Advanced';

  return { ...input, category };
}
```

### 2.4 Complexity Score

```
Complexity = (DOF × 0.4) + (Sensors × 0.3) + (Control Methods × 0.3)
```

Where:
- `DOF` = Degrees of freedom (1-20)
- `Sensors` = Number of sensor types (0-10)
- `Control Methods` = Number of available control methods (1-5)

---

## 3. Control Methods

### 3.1 Control Method Types

#### 3.1.1 Myoelectric Control

Surface EMG signals from residual limb muscles control prosthetic movement.

**Specifications:**
```
Signal Frequency: 20-450 Hz
Sampling Rate: ≥1000 Hz
Electrode Count: 2-16 channels
Processing Delay: <200ms
Classification Accuracy: ≥85%
```

**Algorithm:**
```typescript
interface MyoelectricControl {
  electrodes: ElectrodeConfig[];
  samplingRate: number; // Hz
  filterBandpass: { low: number; high: number };
  threshold: number; // μV
  processingDelay: number; // ms
}

function processEMG(signal: number[], config: MyoelectricControl): ControlSignal {
  // 1. Band-pass filter
  const filtered = bandpassFilter(signal, config.filterBandpass);

  // 2. Feature extraction (RMS, MAV, WL, etc.)
  const features = extractFeatures(filtered);

  // 3. Classification
  const intent = classifyIntent(features);

  // 4. Generate control signal
  return generateControl(intent);
}
```

#### 3.1.2 Neural Direct Control

Direct interface with peripheral or central nervous system.

**Specifications:**
```
Interface Type: Implanted electrodes
Channel Count: 16-128 channels
Impedance: 10-500 kΩ
Signal-to-Noise Ratio: ≥40 dB
Response Time: <100ms
Biocompatibility: Full ISO 10993
```

#### 3.1.3 Pattern Recognition

Machine learning-based gesture classification.

**Specifications:**
```
Training Time: 15-30 minutes initial
Retraining: Weekly recommended
Gesture Library: 10-50 gestures
Accuracy: ≥90% for trained gestures
Cross-session Stability: ≥80%
```

**ML Pipeline:**
```typescript
interface PatternRecognitionConfig {
  algorithm: 'LDA' | 'SVM' | 'CNN' | 'LSTM';
  features: FeatureType[];
  trainingEpochs: number;
  validationSplit: number;
}

function trainPatternRecognition(
  trainingData: EMGData[],
  labels: GestureLabel[],
  config: PatternRecognitionConfig
): TrainedModel {
  // Feature extraction
  const features = trainingData.map(d => extractFeatures(d, config.features));

  // Train model
  const model = trainModel(features, labels, config);

  // Validate
  const accuracy = validateModel(model, validationData);

  return { model, accuracy, config };
}
```

#### 3.1.4 Hybrid Control

Combination of multiple control methods.

**Common Combinations:**
- Myoelectric + Pattern Recognition
- Neural + Myoelectric
- Body-Powered + Myoelectric

### 3.2 Control Performance Metrics

| Metric | Minimum | Target | Measurement Method |
|--------|---------|--------|-------------------|
| Accuracy | 85% | 95% | Gesture classification |
| Response Time | <300ms | <150ms | Intent to motion |
| Learning Time | <60min | <20min | Initial calibration |
| Adaptation | 80% | 95% | Cross-session accuracy |
| Robustness | 75% | 90% | Performance under load |

### 3.3 Control Modes

```typescript
enum ControlMode {
  DIRECT = 'direct',           // One-to-one muscle mapping
  SEQUENTIAL = 'sequential',   // Mode switching
  PROPORTIONAL = 'proportional', // Speed/force control
  SIMULTANEOUS = 'simultaneous', // Multi-joint control
  ADAPTIVE = 'adaptive'        // Learning-based adaptation
}
```

---

## 4. Sensory Feedback Systems

### 4.1 Feedback Modalities

#### 4.1.1 Pressure Feedback

**Requirements:**
```
Dynamic Range: 0-200 N
Resolution: 1 N
Response Time: <50ms
Feedback Method: Vibration, electrical stimulation
Spatial Resolution: ≥5 locations
```

**Implementation:**
```typescript
interface PressureFeedback {
  sensors: PressureSensor[];
  feedbackType: 'vibration' | 'electrical' | 'mechanical';
  intensity: number; // 0-1
  location: SensorLocation;
  calibration: CalibrationData;
}

function providePressureFeedback(
  force: number,
  config: PressureFeedback
): FeedbackSignal {
  // Map force to feedback intensity
  const intensity = mapForceToIntensity(force, config.calibration);

  // Generate feedback signal
  const signal = generateFeedback(intensity, config.feedbackType);

  // Deliver to user
  return deliverFeedback(signal, config.location);
}
```

#### 4.1.2 Temperature Feedback

**Requirements:**
```
Range: 15-45°C
Accuracy: ±1°C
Response Time: <500ms
Safety Cutoff: >45°C or <15°C
```

#### 4.1.3 Position Feedback (Proprioception)

**Requirements:**
```
Joint Angle Accuracy: ±2°
Update Rate: ≥50 Hz
Latency: <100ms
Method: Vibration patterns, sensory substitution
```

#### 4.1.4 Slip Detection

**Requirements:**
```
Detection Threshold: 0.5 mm movement
Response Time: <30ms
False Positive Rate: <5%
Action: Automatic grip adjustment
```

### 4.2 Feedback Encoding Schemes

```typescript
interface FeedbackEncoding {
  modality: FeedbackType;
  encoding: 'amplitude' | 'frequency' | 'spatial' | 'temporal';
  intensityLevels: number;
  mappingFunction: (input: number) => FeedbackSignal;
}

// Example: Pressure mapped to vibration frequency
const pressureToVibration: FeedbackEncoding = {
  modality: 'PRESSURE',
  encoding: 'frequency',
  intensityLevels: 10,
  mappingFunction: (force: number) => ({
    frequency: 50 + (force / 200) * 200, // 50-250 Hz
    amplitude: 0.7,
    duration: 0 // continuous
  })
};
```

### 4.3 Multi-modal Feedback Integration

```typescript
interface MultiModalFeedback {
  pressure: PressureFeedback;
  temperature: TemperatureFeedback;
  position: PositionFeedback;
  priority: FeedbackPriority[];
  fusionAlgorithm: FusionMethod;
}

function integrateFeedback(
  inputs: SensorData[],
  config: MultiModalFeedback
): IntegratedFeedback {
  // Priority-based fusion
  const prioritized = prioritizeFeedback(inputs, config.priority);

  // Combine modalities
  const fused = fuseFeedback(prioritized, config.fusionAlgorithm);

  // Deliver to user
  return deliverIntegratedFeedback(fused);
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
