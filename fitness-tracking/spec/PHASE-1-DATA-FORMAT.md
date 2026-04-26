# WIA-IND-012 PHASE 1 — Data Format Specification

**Standard:** WIA-IND-012
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-IND-012: Fitness Tracking Standard - Technical Specification v1.0

> **Standard ID:** WIA-IND-012
> **Version:** 1.0.0
> **Status:** Active
> **Category:** Industry / Health & Fitness
> **Color:** Indigo (#6366F1)
> **Published:** 2025-12-27
> **Authors:** WIA Health & Fitness Working Group

---

## Abstract

This specification defines the WIA-IND-012 Fitness Tracking Standard, a comprehensive framework for monitoring, recording, and analyzing physical activity and health metrics. The standard provides unified interfaces for activity tracking, heart rate monitoring, calorie calculation, workout logging, and health metrics integration across diverse platforms and devices.

**弘益人間 (Benefit All Humanity)** - This standard promotes global health and wellness through accessible, interoperable fitness tracking technology.

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Terminology](#2-terminology)
3. [Core Architecture](#3-core-architecture)
4. [Activity Tracking](#4-activity-tracking)
5. [Heart Rate Monitoring](#5-heart-rate-monitoring)
6. [Calorie Calculation](#6-calorie-calculation)
7. [Workout Logging](#7-workout-logging)
8. [Health Metrics](#8-health-metrics)
9. [Goal Management](#9-goal-management)
10. [Data Synchronization](#10-data-synchronization)
11. [Privacy & Security](#11-privacy-security)
12. [API Specifications](#12-api-specifications)
13. [Implementation Guidelines](#13-implementation-guidelines)
14. [Certification Requirements](#14-certification-requirements)

---

## 1. Introduction

### 1.1 Purpose

The WIA-IND-012 standard establishes a unified framework for fitness tracking systems, enabling:

- Consistent activity monitoring across devices
- Accurate physiological metrics calculation
- Seamless data exchange between platforms
- Privacy-preserving health data management
- Evidence-based fitness recommendations

### 1.2 Scope

This standard covers:

- **Physical Activity**: Steps, distance, pace, elevation, GPS tracking
- **Cardiovascular**: Heart rate, heart rate variability, VO2 max estimation
- **Energy Expenditure**: Calorie calculation, metabolic equivalents (MET)
- **Exercise Sessions**: Structured workouts, training plans, performance analysis
- **Body Metrics**: Weight, body composition, measurements
- **Recovery**: Sleep quality, rest days, recovery metrics
- **Goals & Achievements**: Target setting, progress tracking, gamification

### 1.3 Target Audience

- Fitness device manufacturers
- Health application developers
- Gym and fitness center platforms
- Healthcare integration systems
- Sports performance analysis tools
- Corporate wellness programs
- Research institutions

### 1.4 Design Principles

1. **Accuracy**: Scientifically validated calculation methods
2. **Privacy**: User data ownership and consent-based sharing
3. **Interoperability**: Cross-platform data exchange
4. **Accessibility**: Support for diverse user populations
5. **Extensibility**: Adaptable to new metrics and modalities

---

## 2. Terminology

### 2.1 Key Terms

**Activity**: Any physical movement that increases energy expenditure above resting levels.

**MET (Metabolic Equivalent of Task)**: Ratio of working metabolic rate to resting metabolic rate (1 MET = 3.5 ml O₂/kg/min).

**Heart Rate Zone**: Range of heart rates corresponding to specific training intensities.

**VO2 Max**: Maximum rate of oxygen consumption during incremental exercise (ml/kg/min).

**TDEE (Total Daily Energy Expenditure)**: Total calories burned in 24 hours including BMR and activity.

**BMR (Basal Metabolic Rate)**: Energy expended at complete rest.

**RMR (Resting Metabolic Rate)**: Energy expended during normal rest (typically ~10% higher than BMR).

**Training Load**: Quantification of workout stress considering duration, intensity, and frequency.

**Recovery Heart Rate**: Decrease in heart rate during specified time after exercise cessation.

**HRV (Heart Rate Variability)**: Variation in time intervals between heartbeats.

### 2.2 Abbreviations

- **HR**: Heart Rate
- **BPM**: Beats Per Minute
- **GPS**: Global Positioning System
- **RPE**: Rate of Perceived Exertion
- **EPOC**: Excess Post-Exercise Oxygen Consumption
- **TRIMP**: Training Impulse
- **TSS**: Training Stress Score
- **FTP**: Functional Threshold Power
- **LTHR**: Lactate Threshold Heart Rate

---

## 3. Core Architecture

### 3.1 System Components

```
┌─────────────────────────────────────────────────────────┐
│                 Fitness Tracking System                 │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Activity   │  │  Heart Rate  │  │   Calorie    │  │
│  │  Tracker    │  │   Monitor    │  │  Calculator  │  │
│  └─────────────┘  └──────────────┘  └──────────────┘  │
│                                                         │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Workout    │  │    Health    │  │     Goal     │  │
│  │   Logger    │  │   Metrics    │  │   Manager    │  │
│  └─────────────┘  └──────────────┘  └──────────────┘  │
│                                                         │
│  ┌─────────────────────────────────────────────────┐  │
│  │          Data Synchronization Layer             │  │
│  └─────────────────────────────────────────────────┘  │
│                                                         │
│  ┌─────────────────────────────────────────────────┐  │
│  │          Privacy & Security Layer               │  │
│  └─────────────────────────────────────────────────┘  │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### 3.2 Data Flow

1. **Data Collection**: Sensors and user input capture raw data
2. **Processing**: Algorithms calculate derived metrics
3. **Storage**: Encrypted local and cloud storage
4. **Analysis**: Pattern recognition and insights generation
5. **Presentation**: User-facing dashboards and reports
6. **Synchronization**: Multi-device data consistency

### 3.3 Integration Points

- **Device APIs**: Smartwatch, fitness tracker, heart rate monitor
- **Platform APIs**: iOS HealthKit, Google Fit, Samsung Health
- **Third-party Services**: Strava, MyFitnessPal, TrainingPeaks
- **Healthcare Systems**: EHR, telemedicine platforms
- **Smart Equipment**: Treadmills, bikes, rowing machines

---

## 4. Activity Tracking

### 4.1 Activity Types

#### 4.1.1 Aerobic Activities

| Activity | MET Value | Tracking Requirements |
|----------|-----------|----------------------|
| Walking (2.5 mph) | 3.0 | Steps, duration, distance |
| Walking (3.0 mph) | 3.5 | Steps, duration, distance |
| Walking (4.0 mph) | 5.0 | Steps, duration, distance |
| Running (5 mph) | 8.3 | GPS, heart rate, cadence |
| Running (6 mph) | 9.8 | GPS, heart rate, cadence |
| Running (7 mph) | 11.0 | GPS, heart rate, cadence |
| Running (8 mph) | 11.8 | GPS, heart rate, cadence |
| Cycling (10-12 mph) | 6.8 | GPS, heart rate, power |
| Cycling (12-14 mph) | 8.0 | GPS, heart rate, power |
| Cycling (14-16 mph) | 10.0 | GPS, heart rate, power |
| Swimming (light) | 5.8 | Duration, strokes, distance |
| Swimming (moderate) | 7.0 | Duration, strokes, distance |
| Swimming (vigorous) | 9.8 | Duration, strokes, distance |

#### 4.1.2 Anaerobic Activities

| Activity | MET Value | Tracking Requirements |
|----------|-----------|----------------------|
| Weight lifting (light) | 3.0 | Sets, reps, weight |
| Weight lifting (moderate) | 5.0 | Sets, reps, weight |
| Weight lifting (vigorous) | 6.0 | Sets, reps, weight |
| HIIT training | 8.0 | Intervals, heart rate |
| CrossFit | 6.0-10.0 | Exercise type, reps, time |
| Plyometrics | 8.0 | Jump count, height |

#### 4.1.3 Sport Activities

| Activity | MET Value | Tracking Requirements |
|----------|-----------|----------------------|
| Basketball | 6.5 | Duration, heart rate |
| Soccer | 7.0 | GPS, distance, sprints |
| Tennis | 7.3 | Duration, heart rate |
| Volleyball | 4.0 | Duration, jumps |
| Golf (walking) | 4.8 | Steps, duration |

### 4.2 Step Counting

#### 4.2.1 Detection Algorithm

```
Step Detection:
1. Collect accelerometer data (x, y, z axes)
2. Calculate magnitude: √(x² + y² + z²)
3. Apply high-pass filter (0.5 Hz cutoff)
4. Detect peaks above threshold (user-calibrated)
5. Validate step pattern (cadence 60-200 steps/min)
6. Count confirmed steps
```

#### 4.2.2 Stride Length Estimation

```
Stride Length (meters) = Height (cm) × 0.415

Alternative (calibrated):
Stride Length = Distance (GPS) / Step Count
```

#### 4.2.3 Distance Calculation

```
Distance (meters) = Steps × Stride Length

For GPS-enabled:
Distance = Σ haversine(lat₁, lon₁, lat₂, lon₂)
```

### 4.3 Pace & Speed

#### 4.3.1 Pace Calculation

```
Pace (min/km) = Duration (minutes) / Distance (km)
Pace (min/mile) = Duration (minutes) / Distance (miles)

Instantaneous Pace = 60 / Current Speed (km/h)
```

#### 4.3.2 Speed Zones

| Zone | Pace (min/km) | Intensity | Purpose |
|------|---------------|-----------|---------|
| Recovery | > 7:00 | Very Easy | Active recovery |
| Easy | 6:00-7:00 | Easy | Base building |
| Moderate | 5:00-6:00 | Moderate | Endurance |
| Tempo | 4:30-5:00 | Hard | Threshold training |
| Interval | 4:00-4:30 | Very Hard | Speed work |
| Sprint | < 4:00 | Maximum | Power development |

### 4.4 Elevation Tracking

#### 4.4.1 Elevation Gain/Loss

```
Elevation Gain = Σ (elevation[i] - elevation[i-1]) where delta > 0
Elevation Loss = Σ (elevation[i-1] - elevation[i]) where delta > 0

Smoothing: Apply moving average (window = 5-10 points)
```

#### 4.4.2 Grade Calculation

```
Grade (%) = (Elevation Change / Horizontal Distance) × 100

Grade-Adjusted Pace (GAP):
GAP = Pace × (1 + (Grade / 100) × 0.033)
```

### 4.5 GPS Tracking

#### 4.5.1 Position Recording

```javascript
interface GPSPoint {
  latitude: number;      // Decimal degrees
  longitude: number;     // Decimal degrees
  elevation: number;     // Meters above sea level
  accuracy: number;      // Meters (±)
  timestamp: number;     // Unix timestamp (ms)
  speed?: number;        // m/s (device-calculated)
  heading?: number;      // Degrees (0-360)
}
```

#### 4.5.2 Distance Calculation (Haversine)

```
a = sin²(Δlat/2) + cos(lat₁) × cos(lat₂) × sin²(Δlon/2)
c = 2 × atan2(√a, √(1-a))
distance = R × c

Where R = 6371 km (Earth's radius)
```

#### 4.5.3 Route Smoothing

```
Apply Kalman filter or moving average to reduce GPS noise:
- Remove outlier points (speed > 15 m/s for running)
- Smooth trajectory (window = 3-5 points)
- Snap to known paths where available
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
