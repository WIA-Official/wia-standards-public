# WIA-AUTO-017 PHASE 1 — Data Format Specification

**Standard:** WIA-AUTO-017
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-AUTO-017: Delivery Drone Specification v1.0

> **Standard ID:** WIA-AUTO-017
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Autonomous Vehicle Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Drone Classifications](#2-drone-classifications)
3. [Flight Control Systems](#3-flight-control-systems)
4. [Navigation and Path Planning](#4-navigation-and-path-planning)
5. [Payload Management](#5-payload-management)
6. [UTM Integration](#6-utm-integration)
7. [Battery and Range Management](#7-battery-and-range-management)
8. [Safety and Emergency Protocols](#8-safety-and-emergency-protocols)
9. [Data Formats](#9-data-formats)
10. [API Interface](#10-api-interface)
11. [Regulatory Compliance](#11-regulatory-compliance)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for autonomous delivery drone systems, enabling safe, efficient, and reliable last-mile aerial delivery services that benefit communities worldwide.

### 1.2 Scope

The standard covers:
- Drone hardware and software specifications
- Flight dynamics and control algorithms
- Navigation and autonomous path planning
- Payload handling and delivery mechanisms
- UTM (Unmanned Traffic Management) integration
- Battery management and range optimization
- Safety protocols and emergency procedures
- Regulatory compliance frameworks

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize access to aerial delivery services, reduce carbon emissions from ground transport, improve delivery efficiency, and provide critical supply chain support to underserved communities.

### 1.4 Terminology

- **BVLOS**: Beyond Visual Line of Sight
- **GNSS**: Global Navigation Satellite System (GPS, GLONASS, Galileo, BeiDou)
- **IMU**: Inertial Measurement Unit
- **UTM**: Unmanned Traffic Management
- **GCS**: Ground Control Station
- **AGL**: Above Ground Level
- **MSL**: Mean Sea Level
- **ROA**: Return-On-Abort

---

## 2. Drone Classifications

### 2.1 Weight Classes

Drones are classified by Maximum Takeoff Weight (MTOW):

#### 2.1.1 Micro Class (0-2 kg)
```
MTOW: 0-2 kg
Payload: 0.1-0.5 kg
Range: 1-3 km
Flight Time: 10-20 min
Max Speed: 15 m/s
```

**Use Cases**: Documents, small parcels, mail

#### 2.1.2 Light Class (2-10 kg)
```
MTOW: 2-10 kg
Payload: 0.5-3 kg
Range: 3-10 km
Flight Time: 20-35 min
Max Speed: 20 m/s
```

**Use Cases**: Standard packages, e-commerce deliveries

#### 2.1.3 Medium Class (10-25 kg)
```
MTOW: 10-25 kg
Payload: 3-8 kg
Range: 10-30 km
Flight Time: 30-50 min
Max Speed: 25 m/s
```

**Use Cases**: Heavy packages, groceries, medical supplies

#### 2.1.4 Heavy Class (25-150 kg)
```
MTOW: 25-150 kg
Payload: 8-50 kg
Range: 30-100 km
Flight Time: 45-90 min
Max Speed: 30 m/s
```

**Use Cases**: Large cargo, disaster relief, rural delivery

### 2.2 Propulsion Types

- **Multi-Rotor**: 4-8 rotors, vertical takeoff/landing, high maneuverability
- **Fixed-Wing**: Airplane-style, long range, high efficiency
- **Hybrid VTOL**: Combines multi-rotor and fixed-wing benefits
- **Tilt-Rotor**: Rotating propulsion system for vertical/horizontal flight

### 2.3 Power Systems

- **Electric Battery**: Lithium-polymer, most common
- **Hybrid Gas-Electric**: Extended range for heavy class
- **Hydrogen Fuel Cell**: Zero emissions, long flight time
- **Solar-Assisted**: Extended flight time in daylight

---

## 3. Flight Control Systems

### 3.1 Flight Dynamics

#### 3.1.1 Lift Force
For multi-rotor systems:

```
L = n × T
```

Where:
- `L` = Total lift force (N)
- `n` = Number of rotors
- `T` = Thrust per rotor (N)

Thrust calculation:
```
T = CT × ρ × A × (ω × r)²
```

Where:
- `CT` = Thrust coefficient (0.01-0.015 for typical rotors)
- `ρ` = Air density (1.225 kg/m³ at sea level)
- `A` = Rotor disk area (m²)
- `ω` = Angular velocity (rad/s)
- `r` = Rotor radius (m)

#### 3.1.2 Hover Power Requirement

```
P = (mg)^(3/2) / √(2ρA)
```

Where:
- `P` = Power required to hover (W)
- `m` = Total mass (kg)
- `g` = Gravitational acceleration (9.81 m/s²)
- `ρ` = Air density (kg/m³)
- `A` = Total rotor disk area (m²)

#### 3.1.3 Drag Force

```
D = ½ × ρ × v² × A × CD
```

Where:
- `D` = Drag force (N)
- `v` = Forward velocity (m/s)
- `A` = Frontal area (m²)
- `CD` = Drag coefficient (1.0-1.5 for multi-rotor)

### 3.2 Stabilization Control

#### 3.2.1 PID Controller

Each axis (roll, pitch, yaw) uses a PID control loop:

```
u(t) = Kp × e(t) + Ki × ∫e(t)dt + Kd × de(t)/dt
```

Where:
- `u(t)` = Control output
- `e(t)` = Error (desired - actual)
- `Kp` = Proportional gain
- `Ki` = Integral gain
- `Kd` = Derivative gain

Typical PID values for multi-rotor:
```
Roll/Pitch:
  Kp = 4.5
  Ki = 0.02
  Kd = 0.18

Yaw:
  Kp = 3.0
  Ki = 0.01
  Kd = 0.10
```

#### 3.2.2 Sensor Fusion

Combine IMU and GNSS data using Extended Kalman Filter:

```
State Vector: [x, y, z, vx, vy, vz, φ, θ, ψ]
```

Where:
- `x, y, z` = Position (m)
- `vx, vy, vz` = Velocity (m/s)
- `φ, θ, ψ` = Roll, pitch, yaw (rad)

### 3.3 Flight Modes

1. **Manual**: Direct pilot control via remote
2. **Assisted**: Pilot control with stabilization
3. **Altitude Hold**: Automatic altitude maintenance
4. **Position Hold**: GPS position lock
5. **Waypoint**: Autonomous waypoint navigation
6. **Return-to-Home**: Automatic return on signal loss
7. **Land**: Automated precision landing

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
