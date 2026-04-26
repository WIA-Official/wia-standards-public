# WIA-AUTO-006 PHASE 1 — Data Format Specification

**Standard:** WIA-AUTO-006
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-AUTO-006: Battery Management System Specification v1.0

> **Standard ID:** WIA-AUTO-006
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [BMS Architecture](#2-bms-architecture)
3. [State of Charge (SoC) Estimation](#3-state-of-charge-soc-estimation)
4. [State of Health (SoH) Monitoring](#4-state-of-health-soh-monitoring)
5. [Cell Balancing](#5-cell-balancing)
6. [Thermal Management](#6-thermal-management)
7. [Safety Monitoring and Protection](#7-safety-monitoring-and-protection)
8. [Battery Cell Modeling](#8-battery-cell-modeling)
9. [Data Formats](#9-data-formats)
10. [API Interface](#10-api-interface)
11. [Communication Protocols](#11-communication-protocols)
12. [Safety and Compliance](#12-safety-and-compliance)
13. [References](#13-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive Battery Management System (BMS) standard for electric vehicles, energy storage systems, and portable power applications. It provides algorithms, safety protocols, and interfaces for optimal battery performance, longevity, and safety.

### 1.2 Scope

The standard covers:
- State of Charge (SoC) and State of Health (SoH) estimation algorithms
- Cell balancing techniques (active and passive)
- Thermal management strategies
- Safety monitoring and protection mechanisms
- Battery modeling and characterization
- Communication interfaces and data formats

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to advance battery technology for the benefit of humanity, enabling the transition to sustainable energy and transportation while ensuring safety and reliability.

### 1.4 Terminology

- **SoC**: State of Charge - percentage of available charge
- **SoH**: State of Health - battery degradation indicator
- **DoD**: Depth of Discharge - percentage of capacity discharged
- **C-rate**: Current relative to capacity (1C = 1× capacity)
- **Cell**: Single electrochemical unit
- **Module**: Group of cells in series/parallel
- **Pack**: Complete battery assembly with BMS
- **OCV**: Open Circuit Voltage
- **ESR**: Equivalent Series Resistance

---

## 2. BMS Architecture

### 2.1 System Components

A compliant BMS consists of:

1. **Measurement Subsystem**
   - Voltage sensing (per cell)
   - Current sensing (pack level)
   - Temperature sensing (multiple points)
   - Isolation monitoring

2. **Processing Unit**
   - Microcontroller/DSP
   - Real-time operating system
   - Algorithm execution engine
   - Data logging

3. **Protection Subsystem**
   - Contactors/relays
   - Fuses and circuit breakers
   - Isolation switches
   - Emergency shutdown

4. **Communication Interface**
   - CAN bus (automotive)
   - I²C/SPI (internal)
   - Ethernet (stationary)
   - Wireless (diagnostic)

5. **Balancing Circuit**
   - Passive resistors
   - Active switches
   - Balancing drivers

### 2.2 Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     Battery Management System                │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │   Voltage    │    │   Current    │    │ Temperature  │  │
│  │   Sensing    │    │   Sensing    │    │   Sensing    │  │
│  │  (per cell)  │    │ (Hall/Shunt) │    │ (Thermal)    │  │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘  │
│         │                   │                   │            │
│         └───────────────────┴───────────────────┘            │
│                             │                                │
│                   ┌─────────▼─────────┐                      │
│                   │   Processing      │                      │
│                   │   - SoC/SoH       │                      │
│                   │   - Balancing     │                      │
│                   │   - Protection    │                      │
│                   └─────────┬─────────┘                      │
│                             │                                │
│         ┌───────────────────┼───────────────────┐            │
│         │                   │                   │            │
│    ┌────▼────┐      ┌───────▼──────┐    ┌──────▼──────┐    │
│    │ Cell    │      │  Protection  │    │ Communi-    │    │
│    │Balancing│      │   Switches   │    │  cation     │    │
│    └─────────┘      └──────────────┘    └─────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### 2.3 Functional Requirements

**FR-001**: Measure individual cell voltages with ±5mV accuracy
**FR-002**: Measure pack current with ±1% accuracy
**FR-003**: Measure temperatures with ±2°C accuracy
**FR-004**: Calculate SoC with ±5% accuracy
**FR-005**: Calculate SoH with ±10% accuracy
**FR-006**: Balance cells to within 10mV variance
**FR-007**: Respond to faults within 100ms
**FR-008**: Log data at minimum 1Hz rate
**FR-009**: Communicate status over CAN at 10Hz
**FR-010**: Support firmware updates over interface

---

## 3. State of Charge (SoC) Estimation

### 3.1 Coulomb Counting Method

The fundamental equation for coulomb counting:

```
SoC(t) = SoC(t₀) + (100/Q_n) × ∫[t₀,t] η(i) × i(τ) dτ
```

Where:
- `SoC(t)` = State of charge at time t (%)
- `SoC(t₀)` = Initial state of charge (%)
- `Q_n` = Nominal battery capacity (Ah)
- `η(i)` = Coulombic efficiency as function of current
- `i(τ)` = Current at time τ (A, negative for discharge)

#### 3.1.1 Discrete Implementation

For digital systems with sampling period Δt:

```
SoC(k) = SoC(k-1) + (100 × Δt)/(Q_n × 3600) × η × I(k)
```

Where:
- `k` = Sample index
- `Δt` = Sampling period (seconds)
- `I(k)` = Measured current (A)
- `3600` = Conversion factor (seconds per hour)

#### 3.1.2 Coulombic Efficiency

Temperature and current dependent efficiency:

```
η(T, I) = η₀ - k_T × (T - T_ref) - k_I × |I/Q_n|
```

Typical values:
- `η₀` = 0.98 (baseline efficiency)
- `k_T` = 0.001/°C (temperature coefficient)
- `k_I` = 0.02 (current coefficient)
- `T_ref` = 25°C (reference temperature)

### 3.2 Open Circuit Voltage (OCV) Method

#### 3.2.1 OCV-SoC Relationship

For lithium-ion batteries (NMC chemistry):

```
OCV(SoC) = a₀ + a₁×SoC + a₂×SoC² + a₃×SoC³ + a₄×SoC⁴
```

Example coefficients for NMC cells:
- `a₀` = 2.8
- `a₁` = 0.015
- `a₂` = -0.00008
- `a₃` = 0.0000002
- `a₄` = -0.0000000002

#### 3.2.2 Inverse Function

To calculate SoC from measured OCV:

```
SoC = f⁻¹(OCV_measured)
```

Implemented via lookup table or Newton-Raphson iteration:

```
SoC(n+1) = SoC(n) - [f(SoC(n)) - OCV_measured] / f'(SoC(n))
```

### 3.3 Extended Kalman Filter (EKF)

#### 3.3.1 Battery Model

First-order equivalent circuit model:

```
V_terminal = OCV(SoC) - I × R₀ - V_RC
```

RC network dynamics:
```
dV_RC/dt = -V_RC/(R₁×C₁) + I/C₁
```

#### 3.3.2 State Space Representation

State vector: `x = [SoC, V_RC]ᵀ`

State equation:
```
x(k+1) = A×x(k) + B×u(k) + w(k)
```

Measurement equation:
```
y(k) = h(x(k), u(k)) + v(k)
```

Where:
- `A` = State transition matrix
- `B` = Input matrix
- `u` = Input (current)
- `w` = Process noise
- `v` = Measurement noise
- `h()` = Nonlinear measurement function

#### 3.3.3 EKF Algorithm

**Prediction Step**:
```
x̂⁻(k) = A×x̂(k-1) + B×u(k)
P⁻(k) = A×P(k-1)×Aᵀ + Q
```

**Update Step**:
```
K(k) = P⁻(k)×Hᵀ × [H×P⁻(k)×Hᵀ + R]⁻¹
x̂(k) = x̂⁻(k) + K(k)×[y(k) - h(x̂⁻(k))]
P(k) = [I - K(k)×H]×P⁻(k)
```

Where:
- `K` = Kalman gain
- `P` = Error covariance matrix
- `Q` = Process noise covariance
- `R` = Measurement noise covariance
- `H` = Jacobian of h()

### 3.4 Multi-Method Fusion

Weighted combination of methods:

```
SoC_final = w₁×SoC_coulomb + w₂×SoC_OCV + w₃×SoC_EKF
```

Where `w₁ + w₂ + w₃ = 1`

Adaptive weights based on confidence:
```
w_i = (1/σ²_i) / Σ(1/σ²_j)
```

Where `σ²` is the estimated variance of each method.

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
