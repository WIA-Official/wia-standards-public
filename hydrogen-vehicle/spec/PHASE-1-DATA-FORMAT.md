# WIA-AUTO-007 PHASE 1 — Data Format Specification

**Standard:** WIA-AUTO-007
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-AUTO-007: Hydrogen Vehicle Specification v1.0

> **Standard ID:** WIA-AUTO-007
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Fuel Cell Technology](#2-fuel-cell-technology)
3. [Hydrogen Storage Systems](#3-hydrogen-storage-systems)
4. [Power Electronics](#4-power-electronics)
5. [Electric Motor Systems](#5-electric-motor-systems)
6. [Refueling Infrastructure](#6-refueling-infrastructure)
7. [Efficiency Calculations](#7-efficiency-calculations)
8. [Data Formats](#8-data-formats)
9. [API Interface](#9-api-interface)
10. [Safety Protocols](#10-safety-protocols)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework for hydrogen fuel cell vehicles (FCVs), based on proven automotive engineering principles, electrochemical science, and hydrogen energy systems.

### 1.2 Scope

The standard covers:
- Proton Exchange Membrane Fuel Cell (PEMFC) technology
- Solid Oxide Fuel Cell (SOFC) technology
- High-pressure hydrogen storage (Type III & IV tanks)
- Power electronics and motor control systems
- Refueling protocols and infrastructure
- Efficiency metrics and calculations
- Safety systems and protocols

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to accelerate the adoption of zero-emission hydrogen vehicles that benefit all of humanity through clean transportation while ensuring safety, efficiency, and interoperability.

### 1.4 Terminology

- **PEMFC**: Proton Exchange Membrane Fuel Cell
- **SOFC**: Solid Oxide Fuel Cell
- **LHV**: Lower Heating Value (120 MJ/kg for H2)
- **HHV**: Higher Heating Value (142 MJ/kg for H2)
- **H70**: 700 bar hydrogen storage standard
- **H35**: 350 bar hydrogen storage standard
- **SOC**: State of Charge
- **BOL**: Beginning of Life
- **EOL**: End of Life

---

## 2. Fuel Cell Technology

### 2.1 Proton Exchange Membrane Fuel Cell (PEMFC)

#### 2.1.1 Operating Principle

The PEMFC operates through electrochemical reactions:

**Anode Reaction:**
```
H2 → 2H⁺ + 2e⁻
```

**Cathode Reaction:**
```
½O2 + 2H⁺ + 2e⁻ → H2O
```

**Overall Reaction:**
```
H2 + ½O2 → H2O + Energy (Electrical + Heat)
```

#### 2.1.2 Cell Voltage

The theoretical open-circuit voltage:

```
E_cell = E⁰ + (RT / 2F) × ln[(P_H2 × √P_O2) / P_H2O]
```

Where:
- `E⁰` = Standard cell potential (1.23 V at 25°C)
- `R` = Universal gas constant (8.314 J/mol·K)
- `T` = Temperature (K)
- `F` = Faraday constant (96,485 C/mol)
- `P_H2`, `P_O2`, `P_H2O` = Partial pressures

Typical operating voltage: 0.6 - 0.7 V per cell

#### 2.1.3 Stack Power Output

```
P_stack = N_cells × V_cell × I_stack
```

Where:
- `P_stack` = Stack power output (W)
- `N_cells` = Number of cells in series
- `V_cell` = Average cell voltage (V)
- `I_stack` = Stack current (A)

#### 2.1.4 PEMFC Efficiency

```
η_PEMFC = (V_cell / E_tn) × 100%
```

Where:
- `η_PEMFC` = Fuel cell efficiency (%)
- `V_cell` = Actual cell voltage (V)
- `E_tn` = Thermoneutral voltage (1.48 V at 25°C)

Typical PEMFC efficiency: 50-60% (electrical)

#### 2.1.5 Operating Conditions

| Parameter | Minimum | Optimal | Maximum |
|-----------|---------|---------|---------|
| Temperature | 60°C | 80°C | 90°C |
| Pressure | 1.5 bar | 2.5 bar | 3.0 bar |
| Humidity | 40% RH | 80% RH | 100% RH |
| Current Density | 0.2 A/cm² | 0.8 A/cm² | 1.5 A/cm² |

### 2.2 Solid Oxide Fuel Cell (SOFC)

#### 2.2.1 Operating Principle

SOFC operates at high temperatures (600-1000°C) with oxygen ion conduction.

**Anode Reaction:**
```
H2 + O²⁻ → H2O + 2e⁻
```

**Cathode Reaction:**
```
½O2 + 2e⁻ → O²⁻
```

#### 2.2.2 SOFC Efficiency

SOFC achieves higher electrical efficiency: 60-70%

Combined heat and power (CHP): up to 90% total efficiency

### 2.3 Fuel Cell Stack Design

#### 2.3.1 Stack Configuration

- **Bipolar Plates**: Graphite or metallic plates for current collection
- **Membrane Electrode Assembly (MEA)**: Catalyst-coated membrane
- **Gas Diffusion Layer (GDL)**: Porous carbon paper/cloth
- **Cooling Channels**: Integrated thermal management

#### 2.3.2 Stack Sizing

```
A_cell = P_target / (N_cells × V_cell × J_avg)
```

Where:
- `A_cell` = Active cell area (cm²)
- `P_target` = Target power output (W)
- `J_avg` = Average current density (A/cm²)

#### 2.3.3 Power Density

Typical power density: 2-4 kW/L (stack level)

Target: >4 kW/L for automotive applications

---

## 3. Hydrogen Storage Systems

### 3.1 Storage Tank Types

#### 3.1.1 Type III Tanks

- **Construction**: Aluminum liner with carbon fiber composite overwrap
- **Operating Pressure**: 350 bar (H35)
- **Weight**: Higher than Type IV
- **Cost**: Lower than Type IV
- **Applications**: Commercial vehicles, buses

#### 3.1.2 Type IV Tanks

- **Construction**: Polymer liner (HDPE) with carbon fiber composite overwrap
- **Operating Pressure**: 700 bar (H70)
- **Weight**: Lighter than Type III
- **Cost**: Higher than Type III
- **Applications**: Passenger vehicles

### 3.2 Storage Capacity

#### 3.2.1 Hydrogen Mass Calculation

```
m_H2 = (P × V × MW_H2) / (Z × R × T)
```

Where:
- `m_H2` = Hydrogen mass (kg)
- `P` = Pressure (Pa)
- `V` = Tank volume (m³)
- `MW_H2` = Molecular weight (2.016 g/mol)
- `Z` = Compressibility factor
- `R` = Gas constant (8.314 J/mol·K)
- `T` = Temperature (K)

#### 3.2.2 Gravimetric Density

```
ρ_grav = m_H2 / (m_H2 + m_tank) × 100%
```

Where:
- `ρ_grav` = Gravimetric density (%)
- `m_tank` = Tank system weight (kg)

Target: >5.5% (DOE 2025 target)


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
