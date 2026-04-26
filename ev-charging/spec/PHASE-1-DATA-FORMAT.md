# WIA-AUTO-005 PHASE 1 — Data Format Specification

**Standard:** WIA-AUTO-005
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-AUTO-005: EV Charging Specification v1.0

> **Standard ID:** WIA-AUTO-005
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Charging Levels](#2-charging-levels)
3. [Connector Standards](#3-connector-standards)
4. [Communication Protocols](#4-communication-protocols)
5. [Smart Charging](#5-smart-charging)
6. [Vehicle-to-Grid (V2G)](#6-vehicle-to-grid-v2g)
7. [Billing and Payment](#7-billing-and-payment)
8. [Data Formats](#8-data-formats)
9. [API Interface](#9-api-interface)
10. [Safety Protocols](#10-safety-protocols)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines standardized protocols, interfaces, and requirements for electric vehicle charging infrastructure to ensure interoperability, safety, and optimal user experience across all charging networks globally.

### 1.2 Scope

The standard covers:
- AC and DC charging technologies (Level 1, 2, and 3)
- Physical connector specifications
- Communication protocols (OCPP, ISO 15118)
- Smart charging and load management
- V2G bidirectional charging
- Payment and billing systems
- Safety and security requirements

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard accelerates the global transition to electric mobility by creating seamless, interoperable charging infrastructure that benefits all of humanity through reduced emissions, improved air quality, and sustainable transportation.

### 1.4 Terminology

- **BEV**: Battery Electric Vehicle - fully electric vehicle
- **PHEV**: Plug-in Hybrid Electric Vehicle
- **SOC**: State of Charge - battery charge level (0-100%)
- **SOH**: State of Health - battery condition indicator
- **EVSE**: Electric Vehicle Supply Equipment - charging station
- **OCPP**: Open Charge Point Protocol
- **V2G**: Vehicle-to-Grid - bidirectional energy transfer
- **kW**: Kilowatt - unit of power
- **kWh**: Kilowatt-hour - unit of energy

---

## 2. Charging Levels

### 2.1 Level 1 Charging (AC)

**Specifications:**
```
Voltage: 120V AC (North America) / 230V AC (Europe)
Current: 12-16A
Power: 1.4-1.9 kW (NA) / 2.3-3.7 kW (EU)
Connector: Standard household outlet (NEMA 5-15, Schuko)
```

**Characteristics:**
- Slowest charging method
- No special equipment required
- Typical use: Overnight home charging
- Charge rate: 3-5 miles of range per hour

**Charging Time (Example: 60 kWh battery, 20% → 80%)**
```
Energy needed = 60 × (0.8 - 0.2) = 36 kWh
Time = 36 / 1.4 = 25.7 hours (worst case)
Time = 36 / 3.7 = 9.7 hours (best case, EU)
```

### 2.2 Level 2 Charging (AC)

**Specifications:**
```
Voltage: 208-240V AC
Current: 12-80A
Power: 3.3-19.2 kW
Connector: SAE J1772 (NA), IEC 62196 Type 2 (EU)
```

**Power Levels:**
- **3.3 kW**: 14A @ 240V (portable chargers)
- **7.2 kW**: 30A @ 240V (home wallbox)
- **11 kW**: 48A @ 240V (commercial)
- **19.2 kW**: 80A @ 240V (high-power commercial)

**Charging Time (60 kWh battery, 20% → 80%)**
```
At 7.2 kW: 36 / 7.2 = 5 hours
At 11 kW: 36 / 11 = 3.3 hours
```

**Installation Requirements:**
- Dedicated 240V circuit
- Proper grounding
- GFCI protection
- Load management system (optional)

### 2.3 Level 3 Charging (DC Fast Charging)

**Specifications:**
```
Voltage: 200-1000V DC
Current: 0-500A
Power: 50-350 kW
Connector: CCS, CHAdeMO, Tesla Supercharger
```

**Power Tiers:**
- **50 kW**: Entry-level DC fast charging
- **150 kW**: Standard highway fast charging
- **250 kW**: High-power charging (Tesla V3, Electrify America)
- **350 kW**: Ultra-fast charging (Ionity, future systems)

**Charging Time (60 kWh battery, 20% → 80%)**
```
At 50 kW: 36 / 50 = 0.72 hours (43 minutes)
At 150 kW: 36 / 150 = 0.24 hours (14.4 minutes)
At 350 kW: 36 / 350 = 0.10 hours (6 minutes)*
```
*Note: Actual charging limited by battery acceptance rate

**Charging Curve:**

DC fast charging follows a tapered curve:
```
0-20% SOC: Maximum power (350 kW)
20-50% SOC: High power (300-350 kW)
50-80% SOC: Reduced power (150-250 kW)
80-100% SOC: Low power (50-100 kW)
```

---

## 3. Connector Standards

### 3.1 CCS (Combined Charging System)

**Type 1 (CCS1)**: North America
```
AC Pins: SAE J1772 (5-pin)
DC Pins: 2 additional high-power pins
Max AC Power: 19.2 kW
Max DC Power: 350 kW
Voltage: Up to 920V DC
Current: Up to 500A
```

**Type 2 (CCS2)**: Europe, Rest of World
```
AC Pins: IEC 62196 Type 2 (7-pin)
DC Pins: 2 additional high-power pins
Max AC Power: 43 kW (3-phase)
Max DC Power: 350 kW
Voltage: Up to 920V DC
Current: Up to 500A
```

**Pin Configuration:**
1. Proximity detection
2. Control pilot (communication)
3. AC Phase 1, 2, 3 (Type 2 only)
4. Neutral
5. Ground/Earth
6. DC+ (high voltage positive)
7. DC- (high voltage negative)

### 3.2 CHAdeMO

**Specifications:**
```
Standard: CHAdeMO 1.0, 2.0, 3.0
Max Power: 62.5 kW (v1.0), 200 kW (v2.0), 400 kW (v3.0)
Voltage: 50-500V DC (v1.0), up to 1000V (v3.0)
Current: Up to 400A
Connector: Dedicated DC connector
Communication: CAN bus
```

**Features:**
- V2G capable from v1.0
- Bidirectional charging
- Popular in Japan and Asia
- Used by Nissan, Mitsubishi

### 3.3 Tesla Supercharger

**Specifications:**
```
Connector: Proprietary (North America), CCS2 (Europe)
V2 Supercharger: Up to 150 kW
V3 Supercharger: Up to 250 kW
V4 Supercharger: Up to 350 kW (future)
Voltage: 50-500V DC
Communication: Proprietary + CAN
```

**Features:**
- Liquid-cooled cables (V3+)
- Plug & Charge (no authentication needed)
- Dynamic power sharing
- Battery preconditioning

### 3.4 GB/T (China Standard)

**Specifications:**
```
AC: GB/T 20234.2
DC: GB/T 20234.3
Max AC Power: 43 kW
Max DC Power: 237.5 kW
Voltage: Up to 950V DC
Current: Up to 250A
```

**Adoption:**
- Mandatory in China
- Used by all Chinese EVs
- Growing international adoption

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
