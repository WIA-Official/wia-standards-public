# WIA-COMM-013 PHASE 1 — Data Format Specification

**Standard:** WIA-COMM-013
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-COMM-013: Data Center Specification v1.0

> **Standard ID:** WIA-COMM-013
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Data Center Infrastructure Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Tier Classification System](#2-tier-classification-system)
3. [Power Infrastructure](#3-power-infrastructure)
4. [Cooling Systems](#4-cooling-systems)
5. [Network Architecture](#5-network-architecture)
6. [Rack Design and Layout](#6-rack-design-and-layout)
7. [Physical Security](#7-physical-security)
8. [Fire Suppression](#8-fire-suppression)
9. [DCIM and Monitoring](#9-dcim-and-monitoring)
10. [Edge Data Centers](#10-edge-data-centers)
11. [Green Data Centers](#11-green-data-centers)
12. [Disaster Recovery](#12-disaster-recovery)
13. [Implementation Guidelines](#13-implementation-guidelines)
14. [References](#14-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for data center design, construction, and operation, covering physical infrastructure, power systems, cooling, networking, security, and management across all facility types from edge to hyperscale.

### 1.2 Scope

The standard covers:
- Tier I-IV classification and compliance requirements
- Electrical power distribution and redundancy
- Mechanical cooling and thermal management
- Network topology and connectivity
- Physical and logical security measures
- Monitoring, automation, and DCIM systems
- Edge computing and distributed facilities
- Sustainability and green energy practices

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Data centers are the foundation of modern digital infrastructure. This standard promotes efficient, reliable, and sustainable facilities that serve society while minimizing environmental impact.

### 1.4 Terminology

- **PUE**: Power Usage Effectiveness (Total Facility Power / IT Equipment Power)
- **CRAC**: Computer Room Air Conditioner
- **CRAH**: Computer Room Air Handler
- **UPS**: Uninterruptible Power Supply
- **PDU**: Power Distribution Unit
- **SLA**: Service Level Agreement
- **DCIM**: Data Center Infrastructure Management
- **kW**: Kilowatt (1,000 watts)
- **MW**: Megawatt (1,000 kilowatts)
- **N+1**: One redundant component beyond operational requirement
- **2N**: Fully redundant parallel systems

---

## 2. Tier Classification System

### 2.1 Tier Standards Overview

Data center tiers define reliability, redundancy, and availability levels:

```
┌──────────┬───────────────────────┬──────────────┬──────────────┐
│   Tier   │   Classification      │ Availability │ Downtime/yr  │
├──────────┼───────────────────────┼──────────────┼──────────────┤
│ Tier I   │ Basic Capacity        │   99.671%    │  28.8 hours  │
│ Tier II  │ Redundant Capacity    │   99.741%    │  22.0 hours  │
│ Tier III │ Concurrent Maintain.  │   99.982%    │   1.6 hours  │
│ Tier IV  │ Fault Tolerant        │   99.995%    │   0.4 hours  │
└──────────┴───────────────────────┴──────────────┴──────────────┘
```

### 2.2 Tier I: Basic Capacity

**Requirements:**
- Single, non-redundant distribution path
- Single capacity components
- No redundant components
- Vulnerable to planned and unplanned outages

**Infrastructure:**
- Single UPS module
- Single generator (N configuration)
- Single path for power and cooling
- No concurrent maintenance capability

**Use Cases:**
- Small businesses
- Development environments
- Non-critical workloads

### 2.3 Tier II: Redundant Capacity Components

**Requirements:**
- Single distribution path
- Redundant capacity components (N+1)
- Partial redundancy
- Vulnerable to planned outages

**Infrastructure:**
- Multiple UPS modules (N+1)
- Multiple generators (N+1)
- Single active power path
- Maintenance requires partial shutdown

**Use Cases:**
- SMB data centers
- Regional offices
- Low-criticality applications

### 2.4 Tier III: Concurrently Maintainable

**Requirements:**
- Multiple active distribution paths (1 active, 1 passive)
- N+1 redundancy
- Concurrent maintenance without downtime
- Protected against planned outages

**Infrastructure:**
- Dual-powered equipment
- Multiple UPS systems (N+1 per path)
- Multiple generators (N+1)
- Dual power distribution
- 72 hours of fuel for generators

**Use Cases:**
- Enterprise data centers
- E-commerce platforms
- Financial services
- Healthcare systems

### 2.5 Tier IV: Fault Tolerant

**Requirements:**
- Multiple active distribution paths (2N or 2N+1)
- Fault-tolerant capability
- Concurrent maintenance and fault tolerance
- Protected against planned and unplanned outages

**Infrastructure:**
- Fully redundant systems (2N+1)
- Multiple independent power paths
- Multiple independent cooling systems
- Compartmentalized architecture
- 96 hours of fuel storage minimum
- Automatic failover mechanisms

**Use Cases:**
- Mission-critical applications
- Government facilities
- Major financial institutions
- Large-scale cloud providers

---

## 3. Power Infrastructure

### 3.1 Electrical Distribution Architecture

```
Power Flow (Top to Bottom):

1. Utility Service
   ├─ Primary Feed (Grid A)
   └─ Secondary Feed (Grid B) [Tier III+]

2. Automatic Transfer Switch (ATS)
   ├─ Monitors utility power
   └─ Switches to generator on failure

3. Generator
   ├─ Diesel/Natural Gas
   ├─ Capacity: 125-150% of peak load
   └─ Startup time: <10 seconds

4. UPS (Uninterruptible Power Supply)
   ├─ Online double-conversion
   ├─ Battery runtime: 5-15 minutes
   └─ Efficiency: 94-98%

5. STS (Static Transfer Switch) [Tier III+]
   ├─ Sub-millisecond transfer
   └─ Dual UPS path synchronization

6. PDU (Power Distribution Unit)
   ├─ Primary distribution (480V)
   └─ Secondary distribution (208V/120V)

7. RPP (Remote Power Panel)
   └─ Zone-level distribution

8. Rack PDU
   └─ Server-level power (120V/208V)
```

### 3.2 UPS Sizing and Configuration

**Capacity Calculation:**
```
UPS Capacity (kVA) = (Total IT Load + Cooling + Lighting) / Efficiency / Power Factor × Safety Margin

Example:
IT Load: 1000 kW
Cooling: 200 kW (PUE 1.2)
Lighting: 20 kW
Efficiency: 0.96
Power Factor: 0.9
Safety Margin: 1.25

UPS Capacity = (1000 + 200 + 20) / 0.96 / 0.9 × 1.25
             = 1768 kVA
```

**UPS Topologies:**
1. **N Configuration**: Single UPS, no redundancy
2. **N+1 Configuration**: N UPS modules + 1 redundant
3. **2N Configuration**: Fully redundant parallel systems
4. **2N+1 Configuration**: Dual systems + extra redundancy

### 3.3 Generator Systems

**Specifications:**
- **Fuel Type**: Diesel (most common), natural gas, propane
- **Capacity**: 125-150% of total facility load
- **Startup Time**: <10 seconds from utility failure
- **Transfer Time**: <200ms with UPS bridge
- **Runtime**: 48-96 hours at full load
- **Maintenance**: Monthly tests, annual full-load tests

**Fuel Storage:**
```
Tier III: 72 hours minimum
Tier IV: 96 hours minimum
Hyperscale: 7-14 days typical

Fuel Calculation:
Fuel Consumption (gal/hr) = Load (kW) × Fuel Factor
(Diesel fuel factor: ~0.06-0.08 gal/hr/kW at full load)

Example (1 MW load, 96 hours):
1000 kW × 0.07 gal/hr/kW × 96 hrs = 6,720 gallons
```

### 3.4 Power Monitoring

**Key Metrics:**
- **Voltage**: Monitor phase voltage (480V/208V/120V)
- **Current**: Track amperage per circuit
- **Power Factor**: Maintain >0.95 for efficiency
- **Harmonics**: Keep THD <5%
- **Frequency**: 60 Hz ±0.1 Hz (US) or 50 Hz (EU)

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
