# WIA-DEF-006 PHASE 1 — Data Format Specification

**Standard:** WIA-DEF-006
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-DEF-006: Electronic Warfare Specification v1.0

> **Standard ID:** WIA-DEF-006
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Defense & Security Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Electronic Warfare Fundamentals](#2-electronic-warfare-fundamentals)
3. [Electronic Attack (EA)](#3-electronic-attack-ea)
4. [Electronic Protection (EP)](#4-electronic-protection-ep)
5. [Electronic Support (ES)](#5-electronic-support-es)
6. [Spectrum Management](#6-spectrum-management)
7. [Signal Intelligence](#7-signal-intelligence)
8. [Jamming Techniques](#8-jamming-techniques)
9. [Countermeasures](#9-countermeasures)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [Safety Protocols](#11-safety-protocols)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for Electronic Warfare (EW) operations in modern defense systems, covering the full electromagnetic spectrum from HF through EHF frequencies.

### 1.2 Scope

The standard covers:
- Electronic attack operations and techniques
- Electronic protection and defensive measures
- Electronic support and signal intelligence
- Spectrum management and coordination
- Jamming and countermeasure systems
- Safety and regulatory compliance

### 1.3 Philosophy

**弘익人間 (Benefit All Humanity)** - This standard aims to provide defensive electronic warfare capabilities that protect civilian infrastructure, minimize collateral effects, and ensure compliance with international humanitarian law.

### 1.4 Terminology

- **EW**: Electronic Warfare - Military action involving electromagnetic spectrum
- **EA**: Electronic Attack - Offensive EW operations
- **EP**: Electronic Protection - Defensive EW operations
- **ES**: Electronic Support - Intelligence gathering and threat warning
- **SIGINT**: Signals Intelligence - Intelligence from electromagnetic emissions
- **ECM**: Electronic Countermeasures - Techniques to disrupt enemy systems
- **ECCM**: Electronic Counter-Countermeasures - Techniques to resist ECM
- **EMI**: Electromagnetic Interference - Disruption of electronic systems
- **EMCON**: Emission Control - Restriction of electromagnetic emissions

---

## 2. Electronic Warfare Fundamentals

### 2.1 The Electromagnetic Spectrum

The EW spectrum is divided into frequency bands:

```
Band  | Frequency      | Wavelength    | Applications
------|----------------|---------------|------------------
HF    | 3-30 MHz       | 100-10 m      | Long-range comms
VHF   | 30-300 MHz     | 10-1 m        | Air traffic, FM
UHF   | 300-3000 MHz   | 1-0.1 m       | Radar, GPS, TV
SHF   | 3-30 GHz       | 10-1 cm       | Satellite, radar
EHF   | 30-300 GHz     | 10-1 mm       | Mil-sat, radar
```

### 2.2 Fundamental Equations

#### 2.2.1 Friis Transmission Equation

```
Pr = Pt × Gt × Gr × (λ / 4πR)²
```

Where:
- `Pr` = Received power (watts)
- `Pt` = Transmitted power (watts)
- `Gt` = Transmitter antenna gain
- `Gr` = Receiver antenna gain
- `λ` = Wavelength (meters)
- `R` = Distance (meters)

#### 2.2.2 Path Loss

```
PL = 20 log₁₀(d) + 20 log₁₀(f) + 32.45
```

Where:
- `PL` = Path loss (dB)
- `d` = Distance (km)
- `f` = Frequency (MHz)

#### 2.2.3 Link Budget

```
Pr (dBm) = Pt (dBm) + Gt (dBi) + Gr (dBi) - PL (dB) - L (dB)
```

Where:
- `L` = Additional losses (cables, atmospheric, etc.)

### 2.3 Power and Energy

#### 2.3.1 Effective Isotropic Radiated Power (EIRP)

```
EIRP = Pt × Gt
```

Or in dB:
```
EIRP (dBm) = Pt (dBm) + Gt (dBi)
```

#### 2.3.2 Power Flux Density

```
S = EIRP / (4πR²)
```

Where:
- `S` = Power flux density (W/m²)
- `R` = Distance from source (meters)

---

## 3. Electronic Attack (EA)

### 3.1 Jamming Fundamentals

Jamming is the deliberate radiation or reflection of electromagnetic energy to disrupt enemy use of electronic systems.

#### 3.1.1 Jamming-to-Signal Ratio

```
J/S = (Pj × Gj × Gr) / (Ps × Gs × Gr × (Rs/Rj)²)
```

Simplified:
```
J/S = (Pj × Gj) / (Ps × Gs) × (Rs/Rj)²
```

Where:
- `J/S` = Jamming-to-Signal ratio
- `Pj` = Jammer power (watts)
- `Gj` = Jammer antenna gain
- `Ps` = Signal power (watts)
- `Gs` = Signal antenna gain
- `Rs` = Distance to signal source
- `Rj` = Distance to jammer

#### 3.1.2 Effective Radiated Power (ERP) Requirement

For effective jamming, typically require:
```
J/S ≥ 10 dB (for noise jamming)
J/S ≥ 20 dB (for deception jamming)
```

### 3.2 Jamming Techniques

#### 3.2.1 Noise Jamming

- **Barrage Noise**: Wideband noise across entire frequency range
- **Spot Noise**: Narrowband noise at specific frequency
- **Swept Noise**: Noise swept across frequency range

Energy requirement:
```
Ej = Pj × BW × t
```

Where:
- `Ej` = Jamming energy (joules)
- `BW` = Bandwidth (Hz)
- `t` = Time (seconds)

#### 3.2.2 Deception Jamming

- **False Target Generation**: Create fake radar returns
- **Range Gate Pull-Off (RGPO)**: Deceive tracking radars
- **Velocity Gate Pull-Off (VGPO)**: Spoof Doppler tracking

Deception signal:
```
Sd(t) = A × cos(2πft + φ + δ(t))
```

Where:
- `A` = Amplitude
- `f` = Frequency
- `φ` = Phase
- `δ(t)` = Time-varying deception parameter

#### 3.2.3 Protocol Jamming

Target communication protocols:
- **Message Injection**: Insert false messages
- **Timing Disruption**: Interfere with sync signals
- **Control Channel Jamming**: Disrupt command channels

### 3.3 Specific Attack Types

#### 3.3.1 Communications Jamming

Target parameters:


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
