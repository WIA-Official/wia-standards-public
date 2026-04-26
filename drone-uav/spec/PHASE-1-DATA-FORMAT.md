# WIA-SPACE-017 PHASE 1 — Data Format Specification

**Standard:** WIA-SPACE-017 Drone / UAV
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `drone-uav-v1.0.md` §3 (Technical
Specifications) and `WIA-SPACE-017-v1.0.md` §6 (Hardware Specifications)

This document defines the canonical data structures exchanged across
WIA-SPACE-017 implementations: aircraft identification, payload
descriptors, telemetry framing, and flight-plan envelopes. Schemas are
expressed in JSON Schema 2020-12 and are stable for the lifetime of
this PHASE.

References:
- ICAO Annex 6 (operational standards)
- IEC 62443 (industrial cybersecurity baseline)
- ISO/IEC 27001:2022 (information security management)
- OpenAPI 3.1, JSON Schema 2020-12

---

## 3. Technical Specifications

### 3.1 Airframe Requirements

**Materials:**
- Carbon fiber composites for performance applications
- Engineering plastics (ABS, PC) for consumer drones
- Aluminum alloys for precision components

**Design Criteria:**
- Strength-to-weight ratio optimization
- Vibration isolation for sensitive components
- Structural integrity under operational loads
- Aerodynamic efficiency

### 3.2 Propulsion Systems

**Brushless Motors:**
- KV Rating: 300-3000 KV depending on application
- Efficiency: Minimum 85%
- Current handling: Appropriate to motor size and application
- Temperature management

**Electronic Speed Controllers (ESCs):**
- Current rating: 20-60A continuous typical
- Firmware: BLHeli_32, BLHeli_S, or equivalent
- Telemetry capability recommended
- Active braking and motor timing adjustment

**Propellers:**
- Material selection based on application (plastic, carbon fiber, composite)
- Diameter and pitch optimization for efficiency
- Balance within 0.5g-cm for smooth operation

### 3.3 Power Systems

**Battery Technology:**
- LiPo or LiHV chemistry for most applications
- Energy density: Minimum 150 Wh/kg
- C-rating appropriate to application (20C-100C)
- Battery Management System (BMS) required for intelligent batteries

**Safety Requirements:**
- Over-charge protection (max 4.2V/cell for LiPo, 4.35V/cell for LiHV)
- Over-discharge protection (min 3.0V/cell)
- Temperature monitoring and cutoff
- Cell balancing during charge
- Short-circuit protection

### 3.4 Flight Control Systems

**Hardware Requirements:**
- 32-bit ARM processor (Cortex-M4 or better)
- 6-axis minimum IMU (3-axis gyro + 3-axis accelerometer)
- Barometric altimeter (±1m accuracy)
- Magnetometer for heading reference
- GPS/GNSS receiver (multi-constellation preferred)

**Software Requirements:**
- PID control loops running at minimum 500Hz for rate control
- Failsafe mechanisms for signal loss, low battery, GPS loss
- Geofencing capability
- Return-to-home functionality
- Mission planning and waypoint navigation
- Compliance with MAVLink or equivalent telemetry protocol

---


## 6. Hardware Specifications

### 6.1 Frame Requirements

**Materials**: Approved materials include:
- Carbon fiber (preferred for strength-to-weight ratio)
- Aluminum alloy (6061-T6 or equivalent)
- Engineering plastics (ABS, PC, nylon)
- Composite materials (fiberglass, aramid)

**Structural Requirements**:
- Minimum safety factor: 2.0 for static loads
- Vibration resistance: 5-2000 Hz
- Operating temperature: -20°C to +50°C
- Crash resistance: Withstand 5G impact

### 6.2 Propulsion System

**Motors**:
- Type: Brushless DC motors (BLDC) required for drones > 250g
- Efficiency: Minimum 80% at rated load
- Temperature rating: -20°C to +100°C
- Lifetime: Minimum 500 flight hours

**ESCs**:
- Protocol support: DShot300/600 minimum
- Current rating: 125% of maximum motor current
- Thermal protection: Auto-shutdown at 100°C
- Firmware: User-updateable (BLHeli_S/BLHeli_32 or equivalent)

**Propellers**:
- Material: Reinforced polymer or carbon fiber
- Balance: < 0.5 gram-cm imbalance
- Inspection: Visual inspection every 50 flight hours

### 6.3 Battery System

**Battery Type**: LiPo (Lithium Polymer) or LiHV (High Voltage) approved

**Requirements**:
- Cell voltage monitoring per cell
- Over-discharge protection (< 3.0V/cell)
- Over-charge protection (> 4.2V/cell for standard LiPo)
- Short-circuit protection
- Temperature monitoring
- Fire-resistant storage bag for transport

**C-Rating**: Minimum discharge rate to support maximum current draw + 20% margin

### 6.4 Flight Controller

**Processor Requirements**:
- Minimum: 32-bit ARM processor, 168MHz (e.g., STM32 F4)
- Recommended: 480MHz (e.g., STM32 H7)

**Sensors**:
- 6-axis IMU (3-axis gyro + 3-axis accelerometer) minimum
- 9-axis IMU (+ magnetometer) recommended
- Barometer for altitude estimation
- Optional: Dual IMU for redundancy

**Update Rate**:
- Gyro sampling: Minimum 4kHz, recommended 8kHz+
- PID loop frequency: Minimum 2kHz, recommended 4kHz+

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
