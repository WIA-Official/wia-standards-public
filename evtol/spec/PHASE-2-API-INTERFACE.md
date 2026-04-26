# WIA-SPACE-019 PHASE 2 — API Interface

**Standard:** WIA-SPACE-019 eVTOL
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `evtol-v1.0.md` §4 (Propulsion System
Requirements) and `WIA-SPACE-019-v1.0.md` §6 (Flight Control Systems)

This document defines the REST and message-bus APIs for telemetry,
operational state, and mission control between vehicle, vertiport, and
ATM provider. The machine-readable definition lives under
`api/openapi.yaml`. Authentication uses OAuth 2.1 (RFC 9700) with
mTLS for fleet operator and vertiport callers.

---

## 4. Propulsion System Requirements

### 4.1 Electric Motors

#### 4.1.1 Specifications
- **Type:** Brushless Permanent Magnet Synchronous Motors (PMSM)
- **Efficiency:** Minimum 93% at rated power
- **Power Density:** Minimum 4 kW/kg
- **Cooling:** Active liquid or forced-air cooling required
- **Redundancy:** Each motor operates independently
- **Operating Voltage:** 400-800V DC typical

#### 4.1.2 Testing Requirements
- Continuous operation at rated power for 30 minutes minimum
- Peak power operation (110% rated) for 5 minutes
- Thermal cycling validation
- Vibration testing per DO-160
- Electromagnetic compatibility (EMC) per DO-160

### 4.2 Battery Systems

#### 4.2.1 Energy Density
- **Minimum:** 180 Wh/kg at pack level
- **Target:** 220-250 Wh/kg
- **Future (solid-state):** 400+ Wh/kg

#### 4.2.2 Power Density
- Capable of 3C continuous discharge
- Peak discharge 5C for takeoff/landing (2 minutes)

#### 4.2.3 Safety Requirements
- Thermal runaway propagation barriers between cells
- Battery Management System (BMS) monitoring all cells
- Dual-redundant BMS architecture
- Automatic fire suppression system
- Crash-resistant housing with energy absorption

#### 4.2.4 Cycle Life
- Minimum 2,000 cycles to 80% capacity
- Calendar life: 10 years minimum
- Operating temperature: -20°C to 60°C

### 4.3 Power Management

- High-voltage DC architecture (400-800V)
- Redundant power distribution buses
- Motor controllers (inverters) with 98%+ efficiency
- Emergency power reserves: 20% minimum battery capacity
- Battery state monitoring at 10 Hz minimum

---


## 6. Flight Control Systems

### 6.1 Fly-By-Wire (FBW)

#### 6.1.1 Architecture
- **Redundancy:** Dual or triple flight control computers
- **Cross-Check:** Majority voting for fault detection
- **Diversity:** Different processor manufacturers to prevent common-mode failures
- **Update Rate:** Minimum 100 Hz control loop

#### 6.1.2 Flight Control Modes
- **Attitude Hold:** Maintain orientation when controls released
- **Position Hold:** GPS-based hover at fixed location
- **Altitude Hold:** Barometric altitude maintenance
- **Heading Hold:** Magnetic compass direction holding
- **Velocity Control:** Commanded speed achievement
- **Path Following:** Waypoint navigation

### 6.2 Sensor Suite

#### 6.2.1 Required Sensors
- **IMU:** 6-axis (3-axis accelerometer + 3-axis gyroscope), dual redundant
- **GPS/GNSS:** Dual receiver, RTK capability for precision
- **Barometer:** Dual altimeter, static pressure compensation
- **Magnetometer:** Dual compass, calibration capability
- **Airspeed:** Pitot-static system for forward flight

#### 6.2.2 Autonomous Flight Sensors
- **LiDAR:** 360-degree scanning, 100m+ range
- **Cameras:** Stereo vision, object detection (AI-based)
- **Radar:** Weather radar, collision avoidance
- **ADS-B:** Traffic awareness, TCAS functionality
- **Ultrasonic:** Ground proximity during landing

### 6.3 Autonomous Flight Levels

#### 6.3.1 Level Classification
- **Level 0:** Fully manual, basic stabilization
- **Level 1:** Assisted automation (altitude/speed hold)
- **Level 2:** Partial automation (auto takeoff/landing)
- **Level 3:** Conditional automation (specific conditions only)
- **Level 4:** High automation (pilot optional, ground monitoring)
- **Level 5:** Full automation (no human intervention)

#### 6.3.2 Requirements by Level
- **Level 2-3:** Pilot always present and monitoring
- **Level 4:** Remote pilot/operator monitoring multiple aircraft
- **Level 5:** AI-only operation with system redundancy


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
