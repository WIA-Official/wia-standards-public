# WIA-SPACE-019 PHASE 3 — Protocol

**Standard:** WIA-SPACE-019 eVTOL
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `evtol-v1.0.md` §6 (Autonomous Flight
Systems) and `WIA-SPACE-019-v1.0.md` §5 (Electric Propulsion Systems)

This document defines the protocols between aircraft autonomy
subsystem, propulsion controller, and ground operator. References
ARP-4754A and DO-178C for the development assurance process; protocol
schemas reflect the message types defined in those documents but do
not redefine them.

References:
- ARP-4754A (development of civil aircraft and systems)
- DO-178C (software considerations)
- DO-254 (hardware considerations)
- ASTM F3338-21 (electric propulsion units)

---

## 6. Autonomous Flight Systems

### 6.1 Sensor Requirements

#### 6.1.1 Navigation Sensors
- **GPS/GNSS:** Triple-redundant with RTK capability
- **IMU:** 6 independent units minimum
- **Air Data:** 6 independent pitot-static systems
- **Magnetometer:** Triple-redundant

#### 6.1.2 Perception Sensors
- **Radar:** 360° coverage, 2 km range minimum
- **LiDAR:** 3 units, 300m range minimum
- **Cameras:** 12 cameras, 360° visual coverage
- **Ultrasonic:** 8 sensors for proximity detection

### 6.2 Flight Control Architecture

- Triple-redundant flight computers
- Diverse software implementations
- 100 Hz control loop minimum
- Automatic failure detection and isolation
- Envelope protection (prevent departure from safe flight regime)

### 6.3 Detect and Avoid

- ADS-B In/Out for cooperative aircraft
- Non-cooperative detect and avoid using radar, LiDAR, cameras
- Collision avoidance maneuvers within 5 seconds of detection
- Minimum separation: 500 ft horizontal, 250 ft vertical

### 6.4 Software Certification

- Flight-critical software: DO-178C Design Assurance Level A (DAL-A)
- Mission-critical software: DAL-B
- Non-critical software: DAL-C or lower
- Formal verification for safety-critical algorithms

---


## 5. Electric Propulsion Systems

### 5.1 Electric Motors

#### 5.1.1 Motor Types
- **Brushless DC (BLDC):** Primary choice - 90-95% efficiency
- **Permanent Magnet Synchronous Motor (PMSM):** High precision applications
- **Power Density:** Target 10-20 kW/kg
- **Efficiency:** Minimum 90% at rated power, 85%+ at partial load

#### 5.1.2 Motor Requirements
- **Redundancy:** N+1 or N+2 configuration
- **Cooling:** Liquid cooling for motors >50 kW
- **Control:** Independent ESC for each motor
- **Monitoring:** Real-time temperature, current, RPM, vibration
- **Lifespan:** Minimum 10,000 hours

### 5.2 Battery Systems

#### 5.2.1 Current Technology (2025-2027)
- **Chemistry:** Lithium-ion NMC or NCA
- **Energy Density:** 250-300 Wh/kg (pack level)
- **Power Density:** Minimum 1,000 W/kg (5C discharge)
- **Cycle Life:** Minimum 3,000 cycles to 80% capacity
- **Safety:** Pass nail penetration, crush, thermal runaway tests

#### 5.2.2 Near-Term Technology (2028-2030)
- **Chemistry:** High-energy NMC, early solid-state
- **Energy Density:** 350-400 Wh/kg
- **Charging:** 80% in 15-30 minutes
- **Operating Temperature:** -20°C to +60°C

#### 5.2.3 Future Technology (2031+)
- **Solid-State:** 400-500 Wh/kg, zero fire risk
- **Lithium-Sulfur:** 600-800 Wh/kg
- **Lithium-Air:** 1,000+ Wh/kg (long-term)

### 5.3 Battery Management System (BMS)

#### 5.3.1 Core Functions
- **Cell Monitoring:** Individual cell voltage, temperature, current
- **SOC Estimation:** ±3% accuracy
- **SOH Estimation:** Remaining useful life prediction
- **Protection:** Overcharge, over-discharge, overcurrent, thermal protection
- **Balancing:** Active or passive cell balancing

#### 5.3.2 Safety Requirements
- **Thermal Isolation:** Ceramic/aerogel between cells
- **Fire Suppression:** Inert gas or coolant injection system
- **Multiple Fuses:** Module and pack level overcurrent protection
- **Real-Time Monitoring:** Temperature, voltage, swelling detection

### 5.4 Power Electronics

#### 5.4.1 Inverters
- **Topology:** Three-phase bridge inverter
- **Semiconductors:** SiC or GaN preferred for >95% efficiency
- **Power Density:** Target 15-20 kW/kg
- **Cooling:** Integrated with motor cooling loop

#### 5.4.2 DC-DC Converters
- **Input Voltage:** 400-800V (battery pack)
- **Output Voltages:** 12V, 28V, 48V (avionics)
- **Efficiency:** Minimum 92%
- **Redundancy:** Dual converters for critical loads


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
