# WIA-AUTO-019 PHASE 2 — API Interface Specification

**Standard:** WIA-AUTO-019
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

Young's Modulus: E = 71.7 GPa

Carbon Fiber:
  Density: ρ = 1600 kg/m³
  Tensile Strength: σ_t = 3500 MPa
  Young's Modulus: E = 230 GPa
```

#### 3.3.2 Pressure Vessel

The pod must maintain internal pressure:
```
t = (P × r) / (σ_allow × η)
```

Where:
- `t` = Wall thickness (mm)
- `P` = Pressure differential (101,325 Pa)
- `r` = Pod radius (1.35 m)
- `σ_allow` = Allowable stress (200 MPa)
- `η` = Joint efficiency (0.85)

Calculation:
```
t = (101,325 × 1.35) / (200 × 10⁶ × 0.85)
t ≈ 0.8 mm
```

Add safety factor of 3:
```
t_actual = 2.4 mm (minimum)
```

### 3.4 Passenger Compartment

#### 3.4.1 Seating Configuration

Standard configuration:
```
Rows: 7-10
Seats per Row: 4 (2+2)
Total Capacity: 28-40 passengers
Seat Pitch: 800-900 mm
Aisle Width: 450 mm
```

#### 3.4.2 Life Support

Oxygen supply:
```
O₂ Rate per Person: 0.84 l/min
CO₂ Production: 0.93 l/min
Duration: 60 minutes minimum
Total O₂ Required: 50.4 liters × 28 passengers = 1,411 liters
```

Storage:
```
Compressed O₂ at 20 MPa (200 bar)
Tank Volume: 7 liters (provides 1,400 liters at STP)
Number of Tanks: 2 (redundant)
```

---

## 4. Magnetic Levitation Systems

### 4.1 Levitation Principles

#### 4.1.1 Electromagnetic Suspension (EMS)

Attractive force between electromagnets and ferromagnetic rail:

```
F_lev = (B² × A) / (2μ₀)
```

Where:
- `F_lev` = Levitation force (newtons)
- `B` = Magnetic flux density (tesla)
- `A` = Pole face area (m²)
- `μ₀` = Permeability of free space (4π × 10⁻⁷ H/m)

#### 4.1.2 Electrodynamic Suspension (EDS)

Repulsive force from induced currents:

```
F_lev = (μ₀ × I² × n²) / (4π × g)
```

Where:
- `I` = Current in superconducting coils (amperes)
- `n` = Number of turns per unit length
- `g` = Levitation gap (meters)

### 4.2 Levitation System Design

#### 4.2.1 EMS Configuration

For 15,000 kg pod:

```
Required Force: F = m × g = 15,000 × 9.81 = 147,150 N
Safety Factor: SF = 2.0
Design Force: F_design = 294,300 N
```

Magnet specifications:
```
Magnetic Field: B = 1.2 Tesla
Pole Face Area: A = 0.4 m² per magnet
Number of Magnets: 8 (4 per side)
Power per Magnet: 5-10 kW
Total Power: 40-80 kW
```

#### 4.2.2 Levitation Gap

Operating parameters:
```
Nominal Gap: g₀ = 10 mm
Maximum Gap: g_max = 15 mm
Minimum Gap: g_min = 8 mm
Gap Sensors: ±0.1 mm accuracy
Control Frequency: 1000 Hz
```

Gap control equation:
```
F_control = K_p × (g_target - g_actual) + K_d × dg/dt
```

Where:
- `K_p` = Proportional gain
- `K_d` = Derivative gain

#### 4.2.3 Energy Consumption

Levitation energy:
```
P_lev = F_lev × v_vertical + P_control + P_loss
```

Typical values:
```
P_control = 40-80 kW (steady state)
P_loss = 10-20 kW (eddy currents, hysteresis)
Total: 50-100 kW
```

### 4.3 Lateral Guidance

#### 4.3.1 Guidance Forces

Lateral stability:
```
F_lateral = K_lateral × y
```

Where:
- `K_lateral` = Lateral stiffness (N/m)
- `y` = Lateral displacement (m)

Required stiffness:
```
K_lateral ≥ 100,000 N/m
Max Lateral Displacement: ±50 mm
```

#### 4.3.2 Curve Negotiation

Centripetal force:
```
F_c = m × v² / R
```

Where:
- `m` = Pod mass (15,000 kg)
- `v` = Velocity (m/s)
- `R` = Curve radius (m)

For passenger comfort (a_lateral ≤ 0.15g):
```
R_min = v² / (0.15 × g)
```

At 300 m/s:
```
R_min = (300)² / (0.15 × 9.81)
R_min ≈ 61,162 m (61 km)
```

---

## 5. Linear Induction Motors

### 5.1 LIM Principles

#### 5.1.1 Thrust Force

Linear motor thrust:
```
F_thrust = B × I × L × n × cos(φ)
```

Where:
- `B` = Magnetic field strength (1.0-1.5 T)
- `I` = Phase current (amperes)
- `L` = Active conductor length (meters)
- `n` = Number of phases (typically 3)
- `φ` = Power factor angle

#### 5.1.2 Synchronous Speed

The speed of the traveling magnetic wave:
```
v_sync = 2 × τ × f
```

Where:
- `τ` = Pole pitch (0.2-0.4 m)
- `f` = Supply frequency (Hz)

For 300 m/s with τ = 0.3 m:
```
f = v_sync / (2 × τ) = 300 / 0.6 = 500 Hz
```

### 5.2 Motor Configuration

#### 5.2.1 Stator Design

Track-mounted stators:
```
Length per Section: 100-500 meters
Number of Sections: Varies by route
Power per Section: 1-5 MW
Cooling: Liquid (water/glycol)
```

#### 5.2.2 Rotor Design

Pod-mounted rotor:
```
Type: Aluminum conductor plate with iron backing
Thickness: 10-20 mm aluminum, 20-30 mm iron
Weight: 500-1000 kg per pod
Cooling: Passive air cooling
```

### 5.3 Acceleration and Braking

#### 5.3.1 Acceleration Profile

Maximum acceleration for passenger comfort:
```
a_max = 0.5g = 4.9 m/s²
```

Time to reach cruising speed (300 m/s):
```
t = v / a = 300 / 4.9 ≈ 61 seconds
```

Distance required:
```
d = v²/ (2a) = (300)² / (2 × 4.9) = 9,184 meters ≈ 9.2 km
```

#### 5.3.2 Energy for Acceleration

Kinetic energy:
```
E_kinetic = ½ × m × v²
E_kinetic = 0.5 × 15,000 × (300)²
E_kinetic = 675,000,000 J = 675 MJ = 187.5 kWh
```

With 85% motor efficiency:
```
E_required = 187.5 / 0.85 = 220.6 kWh
```

#### 5.3.3 Regenerative Braking

Energy recovery during braking:
```
E_recovered = E_kinetic × η_braking × η_storage
```

Where:
- `η_braking` = Braking efficiency (0.85)
- `η_storage` = Storage efficiency (0.90)

```
E_recovered = 187.5 × 0.85 × 0.90 = 143.4 kWh
```

Recovery rate:
```
Recovery Rate = 143.4 / 187.5 = 76.5%
```

### 5.4 Power Distribution

#### 5.4.1 Power Supply

Track-side power:
```
Voltage: 1000-3000 VDC
Current: 500-2000 A per section
Frequency Conversion: DC to variable frequency AC
Substations: Every 5-15 km
```

#### 5.4.2 Energy Storage

Wayside energy storage:
```
Type: Lithium-ion battery or flywheel
Capacity: 500-2000 kWh per station
Purpose: Peak power, regenerative braking, backup
Response Time: <100 ms
```

---


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

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
