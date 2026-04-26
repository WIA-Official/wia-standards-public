# WIA-AUTO-015 PHASE 1 — Data Format Specification

**Standard:** WIA-AUTO-015
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-AUTO-015: Autonomous Ship Specification v1.0

> **Standard ID:** WIA-AUTO-015
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Maritime Autonomy Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [IMO MASS Autonomy Levels](#2-imo-mass-autonomy-levels)
3. [Navigation Systems](#3-navigation-systems)
4. [Collision Avoidance (COLREG)](#4-collision-avoidance-colreg)
5. [Remote Operation Centers](#5-remote-operation-centers)
6. [Sensor Systems](#6-sensor-systems)
7. [Route Planning and Optimization](#7-route-planning-and-optimization)
8. [Cybersecurity](#8-cybersecurity)
9. [Communication Protocols](#9-communication-protocols)
10. [Data Formats](#10-data-formats)
11. [API Interface](#11-api-interface)
12. [Safety Protocols](#12-safety-protocols)
13. [References](#13-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework for autonomous ship operations, encompassing navigation, collision avoidance, remote monitoring, sensor integration, and safety protocols in compliance with International Maritime Organization (IMO) regulations for Maritime Autonomous Surface Ships (MASS).

### 1.2 Scope

The standard covers:
- Autonomous navigation and route planning algorithms
- Sensor fusion and environmental awareness
- COLREG-compliant collision avoidance
- Remote operation and monitoring systems
- Cybersecurity for maritime critical infrastructure
- Integration with existing maritime systems (AIS, ECDIS, VTS)

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to make maritime transportation safer, more efficient, and environmentally sustainable. By reducing human error (responsible for 75% of maritime accidents), optimizing routes, and enabling continuous operation, autonomous ships benefit global trade, seafarers, and the environment.

### 1.4 Terminology

- **IMO**: International Maritime Organization
- **MASS**: Maritime Autonomous Surface Ships
- **COLREG**: International Regulations for Preventing Collisions at Sea (1972)
- **AIS**: Automatic Identification System
- **ECDIS**: Electronic Chart Display and Information System
- **VTS**: Vessel Traffic Service
- **GNSS**: Global Navigation Satellite System (GPS, GLONASS, Galileo, BeiDou)
- **CPA**: Closest Point of Approach
- **TCPA**: Time to Closest Point of Approach
- **ARPA**: Automatic Radar Plotting Aid
- **SOLAS**: Safety of Life at Sea Convention

---

## 2. IMO MASS Autonomy Levels

### 2.1 Level Definitions

The IMO defines four degrees of autonomy for MASS:

#### 2.1.1 Level 0 - Manual Operation
- Traditional ship with manual navigation
- Human crew performs all operations
- No autonomous systems beyond basic automation

#### 2.1.2 Level 1 - On-board Decision Support
- Automated processes assist human operators
- Crew makes all critical decisions
- Systems provide recommendations and warnings

**Examples:**
- Autopilot with weather routing
- Collision avoidance warnings
- Engine optimization recommendations

#### 2.1.3 Level 2 - Remote Control with Seafarers On Board
- Ship controlled from shore-based center
- Seafarers available for emergency intervention
- Remote operators have full situational awareness

**Systems Required:**
- Real-time video monitoring
- Remote control interface
- Redundant communication links
- Emergency override capability

#### 2.1.4 Level 3 - Remote Control without Seafarers
- Fully remote operation from shore
- No crew on board for extended periods
- Shore operators monitor multiple vessels
- Periodic autonomous operation

**Additional Requirements:**
- Advanced autonomous navigation
- Comprehensive sensor suite
- Remote diagnostics and maintenance planning
- Emergency autonomous safe harbor capability

#### 2.1.5 Level 4 - Fully Autonomous
- Complete autonomous operation
- Shore monitoring only (no active control)
- Self-diagnostic and self-maintenance
- AI-driven decision making

**Capabilities:**
- Full COLREG compliance
- Dynamic re-routing
- Weather avoidance
- Port entry and docking
- Emergency management

### 2.2 Autonomy Level Transition

```
Level Transition Algorithm:

1. Assess current conditions:
   - Weather severity
   - Traffic density
   - Communication reliability
   - System health

2. Determine appropriate level:
   IF severe_weather OR high_traffic OR degraded_sensors:
       level = min(level - 1, 2)  # Require human oversight
   ELSE IF optimal_conditions AND all_systems_nominal:
       level = authorized_max_level

3. Request transition:
   SEND transition_request TO shore_control
   AWAIT approval WITH timeout

4. Execute transition:
   LOG level_change
   NOTIFY all_stakeholders
   UPDATE operating_parameters
```

### 2.3 Level-Specific Requirements

| Requirement | L1 | L2 | L3 | L4 |
|-------------|----|----|----|----|
| Onboard crew | Required | Required | Optional | None |
| Remote monitoring | Optional | Required | Required | Required |
| Autonomous navigation | Partial | Partial | Full | Full |
| Collision avoidance | Manual+Auto | Auto+Override | Fully Auto | Fully Auto |
| Port operations | Manual | Remote+Manual | Remote | Autonomous |
| Emergency response | Crew | Crew+Shore | Shore | Autonomous+Shore |

---

## 3. Navigation Systems

### 3.1 Position Determination

#### 3.1.1 GNSS Integration

Primary positioning uses multi-constellation GNSS:

```
Position = weighted_average([
    GPS_position,
    GLONASS_position,
    Galileo_position,
    BeiDou_position
])

Weight(system) = 1 / (uncertainty²)
```

**Accuracy Requirements:**
- Open ocean: ±10 meters (95% confidence)
- Coastal waters: ±5 meters
- Port approach: ±2 meters (with DGPS/RTK)

#### 3.1.2 Dead Reckoning

When GNSS unavailable, use inertial navigation:

```
Position(t) = Position(t₀) + ∫[t₀ to t] Velocity(τ) dτ

Velocity(t) = Velocity(t₀) + ∫[t₀ to t] Acceleration(τ) dτ
```

**Drift Correction:**
```
Estimated_drift = ∫ (gyro_bias + accel_bias) dt

Corrected_position = DR_position - estimated_drift
```

Update with:
- Celestial navigation (sun/star sights)
- Radar fix (known landmarks)
- Depth contour matching
- Visual landmark recognition

### 3.2 Electronic Chart System (ECDIS)

#### 3.2.1 Chart Data Management

```
Chart_coverage(route) = ⋃[waypoints] chart_cell(waypoint)

FOR EACH cell IN route:
    ASSERT cell.edition >= minimum_required_edition
    ASSERT cell.expiry_date > voyage_end_date
    ASSERT cell.scale >= required_scale
```

#### 3.2.2 Route Planning

Optimal route calculation:

```
Route = optimize(
    objective = minimize(voyage_time × fuel_cost + risk_penalty),
    constraints = [
        water_depth ≥ draft + UKC,  # Under-Keel Clearance
        distance_to_hazards ≥ safety_margin,
        waypoint ∈ navigable_waters,
        course_changes ≤ max_rate
    ]
)
```

Where UKC (Under-Keel Clearance) = max(10% × draft, 0.5 meters)

### 3.3 Great Circle Navigation

#### 3.3.1 Distance Calculation

Using Haversine formula:

```
Δφ = φ₂ - φ₁
Δλ = λ₂ - λ₁

a = sin²(Δφ/2) + cos(φ₁) × cos(φ₂) × sin²(Δλ/2)
c = 2 × atan2(√a, √(1-a))
d = R × c

Where:
  R = 3,440.065 NM (Earth radius)
  φ = latitude in radians
  λ = longitude in radians
```

#### 3.3.2 Initial Bearing

```
θ = atan2(
    sin(Δλ) × cos(φ₂),
    cos(φ₁) × sin(φ₂) - sin(φ₁) × cos(φ₂) × cos(Δλ)
)

Bearing = (θ × 180/π + 360) mod 360
```

### 3.4 Waypoint Navigation

#### 3.4.1 Cross-Track Error

```
XTE = asin(
    sin(d₁₃/R) × sin(θ₁₃ - θ₁₂)
) × R

Where:
  d₁₃ = distance from start to current position
  θ₁₃ = bearing from start to current position
  θ₁₂ = bearing from start to next waypoint
```

#### 3.4.2 Course Correction

```
Correction = K_p × XTE + K_d × (dXTE/dt)

New_heading = planned_heading + Correction

Where:
  K_p = proportional gain (typically 1.5)
  K_d = derivative gain (typically 0.5)
```

---

## 4. Collision Avoidance (COLREG)

### 4.1 COLREG Rules Implementation

The system must implement all 38 rules of COLREG 1972, with focus on Part B (Steering and Sailing Rules).

#### 4.1.1 Rule 5 - Look-out

Maintain 360° awareness using:
- Radar (X-band and S-band)
- AIS receiver
- Visual cameras (visible and IR)
- LiDAR
- Human monitoring (for levels 1-3)

```
360° Coverage Algorithm:

sensor_coverage = ⋃[all_sensors] coverage_area(sensor)

ASSERT sensor_coverage ≥ 0.95 × full_sphere  # 95% coverage minimum

IF coverage < 0.95:
    ALERT "Degraded sensor coverage"
    REDUCE autonomy_level
    INCREASE human_supervision
```

#### 4.1.2 Rule 13 - Overtaking

```
is_overtaking = (
    relative_bearing > 112.5° AND
    relative_bearing < 247.5° AND
    own_speed > target_speed
)

IF is_overtaking:
    # Overtaking vessel must keep clear
    action = "maintain_course"  # Give-way is on overtaking vessel
    monitor_target_actions()
ELSE:
    apply_other_rules()
```

#### 4.1.3 Rule 14 - Head-on Situation

```
is_head_on = (
    abs(relative_bearing - 180°) < 10° AND
    CPA < safe_distance
)

IF is_head_on:
    # Both vessels alter course to starboard
    action = "alter_course"
    new_heading = current_heading + 15°  # Turn to starboard
    speed_reduction = 0  # Maintain speed
```

#### 4.1.4 Rule 15 - Crossing Situation

```
is_crossing = (
    relative_bearing > 5° AND
    relative_bearing < 112.5° AND
    CPA < safe_distance
)

IF is_crossing:
    IF target_on_starboard_side:
        # We are give-way vessel
        action = "give_way"
        maneuver = select_avoidance_maneuver()
    ELSE:
        # We are stand-on vessel
        action = "maintain_course_and_speed"
        monitor_give_way_vessel()
```

### 4.2 Collision Risk Assessment

#### 4.2.1 CPA and TCPA Calculation

```
Relative velocity:
  v_rel = v_target - v_own

Relative position:
  r_rel = position_target - position_own

Closest Point of Approach (CPA):
  CPA = |r_rel × v̂_rel| / |v̂_rel|

  Where v̂_rel = normalized velocity vector

Time to CPA (TCPA):
  TCPA = -(r_rel · v_rel) / |v_rel|²
```

#### 4.2.2 Collision Risk Index

```
Risk = calculate_risk(CPA, TCPA, uncertainty)

risk_index = (D_safe / CPA) × (T_safe / TCPA) × uncertainty_factor

Where:
  D_safe = 2.0 NM (safe passing distance)
  T_safe = 20 minutes (safe time margin)
  uncertainty_factor = 1 + sensor_error + target_maneuver_probability

Risk Levels:
  risk_index < 0.3:  LOW - monitor only
  0.3 ≤ risk_index < 0.7:  MEDIUM - prepare maneuver
  0.7 ≤ risk_index < 1.0:  HIGH - execute maneuver
  risk_index ≥ 1.0:  CRITICAL - emergency action
```

### 4.3 Collision Avoidance Maneuvers

#### 4.3.1 Course Alteration

```
Optimal course change:

Δθ = arcsin(D_safe / CPA_current) + safety_margin

Where safety_margin = 10° (standard) to 30° (restricted visibility)

Maneuver constraints:
  - Min turn radius = (V² × T) / (35 × L)  [IMO standards]
  - Max rate of turn = 4.5° × √(K/L)  [ship maneuverability]
  - Return to course after CPA + safety_time
```

#### 4.3.2 Speed Reduction

```
New speed calculation:

V_new = V_current × (1 - reduction_factor)

reduction_factor = min(
    (D_safe - CPA) / D_safe,
    0.5  # Maximum 50% reduction for stability
)

Constraints:
  - V_new ≥ V_min_steerage  (minimum for steering control)
  - Gradual reduction: dV/dt ≤ 2 knots/minute
  - Consider stopping distance
```

#### 4.3.3 Combined Maneuver

For complex situations with multiple targets:

```
optimal_maneuver = optimize(
    cost = Σ[all_targets] (
        collision_risk(target) +
        fuel_cost(maneuver) +
        delay_penalty(maneuver)
    ),
    constraints = [
        CPA(target) ≥ D_safe ∀ targets,
        COLREG_compliance(maneuver) = true,
        ship_dynamics_feasible(maneuver) = true
    ]
)
```

### 4.4 Ship Domain Concept

The ship domain represents the surrounding area a ship navigates to avoid:

```
Domain calculation (Fujii model):

L_fore = L_ship × (1 + 2.5 × V/V_max)  # Forward
L_aft = L_ship × (1 + 0.5 × V/V_max)   # Aft
L_star = B_ship × (1 + 1.5 × V/V_max)  # Starboard
L_port = B_ship × (1 + 1.2 × V/V_max)  # Port

Where:
  L_ship = ship length
  B_ship = ship beam
  V = current speed
  V_max = maximum speed
```

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
