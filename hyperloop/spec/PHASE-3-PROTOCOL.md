# WIA-AUTO-019 PHASE 3 — Protocol Specification

**Standard:** WIA-AUTO-019
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 6. Station and Boarding Systems

### 6.1 Station Design

#### 6.1.1 Airlock System

Multi-chamber airlock:
```
Chamber 1: Atmospheric pressure → 10,000 Pa
Chamber 2: 10,000 Pa → 1,000 Pa
Chamber 3: 1,000 Pa → 100 Pa
```

Pump-down time:
```
t_pumpdown = (V × ln(P₁/P₂)) / S
```

Where:
- `V` = Chamber volume (100-200 m³)
- `P₁` = Initial pressure
- `P₂` = Final pressure
- `S` = Pump speed (5000 m³/hr)

Total evacuation time: 3-5 minutes

#### 6.1.2 Boarding Process

Passenger flow:
```
1. Security and Check-in: 10 minutes
2. Airlock Entry: 1 minute
3. Evacuation: 4 minutes
4. Pod Boarding: 2 minutes
5. Pre-departure: 1 minute
Total: 18 minutes
```

Throughput:
```
Pods per Hour: 60 / 18 = 3.3 pods/hour (conservative)
Optimized: 6-10 pods/hour
```

### 6.2 Platform Configuration

#### 6.2.1 Platform Layout

```
Length: 50-100 meters
Width: 5-10 meters
Platforms: 2-4 (bidirectional)
Height: Variable (airlock floor level)
Accessibility: ADA compliant
```

#### 6.2.2 Pod Docking

Docking mechanism:
```
Positioning Accuracy: ±50 mm
Docking Force: <500 N
Seal Type: Inflatable rubber gasket
Seal Pressure: 150 kPa
Leak Rate: <0.1 Pa·l/s
```

### 6.3 Emergency Egress

#### 6.3.1 Emergency Airlocks

Spacing along tube:
```
Interval: Every 500 meters
Type: Single-chamber rapid evacuation
Evacuation Time: 2 minutes
Capacity: 50 passengers
```

#### 6.3.2 Escape Pods

Dedicated escape system:
```
Type: Self-contained pressure vessel
Capacity: 4-6 passengers
Propulsion: Compressed air or battery
Speed: 5-10 m/s
Range: To nearest emergency airlock
```

---

## 7. Emergency Systems

### 7.1 Emergency Scenarios

#### 7.1.1 Power Loss

Response protocol:
```
1. Detect power loss (10 ms)
2. Activate onboard batteries (50 ms)
3. Initiate controlled deceleration (1 second)
4. Deploy mechanical brakes if needed
5. Coast to nearest station or emergency stop
```

Battery backup:
```
Capacity: 100-200 kWh
Duration: 30-60 minutes
Systems Powered:
  - Life support
  - Communications
  - Emergency lighting
  - Magnetic brakes
```

#### 7.1.2 Pressure Loss

If tube pressure rises above threshold:
```
Threshold: P > 1,000 Pa (10× normal)
Detection: Continuous pressure monitoring
Response Time: <1 second

Actions:
1. Alert all pods in affected section
2. Initiate emergency braking
3. Isolate section with emergency valves
4. Activate emergency pumps
5. Route pods to safe stations
```

#### 7.1.3 Fire Suppression

Fire detection and suppression:
```
Detection: Optical smoke, heat, CO sensors
Response Time: <5 seconds
Suppression: Inert gas (Nitrogen or CO₂)
Oxygen Reduction: 21% → 12% (prevents combustion)
Passenger Safety: O₂ masks automatically deploy
```

### 7.2 Braking Systems

#### 7.2.1 Primary Braking

Regenerative electromagnetic:
```
Type: Reverse LIM operation
Deceleration: 0.5g (4.9 m/s²)
Energy Recovery: 75-85%
Brake Distance: 9.2 km from max speed
Control: Computer-controlled, variable rate
```

#### 7.2.2 Secondary Braking

Eddy current brakes:
```
Type: Non-contact electromagnetic
Deceleration: 0.3g (2.9 m/s²)
Activation: Automatic if primary fails
No wear: No mechanical contact
Brake Distance: 15.4 km from max speed
```

#### 7.2.3 Emergency Braking

Mechanical friction brakes:
```
Type: Caliper brakes on guide rail
Deceleration: Up to 1.5g (14.7 m/s²)
Activation: Manual override or system failure
Brake Distance: 3.1 km from max speed
Wear: Requires periodic replacement
```

### 7.3 Communication Systems

#### 7.3.1 Pod-to-Control

Primary communication:
```
Type: Fiber optic + wireless backup
Bandwidth: 100 Mbps (bidirectional)
Latency: <10 ms
Range: Entire route
Protocol: Redundant encrypted channels
```

#### 7.3.2 Passenger Communication

Internal systems:
```
PA System: All pods
Video: Real-time route information
Emergency Call: Direct to control center
Mobile: 4G/5G repeaters in tube
```

### 7.4 Collision Avoidance

#### 7.4.1 Pod Spacing

Minimum separation:
```
Spacing at Max Speed: 10 km
Time Headway: 33 seconds
Pods in System: Route length / 10 km
Safety Factor: 2× minimum braking distance
```

#### 7.4.2 Automated Control

Control system:
```
Type: Moving block with continuous monitoring
Position Accuracy: ±1 meter
Velocity Measurement: ±0.1 m/s
Update Rate: 100 Hz
Fail-Safe: Automatic emergency braking
```

---

## 8. Data Formats

### 8.1 Pod Telemetry

#### 8.1.1 Real-Time Data

```json
{
  "pod_id": "HP-2025-001",
  "timestamp": "2025-12-26T14:30:00Z",
  "position": {
    "distance_from_origin": 125400.5,
    "latitude": 34.0522,
    "longitude": -118.2437,
    "altitude": 125.3
  },
  "velocity": {
    "speed": 299.8,
    "acceleration": 0.02,
    "direction": "northbound"
  },
  "systems": {
    "levitation": {
      "gap_front_left": 10.2,
      "gap_front_right": 10.1,
      "gap_rear_left": 10.3,
      "gap_rear_right": 10.2,
      "power_consumption": 75.4
    },
    "propulsion": {
      "motor_current": 450,
      "motor_voltage": 2400,
      "power_output": 1080,
      "temperature": 65.5
    },
    "pressure": {
      "internal": 101325,
      "external": 98,
      "differential": 101227
    },
    "life_support": {
      "oxygen_level": 20.9,
      "co2_level": 0.04,
      "temperature": 22.5,
      "humidity": 45
    }
  },
  "passengers": 28,
  "status": "cruising"
}
```

### 8.2 Journey Data

#### 8.2.1 Trip Record

```json
{
  "journey_id": "J-2025122614300001",
  "pod_id": "HP-2025-001",
  "route": {
    "origin": "Los Angeles Central",
    "destination": "San Francisco Downtown",
    "distance": 600000,
    "waypoints": []
  },
  "schedule": {
    "departure_scheduled": "2025-12-26T14:00:00Z",
    "departure_actual": "2025-12-26T14:00:12Z",
    "arrival_scheduled": "2025-12-26T14:35:00Z",
    "arrival_estimated": "2025-12-26T14:35:05Z"
  },
  "performance": {
    "max_speed": 333.2,
    "average_speed": 285.4,
    "energy_consumed": 245.8,
    "energy_recovered": 186.2,
    "net_energy": 59.6
  },
  "passengers": {
    "boarded": 28,
    "capacity": 40,
    "load_factor": 0.70
  }
}
```

### 8.3 Station Status

#### 8.3.1 Platform Data

```json
{
  "station_id": "LAX-CENTRAL",
  "name": "Los Angeles Central Hyperloop",
  "timestamp": "2025-12-26T14:30:00Z",
  "platforms": [
    {
      "platform_id": "P1",
      "direction": "northbound",
      "status": "occupied",
      "pod_id": "HP-2025-002",
      "airlock_pressure": 101325,
      "next_departure": "2025-12-26T14:32:00Z"
    },
    {
      "platform_id": "P2",
      "direction": "northbound",
      "status": "ready",
      "pod_id": null,
      "airlock_pressure": 98,
      "next_arrival": "2025-12-26T14:35:00Z"
    }
  ],
  "tube_pressure": 102,
  "system_status": "operational",
  "capacity": {
    "current": 45,
    "maximum": 200
  }
}
```

---


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

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
