# WIA-AUTO-017 PHASE 3 — Protocol Specification

**Standard:** WIA-AUTO-017
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 7. Battery and Range Management

### 7.1 Battery Capacity

#### 7.1.1 Energy Calculation

```
E = V × C
```

Where:
- `E` = Energy capacity (Wh)
- `V` = Nominal voltage (V)
- `C` = Capacity (Ah)

Example:
```
14.8V × 10Ah = 148 Wh
```

#### 7.1.2 State of Charge (SOC)

```
SOC(t) = SOC0 - ∫(I(t) / C)dt
```

Where:
- `SOC(t)` = State of charge at time t (0-1)
- `SOC0` = Initial state of charge
- `I(t)` = Current draw (A)
- `C` = Battery capacity (Ah)

### 7.2 Range Estimation

#### 7.2.1 Theoretical Range

```
R = (E × η) / P × v
```

Where:
- `R` = Maximum range (m)
- `E` = Battery energy (Wh)
- `η` = Efficiency (0.7-0.9)
- `P` = Average power consumption (W)
- `v` = Cruise speed (m/s)

#### 7.2.2 Practical Range

Account for reserves and conditions:

```
Rpractical = R × (1 - Rreserve) × Kwind × Kpayload × Kalt
```

Where:
- `Rreserve` = Reserve margin (0.2-0.3)
- `Kwind` = Wind factor (0.7-1.0)
- `Kpayload` = Payload factor (0.8-1.0)
- `Kalt` = Altitude factor (0.9-1.0)

### 7.3 Power Consumption Model

```
P = Phover + Pcruise + Pavionics

Phover = (mg)^(3/2) / √(2ρA)

Pcruise = ½ × ρ × v³ × A × CD / η

Pavionics = 10-50W (constant)
```

### 7.4 Battery Safety

Requirements:
- Temperature monitoring (5-45°C operating range)
- Overcharge protection
- Over-discharge prevention (land at 20% SOC)
- Cell balancing
- Short circuit protection
- Fire suppression system (for large batteries)

---

## 8. Safety and Emergency Protocols

### 8.1 Pre-Flight Checklist

- [ ] Battery charged > 80%
- [ ] GPS lock achieved (8+ satellites)
- [ ] IMU calibrated
- [ ] Propellers secure and undamaged
- [ ] Payload properly secured
- [ ] Flight plan approved by UTM
- [ ] Weather conditions acceptable
- [ ] Emergency landing zones identified
- [ ] Communication link verified
- [ ] Geofencing active

### 8.2 Geofencing

#### 8.2.1 No-Fly Zones

```
Priority Levels:
1. CRITICAL: Airports, military bases (hard boundary)
2. HIGH: Schools, hospitals (soft boundary with authorization)
3. MEDIUM: Parks, stadiums (temporal restrictions)
4. LOW: Residential areas (altitude restrictions)
```

#### 8.2.2 Geofence Implementation

```python
def check_geofence(position):
    for zone in no_fly_zones:
        distance = calculate_distance(position, zone.center)

        if distance < zone.radius:
            if zone.priority == "CRITICAL":
                return {"allowed": False, "action": "immediate_land"}
            elif zone.priority == "HIGH":
                return {"allowed": False, "action": "return_to_home"}
            else:
                return {"allowed": True, "warning": "restricted_area"}

    return {"allowed": True}
```

### 8.3 Emergency Procedures

#### 8.3.1 Loss of GPS

```
1. Switch to optical flow/visual odometry
2. Reduce altitude to 10m AGL
3. Hover in place
4. Wait for GPS recovery (max 60s)
5. If not recovered, initiate emergency landing
```

#### 8.3.2 Low Battery

```
Battery Levels:
- 30%: Warning, suggest RTH
- 20%: Automatic RTH initiated
- 10%: Emergency landing at nearest safe location
- 5%: Immediate emergency landing
```

#### 8.3.3 Motor Failure

```
Single Motor (Hexacopter+):
1. Detect failure via current/RPM sensors
2. Compensate with remaining motors
3. Reduce altitude gradually
4. Navigate to emergency landing zone
5. Execute controlled landing

Multiple Motors (Quadcopter):
1. Detect failure
2. Deploy emergency parachute (if equipped)
3. Cut power to remaining motors
4. Broadcast emergency signal
5. Record flight data for analysis
```

#### 8.3.4 Communication Loss

```
1. Detect signal loss (> 3s)
2. Continue current mission for 10s
3. If signal not restored:
   - Execute Return-On-Abort (ROA)
   - Climb to safe altitude
   - Return via pre-planned route
   - Land at home location
   - If home location blocked, land at nearest safe zone
```

### 8.4 Parachute System

For drones > 10 kg MTOW:

```
Deployment Conditions:
- Critical motor failure
- Loss of control
- Structural failure detected
- Manual pilot trigger

Deployment Sequence:
1. Detect critical failure
2. Cut motor power
3. Deploy parachute (0.2s)
4. Broadcast emergency signal
5. Activate strobe light
6. Log GPS coordinates
```

---

## 9. Data Formats

### 9.1 Waypoint Format

```json
{
  "waypoint_id": "WP-001",
  "position": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude_msl": 120,
    "altitude_agl": 30
  },
  "speed": 15.0,
  "heading": 45,
  "actions": [
    {"type": "take_photo", "params": {}},
    {"type": "hover", "params": {"duration": 5}}
  ],
  "acceptance_radius": 5.0
}
```

### 9.2 Delivery Package Format

```json
{
  "package_id": "PKG-20250101-1234",
  "weight": 2.5,
  "dimensions": {
    "length": 30,
    "width": 20,
    "height": 15,
    "unit": "cm"
  },
  "fragile": false,
  "temperature_sensitive": false,
  "value": 150.00,
  "insurance": true,
  "special_instructions": "Leave at front door",
  "tracking_code": "1Z999AA10123456784"
}
```

### 9.3 Flight Log Format

```json
{
  "flight_id": "FLT-20250101-1234",
  "drone_id": "WIA-DRN-X1-0042",
  "operator_id": "WIA-OP-001",
  "start_time": "2025-01-01T10:00:00Z",
  "end_time": "2025-01-01T10:15:00Z",
  "duration": 900,
  "distance": 5000,
  "max_altitude": 120,
  "max_speed": 22.5,
  "battery_consumed": 22,
  "waypoints_completed": 15,
  "incidents": [],
  "telemetry_file": "telemetry-20250101-1234.bin"
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
