# WIA-AUTO-019 PHASE 4 — Integration Specification

**Standard:** WIA-AUTO-019
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 9. API Interface

### 9.1 Core Functions

#### 9.1.1 Calculate Drag Force

```typescript
interface DragCalculation {
  pressure: number;        // Pa
  velocity: number;        // m/s
  dragCoefficient: number; // dimensionless
  crossSectionArea: number; // m²
}

interface DragResult {
  force: number;           // N
  power: number;           // W
  energyPer100km: number;  // kWh
}
```

#### 9.1.2 Calculate Levitation Force

```typescript
interface LevitationParams {
  magneticFieldStrength: number; // Tesla
  effectiveArea: number;         // m²
  podMass: number;               // kg
  levitationGap: number;         // mm
}

interface LevitationResult {
  force: number;                 // N
  powerRequired: number;         // kW
  gapStability: number;          // 0-1
  isStable: boolean;
}
```

#### 9.1.3 Calculate Energy Consumption

```typescript
interface JourneyParams {
  distance: number;              // meters
  podMass: number;               // kg
  maxSpeed: number;              // m/s
  passengers: number;
  cargo: number;                 // kg
  elevationChange: number;       // meters
}

interface EnergyResult {
  acceleration: number;          // kWh
  cruising: number;              // kWh
  braking: number;               // kWh (negative = recovery)
  auxiliary: number;             // kWh
  total: number;                 // kWh
  perPassenger: number;          // kWh
  efficiency: number;            // kWh/100km
}
```

### 9.2 Validation Functions

#### 9.2.1 Validate Pod Design

```typescript
interface PodDesign {
  length: number;                // meters
  diameter: number;              // meters
  mass: number;                  // kg
  dragCoefficient: number;
  passengerCapacity: number;
  cargoCapacity: number;         // kg
}

interface ValidationResult {
  isValid: boolean;
  errors: string[];
  warnings: string[];
  metrics: {
    tubeAreaRatio: number;
    kantrowitzLimit: number;     // m/s
    powerToWeight: number;        // W/kg
  };
}
```

### 9.3 Simulation Functions

#### 9.3.1 Simulate Journey

```typescript
interface SimulationParams {
  origin: string;
  destination: string;
  podType: string;
  passengers: number;
  departureTime: Date;
}

interface SimulationResult {
  duration: number;              // seconds
  distance: number;              // meters
  maxSpeed: number;              // m/s
  averageSpeed: number;          // m/s
  energyConsumption: EnergyResult;
  timeline: {
    phase: string;
    startTime: number;           // seconds from start
    endTime: number;
    distance: number;
    speed: number;
  }[];
  environmental: {
    co2Savings: number;          // kg vs airplane
    energySavings: number;       // kWh vs car
  };
}
```

---

## 10. Safety Protocols

### 10.1 Pre-Departure Checklist

- [ ] Tube pressure verified < 150 Pa
- [ ] Pod systems operational (levitation, propulsion, life support)
- [ ] Communication systems tested
- [ ] Passenger count confirmed
- [ ] Emergency equipment verified
- [ ] Route clear of obstacles
- [ ] Weather conditions acceptable (for above-ground sections)
- [ ] Braking systems tested
- [ ] Energy storage charged
- [ ] Control system handshake successful

### 10.2 Operating Limits

**Maximum Speeds**:
```
Cruising: 1200 km/h (333 m/s)
Curves (R>60km): 1000 km/h (278 m/s)
Curves (R>30km): 800 km/h (222 m/s)
Station Approach: 100 km/h (28 m/s)
Emergency: 50 km/h (14 m/s)
```

**Pressure Limits**:
```
Normal Operation: 80-120 Pa
Warning Threshold: 150 Pa
Emergency Threshold: 500 Pa
Critical: 1000 Pa (initiate evacuation)
```

**Acceleration Limits**:
```
Normal Acceleration: 0.5g
Emergency Braking: 1.5g
Lateral (curves): 0.15g
Vertical (elevation): 0.1g
```

### 10.3 Maintenance Schedules

#### 10.3.1 Daily Inspections

- Visual inspection of tube segments
- Pressure monitoring system check
- Pump performance verification
- Emergency system test
- Communication system test

#### 10.3.2 Weekly Maintenance

- Levitation system calibration
- Motor winding inspection
- Brake system test
- Airlock seal inspection
- Battery charge verification

#### 10.3.3 Monthly Overhaul

- Structural integrity assessment
- Non-destructive testing (NDT)
- Vacuum pump servicing
- Emergency egress system drill
- Software system updates

#### 10.3.4 Annual Certification

- Complete tube inspection
- Full system load test
- Emergency scenario simulation
- Regulatory compliance audit
- Safety equipment certification

### 10.4 Emergency Response

#### 10.4.1 Response Teams

```
Level 1: On-site technicians (always present)
Level 2: Emergency response team (15-minute response)
Level 3: Specialized rescue team (60-minute response)
Level 4: External emergency services (coordination)
```

#### 10.4.2 Evacuation Procedures

**In-Tube Evacuation**:
```
1. Stop pod at nearest emergency airlock (5 min)
2. Open emergency airlock (2 min)
3. Passengers enter airlock chamber (3 min)
4. Pressurize airlock to atmospheric (3 min)
5. Exit to surface via emergency stairs (10 min)
Total Time: ~23 minutes
```

**Station Evacuation**:
```
1. Alert all passengers (immediate)
2. Guide to emergency exits (2 min)
3. Evacuate via normal exits (5 min)
4. Muster at assembly point (3 min)
Total Time: ~10 minutes
```

---

## 11. References

### 11.1 Technical Papers

1. Musk, E. (2013). "Hyperloop Alpha"
4. Opgenoord, M.M.J., Caplan, P.C. (2018). "Aerodynamic Design of the Hyperloop Concept"

### 11.2 Standards References

- ISO 2631: Mechanical vibration and shock
- IEC 61375: Electronic railway equipment - Train communication network
- EN 14067: Railway applications - Aerodynamics
- ASME BPVC: Boiler and Pressure Vessel Code
- NFPA 130: Standard for Fixed Guideway Transit Systems

### 11.3 Physical Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Atmospheric pressure | P₀ | 101,325 Pa |
| Air density (STP) | ρ₀ | 1.225 kg/m³ |
| Gas constant | R | 8.314 J/mol·K |
| Magnetic permeability | μ₀ | 4π × 10⁻⁷ H/m |
| Gravitational acceleration | g | 9.81 m/s² |
| Speed of sound (air) | c | 340 m/s |

### 11.4 WIA Standards

- WIA-INTENT: Intent-based transportation interfaces
- WIA-OMNI-API: Universal transportation API
- WIA-ENERGY: Energy management and optimization
- WIA-SOCIAL: Social mobility coordination
- WIA-QUANTUM: Quantum-secure communication

---

## Appendix A: Example Calculations

### A.1 Drag Force at Cruising Speed

```
Given:
- Pressure: 100 Pa
- Velocity: 300 m/s (1080 km/h)
- Drag coefficient: 0.15
- Cross-sectional area: 5.72 m²

Air density at 100 Pa:
ρ = 0.0012 kg/m³

Calculation:
F_drag = 0.5 × 0.0012 × (300)² × 0.15 × 5.72
F_drag = 46.5 N

Power required:
P = F × v = 46.5 × 300 = 13,950 W ≈ 14 kW

Comparison to atmospheric pressure:
At 101,325 Pa: F_drag ≈ 46,500 N (1000× more!)
```

### A.2 Energy for 600 km Journey

```
Given:
- Distance: 600 km
- Pod mass: 15,000 kg
- Max speed: 300 m/s
- Passengers: 28

Acceleration energy:
E_accel = 0.5 × 15,000 × (300)²
E_accel = 675 MJ = 187.5 kWh

Cruising energy (drag):
E_cruise = F_drag × d = 46.5 × 600,000
E_cruise = 27.9 MJ = 7.75 kWh

Auxiliary systems:
E_aux = 100 kW × 0.583 hours = 58.3 kWh

Total energy required:
E_total = 187.5 + 7.75 + 58.3 = 253.55 kWh

With regenerative braking (75% recovery):
E_recovered = 187.5 × 0.75 = 140.6 kWh
E_net = 253.55 - 140.6 = 112.95 kWh

Per passenger:
E_per_passenger = 112.95 / 28 = 4.03 kWh

Comparison:
- Airplane: 900 kWh per passenger (223× more!)
- Car: 200 kWh per passenger (50× more!)
- High-speed rail: 20 kWh per passenger (5× more!)
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-019 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
