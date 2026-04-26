# WIA-AUTO-007 PHASE 3 — Protocol Specification

**Standard:** WIA-AUTO-007
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

```

Where:
- `Range` = Vehicle range (km)
- `m_H2` = Hydrogen capacity (kg)
- `LHV_H2` = 120 MJ/kg
- `η_powertrain` = Powertrain efficiency (0.38-0.48)
- `E_consumption` = Energy consumption (MJ/km)

Example for 5.6 kg H2 capacity:
```
Range = (5.6 × 120 × 0.45) / 0.95
Range ≈ 318 km
```

### 7.4 Well-to-Wheel Efficiency

```
η_WtW = η_production × η_compression × η_transport × η_powertrain
```

Where:
- `η_production` = H2 production efficiency
  - Electrolysis: 60-80%
  - Steam methane reforming: 70-85%
- `η_compression` = Compression to 700 bar (0.88-0.92)
- `η_transport` = Distribution efficiency (0.95-0.98)
- `η_powertrain` = Vehicle efficiency (0.38-0.48)

**Green H2 (Electrolysis):**
```
η_WtW = 0.70 × 0.90 × 0.97 × 0.45 ≈ 27.5%
```

**Gray H2 (SMR with CCS):**
```
η_WtW = 0.75 × 0.90 × 0.97 × 0.45 ≈ 29.5%
```

### 7.5 Energy Consumption

#### 7.5.1 NEDC Cycle

Average energy consumption: 0.85-1.05 MJ/km (0.24-0.29 kWh/km)

#### 7.5.2 WLTC Cycle

Average energy consumption: 0.95-1.15 MJ/km (0.26-0.32 kWh/km)

#### 7.5.3 Highway Driving

Average energy consumption: 1.10-1.40 MJ/km (0.31-0.39 kWh/km)

---

## 8. Data Formats

### 8.1 Vehicle Status Data

```json
{
  "vehicle_id": "WIA-AUTO-007-12345",
  "timestamp": "2025-12-26T10:30:00Z",
  "fuel_cell": {
    "power_output": 85.5,
    "voltage": 385.2,
    "current": 222.0,
    "efficiency": 0.58,
    "temperature": 78.5,
    "status": "active"
  },
  "hydrogen_storage": {
    "pressure": 580.0,
    "temperature": 18.5,
    "mass_remaining": 3.2,
    "soc": 0.57,
    "tank_type": "Type IV H70"
  },
  "power_electronics": {
    "dc_bus_voltage": 650.0,
    "converter_efficiency": 0.97,
    "inverter_efficiency": 0.95
  },
  "motor": {
    "power_output": 78.0,
    "torque": 245.0,
    "rpm": 3040,
    "efficiency": 0.94
  },
  "battery_buffer": {
    "soc": 0.75,
    "voltage": 380.0,
    "power": 5.5,
    "mode": "assist"
  },
  "vehicle": {
    "speed": 85.0,
    "range_remaining": 185.0,
    "odometer": 12540.5,
    "energy_consumption": 0.98
  }
}
```

### 8.2 Refueling Session Data

```json
{
  "session_id": "REF-2025-12-26-001",
  "timestamp_start": "2025-12-26T10:15:00Z",
  "timestamp_end": "2025-12-26T10:18:30Z",
  "vehicle_id": "WIA-AUTO-007-12345",
  "dispenser_id": "H70-STN-001-DSP-1",
  "protocol": "SAE J2601",
  "initial_state": {
    "pressure": 150.0,
    "temperature": 22.0,
    "mass": 1.2
  },
  "final_state": {
    "pressure": 695.0,
    "temperature": 45.0,
    "mass": 5.6
  },
  "refueling_params": {
    "pre_cool_temp": -38.0,
    "peak_flow_rate": 0.068,
    "average_flow_rate": 0.062,
    "total_mass": 4.4,
    "duration": 210.0,
    "pressure_ramp_rate": 2.6
  },
  "station_info": {
    "location": "Seoul Gangnam Station",
    "supply_pressure": 925.0,
    "ambient_temp": 15.0
  }
}
```

### 8.3 Fuel Cell Performance Map

```json
{
  "map_id": "PEMFC-100kW-v1.0",
  "rated_power": 100,
  "operating_points": [
    {
      "current_density": 0.2,
      "voltage": 0.85,
      "power_density": 0.17,
      "efficiency": 0.62
    },
    {
      "current_density": 0.6,
      "voltage": 0.72,
      "power_density": 0.43,
      "efficiency": 0.58
    },
    {
      "current_density": 1.0,
      "voltage": 0.65,
      "power_density": 0.65,
      "efficiency": 0.52
    },
    {
      "current_density": 1.4,
      "voltage": 0.58,
      "power_density": 0.81,
      "efficiency": 0.45
    }
  ]
}
```

---

## 9. API Interface

### 9.1 Core Functions

#### 9.1.1 Calculate Fuel Cell Efficiency

```typescript
interface FuelCellParams {
  powerOutput: number;        // kW
  hydrogenFlowRate: number;   // kg/h
  stackVoltage: number;       // V
  stackCurrent: number;       // A
}

interface EfficiencyResult {
  efficiency: number;         // 0-1
  powerDensity: number;       // kW/L
  currentDensity: number;     // A/cm²
  voltageEfficiency: number;  // 0-1
}
```

#### 9.1.2 Calculate Vehicle Range

```typescript
interface RangeParams {
  hydrogenCapacity: number;   // kg
  fuelCellEfficiency: number; // 0-1
  systemEfficiency: number;   // 0-1
  energyConsumption: number;  // MJ/km
}

interface RangeResult {
  range: number;              // km
  energyAvailable: number;    // MJ
  energyUsable: number;       // MJ
  rangeBuffer: number;        // km (reserve)
}
```

#### 9.1.3 Validate Tank Pressure

```typescript
interface TankValidation {
  pressure: number;           // bar
  temperature: number;        // °C
  tankType: 'Type III' | 'Type IV';
  standard: 'H35' | 'H70';
}

interface TankValidationResult {
  isValid: boolean;
  warnings: string[];
  errors: string[];
  safetyMargin: number;       // %
  maxAllowedPressure: number; // bar


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
