# WIA-AUTO-006 PHASE 4 — Integration Specification

**Standard:** WIA-AUTO-006
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

currentCapacity?: number;    // Ah
  ratedCapacity: number;       // Ah
  currentResistance?: number;  // Ohms
  initialResistance?: number;  // Ohms
  cycles?: number;
  age?: number;                // Days
  temperature?: number;        // Celsius
}

interface SoHResponse {
  percentage: number;          // 0-100%
  remainingLife: number;       // Estimated cycles
  degradationRate: number;     // %/cycle
  confidence: number;          // 0-1
  timestamp: Date;
}
```

#### 10.1.3 Cell Balancing

```typescript
interface BalancingRequest {
  method: 'passive' | 'active';
  cellVoltages: number[];      // Volts
  targetDelta: number;         // Maximum voltage difference (V)
  maxCurrent?: number;         // Balancing current limit (A)
  maxTime?: number;            // Maximum balancing time (s)
}

interface BalancingResponse {
  cellsToBalance: number[];    // Cell indices
  estimatedTime: number;       // Seconds
  energyDissipated: number;    // Joules
  method: string;
  status: 'complete' | 'in-progress' | 'failed';
}
```

#### 10.1.4 Thermal Management

```typescript
interface ThermalRequest {
  temperatures: number[];      // Celsius (multiple sensors)
  ambientTemp: number;         // Celsius
  packCurrent: number;         // Amperes
  coolingMethod: 'air' | 'liquid' | 'pcm';
}

interface ThermalResponse {
  maxTemperature: number;      // Celsius
  avgTemperature: number;      // Celsius
  hotspotLocation: number;     // Sensor index
  coolingRequired: boolean;
  coolingPower: number;        // Watts
  timeToLimit: number;         // Seconds until T_max
  status: 'ok' | 'warning' | 'critical';
}
```

### 10.2 Safety Functions

#### 10.2.1 Protection Check

```typescript
interface ProtectionRequest {
  cellVoltages: number[];
  packCurrent: number;
  temperatures: number[];
  soc: number;
}

interface ProtectionResponse {
  safe: boolean;
  faults: Fault[];
  warnings: Warning[];
  actionRequired: Action[];
  powerLimit: number;          // Watts
}
```

---

## 11. Communication Protocols

### 11.1 CAN Bus Messages

#### 11.1.1 BMS Status (ID: 0x100)

| Byte | Bits | Description | Unit | Scale |
|------|------|-------------|------|-------|
| 0-1 | 0-15 | Pack Voltage | V | 0.1 |
| 2-3 | 16-31 | Pack Current | A | 0.1 |
| 4 | 32-39 | SoC | % | 1 |
| 5 | 40-47 | Max Temp | °C | 1 |
| 6 | 48-55 | Min Cell V | V×100 | 1 |
| 7 | 56-63 | Status Flags | - | bitmap |

Update rate: 10 Hz

#### 11.1.2 Cell Voltages (ID: 0x101-0x110)

16 messages, 6 cell voltages each (96 cells total)

| Byte | Bits | Description | Unit | Scale |
|------|------|-------------|------|-------|
| 0-1 | 0-15 | Cell N+0 | mV | 1 |
| 2-3 | 16-31 | Cell N+1 | mV | 1 |
| 4-5 | 32-47 | Cell N+2 | mV | 1 |
| 6-7 | 48-63 | Cell N+3 | mV | 1 |

Update rate: 1 Hz

#### 11.1.3 Faults and Warnings (ID: 0x120)

| Byte | Description |
|------|-------------|
| 0 | Fault flags byte 1 |
| 1 | Fault flags byte 2 |
| 2 | Warning flags byte 1 |
| 3 | Warning flags byte 2 |
| 4-7 | Fault data |

Fault flags (bit mapping):
- Bit 0: Overvoltage
- Bit 1: Undervoltage
- Bit 2: Overcurrent charge
- Bit 3: Overcurrent discharge
- Bit 4: Overtemperature
- Bit 5: Undertemperature
- Bit 6: Cell imbalance
- Bit 7: Communication error

### 11.2 I²C Internal Communication

Slave addresses:
- 0x50-0x5F: Cell monitoring ICs
- 0x48-0x4F: Temperature sensors
- 0x68: Real-time clock
- 0x70: EEPROM

Clock speed: 100 kHz (standard) or 400 kHz (fast mode)

### 11.3 Diagnostic Interface

UART or USB interface for diagnostics:
- Baud rate: 115200
- Data format: 8N1
- Protocol: JSON over serial

Commands:
```
GET_STATUS
GET_CELLS
GET_HISTORY
SET_CONFIG
CALIBRATE
RESET_FAULTS
```

---

## 12. Safety and Compliance

### 12.1 Functional Safety (ISO 26262)

ASIL (Automotive Safety Integrity Level) requirements:

| Function | ASIL | Requirement |
|----------|------|-------------|
| Overvoltage Protection | C | Dual redundant |
| Overcurrent Protection | C | Hardware + software |
| Thermal Protection | B | Dual sensors |
| SoC Estimation | A | Single method acceptable |

### 12.2 Standards Compliance

- **UN/ECE R100.02**: Electric vehicle safety
- **IEC 62619**: Secondary cells and batteries for industrial applications
- **UL 2580**: Batteries for use in electric vehicles
- **SAE J2464**: Electric and hybrid vehicle battery systems crash integrity
- **ISO 6469-1**: Electric road vehicles - Safety specifications

### 12.3 Testing Requirements

**Performance tests**:
- SoC accuracy: ±5% over full range
- SoH accuracy: ±10% over lifetime
- Response time: < 100ms for faults
- Data logging: No data loss for 24h

**Environmental tests**:
- Temperature: -40°C to +85°C
- Humidity: 5% to 95% RH
- Vibration: Per ISO 16750-3
- EMC: Per ISO 11452

**Safety tests**:
- Short circuit protection
- Overcharge protection
- Over-discharge protection
- Thermal runaway prevention

### 12.4 Certification Requirements

Minimum requirements for WIA-AUTO-006 certification:

1. Full compliance with data formats (Section 9)
2. Implementation of at least 2 SoC methods
3. Implementation of cell balancing (passive or active)
4. All safety protections (Section 7)
5. CAN bus communication (11.1)
6. Passing certification test suite

---

## 13. References

### 13.1 Technical Standards

1. ISO 26262: Road vehicles - Functional safety
2. IEC 62619: Secondary cells and batteries containing alkaline or other non-acid electrolytes
3. SAE J2464: Electric and hybrid vehicle rechargeable energy storage system (RESS) safety and abuse testing
4. UL 2580: Batteries for use in electric vehicles
5. UN/ECE R100.02: Electric vehicle safety requirements

### 13.2 Scientific References

1. Plett, G.L. (2015). "Battery Management Systems, Volume I: Battery Modeling"
2. Plett, G.L. (2015). "Battery Management Systems, Volume II: Equivalent-Circuit Methods"
3. Andrea, D. (2010). "Battery Management Systems for Large Lithium-Ion Battery Packs"

### 13.3 Battery Chemistry References

| Chemistry | Reference | Key Parameters |
|-----------|-----------|----------------|
| NMC | Journal of Power Sources, 2015 | V_nom=3.7V, Q=40-100Ah |
| LFP | Journal of Power Sources, 2016 | V_nom=3.2V, Q=50-200Ah |
| NCA | Journal of the Electrochemical Society | V_nom=3.6V, Q=40-80Ah |

### 13.4 WIA Standards

- WIA-INTENT: Intent-based control interfaces
- WIA-OMNI-API: Universal API gateway
- WIA-AUTO-001: Vehicle energy management
- WIA-ENERGY: Grid energy storage
- WIA-SOCIAL: Fleet management and data sharing

---

## Appendix A: Example Calculations

### A.1 SoC Calculation (Coulomb Counting)

```
Given:
- Initial SoC: 80%
- Discharge current: 50A
- Duration: 1 hour (3600 seconds)
- Battery capacity: 75 Ah
- Coulombic efficiency: 0.98

Calculation:
- Charge removed: Q = 50A × 1h = 50 Ah
- With efficiency: Q_eff = 50 / 0.98 = 51.02 Ah
- SoC change: ΔSoC = (51.02 / 75) × 100% = 68.03%
- Final SoC: 80% - 68.03% = 11.97% ≈ 12%
```

### A.2 Cell Balancing Time

```
Given:
- Cell voltages: [3.85V, 3.87V, 3.84V, 3.86V]
- Balancing resistor: 50Ω
- Target imbalance: < 10mV

Calculation:
- Max voltage: 3.87V
- Min voltage: 3.84V
- Imbalance: 30mV
- Balancing current: I = 3.87V / 50Ω = 77.4mA
- Charge to remove (approx): Q = C×ΔV ≈ 2F × 0.03V = 60mC
- Time: t = Q / I = 60mC / 77.4mA ≈ 0.77s

(Simplified calculation; actual time depends on cell capacity and discharge curve)
```

### A.3 Thermal Management

```
Given:
- Heat generation: 500W
- Pack surface area: 2 m²
- Ambient temperature: 25°C
- Maximum cell temperature: 45°C
- Heat transfer coefficient (air): 10 W/m²·K

Calculation:
- Temperature rise with natural convection:
  ΔT = Q / (h×A) = 500W / (10 W/m²·K × 2m²) = 25°C
- Resulting temperature: 25°C + 25°C = 50°C

Exceeds limit! Need forced cooling:
- Required h: h = Q / (A×ΔT) = 500W / (2m² × 20°C) = 12.5 W/m²·K
- Forced air: h ≈ 25-250 W/m²·K ✓
- Liquid cooling: h ≈ 500-10,000 W/m²·K ✓
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-006 Specification v1.0*
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
