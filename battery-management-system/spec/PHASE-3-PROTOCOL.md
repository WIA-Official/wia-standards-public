# WIA-AUTO-006 PHASE 3 — Protocol Specification

**Standard:** WIA-AUTO-006
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 7. Safety Monitoring and Protection

### 7.1 Voltage Protection

#### 7.1.1 Overvoltage Protection (OVP)

```
IF V_cell > V_overvoltage THEN
  SET fault_OVP = TRUE
  DISABLE charging
  ALERT user
END IF
```

Thresholds:
- Warning: V_nom + 0.3V
- Protection: V_max + 0.05V
- Emergency: V_max + 0.1V

#### 7.1.2 Undervoltage Protection (UVP)

```
IF V_cell < V_undervoltage THEN
  SET fault_UVP = TRUE
  DISABLE discharging
  ALERT user
END IF
```

Thresholds:
- Warning: V_min + 0.2V
- Protection: V_min + 0.1V
- Emergency: V_min

### 7.2 Current Protection

#### 7.2.1 Overcurrent Detection

Instantaneous overcurrent:
```
I_fault = I_max_continuous × k_fault
```

Where `k_fault` = 1.2-1.5 (safety factor)

Time-delayed overcurrent (I²t protection):

```
∫ I²(t) dt > I²_threshold × t_threshold
```

#### 7.2.2 Short Circuit Protection

Detection time: < 10μs
Response time: < 100μs

```
IF I > I_short_circuit THEN
  OPEN main contactors
  ACTIVATE fuse/breaker
  SET emergency_shutdown = TRUE
END IF
```

### 7.3 Temperature Protection

Multi-level protection:

```
Level 1 (T > T_warning): Reduce current to 0.5C
Level 2 (T > T_critical): Reduce current to 0.2C
Level 3 (T > T_emergency): Emergency shutdown
```

Temperature rate limit:

```
IF (dT/dt) > rate_limit THEN
  REDUCE power by 50%
  INCREASE cooling
END IF
```

### 7.4 Isolation Monitoring

Measure insulation resistance:

```
R_isolation = V_measured / I_leakage
```

Minimum requirement: `R_isolation > 100 Ω/V`

For 400V pack: `R_min = 40 kΩ`

### 7.5 State of Function (SoF)

Available power calculation:

```
P_available = min(P_voltage, P_current, P_thermal, P_SoC)
```

Where each limit considers:

```
P_voltage = V_cell × I_max(V_cell)
P_current = V_nominal × I_max_allowed
P_thermal = P_rated × k_thermal(T)
P_SoC = P_rated × k_SoC(SoC)
```

Derating factors:

```
k_thermal(T) = {
  1.0,           T < T_nominal
  1 - (T-T_nom)/ΔT,  T_nom ≤ T < T_max
  0,             T ≥ T_max
}

k_SoC(SoC) = {
  SoC/20,        SoC < 20%
  1.0,           20% ≤ SoC ≤ 80%
  (100-SoC)/20,  SoC > 80%
}
```

---

## 8. Battery Cell Modeling

### 8.1 Equivalent Circuit Model

First-order RC model:

```
V_terminal = OCV(SoC) - I×R₀ - V_RC
dV_RC/dt = -V_RC/(R₁×C₁) + I/C₁
```

Second-order RC model (improved accuracy):

```
V_terminal = OCV(SoC) - I×R₀ - V_RC1 - V_RC2
dV_RC1/dt = -V_RC1/(R₁×C₁) + I/C₁
dV_RC2/dt = -V_RC2/(R₂×C₂) + I/C₂
```

### 8.2 Parameter Identification

#### 8.2.1 Pulse Test Method

During current pulse:

```
R₀ = ΔV_instant / ΔI
R₁ = ΔV_ss / ΔI - R₀
τ₁ = R₁ × C₁
```

From voltage response curve fitting.

#### 8.2.2 EIS (Electrochemical Impedance Spectroscopy)

Impedance at frequency f:

```
Z(jω) = R₀ + R₁/(1+jωR₁C₁) + R₂/(1+jωR₂C₂)
```

Nyquist plot fitting to extract R and C values.

### 8.3 Temperature Dependence

Resistance temperature coefficient:

```
R(T) = R(T_ref) × exp[k_R × (1/T - 1/T_ref)]
```

Capacity temperature dependence:

```
Q(T) = Q(T_ref) × [1 + k_Q × (T - T_ref)]
```

Typical values:
- `k_R` = 1000-3000 K
- `k_Q` = 0.005-0.01 K⁻¹

### 8.4 Aging Model

Capacity fade over cycles:

```
Q(n, T) = Q₀ × exp(-k_cycle × n^0.5) × exp(-k_cal × t × exp(-E_a/(R×T)))
```

Resistance growth:

```
R(n, T) = R₀ × [1 + k_R,cycle × n^0.5 + k_R,cal × t × exp(-E_a/(R×T))]
```

---

## 9. Data Formats

### 9.1 Cell Status Message

```json
{
  "cell_id": "C001",
  "voltage": 3.856,
  "temperature": 28.5,
  "soc": 75.2,
  "resistance": 0.025,
  "balancing_active": false,
  "fault_flags": 0,
  "timestamp": "2025-12-26T10:30:45.123Z"
}
```

### 9.2 Pack Status Message

```json
{
  "pack_id": "PACK001",
  "pack_voltage": 370.176,
  "pack_current": -45.3,
  "soc": 74.8,
  "soh": 92.5,
  "cell_count": 96,
  "min_cell_voltage": 3.842,
  "max_cell_voltage": 3.869,
  "avg_temperature": 29.1,
  "max_temperature": 32.4,
  "min_temperature": 26.8,
  "balancing_status": "active",
  "faults": [],
  "warnings": ["cell_imbalance"],
  "power_available": 85000,
  "energy_remaining": 52.5,
  "time_to_empty": 4200,
  "cycles": 487,
  "timestamp": "2025-12-26T10:30:45.123Z"
}
```

### 9.3 Alert/Fault Message

```json
{
  "alert_id": "A12345",
  "severity": "warning",
  "type": "overvoltage",
  "description": "Cell voltage exceeds warning threshold",
  "cell_id": "C042",
  "measured_value": 4.23,
  "threshold": 4.20,
  "action_taken": "reduced_charge_current",
  "timestamp": "2025-12-26T10:30:45.123Z"
}
```

### 9.4 Configuration Message

```json
{
  "pack_config": {
    "pack_id": "PACK001",
    "cell_count": 96,
    "series_groups": 96,
    "parallel_groups": 1,
    "cell_chemistry": "lithium-ion-nmc",
    "nominal_voltage": 3.7,
    "max_voltage": 4.2,
    "min_voltage": 2.5,
    "capacity": 75.0,
    "max_charge_current": 75.0,
    "max_discharge_current": 225.0,
    "max_temperature": 60,
    "min_temperature": -20
  },
  "protection_thresholds": {
    "overvoltage_warning": 4.15,
    "overvoltage_fault": 4.25,
    "undervoltage_warning": 2.7,
    "undervoltage_fault": 2.5,
    "overtemperature_warning": 50,
    "overtemperature_fault": 60,
    "overcurrent_charge": 90,
    "overcurrent_discharge": 270
  }
}
```

---

## 10. API Interface

### 10.1 Core Functions

#### 10.1.1 SoC Calculation

```typescript
interface SoCRequest {
  method: 'coulomb-counting' | 'ocv' | 'ekf' | 'fusion';
  initialSoC?: number;
  current: number;        // Amperes (negative for discharge)
  voltage?: number;       // Volts
  temperature?: number;   // Celsius
  duration: number;       // Seconds
  capacity: number;       // Ah
}

interface SoCResponse {
  percentage: number;     // 0-100%
  confidence: number;     // 0-1
  method: string;
  timestamp: Date;
}
```

#### 10.1.2 SoH Calculation

```typescript
interface SoHRequest {
  method: 'capacity' | 'resistance' | 'impedance' | 'model';


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
