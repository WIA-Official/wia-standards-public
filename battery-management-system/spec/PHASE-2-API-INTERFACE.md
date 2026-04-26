# WIA-AUTO-006 PHASE 2 — API Interface Specification

**Standard:** WIA-AUTO-006
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 4. State of Health (SoH) Monitoring

### 4.1 Capacity-Based SoH

```
SoH_capacity = (Q_current / Q_rated) × 100%
```

Where:
- `Q_current` = Current maximum capacity (Ah)
- `Q_rated` = Rated capacity when new (Ah)

#### 4.1.1 Capacity Measurement

Full discharge method:
```
Q_measured = ∫[t_full, t_empty] |I(τ)| dτ
```

Partial discharge with SoC estimation:
```
Q_estimated = ΔQ / (SoC_start - SoC_end) × 100
```

### 4.2 Resistance-Based SoH

```
SoH_resistance = (R_initial / R_current) × 100%
```

Internal resistance measurement:
```
R_internal = (V₁ - V₂) / (I₁ - I₂)
```

During pulse test with current change.

### 4.3 Impedance-Based SoH

Complex impedance at frequency ω:

```
Z(jω) = R_s + (R_p / (1 + jωR_p×C_p))
```

SoH indicators:
- Increasing real part → degradation
- Phase angle shift → chemistry changes

### 4.4 Cycle Life Modeling

Cycle-based degradation:

```
Q_fade(n) = Q_rated × [1 - k_cycle × n^α]
```

Where:
- `n` = Number of cycles
- `k_cycle` = Degradation rate constant
- `α` = Degradation exponent (typically 0.5-0.8)

Calendar aging:

```
Q_calendar(t) = Q_rated × [1 - k_cal × t^β × exp(E_a/(R×T))]
```

Where:
- `t` = Time in storage (days)
- `k_cal` = Calendar aging constant
- `β` = Time exponent
- `E_a` = Activation energy
- `R` = Gas constant
- `T` = Temperature (K)

### 4.5 SoH Fusion

Combined SoH estimation:

```
SoH = α₁×SoH_cap + α₂×SoH_res + α₃×SoH_model
```

With confidence-weighted coefficients.

---

## 5. Cell Balancing

### 5.1 Balancing Necessity

Imbalance metric:

```
ΔV = V_max - V_min
```

Balancing threshold: `ΔV > 10mV` (typical)

### 5.2 Passive Balancing

#### 5.2.1 Resistive Discharge

Energy dissipation from highest cells:

```
E_dissipated = ∫ V_cell × I_balance dt
```

Balancing current:
```
I_balance = (V_cell - V_target) / R_balance
```

Typical values:
- `R_balance` = 10-100 Ω
- `I_balance` = 50-200 mA
- `P_dissipated` = 0.2-1 W per cell

#### 5.2.2 Algorithm

```
FOR each cell i:
  IF V[i] > V_avg + threshold:
    ENABLE balancing resistor[i]
    SET timer[i] = calculate_time(V[i], V_target)
  ELSE:
    DISABLE balancing resistor[i]
```

### 5.3 Active Balancing

#### 5.3.1 Capacitive Balancing

Charge transfer via flying capacitor:

```
Q_transferred = C × (V_high - V_low)
```

Energy efficiency:
```
η_active = E_received / E_removed ≈ 0.85-0.95
```

#### 5.3.2 Inductive Balancing

DC-DC converter approach:

```
V_out/V_in = D/(1-D)
```

Where `D` is duty cycle (0-1).

Transfer power:
```
P_transfer = V_in × I_avg × D
```

#### 5.3.3 Algorithm

```
1. Identify highest (H) and lowest (L) cell
2. Calculate energy difference: ΔE = C×(V_H² - V_L²)/2
3. Activate transfer circuit H→L
4. Monitor voltage convergence
5. Stop when |V_H - V_L| < threshold
```

### 5.4 Balancing Strategy Selection

| Condition | Method | Reason |
|-----------|--------|--------|
| ΔV < 50mV | Passive | Simple, low cost |
| ΔV > 50mV | Active | Faster, less heat |
| Charging | Passive | Time available |
| Driving | Active | Efficiency critical |
| Large pack | Active | Scalability |

---

## 6. Thermal Management

### 6.1 Heat Generation Model

Total heat generation:

```
Q_total = Q_joule + Q_reaction + Q_mixing
```

#### 6.1.1 Joule Heating

```
Q_joule = I² × R_internal
```

#### 6.1.2 Reaction Heat

```
Q_reaction = I × T × (dOCV/dT)
```

Where:
- `T` = Absolute temperature (K)
- `dOCV/dT` = Entropy coefficient (typically -0.5 to 0.5 mV/K)

#### 6.1.3 Mixing/Polarization Heat

```
Q_mixing = I × (V_terminal - OCV - I×R_internal)
```

### 6.2 Thermal Model

Lumped parameter model:

```
m×c_p×(dT/dt) = Q_total - h×A×(T - T_ambient)
```

Where:
- `m` = Mass (kg)
- `c_p` = Specific heat capacity (J/kg·K)
- `h` = Heat transfer coefficient (W/m²·K)
- `A` = Surface area (m²)

Multi-node model for pack:

```
C_i×(dT_i/dt) = Q_gen,i + Σ[G_ij×(T_j - T_i)] - h_i×A_i×(T_i - T_amb)
```

Where:
- `C_i` = Thermal capacitance of node i
- `G_ij` = Thermal conductance between nodes i and j
- `Q_gen,i` = Heat generation at node i

### 6.3 Cooling Requirements

Required cooling power:

```
P_cooling = Q_total - C×(dT/dt)_allowed
```

For steady state at elevated temperature:

```
P_cooling = h×A×(T_cell - T_ambient)
```

Solving for required h:

```
h_required = Q_total / [A×(T_max - T_amb)]
```

### 6.4 Thermal Management Strategies

#### 6.4.1 Air Cooling

Fan power calculation:

```
P_fan = (ṁ × Δp) / (ρ × η_fan)
```

Where:
- `ṁ` = Mass flow rate (kg/s)
- `Δp` = Pressure drop (Pa)
- `ρ` = Air density (kg/m³)
- `η_fan` = Fan efficiency

#### 6.4.2 Liquid Cooling

Heat transfer with coolant:

```
Q = ṁ×c_p×(T_out - T_in)
```

Required flow rate:

```
ṁ = Q_total / [c_p×(T_out - T_in)]
```

#### 6.4.3 Phase Change Materials

Energy storage in PCM:

```
Q_stored = m_PCM × [c_p,s×ΔT + L_fusion + c_p,l×ΔT]
```

Where:
- `L_fusion` = Latent heat of fusion (J/kg)
- `c_p,s`, `c_p,l` = Specific heat (solid, liquid)

### 6.5 Temperature Monitoring

Sensor placement requirements:
- Minimum 1 sensor per 8 cells
- 1 sensor at pack inlet/outlet
- 1 sensor at hottest predicted location

Alert thresholds:
```
T_warning = T_nominal + 20°C
T_critical = T_nominal + 35°C
T_emergency = T_max - 5°C
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
