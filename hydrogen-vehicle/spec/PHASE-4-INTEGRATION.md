# WIA-AUTO-007 PHASE 4 — Integration Specification

**Standard:** WIA-AUTO-007
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

}
```

#### 9.1.4 Optimize Refueling

```typescript
interface RefuelingParams {
  targetPressure: number;     // bar
  ambientTemp: number;        // °C
  currentPressure: number;    // bar
  currentTemp: number;        // °C
  tankVolume: number;         // L
}

interface RefuelingPlan {
  preCoolTemp: number;        // °C
  flowRate: number;           // kg/min
  estimatedTime: number;      // min
  finalTemp: number;          // °C
  safetyChecks: SafetyCheck[];
}
```

---

## 10. Safety Protocols

### 10.1 Hydrogen Leak Detection

#### 10.1.1 Detection Threshold

Lower Explosive Limit (LEL) for H2: 4% by volume in air

Detection threshold: 1% LEL (0.04% H2)

#### 10.1.2 Sensor Requirements

- **Type**: Catalytic bead or electrochemical
- **Response Time**: <5 seconds
- **Location**: Near tank, fuel cell, refueling port
- **Redundancy**: Minimum 2 sensors per critical area

### 10.2 Pressure Relief Systems

#### 10.2.1 Thermally Activated Pressure Relief Device (TPRD)

**Activation Temperature**: 110°C ± 10°C

**Flow Capacity**: Discharge all H2 in <60 seconds

**Location**: On each storage tank

#### 10.2.2 Pressure Relief Valve (PRV)

**Set Pressure**: 125% of nominal working pressure
- H70: 875 bar
- H35: 437.5 bar

### 10.3 Crash Safety

#### 10.3.1 Tank Protection

- Tanks located in protected zone
- Front and rear crumple zones
- Side impact protection bars
- Underbody protection plate

#### 10.3.2 Emergency Shut-off

Automatic H2 shut-off in case of:
- Collision detection (>10g deceleration)
- Rollover detection (>60° tilt)
- Fire detection
- Manual trigger

**Shut-off Time**: <100 milliseconds

### 10.4 Ventilation Requirements

#### 10.4.1 Enclosed Spaces

Minimum air exchange: 4 ACH (air changes per hour)

Hydrogen rises, so ventilation at ceiling level

#### 10.4.2 Garage/Parking

Natural or mechanical ventilation required

No ignition sources within 3 meters of vehicle

### 10.5 Fire Safety

#### 10.5.1 Fire Classification

Hydrogen fires are Class C (flammable gas)

Do not extinguish unless supply can be shut off

#### 10.5.2 Safety Distances

In case of TPRD activation:
- 10 meters minimum safe distance
- 25 meters for fire/rescue personnel approach

### 10.6 Maintenance Safety

#### 10.6.1 Pre-maintenance Checks

- Verify zero pressure in H2 system
- Purge system with nitrogen
- Ground vehicle to prevent static
- Use non-sparking tools

#### 10.6.2 Personal Protective Equipment

- Safety glasses
- Fire-resistant clothing
- Grounded anti-static footwear
- H2 gas detector (personal)

---

## 11. References

### 11.1 Technical Standards

1. SAE J2601: "Fueling Protocols for Light Duty Gaseous Hydrogen Surface Vehicles"
2. ISO 19881: "Gaseous hydrogen — Land vehicle fuel containers"
3. ISO 19882: "Gaseous hydrogen — Thermally activated pressure relief devices"
4. SAE J2719: "Hydrogen Fuel Quality for Fuel Cell Vehicles"
5. GTR No. 13: "Global Technical Regulation on Hydrogen Fuel Cell Vehicles"

### 11.2 Fuel Cell Technology

1. Larminie, J., Dicks, A. (2003). "Fuel Cell Systems Explained"
3. Zhang, J. (2008). "PEM Fuel Cell Electrocatalysts and Catalyst Layers"

### 11.3 Hydrogen Storage

2. DOE (2020). "Hydrogen Storage Technical Team Roadmap"

### 11.4 Vehicle Integration

1. Toyota (2015). "Mirai Technical Overview"
2. Hyundai (2020). "NEXO Fuel Cell Vehicle Technology"
3. Honda (2016). "Clarity Fuel Cell Technical Guide"

### 11.5 Safety Standards

1. ISO/TR 15916: "Basic considerations for the safety of hydrogen systems"
2. NFPA 2: "Hydrogen Technologies Code"
3. SAE J2578: "Recommended Practice for General Fuel Cell Vehicle Safety"

### 11.6 Physical Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Lower Heating Value (H2) | LHV_H2 | 120 MJ/kg |
| Higher Heating Value (H2) | HHV_H2 | 142 MJ/kg |
| Hydrogen Density (STP) | ρ_H2 | 0.0899 kg/m³ |
| H2 Specific Heat | c_p | 14.3 kJ/kg·K |
| Faraday Constant | F | 96,485 C/mol |
| Universal Gas Constant | R | 8.314 J/mol·K |

### 11.7 WIA Standards

- WIA-INTENT: Intent-based vehicle control
- WIA-OMNI-API: Universal automotive API
- WIA-ENERGY: Smart energy systems
- WIA-SOCIAL: Vehicle communication protocols

---

## Appendix A: Example Calculations

### A.1 Fuel Cell Efficiency

```
Given:
- Stack Power: 100 kW
- Stack Voltage: 400 V
- Stack Current: 250 A
- H2 Flow Rate: 0.9 kg/h

Calculation:
- Power Output: P = V × I = 400 × 250 = 100,000 W = 100 kW
- H2 Energy Input: E_in = 0.9 kg/h × 120 MJ/kg = 108 MJ/h = 30 kW
- Efficiency: η = 100 / 30 = 0.60 or 60%

Result: Fuel cell operating at 60% efficiency (typical for PEMFC)
```

### A.2 Vehicle Range

```
Given:
- H2 Capacity: 5.6 kg
- FC Efficiency: 60%
- System Efficiency: 90%
- Motor Efficiency: 94%
- Energy Consumption: 0.95 MJ/km

Calculation:
- Total Energy: E = 5.6 × 120 = 672 MJ
- Usable Energy: E_usable = 672 × 0.60 × 0.90 × 0.94 = 341 MJ
- Range: R = 341 / 0.95 = 359 km

Result: Vehicle range approximately 359 km (similar to Toyota Mirai)
```

### A.3 Refueling Time

```
Given:
- Target Mass: 5.6 kg
- Current Mass: 0.5 kg
- Flow Rate: 0.060 kg/s

Calculation:
- Mass to Fill: Δm = 5.6 - 0.5 = 5.1 kg
- Fill Time: t = 5.1 / 0.060 = 85 seconds ≈ 1.4 minutes

Plus pre-cooling and pressurization ramp: Total ≈ 3-4 minutes
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-007 Specification v1.0*
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
