# WIA-CHIP-006 PHASE 4 — Integration

**Standard:** WIA-CHIP-006 Advanced Packaging
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `advanced-packaging-spec-v1.0.md` §8
(Chiplet Integration), §10 (Thermal Management), §11 (Power Delivery)

This document defines the integration concerns for WIA-CHIP-006:
multi-die thermal envelope, power-delivery network signing,
SBOM-and-attestation packaging via CycloneDX 1.5, and supply-chain
attestation per in-toto Attestation Framework 1.0.

References:
- CycloneDX 1.5 (SBOM format)
- SPDX 2.3 (alternate SBOM format)
- in-toto Attestation Framework 1.0 (build provenance)
- Sigstore (DSSE envelope, Rekor transparency log)

---

## 8. Chiplet Integration Standard

### 8.1 Die-to-Die Interface Protocol

Standard die-to-die interfaces SHALL support:

- **Physical layer**: UCIe (Universal Chiplet Interconnect Express) or equivalent
- **Signaling**: Single-ended or differential
- **Data rate**: Up to 32 Gbps per pin
- **Protocols**: PCIe, CXL, AXI, or custom
- **Power modes**: Active, idle, sleep

### 8.2 Chiplet Design Rules

Chiplets SHALL include:

- **Standard edge placement**: Die-to-die interfaces on designated edges
- **Power/ground distribution**: Dedicated TSVs or micro-bumps
- **Test access**: Boundary scan (IEEE 1149.1) support
- **Thermal sensors**: On-die temperature monitoring
- **Redundancy**: Optional redundant interconnects for high-reliability applications

### 8.3 HBM Integration

HBM stacks SHALL be integrated according to:

- **Interface width**: 1024 bits per channel
- **Channels**: 8 or 16 per stack
- **Data rate**: 2.4-4.0 Gbps per pin (HBM2e/HBM3)
- **Stack height**: 4, 8, or 12 dies
- **Base die**: Logic die with PHY and controllers

---


## 10. Thermal Management

### 10.1 Thermal Design Rules

Packages SHALL be designed to:

- **Junction temperature**: <125°C for automotive, <105°C for commercial
- **Thermal gradient**: <50°C across die
- **Hotspot temperature**: <10°C above average
- **Thermal cycling**: -40°C to +125°C for 1000 cycles

### 10.2 Thermal Interface Materials

TIMs SHALL meet:

- **Thermal conductivity**: >1 W/m·K (gap fillers), >3 W/m·K (thermal paste)
- **Thickness**: 50-200μm
- **Bond line thickness variation**: ±20μm
- **Pump-out resistance**: Stable after 1000 thermal cycles

### 10.3 Heat Spreader Design

Integrated heat spreaders SHALL:

- **Material**: Copper, aluminum, or composite
- **Thickness**: 0.5-3mm
- **Thermal conductivity**: >150 W/m·K
- **Flatness**: <50μm across surface

---


## 11. Power Delivery

### 11.1 Power Distribution Network

PDN SHALL be designed to:

- **IR drop**: <5% of supply voltage
- **Impedance**: <1mΩ at DC, <10mΩ at switching frequencies
- **Decoupling**: Multi-stage with on-die, package, and board capacitors
- **Resonance**: Avoid resonant peaks >10mΩ

### 11.2 Voltage Regulation

On-package regulation MAY include:

- **VR location**: On interposer or separate die
- **Efficiency**: >85% at nominal load
- **Response time**: <1μs for load transients
- **Ripple**: <50mV peak-to-peak

---


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
