# WIA-SPACE-027 PHASE 3 — Protocol

**Standard:** WIA-SPACE-027 Asteroid Mining
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-SPACE-027-v1.0.md` §6 (Space
Refining and Processing) and §7 (Transportation)

This document defines the protocols between mining vehicle, refinery
platform, transport, and Earth-side ingestion: hand-off events,
custody-chain anchoring, and capability negotiation. Anchoring uses
in-toto Attestation Framework 1.0 with Sigstore Rekor entries to make
provenance verifiable end-to-end.

References:
- in-toto Attestation Framework 1.0
- Sigstore (DSSE envelope, Rekor transparency log)
- IETF RFC 9162 (Certificate Transparency v2.0; same anchoring pattern)

---

## 6. Space Refining and Processing

### 6.1 Water Electrolysis

Electrolysis systems SHALL meet:

- **Voltage:** 1.48V (operating), 1.23V (theoretical minimum)
- **Efficiency:** > 70% (electrical to chemical energy)
- **Output:** H₂ : O₂ mass ratio = 1:8
- **Power:** 100 kW electrolizer produces 111 kg H₂ + 889 kg O₂ per 1000 kg H₂O

### 6.2 Orbital Smelting

Metal refining SHALL use:

- **Solar Concentrators:** Fresnel lenses or parabolic mirrors (temperature > 2000°C)
- **Electric Arc Furnace:** For precise temperature control
- **Vacuum Advantage:** No oxidation, higher purity metals

**Standard:** All refined metals SHALL be certified for composition and tested for mechanical properties.

### 6.3 3D Printing

Additive manufacturing SHALL support:

- **Materials:** Titanium, aluminum, iron-nickel alloy, silicate concrete
- **Technologies:** SLM (Selective Laser Melting), EBM (Electron Beam Melting)
- **Quality:** Parts SHALL meet ASTM standards for space applications
- **Documentation:** Full traceability of material provenance


## 7. Transportation

### 7.1 Orbital Transfer

Transport vehicles SHALL calculate optimal trajectories using:

- **Lambert Solver:** Find minimum Δv trajectory for given time-of-flight
- **Pork-Chop Plots:** Identify optimal launch windows
- **Gravity Assists:** Utilize Earth-Moon system for Δv reduction

### 7.2 Propulsion Systems

Cargo transport SHALL use:

| Propulsion | Isp (seconds) | Thrust (N) | Power (kW) | Payload (tonnes) |
|------------|---------------|------------|------------|------------------|
| Chemical (LH₂/LOX) | 450 | 10,000+ | - | 10-50 |
| Ion Engine | 3000-5000 | 0.1-1.0 | 50-200 | 100-500 |
| Hall Thruster | 1500-2000 | 0.5-5.0 | 10-100 | 50-200 |
| Solar Sail | ∞ | 0.01-0.1 | 0 | 10-100 |

**Mission Duration:** NEA to Earth orbit: 12-36 months (electric propulsion).

### 7.3 Atmospheric Reentry

Return capsules SHALL comply with:

- **Entry Velocity:** 11-13 km/s
- **Peak Temperature:** < 3000°C (heat shield design limit)
- **Landing Precision:** < 10 km from target zone
- **Safety Zones:** No reentry over populated areas (notification 7 days prior)

### 7.4 Lagrange Point Depots

Resource depots SHOULD be established at Earth-Moon L1/L2 for:

- **Fuel Storage:** LH₂/LOX for lunar/Mars missions
- **Construction Materials:** Silicates for space station assembly
- **Staging Point:** Minimize Δv for cislunar operations


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
