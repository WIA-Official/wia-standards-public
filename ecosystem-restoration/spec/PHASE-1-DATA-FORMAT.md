# WIA-ENE-072: Ecosystem Restoration Standard
## Phase 01: Foundation & Requirements

### Version: 1.0
### Status: Active
### Last Updated: 2025-12-25

---

## 1. Introduction

This document defines Phase 01 of the WIA-ENE-072 Ecosystem Restoration Standard, establishing foundational requirements for planning and initiating ecosystem restoration projects.

### 1.1 Purpose

Phase 01 provides the framework for:
- Preliminary assessment and scoping
- Stakeholder identification and engagement
- Baseline data collection
- Goal setting and success criteria definition
- Regulatory compliance planning

### 1.2 Scope

This phase applies to all ecosystem restoration projects regardless of scale, ecosystem type, or geographic location.

---

## 2. Preliminary Assessment

### 2.1 Site Characterization

**REQUIREMENT 2.1.1**: Projects SHALL conduct comprehensive site characterization including:
- Geographic location and boundaries (GPS coordinates, area)
- Ecosystem type classification
- Current land use and ownership
- Access and infrastructure assessment
- Historical conditions review

**REQUIREMENT 2.1.2**: Projects SHALL document degradation factors:
- Type and severity of degradation
- Timeline of degradation
- Root causes and ongoing stressors
- Spatial extent of impacts

### 2.2 Reference Ecosystem Selection

**REQUIREMENT 2.2.1**: Projects SHALL identify appropriate reference ecosystems:
- Geographic proximity (preferably within same ecoregion)
- Similar abiotic conditions (soils, hydrology, climate)
- Minimal degradation (representative of desired state)
- Accessibility for assessment

**REQUIREMENT 2.2.2**: Reference ecosystem data SHALL include:
- Species composition and structure
- Key ecological processes
- Abiotic conditions (soil, water, topography)
- Natural disturbance regimes

---

## 3. Stakeholder Engagement

### 3.1 Stakeholder Identification

**REQUIREMENT 3.1.1**: Projects SHALL identify all relevant stakeholders:
- Landowners and adjacent property owners
- Indigenous communities with traditional territories
- Local residents and community groups
- Resource users (hunters, fishers, grazers)
- Government agencies (regulatory, management)
- Non-governmental organizations
- Academic and research institutions

**REQUIREMENT 3.1.2**: Projects SHALL assess stakeholder interests:
- Rights and legal standing
- Economic interests
- Cultural and spiritual values
- Potential conflicts and synergies

### 3.2 Engagement Planning

**REQUIREMENT 3.2.1**: Projects SHALL develop stakeholder engagement plan:
- Communication methods and frequency
- Consultation and participation opportunities
- Conflict resolution mechanisms
- Traditional knowledge integration protocols

---

## 4. Baseline Data Collection

### 4.1 Physical Environment

**REQUIREMENT 4.1.1**: Soil assessment SHALL include:
- Texture, structure, depth
- pH, organic matter, nutrients
- Bulk density and compaction
- Erosion status and soil loss

**REQUIREMENT 4.1.2**: Hydrological assessment SHALL include:
- Surface water characteristics (flow, quality)
- Groundwater conditions (depth, quality)
- Drainage patterns and alterations
- Flood regime and inundation patterns

### 4.2 Biological Communities

**REQUIREMENT 4.2.1**: Vegetation assessment SHALL include:
- Species inventory (minimum 80% of species present)
- Cover and abundance by species
- Structural attributes (layers, height, density)
- Invasive species presence and cover

**REQUIREMENT 4.2.2**: Fauna assessment SHALL include:
- Bird surveys (minimum 3 visits during breeding season)
- Mammal surveys (camera traps, sign surveys)
- Herpetofauna surveys (amphibians, reptiles)
- Aquatic invertebrates (if applicable)

### 4.3 Documentation Standards

**REQUIREMENT 4.3.1**: All baseline data SHALL be:
- Georeferenced with accuracy ±5 meters
- Photographically documented with date and location
- Collected using standardized protocols
- Entered into digital database within 30 days

---

## 5. Goal Setting

### 5.1 Restoration Vision

**REQUIREMENT 5.1.1**: Projects SHALL develop clear restoration vision:
- Broad aspirational statement
- Alignment with landscape context
- Integration of stakeholder values
- 20-50 year timeframe

### 5.2 Goals and Objectives

**REQUIREMENT 5.2.1**: Goals SHALL be SMART:
- Specific (clearly defined outcomes)
- Measurable (quantifiable metrics)
- Achievable (realistic given constraints)
- Relevant (aligned with vision)
- Time-bound (specific deadlines)

**REQUIREMENT 5.2.2**: Objectives SHALL include:
- Ecological objectives (biodiversity, function, structure)
- Social objectives (community benefits, livelihoods)
- Economic objectives (ecosystem services, cost-effectiveness)

### 5.3 Success Criteria

**REQUIREMENT 5.3.1**: Success criteria SHALL define:
- Target values for key indicators
- Timeframes for achievement
- Comparison to reference ecosystems
- Acceptable ranges of variability

---

## 6. Regulatory Compliance

### 6.1 Permits and Approvals

**REQUIREMENT 6.1.1**: Projects SHALL identify all required permits:
- Environmental impact assessments
- Endangered species consultations
- Water quality certifications
- Cultural heritage clearances
- Land use approvals

**REQUIREMENT 6.1.2**: Permit applications SHALL be submitted:
- Minimum 90 days before project start
- With complete supporting documentation
- Following agency consultation

### 6.2 Compliance Monitoring

**REQUIREMENT 6.2.1**: Projects SHALL maintain compliance:
- Permit conditions documented and tracked
- Regular compliance reporting
- Violation response procedures

---

## 7. Phase 01 Completion Criteria

### 7.1 Deliverables

Phase 01 is complete when the following are delivered:

- [ ] Site characterization report
- [ ] Reference ecosystem assessment
- [ ] Stakeholder engagement plan
- [ ] Baseline data report (all required surveys)
- [ ] Restoration goals and objectives document
- [ ] Success criteria framework
- [ ] Regulatory compliance plan
- [ ] All required permits obtained or in process

### 7.2 Review and Approval

**REQUIREMENT 7.2.1**: Phase 01 deliverables SHALL be:
- Reviewed by qualified restoration ecologist
- Approved by project steering committee
- Accepted by regulatory agencies (if applicable)
- Shared with stakeholders for comment

---

## 8. Quality Assurance

### 8.1 Data Quality

**REQUIREMENT 8.1.1**: All data SHALL meet quality standards:
- Accuracy within specified tolerances
- Completeness (no missing required fields)
- Consistency (standardized methods)
- Traceability (documented chain of custody)

### 8.2 Documentation

**REQUIREMENT 8.2.1**: All Phase 01 activities SHALL be documented:
- Field data sheets and photographs
- Meeting minutes and consultation records
- GIS data layers and maps
- Technical reports and analyses

---

## Appendix A: Required Data Standards

### A.1 Geospatial Data
- Coordinate system: WGS84 or local datum
- Minimum accuracy: ±5 meters
- Format: Shapefile, GeoJSON, or KML

### A.2 Vegetation Data
- Plot size: Standardized by vegetation type
- Species nomenclature: Current scientific names
- Cover estimates: Visual or point-intercept
- Minimum plots: 20-30 per vegetation type

### A.3 Soil Data
- Sampling depth: 0-30 cm (minimum)
- Sample density: 1 per 2-5 hectares
- Analysis: Accredited laboratory
- Parameters: pH, OM, N, P, K, texture

---

**© 2025 WIA (World Certification Industry Association)**  
**弘益人間 (홍익인간) · Benefit All Humanity**


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
