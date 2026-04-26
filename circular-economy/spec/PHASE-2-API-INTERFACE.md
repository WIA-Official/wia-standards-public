# WIA-IND-030 PHASE 2 — API Interface Specification

**Standard:** WIA-IND-030
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

- Data fields (stage-specific)
- Blockchain transaction hash (if applicable)

### 7.3 Lifecycle Event Format

```json
{
  "eventId": "EVT-2025-001234",
  "type": "refurbishment",
  "timestamp": "2025-06-15T14:30:00Z",
  "location": {
    "name": "RefurbTech Center A",
    "address": "456 Repair Street, Oakland, CA",
    "coordinates": {
      "latitude": 37.8044,
      "longitude": -122.2712
    }
  },
  "actor": "RefurbTech Inc.",
  "description": "Battery replacement and screen repair",
  "conditionBefore": "fair",
  "conditionAfter": "like-new",
  "data": {
    "partsReplaced": ["battery", "screen"],
    "cost": 125.00,
    "currency": "USD",
    "warrantyExtension": 365
  },
  "txHash": "0x7f8c..."
}
```

### 7.4 Ownership Tracking

Product ownership SHALL be tracked with:

- Owner ID and name
- Start date
- End date (if transferred)
- Ownership type (purchase, lease, rental, sharing)
- Transfer method
- Transfer price (if applicable)

### 7.5 Usage Metrics

For products with embedded sensors or connectivity, usage metrics MAY be tracked:

- Usage hours
- Usage cycles
- Distance traveled (vehicles)
- Energy consumed
- Maintenance events
- Performance degradation
- Environmental conditions

---

## 8. Circularity Metrics and Assessment

### 8.1 Material Circularity Indicator (MCI)

The Material Circularity Indicator measures how restorative material flows are in a product or company.

#### 8.1.1 MCI Calculation

```
MCI = (1 - V) × (1 - W/2)

Where:
V = Virgin material input ratio
W = Unrecoverable waste ratio
```

Detailed formula:

```
V = (M_virgin - M_recovered_from_product) / M

W = (W_product + W_process) / M

Where:
M = Total mass of product
M_virgin = Mass of virgin materials
M_recovered_from_product = Mass recovered from product at end-of-life
W_product = Unrecoverable waste from product at end-of-life
W_process = Unrecoverable waste during production
```

MCI ranges from 0 (completely linear) to 1 (perfectly circular).

### 8.2 Circularity Score

The overall circularity score (0-100) is calculated as:

```
Circularity Score = (R_in + L + D + R_out + Rep + E) / 6

Where:
R_in = Recycled input score (0-100)
L = Longevity score (0-100)
D = Design for circularity score (0-100)
R_out = End-of-life recovery score (0-100)
Rep = Reparability score (0-100)
E = Material efficiency score (0-100)
```

#### 8.2.1 Recycled Input Score

```
R_in = (Mass of recycled materials / Total mass) × 100
```

#### 8.2.2 Longevity Score

```
L = min(100, (Actual lifespan / Expected lifespan) × 100)
```

#### 8.2.3 Design for Circularity Score

Weighted average of:
- Modular design (20%)
- Disassemblability (20%)
- Standardized components (15%)
- Reparability index (20%)
- Upgradeability (15%)
- Material compatibility (10%)

#### 8.2.4 End-of-Life Recovery Score

```
R_out = Recovery rate (%)
```

#### 8.2.5 Reparability Score

```
Rep = Reparability Index × 10
```

#### 8.2.6 Material Efficiency Score

```
E = (Useful output mass / Total input mass) × 100
```

### 8.3 Circularity Rating

Based on the circularity score:

- **A (90-100)**: Excellent circularity
- **B (80-89)**: Good circularity
- **C (70-79)**: Moderate circularity
- **D (60-69)**: Limited circularity
- **E (0-59)**: Poor circularity

### 8.4 Resource Productivity

```
Resource Productivity = Economic value generated / Total material mass
```

Measured in currency per kilogram (e.g., USD/kg).

### 8.5 Waste Reduction Rate

```
Waste Reduction Rate = ((Baseline waste - Current waste) / Baseline waste) × 100
```

### 8.6 Circularity Assessment Report

A comprehensive circularity assessment SHALL include:

- Overall circularity score and rating
- MCI value
- Breakdown of sub-scores
- Carbon footprint comparison (circular vs linear)
- Waste reduction metrics
- Resource productivity
- Recommendations for improvement
- Benchmark against industry average
- Certification eligibility

---

## 9. Design for Circularity

### 9.1 Design Principles

#### 9.1.1 Design for Durability

Products SHALL be designed to:
- Use robust, high-quality materials
- Withstand expected usage conditions
- Include protective features
- Resist wear and degradation
- Meet or exceed expected lifespan

#### 9.1.2 Design for Disassembly

Products SHALL be designed to:
- Use reversible fasteners (screws, clips, snaps)
- Avoid permanent adhesives where possible
- Label material types on components
- Provide disassembly instructions
- Minimize disassembly time (< 30 minutes preferred)
- Require only standard tools

#### 9.1.3 Design for Modularity

Products SHOULD be designed with:
- Interchangeable modules
- Standardized interfaces
- Independent subsystems
- Plug-and-play components
- Common platforms across product lines

#### 9.1.4 Design for Repair

Products SHALL be designed to:
- Provide access to commonly failing parts
- Use standardized replacement parts
- Include repair manuals
- Make spare parts available for minimum 7 years
- Price spare parts reasonably (< 30% of new product)

#### 9.1.5 Design for Upgrade

Products SHOULD enable:
- Performance improvements over time
- Component upgrades
- Software updates
- Backward compatibility
- Future-proofing

#### 9.1.6 Design for Recycling

Products SHALL be designed to:
- Use mono-materials where possible
- Minimize material types (< 5 preferred)
- Avoid composite materials
- Use recyclable materials (> 90% by mass)
- Label materials clearly
- Separate incompatible materials

### 9.2 Reparability Index

The reparability index (0-10) is calculated from five criteria:

1. **Documentation** (20%): Availability of repair manuals, diagrams
2. **Disassembly** (20%): Ease of accessing parts, tool requirements
3. **Part Availability** (20%): Access to spare parts, delivery time
4. **Part Price** (20%): Cost of spare parts relative to new product
5. **Product-Specific** (20%): Additional criteria per product category

### 9.3 Material Selection Guidelines

#### 9.3.1 Preferred Materials

- Recycled content > 50%
- Recyclability > 90%
- Non-toxic
- Renewable or abundant
- Durable

#### 9.3.2 Restricted Materials

Products SHOULD avoid:
- Hazardous substances (RoHS, REACH)
- Critical raw materials (unless recycled)
- Materials difficult to recycle
- Composite materials without separation capability
- Microplastics

### 9.4 Design Documentation

Design for circularity documentation SHALL include:

- Material bill of materials (BOM)
- Disassembly instructions with diagrams
- Repair manual
- Spare parts catalog
- Expected lifetime by component
- Recycling instructions
- Material safety data sheets (MSDS)

---

## 10. Recycling and Recovery

### 10.1 Collection Systems

#### 10.1.1 Take-Back Programs

Manufacturers SHOULD implement take-back programs featuring:
- Multiple collection points
- Prepaid shipping labels
- Drop-off locations
- Incentive programs (discounts, credits)
- Clear communication to consumers

#### 10.1.2 Collection Targets

Organizations SHALL set collection targets:
- Minimum 75% collection rate within 5 years
- 85% collection rate target within 10 years
- Annual reporting of collection rates

### 10.2 Sorting and Separation

#### 10.2.1 Manual Sorting

For products requiring manual disassembly:
- Train personnel on product disassembly
- Follow manufacturer disassembly instructions
- Separate materials into categories
- Identify and remove hazardous components
- Document material quantities

#### 10.2.2 Automated Sorting

For automated processing:
- Use optical sorting for plastics
- Use magnetic separation for ferrous metals
- Use eddy current separation for non-ferrous metals
- Use density separation for mixed materials
- Use AI/vision systems for identification

### 10.3 Material Recovery

#### 10.3.1 Recovery Targets

Material recovery rates SHALL meet:
- Metals: > 95%
- Glass: > 90%
- Plastics: > 75%
- Electronics: > 85%
- Batteries: > 90%

#### 10.3.2 Recovery Quality

Recovered materials SHALL meet quality standards:
- Material purity > 95%
- Contamination < 5%
- Performance equivalent to virgin materials
- Certification of quality

### 10.4 Recycling Routes

#### 10.4.1 Mechanical Recycling

Physical processes:
- Shredding and grinding
- Washing and cleaning
- Melting and reforming
- Extrusion
- Maintains material structure

#### 10.4.2 Chemical Recycling

Chemical processes:
- Pyrolysis
- Gasification
- Depolymerization
- Solvolysis
- Breaks down to molecular level

#### 10.4.3 Biological Recycling

For organic materials:
- Composting
- Anaerobic digestion
- Enzyme treatment
- Biodegradation

### 10.5 Recycling Facility Requirements

Recycling facilities SHALL:
- Hold relevant certifications (e.g., R2, e-Stewards)
- Maintain environmental permits
- Follow safety standards
- Track material flows
- Report recovery rates
- Prevent export of hazardous waste
- Ensure worker safety

### 10.6 Recycling Route Optimization

The optimal recycling route SHALL be determined by:

```
Optimization Score = (Recovery Rate × 0.4) + (Economic Value × 0.3)
                    + (Environmental Impact × 0.2) + (Distance × 0.1)

Where all factors are normalized to 0-100 scale
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
