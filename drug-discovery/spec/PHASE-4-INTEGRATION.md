# WIA-BIO-009 PHASE 4 — Integration Specification

**Standard:** WIA-BIO-009
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 9. Implementation Guidelines

### 9.1 Required Components

Any WIA-BIO-009 compliant system must include:

1. **Compound Registry**: Unique IDs, structures, properties
2. **Assay Database**: Protocols, results, quality metrics
3. **ADMET Predictor**: Computational models
4. **Clinical Data Manager**: EDC system
5. **Regulatory Module**: Document generation

### 9.2 API Interface

#### 9.2.1 Screen Compounds
```typescript
interface ScreeningRequest {
  target: DrugTarget;
  compounds: Compound[];
  assayType: 'binding' | 'enzymatic' | 'cellular' | 'phenotypic';
  threshold: {
    ic50?: number;
    ec50?: number;
    ki?: number;
  };
}

interface ScreeningResult {
  hits: CompoundHit[];
  statistics: {
    totalCompounds: number;
    hitRate: number;
    zFactor: number;
  };
}
```

#### 9.2.2 Predict ADMET
```typescript
interface ADMETRequest {
  smiles: string;
  models: ('solubility' | 'permeability' | 'metabolism' | 'hERG' | 'cyp450')[];
}

interface ADMETProfile {
  absorption: {
    solubility: number;        // LogS
    permeability: number;      // Papp (nm/s)
    bioavailability: number;   // F%
  };
  distribution: {
    volumeOfDistribution: number;  // Vd (L/kg)
    plasmaProteinBinding: number;  // %
  };
  metabolism: {
    halfLife: number;              // t½ (h)
    clearance: number;             // CL (mL/min/kg)
    cyp450Inhibition: Record<string, number>;  // IC50 values
  };
  toxicity: {
    hERG_IC50: number;
    ames: 'positive' | 'negative' | 'inconclusive';
    hepatotoxicityRisk: 'low' | 'medium' | 'high';
  };
}
```

### 9.3 Data Formats

#### 9.3.1 Compound Structure
```json
{
  "id": "WIA-BIO-001",
  "name": "Aspirin",
  "smiles": "CC(=O)Oc1ccccc1C(=O)O",
  "inchi": "InChI=1S/C9H8O4/c1-6(10)13-8-5-3-2-4-7(8)9(11)12/h2-5H,1H3,(H,11,12)",
  "molecularWeight": 180.16,
  "formula": "C9H8O4"
}
```

#### 9.3.2 Assay Data
```json
{
  "assayId": "EGFR-Kinase-001",
  "compoundId": "WIA-BIO-001",
  "targetName": "EGFR",
  "assayType": "enzymatic",
  "ic50": 1.2e-6,
  "ic50_unit": "M",
  "hillSlope": 1.1,
  "r_squared": 0.98,
  "replicates": 3,
  "cv_percent": 8.5,
  "date": "2025-12-26"
}
```

### 9.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| B001 | Invalid SMILES | Correct structure |
| B002 | Target not found | Check target database |
| B003 | Assay quality low | Repeat experiment |
| B004 | ADMET prediction failed | Use experimental data |
| B005 | Toxicity alert | Halt development |
| B006 | Regulatory non-compliance | Update documentation |

---

## 10. References

### 10.1 Scientific Literature

2. Hopkins, A.L., Groom, C.R. (2002). "The druggable genome"
3. Zhang, M.Q., Wilkinson, B. (2007). "Drug discovery beyond the 'rule-of-five'"

### 10.2 Regulatory Guidelines

| Agency | Guideline | Topic |
|--------|-----------|-------|
| FDA | ICH M3(R2) | Nonclinical Safety Studies |
| FDA | ICH S7B | Safety Pharmacology (hERG) |
| FDA | ICH M7 | Genotoxic Impurities |
| EMA | ICH E6(R2) | Good Clinical Practice |
| FDA | 21 CFR 312 | IND Regulations |

### 10.3 Key Metrics

| Parameter | Symbol | Typical Value | Unit |
|-----------|--------|---------------|------|
| IC50 | IC50 | 10 nM - 10 μM | M |
| Binding Affinity | Kd | 1 nM - 1 μM | M |
| Lipophilicity | LogP | 0-3 | dimensionless |
| Solubility | LogS | -4 to 0 | log(M) |
| Permeability | Papp | 50-500 | nm/s |
| Bioavailability | F | 30-100 | % |
| Half-Life | t½ | 4-12 | hours |
| Clearance | CL | 5-20 | mL/min/kg |

### 10.4 WIA Standards

- WIA-INTENT: Natural language interfaces
- WIA-OMNI-API: Universal API gateway
- WIA-CHEM: Chemical structure standards
- WIA-CLINICAL: Clinical trial data
- WIA-GENOMICS: Genomic data integration

---

## Appendix A: Example Calculations

### A.1 IC50 Calculation

```
Given dose-response data:
[Compound] (μM): 0.001, 0.01, 0.1, 1, 10, 100
Inhibition (%): 5, 10, 25, 55, 85, 95

Fit to Hill equation:
Y = Bottom + (Top - Bottom) / (1 + 10^((LogIC50 - X) × HillSlope))

Result:
IC50 = 0.8 μM
Hill Slope = 1.2
R² = 0.99
```

### A.2 Lipinski Compliance

```
Compound: Aspirin
- MW = 180.16 Da ✓ (< 500)
- LogP = 1.19 ✓ (< 5)
- HBD = 1 ✓ (< 5)
- HBA = 4 ✓ (< 10)

Result: Lipinski Compliant (4/4 rules)
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-009 Specification v1.0*
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
