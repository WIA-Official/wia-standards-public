# WIA-BIO-003 PHASE 4 — Integration Specification

**Standard:** WIA-BIO-003
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

}

interface SafetyResult {
  score: number;                // 0-100 safety score
  riskLevel: 'low' | 'medium' | 'high' | 'extreme';
  warnings: string[];
  recommendations: string[];
  monitoringPlan: {
    frequency: string;
    biomarkers: string[];
  };
}
```

#### 9.2.3 Monitor Expression
```typescript
interface ExpressionMonitoring {
  gene: string;                 // Therapeutic gene
  timePoints: number[];         // Days post-treatment
  method: 'qPCR' | 'ELISA' | 'activity-assay';
  baselineValue?: number;
}

interface ExpressionResult {
  measurements: {
    timepoint: number;
    value: number;
    unit: string;
    percentOfNormal: number;
  }[];
  trend: 'increasing' | 'stable' | 'decreasing';
  therapeuticRange: boolean;
  peakExpression: {
    value: number;
    timepoint: number;
  };
}
```

### 9.3 Data Formats

#### 9.3.1 Patient Record
```json
{
  "patientId": "PT-12345",
  "diagnosis": "Hemophilia A",
  "genotype": "F8 intron-22 inversion",
  "baselineFactorLevel": 0.5,
  "NAb_titer": {
    "AAV9": "<1:5",
    "AAV8": "1:10"
  },
  "weight": 70,
  "age": 35,
  "liverFunction": {
    "ALT": 25,
    "AST": 30,
    "totalBilirubin": 0.8
  }
}
```

#### 9.3.2 Treatment Plan
```json
{
  "treatmentId": "TX-67890",
  "patientId": "PT-12345",
  "vector": {
    "type": "AAV9",
    "gene": "F8",
    "promoter": "HLP",
    "dose_vg": 2e13,
    "dose_vg_per_kg": 2.86e12,
    "volume_ml": 50,
    "route": "IV infusion"
  },
  "premedication": [
    "Methylprednisolone 500mg IV",
    "Diphenhydramine 50mg IV"
  ],
  "monitoring": {
    "hospital_observation": "24 hours",
    "weekly_labs": "Weeks 1-4",
    "monthly_labs": "Months 2-6"
  }
}
```

### 9.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| B001 | Dose exceeds MTD | Reduce dose or split administration |
| B002 | High NAb titer detected | Consider alternative serotype |
| B003 | Severe liver dysfunction | Defer treatment until resolved |
| B004 | Off-target rate too high | Redesign guide RNA |
| B005 | Immunogenicity alert | Initiate immunosuppression |
| B006 | Vector quality failure | Do not use, investigate production |

---

## 10. References

### 10.1 Seminal Publications

2. Naldini, L. (2015). "Gene therapy returns to centre stage." *Nature* 526:351-360.
3. High, K.A., Roncarolo, M.G. (2019). "Gene Therapy." *NEJM* 381:455-464.
4. Wang, D., Tai, P.W.L., Gao, G. (2019). "Adeno-associated virus vector as a platform for gene therapy." *Nat Rev Drug Discov* 18:358-378.
5. Doudna, J.A., Charpentier, E. (2014). "The new frontier of genome engineering with CRISPR-Cas9." *Science* 346(6213).

### 10.2 Regulatory Guidance

- FDA: "Human Gene Therapy for Rare Diseases" (2020)
- EMA: "Guideline on quality, non-clinical and clinical aspects of gene therapy" (2018)
- ICH S12: "Nonclinical Biodistribution Considerations for Gene Therapy Products" (2022)

### 10.3 Clinical Trial Examples

| Disease | Vector | Gene | Dose | Status | Reference |
|---------|--------|------|------|--------|-----------|
| Hemophilia A | AAV5-hFVIII-SQ | F8 | 6×10¹³ vg/kg | Approved | Zolgensma |
| Hemophilia B | AAV-Spark200-hFIX | F9 | 2×10¹² vg/kg | Approved | Etranacogene dezaparvovec |
| SMA | AAV9-SMN1 | SMN1 | 1.1×10¹⁴ vg | Approved | Onasemnogene abeparvovec |
| LCA | AAV2-hRPE65 | RPE65 | 1.5×10¹¹ vg/eye | Approved | Voretigene neparvovec |

### 10.4 WIA Standards

- WIA-INTENT: Intent-based therapy design
- WIA-OMNI-API: Universal genomics data integration
- WIA-HEALTH: Electronic health record standards
- WIA-DATA: Clinical research data management

---

## Appendix A: Example Calculations

### A.1 Hemophilia A Treatment (AAV9-F8)

```
Given:
- Patient weight: 70 kg
- Target: Liver hepatocytes
- Vector: AAV9-F8
- Target expression: 40% of normal FVIII

Calculation:
- Hepatocyte count: ~2×10¹¹ cells
- Required vg/cell: 10⁵ (for 40% expression)
- Total vg needed: 2×10¹¹ × 10⁵ = 2×10¹⁶ vg
- Dose per kg: 2×10¹⁶ / 70 = 2.86×10¹⁴ vg/kg

Volume:
- Vector titer: 1×10¹³ vg/mL
- Required volume: 2×10¹⁶ / 10¹³ = 2000 mL (impractical)
- Concentrate to: 5×10¹³ vg/mL → 400 mL (feasible)

Result:
- Dose: 2.86×10¹⁴ vg/kg
- Volume: 400 mL IV infusion over 2 hours
```

### A.2 Off-target Risk Calculation

```
Given:
- Guide RNA: 5'-GACCCCCTCCACCCCGCCTC-3'
- Off-target site: 5'-GACCCGCTCCACCCCGCCTC-3' (1 mismatch at position 6)

Calculation:
- Mismatch at position 6 (PAM-proximal): -5 penalty
- OT_score = -5
- Risk: HIGH (score > -10)

Action:
- Redesign guide RNA or use high-fidelity Cas9 variant
```

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-003 Specification v1.0*
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
