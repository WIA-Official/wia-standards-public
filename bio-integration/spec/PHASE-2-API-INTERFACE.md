# WIA-AUG-011 PHASE 2 — API Interface Specification

**Standard:** WIA-AUG-011
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. Neural Integration Pathways

### 5.1 Neural Interface Types

```
Peripheral Nerve Interfaces:
- Epineural electrodes: Around nerve (non-penetrating)
- Intraneural electrodes: Within nerve (penetrating)
- Regenerative electrodes: Sieve/channel guides

Central Nervous System Interfaces:
- Cortical surface arrays: Epidural/subdural
- Intracortical probes: Penetrating microelectrodes
- Deep brain stimulation: Subcortical targets
```

### 5.2 Neural Integration Timeline

```
Acute Phase (0-4 weeks):
- Surgical injury response
- Blood-brain/nerve barrier disruption
- Acute inflammation
- Initial glial scarring

Metrics:
- Impedance: 2-10x baseline (elevated)
- Signal amplitude: 50-70% of acute recording
- Neuronal loss: 0-50 μm radius

Subacute Phase (1-3 months):
- Chronic inflammation subsides
- Glial scar formation stabilizes
- Some neural regeneration
- Interface encapsulation

Metrics:
- Impedance: 1.5-3x baseline
- Signal amplitude: 40-60% of acute
- Recording channels: 60-80% functional

Chronic Phase (3+ months):
- Stable glial scar (10-100 μm)
- Steady-state tissue response
- Long-term recording stability

Metrics:
- Impedance: 1.2-2x baseline (stable)
- Signal amplitude: 30-50% of acute
- Recording channels: 40-70% functional
- Longevity: Varies (months to years)
```

### 5.3 Neural Integration Optimization

```typescript
interface NeuralIntegrationProtocol {
  // Electrode design
  electrodeGeometry: {
    tip Diameter: number;               // μm (1-100)
    shankLength: number;                // mm
    surfaceArea: number;                // μm²
    siteSpacing: number;                // μm (50-400)
  };

  // Material selection
  materials: {
    conductor: string;                  // Pt, IrOx, TiN, PEDOT
    insulation: string;                 // Parylene, polyimide
    coating: string[];                  // Anti-fouling, bioactive
  };

  // Insertion technique
  insertion: {
    speed: number;                      // mm/s (1-2 optimal)
    angle: number;                      // degrees
    pneumaticVsBlade: string;           // insertion method
    multiStage: boolean;                // staged insertion
  };

  // Post-implant care
  stabilization: {
    antiInflammatory: boolean;          // Dexamethasone delivery
    neurotrophicFactors: boolean;       // NGF, BDNF, GDNF
    electricalStimulation: boolean;     // Promotes integration
  };
}
```

### 5.4 Signal Quality Metrics

```
Neural Recording Quality:
- Signal-to-noise ratio: >3:1 (single unit), >10:1 (optimal)
- Unit yield: >0.5 units/channel
- Spike amplitude: >50 μV (single unit)
- Noise floor: <10 μV RMS

Neural Stimulation Efficacy:
- Activation threshold: <1 mA
- Selectivity: >80% target activation
- Current density: <350 μC/cm²/phase
- Charge balance: >99%
```

---

## 6. Vascular Integration

### 6.1 Blood-Device Interface

```
Vascular Interface Types:
1. Endovascular:
   - Catheter/stent luminal surface
   - Direct blood contact
   - High thrombosis risk

2. Percutaneous:
   - Vessel wall penetration
   - Transmural integration
   - Infection risk

3. Vascular Graft:
   - Replacement vessel segment
   - Endothelialization required
   - Flow dynamics critical
```

### 6.2 Thrombosis Prevention

```
Surface Treatments:
- Heparin coating: Anti-coagulant
- Endothelial cell seeding: Living surface
- Phosphorylcholine: Biomimetic surface
- Drug-eluting: Sustained antiproliferative

Design Considerations:
- Smooth surface: Ra <0.1 μm
- Minimize flow disruption
- Avoid stagnant zones
- Laminar flow maintenance

Assessment:
- Platelet adhesion: <5% surface coverage
- Thrombin generation: <10 nM
- Fibrin formation: Minimal
- Endothelialization: >80% coverage (4-8 weeks)
```

### 6.3 Vascular Integration Metrics

```typescript
interface VascularIntegrationStatus {
  // Thrombogenicity
  thrombusFormation: boolean;
  plateletActivation: number;           // % activated
  coagulationCascade: string;           // Normal/Activated

  // Endothelialization
  endothelialCoverage: number;          // % surface
  cellMorphology: string;               // Cobblestone/Irregular
  flowResponsiveness: boolean;          // Shear stress adaptation

  // Hemodynamics
  flowVelocity: number;                 // cm/s
  wallShearStress: number;              // dynes/cm²
  turbulence: boolean;

  // Inflammation
  cReactiveProtein: number;             // mg/L
  interleukin6: number;                 // pg/mL
  endothelialActivation: boolean;
}
```

### 6.4 Neovascularization

```
Definition: Formation of new blood vessels to supply peri-implant tissue

Promotion Strategies:
- VEGF (Vascular Endothelial Growth Factor) incorporation
- Porous scaffold design (100-300 μm pores)
- Hypoxia-mimicking surface chemistry
- Pro-angiogenic coating

Assessment:
- Vessel density: >50 vessels/mm² (optimal)
- Perfusion: >80% of native tissue
- Vessel diameter: 10-100 μm
- Functional flow: Confirmed by imaging (ultrasound, OCT)

Timeline:
- Week 1-2: Angiogenic sprouting
- Week 2-4: Vessel network formation
- Week 4-8: Vessel maturation and stabilization
- Week 8+: Long-term vascular supply
```

---

## 7. Immune System Modulation

### 7.1 Foreign Body Response

```
Acute Inflammation (0-7 days):
- Neutrophil infiltration
- Cytokine release: IL-1β, IL-6, TNF-α
- Complement activation
- Mast cell degranulation

Chronic Inflammation (1-4 weeks):
- Macrophage accumulation
- Foreign body giant cell (FBGC) formation
- M1 (pro-inflammatory) macrophage dominance

Fibrous Encapsulation (2-8 weeks):
- Fibroblast migration and proliferation
- Collagen deposition
- Fibrous capsule formation
- Thickness: 10-500 μm (varies by material)

Resolution (2+ months):
- M2 (healing) macrophage polarization
- Reduced inflammation
- Stable fibrous capsule
- Neovascularization of capsule
```

### 7.2 Immune Response Metrics

```typescript
interface ImmuneResponse {
  // Inflammatory markers
  cytokines: {
    il6: number;                        // pg/mL
    tnfAlpha: number;                   // pg/mL
    il10: number;                       // pg/mL (anti-inflammatory)
  };

  // Cellular response
  macrophageRatio: {
    m1: number;                         // % M1 (pro-inflammatory)
    m2: number;                         // % M2 (healing)
    ratio: number;                      // M1/M2 (lower is better)
  };

  // Fibrosis
  fibrousCapsule: {
    present: boolean;
    thickness: number;                  // μm
    vascularity: string;                // None/Sparse/Moderate/High
    cellularity: string;                // Low/Moderate/High
  };

  // Immunogenicity
  antibodyProduction: boolean;
  complementActivation: boolean;
  tcellResponse: boolean;
}
```

### 7.3 Immunomodulation Strategies

```
Material Selection:
- Low immunogenicity: Titanium, PEEK, silicone
- Surface modification: PEG, zwitterionic polymers
- Biomimetic coatings: ECM proteins

Drug Delivery:
- Corticosteroids: Dexamethasone (local)
- Immunosuppressants: Tacrolimus (localized)
- Anti-inflammatory: NSAIDs, COX-2 inhibitors

Design Optimization:
- Minimize surface area
- Smooth surfaces (reduce FBGC)
- Porous structures (permit tissue integration)
- Gradual stiffness transition

Biological Approaches:
- Mesenchymal stem cells (MSC) co-delivery
- Regulatory T-cell recruitment
- M2 macrophage polarization signals
- Anti-inflammatory cytokines (IL-10, IL-4)
```

### 7.4 Biocompatibility Assessment

```
ISO 10993 Testing:
- Cytotoxicity (Part 5): In vitro cell viability
- Sensitization (Part 10): Delayed hypersensitivity
- Irritation (Part 10): Local tissue response
- Systemic toxicity (Part 11): Acute/subchronic
- Genotoxicity (Part 3): DNA damage
- Implantation (Part 6): Tissue response in vivo

Acceptance Criteria:
- Cell viability: >70% (cytotoxicity)
- No sensitization response
- Irritation score: <2 (ISO scale)
- No systemic toxicity signs
- No genotoxic effects
- Tissue response: Minimal (score <9)
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
