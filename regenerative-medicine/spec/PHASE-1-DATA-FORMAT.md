# WIA-BIO-020 — Phase 1: Data Format

> Regenerative-medicine canonical Phase 1: stem-cell + growth-factor + scaffold + clinical-application + QC envelopes.

# WIA-BIO-020: Regenerative Medicine Specification v1.0

> **Standard ID:** WIA-BIO-020
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Regenerative Medicine Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Stem Cell Types and Classification](#2-stem-cell-types-and-classification)
3. [Regeneration Mechanisms](#3-regeneration-mechanisms)
4. [Tissue Engineering and Scaffolds](#4-tissue-engineering-and-scaffolds)
5. [Growth Factor Delivery Systems](#5-growth-factor-delivery-systems)
6. [Clinical Applications](#6-clinical-applications)
7. [Regulatory Requirements](#7-regulatory-requirements)
8. [Implementation Guidelines](#8-implementation-guidelines)
9. [Safety Protocols](#9-safety-protocols)
10. [References](#10-references)

---


## 2. Stem Cell Types and Classification

### 2.1 Embryonic Stem Cells (ESCs)

**Source**: Inner cell mass of blastocyst (day 5-6 embryo)

**Characteristics**:
- Pluripotent: Can differentiate into all cell types
- High proliferation rate
- Express OCT4, SOX2, NANOG markers
- Telomerase positive (unlimited division)

**Applications**:
- Disease modeling
- Drug screening
- Cellular therapy (pending ethical approval)

**Culture Conditions**:
```
Medium: mTeSR1 or E8
Substrate: Matrigel or vitronectin
Temperature: 37°C, 5% CO₂
Passage: Every 3-5 days (70-80% confluence)
Pluripotency markers: OCT4⁺, NANOG⁺, SSEA-4⁺
```

### 2.2 Induced Pluripotent Stem Cells (iPSCs)

**Source**: Reprogrammed somatic cells (fibroblasts, blood cells)

**Reprogramming Factors** (Yamanaka Factors):
- OCT4 (Octamer-binding transcription factor 4)
- SOX2 (Sex determining region Y-box 2)
- KLF4 (Kruppel-like factor 4)
- c-MYC (Cellular myelocytomatosis oncogene)

**Reprogramming Efficiency**:
```
η = (N_iPSC / N_initial) × 100%
```

Typical efficiency: 0.01% - 3% depending on method

**Advantages**:
- Patient-specific (autologous)
- No ethical concerns
- Unlimited cell source
- Disease modeling in patient cells

**Quality Control**:
- Karyotype analysis (normal 46 chromosomes)
- Pluripotency marker expression (>90%)
- Teratoma formation assay
- Differentiation potential to 3 germ layers
- Epigenetic reprogramming verification

### 2.3 Mesenchymal Stem Cells (MSCs)

**Sources**:
- Bone marrow (BM-MSCs)
- Adipose tissue (AD-MSCs)
- Umbilical cord (UC-MSCs)
- Dental pulp (DP-MSCs)

**Defining Criteria** (ISCT 2006):
1. Plastic adherence
2. Positive markers: CD73⁺, CD90⁺, CD105⁺ (>95%)
3. Negative markers: CD45⁻, CD34⁻, CD14⁻, CD19⁻, HLA-DR⁻ (<2%)
4. Tri-lineage differentiation: osteogenic, adipogenic, chondrogenic

**Differentiation Potential**:
```
Osteogenic: Bone (osteoblasts, osteocytes)
Adipogenic: Fat (adipocytes)
Chondrogenic: Cartilage (chondrocytes)
Myogenic: Muscle (myocytes)
Neurogenic: Neural cells (limited)
```

**Immunomodulatory Properties**:
- Secrete IL-10, TGF-β, PGE2
- Suppress T-cell proliferation
- Modulate macrophage polarization (M1→M2)
- Low immunogenicity (low HLA class I, no HLA class II)

### 2.4 Hematopoietic Stem Cells (HSCs)

**Source**: Bone marrow, peripheral blood, umbilical cord blood

**Markers**:
- Positive: CD34⁺, CD133⁺, CD90⁺
- Negative: Lineage markers (Lin⁻)

**Differentiation**:
```
HSCs → Myeloid lineage → RBCs, platelets, granulocytes, monocytes
HSCs → Lymphoid lineage → T cells, B cells, NK cells
```

**Clinical Use**:
- Bone marrow transplantation
- Treatment of leukemia, lymphoma
- Sickle cell disease, thalassemia
- Immune deficiencies

---



## 5. Growth Factor Delivery Systems

### 5.1 Key Growth Factors

| Growth Factor | Function | Target Cells | Optimal Dose |
|--------------|----------|--------------|--------------|
| VEGF | Angiogenesis | Endothelial cells | 10-50 ng/ml |
| FGF-2 | Cell proliferation | Multiple | 5-20 ng/ml |
| TGF-β | ECM production | Fibroblasts | 1-10 ng/ml |
| BMP-2 | Bone formation | Osteoblasts | 50-200 ng/ml |
| PDGF | Wound healing | Fibroblasts | 10-100 ng/ml |
| IGF-1 | Cell growth | Multiple | 50-200 ng/ml |
| EGF | Epithelial growth | Keratinocytes | 5-50 ng/ml |
| BDNF | Neurogenesis | Neurons | 10-100 ng/ml |
| NGF | Nerve growth | Neurons | 50-200 ng/ml |

### 5.2 Delivery Methods

#### 5.2.1 Bolus Injection

**Advantages**: Simple, immediate effect
**Disadvantages**: Short half-life, burst release

**Pharmacokinetics**:
```
C(t) = C₀ × e^(-kt)
```

Half-life: 2-24 hours for most growth factors

#### 5.2.2 Scaffold-Based Delivery

**Release Kinetics**:
```
M_released(t) = M_total × (1 - e^(-kt))
```

**Release Patterns**:
- Immediate release: 50-80% in 24h
- Sustained release: 10-20% per week
- Controlled release: Programmed pattern

#### 5.2.3 Encapsulation (Microspheres)

**Size Range**: 1-200 μm

**Release Mechanisms**:
1. Diffusion through polymer matrix
2. Polymer degradation
3. Osmotic pressure

**Encapsulation Efficiency**:
```
E_eff = (M_encapsulated / M_initial) × 100%
```

Target: >70%

#### 5.2.4 Gene Delivery (Plasmid/Viral)

**Viral Vectors**:
- Adenovirus: High efficiency, transient
- Lentivirus: Stable integration
- AAV: Safe, long-term expression

**Non-Viral**:
- Plasmid DNA
- mRNA
- Lipid nanoparticles

### 5.3 Combination Therapy

**Synergistic Effects**:
```
Effect_combined > Effect_A + Effect_B
```

**Examples**:
- VEGF + FGF-2: Enhanced angiogenesis (2-3× increase)
- BMP-2 + TGF-β3: Improved bone formation
- NGF + BDNF: Superior neural regeneration

---




---

## A.1 Stem-cell-record envelope

The Phase 1 envelope groups stem-cell records by source class (embryonic stem cell — ESC; mesenchymal stem cell — MSC from bone-marrow / adipose / umbilical-cord / dental pulp; hematopoietic stem cell — HSC; neural stem cell — NSC; induced pluripotent stem cell — iPSC reprogrammed from somatic donor cells) with the canonical fields: cell-record identifier (pseudonymous; consent-bound to the donor's WIA-A11Y attestation chain), donor metadata (ID, age, sex, ethnicity where consented, lot number), passage number, viability at thaw in %, sterility status (mycoplasma-free per USP <63>; endotoxin per USP <85> at less than 0.5 EU/mL), karyotype (for stem cells; G-band karyotyping is required for clinical-grade preparations), differentiation-marker panel (CD markers for MSC: CD73+ CD90+ CD105+ CD34- CD45- CD11b- CD19- CD79a- HLA-DR-), and the chain-of-custody hash that ties the cells to the contributing institution's signing key.

## A.2 Growth-factor descriptor

Growth-factor descriptors carry: factor identifier (BMP-2 / BMP-7 / TGF-beta / VEGF / PDGF / FGF-2 / IGF-1 / NGF / GDNF / EGF / HGF / SDF-1), source (recombinant in E. coli / yeast / mammalian; native isolated from platelet-rich-plasma; cell-free conditioned medium), purity per HPLC, biological-activity per the standard activity-assay envelope, the formulation envelope (lyophilised, liquid carrier with the buffer composition, with-or-without carrier protein), and the storage envelope (typical -20 C to -80 C with the documented stability profile). Clinical-grade descriptors cross-reference the Phase 4 §A.3 regulatory envelope.

## A.3 Scaffold-and-delivery descriptor

Scaffold-and-delivery descriptors carry the scaffold-of-record per the canonical material classification (cross-reference WIA-BIO-006 tissue-engineering Phase 1 §A.1 for the comprehensive material catalogue), the delivery-vehicle envelope (injectable hydrogel; in-situ-forming gel; pre-formed scaffold; microsphere-encapsulated; cell-laden bioink), the cell-density envelope (1e6-1e8 cells/mL typical for therapeutic injection; tissue-specific densities per Phase 1 §A.4), and the release-profile envelope (sustained-release of growth factors per the operator's pharmacokinetic study). The descriptor cross-references the IPSC reprogramming-vector envelope where iPSCs are derived in-house.

## A.4 Clinical-application descriptor

Clinical-application descriptors carry: indication identifier per WHO ICD-11 + SNOMED-CT, target tissue, intended-clinical-effect envelope (replace lost tissue / promote endogenous regeneration / immunomodulate), trial-stage envelope (preclinical / phase 1 safety / phase 2 dose-finding-and-efficacy / phase 3 confirmatory / post-market surveillance), comparator-arm envelope (placebo / standard-of-care / no-intervention with the matching ethics-board approval), per-trial endpoint envelope (primary safety, primary efficacy, secondary endpoints, exploratory biomarkers), and the registration envelope (ClinicalTrials.gov / EU CTIS / KR CRIS / WHO ICTRP identifiers).

## A.5 Quality-test envelope

Quality-test envelopes for cell-therapy products follow the cross-walk in Phase 4 §A.1 plus the cell-therapy-specific tests: identity (STR profiling for human cells; immunophenotyping per the marker-panel envelope), purity (residual-host-protein / DNA / serum / antibiotics), potency (per indication-specific functional assay — bone-formation per ALP+OCN secretion; cartilage per GAG production; cardiac per beating-rate-and-electrophysiology; neurological per dopaminergic-marker expression for Parkinson's-disease iPSC-derived neurons), tumourigenicity (soft-agar growth + immunodeficient-mouse xenograft), genetic stability (whole-genome-sequencing or array-CGH per the operator's release-test cadence), and microbiological safety (sterility per USP <71>; mycoplasma per USP <63>; endotoxin per USP <85>).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/regenerative-medicine/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-regenerative-medicine-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/regenerative-medicine-host:1.0.0` ships every regenerative-medicine envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/regenerative-medicine.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Regenerative-medicine deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
