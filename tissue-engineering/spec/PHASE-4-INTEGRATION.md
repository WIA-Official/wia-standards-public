# WIA-BIO-006 — Phase 4: Integration

> Tissue-engineering canonical Phase 4: ecosystem integration (ISO 10993 + GMP + FDA HCT/P + ATMP + future).

# WIA-BIO-006: Tissue Engineering Specification v1.0

> **Standard ID:** WIA-BIO-006
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biomedical Engineering Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scaffold Materials and Design](#2-scaffold-materials-and-design)
3. [3D Bioprinting Protocols](#3-3d-bioprinting-protocols)
4. [Bioreactor Systems](#4-bioreactor-systems)
5. [Vascularization Strategies](#5-vascularization-strategies)
6. [Cell Culture and Seeding](#6-cell-culture-and-seeding)
7. [Quality Testing Standards](#7-quality-testing-standards)
8. [Regulatory Requirements](#8-regulatory-requirements)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---


## 7. Quality Testing Standards

### 7.1 Sterility Testing

**Methods**:
- **USP <71>**: 14-day incubation in media
- **Endotoxin**: LAL assay (<0.5 EU/mL)
- **Mycoplasma**: PCR or culture

**Frequency**: Every batch

### 7.2 Cell Viability Assays

#### 7.2.1 Live/Dead Staining

**Reagents**:
- Calcein AM (live cells, green)
- Ethidium homodimer-1 (dead cells, red)

**Protocol**:
1. Incubate 15-30 min
2. Image with fluorescence microscopy
3. Quantify with ImageJ

**Acceptance**: >80% viability

#### 7.2.2 MTT/MTS Assay

**Principle**: Metabolic activity

```
Viability (%) = (A_sample / A_control) × 100%
```

**Protocol**:
1. Add MTT/MTS reagent
2. Incubate 1-4 hours
3. Measure absorbance (490-570 nm)

### 7.3 Mechanical Testing

#### 7.3.1 Compression Testing

**Standard**: ASTM D1621

**Parameters**:
- Strain rate: 0.5-1 mm/min
- Preload: 0.01-0.1 N
- Deformation: 10-50%

**Outputs**:
- Young's modulus (E)
- Ultimate strength (σ_max)
- Strain at failure (ε_f)

#### 7.3.2 Tensile Testing

**Standard**: ASTM D638

**Parameters**:
- Gauge length: 10-50 mm
- Strain rate: 1-10 mm/min
- Sample shape: Dumbbell

### 7.4 Biocompatibility Testing

#### 7.4.1 Cytotoxicity (ISO 10993-5)

**Extract test**:
1. Incubate material in medium (24h, 37°C)
2. Expose cells to extract
3. Measure viability (MTT, LDH)

**Acceptance**: >70% viability relative to control

#### 7.4.2 Immunogenicity

**Assays**:
- Cytokine release (IL-1β, IL-6, TNF-α)
- T-cell proliferation
- Complement activation (C3a, C5a)

**Target**: <2-fold increase vs. control

#### 7.4.3 In Vivo Biocompatibility

**Subcutaneous implantation**:
- Duration: 1-12 weeks
- Histology: H&E, Masson's trichrome
- Grading: ISO 10993-6

**Scores**:
- 0: No reaction
- 1: Minimal reaction
- 2-4: Increasing severity

**Acceptance**: Score ≤1

### 7.5 Functional Assays

#### 7.5.1 Tissue-Specific Markers

**Bone**:
- Alkaline phosphatase (ALP)
- Osteocalcin (OCN)
- Calcium deposition (Alizarin Red)

**Cartilage**:
- Glycosaminoglycans (GAG, Alcian Blue)
- Collagen type II (immunostaining)
- Aggrecan (Western blot)

**Liver**:
- Albumin secretion (ELISA)
- Urea synthesis (colorimetric assay)
- CYP450 activity (luminescent assay)

#### 7.5.2 Genetic Expression

**qRT-PCR**:
- Extract RNA (TRIzol)
- Reverse transcribe (cDNA)
- Quantify genes (SYBR Green)
- Normalize to housekeeping genes (GAPDH, β-actin)

**Fold change**:
```
FC = 2^(-ΔΔCt)
```

---



## 8. Regulatory Requirements

### 8.1 FDA Classification

**Class II**: Most tissue-engineered products
- 510(k) premarket notification
- Good Manufacturing Practice (GMP)
- Quality System Regulation (QSR)

**Class III**: High-risk products
- Premarket Approval (PMA)
- Clinical trials (Phase I-III)

### 8.2 Good Manufacturing Practice (GMP)

**Requirements**:
- Validated processes
- Controlled environment (ISO 5-7 cleanroom)
- Documented procedures (SOPs)
- Traceability (batch records)
- Quality control (in-process testing)

### 8.3 Preclinical Testing

**Sequence**:
1. In vitro biocompatibility (ISO 10993)
2. Small animal models (rat, rabbit)
3. Large animal models (pig, sheep)
4. Dose-response studies
5. Long-term safety (6-12 months)

### 8.4 Clinical Trials

**Phase I**: Safety (10-30 patients)
**Phase II**: Efficacy (30-100 patients)
**Phase III**: Confirmatory (100-1000 patients)

**Endpoints**:
- Primary: Safety, efficacy
- Secondary: Quality of life, cost-effectiveness

---



## 10. References

### 10.1 Scientific Literature

1. Langer, R., Vacanti, J.P. (1993). "Tissue Engineering." Science
2. Murphy, S.V., Atala, A. (2014). "3D Bioprinting of Tissues and Organs." Nature Biotechnology
3. Rouwkema, J., Khademhosseini, A. (2016). "Vascularization in Tissue Engineering." Trends in Biotechnology
4. Hollister, S.J. (2005). "Porous Scaffold Design for Tissue Engineering." Nature Materials
5. Griffith, L.G., Naughton, G. (2002). "Tissue Engineering--Current Challenges." Science

### 10.2 Standards and Regulations

| Standard | Title |
|----------|-------|
| ISO 10993 | Biological evaluation of medical devices |
| ASTM F2150 | Guide for characterization and testing of biomaterial scaffolds |
| ASTM F2603 | Standard guide for interpretation of TEM images |
| ISO 13485 | Quality management systems for medical devices |
| 21 CFR 1271 | FDA regulation of human cells and tissues |

### 10.3 Biomaterial Properties

| Material | Density (g/cm³) | Degradation | Biocompatibility |
|----------|----------------|-------------|------------------|
| Collagen I | 1.35 | 2-8 weeks | Excellent |
| Gelatin | 1.27 | 1-4 weeks | Excellent |
| Chitosan | 1.43 | 4-12 weeks | Good |
| Alginate | 1.60 | 2-8 weeks | Good |
| PCL | 1.15 | 6-24 months | Good |
| PLA | 1.24 | 6-12 months | Good |
| PLGA | 1.34 | 1-6 months | Good |

### 10.4 WIA Standards

- WIA-INTENT: Intent-based tissue engineering requests
- WIA-OMNI-API: Universal biomedical API gateway
- WIA-SOCIAL: Collaborative research protocols
- WIA-BIO-001 to BIO-005: Related biomedical standards

---

## Appendix A: Example Calculations

### A.1 Scaffold Porosity

```
Given:
- Material density: 1.15 g/cm³ (PCL)
- Apparent density: 0.29 g/cm³

Calculation:
P = (1 - 0.29/1.15) × 100%
P = (1 - 0.252) × 100%
P = 74.8%

Result: Porosity is 75%, suitable for cartilage tissue
```

### A.2 Cell Seeding Density

```
Given:
- Scaffold volume: 1.0 cm³
- Target density: 5 × 10⁷ cells/cm³

Calculation:
N_cells = D × V
N_cells = 5 × 10⁷ × 1.0
N_cells = 5 × 10⁷ cells

Suspension preparation:
- Cell concentration: 5 × 10⁶ cells/mL
- Volume needed: 10 mL
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-006 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*



---

## A.1 Regulatory cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| Biocompatibility — cytotoxicity | ISO 10993-5                              |
| Biocompatibility — implantation | ISO 10993-6                              |
| Biocompatibility — sensitization| ISO 10993-10                             |
| Sterility — finished product    | USP <71>                                 |
| Endotoxin testing               | USP <85> / LAL assay                     |
| Mycoplasma testing              | USP <63> / European Pharmacopoeia 2.6.7  |
| Mechanical — compression        | ASTM F2150 / ISO 13314                   |
| Mechanical — tensile            | ASTM D638                                |
| Quality management — devices    | ISO 13485:2016                           |
| FDA — HCT/P regulation          | 21 CFR 1271                              |
| EU — ATMP regulation            | EC No 1394/2007                          |
| KR — biological products        | KFDA Notification 2021-92                |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 Manufacturing and GMP integration

Manufacturing integration covers cleanroom qualification (ISO 14644 — Class A laminar-flow within Class B background; environmental monitoring per EU GMP Annex 1), validated processes (process performance qualification per ICH Q7), batch-record generation, traceability from donor through finished construct, in-process quality control sampling plan, and the certificate-of-conformance for each shipment. Manufacturers map their internal MES/PLM identifiers onto the WIA `scaffoldId`, `bioinkId`, and `constructId` so downstream regulators inherit the provenance graph.

## A.3 FDA / global-regulator integration

Tissue-engineering products are classified by the FDA along the HCT/P (Section 361 versus Section 351) decision tree per 21 CFR 1271. Section 351 products require BLA filing with phase I/II/III clinical data; Section 361 products are minimally manipulated and used homologously, with reduced regulatory burden. The standard's regulatory envelope captures the classification rationale, the consultation history (pre-IND, Type B/C/D), and the post-market surveillance commitments. Equivalent envelopes track EMA ATMP classification (gene therapy, somatic-cell therapy, tissue-engineered product, combined ATMP) and KFDA / PMDA / NMPA submissions.

## A.4 Safety and biocompatibility integration

ISO 10993-1:2018 risk-based selection drives the biocompatibility test matrix; the test plan envelope at Phase 1 §A.5 captures the rationale per device contact category (surface-contacting versus implanted; contact duration limited, prolonged, or permanent). Sterility validation follows ISO 11737 (microbiological methods) and ISO 11135 (ethylene-oxide), or aseptic-processing validation per FDA 2004 guidance for sterile drug products. Cell-product safety includes mycoplasma, endotoxin, sterility, identity (STR profiling for human cells), purity, and potency assays — each of which carries a documented acceptance criterion and a release-test record.

## A.5 Future directions

Active research tracks: organ-scale bioprinting with sacrificial vascular templates, in-situ bioprinting on the surgical field, decellularized whole-organ scaffolds re-cellularised with patient-derived iPSCs, organoid-based personalized tissue models for drug screening, smart bioreactors with real-time imaging and AI-driven feedback control, and the use of CRISPR-edited cells for autologous immune-evasive constructs. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- ISO 10993 series — biological evaluation of medical devices
- ASTM F2150 — characterization and testing of biomaterial scaffolds
- ASTM F2603 — interpretation of TEM images for tissue-engineered medical products
- USP <71>, <85>, <63> — sterility, endotoxin, mycoplasma testing
- ISO 13485:2016 — quality management systems for medical devices
- 21 CFR 1271 — FDA regulation of human cells, tissues, and cellular and tissue-based products
- EC No 1394/2007 — EU advanced-therapy medicinal products regulation
- ICH Q7 / Q8 / Q9 / Q10 — GMP for active pharmaceutical ingredients, pharmaceutical development, quality risk management, pharmaceutical quality system
- Crapo, T.H. Gilbert, S.F. Badylak (2011, Biomaterials) — overview of decellularization processes
- S.V. Murphy, A. Atala (2014, Nature Biotechnology) — 3D bioprinting of tissues and organs


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/tissue-engineering/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-tissue-engineering-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/tissue-engineering-host:1.0.0` ships every tissue-engineering envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/tissue-engineering.sh` ships sample envelope generators with no
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
ecosystem. Tissue-engineering deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
