# WIA-BIO-020 — Phase 4: Integration

> Regenerative-medicine canonical Phase 4: ecosystem integration (USP + ISO 14644 + 21 CFR 1271 + EU ATMP + ICH + GMP Annex 1).

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


## 7. Regulatory Requirements

### 7.1 FDA RMAT (Regenerative Medicine Advanced Therapy)

**Criteria for RMAT Designation**:
1. Regenerative medicine therapy (cell/gene therapy)
2. Intended to treat/modify/cure serious condition
3. Preliminary clinical evidence of potential benefit

**Expedited Programs**:
- Fast Track
- Breakthrough Therapy
- Priority Review
- Accelerated Approval

**Submission Requirements**:
- Pre-IND meeting documentation
- CMC (Chemistry, Manufacturing, Controls) section
- Nonclinical studies (toxicology, biodistribution)
- Clinical trial protocols (Phase I/II/III)

### 7.2 EMA ATMP (Advanced Therapy Medicinal Products)

**Categories**:
1. Gene therapy medicinal products
2. Somatic cell therapy medicinal products
3. Tissue engineered products
4. Combined ATMPs

**Requirements**:
- Quality (GMP manufacturing)
- Safety (preclinical studies)
- Efficacy (clinical trials)
- Risk management plan
- Pharmacovigilance system

### 7.3 GMP (Good Manufacturing Practice)

**Critical Parameters**:
- Clean room: ISO Class 5 (Class 100)
- Air quality: <3,520 particles ≥0.5 μm/m³
- Temperature: 20-24°C
- Humidity: 30-60% RH
- Sterility testing: <1 CFU/sample

**Documentation**:
- Master Cell Bank (MCB) characterization
- Batch production records
- Quality control testing results
- Chain of custody documentation

### 7.4 Quality Control Testing

**Release Criteria**:

| Test | Acceptance Criteria |
|------|---------------------|
| Sterility | No growth (14 days) |
| Endotoxin | <5 EU/kg body weight |
| Mycoplasma | Negative |
| Viability | >70% |
| Identity | Marker expression >90% |
| Purity | Contaminating cells <5% |
| Potency | Functional assay specific |
| Karyotype | Normal (if applicable) |

---



## 10. References

### 10.1 Scientific Literature

1. Takahashi, K., Yamanaka, S. (2006). "Induction of pluripotent stem cells from mouse embryonic and adult fibroblast cultures by defined factors." Cell 126(4): 663-676.
4. Langer, R., Vacanti, J.P. (1993). "Tissue engineering." Science 260(5110): 920-926.
5. Khademhosseini, A., Langer, R. (2016). "A decade of progress in tissue engineering." Nature Protocols 11: 1775-1781.

### 10.2 Regulatory Guidelines

- FDA Guidance: "Regulatory Considerations for Human Cells, Tissues, and Cellular and Tissue-Based Products: Minimal Manipulation and Homologous Use" (2020)
- EMA Guideline: "Guideline on human cell-based medicinal products" (2008)
- ICH Q5A(R1): "Viral Safety Evaluation of Biotechnology Products Derived from Cell Lines of Human or Animal Origin" (1999)
- ISO 10993: "Biological evaluation of medical devices"

### 10.3 Clinical Trial Registries

- ClinicalTrials.gov (USA)
- EU Clinical Trials Register
- ISRCTN Registry (International)

### 10.4 WIA Standards

- WIA-BIO-002: Cell Culture Technology
- WIA-BIO-005: Tissue Engineering
- WIA-HEALTH: Patient Health Monitoring
- WIA-OMNI-API: Universal Biomedical API

---

## Appendix A: Growth Factor Table (Comprehensive)

| Factor | MW (kDa) | Half-life | Stability | Storage |
|--------|----------|-----------|-----------|---------|
| VEGF-A | 38-45 | 2-4 h | Moderate | -80°C |
| FGF-2 | 18 | 3-7 h | High | -20°C |
| TGF-β1 | 25 | 2-3 min | Low (pH sensitive) | -80°C |
| BMP-2 | 26 | 7 min | High | -20°C |
| PDGF-BB | 24 | 2-4 h | Moderate | -80°C |
| IGF-1 | 7.6 | 12-15 h | High | -20°C |
| EGF | 6 | 8-10 h | High | -20°C |
| BDNF | 27 | 1-10 min | Low | -80°C |
| NGF | 26 | 2-3 h | Moderate | -80°C |
| HGF | 82 | 5-10 min | Moderate | -80°C |

## Appendix B: Scaffold Material Comparison

| Property | Collagen | Chitosan | PLGA | PCL | Alginate |
|----------|----------|----------|------|-----|----------|
| Origin | Animal | Natural | Synthetic | Synthetic | Natural |
| Biocompatibility | Excellent | Good | Good | Good | Excellent |
| Cell adhesion | Excellent | Good | Poor (needs modification) | Poor | Poor |
| Degradation | 2-8 weeks | 4-12 weeks | 1-12 months | 24-36 months | Days-weeks |
| Mechanical strength | Low-Medium | Low | Medium-High | High | Low |
| Cost | High | Medium | Low | Low | Low |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-020 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*



---

## A.1 Quality, safety, and regulatory cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| Sterility — finished product  | USP <71>                                  |
| Endotoxin testing             | USP <85> + EU Pharmacopoeia 2.6.14        |
| Mycoplasma testing            | USP <63> + EU Pharmacopoeia 2.6.7         |
| Cell-therapy potency          | USP <1046>                                |
| Identity by STR profiling     | ANSI/ATCC ASN-0002                        |
| Genetic stability             | ISO 20387 biobanking + ICH Q5D            |
| Cleanroom classification      | ISO 14644-1 / -2 / -7                     |
| Donor eligibility (US)        | 21 CFR 1271 Subpart C                     |
| Donor eligibility (EU)        | EU Directives 2004/23/EC + 2006/17/EC     |
| HCT/P regulation (US)         | 21 CFR 1271                               |
| ATMP regulation (EU)          | EC Regulation 1394/2007                   |
| ATMP scientific guidance      | EMA Guideline EMA/CAT/600280/2010         |
| Clinical-trial regulation (EU)| EU Regulation 536/2014                    |
| GMP for cell therapy          | EU GMP Annex 1 + EU GMP Annex 2           |
| ICH                           | ICH Q5A/B/C/D + Q7 + Q8/9/10              |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 GMP and cleanroom integration

GMP integration captures the cleanroom-classification envelope per ISO 14644-1 (Class A laminar-flow within Class B background per EU GMP Annex 1; ISO 5 within ISO 7 per US FDA Aseptic Processing guidance), the environmental-monitoring envelope (viable-air, surface, personnel monitoring; particle-counter-of-record; pressure-cascade verification), the gowning envelope, the validated-process envelope (PQ — Process Performance Qualification per ICH Q7), the batch-record-of-record envelope, the in-process control envelope, and the certificate-of-analysis envelope at lot release.

## A.3 ATMP / HCT/P / cell-therapy regulatory integration

Cell-therapy regulatory integration covers the FDA HCT/P decision tree (Section 361 minimally manipulated and homologous use — reduced regulatory burden; Section 351 — full BLA), the EMA ATMP classification (gene therapy / somatic-cell therapy / tissue-engineered product / combined ATMP) with the centralized-procedure envelope per EMA, the KFDA cell-therapy regulatory framework, the PMDA Sakigake conditional-approval envelope, and the NMPA cell-therapy classification envelope. The integration envelope captures the per-jurisdiction filing strategy, the consultation history (pre-IND / Type B/C/D / Pre-Submission), and the post-market surveillance commitments.

## A.4 Clinical-trial integration

Clinical-trial integration captures the trial-registration envelope (ClinicalTrials.gov / EudraCT-CTIS / WHO ICTRP / KR CRIS / JP UMIN / CN ChiCTR / Australian-NZ ANZCTR), the IRB / IEC / Ethics-Committee approval envelope, the Data and Safety Monitoring Board (DSMB) envelope, the CONSORT 2010 reporting envelope, the per-jurisdiction adverse-event-reporting cadence (FDA IND Safety per 21 CFR 312.32; EU EudraVigilance per Regulation 536/2014; equivalents), and the trial-results-publication envelope per the WMA Declaration of Helsinki §35-36.

## A.5 Future directions

Active research tracks: organoid-based personalised disease modelling and screening; autologous iPSC-derived cell therapies for Parkinson's, type-1 diabetes, retinal degeneration, sickle-cell disease, severe-combined-immunodeficiency; allogeneic universal-donor iPSC banks with HLA-engineered immune evasion; in-situ programming via CAR-T-engineered cells for non-oncology indications; gene-edited stem-cell therapies (CRISPR-Cas9 + base / prime editing) under the relevant regulatory frameworks for genetically-modified cell products; bioprinted vascularized organ constructs for transplantation. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- USP <71> / <85> / <63> / <1046> — sterility, endotoxin, mycoplasma, cell-therapy potency
- ANSI/ATCC ASN-0002 — STR profiling for human cell line identity
- ISO 20387 — Biotechnology — biobanking general requirements
- ISO 14644 series — cleanrooms and associated controlled environments
- ICH Q5A / Q5B / Q5C / Q5D — quality of biotechnological products
- ICH Q7 / Q8 / Q9 / Q10 / Q11 — GMP, pharmaceutical development, quality risk management
- 21 CFR 1271 — FDA regulation of human cells, tissues, and cellular and tissue-based products
- EU Directives 2004/23/EC + 2006/17/EC — quality and safety standards for tissues and cells
- EC Regulation 1394/2007 — Advanced Therapy Medicinal Products
- EMA Guideline EMA/CAT/600280/2010 — risk-based approach for cell-based ATMPs
- EU Regulation 536/2014 — Clinical Trials Regulation
- EU GMP Annex 1 / Annex 2 — sterile / biological medicinal products
- WMA Declaration of Helsinki 2013 — ethical principles for medical research
- ISO/TR 22916 — biotechnology — cellular product reference


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
