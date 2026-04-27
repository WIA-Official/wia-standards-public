# WIA-longevity-gene PHASE 1 — Data Format Specification

**Standard:** WIA-longevity-gene
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for longevity-
gene research and clinical genomics: variant records linked to
longevity-relevant phenotypes, polygenic-risk scores for healthy
ageing, biological-age estimators (DNA methylation clocks,
telomere length, gene-expression panels), and the cross-references
that bind these genomic artefacts to clinical and lifestyle
records. The shape interoperates with HL7 FHIR R5 Genomics
Reporting and with GA4GH Variation Representation Specification
(VRS) so existing clinical-genomics deployments adopt this PHASE
without parallel data models.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Genomics Reporting Implementation Guide,
  Observation, MolecularSequence, DiagnosticReport
- GA4GH Variation Representation Specification (VRS) 1.3
- GA4GH Phenopacket Schema 2.0
- HGNC (HUGO Gene Nomenclature Committee) gene symbols and
  identifiers
- Ensembl / RefSeq / LRG transcript and reference-sequence systems
- HGVS sequence-variant nomenclature 21.0.5 (https://hgvs-nomenclature.org)
- ACMG/AMP 2015 + 2017 amendments — sequence-variant
  interpretation
- ClinVar / dbSNP — public variant references
- ISO 20691:2022 — Bioinformatics standard data formats for
  next-generation sequencing
- ISO/TS 22692:2020 — Quality control for NGS
- ISO 23950 / Z39.50 — only as legacy (not preferred)
- CDISC SDTM, CDASH — clinical research data standards
- WHO ICD-11
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 3339

---

## §1 Scope

This PHASE applies to systems that:

- Generate or interpret germline variants relevant to healthy
  ageing or longevity (APOE, FOXO3, TP53, SIRT genes, telomere-
  pathway genes, etc.)
- Compute polygenic-risk scores (PRS) for ageing-related
  phenotypes (cardiovascular, cognitive, metabolic, frailty)
- Quantify biological age (epigenetic clocks: Horvath, Hannum,
  PhenoAge, GrimAge, DunedinPACE; telomere length; transcriptomic
  age)
- Combine genomic and phenotypic signals into longevity reports
  for individual or population studies

Out of scope: somatic-cancer variant calling (covered by sibling
oncology-genomics standard) and pharmacogenomic dosing (covered
by sibling pharmacogenomics standard).

The standard is jurisdiction-aware: deployments declare the
applicable genetic-information regulation (US GINA, EU GDPR
Article 9 + national genetics laws, K-PIPA 유전정보 특별보호,
JP Act on the Protection of Personal Information, BR LGPD).

## §2 Sequence-variant identity

Variants are identified following GA4GH VRS:

- `variantRef` — URN of form `urn:wia:long:variant:<vrs-id>`
  where `vrs-id` is the deterministic VRS identifier
- `genomeAssembly` — URN of the reference assembly
  (`urn:wia:long:assembly:GRCh38.p14`, `GRCh37.p13`, `T2T-CHM13v2.0`)
- `transcriptRef` — Ensembl/RefSeq/LRG transcript URN where the
  variant is annotated against a transcript
- `geneSymbol` — HGNC-approved gene symbol (e.g., `APOE`)
- `hgvsExpressions[]` — HGVS string forms across genomic, coding
  DNA, and protein where applicable
- `clinvarId` / `dbsnpId` — public registry cross-references

The boundary verifies VRS identifiers via canonicalisation;
non-canonical forms are normalised before storage so duplicate
representations of the same variant collapse to one record.

## §3 Variant interpretation record

Each variant interpretation is a FHIR Observation profiled to
the Genomics Reporting IG:

| FHIR field            | Binding                                                          |
|-----------------------|------------------------------------------------------------------|
| `status`              | preliminary / final / amended / corrected                        |
| `category`            | `genomics`                                                       |
| `code`                | LOINC variant-interpretation code (53037-8 for sequence variant) |
| `subject`             | pseudonymous individual                                          |
| `effectiveDateTime`   | RFC 3339 with offset                                            |
| `valueCodeableConcept`| ACMG/AMP classification (Pathogenic / Likely Pathogenic / VUS / Likely Benign / Benign) |
| `interpretation`      | longevity-relevant interpretation (e.g., `risk-allele`, `protective-allele`) |
| `evidence[]`          | references to ClinVar, literature DOIs, in-silico predictors,
                          functional studies                                              |
| `note[]`              | clinician/curator commentary                                      |

Interpretation criteria use the ACMG/AMP framework; the boundary
records the specific criteria invoked (PVS1, PS1, PM2, etc.) so
re-interpretation in light of new evidence is traceable.

## §4 Polygenic-risk score record

PRS records are FHIR Observations with structured PRS-specific
extensions:

- `prsId` — URN
- `prsModelRef` — URN of the PRS model with version
  (e.g., `urn:wia:long:prs-model:PGS001234:v1.0`)
- `subject` — pseudonymous individual
- `effectiveDateTime` — RFC 3339
- `rawScore` — model output (typically a continuous value)
- `percentile` — within-population percentile with declared
  population reference
- `populationReferenceRef` — URN of the reference population
  (1000 Genomes Project, gnomAD, UK Biobank, KOREA-1K, J-MICC)
- `effectMeasure` — declared effect measure (odds ratio,
  hazard ratio, age-adjusted risk)
- `confidenceInterval` — declared CI (95% conventional)
- `model.qcStatus` — `passed` / `failed` per the model's QC
  cadence

PRS scores are not portable across populations without
re-calibration; the boundary refuses to display a score
computed against a population reference that does not match
the subject's declared ancestry without an explicit
disclaimer record.

## §5 Biological-age record

Biological-age estimates are FHIR Observations with epigenetic-
or transcriptomic-clock metadata:

- `bioAgeId` — URN
- `clockModelRef` — URN of the clock model with version
  (e.g., `urn:wia:long:clock-model:Horvath-2013:v1.0`)
- `subject` — pseudonymous individual
- `effectiveDateTime` — RFC 3339 of sample acquisition
- `chronologicalAgeYears` — declared at sample time
- `biologicalAgeYears` — model output
- `accelerationYears` — bio-age − chronological-age
- `tissueRef` — sample tissue (whole blood, saliva, buccal,
  PBMC, dermal fibroblast, etc.)
- `assayPlatform` — methylation array (e.g., Illumina
  EPICv1.0/v2.0, HumanMethylation450), sequencing-based
  methylation, transcriptomic platform
- `qcMetrics` — bisulfite-conversion efficiency, missing-CpG
  rate, batch effect indicator

Biological-age measurements are tissue- and platform-specific;
the boundary records both metadata fields on every record so
cross-study aggregation honours methodological boundaries.

## §6 Telomere-length record

Telomere length is captured per cell type or whole sample:

- `telomereId` — URN
- `subject` — pseudonymous individual
- `effectiveDateTime` — RFC 3339
- `tissueRef` — sample tissue
- `cellType` — for sorted-cell measurements (e.g., naive CD4+
  T-cells); null for whole-blood
- `methodRef` — URN denoting method (qPCR T/S ratio, TRF
  Southern blot, STELA, qFISH, computational from WGS)
- `mean` — value with unit (`base-pair`, `T/S`)
- `confidenceInterval` — declared CI
- `qcStatus` — `passed` / `failed`

## §7 Phenotype linkage

Variant interpretations and biological-age records reference
a Phenopacket-aligned phenotype record:

- `phenotypeRef` — URN of the linked Phenopacket
- `phenotypeOnsetAge` — declared onset (HPO age group or
  ISO 8601 duration)
- `phenotypeSeverity` — HPO severity term where applicable

Phenopackets carry the longitudinal phenotype context
(disease history, family history, environmental exposures)
that contextualises the genomic finding.

## §8 Cross-domain references

| Reference                  | Use site                                                  |
|----------------------------|-----------------------------------------------------------|
| WIA-medical-data-privacy   | every record references the consent record (genetics-specific consent class) |
| WIA-medical-iot            | wearables that contribute to bio-age inputs (sleep, HRV)  |
| WIA-medication-adherence   | longitudinal pharmacological exposures                    |
| WIA-network-security       | TLS cipher-suite floor for clinical-lab data flows        |
| WIA-pq-crypto              | post-quantum migration phase                              |

## §9 Subject identifier scope

Subject identifiers are pseudonymous per WIA-medical-data-privacy
`subjectRef` shape. Genetic data is exceptionally high-sensitivity
PHI and inherits the highest-class consent and disclosure
discipline.

## §10 Conformance levels

| Level     | Scope                                                              |
|-----------|--------------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                               |
| Verified  | annual third-party audit including GA4GH VRS conformance review    |
| Anchored  | continuous evidence package + ISO 20691 NGS quality-control attestation |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked variant interpretation (informative)

```json
{
  "resourceType": "Observation",
  "status": "final",
  "category": [{"coding": [{"system": "http://terminology.hl7.org/CodeSystem/observation-category", "code": "genomics"}]}],
  "code": {"coding": [{"system": "http://loinc.org", "code": "53037-8", "display": "Genetic variation clinical significance"}]},
  "subject": {"reference": "urn:wia:mdp:subject:f4c2-9bd1-7a05-3e8e"},
  "effectiveDateTime": "2026-04-28T12:00:00+09:00",
  "valueCodeableConcept": {"coding": [{"system": "http://terminology.hl7.org/CodeSystem/v3-ObservationInterpretation", "code": "B", "display": "Benign"}]},
  "extension": [{
    "url": "https://wia.example/fhir/StructureDefinition/wia-longevity-variant",
    "extension": [
      {"url": "variantRef", "valueUri": "urn:wia:long:variant:ga4gh:VA.E0o4HzNvjnqQyDg-Sp-ddlyZpv9R-7TI"},
      {"url": "geneSymbol", "valueString": "APOE"},
      {"url": "hgvsCdna", "valueString": "NM_000041.4:c.388T>C"},
      {"url": "hgvsProtein", "valueString": "NP_000032.1:p.Cys130Arg"},
      {"url": "longevityInterpretation", "valueCode": "risk-allele"}
    ]
  }]
}
```

## Annex B — Negative test vectors (informative)

| Stimulus                                                | Expected outcome                                  |
|---------------------------------------------------------|---------------------------------------------------|
| Variant interpretation without ACMG/AMP criteria         | 422 + `acmg-criteria-required`                    |
| PRS computed against mismatched population reference     | accepted with disclaimer; flagged in summary      |
| Bio-age record without QC metrics                        | 422 + `bio-age-qc-required`                       |
| Telomere-length record without method reference          | 422 + `telomere-method-required`                  |
| Variant interpretation referencing retracted publication | accepted, flagged for re-interpretation review    |

## Annex C — Longevity-gene panel reference (informative)

The deployment publishes its longevity-gene panel as a versioned
catalogue; each entry includes the gene symbol, longevity
relevance category, and supporting literature reference:

- **APOE** — Alzheimer's risk and lipid metabolism;
  ε2/ε3/ε4 alleles influence late-onset Alzheimer's risk
- **FOXO3** — extreme-longevity association across multiple
  populations (Okinawan, Ashkenazi, Italian centenarian cohorts)
- **TP53** — somatic-stability and ageing; rare germline
  variants influence longevity through senescence
- **KLOTHO** — KL gene variants influencing klotho protein
  levels; longevity association in some populations
- **SIRT1, SIRT3, SIRT6** — sirtuin family
- **TERT, TERC, DKC1** — telomere-pathway core genes
- **WRN, BLM, RECQL4** — DNA repair (RecQ helicases)
- **APOC3** — protective rare variants

The catalogue is signed; updates are version-bumped and
notified to consumers.

## Annex D — Versioning and deprecation

Versioning follows Semantic Versioning 2.0.0. Major bumps
require ≥ 90 days overlap on all fielded reference
implementations. Deprecated VRS-version representations
remain valid for verification of historical observations.
