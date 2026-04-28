# WIA-microbiome PHASE 4 — Integration Specification

**Standard:** WIA-microbiome
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how WIA-microbiome integrates
with adjacent regulatory, clinical, archival, and
research-data systems: INSDC public archives, MGnify
and MGnifyDB, the NIH Common Fund Human Microbiome
Project data conventions, EU EOSC, GTDB and SILVA
reference releases, FHIR R5 clinical exchange, food-
safety regulators, and live-biotherapeutic-product
sponsors operating under FDA / EMA / MFDS / PMDA. It
also specifies the operational binding to companion
WIA standards.

References (CITATION-POLICY ALLOW only):
- INSDC — NCBI / ENA / DDBJ submission and exchange
- MGnify — EMBL-EBI microbiome resource and pipeline reference
- GTDB — Genome Taxonomy Database release versioning
- SILVA / Greengenes2 — rRNA reference databases
- KEGG / MetaCyc / UniProt / EC IUBMB — functional reference databases
- HL7 FHIR R5 — Specimen, Observation, MolecularSequence, Bulk Data Access
- HL7 SMART App Launch 2.0
- Codex Alimentarius CAC/RCP 1-1969 — General Principles of Food Hygiene
- Codex Alimentarius MRL — maximum residue limits
- ISO 7218:2024 — Microbiology of food and animal feeding stuffs
- US FDA BAM (Bacteriological Analytical Manual) chapters on metagenomic methods
- EU Regulation 178/2002 (general food law), Regulation 2073/2005 (microbiological criteria)
- US FDA — LBP draft guidance; EMA — Q&A on quality of LBPs
- 21 CFR Part 11 — electronic records / signatures
- ISO/IEC 27001:2022, ISO/IEC 27701:2019 (privacy extension)
- IEEE 11073-10101 — laboratory device nomenclature
- IETF RFC 9110 (HTTP), RFC 7515 (JWS), RFC 8259 (JSON), RFC 8785 (JCS)

---

## §1 INSDC archive integration

Public studies submit to one of the three INSDC nodes:

| Archive  | Operator       | Submission portal           |
|----------|----------------|-----------------------------|
| NCBI SRA / GenBank | NIH NLM | SRA submission portal      |
| ENA      | EMBL-EBI       | Webin REST submission       |
| DDBJ DRA | NIG / ROIS     | DDBJ DRA submission         |

Per-record cross-walk (PHASE 1 §2 → INSDC):

```
WIA Specimen   →  INSDC BioSample (with MIxS package)
WIA Library    →  INSDC Experiment
WIA Run        →  INSDC Run
WIA Read set   →  INSDC SRA / ENA Run object
WIA Study      →  INSDC BioProject
WIA Assembly   →  GenBank / ENA Assembly
```

The `submissionRef` (PHASE 1 §4) records the assigned
accession; release events update the public visibility
on the WIA record and replicate to the public sync.

## §2 MGnify integration

Studies that wish to expose results through MGnify's
analysis catalogue submit per the MGnify pipeline
inputs (raw read accessions on ENA + a metadata sheet
following the MIxS package the study selected). The
MGnify analysis output URI is recorded as a
`derivedAnalysisRef` on the analytical-result record so
downstream users can compare WIA-pipeline results to
MGnify's reference pipeline.

## §3 FHIR R5 clinical exchange

| WIA record           | FHIR R5 resource                                |
|----------------------|-------------------------------------------------|
| Specimen             | Specimen                                        |
| Library / run        | Procedure (for clinical contexts)               |
| Read set             | DocumentReference + MolecularSequence           |
| Taxonomy result      | Observation (LOINC for microbiome panels)        |
| Functional result    | Observation (LOINC functional panel)             |
| Biomarker call       | DiagnosticReport (with reviewer credentials)     |

Clinical microbiome reporting uses the LOINC microbiome
panel codes (e.g. 88072-1 stool microbiome panel, 99005-
0 vaginal flora) when a corresponding LOINC term exists.

## §4 Reference-database lifecycle binding

Reference databases evolve. The analytical-result record
pins the reference release used:

| Database     | Release identifier example       |
|--------------|----------------------------------|
| SILVA        | SILVA 138.2                      |
| Greengenes2  | gg2-2024.09                      |
| GTDB         | GTDB R220 (2026 release)         |
| NCBI RefSeq  | RefSeq release number + date     |
| KEGG         | KEGG version + release date       |
| CARD         | CARD ontology release             |
| VFDB         | VFDB release                     |

Re-running an analysis under a newer reference release
is a fresh analytical-result; the prior result is not
mutated.

## §5 Food-safety integration

For food-process microbiome:

- regulator-mandated tests (Salmonella, Listeria
  monocytogenes, EHEC, Cronobacter) per Codex / Reg
  2073/2005 / FDA BAM are recorded as analytical-result
  records with `metric=regulator-pathogen-screen`
- positive findings transmit through the relevant
  regulator pathway (RASFF for EU, FoodNet / FDA Safety
  Reporting Portal for US, MFDS Food Safety Korea, PMDA
  Japan)
- recalls reference the affected lot through the food-
  traceability standard (WIA-food-traceability)

## §6 Live-biotherapeutic-product (LBP) integration

LBP sponsors carry the strain identity, manufacturing
lot, fermentation parameters, viability metrics, and
clinical investigation cross-references on the
intervention record. WIA-microbiome links the LBP
release-test microbiome characterisation (purity,
strain authenticity, contamination screen) to the lot.
Regulatory pathways:

| Regulator | LBP pathway                                    |
|-----------|------------------------------------------------|
| FDA (CBER)| BLA per 21 CFR §601, IND per 21 CFR §312       |
| EMA       | ATMP regulation (where applicable) or          |
|           | medicinal-product CTA per Reg 536/2014          |
| MFDS      | Biological-medicinal-product approval           |
| PMDA      | Biological-product / regenerative-medicine path |

## §7 Cross-domain WIA bindings

| Companion standard          | Binding purpose                                |
|-----------------------------|------------------------------------------------|
| WIA-medical-data-privacy    | special-category human samples                  |
| WIA-medical-research-data   | cohort metadata exchange                        |
| WIA-food-traceability       | food-process lot binding                        |
| WIA-food-safety             | regulator pathogen reporting                    |
| WIA-clinical-decision-support | biomarker call interpretation                |
| WIA-emergency-medical-data  | foodborne outbreak escalation                   |
| WIA-content-ai              | AI-assisted taxonomic classification governance |

Each binding identifies the consumed PHASE in the
companion standard.

## §8 Long-term archival

Microbiome archival horizons:

| Context              | Retention target                            |
|----------------------|---------------------------------------------|
| Public INSDC         | indefinite (archive policy)                 |
| Sponsor-controlled   | minimum protocol clock + regulator clock    |
| Regulator inspection | minimum 5 years post-study close             |
| Biobank aliquot      | indefinite or until consent withdrawal      |

Withdrawal of consent triggers deletion of the
specimen and read-set raw data; analytical results
already published in aggregate de-identified form are
retained per the original consent.

## §9 Conformance test suite

The reference test suite covers:

- chain-of-custody completeness on a synthetic specimen
- contamination-control gate enforcement
- reproducibility tier classification
- INSDC submission cross-walk (BioSample / Experiment / Run)
- FHIR R5 export of Specimen + Observation
- LOINC code resolution on a clinical panel
- audit-chain hash continuity
- GDPR-aligned host-read removal verification
- batch-effect declaration on a multi-batch study
- regulator-screen positive pathogen escalation

## §10 Internationalisation

User-facing strings (specimen labels, report headings,
biomarker interpretation) carry the BCP 47 language
tag. Country-specific regulator paths are resolved by
the study's primary registry country code (ISO 3166-1
alpha-3).

## §11 Security and privacy posture

- Transport: TLS 1.3 with mutual TLS for archive
  submissions; ETSI EN 319 522 registered electronic
  delivery for jurisdictions that require it
- Authentication: SMART on FHIR for clinical contexts;
  client_credentials with key attestation for sponsor /
  archive integrations
- At-rest: AES-256-GCM with sponsor-controlled KMS;
  per-study key-wrapping per ISO/IEC 27002 §8.24
- Audit: tamper-evident chain (PHASE 3 §6) exportable
  per ISO/IEC 27037 forensic-evidence guidance
- Privacy: subject identifiers opaque; host-read
  fraction reported and verified before public release;
  re-linkage table held by sponsor under regulator-
  approved DPIA

## §12 Operational metrics

Sponsors report (informationally) on the WIA registry:

- specimens collected vs. submitted
- run QC pass rate
- analyses succeeded / failed / abandoned
- contamination-control pass rate
- public-release timeliness vs. embargo

## §13 Recovery and continuity

- API outage — local lab capture; sync on reconnect
- archive outage — queued submission events; replay on
  recovery
- reference-database outage — pin a local mirror with
  the same release identifier; document mirror integrity
  via SHA-256 manifest

## Annex A — Worked end-to-end example (informative)

A multicentre study of antibiotic-associated dysbiosis
runs at four academic medical centres. Specimens (stool,
preserved in OMNIgene-GUT, shipped on dry ice within
24 h) are received at the central sequencing facility,
extracted in batches of 96 with an extraction blank per
batch, prepared as 16S V3V4 amplicon libraries, and
sequenced on a NovaSeq SP flow cell. Analyses run on
nf-core/ampliseq pinned to v2.7 with the SILVA 138.2
reference. Decontamination uses extraction-blank reads
to remove reagent ASVs. Results submit to ENA under
embargo until publication; the analytical-result record
declares reproducible-strong. Publication releases the
ENA records and the WIA analytical-result.

## Annex B — Conformance disclosure

Implementations declare the INSDC node they bind to,
the FHIR profile versions they expose, the reference-
database releases they support, the LOINC release they
match against, and the workflow runtimes they accept.

## Annex C — Versioning

PHASE 4 follows the standard's semantic-versioning
policy. Adding a new regulator gateway is minor;
changing the FHIR mapping is major.

## Annex D — Submission embargo policy (informative)

Embargo periods reflect the upstream archive policy:

| Archive       | Maximum embargo                              |
|---------------|----------------------------------------------|
| NCBI SRA      | 4 years from submission (extendable on case) |
| ENA           | 2 years (default; extendable per request)    |
| DDBJ DRA      | 2 years                                      |

Embargo lift events are recorded on the WIA submission
record so downstream consumers see the expected public
release date without polling the upstream archive.

## Annex E — Public-summary bridge

Where regulators (e.g. FDA Project Patron, EMA OPEN+)
require a public-summary alongside the underlying data,
the standard binds the summary URI to the analytical-
result record as `publicSummaryRef`. Updates to the
summary emit audit events linked to the underlying
result.
