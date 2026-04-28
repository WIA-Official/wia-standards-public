# WIA-microbiome PHASE 1 — Data Format Specification

**Standard:** WIA-microbiome
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for human,
animal, environmental, and food-process microbiome data
covering specimen collection, library preparation,
sequencing run, taxonomic classification, functional
profile, and downstream analytical output. Records are
shaped so a regulator, a clinical sponsor, an academic
biobank, or an industrial fermentation operator can
exchange microbiome datasets without reconciliation,
and so cohort studies remain reproducible across
sequencing platforms and bioinformatic pipelines.

References (CITATION-POLICY ALLOW only):
- ISO 20387:2018 — General requirements for biobanking
- ISO 22174:2019 / ISO 22118:2011 / ISO 20837:2006 — molecular methods for foodborne pathogens
- ISO/IEC 17025:2017 — Testing and calibration laboratories
- INSDC — International Nucleotide Sequence Database Collaboration (NCBI / ENA / DDBJ)
- MIxS / MIMARKS / MIMS — GSC (Genomic Standards Consortium) minimum information checklists
- MGnify (EMBL-EBI) — microbiome resource and pipeline reference
- NIH Human Microbiome Project (HMP), HMP2 / iHMP data conventions
- GTDB — Genome Taxonomy Database
- SILVA / Greengenes2 — rRNA reference databases
- KEGG / MetaCyc / EC nomenclature (IUBMB) — metabolic pathway references
- FHIR R5 — Specimen, Observation, MolecularSequence
- HL7 v2.5 OUL^R22 — laboratory result message
- Codex Alimentarius CAC/RCP 1-1969 — General Principles of Food Hygiene
- US FDA — Live Biotherapeutic Products (LBP) draft guidance; EMA Q&A on quality of LBPs
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID)

---

## §1 Scope

This PHASE applies to systems that generate, archive,
or interpret microbiome data from any sample matrix:
human stool, oral, skin, vaginal, respiratory, or
tissue specimens; animal husbandry samples; soil,
water, marine, or built-environment samples; and food
production fermentation, brewing, dairy, or biofilm
matrices.

In scope: specimen record, collection record, library-
preparation record, sequencing-run record, read record,
taxonomy record, functional-profile record, and
analytical-result record (alpha- / beta-diversity,
differential abundance, biomarker call). Out of scope:
host genotype data (handled by population-genomics
standards), unrelated single-organism whole-genome
sequencing (handled by clinical genomics standards),
and metabolomics independent of microbial community
context.

## §2 Specimen record

| Field                  | Source / Binding                                  |
|------------------------|---------------------------------------------------|
| `specimenRef`          | UUID (RFC 4122)                                   |
| `subjectRef`           | for human/animal — opaque study-local identifier  |
| `matrix`               | controlled term per MIxS environmental package    |
|                        | (`stool`, `oral`, `skin`, `vaginal`, `nasal`,     |
|                        | `soil`, `water`, `food`, `surface`, `air`)        |
| `anatomicalSite`       | UBERON ontology term, where applicable            |
| `siteCondition`        | clinical / dietary / habitat context (free text   |
|                        | bound to a controlled vocabulary per study)       |
| `collectionDate`       | ISO 8601                                          |
| `preservationMethod`   | `none`, `OMNIgene-GUT`, `RNAlater`, `DNA/RNA-     |
|                        | Shield`, `flash-frozen-LN2`, `dry-ice`,           |
|                        | `ethanol-95`, `formalin-FFPE`                     |
| `storageTemp`          | numeric °C; ISO 80000-5 °C unit                   |
| `chainOfCustodyRef[]`  | events from collection to extraction              |
| `studyRef`             | parent study identifier                           |
| `consentRef`           | for human / animal subjects                       |

Specimens that fail preservation criteria carry
`status=excluded` and are not bound to downstream
records.

## §3 Library-preparation record

| Field                  | Source / Binding                                  |
|------------------------|---------------------------------------------------|
| `libraryRef`           | UUID                                              |
| `specimenRef`          | §2                                                |
| `extractionKit`        | manufacturer + catalogue number + lot             |
| `targetRegion`         | for amplicon: `16S-V1V2`, `16S-V3V4`, `16S-V4`,   |
|                        | `ITS1`, `ITS2`, `18S-V9`; or `shotgun-WMS`        |
| `primerSet`            | for amplicon: forward / reverse primer sequences  |
| `pcrCycles`            | numeric                                           |
| `quantification`       | Qubit dsDNA (ng/µL), Bioanalyzer fragment size    |
| `barcodeIndex`         | i7 / i5 index sequences                           |
| `negativeControlRef`   | extraction blank linkage                          |
| `positiveControlRef`   | mock-community linkage (e.g. ZymoBIOMICS,         |
|                        | NIST RM 8376, ATCC mock)                          |

Negative-control linkage is required so contamination
sources can be subtracted at the analysis step.

## §4 Sequencing-run record

| Field                  | Source / Binding                                  |
|------------------------|---------------------------------------------------|
| `runRef`               | UUID                                              |
| `instrument`           | manufacturer + model (Illumina MiSeq / NovaSeq /  |
|                        | NextSeq, PacBio Sequel / Revio, Oxford Nanopore   |
|                        | MinION / GridION / PromethION, Element AVITI)     |
| `flowcellId`           | flow-cell identifier                              |
| `runDate`              | ISO 8601                                          |
| `chemistry`            | manufacturer chemistry version                    |
| `readLength`           | numeric (per end if paired-end)                   |
| `endedness`            | `single`, `paired`                                |
| `librariesRef[]`       | libraries multiplexed on this run                 |
| `qcMetrics`            | per-run Q30 (%), cluster density, demux rate      |
| `submissionRef`        | INSDC accession (BioProject / BioSample / SRA /   |
|                        | ENA Run accession) once submitted                 |

The `submissionRef` is mandatory for studies declared
public; private analyses retain a placeholder until
submission.

## §5 Read record

| Field                  | Source / Binding                                  |
|------------------------|---------------------------------------------------|
| `readSetRef`           | UUID                                              |
| `runRef`               | §4                                                |
| `libraryRef`           | §3                                                |
| `fileFormat`           | `FASTQ`, `BAM`, `CRAM`, `unaligned-BAM`           |
| `fileChecksum`         | SHA-256                                           |
| `byteSize`             | numeric                                           |
| `qcSummary`            | per-base Q30 distribution, adapter rate, length   |
|                        | distribution, host-read fraction (for human       |
|                        | microbiome)                                       |

Host-read removal is recorded as a numbered processing
step rather than overwriting the original read set.

## §6 Taxonomy record

| Field                  | Source / Binding                                  |
|------------------------|---------------------------------------------------|
| `taxonomyRef`          | UUID                                              |
| `inputRef`             | read-set or assembly                              |
| `pipelineVersion`      | tool + version (DADA2 / QIIME 2 / mothur /        |
|                        | Kraken2 + Bracken / MetaPhlAn / sourmash)         |
| `referenceDb`          | SILVA / Greengenes2 / GTDB / NCBI RefSeq + version|
| `featureTable`         | URI to BIOM v2.1 or HDF5 feature table            |
| `taxonomyAssignment`   | URI to feature-to-taxonomy mapping                |
| `confidenceMetric`     | per-feature classification confidence             |
| `unclassifiedFraction` | proportion of reads not assigned                  |

Assignment is preserved at all canonical ranks (domain,
phylum, class, order, family, genus, species, strain
when shotgun supports it) per GTDB conventions.

## §7 Functional-profile record

| Field                  | Source / Binding                                  |
|------------------------|---------------------------------------------------|
| `functionalRef`        | UUID                                              |
| `inputRef`             | read-set or contig assembly                       |
| `pipelineVersion`      | HUMAnN3, MEGAHIT + Bowtie2 + DIAMOND + KEGG /     |
|                        | MetaCyc, AMRFinder, eggNOG-mapper, CARD RGI       |
| `pathwayCatalogue`     | KEGG / MetaCyc / SEED + version                   |
| `geneFamilies`         | URI to gene-family abundance table                |
| `pathwayAbundance`     | URI to pathway-abundance table                    |
| `amrGenes`             | URI to AMR-gene call table (CARD / NCBI AMR)      |
| `virulenceGenes`       | URI to virulence-factor call table (VFDB)         |

## §8 Analytical-result record

| Field                  | Source / Binding                                  |
|------------------------|---------------------------------------------------|
| `resultRef`            | UUID                                              |
| `studyRef`             | parent study                                      |
| `cohortDefinition`     | inclusion / exclusion criteria payload            |
| `metric`               | `alpha-diversity-shannon`, `alpha-richness-       |
|                        | observed`, `beta-diversity-bray-curtis`,          |
|                        | `beta-diversity-unifrac-weighted`,                |
|                        | `differential-abundance-ANCOM-BC`,                |
|                        | `differential-abundance-MaAsLin2`,                |
|                        | `random-forest-classifier`, `picrust2-prediction` |
| `analysisCode`         | URI to reproducible workflow (Nextflow, Snakemake,|
|                        | WDL, CWL, with container digest)                  |
| `inputManifest`        | feature table + metadata table digest             |
| `resultArtefact`       | URI to figure / table (with file digest)          |
| `interpretation`       | bound text + reviewer credentials                 |

Reproducibility requires the workflow URI, the input
manifest digest, and the container image digest to be
recorded so an auditor can re-execute byte-for-byte.

## §9 Cross-domain references (informative)

- WIA-medical-data-privacy — special-category human samples
- WIA-food-traceability — food-process microbiome lots
- WIA-clinical-decision-support — biomarker reporting

## Annex A — Mock-community manifest (informative)

```json
{
  "mockId": "ZymoBIOMICS-D6300-Lot-2026-A",
  "expectedComposition": [
    {"species": "Pseudomonas aeruginosa", "percent": 12.0},
    {"species": "Escherichia coli",       "percent": 12.0},
    {"species": "Salmonella enterica",    "percent": 12.0},
    {"species": "Lactobacillus fermentum","percent": 12.0},
    {"species": "Enterococcus faecalis",  "percent": 12.0},
    {"species": "Staphylococcus aureus",  "percent": 12.0},
    {"species": "Listeria monocytogenes", "percent": 12.0},
    {"species": "Bacillus subtilis",      "percent": 12.0},
    {"species": "Saccharomyces cerevisiae","percent": 2.0},
    {"species": "Cryptococcus neoformans","percent": 2.0}
  ]
}
```

## Annex B — INSDC submission cross-walk

| Record         | INSDC artefact                                     |
|----------------|----------------------------------------------------|
| Specimen       | BioSample (with MIxS environmental package)         |
| Library        | Experiment + run-association                        |
| Read set       | SRA / ENA Run                                       |
| Assembly       | GenBank / ENA Assembly                              |
| Project        | BioProject                                          |

Public studies submit before publication; embargoed
studies hold accessions in suppressed state and release
on the embargo lift date.

## Annex C — Conformance disclosure

Implementations declare the MIxS package version they
emit, the reference databases they index against, and
the BIOM / Parquet schema versions for feature tables.

## Annex D — Versioning

This PHASE follows the standard's semantic-versioning
policy. Adding a new pipeline-version slot is a minor
bump; changing the feature-table semantics is major.
