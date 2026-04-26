# WIA-BIO-003: Data Formats Specification

## Version 1.0 | 2025-01-15

## Table of Contents
1. [Overview](#overview)
2. [Biomarker Data Exchange Formats](#biomarker-data-exchange-formats)
3. [Omics Data Formats](#omics-data-formats)
4. [Clinical Data Formats](#clinical-data-formats)
5. [Interoperability Standards](#interoperability-standards)

## Overview

WIA-BIO-003 defines standardized data formats for biomarker discovery, validation, and clinical implementation ensuring interoperability across platforms and regulatory compliance.

### Format Priority

| Data Type | Primary Format | Alternative | Use Case |
|-----------|---------------|-------------|----------|
| **Biomarker Definition** | JSON, FHIR | XML | Registry, EHR integration |
| **Genomic Data** | VCF, BAM | FASTA | Mutation biomarkers |
| **Proteomic Data** | mzML, mzIdentML | CSV | Mass spec quantification |
| **Metabolomic Data** | mzML, ISA-Tab | Custom JSON | Metabolite profiling |
| **Clinical Data** | HL7 FHIR, CDISC ODM | CSV | Trials, validation |

## Biomarker Data Exchange Formats

### JSON Biomarker Definition

```json
{
  "biomarkerId": "BM_001",
  "name": "Oncotype DX",
  "version": "1.0",
  "type": "prognostic",
  "indication": {
    "disease": "Breast Cancer",
    "icd10": "C50",
    "stage": "Early stage, ER+, HER2-, node-negative",
    "population": "Female, 18-70 years"
  },
  "analytes": [
    {
      "gene": "ESR1",
      "type": "mRNA_expression",
      "weight": 0.8,
      "method": "RT-PCR"
    },
    {
      "gene": "PGR",
      "type": "mRNA_expression",
      "weight": 0.5,
      "method": "RT-PCR"
    }
  ],
  "algorithm": {
    "type": "weighted_sum",
    "formula": "RS = 0.47 × GRB7_group + 0.34 × ER_group - 0.03 × PR_group + 0.10 × Proliferation_group + 0.05 × Invasion_group",
    "scoreRange": [0, 100],
    "interpretation": {
      "low": {"range": [0, 17], "recurrenceRisk": "<10%"},
      "intermediate": {"range": [18, 30], "recurrenceRisk": "10-20%"},
      "high": {"range": [31, 100], "recurrenceRisk": ">20%"}
    }
  },
  "validation": {
    "analyticalValidation": {
      "LOD": "10 ng total RNA",
      "LOQ": "25 ng total RNA",
      "precision_within_run": "CV 3.5%",
      "precision_between_run": "CV 5.2%"
    },
    "clinicalValidation": {
      "studyName": "TAILORx",
      "patients": 10273,
      "AUC": 0.83,
      "sensitivity": 0.88,
      "specificity": 0.85
    }
  },
  "specimen": {
    "type": "FFPE_tissue",
    "volume": "1 tumor block",
    "collection": "Standard surgical excision",
    "storage": "Room temperature (FFPE) or -80°C (frozen)"
  },
  "turnaroundTime": "7-10 business days",
  "regulatoryStatus": {
    "FDA": "510(k) cleared (K052990)",
    "CE_Mark": true,
    "CLIA_certified": true
  }
}
```

### LOINC Coding (Standard Vocabulary)

```
Biomarker Test LOINC Codes:
- Oncotype DX: 77895-3
- BRCA1 mutation analysis: 81479-0
- BRCA2 mutation analysis: 81480-8
- PSA (prostate specific antigen): 2857-1
- Troponin I, cardiac: 10839-9
- HbA1c: 4548-4
```

## Omics Data Formats

### Genomic Biomarkers (VCF)

```vcf
##fileformat=VCFv4.3
##INFO=<ID=CLINSIG,Number=.,Type=String,Description="Clinical significance from ClinVar">
##INFO=<ID=AF,Number=A,Type=Float,Description="Allele frequency from population databases">
#CHROM  POS     ID      REF ALT QUAL    FILTER  INFO
chr17   43044295    rs80357906  G   A   99  PASS    CLINSIG=Pathogenic;AF=0.0001;GENE=BRCA1
chr13   32315474    rs80359550  G   T   99  PASS    CLINSIG=Pathogenic;AF=0.00005;GENE=BRCA2
chr7    55191822    rs121434568 T   G   99  PASS    CLINSIG=Pathogenic;AF=0.005;GENE=EGFR;MUTATION=T790M
```

### Proteomic Biomarkers (mzML)

```xml
<mzML xmlns="http://psi.hupo.org/ms/mzml" version="1.1.0">
  <cvList>
    <cv id="MS" fullName="Proteomics Standards Initiative Mass Spectrometry Ontology"/>
    <cv id="UO" fullName="Unit Ontology"/>
  </cvList>
  <run id="BIOMARKER_RUN_001">
    <chromatogram id="TIC">
      <cvParam cvRef="MS" accession="MS:1000235" name="total ion current chromatogram"/>
    </chromatogram>
    <spectrum id="scan=1">
      <cvParam cvRef="MS" accession="MS:1000511" name="ms level" value="1"/>
      <cvParam cvRef="MS" accession="MS:1000016" name="scan start time" value="5.2" unitCvRef="UO" unitAccession="UO:0000031" unitName="minute"/>
      <binaryDataArrayList count="2">
        <binaryDataArray encodedLength="160">
          <cvParam cvRef="MS" accession="MS:1000514" name="m/z array"/>
          <!-- Base64 encoded m/z values -->
        </binaryDataArray>
        <binaryDataArray encodedLength="160">
          <cvParam cvRef="MS" accession="MS:1000515" name="intensity array"/>
          <!-- Base64 encoded intensity values -->
        </binaryDataArray>
      </binaryDataArrayList>
    </spectrum>
  </run>
</mzML>
```

### Metabolomic Biomarkers (ISA-Tab)

**i_investigation.txt:**
```
ONTOLOGY SOURCE REFERENCE
Term Source Name    CHEBI    HMDB
Term Source File    http://www.ebi.ac.uk/chebi/    http://www.hmdb.ca/
```

**s_study.txt:**
```
Sample Name    Organism    Tissue    Disease
PATIENT_001    Homo sapiens    Plasma    Diabetes Type 2
PATIENT_002    Homo sapiens    Plasma    Healthy Control
```

**a_assay.txt:**
```
Sample Name    Protocol REF    MS Assay Name    Metabolite Identifier    Concentration    Unit
PATIENT_001    LC-MS/MS    ASSAY_001    HMDB0000517    25.3    μM
PATIENT_001    LC-MS/MS    ASSAY_001    HMDB0000191    8.7    μM
```

## Clinical Data Formats

### HL7 FHIR Observation (Biomarker Result)

```json
{
  "resourceType": "Observation",
  "id": "biomarker-oncotype-001",
  "status": "final",
  "code": {
    "coding": [
      {
        "system": "http://loinc.org",
        "code": "77895-3",
        "display": "Oncotype DX Breast Recurrence Score"
      }
    ]
  },
  "subject": {
    "reference": "Patient/PATIENT_001"
  },
  "effectiveDateTime": "2025-01-15T09:00:00Z",
  "issued": "2025-01-22T14:00:00Z",
  "performer": [
    {
      "reference": "Organization/GENOMIC_HEALTH_INC"
    }
  ],
  "valueQuantity": {
    "value": 15,
    "unit": "Recurrence Score",
    "system": "http://unitsofmeasure.org",
    "code": "{score}"
  },
  "interpretation": [
    {
      "coding": [
        {
          "system": "http://terminology.hl7.org/CodeSystem/v3-ObservationInterpretation",
          "code": "L",
          "display": "Low"
        }
      ],
      "text": "Low risk of recurrence (<10% at 10 years)"
    }
  ],
  "referenceRange": [
    {
      "low": {"value": 0},
      "high": {"value": 17},
      "type": {
        "coding": [
          {
            "system": "http://terminology.hl7.org/CodeSystem/referencerange-meaning",
            "code": "normal",
            "display": "Low risk"
          }
        ]
      }
    }
  ],
  "component": [
    {
      "code": {
        "text": "Chemotherapy Recommendation"
      },
      "valueString": "Chemotherapy not recommended based on low recurrence score"
    }
  ]
}
```

### CDISC SDTM (Biomarker Findings)

```sas
/* BM (Biomarker) Domain */
data bm;
  length STUDYID $ 20 USUBJID $ 40 BMTEST $ 40 BMCAT $ 40;
  
  STUDYID = "STUDY_001";
  USUBJID = "STUDY_001_SUBJ_001";
  BMTESTCD = "ONCOTYPE";
  BMTEST = "Oncotype DX Recurrence Score";
  BMCAT = "GENOMIC BIOMARKER";
  BMORRES = "15";
  BMORRESU = "score";
  BMSTRESC = "15";
  BMSTRESN = 15;
  BMSTRESU = "score";
  VISITNUM = 1;
  VISIT = "Screening";
  BMDTC = "2025-01-15";
  
  output;
run;
```

## Interoperability Standards

### GA4GH Phenopackets (Biomarker Context)

```json
{
  "id": "PHENOPACKET_BIOMARKER_001",
  "subject": {
    "id": "PATIENT_001",
    "sex": "FEMALE",
    "timeAtLastEncounter": {
      "age": {"iso8601duration": "P55Y"}
    }
  },
  "diseases": [
    {
      "term": {
        "id": "MONDO:0007254",
        "label": "Breast carcinoma"
      },
      "diseaseStage": [
        {
          "term": {
            "id": "NCIT:C27966",
            "label": "Stage II Breast Cancer"
          }
        }
      ]
    }
  ],
  "measurements": [
    {
      "assay": {
        "id": "LOINC:77895-3",
        "label": "Oncotype DX Breast Recurrence Score"
      },
      "value": {
        "quantity": {
          "value": 15.0,
          "unit": {
            "id": "UCUM:{score}",
            "label": "score"
          }
        }
      },
      "timeObserved": {
        "timestamp": "2025-01-15T09:00:00Z"
      }
    },
    {
      "assay": {
        "id": "LOINC:16112-5",
        "label": "Estrogen receptor"
      },
      "value": {
        "quantity": {
          "value": 95.0,
          "unit": {
            "id": "UCUM:%",
            "label": "percent"
          }
        }
      }
    }
  ]
}
```

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA BIO Working Group
**License:** Apache 2.0


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
