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
