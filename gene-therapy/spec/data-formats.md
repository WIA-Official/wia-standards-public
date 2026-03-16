# WIA-BIO-002: Data Formats Specification

## Version 1.0 | 2025-01-15

---

## Table of Contents
1. [Overview](#overview)
2. [Vector Design Formats](#vector-design-formats)
3. [Manufacturing Data](#manufacturing-data)
4. [Clinical Trial Data](#clinical-trial-data)
5. [Regulatory Submission Formats](#regulatory-submission-formats)
6. [Integration Site Formats](#integration-site-formats)
7. [Interoperability Standards](#interoperability-standards)

---

## Overview

This specification defines data formats for gene therapy development, manufacturing, and clinical deployment under WIA-BIO-002. Formats enable interoperability between plasmid design tools, manufacturing execution systems, and regulatory databases.

### Format Categories

| Category | Primary Format | Alternative | Use Case |
|----------|---------------|-------------|----------|
| **Vector Design** | GenBank (.gb) | FASTA, SnapGene | Plasmid maps, sequence annotation |
| **Manufacturing** | JSON, LIMS XML | CSV, Excel | Batch records, QC data |
| **Clinical Data** | HL7 FHIR | CDISC ODM | Patient outcomes, AEs |
| **Regulatory** | eCTD (XML) | PDF | IND/BLA submissions |
| **Integration Sites** | BED, BAM | Custom JSON | LAM-PCR mapping |

---

## Vector Design Formats

### GenBank Format (.gb)

**Standard for plasmid/vector annotation.**

**Example: AAV2-CMV-GFP Vector**
```genbank
LOCUS       AAV2_CMV_GFP            4679 bp    DNA     circular SYN 15-JAN-2025
DEFINITION  AAV2 vector expressing GFP under CMV promoter
ACCESSION   WIA_BIO_002_001
VERSION     WIA_BIO_002_001.1
KEYWORDS    gene therapy; AAV; GFP reporter
SOURCE      synthetic DNA construct
  ORGANISM  synthetic DNA construct
FEATURES             Location/Qualifiers
     rep_origin      1..589
                     /label=AAV2 ITR (Left)
                     /note="Inverted terminal repeat"
     promoter        612..1211
                     /label=CMV immediate-early promoter
                     /note="Human cytomegalovirus immediate early promoter"
     CDS             1235..1954
                     /codon_start=1
                     /label=GFP
                     /note="Green fluorescent protein"
                     /translation="MSKGEELFTGVVPILVELDGDVNGHKFSVSGEGEGDATYGKLT
                     LKFICTTGKLPVPWPTLVTTFSYGVQCFSRYPDHMKQHDFFKSAMPEGYVQERTIFF
                     KDDGNYKTRAEVKFEGDTLVNRIELKGIDFKEDGNILGHKLEYNYNSHNVYIMADKQ
                     KNGIKVNFKIRHNIEDGSVQLADHYQQNTPIGDGPVLLPDNHYLSTQSALSKDPNEK
                     RDHMVLLEFVTAAGITHGMDELYK"
     polyA_signal    1988..2212
                     /label=SV40 late polyA
                     /note="Simian virus 40 late polyadenylation signal"
     rep_origin      2250..2839
                     /label=AAV2 ITR (Right)
                     /note="Inverted terminal repeat (reversed complement)"
     rep_origin      2900..3488
                     /label=pUC ori
                     /note="Bacterial origin of replication"
     CDS             complement(3600..4460)
                     /codon_start=1
                     /label=AmpR
                     /note="Beta-lactamase (ampicillin resistance)"
                     /translation="MSIQHFRVALIPFFAAFCLPVFAHPETLVKVKDAEDQLGARVG
                     YIELDLNSGKILESFRPEERFPMMSTFKVLLCGAVLSRVDAGQEQLGRRIHYSQNDI
                     VVAIEPLKVVPNFYLRKPVQMDLSQNTYSGIQRTSPPRRSLLLAAIALFVLAFWSSD
                     SFGKQSSRSAGGVQKGCGPAHQDSTDRNQKRLTDSLVGKAGFKADYSNASDAKGFST
                     ATDEAKRFFAGVPMSAIGDKGVTIDAATEEVEKTKKSALKQPVKEKVAAADSTGGGS
                     SGGVGKDSMKSKTDLLTSVGAQKRDRKS"
ORIGIN
        1 cctgcaggca gctgcgcgct cgctcgctca ctgaggccgc ccgggcaaag cccgggcgtc
       61 gggcgacctt tggtcgcccg gcctcagtga gcgagcgagc gcgcagagag ggagtggcca
...
//
```

**Required FEATURES:**

| Feature Type | Required Fields | Example |
|--------------|-----------------|---------|
| **ITR** (AAV) | `/label`, `/note` | "AAV2 ITR Left" |
| **Promoter** | `/label`, tissue specificity | "CMV-IE enhancer" |
| **CDS** | `/translation`, `/codon_start` | Transgene ORF |
| **polyA** | `/label` | "SV40 late polyA" |
| **Regulatory** | LTR (lentivirus), WPRE, etc. | Annotate all elements |

---

### FASTA Format (Simplified Sequence)

**For sequence submission, BLAST queries.**

```fasta
>AAV2_CMV_GFP_vector WIA-BIO-002 reference construct
CCTGCAGGCAGCTGCGCGCTCGCTCGCTCACTGAGGCCGCCCGGGCAAAGCCCGGGCGTC
GGGCGACCTTTGGTCGCCCGGCCTCAGTGAGCGAGCGAGCGCGCAGAGAGGGAGTGGCCA
ACTCCATCACTAGGGGTTCCTTGTAGTTAATGATTAACCCGCCATGCTACTTATCTACGTA
...
```

**Naming Convention:**
```
>VectorID_Serotype_Promoter_Transgene version=1.0 date=2025-01-15
```

---

### SnapGene/Benchling JSON (Modern Tools)

**SnapGene DNA File Format (proprietary but widely used):**

```json
{
  "name": "AAV2-CMV-GFP",
  "sequence": "CCTGCAGGCA...",
  "circular": true,
  "features": [
    {
      "name": "AAV2 ITR",
      "type": "rep_origin",
      "start": 0,
      "end": 589,
      "strand": 1,
      "color": "#ff9ccd"
    },
    {
      "name": "CMV promoter",
      "type": "promoter",
      "start": 611,
      "end": 1210,
      "strand": 1,
      "color": "#00ccff",
      "notes": "Strong constitutive promoter"
    }
  ],
  "primers": [
    {
      "name": "ITR_F",
      "sequence": "CCTGCAGGCAGCTGCGCGCT",
      "tm": 62.5
    }
  ]
}
```

---

## Manufacturing Data

### Batch Record Format (JSON)

**WIA-BIO-002 Standard Batch Record:**

```json
{
  "batchId": "AAV9-SMN-20250115-001",
  "productName": "AAV9-hSMN1",
  "vectorDesign": {
    "serotype": "AAV9",
    "transgene": "hSMN1",
    "promoter": "CB7",
    "genBankFile": "AAV9-SMN-v1.0.gb",
    "vectorGenomeSize": 4235
  },
  "manufacturing": {
    "method": "triple-transfection",
    "cellLine": "HEK293",
    "passage": "P12",
    "startDate": "2025-01-15T08:00:00Z",
    "harvestDate": "2025-01-18T16:00:00Z",
    "operator": "OPERATOR_001"
  },
  "plasmids": [
    {
      "name": "pAAV-CB7-SMN",
      "role": "transgene",
      "amount": "12.5 µg per 10^7 cells",
      "lot": "PLASMID_001",
      "qc": {
        "purity": "A260/A280 = 1.85",
        "endotoxin": "0.02 EU/µg",
        "sequenceVerified": true
      }
    },
    {
      "name": "pHelper",
      "role": "adenovirus helper",
      "amount": "25 µg per 10^7 cells",
      "lot": "PLASMID_002"
    },
    {
      "name": "pRepCap-AAV9",
      "role": "rep/cap genes",
      "amount": "12.5 µg per 10^7 cells",
      "lot": "PLASMID_003"
    }
  ],
  "harvest": {
    "cellCount": "3.2×10^9",
    "viability": "92%",
    "volumeHarvested": "5.0 L",
    "crudeYield": "8.2×10^13 vg"
  },
  "purification": {
    "steps": [
      {
        "step": "Benzonase treatment",
        "time": "37°C for 1 hour",
        "reagents": ["Benzonase 50 U/mL"]
      },
      {
        "step": "Iodixanol gradient ultracentrifugation",
        "fractions": ["40% iodixanol (full capsids)"],
        "recovery": "72%"
      },
      {
        "step": "Ion exchange chromatography (POROS HQ)",
        "elution": "NaCl gradient 0-500 mM",
        "peakFractions": "280-320 mM"
      },
      {
        "step": "Tangential flow filtration (TFF)",
        "finalBuffer": "PBS + 0.001% Pluronic F68",
        "concentration": "5×10^13 vg/mL"
      }
    ]
  },
  "finalProduct": {
    "lot": "AAV9-SMN-FP-001",
    "volume": "55 mL",
    "concentration": "5.2×10^13 vg/mL",
    "totalYield": "2.86×10^15 vg",
    "fillDate": "2025-01-22T14:00:00Z",
    "fillVolume": "5.5 mL per vial",
    "vialCount": 10,
    "storage": "-80°C"
  },
  "qualityControl": {
    "titer_qPCR": {
      "result": "5.2×10^13 vg/mL",
      "method": "ITR qPCR",
      "analyst": "ANALYST_001",
      "date": "2025-01-20",
      "status": "PASS"
    },
    "infectivity_TCID50": {
      "result": "2.6×10^12 IU/mL",
      "vg_to_IU_ratio": "20:1",
      "status": "PASS"
    },
    "purity_SDS_PAGE": {
      "VP1_VP2_VP3_ratio": "1:1:9.5",
      "status": "PASS"
    },
    "fullEmpty_AUC": {
      "percentFull": "84%",
      "status": "PASS"
    },
    "endotoxin": {
      "result": "0.8 EU/mL",
      "spec": "<5 EU/kg",
      "status": "PASS"
    },
    "sterility": {
      "bacteria": "No growth",
      "fungi": "No growth",
      "status": "PASS"
    },
    "mycoplasma": {
      "culture": "Negative",
      "PCR": "Negative",
      "status": "PASS"
    },
    "RCR": {
      "result": "Not detected",
      "LOD": "1 in 10^12 vg",
      "status": "PASS"
    }
  },
  "disposition": "RELEASED",
  "releaseDate": "2025-01-23T10:00:00Z",
  "approvedBy": "QA_MANAGER_001"
}
```

---

### LIMS Integration (XML)

**Example: Watson LIMS Sample Tracking**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<Sample xmlns="http://wia-bio.org/schema/gene-therapy/v1">
  <SampleID>AAV9-SMN-20250115-001</SampleID>
  <SampleType>AAV Vector</SampleType>
  <Status>Released</Status>
  <Tests>
    <Test>
      <TestID>TITER-001</TestID>
      <TestName>Vector Genome Titer</TestName>
      <Method>qPCR (ITR)</Method>
      <Result>5.2e13</Result>
      <Units>vg/mL</Units>
      <Specification>
        <Min>4.0e13</Min>
        <Max>6.0e13</Max>
      </Specification>
      <Status>PASS</Status>
      <Analyst>ANALYST_001</Analyst>
      <TestDate>2025-01-20T14:30:00Z</TestDate>
    </Test>
    <Test>
      <TestID>PURITY-001</TestID>
      <TestName>Full/Empty Ratio</TestName>
      <Method>Analytical Ultracentrifugation</Method>
      <Result>84</Result>
      <Units>% Full Capsids</Units>
      <Specification>
        <Min>80</Min>
      </Specification>
      <Status>PASS</Status>
    </Test>
  </Tests>
</Sample>
```

---

## Clinical Trial Data

### HL7 FHIR Resources

**MedicationAdministration (Gene Therapy Dose):**

```json
{
  "resourceType": "MedicationAdministration",
  "id": "gene-therapy-001",
  "status": "completed",
  "medicationCodeableConcept": {
    "coding": [
      {
        "system": "http://wia-bio.org/gene-therapy",
        "code": "AAV9-SMN",
        "display": "Onasemnogene abeparvovec (Zolgensma)"
      }
    ]
  },
  "subject": {
    "reference": "Patient/PATIENT_001",
    "display": "Patient with SMA Type 1"
  },
  "effectiveDateTime": "2025-01-15T10:00:00Z",
  "dosage": {
    "dose": {
      "value": 1.1e14,
      "unit": "vg/kg",
      "system": "http://unitsofmeasure.org",
      "code": "vg/kg"
    },
    "route": {
      "coding": [
        {
          "system": "http://snomed.info/sct",
          "code": "47625008",
          "display": "Intravenous route"
        }
      ]
    },
    "rateQuantity": {
      "value": 60,
      "unit": "min",
      "system": "http://unitsofmeasure.org",
      "code": "min"
    }
  },
  "note": [
    {
      "text": "Patient weight: 8.5 kg. Total dose: 9.35×10^14 vg in 5.5 mL. Infused over 60 minutes. Vital signs stable throughout."
    }
  ]
}
```

**Observation (Transgene Expression Monitoring):**

```json
{
  "resourceType": "Observation",
  "id": "transgene-expression-001",
  "status": "final",
  "code": {
    "coding": [
      {
        "system": "http://loinc.org",
        "code": "LA6576-8",
        "display": "Gene expression level"
      }
    ],
    "text": "SMN Protein Level"
  },
  "subject": {
    "reference": "Patient/PATIENT_001"
  },
  "effectiveDateTime": "2025-02-15T09:00:00Z",
  "valueQuantity": {
    "value": 4.2,
    "unit": "ng/mL",
    "system": "http://unitsofmeasure.org",
    "code": "ng/mL"
  },
  "referenceRange": [
    {
      "low": {
        "value": 2.0,
        "unit": "ng/mL"
      },
      "high": {
        "value": 8.0,
        "unit": "ng/mL"
      },
      "type": {
        "text": "Normal range for treated SMA patients"
      }
    }
  ],
  "note": [
    {
      "text": "4 weeks post-treatment. Protein levels within therapeutic range."
    }
  ]
}
```

---

### CDISC ODM (Clinical Data Interchange Standards Consortium)

**Used for regulatory submission of clinical trial data.**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<ODM xmlns="http://www.cdisc.org/ns/odm/v1.3"
     ODMVersion="1.3.2"
     FileType="Snapshot"
     FileOID="STUDY_AAV9_SMN_001">
  <Study OID="STUDY_001">
    <GlobalVariables>
      <StudyName>Phase III AAV9-SMN1 for SMA Type 1</StudyName>
      <StudyDescription>Efficacy and safety study</StudyDescription>
      <ProtocolName>WIA-BIO-002-001</ProtocolName>
    </GlobalVariables>
    <MetaDataVersion OID="MDV_001" Name="Version 1.0">
      <ItemGroupDef OID="IG_DOSE" Name="Dosing" Repeating="No">
        <ItemRef ItemOID="IT_DOSE_VG" Mandatory="Yes"/>
        <ItemRef ItemOID="IT_DOSE_DATE" Mandatory="Yes"/>
      </ItemGroupDef>
      <ItemDef OID="IT_DOSE_VG" Name="Vector Genome Dose" DataType="float">
        <Question><TranslatedText>Total vector genomes administered</TranslatedText></Question>
        <MeasurementUnitRef MeasurementUnitOID="MU_VG"/>
      </ItemDef>
    </MetaDataVersion>
    <ClinicalData StudyOID="STUDY_001" MetaDataVersionOID="MDV_001">
      <SubjectData SubjectKey="SUBJ_001">
        <FormData FormOID="FORM_DOSING">
          <ItemGroupData ItemGroupOID="IG_DOSE">
            <ItemData ItemOID="IT_DOSE_VG" Value="9.35e14"/>
            <ItemData ItemOID="IT_DOSE_DATE" Value="2025-01-15"/>
          </ItemGroupData>
        </FormData>
      </SubjectData>
    </ClinicalData>
  </Study>
</ODM>
```

---

## Regulatory Submission Formats

### eCTD (Electronic Common Technical Document)

**Folder Structure (FDA/EMA):**

```
eCTD/
├── m1-regional/
│   ├── us/
│   │   ├── 156-cover-letter.pdf
│   │   ├── 157-application-form.xml (FDA Form 356h)
│   │   └── 158-administrative-information.pdf
├── m2-summaries/
│   ├── 23-quality-overall-summary.pdf
│   ├── 24-nonclinical-overview.pdf
│   ├── 25-clinical-overview.pdf
│   └── 27-clinical-summary.pdf
├── m3-quality/
│   ├── 32-body-of-data/
│   │   ├── 32s-substance/
│   │   │   ├── 32s1-general-information.pdf
│   │   │   ├── 32s23-manufacture.pdf (vector production)
│   │   │   ├── 32s4-control-of-substance.pdf (QC methods)
│   │   │   └── 32s6-container-closure.pdf
│   │   └── 32p-product/
│   │       ├── 32p1-description-composition.pdf
│   │       ├── 32p3-manufacture.pdf (fill/finish)
│   │       ├── 32p5-control-of-product.pdf (release tests)
│   │       └── 32p8-stability.pdf
├── m4-nonclinical/
│   ├── 42-study-reports/
│   │   ├── 423-toxicology/
│   │   │   ├── biodistribution-mouse.pdf
│   │   │   ├── biodistribution-NHP.pdf
│   │   │   └── toxicology-GLP-NHP.pdf
│   │   └── 424-genetic-toxicology/
│   │       └── integration-site-analysis.pdf
└── m5-clinical/
    ├── 53-clinical-study-reports/
    │   ├── 535-efficacy-safety/
    │   │   ├── study-001-report.pdf
    │   │   ├── study-001-datasets.zip (CDISC SDTM)
    │   │   └── study-001-analysis.sas
    └── 54-literature-references.pdf
```

**Module 3.2.S.2.3: Manufacturing Process (Example Content):**

```markdown
# 3.2.S.2.3 Manufacturing Process Description

## AAV9-hSMN1 Vector Production

### Cell Culture and Transfection
- **Cell Line:** HEK293 (ATCC CRL-1573), Passage 12-20
- **Culture Medium:** DMEM + 10% FBS, L-glutamine
- **Seeding Density:** 4×10^7 cells per 10-layer CellStack (Corning)
- **Transfection Method:** PEI-based triple transfection at 80% confluency
- **Plasmid Ratio:** pAAV-CMV-SMN : pHelper : pRepCap9 = 1:2:1 (mass ratio)

### Harvest
- **Timing:** 72 hours post-transfection
- **Method:** Freeze-thaw (3 cycles) + benzonase treatment

### Purification
1. **Iodixanol density gradient ultracentrifugation** (OptiPrep)
   - Collect 40% fraction (ρ = 1.37 g/mL)
2. **Ion exchange chromatography** (POROS 50 HQ)
   - Load in 20 mM Tris pH 8.0
   - Elute with 0-500 mM NaCl gradient
3. **Tangential flow filtration** (Pellicon 100 kDa MWCO)
   - Buffer exchange into PBS + 0.001% Pluronic F68
   - Concentrate to 1×10^13 vg/mL

### Critical Process Parameters (CPPs)
| Parameter | Range | Justification |
|-----------|-------|---------------|
| Transfection efficiency | >75% | Impacts crude yield |
| Cell viability at harvest | >90% | Minimizes debris |
| Ultracentrifuge speed | 69,000 rpm (350,000 × g) | Full/empty separation |
| IEX elution pH | 8.0 ± 0.1 | Capsid stability |

### In-Process Controls (IPCs)
- **IPC-1:** Cell count and viability (Trypan blue) before transfection
- **IPC-2:** Crude titer (qPCR) in harvest lysate
- **IPC-3:** SDS-PAGE of peak fractions during IEX

---
```

---

## Integration Site Formats

### BED Format (Integration Site Mapping)

**LAM-PCR Integration Sites:**

```bed
#track name="LV-CAR19-Integration" description="Lentiviral integration sites from CAR-T product" useScore=1
chr1	12450123	12450124	INTEGRATION_001	120	+	PatientA	Week4	0.8%
chr1	45678901	45678902	INTEGRATION_002	85	-	PatientA	Week4	0.3%
chr2	98765432	98765433	INTEGRATION_003	200	+	PatientA	Week4	1.5%
chr7	55012345	55012346	INTEGRATION_004	450	+	PatientA	Week4	3.2%
chr11	2012345	2012346	INTEGRATION_005	1200	+	PatientA	Week4	9.1%
```

**BED Field Definitions:**

| Column | Field | Description |
|--------|-------|-------------|
| 1 | chrom | Chromosome (chr1-22, X, Y) |
| 2 | chromStart | Integration site position (0-based) |
| 3 | chromEnd | Position + 1 (BED convention) |
| 4 | name | Unique integration site ID |
| 5 | score | Number of reads supporting site |
| 6 | strand | + or - (orientation of provirus) |
| 7 | patient | Patient identifier |
| 8 | timepoint | Sample collection time |
| 9 | abundance | % of total integrations (clonality) |

---

### JSON Integration Report

```json
{
  "sampleId": "CAR-T-PATIENT-A-WEEK4",
  "vectorType": "Lentivirus-CAR19",
  "analysisDate": "2025-01-20",
  "method": "LAM-PCR + Illumina NGS",
  "totalReads": 1523456,
  "uniqueIntegrationSites": 8234,
  "clonality": {
    "polyclonal": true,
    "dominantClone": {
      "abundance": "9.1%",
      "gene": "BACH2",
      "distance_to_TSS": "12kb downstream",
      "risk": "LOW (not proto-oncogene)"
    }
  },
  "genomicDistribution": {
    "genic": "78%",
    "intergenic": "22%",
    "nearPromoter_1kb": "12%",
    "withinGene": "66%"
  },
  "flaggedSites": [
    {
      "chr": "chr11",
      "position": 2012345,
      "gene": "BACH2",
      "abundance": "9.1%",
      "reason": "Clone >5%, enhanced monitoring recommended",
      "action": "Monthly follow-up"
    }
  ],
  "integrationSites": [
    {
      "chr": "chr1",
      "position": 12450123,
      "strand": "+",
      "reads": 120,
      "abundance": "0.8%",
      "nearestGene": {
        "symbol": "GENE1",
        "distance": "5.2kb upstream",
        "function": "Transcription factor"
      }
    }
  ]
}
```

---

## Interoperability Standards

### GA4GH Phenopackets (Gene Therapy Context)

```json
{
  "id": "PHENOPACKET_SMA_001",
  "subject": {
    "id": "PATIENT_001",
    "timeAtLastEncounter": {
      "age": {
        "iso8601duration": "P5M"
      }
    },
    "sex": "MALE"
  },
  "phenotypicFeatures": [
    {
      "type": {
        "id": "HP:0001324",
        "label": "Muscle weakness"
      },
      "severity": {
        "id": "HP:0012828",
        "label": "Severe"
      }
    },
    {
      "type": {
        "id": "HP:0002747",
        "label": "Respiratory insufficiency"
      }
    }
  ],
  "diseases": [
    {
      "term": {
        "id": "OMIM:253300",
        "label": "Spinal Muscular Atrophy Type 1"
      },
      "onset": {
        "age": {
          "iso8601duration": "P2M"
        }
      }
    }
  ],
  "medicalActions": [
    {
      "treatment": {
        "agent": {
          "id": "NCIT:C173811",
          "label": "Onasemnogene Abeparvovec"
        },
        "doseIntervals": [
          {
            "quantity": {
              "value": 1.1e14,
              "unit": {
                "id": "UO:0000301",
                "label": "vector genomes per kilogram"
              }
            },
            "scheduleFrequency": {
              "id": "NCIT:C64576",
              "label": "Once"
            }
          }
        ],
        "routeOfAdministration": {
          "id": "NCIT:C38276",
          "label": "Intravenous route of administration"
        }
      },
      "treatmentTarget": {
        "id": "OMIM:253300",
        "label": "Spinal Muscular Atrophy Type 1"
      },
      "treatmentIntent": {
        "id": "NCIT:C62220",
        "label": "Curative"
      },
      "responseToTreatment": {
        "id": "NCIT:C94332",
        "label": "Improved"
      },
      "adverseEvents": []
    }
  ],
  "files": [
    {
      "uri": "file:///data/AAV9-SMN-batch-001.json",
      "fileAttributes": {
        "fileFormat": "JSON",
        "description": "Manufacturing batch record"
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
