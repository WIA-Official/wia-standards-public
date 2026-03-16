# WIA-BIO-002: Integration & API Specification

## Version 1.0 | 2025-01-15

---

## Table of Contents
1. [Overview](#overview)
2. [REST API](#rest-api)
3. [Vector Design API](#vector-design-api)
4. [Manufacturing API](#manufacturing-api)
5. [Clinical Monitoring API](#clinical-monitoring-api)
6. [Regulatory Submission Integration](#regulatory-submission-integration)
7. [Interoperability Standards](#interoperability-standards)

---

## Overview

WIA-BIO-002 provides comprehensive APIs for gene therapy development, manufacturing, and clinical deployment. The APIs enable integration with plasmid design tools, manufacturing execution systems (MES), electronic health records (EHR), and regulatory submission platforms.

### API Design Principles

| Principle | Implementation |
|-----------|----------------|
| **RESTful** | HTTP verbs, resource-based URLs |
| **Versioned** | `/v1/`, `/v2/` in URL path |
| **Authenticated** | OAuth 2.0 + API keys |
| **Documented** | OpenAPI 3.0 specification |
| **Secure** | TLS 1.3, HIPAA-compliant |
| **Rate-Limited** | 1000 requests/hour (authenticated) |

---

## REST API

### Base URLs

```
Production:  https://api.wia-bio.org/gene-therapy/v1
Staging:     https://api-staging.wia-bio.org/gene-therapy/v1
Regulatory:  https://api-regulatory.wia-bio.org/gene-therapy/v1 (FDA/EMA access)
```

### Authentication

**OAuth 2.0 (Recommended):**
```bash
# Get access token
curl -X POST https://auth.wia-bio.org/oauth/token \
  -H "Content-Type: application/json" \
  -d '{
    "grant_type": "client_credentials",
    "client_id": "YOUR_CLIENT_ID",
    "client_secret": "YOUR_CLIENT_SECRET",
    "scope": "gene_therapy.read gene_therapy.write manufacturing.read"
  }'

# Response
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "gene_therapy.read gene_therapy.write manufacturing.read"
}

# Use token
curl -H "Authorization: Bearer <token>" \
  https://api.wia-bio.org/gene-therapy/v1/vectors
```

---

## Vector Design API

### Endpoints

#### 1. Create Vector Design

**POST** `/v1/vectors`

```json
{
  "name": "AAV9-CMV-GFP",
  "vectorType": "AAV",
  "serotype": "AAV9",
  "payload": {
    "promoter": {
      "name": "CMV-IE",
      "sequence": "ATCGATCGATCG...",
      "tissueSpecificity": "ubiquitous"
    },
    "transgene": {
      "name": "eGFP",
      "cds": "ATGGTGAGCAAG...",
      "codonOptimized": true,
      "species": "human"
    },
    "polyA": {
      "name": "SV40-late",
      "sequence": "AATAAAAGATCT..."
    },
    "regulatoryElements": [
      {
        "name": "WPRE",
        "sequence": "AATGAAAGAC...",
        "purpose": "enhance expression"
      }
    ]
  },
  "ITRs": {
    "type": "AAV2",
    "sequence5prime": "CCTGCAGGCAGCTGCGCGCTCGCTCGCTCA...",
    "sequence3prime": "TGAGCGAGCGAGCGCGCAGCTGCCTGCAGG..."
  },
  "backbone": {
    "origin": "pUC",
    "selectionMarker": "AmpR"
  },
  "totalSize": 4235
}
```

**Response (201 Created):**
```json
{
  "vectorId": "VECTOR_001",
  "genBankFile": "https://api.wia-bio.org/gene-therapy/v1/vectors/VECTOR_001/genbank",
  "fastaFile": "https://api.wia-bio.org/gene-therapy/v1/vectors/VECTOR_001/fasta",
  "validationResults": {
    "sizeLimits": {
      "payloadSize": 2145,
      "maxSize": 4700,
      "status": "PASS"
    },
    "sequenceCheck": {
      "crypticSpliceSites": 0,
      "repeatElements": 0,
      "gcContent": 54.2,
      "status": "PASS"
    },
    "regulatoryCompliance": {
      "terminationSignals": "present",
      "kozakSequence": "optimal",
      "status": "PASS"
    }
  },
  "createdAt": "2025-01-15T10:00:00Z"
}
```

---

#### 2. Get Vector Details

**GET** `/v1/vectors/{vectorId}`

```json
{
  "vectorId": "VECTOR_001",
  "name": "AAV9-CMV-GFP",
  "version": "1.0",
  "status": "validated",
  "sequence": {
    "genBank": "https://api.wia-bio.org/.../genbank",
    "fasta": "https://api.wia-bio.org/.../fasta",
    "snapGene": "https://api.wia-bio.org/.../snapgene"
  },
  "annotations": [
    {
      "feature": "promoter",
      "name": "CMV-IE",
      "start": 612,
      "end": 1211,
      "strand": "+"
    },
    {
      "feature": "CDS",
      "name": "eGFP",
      "start": 1235,
      "end": 1954,
      "strand": "+",
      "translation": "MVSKGEELFTGVVP..."
    }
  ],
  "manufacturingBatches": [
    {
      "batchId": "BATCH_001",
      "status": "released",
      "titer": "5.2e13 vg/mL"
    }
  ]
}
```

---

#### 3. Validate Vector Design

**POST** `/v1/vectors/validate`

```json
{
  "sequence": "CCTGCAGGCAGCTGCGCGCTCGCTCGCTCACTGAGG...",
  "vectorType": "AAV",
  "serotype": "AAV9"
}
```

**Response:**
```json
{
  "valid": true,
  "checks": {
    "sizeLimit": {
      "payloadSize": 2145,
      "maxSize": 4700,
      "withinLimit": true
    },
    "crypticSpliceSites": {
      "count": 0,
      "status": "PASS"
    },
    "repeatElements": {
      "count": 0,
      "status": "PASS"
    },
    "kozakConsensus": {
      "sequence": "gccRccATGG",
      "optimal": true
    },
    "polyA_signal": {
      "present": true,
      "sequence": "AATAAA"
    },
    "restrictions": {
      "BamHI": 0,
      "EcoRI": 0,
      "XbaI": 1,
      "status": "WARNING - XbaI site present"
    }
  },
  "warnings": [
    "XbaI restriction site at position 2345 may interfere with cloning"
  ],
  "recommendations": [
    "Consider silent mutation to remove XbaI site",
    "GC content 54.2% is within optimal range (40-60%)"
  ]
}
```

---

## Manufacturing API

### Batch Tracking

#### 1. Create Manufacturing Batch

**POST** `/v1/manufacturing/batches`

```json
{
  "vectorId": "VECTOR_001",
  "productName": "AAV9-hSMN1",
  "method": "triple-transfection",
  "scale": "10-layer CellStack",
  "targetTiter": "5e13 vg/mL",
  "targetVolume": "50 mL",
  "gmpGrade": true,
  "scheduledStart": "2025-01-20T08:00:00Z"
}
```

**Response (201 Created):**
```json
{
  "batchId": "BATCH_002",
  "status": "scheduled",
  "estimatedCompletion": "2025-01-27",
  "trackingUrl": "https://api.wia-bio.org/gene-therapy/v1/manufacturing/batches/BATCH_002"
}
```

---

#### 2. Update Batch Status

**PATCH** `/v1/manufacturing/batches/{batchId}`

```json
{
  "status": "harvest",
  "harvestData": {
    "timestamp": "2025-01-23T16:00:00Z",
    "cellCount": "3.2e9",
    "viability": "92%",
    "volumeHarvested": "5.0 L",
    "crudeYield": "8.2e13 vg"
  }
}
```

---

#### 3. Submit QC Results

**POST** `/v1/manufacturing/batches/{batchId}/qc`

```json
{
  "testType": "titer",
  "method": "qPCR_ITR",
  "result": {
    "value": "5.2e13",
    "unit": "vg/mL"
  },
  "specification": {
    "min": "4.0e13",
    "max": "6.0e13"
  },
  "status": "PASS",
  "analyst": "ANALYST_001",
  "instrument": "QuantStudio7",
  "rawData": "https://lims.example.com/data/qPCR_20250120_001.eds"
}
```

**Response:**
```json
{
  "qcId": "QC_001",
  "batchId": "BATCH_002",
  "status": "PASS",
  "allTestsComplete": false,
  "remainingTests": ["infectivity", "purity", "sterility", "mycoplasma", "endotoxin", "RCR"]
}
```

---

#### 4. Get Batch Certificate of Analysis (COA)

**GET** `/v1/manufacturing/batches/{batchId}/coa`

**Response (PDF):**
```
Certificate of Analysis
Batch: AAV9-SMN-20250115-001
Product: AAV9-hSMN1

Test Results:
┌────────────────────┬──────────────┬──────────────┬────────┐
│ Test               │ Result       │ Specification│ Status │
├────────────────────┼──────────────┼──────────────┼────────┤
│ Titer (vg/mL)      │ 5.2×10¹³     │ 4.0-6.0×10¹³ │ PASS   │
│ Infectivity        │ 2.6×10¹²     │ vg:IU <100:1 │ PASS   │
│ Full Capsids (%)   │ 84%          │ >80%         │ PASS   │
│ Endotoxin (EU/mL)  │ 0.8          │ <5           │ PASS   │
│ Sterility          │ No growth    │ No growth    │ PASS   │
│ Mycoplasma         │ Negative     │ Negative     │ PASS   │
│ RCR                │ Not detected │ Not detected │ PASS   │
└────────────────────┴──────────────┴──────────────┴────────┘

Disposition: RELEASED
Approved by: QA_MANAGER_001
Date: 2025-01-23
```

---

## Clinical Monitoring API

### Patient Monitoring

#### 1. Record Gene Therapy Administration

**POST** `/v1/clinical/administrations`

```json
{
  "patientId": "PATIENT_001_DEIDENTIFIED",
  "studyId": "STUDY_AAV9_SMA_001",
  "vectorProduct": "AAV9-hSMN1",
  "batchLot": "BATCH_002",
  "administrationDate": "2025-01-25T10:00:00Z",
  "dose": {
    "value": "1.1e14",
    "unit": "vg/kg"
  },
  "patientWeight": "8.5 kg",
  "totalDose": "9.35e14 vg",
  "route": "intravenous",
  "infusionDuration": "60 minutes",
  "vitalSigns": {
    "preInfusion": {
      "BP": "95/60 mmHg",
      "HR": "120 bpm",
      "temp": "36.8 C",
      "O2Sat": "99%"
    },
    "postInfusion": {
      "BP": "98/62 mmHg",
      "HR": "115 bpm",
      "temp": "37.0 C",
      "O2Sat": "99%"
    }
  },
  "adverseEvents": []
}
```

---

#### 2. Submit Follow-Up Data

**POST** `/v1/clinical/followup`

```json
{
  "patientId": "PATIENT_001_DEIDENTIFIED",
  "visitType": "week4",
  "visitDate": "2025-02-22",
  "assessments": [
    {
      "type": "transgene_expression",
      "assay": "SMN_protein_ELISA",
      "result": {
        "value": 4.2,
        "unit": "ng/mL"
      },
      "referenceRange": {
        "low": 2.0,
        "high": 8.0
      },
      "interpretation": "Within therapeutic range"
    },
    {
      "type": "liver_function",
      "tests": {
        "ALT": {"value": 45, "unit": "U/L", "status": "NORMAL"},
        "AST": {"value": 38, "unit": "U/L", "status": "NORMAL"}
      }
    },
    {
      "type": "vector_shedding",
      "sample": "blood",
      "qPCR_result": {
        "vectorCopies": "2.3e3",
        "unit": "vg/mL"
      },
      "status": "Detectable, expected at this timepoint"
    }
  ],
  "motorMilestones": {
    "headControl": true,
    "sitting": false,
    "standing": false
  },
  "adverseEvents": []
}
```

---

#### 3. Report Adverse Event

**POST** `/v1/clinical/adverse-events`

```json
{
  "patientId": "PATIENT_001_DEIDENTIFIED",
  "studyId": "STUDY_AAV9_SMA_001",
  "eventType": "hepatotoxicity",
  "severity": "Grade 2",
  "onset": "2025-02-05T09:00:00Z",
  "description": "Elevated ALT (285 U/L) detected at Week 3 post-infusion",
  "labValues": {
    "ALT": {"value": 285, "unit": "U/L", "baseline": 32},
    "AST": {"value": 210, "unit": "U/L", "baseline": 28}
  },
  "causality": "Probably Related",
  "action": {
    "treatment": "Prednisone 1 mg/kg PO daily initiated",
    "doseModification": "N/A (one-time treatment)",
    "outcome": "Improving - ALT decreased to 145 U/L after 1 week of steroids"
  },
  "serious": false,
  "reportedToRegulator": false,
  "reportedToIRB": true
}
```

**Response:**
```json
{
  "aeId": "AE_001",
  "status": "reported",
  "regulatoryReportingRequired": false,
  "followUpSchedule": "Weekly LFTs for 4 weeks",
  "createdAt": "2025-02-05T10:30:00Z"
}
```

---

## Regulatory Submission Integration

### eCTD Module Generation

#### 1. Generate Module 3.2.S (Drug Substance)

**POST** `/v1/regulatory/ectd/module-3.2.S`

```json
{
  "vectorId": "VECTOR_001",
  "batchIds": ["BATCH_001", "BATCH_002", "BATCH_003"],
  "sections": [
    "3.2.S.1-general-information",
    "3.2.S.2-manufacture",
    "3.2.S.3-characterization",
    "3.2.S.4-control",
    "3.2.S.5-reference-standards",
    "3.2.S.6-container-closure",
    "3.2.S.7-stability"
  ],
  "confidentialityLevel": "CCI",
  "submissionType": "IND"
}
```

**Response:**
```json
{
  "ectdPackage": "https://api.wia-bio.org/regulatory/downloads/eCTD_Module3.2.S_VECTOR_001.zip",
  "contents": [
    "3.2.S.1/general-information.pdf",
    "3.2.S.2/manufacture-flowchart.pdf",
    "3.2.S.2.3/manufacturing-process-description.pdf",
    "3.2.S.4.1/analytical-methods.pdf",
    "3.2.S.4.4/batch-analysis-data.pdf",
    "3.2.S.7/stability-protocol.pdf"
  ],
  "xmlIndex": "module-3.2.S.xml",
  "validated": true,
  "eCTD_version": "4.0"
}
```

---

#### 2. Submit to FDA Gateway

**POST** `/v1/regulatory/fda/submit`

```json
{
  "submissionType": "IND",
  "indNumber": "IND_012345",
  "ectdPackage": "eCTD_Module3.2.S_VECTOR_001.zip",
  "transmittalLetter": "Dear FDA, please find attached Module 3.2.S for IND 012345...",
  "contacts": {
    "sponsor": "BioTherapeutics Inc.",
    "regulatoryContact": "regulatory@biotherapeutics.com"
  }
}
```

**Response:**
```json
{
  "submissionId": "FDA_SUB_001",
  "status": "transmitted",
  "confirmationNumber": "1234567890",
  "esgURL": "https://esg.fda.gov/submission/1234567890",
  "acknowledgmentExpected": "2025-01-30",
  "reviewClock": {
    "starts": "2025-01-26",
    "30dayDeadline": "2025-02-25"
  }
}
```

---

## Interoperability Standards

### HL7 FHIR Integration

**MedicationKnowledge (Vector Product Definition):**

```json
{
  "resourceType": "MedicationKnowledge",
  "id": "AAV9-SMN",
  "code": {
    "coding": [
      {
        "system": "http://wia-bio.org/gene-therapy",
        "code": "AAV9-hSMN1",
        "display": "AAV9-hSMN1 Gene Therapy"
      }
    ]
  },
  "status": "active",
  "manufacturer": {
    "reference": "Organization/BIOTHERAPEUTICS_INC"
  },
  "doseForm": {
    "coding": [
      {
        "system": "http://snomed.info/sct",
        "code": "385219001",
        "display": "Solution for injection"
      }
    ]
  },
  "amount": {
    "value": 5.5,
    "unit": "mL",
    "system": "http://unitsofmeasure.org",
    "code": "mL"
  },
  "ingredient": [
    {
      "itemCodeableConcept": {
        "text": "AAV9 vector containing hSMN1 transgene"
      },
      "strength": {
        "numerator": {
          "value": 2e13,
          "unit": "vg/mL"
        }
      }
    }
  ],
  "administrationGuidelines": [
    {
      "dosage": [
        {
          "type": {
            "text": "Weight-based dosing"
          },
          "doseAndRate": [
            {
              "doseQuantity": {
                "value": 1.1e14,
                "unit": "vg/kg"
              },
              "rateQuantity": {
                "value": 60,
                "unit": "minutes"
              }
            }
          ]
        }
      ]
    }
  ],
  "packaging": {
    "type": {
      "coding": [
        {
          "system": "http://snomed.info/sct",
          "code": "415818006",
          "display": "Vial"
        }
      ]
    },
    "quantity": {
      "value": 1
    }
  },
  "regulatoryStatus": [
    {
      "identifier": {
        "system": "http://hl7.org/fhir/sid/fda-nda",
        "value": "BLA_125694"
      },
      "status": {
        "coding": [
          {
            "system": "http://hl7.org/fhir/medicationknowledge-status",
            "code": "active",
            "display": "Active"
          }
        ]
      }
    }
  ]
}
```

---

### SDK & Client Libraries

#### Python SDK

```python
from wia_bio import GeneTherapyClient

client = GeneTherapyClient(api_key='your_api_key')

# Create vector design
vector = client.vectors.create(
    name='AAV9-CMV-GFP',
    vector_type='AAV',
    serotype='AAV9',
    promoter='CMV-IE',
    transgene='eGFP',
    poly_a='SV40-late'
)
print(f"Vector created: {vector.id}")

# Validate design
validation = client.vectors.validate(vector.id)
if validation.valid:
    print("Vector design passed all checks")
else:
    print(f"Warnings: {validation.warnings}")

# Start manufacturing batch
batch = client.manufacturing.create_batch(
    vector_id=vector.id,
    method='triple-transfection',
    target_titer=5e13,
    gmp_grade=True
)
print(f"Batch {batch.id} scheduled for {batch.scheduled_start}")

# Monitor batch progress
status = client.manufacturing.get_batch_status(batch.id)
print(f"Current status: {status.stage} - {status.percent_complete}% complete")

# Submit QC results
qc_result = client.manufacturing.submit_qc(
    batch_id=batch.id,
    test_type='titer',
    result=5.2e13,
    method='qPCR_ITR'
)

# Record clinical administration
admin = client.clinical.record_administration(
    patient_id='PATIENT_001_DEIDENTIFIED',
    batch_lot=batch.lot_number,
    dose=1.1e14,
    dose_unit='vg/kg',
    patient_weight=8.5
)
print(f"Administration recorded: {admin.id}")
```

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA BIO Working Group
**License:** Apache 2.0
