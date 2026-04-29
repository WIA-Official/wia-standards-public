# WIA-GENOME_EDITING: Phase 2 - API Interface Specification

**Version:** 1.0
**Status:** FULL Implementation
**Last Updated:** 2026-01-12

---

## 1. Overview

This specification defines RESTful API interfaces for genome editing operations, including CRISPR-Cas9 design, base editing, prime editing, validation, and clinical data management.

### 1.1 Base URL

```
Production: https://api.wia-genome-editing.org/v1
Staging: https://staging-api.wia-genome-editing.org/v1
Development: https://dev-api.wia-genome-editing.org/v1
```

### 1.2 Authentication

All API requests require authentication using OAuth 2.0 Bearer tokens:

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 1.3 Rate Limiting

- Standard tier: 1000 requests/hour
- Professional tier: 10000 requests/hour
- Enterprise tier: Unlimited

Headers:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1736683200
```

---

## 2. Sequence Management API

### 2.1 Upload Sequence

**Endpoint:** `POST /sequences`

**Request:**
```json
{
  "organism": "Homo sapiens",
  "chromosome": "11",
  "start_position": 5227002,
  "end_position": 5229395,
  "strand": "+",
  "sequence_data": "ATGGTGCATCTGACTCCTGAGGAGAAGTCTGCCGTTACTGCCCTGTGGGGCAAGGTGAACGTGGATGAAGTTGGTGGTGAGGCCCTGGGCAG...",
  "gene_symbol": "HBB",
  "reference_genome": "GRCh38.p14",
  "metadata": {
    "patient_id": "PT-SCD-001",
    "condition": "sickle_cell_disease",
    "notes": "Target for therapeutic editing"
  }
}
```

**Response:** `201 Created`
```json
{
  "sequence_id": "SEQ-2026-001",
  "status": "validated",
  "quality_score": 99.8,
  "coverage_depth": 150,
  "created_at": "2026-01-12T10:30:00Z",
  "analysis_id": "ANALYSIS-001",
  "links": {
    "self": "/sequences/SEQ-2026-001",
    "analysis": "/sequences/SEQ-2026-001/analysis",
    "edits": "/sequences/SEQ-2026-001/edits"
  }
}
```

### 2.2 Get Sequence

**Endpoint:** `GET /sequences/{sequence_id}`

**Response:** `200 OK`
```json
{
  "sequence_id": "SEQ-2026-001",
  "type": "genomic_dna",
  "organism": "Homo sapiens",
  "chromosome": "11",
  "start_position": 5227002,
  "end_position": 5229395,
  "strand": "+",
  "sequence_data": "ATGGTGCATCTGACTCCTGAGGAGAAGTCTGCCGTTACTGCCCTGTGGGGCAAGGTGAACGTGGATGAAGTTGGTGGTGAGGCCCTGGGCAG...",
  "gene_symbol": "HBB",
  "gene_name": "hemoglobin subunit beta",
  "reference_genome": "GRCh38.p14",
  "quality_score": 99.8,
  "annotations": {
    "exons": [
      {"number": 1, "start": 5225464, "end": 5225726},
      {"number": 2, "start": 5226929, "end": 5227071},
      {"number": 3, "start": 5227234, "end": 5229395}
    ],
    "pathogenic_variants": [
      {
        "position": 5227021,
        "ref": "A",
        "alt": "T",
        "variant": "HbS (E6V)",
        "consequence": "sickle_cell_disease"
      }
    ]
  }
}
```

### 2.3 Analyze Sequence

**Endpoint:** `POST /sequences/{sequence_id}/analyze`

**Request:**
```json
{
  "analysis_types": [
    "variant_detection",
    "mutation_analysis",
    "conservation_score",
    "regulatory_elements"
  ],
  "reference_genome": "GRCh38.p14"
}
```

**Response:** `200 OK`
```json
{
  "analysis_id": "ANALYSIS-001",
  "sequence_id": "SEQ-2026-001",
  "variants": [
    {
      "position": 5227021,
      "ref": "A",
      "alt": "T",
      "type": "SNV",
      "consequence": "missense",
      "amino_acid_change": "E6V",
      "clinical_significance": "pathogenic",
      "disease": "sickle_cell_disease",
      "allele_frequency": 0.02
    }
  ],
  "conservation_scores": {
    "position_5227021": {
      "phyloP": 2.85,
      "phastCons": 0.98,
      "gerp": 4.12
    }
  },
  "regulatory_elements": [
    {
      "type": "enhancer",
      "position": 5226500,
      "activity": "erythroid_specific",
      "source": "ENCODE"
    }
  ]
}
```

---

## 3. CRISPR-Cas9 Design API

### 3.1 Design Guide RNAs

**Endpoint:** `POST /crispr/design`

**Request:**
```json
{
  "sequence_id": "SEQ-2026-001",
  "target_region": {
    "start": 5227015,
    "end": 5227030
  },
  "cas_enzyme": "SpCas9",
  "pam_sequence": "NGG",
  "design_parameters": {
    "min_specificity": 95.0,
    "min_on_target_score": 90.0,
    "max_off_targets": 5,
    "avoid_polymorphisms": true
  }
}
```

**Response:** `200 OK`
```json
{
  "design_id": "DESIGN-001",
  "guide_rnas": [
    {
      "id": "gRNA-HBB-001",
      "sequence": "GTGCACCTGACTCCTGAGGAGAA",
      "pam_sequence": "AGG",
      "genomic_position": 5227020,
      "strand": "+",
      "design_score": 95.2,
      "specificity_score": 98.5,
      "on_target_score": 97.3,
      "off_target_count": 3,
      "predicted_efficiency": 89.5,
      "recommendation": "highly_recommended"
    },
    {
      "id": "gRNA-HBB-002",
      "sequence": "CACCTGACTCCTGAGGAGAAGAA",
      "pam_sequence": "TGG",
      "genomic_position": 5227022,
      "strand": "+",
      "design_score": 88.7,
      "specificity_score": 92.1,
      "on_target_score": 94.2,
      "off_target_count": 8,
      "predicted_efficiency": 82.3,
      "recommendation": "acceptable"
    }
  ],
  "created_at": "2026-01-12T11:00:00Z"
}
```

### 3.2 Predict Off-Targets

**Endpoint:** `POST /crispr/off-targets`

**Request:**
```json
{
  "guide_rna_sequence": "GTGCACCTGACTCCTGAGGAGAA",
  "pam_sequence": "AGG",
  "cas_enzyme": "SpCas9",
  "reference_genome": "GRCh38.p14",
  "max_mismatches": 4,
  "analysis_method": "cas-offinder"
}
```

**Response:** `200 OK`
```json
{
  "analysis_id": "OT-001",
  "guide_rna": "GTGCACCTGACTCCTGAGGAGAA",
  "total_sites_analyzed": 23500000000,
  "off_target_sites": [
    {
      "chromosome": "15",
      "position": 8234501,
      "sequence": "GTGCACCTGACTGCTGAGGAGAA",
      "mismatches": 1,
      "mismatch_positions": [12],
      "bulges": 0,
      "score": 12.4,
      "in_gene": false,
      "risk_level": "low"
    },
    {
      "chromosome": "7",
      "position": 15678234,
      "sequence": "GTGCACCTGAGTCCTGAGGAGAA",
      "mismatches": 1,
      "mismatch_positions": [10],
      "bulges": 0,
      "score": 8.7,
      "in_gene": true,
      "gene": "ENSG00000123456",
      "consequence": "intronic",
      "risk_level": "low"
    }
  ],
  "summary": {
    "high_risk": 0,
    "medium_risk": 2,
    "low_risk": 14,
    "safety_score": 96.8
  }
}
```

---

## 4. Base Editing API

### 4.1 Design Base Edit

**Endpoint:** `POST /base-editing/design`

**Request:**
```json
{
  "sequence_id": "SEQ-2026-001",
  "edit_type": "adenine_base_editor",
  "editor_variant": "ABE8.20-m",
  "target_position": 5227021,
  "target_base": "A",
  "desired_base": "G",
  "design_parameters": {
    "minimize_bystander": true,
    "optimize_window": true,
    "max_off_targets": 10
  }
}
```

**Response:** `200 OK`
```json
{
  "design_id": "BE-DESIGN-001",
  "edit_id": "BE-HBB-E6V-001",
  "editor": "ABE8.20-m",
  "guide_rna": "GTGCACCTGACTCCTGAGGAGAA",
  "pam_sequence": "AGG",
  "editing_window": {
    "start": 4,
    "end": 8,
    "target_position": 6,
    "positions": [4, 5, 6, 7, 8]
  },
  "predicted_outcomes": {
    "on_target_efficiency": 75.3,
    "target_edit": {
      "position": 5227021,
      "change": "A>G",
      "frequency": 68.5
    },
    "bystander_edits": [
      {
        "position": 5227018,
        "change": "A>G",
        "frequency": 5.2,
        "effect": "silent_mutation"
      }
    ],
    "product_purity": 92.3
  },
  "off_target_analysis": {
    "high_risk": 0,
    "medium_risk": 2,
    "low_risk": 15,
    "safety_score": 94.5
  },
  "recommendation": "approved_for_testing"
}
```

### 4.2 Simulate Base Edit

**Endpoint:** `POST /base-editing/simulate`

**Request:**
```json
{
  "edit_id": "BE-HBB-E6V-001",
  "simulation_parameters": {
    "cell_type": "CD34_hematopoietic_stem_cell",
    "delivery_method": "electroporation",
    "editor_concentration": 50,
    "incubation_time": 48
  }
}
```

**Response:** `200 OK`
```json
{
  "simulation_id": "SIM-001",
  "predicted_outcomes": {
    "editing_efficiency": 68.5,
    "allele_distribution": {
      "wildtype": 24.8,
      "edited": 68.5,
      "indels": 2.1,
      "other": 4.6
    },
    "bystander_frequency": 5.2,
    "cell_viability": 94.5,
    "time_to_peak_editing": 36
  },
  "confidence_interval": {
    "efficiency_lower": 62.3,
    "efficiency_upper": 74.7,
    "confidence_level": 95
  }
}
```

---

## 5. Prime Editing API

### 5.1 Design Prime Edit

**Endpoint:** `POST /prime-editing/design`

**Request:**
```json
{
  "sequence_id": "SEQ-2026-001",
  "edit_type": "substitution",
  "target_position": 5227021,
  "original_sequence": "GAG",
  "replacement_sequence": "GTG",
  "editor_system": "PE5",
  "design_parameters": {
    "pbs_length_range": [10, 15],
    "rt_template_length_range": [10, 20],
    "optimize_efficiency": true,
    "minimize_indels": true
  }
}
```

**Response:** `200 OK`
```json
{
  "design_id": "PE-DESIGN-001",
  "edit_id": "PE-HBB-001",
  "editor_system": "PE5",
  "pegRNA": {
    "spacer": "GTGCACCTGACTCCTGAGGA",
    "pbs_length": 13,
    "pbs_sequence": "TCCTCAGGAGTCA",
    "rt_template_length": 16,
    "rt_template": "GTGCACCTGACTCCTG",
    "edit_position": 5,
    "replacement": "GTG",
    "full_sequence": "GTGCACCTGACTCCTGAGGA...TCCTCAGGAGTCA...GTGCACCTGACTCCTG"
  },
  "ngRNA": {
    "sequence": "CTGGGCAGGTTGGTATCAAG",
    "nick_distance": 65,
    "nick_strand": "non_edited",
    "optimal": true
  },
  "predicted_performance": {
    "efficiency": 52.8,
    "precision": 95.6,
    "install_rate": 48.5,
    "indel_frequency": 2.1,
    "byproduct_rate": 4.2
  },
  "recommendation": "proceed_to_validation"
}
```

### 5.2 Optimize Prime Edit

**Endpoint:** `POST /prime-editing/{edit_id}/optimize`

**Request:**
```json
{
  "optimization_targets": [
    "maximize_efficiency",
    "minimize_indels",
    "balance_efficiency_precision"
  ],
  "constraints": {
    "min_efficiency": 40.0,
    "max_indel_rate": 5.0,
    "min_precision": 90.0
  }
}
```

**Response:** `200 OK`
```json
{
  "optimization_id": "OPT-001",
  "original_design": {
    "efficiency": 52.8,
    "precision": 95.6,
    "indel_rate": 2.1
  },
  "optimized_designs": [
    {
      "variant": "OPT-A",
      "pbs_length": 14,
      "rt_template_length": 18,
      "efficiency": 58.3,
      "precision": 96.2,
      "indel_rate": 1.8,
      "improvement_score": 12.5
    },
    {
      "variant": "OPT-B",
      "pbs_length": 12,
      "rt_template_length": 15,
      "efficiency": 55.1,
      "precision": 97.1,
      "indel_rate": 1.5,
      "improvement_score": 10.8
    }
  ],
  "recommendation": "OPT-A"
}
```

---

## 6. Validation and Quality Control API

### 6.1 Submit for Validation

**Endpoint:** `POST /validation/submit`

**Request:**
```json
{
  "edit_id": "BE-HBB-E6V-001",
  "validation_type": "next_generation_sequencing",
  "sequencing_data": {
    "platform": "Illumina NovaSeq",
    "read_length": 150,
    "paired_end": true,
    "target_depth": 50000,
    "fastq_r1": "s3://bucket/sample_R1.fastq.gz",
    "fastq_r2": "s3://bucket/sample_R2.fastq.gz"
  },
  "analysis_parameters": {
    "variant_caller": "deepvariant",
    "min_base_quality": 30,
    "min_mapping_quality": 60
  }
}
```

**Response:** `202 Accepted`
```json
{
  "validation_id": "VAL-001",
  "status": "processing",
  "estimated_completion": "2026-01-12T14:00:00Z",
  "job_id": "JOB-NGS-001",
  "links": {
    "status": "/validation/VAL-001/status",
    "results": "/validation/VAL-001/results"
  }
}
```

### 6.2 Get Validation Results

**Endpoint:** `GET /validation/{validation_id}/results`

**Response:** `200 OK`
```json
{
  "validation_id": "VAL-001",
  "edit_id": "BE-HBB-E6V-001",
  "status": "completed",
  "completed_at": "2026-01-12T13:45:00Z",
  "results": {
    "on_target_efficiency": 68.5,
    "precision": 95.2,
    "sequencing_depth": 52340,
    "allele_frequencies": {
      "wildtype": 24.8,
      "edited": 68.5,
      "indels": 2.1,
      "other": 4.6
    },
    "indel_spectrum": {
      "1bp_deletion": 1.2,
      "1bp_insertion": 0.5,
      "2bp_deletion": 0.3,
      "other": 0.1
    },
    "product_purity": 92.3,
    "bystander_editing": 5.2,
    "off_target_edits": 0.3
  },
  "quality_metrics": {
    "sequence_quality": "pass",
    "coverage_uniformity": 98.5,
    "strand_bias": 0.52,
    "mapping_quality": 98.7
  },
  "clinical_recommendation": {
    "safety": "acceptable",
    "efficacy": "high",
    "approval": "recommended"
  }
}
```

---

## 7. Gene Therapy Clinical API

### 7.1 Register Patient

**Endpoint:** `POST /clinical/patients`

**Request:**
```json
{
  "demographics": {
    "age": 28,
    "sex": "female",
    "weight_kg": 65.5,
    "ethnicity": "African_American"
  },
  "diagnosis": {
    "condition": "sickle_cell_disease",
    "genotype": "HbSS",
    "severity": "severe",
    "disease_duration_years": 26
  },
  "baseline_data": {
    "vaso_occlusive_crises_per_year": 8,
    "hospitalizations_per_year": 4,
    "baseline_hgb": 7.2,
    "hgb_f_percent": 2.1,
    "transfusions_lifetime": 45
  },
  "consent": {
    "informed_consent": true,
    "consent_date": "2025-12-15",
    "protocol_version": "CASGEVY-v2.0"
  }
}
```

**Response:** `201 Created`
```json
{
  "patient_id": "PT-SCD-001",
  "status": "registered",
  "eligibility": "confirmed",
  "enrolled_date": "2026-01-05",
  "protocol": "CASGEVY-v2.0",
  "links": {
    "patient": "/clinical/patients/PT-SCD-001",
    "treatment": "/clinical/patients/PT-SCD-001/treatment",
    "outcomes": "/clinical/patients/PT-SCD-001/outcomes"
  }
}
```

### 7.2 Record Treatment

**Endpoint:** `POST /clinical/patients/{patient_id}/treatment`

**Request:**
```json
{
  "therapy_id": "GT-CASGEVY-001",
  "therapy_name": "exagamglogene_autotemcel",
  "treatment_date": "2026-01-08",
  "manufacturing": {
    "cell_source": "autologous_CD34+_HSPCs",
    "cell_dose": 8.5e6,
    "viability": 97.2,
    "editing_efficiency": 78.5,
    "lot_number": "CASGEVY-2026-001"
  },
  "protocol": {
    "mobilization": {
      "agent": "plerixafor",
      "dose": "0.24_mg/kg",
      "duration_days": 5
    },
    "conditioning": {
      "agent": "busulfan",
      "dose": "targeted_AUC",
      "duration_days": 4
    }
  }
}
```

**Response:** `201 Created`
```json
{
  "treatment_id": "TX-001",
  "patient_id": "PT-SCD-001",
  "status": "administered",
  "treatment_date": "2026-01-08",
  "follow_up_schedule": [
    {"day": 7, "assessment": "engraftment_monitoring"},
    {"day": 14, "assessment": "engraftment_monitoring"},
    {"day": 30, "assessment": "efficacy_safety"},
    {"day": 90, "assessment": "efficacy_safety"},
    {"day": 180, "assessment": "efficacy_safety"},
    {"day": 365, "assessment": "long_term_efficacy"}
  ],
  "links": {
    "outcomes": "/clinical/patients/PT-SCD-001/outcomes",
    "adverse_events": "/clinical/patients/PT-SCD-001/adverse-events"
  }
}
```

### 7.3 Record Outcomes

**Endpoint:** `POST /clinical/patients/{patient_id}/outcomes`

**Request:**
```json
{
  "follow_up_day": 180,
  "assessment_date": "2026-07-07",
  "hematology": {
    "total_hgb": 12.3,
    "hgb_f_percent": 42.5,
    "wbc": 6.5,
    "platelets": 245,
    "neutrophils": 4.2
  },
  "clinical_status": {
    "vaso_occlusive_crises": 0,
    "hospitalizations": 0,
    "transfusions": 0,
    "transfusion_independence": true
  },
  "quality_of_life": {
    "score": 85.2,
    "domain_scores": {
      "physical": 88,
      "emotional": 82,
      "social": 86,
      "functional": 84
    }
  },
  "adverse_events": []
}
```

**Response:** `201 Created`
```json
{
  "outcome_id": "OUT-001",
  "patient_id": "PT-SCD-001",
  "follow_up_day": 180,
  "treatment_success": true,
  "transfusion_free_days": 180,
  "crisis_free": true,
  "quality_of_life_improvement": 125.3,
  "overall_assessment": "excellent_response",
  "continue_follow_up": true
}
```

---

## 8. Safety and Reporting API

### 8.1 Report Adverse Event

**Endpoint:** `POST /clinical/adverse-events`

**Request:**
```json
{
  "patient_id": "PT-SCD-001",
  "treatment_id": "TX-001",
  "event": {
    "term": "febrile_neutropenia",
    "grade": 3,
    "onset_date": "2026-01-16",
    "severity": "severe",
    "attribution": "definite",
    "expected": true
  },
  "management": {
    "intervention": "broad_spectrum_antibiotics",
    "hospitalization": true,
    "resolution_date": "2026-01-23",
    "outcome": "resolved"
  }
}
```

**Response:** `201 Created`
```json
{
  "adverse_event_id": "AE-001",
  "status": "reported",
  "regulatory_reporting": {
    "fda_required": false,
    "ema_required": false,
    "reported_to_irb": true
  },
  "follow_up_required": false
}
```

### 8.2 Generate Safety Report

**Endpoint:** `GET /clinical/safety-report`

**Query Parameters:**
```
start_date=2026-01-01&end_date=2026-12-31&therapy=CASGEVY
```

**Response:** `200 OK`
```json
{
  "report_id": "SAFETY-2026-Q1",
  "period": {
    "start": "2026-01-01",
    "end": "2026-12-31"
  },
  "therapy": "CASGEVY",
  "summary": {
    "total_patients": 125,
    "total_treatments": 125,
    "total_adverse_events": 387,
    "serious_adverse_events": 42,
    "deaths": 0
  },
  "adverse_events_by_grade": {
    "grade_1_2": 298,
    "grade_3": 78,
    "grade_4": 11,
    "grade_5": 0
  },
  "common_events": [
    {"term": "mucositis", "count": 98, "percent": 78.4},
    {"term": "febrile_neutropenia", "count": 45, "percent": 36.0},
    {"term": "nausea", "count": 72, "percent": 57.6}
  ],
  "efficacy_summary": {
    "transfusion_independence": 92.8,
    "crisis_free": 95.2,
    "engraftment_success": 99.2
  }
}
```

---

## 9. Error Handling

### 9.1 Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_FAILED",
    "message": "Sequence validation failed: insufficient quality score",
    "details": {
      "field": "quality_score",
      "value": 85.2,
      "required": 90.0
    },
    "request_id": "req-abc123",
    "timestamp": "2026-01-12T12:00:00Z"
  }
}
```

### 9.2 Common Error Codes

| Code | Status | Description |
|------|--------|-------------|
| INVALID_REQUEST | 400 | Malformed request |
| UNAUTHORIZED | 401 | Invalid authentication |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| CONFLICT | 409 | Resource conflict |
| VALIDATION_FAILED | 422 | Data validation error |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Server error |
| SERVICE_UNAVAILABLE | 503 | Service temporarily unavailable |

---

## 10. Webhook Notifications

### 10.1 Register Webhook

**Endpoint:** `POST /webhooks`

**Request:**
```json
{
  "url": "https://your-app.com/webhooks/genome-editing",
  "events": [
    "validation.completed",
    "analysis.completed",
    "treatment.administered",
    "adverse_event.reported"
  ],
  "secret": "whsec_abc123xyz"
}
```

**Response:** `201 Created`
```json
{
  "webhook_id": "WH-001",
  "url": "https://your-app.com/webhooks/genome-editing",
  "events": ["validation.completed", "analysis.completed"],
  "status": "active",
  "created_at": "2026-01-12T12:00:00Z"
}
```

### 10.2 Webhook Payload

```json
{
  "event": "validation.completed",
  "timestamp": "2026-01-12T13:45:00Z",
  "data": {
    "validation_id": "VAL-001",
    "edit_id": "BE-HBB-E6V-001",
    "status": "completed",
    "on_target_efficiency": 68.5,
    "precision": 95.2,
    "recommendation": "approved"
  },
  "signature": "sha256=abc123..."
}
```

---

## 11. SDK Support

### 11.1 Available SDKs

- **TypeScript/JavaScript**: `@wia/genome-editing-sdk`
- **Python**: `wia-genome-editing`
- **R**: `WIAGenomeEditing`
- **Java**: `com.wia.genome-editing`

### 11.2 Example Usage (TypeScript)

```typescript
import { GenomeEditingClient } from '@wia/genome-editing-sdk';

const client = new GenomeEditingClient({
  apiKey: process.env.WIA_API_KEY
});

// Design base edit
const design = await client.baseEditing.design({
  sequenceId: 'SEQ-2026-001',
  editType: 'adenine_base_editor',
  targetPosition: 5227021,
  targetBase: 'A',
  desiredBase: 'G'
});

console.log(`Predicted efficiency: ${design.predictedOutcomes.onTargetEfficiency}%`);
```

---

## 12. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-12 | Initial FULL API specification |

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (홍익인간) - Benefit All Humanity
