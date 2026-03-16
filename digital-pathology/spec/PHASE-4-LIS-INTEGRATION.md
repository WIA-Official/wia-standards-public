# WIA-MED-008 Phase 4: LIS Integration Specification

## Version 1.0.0

**Status:** ✅ Complete  
**Last Updated:** 2025-01-15

---

## 1. HL7 v2.x Integration

### 1.1 Order Message (ORM^O01)

```
MSH|^~\&|EMR|HOSPITAL|LIS|PATHLAB|20250115090000||ORM^O01|MSG001|P|2.5
PID|1||P123456||DOE^JANE||19800515|F
ORC|NW|ORDER123||||||20250115090000
OBR|1|ORDER123||88305^SURGICAL PATHOLOGY|||20250115085500
OBX|1|TX|CLINICAL||Right breast mass, r/o cancer
```

### 1.2 Result Message (ORU^R01)

```
MSH|^~\&|LIS|PATHLAB|EMR|HOSPITAL|20250115152600||ORU^R01|MSG002|P|2.5
PID|1||P123456||DOE^JANE||19800515|F
OBR|1|ORDER123|S25-00123|88305|||20250115152000|||||||F
OBX|1|TX|DIAGNOSIS||Invasive ductal carcinoma, Grade II
OBX|2|TX|AI_RESULT||Cancer detected (96% confidence)
```

---

## 2. FHIR Integration

### 2.1 DiagnosticReport Resource

```json
{
  "resourceType": "DiagnosticReport",
  "id": "REPORT-S25-00123",
  "status": "final",
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "60568-3",
      "display": "Pathology Synoptic report"
    }]
  },
  "subject": {"reference": "Patient/P123456"},
  "effectiveDateTime": "2025-01-15T15:20:00Z",
  "conclusion": "Invasive ductal carcinoma, Grade II"
}
```

### 2.2 Observation Resource (AI Results)

```json
{
  "resourceType": "Observation",
  "id": "AI-RESULT-001",
  "status": "final",
  "code": {
    "text": "AI Cancer Detection"
  },
  "subject": {"reference": "Patient/P123456"},
  "valueCodeableConcept": {
    "coding": [{
      "system": "http://snomed.info/sct",
      "code": "408643008",
      "display": "Infiltrating duct carcinoma"
    }]
  },
  "note": [{
    "text": "AI Model: BreastCancerDetector v2.1, Confidence: 96%"
  }]
}
```

---

## 3. Workflow Management

### 3.1 Status Updates

```http
POST /api/v1/workflow/status

Body: {
  "accession_number": "S25-00123",
  "status": "scanned|ai_analyzed|pathologist_reviewed|signed",
  "timestamp": "2025-01-15T14:12:00Z",
  "metadata": { additional info }
}
```

### 3.2 Worklist API

```http
# Get pathologist worklist
GET /api/v1/worklist?pathologist_id={id}&status=pending

Response: {
  "items": [
    {
      "accession_number": "S25-00123",
      "priority": "ROUTINE",
      "ai_flag": "cancer_detected",
      "ai_confidence": 0.96
    }
  ]
}
```

---

## 4. Security & Privacy

### 4.1 HIPAA Compliance

- **Encryption:** TLS 1.3 (transport), AES-256 (storage)
- **Access Control:** RBAC with MFA
- **Audit Logging:** All access logged, 6-year retention
- **Data Integrity:** SHA-256 checksums, digital signatures

### 4.2 De-identification

```json
{
  "study_id": "STUDY-001-0042",
  "age_at_diagnosis": 44,
  "diagnosis": "invasive_ductal_carcinoma",
  "removed_fields": [
    "patient_name",
    "patient_id",
    "mrn",
    "dob",
    "exact_dates"
  ]
}
```

---

**© 2025 WIA**  
**License:** MIT License
