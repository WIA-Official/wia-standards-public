# WIA-MED-015 API Specification

## FHIR Resources

### Required Resources

- `Patient` - Patient demographics
- `Observation` - Vital signs, lab results
- `MedicationRequest` - Prescriptions
- `Condition` - Diagnoses
- `AllergyIntolerance` - Allergy history

### CDS Hooks

#### medication-prescribe

Triggered when prescribing medication.

**Request:**
```json
{
  "hook": "medication-prescribe",
  "context": {
    "patientId": "12345",
    "medications": [...]
  }
}
```

**Response:**
```json
{
  "cards": [
    {
      "summary": "Drug Interaction Warning",
      "indicator": "warning",
      "source": {
        "label": "DDI Database"
      },
      "suggestions": [
        {"label": "Alternative medication"}
      ]
    }
  ]
}
```

## Alert Severity Levels

- `critical` - Block action, requires override
- `warning` - Requires acknowledgment
- `info` - Passive notification

## Performance Requirements

- Response time: < 1 second
- Availability: 99.9%
- Alert acceptance rate: > 30%
- Alert override rate: < 50%

---

© 2025 WIA
