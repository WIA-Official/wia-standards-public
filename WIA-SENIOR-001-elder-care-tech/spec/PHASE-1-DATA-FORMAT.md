# WIA-SENIOR-001: Elder Care Technology Standard
## PHASE 1: DATA FORMAT SPECIFICATION

> 弘益人間 (Benefit All Humanity)

**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-01-15

---

## 1. Overview

This document defines the standardized data formats for elder care technology systems, ensuring interoperability, consistency, and compliance with healthcare regulations.

## 2. Core Data Structures

### 2.1 Elder Profile

```json
{
  "id": "string (UUID)",
  "personalInfo": {
    "firstName": "string",
    "lastName": "string",
    "dateOfBirth": "ISO 8601 date",
    "gender": "male|female|other|prefer-not-to-say",
    "language": "string (ISO 639-1)",
    "photoUrl": "string (URL, optional)",
    "address": {
      "street": "string",
      "city": "string",
      "state": "string",
      "zipCode": "string",
      "country": "string (ISO 3166-1)"
    }
  },
  "medicalInfo": {
    "bloodType": "string (optional)",
    "allergies": ["string"],
    "chronicConditions": ["string"],
    "medications": [...],
    "primaryPhysician": {...},
    "insuranceInfo": {...}
  },
  "carePreferences": {
    "mobilityLevel": "independent|needs-assistance|walker|wheelchair|bedridden",
    "cognitiveLevel": "normal|mild-impairment|moderate-impairment|severe-impairment|dementia",
    "communicationPreferences": [...],
    "activityPreferences": ["string"],
    "dietaryRestrictions": ["string"],
    "religiousCulturalPreferences": ["string"]
  },
  "emergencyContacts": [...],
  "createdAt": "ISO 8601 timestamp",
  "updatedAt": "ISO 8601 timestamp"
}
```

### 2.2 Vital Signs

```json
{
  "timestamp": "ISO 8601 timestamp",
  "elderId": "string (UUID)",
  "heartRate": {
    "bpm": "number (30-200)",
    "rhythm": "regular|irregular",
    "isAbnormal": "boolean"
  },
  "bloodPressure": {
    "systolic": "number (60-300)",
    "diastolic": "number (40-200)",
    "unit": "mmHg",
    "isAbnormal": "boolean"
  },
  "temperature": {
    "value": "number",
    "unit": "celsius|fahrenheit",
    "location": "oral|temporal|axillary|tympanic",
    "isAbnormal": "boolean"
  },
  "oxygenSaturation": {
    "percentage": "number (0-100)",
    "isAbnormal": "boolean"
  },
  "respiratoryRate": {
    "breathsPerMinute": "number (8-40)",
    "isAbnormal": "boolean"
  },
  "weight": {
    "value": "number",
    "unit": "kg|lbs"
  },
  "bloodGlucose": {
    "value": "number",
    "unit": "mg/dL|mmol/L",
    "measurementContext": "fasting|post-meal|random",
    "isAbnormal": "boolean"
  }
}
```

### 2.3 Alert

```json
{
  "id": "string (UUID)",
  "type": "VITAL_SIGN_ABNORMAL|FALL_DETECTED|MEDICATION_MISSED|EMERGENCY_BUTTON|UNUSUAL_ACTIVITY|DEVICE_OFFLINE|LOW_BATTERY|GEOFENCE_BREACH|TEMPERATURE_EXTREME|CUSTOM",
  "severity": "LOW|MEDIUM|HIGH|CRITICAL",
  "elderId": "string (UUID)",
  "title": "string",
  "message": "string",
  "timestamp": "ISO 8601 timestamp",
  "data": "object (optional, type-specific data)",
  "resolved": "boolean",
  "resolvedAt": "ISO 8601 timestamp (optional)",
  "resolvedBy": "string (UUID, optional)",
  "actions": [
    {
      "id": "string",
      "label": "string",
      "actionType": "string",
      "data": "object (optional)"
    }
  ]
}
```

### 2.4 Medication

```json
{
  "id": "string (UUID)",
  "name": "string",
  "dosage": "string",
  "frequency": "once-daily|twice-daily|three-times-daily|four-times-daily|as-needed|custom",
  "schedule": ["string (HH:MM format)"],
  "startDate": "ISO 8601 date",
  "endDate": "ISO 8601 date (optional)",
  "instructions": "string (optional)",
  "prescribedBy": "string (optional)"
}
```

### 2.5 Activity

```json
{
  "id": "string (UUID)",
  "elderId": "string (UUID)",
  "type": "SLEEP|MEAL|MEDICATION|EXERCISE|SOCIAL|BATHROOM|SHOWER|ENTERTAINMENT|MEDICAL_APPOINTMENT|THERAPY|OTHER",
  "startTime": "ISO 8601 timestamp",
  "endTime": "ISO 8601 timestamp (optional)",
  "duration": "number (seconds, optional)",
  "location": "string (optional)",
  "notes": "string (optional)",
  "completedBy": "string (UUID, optional)"
}
```

## 3. Data Validation Rules

### 3.1 Required Fields
- All `id` fields must be valid UUIDs
- All `timestamp` and date fields must be ISO 8601 format
- `elderId` must reference an existing elder profile

### 3.2 Value Ranges
- Heart rate: 30-200 bpm
- Blood pressure: Systolic 60-300, Diastolic 40-200 mmHg
- Temperature: 35-42°C or 95-108°F
- Oxygen saturation: 0-100%
- Respiratory rate: 8-40 breaths per minute

### 3.3 Abnormal Thresholds (Default)

| Vital Sign | Normal Range | Abnormal If |
|------------|--------------|-------------|
| Heart Rate | 60-100 bpm | <60 or >100 |
| Blood Pressure | <120/80 mmHg | ≥140/90 mmHg |
| Temperature | 36.5-37.5°C | <36°C or >38°C |
| O2 Saturation | 95-100% | <95% |
| Respiratory Rate | 12-20 bpm | <12 or >20 |
| Blood Glucose (fasting) | 70-100 mg/dL | <70 or >100 |

## 4. Data Encoding

### 4.1 Character Encoding
- UTF-8 encoding for all text fields
- Support for multilingual content (99 languages)

### 4.2 Date/Time Format
- ISO 8601: `YYYY-MM-DDTHH:mm:ss.sssZ`
- All timestamps in UTC
- Include timezone offset when local time is relevant

### 4.3 Units
- Metric system preferred (kg, cm, °C)
- Imperial units supported with explicit unit specification
- Always include unit field with measurements

## 5. HL7 FHIR Compatibility

### 5.1 FHIR Resource Mapping

| WIA-SENIOR-001 | FHIR Resource | Profile |
|----------------|---------------|---------|
| Elder Profile | Patient | US Core Patient |
| Vital Signs | Observation | Vital Signs Profile |
| Medication | MedicationRequest | US Core Medication |
| Alert | Flag, DetectedIssue | Alert Profile |
| Activity | Procedure, ServiceRequest | Activity Profile |

### 5.2 FHIR Export Format

```json
{
  "resourceType": "Bundle",
  "type": "collection",
  "entry": [
    {
      "resource": {
        "resourceType": "Patient",
        "id": "elder-123",
        "identifier": [...],
        "name": [...],
        "birthDate": "1940-05-15",
        ...
      }
    },
    {
      "resource": {
        "resourceType": "Observation",
        "id": "vitals-456",
        "status": "final",
        "category": [{
          "coding": [{
            "system": "http://terminology.hl7.org/CodeSystem/observation-category",
            "code": "vital-signs"
          }]
        }],
        ...
      }
    }
  ]
}
```

## 6. CSV Export Format

### 6.1 Vital Signs CSV

```csv
timestamp,elder_id,heart_rate_bpm,bp_systolic,bp_diastolic,temperature_c,o2_sat_pct
2025-01-15T08:00:00Z,elder-123,72,120,80,36.6,98
2025-01-15T12:00:00Z,elder-123,75,118,78,36.7,99
```

### 6.2 Medications CSV

```csv
medication_id,elder_id,name,dosage,frequency,schedule,start_date
med-001,elder-123,Aspirin,81mg,once-daily,08:00,2025-01-01
med-002,elder-123,Metformin,500mg,twice-daily,"08:00,20:00",2025-01-01
```

## 7. Data Privacy & Security

### 7.1 PHI/PII Fields
Protected Health Information (PHI) and Personally Identifiable Information (PII):
- Name, date of birth, address
- Medical history, allergies, medications
- Vital signs, health records
- Photos, biometric data

### 7.2 Encryption Requirements
- At rest: AES-256 encryption
- In transit: TLS 1.3
- End-to-end encryption for sensitive communications

### 7.3 Access Control
- Role-based access control (RBAC)
- Audit logging for all data access
- Granular consent management

## 8. Compliance

### 8.1 Standards Compliance
- ✅ HIPAA (Health Insurance Portability and Accountability Act)
- ✅ GDPR (General Data Protection Regulation)
- ✅ HL7 FHIR R4
- ✅ ISO 27001 (Information Security)
- ✅ FDA CFR Part 11 (Electronic Records)

### 8.2 Audit Trail
All data modifications must include:
```json
{
  "modifiedBy": "string (UUID)",
  "modifiedAt": "ISO 8601 timestamp",
  "modificationReason": "string (optional)",
  "previousValue": "object (optional)"
}
```

## 9. Versioning

### 9.1 Data Format Version
- Current version: 1.0.0
- Version field in all API responses: `"dataFormatVersion": "1.0.0"`

### 9.2 Backward Compatibility
- Minor version changes: backward compatible
- Major version changes: migration path provided

---

**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity
**Copyright:** © 2025 SmileStory Inc. / WIA
**License:** Apache 2.0
