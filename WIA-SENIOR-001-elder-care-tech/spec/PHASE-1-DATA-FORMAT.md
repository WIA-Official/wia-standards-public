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

## P.1 Data Format Cross-References

This Phase defines the canonical data types referenced by the API surface (Phase 2),
the wire protocol (Phase 3), and integration scenarios (Phase 4). Implementations
MUST round-trip every canonical type through serialization and deserialization
without loss of precision or semantics.

### P.1.1 Canonical Encoding Rules

1. UTF-8 is the required character encoding for textual fields.
2. Numeric fields use IEEE 754 binary64 unless explicitly marked as fixed-point.
3. Timestamps use RFC 3339 with timezone offset; durations use ISO 8601.
4. UUIDs follow RFC 4122 v4 unless deterministic IDs are required.
5. Binary payloads are encoded as Base64 (RFC 4648 §4) in JSON contexts and as
   raw octet strings in Protocol Buffers / CBOR contexts.

### P.1.2 Schema Evolution

Schema changes follow these compatibility classes:

| Class | Allowed Changes | Wire-Compat |
|-------|-----------------|-------------|
| Patch | Doc fixes, examples, validator tightening within existing range | Forward & backward |
| Minor | New optional fields, new enum values with default fallback        | Forward |
| Major | Field rename, type change, removal, semantics change              | None |

### P.1.3 Validation Order

Validators MUST apply checks in this order: (1) syntactic well-formedness,
(2) schema conformance, (3) cross-field invariants, (4) external referential
integrity, (5) policy / authorization. A failure short-circuits subsequent
checks; the response message identifies the first failing rule by ID.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of WIA-SENIOR-001-elder-care-tech so that conformance claims at any
Phase remain unambiguous.*

