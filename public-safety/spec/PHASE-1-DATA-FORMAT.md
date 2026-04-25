# WIA-SOC-004 Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Approved
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines the standardized data formats for public safety systems, including emergency incidents, first responder data, alert notifications, and communication logs. All data MUST use JSON-LD format for semantic interoperability.

## 2. Core Data Types

### 2.1 Emergency Incident

```json
{
  "@context": "https://wiastandards.com/soc-004/v1",
  "@type": "EmergencyIncident",
  "incidentId": "INC-2025-XXXX-YYYY",
  "incidentType": "fire|medical|police|natural_disaster|hazmat|accident",
  "priority": "low|medium|high|critical",
  "status": "reported|dispatched|responded|resolved|closed",
  "reportedAt": "ISO8601 datetime",
  "location": {
    "latitude": "float",
    "longitude": "float",
    "address": "string",
    "landmark": "string (optional)",
    "floor": "integer (optional)",
    "room": "string (optional)"
  },
  "caller": {
    "phoneNumber": "string",
    "name": "string (optional)",
    "language": "ISO 639-1 code"
  },
  "description": "string",
  "severity": 1-10,
  "casualties": {
    "injured": "integer",
    "critical": "integer",
    "fatalities": "integer"
  }
}
```

### 2.2 First Responder Unit

```json
{
  "@type": "FirstResponder",
  "unitId": "UNIT-XXXX-YYYY",
  "unitType": "fire_engine|ambulance|police_car|hazmat_team|rescue_squad",
  "status": "available|dispatched|on_scene|returning|unavailable",
  "location": {
    "latitude": "float",
    "longitude": "float",
    "heading": "float (degrees 0-360)",
    "speed": "float (km/h)"
  },
  "crew": [
    {
      "id": "string",
      "name": "string",
      "role": "paramedic|firefighter|officer|specialist",
      "certifications": ["array", "of", "certifications"]
    }
  ],
  "equipment": {
    "medical": ["array", "of", "equipment"],
    "rescue": ["array", "of", "equipment"],
    "communication": ["array", "of", "devices"]
  },
  "capacity": {
    "personnel": "integer",
    "patients": "integer"
  },
  "eta": "integer (seconds)"
}
```

### 2.3 Alert Notification

```json
{
  "@type": "EmergencyAlert",
  "alertId": "ALERT-XXXX-YYYY",
  "alertType": "amber|silver|weather|earthquake|tsunami|fire|hazmat|active_shooter",
  "priority": "info|warning|urgent|critical",
  "issuedAt": "ISO8601 datetime",
  "expiresAt": "ISO8601 datetime",
  "affectedArea": {
    "type": "circle|polygon|administrative",
    "coordinates": [[lat, lng], ...],
    "radius": "float (km, for circle)",
    "population": "integer (estimated)"
  },
  "message": {
    "headline": "string",
    "description": "string",
    "instruction": "string",
    "languages": {
      "en": "english message",
      "es": "spanish message",
      "zh": "chinese message"
    }
  },
  "severity": "minor|moderate|severe|extreme",
  "certainty": "observed|likely|possible|unlikely",
  "urgency": "immediate|expected|future|past"
}
```

### 2.4 Communication Log

```json
{
  "@type": "CommunicationLog",
  "logId": "LOG-XXXX-YYYY",
  "incidentId": "INC-XXXX-YYYY",
  "timestamp": "ISO8601 datetime",
  "channel": "phone|radio|text|video|data",
  "participants": [
    {
      "id": "string",
      "role": "dispatcher|responder|caller|command",
      "callsign": "string (optional)"
    }
  ],
  "duration": "integer (seconds)",
  "recording": {
    "available": "boolean",
    "url": "string (optional)",
    "transcription": "string (optional)"
  },
  "priority": "routine|priority|emergency",
  "encryption": "boolean"
}
```

## 3. Location Data Formats

### 3.1 Geographic Coordinates

```json
{
  "@type": "GeoLocation",
  "latitude": "float (-90 to 90)",
  "longitude": "float (-180 to 180)",
  "altitude": "float (meters, optional)",
  "accuracy": "float (meters)",
  "heading": "float (degrees 0-360, optional)",
  "speed": "float (m/s, optional)",
  "timestamp": "ISO8601 datetime"
}
```

### 3.2 Address Information

```json
{
  "@type": "Address",
  "streetAddress": "string",
  "unit": "string (optional)",
  "city": "string",
  "state": "string",
  "postalCode": "string",
  "country": "ISO 3166-1 alpha-2",
  "geocoded": {
    "latitude": "float",
    "longitude": "float"
  },
  "plusCode": "string (optional)",
  "what3words": "string (optional)"
}
```

## 4. Event Data Format

```json
{
  "@type": "SafetyEvent",
  "eventId": "UUID",
  "timestamp": "ISO8601 datetime",
  "eventType": "dispatch|arrival|departure|status_change|escalation",
  "incidentId": "INC-XXXX-YYYY",
  "unitId": "UNIT-XXXX-YYYY (optional)",
  "data": {
    "previousStatus": "string",
    "newStatus": "string",
    "reason": "string (optional)",
    "location": "GeoLocation (optional)"
  }
}
```

## 5. Dispatch Command Format

```json
{
  "@type": "DispatchCommand",
  "commandId": "UUID",
  "timestamp": "ISO8601 datetime",
  "incidentId": "INC-XXXX-YYYY",
  "action": "dispatch|recall|redirect|escalate|standby",
  "units": ["UNIT-ID-1", "UNIT-ID-2"],
  "priority": "routine|urgent|emergency",
  "route": {
    "waypoints": [[lat, lng], ...],
    "distance": "float (km)",
    "estimatedDuration": "integer (seconds)",
    "trafficConditions": "clear|moderate|heavy|blocked"
  },
  "instructions": "string",
  "respondBy": "ISO8601 datetime"
}
```

## 6. Medical Data Format

```json
{
  "@type": "MedicalReport",
  "reportId": "MED-XXXX-YYYY",
  "incidentId": "INC-XXXX-YYYY",
  "timestamp": "ISO8601 datetime",
  "patient": {
    "age": "integer (optional)",
    "gender": "M|F|other|unknown",
    "chiefComplaint": "string",
    "consciousness": "alert|verbal|pain|unresponsive",
    "breathing": "normal|difficulty|absent",
    "circulation": "normal|weak|absent"
  },
  "vitals": {
    "heartRate": "integer (bpm)",
    "bloodPressure": "string (systolic/diastolic)",
    "respiratoryRate": "integer (per min)",
    "temperature": "float (°C)",
    "oxygenSaturation": "integer (%)"
  },
  "treatment": [
    {
      "timestamp": "ISO8601 datetime",
      "intervention": "string",
      "medication": "string (optional)",
      "dosage": "string (optional)"
    }
  ],
  "destination": {
    "facilityId": "string",
    "facilityName": "string",
    "estimatedArrival": "ISO8601 datetime"
  }
}
```

## 7. Validation Rules

1. All timestamps MUST use ISO 8601 format with timezone (UTC preferred)
2. All geographic coordinates MUST use WGS84 datum
3. All measurements MUST use SI units
4. Phone numbers MUST use E.164 format
5. Language codes MUST use ISO 639-1
6. Country codes MUST use ISO 3166-1 alpha-2
7. Required fields MUST NOT be null
8. Enum values MUST match specification exactly
9. PII (Personally Identifiable Information) MUST be encrypted at rest
10. All communication logs MUST have retention policies

## 8. Privacy and Security

### 8.1 Data Classification

```json
{
  "@type": "DataClassification",
  "classificationLevel": "public|internal|confidential|restricted",
  "piiCategories": ["name", "phone", "address", "medical"],
  "retentionPeriod": "ISO8601 duration (e.g., P7Y for 7 years)",
  "encryptionRequired": "boolean",
  "accessControl": {
    "roles": ["dispatcher", "supervisor", "investigator"],
    "auditLog": "boolean"
  }
}
```

### 8.2 Anonymization

For analytics and public data releases, PII MUST be anonymized:

```json
{
  "@type": "AnonymizedIncident",
  "incidentId": "hashed",
  "incidentType": "fire",
  "priority": "high",
  "location": {
    "approximateArea": "downtown",
    "postalCode": "94102",
    "coordinates": "rounded to 0.01 degrees"
  },
  "responseTime": "integer (seconds)",
  "resolved": "boolean"
}
```

## 9. Extensibility

Implementations MAY add custom fields prefixed with "x_" to avoid conflicts:

```json
{
  "@type": "EmergencyIncident",
  "incidentId": "INC-2025-0001",
  "x_localJurisdiction": "District 5",
  "x_customProtocol": "vendor-specific-data"
}
```

---

© 2025 WIA · MIT License

## 10. Data Examples

### Complete Emergency Incident Example

```json
{
  "@context": "https://wiastandards.com/soc-004/v1",
  "@type": "EmergencyIncident",
  "incidentId": "INC-2025-12-26-0042",
  "incidentType": "medical",
  "priority": "critical",
  "status": "dispatched",
  "reportedAt": "2025-12-26T14:23:15Z",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "address": "123 Market Street, San Francisco, CA 94102",
    "floor": 3,
    "room": "Conference Room A"
  },
  "caller": {
    "phoneNumber": "+14155551234",
    "name": "John Doe",
    "language": "en"
  },
  "description": "Male, approximately 50 years old, chest pain, difficulty breathing",
  "severity": 9,
  "casualties": {
    "injured": 1,
    "critical": 1,
    "fatalities": 0
  }
}
```

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-SOC-PUBLIC-SAFETY (Public Safety) is evaluated across three tiers, applied to dispatch · alerting · responder coordination · evidence chain-of-custody:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | None (annual self-review recommended) |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST clearly disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the following published standards. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- OASIS CAP 1.2 — Common Alerting Protocol
- OASIS EDXL-DE 2.0 — Emergency Data Exchange Language
- ISO 22320:2018 — Emergency management — Guidelines for incident management
- NIST SP 800-53 Rev. 5 — Security and privacy controls (federal reference baseline)
- IETF RFC 8259 — JSON data interchange (alert payload encoding)

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/public-safety/api/` — TypeScript SDK skeleton
- `wia-standards/standards/public-safety/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/public-safety/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
