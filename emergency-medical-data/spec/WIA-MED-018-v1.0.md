# WIA MED-018: Emergency Medical Data Standard

**Version:** 1.0.0
**Status:** Published
**Date:** 2025-12-26
**Organization:** WIA - World Certification Industry Association
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## Abstract

WIA MED-018 defines the standard for Emergency Medical Data, providing comprehensive specifications for EMS data formats, patient handoff protocols, trauma registry, real-time location sharing, hospital capacity data, triage systems, ambulance connectivity, and disaster response coordination.

## 1. Scope

This standard applies to:
- Emergency Medical Services (EMS) data collection and transmission
- Patient handoff between field personnel and hospital staff
- Trauma registry and injury surveillance systems
- Real-time GPS tracking of ambulances and incident locations
- Hospital capacity and resource availability data
- Patient triage and prioritization systems
- Connected ambulance communication infrastructure
- Mass casualty incident (MCI) and disaster response coordination

## 2. Normative References

- HL7 FHIR R4
- HL7 v2.x
- NEMSIS (National EMS Information System) v3.5
- ISO/IEEE 11073 (Point-of-care medical device communication)
- HIPAA Security Rule
- GDPR
- NIST Cybersecurity Framework
- ICS (Incident Command System)

## 3. Terms and Definitions

### 3.1 Emergency Medical Services (EMS)
Prehospital emergency care and patient transportation to definitive medical facilities.

### 3.2 Patient Handoff
Transfer of patient care responsibility and information between healthcare providers.

### 3.3 Trauma Registry
Systematic collection and analysis of data on traumatic injury patients.

### 3.4 Triage
Process of determining patient treatment priority based on severity of condition.

### 3.5 Mass Casualty Incident (MCI)
Event that generates more patients than available resources can manage using routine procedures.

## 4. EMS Data Format

### 4.1 Core Data Structure

All EMS data MUST be transmitted in standardized JSON format conforming to WIA MED-018 schema.

```json
{
  "standard": "WIA-MED-018",
  "version": "1.0.0",
  "messageType": "EMS_DISPATCH" | "EMS_ARRIVAL" | "EMS_TRANSPORT" | "EMS_HANDOFF",
  "timestamp": "ISO 8601 datetime",
  "incidentId": "Unique incident identifier",

  "incident": {
    "callTime": "ISO 8601 datetime",
    "dispatchTime": "ISO 8601 datetime",
    "priority": "CRITICAL" | "URGENT" | "ROUTINE",
    "type": "TRAUMA" | "CARDIAC" | "STROKE" | "RESPIRATORY" | "OTHER",
    "location": {
      "address": "string",
      "latitude": "number",
      "longitude": "number",
      "altitude": "number (optional)",
      "accuracy": "number (meters)"
    }
  },

  "patient": {
    "demographics": {
      "estimatedAge": "integer",
      "gender": "MALE" | "FEMALE" | "OTHER",
      "weight": "number (kg, optional)",
      "height": "number (cm, optional)"
    },
    "chiefComplaint": "string",
    "allergies": ["array of strings"],
    "medications": ["array of strings"],
    "medicalHistory": ["array of strings"]
  },

  "vitals": [{
    "timestamp": "ISO 8601 datetime",
    "bloodPressure": {
      "systolic": "integer (60-250 mmHg)",
      "diastolic": "integer (40-150 mmHg)"
    },
    "heartRate": "integer (30-300 bpm)",
    "respiratoryRate": "integer (8-60 breaths/min)",
    "temperature": "number (32.0-42.0 °C)",
    "oxygenSaturation": "integer (0-100 %)",
    "glucoseLevel": "integer (20-600 mg/dL, optional)",
    "painLevel": "integer (0-10, optional)",
    "consciousnessLevel": "ALERT" | "VERBAL" | "PAIN" | "UNRESPONSIVE"
  }],

  "assessment": {
    "primarySurvey": {
      "airway": "PATENT" | "COMPROMISED" | "OBSTRUCTED",
      "breathing": "SPONTANEOUS" | "ASSISTED" | "ABSENT",
      "circulation": "STABLE" | "UNSTABLE" | "CRITICAL"
    },
    "secondarySurvey": {
      "ecg": "string (optional)",
      "trauma": "object (optional)",
      "neurologicalStatus": "string (optional)"
    },
    "workingDiagnosis": "string"
  },

  "interventions": [{
    "timestamp": "ISO 8601 datetime",
    "intervention": "string",
    "details": "string",
    "performedBy": "string"
  }],

  "transport": {
    "ambulanceId": "string",
    "departureTime": "ISO 8601 datetime",
    "destinationHospital": {
      "id": "string",
      "name": "string",
      "department": "string",
      "eta": "ISO 8601 datetime"
    }
  }
}
```

### 4.2 Required Fields

The following fields are REQUIRED for all EMS transmissions:
- `standard`, `version`, `messageType`, `timestamp`, `incidentId`
- `incident.location` (at minimum latitude/longitude)
- `patient.demographics.estimatedAge`, `patient.demographics.gender`
- At least one `vitals` entry with `timestamp`, `consciousnessLevel`

### 4.3 Optional Fields

All other fields are OPTIONAL but RECOMMENDED when data is available.

## 5. Patient Handoff Protocol

### 5.1 SBAR Framework

All patient handoffs MUST follow the SBAR (Situation-Background-Assessment-Recommendation) framework.

### 5.2 Handoff Data Structure

```json
{
  "messageType": "EMS_HANDOFF",
  "handoffProtocol": "SBAR",
  "handoffTime": "ISO 8601 datetime",
  "receivingPhysician": "string",

  "situation": {
    "chiefComplaint": "string",
    "currentStatus": "string",
    "acuityLevel": "CRITICAL" | "URGENT" | "ROUTINE"
  },

  "background": {
    "medicalHistory": ["array"],
    "medications": ["array"],
    "allergies": ["array"]
  },

  "assessment": {
    "vitalsTrend": ["array of vitals"],
    "physicalExam": "object",
    "workingDiagnosis": "string"
  },

  "recommendation": {
    "interventionsPerformed": ["array"],
    "suggestedActions": ["array"],
    "specialConsiderations": ["array"]
  },

  "handoffQuality": {
    "completenessScore": "number (0-100)",
    "readBackConfirmed": "boolean",
    "timeSpent": "integer (seconds)"
  }
}
```

## 6. Trauma Registry

### 6.1 Injury Severity Scoring

Support for the following standardized injury severity scores:
- AIS (Abbreviated Injury Scale)
- ISS (Injury Severity Score)
- RTS (Revised Trauma Score)
- TRISS (Trauma and Injury Severity Score)
- GCS (Glasgow Coma Scale)

### 6.2 Minimum Dataset

All trauma registry entries MUST include:
- Patient demographics
- Injury mechanism and intent
- Injury date, time, and location
- Prehospital care details
- Hospital arrival vitals
- Injury descriptions with AIS codes
- Procedures performed
- Outcomes and disposition

## 7. Real-time Location Data

### 7.1 GPS Accuracy Requirements

- Horizontal accuracy: ≤ 10 meters (95% confidence)
- Update frequency: ≤ 10 seconds
- Support for indoor positioning using WiFi/Bluetooth/UWB

### 7.2 Location Data Format

```json
{
  "entity": {
    "type": "AMBULANCE" | "PATIENT" | "INCIDENT_SITE",
    "id": "string"
  },

  "location": {
    "coordinates": {
      "latitude": "number",
      "longitude": "number",
      "altitude": "number (optional)",
      "accuracy": "number (meters)",
      "verticalAccuracy": "number (meters, optional)"
    },
    "address": {
      "full": "string",
      "floor": "integer (optional)",
      "unit": "string (optional)"
    }
  },

  "movement": {
    "speed": "number (km/h)",
    "heading": "number (0-359 degrees)",
    "isMoving": "boolean"
  }
}
```

## 8. Hospital Capacity Data

### 8.1 Real-time Capacity Updates

Hospitals MUST update capacity data at least every 5 minutes.

### 8.2 Capacity Data Structure

```json
{
  "hospitalId": "string",
  "updateTime": "ISO 8601 datetime",

  "emergencyDepartment": {
    "status": "OPEN" | "CLOSED" | "DIVERTED",
    "totalBeds": "integer",
    "availableBeds": "integer",
    "occupancyRate": "number (percentage)",
    "waitTime": "integer (minutes)"
  },

  "intensiveCare": {
    "medical": {"total": "integer", "available": "integer"},
    "surgical": {"total": "integer", "available": "integer"},
    "cardiac": {"total": "integer", "available": "integer"}
  },

  "specializedServices": {
    "cathLab": {"available": "boolean", "nextSlot": "string"},
    "ctScanner": {"available": "boolean", "waitTime": "integer"},
    "strokeCenter": {"active": "boolean", "teamReady": "boolean"}
  }
}
```

## 9. Triage System

### 9.1 Supported Triage Systems

- KTAS (Korean Triage and Acuity Scale) - 5 levels
- ESI (Emergency Severity Index) - 5 levels
- START (Simple Triage And Rapid Treatment) - 4 categories
- JumpSTART (Pediatric triage)

### 9.2 Triage Levels

1. **Level 1 (Red)**: Resuscitation - Immediate threat to life
2. **Level 2 (Orange)**: Emergency - High risk, should be seen within 15 minutes
3. **Level 3 (Yellow)**: Urgent - Moderate risk, 30-60 minutes
4. **Level 4 (Green)**: Less Urgent - Low risk, 1-2 hours
5. **Level 5 (White)**: Non-Urgent - Very low risk, can wait

## 10. Ambulance Connectivity

### 10.1 Network Requirements

- Primary: 5G/LTE with ≥ 10 Mbps bandwidth
- Fallback: 3G/Satellite
- Latency: ≤ 100ms (95th percentile)
- Uptime: ≥ 99.5%

### 10.2 Data Transmission

Support for real-time streaming of:
- Vital signs (continuous)
- 12-lead ECG
- Video consultation (HD quality)
- Ultrasound imaging

## 11. Disaster Response

### 11.1 MCI Declaration Thresholds

- **Level 1**: > 100 casualties
- **Level 2**: 50-100 casualties
- **Level 3**: 20-50 casualties
- **Level 4**: 10-20 casualties
- **Level 5**: 5-10 casualties

### 11.2 ICS Integration

Full support for Incident Command System (ICS) structure and communication protocols.

## 12. Security and Privacy

### 12.1 Encryption

- **Transport**: TLS 1.3 minimum
- **Storage**: AES-256-GCM
- **Keys**: RSA-4096 or ECDSA-P256

### 12.2 Authentication

- Mutual TLS (mTLS) for device authentication
- OAuth 2.0 + JWT for user authentication
- Multi-factor authentication for sensitive operations

### 12.3 Compliance

- HIPAA compliant (US)
- GDPR compliant (EU)
- PIPEDA compliant (Canada)
- PHIPA compliant (Ontario)

## 13. Data Quality Requirements

### 13.1 Completeness

- Required fields: 100%
- Recommended fields: ≥ 80%
- Optional fields: as available

### 13.2 Accuracy

- Vital signs: Within manufacturer specifications
- Timestamps: Synchronized to NTP (±1 second)
- Location: As specified in Section 7.1

### 13.3 Timeliness

- Real-time data: ≤ 30 seconds from capture to transmission
- Historical data: Available within 24 hours

## 14. Interoperability

### 14.1 APIs

RESTful APIs MUST support:
- JSON payload format
- HTTPS transport
- OAuth 2.0 authentication
- Rate limiting: ≥ 1000 requests/minute

### 14.2 HL7 FHIR Mapping

Support for conversion to/from HL7 FHIR resources:
- Patient
- Observation
- Condition
- Procedure
- Encounter
- Location

## 15. Testing and Certification

### 15.1 Conformance Testing

Implementations MUST pass WIA conformance tests covering:
- Data format validation
- API functionality
- Security requirements
- Performance benchmarks

### 15.2 Certification

Organizations may obtain WIA MED-018 certification by:
1. Passing all conformance tests
2. Demonstrating 30-day operational stability
3. Annual recertification required

## Appendix A: Sample Implementation

See `/examples` directory for reference implementations in:
- TypeScript/Node.js
- Python
- Java
- C#

## Appendix B: Changelog

### Version 1.0.0 (2025-12-26)
- Initial release

---

**Contact:**
WIA - World Certification Industry Association
Email: standards@wiastandards.com
Website: https://wiastandards.com

**License:**
MIT License - Free for all humanity

**Philosophy:**
弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 WIA - World Certification Industry Association
