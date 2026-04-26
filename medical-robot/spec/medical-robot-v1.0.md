# WIA-ROB-005: Medical Robot Interoperability Standard
## Version 1.0 - Technical Specification

**Published:** January 2025
**Status:** Normative
**Category:** ROB (Robotics)
**Color Code:** #10B981 (Green)

---

## Philosophy

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

This standard embodies the Korean philosophical principle of broadly benefiting humanity by establishing universal interoperability standards for medical robotics, ensuring advanced surgical technology serves all patients regardless of geographic location or economic status.

---

## 1. Executive Summary

WIA-ROB-005 defines comprehensive interoperability, safety, and integration standards for medical robots across surgical, diagnostic, rehabilitation, hospital service, and radiosurgery categories. The standard addresses critical challenges including vendor lock-in, data silos, safety protocol inconsistencies, regulatory compliance complexity, and hospital system integration barriers.

### 1.1 Scope

This standard applies to:
- **Surgical Robots:** da Vinci systems, ROSA Brain/Knee, Mako SmartRobotics, Senhance
- **Rehabilitation Robots:** Lokomat, ARMEO, Ekso GT, ReWalk exoskeletons
- **Diagnostic Robots:** Automated screening systems, AI-powered analysis platforms
- **Hospital Service Robots:** Aethon TUG, Swisslog RoboCourier, Xenex UV disinfection
- **Radiosurgery Systems:** CyberKnife, Gamma Knife Icon, ZAP-X

### 1.2 Normative References

- **HL7 FHIR R4:** Fast Healthcare Interoperability Resources Release 4 (normative)
- **DICOM PS3.1-PS3.22:** Digital Imaging and Communications in Medicine
- **ISO 13485:2016:** Medical devices — Quality management systems
- **IEC 60601-1:2012:** Medical electrical equipment — Basic safety and essential performance
- **IEC 62304:2006+AMD1:2015:** Medical device software — Software life cycle processes
- **ISO 14971:2019:** Medical devices — Application of risk management
- **IEC 62366-1:2015:** Medical devices — Usability engineering
- **IEC 62443-4-2:2019:** Security for industrial automation and control systems
- **LOINC v2.73:** Logical Observation Identifiers Names and Codes
- **SNOMED CT 2024:** Systematized Nomenclature of Medicine Clinical Terms
- **ROS2 Humble:** Robot Operating System 2
- **MQTT v5.0:** Message Queuing Telemetry Transport
- **OpenAPI 3.0.3:** OpenAPI Specification
- **OAuth 2.0 RFC 6749:** The OAuth 2.0 Authorization Framework
- **OpenID Connect Core 1.0:** OpenID Connect authentication protocol

---

## 2. Four-Phase Architecture

### 2.1 Phase 1: Data Format Standardization

**Objective:** Establish unified data formats for patient demographics, vital signs, medical imaging, surgical procedures, robot telemetry, and adverse events.

#### 2.1.1 HL7 FHIR R4 Resources

**Required Resources:**
- **Patient:** Demographics, identifiers, contact information, emergency contacts
- **Observation:** Vital signs with LOINC codes (8867-4 heart rate, 8480-6/8462-4 blood pressure, 59408-5 SpO2, 8310-5 temperature)
- **Procedure:** Surgical procedures with SNOMED CT codes
- **MedicationAdministration:** Medication delivery tracking
- **AdverseEvent:** Safety event reporting linked to FDA MAUDE database
- **Device:** Robot device information, manufacturer, model, serial number
- **Practitioner:** Surgeon, nurse, technician credentials
- **Location:** Operating room, patient location

**Validation Requirements:**
- All FHIR resources MUST validate against official HL7 FHIR R4 profiles
- US Core implementation guide compliance for US deployments
- Support for FHIR search parameters: _id, patient, date, performer, code

#### 2.1.2 DICOM Medical Imaging

**Required SOP Classes:**
- CT Image Storage (1.2.840.10008.5.1.4.1.1.2)
- MR Image Storage (1.2.840.10008.5.1.4.1.1.4)
- Digital X-Ray Image Storage (1.2.840.10008.5.1.4.1.1.1)
- Secondary Capture Image Storage (1.2.840.10008.5.1.4.1.1.7)

**Mandatory DICOM Tags:**
- (0010,0010) Patient Name
- (0010,0020) Patient ID
- (0020,000D) Study Instance UID
- (0020,000E) Series Instance UID
- (0020,0032) Image Position (Patient)
- (0020,0037) Image Orientation (Patient)
- (0028,0030) Pixel Spacing

**DICOM Web Services:**
- WADO-RS (Web Access to DICOM Objects): Retrieve studies, series, instances via HTTP GET
- STOW-RS (Store Over Web): Upload DICOM instances via HTTP POST multipart/related
- QIDO-RS (Query based on ID): Search for studies using query parameters

#### 2.1.3 Robot Telemetry JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ROB-005 Robot Telemetry",
  "type": "object",
  "required": ["timestamp", "robotId", "pose", "status"],
  "properties": {
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp"
    },
    "robotId": {
      "type": "string",
      "description": "Unique robot identifier"
    },
    "instrumentId": {
      "type": "string",
      "description": "Current instrument identifier"
    },
    "pose": {
      "type": "object",
      "required": ["position", "orientation"],
      "properties": {
        "position": {
          "type": "object",
          "required": ["x", "y", "z", "unit", "coordinateFrame"],
          "properties": {
            "x": {"type": "number"},
            "y": {"type": "number"},
            "z": {"type": "number"},
            "unit": {"type": "string", "enum": ["mm", "m"]},
            "coordinateFrame": {"type": "string"}
          }
        },
        "orientation": {
          "type": "object",
          "required": ["quaternion"],
          "properties": {
            "quaternion": {
              "type": "object",
              "required": ["w", "x", "y", "z"],
              "properties": {
                "w": {"type": "number", "minimum": -1, "maximum": 1},
                "x": {"type": "number", "minimum": -1, "maximum": 1},
                "y": {"type": "number", "minimum": -1, "maximum": 1},
                "z": {"type": "number", "minimum": -1, "maximum": 1}
              }
            }
          }
        }
      }
    },
    "velocity": {
      "type": "object",
      "properties": {
        "linear": {
          "type": "object",
          "properties": {
            "x": {"type": "number"},
            "y": {"type": "number"},
            "z": {"type": "number"},
            "unit": {"type": "string", "enum": ["mm/s", "m/s"]}
          }
        },
        "angular": {
          "type": "object",
          "properties": {
            "x": {"type": "number"},
            "y": {"type": "number"},
            "z": {"type": "number"},
            "unit": {"type": "string", "enum": ["deg/s", "rad/s"]}
          }
        }
      }
    },
    "forceTorque": {
      "type": "object",
      "properties": {
        "force": {
          "type": "object",
          "properties": {
            "x": {"type": "number"},
            "y": {"type": "number"},
            "z": {"type": "number"},
            "unit": {"type": "string", "enum": ["N", "kN"]}
          }
        },
        "torque": {
          "type": "object",
          "properties": {
            "x": {"type": "number"},
            "y": {"type": "number"},
            "z": {"type": "number"},
            "unit": {"type": "string", "enum": ["Nm", "kNm"]}
          }
        }
      }
    },
    "status": {
      "type": "string",
      "enum": ["idle", "active", "paused", "error", "emergency_stop"]
    },
    "errorCode": {
      "type": ["string", "null"]
    },
    "batteryLevel": {
      "type": "number",
      "minimum": 0,
      "maximum": 100
    },
    "temperature": {
      "type": "number",
      "description": "Operating temperature in Celsius"
    }
  }
}
```

### 2.2 Phase 2: API Interface Standards

**Objective:** Define RESTful HTTP APIs with OpenAPI 3.0 specifications, OAuth 2.0 authentication, and comprehensive security controls.

#### 2.2.1 OpenAPI 3.0 Specification Requirements

All robot APIs MUST provide complete OpenAPI 3.0 specifications including:
- Info object (title, version, description, contact, license)
- Server objects with base URLs
- Paths with operations (GET, POST, PUT, PATCH, DELETE)
- Request/response schemas using JSON Schema
- Security schemes (OAuth 2.0, API keys)
- Error response definitions

#### 2.2.2 Authentication & Authorization

**OAuth 2.0 + OpenID Connect:**
- Authorization Code flow for interactive applications
- Client Credentials flow for system-to-system communication
- JWT (JSON Web Tokens) for identity and permissions
- Token lifetime: Access token 1 hour, refresh token 30 days
- Revocation endpoint support

**Role-Based Access Control (RBAC):**
- **Surgeon:** Full robot control, procedure initiation, emergency stop
- **Nurse:** Monitoring, vitals access, documentation, emergency stop
- **Technician:** Configuration, calibration, maintenance, diagnostics
- **Administrator:** User management, system configuration, audit logs
- **Read-Only Observer:** Monitoring only, no control capabilities

#### 2.2.3 Core API Endpoints

```
POST /robots/{robotId}/initialize
POST /robots/{robotId}/move
  Body: { targetPose, speed, interpolationType }
POST /robots/{robotId}/emergency-stop
  Response: { responseTime, status }
GET /robots/{robotId}/pose
  Response: { position, orientation, timestamp }
GET /robots/{robotId}/status
  Response: { operationalState, errors, warnings, batteryLevel }
GET /robots/{robotId}/sensors/force
  Response: { force, torque, timestamp }
PUT /robots/{robotId}/config
  Body: { parameters, calibration, safetyLimits }
GET /robots/{robotId}/logs
  Query: startDate, endDate, eventType
POST /robots/{robotId}/procedures
  Body: FHIR Procedure resource
GET /patients/{patientId}
  Response: FHIR Patient resource
POST /imaging/dicom/retrieve
  Body: { studyInstanceUID, seriesInstanceUID }
```

### 2.3 Phase 3: Real-Time Protocol Standards

**Objective:** Implement sub-10ms latency communication for time-critical operations using MQTT, WebSocket, and ROS2.

#### 2.3.1 MQTT Requirements

- **Protocol Version:** MQTT v5.0
- **QoS Levels:**
  - QoS 0 for high-frequency low-criticality telemetry (pose, vitals)
  - QoS 1 for moderate criticality (status updates, warnings)
  - QoS 2 for critical safety messages (emergency stop, collision alerts)
- **Topic Structure:** `robots/{robotId}/{category}/{dataType}`
- **Retained Messages:** Status topics retain last message for new subscribers
- **Last Will and Testament:** Automatic disconnection notification
- **Security:** TLS 1.3 encryption, certificate-based authentication

#### 2.3.2 WebSocket Requirements

- **Protocol:** RFC 6455 WebSocket over HTTPS (wss://)
- **Subprotocols:** robot-telemetry-v1, robot-control-v1
- **Heartbeat:** Ping/Pong frames every 30 seconds
- **Binary Frames:** H.264/VP9 video streams, compressed telemetry
- **Text Frames:** JSON commands, status updates
- **Reconnection:** Exponential backoff (1s, 2s, 4s, 8s, max 60s)

#### 2.3.3 Safety Protocol Requirements

**Emergency Stop:**
- Level 1 (Pause): Software stop, <100ms response, immediate resume capability
- Level 2 (Safe Stop): Controlled deceleration, <500ms, electromagnetic brakes, deliberate restart
- Level 3 (Emergency): Hardware interlock, <50ms, mechanical brakes, manual reset

**Collision Detection:**
- Force/torque thresholds: 5N for soft tissue, 20N for bone/hard tissue
- Response: Immediate stop, visual/audible alert, logged event
- Recovery: Manual review and clearance required

**Vital Signs Monitoring:**
- Continuous polling: 1 Hz for stable patients, 10 Hz for critical patients
- Critical thresholds: SpO2 <88%, HR <40 or >140 bpm, SBP <70 or >200 mmHg
- Action: Automated procedure suspension, alert clinical team

### 2.4 Phase 4: Hospital System Integration

**Objective:** Seamless integration with EHR, PACS, HL7 interfaces, and hospital authentication systems.

#### 2.4.1 EHR Integration Requirements

**Epic:**
- Interconnect certified application
- FHIR R4 API integration (read/write Patient, Observation, Procedure)
- SMART on FHIR authorization
- MyChart patient communication

**Cerner:**
- Millennium platform integration
- FHIR R4 API compliance
- PowerChart workflow integration
- HealtheIntent analytics submission

#### 2.4.2 PACS Integration Requirements

- DICOM Modality Worklist (MWL) for procedure scheduling
- C-FIND, C-MOVE for legacy PACS
- DICOM Web (WADO-RS, STOW-RS, QIDO-RS) for modern PACS
- Automated image retrieval on procedure scheduling
- Intraoperative image storage back to PACS

#### 2.4.3 HL7 v2.x Messaging

- ADT^A01 (Admit), ADT^A03 (Discharge) for patient tracking
- ORM^O01 (Order Entry) for procedure scheduling
- ORU^R01 (Observation Result) for robot findings
- DFT^P03 (Detailed Financial Transaction) for billing

---

## 3. Certification Levels

### 3.1 Bronze Certification

**Requirements:**
- Phase 1 data format compliance (FHIR, DICOM, JSON Schema)
- Basic safety protocols (emergency stop <500ms)
- Technical documentation and user manual
- 100% pass rate on WIA conformance tests

**Typical Devices:** Hospital service robots, basic rehabilitation devices, telemedicine robots

### 3.2 Silver Certification

**Requirements:**
- Bronze requirements plus Phase 2 API compliance
- OpenAPI 3.0 specifications for all endpoints
- OAuth 2.0 + OIDC authentication
- ISO 13485:2016 quality management certification
- ISO 14971:2019 risk management file

**Typical Devices:** Diagnostic robots, surgical planning systems, pharmacy automation

### 3.3 Gold Certification

**Requirements:**
- Silver requirements plus Phases 3-4 implementation
- Real-time protocols with <10ms latency validation
- EHR integration (Epic, Cerner, or equivalent)
- PACS integration with DICOM Modality Worklist
- FDA 510(k) clearance or CE marking under MDR
- Clinical validation (20+ procedures for surgical robots)
- IEC 62443 cybersecurity assessment

**Typical Devices:** Surgical robots, radiosurgery systems, autonomous diagnostic systems

---

## 4. Compliance and Validation

### 4.1 Automated Conformance Testing

WIA provides automated test suites validating:
- FHIR resource validation against official profiles
- DICOM tag extraction and validation
- JSON Schema validation for telemetry
- OpenAPI specification compliance
- OAuth 2.0 flow validation
- API rate limiting and quota enforcement
- Emergency stop response time measurement

### 4.2 Manual Testing Procedures

- Safety system validation (FMEA, fault injection)
- Usability testing per IEC 62366
- Cybersecurity penetration testing
- Clinical validation studies
- On-site inspection for Gold certification

---

## 5. Security Requirements

### 5.1 Data Encryption

- TLS 1.3 for all network communication
- AES-256 for data at rest
- End-to-end encryption for patient data

### 5.2 Access Control

- Multi-factor authentication for surgical procedures
- Biometric authentication for high-risk operations
- Audit logging of all robot control actions
- 90-day log retention minimum

### 5.3 Vulnerability Management

- Quarterly security assessments
- 30-day patch deployment for critical vulnerabilities
- Coordinated disclosure process
- CVE registration for discovered vulnerabilities

---

## 6. Change History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-15 | Initial release - normative standard |

---

## 7. Contact Information

**WIA (World Certification Industry Association)**

- Website: https://wia-standards.org
- Email: medical-robotics@wia-standards.org
- Certification Portal: https://cert.wia-standards.org

---

© 2025 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity
