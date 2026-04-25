# WIA-ROB-006: Surgical Robot Standard
## Technical Specification v1.0

> 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

### 1.1 Scope
This specification defines technical requirements for surgical robotic systems seeking WIA-ROB-006 certification. The standard establishes data formats, APIs, protocols, and integration requirements enabling interoperability, safety, and innovation across all certified platforms.

### 1.2 Normative References
- ISO 13482: Robots and robotic devices — Safety requirements for personal care robots
- IEC 60601: Medical electrical equipment — Safety and essential performance
- ISO 14971: Medical devices — Risk management
- HL7 FHIR R4: Fast Healthcare Interoperability Resources
- DICOM 3.0: Digital Imaging and Communications in Medicine
- IEEE 11073: Personal Health Device Communication
- ISO 8601: Date and time format
- RFC 7519: JSON Web Token (JWT)
- RFC 6749: OAuth 2.0 Authorization Framework

### 1.3 Definitions
- **Surgical Robot**: Electromechanical device providing motion scaling, tremor reduction, and enhanced visualization for minimally invasive surgery
- **Teleoperation**: Master-slave control where surgeon manipulates master controls translated to slave instrument motion
- **Haptic Feedback**: Force and tactile sensation transmitted from instruments to surgeon console
- **Interlock**: Safety mechanism preventing operation under unsafe conditions
- **E-Stop**: Emergency stop immediately halting all robot motion

---

## 2. System Architecture

### 2.1 Components
All WIA-ROB-006 certified systems must include:

1. **Surgeon Console**: Master manipulators, stereoscopic display, foot pedals, user interface
2. **Patient-Side Cart**: Robotic arms, instrument mounts, camera system
3. **Vision System**: High-definition 3D camera, illumination, image processing
4. **Control System**: Real-time control computer, safety systems, communication interfaces
5. **Instrument Set**: Interchangeable surgical tools with defined interfaces

### 2.2 Coordinate Systems
Three standardized reference frames:

- **World Frame** (W): Fixed OR coordinate system, origin at table center, Z-axis vertical
- **Patient Frame** (P): Anatomical coordinates, origin at surgical site
- **Instrument Frame** (I): Local coordinates at instrument tip, Z-axis along shaft

Transformations: T_WP (world-to-patient), T_WI (world-to-instrument), T_PI (patient-to-instrument)

---

## 3. Phase 1: Data Format Specifications

### 3.1 Surgical Telemetry Format

**Format**: JSON  
**Encoding**: UTF-8  
**Timestamp**: ISO 8601 (YYYY-MM-DDTHH:MM:SS.sssZ)

**Minimum Fields**:
```json
{
  "session_id": "string (UUID v4)",
  "timestamp": "ISO 8601 string",
  "robot": {
    "manufacturer": "string",
    "model": "string",
    "serial": "string",
    "firmware_version": "string"
  },
  "instruments": [
    {
      "id": "string (unique identifier)",
      "type": "enum (grasper|scissors|cautery|needle_driver|...)",
      "position": {"x": "float (mm)", "y": "float (mm)", "z": "float (mm)"},
      "orientation": {"w": "float", "x": "float", "y": "float", "z": "float"},
      "velocity": {"x": "float (mm/s)", "y": "float (mm/s)", "z": "float (mm/s)"},
      "force": {"fx": "float (N)", "fy": "float (N)", "fz": "float (N)"},
      "state": "enum (idle|moving|active|disabled|error)"
    }
  ]
}
```

**Update Frequency**: 10-100 Hz  
**Latency**: < 50 ms end-to-end

### 3.2 Video Data

**Codec**: H.265 (HEVC) or VP9  
**Resolution**: Minimum 1920x1080 @ 30fps, Preferred 3840x2160 @ 60fps  
**Color Space**: Rec. 709 (SDR) or Rec. 2020 (HDR)  
**Bitrate**: 10-50 Mbps depending on resolution  
**Container**: MP4 or WebM  
**Metadata**: Embedded timecode synchronized with telemetry

### 3.3 Patient Anatomy

**Pre-operative Imaging**: DICOM format (CT, MRI)  
**3D Models**: STL (surface tessellation) or OBJ (mesh with textures)  
**Segmentation**: Labeled anatomical structures using SNOMED CT terminology  
**Registration Fiducials**: Minimum 3 non-collinear points, accuracy ±2mm

### 3.4 Force/Torque Data

**Sampling Rate**: 100-1000 Hz  
**Resolution**: 0.01 N (force), 0.001 Nm (torque)  
**Range**: 0-10 N (force), 0-0.5 Nm (torque)  
**Format**: JSON with SI units (Newtons, Newton-meters)

---

## 4. Phase 2: API Interface Specifications

### 4.1 REST API Endpoints

**Base URL**: `https://{robot-hostname}/api/v1`

**Authentication**: OAuth 2.0 Bearer Token  
**Encryption**: TLS 1.3 mandatory  
**Response Format**: JSON

#### Core Endpoints:

| Endpoint | Method | Purpose | Auth Required |
|----------|--------|---------|---------------|
| `/robot/status` | GET | Retrieve system status | Yes |
| `/robot/control` | POST | Send control commands | Yes (surgeon role) |
| `/instruments` | GET | List installed instruments | Yes |
| `/instruments/{id}/move` | POST | Command instrument movement | Yes (surgeon role) |
| `/video/stream` | WebSocket | Real-time video stream | Yes |
| `/telemetry` | WebSocket | Real-time telemetry stream | Yes |
| `/emergency/stop` | POST | Emergency stop | No (safety critical) |

### 4.2 WebSocket Specifications

**Protocol**: WSS (WebSocket Secure over TLS 1.3)  
**Frame Rate**: 30-60 Hz for telemetry, 30-60 FPS for video  
**Latency**: < 100 ms end-to-end  
**Heartbeat**: Every 1 second, disconnect if no response within 5 seconds

### 4.3 Authentication & Authorization

**OAuth 2.0 Flows**: Authorization Code Grant for user login  
**Token Lifetime**: Access token: 1 hour, Refresh token: 30 days  
**Roles**: Surgeon (full control), Assistant (limited control), Observer (read-only), Technician (diagnostics)

---

## 5. Phase 3: Protocol Specifications

### 5.1 Workflow State Machine

**States**:
1. SETUP: Robot positioning and draping
2. INSTRUMENT_INSTALL: Installing surgical instruments
3. SYSTEM_CHECK: Pre-operative verification
4. TIMEOUT: Surgical safety timeout (WHO checklist)
5. ACTIVE: Surgical procedure in progress
6. CLOSURE: Closing incisions
7. INSTRUMENT_REMOVAL: Removing instruments
8. SIGN_OUT: Post-operative sign-out
9. SHUTDOWN: System shutdown
10. EMERGENCY: Emergency stop state (can interrupt any state)

**Transitions**: Must be logged with timestamp, operator ID, and reason

### 5.2 Safety Interlocks

**Hardware Interlocks** (mandatory):
- Foot pedal enabling (instruments move only when pedal depressed)
- Emergency stop buttons (console, patient-side cart, OR walls)
- Dead-man switch (console hand presence detection)
- Mechanical brakes (engage on power loss)

**Software Interlocks** (mandatory):
- Collision detection (halt motion if arm proximity < 50mm)
- Force limit (restrict torque if applied force > 10N)
- Velocity limit (cap speed at 100 mm/s)
- Workspace boundaries (prevent exit from defined safe volume)
- Communication timeout (safe state if telemetry loss > 100ms)

### 5.3 Emergency Stop Protocol

**Response Time**: < 100ms from button press to motor cut-off  
**Behavior**: 
1. Cut power to all motors
2. Engage mechanical brakes
3. Maintain current instrument positions
4. Visual and audible alarms throughout OR
5. Log event with timestamp and triggering condition
6. Require manual reset after fault resolution

---

## 6. Phase 4: Integration Specifications

### 6.1 OR System Integration

**Standard**: HL7 FHIR R4  
**Resources**: Patient, Practitioner, Encounter, Procedure, Observation, DiagnosticReport

**Required Integrations**:
- OR scheduling systems
- Equipment tracking
- Supply chain management
- Staff coordination

### 6.2 Imaging Integration

**Pre-operative**: DICOM retrieve from PACS  
**Intra-operative**: Real-time DICOM feed from C-arm, ultrasound, fluoroscopy  
**Post-operative**: Video archival to PACS in DICOM-wrapped format

### 6.3 Anesthesia Monitoring

**Standard**: IEEE 11073 (Personal Health Device Communication)  
**Parameters**: Heart rate, blood pressure, SpO2, EtCO2, temperature  
**Alert Thresholds**: Configurable per institution, transmitted to robot console

### 6.4 IoT Device Communication

**Protocol**: MQTT (Message Queuing Telemetry Transport)  
**Devices**: Surgical lights, insufflators, electrocautery, warming devices  
**QoS**: Level 1 (at least once delivery)

---

## 7. Safety Requirements

### 7.1 Risk Classification (ISO 14971)

Surgical robots are Class III medical devices requiring comprehensive risk management:

**Hazard Categories**:
- Mechanical hazards (collision, pinching, crushing)
- Electrical hazards (shock, fire)
- Thermal hazards (overheating)
- Radiation hazards (imaging systems)
- Software hazards (erroneous output, data corruption)

**Risk Mitigation**: Hazard analysis, FMEA, fault tree analysis, residual risk evaluation

### 7.2 Electrical Safety (IEC 60601)

- Type CF applied parts (cardiac-protected patient connection)
- Leakage current: < 10 μA normal, < 50 μA single fault condition
- EMC compliance: IEC 60601-1-2
- Grounding: Protective earth required

### 7.3 Cybersecurity

**Data Encryption**:
- At rest: AES-256
- In transit: TLS 1.3
- Video: SRTP (Secure Real-time Transport Protocol)

**Access Control**:
- Multi-factor authentication for administrative access
- Role-based access control (RBAC)
- Audit logging of all access attempts

---

## 8. Performance Requirements

### 8.1 Motion Control

**Position Accuracy**: ± 1.0 mm  
**Position Repeatability**: ± 0.5 mm  
**Orientation Accuracy**: ± 2.0 degrees  
**Maximum Velocity**: 100 mm/s (software limited)  
**Maximum Force**: 10 N (software limited)  
**Tremor Reduction**: > 90% at 5-12 Hz

### 8.2 Vision System

**Resolution**: Minimum 1920x1080, Preferred 3840x2160  
**Frame Rate**: 30-60 FPS  
**Depth Perception**: Stereoscopic 3D with adjustable IPD  
**Magnification**: 10-15x optical  
**Illumination**: LED white light + optional fluorescence (ICG, 5-ALA)

### 8.3 Latency Budgets

**Total end-to-end latency** (surgeon hand motion to instrument motion):
- **Target**: < 50 ms
- **Maximum acceptable**: < 100 ms

**Component breakdown**:
- Master manipulator sensing: < 10 ms
- Control computation: < 20 ms
- Network transmission: < 20 ms
- Slave actuator response: < 10 ms
- Visual feedback: < 50 ms (30 Hz video)

---

## 9. Testing and Validation

### 9.1 Data Format Validation

**Test Method**: Automated JSON schema validator  
**Pass Criteria**: 100% compliance with WIA-ROB-006 schemas  
**Sample Size**: Minimum 1000 telemetry messages per session

### 9.2 API Conformance Testing

**Functional Tests**: Each endpoint tested with valid and invalid inputs  
**Performance Tests**: Latency < 100ms, throughput > 60 requests/second  
**Security Tests**: Penetration testing, authentication bypass attempts  
**Load Tests**: 10 concurrent users, 24-hour continuous operation

### 9.3 Safety Validation

**Emergency Stop Test**: 100 trials, all must achieve < 100ms response  
**Collision Avoidance**: 50 intentional collision scenarios, all prevented  
**Force Limiting**: Apply excessive force, verify torque limitation to < 10N  
**Power Failure**: 20 trials during active motion, all achieve safe state

### 9.4 Clinical Validation

**Minimum Requirements**:
- 20-50 clinical procedures at WIA-certified sites
- Diverse procedure types and patient populations
- Outcome metrics: operative time, blood loss, complications, conversion rate
- User feedback: surgeon questionnaires, OR team interviews

---

## 10. Certification Levels

### 10.1 Level 1 (Core)

**Requirements**:
- Phase 1: Data Format compliance
- Basic safety interlocks (E-stop, force limit)
- Documented safety procedures

**Testing**:
- Data format validation
- Safety protocol verification
- Basic performance benchmarks

**Certification Cost**: ~$140,000 over 3 years

### 10.2 Level 2 (Advanced)

**Requirements**:
- Level 1 + Phase 2: API Interface
- Enhanced safety features (collision avoidance, workspace boundaries)
- Real-time telemetry streaming

**Testing**:
- API conformance testing
- Interoperability validation
- Extended safety validation

**Certification Cost**: ~$240,000 over 3 years

### 10.3 Level 3 (Complete)

**Requirements**:
- Levels 1-2 + Phases 3-4: Protocol and Integration
- Full OR system integration
- Advanced features (haptics, AR, autonomous functions if applicable)

**Testing**:
- Comprehensive workflow validation
- EMR/PACS integration testing
- Clinical validation (20-50 cases)

**Certification Cost**: ~$350,000 over 3 years

---

## 11. Regulatory Compliance

### 11.1 FDA (United States)

**Class II (510k)**: WIA-ROB-006 Level 2 supports substantial equivalence  
**Class III (PMA)**: WIA-ROB-006 Level 3 provides standardized data for clinical trials  
**Cybersecurity**: Compliance with FDA pre-market and post-market guidance

### 11.2 CE Mark (European Union)

**MDR Compliance**: Medical Device Regulation (EU) 2017/745  
**Notified Body**: WIA certification documentation accepted by major notified bodies  
**Clinical Evaluation**: Standardized data supports clinical evaluation reports

### 11.3 PMDA (Japan)

**Pre-Market Approval**: WIA test protocols aligned with PMDA requirements  
**Data Formats**: Compatible with Japanese regulatory reporting systems

### 11.4 NMPA (China)

**Registration**: WIA certification demonstrates international standards compliance  
**Clinical Trials**: Standardized data collection for Chinese clinical requirements

---

## 12. Post-Market Surveillance

### 12.1 Ongoing Monitoring

**Required Reporting** (to WIA and regulators):
- Adverse events (patient harm)
- Technical failures (E-stop activations, malfunctions)
- Software updates (major revisions)
- Field actions (recalls, safety notices)

**Frequency**: Quarterly reports + immediate notification of serious adverse events

### 12.2 Performance Metrics

**Mandatory Tracking**:
- Operative time (setup, console, total)
- Blood loss (estimated volume)
- Complications (intra-operative, post-operative)
- Conversion rate (to open surgery)
- Technical failure rate
- E-stop activation frequency

---

## 13. Conformance Statement

Manufacturers claiming WIA-ROB-006 compliance must provide:

1. **Conformance Matrix**: Spreadsheet mapping each requirement to implementation
2. **Test Reports**: Results from validation testing
3. **User Documentation**: Manuals, training materials, quick reference guides
4. **Technical Documentation**: Architecture diagrams, data dictionaries, API specifications
5. **Clinical Evidence**: Procedure reports, outcome data, user feedback

---

## 14. Future Roadmap

### 14.1 Version 1.5 (Target: 2027)
- Haptic feedback standardization
- AI/ML integration guidelines
- 5G remote surgery protocols

### 14.2 Version 2.0 (Target: 2030)
- Autonomous surgery frameworks
- Micro-robotics specifications
- Next-generation imaging modalities

### 14.3 Version 3.0 (Target: 2035)
- Emerging paradigms TBD based on technological advances

---

## Appendix A: Acronyms and Abbreviations

- API: Application Programming Interface
- DICOM: Digital Imaging and Communications in Medicine
- EMR: Electronic Medical Record
- FHIR: Fast Healthcare Interoperability Resources
- FMEA: Failure Modes and Effects Analysis
- HL7: Health Level Seven
- ICG: Indocyanine Green
- IEEE: Institute of Electrical and Electronics Engineers
- IoT: Internet of Things
- JSON: JavaScript Object Notation
- MQTT: Message Queuing Telemetry Transport
- OAuth: Open Authorization
- OR: Operating Room
- PACS: Picture Archiving and Communication System
- REST: Representational State Transfer
- STL: Stereolithography (file format)
- TLS: Transport Layer Security
- UUID: Universally Unique Identifier
- WebRTC: Web Real-Time Communication
- WSS: WebSocket Secure

---

## Appendix B: Contact Information

**World Certification Industry Association (WIA)**  
Standards Department  
Email: rob-006@wia-official.org  
Web: https://wia-official.org/standards/rob-006

**Technical Committee**  
Email: rob-committee@wia-official.org

**Certification Inquiries**  
Email: certification@wia-official.org

---

**Document Version**: 1.0  
**Publication Date**: January 2025  
**Next Review**: January 2028  

**© 2025 World Certification Industry Association (WIA)**

**弘益人間 · Benefit All Humanity**
