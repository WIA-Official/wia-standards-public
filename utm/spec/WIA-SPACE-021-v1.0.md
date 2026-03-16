# WIA-SPACE-021: UTM (Unmanned Aircraft System Traffic Management)

**Version:** 1.0
**Status:** Published
**Date:** 2025-12-26
**Category:** Space & Aviation Standards

---

## Abstract

This standard defines the requirements, architecture, protocols, and operational procedures for Unmanned Aircraft System Traffic Management (UTM). UTM enables safe, efficient, and scalable integration of drones into the national airspace system by providing real-time traffic management, conflict resolution, and regulatory compliance for low-altitude drone operations.

---

## 1. Scope

### 1.1 Purpose
WIA-SPACE-021 establishes a comprehensive framework for UTM systems that:
- Manage drone traffic in low-altitude airspace (typically below 400 feet AGL)
- Enable safe coexistence of manned and unmanned aircraft
- Support diverse operational scenarios from recreational to commercial to emergency operations
- Provide interoperability across manufacturers, operators, and jurisdictions

### 1.2 Applicability
This standard applies to:
- UTM system developers and operators
- USS (UAS Service Supplier) providers
- FIMS (Flight Information Management System) operators
- Drone manufacturers
- Drone operators (commercial, public safety, recreational)
- Regulatory authorities
- Airspace managers

---

## 2. Normative References

- **ASTM F3411:** Standard Specification for Remote ID and Tracking
- **ASTM F3548:** Standard Specification for USS-to-USS Interoperability
- **RTCA DO-365:** Minimum Operational Performance Standards for Detect and Avoid
- **RTCA DO-377:** Minimum Operational Performance Standards for UAS
- **ISO 21384:** UAS General Requirements
- **NASA UTM Framework:** Technical Capability Levels 1-4
- **ICAO Annex 2:** Rules of the Air
- **U-space Regulation (EU) 2021/664**

---

## 3. Terms and Definitions

**UTM (Unmanned Aircraft System Traffic Management):** A system to manage drone traffic safely and efficiently.

**USS (UAS Service Supplier):** Commercial service provider offering UTM services to drone operators.

**FIMS (Flight Information Management System):** Government-operated interface providing airspace information and regulatory compliance.

**PSU (Public Safety USS):** Specialized USS for emergency and law enforcement operations.

**Remote ID:** Real-time broadcast of drone identification and position information.

**LAANC (Low Altitude Authorization and Notification Capability):** Automated system for airspace authorization.

**DAA (Detect and Avoid):** System to detect potential collisions and execute avoidance maneuvers.

**Geofencing:** Virtual geographic boundaries that restrict or prevent drone operations.

**Operation Volume:** 4D space-time volume reserved for a drone operation.

**Well Clear:** Minimum separation distance required between aircraft to ensure safety.

---

## 4. UTM Architecture

### 4.1 System Components

#### 4.1.1 FIMS (Flight Information Management System)
**Requirements:**
- SHALL provide authoritative airspace information including NFZs, TFRs, and UAS facility maps
- SHALL approve or deny operation requests based on regulatory compliance
- SHALL integrate with ATM systems to coordinate manned/unmanned operations
- SHALL maintain 99.9% availability
- SHALL log all transactions for audit purposes

**Data Interfaces:**
- Airspace constraints (output to USS)
- Flight authorizations (bidirectional with USS)
- Real-time telemetry (input from USS)
- Emergency notifications (output to USS/PSU)

#### 4.1.2 USS (UAS Service Supplier)
**Requirements:**
- SHALL be certified by aviation authority
- SHALL implement USS-to-USS communication per ASTM F3548
- SHALL validate flight plans for safety and regulatory compliance
- SHALL track drone positions in real-time
- SHALL coordinate with other USS to prevent conflicts
- SHALL support at least 10,000 concurrent operations

**Core Functions:**
- Flight planning and authorization
- Real-time position tracking
- Conflict detection and resolution
- Weather information provision
- Geofencing enforcement
- Emergency management

#### 4.1.3 PSU (Public Safety USS)
**Requirements:**
- SHALL have priority override capability
- SHALL coordinate with law enforcement and emergency services
- SHALL enable immediate airspace closure for emergencies
- SHALL force-land non-compliant drones when authorized

#### 4.1.4 Drone Systems
**Requirements:**
- SHALL broadcast Remote ID per ASTM F3411
- SHALL transmit position updates every 1 second
- SHALL comply with geofencing boundaries
- SHALL implement return-to-home (RTH) for lost link
- SHALL log flight data for post-flight analysis

### 4.2 Communication Protocols

#### 4.2.1 USS-FIMS Interface
**Protocol:** RESTful API over HTTPS with TLS 1.3
**Data Format:** JSON/GeoJSON
**Authentication:** OAuth 2.0 with client credentials

**Endpoints:**
```
POST   /operations          - Submit operation plan
GET    /operations/{id}     - Query operation status
PUT    /operations/{id}/position - Update position
DELETE /operations/{id}     - Cancel operation
GET    /constraints         - Get airspace constraints
```

#### 4.2.2 USS-USS Interface (U2U)
**Standard:** ASTM F3548
**Messages:**
- Operation Intent: Flight plan sharing
- Operational Status: Real-time position and state
- Constraint Information: Airspace restrictions
- Conformance Monitoring: Compliance alerts

#### 4.2.3 Remote ID
**Standard:** ASTM F3411
**Broadcast Methods:**
- Wi-Fi Beacon (802.11) - range ~1 km
- Bluetooth 5 Long Range - range ~1 km
- Network (cellular/satellite) - unlimited range

**Required Data Fields:**
- UAS Serial Number or Session ID
- Operator Location (lat/lon/alt)
- UAS Position (lat/lon/alt)
- UAS Velocity and Track
- Timestamp (UTC)

---

## 5. Operational Procedures

### 5.1 Flight Planning
1. Operator submits flight plan to USS including:
   - Operation volumes (4D polygons)
   - Drone identification
   - Operator credentials
   - Priority level
2. USS validates against airspace constraints, weather, terrain
3. USS coordinates with other USS to deconflict
4. USS submits to FIMS for regulatory approval
5. FIMS grants authorization or provides denial with reason

### 5.2 In-Flight Operations
1. Drone broadcasts Remote ID continuously
2. USS tracks position and compares to authorized volume
3. USS monitors for conflicts with other operations
4. USS alerts operator of deviations or hazards
5. USS reports conformance to FIMS

### 5.3 Emergency Procedures
**Communication Loss:**
- Attempt reconnection for 30 seconds
- Initiate Return to Home (RTH)
- Land safely if RTH not possible

**Airspace Closure:**
- USS broadcasts emergency closure notification
- All drones exit affected airspace within 5 minutes
- PSU drones given immediate access

**System Failure:**
- Fallback to manual control if available
- Execute pre-programmed safe landing
- Notify USS of emergency

---

## 6. Safety Requirements

### 6.1 Detect and Avoid (DAA)
**Sensor Requirements:**
- Detection range: 500m minimum (drone-drone), 2km (manned aircraft)
- Update rate: ≥ 10 Hz
- False negative rate: < 0.001%
- Reaction time: < 1 second

**Well Clear Standards:**
- Horizontal: 100m (drone-drone), 500m (drone-aircraft)
- Vertical: 50m (drone-drone), 150m (drone-aircraft)
- Temporal: 30 seconds minimum separation

### 6.2 Geofencing
**Implementation:**
- Hard geofences SHALL prevent physical entry
- Soft geofences SHOULD warn but allow override with justification
- Dynamic geofences SHALL update within 10 seconds
- Altitude-based limits SHALL be enforced

**Common Geofenced Areas:**
- Airports: 5km radius minimum
- Critical infrastructure
- National borders
- Temporary event airspace

### 6.3 Risk Mitigation
**Ground Risk:**
- Population density assessment
- Overfly restrictions for crowds
- Ballistic descent zones

**Air Risk:**
- Collision probability calculation
- Proximity to manned aircraft routes
- Weather hazard assessment

---

## 7. Security & Privacy

### 7.1 Cybersecurity
**Requirements:**
- All communications encrypted with TLS 1.3 minimum
- Digital signatures for critical messages
- Multi-factor authentication for USS/FIMS access
- Penetration testing annually
- Incident response plan required

**Threat Protection:**
- GPS spoofing detection via multi-GNSS
- Anti-jamming measures
- DDoS mitigation
- Insider threat monitoring

### 7.2 Data Privacy
**Compliance:**
- GDPR (Europe)
- CCPA (California)
- Local privacy regulations

**Requirements:**
- Data minimization
- Purpose limitation
- Retention limits (90 days default)
- User consent management
- Right to erasure support

---

## 8. Performance Metrics

### 8.1 System Performance
- **Availability:** ≥ 99.9%
- **Latency:** Position updates < 1 second, approvals < 30 seconds
- **Capacity:** ≥ 10,000 operations per USS
- **Accuracy:** Position error < 5m horizontal, < 10m vertical

### 8.2 Safety Metrics
- **Collision Rate:** < 1 per million flight hours
- **Incident Response Time:** < 5 minutes for emergencies
- **Compliance Rate:** > 99% of flights within authorized volumes

---

## 9. Testing & Certification

### 9.1 USS Certification
1. Functional testing of all interfaces
2. Security audit and penetration testing
3. Load testing (10,000+ concurrent operations)
4. Interoperability testing with other USS
5. Disaster recovery testing
6. Regulatory compliance review

### 9.2 Drone Certification
1. Remote ID compliance testing (ASTM F3411)
2. Geofencing validation
3. DAA system testing (if equipped)
4. Communication reliability testing
5. Failsafe behavior verification

---

## 10. Conformance

Systems claiming conformance to WIA-SPACE-021 SHALL:
1. Implement all "SHALL" requirements
2. Document implementation of "SHOULD" requirements or provide rationale for deviation
3. Pass certification testing
4. Maintain operational logs for 1 year minimum
5. Report incidents within 24 hours
6. Participate in periodic audits

---

## Appendix A: Sample API Specifications

See ebook chapters for detailed API examples and message formats.

---

## Appendix B: Regulatory Mapping

- **USA:** Compliant with FAA Part 107, Part 89 (Remote ID), LAANC
- **Europe:** Aligned with U-space Regulation (EU) 2021/664, EASA regulations
- **ICAO:** Compatible with ICAO RPAS framework

---

## Appendix C: Future Extensions

This standard will be extended to cover:
- Urban Air Mobility (UAM) / eVTOL operations
- Beyond Visual Line of Sight (BVLOS) at scale
- Autonomous swarm operations
- Space-based UTM for high-altitude operations

---

**Document Control:**
© 2025 SmileStory Inc. / WIA
**License:** Open Standard - Free to implement
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

**For questions or contributions:**
WIA Standards Committee
https://github.com/WIA-Official/wia-standards
