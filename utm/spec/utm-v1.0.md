# WIA-SPACE-021: UTM - Unmanned Traffic Management
## Technical Specification v1.0

**Standard ID:** WIA-SPACE-021
**Title:** UTM (Unmanned Traffic Management / 무인기 교통 관리)
**Version:** 1.0
**Status:** Published
**Date:** 2025-01-15
**Organization:** WIA (World Certification Industry Association)
**Category:** SPACE (Aerospace & Aviation Standards)

---

## Philosophy

**弘益人間 (홍익인간) - Benefit All Humanity**

This standard embodies the principle of benefiting all humanity by creating accessible, safe, and equitable systems for unmanned aircraft operations. UTM democratizes airspace access, ensuring that innovators worldwide—from small startups to large corporations, from developed to developing nations—can safely operate drones for societal benefit.

---

## 1. Executive Summary

### 1.1 Purpose
This specification defines the technical requirements, architecture, protocols, and operational procedures for Unmanned Traffic Management (UTM) systems. UTM enables safe, efficient, and scalable integration of unmanned aircraft systems (UAS) into low-altitude airspace shared with manned aviation.

### 1.2 Scope
This standard applies to:
- UTM Service Suppliers (USS) providing traffic management services
- UAS operators conducting commercial and recreational operations
- Drone manufacturers implementing Remote ID and geofencing
- Aviation authorities implementing UTM frameworks
- Technology providers developing C2 links, DAA systems, and related infrastructure

### 1.3 Key Benefits
- **Scalability:** Support thousands of simultaneous low-altitude operations
- **Safety:** Automated deconfliction and conformance monitoring
- **Efficiency:** Instant authorization replacing manual review processes
- **Accessibility:** Competitive marketplace ensuring equitable access
- **Interoperability:** Global standards enabling cross-border operations

---

## 2. Architecture Overview

### 2.1 Three-Layer Model

#### Layer 1: UAS Operations
- Unmanned aircraft systems (drones)
- Remote pilots / operators
- Ground control stations
- Payload systems

#### Layer 2: UTM Services (USS Network)
- Competitive USS providers delivering services
- Flight planning and optimization
- Strategic deconfliction
- Conformance monitoring
- Traffic awareness
- Emergency coordination

#### Layer 3: Government Services (FIMS)
- Flight Information Management System
- Airspace constraint distribution
- Authorization gateway
- ATC interface
- USS registry and discovery

### 2.2 System Components

```
┌─────────────────────────────────────────────────────────┐
│                   FIMS (Government)                      │
│  • Constraints  • Authorization  • USS Discovery        │
└────────────────────┬────────────────────────────────────┘
                     │
        ┌────────────┴────────────┐
        │                         │
┌───────▼──────┐         ┌────────▼───────┐
│   USS-A      │◄────────►│    USS-B       │
│  Services    │   F3548  │   Services     │
└──────┬───────┘          └────────┬───────┘
       │                           │
  ┌────▼─────┐               ┌─────▼────┐
  │ Drone 1  │               │ Drone 2  │
  │ (Remote  │               │ (Remote  │
  │  ID)     │               │  ID)     │
  └──────────┘               └──────────┘
```

---

## 3. Remote ID Requirements (ASTM F3411)

### 3.1 Message Types

#### Basic ID Message
- UAS Serial Number or Session ID
- UA Type (multirotor, fixed-wing, VTOL, etc.)
- Broadcast frequency: ≥ 1 Hz

#### Location/Vector Message
- Latitude, Longitude (WGS84)
- Geodetic Altitude (MSL)
- Height above takeoff (AGL)
- Horizontal/Vertical speed
- Track direction
- Timestamp (UTC)
- Position accuracy category
- Broadcast frequency: ≥ 1 Hz

#### System Message
- Operator Location (lat/lon)
- Area Count (multiple aircraft under one operator)
- Operational Status
- Timestamp
- Broadcast frequency: ≥ 0.2 Hz

#### Authentication Message
- Digital signature
- Authentication type
- Authentication data
- Broadcast frequency: ≥ 0.2 Hz

### 3.2 Broadcast Methods

**Standard Remote ID:**
- Integrated into drone at manufacture
- WiFi (2.4 GHz or 5.8 GHz) or Bluetooth 5.0+
- Range: 400+ meters typical

**Remote ID Broadcast Module:**
- External device for retrofit
- Same broadcast requirements as standard
- Must activate with drone power-on

**Network Remote ID:**
- Internet-based reporting via cellular/satellite
- Backup/supplement to broadcast
- API access for authorized users

### 3.3 Accuracy Requirements

| Parameter | Requirement |
|-----------|-------------|
| Horizontal Position | ≤ 30 meters (95% confidence) |
| Vertical Position | ≤ 15 meters (95% confidence) |
| Velocity | ≤ 3 m/s |
| Timestamp | ≤ 1 second UTC |
| Update Rate | ≥ 1 Hz for position data |

---

## 4. USS Requirements (ASTM F3548)

### 4.1 Core Services

**Strategic Deconfliction:**
- Compare operation requests against all accepted operations
- Identify 4D conflicts (3D space + time)
- Resolve conflicts before authorization

**Flight Authorization:**
- Submit requests to FIMS
- Process approvals/denials
- Manage authorization lifecycle

**Conformance Monitoring:**
- Track actual vs. planned flight paths
- Alert on deviations
- Trigger contingency procedures

**USS-to-USS Coordination:**
- Operation intent exchange
- Position report sharing
- Conflict notification
- Emergency coordination

### 4.2 Technical Requirements

| Requirement | Specification |
|-------------|---------------|
| Availability | ≥ 99.9% uptime |
| Position Update Rate | ≥ 1 Hz |
| Alert Latency | ≤ 2 seconds end-to-end |
| Data Encryption | TLS 1.3+ for all communications |
| Authentication | OAuth 2.0 or equivalent |
| API Standards | ASTM F3548 compliance |
| Data Retention | Minimum 90 days for operations |

### 4.3 Data Exchange Format

```json
{
  "operation_id": "uuid-string",
  "uss_base_url": "https://uss.example.com",
  "state": "Accepted | Activated | Ended",
  "volumes": [
    {
      "volume": {
        "outline_polygon": {
          "vertices": [[lat, lng], [lat, lng], ...]
        },
        "altitude_lower": {
          "value": 50,
          "reference": "W84",
          "units": "M"
        },
        "altitude_upper": {
          "value": 120,
          "reference": "W84",
          "units": "M"
        }
      },
      "time_start": "2025-01-15T14:00:00Z",
      "time_end": "2025-01-15T14:30:00Z"
    }
  ],
  "priority": 0,
  "off_nominal_volumes": [],
  "uas_state": "Nominal | Contingent | NonConforming"
}
```

---

## 5. LAANC (Low Altitude Authorization and Notification Capability)

### 5.1 Authorization Workflow

1. **Operator Request:** Submit via USS application
   - Location (lat/lon bounding box)
   - Maximum altitude AGL
   - Time window (start/end)
   - Drone registration & operator credentials

2. **USS Validation:**
   - Check USFM altitude ceiling
   - Verify no TFRs/NOTAMs
   - Confirm operator credentials
   - Submit to LAANC if valid

3. **LAANC Processing:**
   - Automated compliance checking
   - USFM ceiling comparison
   - ATC coordination (if required)
   - Instant decision (typically < 5 seconds)

4. **Authorization Delivery:**
   - Approval/denial notification
   - Authorization number
   - Approved volume details
   - Special conditions/limitations

### 5.2 USFM (UAS Facility Maps)

Grid-based representation of altitude ceilings near airports:

| Zone Type | Typical Ceiling | Authorization |
|-----------|----------------|---------------|
| Surface Area (0) | 0 ft | Manual review required |
| Surface Area (50) | 50 ft AGL | Instant automated |
| Transition | 100-200 ft AGL | Instant automated |
| Outer | 200-400 ft AGL | Instant automated |
| Beyond Controlled | 400 ft AGL | Notification only |

---

## 6. Detect and Avoid (DAA) Systems

### 6.1 Performance Requirements (RTCA SC-228)

| Parameter | Requirement |
|-----------|-------------|
| Detection Range | ≥ 2 NM (3.7 km) for aircraft |
| Detection Probability | > 90% for relevant traffic |
| False Alarm Rate | < 1 per hour |
| Altitude Accuracy | ± 100 ft |
| Update Rate | ≥ 1 Hz |
| System Latency | < 2 seconds end-to-end |

### 6.2 Sensor Technologies

**Cooperative Detection:**
- ADS-B In receivers
- Mode C/S transponders
- UTM traffic data

**Non-Cooperative Detection:**
- Radar (pulse-Doppler, FMCW)
- Electro-optical cameras
- Infrared sensors
- Acoustic detection

**Sensor Fusion:**
- Kalman filtering
- Track association
- Confidence scoring
- Threat prioritization

### 6.3 Well Clear Definition

**Separation Standards:**
- Horizontal: ≥ 2000 ft (609 m)
- Vertical: ≥ 250 ft (76 m)
- Time-based: ≥ 35 seconds to closest point of approach

---

## 7. Communication Networks (C2 Links)

### 7.1 Performance Requirements

| Operation Type | Max Latency | Success Rate | Availability | Redundancy |
|----------------|-------------|--------------|--------------|------------|
| VLOS | < 1 second | > 95% | > 95% | Optional |
| BVLOS | < 300 ms | > 99% | > 99.9% | Required |

### 7.2 Technology Options

**Cellular (4G/5G):**
- Wide area coverage
- 5G URLLC: < 10 ms latency
- Network slicing for guaranteed QoS

**Dedicated RF:**
- 2.4 GHz, 5.8 GHz ISM bands
- Licensed spectrum for critical operations
- Direct link, no infrastructure dependency

**Satellite:**
- Global coverage
- LEO constellations: 25-50 ms latency
- Backup/supplement to terrestrial

### 7.3 Security Requirements

- Encryption: AES-256 or equivalent
- Authentication: Certificate-based (X.509)
- Anti-jamming: Frequency hopping, spread spectrum
- Lost link procedures: RTH, land, orbit, continue mission

---

## 8. Airspace Integration

### 8.1 Airspace Classification

| Class | Characteristics | UAS Access | UTM Role |
|-------|----------------|------------|----------|
| G | Uncontrolled | Part 107 allowed | Primary UTM domain; USS coordination |
| E | Controlled | Below 400 ft allowed | Notifications; conformance monitoring |
| D | Tower airports | LAANC required | Automated authorization |
| C | Busy airports | LAANC required | Instant approval for compliant ops |
| B | Major airports | LAANC required | Altitude limits; real-time coordination |

### 8.2 Geofencing

**Types:**
- Regulatory: Permanent no-fly zones (airports, restricted areas)
- Temporary: TFRs, emergency operations
- Operational: Self-imposed boundaries for missions

**Implementation:**
- GeoJSON polygon format
- WGS84 coordinate system
- 3D volumes (horizontal + altitude limits)
- Time validity

---

## 9. Global Standards Alignment

### 9.1 Standards Organizations

| Organization | Contribution |
|--------------|--------------|
| ICAO | Global framework, SARPs |
| FAA | UTM framework, LAANC, Remote ID |
| EASA | U-Space, European regulations |
| ASTM | F3411 (Remote ID), F3548 (USS) |
| RTCA | SC-228 (DAA MOPS) |
| JARUS | SORA risk assessment |

### 9.2 International Interoperability

- Mutual recognition of USS certification
- Harmonized Remote ID formats
- Cross-border operation protocols
- Standardized data exchange (ASTM F3548)

---

## 10. Implementation Requirements

### 10.1 For USS Providers

**Mandatory:**
- ASTM F3548 API compliance
- FIMS integration
- 99.9% uptime SLA
- Security certification
- Data retention (90+ days)

**Recommended:**
- Multi-region coverage
- Advanced analytics
- Fleet management features
- API access for custom integrations

### 10.2 For Drone Manufacturers

**Mandatory:**
- Remote ID (ASTM F3411) compliance
- Geofencing capability
- Standard data interfaces
- Lost link procedures

**Recommended:**
- ADS-B In receiver
- DAA sensor integration
- 5G connectivity
- Cloud telemetry

### 10.3 For Operators

**Mandatory:**
- USS registration
- Remote ID active on all flights
- LAANC authorization (controlled airspace)
- Part 107 or equivalent certification

**Recommended:**
- Flight logging
- Pre-flight NOTAM/weather checks
- Emergency procedures training
- Insurance coverage

---

## 11. Compliance and Certification

### 11.1 USS Certification Process

1. Application & Documentation
2. Lab Testing (API compliance)
3. Interoperability Testing (with other USS)
4. Pilot Operations
5. Full Certification
6. Ongoing Audits

### 11.2 Drone Certification

- Remote ID compliance testing
- Geofencing validation
- C2 link performance verification
- DAA system validation (if applicable)

---

## 12. Future Evolution

### 12.1 Near-Term (2025-2027)

- LAANC global expansion
- Remote ID universal enforcement
- BVLOS routine approvals (low-risk)
- 5G network integration

### 12.2 Mid-Term (2027-2030)

- Urban Air Mobility (UAM) integration
- AI-driven traffic management
- Advanced autonomy (Level 4)
- International harmonization complete

### 12.3 Long-Term (2030-2035)

- Full autonomous operations (Level 5)
- Global UTM network
- Seamless manned/unmanned integration
- $100B+ global drone economy

---

## 13. References

### 13.1 Standards

- ASTM F3411-22: Standard Specification for Remote ID and Tracking
- ASTM F3548-21: Standard Specification for UAS Traffic Management (UTM) UAS Service Supplier (USS) Interoperability
- RTCA DO-365: Minimum Operational Performance Standards for Detect and Avoid Systems
- ICAO Annex 19: Safety Management

### 13.2 Regulatory

- FAA Part 107: Small Unmanned Aircraft Systems
- FAA UTM Framework v2.0
- EASA Easy Access Rules for Unmanned Aircraft Systems
- JARUS SORA (Specific Operations Risk Assessment)

### 13.3 Resources

- NASA UTM Research Transition Team
- FAA LAANC Information: https://www.faa.gov/laanc
- ASTM F38 Committee on Unmanned Aircraft Systems
- WIA Standards Portal: https://wia.org/standards

---

## Appendix A: Glossary

**ADS-B:** Automatic Dependent Surveillance-Broadcast
**AGL:** Above Ground Level
**ATC:** Air Traffic Control
**BVLOS:** Beyond Visual Line of Sight
**C2:** Command and Control
**DAA:** Detect and Avoid
**FIMS:** Flight Information Management System
**LAANC:** Low Altitude Authorization and Notification Capability
**MSL:** Mean Sea Level
**NOTAM:** Notice to Airmen
**TFR:** Temporary Flight Restriction
**UAS:** Unmanned Aircraft System
**USFM:** UAS Facility Map
**USS:** UTM Service Supplier
**UTM:** Unmanned Traffic Management
**VLOS:** Visual Line of Sight

---

## Document Control

**Version History:**

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 2025-01-15 | Initial release | WIA Standards Committee |

**Approval:**

This specification has been reviewed and approved by the WIA SPACE Standards Committee.

**Copyright:**

© 2025 WIA (World Certification Industry Association)

**License:**

This standard is published under Creative Commons Attribution 4.0 International (CC BY 4.0).
You are free to share and adapt with attribution.

---

**弘益人間 (홍익인간) · Benefit All Humanity**

This standard is dedicated to the principle that safe, accessible airspace benefits all of humanity. By creating open, interoperable systems, we enable innovation and services that improve lives worldwide—from medical deliveries in remote areas to efficient urban logistics, from agricultural optimization to emergency response. UTM is not just technology; it's a framework for democratizing the skies.

---

**End of Specification**
