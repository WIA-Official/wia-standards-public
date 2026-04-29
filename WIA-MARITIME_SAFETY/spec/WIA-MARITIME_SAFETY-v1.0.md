# WIA-MARITIME_SAFETY Standard Specification v1.0

**Version:** 1.0
**Status:** Official
**Published:** January 2026
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Executive Summary

The WIA-MARITIME_SAFETY standard provides a comprehensive framework for maritime safety operations, vessel tracking, emergency response, and compliance with international maritime regulations. This standard enables interoperability between vessel monitoring systems, port management platforms, coast guard networks, and meteorological services to enhance safety at sea.

### Key Features
- **Vessel Tracking**: Real-time AIS position reporting and monitoring
- **Emergency Response**: Integrated GMDSS distress alert and SAR coordination
- **Weather Routing**: Optimized route planning with meteorological data
- **Compliance**: SOLAS, MARPOL, and IMO regulation adherence
- **Port Integration**: Seamless port community system connectivity
- **Cyber Security**: Maritime cyber risk management framework

### Compliance Standards
- IMO SOLAS (Safety of Life at Sea)
- IMO MARPOL (Marine Pollution Prevention)
- IEC 61162 (Maritime Navigation Equipment)
- ITU-R M.1371 (AIS Technical Characteristics)
- ISO 19847 (Ships and Marine Technology)

---

## Table of Contents

1. [Data Format Specification](#1-data-format-specification)
2. [API Interface Specification](#2-api-interface-specification)
3. [Protocol Specification](#3-protocol-specification)
4. [Integration Specification](#4-integration-specification)
5. [Safety Management](#5-safety-management)
6. [Crew Training & Certification](#6-crew-training--certification)
7. [Implementation Guidelines](#7-implementation-guidelines)
8. [Appendices](#8-appendices)

---

## 1. Data Format Specification

### 1.1 Overview

All maritime safety data MUST follow standardized formats to ensure interoperability across vessel tracking, safety monitoring, and emergency response systems worldwide.

### 1.2 Core Data Structures

#### 1.2.1 Vessel Identity Record

```json
{
  "type": "WIA-MARITIME_SAFETY-VesselIdentity",
  "version": "1.0",
  "mmsi": "string (9 digits)",
  "imo": "string (7 digits)",
  "callSign": "string",
  "vesselName": "string",
  "vesselType": "string",
  "dimensions": {
    "length": "number (meters)",
    "beam": "number (meters)",
    "draft": "number (meters)",
    "height": "number (meters)"
  },
  "flag": "string (ISO 3166-1 alpha-2)",
  "grossTonnage": "number",
  "deadweight": "number",
  "yearBuilt": "number",
  "timestamp": "ISO 8601",
  "signature": "string (Ed25519)"
}
```

#### 1.2.2 AIS Position Report (Type 1, 2, 3)

```json
{
  "type": "WIA-MARITIME_SAFETY-AISPosition",
  "version": "1.0",
  "mmsi": "string (9 digits)",
  "timestamp": "ISO 8601",
  "position": {
    "latitude": "number (-90 to 90)",
    "longitude": "number (-180 to 180)",
    "accuracy": "boolean"
  },
  "navigation": {
    "status": "enum",
    "rateOfTurn": "number (degrees/min)",
    "speedOverGround": "number (knots)",
    "courseOverGround": "number (degrees)",
    "heading": "number (degrees)"
  },
  "maneuver": "enum",
  "raim": "boolean",
  "signature": "string (Ed25519)"
}
```

#### 1.2.3 Weather Data Format

```json
{
  "type": "WIA-MARITIME_SAFETY-Weather",
  "version": "1.0",
  "id": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "position": {
    "latitude": "number",
    "longitude": "number"
  },
  "conditions": {
    "windSpeed": "number (knots)",
    "windDirection": "number (degrees)",
    "waveHeight": "number (meters)",
    "wavePeriod": "number (seconds)",
    "visibility": "number (nautical miles)",
    "seaTemperature": "number (celsius)",
    "airTemperature": "number (celsius)",
    "barometricPressure": "number (hPa)"
  },
  "forecast": {
    "validUntil": "ISO 8601",
    "warnings": ["array of strings"]
  }
}
```

#### 1.2.4 Safety Alert Format

```json
{
  "type": "WIA-MARITIME_SAFETY-Alert",
  "version": "1.0",
  "id": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "severity": "enum (info|warning|critical|distress)",
  "category": "enum (collision|grounding|fire|flooding|medical|piracy|pollution)",
  "position": {
    "latitude": "number",
    "longitude": "number"
  },
  "vessel": {
    "mmsi": "string",
    "name": "string"
  },
  "description": "string",
  "status": "enum (active|acknowledged|resolved)",
  "responders": ["array of MMSI strings"]
}
```

#### 1.2.5 Cargo Manifest Data

```json
{
  "type": "WIA-MARITIME_SAFETY-CargoManifest",
  "version": "1.0",
  "vessel": {
    "mmsi": "string",
    "imo": "string"
  },
  "voyage": {
    "voyageNumber": "string",
    "from": "string (port code)",
    "to": "string (port code)"
  },
  "cargo": [
    {
      "containerNumber": "string",
      "cargoType": "string",
      "imdgClass": "string",
      "weight": "number (kg)",
      "hazmat": "boolean",
      "unNumber": "string"
    }
  ],
  "totals": {
    "containers": "number",
    "teu": "number",
    "hazmatContainers": "number"
  }
}
```

### 1.3 Field Definitions

#### 1.3.1 Vessel Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| mmsi | string | Yes | Maritime Mobile Service Identity (9 digits) |
| imo | string | No | International Maritime Organization number (7 digits) |
| callSign | string | Yes | Radio call sign |
| vesselName | string | Yes | Vessel name (max 20 chars) |
| vesselType | enum | Yes | See section 1.3.3 for types |
| flag | string | Yes | Flag state (ISO 3166-1 alpha-2) |

#### 1.3.2 Position Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| latitude | number | Yes | Latitude in decimal degrees (-90 to 90) |
| longitude | number | Yes | Longitude in decimal degrees (-180 to 180) |
| accuracy | boolean | Yes | Position accuracy: true = high (<10m), false = low (>10m) |
| speedOverGround | number | Yes | Speed in knots (0 to 102.2) |
| courseOverGround | number | Yes | Course in degrees (0 to 359.9) |
| heading | number | Yes | True heading in degrees (0 to 359) |

#### 1.3.3 Vessel Type Enumeration

| Code | Type | Description |
|------|------|-------------|
| 20-29 | Wing In Ground | WIG craft |
| 30 | Fishing | Fishing vessel |
| 31-32 | Towing | Towing or pushing |
| 33 | Dredging | Dredging or underwater operations |
| 34 | Diving | Diving operations |
| 35 | Military | Military operations |
| 36 | Sailing | Sailing vessel |
| 37 | Pleasure | Pleasure craft |
| 40-49 | High Speed | High-speed craft |
| 50 | Pilot | Pilot vessel |
| 51 | SAR | Search and rescue |
| 52 | Tug | Tug |
| 53 | Port Tender | Port tender |
| 60-69 | Passenger | Passenger ship |
| 70-79 | Cargo | Cargo ship |
| 80-89 | Tanker | Tanker |

#### 1.3.4 Navigation Status Enumeration

| Code | Status | Description |
|------|--------|-------------|
| 0 | Under way using engine | Powered navigation |
| 1 | At anchor | Anchored |
| 2 | Not under command | Unable to maneuver |
| 3 | Restricted maneuverability | Limited maneuverability |
| 4 | Constrained by draught | Deep draft restrictions |
| 5 | Moored | Moored to shore/dock |
| 6 | Aground | Run aground |
| 7 | Engaged in fishing | Fishing operations |
| 8 | Under way sailing | Sailing |
| 14 | AIS-SART | Search and Rescue Transmitter |
| 15 | Undefined | Status not defined |

### 1.4 Encoding Requirements

#### 1.4.1 Character Encoding
- All text MUST be UTF-8 encoded
- No BOM (Byte Order Mark) allowed
- Line endings: LF (Unix style)
- Vessel names: ASCII characters only (per AIS specification)

#### 1.4.2 Binary Encoding
- AIS messages: 6-bit ASCII encoding
- Position coordinates: Signed integers (latitude: 1/10000 minute precision)
- Time: UTC timezone
- Timestamps: ISO 8601 format with 'Z' suffix

#### 1.4.3 Precision Requirements

| Data Type | Precision | Format |
|-----------|-----------|--------|
| Latitude | ±0.0001 degrees | 4 decimal places |
| Longitude | ±0.0001 degrees | 4 decimal places |
| Speed | ±0.1 knots | 1 decimal place |
| Course | ±0.1 degrees | 1 decimal place |
| Distance | ±0.01 nautical miles | 2 decimal places |

### 1.5 Validation Rules

#### 1.5.1 Schema Validation

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["type", "version", "mmsi", "timestamp", "position"],
  "properties": {
    "type": {
      "type": "string",
      "enum": [
        "WIA-MARITIME_SAFETY-VesselIdentity",
        "WIA-MARITIME_SAFETY-AISPosition",
        "WIA-MARITIME_SAFETY-Weather",
        "WIA-MARITIME_SAFETY-Alert"
      ]
    },
    "version": { "type": "string", "pattern": "^\\d+\\.\\d+$" },
    "mmsi": { "type": "string", "pattern": "^[0-9]{9}$" },
    "timestamp": { "type": "string", "format": "date-time" },
    "position": {
      "type": "object",
      "required": ["latitude", "longitude"],
      "properties": {
        "latitude": { "type": "number", "minimum": -90, "maximum": 90 },
        "longitude": { "type": "number", "minimum": -180, "maximum": 180 }
      }
    }
  }
}
```

#### 1.5.2 Validation Errors

| Code | Message | Resolution |
|------|---------|------------|
| E001 | Invalid MMSI format | Must be exactly 9 digits |
| E002 | Invalid IMO format | Must be exactly 7 digits with valid checksum |
| E003 | Position out of range | Latitude: -90 to 90, Longitude: -180 to 180 |
| E004 | Invalid timestamp | Must be ISO 8601 format in UTC |
| E005 | Speed out of range | Must be 0 to 102.2 knots |
| E006 | Course out of range | Must be 0 to 359.9 degrees |
| E007 | Invalid vessel type | Must be valid AIS ship type code |

---

## 2. API Interface Specification

### 2.1 Base Configuration

#### 2.1.1 API Endpoints
```
Production: https://api.wia.org/maritime-safety/v1
Staging: https://staging-api.wia.org/maritime-safety/v1
Development: http://localhost:8080/api/v1
```

#### 2.1.2 Authentication

All API requests require authentication using API keys:

```http
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json
```

#### 2.1.3 Rate Limiting

| Tier | Requests/minute | Requests/day |
|------|----------------|--------------|
| Free | 60 | 10,000 |
| Standard | 600 | 100,000 |
| Enterprise | 6,000 | 1,000,000 |
| Emergency | Unlimited | Unlimited |

### 2.2 Vessel Tracking API

#### 2.2.1 Get Vessel Position

**Endpoint:** `GET /vessels/{mmsi}/position`

**Request:**
```http
GET /vessels/367123450/position HTTP/1.1
Host: api.wia.org
Authorization: Bearer YOUR_API_KEY
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "mmsi": "367123450",
    "vesselName": "PACIFIC GLORY",
    "timestamp": "2026-01-12T14:30:00Z",
    "position": {
      "latitude": 37.8044,
      "longitude": -122.4162,
      "accuracy": true
    },
    "navigation": {
      "status": "Under way using engine",
      "speedOverGround": 12.5,
      "courseOverGround": 285.3,
      "heading": 287
    }
  }
}
```

#### 2.2.2 Track Multiple Vessels

**Endpoint:** `POST /vessels/track`

**Request:**
```json
{
  "mmsiList": ["367123450", "431234567", "235012345"],
  "fields": ["position", "navigation", "identity"],
  "realtime": true
}
```

#### 2.2.3 Search Vessels by Area

**Endpoint:** `POST /vessels/search`

Supports circle, polygon, and rectangle area searches with filtering by vessel type, size, and flag state.

### 2.3 Weather & Environment API

#### 2.3.1 Get Marine Weather

**Endpoint:** `GET /weather`

Provides current conditions and forecast for specified coordinates with parameters including wind, waves, visibility, and temperature.

#### 2.3.2 Get Route Weather

**Endpoint:** `POST /weather/route`

Calculates weather conditions along planned route with recommendations for optimal departure time.

### 2.4 Safety Alert API

#### 2.4.1 Create Alert

**Endpoint:** `POST /alerts`

Creates safety alerts with automatic notification to nearby vessels and authorities.

#### 2.4.2 Get Active Alerts

**Endpoint:** `GET /alerts`

Retrieves active alerts with filtering by severity, category, and geographic area.

### 2.5 Error Handling

#### 2.5.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "INVALID_POSITION",
    "message": "Position coordinates out of valid range",
    "details": {
      "field": "latitude",
      "value": 95.0,
      "constraint": "Must be between -90 and 90"
    }
  },
  "timestamp": "2026-01-12T14:30:00Z"
}
```

---

## 3. Protocol Specification

### 3.1 AIS Protocol Implementation

#### 3.1.1 AIS Message Types

| Type | Name | Update Rate | Purpose |
|------|------|-------------|---------|
| 1, 2, 3 | Position Report | 2-10 seconds | Dynamic position information |
| 4 | Base Station Report | 10 seconds | Shore station position |
| 5 | Static and Voyage | 6 minutes | Vessel identity and voyage data |
| 18 | Class B Position | 30 seconds | Class B transponder position |
| 21 | Aid-to-Navigation | 3 minutes | Buoy, lighthouse positions |
| 24 | Static Data Report | 6 minutes | Class B vessel identity |

#### 3.1.2 Message Priority Levels

```typescript
enum MessagePriority {
  DISTRESS = 0,      // Immediate transmission
  URGENT = 1,        // High priority (safety)
  ROUTINE = 2,       // Normal operations
  BACKGROUND = 3     // Low priority (informational)
}
```

### 3.2 GMDSS Communication Protocol

#### 3.2.1 Distress Communication

**DSC Distress Call Format:**
```
FORMAT SPECIFIER: Distress
CATEGORY: Distress Alert
NATURE: Fire/Explosion/Flooding/Collision/Grounding/Listing/Sinking/
         Disabled/Undesignated/Abandoning Ship/Piracy/Man Overboard
POSITION: Latitude/Longitude
TIME: UTC
```

**Transmission Sequence:**
1. Send DSC distress alert (automatic)
2. Await acknowledgment (45 seconds)
3. If no ACK, retransmit
4. Begin voice mayday on VHF Ch 16

#### 3.2.2 VHF Channel Assignments

| Channel | Frequency (MHz) | Purpose |
|---------|----------------|---------|
| 16 | 156.800 | Distress, Safety, Calling |
| 6 | 156.300 | Inter-ship safety |
| 13 | 156.650 | Navigation (bridge-to-bridge) |
| 70 | 156.525 | Digital Selective Calling (DSC) |
| AIS1 | 161.975 | AIS channel A |
| AIS2 | 162.025 | AIS channel B |

### 3.3 Emergency Response Protocol

#### 3.3.1 Distress Signal Procedure

```typescript
interface DistressProtocol {
  steps: [
    {
      step: 1,
      action: 'Activate EPIRB/DSC distress button',
      automatic: true,
      timeout: 0
    },
    {
      step: 2,
      action: 'Send DSC distress alert',
      channel: 70,
      format: 'digital'
    },
    {
      step: 3,
      action: 'Transmit MAYDAY on VHF Ch 16',
      format: 'MAYDAY MAYDAY MAYDAY, This is [VESSEL], [POSITION], [NATURE]'
    },
    {
      step: 4,
      action: 'Activate AIS SART or EPIRB',
      automatic: true
    }
  ]
}
```

### 3.4 Collision Avoidance Protocol (COLREGS)

#### 3.4.1 COLREGS Rules Implementation

| Rule | Situation | Action Required |
|------|-----------|----------------|
| 13 | Overtaking | Give way to vessel being overtaken |
| 14 | Head-on | Both vessels alter course to starboard |
| 15 | Crossing | Vessel with other on starboard gives way |
| 16 | Give-way vessel | Take early and substantial action |
| 17 | Stand-on vessel | Maintain course and speed |
| 18 | Responsibilities | Power gives way to sail, sail to fishing, etc. |

---

## 4. Integration Specification

### 4.1 Port Management System Integration

#### 4.1.1 Port Community System (PCS) Interface

Enables integration with port management platforms for berth assignment, cargo declarations, and customs clearance.

#### 4.1.2 Vessel Traffic Service (VTS) Integration

Provides real-time vessel tracking data to VTS centers with position reporting and traffic advisories.

### 4.2 Coast Guard & SAR Integration

#### 4.2.1 Maritime Rescue Coordination Center (MRCC)

Automatic distress alert relay to MRCC with incident details and SAR asset coordination.

#### 4.2.2 AMVER Participation

Automated Mutual-Assistance Vessel Rescue position reporting for vessels available to assist in emergencies.

### 4.3 Meteorological Service Integration

#### 4.3.1 GMDSS Weather Broadcasts

Integration with NAVTEX and SafetyNET weather warning systems.

#### 4.3.2 WMO Data Exchange

Access to global meteorological models (GFS, ECMWF, JMA) for route planning.

---

## 5. Safety Management

### 5.1 International Safety Management (ISM) Code

#### 5.1.1 Safety Management System (SMS)

The SMS must include:
- Safety and environmental protection policy
- Instructions and procedures for safe vessel operation
- Defined levels of authority and communication
- Procedures for reporting accidents and non-conformities
- Procedures to prepare for and respond to emergencies
- Procedures for internal audits and management reviews

#### 5.1.2 Document of Compliance (DOC)

Issued to shipping companies demonstrating SMS implementation and maintenance.

#### 5.1.3 Safety Management Certificate (SMC)

Issued to individual vessels confirming SMS operation onboard.

### 5.2 SOLAS Requirements

#### 5.2.1 Chapter III - Life-Saving Appliances

- Lifeboats and life rafts capacity
- Lifebuoys and lifejackets
- Line-throwing appliances
- Emergency signals
- Operational readiness and maintenance

#### 5.2.2 Chapter IV - Radiocommunications

- GMDSS equipment requirements by sea area
- Satellite EPIRBs
- VHF DSC and SART
- Radio logs and watchkeeping

#### 5.2.3 Chapter V - Safety of Navigation

- Bridge design and equipment
- AIS and radar requirements
- ECDIS navigation
- Voyage planning
- Reporting requirements

### 5.3 MARPOL Environmental Compliance

#### 5.3.1 Annex I - Oil Pollution Prevention

- IOPP Certificate
- Oil Record Book
- SOPEP (Shipboard Oil Pollution Emergency Plan)
- Oily water separator requirements

#### 5.3.2 Annex VI - Air Pollution Prevention

- Engine EIAPP Certificate
- Fuel sulphur limits (0.5% global, 0.1% ECA)
- SEEMP (Ship Energy Efficiency Management Plan)
- Annual fuel consumption reporting

---

## 6. Crew Training & Certification

### 6.1 STCW Requirements

#### 6.1.1 Master and Deck Officers

**Certificates of Competency:**
- Master Unlimited
- Chief Mate Unlimited
- Second Mate Unlimited
- Officer in Charge of Navigational Watch

**Required Training:**
- Basic Safety Training
- Advanced Firefighting
- Medical First Aid
- Ship Security Officer
- GMDSS General Operator Certificate

#### 6.1.2 Engineering Officers

**Certificates of Competency:**
- Chief Engineer Unlimited
- Second Engineer Unlimited
- Officer in Charge of Engineering Watch

**Required Training:**
- Basic Safety Training
- Advanced Firefighting
- Medical First Aid
- STCW Engineering Competencies

### 6.2 Specialized Training

#### 6.2.1 ECDIS Training

- Generic ECDIS Training (40 hours minimum)
- Type-specific ECDIS Training (manufacturer specific)
- Practical assessment and certification

#### 6.2.2 BRM (Bridge Resource Management)

- Team coordination and communication
- Decision-making and problem-solving
- Situational awareness
- Leadership and team working

#### 6.2.3 Cargo Operations

- Tanker Training (Basic and Advanced)
- Dangerous Goods Training (IMDG Code)
- Container Securing and Lashing
- Ro-Ro Operations

---

## 7. Implementation Guidelines

### 7.1 System Requirements

#### 7.1.1 Hardware Requirements

**Bridge Navigation System:**
- Multi-core processor (4+ cores, 2.5GHz+)
- 16GB RAM minimum
- 500GB SSD storage
- Redundant power supply
- Environmental rating: IP65+
- Operating temperature: -15°C to +55°C

**Communication Equipment:**
- VHF DSC Radio (Class A)
- INMARSAT Fleet One or equivalent
- AIS Transponder (Class A)
- NAVTEX Receiver
- EPIRB (406 MHz with GPS)
- SART (AIS or Radar)

#### 7.1.2 Software Requirements

- Operating System: Linux-based or certified marine OS
- Database: PostgreSQL 14+ or equivalent
- Real-time OS for critical systems
- Automated backup systems
- Intrusion detection software

### 7.2 Installation Procedures

#### 7.2.1 Pre-Installation Checklist

1. Verify electrical power supply (110/220V AC, UPS backup)
2. Confirm network infrastructure (Ethernet, fiber optic)
3. Antenna installation locations (AIS, GPS, VSAT)
4. Bridge integration planning
5. Training schedule preparation

#### 7.2.2 Installation Steps

1. **Phase 1 - Hardware Installation** (Day 1-3)
   - Mount displays and control units
   - Install antennas and cabling
   - Connect power and network

2. **Phase 2 - Software Configuration** (Day 4-5)
   - Operating system installation
   - WIA-MARITIME_SAFETY system setup
   - Integration with existing systems
   - Database initialization

3. **Phase 3 - Testing & Commissioning** (Day 6-7)
   - System functionality tests
   - Communication equipment tests
   - Integration verification
   - Performance benchmarking

4. **Phase 4 - Training & Handover** (Day 8-10)
   - Crew training sessions
   - Documentation handover
   - Support contact establishment
   - Final acceptance testing

### 7.3 Maintenance Procedures

#### 7.3.1 Routine Maintenance Schedule

**Daily:**
- Visual inspection of displays and controls
- Verification of position accuracy
- Check communication systems status
- Review system logs for errors

**Weekly:**
- Backup verification
- Software update check
- Performance monitoring review
- Clean displays and equipment

**Monthly:**
- Full system backup
- Security patch application
- Antenna inspection
- Cable and connection inspection

**Annual:**
- Complete system audit
- Hardware diagnostic testing
- Software license renewal
- Professional calibration service

---

## 8. Appendices

### Appendix A: IMO Resolutions Reference

- MSC.74(69) - Adoption of new and amended performance standards
- MSC.191(79) - Performance standards for presentation of navigation information
- MSC.192(79) - Revised performance standards for radar equipment
- MSC.302(87) - Adoption of amendments to performance standards for AIS
- MSC-FAL.1/Circ.3 - Guidelines on maritime cyber risk management

### Appendix B: Regional Regulations

#### B.1 United States

- 33 CFR Part 164 - Navigation Safety Regulations
- 46 CFR Subchapter N - Dangerous Cargoes
- USCG Navigation and Vessel Inspection Circular (NVIC)

#### B.2 European Union

- EU Directive 2002/59/EC - Vessel Traffic Monitoring
- EU Regulation 1406/2002 - European Maritime Safety Agency
- Port State Control Directive 2009/16/EC

#### B.3 Asia-Pacific

- Tokyo MOU Port State Control
- ASEAN Maritime Transport Working Group Standards
- Singapore Maritime Authority Regulations

### Appendix C: Emergency Contact Information

#### C.1 Global Emergency Services

- **INMARSAT Emergency:** +870 776 439 110
- **IAMSAR Coordination:** Contact nearest MRCC via DSC Ch 70

#### C.2 Maritime Rescue Coordination Centers

| Region | MRCC | Contact |
|--------|------|---------|
| North Atlantic | USCG Boston | +1-617-223-8555 |
| Pacific | JMSDF Tokyo | +81-3-3591-6361 |
| Indian Ocean | MRCC Mumbai | +91-22-2431-6558 |
| Mediterranean | MRCC Rome | +39-06-5908-4409 |

### Appendix D: Glossary

**AIS** - Automatic Identification System
**ARPA** - Automatic Radar Plotting Aid
**CPA** - Closest Point of Approach
**DSC** - Digital Selective Calling
**ECDIS** - Electronic Chart Display and Information System
**EPIRB** - Emergency Position Indicating Radio Beacon
**GMDSS** - Global Maritime Distress and Safety System
**IMO** - International Maritime Organization
**LRIT** - Long Range Identification and Tracking
**MARPOL** - Marine Pollution Convention
**MMSI** - Maritime Mobile Service Identity
**MRCC** - Maritime Rescue Coordination Center
**NAVTEX** - Navigational Telex
**SART** - Search and Rescue Transponder
**SOLAS** - Safety of Life at Sea
**STCW** - Standards of Training, Certification and Watchkeeping
**TCPA** - Time to Closest Point of Approach
**VHF** - Very High Frequency
**VTS** - Vessel Traffic Service

### Appendix E: Data Examples

#### E.1 Complete Vessel Position Report

```json
{
  "type": "WIA-MARITIME_SAFETY-AISPosition",
  "version": "1.0",
  "mmsi": "367123450",
  "timestamp": "2026-01-12T14:30:00Z",
  "position": {
    "latitude": 37.8044,
    "longitude": -122.4162,
    "accuracy": true
  },
  "navigation": {
    "status": "Under way using engine",
    "rateOfTurn": 0,
    "speedOverGround": 12.5,
    "courseOverGround": 285.3,
    "heading": 287
  },
  "maneuver": "no special maneuver",
  "raim": true,
  "signature": "3045022100ab2c..."
}
```

#### E.2 Distress Alert

```json
{
  "type": "WIA-MARITIME_SAFETY-Alert",
  "version": "1.0",
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2026-01-12T15:45:30Z",
  "severity": "distress",
  "category": "fire",
  "position": {
    "latitude": 35.6762,
    "longitude": 139.6503
  },
  "vessel": {
    "mmsi": "431234567",
    "name": "OCEAN STAR"
  },
  "description": "Engine room fire, requesting immediate assistance",
  "status": "active",
  "responders": ["431000001", "431000002"]
}
```

### Appendix F: Certification Process

#### F.1 System Certification Steps

1. **Application Submission**
   - Complete WIA-MARITIME_SAFETY application form
   - Submit system documentation
   - Pay certification fee

2. **Documentation Review**
   - Technical specifications review
   - Compliance verification
   - Gap analysis report

3. **Laboratory Testing**
   - Functional testing
   - Performance benchmarking
   - Interoperability testing
   - Security assessment

4. **Field Trial**
   - Onboard installation
   - Real-world operation testing
   - Crew feedback collection
   - Performance monitoring

5. **Certification Decision**
   - Final report preparation
   - Certification committee review
   - Certificate issuance
   - Public listing

### Appendix G: Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | January 2026 | Initial release |

---

## License

This standard is released under the MIT License.

```
MIT License

Copyright (c) 2025 WIA - World Certification Industry Association

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*Published: January 2026*
*Standard ID: WIA-MARITIME_SAFETY-v1.0*

**Contact Information:**
- Website: https://wia.org
- Email: standards@wia.org
- GitHub: https://github.com/WIA-Official/wia-standards
