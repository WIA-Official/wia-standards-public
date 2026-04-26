# Chapter 3: WIA Standard Overview

## Overview

The WIA Fire Safety System Standard employs a comprehensive four-phase architecture that systematically addresses the challenges identified in Chapter 2. This chapter provides an overview of the standard's structure, design principles, and how each phase contributes to the overall goal of interoperable, secure, and cost-effective fire safety systems.

---

## The Four-Phase Architecture

### Architectural Philosophy

The WIA Standard separates concerns into distinct, manageable layers, each building upon the previous:

```
┌─────────────────────────────────────────────────────────┐
│ PHASE 4: SYSTEM INTEGRATION                             │
│ Building systems, emergency services, third-party apps  │
├─────────────────────────────────────────────────────────┤
│ PHASE 3: COMMUNICATION PROTOCOLS                        │
│ Fire detection protocols, evacuation coordination       │
├─────────────────────────────────────────────────────────┤
│ PHASE 2: API INTERFACE                                  │
│ RESTful APIs, WebSocket events, authentication          │
├─────────────────────────────────────────────────────────┤
│ PHASE 1: DATA FORMAT                                    │
│ JSON schemas, sensor data, alarm events, metadata       │
└─────────────────────────────────────────────────────────┘
```

### Why Four Phases?

**Separation of Concerns:**
- Each phase addresses distinct technical requirements
- Clear boundaries simplify implementation
- Phases can be certified independently
- Incremental adoption possible

**Progressive Complexity:**
- Phase 1 provides foundation (data formats)
- Phase 2 builds communication (APIs)
- Phase 3 ensures reliability (protocols)
- Phase 4 enables integration (systems)

**Implementation Flexibility:**
- Organizations can adopt phases incrementally
- Different components may implement different phases
- Backward compatibility maintained across phases

---

## Phase 1: Data Format Standardization

### Purpose

Establish standardized data schemas ensuring all systems represent information consistently, enabling interoperability at the most fundamental level.

### Key Objectives

**Unified Representation:**
- Single way to represent sensor data
- Consistent alarm event format
- Standardized device metadata
- Universal timestamp handling

**Technology Choice:**
```
Format: JSON (JavaScript Object Notation)
Rationale:
  ✓ Human-readable and editable
  ✓ Native support in all modern languages
  ✓ Efficient parsing and generation
  ✓ Schema validation available (JSON Schema)
  ✓ Extensive tooling ecosystem
  ✓ Industry-standard (RFC 8259)
```

### Core Schemas

**1. Sensor Data Schema**

Every sensor data point follows this structure:

```json
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "location": {
    "building": "Main Tower",
    "floor": 12,
    "zone": "East Wing",
    "coordinates": {"x": 45.2, "y": 23.8, "z": 3.5}
  },
  "status": "alarm",
  "readings": {
    "value": 4.2,
    "unit": "OD/meter",
    "timestamp": "2025-12-27T14:32:15Z"
  },
  "metadata": {
    "manufacturer": "SafetyTech Inc",
    "model": "ST-5000",
    "firmwareVersion": "2.4.1",
    "installationDate": "2024-06-15T10:00:00Z",
    "lastMaintenance": "2025-10-12T09:30:00Z"
  }
}
```

**2. Alarm Event Schema**

All alarm events use this format:

```json
{
  "eventId": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "eventType": "fire",
  "priority": "critical",
  "source": {
    "deviceId": "550e8400-e29b-41d4-a716-446655440000",
    "deviceType": "smoke_detector",
    "location": { /* Location object */ }
  },
  "timestamp": "2025-12-27T14:32:15Z",
  "description": "Smoke detected in East Wing Zone 12E",
  "notifications": ["panel", "monitoring-center", "emergency-services"],
  "status": "active",
  "acknowledgedBy": null,
  "acknowledgedAt": null,
  "resolvedAt": null
}
```

### Benefits of Phase 1

- **Universal Understanding:** All systems interpret data identically
- **Tool Compatibility:** Standard tools can process any compliant system's data
- **Future-Proof:** New fields can be added without breaking compatibility
- **Validation:** JSON Schema enables automatic validation

---

## Phase 2: API Interface Specification

### Purpose

Define standardized application programming interfaces enabling software applications and building systems to interact with fire safety infrastructure uniformly.

### Key Objectives

**RESTful API Design:**
- HTTP/HTTPS based (universal protocol)
- Resource-oriented URLs
- Standard HTTP methods (GET, POST, PUT, DELETE)
- JSON request/response bodies
- Predictable behavior

**Real-Time Events:**
- WebSocket connections for live updates
- Event-driven architecture
- Low-latency notifications
- Scalable pub/sub pattern

### Core API Endpoints

**Device Management:**
```
GET    /api/v1/devices              List all devices
GET    /api/v1/devices/{id}         Get device details
PUT    /api/v1/devices/{id}         Update configuration
POST   /api/v1/devices/{id}/test    Test device
DELETE /api/v1/devices/{id}         Remove device
```

**Alarm Management:**
```
GET    /api/v1/alarms               List active alarms
GET    /api/v1/alarms/{id}          Get alarm details
POST   /api/v1/alarms/{id}/acknowledge  Acknowledge alarm
POST   /api/v1/alarms/{id}/silence      Silence notifications
DELETE /api/v1/alarms/{id}          Clear resolved alarm
```

**System Status:**
```
GET    /api/v1/status               Overall system health
GET    /api/v1/status/panels        Control panel status
GET    /api/v1/status/zones         Zone-by-zone status
```

### Authentication & Authorization

**Security Layers:**
```
┌─────────────────────────────────────┐
│ Role-Based Access Control (RBAC)   │  Authorization
├─────────────────────────────────────┤
│ Bearer Token Authentication (JWT)   │  Identity
├─────────────────────────────────────┤
│ TLS 1.3 Encryption                  │  Transport Security
└─────────────────────────────────────┘
```

**User Roles:**
- **Viewer:** Read-only access to status
- **Operator:** Acknowledge alarms, silence notifications
- **Technician:** Device testing, configuration
- **Administrator:** Full system access

### WebSocket Events

Real-time event stream for live updates:

```javascript
// Connect to event stream
const ws = new WebSocket('wss://panel.example.com/api/v1/events');

// Receive events
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);

  switch(data.eventType) {
    case 'alarm.triggered':
      handleNewAlarm(data);
      break;
    case 'device.status.changed':
      updateDeviceStatus(data);
      break;
  }
};
```

### Benefits of Phase 2

- **Universal API:** All vendors expose identical interfaces
- **Rapid Integration:** Standard APIs reduce integration time by 80%
- **Third-Party Apps:** Enables ecosystem of applications
- **Remote Management:** Cloud-based monitoring and control

---

## Phase 3: Communication Protocol Design

### Purpose

Specify communication protocols ensuring reliable, secure, and timely operation of fire safety systems across diverse network infrastructures.

### Key Objectives

**Reliability:**
- Deterministic behavior during emergencies
- Guaranteed message delivery
- Automatic retry mechanisms
- Network failure handling

**Low Latency:**
- Maximum end-to-end latency: 3 seconds
- Sensor to panel: <1 second
- Alarm notification: <1 second
- Optimized protocol overhead

**Security:**
- Mandatory encryption (TLS 1.3)
- Device authentication
- Message integrity verification
- Replay attack prevention

### Protocol State Machines

**Fire Detection Protocol:**

```
State Transition Diagram:

    ┌──────────┐
    │  NORMAL  │ ◄──────────────────┐
    └────┬─────┘                     │
         │                           │
         │ Sensor trigger            │
         │                           │
         ▼                           │
    ┌──────────┐                     │
    │PRE-ALARM │                     │
    └────┬─────┘                     │
         │                           │
         │ Verification              │
         │                           │
         ▼                           │
    ┌──────────┐                     │
    │  ALARM   │                     │
    └────┬─────┘                     │
         │                           │
         │ Acknowledged              │
         │                           │
         ▼                           │
    ┌──────────┐                     │
    │   ACK    │ ────────────────────┘
    └──────────┘
         │
         │ Resolved
         │
         └────────────────────────────┘

Timing Requirements:
- NORMAL → PRE-ALARM: <500ms
- PRE-ALARM → ALARM: <2s (verification)
- ALARM → Notification: <1s
```

### Message Format

```
Protocol Message Structure:

┌────────┬──────┬──────────┬────────┬───────────┬──────┬──────────┐
│ HEADER │ TYPE │ PRIORITY │ SOURCE │ TIMESTAMP │ DATA │ CHECKSUM │
└────────┴──────┴──────────┴────────┴───────────┴──────┴──────────┘
  4 bytes  2 B     1 byte    16 B      8 bytes    var    4 bytes

Header: Protocol version and flags
Type: Message type identifier
Priority: Message priority (0=critical, 1=high, 2=normal, 3=low)
Source: Device UUID
Timestamp: Unix timestamp (milliseconds)
Data: Payload (Phase 1 JSON format)
Checksum: CRC32 for integrity
```

### Benefits of Phase 3

- **Guaranteed Performance:** Meet critical life-safety timing requirements
- **Reliable Operation:** Function correctly under adverse conditions
- **Network Resilience:** Handle failures gracefully
- **Security Assurance:** Comprehensive protection against threats

---

## Phase 4: System Integration Framework

### Purpose

Define integration specifications enabling fire safety systems to coordinate with building management, access control, HVAC, elevator control, and emergency services.

### Key Objectives

**Coordinated Response:**
- Automated actions across building systems
- Phased evacuation support
- Emergency services notification
- Integrated command and control

**Standard Interfaces:**
- BACnet integration
- Modbus connectivity
- OPC UA support
- Custom API bridges

### Integration Architecture

```
Fire Safety System Integration:

┌─────────────────────────────────────────────────────────┐
│                  Fire Alarm Panel                       │
│              (WIA Standard Interface)                   │
└────────┬───────────┬───────────┬──────────┬─────────────┘
         │           │           │          │
    ┌────▼────┐ ┌───▼────┐ ┌───▼────┐ ┌───▼────┐
    │  HVAC   │ │ Access │ │Elevator│ │Emergency│
    │ Control │ │Control │ │Control │ │Services│
    └─────────┘ └────────┘ └────────┘ └────────┘
         │           │           │          │
    Shut down  Unlock all   Recall to  Automatic
    air fans   exit doors   ground     911 call
```

### Building System Actions

**On Fire Alarm:**

**HVAC System:**
- Shutdown supply/return fans in alarm zones
- Close smoke dampers
- Activate smoke exhaust systems
- Pressurize stairwells

**Access Control:**
- Unlock all exit doors
- Release electromagnetic locks
- Open turnstiles and gates
- Maintain perimeter security

**Elevator Control:**
- Recall all elevators to designated floors
- Disable car calls
- Enable firefighter service mode
- Lock out normal operation

**Lighting Control:**
- Switch to emergency lighting
- Illuminate exit paths
- Flash exit signs
- Disable dimming systems

### Emergency Services Integration

Automated notification includes:

```json
{
  "facilityId": "BLDG-12345",
  "facilityName": "Acme Office Tower",
  "address": {
    "street": "123 Main Street",
    "city": "Anytown",
    "state": "CA",
    "postalCode": "90210",
    "gpsCoordinates": {
      "latitude": 34.0522,
      "longitude": -118.2437
    }
  },
  "emergencyType": "fire",
  "severity": "critical",
  "location": {
    "building": "Main Tower",
    "floor": 12,
    "zone": "East Wing"
  },
  "occupancyEstimate": 450,
  "hazardousMaterials": [],
  "contactPerson": {
    "name": "John Smith",
    "phone": "+1-555-0123"
  },
  "videoFeedUrl": "https://cctv.example.com/live/floor12"
}
```

### Benefits of Phase 4

- **Coordinated Response:** All systems work together
- **Enhanced Safety:** Faster evacuation, better access for responders
- **Reduced False Alarms:** Integrated verification
- **Operational Efficiency:** Automated response reduces human error

---

## Design Principles

### 1. Simplicity

**Principle:** Prefer simple, proven approaches over complex novel ones.

**Application:**
- JSON over custom binary formats
- RESTful APIs over custom protocols
- Standard HTTP over proprietary transports
- Clear, readable specifications

**Rationale:**
- Easier implementation reduces errors
- Lower training requirements
- Better long-term maintainability
- Wider technology compatibility

### 2. Security by Design

**Principle:** Security is mandatory, not optional.

**Application:**
- TLS 1.3 required for all communications
- Multi-factor authentication for critical operations
- Comprehensive audit logging
- Secure boot and code signing
- Regular security updates

**Rationale:**
- Fire safety systems are critical infrastructure
- Cyberattacks can have life-safety consequences
- Prevention is cheaper than remediation
- Compliance with security standards

### 3. Interoperability

**Principle:** Work correctly with any compliant implementation.

**Application:**
- Clear, unambiguous specifications
- Comprehensive test suites
- Reference implementations
- Certification program

**Rationale:**
- Multi-vendor ecosystems require guaranteed compatibility
- Reduces deployment risk
- Enables customer choice
- Drives healthy competition

### 4. Performance

**Principle:** Meet latency and throughput requirements for life safety.

**Application:**
- 3-second maximum detection-to-alert latency
- 100ms API response time
- 10,000 events/second capacity
- 99.99% uptime requirement

**Rationale:**
- Life safety demands rapid response
- Scalability supports large deployments
- Reliability is non-negotiable
- Performance predictability is essential

### 5. Extensibility

**Principle:** Support future enhancements while maintaining compatibility.

**Application:**
- Versioned APIs (v1, v2, etc.)
- Optional fields in schemas
- Extension points defined
- Backward compatibility commitment

**Rationale:**
- Technology evolves continuously
- Long product lifecycles (15-20 years)
- New features must not break existing systems
- Innovation enabled without disruption

### 6. 弘익人間 (Hongik Ingan)

**Principle:** Benefit all humanity.

**Application:**
- Open specifications (no license fees)
- Public reference implementations
- Community-driven development
- Global accessibility

**Rationale:**
- Fire safety is a human right
- Open standards benefit society
- Collaboration accelerates progress
- Ethical technology development

---

## Certification Levels

### Level 1: Basic Conformance

**Requirements:**
- Phase 1 data format compliance
- Mandatory security features
- Performance baseline met
- Single-vendor deployment

**Benefits:**
- Foundation for interoperability
- Security assurance
- Performance guarantee
- Certification badge

### Level 2: Multi-Vendor

**Requirements:**
- Phase 1 and Phase 2 compliance
- Demonstrated interoperability with 3+ vendors
- Mixed vendor device support
- Integration testing passed

**Benefits:**
- Proven multi-vendor compatibility
- Flexible deployment options
- Competitive pricing
- Vendor choice freedom

### Level 3: Advanced Features

**Requirements:**
- All phases (1-4) compliant
- Optional features implemented
- Enhanced integration capabilities
- Analytics and predictive maintenance

**Benefits:**
- Full standard benefits
- Advanced capabilities
- Maximum integration
- Future-proof deployment

---

## Key Takeaways

1. **Four-phase architecture systematically addresses** data formats, APIs, protocols, and system integration.

2. **Each phase builds upon the previous**, creating a coherent, comprehensive standard.

3. **Design principles ensure** simplicity, security, interoperability, performance, and extensibility.

4. **Certification levels provide** flexible adoption paths from basic compliance to advanced features.

5. **The WIA Standard enables** true multi-vendor interoperability while maintaining highest safety and security standards.

---

## Review Questions

1. What problem does each of the four phases address?
2. Why was JSON chosen as the data format standard?
3. What are the key benefits of standardized API interfaces?
4. How does Phase 3 ensure reliable operation during emergencies?
5. What building systems integrate with fire safety under Phase 4?
6. Which design principle is most important to you and why?

---

## Next Steps

Chapters 4-7 provide detailed technical specifications for each phase:
- Chapter 4: Data Format Specifications (Phase 1)
- Chapter 5: API Interface Design (Phase 2)
- Chapter 6: Communication Protocols (Phase 3)
- Chapter 7: System Integration (Phase 4)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
