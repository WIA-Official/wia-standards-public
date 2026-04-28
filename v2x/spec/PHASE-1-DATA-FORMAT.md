# WIA-AUTO-003 — Phase 1: Data Format

> V2X canonical Phase 1: message-catalogue + station + comm-stack + PNT + cooperative-perception envelopes.

# WIA-AUTO-003: V2X - Vehicle-to-Everything Communication Specification v1.0

> **Standard ID:** WIA-AUTO-003
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [V2X Architecture](#2-v2x-architecture)
3. [Communication Technologies](#3-communication-technologies)
4. [Message Types and Formats](#4-message-types-and-formats)
5. [Security and Authentication](#5-security-and-authentication)
6. [Data Protocols](#6-data-protocols)
7. [API Interface](#7-api-interface)
8. [Latency and Performance Requirements](#8-latency-and-performance-requirements)
9. [Safety Protocols](#9-safety-protocols)
10. [References](#10-references)

---


## 4. Message Types and Formats

### 4.1 Basic Safety Message (BSM) / CAM

The most critical V2X message, broadcast by all vehicles at 10 Hz.

#### 4.1.1 BSM Part I (Core Data)

```
Message ID: 20 (BSM)
Transmission Rate: 10 Hz (every 100 ms)
Size: ~200 bytes

Fields:
- Temporary ID (4 bytes) - Random ID for privacy
- Timestamp (2 bytes) - DSecond (0.01s resolution)
- Latitude (4 bytes) - 1/10 micro degree resolution
- Longitude (4 bytes) - 1/10 micro degree resolution
- Elevation (2 bytes) - 0.1 meter resolution
- Position Accuracy (3 bytes) - Semi-major/minor axis
- Speed (2 bytes) - 0.02 m/s resolution
- Heading (2 bytes) - 0.0125 degree resolution
- Steering Wheel Angle (1 byte) - 1.5 degree resolution
- Acceleration (7 bytes) - Longitudinal, lateral, vertical, yaw
- Brake Status (2 bytes) - Brake applied, ABS, traction control
- Vehicle Size (3 bytes) - Width, length
```

#### 4.1.2 BSM Part II (Extended Data)

Optional data elements transmitted based on events:

```
- Path History (23 waypoints)
- Path Prediction (future trajectory)
- Vehicle Classification
- Exterior Lights (headlights, turn signals, brake lights)
- Weather Report
- Safety Extensions (ABS, traction control, airbag)
- Special Vehicle Extensions (emergency, transit)
```

#### 4.1.3 Message Structure (ASN.1)

```asn1
BasicSafetyMessage ::= SEQUENCE {
    msgID DSRCmsgID,
    blob1 BSMblob, -- Size (38 bytes)
    blob2 BSMblob OPTIONAL -- Part II
}

BSMcoreData ::= SEQUENCE {
    msgCnt MsgCount,
    id TemporaryID,
    secMark DSecond,
    lat Latitude,
    long Longitude,
    elev Elevation,
    accuracy PositionalAccuracy,
    transmission TransmissionState,
    speed Speed,
    heading Heading,
    angle SteeringWheelAngle,
    accelSet AccelerationSet4Way,
    brakes BrakeSystemStatus,
    size VehicleSize
}
```

### 4.2 SPaT (Signal Phase and Timing)

Traffic signal information for intersection safety.

```
Message ID: 19 (SPaT)
Transmission Rate: 1-10 Hz
Source: Road-Side Unit (RSU)
Size: ~300 bytes

Fields:
- Intersection ID
- Timestamp
- Signal Groups (per lane/movement):
  - Current Phase (green, yellow, red)
  - Time Remaining (0.1s resolution)
  - Next Phase
  - Confidence Level
- Pedestrian Signals
- Priority Vehicle Status
```

**Use Cases**:
- Red light violation warning
- Green wave coordination
- Pedestrian crossing safety
- Emergency vehicle preemption

### 4.3 MAP (Map Data)

Road geometry and intersection topology.

```
Message ID: 18 (MAP)
Update Rate: On change or periodic (low frequency)
Source: RSU or cloud
Size: ~1-5 KB per intersection

Data Elements:
- Road Segments
  - Segment ID
  - Reference Point (lat/lon)
  - Lane Width
  - Lane Type (vehicle, pedestrian, bike)
- Intersections
  - Intersection ID
  - Reference Point
  - Approach Lanes
  - Ingress/Egress Lanes
  - Lane Connections
  - Speed Limits
- Traffic Control Devices
  - Stop Signs
  - Yield Signs
  - Traffic Signals (reference to SPaT)
```

### 4.4 TIM (Traveler Information Message)

Road conditions and hazard warnings.

```
Message ID: 31 (TIM)
Transmission: Event-driven
Sources: RSU, vehicles, traffic management center
Size: Variable (200-1000 bytes)

Content Types:
- Work Zone Information
  - Location
  - Duration
  - Lane Closures
  - Speed Limit
- Road Conditions
  - Icy roads
  - Flooding
  - Reduced visibility
- Incidents
  - Accidents
  - Disabled vehicles
  - Debris on road
- Parking Information
- Special Events
```

### 4.5 DENM (Decentralized Environmental Notification)

European standard for hazard warnings.

```
Message Type: DENM
Trigger: Event-based
Transmission: Immediate + periodic updates
Range: Geo-broadcast (configurable)

Event Types:
- Emergency Brake Warning
- Traffic Jam Detection
- Stationary Vehicle
- Adverse Weather Condition
- Roadworks
- Accident
- Post-Crash Warning
```

### 4.6 PVD (Probe Vehicle Data)

Aggregated sensor data for traffic analytics.

```
Data Elements:
- Snapshot Time
- Vehicle Path (sequence of positions)
- Speed Profile
- Weather Conditions (if equipped)
  - Temperature
  - Windshield wipers status
  - Air pressure
- Road Surface Condition (if equipped)
  - Friction coefficient
  - Road roughness
```

### 4.7 PSM (Personal Safety Message)

Vulnerable road user (VRU) protection.

```
Message ID: 32 (PSM)
Transmission Rate: 10 Hz when moving
Source: Pedestrian/cyclist smartphone or device
Size: ~150 bytes

Fields:
- User Type (pedestrian, cyclist, wheelchair)
- Position (lat/lon)
- Speed
- Heading
- Movement Prediction
- Device Type
- Cluster Size (group of pedestrians)
```

---



## 6. Data Protocols

### 6.1 Message Encoding

#### 6.1.1 ASN.1 UPER (Unaligned Packed Encoding Rules)

```
Standard: ITU-T X.691
Usage: SAE J2735 messages (North America)
Advantages:
- Compact binary encoding
- Efficient bandwidth usage
- Standardized schema

Example (BSM encoding):
Input: JSON/struct → ASN.1 compiler → UPER bytes
Output: ~200 bytes BSM
```

#### 6.1.2 Protocol Buffers

```
Alternative encoding for cloud communication:
- Human-readable schema (.proto files)
- Backward/forward compatibility
- Multi-language support
- Efficient serialization

Use Case: V2C (Vehicle-to-Cloud) data aggregation
```

#### 6.1.3 JSON

```
Usage: Development, testing, debugging
Advantages:
- Human-readable
- Easy integration with web services
- No compilation required

Disadvantages:
- Large message size (3-5× larger than UPER)
- Not suitable for real-time V2V
```

### 6.2 Network Protocols

#### 6.2.1 GeoNetworking

```
Purpose: Position-based routing for V2X
Standard: ETSI EN 302 636-4-1

Forwarding Methods:
- Geo-Broadcast: Area-based flooding
- Geo-Anycast: Nearest recipient in area
- Geo-Unicast: Specific position

Packet Structure:
[Basic Header | Common Header | Extended Header | Payload]

- Basic Header: Version, next header, lifetime
- Common Header: Traffic class, hop limit, payload type
- Extended Header: Source/dest position, sequence number
```

#### 6.2.2 IPv6 Networking

```
Addressing:
- Link-local: fe80::/10
- Global: 2001:db8::/32 (example)
- Multicast: ff02::1 (all nodes)

Protocols:
- UDP: For real-time messages (low overhead)
- TCP: For firmware updates, bulk data
- ICMPv6: Neighbor discovery
```

#### 6.2.3 WSMP (Wave Short Message Protocol)

```
Purpose: Lightweight alternative to IPv6 for DSRC
Standard: IEEE 1609.3

Advantages:
- Lower latency than IPv6
- Smaller header overhead
- Optimized for broadcast

Header Format:
- Version (1 byte)
- PSID (Provider Service ID, 1-4 bytes)
- Wave Element ID extensions
- Length
- Data
```

### 6.3 Quality of Service (QoS)

#### 6.3.1 Message Priority

```
Priority Levels (0-7):
Level 7: Emergency vehicle warning
Level 6: Safety warnings (collision, hazard)
Level 5: Traffic signal (SPaT)
Level 4: Road conditions
Level 3: Traffic information
Level 2: Infotainment
Level 1: Background data
Level 0: Best effort
```

#### 6.3.2 Channel Access

```
EDCA (Enhanced Distributed Channel Access):
- AC_VO (Voice): Highest priority
- AC_VI (Video): High priority
- AC_BE (Best Effort): Normal priority
- AC_BK (Background): Lowest priority

Parameters per Access Category:
- AIFS (Arbitration Inter-Frame Space)
- CWmin (Minimum Contention Window)
- CWmax (Maximum Contention Window)
- TXOP (Transmission Opportunity)
```

---




---

## A.1 Message-catalogue envelope

The Phase 1 envelope catalogues the V2X message set per SAE J2735 (US) and ETSI EN 302 637-2 / EN 302 637-3 (EU): BSM (Basic Safety Message, US) / CAM (Cooperative Awareness Message, EU); DENM (Decentralised Environmental Notification Message, EU) / RSA (Roadside Alert, US); MAP (intersection map data); SPaT (Signal Phase and Timing); TIM (Traveller Information Message); SDSM (Sensor Data Sharing Message, J3224); MCM (Manoeuvre Coordination Message, ETSI work item); CPM (Collective Perception Message, ETSI EN 103 324); VAM (Vulnerable-road-user Awareness Message, ETSI). Every message envelope carries the IEEE 1609.2 security envelope: enrolment certificate identifier, pseudonym certificate, signature, generation time per IEEE 1588 (Precision Time Protocol).

## A.2 Vehicle and roadside-unit descriptor

Vehicle and RSU descriptors carry: station identifier (anonymous pseudonym chain rotated per ETSI TS 103 097); station type (passenger car, light truck, heavy truck, bus, motorcycle, bicycle, pedestrian RSU, traffic light RSU, road works RSU); GeoJSON Point geometry in WGS 84 plus the elevation in metres above WGS 84 ellipsoid; heading in centi-degrees per the SAE J2735 convention; speed in centimetres per second; longitudinal and lateral acceleration; ABS / TC / ESP status flags where applicable; the originating-station's hardware certificate envelope per IEEE 1609.2.

## A.3 Communication-stack descriptor

The communication-stack descriptor enumerates the radio access technology: DSRC IEEE 802.11p / IEEE 802.11bd (5.9 GHz, 10 MHz channels); C-V2X PC5 (3GPP Release 14 LTE-V2X / Release 16-17 NR-V2X sidelink); cellular Uu (LTE / 5G NR uplink-downlink); LEO-satellite back-up where adopted. Each entry carries the channel allocation per ITU-R region (Region 1/2/3), the transmit-power envelope per the regional regulator (FCC 47 CFR Part 90 in the US; ETSI EN 302 571 in the EU; KCC Notification in KR), the modulation-and-coding profile, the latency envelope, and the per-message MTU including security overhead.

## A.4 Time-synchronisation and PNT envelope

Time synchronisation across vehicles and RSUs follows IEEE 1588 (PTPv2) plus the GNSS time-source envelope (GPS / GLONASS / Galileo / BeiDou / QZSS / NavIC). PNT (Position-Navigation-Timing) records carry the GNSS receiver class (single-frequency / dual-frequency / multi-frequency / multi-constellation), the augmentation envelope (SBAS — WAAS/EGNOS/MSAS/GAGAN; PPP / RTK / PPP-RTK with the correction service identifier), the antenna-of-record envelope, and the integrity-monitoring envelope per RTCA DO-229 / ITU-R M.823 with the protection-level envelope used for safety-relevant decisions.

## A.5 Cooperative-perception and CPM envelope

Cooperative-perception envelopes carry detected-object lists shared between vehicles and RSUs per ETSI EN 103 324 CPM: per-object identifier, object class (vehicle / VRU / animal / unknown), object position relative to the originating station's reference point, object velocity, object dimensions (length / width / height), object confidence, and the sensor-of-record envelope (camera, radar, LiDAR, ultrasonic, fused). The envelope cross-references the SDSM (J3224) profile in deployments that use the SAE catalogue rather than the ETSI catalogue.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/v2x/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-v2x-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/v2x-host:1.0.0` ships every v2x envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/v2x.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. V2x deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
