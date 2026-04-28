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

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for Vehicle-to-Everything (V2X) communication systems, enabling vehicles to exchange real-time information with other vehicles, infrastructure, pedestrians, networks, and cloud services to enhance road safety, traffic efficiency, and enable autonomous driving capabilities.

### 1.2 Scope

The standard covers:
- V2V, V2I, V2P, V2N, and V2C communication protocols
- Multiple wireless technologies (DSRC, C-V2X, 5G-V2X)
- Message types and data formats
- Security and privacy mechanisms
- Performance and latency requirements
- Safety protocols and certification

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard is designed to save lives by preventing collisions, reduce traffic congestion, minimize environmental impact, and create a foundation for fully autonomous vehicles that serve the common good.

### 1.4 Terminology

- **V2X**: Vehicle-to-Everything communication
- **V2V**: Vehicle-to-Vehicle direct communication
- **V2I**: Vehicle-to-Infrastructure communication
- **V2P**: Vehicle-to-Pedestrian communication
- **V2N**: Vehicle-to-Network communication
- **V2C**: Vehicle-to-Cloud communication
- **BSM**: Basic Safety Message
- **SPaT**: Signal Phase and Timing
- **MAP**: Map Data message
- **DENM**: Decentralized Environmental Notification Message
- **CAM**: Cooperative Awareness Message
- **DSRC**: Dedicated Short-Range Communication
- **C-V2X**: Cellular V2X
- **OBU**: On-Board Unit (vehicle)
- **RSU**: Road-Side Unit (infrastructure)

---

## 2. V2X Architecture

### 2.1 System Components

The V2X ecosystem consists of five primary communication domains:

#### 2.1.1 V2V (Vehicle-to-Vehicle)

Direct communication between vehicles for:
- Collision avoidance and warnings
- Cooperative adaptive cruise control
- Emergency electronic brake lights
- Lane change assistance
- Intersection movement assist
- Blind spot warning

**Range**: 300-1000 meters
**Latency**: < 100 ms
**Update Rate**: 10 Hz minimum

#### 2.1.2 V2I (Vehicle-to-Infrastructure)

Communication with road infrastructure:
- Traffic signal phase and timing (SPaT)
- Speed limit and regulatory information
- Road geometry and topology (MAP)
- Toll collection
- Parking availability
- Weather and road conditions

**Infrastructure Types**:
- Traffic signals
- Road-Side Units (RSU)
- Electronic road signs
- Parking facilities
- Toll gates

#### 2.1.3 V2P (Vehicle-to-Pedestrian)

Safety communication with vulnerable road users:
- Pedestrian presence detection
- Collision warning to driver
- Alert to pedestrian via smartphone
- Cyclist and motorcyclist detection
- School zone warnings

**Device Support**:
- Smartphones (iOS/Android)
- Wearables (smartwatches)
- Dedicated pedestrian devices

#### 2.1.4 V2N (Vehicle-to-Network)

Cellular network connectivity:
- Real-time traffic information
- Software updates (OTA)
- Infotainment services
- Fleet management
- Emergency services (eCall)

**Network Types**:
- 4G LTE
- 5G NR
- WiFi backhaul

#### 2.1.5 V2C (Vehicle-to-Cloud)

Cloud services for advanced features:
- Predictive maintenance
- Route optimization
- Historical traffic analytics
- Machine learning model updates
- Remote diagnostics
- Insurance telematics

### 2.2 Communication Stack

```
┌─────────────────────────────────────────┐
│     Application Layer                   │
│  (Safety Apps, Traffic Apps, etc.)      │
├─────────────────────────────────────────┤
│     Facilities Layer                    │
│  (Message Encoding/Decoding)            │
│  ASN.1 UPER, JSON, Protocol Buffers     │
├─────────────────────────────────────────┤
│     Network & Transport Layer           │
│  (GeoNetworking, IPv6, UDP, TCP)        │
├─────────────────────────────────────────┤
│     Access Layer                        │
│  (DSRC 802.11p, C-V2X PC5, Uu)         │
├─────────────────────────────────────────┤
│     Physical Layer                      │
│  (5.9 GHz, 5G NR, LTE)                 │
└─────────────────────────────────────────┘
```

### 2.3 Deployment Models

#### 2.3.1 DSRC-Only
- IEEE 802.11p based
- 5.9 GHz frequency band
- Direct communication (ad-hoc)
- No cellular infrastructure needed

#### 2.3.2 C-V2X Only
- 3GPP Release 14+ based
- PC5 interface for direct communication
- Uu interface for network communication
- Better performance than DSRC

#### 2.3.3 Hybrid (DSRC + C-V2X)
- Dual-mode operation
- Technology fallback/failover
- Maximum coverage and reliability
- Future-proof deployment

---

## 3. Communication Technologies

### 3.1 DSRC (Dedicated Short-Range Communication)

#### 3.1.1 Technical Specifications

```
Standard: IEEE 802.11p / IEEE 1609.x
Frequency: 5.850 - 5.925 GHz (5.9 GHz band)
Channel Bandwidth: 10 MHz
Number of Channels: 7 (in North America)
Modulation: OFDM (BPSK, QPSK, 16-QAM, 64-QAM)
Data Rates: 3, 4.5, 6, 9, 12, 18, 27 Mbps
Maximum Range: 1000 meters (line of sight)
Typical Range: 300 meters (urban)
Latency: < 100 ms
Tx Power: Up to 33 dBm (2W)
```

#### 3.1.2 Channel Allocation (US)

```
Channel 172 (5860 MHz): Medium priority V2V
Channel 174 (5870 MHz): Not used
Channel 176 (5880 MHz): Medium priority V2V
Channel 178 (5890 MHz): Control Channel (CCH) - Safety
Channel 180 (5900 MHz): High priority V2V/V2I
Channel 182 (5910 MHz): High priority V2V/V2I
Channel 184 (5920 MHz): High power public safety
```

#### 3.1.3 WAVE (Wireless Access in Vehicular Environments)

```
IEEE 1609.1: Resource Manager
IEEE 1609.2: Security Services
IEEE 1609.3: Networking Services (WSMP)
IEEE 1609.4: Multi-channel Operation
IEEE 1609.12: Provider Service Identifier (PSID)
```

### 3.2 C-V2X (Cellular V2X)

#### 3.2.1 LTE-V2X (3GPP Release 14)

```
Standard: 3GPP Release 14
Frequency: 5.9 GHz (ITS band)
Interface: PC5 (sidelink for direct communication)
Modulation: QPSK, 16-QAM, 64-QAM
Resource Allocation: Sensing-based SPS (Mode 4)
Data Rates: Up to 100 Mbps
Maximum Range: 1500 meters
Latency: < 50 ms
Power Control: Dynamic
Retransmissions: HARQ for reliability
```

**PC5 Interface Features**:
- Direct device-to-device communication
- No network infrastructure required
- Autonomous resource selection
- Priority-based transmission
- In-coverage and out-of-coverage support

#### 3.2.2 5G-V2X (3GPP Release 16+)

```
Standard: 3GPP Release 16/17
Frequency: 5.9 GHz + mmWave (28/39 GHz)
Technology: 5G NR (New Radio)
Latency: < 20 ms (URLLC)
Data Rate: 1+ Gbps
Reliability: 99.999% (ultra-reliable)
Maximum Range: 2000+ meters
Positioning: Sub-meter accuracy
```

**Advanced Features**:
- Ultra-Reliable Low-Latency Communication (URLLC)
- Enhanced Mobile Broadband (eMBB)
- Massive Machine-Type Communication (mMTC)
- Network slicing for V2X
- Edge computing integration
- Advanced sensor sharing

### 3.3 Technology Comparison

| Feature | DSRC | LTE-V2X | 5G-V2X |
|---------|------|---------|---------|
| Standardization | IEEE | 3GPP R14 | 3GPP R16+ |
| Maturity | High | Medium | Low-Medium |
| Latency | <100ms | <50ms | <20ms |
| Range | 1000m | 1500m | 2000m+ |
| Reliability | 95% | 98% | 99.9%+ |
| Data Rate | 27 Mbps | 100 Mbps | 1+ Gbps |
| Infrastructure | Not required | Optional | Optional |
| Evolution Path | Limited | 5G migration | Future-proof |
| Deployment | Existing | Growing | Emerging |

### 3.4 Frequency Bands

#### 3.4.1 Global ITS Spectrum Allocation

```
Region          Frequency Band    Bandwidth
Europe          5.875-5.905 GHz   30 MHz
US/Americas     5.850-5.925 GHz   75 MHz
China           5.905-5.925 GHz   20 MHz
Japan           755.5-764.5 MHz   9 MHz (V2V)
                5.770-5.850 GHz   80 MHz (V2I)
Korea           5.855-5.925 GHz   70 MHz
```

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

## 5. Security and Authentication

### 5.1 Security Architecture

V2X security is based on Public Key Infrastructure (PKI) with the following components:

```
┌─────────────────────────────────────┐
│     Root Certificate Authority      │
│           (Root CA)                 │
└────────────┬────────────────────────┘
             │
    ┌────────┴────────┐
    │                 │
┌───▼────┐      ┌────▼──────┐
│ Enroll │      │  Long Term│
│  CA    │      │    CA     │
└───┬────┘      └─────┬─────┘
    │                 │
    │          ┌──────▼──────┐
    │          │ Pseudonym   │
    │          │    CA       │
    │          └──────┬──────┘
    │                 │
    └────────┬────────┘
             │
      ┌──────▼────────┐
      │   Vehicle OBU │
      │  (20+ certs)  │
      └───────────────┘
```

### 5.2 Certificate Types

#### 5.2.1 Enrollment Certificate

```
Purpose: Initial vehicle registration
Validity: Long-term (3-5 years)
Usage: Request pseudonym certificates
Contains:
- Vehicle Public Key
- Enrollment CA signature
- Validity period
- Permissions (message types allowed)
```

#### 5.2.2 Pseudonym Certificates

```
Purpose: Message signing (anonymous)
Validity: Short-term (1 week typical)
Quantity: 20+ certificates per vehicle
Rotation: Every 5-10 minutes
Contains:
- Pseudonym Public Key
- Pseudonym CA signature
- Validity period
- Permissions bitmap
- NO vehicle identity information
```

#### 5.2.3 Authorization Tickets

```
Purpose: Authorize specific actions
Examples:
- Special vehicle priority (emergency, transit)
- Road operator credentials
- Misbehavior reporting authority
```

### 5.3 Message Security

#### 5.3.1 Signature Generation

```
For each outgoing V2X message:

1. Select current pseudonym certificate
2. Serialize message content
3. Calculate hash (SHA-256 or SHA-384)
4. Sign hash with private key (ECDSA)
5. Attach signature + certificate to message
6. Transmit secured message
```

#### 5.3.2 Signature Verification

```
For each incoming V2X message:

1. Extract certificate from message
2. Verify certificate signature (against CA)
3. Check certificate validity period
4. Check certificate not revoked (CRL)
5. Verify certificate permissions
6. Verify message signature
7. Check message plausibility
8. Accept or reject message
```

#### 5.3.3 Cryptographic Algorithms

```
Elliptic Curve: NIST P-256 (secp256r1)
Signature: ECDSA-256 or ECDSA-384
Hash: SHA-256 or SHA-384
Symmetric Encryption: AES-128-CCM
Key Agreement: ECIES (Elliptic Curve Integrated Encryption)
```

### 5.4 Privacy Protection

#### 5.4.1 Certificate Rotation

```
Rotation Triggers:
- Time-based: Every 5-10 minutes
- Location-based: When entering new region
- Event-based: After generating misbehavior report

Goals:
- Prevent long-term vehicle tracking
- Ensure anonymity in V2X communication
- Balance privacy vs. accountability
```

#### 5.4.2 Pseudonym Linking Resistance

```
Techniques:
1. Silent Period (30-60 seconds) before rotation
2. Multiple certificate changes in quick succession
3. Synchronized rotation in traffic (mix zones)
4. Random rotation timing (within window)
```

### 5.5 Misbehavior Detection

#### 5.5.1 Local Detection

```
Checks per received message:
- Position plausibility (speed, acceleration limits)
- Consistency (successive messages from same sender)
- Beacon frequency (not too high/low)
- Proximity (claimed position vs. signal strength)
- Environmental consistency (other vehicles agree)
```

#### 5.5.2 Global Detection

```
Misbehavior Authority (MA) analyzes:
- Aggregated reports from multiple vehicles
- Patterns of suspicious behavior
- Certificate usage patterns
- Revocation requests from law enforcement
```

#### 5.5.3 Certificate Revocation

```
Methods:
- Certificate Revocation List (CRL)
  - Distributed periodically (daily)
  - Contains revoked certificate IDs
  - Vehicles check before accepting messages

- Online Certificate Status Protocol (OCSP)
  - Real-time revocation checking
  - Requires network connectivity
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

## 7. API Interface

### 7.1 Core APIs

#### 7.1.1 Message Transmission API

```typescript
// Send Basic Safety Message
interface SendBSMRequest {
  position: GeoPosition;
  speed: number;          // m/s
  heading: number;        // degrees
  acceleration: Vector3;
  brakeStatus: BrakeStatus;
  vehicleSize: VehicleSize;
  partII?: BSMPartII;    // Optional extended data
}

interface SendBSMResponse {
  messageId: string;
  timestamp: number;
  transmissionStatus: 'success' | 'queued' | 'failed';
  error?: string;
}

async function sendBasicSafetyMessage(
  request: SendBSMRequest
): Promise<SendBSMResponse>;
```

#### 7.1.2 Message Reception API

```typescript
// Register message handler
interface MessageHandler {
  onBasicSafetyMessage(msg: BasicSafetyMessage): void;
  onSpatMessage(msg: SignalPhaseAndTiming): void;
  onMapMessage(msg: MapData): void;
  onHazardWarning(msg: HazardWarning): void;
}

function registerMessageHandler(handler: MessageHandler): void;

// Message filtering
interface MessageFilter {
  messageTypes?: MessageType[];
  geoFilter?: GeographicArea;
  senderId?: string[];
  minPriority?: number;
}

function setMessageFilter(filter: MessageFilter): void;
```

#### 7.1.3 Security API

```typescript
// Certificate management
interface CertificateInfo {
  certificateId: string;
  type: 'enrollment' | 'pseudonym' | 'authorization';
  validFrom: Date;
  validUntil: Date;
  permissions: Permission[];
}

async function getCertificates(): Promise<CertificateInfo[]>;
async function requestPseudonymCertificates(count: number): Promise<void>;
async function rotateCertificate(): Promise<void>;

// Message verification
interface VerificationResult {
  isValid: boolean;
  certificateValid: boolean;
  signatureValid: boolean;
  notRevoked: boolean;
  plausibilityCheck: boolean;
  errors?: string[];
}

async function verifyMessage(msg: V2XMessage): Promise<VerificationResult>;
```

#### 7.1.4 Communication Control API

```typescript
// Start/stop V2X communication
interface V2XConfig {
  vehicleId: string;
  technology: 'DSRC' | 'C-V2X' | 'Hybrid';
  frequency: number;        // MHz
  transmitPower: number;    // dBm
  bsmRate: number;          // Hz
  securityLevel: 'low' | 'medium' | 'high';
}

async function startV2XCommunication(config: V2XConfig): Promise<void>;
async function stopV2XCommunication(): Promise<void>;
async function getConnectionStatus(): Promise<ConnectionStatus>;
```

### 7.2 Application APIs

#### 7.2.1 Collision Warning API

```typescript
interface CollisionWarning {
  severity: 'low' | 'medium' | 'high' | 'critical';
  timeToCollision: number;  // seconds
  distance: number;         // meters
  relativeSpeed: number;    // m/s
  collisionType: 'frontal' | 'rear' | 'side' | 'intersection';
  otherVehicle: VehicleInfo;
  recommendedAction: 'brake' | 'steer-left' | 'steer-right' | 'none';
}

function onCollisionWarning(callback: (warning: CollisionWarning) => void): void;
```

#### 7.2.2 Traffic Signal API

```typescript
interface TrafficSignalInfo {
  intersectionId: string;
  currentPhase: 'green' | 'yellow' | 'red';
  timeRemaining: number;    // seconds
  nextPhase: string;
  distance: number;         // meters to intersection
  recommendedSpeed: number; // m/s for green wave
}

function onTrafficSignal(callback: (signal: TrafficSignalInfo) => void): void;
async function requestSignalPriority(reason: 'emergency' | 'transit'): Promise<boolean>;
```

#### 7.2.3 Platooning API

```typescript
interface PlatoonInfo {
  platoonId: string;
  leadVehicle: string;
  members: string[];
  targetSpeed: number;      // m/s
  targetSpacing: number;    // meters
  position: number;         // Position in platoon (0 = leader)
}

async function joinPlatoon(platoonId: string): Promise<boolean>;
async function leavePlatoon(): Promise<void>;
async function createPlatoon(): Promise<string>;
function onPlatoonUpdate(callback: (info: PlatoonInfo) => void): void;
```

### 7.3 Data Access APIs

#### 7.3.1 Vehicle State API

```typescript
interface VehicleState {
  position: GeoPosition;
  speed: number;
  heading: number;
  acceleration: Vector3;
  steeringAngle: number;
  brakes: BrakeStatus;
  lights: LightStatus;
  transmission: TransmissionState;
  vehicleType: VehicleType;
}

function getCurrentVehicleState(): VehicleState;
function subscribeToVehicleState(callback: (state: VehicleState) => void): void;
```

#### 7.3.2 Nearby Vehicles API

```typescript
interface NearbyVehicle {
  vehicleId: string;        // Temporary/pseudonym ID
  position: GeoPosition;
  relativePosition: Vector3; // Relative to ego vehicle
  speed: number;
  heading: number;
  vehicleType: VehicleType;
  lastUpdate: number;       // Timestamp
  confidence: number;       // 0-1
}

function getNearbyVehicles(maxDistance: number): NearbyVehicle[];
function subscribeToNearbyVehicles(
  callback: (vehicles: NearbyVehicle[]) => void,
  updateRate: number
): void;
```

---

## 8. Latency and Performance Requirements

### 8.1 End-to-End Latency

```
Critical Safety Messages (BSM, DENM):
- Target: < 100 ms (DSRC)
- Target: < 50 ms (C-V2X)
- Target: < 20 ms (5G-V2X)

Breakdown:
- Message generation: < 10 ms
- Security (signing): < 5 ms
- Transmission queuing: < 5 ms
- Radio transmission: < 20 ms
- Reception & processing: < 10 ms
- Security verification: < 10 ms
- Application processing: < 20 ms
- Display/actuation: < 20 ms
```

### 8.2 Message Generation Rates

```
BSM/CAM: 10 Hz (every 100 ms) mandatory
         20 Hz (every 50 ms) for high-speed scenarios

SPaT: 1-10 Hz depending on signal timing

MAP: 1 Hz or on-change (low frequency)

DENM: Event-triggered (immediate)
      Updates every 1 second while active

PVD: 1 Hz for traffic analytics
```

### 8.3 Communication Range Requirements

```
Urban Environment:
- V2V: 300 meters minimum
- V2I: 500 meters minimum
- V2P: 150 meters minimum

Highway:
- V2V: 500 meters minimum
- V2I: 800 meters minimum

Emergency Warning:
- Geo-broadcast: 1000+ meters
- Multi-hop forwarding enabled
```

### 8.4 Reliability Requirements

```
Safety-Critical Messages:
- Packet Delivery Ratio (PDR): > 95%
- Duplicate Message Rate: < 1%
- Message Integrity: 99.999%

Non-Safety Messages:
- PDR: > 90%
```

### 8.5 Positioning Accuracy

```
Position Accuracy:
- Lane-level: < 1.5 meters (95% confidence)
- High-definition: < 0.5 meters (5G-V2X with RTK)

Heading Accuracy:
- < 5 degrees (95% confidence)

Time Synchronization:
- GPS time: < 1 millisecond across all vehicles
```

### 8.6 Scalability

```
Vehicle Density Support:
- Urban: 1000+ vehicles per km²
- Highway: 500+ vehicles per km

Channel Load Management:
- Decentralized Congestion Control (DCC)
- Adaptive message rate (reduce BSM rate if congested)
- Priority-based transmission queuing
- Maximum channel utilization: 70%
```

---

## 9. Safety Protocols

### 9.1 Collision Avoidance

#### 9.1.1 Forward Collision Warning (FCW)

```
Trigger Conditions:
- Time-to-collision (TTC) < 2.7 seconds
- Relative speed > 5 m/s (closing)
- Both vehicles in same lane
- No obstacle between vehicles

Warning Levels:
Level 1 (Visual): TTC 2.7 - 2.0 seconds
Level 2 (Audio): TTC 2.0 - 1.5 seconds
Level 3 (Haptic/Brake): TTC < 1.5 seconds

Actions:
- Display warning to driver
- Audible alert
- Pre-charge brakes
- Automatic emergency braking (if equipped)
```

#### 9.1.2 Intersection Movement Assist (IMA)

```
Scenario: Vehicle approaching intersection
Data Sources:
- MAP: Intersection geometry
- SPaT: Traffic signal status
- BSM: Other vehicles approaching

Algorithm:
1. Determine ego vehicle trajectory
2. Identify potential conflicting vehicles
3. Calculate time-to-arrival for all vehicles
4. Detect violations:
   - Red light runner
   - Stop sign violation
   - Right-of-way violation
5. Alert driver if collision risk detected

Warning Threshold:
- Warn if TTC < 5 seconds and collision predicted
```

#### 9.1.3 Blind Spot Warning (BSW)

```
Detection:
- V2V BSM from vehicles in adjacent lanes
- Radar/camera sensor fusion

Criteria:
- Vehicle in blind spot zone
- Lateral distance < 3 meters
- Longitudinal distance: -5m to +15m relative
- Speed difference < 20 m/s

Actions:
- Visual indicator (mirror or A-pillar)
- Increased alert if turn signal activated
- Steering resistance/correction
```

### 9.2 Traffic Efficiency

#### 9.2.1 Green Light Optimal Speed Advisory (GLOSA)

```
Inputs:
- SPaT: Current signal phase and timing
- MAP: Distance to intersection
- Vehicle speed and position

Algorithm:
1. Calculate time-to-arrival at current speed
2. Predict signal phase at arrival
3. If red-at-arrival:
   - Calculate speed to arrive during green
   - Display recommended speed
4. If green-at-arrival:
   - Maintain or adjust speed for green wave
   - Avoid arriving during yellow/red

Benefits:
- Reduce stops at red lights
- Lower fuel consumption
- Reduce emissions
- Smoother traffic flow
```

#### 9.2.2 Cooperative Adaptive Cruise Control (CACC)

```
Requirements:
- V2V communication with preceding vehicle
- Radar for redundancy/safety

Control Loop:
1. Receive BSM from lead vehicle (10 Hz)
2. Extract: position, speed, acceleration
3. Calculate desired spacing:
   spacing = time_gap × speed + standstill_distance
   (time_gap = 0.6-1.0 seconds for CACC)
4. Compute acceleration command
5. Apply throttle/brake

Advantages over ACC:
- Shorter following distances (platoon)
- Faster response to lead vehicle
- Improved fuel efficiency
- Increased road capacity
```

### 9.3 Emergency Response

#### 9.3.1 Emergency Electronic Brake Light (EEBL)

```
Trigger:
- Hard braking detected (deceleration > 4 m/s²)
- Generates DENM immediately
- Broadcast to following vehicles

Message Content:
- Event type: Emergency braking
- Vehicle position, speed, heading
- Deceleration magnitude
- Hazard duration

Following Vehicle Response:
- Display warning: "Vehicle ahead braking hard!"
- Pre-charge brakes
- Prepare for emergency braking
- Forward warning to vehicles further back (multi-hop)
```

#### 9.3.2 Emergency Vehicle Alert

```
Emergency Vehicle (Ambulance, Fire, Police):
- Broadcasts high-priority BSM with special status
- Requests signal priority at intersections
- Larger broadcast range (1+ km)

Other Vehicles:
- Receive alert with emergency vehicle location
- Display direction and distance
- Visual/audio warning
- Navigation system suggests pull-over location

Traffic Signals:
- Grant green phase to emergency vehicle route
- Hold conflicting phases
```

### 9.4 Vulnerable Road User Protection

#### 9.4.1 Pedestrian Collision Warning

```
Detection:
- PSM from pedestrian smartphone/device
- Camera/LIDAR sensors

Warning Conditions:
- Pedestrian in vehicle path
- Time-to-collision < 3 seconds
- Vehicle speed > 5 km/h

Actions:
- Alert driver (visual + audio)
- Automatic emergency braking
- Alert pedestrian (smartphone vibration/alert)
```

#### 9.4.2 Cyclist Detection and Warning

```
Scenario 1: Cyclist in blind spot
- Detect via PSM or camera
- Warn driver if door opening risk
- Warn when turning across bike lane

Scenario 2: Intersection collision
- Cyclist approaching from right (right-hook)
- Warn both driver and cyclist
- Prevent turn if collision imminent
```

### 9.5 Hazard Notification

#### 9.5.1 Road Hazard Detection and Warning

```
Hazard Types:
- Slippery road (detected via ESC activation)
- Pothole (suspension/accelerometer)
- Debris on road
- Reduced visibility (fog, rain)
- Stopped vehicle in road

Process:
1. Vehicle detects hazard
2. Generates DENM
3. Broadcasts to area (geo-broadcast)
4. Receiving vehicles:
   - Display warning if approaching hazard
   - Reduce speed if applicable
   - Forward warning (multi-hop)

Hazard Lifetime:
- Active: 10 minutes (periodic updates)
- Cancelled: When hazard cleared
- Automatic expiry: No updates for 5 minutes
```

### 9.6 Testing and Certification

#### 9.6.1 Conformance Testing

```
Protocol Tests:
- Message format compliance (ASN.1 validation)
- Timing requirements (latency, message rate)
- Radio performance (range, reliability)
- Security (signature generation/verification)

Application Tests:
- Collision warning accuracy
- False alarm rate (< 5%)
- Detection rate (> 95%)
- Response time
```

#### 9.6.2 Interoperability Testing

```
Multi-vendor testing:
- OBU from Vendor A ↔ OBU from Vendor B
- DSRC ↔ C-V2X interoperability
- Different message encodings
- Cross-region compatibility

Plugfest Events:
- Annual V2X interoperability testing
- Multiple manufacturers
- Real-world scenarios
```

---

## 10. References

### 10.1 Standards and Specifications

#### 10.1.1 SAE International

```
SAE J2735: Dedicated Short Range Communications (DSRC) Message Set Dictionary
SAE J2945/1: On-Board System Requirements for V2V Safety Communications
SAE J2945/9: Vulnerable Road User Safety Message Minimum Performance Requirements
SAE J3161: On-Board System Requirements for V2I Applications
```

#### 10.1.2 IEEE

```
IEEE 802.11p: Wireless LAN for Vehicular Environments
IEEE 1609.1: WAVE Resource Manager
IEEE 1609.2: WAVE Security Services for Applications and Management Messages
IEEE 1609.3: WAVE Networking Services
IEEE 1609.4: WAVE Multi-Channel Operations
IEEE 1609.12: WAVE Identifier Allocations
```

#### 10.1.3 ETSI (European)

```
ETSI EN 302 637-2: Cooperative Awareness Message (CAM)
ETSI EN 302 637-3: Decentralized Environmental Notification Message (DENM)
ETSI EN 302 663: ITS-G5 Access Layer Specification
ETSI EN 303 613: LTE-V2X Access Layer Specification
ETSI TS 102 940: ITS Security Architecture and Security Management
ETSI TS 103 097: Security Header and Certificate Formats
```

#### 10.1.4 3GPP (Cellular V2X)

```
3GPP TS 22.185: Service requirements for V2X services
3GPP TS 23.285: Architecture enhancements for V2X services
3GPP TS 36.300: LTE-V2X overall description (Release 14)
3GPP TS 38.300: 5G-V2X overall description (Release 16+)
```

### 10.2 Regulatory

```
US:
- FCC Part 95 Subpart L: Dedicated Short-Range Communications
- NHTSA: V2V Communications for Safety

Europe:
- EC Decision 2008/671/EC: ITS Frequency Bands
- Delegated Regulation (EU) 2019/2144: Vehicle Safety

China:
- GB Standard for Intelligent Connected Vehicles
```

### 10.3 Technical Parameters

```
Frequency Bands:
- 5.850 - 5.925 GHz (ITS-G5, DSRC)
- 5.905 - 5.925 GHz (China)
- 755.5 - 764.5 MHz (Japan V2V)

Speed of Light: 299,792,458 m/s
Radio Propagation: ~300 m/μs
GPS Time: TAI - 19 seconds (as of 2025)
```

### 10.4 WIA Standards

```
- WIA-INTENT: Intent-based vehicle control and routing
- WIA-OMNI-API: Universal automotive API gateway
- WIA-SOCIAL: Collaborative mobility and platooning
- WIA-CLOUD: Cloud services for connected vehicles
- WIA-AI: AI/ML for autonomous driving
```

---

## Appendix A: Example Messages

### A.1 Basic Safety Message (JSON)

```json
{
  "messageId": 20,
  "value": {
    "coreData": {
      "msgCnt": 127,
      "id": "A3F29C81",
      "secMark": 45231,
      "lat": 377749500,
      "long": -1224194000,
      "elev": 100,
      "accuracy": {
        "semiMajor": 12,
        "semiMinor": 10,
        "orientation": 45
      },
      "transmission": "forwardGears",
      "speed": 1275,
      "heading": 4500,
      "angle": 0,
      "accelSet": {
        "long": 50,
        "lat": 0,
        "vert": 0,
        "yaw": 0
      },
      "brakes": {
        "wheelBrakes": "00000",
        "traction": "off",
        "abs": "off",
        "scs": "off",
        "brakeBoost": "off",
        "auxBrakes": "off"
      },
      "size": {
        "width": 190,
        "length": 450
      }
    }
  }
}
```

### A.2 SPaT Message (JSON)

```json
{
  "messageId": 19,
  "value": {
    "timeStamp": 123456,
    "intersections": [
      {
        "id": {
          "id": 12345
        },
        "status": "00000000",
        "states": [
          {
            "movementName": "North-South Green",
            "signalGroup": 1,
            "state-time-speed": [
              {
                "eventState": "protected-Movement-Allowed",
                "timing": {
                  "minEndTime": 235
                }
              }
            ]
          },
          {
            "movementName": "East-West Red",
            "signalGroup": 2,
            "state-time-speed": [
              {
                "eventState": "stop-And-Remain",
                "timing": {
                  "minEndTime": 235
                }
              }
            ]
          }
        ]
      }
    ]
  }
}
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-003 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
