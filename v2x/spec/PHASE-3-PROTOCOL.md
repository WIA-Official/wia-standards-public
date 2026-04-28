# WIA-AUTO-003 — Phase 3: Protocol

> V2X canonical Phase 3: protocols (architecture + comm-tech + security + cooperative-perception + safety).

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




---

## A.1 V2X-architecture protocol

The V2X-architecture protocol enumerates the three layers: access (DSRC IEEE 802.11p / 802.11bd; C-V2X PC5 sidelink; cellular Uu), facilities (CAM/DENM/MAP/SPaT/CPM/VAM/IVI/SREM-SSEM message handling), and the application layer (collision-warning, intersection-movement-assist, blind-spot warning, vulnerable-road-user warning, signal-priority, environmental hazard warning, electric-vehicle charging-spot reservation). Each layer carries conformance gates: PHY/MAC per IEEE 802.11p / 3GPP TS 38.331 NR-V2X; security per IEEE 1609.2 / ETSI TS 103 097; messages per SAE J2735 / ETSI EN 302 637-2/-3.

## A.2 Communication-technology protocol

DSRC operates at 5.9 GHz with 10 MHz channels per FCC 47 CFR Part 90 (Channel 172 reserved for safety). 802.11bd extends 802.11p with higher data rates and dual-carrier modulation. C-V2X PC5 Mode 4 (Release 14) operates without network coverage using sensing-based resource allocation; Release 16/17 NR-V2X PC5 adds unicast/multicast and improved scheduling. Cellular Uu V2N (vehicle-to-network) leverages LTE/5G for non-safety-critical use cases (HD map updates, vulnerable-road-user awareness via MEC). The protocol selection is captured in the station's Phase 1 §A.3 descriptor and validated at registration time.

## A.3 Security and authentication protocol

Security and authentication follows the IEEE 1609.2 security framework with the SCMS (Security Credential Management System) per IEEE 1609.2.1 and the ETSI Trust List per TS 103 097 / TS 102 941. Each station carries an enrolment certificate (long-lived) and a batch of pseudonym certificates (short-lived, rotated to defend against tracking). Messages are signed with ECDSA over P-256 (with migration paths to ML-DSA per NIST FIPS 204 for post-quantum); receivers verify the signature, the certificate chain to a trusted root, and the certificate's not-revoked status against the latest CRL.

## A.4 Cooperative-perception and manoeuvre-coordination protocol

Cooperative perception (ETSI EN 103 324 CPM / SAE J3224 SDSM) shares per-object detections among vehicles and RSUs, enabling each station to extend its situational awareness beyond its own sensor field of view. Manoeuvre coordination (ETSI MCM work item) extends this to negotiated manoeuvres for cooperative lane change, intersection traversal, and platooning. The protocol enforces consistency between perceived objects and authored manoeuvres so a station does not announce a manoeuvre that conflicts with the cooperative-perception ground truth.

## A.5 Safety and signal-priority protocol

Safety-critical messages (BSM / CAM at 10 Hz; DENM event-triggered) ride on Channel 172 (US) or the 5.9 GHz safety channel allocation (EU) with strict priority over service-channel traffic. Emergency-vehicle preemption follows NEMA TS 2 plus the regional preemption policy; transit-signal priority follows the NTCIP 1211 / J2735 SREM-SSEM exchange. Vulnerable-road-user safety adds the VAM (ETSI TS 103 300) profile so smartphones and dedicated VRU devices can announce their presence; vehicles with VAM-aware infotainment surface the warning to the driver per the operator's HMI guidelines.

## A.6 Replay and integrity defence

V2X messages carry a generation-time per IEEE 1588 PTPv2 with the receiver enforcing a freshness window (default 1 second for safety messages); messages outside the window are dropped. Pseudonym-certificate rotation defeats long-term tracking but does not weaken the integrity check: every rotated certificate carries a chain to the same enrolment certificate, and the SCMS preserves the link-back capability for misbehaviour-report investigation. Post-quantum migration tracks NIST FIPS 203/204/205 with hybrid signatures during the transition window.


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
