# WIA-AUTO-003 — Phase 4: Integration

> V2X canonical Phase 4: ecosystem integration (SAE J2735 + ETSI + IEEE 1609 + cellular/5G + ISO/SAE 21434 + privacy).

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



---

## A.1 Standards cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| US message catalogue          | SAE J2735                                 |
| US sensor-data sharing        | SAE J3224 (SDSM)                          |
| EU message catalogue          | ETSI EN 302 637-2 (CAM) / -3 (DENM)       |
| EU cooperative perception     | ETSI EN 103 324 (CPM)                     |
| EU VRU awareness              | ETSI TS 103 300 (VAM)                     |
| Security                      | IEEE 1609.2 / 1609.2.1 + ETSI TS 103 097  |
| SCMS / Trust management       | ETSI TS 102 941                           |
| DSRC                          | IEEE 802.11p / 802.11bd                   |
| C-V2X PC5                     | 3GPP TS 38.331 / TS 24.387                |
| Time synchronisation          | IEEE 1588 (PTPv2)                         |
| GNSS                          | RTCA DO-229 / ITU-R M.823                 |
| Spectrum (US)                 | FCC 47 CFR Part 90                        |
| Spectrum (EU)                 | ETSI EN 302 571                           |
| Spectrum (KR)                 | KCC Notification 2014-94 (5.9 GHz ITS)    |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 Cellular and 5G integration

Cellular and 5G integration covers the Uu (vehicle-to-network) modes for non-safety-critical traffic: HD-map distribution via MEC (Multi-access Edge Computing) per ETSI MEC 003; remote-driving and tele-operation use cases per 5GAA white papers; cooperative ITS service offload via Uu-and-PC5 hybrid stacks. Operators integrating both DSRC and C-V2X document the dual-radio coexistence envelope, the message-deduplication policy when both stacks deliver the same broadcast, and the per-message-class radio-access selection.

## A.3 Cybersecurity and SOTIF integration

Cybersecurity integration follows the ISO/SAE 21434 (road vehicles cybersecurity engineering) framework and UN ECE WP.29 R155 (cybersecurity management system) plus R156 (software updates). The integration envelope captures the type-approval cybersecurity management system identifier, the threat-model envelope, the post-deployment vulnerability-management envelope, and the OTA-update protocol per UN R156 / ISO 24089. SOTIF integration per ISO 21448 covers the unknown-unsafe envelope for cooperative-perception decisions, particularly when low-confidence detections from peer stations should trigger conservative behaviour rather than direct action.

## A.4 Privacy and data-protection integration

Pseudonym-certificate rotation per IEEE 1609.2.1 is the primary technical privacy control; integration with the operator's data-protection regime (GDPR / CCPA / PIPL / PIPA / APPI / LGPD / PIPEDA) captures the lawful-basis declaration (legitimate interest for safety messaging; consent for non-safety commercial use cases), the retention envelope (typical 90 day retention for raw messages at the operator's tenant boundary; aggregate counts at k>=20 retained indefinitely), and the data-subject-rights envelope. The integration envelope also captures the cross-border transfer mechanism per Article 49 GDPR (consent / standard contractual clauses / adequacy decisions) where applicable.

## A.5 Future directions

Active research tracks: NR-V2X PC5 Mode 2 advanced sensing-based resource allocation; cellular and PC5 hybrid stacks for predictable 99.999% reliability targets; cooperative perception with end-to-end probabilistic guarantees; MCM-based negotiated manoeuvres at intersections at scale; verifiable cooperative-perception via attestation chains so consumers can validate the sensor-of-record; URLLC-enabled remote driving; integrated terrestrial-and-non-terrestrial V2X via LEO constellations for over-the-horizon safety messaging. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- SAE J2735 — V2X message set dictionary
- SAE J3224 — sensor data sharing message
- IEEE 1609.0 / 1609.2 / 1609.2.1 / 1609.3 / 1609.4 — WAVE family
- IEEE 802.11p / 802.11bd — DSRC PHY/MAC
- 3GPP TS 24.387 / TS 38.331 — C-V2X PC5 / NR-V2X
- ETSI EN 302 637-2 / EN 302 637-3 — CAM / DENM
- ETSI EN 103 324 — CPM
- ETSI TS 103 300 — VAM
- ETSI TS 102 941 / TS 103 097 — Trust and security
- ISO/SAE 21434 — road vehicles cybersecurity engineering
- ISO 24089 — software updates engineering
- UN ECE WP.29 R155 / R156 — cybersecurity / software updates
- ISO 21448 — SOTIF
- RTCA DO-229 — GNSS minimum operational performance
- IEEE 1588 — Precision Time Protocol
- NIST FIPS 203/204/205 — post-quantum cryptography
- 5GAA white papers — V2X service requirements and architectures


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
