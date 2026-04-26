# Chapter 6: Communication Protocols (Phase 3)

## Overview

Phase 3 of the WIA Standard specifies the communication protocols that ensure reliable, secure, and timely operation of fire safety systems. This chapter details protocol state machines, message formats, timing requirements, security specifications, and network resilience mechanisms critical for life-safety applications.

---

## Protocol Layer Architecture

### Layered Communication Model

```
┌─────────────────────────────────────────────────────────┐
│ Application Layer                                       │
│ Fire detection logic, alarm processing                  │
├─────────────────────────────────────────────────────────┤
│ WIA Protocol Layer ← Phase 3 Focus                      │
│ Fire detection protocol, evacuation coordination        │
├─────────────────────────────────────────────────────────┤
│ Transport Layer                                         │
│ TCP (reliable) / UDP (real-time)                        │
├─────────────────────────────────────────────────────────┤
│ Security Layer                                          │
│ TLS 1.3 encryption, authentication                      │
├─────────────────────────────────────────────────────────┤
│ Network Layer                                           │
│ IP routing, addressing                                  │
├─────────────────────────────────────────────────────────┤
│ Data Link Layer                                         │
│ Ethernet, WiFi                                          │
├─────────────────────────────────────────────────────────┤
│ Physical Layer                                          │
│ Cables, wireless signals                                │
└─────────────────────────────────────────────────────────┘
```

### Protocol Design Goals

**Reliability:**
- Guaranteed message delivery for critical events
- Automatic retry with exponential backoff
- Duplicate detection and elimination
- Message ordering preservation

**Low Latency:**
- Maximum 3-second detection-to-alert
- Optimized message formats
- Priority-based queuing
- Direct device-to-panel communication

**Security:**
- End-to-end encryption mandatory
- Device authentication required
- Message integrity verification
- Replay attack prevention

**Scalability:**
- Support 10,000+ devices per panel
- Efficient multicast for notifications
- Load balancing across panels
- Hierarchical network topology

---

## Fire Detection Protocol

### State Machine

The fire detection protocol implements a deterministic state machine ensuring predictable behavior:

```
Fire Detection State Machine:

    Initial State
         │
         ▼
    ┌──────────┐
    │  NORMAL  │ ◄──────────────────────────────┐
    └────┬─────┘                                 │
         │                                       │
         │ Sensor reading exceeds threshold     │
         │ or manual activation                 │
         │                                       │
         ▼                                       │
    ┌──────────┐                                 │
    │PRE-ALARM │                                 │
    └────┬─────┘                                 │
         │                                       │
         │ Verification period (0-30s)          │
         │ Multi-sensor confirmation or         │
         │ single sensor sustained reading      │
         │                                       │
         ├────────► [FALSE ALARM] ──────────────┘
         │          (if not verified)
         │
         ▼
    ┌──────────┐
    │  ALARM   │
    └────┬─────┘
         │
         │ Notifications sent
         │ Building systems activated
         │
         ├────────► Operator acknowledges
         │
         ▼
    ┌──────────┐
    │   ACK    │
    └────┬─────┘
         │
         │ Investigation & remediation
         │ Condition verified cleared
         │
         ▼
    ┌──────────┐
    │ RESOLVED │ ─────────────────────────────────┘
    └──────────┘

Timing Requirements:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Event                          Maximum Time
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Sensor detection → Signal      <500ms
Signal → Panel processing      <500ms
Panel → Pre-alarm state        <100ms
Pre-alarm → Alarm (verified)   <30s
Alarm → Notification sent      <1s
Total detection → Alert        <3s (critical path)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### State Transitions

**NORMAL → PRE-ALARM:**
```
Trigger Conditions:
1. Sensor reading > alarm threshold
2. Manual pull station activated
3. Rate-of-rise exceeds limit (heat detectors)
4. Multiple criteria met (multi-sensor devices)

Message Sent:
{
  "messageType": "PRE_ALARM",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-27T14:32:15.234Z",
  "readings": {
    "value": 4.2,
    "threshold": 3.5,
    "unit": "OD/meter"
  },
  "requiresVerification": true,
  "verificationWindow": 30
}
```

**PRE-ALARM → ALARM:**
```
Verification Methods:
1. Sustained reading (>30 seconds above threshold)
2. Multi-sensor confirmation (2+ devices in zone)
3. Manual pull station (immediate verification)
4. Rapid increase trend (predictive algorithm)

Message Sent:
{
  "messageType": "ALARM",
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-27T14:32:45.567Z",
  "alarmType": "fire",
  "priority": "critical",
  "verificationMethod": "sustained_reading",
  "location": {
    "building": "Main Tower",
    "floor": 12,
    "zone": "East Wing E12-A"
  }
}
```

**ALARM → ACKNOWLEDGED:**
```
Operator Action Required:
1. Authentication verification
2. Alarm acknowledgment button/command
3. Optional notes entry

Message Sent:
{
  "messageType": "ALARM_ACK",
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "timestamp": "2025-12-27T14:35:20.123Z",
  "acknowledgedBy": "operator-john-smith",
  "notes": "Fire department en route"
}
```

**ACKNOWLEDGED → RESOLVED:**
```
Resolution Conditions:
1. Fire extinguished/cleared
2. Sensor readings return to normal
3. Investigation complete
4. System reset authorized

Message Sent:
{
  "messageType": "ALARM_RESOLVED",
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "timestamp": "2025-12-27T15:20:15.890Z",
  "resolvedBy": "fire-chief-williams",
  "resolution": "False alarm - dust from construction",
  "actionsTaken": ["Area inspected", "Sensor cleaned", "System tested"]
}
```

---

## Message Format Specification

### Binary Message Structure

For maximum efficiency and reliability, critical messages use a binary format:

```
WIA Fire Safety Protocol Message Format v1.0

┌────────┬──────┬──────────┬────────┬───────────┬──────┬──────────┐
│ HEADER │ TYPE │ PRIORITY │ SOURCE │ TIMESTAMP │ DATA │ CHECKSUM │
└────────┴──────┴──────────┴────────┴───────────┴──────┴──────────┘
  4 bytes  2 B     1 byte    16 B      8 bytes    var    4 bytes

Field Details:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Field       Bytes   Description
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
HEADER      4       Protocol version (2B) + Flags (2B)
TYPE        2       Message type identifier
PRIORITY    1       0=critical, 1=high, 2=normal, 3=low
SOURCE      16      Device UUID (binary format)
TIMESTAMP   8       Unix timestamp (milliseconds)
DATA        var     Payload (JSON format from Phase 1)
CHECKSUM    4       CRC32 for message integrity
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Header Flags (bit positions):
  0: Acknowledgment required
  1: Encrypted payload
  2: Compressed payload
  3: Fragmented message
  4-15: Reserved for future use
```

### Message Types

```
Message Type Registry:

Type ID   Name                    Description
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
0x0001    HEARTBEAT              Device status keepalive
0x0002    SENSOR_READING         Normal sensor reading
0x0010    PRE_ALARM              Pre-alarm condition
0x0011    ALARM                  Confirmed alarm
0x0012    ALARM_ACK              Alarm acknowledged
0x0013    ALARM_RESOLVED         Alarm resolved
0x0020    SUPERVISORY            Supervisory signal
0x0021    TROUBLE                Trouble condition
0x0030    TEST_INITIATED         Device test started
0x0031    TEST_RESULT            Device test result
0x0040    CONFIG_UPDATE          Configuration change
0x0050    FIRMWARE_UPDATE        Firmware update status
0x00FF    SYSTEM_MESSAGE         General system message
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### Example Message (Hexadecimal)

```
ALARM Message Example:

00 01 00 03 00 11 00 55 0e 84 00 e2 9b 41 d4 a7
16 44 66 55 44 00 00 00 01 8c 3e 5f 2a 10 7b 22
...  [JSON payload] ...
a3 c4 f2 e1

Breakdown:
00 01          - Protocol version 1.0
00 03          - Flags: Ack required + Encrypted
00 11          - Type: ALARM
00             - Priority: Critical (0)
55 0e ... 00   - Source UUID (16 bytes)
00 00 ... 10   - Timestamp: 1735315965000
7b 22 ...      - JSON payload (Phase 1 format)
a3 c4 f2 e1    - CRC32 checksum
```

---

## Evacuation Coordination Protocol

### Phased Evacuation Strategy

```
Phased Evacuation Timeline:

T+0s    ┌─────────────────────────────────────────┐
        │ ALERT PHASE                             │
        │ • Silent notification to security/mgmt  │
        │ • Fire department auto-dispatch         │
        │ • CCTV directed to alarm zone           │
        │ • Fire panel displays alarm location    │
        └─────────────────────────────────────────┘

T+30s   ┌─────────────────────────────────────────┐
        │ ZONE EVACUATION                         │
        │ • Alarm zone audible/visual activated   │
        │ • Voice message: "Attention. An alarm   │
        │   has been activated. Please evacuate   │
        │   via nearest exit."                    │
        │ • Exit doors unlock automatically       │
        │ • Emergency lighting activates          │
        └─────────────────────────────────────────┘

T+60s   ┌─────────────────────────────────────────┐
        │ FLOOR EVACUATION                        │
        │ • Entire floor alarms activate          │
        │ • Above/below floors receive alert tone │
        │ • Elevators recalled to ground floor    │
        │ • HVAC shutdown in affected areas       │
        │ • Stairwell pressurization activates    │
        └─────────────────────────────────────────┘

T+90s   ┌─────────────────────────────────────────┐
        │ BUILDING EVACUATION                     │
        │ • All floors receive evacuation order   │
        │ • All exit doors unlock                 │
        │ • Emergency power systems active        │
        │ • Firefighter elevators enabled         │
        │ • Building lockdown except egress       │
        └─────────────────────────────────────────┘
```

### Voice Evacuation Messages

**Message Requirements:**
```
Voice Message Specifications:

Content:
✓ Clear, calm, authoritative tone
✓ Specific instructions
✓ Avoid panic-inducing language
✓ Repeat every 10-20 seconds
✓ Multi-language support (configurable)

Technical:
✓ Minimum 90 dB at all points
✓ Maximum background noise: 80 dB
✓ Intelligibility: >0.90 CIS
✓ Frequency response: 400-4000 Hz
✓ Multiple speakers per zone (redundancy)

Example Messages:

Alert Phase:
"Attention. The fire alarm system has been activated.
Please stand by for further instructions."

Evacuation Phase:
"Attention please. An emergency has been reported in
this building. Please proceed to the nearest exit.
Do not use elevators. Walk, do not run."

All-Clear:
"Attention. The emergency situation has been resolved.
You may return to the building. Thank you for your
cooperation."
```

---

## Timing Requirements

### Critical Path Analysis

```
Fire Detection Critical Path (Maximum Latency):

Event                                    Max Time    Cumulative
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. Fire ignition → Detectable smoke     Variable    -
2. Smoke → Sensor detection              <500ms      500ms
3. Sensor → Signal generation            <100ms      600ms
4. Signal → Panel reception              <400ms      1000ms
5. Panel processing & verification       <500ms      1500ms
6. Panel → Alarm decision                <100ms      1600ms
7. Alarm → Notification command          <100ms      1700ms
8. Command → Appliance activation        <300ms      2000ms
9. Appliance → Audible output            <50ms       2050ms
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TOTAL SYSTEM LATENCY:                                2050ms

NFPA 72 Requirement: <3 seconds ✓ Met with 950ms margin
```

### Performance Requirements by Operation

| Operation | Target | Maximum | Unit |
|-----------|--------|---------|------|
| Sensor reading transmission | 50ms | 100ms | Per reading |
| Heartbeat interval | 30s | 60s | Between beats |
| Panel processing | 250ms | 500ms | Per event |
| API read response | 50ms | 100ms | HTTP request |
| API write response | 250ms | 500ms | HTTP request |
| WebSocket latency | 10ms | 50ms | Event delivery |
| Database query | 20ms | 100ms | Simple query |
| Alarm propagation | 500ms | 1000ms | Multi-panel |
| Emergency services notification | 1s | 2s | From alarm |

---

## Network Reliability

### Automatic Retry Mechanism

```
Retry Algorithm (Exponential Backoff):

Attempt    Delay        Cumulative    Notes
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1          0ms          0ms           Initial attempt
2          100ms        100ms         First retry
3          200ms        300ms
4          400ms        700ms
5          800ms        1500ms
6          1600ms       3100ms
7          3200ms       6300ms        Max for critical
8          6400ms       12700ms       Max for non-critical

Critical Messages:
- Maximum 7 attempts (6.3s total)
- After failure, local alarm activation
- Redundant transmission paths

Non-Critical Messages:
- Maximum 10 attempts (60s total)
- After failure, logged for later retry
- No local alarm needed
```

### Redundancy Strategies

**Network Redundancy:**
```
Primary and Backup Networks:

┌─────────────┐         ┌─────────────┐
│  Device A   │         │  Device B   │
└──────┬──────┘         └──────┬──────┘
       │                       │
       ├───────Primary─────────┤
       │      Network 1        │
       │                       │
       ├───────Backup──────────┤
       │      Network 2        │
       │                       │
       ▼                       ▼
┌─────────────┐         ┌─────────────┐
│  Panel A    │         │  Panel B    │
│  (Primary)  │◄───────►│  (Backup)   │
└─────────────┘  Sync   └─────────────┘

Failover Criteria:
- Network 1 failure: Auto-switch to Network 2 (<1s)
- Panel A failure: Panel B assumes control (<5s)
- Both networks down: Local standalone mode
```

**Device-Level Redundancy:**
```
Multi-Sensor Coverage:

Zone with Redundant Detection:

    [Sensor A]     [Sensor B]     [Sensor C]
         │             │              │
         └─────────────┴──────────────┘
                       │
                  Any 2 of 3
                   Triggers
                    Alarm
```

### Connection Monitoring

```
Heartbeat Protocol:

Device → Panel:
{
  "messageType": "HEARTBEAT",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-27T14:32:15.000Z",
  "status": "normal",
  "signalStrength": -45,
  "batteryLevel": 100
}

Interval: 30 seconds (configurable: 15-60s)

Missed Heartbeat Actions:
1 missed  : No action (grace period)
2 missed  : Warning logged
3 missed  : Trouble condition raised
4 missed  : Device marked offline
5 missed  : Maintenance alert triggered
```

---

## Security Requirements

### Encryption

**TLS 1.3 Mandatory:**
```
Required Configuration:

Cipher Suites (in order of preference):
1. TLS_AES_256_GCM_SHA384
2. TLS_AES_128_GCM_SHA256
3. TLS_CHACHA20_POLY1305_SHA256

Key Exchange:
- ECDHE with P-256 or P-384 curves
- Minimum 2048-bit RSA (or 256-bit ECC)
- Perfect Forward Secrecy (PFS) required

Certificate Requirements:
- Issued by recognized CA or private CA
- Minimum 2048-bit RSA key
- SHA-256 or stronger signature
- Valid (not expired or revoked)
- Subject matches device/panel identity
```

### Authentication

**Device Authentication:**
```
Mutual TLS Authentication:

Panel                              Device
  │                                  │
  ├────── ClientHello ───────────────►│
  │                                  │
  │◄────── ServerHello ───────────────┤
  │◄────── Certificate ───────────────┤
  │◄─── CertificateRequest ───────────┤
  │                                  │
  ├────── Certificate ───────────────►│
  ├────── ClientKeyExchange ─────────►│
  ├─── CertificateVerify ────────────►│
  │                                  │
  │◄────── Finished ──────────────────┤
  ├────── Finished ──────────────────►│
  │                                  │
  │      Secure Communication        │
  │◄────────────────────────────────►│

Both parties verify:
✓ Certificate validity
✓ Certificate chain trust
✓ Subject identity match
✓ Certificate not revoked
```

---

## Key Takeaways

1. **Deterministic state machines** ensure predictable fire detection behavior meeting critical timing requirements.

2. **Binary message format** optimizes efficiency while JSON payloads maintain interoperability.

3. **Phased evacuation protocol** coordinates building-wide response with appropriate escalation.

4. **Sub-3-second latency** from detection to alert exceeds NFPA 72 requirements with safety margin.

5. **Network redundancy and retry mechanisms** ensure reliability even under adverse conditions.

6. **Mandatory TLS 1.3 encryption and mutual authentication** protect against cyber threats.

---

## Review Questions

1. What are the five states in the fire detection state machine?
2. What is the maximum allowable latency from detection to alert?
3. How does phased evacuation differ from immediate building-wide evacuation?
4. What retry strategy is used for critical messages?
5. Why is mutual TLS authentication required?

---

## Next Steps

Chapter 7 explores Phase 4, defining system integration specifications that coordinate fire safety with building management, access control, and emergency services.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
