# WIA Deep Sea Exploration - Phase 3: Protocol Specification
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 of the WIA Deep Sea Exploration Standard defines communication protocols for underwater acoustic networks, ensuring reliable data transmission in the challenging deep ocean environment. This specification addresses the unique challenges of underwater communication including high latency, low bandwidth, signal attenuation, and multi-path propagation.

### 1.1 Scope

This specification covers:
- Acoustic modem physical layer specifications
- Data link layer protocols for error correction
- Network layer routing in underwater networks
- Transport layer reliability mechanisms
- Application layer protocols for mission-critical operations
- Multi-vehicle coordination protocols
- Emergency communication protocols
- Hybrid acoustic-fiber communication systems

### 1.2 Design Goals

1. **Reliability**: Robust error correction for high packet loss environments
2. **Efficiency**: Optimize for low bandwidth (typically 2-10 kbps)
3. **Scalability**: Support networks of 20+ vehicles/sensors
4. **Adaptability**: Dynamic adjustment to changing ocean conditions
5. **Prioritization**: Critical messages get priority delivery
6. **Interoperability**: Compatible with existing acoustic modem standards

---

## 2. Physical Layer Specification

### 2.1 Acoustic Frequency Bands

WIA Deep Sea Exploration defines four frequency bands for different use cases:

| Band | Frequency Range | Typical Range | Data Rate | Use Case |
|------|----------------|---------------|-----------|----------|
| VLF | 1-3 kHz | 10-20 km | 100-500 bps | Long-range coordination |
| LF | 3-8 kHz | 5-10 km | 500-2000 bps | Inter-vehicle communication |
| MF | 8-16 kHz | 1-5 km | 2-10 kbps | Standard telemetry |
| HF | 16-30 kHz | 100-1000 m | 10-50 kbps | High-bandwidth short-range |

### 2.2 Modulation Schemes

**Primary Modulation:** Frequency Shift Keying (FSK)
- Binary FSK (BFSK) for long-range, low-rate
- M-ary FSK (MFSK) for higher data rates

**Secondary Modulation:** Phase Shift Keying (PSK)
- QPSK for medium-range applications
- OFDM for high-bandwidth short-range

### 2.3 Signal Parameters

**Transmission Power:**
- Maximum: 190 dB re 1 μPa @ 1m
- Adjustable in 1 dB steps
- Automatic power control based on link quality

**Signal Bandwidth:**
- VLF/LF: 500 Hz - 2 kHz
- MF: 2 kHz - 8 kHz
- HF: 8 kHz - 14 kHz

**Transmission Duration:**
- Minimum packet: 100 ms
- Maximum packet: 10 seconds
- Adaptive based on link conditions

### 2.4 Physical Layer Frame Format

```
┌─────────────┬──────────┬─────────────┬──────────┬─────────┐
│  Preamble   │   Sync   │   Header    │ Payload  │   FCS   │
│  (50-100ms) │ (10-20ms)│  (20-40ms)  │ Variable │ (10-20ms)│
└─────────────┴──────────┴─────────────┴──────────┴─────────┘
```

**Preamble:** Known sequence for receiver synchronization and channel estimation
**Sync:** Start-of-frame delimiter
**Header:** Modulation scheme, packet length, addressing
**Payload:** User data
**FCS:** Frame Check Sequence (CRC-32)

---

## 3. Data Link Layer Protocol

### 3.1 Frame Structure

```
0                   1                   2                   3
0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
┌───────────────┬───────────────┬───────────────────────────────┐
│   Version (4) │   Type (4)    │        Sequence Number        │
├───────────────┴───────────────┼───────────────────────────────┤
│          Source Address       │      Destination Address      │
├───────────────────────────────┼───┬───────────┬───────────────┤
│         Payload Length        │Pri│  Reserved │   Hop Count   │
├───────────────────────────────┴───┴───────────┴───────────────┤
│                         Timestamp                             │
├───────────────────────────────────────────────────────────────┤
│                         Payload Data                          │
│                        (0-256 bytes)                          │
├───────────────────────────────────────────────────────────────┤
│                          CRC-32                               │
└───────────────────────────────────────────────────────────────┘
```

**Field Descriptions:**

| Field | Size | Description |
|-------|------|-------------|
| Version | 4 bits | Protocol version (current: 1) |
| Type | 4 bits | Frame type (DATA, ACK, NACK, BEACON) |
| Sequence Number | 16 bits | Monotonically increasing sequence number |
| Source Address | 16 bits | Transmitter node address |
| Destination Address | 16 bits | Receiver node address (0xFFFF = broadcast) |
| Payload Length | 10 bits | Length of payload in bytes (0-256) |
| Priority | 2 bits | Priority level (0=LOW, 1=NORMAL, 2=HIGH, 3=CRITICAL) |
| Reserved | 4 bits | Reserved for future use |
| Hop Count | 8 bits | TTL for multi-hop routing |
| Timestamp | 32 bits | Unix timestamp (seconds) |
| Payload Data | Variable | User data (0-256 bytes) |
| CRC-32 | 32 bits | Cyclic redundancy check |

### 3.2 Frame Types

| Type Code | Name | Description |
|-----------|------|-------------|
| 0x0 | DATA | Standard data packet |
| 0x1 | ACK | Acknowledgement packet |
| 0x2 | NACK | Negative acknowledgement |
| 0x3 | BEACON | Network discovery/sync |
| 0x4 | ROUTE_REQUEST | Routing discovery |
| 0x5 | ROUTE_REPLY | Routing response |
| 0x6 | EMERGENCY | Emergency alert |
| 0x7 | CONTROL | Network control message |

### 3.3 Error Correction

**Forward Error Correction (FEC):**
- Reed-Solomon RS(255, 223) for payload
- Convolutional coding (rate 1/2, K=7) for header
- Interleaving depth: 4 frames

**Automatic Repeat Request (ARQ):**
- Stop-and-Wait ARQ for critical data
- Selective Repeat ARQ for bulk data
- Timeout values adaptive based on round-trip time
- Maximum retransmissions: 5

**Hybrid ARQ:**
- Combine FEC and ARQ for optimal performance
- Type-II HARQ with incremental redundancy
- Chase combining for repeated transmissions

### 3.4 Channel Access Control

**MAC Protocol:** TDMA with dynamic slot allocation

**Superframe Structure (10 seconds):**
```
┌────────────┬──────────────────────────────────┬──────────────┐
│  Beacon    │         Data Slots (20)          │   Contention │
│  (500ms)   │        (400ms each)              │   (500ms)    │
└────────────┴──────────────────────────────────┴──────────────┘
```

**Slot Allocation:**
- Network coordinator broadcasts beacon with slot schedule
- Nodes request slots based on traffic demand
- Dynamic reallocation every 10 superframes
- Priority-based slot assignment

**Collision Avoidance:**
- Carrier Sense Multiple Access (CSMA) in contention period
- Binary exponential backoff on collision
- Random backoff window: 0 to 2^n - 1 (n = retry count)

---

## 4. Network Layer Protocol

### 4.1 Addressing Scheme

**Node Address Format:**
```
┌────────────┬────────────────┐
│ Network ID │    Node ID     │
│  (8 bits)  │    (8 bits)    │
└────────────┴────────────────┘
```

- **Network ID:** Identifies mission/deployment (0-255)
- **Node ID:** Unique node within network (0-255)
- **Reserved Addresses:**
  - 0x0000: Invalid/Unassigned
  - 0xFFFF: Broadcast to all nodes
  - 0xFF00-0xFFFE: Multicast groups

### 4.2 Routing Protocol

**Protocol:** WIA-UWRP (Underwater Routing Protocol)

**Routing Algorithm:** Hybrid reactive/proactive
- Proactive: Maintain routes to frequently contacted nodes
- Reactive: On-demand route discovery for infrequent destinations

**Route Discovery:**
1. Source broadcasts ROUTE_REQUEST
2. Intermediate nodes rebroadcast (with hop count increment)
3. Destination responds with ROUTE_REPLY
4. Source selects best route based on:
   - Hop count
   - Link quality
   - Estimated delay
   - Node energy

**Route Maintenance:**
- Periodic HELLO messages to neighbors
- Link quality monitoring via SNR and packet loss
- Route timeout: 300 seconds (configurable)
- Backup route maintenance for critical paths

**Routing Metrics:**
```
Route_Cost = α × HopCount + β × Delay + γ × PacketLoss + δ × (1/Energy)

where:
  α = 1.0 (hop count weight)
  β = 0.5 (delay weight, seconds)
  γ = 10.0 (packet loss weight, percentage)
  δ = 0.1 (energy weight, inverse of remaining battery %)
```

### 4.3 Geographic Routing (Optional)

For nodes with position knowledge:

**GPSR (Greedy Perimeter Stateless Routing):**
- Forward to neighbor closest to destination
- Perimeter mode when greedy fails
- Face routing algorithm for guaranteed delivery

**Position Dissemination:**
- Nodes broadcast position every 60 seconds
- Position cache timeout: 300 seconds
- Depth-aware routing to avoid thermoclines

---

## 5. Transport Layer Protocol

### 5.1 WIA-UTP (Underwater Transport Protocol)

**Features:**
- Reliable, connection-oriented transport
- Adaptive timeout based on measured RTT
- Congestion control for shared medium
- Segmentation and reassembly for large messages

**Connection Establishment (3-way handshake):**
```
Node A                           Node B
  │                                │
  │───── SYN (seq=100) ───────────>│
  │                                │
  │<──── SYN-ACK (seq=200, ────────│
  │           ack=101)             │
  │                                │
  │───── ACK (seq=101, ───────────>│
  │           ack=201)             │
  │                                │
  │        CONNECTION OPEN         │
```

**Timeout Calculation:**
```
RTT_measured = time(ACK_received) - time(packet_sent)
RTT_smooth = (1-α) × RTT_smooth + α × RTT_measured    (α = 0.125)
RTT_variance = (1-β) × RTT_variance + β × |RTT_smooth - RTT_measured|  (β = 0.25)
Timeout = RTT_smooth + 4 × RTT_variance

Minimum timeout: 5 seconds
Maximum timeout: 60 seconds
```

### 5.2 Message Fragmentation

For messages exceeding 256 bytes:

```
Original Message (1024 bytes)
        ↓
┌──────────┬──────────┬──────────┬──────────┐
│Fragment 0│Fragment 1│Fragment 2│Fragment 3│
│(256 byte)│(256 byte)│(256 byte)│(256 byte)│
└──────────┴──────────┴──────────┴──────────┘

Fragment Header:
  Fragment ID: 16 bits (identifies original message)
  Fragment Number: 8 bits (0 to N-1)
  Total Fragments: 8 bits (N)
  Fragment Offset: 16 bits (byte offset in original)
```

**Reassembly:**
- Receiver buffers fragments until all received
- Out-of-order delivery supported
- Reassembly timeout: 120 seconds
- Selective ACK for missing fragments

---

## 6. Application Layer Protocols

### 6.1 Command/Response Protocol

**Command Message Format:**
```json
{
  "messageType": "COMMAND",
  "commandId": "CMD-12345",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "source": "SURFACE-STATION-01",
  "destination": "ROV-ATLANTIS-001",
  "command": "NAVIGATE_TO_WAYPOINT",
  "parameters": {
    "latitude": 36.7980,
    "longitude": -121.8475,
    "depth": 3550.0
  },
  "priority": "HIGH",
  "timeout": 300
}
```

**Response Message Format:**
```json
{
  "messageType": "RESPONSE",
  "commandId": "CMD-12345",
  "timestamp": "2025-01-15T14:30:05.000Z",
  "source": "ROV-ATLANTIS-001",
  "destination": "SURFACE-STATION-01",
  "status": "ACKNOWLEDGED",
  "result": {
    "executionStarted": true,
    "estimatedCompletion": "2025-01-15T14:35:00.000Z"
  }
}
```

### 6.2 Telemetry Streaming Protocol

**Compressed Telemetry Format:**

Due to bandwidth constraints, telemetry uses binary encoding:

```
┌────────┬──────────┬─────────┬─────────┬──────────┬─────────┐
│NodeID  │Timestamp │ Depth   │  Temp   │ Pressure │ Battery │
│(2 byte)│(4 byte)  │(2 byte) │(2 byte) │(2 byte)  │(1 byte) │
└────────┴──────────┴─────────┴─────────┴──────────┴─────────┘
       Total: 13 bytes (vs 200+ bytes in JSON)

Encoding:
  Depth: uint16, centimeters (0-65535 cm = 0-655.35 m)
  Temp: int16, 0.01°C resolution (-327.68°C to 327.67°C)
  Pressure: uint16, 0.1 bar resolution (0-6553.5 bar)
  Battery: uint8, percentage (0-100%)
```

### 6.3 File Transfer Protocol

**WIA-UFTP (Underwater File Transfer Protocol)**

Optimized for large file transfers over acoustic links:

**Features:**
- Chunked transfer with per-chunk acknowledgment
- Resume capability after connection interruption
- Priority-based chunk scheduling
- Compression (LZ4 algorithm)

**Transfer Process:**
1. Client sends FILE_REQUEST with metadata
2. Server responds with FILE_ACCEPT and chunk map
3. Server transmits chunks (with FEC)
4. Client acknowledges received chunks
5. Server retransmits missing chunks
6. Client sends FILE_COMPLETE when all chunks received

**Typical Performance:**
- 1 MB file over 5 kbps link: ~35 minutes
- 10 MB file over 10 kbps link: ~2.5 hours
- Compression ratio: ~2-3x for text/logs
- Effective throughput: 60-80% of link capacity

---

## 7. Multi-Vehicle Coordination Protocol

### 7.1 Distributed Consensus

For coordinated operations (e.g., synchronized sampling):

**Protocol:** Modified Raft consensus algorithm adapted for high-latency networks

**Key Modifications:**
- Extended heartbeat intervals (30 seconds vs milliseconds)
- Acoustic-aware leader election
- Geographic proximity preference for leader selection

### 7.2 Time Synchronization

**Precision Time Protocol for Underwater (PTPu):**

**Sync Message Exchange:**
```
Surface Station                    ROV
     │                              │
     │──── SYNC ───────────────────>│ (t1)
     │                              │ (t2)
     │<──── DELAY_REQ ──────────────│ (t3)
     │                              │
     │──── DELAY_RESP ─────────────>│ (t4)

Offset = [(t2 - t1) - (t4 - t3)] / 2
Delay = [(t2 - t1) + (t4 - t3)] / 2
```

**Accuracy:**
- Target: ±100 ms synchronization
- Sync interval: 60 seconds
- Drift compensation for vehicle clocks

---

## 8. Emergency Communication Protocol

### 8.1 Emergency Alert Format

**Highest priority, maximum transmission power:**

```
┌──────────┬──────────┬──────────────┬────────────────┐
│ EMERGENCY│ Node ID  │  Location    │  Emergency     │
│  Flag    │(2 bytes) │ (Lat/Lon/Dep)│  Type          │
│(0xFF)    │          │ (12 bytes)   │  (1 byte)      │
└──────────┴──────────┴──────────────┴────────────────┘

Emergency Types:
  0x01: CRITICAL_SYSTEM_FAILURE
  0x02: LOSS_OF_PROPULSION
  0x03: LEAK_DETECTED
  0x04: POWER_CRITICAL
  0x05: ENTANGLEMENT
  0x06: OPERATOR_EMERGENCY
```

**Emergency Protocol:**
1. Node broadcasts emergency at maximum power
2. All receiving nodes rebroadcast (flooding)
3. Surface station acknowledges
4. Rescue protocol initiated

### 8.2 Emergency Ascent Beacon

Continuous acoustic beacon during emergency ascent:

- Frequency: 10 kHz
- Pattern: 1-second pulse every 5 seconds
- Duration: Until surface or battery depletion
- Contains node ID and last known position

---

## 9. Quality of Service (QoS)

### 9.1 Priority Classes

| Priority | Class | Max Delay | Use Case |
|----------|-------|-----------|----------|
| 3 | CRITICAL | 5 seconds | Emergency alerts |
| 2 | HIGH | 30 seconds | Vehicle commands |
| 1 | NORMAL | 2 minutes | Telemetry |
| 0 | LOW | Best effort | Bulk data transfer |

### 9.2 QoS Implementation

**Priority Queue Management:**
- Strict priority scheduling for CRITICAL
- Weighted fair queueing for others
- Starvation prevention for LOW priority (minimum 10% bandwidth)

**Bandwidth Allocation:**
- CRITICAL: 20% reserved
- HIGH: 40% reserved
- NORMAL: 30% reserved
- LOW: 10% minimum

---

## 10. Protocol Compliance Testing

### 10.1 Conformance Tests

Required tests for WIA Deep Sea Exploration Protocol compliance:

1. **Physical Layer Tests:**
   - Frequency accuracy: ±10 Hz
   - Power output: ±1 dB of specified
   - Modulation quality: EVM < -15 dB

2. **Data Link Layer Tests:**
   - FEC decoding: BER < 10^-6 after correction
   - ARQ functionality: successful retransmission
   - Frame parsing: 100% success rate

3. **Network Layer Tests:**
   - Route discovery: < 30 seconds
   - Route maintenance: link failure detection < 60 seconds
   - Routing loop prevention: verified

4. **Transport Layer Tests:**
   - Connection establishment: < 15 seconds
   - Reliable delivery: 100% for non-timeout cases
   - Fragmentation/reassembly: verified for up to 10 KB

5. **Application Layer Tests:**
   - Command/response: end-to-end latency < 60 seconds
   - Telemetry: delivery rate > 95%
   - Emergency alerts: < 5 seconds delivery

---

**Document Control:**
- Author: WIA Standards Committee
- Contributors: MIT AUV Lab, WHOI Acoustic Communications Group
- License: CC BY 4.0

弘益人間 · Benefit All Humanity
