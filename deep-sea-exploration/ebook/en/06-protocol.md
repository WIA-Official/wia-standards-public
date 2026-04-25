# Chapter 6: Communication Protocol (Phase 3)

## Underwater Communication, Networking, and Real-Time Data Transmission

---

## 6.1 Acoustic Communication Protocols

### The Underwater Channel

Acoustic communication in the ocean presents unique challenges compared to terrestrial wireless systems. Sound travels approximately 1,500 m/s in seawater (vs. 300,000,000 m/s for radio), creating significant latency. The channel is characterized by:

- **Multipath propagation**: Sound reflects off surface, bottom, and thermoclines
- **Doppler effects**: Moving vehicles shift frequencies
- **Ambient noise**: Shipping, marine life, weather
- **Variable sound speed**: Temperature and salinity gradients bend sound paths

### WIA Acoustic Protocol Stack

```
┌────────────────────────────────────────────┐
│  Application Layer (WIA Messages)          │
├────────────────────────────────────────────┤
│  Transport Layer (Reliable/Unreliable)     │
├────────────────────────────────────────────┤
│  Network Layer (Addressing, Routing)       │
├────────────────────────────────────────────┤
│  Data Link Layer (Framing, ARQ)            │
├────────────────────────────────────────────┤
│  Physical Layer (Acoustic Modulation)      │
└────────────────────────────────────────────┘
```

### Frame Format

**Acoustic Frame Structure**:

| Field | Size | Description |
|-------|------|-------------|
| Preamble | 16 bits | Synchronization pattern |
| Sync | 8 bits | Frame delimiter |
| Source Address | 8 bits | Sender ID (0-255) |
| Destination | 8 bits | Recipient ID (0-255, 255=broadcast) |
| Sequence | 16 bits | Frame sequence number |
| Type | 4 bits | Frame type (DATA, ACK, NAK, PING) |
| Priority | 4 bits | Transmission priority |
| Length | 16 bits | Payload length |
| Payload | Variable | Data (max 1024 bytes) |
| CRC-32 | 32 bits | Error detection |

### Modulation Schemes

| Scheme | Data Rate | Range | Robustness |
|--------|-----------|-------|------------|
| FSK | 100-1,000 bps | 10+ km | High |
| PSK | 1-10 kbps | 1-5 km | Medium |
| QAM | 10-100 kbps | 0.5-2 km | Lower |
| OFDM | 10-50 kbps | 1-5 km | Medium-High |

### Reliable Transmission

**Stop-and-Wait ARQ**:
```
Sender                    Receiver
  |                          |
  |-------- DATA[0] -------->|
  |                          | Process
  |<-------- ACK[0] ---------|
  |                          |
  |-------- DATA[1] -------->|
  |                          | Process
  |<-------- ACK[1] ---------|
```

**Timeout and Retry**:
```python
TIMEOUT_BASE = 2 * (range_meters / 1500) + processing_time
MAX_RETRIES = 3
BACKOFF_FACTOR = 1.5

for attempt in range(MAX_RETRIES):
    send_frame(data)
    timeout = TIMEOUT_BASE * (BACKOFF_FACTOR ** attempt)
    if wait_for_ack(timeout):
        return SUCCESS
    log_retry(attempt)
return FAILURE
```

---

## 6.2 Fiber-Optic Tether Specifications

### Tether Architecture

For ROV operations requiring high bandwidth, fiber-optic tethers provide gigabit connectivity:

**Single-Mode Fiber Specifications**:

| Parameter | Value |
|-----------|-------|
| Fiber type | G.652D single-mode |
| Core diameter | 9 μm |
| Cladding diameter | 125 μm |
| Wavelength | 1310 nm, 1550 nm |
| Attenuation | 0.35 dB/km @ 1310nm |
| Bandwidth | 10+ Gbps |
| Max length | 10 km (practical) |

### Tether Data Channels

| Channel | Bandwidth | Direction | Content |
|---------|-----------|-----------|---------|
| Video 1 | 50 Mbps | Up | HD main camera |
| Video 2 | 25 Mbps | Up | HD aux camera |
| Telemetry | 10 Mbps | Up | Sensors, navigation |
| Control | 1 Mbps | Down | Commands, waypoints |
| Voice | 64 kbps | Bidirectional | Pilot communication |
| Emergency | 10 kbps | Bidirectional | Priority channel |

### Physical Layer Protocol

**Frame Structure (Ethernet-based)**:
```
┌──────────────────────────────────────────────────┐
│ Preamble │ SFD │ Dest │ Src │ Type │ Data │ FCS │
│  7 bytes │ 1B  │ 6B   │ 6B  │  2B  │ var  │ 4B  │
└──────────────────────────────────────────────────┘
```

**Quality of Service (QoS) Classes**:

| Class | Priority | Examples |
|-------|----------|----------|
| CRITICAL | 7 | Emergency abort |
| HIGH | 5-6 | Control commands |
| NORMAL | 3-4 | Telemetry, video |
| LOW | 1-2 | Bulk data transfer |
| BACKGROUND | 0 | Non-urgent logs |

---

## 6.3 Mesh Networking for Multi-Vehicle Operations

### Network Topology

Multi-vehicle operations require coordinated networking:

```
          Surface Vessel
               │
     ┌─────────┼─────────┐
     │         │         │
   AUV-1    AUV-2    AUV-3
     │         │         │
     └────┬────┴────┬────┘
          │         │
     Seafloor Transponder Network
```

### Addressing Scheme

**WIA Network Address Format**:
```
[Network ID].[Vehicle Type].[Instance]

Examples:
10.1.1    - Network 10, ROV type, instance 1
10.2.5    - Network 10, AUV type, instance 5
10.3.1    - Network 10, sensor node, instance 1
10.0.255  - Network 10, broadcast address
```

### Routing Protocol

**Distance Vector Routing**:
```python
class AcousticRouter:
    def __init__(self, node_id):
        self.node_id = node_id
        self.routing_table = {}  # destination -> (next_hop, cost)

    def update_route(self, destination, next_hop, cost):
        current = self.routing_table.get(destination, (None, float('inf')))
        if cost < current[1]:
            self.routing_table[destination] = (next_hop, cost)
            self.broadcast_update()

    def get_next_hop(self, destination):
        if destination in self.routing_table:
            return self.routing_table[destination][0]
        return None  # No route known
```

### Time-Division Multiple Access (TDMA)

For coordinated multi-vehicle operations:

```
|--Slot 1--|--Slot 2--|--Slot 3--|--Guard--|
|  AUV-1   |  AUV-2   |  AUV-3   |  Empty  |
|  100ms   |  100ms   |  100ms   |  50ms   |

Frame period: 350ms (2.86 frames/second)
```

---

## 6.4 Data Compression and Prioritization

### Compression Algorithms

| Data Type | Algorithm | Compression Ratio |
|-----------|-----------|-------------------|
| Telemetry (JSON) | GZIP | 5-10x |
| Telemetry (binary) | LZ4 | 2-3x |
| Bathymetry | Delta + GZIP | 10-20x |
| Video | H.265 | 100-200x |
| Images | JPEG/WebP | 10-30x |

### Priority Queue

```python
class PriorityMessageQueue:
    def __init__(self):
        self.queues = {
            'CRITICAL': deque(maxlen=10),
            'HIGH': deque(maxlen=50),
            'NORMAL': deque(maxlen=200),
            'LOW': deque(maxlen=500)
        }

    def enqueue(self, message, priority):
        self.queues[priority].append(message)

    def dequeue(self):
        for priority in ['CRITICAL', 'HIGH', 'NORMAL', 'LOW']:
            if self.queues[priority]:
                return self.queues[priority].popleft()
        return None
```

### Adaptive Transmission

```python
def select_transmission_mode(channel_quality, message_priority):
    if message_priority == 'CRITICAL':
        return {'modulation': 'FSK', 'fec': 'TURBO', 'rate': 'LOW'}

    if channel_quality > 0.9:
        return {'modulation': 'QAM16', 'fec': 'LDPC', 'rate': 'HIGH'}
    elif channel_quality > 0.7:
        return {'modulation': 'PSK', 'fec': 'CONV', 'rate': 'MEDIUM'}
    else:
        return {'modulation': 'FSK', 'fec': 'TURBO', 'rate': 'LOW'}
```

---

## 6.5 Error Correction and Retry Mechanisms

### Forward Error Correction (FEC)

| Code | Overhead | Correction Capability |
|------|----------|----------------------|
| Convolutional (1/2) | 100% | Moderate |
| Reed-Solomon | 10-30% | Burst errors |
| LDPC | 10-50% | Near Shannon limit |
| Turbo | 33-100% | Excellent |

### Interleaving

Combats burst errors by spreading bits across time:

```
Original:    AAAABBBBCCCCDDDD
Interleaved: ABCDABCDABCDABCD

Burst error hits: A_CD__CD__CD__CD
De-interleaved:   A___B___C___D___
                  (Single errors per block - correctable)
```

### Acknowledgment Strategies

**Selective ACK**:
```json
{
  "type": "SACK",
  "received": [1, 2, 3, 5, 6, 8],
  "missing": [4, 7],
  "timestamp": "2025-01-15T14:30:00.000Z"
}
```

**Negative ACK (NAK)**:
```json
{
  "type": "NAK",
  "sequence": 4,
  "reason": "CRC_FAILURE",
  "retryRequest": true
}
```

---

## 6.6 Surface-to-Underwater Gateways

### Gateway Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Surface Gateway                     │
├─────────────────────────────────────────────────────┤
│  ┌─────────┐  ┌──────────┐  ┌─────────────────────┐│
│  │ Sat Link │  │ Ship LAN │  │ Acoustic Transceiver││
│  └────┬────┘  └────┬─────┘  └──────────┬──────────┘│
│       └────────────┴──────────────────-┘           │
│                     │                               │
│              Protocol Converter                     │
│                     │                               │
│  ┌──────────────────┴───────────────────────────┐  │
│  │          WIA Message Router                   │  │
│  └───────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

### Protocol Translation

**HTTP/REST to Acoustic**:
```python
def translate_http_to_acoustic(http_request):
    # Extract essential command
    command = {
        'type': http_request.method,
        'path': compress_path(http_request.path),
        'body': compress_json(http_request.json) if http_request.json else None
    }

    # Encode for acoustic transmission
    acoustic_frame = encode_compact(command)
    return acoustic_frame

def compress_path(path):
    # /api/v1/commands -> 0x01 (lookup table)
    path_table = {'/api/v1/commands': 0x01, '/api/v1/telemetry': 0x02}
    return path_table.get(path, path)
```

### Store-and-Forward

For AUV missions beyond acoustic range:

```python
class StoreAndForward:
    def __init__(self):
        self.message_store = []

    def store_for_delivery(self, message, destination, expires):
        self.message_store.append({
            'message': message,
            'destination': destination,
            'expires': expires,
            'stored_at': datetime.utcnow()
        })

    def on_vehicle_contact(self, vehicle_id):
        # Deliver stored messages when vehicle comes in range
        to_deliver = [m for m in self.message_store
                      if m['destination'] == vehicle_id
                      and m['expires'] > datetime.utcnow()]
        for msg in to_deliver:
            self.transmit(msg)
            self.message_store.remove(msg)
```

---

## 6.7 Emergency Communication Procedures

### Emergency Message Types

| Type | Code | Description |
|------|------|-------------|
| ABORT | 0x01 | Immediate surface |
| EMERGENCY_SURFACE | 0x02 | Controlled ascent |
| LOST_COMMUNICATION | 0x03 | Comm failure |
| POWER_CRITICAL | 0x04 | Low power |
| ENTANGLEMENT | 0x05 | Physical trap |
| FLOODING | 0x06 | Water ingress |
| FIRE | 0x07 | Onboard fire |

### Emergency Frame

```
┌────────────────────────────────────────────────┐
│ EMERGENCY FRAME (minimum 16 bytes)              │
├────────────────────────────────────────────────┤
│ Magic: 0xEMER (4 bytes)                        │
│ Source ID (2 bytes)                            │
│ Emergency Type (1 byte)                        │
│ Severity (1 byte): 1-5                         │
│ Position (8 bytes): Lat/Lon/Depth compressed   │
│ CRC-16 (2 bytes)                               │
└────────────────────────────────────────────────┘
```

### Emergency Response Protocol

```python
def handle_emergency(emergency_frame):
    # 1. Acknowledge immediately
    send_ack(emergency_frame.source, priority='CRITICAL')

    # 2. Alert all operators
    broadcast_alert(emergency_frame)

    # 3. Log with timestamp
    log_emergency(emergency_frame)

    # 4. Initiate response based on type
    if emergency_frame.type == ABORT:
        initiate_abort_sequence(emergency_frame.source)
    elif emergency_frame.type == LOST_COMMUNICATION:
        start_recovery_protocol(emergency_frame.source)
```

### Automatic Recovery Behaviors

| Condition | Trigger | Response |
|-----------|---------|----------|
| Lost comms | No contact 5 min | Surface to last known position |
| Low power | <10% battery | Abort mission, surface |
| Depth exceeded | >max depth | Emergency ascent |
| Entanglement | No motion 2 min | Alert, drop weights |

---

## Chapter Summary

Phase 3 of the WIA Deep Sea Exploration Standard defines robust communication protocols for the challenging underwater environment. From acoustic frame formats to fiber-optic QoS, the standard addresses the full range of communication scenarios encountered in deep-sea operations.

Multi-vehicle networking enables coordinated operations, while priority queuing ensures critical messages are delivered first. Comprehensive error correction and retry mechanisms maximize reliability over the lossy acoustic channel.

Emergency procedures ensure that safety-critical communications always get through, with automatic recovery behaviors providing additional protection for vehicles operating beyond communication range.

---

## Key Takeaways

1. **Acoustic communication is 10,000x slower than terrestrial wireless**
2. **Fiber-optic tethers provide gigabit bandwidth** but limit mobility
3. **Multi-vehicle networks use TDMA** for coordinated access
4. **Priority queuing ensures critical messages** are transmitted first
5. **Emergency frames use minimal overhead** for reliability

---

## Review Questions

1. What is the approximate latency for acoustic communication to Challenger Deep (11,000m)?
2. Compare FSK and QAM modulation for underwater acoustic communication.
3. Design a TDMA schedule for 5 AUVs sharing an acoustic channel.
4. What error correction code would you choose for a noisy channel?
5. Describe the automatic recovery behavior for a lost communication scenario.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
