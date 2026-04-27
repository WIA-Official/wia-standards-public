# PHASE 1 — Data Format

> V2X canonical message envelopes: BSM, CAM, DENM, SPaT, MAP,
> and the technology-standards landscape (DSRC, C-V2X PC5,
> 5G NR-V2X) that frames the wire formats. Performance
> requirements (latency, packet rate, range) live alongside the
> formats because they constrain valid encodings.

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework for Vehicle-to-Everything (V2X) communication systems, enabling vehicles to exchange safety-critical information with other vehicles, infrastructure, pedestrians, and cloud networks.

### 1.2 Scope

The standard covers:
- Communication protocols for V2V, V2I, V2P, and V2N
- Message format specifications
- Security and privacy mechanisms
- Latency and reliability requirements
- Collision avoidance algorithms
- Platooning coordination protocols

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to save lives through vehicular communication, reduce traffic congestion, enable autonomous driving, and create a safer transportation ecosystem for all road users.

### 1.4 Terminology

- **V2X**: Vehicle-to-Everything communication
- **V2V**: Vehicle-to-Vehicle communication
- **V2I**: Vehicle-to-Infrastructure communication
- **V2P**: Vehicle-to-Pedestrian communication
- **V2N**: Vehicle-to-Network communication
- **BSM**: Basic Safety Message (SAE J2735)
- **CAM**: Cooperative Awareness Message (ETSI)
- **DENM**: Decentralized Environmental Notification Message
- **RSU**: Road Side Unit
- **OBU**: On-Board Unit
- **DSRC**: Dedicated Short-Range Communications (IEEE 802.11p)
- **C-V2X**: Cellular V2X (LTE-V2X, 5G NR-V2X)

---


## 2. V2X Communication Architecture

### 2.1 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    V2X Communication System                  │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────┐     ┌──────────┐     ┌──────────┐            │
│  │ Vehicle  │────▶│   V2V    │◀────│ Vehicle  │            │
│  │   OBU    │     │ Messages │     │   OBU    │            │
│  └────┬─────┘     └──────────┘     └──────────┘            │
│       │                                                       │
│       │           ┌──────────┐                               │
│       ├──────────▶│   V2I    │◀────┐                        │
│       │           │ Messages │     │                        │
│       │           └──────────┘     │                        │
│       │                            │                        │
│       │           ┌──────────┐   ┌┴─────┐                  │
│       ├──────────▶│   V2P    │   │ RSU  │                  │
│       │           │ Messages │   └──────┘                  │
│       │           └──────────┘                               │
│       │                                                       │
│       │           ┌──────────┐                               │
│       └──────────▶│   V2N    │◀──── Cloud Services         │
│                   │ Messages │                               │
│                   └──────────┘                               │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Communication Stack

```
┌─────────────────────────────────────┐
│     Application Layer               │  V2X Applications
├─────────────────────────────────────┤
│     Facilities Layer                │  Message generation/processing
├─────────────────────────────────────┤
│     Security Layer                  │  Authentication, Encryption
├─────────────────────────────────────┤
│     Network & Transport Layer       │  IPv6, TCP/UDP
├─────────────────────────────────────┤
│     Access Layer                    │  DSRC (802.11p) or C-V2X
├─────────────────────────────────────┤
│     Physical Layer                  │  5.9 GHz radio
└─────────────────────────────────────┘
```

### 2.3 Frequency Allocation

**ITS Band (5.9 GHz):**
- 5.850 - 5.925 GHz (US, 75 MHz)
- 5.875 - 5.905 GHz (EU, 30 MHz)
- 5.770 - 5.850 GHz (Japan, 80 MHz)

**Channel Allocation:**
```
Channel 172: 5.860 GHz (Safety critical - CCH)
Channel 174: 5.870 GHz (Service channel)
Channel 176: 5.880 GHz (Service channel)
Channel 178: 5.890 GHz (Service channel)
Channel 180: 5.900 GHz (Service channel)
Channel 182: 5.910 GHz (Service channel)
Channel 184: 5.920 GHz (Service channel)
```

---


## 3. Technology Standards

### 3.1 DSRC (IEEE 802.11p)

**Physical Layer:**
- Frequency: 5.9 GHz
- Bandwidth: 10 MHz per channel
- Modulation: OFDM (BPSK, QPSK, 16-QAM, 64-QAM)
- Data rate: 3-27 Mbps

**MAC Layer:**
- Protocol: CSMA/CA (Carrier Sense Multiple Access)
- No association required
- Broadcast and unicast support
- Quality of Service (EDCA)

**Range and Performance:**
```
Line-of-Sight:      Up to 1000 meters
Urban:              300-500 meters
Highway:            500-800 meters
Latency:            5-10 ms (typical)
Packet Loss:        <5% (at 300m)
```

### 3.2 C-V2X (Cellular V2X)

**LTE-V2X (Release 14):**
- Frequency: 5.9 GHz (PC5) or cellular bands (Uu)
- Mode 3: Network-scheduled (eNodeB)
- Mode 4: Autonomous sensing and scheduling
- Sidelink communication (PC5 interface)

**5G NR-V2X (Release 16+):**
- Ultra-reliable low-latency (URLLC)
- Enhanced sidelink (PC5)
- Network slicing support
- Higher data rates (up to 1 Gbps)

**Performance:**
```
LTE-V2X:
  Range:            500-1500 meters
  Latency:          10-20 ms
  Reliability:      98% (at 500m)

5G NR-V2X:
  Range:            1000-2000 meters
  Latency:          1-5 ms
  Reliability:      99.9% (at 500m)
```

### 3.3 Coexistence

**Hybrid Deployment:**
- Dual-mode devices (DSRC + C-V2X)
- Message translation layer
- Technology selection based on availability
- Handover protocols

---


## 4. Message Formats

### 4.1 Basic Safety Message (BSM)

**BSM Part I (Core data, 10 Hz):**
```json
{
  "messageType": "BSM",
  "msgCount": 127,
  "id": "00000001",
  "timestamp": 65535,
  "position": {
    "latitude": 37774900000,    // 1/10 micro degree
    "longitude": -122419400000,
    "elevation": 100,            // decimeters
    "accuracy": {
      "semiMajor": 12,          // cm
      "semiMinor": 10,
      "orientation": 0
    }
  },
  "speed": 1800,                // 0.02 m/s units (= 36 m/s = 65 km/h)
  "heading": 18000,             // 0.0125 degree units (= 225°)
  "acceleration": {
    "longitudinal": 50,         // 0.01 m/s²
    "lateral": 0,
    "vertical": 0,
    "yawRate": 100              // 0.01 deg/s
  },
  "steeringAngle": 0,           // 1.5 degrees per LSB
  "brakeStatus": {
    "wheelBrakes": "00000",
    "tractionControl": "off",
    "abs": "off",
    "stabilityControl": "off",
    "brakeBoost": "off",
    "auxBrakes": "off"
  },
  "vehicleSize": {
    "width": 200,               // cm
    "length": 480               // cm
  }
}
```

**BSM Part II (Optional, event-driven):**
- Path history (previous positions)
- Path prediction (intended trajectory)
- Vehicle classification
- Weather information
- Road surface conditions

### 4.2 Cooperative Awareness Message (CAM)

**CAM Structure (ETSI EN 302 637-2):**
```json
{
  "messageType": "CAM",
  "protocolVersion": 1,
  "stationID": 123456,
  "generationTime": 1640000000000,
  "basicContainer": {
    "stationType": "passengerCar",
    "referencePosition": {
      "latitude": 377749000,
      "longitude": -1224194000,
      "altitude": 100,
      "confidence": {
        "position": "a50m",
        "altitude": "alt-005-00"
      }
    }
  },
  "highFrequencyContainer": {
    "heading": 2250,              // 0.1 degree
    "speed": 180,                 // 0.01 m/s
    "driveDirection": "forward",
    "vehicleLength": 48,          // 0.1 m
    "vehicleWidth": 20,           // 0.1 m
    "longitudinalAcceleration": 5, // 0.1 m/s²
    "curvature": 0,
    "yawRate": 10                 // 0.01 deg/s
  }
}
```

### 4.3 Decentralized Environmental Notification Message (DENM)

**DENM Structure (ETSI EN 302 637-3):**
```json
{
  "messageType": "DENM",
  "protocolVersion": 1,
  "stationID": 123456,
  "management": {
    "actionID": {
      "originatingStationID": 123456,
      "sequenceNumber": 1
    },
    "detectionTime": 1640000000000,
    "referenceTime": 1640000000000,
    "eventPosition": {
      "latitude": 377749000,
      "longitude": -1224194000,
      "altitude": 100
    },
    "relevanceDistance": "lessThan500m",
    "relevanceTrafficDirection": "allTrafficDirections",
    "validityDuration": 600,      // seconds
    "transmissionInterval": 1000  // ms
  },
  "situation": {
    "eventType": {
      "causeCode": "collision",
      "subCauseCode": "chainCollision"
    },
    "severity": "danger"
  },
  "location": {
    "eventSpeed": 0,
    "eventPositionHeading": 1800,
    "traces": []
  }
}
```

### 4.4 Signal Phase and Timing (SPaT)

**SPaT Message (SAE J2735):**
```json
{
  "messageType": "SPaT",
  "timestamp": 65535,
  "intersectionID": 12345,
  "status": "signalOn",
  "states": [
    {
      "movementName": "Phase 1",
      "signalGroup": 1,
      "state": "protected-Movement-Allowed",
      "timing": {
        "minEndTime": 15000,      // ms
        "maxEndTime": 18000,
        "likelyTime": 16500,
        "confidence": 80,
        "nextTime": 75000
      },
      "maneuverAssist": {
        "connectionID": 1,
        "queueLength": 5,
        "availableStorageLength": 50,
        "waitOnStop": true,
        "pedBicycleDetect": false
      }
    }
  ]
}
```

### 4.5 Map Data (MAP)

**MAP Message (SAE J2735):**
```json
{
  "messageType": "MAP",
  "timestamp": 65535,
  "intersectionID": 12345,
  "referencePoint": {
    "latitude": 377749000,
    "longitude": -1224194000,
    "elevation": 100
  },
  "laneSet": [
    {
      "laneID": 1,
      "laneAttributes": {
        "directionalUse": "ingressPath",
        "sharedWith": "none",
        "laneType": "vehicle"
      },
      "maneuvers": ["straight", "left"],
      "nodeList": {
        "nodes": [
          {"delta": {"dx": 0, "dy": 0}},
          {"delta": {"dx": 500, "dy": 0}},
          {"delta": {"dx": 1000, "dy": 0}}
        ]
      },
      "connectsTo": [
        {
          "connectingLane": 5,
          "signalGroup": 1,
          "connectionID": 1
        }
      ],
      "speedLimits": [
        {
          "type": "vehicleMaxSpeed",
          "speed": 13.89  // m/s (50 km/h)
        }
      ]
    }
  ]
}
```

---


## 10. Performance Requirements

### 10.1 Latency Requirements

| Application | End-to-End Latency | Processing Time | Transmission Time |
|-------------|-------------------|-----------------|-------------------|
| Emergency Brake Warning | 5 ms | 2 ms | 3 ms |
| Collision Warning | 10 ms | 5 ms | 5 ms |
| Lane Change Warning | 20 ms | 10 ms | 10 ms |
| Traffic Signal Info | 100 ms | 50 ms | 50 ms |
| Map Update | 500 ms | 200 ms | 300 ms |

### 10.2 Reliability Requirements

**Packet Delivery Ratio (PDR):**
```
Safety-Critical (Collision Avoidance):  ≥ 99.9% at 300m
High Priority (Warnings):               ≥ 99% at 500m
Medium Priority (Awareness):            ≥ 95% at 500m
Low Priority (Info):                    ≥ 90% at 1000m
```

**Message Frequency:**
```
BSM/CAM (Position):        10 Hz (100 ms interval)
DENM (Event):              Event-triggered, min 1 Hz
SPaT (Traffic Signal):     10 Hz
MAP (Road Geometry):       1 Hz or on-change
PSM (Pedestrian):          2 Hz
```

### 10.3 Range Requirements

**Communication Range:**
```
V2V Direct:
  - Urban: 300-500m
  - Highway: 500-1000m
  - LOS: Up to 1000m

V2I (RSU):
  - Intersection: 300m radius
  - Highway: 500m radius
  - Range extension: Multi-hop relaying

V2P:
  - Pedestrian detection: 50-100m
  - Cyclist detection: 100-200m

V2N:
  - Cellular coverage dependent
  - Target: 99% coverage in urban areas
```

### 10.4 Scalability

**Network Capacity:**
```
Vehicles per Channel:
  - DSRC: 100-200 vehicles/channel (10 MHz)
  - C-V2X: 200-500 vehicles/channel
  - 5G NR-V2X: 500-1000 vehicles/channel

Congestion Control:
  - Adaptive message rate
  - Transmit power control
  - Channel coordination
  - Priority-based access
```

**Channel Load Management:**
```python
def adapt_message_rate(channel_busy_ratio):
    if channel_busy_ratio > 0.7:  # 70% channel busy
        # Reduce BSM rate
        return {
            'bsm_rate': 5,  # Hz (reduced from 10)
            'power': -10,   # dBm (reduced from 20)
            'priority': 'HIGH_ONLY'
        }
    elif channel_busy_ratio > 0.5:  # 50% busy
        return {
            'bsm_rate': 7,  # Hz
            'power': 0,     # dBm
            'priority': 'NORMAL'
        }
    else:
        return {
            'bsm_rate': 10,  # Hz (normal)
            'power': 20,     # dBm (max)
            'priority': 'ALL'
        }
```

---


