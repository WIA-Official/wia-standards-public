# WIA-MARITIME_SAFETY: Phase 3 - Protocol Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document defines the communication protocols, procedures, and standards for Maritime Safety operations, including AIS message handling, emergency procedures, and inter-system communication.

## 2. AIS Protocol Implementation

### 2.1 AIS Message Types

| Type | Name | Update Rate | Purpose |
|------|------|-------------|---------|
| 1, 2, 3 | Position Report | 2-10 seconds | Dynamic position information |
| 4 | Base Station Report | 10 seconds | Shore station position |
| 5 | Static and Voyage | 6 minutes | Vessel identity and voyage data |
| 18 | Class B Position | 30 seconds | Class B transponder position |
| 21 | Aid-to-Navigation | 3 minutes | Buoy, lighthouse positions |
| 24 | Static Data Report | 6 minutes | Class B vessel identity |

### 2.2 Message Priority Levels

```typescript
enum MessagePriority {
  DISTRESS = 0,      // Immediate transmission
  URGENT = 1,        // High priority (safety)
  ROUTINE = 2,       // Normal operations
  BACKGROUND = 3     // Low priority (informational)
}
```

### 2.3 AIS Encoding Protocol

**Message Format:**
```
!AIVDM,1,1,,A,13u?etPv2;0n:dDPwUM1U1Cb069D,0*24
```

**Field Breakdown:**
- `!AIVDM` - AIS message identifier
- `1` - Total fragments
- `1` - Fragment number
- `` - Sequential message ID
- `A` - VHF channel (A or B)
- `13u?et...` - Payload (6-bit ASCII)
- `0*24` - Fill bits and checksum

### 2.4 Position Report Encoding

```typescript
interface AISType1Message {
  messageType: 1 | 2 | 3;
  repeatIndicator: 0 | 1 | 2 | 3;
  mmsi: number;          // 30 bits
  navigationStatus: number;  // 4 bits
  rateOfTurn: number;    // 8 bits (-128 to 127)
  speedOverGround: number;   // 10 bits (0.1 knot resolution)
  positionAccuracy: boolean; // 1 bit
  longitude: number;     // 28 bits (1/10000 minute)
  latitude: number;      // 27 bits (1/10000 minute)
  courseOverGround: number;  // 12 bits (0.1 degree)
  trueHeading: number;   // 9 bits
  timeStamp: number;     // 6 bits (UTC second)
  maneuverIndicator: number; // 2 bits
  spare: number;         // 3 bits
  raim: boolean;         // 1 bit
  radioStatus: number;   // 19 bits
}
```

## 3. GMDSS Communication Protocol

### 3.1 Distress Communication

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

### 3.2 GMDSS Message Priority

```typescript
enum GMDSSPriority {
  DISTRESS = 'MAYDAY',           // Life-threatening emergency
  URGENCY = 'PAN-PAN',           // Urgent but not distress
  SAFETY = 'SECURITE',           // Safety navigation warning
  ROUTINE = 'ROUTINE'            // Normal communications
}
```

### 3.3 VHF Channel Assignments

| Channel | Frequency (MHz) | Purpose |
|---------|----------------|---------|
| 16 | 156.800 | Distress, Safety, Calling |
| 6 | 156.300 | Inter-ship safety |
| 13 | 156.650 | Navigation (bridge-to-bridge) |
| 70 | 156.525 | Digital Selective Calling (DSC) |
| 9 | 156.450 | Port operations |
| 12 | 156.600 | Port operations |
| AIS1 | 161.975 | AIS channel A |
| AIS2 | 162.025 | AIS channel B |

## 4. Emergency Response Protocol

### 4.1 Distress Signal Procedure

**Step-by-Step Protocol:**

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
    },
    {
      step: 5,
      action: 'Monitor VHF Ch 16 for response',
      duration: 'continuous'
    }
  ]
}
```

### 4.2 Search and Rescue (SAR) Coordination

**SAR Message Format:**
```json
{
  "type": "SAR-COORDINATION",
  "incident": {
    "id": "SAR-2026-0112-001",
    "distressType": "sinking",
    "position": {
      "latitude": 35.6762,
      "longitude": 139.6503,
      "accuracy": "±100m"
    },
    "timestamp": "2026-01-12T14:30:00Z",
    "pob": 15,
    "vessel": {
      "name": "OCEAN STAR",
      "mmsi": "431234567"
    }
  },
  "assets": [
    {
      "type": "helicopter",
      "callSign": "SAR-01",
      "eta": "2026-01-12T15:15:00Z"
    },
    {
      "type": "vessel",
      "mmsi": "431000001",
      "eta": "2026-01-12T16:00:00Z"
    }
  ],
  "onSceneCoordinator": "Coast Guard Cutter VIGILANT"
}
```

### 4.3 Man Overboard (MOB) Protocol

```typescript
interface MOBProtocol {
  immediate: {
    actions: [
      'Press MOB button on GPS/chartplotter',
      'Throw life ring/buoy',
      'Assign spotter to keep visual contact',
      'Sound alarm (3 long blasts)',
      'Begin Williamson Turn maneuver'
    ]
  },
  communication: {
    vhf: {
      channel: 16,
      message: 'PAN-PAN, Man Overboard at [POSITION]'
    },
    dsc: {
      channel: 70,
      category: 'urgency',
      nature: 'man_overboard'
    }
  },
  recovery: {
    maneuver: 'Williamson Turn',
    approach: 'Head to wind, person on lee side',
    recovery: 'Use MOB ladder/sling'
  }
}
```

## 5. Collision Avoidance Protocol (COLREGS)

### 5.1 Automated Collision Warning

```typescript
interface CollisionWarning {
  assessment: {
    targetVessel: {
      mmsi: string;
      position: Position;
      course: number;
      speed: number;
    },
    ownVessel: {
      position: Position;
      course: number;
      speed: number;
    },
    collision: {
      risk: 'low' | 'medium' | 'high' | 'critical';
      cpa: number;              // Closest Point of Approach (nm)
      tcpa: number;             // Time to CPA (minutes)
      bearingChange: boolean;   // True if bearing not changing
    }
  },
  action: {
    required: boolean;
    recommendation: string;
    colregsRule: number;        // Rule 13-18
    soundSignal: string;
  }
}
```

### 5.2 COLREGS Rules Implementation

| Rule | Situation | Action Required |
|------|-----------|----------------|
| 13 | Overtaking | Give way to vessel being overtaken |
| 14 | Head-on | Both vessels alter course to starboard |
| 15 | Crossing | Vessel with other on starboard gives way |
| 16 | Give-way vessel | Take early and substantial action |
| 17 | Stand-on vessel | Maintain course and speed |
| 18 | Responsibilities | Power gives way to sail, sail to fishing, etc. |

### 5.3 Sound Signals

```typescript
const SoundSignals = {
  oneShort: 'Altering course to starboard',
  twoShort: 'Altering course to port',
  threeShort: 'Operating astern propulsion',
  fiveShort: 'Danger/doubt signal',
  oneLong: 'Vessel leaving berth',
  prolongedBlast: 'Vessel in restricted visibility (every 2 min)'
};
```

## 6. Weather Routing Protocol

### 6.1 Weather Route Optimization

```typescript
interface WeatherRoutingProtocol {
  input: {
    departure: Position;
    destination: Position;
    departureTime: DateTime;
    vessel: VesselCharacteristics;
    constraints: {
      maxWaveHeight: number;
      maxWindSpeed: number;
      avoidIce: boolean;
      preferredSpeed: number;
    }
  },
  optimization: {
    objective: 'minimize_time' | 'minimize_fuel' | 'maximize_comfort';
    weatherModel: 'GFS' | 'ECMWF' | 'WW3';
    updateInterval: number;    // hours
  },
  output: {
    waypoints: Waypoint[];
    eta: DateTime;
    fuelConsumption: number;
    maxConditions: {
      waveHeight: number;
      windSpeed: number;
    };
    alternatives: Route[];
  }
}
```

### 6.2 Weather Update Protocol

**Update Frequency:**
- Routine voyage: Every 6 hours
- Poor conditions: Every 3 hours
- Severe weather: Every hour
- Storm/typhoon: Every 30 minutes

## 7. Port State Control Protocol

### 7.1 PSC Inspection Request

```json
{
  "type": "PSC-INSPECTION",
  "vessel": {
    "mmsi": "367123450",
    "name": "PACIFIC GLORY",
    "imo": "9234567",
    "flag": "US"
  },
  "port": "USOAK",
  "arrivalTime": "2026-01-15T08:00:00Z",
  "certificates": {
    "safetyManagement": {
      "valid": true,
      "expires": "2027-06-30"
    },
    "loadLine": {
      "valid": true,
      "expires": "2026-12-31"
    },
    "iopp": {
      "valid": true,
      "expires": "2027-03-15"
    }
  },
  "deficiencies": [],
  "lastInspection": "2025-10-12T00:00:00Z"
}
```

### 7.2 Deficiency Reporting

| Code | Category | Severity |
|------|----------|----------|
| 01 | Certificates & Documentation | Minor/Major |
| 02 | Structural Conditions | Major/Critical |
| 03 | Water/Weathertight | Major |
| 04 | Emergency Systems | Critical |
| 05 | Radio Communications | Major |
| 06 | Cargo Operations | Major/Critical |
| 07 | Fire Safety | Critical |
| 08 | Alarms | Major |
| 09 | Working & Living Conditions | Minor/Major |
| 10 | Safety of Navigation | Critical |

## 8. Cyber Security Protocol

### 8.1 Maritime Cyber Security Framework

```typescript
interface CyberSecurityProtocol {
  authentication: {
    method: 'certificate' | 'token' | 'biometric';
    multiFactorRequired: boolean;
    certificateType: 'X.509';
  },
  encryption: {
    inTransit: 'TLS 1.3',
    atRest: 'AES-256',
    keyManagement: 'PKI'
  },
  monitoring: {
    intrusionDetection: boolean;
    logRetention: number;      // days
    alertThreshold: 'low' | 'medium' | 'high';
  },
  incident: {
    reportingTime: number;     // minutes
    authority: 'flag_state' | 'coast_guard' | 'both';
    isolationProtocol: 'automatic' | 'manual';
  }
}
```

### 8.2 Cyber Incident Response

**Priority Levels:**
1. **Critical**: Navigation systems compromised - Isolate immediately
2. **High**: Communication systems affected - Alert authorities
3. **Medium**: Non-essential systems - Monitor and log
4. **Low**: Suspicious activity - Increase monitoring

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
