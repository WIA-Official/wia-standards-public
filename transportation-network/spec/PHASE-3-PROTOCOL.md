# WIA-UNI-008 - Phase 3: Protocol

**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

## 1. Overview

Phase 3 of the WIA-UNI-008 Transportation Network Standard defines communication protocols and technical standards for inter-Korean transportation systems integration. This includes railway signaling, traffic control, aviation coordination, maritime communications, and IoT sensor networks.

### 1.1 Protocol Objectives

- Ensure real-time interoperability between North and South Korean systems
- Maintain safety and security standards across all transportation modes
- Enable seamless cross-border operations
- Support emergency communication and coordination
- Facilitate IoT integration for smart transportation

### 1.2 Scope

This specification covers:

- Railway signaling protocols (ERTMS/ETCS)
- Intelligent Transportation Systems (ITS)
- Aviation coordination (ICAO compliance)
- Maritime communication (AIS, VHF)
- IoT sensor networks (MQTT, CoAP)
- Emergency communication protocols
- Security and encryption standards

## 2. Railway Signaling Protocols

### 2.1 ERTMS/ETCS Level 2

**European Railway Traffic Management System / European Train Control System**

#### 2.1.1 System Architecture

```
┌─────────────────┐
│ Radio Block     │
│ Centre (RBC)    │
└────────┬────────┘
         │ GSM-R
    ┌────┴────┐
    │ Eurobal │ (Balises)
    │ ise     │
    └────┬────┘
         │
┌────────┴────────┐
│ On-board Unit   │
│ (ETCS DMI)      │
└─────────────────┘
```

#### 2.1.2 Message Specifications

**Movement Authority (MA):**
```json
{
  "messageType": "MA",
  "trainId": "TKR-2025-001",
  "timestamp": "2025-12-25T09:00:00Z",
  "authority": {
    "endOfAuthority": {
      "location": "km 125.500",
      "releaseSpeed": 280
    },
    "dangerPoint": "km 125.800",
    "overlapDistance": 200,
    "sections": [
      {
        "from": "km 100.000",
        "to": "km 125.500",
        "maxSpeed": 350,
        "gradient": "+2.5%"
      }
    ]
  },
  "rbcId": "RBC-KAE-01",
  "priority": "normal"
}
```

**Position Report:**
```json
{
  "messageType": "POSITION_REPORT",
  "trainId": "TKR-2025-001",
  "timestamp": "2025-12-25T09:05:30Z",
  "position": {
    "baliseGroup": "BG-125-003",
    "distance": "km 112.345",
    "confidence": "safe"
  },
  "speed": {
    "current": 285,
    "max": 350,
    "unit": "km/h"
  },
  "mode": "FS",
  "direction": "nominal"
}
```

#### 2.1.3 GSM-R Communication

- **Frequency:** 876-880 MHz (uplink), 921-925 MHz (downlink)
- **Protocol:** GSM-R (specialized for railways)
- **Handover:** Seamless between cells
- **Priority:** Emergency > Operational > Administrative

### 2.2 Interlocking Systems

**Computer-Based Interlocking (CBI):**

```json
{
  "type": "ROUTE_REQUEST",
  "stationId": "KAE-STATION",
  "routeId": "R-101",
  "from": "SIGNAL-A12",
  "to": "SIGNAL-B05",
  "via": ["POINT-301", "POINT-302"],
  "trainId": "TKR-2025-001",
  "priority": "normal",
  "timestamp": "2025-12-25T09:00:00Z"
}
```

**Route Confirmation:**
```json
{
  "type": "ROUTE_SET",
  "routeId": "R-101",
  "status": "locked",
  "signals": {
    "SIGNAL-A12": "green",
    "SIGNAL-B05": "red"
  },
  "points": {
    "POINT-301": "normal",
    "POINT-302": "reverse"
  },
  "trackCircuits": ["TC-45", "TC-46", "TC-47"],
  "allClear": true
}
```

## 3. Intelligent Transportation Systems (ITS)

### 3.1 Highway Traffic Control

#### 3.1.1 DATEX II Protocol

```xml
<?xml version="1.0" encoding="UTF-8"?>
<d2LogicalModel xmlns="http://datex2.eu/schema/2/2_0">
  <exchange>
    <supplierIdentification>
      <country>kr</country>
      <nationalIdentifier>MOLIT-KR</nationalIdentifier>
    </supplierIdentification>
  </exchange>
  <payloadPublication>
    <situationRecord id="INCIDENT-001">
      <situationType>accident</situationType>
      <severity>medium</severity>
      <location>
        <locationForDisplay>Gyeongui Highway, km 45.2</locationForDisplay>
        <locationCoordinates>
          <latitude>37.8234</latitude>
          <longitude>126.7123</longitude>
        </locationCoordinates>
      </location>
      <impact>
        <delays>
          <delayTimeValue>15</delayTimeValue>
        </delays>
      </impact>
    </situationRecord>
  </payloadPublication>
</d2LogicalModel>
```

#### 3.1.2 Variable Message Signs (VMS)

```json
{
  "type": "VMS_MESSAGE",
  "signId": "VMS-GYE-045",
  "location": {
    "highway": "Gyeongui-01",
    "direction": "northbound",
    "km": 45.2
  },
  "message": {
    "ko": "전방 15km 사고 - 우측차로 통행",
    "en": "Accident 15km ahead - Use right lane"
  },
  "priority": "high",
  "displayDuration": 300,
  "timestamp": "2025-12-25T09:10:00Z"
}
```

### 3.2 Toll Collection (ETC)

**Electronic Toll Collection via DSRC:**

```json
{
  "type": "TOLL_TRANSACTION",
  "tollPlazaId": "TOLL-GYE-12",
  "laneId": "LANE-03",
  "vehicle": {
    "id": "OBU-123456",
    "class": "class-2",
    "plate": "12가3456"
  },
  "timestamp": "2025-12-25T09:15:00Z",
  "amount": {
    "value": 5000,
    "currency": "KRW"
  },
  "paymentMethod": "DSRC",
  "status": "success"
}
```

## 4. Aviation Protocols

### 4.1 ICAO Standards Compliance

#### 4.1.1 ADS-B (Automatic Dependent Surveillance-Broadcast)

```json
{
  "messageType": "ADS-B",
  "aircraftId": "HL7788",
  "flightNumber": "KE2025",
  "timestamp": "2025-12-25T09:20:00Z",
  "position": {
    "latitude": 37.9704,
    "longitude": 126.5550,
    "altitude": 35000,
    "altitudeType": "barometric"
  },
  "velocity": {
    "groundSpeed": 450,
    "heading": 358,
    "verticalRate": 0
  },
  "surveillance": {
    "nicCategory": 7,
    "nacVelocity": 1,
    "sil": 3
  }
}
```

#### 4.1.2 AIDC (ATS Interfacility Data Communication)

```json
{
  "messageType": "AIDC",
  "messageId": "AIDC-2025-001",
  "from": "RKSS",
  "to": "ZKPY",
  "timestamp": "2025-12-25T09:00:00Z",
  "flightData": {
    "callsign": "KE2025",
    "aircraftType": "B77W",
    "departure": "RKSI",
    "destination": "ZKPY",
    "route": "RKSI DCT BIKMA DCT ZKPY",
    "altitude": "FL350",
    "estimatedTime": "2025-12-25T09:45:00Z"
  },
  "coordinationMessage": "EST_HANDOFF"
}
```

### 4.2 Air Traffic Control Coordination

```json
{
  "type": "ATC_COORDINATION",
  "sector": {
    "from": "INCHEON-ACC",
    "to": "PYONGYANG-ACC"
  },
  "flights": [
    {
      "callsign": "KE2025",
      "handoffPoint": "BIKMA",
      "estimatedTime": "2025-12-25T09:45:00Z",
      "altitude": 35000,
      "speed": 450
    }
  ],
  "frequency": "132.450",
  "timestamp": "2025-12-25T09:30:00Z"
}
```

## 5. Maritime Protocols

### 5.1 AIS (Automatic Identification System)

**AIS Message Type 1 (Position Report):**

```json
{
  "messageType": 1,
  "mmsi": "440123456",
  "vesselName": "KOREA EXPRESS",
  "timestamp": "2025-12-25T09:00:00Z",
  "position": {
    "latitude": 37.4563,
    "longitude": 126.4235,
    "accuracy": "high"
  },
  "navigationStatus": "under way using engine",
  "rateOfTurn": 0,
  "speedOverGround": 18.5,
  "courseOverGround": 270,
  "trueHeading": 270,
  "positionTimestamp": "2025-12-25T09:00:00Z"
}
```

**AIS Message Type 5 (Static Voyage Data):**

```json
{
  "messageType": 5,
  "mmsi": "440123456",
  "imoNumber": "IMO9876543",
  "callsign": "HLKO",
  "vesselName": "KOREA EXPRESS",
  "shipType": "cargo",
  "dimensions": {
    "length": 180,
    "beam": 28,
    "draft": 9.5
  },
  "destination": "NAMPO PORT",
  "eta": "2025-12-25T15:00:00Z"
}
```

### 5.2 VTS (Vessel Traffic Service)

```json
{
  "type": "VTS_REPORT",
  "vtsArea": "INCHEON-VTS",
  "timestamp": "2025-12-25T09:00:00Z",
  "vessels": [
    {
      "mmsi": "440123456",
      "name": "KOREA EXPRESS",
      "position": { "lat": 37.4563, "lon": 126.4235 },
      "speed": 18.5,
      "course": 270,
      "status": "normal",
      "clearance": "granted",
      "channel": "VTS-CHANNEL-12"
    }
  ],
  "conditions": {
    "visibility": "good",
    "seaState": 2,
    "windSpeed": 15,
    "windDirection": 290
  }
}
```

## 6. IoT Sensor Networks

### 6.1 MQTT Protocol

**Topic Structure:**
```
wia/uni-008/{region}/{transportMode}/{vehicleId}/{dataType}

Examples:
- wia/uni-008/seoul/rail/TKR-2025-001/location
- wia/uni-008/pyongyang/highway/BUS-123/speed
- wia/uni-008/kaesong/cargo/CONT-456/temperature
```

**MQTT Message Payload:**

```json
{
  "deviceId": "SENSOR-TKR-001",
  "timestamp": "2025-12-25T09:00:00Z",
  "vehicleId": "TKR-2025-001",
  "measurements": {
    "location": {
      "latitude": 37.8234,
      "longitude": 126.7123,
      "altitude": 45.2
    },
    "speed": 285,
    "acceleration": 0.5,
    "vibration": {
      "x": 0.02,
      "y": 0.03,
      "z": 0.01
    },
    "temperature": {
      "engine": 85.5,
      "cabin": 22.0
    }
  },
  "quality": "good"
}
```

### 6.2 CoAP Protocol

**CoAP Request:**
```
coap://sensor.transportation.wia/vehicle/TKR-2025-001/status

Method: GET
Content-Format: application/json
```

**CoAP Response:**
```json
{
  "vehicleId": "TKR-2025-001",
  "status": "operational",
  "health": {
    "overall": "good",
    "systems": {
      "brakes": "optimal",
      "doors": "optimal",
      "hvac": "optimal",
      "communications": "optimal"
    }
  },
  "maintenance": {
    "nextScheduled": "2026-01-15",
    "lastCompleted": "2025-11-20"
  }
}
```

## 7. Emergency Communication

### 7.1 Emergency Alert Format

```json
{
  "type": "EMERGENCY_ALERT",
  "severity": "critical",
  "category": "accident|fire|security|medical|natural-disaster",
  "timestamp": "2025-12-25T09:00:00Z",
  "location": {
    "transportMode": "rail",
    "vehicleId": "TKR-2025-001",
    "position": {
      "latitude": 37.8234,
      "longitude": 126.7123
    },
    "description": "Between Kaesong and Pyongyang, km 112"
  },
  "incident": {
    "type": "medical-emergency",
    "description": "Passenger requires immediate medical attention",
    "casualties": {
      "injured": 1,
      "critical": 0,
      "deceased": 0
    }
  },
  "response": {
    "required": ["medical", "police"],
    "nearestFacility": "Kaesong Hospital",
    "estimatedTime": 15
  },
  "contact": {
    "name": "Train Conductor Kim",
    "phone": "+850-XX-XXX-XXXX"
  }
}
```

### 7.2 Emergency Broadcast

**Cross-border Emergency Communication:**

```json
{
  "type": "EMERGENCY_BROADCAST",
  "priority": "urgent",
  "recipients": ["all-stations", "all-vehicles", "emergency-services"],
  "regions": ["seoul", "kaesong", "pyongyang"],
  "message": {
    "ko": "긴급: TKR-2025-001 열차 의료 응급 상황",
    "en": "URGENT: Medical emergency on train TKR-2025-001"
  },
  "actionRequired": "Prepare medical team at Kaesong Station",
  "validUntil": "2025-12-25T10:00:00Z"
}
```

## 8. Security & Encryption

### 8.1 TLS 1.3

All API communications must use TLS 1.3 with the following cipher suites:

- `TLS_AES_256_GCM_SHA384`
- `TLS_CHACHA20_POLY1305_SHA256`
- `TLS_AES_128_GCM_SHA256`

### 8.2 Message Authentication

**HMAC-SHA256 Signature:**

```javascript
const message = JSON.stringify(payload);
const signature = crypto
  .createHmac('sha256', SECRET_KEY)
  .update(message)
  .digest('hex');

headers['X-WIA-Signature'] = `sha256=${signature}`;
```

### 8.3 End-to-End Encryption

For sensitive data (passenger info, cargo details):

```json
{
  "encryptedPayload": "base64-encoded-encrypted-data",
  "encryption": {
    "algorithm": "AES-256-GCM",
    "keyId": "key-2025-001",
    "iv": "base64-encoded-iv",
    "authTag": "base64-encoded-auth-tag"
  }
}
```

## 9. Data Exchange Protocols

### 9.1 Real-time Data Streaming

**Server-Sent Events (SSE):**

```http
GET /v1/tracking/stream HTTP/1.1
Accept: text/event-stream

HTTP/1.1 200 OK
Content-Type: text/event-stream

event: location_update
data: {"vehicleId":"TKR-2025-001","lat":37.8234,"lon":126.7123}

event: speed_change
data: {"vehicleId":"TKR-2025-001","speed":285,"timestamp":"2025-12-25T09:00:00Z"}
```

### 9.2 Bulk Data Synchronization

**FTP/SFTP for Historical Data:**

```
sftp://data.transportation.wia/exports/
├── routes/
│   └── 2025-12-25_routes.json
├── bookings/
│   └── 2025-12-25_bookings.json
└── tracking/
    └── 2025-12-25_tracking.jsonl
```

## 10. Protocol Compliance

### 10.1 Testing & Certification

All implementations must pass compliance tests for:

1. **ERTMS/ETCS:** ERA certification required
2. **ITS:** ISO 14813 compliance
3. **ICAO:** Annex 10 compliance
4. **IMO:** SOLAS requirements
5. **IoT:** MQTT 5.0 / CoAP RFC 7252 compliance

### 10.2 Interoperability Testing

Annual interoperability tests conducted at:
- Seoul Testing Facility
- Kaesong Joint Operations Center
- Pyongyang Control Center

## 11. References

- **ERTMS/ETCS:** UNISIG SUBSET-026
- **DATEX II:** CEN/TS 16157
- **ICAO Annex 10:** Aeronautical Telecommunications
- **AIS:** ITU-R M.1371
- **MQTT:** OASIS MQTT 5.0
- **CoAP:** RFC 7252
- **TLS 1.3:** RFC 8446

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
