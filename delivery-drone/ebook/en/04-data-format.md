# Chapter 4: Data Formats and Message Specifications

## Phase 1: Interoperable Data Standards for Drone Delivery Systems

---

## 4.1 Data Format Design Principles

### Core Requirements

The WIA-AUTO-017 data formats are designed with the following requirements:

1. **Machine Parseable**: Structured formats that can be efficiently processed
2. **Human Readable**: Easy to debug and understand during development
3. **Version Controlled**: Support for schema evolution without breaking changes
4. **Compact**: Efficient for transmission over limited bandwidth
5. **Self-Describing**: Metadata enables interpretation without external documentation

### Format Selection

| Use Case | Primary Format | Alternate Format |
|----------|----------------|------------------|
| API communication | JSON | Protocol Buffers |
| Telemetry streaming | MessagePack | Binary |
| Flight logs | JSON Lines | Parquet |
| Configuration | YAML | JSON |
| Geospatial data | GeoJSON | Shapefile |

### Schema Validation

All messages must validate against JSON Schema definitions:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/delivery-drone/base-message.json",
  "type": "object",
  "required": ["wiaVersion", "messageType", "timestamp", "sourceId"],
  "properties": {
    "wiaVersion": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+$"
    },
    "messageType": {
      "type": "string",
      "enum": ["TELEMETRY", "WAYPOINT", "COMMAND", "STATUS", "DELIVERY", "FLIGHT_LOG"]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "sourceId": {
      "type": "string",
      "pattern": "^WIA-DRN-[A-Z0-9]+-\\d{4}$"
    }
  }
}
```

---

## 4.2 Base Message Format

### Message Envelope

Every WIA message shares a common envelope structure:

```json
{
  "wiaVersion": "1.0",
  "messageType": "TELEMETRY",
  "messageId": "msg-20250101-abcd1234",
  "timestamp": "2025-01-01T10:00:00.000Z",
  "sourceId": "WIA-DRN-X1-0042",
  "sequenceNumber": 12345,
  "priority": "NORMAL",
  "payload": {
    // Message-specific content
  },
  "checksum": "SHA256:a1b2c3d4..."
}
```

### Field Specifications

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| wiaVersion | string | Yes | Standard version (e.g., "1.0") |
| messageType | enum | Yes | Type of message content |
| messageId | string | Yes | Unique message identifier |
| timestamp | ISO8601 | Yes | UTC timestamp with milliseconds |
| sourceId | string | Yes | Drone identifier |
| sequenceNumber | integer | No | Monotonic sequence for ordering |
| priority | enum | No | LOW, NORMAL, HIGH, CRITICAL |
| payload | object | Yes | Message-specific content |
| checksum | string | No | SHA256 hash of payload |

### Message Types

```typescript
enum MessageType {
  // Real-time data
  TELEMETRY = "TELEMETRY",
  POSITION = "POSITION",
  STATUS = "STATUS",

  // Mission data
  WAYPOINT = "WAYPOINT",
  FLIGHT_PLAN = "FLIGHT_PLAN",
  MISSION = "MISSION",

  // Delivery data
  PACKAGE = "PACKAGE",
  DELIVERY = "DELIVERY",

  // Control
  COMMAND = "COMMAND",
  RESPONSE = "RESPONSE",

  // Logging
  FLIGHT_LOG = "FLIGHT_LOG",
  EVENT = "EVENT"
}
```

---

## 4.3 Telemetry Format

### Real-Time Position Telemetry

Transmitted at 1-10 Hz during flight:

```json
{
  "wiaVersion": "1.0",
  "messageType": "TELEMETRY",
  "timestamp": "2025-01-01T10:05:30.123Z",
  "sourceId": "WIA-DRN-X1-0042",
  "sequenceNumber": 12345,
  "payload": {
    "position": {
      "latitude": 37.7749,
      "longitude": -122.4194,
      "altitudeMSL": 125.5,
      "altitudeAGL": 35.2,
      "accuracy": {
        "horizontal": 1.5,
        "vertical": 2.0,
        "source": "RTK_FIXED"
      }
    },
    "velocity": {
      "north": 8.5,
      "east": 12.3,
      "down": -0.5,
      "groundSpeed": 15.0,
      "airSpeed": 16.2
    },
    "attitude": {
      "roll": 2.3,
      "pitch": 5.1,
      "yaw": 45.0
    },
    "heading": {
      "magnetic": 48.5,
      "true": 45.0
    },
    "satellites": 14,
    "hdop": 0.8,
    "vdop": 1.2
  }
}
```

### System Status Telemetry

Transmitted at 0.2-1 Hz during flight:

```json
{
  "wiaVersion": "1.0",
  "messageType": "STATUS",
  "timestamp": "2025-01-01T10:05:30.000Z",
  "sourceId": "WIA-DRN-X1-0042",
  "payload": {
    "battery": {
      "voltage": 22.1,
      "current": 25.3,
      "capacity": 10000,
      "remaining": 7800,
      "percentage": 78,
      "cellVoltages": [3.68, 3.69, 3.68, 3.70, 3.69, 3.68],
      "temperature": 32.5,
      "estimatedFlightTime": 1200
    },
    "motors": [
      {"id": 1, "rpm": 5200, "current": 4.2, "temperature": 45.0, "status": "OK"},
      {"id": 2, "rpm": 5180, "current": 4.1, "temperature": 44.5, "status": "OK"},
      {"id": 3, "rpm": 5220, "current": 4.3, "temperature": 46.0, "status": "OK"},
      {"id": 4, "rpm": 5190, "current": 4.2, "temperature": 45.5, "status": "OK"},
      {"id": 5, "rpm": 5210, "current": 4.1, "temperature": 45.0, "status": "OK"},
      {"id": 6, "rpm": 5200, "current": 4.2, "temperature": 45.2, "status": "OK"}
    ],
    "flightMode": "WAYPOINT",
    "armed": true,
    "airborne": true,
    "gpsStatus": "RTK_FIXED",
    "linkQuality": {
      "primary": 95,
      "backup": 88,
      "latency": 45
    },
    "sensors": {
      "imu": "OK",
      "gps": "OK",
      "barometer": "OK",
      "magnetometer": "OK",
      "lidar": "OK",
      "camera": "OK"
    },
    "warnings": [],
    "errors": []
  }
}
```

### Compact Telemetry Format

For bandwidth-limited scenarios, a compact binary format is available:

```python
import struct

def encode_compact_telemetry(telemetry: dict) -> bytes:
    """
    Encode telemetry into compact binary format.

    Format (48 bytes):
    - timestamp: 8 bytes (uint64, milliseconds since epoch)
    - latitude: 4 bytes (int32, microdegrees)
    - longitude: 4 bytes (int32, microdegrees)
    - altitude_msl: 2 bytes (int16, decimeters)
    - altitude_agl: 2 bytes (int16, decimeters)
    - ground_speed: 2 bytes (uint16, cm/s)
    - heading: 2 bytes (uint16, centidegrees)
    - roll: 2 bytes (int16, centidegrees)
    - pitch: 2 bytes (int16, centidegrees)
    - battery_percent: 1 byte (uint8)
    - satellites: 1 byte (uint8)
    - status_flags: 2 bytes (uint16)
    - motor_status: 8 bytes (8x uint8, status per motor)
    - reserved: 8 bytes
    """
    pos = telemetry['position']
    vel = telemetry['velocity']
    att = telemetry['attitude']
    bat = telemetry['battery']

    return struct.pack(
        '!QiihhHHhhBBH8s8s',
        int(telemetry['timestamp'] * 1000),  # ms since epoch
        int(pos['latitude'] * 1e6),
        int(pos['longitude'] * 1e6),
        int(pos['altitudeMSL'] * 10),
        int(pos['altitudeAGL'] * 10),
        int(vel['groundSpeed'] * 100),
        int(att['yaw'] * 100),
        int(att['roll'] * 100),
        int(att['pitch'] * 100),
        int(bat['percentage']),
        telemetry['satellites'],
        telemetry.get('status_flags', 0),
        b'\x00' * 8,  # motor status
        b'\x00' * 8   # reserved
    )

def decode_compact_telemetry(data: bytes) -> dict:
    """Decode compact telemetry from binary format."""
    values = struct.unpack('!QiihhHHhhBBH8s8s', data)

    return {
        'timestamp': values[0] / 1000,
        'position': {
            'latitude': values[1] / 1e6,
            'longitude': values[2] / 1e6,
            'altitudeMSL': values[3] / 10,
            'altitudeAGL': values[4] / 10
        },
        'velocity': {
            'groundSpeed': values[5] / 100
        },
        'attitude': {
            'yaw': values[6] / 100,
            'roll': values[7] / 100,
            'pitch': values[8] / 100
        },
        'battery': {
            'percentage': values[9]
        },
        'satellites': values[10],
        'status_flags': values[11]
    }
```

---

## 4.4 Waypoint and Flight Plan Format

### Single Waypoint

```json
{
  "waypointId": "WP-001",
  "position": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitudeMSL": 120,
    "altitudeAGL": 30
  },
  "speed": 15.0,
  "heading": 45,
  "headingMode": "AUTO",
  "arrivalRadius": 5.0,
  "holdTime": 0,
  "actions": [
    {
      "type": "TAKE_PHOTO",
      "params": {}
    }
  ],
  "constraints": {
    "minAltitude": 20,
    "maxAltitude": 120,
    "maxSpeed": 20
  }
}
```

### Waypoint Action Types

| Action | Description | Parameters |
|--------|-------------|------------|
| TAKE_PHOTO | Capture image | resolution, format |
| START_VIDEO | Begin recording | resolution, fps |
| STOP_VIDEO | End recording | - |
| HOVER | Hold position | duration |
| CHANGE_SPEED | Adjust velocity | speed |
| CHANGE_ALTITUDE | Adjust height | altitude, rate |
| RELEASE_PAYLOAD | Drop/lower package | method |
| WAIT_FOR_TRIGGER | Pause for condition | condition |

### Complete Flight Plan

```json
{
  "wiaVersion": "1.0",
  "messageType": "FLIGHT_PLAN",
  "messageId": "fp-20250101-1234",
  "timestamp": "2025-01-01T09:00:00.000Z",
  "sourceId": "WIA-GCS-001",
  "payload": {
    "flightPlanId": "FP-20250101-1234",
    "name": "Delivery Mission Alpha",
    "description": "Package delivery to 456 Mission St",
    "drone": "WIA-DRN-X1-0042",
    "operator": "WIA-OP-001",
    "departure": {
      "location": {
        "latitude": 37.7749,
        "longitude": -122.4194,
        "altitude": 10
      },
      "address": "123 Market St, San Francisco, CA",
      "scheduledTime": "2025-01-01T10:00:00Z"
    },
    "arrival": {
      "location": {
        "latitude": 37.7849,
        "longitude": -122.4094,
        "altitude": 0
      },
      "address": "456 Mission St, San Francisco, CA",
      "scheduledTime": "2025-01-01T10:15:00Z"
    },
    "waypoints": [
      {
        "waypointId": "WP-001",
        "type": "TAKEOFF",
        "position": {"latitude": 37.7749, "longitude": -122.4194, "altitudeMSL": 50},
        "speed": 5.0
      },
      {
        "waypointId": "WP-002",
        "type": "CRUISE",
        "position": {"latitude": 37.7799, "longitude": -122.4144, "altitudeMSL": 120},
        "speed": 20.0
      },
      {
        "waypointId": "WP-003",
        "type": "CRUISE",
        "position": {"latitude": 37.7839, "longitude": -122.4104, "altitudeMSL": 120},
        "speed": 20.0
      },
      {
        "waypointId": "WP-004",
        "type": "DESCENT",
        "position": {"latitude": 37.7849, "longitude": -122.4094, "altitudeMSL": 30},
        "speed": 10.0
      },
      {
        "waypointId": "WP-005",
        "type": "DELIVERY",
        "position": {"latitude": 37.7849, "longitude": -122.4094, "altitudeMSL": 5},
        "speed": 2.0,
        "actions": [{"type": "RELEASE_PAYLOAD", "params": {"method": "WINCH"}}]
      }
    ],
    "constraints": {
      "maxAltitude": 120,
      "maxSpeed": 22,
      "geofence": {
        "type": "POLYGON",
        "coordinates": [[37.77, -122.43], [37.79, -122.43], [37.79, -122.40], [37.77, -122.40]]
      }
    },
    "contingency": {
      "lostLink": "RETURN_TO_HOME",
      "lowBattery": "RETURN_TO_HOME",
      "geofenceViolation": "RETURN_TO_HOME",
      "alternativeLandingSites": [
        {"latitude": 37.7780, "longitude": -122.4150, "priority": 1}
      ]
    }
  }
}
```

---

## 4.5 Package and Delivery Format

### Package Specification

```json
{
  "packageId": "PKG-20250101-1234",
  "trackingCode": "1Z999AA10123456784",
  "weight": {
    "value": 2.5,
    "unit": "kg"
  },
  "dimensions": {
    "length": 30,
    "width": 20,
    "height": 15,
    "unit": "cm"
  },
  "volumetricWeight": {
    "value": 1.8,
    "unit": "kg"
  },
  "classification": {
    "fragile": false,
    "hazardous": false,
    "hazardClass": null,
    "temperatureSensitive": false,
    "temperatureRange": null,
    "perishable": false,
    "expiryDate": null
  },
  "value": {
    "amount": 150.00,
    "currency": "USD"
  },
  "insurance": {
    "covered": true,
    "provider": "DroneInsure Co",
    "policyNumber": "DI-2025-001234"
  },
  "sender": {
    "name": "ABC Electronics",
    "address": "123 Market St, San Francisco, CA 94102",
    "phone": "+1-555-0100"
  },
  "recipient": {
    "name": "Jane Smith",
    "address": "456 Mission St, San Francisco, CA 94105",
    "phone": "+1-555-0200",
    "instructions": "Leave at front door"
  },
  "deliveryPreferences": {
    "leaveAtDoor": true,
    "signatureRequired": false,
    "ageVerification": false,
    "photoConfirmation": true
  }
}
```

### Delivery Event Format

```json
{
  "wiaVersion": "1.0",
  "messageType": "DELIVERY",
  "timestamp": "2025-01-01T10:14:30.000Z",
  "sourceId": "WIA-DRN-X1-0042",
  "payload": {
    "deliveryId": "DEL-20250101-1234",
    "missionId": "MSN-20250101-1234",
    "packageId": "PKG-20250101-1234",
    "event": "DELIVERED",
    "eventTimestamp": "2025-01-01T10:14:30.000Z",
    "location": {
      "latitude": 37.7849,
      "longitude": -122.4094,
      "altitudeAGL": 0.5,
      "accuracy": 0.3
    },
    "deliveryMethod": "WINCH",
    "details": {
      "winchDeployed": true,
      "winchLoweredDistance": 3.5,
      "packageReleased": true,
      "groundContactDetected": true,
      "winchRetracted": true
    },
    "confirmation": {
      "photoTaken": true,
      "photoUrl": "https://storage.wia.com/delivery-photos/DEL-20250101-1234.jpg",
      "signatureCollected": false,
      "recipientPresent": false
    },
    "timing": {
      "missionStarted": "2025-01-01T10:00:00.000Z",
      "packagePickedUp": "2025-01-01T10:02:00.000Z",
      "deliveryStarted": "2025-01-01T10:12:00.000Z",
      "deliveryCompleted": "2025-01-01T10:14:30.000Z"
    }
  }
}
```

### Delivery Status Enumeration

```typescript
enum DeliveryStatus {
  // Pre-flight
  PENDING = "PENDING",
  SCHEDULED = "SCHEDULED",
  PACKAGE_RECEIVED = "PACKAGE_RECEIVED",
  PACKAGE_LOADED = "PACKAGE_LOADED",

  // In-flight
  DEPARTED = "DEPARTED",
  EN_ROUTE = "EN_ROUTE",
  APPROACHING = "APPROACHING",
  HOVERING = "HOVERING",
  DELIVERING = "DELIVERING",

  // Completed
  DELIVERED = "DELIVERED",
  CONFIRMED = "CONFIRMED",

  // Issues
  ATTEMPTED = "ATTEMPTED",
  RETURNED = "RETURNED",
  CANCELLED = "CANCELLED",
  FAILED = "FAILED"
}
```

---

## 4.6 Flight Log Format

### Complete Flight Log

```json
{
  "wiaVersion": "1.0",
  "messageType": "FLIGHT_LOG",
  "timestamp": "2025-01-01T10:20:00.000Z",
  "sourceId": "WIA-DRN-X1-0042",
  "payload": {
    "flightId": "FLT-20250101-1234",
    "missionId": "MSN-20250101-1234",
    "droneId": "WIA-DRN-X1-0042",
    "operatorId": "WIA-OP-001",
    "pilotId": "PILOT-001",
    "timing": {
      "scheduled": "2025-01-01T10:00:00.000Z",
      "armed": "2025-01-01T10:00:15.000Z",
      "takeoff": "2025-01-01T10:00:30.000Z",
      "landing": "2025-01-01T10:14:45.000Z",
      "disarmed": "2025-01-01T10:15:00.000Z"
    },
    "duration": {
      "total": 900,
      "airborne": 855,
      "hover": 120,
      "cruise": 735
    },
    "distance": {
      "total": 5200,
      "horizontal": 5150,
      "unit": "meters"
    },
    "altitude": {
      "max": 125,
      "min": 0,
      "average": 85,
      "unit": "meters"
    },
    "speed": {
      "max": 22.5,
      "average": 15.3,
      "unit": "m/s"
    },
    "battery": {
      "start": 100,
      "end": 56,
      "consumed": 44,
      "energyUsed": 98.5,
      "unit": "Wh"
    },
    "waypoints": {
      "total": 5,
      "completed": 5,
      "skipped": 0
    },
    "delivery": {
      "packageId": "PKG-20250101-1234",
      "success": true,
      "method": "WINCH"
    },
    "weather": {
      "temperature": 18.5,
      "windSpeed": 5.2,
      "windDirection": 270,
      "visibility": 10000,
      "precipitation": false
    },
    "incidents": [],
    "telemetryFile": {
      "format": "BINARY",
      "url": "https://storage.wia.com/telemetry/FLT-20250101-1234.bin",
      "size": 2457600,
      "records": 8550
    }
  }
}
```

### Flight Log Query Parameters

```
GET /api/v1/flight-logs?
  droneId=WIA-DRN-X1-0042&
  startDate=2025-01-01&
  endDate=2025-01-31&
  status=COMPLETED&
  deliverySuccess=true&
  minDistance=1000&
  maxDistance=10000&
  limit=100&
  offset=0
```

---

## 4.7 Command and Response Format

### Command Message

```json
{
  "wiaVersion": "1.0",
  "messageType": "COMMAND",
  "messageId": "cmd-20250101-abcd",
  "timestamp": "2025-01-01T10:05:00.000Z",
  "sourceId": "WIA-GCS-001",
  "priority": "HIGH",
  "payload": {
    "commandId": "CMD-20250101-1234",
    "targetId": "WIA-DRN-X1-0042",
    "commandType": "RETURN_TO_HOME",
    "parameters": {},
    "timeout": 30000,
    "requiresAck": true
  }
}
```

### Command Types

| Command | Description | Parameters |
|---------|-------------|------------|
| ARM | Arm motors | - |
| DISARM | Disarm motors | - |
| TAKEOFF | Begin flight | altitude |
| LAND | Land immediately | - |
| RETURN_TO_HOME | RTH procedure | - |
| GOTO_WAYPOINT | Navigate to waypoint | waypointId |
| PAUSE | Hover in place | - |
| RESUME | Continue mission | - |
| ABORT | Emergency stop | - |
| SET_SPEED | Change speed | speed |
| SET_ALTITUDE | Change altitude | altitude |
| RELEASE_PAYLOAD | Drop package | method |
| TAKE_PHOTO | Capture image | - |
| START_VIDEO | Begin recording | - |
| STOP_VIDEO | End recording | - |

### Response Message

```json
{
  "wiaVersion": "1.0",
  "messageType": "RESPONSE",
  "messageId": "rsp-20250101-efgh",
  "timestamp": "2025-01-01T10:05:00.250Z",
  "sourceId": "WIA-DRN-X1-0042",
  "payload": {
    "commandId": "CMD-20250101-1234",
    "status": "ACCEPTED",
    "message": "Return to home initiated",
    "estimatedCompletion": "2025-01-01T10:12:00.000Z",
    "details": {
      "currentPosition": {"latitude": 37.7800, "longitude": -122.4150},
      "homePosition": {"latitude": 37.7749, "longitude": -122.4194},
      "estimatedDistance": 650,
      "estimatedTime": 420
    }
  }
}
```

### Command Response Status

| Status | Description |
|--------|-------------|
| ACCEPTED | Command received and queued |
| EXECUTING | Command in progress |
| COMPLETED | Command finished successfully |
| REJECTED | Command denied (see reason) |
| FAILED | Command execution failed |
| TIMEOUT | Command timed out |

---

## 4.8 Event and Alert Format

### Event Message

```json
{
  "wiaVersion": "1.0",
  "messageType": "EVENT",
  "timestamp": "2025-01-01T10:06:30.000Z",
  "sourceId": "WIA-DRN-X1-0042",
  "payload": {
    "eventId": "EVT-20250101-5678",
    "eventType": "WARNING",
    "category": "BATTERY",
    "code": "BAT_LOW_WARNING",
    "severity": "MEDIUM",
    "message": "Battery level below 30%",
    "details": {
      "currentLevel": 28,
      "threshold": 30,
      "estimatedFlightTime": 480
    },
    "recommendedAction": "Consider initiating return to home",
    "autoResponse": {
      "triggered": false,
      "action": null
    },
    "location": {
      "latitude": 37.7820,
      "longitude": -122.4120,
      "altitude": 100
    }
  }
}
```

### Event Categories and Codes

| Category | Code | Severity | Description |
|----------|------|----------|-------------|
| BATTERY | BAT_LOW_WARNING | MEDIUM | Battery below 30% |
| BATTERY | BAT_CRITICAL | CRITICAL | Battery below 15% |
| GPS | GPS_DEGRADED | MEDIUM | Reduced GPS accuracy |
| GPS | GPS_LOST | HIGH | No GPS signal |
| MOTOR | MOTOR_ANOMALY | MEDIUM | Motor performance issue |
| MOTOR | MOTOR_FAILED | CRITICAL | Motor failure detected |
| COMM | LINK_DEGRADED | MEDIUM | Communication quality poor |
| COMM | LINK_LOST | HIGH | Communication lost |
| GEOFENCE | GEOFENCE_WARNING | HIGH | Approaching boundary |
| GEOFENCE | GEOFENCE_VIOLATION | CRITICAL | Boundary crossed |
| OBSTACLE | OBSTACLE_DETECTED | MEDIUM | Obstacle in path |
| WEATHER | WIND_WARNING | MEDIUM | High wind detected |

---

## Chapter Summary

The WIA-AUTO-017 data format specification defines comprehensive, interoperable message formats for all aspects of drone delivery operations. From real-time telemetry to flight plans, package specifications to delivery events, the formats ensure consistent communication across all system components.

The base message envelope provides versioning, identification, and integrity verification. Telemetry formats support both high-fidelity JSON and bandwidth-efficient binary encoding. Flight plans specify complete mission parameters including waypoints, actions, and contingency procedures.

Package and delivery formats capture the full lifecycle of a delivery, from package specification to confirmation. Flight logs provide comprehensive records for analysis and compliance. Command and response formats enable reliable remote control, while events and alerts ensure timely notification of operational conditions.

---

## Key Takeaways

1. **All messages share a common envelope** with version, type, timestamp, and source
2. **Telemetry supports both JSON and compact binary** for different bandwidth needs
3. **Flight plans include complete waypoints, constraints, and contingency procedures**
4. **Delivery formats track the full package lifecycle** from sender to recipient
5. **Events and alerts use severity levels** for appropriate operator response

---

## Review Questions

1. What fields are required in every WIA base message?
2. Design a compact telemetry format that uses only 32 bytes.
3. What information must a flight plan include for regulatory compliance?
4. How would you extend the delivery format to support return shipments?
5. Create an event format for detecting and reporting airspace conflicts.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
