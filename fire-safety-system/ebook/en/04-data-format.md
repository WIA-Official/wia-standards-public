# Chapter 4: Data Format Specifications (Phase 1)

## Overview

Phase 1 of the WIA Standard establishes the foundational data format specifications that enable all fire safety system components to represent and exchange information consistently. This chapter provides comprehensive technical details of the JSON schemas, data validation requirements, and implementation guidelines.

---

## Why Data Format Standardization Matters

### The Foundation of Interoperability

Without standardized data formats, systems cannot communicate effectively:

```
Problem: Incompatible Data Representations

Vendor A Sensor Reading:
{"dev":12345,"typ":1,"val":3.2,"ts":1640995200}

Vendor B Sensor Reading:
{"deviceIdentifier":"SD-0012345","sensorType":"SMOKE_OPTICAL",
 "measurementValue":{"value":3.2,"unit":"OD_PER_METER"},
 "timestamp":"2021-12-31T12:00:00Z"}

Result: Integration requires custom parsing for each vendor
Cost: $15,000-$50,000 per vendor integration

Solution: WIA Standard Data Format

All Vendors Use:
{"sensorId":"550e8400-e29b-41d4-a716-446655440000",
 "type":"smoke","status":"normal",
 "readings":{"value":3.2,"unit":"OD_PER_METER",
             "timestamp":"2021-12-31T12:00:00Z"},
 ...}

Result: Universal compatibility, zero integration cost
```

### Benefits of JSON Format

**Human Readable:**
```json
// Easy to read and understand
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "status": "alarm"
}
```

**Machine Parsable:**
- Native support in all modern programming languages
- Efficient parsing libraries available
- Automatic serialization/deserialization

**Validated:**
- JSON Schema enables automatic validation
- Type checking and constraints
- Documentation generation

**Extensible:**
- New fields can be added without breaking compatibility
- Optional fields support evolution
- Version-agnostic design

---

## Core Data Schemas

### 1. Sensor Data Schema

#### Complete Schema Definition

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA Fire Safety Sensor Data",
  "type": "object",
  "required": ["sensorId", "type", "location", "status", "readings", "metadata"],
  "properties": {
    "sensorId": {
      "type": "string",
      "format": "uuid",
      "description": "Globally unique sensor identifier (UUID v4)"
    },
    "type": {
      "type": "string",
      "enum": ["smoke", "heat", "flame", "co", "multi"],
      "description": "Sensor type classification"
    },
    "location": {
      "type": "object",
      "required": ["building", "floor", "zone"],
      "properties": {
        "building": {
          "type": "string",
          "description": "Building identifier or name"
        },
        "floor": {
          "type": "integer",
          "description": "Floor number (0=ground, negative=basement)"
        },
        "zone": {
          "type": "string",
          "description": "Zone identifier within floor"
        },
        "coordinates": {
          "type": "object",
          "description": "Physical coordinates (optional)",
          "properties": {
            "x": {"type": "number", "description": "X coordinate in meters"},
            "y": {"type": "number", "description": "Y coordinate in meters"},
            "z": {"type": "number", "description": "Z coordinate in meters (optional)"}
          }
        }
      }
    },
    "status": {
      "type": "string",
      "enum": ["normal", "alarm", "trouble", "disabled"],
      "description": "Current sensor status"
    },
    "readings": {
      "type": "object",
      "required": ["value", "unit", "timestamp"],
      "properties": {
        "value": {
          "type": "number",
          "description": "Sensor reading value"
        },
        "unit": {
          "type": "string",
          "description": "Unit of measurement"
        },
        "timestamp": {
          "type": "string",
          "format": "date-time",
          "description": "ISO 8601 timestamp with UTC timezone"
        }
      }
    },
    "metadata": {
      "type": "object",
      "required": ["manufacturer", "model", "firmwareVersion"],
      "properties": {
        "manufacturer": {"type": "string"},
        "model": {"type": "string"},
        "firmwareVersion": {"type": "string"},
        "installationDate": {"type": "string", "format": "date-time"},
        "lastMaintenance": {"type": "string", "format": "date-time"}
      }
    }
  }
}
```

#### Example Sensor Data Instances

**Smoke Detector - Normal Status:**
```json
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "location": {
    "building": "Main Tower",
    "floor": 12,
    "zone": "East Wing E12-A",
    "coordinates": {
      "x": 45.2,
      "y": 23.8,
      "z": 3.5
    }
  },
  "status": "normal",
  "readings": {
    "value": 0.8,
    "unit": "OD/meter",
    "timestamp": "2025-12-27T14:32:15Z"
  },
  "metadata": {
    "manufacturer": "SafetyTech Inc",
    "model": "ST-5000",
    "firmwareVersion": "2.4.1",
    "installationDate": "2024-06-15T10:00:00Z",
    "lastMaintenance": "2025-10-12T09:30:00Z"
  }
}
```

**Heat Detector - Alarm Status:**
```json
{
  "sensorId": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "type": "heat",
  "location": {
    "building": "West Annex",
    "floor": 3,
    "zone": "Storage Room W3-S2"
  },
  "status": "alarm",
  "readings": {
    "value": 72.5,
    "unit": "°C",
    "timestamp": "2025-12-27T14:35:42Z"
  },
  "metadata": {
    "manufacturer": "FireGuard Systems",
    "model": "FG-HT200",
    "firmwareVersion": "1.8.3",
    "installationDate": "2023-03-22T08:15:00Z",
    "lastMaintenance": "2025-09-05T11:20:00Z"
  }
}
```

**CO Detector - Trouble Status:**
```json
{
  "sensorId": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
  "type": "co",
  "location": {
    "building": "Parking Structure",
    "floor": -2,
    "zone": "Level B2 North"
  },
  "status": "trouble",
  "readings": {
    "value": 0,
    "unit": "ppm",
    "timestamp": "2025-12-27T14:38:20Z"
  },
  "metadata": {
    "manufacturer": "AirWatch Pro",
    "model": "AW-CO500",
    "firmwareVersion": "3.2.0",
    "installationDate": "2024-11-10T13:45:00Z",
    "lastMaintenance": "2025-11-08T10:30:00Z"
  }
}
```

#### Field Specifications

**sensorId (required):**
- Format: UUID v4 (RFC 4122)
- Must be globally unique
- Generated at device manufacturing
- Never changes during device lifetime
- Example: `550e8400-e29b-41d4-a716-446655440000`

**type (required):**
- Valid values: `smoke`, `heat`, `flame`, `co`, `multi`
- `multi` indicates multi-criteria detection
- Fixed at device manufacturing
- Determines expected readings format

**location (required):**
- `building`: Building identifier (string, max 100 chars)
- `floor`: Floor number (integer, 0=ground, negative=basement)
- `zone`: Zone identifier (string, max 50 chars)
- `coordinates`: Optional physical location in meters

**status (required):**
- `normal`: Operating normally, no alarm
- `alarm`: Fire condition detected
- `trouble`: Fault condition requiring attention
- `disabled`: Manually disabled or offline

**readings (required):**
- `value`: Numeric sensor reading
- `unit`: Unit of measurement (standardized units required)
- `timestamp`: ISO 8601 format with UTC timezone

**Common Units by Sensor Type:**
```
Smoke Detectors:
  - OD/meter (Optical Density per meter)
  - %/foot (Obscuration per foot)

Heat Detectors:
  - °C (Degrees Celsius)
  - °F (Degrees Fahrenheit)
  - °C/min (Rate of rise)

Flame Detectors:
  - intensity (0-100 scale)
  - W/m² (Infrared radiation)

CO Detectors:
  - ppm (Parts per million)
  - mg/m³ (Milligrams per cubic meter)
```

**metadata (required):**
- Device identification and history
- `manufacturer`: Company name
- `model`: Model number/name
- `firmwareVersion`: Current firmware version
- `installationDate`: When device was installed
- `lastMaintenance`: Last maintenance/testing date

---

### 2. Alarm Event Schema

#### Complete Schema Definition

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA Fire Safety Alarm Event",
  "type": "object",
  "required": ["eventId", "eventType", "priority", "source", "timestamp", "status"],
  "properties": {
    "eventId": {
      "type": "string",
      "format": "uuid",
      "description": "Globally unique event identifier (UUID v4)"
    },
    "eventType": {
      "type": "string",
      "enum": ["fire", "supervisory", "trouble", "test"],
      "description": "Type of alarm event"
    },
    "priority": {
      "type": "string",
      "enum": ["critical", "high", "medium", "low"],
      "description": "Event priority level"
    },
    "source": {
      "type": "object",
      "required": ["deviceId", "deviceType", "location"],
      "properties": {
        "deviceId": {"type": "string", "format": "uuid"},
        "deviceType": {"type": "string"},
        "location": {
          "type": "object",
          "properties": {
            "building": {"type": "string"},
            "floor": {"type": "integer"},
            "zone": {"type": "string"}
          }
        }
      }
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "Event occurrence time (ISO 8601 UTC)"
    },
    "description": {
      "type": "string",
      "description": "Human-readable event description"
    },
    "notifications": {
      "type": "array",
      "items": {"type": "string"},
      "description": "List of notification targets"
    },
    "status": {
      "type": "string",
      "enum": ["active", "acknowledged", "resolved"],
      "description": "Current event status"
    },
    "acknowledgedBy": {
      "type": "string",
      "description": "User ID who acknowledged (optional)"
    },
    "acknowledgedAt": {
      "type": "string",
      "format": "date-time",
      "description": "Acknowledgment timestamp (optional)"
    },
    "resolvedAt": {
      "type": "string",
      "format": "date-time",
      "description": "Resolution timestamp (optional)"
    }
  }
}
```

#### Example Alarm Event Instances

**Fire Alarm - Active:**
```json
{
  "eventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "eventType": "fire",
  "priority": "critical",
  "source": {
    "deviceId": "550e8400-e29b-41d4-a716-446655440000",
    "deviceType": "smoke_detector",
    "location": {
      "building": "Main Tower",
      "floor": 12,
      "zone": "East Wing E12-A"
    }
  },
  "timestamp": "2025-12-27T14:32:15Z",
  "description": "Smoke detected in East Wing Zone E12-A, Floor 12",
  "notifications": [
    "panel",
    "monitoring-center",
    "emergency-services",
    "building-management",
    "security-desk"
  ],
  "status": "active",
  "acknowledgedBy": null,
  "acknowledgedAt": null,
  "resolvedAt": null
}
```

**Supervisory Alarm - Acknowledged:**
```json
{
  "eventId": "1a2b3c4d-5e6f-4a5b-8c9d-0e1f2a3b4c5d",
  "eventType": "supervisory",
  "priority": "high",
  "source": {
    "deviceId": "b3c4d5e6-f7a8-4b9c-0d1e-2f3a4b5c6d7e",
    "deviceType": "sprinkler_valve",
    "location": {
      "building": "Main Tower",
      "floor": 10,
      "zone": "Mechanical Room M10"
    }
  },
  "timestamp": "2025-12-27T10:15:30Z",
  "description": "Sprinkler valve tamper detected - Floor 10 Mechanical Room",
  "notifications": [
    "panel",
    "monitoring-center",
    "maintenance-team"
  ],
  "status": "acknowledged",
  "acknowledgedBy": "maintenance-tech-042",
  "acknowledgedAt": "2025-12-27T10:18:45Z",
  "resolvedAt": null
}
```

**Trouble Event - Resolved:**
```json
{
  "eventId": "8e7f6d5c-4b3a-4291-8a7b-6c5d4e3f2a1b",
  "eventType": "trouble",
  "priority": "medium",
  "source": {
    "deviceId": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
    "deviceType": "co_detector",
    "location": {
      "building": "Parking Structure",
      "floor": -2,
      "zone": "Level B2 North"
    }
  },
  "timestamp": "2025-12-27T08:22:10Z",
  "description": "CO detector communication failure - Parking Level B2",
  "notifications": [
    "panel",
    "maintenance-team"
  ],
  "status": "resolved",
  "acknowledgedBy": "tech-156",
  "acknowledgedAt": "2025-12-27T09:05:30Z",
  "resolvedAt": "2025-12-27T11:42:15Z"
}
```

#### Event Lifecycle

```
Alarm Event State Machine:

┌────────────────────────────────────────────────────────┐
│                                                        │
│  [TRIGGERED] ──────────────────────────┐              │
│       │                                 │              │
│       │ Event created                   │              │
│       │                                 │              │
│       ▼                                 │              │
│   [ACTIVE] ───────────────────────┐    │              │
│       │                            │    │              │
│       │ User acknowledges          │    │ Auto-clear  │
│       │                            │    │ (test mode) │
│       ▼                            │    │              │
│ [ACKNOWLEDGED]                     │    │              │
│       │                            │    │              │
│       │ Investigation complete     │    │              │
│       │ Condition cleared          │    │              │
│       │                            │    │              │
│       ▼                            ▼    ▼              │
│   [RESOLVED] ◄─────────────────────────────────────────┘
│                                                         │
└─────────────────────────────────────────────────────────┘

Timing Requirements:
- TRIGGERED → ACTIVE: Immediate (0ms)
- ACTIVE → ACKNOWLEDGED: User action (typically 30s-5min)
- ACKNOWLEDGED → RESOLVED: Variable (investigation + remediation)
```

---

### 3. Device Metadata Schema

#### Complete Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA Fire Safety Device Metadata",
  "type": "object",
  "required": ["deviceId", "deviceType", "manufacturer", "model"],
  "properties": {
    "deviceId": {
      "type": "string",
      "format": "uuid"
    },
    "deviceType": {
      "type": "string",
      "enum": [
        "smoke_detector",
        "heat_detector",
        "flame_detector",
        "co_detector",
        "multi_sensor",
        "manual_pull_station",
        "horn",
        "strobe",
        "horn_strobe",
        "speaker",
        "control_panel",
        "annunciator",
        "relay_module",
        "interface_module"
      ]
    },
    "manufacturer": {"type": "string"},
    "model": {"type": "string"},
    "firmwareVersion": {"type": "string"},
    "hardwareRevision": {"type": "string"},
    "serialNumber": {"type": "string"},
    "manufacturingDate": {"type": "string", "format": "date"},
    "installationDate": {"type": "string", "format": "date-time"},
    "warrantyExpiration": {"type": "string", "format": "date"},
    "lastMaintenance": {"type": "string", "format": "date-time"},
    "maintenanceInterval": {
      "type": "integer",
      "description": "Days between maintenance"
    },
    "certifications": {
      "type": "array",
      "items": {"type": "string"},
      "description": "UL, FM, EN54, etc."
    }
  }
}
```

---

## Data Validation

### Validation Requirements

All data MUST be validated before processing:

**Schema Validation:**
```javascript
const Ajv = require('ajv');
const ajv = new Ajv();

// Load schema
const sensorSchema = require('./schemas/sensor-data.json');
const validate = ajv.compile(sensorSchema);

// Validate data
function validateSensorData(data) {
  const valid = validate(data);
  if (!valid) {
    console.error('Validation errors:', validate.errors);
    return false;
  }
  return true;
}

// Example usage
const sensorData = {
  sensorId: "550e8400-e29b-41d4-a716-446655440000",
  type: "smoke",
  // ... rest of data
};

if (validateSensorData(sensorData)) {
  processSensorData(sensorData);
}
```

### Common Validation Errors

**Invalid UUID Format:**
```json
// INVALID
{"sensorId": "12345"}

// VALID
{"sensorId": "550e8400-e29b-41d4-a716-446655440000"}
```

**Invalid Enumeration Value:**
```json
// INVALID
{"type": "smoke-detector"}

// VALID
{"type": "smoke"}
```

**Missing Required Fields:**
```json
// INVALID (missing status)
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "location": {...},
  "readings": {...}
}

// VALID
{
  "sensorId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "smoke",
  "location": {...},
  "status": "normal",
  "readings": {...},
  "metadata": {...}
}
```

**Invalid Timestamp Format:**
```json
// INVALID
{"timestamp": "2025-12-27 14:32:15"}

// VALID
{"timestamp": "2025-12-27T14:32:15Z"}
```

---

## Key Takeaways

1. **Standardized JSON schemas** ensure universal data representation across all vendors.

2. **Sensor data schema** provides complete device information including location, status, readings, and metadata.

3. **Alarm event schema** tracks the complete lifecycle from trigger to resolution.

4. **Validation is mandatory** using JSON Schema to ensure data integrity.

5. **UUID v4 identifiers** guarantee global uniqueness without coordination.

---

## Review Questions

1. Why is JSON chosen as the data format standard?
2. What are the five valid sensor types?
3. What is the lifecycle of an alarm event?
4. Why must sensor IDs be UUID v4 format?
5. What validation mechanisms ensure data integrity?

---

## Next Steps

Chapter 5 explores Phase 2, defining the API interfaces that use these data formats for system communication.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
