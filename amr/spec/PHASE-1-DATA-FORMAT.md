# WIA-AMR Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## 1. Overview

Phase 1 of the WIA-AMR standard defines the data formats used for information exchange between Autonomous Mobile Robots (AMRs), Fleet Management Systems (FMS), and enterprise systems. This specification uses JSON Schema to provide precise, machine-readable definitions.

### 1.1 Design Principles

- **Simplicity**: Minimal required fields, optional extensions
- **Compatibility**: Alignment with VDA 5050 where applicable
- **Extensibility**: Custom fields via extensions namespace
- **Validation**: Strict JSON Schema for automatic validation

### 1.2 Document Conventions

- **MUST/SHALL**: Absolute requirement
- **SHOULD**: Recommended but not mandatory
- **MAY**: Optional feature

---

## 2. Coordinate System and Units

### 2.1 Standard Units

All measurements use SI units unless otherwise specified:

| Physical Quantity | Unit | Symbol | Notes |
|-------------------|------|--------|-------|
| Distance | meter | m | Millimeter precision recommended |
| Angle | radian | rad | Range: -π to π |
| Time | second | s | Timestamps in ISO 8601 |
| Velocity | meter/second | m/s | |
| Angular Velocity | radian/second | rad/s | |
| Acceleration | meter/second² | m/s² | |
| Mass | kilogram | kg | |
| Temperature | Celsius | °C | |
| Voltage | volt | V | |
| Current | ampere | A | |

### 2.2 Coordinate System

WIA-AMR uses a **right-handed coordinate system**:

- **X-axis**: Forward (robot's direction of travel)
- **Y-axis**: Left (perpendicular to X)
- **Z-axis**: Up (vertical)
- **Theta (θ)**: Counter-clockwise positive from X-axis

```
        Y (Left)
        ^
        |
        |
        +-------> X (Forward)
       /
      /
     Z (Up)
```

### 2.3 Timestamp Format

All timestamps MUST be in ISO 8601 format with timezone:

```
2025-01-15T10:30:00.123Z
2025-01-15T19:30:00.123+09:00
```

---

## 3. Core Data Types

### 3.1 Position

Represents a 2D or 3D position in the environment.

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wiastandards.com/schemas/amr/position.json",
  "title": "Position",
  "type": "object",
  "properties": {
    "x": {
      "type": "number",
      "description": "X coordinate in meters"
    },
    "y": {
      "type": "number",
      "description": "Y coordinate in meters"
    },
    "z": {
      "type": "number",
      "description": "Z coordinate in meters (optional for 2D)"
    },
    "theta": {
      "type": "number",
      "minimum": -3.14159265359,
      "maximum": 3.14159265359,
      "description": "Orientation in radians"
    },
    "mapId": {
      "type": "string",
      "description": "Reference map identifier"
    },
    "positionInitialized": {
      "type": "boolean",
      "description": "Whether position is reliably initialized"
    },
    "localizationScore": {
      "type": "number",
      "minimum": 0,
      "maximum": 1,
      "description": "Confidence of localization (0-1)"
    }
  },
  "required": ["x", "y", "theta"]
}
```

### 3.2 Velocity

Represents linear and angular velocity.

```json
{
  "$id": "https://wiastandards.com/schemas/amr/velocity.json",
  "title": "Velocity",
  "type": "object",
  "properties": {
    "linear": {
      "type": "number",
      "description": "Linear velocity in m/s"
    },
    "angular": {
      "type": "number",
      "description": "Angular velocity in rad/s"
    },
    "vx": {
      "type": "number",
      "description": "X component of velocity (m/s)"
    },
    "vy": {
      "type": "number",
      "description": "Y component of velocity (m/s)"
    },
    "omega": {
      "type": "number",
      "description": "Rotational velocity (rad/s)"
    }
  }
}
```

### 3.3 BatteryState

Battery status information.

```json
{
  "$id": "https://wiastandards.com/schemas/amr/battery-state.json",
  "title": "BatteryState",
  "type": "object",
  "properties": {
    "level": {
      "type": "number",
      "minimum": 0,
      "maximum": 100,
      "description": "Battery level percentage"
    },
    "voltage": {
      "type": "number",
      "description": "Battery voltage in volts"
    },
    "current": {
      "type": "number",
      "description": "Current draw in amperes"
    },
    "temperature": {
      "type": "number",
      "description": "Battery temperature in Celsius"
    },
    "charging": {
      "type": "boolean",
      "description": "Whether battery is currently charging"
    },
    "health": {
      "type": "number",
      "minimum": 0,
      "maximum": 100,
      "description": "Battery health percentage"
    },
    "timeToEmpty": {
      "type": "integer",
      "description": "Estimated time to empty in seconds"
    },
    "timeToFull": {
      "type": "integer",
      "description": "Estimated time to full charge in seconds"
    }
  },
  "required": ["level"]
}
```

---

## 4. RobotState Schema

The RobotState schema represents the complete current state of an AMR.

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wiastandards.com/schemas/amr/robot-state.json",
  "title": "RobotState",
  "description": "Current state of an AMR robot",
  "type": "object",
  "properties": {
    "robotId": {
      "type": "string",
      "pattern": "^[A-Za-z0-9-_]+$",
      "description": "Unique robot identifier"
    },
    "manufacturer": {
      "type": "string",
      "description": "Robot manufacturer name"
    },
    "serialNumber": {
      "type": "string",
      "description": "Robot serial number"
    },
    "model": {
      "type": "string",
      "description": "Robot model name"
    },
    "softwareVersion": {
      "type": "string",
      "description": "Robot software version"
    },
    "position": {
      "$ref": "position.json"
    },
    "velocity": {
      "$ref": "velocity.json"
    },
    "battery": {
      "$ref": "battery-state.json"
    },
    "operatingState": {
      "type": "string",
      "enum": [
        "IDLE",
        "NAVIGATING",
        "WAITING",
        "CHARGING",
        "PAUSED",
        "ERROR",
        "EMERGENCY_STOP",
        "MANUAL"
      ],
      "description": "Current operating state"
    },
    "safetyState": {
      "type": "string",
      "enum": [
        "SAFE",
        "WARNING",
        "PROTECTIVE_STOP",
        "EMERGENCY_STOP"
      ],
      "description": "Safety system state"
    },
    "driving": {
      "type": "boolean",
      "description": "Whether robot is currently moving"
    },
    "paused": {
      "type": "boolean",
      "description": "Whether robot is paused"
    },
    "currentTaskId": {
      "type": "string",
      "description": "Currently executing task ID"
    },
    "lastCompletedTaskId": {
      "type": "string",
      "description": "Last completed task ID"
    },
    "loads": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Load"
      },
      "description": "Current loads on robot"
    },
    "errors": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Error"
      },
      "description": "Active errors"
    },
    "warnings": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/Warning"
      },
      "description": "Active warnings"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "State measurement time"
    },
    "extensions": {
      "type": "object",
      "description": "Vendor-specific extensions"
    }
  },
  "required": ["robotId", "position", "operatingState", "timestamp"],
  "definitions": {
    "Load": {
      "type": "object",
      "properties": {
        "loadId": { "type": "string" },
        "loadType": { "type": "string" },
        "weight": { "type": "number" },
        "dimensions": {
          "type": "object",
          "properties": {
            "length": { "type": "number" },
            "width": { "type": "number" },
            "height": { "type": "number" }
          }
        }
      }
    },
    "Error": {
      "type": "object",
      "properties": {
        "errorId": { "type": "string" },
        "errorType": { "type": "string" },
        "errorLevel": {
          "type": "string",
          "enum": ["WARNING", "ERROR", "FATAL"]
        },
        "errorDescription": { "type": "string" },
        "errorHint": { "type": "string" },
        "timestamp": { "type": "string", "format": "date-time" }
      },
      "required": ["errorId", "errorType", "errorLevel"]
    },
    "Warning": {
      "type": "object",
      "properties": {
        "warningId": { "type": "string" },
        "warningType": { "type": "string" },
        "warningDescription": { "type": "string" },
        "timestamp": { "type": "string", "format": "date-time" }
      }
    }
  }
}
```

---

## 5. Task Schema

The Task schema defines work items assigned to robots.

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wiastandards.com/schemas/amr/task.json",
  "title": "Task",
  "type": "object",
  "properties": {
    "taskId": {
      "type": "string",
      "description": "Unique task identifier"
    },
    "taskType": {
      "type": "string",
      "enum": ["NAVIGATE", "PICK", "PLACE", "DOCK", "UNDOCK", "WAIT", "CHARGE", "CUSTOM"],
      "description": "Type of task"
    },
    "priority": {
      "type": "integer",
      "minimum": 0,
      "maximum": 100,
      "default": 50,
      "description": "Task priority (higher = more urgent)"
    },
    "robotId": {
      "type": "string",
      "description": "Assigned robot ID"
    },
    "source": {
      "$ref": "position.json",
      "description": "Source location"
    },
    "destination": {
      "$ref": "position.json",
      "description": "Destination location"
    },
    "actions": {
      "type": "array",
      "items": { "$ref": "#/definitions/Action" },
      "description": "Actions to perform at destination"
    },
    "constraints": {
      "$ref": "#/definitions/TaskConstraints"
    },
    "status": {
      "type": "string",
      "enum": ["PENDING", "ASSIGNED", "IN_PROGRESS", "COMPLETED", "FAILED", "CANCELLED"],
      "description": "Current task status"
    },
    "progress": {
      "type": "number",
      "minimum": 0,
      "maximum": 100,
      "description": "Task completion percentage"
    },
    "externalRef": {
      "type": "object",
      "properties": {
        "system": { "type": "string" },
        "orderId": { "type": "string" },
        "lineItemId": { "type": "string" }
      },
      "description": "Reference to external system"
    },
    "createdAt": { "type": "string", "format": "date-time" },
    "assignedAt": { "type": "string", "format": "date-time" },
    "startedAt": { "type": "string", "format": "date-time" },
    "completedAt": { "type": "string", "format": "date-time" },
    "payload": {
      "type": "object",
      "description": "Task-specific data"
    }
  },
  "required": ["taskId", "taskType"],
  "definitions": {
    "Action": {
      "type": "object",
      "properties": {
        "actionId": { "type": "string" },
        "actionType": {
          "type": "string",
          "enum": ["LIFT", "DROP", "BEEP", "WAIT", "SCAN", "OPEN_DOOR", "CLOSE_DOOR", "CUSTOM"]
        },
        "parameters": { "type": "object" },
        "blockingType": {
          "type": "string",
          "enum": ["HARD", "SOFT", "NONE"]
        }
      },
      "required": ["actionType"]
    },
    "TaskConstraints": {
      "type": "object",
      "properties": {
        "deadline": { "type": "string", "format": "date-time" },
        "maxDuration": { "type": "integer" },
        "allowedZones": { "type": "array", "items": { "type": "string" } },
        "forbiddenZones": { "type": "array", "items": { "type": "string" } },
        "requiredCapabilities": { "type": "array", "items": { "type": "string" } },
        "maxSpeed": { "type": "number" }
      }
    }
  }
}
```

---

## 6. Telemetry Schema

High-frequency sensor and system data.

```json
{
  "$id": "https://wiastandards.com/schemas/amr/telemetry.json",
  "title": "Telemetry",
  "type": "object",
  "properties": {
    "robotId": { "type": "string" },
    "timestamp": { "type": "string", "format": "date-time" },
    "sensors": {
      "type": "object",
      "properties": {
        "lidar": {
          "type": "object",
          "properties": {
            "ranges": { "type": "array", "items": { "type": "number" } },
            "angleMin": { "type": "number" },
            "angleMax": { "type": "number" },
            "angleIncrement": { "type": "number" },
            "rangeMin": { "type": "number" },
            "rangeMax": { "type": "number" }
          }
        },
        "imu": {
          "type": "object",
          "properties": {
            "orientation": { "$ref": "#/definitions/Quaternion" },
            "angularVelocity": { "$ref": "#/definitions/Vector3" },
            "linearAcceleration": { "$ref": "#/definitions/Vector3" }
          }
        },
        "odometry": {
          "type": "object",
          "properties": {
            "pose": { "$ref": "position.json" },
            "twist": {
              "type": "object",
              "properties": {
                "linear": { "$ref": "#/definitions/Vector3" },
                "angular": { "$ref": "#/definitions/Vector3" }
              }
            }
          }
        }
      }
    },
    "system": {
      "type": "object",
      "properties": {
        "cpuUsage": { "type": "number", "minimum": 0, "maximum": 100 },
        "memoryUsage": { "type": "number", "minimum": 0, "maximum": 100 },
        "diskUsage": { "type": "number", "minimum": 0, "maximum": 100 },
        "networkLatency": { "type": "number" },
        "uptime": { "type": "integer" }
      }
    }
  },
  "definitions": {
    "Quaternion": {
      "type": "object",
      "properties": {
        "x": { "type": "number" },
        "y": { "type": "number" },
        "z": { "type": "number" },
        "w": { "type": "number" }
      },
      "required": ["x", "y", "z", "w"]
    },
    "Vector3": {
      "type": "object",
      "properties": {
        "x": { "type": "number" },
        "y": { "type": "number" },
        "z": { "type": "number" }
      },
      "required": ["x", "y", "z"]
    }
  }
}
```

---

## 7. Schema Validation

### 7.1 Validation Requirements

- All incoming data MUST be validated against the appropriate schema
- Invalid data SHOULD be rejected with a descriptive error
- Unknown properties SHOULD be ignored (for forward compatibility)

### 7.2 Validation Example (TypeScript)

```typescript
import Ajv from 'ajv';
import addFormats from 'ajv-formats';

const ajv = new Ajv({ allErrors: true });
addFormats(ajv);

const schema = require('./schemas/robot-state.json');
const validate = ajv.compile(schema);

function validateRobotState(data: unknown): boolean {
  const valid = validate(data);
  if (!valid) {
    console.error('Validation errors:', validate.errors);
  }
  return valid;
}
```

---

## 8. Extensions

Vendor-specific data can be added using the `extensions` field:

```json
{
  "robotId": "amr-001",
  "position": { "x": 10.5, "y": 20.3, "theta": 1.57 },
  "operatingState": "NAVIGATING",
  "timestamp": "2025-01-15T10:30:00Z",
  "extensions": {
    "vendor:acme": {
      "internalTemperature": 45.2,
      "motorCurrent": [1.2, 1.3, 1.1, 1.4]
    }
  }
}
```

**Extension naming convention**: `vendor:{company}:{field}`

---

## 9. References

- JSON Schema: https://json-schema.org/
- VDA 5050: https://github.com/VDA5050/VDA5050
- ISO 8601: https://www.iso.org/iso-8601-date-and-time-format.html

---

© 2025 WIA Standards | 弘益人間 · Benefit All Humanity
