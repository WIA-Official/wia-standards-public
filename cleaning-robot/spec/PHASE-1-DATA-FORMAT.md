# WIA-ROB-011 Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines the standardized data formats for cleaning robots, including robot state representation, map data structures, sensor readings, and cleaning logs. All data MUST use JSON-LD format for semantic interoperability.

## 2. Core Data Types

### 2.1 Robot Identity

```json
{
  "@context": "https://wiastandards.com/rob-011/v1",
  "@type": "CleaningRobot",
  "robotId": "ROB-2025-XXXX-YYYY",
  "manufacturer": "string",
  "model": "string",
  "serialNumber": "string",
  "firmwareVersion": "semver",
  "hardwareVersion": "string",
  "manufactureDate": "ISO8601 date",
  "capabilities": ["array", "of", "capability", "strings"]
}
```

### 2.2 Robot State

```json
{
  "@type": "RobotState",
  "timestamp": "ISO8601 datetime",
  "battery": {
    "percentage": 0-100,
    "voltage": "float (V)",
    "current": "float (A)",
    "temperature": "float (°C)",
    "health": 0-100,
    "cycleCount": "integer"
  },
  "pose": {
    "x": "float (meters)",
    "y": "float (meters)",
    "theta": "float (radians)",
    "floor": "integer",
    "confidence": 0-1
  },
  "mode": "idle|cleaning|charging|returning|error",
  "cleaningMode": "auto|spot|edge|zigzag|spiral",
  "suctionPower": 0-100,
  "brushSpeed": 0-100,
  "waterFlow": 0-100,
  "sensors": {
    "lidar": "SensorReading",
    "camera": "SensorReading",
    "cliff": ["boolean", "array"],
    "bumper": "boolean",
    "dirtLevel": 0-100
  }
}
```

### 2.3 Map Data Format

```json
{
  "@type": "OccupancyGrid",
  "resolution": 0.05,
  "width": "integer (cells)",
  "height": "integer (cells)",
  "origin": {
    "x": "float (meters)",
    "y": "float (meters)",
    "theta": "float (radians)"
  },
  "data": "base64 encoded uint8 array",
  "rooms": [
    {
      "id": "UUID",
      "name": "string",
      "polygon": [[x, y], ...],
      "surfaceType": "hardwood|tile|carpet|vinyl|marble",
      "lastCleaned": "ISO8601 datetime",
      "cleanCount": "integer"
    }
  ],
  "noGoZones": [
    {
      "id": "UUID",
      "polygon": [[x, y], ...],
      "type": "no_go|restricted"
    }
  ],
  "dockLocation": {
    "x": "float",
    "y": "float",
    "theta": "float"
  }
}
```

### 2.4 Cleaning Session Log

```json
{
  "@type": "CleaningSession",
  "sessionId": "UUID",
  "startTime": "ISO8601 datetime",
  "endTime": "ISO8601 datetime",
  "duration": "integer (seconds)",
  "areaCleaned": "float (m²)",
  "distanceTraveled": "float (m)",
  "batteryConsumed": "float (Wh)",
  "cleaningMode": "string",
  "rooms": ["room_id", "array"],
  "interruptions": [
    {
      "timestamp": "ISO8601 datetime",
      "reason": "stuck|battery_low|manual_stop|error",
      "duration": "integer (seconds)"
    }
  ],
  "statistics": {
    "avgSpeed": "float (m/s)",
    "avgSuction": "float (%)",
    "dirtCollected": "float (g)",
    "coverage": "float (%)"
  }
}
```

## 3. Sensor Data Formats

### 3.1 LiDAR Scan

```json
{
  "@type": "LidarScan",
  "timestamp": "ISO8601 datetime",
  "angleMin": "float (radians)",
  "angleMax": "float (radians)",
  "angleIncrement": "float (radians)",
  "rangeMin": "float (meters)",
  "rangeMax": "float (meters)",
  "ranges": ["float", "array"],
  "intensities": ["float", "array (optional)"]
}
```

### 3.2 Camera Image

```json
{
  "@type": "CameraImage",
  "timestamp": "ISO8601 datetime",
  "width": "integer",
  "height": "integer",
  "encoding": "rgb8|bgr8|jpeg|png",
  "data": "base64 string",
  "detectedObjects": [
    {
      "class": "string",
      "confidence": 0-1,
      "boundingBox": {"x": int, "y": int, "w": int, "h": int}
    }
  ]
}
```

## 4. Event Data Format

```json
{
  "@type": "RobotEvent",
  "eventId": "UUID",
  "timestamp": "ISO8601 datetime",
  "type": "info|warning|error|critical",
  "category": "navigation|cleaning|battery|sensor|communication",
  "message": "string",
  "details": {
    "arbitrary": "key-value pairs"
  }
}
```

## 5. Command Format

```json
{
  "@type": "RobotCommand",
  "commandId": "UUID",
  "timestamp": "ISO8601 datetime",
  "action": "start|stop|pause|resume|dock|set_mode",
  "parameters": {
    "mode": "string (optional)",
    "rooms": ["array (optional)"],
    "powerLevel": "integer (optional)"
  }
}
```

## 6. Validation Rules

1. All timestamps MUST use ISO 8601 format with timezone
2. All measurements MUST use SI units
3. All arrays MUST have consistent element types
4. All UUIDs MUST be version 4
5. Required fields MUST NOT be null
6. Enum values MUST match specification exactly

## 7. Extensibility

Implementations MAY add custom fields prefixed with "x_" to avoid conflicts with future standard additions.

Example:
```json
{
  "@type": "RobotState",
  "battery": 75,
  "x_customField": "vendor-specific data"
}
```

---

© 2025 WIA · MIT License

## 8. Data Versioning

All data formats include version information for backward compatibility:

```json
{
  "@context": "https://wiastandards.com/rob-011/v1",
  "@version": "1.0.0",
  "schemaVersion": "1.0.0"
}
```

### Migration Guidelines

When updating data formats:
1. Maintain backward compatibility for minor versions
2. Provide migration tools for major versions
3. Document all breaking changes
4. Support multiple versions during transition periods

## 9. Data Examples

### Complete Robot State Example

```json
{
  "@context": "https://wiastandards.com/rob-011/v1",
  "@type": "RobotState",
  "timestamp": "2025-12-26T14:30:00Z",
  "battery": {
    "percentage": 75,
    "voltage": 14.8,
    "current": 2.5,
    "temperature": 35.2,
    "health": 95,
    "cycleCount": 245,
    "charging": false,
    "timeToEmpty": 3600
  },
  "pose": {
    "x": 2.5,
    "y": 3.1,
    "theta": 1.57,
    "floor": 0,
    "confidence": 0.95
  },
  "mode": "cleaning",
  "cleaningMode": "auto",
  "suctionPower": 80,
  "brushSpeed": 100,
  "sensors": {
    "lidar": {
      "timestamp": "2025-12-26T14:30:00Z",
      "ranges": [1.2, 1.5, 2.0]
    },
    "cliff": [false, false, false, false],
    "bumper": false,
    "dirtLevel": 45
  }
}
```

