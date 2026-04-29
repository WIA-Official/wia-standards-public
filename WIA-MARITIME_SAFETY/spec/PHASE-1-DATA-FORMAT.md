# WIA-MARITIME_SAFETY: Phase 1 - Data Format Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the data format requirements for Maritime Safety systems. All implementations MUST follow these specifications to ensure interoperability across vessel tracking, safety monitoring, and emergency response systems worldwide.

## 2. Core Data Structures

### 2.1 Vessel Identity Record

```json
{
  "type": "WIA-MARITIME_SAFETY-VesselIdentity",
  "version": "1.0",
  "mmsi": "string (9 digits)",
  "imo": "string (7 digits)",
  "callSign": "string",
  "vesselName": "string",
  "vesselType": "string",
  "dimensions": {
    "length": "number (meters)",
    "beam": "number (meters)",
    "draft": "number (meters)",
    "height": "number (meters)"
  },
  "flag": "string (ISO 3166-1 alpha-2)",
  "timestamp": "ISO 8601",
  "signature": "string (Ed25519)"
}
```

### 2.2 AIS Position Report (Type 1, 2, 3)

```json
{
  "type": "WIA-MARITIME_SAFETY-AISPosition",
  "version": "1.0",
  "mmsi": "string (9 digits)",
  "timestamp": "ISO 8601",
  "position": {
    "latitude": "number (-90 to 90)",
    "longitude": "number (-180 to 180)",
    "accuracy": "boolean"
  },
  "navigation": {
    "status": "enum",
    "rateOfTurn": "number (degrees/min)",
    "speedOverGround": "number (knots)",
    "courseOverGround": "number (degrees)",
    "heading": "number (degrees)"
  },
  "maneuver": "enum",
  "raim": "boolean",
  "signature": "string (Ed25519)"
}
```

### 2.3 Weather Data Format

```json
{
  "type": "WIA-MARITIME_SAFETY-Weather",
  "version": "1.0",
  "id": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "position": {
    "latitude": "number",
    "longitude": "number"
  },
  "conditions": {
    "windSpeed": "number (knots)",
    "windDirection": "number (degrees)",
    "waveHeight": "number (meters)",
    "wavePeriod": "number (seconds)",
    "visibility": "number (nautical miles)",
    "seaTemperature": "number (celsius)",
    "airTemperature": "number (celsius)",
    "barometricPressure": "number (hPa)"
  },
  "forecast": {
    "validUntil": "ISO 8601",
    "warnings": ["array of strings"]
  }
}
```

### 2.4 Safety Alert Format

```json
{
  "type": "WIA-MARITIME_SAFETY-Alert",
  "version": "1.0",
  "id": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "severity": "enum (info|warning|critical|distress)",
  "category": "enum (collision|grounding|fire|flooding|medical|piracy|pollution)",
  "position": {
    "latitude": "number",
    "longitude": "number"
  },
  "vessel": {
    "mmsi": "string",
    "name": "string"
  },
  "description": "string",
  "status": "enum (active|acknowledged|resolved)",
  "responders": ["array of MMSI strings"]
}
```

## 3. Field Definitions

### 3.1 Vessel Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| mmsi | string | Yes | Maritime Mobile Service Identity (9 digits) |
| imo | string | No | International Maritime Organization number (7 digits) |
| callSign | string | Yes | Radio call sign |
| vesselName | string | Yes | Vessel name (max 20 chars) |
| vesselType | enum | Yes | See section 3.3 for types |
| flag | string | Yes | Flag state (ISO 3166-1 alpha-2) |

### 3.2 Position Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| latitude | number | Yes | Latitude in decimal degrees (-90 to 90) |
| longitude | number | Yes | Longitude in decimal degrees (-180 to 180) |
| accuracy | boolean | Yes | Position accuracy: true = high (<10m), false = low (>10m) |
| speedOverGround | number | Yes | Speed in knots (0 to 102.2) |
| courseOverGround | number | Yes | Course in degrees (0 to 359.9) |
| heading | number | Yes | True heading in degrees (0 to 359) |

### 3.3 Vessel Type Enumeration

| Code | Type | Description |
|------|------|-------------|
| 20-29 | Wing In Ground | WIG craft |
| 30 | Fishing | Fishing vessel |
| 31-32 | Towing | Towing or pushing |
| 33 | Dredging | Dredging or underwater operations |
| 34 | Diving | Diving operations |
| 35 | Military | Military operations |
| 36 | Sailing | Sailing vessel |
| 37 | Pleasure | Pleasure craft |
| 40-49 | High Speed | High-speed craft |
| 50 | Pilot | Pilot vessel |
| 51 | SAR | Search and rescue |
| 52 | Tug | Tug |
| 53 | Port Tender | Port tender |
| 60-69 | Passenger | Passenger ship |
| 70-79 | Cargo | Cargo ship |
| 80-89 | Tanker | Tanker |

### 3.4 Navigation Status Enumeration

| Code | Status | Description |
|------|--------|-------------|
| 0 | Under way using engine | Powered navigation |
| 1 | At anchor | Anchored |
| 2 | Not under command | Unable to maneuver |
| 3 | Restricted maneuverability | Limited maneuverability |
| 4 | Constrained by draught | Deep draft restrictions |
| 5 | Moored | Moored to shore/dock |
| 6 | Aground | Run aground |
| 7 | Engaged in fishing | Fishing operations |
| 8 | Under way sailing | Sailing |
| 14 | AIS-SART | Search and Rescue Transmitter |
| 15 | Undefined | Status not defined |

## 4. Encoding Requirements

### 4.1 Character Encoding
- All text MUST be UTF-8 encoded
- No BOM (Byte Order Mark) allowed
- Line endings: LF (Unix style)
- Vessel names: ASCII characters only (per AIS specification)

### 4.2 Binary Encoding
- AIS messages: 6-bit ASCII encoding
- Position coordinates: Signed integers (latitude: 1/10000 minute precision)
- Time: UTC timezone
- Timestamps: ISO 8601 format with 'Z' suffix

### 4.3 Precision Requirements

| Data Type | Precision | Format |
|-----------|-----------|--------|
| Latitude | ±0.0001 degrees | 4 decimal places |
| Longitude | ±0.0001 degrees | 4 decimal places |
| Speed | ±0.1 knots | 1 decimal place |
| Course | ±0.1 degrees | 1 decimal place |
| Distance | ±0.01 nautical miles | 2 decimal places |

## 5. Validation Rules

### 5.1 Schema Validation

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["type", "version", "mmsi", "timestamp", "position"],
  "properties": {
    "type": {
      "type": "string",
      "enum": [
        "WIA-MARITIME_SAFETY-VesselIdentity",
        "WIA-MARITIME_SAFETY-AISPosition",
        "WIA-MARITIME_SAFETY-Weather",
        "WIA-MARITIME_SAFETY-Alert"
      ]
    },
    "version": { "type": "string", "pattern": "^\\d+\\.\\d+$" },
    "mmsi": { "type": "string", "pattern": "^[0-9]{9}$" },
    "timestamp": { "type": "string", "format": "date-time" },
    "position": {
      "type": "object",
      "required": ["latitude", "longitude"],
      "properties": {
        "latitude": { "type": "number", "minimum": -90, "maximum": 90 },
        "longitude": { "type": "number", "minimum": -180, "maximum": 180 }
      }
    }
  }
}
```

### 5.2 Validation Errors

| Code | Message | Resolution |
|------|---------|------------|
| E001 | Invalid MMSI format | Must be exactly 9 digits |
| E002 | Invalid IMO format | Must be exactly 7 digits with valid checksum |
| E003 | Position out of range | Latitude: -90 to 90, Longitude: -180 to 180 |
| E004 | Invalid timestamp | Must be ISO 8601 format in UTC |
| E005 | Speed out of range | Must be 0 to 102.2 knots |
| E006 | Course out of range | Must be 0 to 359.9 degrees |
| E007 | Invalid vessel type | Must be valid AIS ship type code |

## 6. Versioning

### 6.1 Version Format
- Major.Minor (e.g., 1.0, 2.1)
- Major version: Breaking changes to data structure
- Minor version: Backward-compatible additions

### 6.2 Migration Path
- v1.0 → v1.1: Add optional cyber security fields
- v1.x → v2.0: Enhanced weather integration

## 7. Examples

### 7.1 Complete Vessel Position Report

```json
{
  "type": "WIA-MARITIME_SAFETY-AISPosition",
  "version": "1.0",
  "mmsi": "367123450",
  "timestamp": "2026-01-12T14:30:00Z",
  "position": {
    "latitude": 37.8044,
    "longitude": -122.4162,
    "accuracy": true
  },
  "navigation": {
    "status": "Under way using engine",
    "rateOfTurn": 0,
    "speedOverGround": 12.5,
    "courseOverGround": 285.3,
    "heading": 287
  },
  "maneuver": "no special maneuver",
  "raim": true,
  "signature": "3045022100ab2c..."
}
```

### 7.2 Distress Alert

```json
{
  "type": "WIA-MARITIME_SAFETY-Alert",
  "version": "1.0",
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2026-01-12T15:45:30Z",
  "severity": "distress",
  "category": "fire",
  "position": {
    "latitude": 35.6762,
    "longitude": 139.6503
  },
  "vessel": {
    "mmsi": "431234567",
    "name": "OCEAN STAR"
  },
  "description": "Engine room fire, requesting immediate assistance",
  "status": "active",
  "responders": ["431000001", "431000002"]
}
```

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
