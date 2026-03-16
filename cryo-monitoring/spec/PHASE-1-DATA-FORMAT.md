# WIA Cryo-Monitoring Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Final
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)
**Standard ID**: WIA-CRYO-008

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Message Structure](#base-message-structure)
4. [Sensor Reading Format](#sensor-reading-format)
5. [Alert Event Format](#alert-event-format)
6. [LN2 Level Format](#ln2-level-format)
7. [Calibration Record Format](#calibration-record-format)
8. [JSON Schema Definitions](#json-schema-definitions)
9. [Validation Rules](#validation-rules)
10. [Examples](#examples)

---

## 1. Overview

### 1.1 Purpose

The WIA Cryo-Monitoring Data Format Standard defines standardized JSON-based structures for representing sensor readings, alerts, LN2 levels, and calibration records in cryogenic storage facilities. This ensures interoperability between monitoring systems, data analysis tools, and cloud platforms.

### 1.2 Scope

This standard covers:

- **Sensor Data**: Temperature readings from RTD, thermocouples, and thermistors
- **Alert Events**: Notifications of threshold breaches and anomalies
- **LN2 Level Data**: Liquid nitrogen inventory and consumption metrics
- **Calibration Records**: Sensor calibration history and certificates
- **System Metadata**: Facility, zone, and equipment identification

### 1.3 Design Principles

1. **JSON Format**: Human-readable, widely supported, easy to parse
2. **Self-Describing**: Include metadata for context and traceability
3. **Extensible**: Support custom fields without breaking compatibility
4. **Typed**: Explicit data types (string, number, boolean, timestamp)
5. **Validated**: JSON Schema enforcement for data integrity

---

## 2. Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Facility** | Physical location containing cryogenic storage equipment |
| **Zone** | Distinct storage area within a facility (e.g., Zone-A, Zone-B) |
| **Sensor** | Device measuring temperature or LN2 level |
| **Reading** | Single measurement from a sensor at a point in time |
| **Alert** | Notification triggered when conditions breach thresholds |
| **Quality Code** | Indicator of measurement reliability (GOOD, BAD, UNCERTAIN) |

### 2.2 Data Types

| Type | Description | Format | Example |
|------|-------------|--------|---------|
| `string` | UTF-8 text | N/A | `"RTD-A1"` |
| `number` | IEEE 754 double | Decimal | `-195.8` |
| `integer` | Signed 64-bit int | Whole number | `42` |
| `boolean` | True/false | `true` or `false` | `true` |
| `timestamp` | ISO 8601 datetime | `YYYY-MM-DDTHH:MM:SS.SSSZ` | `"2025-12-25T14:32:15.847Z"` |
| `enum` | Fixed set of values | String | `"CRITICAL"` |

---

## 3. Base Message Structure

All messages share a common base structure:

```json
{
  "messageType": "sensorReading | alertEvent | ln2Level | calibrationRecord",
  "version": "1.0.0",
  "timestamp": "2025-12-25T14:32:15.847Z",
  "facilityId": "WIA-CRYO-FAC-001",
  "messageId": "MSG-UUID-123456789"
}
```

### 3.1 Field Specifications

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `messageType` | enum | YES | Type of message (see list above) |
| `version` | string | YES | Data format version (SemVer) |
| `timestamp` | timestamp | YES | When message was generated (UTC) |
| `facilityId` | string | YES | Unique facility identifier |
| `messageId` | string | NO | Unique message ID (UUID recommended) |

---

## 4. Sensor Reading Format

### 4.1 Structure

```json
{
  "messageType": "sensorReading",
  "version": "1.0.0",
  "timestamp": "2025-12-25T14:32:15.847Z",
  "facilityId": "WIA-CRYO-FAC-001",
  "zoneId": "ZONE-A",
  "sensor": {
    "id": "RTD-A1",
    "type": "PT100_RTD",
    "location": "Dewar-01-Top",
    "serialNumber": "RTD-2025-0341",
    "calibrationDate": "2025-01-15",
    "calibrationDue": "2026-01-15"
  },
  "reading": {
    "temperature": -195.8,
    "unit": "CELSIUS",
    "uncertainty": 0.1,
    "qualityCode": "GOOD"
  },
  "metadata": {
    "acquisitionTime": 0.234,
    "sampleRate": "1Hz",
    "filterApplied": "MOVING_AVERAGE_5"
  }
}
```

### 4.2 Field Specifications

#### sensor Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | string | YES | Unique sensor identifier within facility |
| `type` | enum | YES | Sensor type: `PT100_RTD`, `PT1000_RTD`, `TYPE_K_TC`, `TYPE_T_TC`, `NTC_THERMISTOR` |
| `location` | string | YES | Physical location description |
| `serialNumber` | string | NO | Manufacturer serial number |
| `calibrationDate` | string (ISO 8601 date) | YES | Last calibration date |
| `calibrationDue` | string (ISO 8601 date) | YES | Next calibration due date |

#### reading Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `temperature` | number | YES | Temperature value |
| `unit` | enum | YES | `CELSIUS`, `FAHRENHEIT`, `KELVIN` |
| `uncertainty` | number | NO | Measurement uncertainty (±) |
| `qualityCode` | enum | YES | `GOOD`, `BAD`, `UNCERTAIN`, `MAINTENANCE` |

#### metadata Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `acquisitionTime` | number | NO | Time to acquire reading (seconds) |
| `sampleRate` | string | NO | Sampling frequency (e.g., "1Hz", "10Hz") |
| `filterApplied` | string | NO | Signal processing applied |

---

## 5. Alert Event Format

### 5.1 Structure

```json
{
  "messageType": "alertEvent",
  "version": "1.0.0",
  "timestamp": "2025-12-25T14:32:15.847Z",
  "facilityId": "WIA-CRYO-FAC-001",
  "alertId": "ALERT-2025-12-25-001",
  "severity": "CRITICAL",
  "alertType": "TEMPERATURE_EXCURSION",
  "status": "ACTIVE",
  "zone": "ZONE-B",
  "sensor": {
    "id": "RTD-B3",
    "type": "PT100_RTD",
    "location": "Dewar-03-Middle"
  },
  "condition": {
    "parameter": "temperature",
    "currentValue": -185.2,
    "threshold": -190.0,
    "unit": "CELSIUS",
    "deviation": 4.8
  },
  "message": "CRITICAL: Temperature excursion detected in Zone-B",
  "details": "Sensor RTD-B3 reading -185.2°C exceeds threshold of -190.0°C by 4.8°C",
  "recommendedAction": "Immediate investigation required. Check LN2 level and equipment status.",
  "escalationLevel": 0,
  "acknowledgmentRequired": true,
  "acknowledgedBy": null,
  "acknowledgedAt": null
}
```

### 5.2 Severity Levels

| Severity | Numeric | Response Time | Description |
|----------|---------|---------------|-------------|
| `CRITICAL` | 4 | < 1 minute | Immediate threat to samples |
| `HIGH` | 3 | < 5 minutes | Urgent attention required |
| `MEDIUM` | 2 | < 15 minutes | Warning condition |
| `LOW` | 1 | < 1 hour | Informational |
| `INFO` | 0 | As needed | Normal events |

### 5.3 Alert Types

- `TEMPERATURE_EXCURSION`: Temperature outside acceptable range
- `TEMPERATURE_RATE_CHANGE`: Rapid temperature change detected
- `LN2_LEVEL_LOW`: Liquid nitrogen inventory below threshold
- `LN2_CONSUMPTION_HIGH`: Abnormal LN2 consumption rate
- `SENSOR_FAULT`: Sensor malfunction or communication failure
- `SENSOR_DRIFT`: Sensor calibration drift detected
- `POWER_FAILURE`: Loss of grid power
- `SYSTEM_OFFLINE`: Monitoring system unavailable
- `CALIBRATION_DUE`: Sensor calibration expiring soon

---

## 6. LN2 Level Format

### 6.1 Structure

```json
{
  "messageType": "ln2Level",
  "version": "1.0.0",
  "timestamp": "2025-12-25T14:32:15.847Z",
  "facilityId": "WIA-CRYO-FAC-001",
  "tank": {
    "id": "TANK-03",
    "capacity": 1000,
    "unit": "LITERS"
  },
  "sensor": {
    "id": "LN2-CAP-03",
    "type": "CAPACITIVE",
    "serialNumber": "CAP-2025-0089",
    "calibrationDate": "2025-01-10"
  },
  "reading": {
    "levelPercent": 42.3,
    "volumeLiters": 423,
    "qualityCode": "GOOD"
  },
  "consumption": {
    "last24h": 3.2,
    "last7d": 22.8,
    "predictedDaysToRefill": 12
  },
  "autoFill": {
    "enabled": true,
    "lowThreshold": 40,
    "highThreshold": 85,
    "status": "STANDBY",
    "lastFillTimestamp": "2025-12-24T08:15:00Z",
    "lastFillVolume": 450
  }
}
```

---

## 7. Calibration Record Format

### 7.1 Structure

```json
{
  "messageType": "calibrationRecord",
  "version": "1.0.0",
  "timestamp": "2025-01-15T10:30:00Z",
  "facilityId": "WIA-CRYO-FAC-001",
  "sensor": {
    "id": "RTD-A1",
    "type": "PT100_RTD",
    "serialNumber": "RTD-2025-0341"
  },
  "calibration": {
    "date": "2025-01-15",
    "nextDue": "2026-01-15",
    "technician": "John Smith",
    "certificateNumber": "CAL-2025-0123",
    "accreditedLab": "NIST-Traceable Calibration Lab",
    "method": "ICE_BATH_AND_LN2",
    "points": [
      {
        "referenceTemp": 0.0,
        "sensorReading": 0.02,
        "error": 0.02
      },
      {
        "referenceTemp": -78.5,
        "sensorReading": -78.4,
        "error": 0.1
      },
      {
        "referenceTemp": -196.0,
        "sensorReading": -195.9,
        "error": 0.1
      }
    ],
    "result": "PASS",
    "correctionFactors": {
      "slope": 1.0001,
      "offset": -0.02
    }
  }
}
```

---

## 8. JSON Schema Definitions

JSON Schema files are provided for validation:

- `sensor-reading-schema.json`
- `alert-event-schema.json`
- `ln2-level-schema.json`
- `calibration-record-schema.json`

Example validation (Python):

```python
import json
import jsonschema

with open('sensor-reading-schema.json') as f:
    schema = json.load(f)

data = {
    "messageType": "sensorReading",
    # ... rest of data
}

jsonschema.validate(data, schema)  # Raises exception if invalid
```

---

## 9. Validation Rules

### 9.1 Mandatory Validations

1. **Timestamp**: Must be valid ISO 8601 format, timezone-aware (UTC preferred)
2. **Temperature Range**: -273.15°C (absolute zero) to +100°C (upper limit for cryogenic context)
3. **LN2 Level**: 0-100% (cannot exceed tank capacity)
4. **Sensor ID**: Must be unique within facility, alphanumeric with hyphens allowed
5. **Quality Code**: Must be one of defined enum values

### 9.2 Recommended Validations

1. **Future Timestamps**: Reject timestamps more than 1 minute in future (clock skew tolerance)
2. **Old Timestamps**: Warn if timestamp > 1 hour old (delayed data)
3. **Sensor Drift**: Flag if temperature deviates > 0.5°C from adjacent sensors
4. **Calibration Expiry**: Warn if `calibrationDue` date has passed

---

## 10. Examples

### 10.1 Complete Temperature Reading

```json
{
  "messageType": "sensorReading",
  "version": "1.0.0",
  "timestamp": "2025-12-25T14:32:15.847Z",
  "facilityId": "WIA-CRYO-FAC-001",
  "zoneId": "ZONE-A",
  "sensor": {
    "id": "RTD-A1",
    "type": "PT100_RTD",
    "location": "Dewar-01-Top",
    "serialNumber": "RTD-2025-0341",
    "calibrationDate": "2025-01-15",
    "calibrationDue": "2026-01-15"
  },
  "reading": {
    "temperature": -195.8,
    "unit": "CELSIUS",
    "uncertainty": 0.1,
    "qualityCode": "GOOD"
  },
  "metadata": {
    "acquisitionTime": 0.234,
    "sampleRate": "1Hz",
    "filterApplied": "MOVING_AVERAGE_5"
  }
}
```

### 10.2 Critical Alert Example

```json
{
  "messageType": "alertEvent",
  "version": "1.0.0",
  "timestamp": "2025-12-25T14:45:33.123Z",
  "facilityId": "WIA-CRYO-FAC-001",
  "alertId": "ALERT-2025-12-25-14-45-33-001",
  "severity": "CRITICAL",
  "alertType": "TEMPERATURE_EXCURSION",
  "status": "ACTIVE",
  "zone": "ZONE-B",
  "sensor": {
    "id": "RTD-B3",
    "type": "PT100_RTD",
    "location": "Dewar-03-Middle"
  },
  "condition": {
    "parameter": "temperature",
    "currentValue": -185.2,
    "threshold": -190.0,
    "unit": "CELSIUS",
    "deviation": 4.8
  },
  "message": "CRITICAL: Temperature excursion detected in Zone-B",
  "details": "Sensor RTD-B3 reading -185.2°C exceeds threshold of -190.0°C by 4.8°C. Immediate action required.",
  "recommendedAction": "Check LN2 level in Tank-03. Verify auto-fill system operational. Inspect for equipment failure.",
  "escalationLevel": 0,
  "acknowledgmentRequired": true,
  "acknowledgedBy": null,
  "acknowledgedAt": null
}
```

---

## Appendix A: Sensor Type Codes

| Code | Description | Typical Range | Accuracy |
|------|-------------|---------------|----------|
| `PT100_RTD` | Platinum RTD, 100Ω @ 0°C | -200°C to +850°C | ±0.1°C |
| `PT1000_RTD` | Platinum RTD, 1000Ω @ 0°C | -200°C to +850°C | ±0.1°C |
| `TYPE_K_TC` | Type K Thermocouple | -270°C to +1372°C | ±0.5°C |
| `TYPE_T_TC` | Type T Thermocouple | -270°C to +400°C | ±0.3°C |
| `NTC_THERMISTOR` | NTC Thermistor | -80°C to +150°C | ±0.05°C |
| `CAPACITIVE_LN2` | Capacitive LN2 Level | 0-100% | ±2% FS |
| `ULTRASONIC_LN2` | Ultrasonic LN2 Level | 0-100% | ±3-5% FS |

---

## Appendix B: Quality Codes

| Code | Description | Usage |
|------|-------------|-------|
| `GOOD` | Measurement is reliable and within specifications | Normal operation |
| `BAD` | Measurement is unreliable or sensor has failed | Sensor fault, out-of-range |
| `UNCERTAIN` | Measurement may be questionable | Sensor drift suspected, noisy signal |
| `MAINTENANCE` | Sensor under maintenance, reading invalid | During calibration, repairs |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

© 2025 WIA (World Certification Industry Association)
License: MIT

弘益人間 · Benefit All Humanity
