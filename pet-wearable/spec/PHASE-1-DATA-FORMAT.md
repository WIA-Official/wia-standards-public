# WIA-PET-007 PHASE 1: DATA FORMAT SPECIFICATION

**Version:** 1.0.0  
**Date:** 2025-12-25  
**Status:** Active Standard

---

## 1. Overview

PHASE 1 defines the core data formats and structures for WIA-PET-007 Pet Wearable devices. All compliant devices must use these standardized JSON formats for data storage, transmission, and API communication.

### 1.1 Design Principles

- **Human-Readable:** JSON format for easy inspection and debugging
- **Self-Describing:** Clear field names and structure
- **Extensible:** Forward-compatible with future additions
- **Compact:** Optimized for bandwidth efficiency
- **Versioned:** Schema version tracking for compatibility

---

## 2. Base Data Structure

All WIA-PET-007 data objects share a common base structure:

```json
{
  "schemaVersion": "1.0.0",
  "standard": "WIA-PET-007",
  "deviceId": "PW-{SPECIES}-{SERIAL}",
  "timestamp": "ISO8601 UTC",
  "petProfile": { },
  "metadata": { }
}
```

### 2.1 Field Specifications

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| schemaVersion | string | Yes | Format: MAJOR.MINOR.PATCH |
| standard | string | Yes | Always "WIA-PET-007" |
| deviceId | string | Yes | Unique device identifier |
| timestamp | string | Yes | ISO 8601 UTC format |
| petProfile | object | Yes | Pet information (see §3) |
| metadata | object | Yes | Device metadata (see §4) |

---

## 3. Pet Profile Format

```json
{
  "petProfile": {
    "petId": "PET-UUID-12345",
    "name": "Max",
    "species": "dog",
    "breed": "Golden Retriever",
    "birthdate": "YYYY-MM-DD",
    "gender": "male|female|neutered_male|spayed_female",
    "weight": 30.5,
    "weightUnit": "kg|lbs",
    "microchipId": "985112345678901",
    "color": "Golden",
    "distinguishingMarks": "White patch on chest"
  }
}
```

### 3.1 Species Values

**Standard species codes:**
- `dog` - Canis familiaris
- `cat` - Felis catus
- `rabbit` - Oryctolagus cuniculus
- `ferret` - Mustela putorius furo
- `other` - Other companion animals

---

## 4. Health Data Format

```json
{
  "healthData": {
    "timestamp": "2025-12-25T10:30:00.000Z",
    "heartRate": {
      "value": 95,
      "unit": "bpm",
      "quality": 92,
      "qualityScale": "0-100",
      "context": "resting|light|moderate|vigorous"
    },
    "temperature": {
      "value": 38.5,
      "unit": "celsius|fahrenheit",
      "quality": 88,
      "qualityScale": "0-100",
      "measurementSite": "collar_contact|rectal|ear"
    },
    "respiratoryRate": {
      "value": 22,
      "unit": "breaths_per_minute",
      "quality": 78,
      "qualityScale": "0-100",
      "measurementDuration": 60
    },
    "heartRateVariability": {
      "sdnn": 52,
      "rmssd": 38,
      "pnn50": 15.5,
      "unit": "milliseconds"
    }
  }
}
```

### 4.1 Quality Score Definition

Quality scores (0-100) indicate data reliability:

- **90-100:** Excellent - Clinical-grade accuracy
- **70-89:** Good - Suitable for wellness tracking
- **50-69:** Fair - General trends only
- **0-49:** Poor - Should be discarded

---

## 5. Activity Data Format

```json
{
  "activityData": {
    "date": "YYYY-MM-DD",
    "summary": {
      "totalSteps": 8523,
      "distance": {
        "value": 6.2,
        "unit": "kilometers|miles"
      },
      "calories": {
        "value": 245,
        "unit": "kcal"
      },
      "activeMinutes": 87,
      "restMinutes": 1353,
      "goalAchievement": {
        "steps": 85,
        "activeMinutes": 72,
        "unit": "percent"
      }
    },
    "hourlyBreakdown": [
      {
        "hour": 0-23,
        "steps": 520,
        "distance": 0.4,
        "calories": 18,
        "dominantActivity": "resting|standing|walking|running|playing"
      }
    ],
    "activities": [
      {
        "startTime": "ISO8601",
        "endTime": "ISO8601",
        "type": "walking|running|playing|resting|eating|drinking",
        "duration": 900,
        "durationUnit": "seconds",
        "intensity": "low|moderate|high|vigorous",
        "steps": 1200,
        "distance": 0.8,
        "calories": 45
      }
    ]
  }
}
```

---

## 6. Location Data Format

```json
{
  "locationData": {
    "timestamp": "ISO8601",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitude": 35.0,
      "accuracy": 5.0,
      "altitudeAccuracy": 12.0,
      "unit": "meters"
    },
    "source": "gps|wifi|cell|bluetooth|fusion",
    "satellites": 12,
    "hdop": 1.2,
    "speed": {
      "value": 4.5,
      "unit": "km_per_hour|mph",
      "heading": 234,
      "headingUnit": "degrees"
    },
    "address": {
      "formatted": "Seoul, South Korea",
      "street": "123 Main Street",
      "locality": "Seoul",
      "region": "Seoul",
      "postalCode": "12345",
      "country": "KR"
    },
    "geofenceStatus": {
      "insideZone": true,
      "zoneName": "Home Safe Zone",
      "zoneId": "FENCE-HOME-001"
    }
  }
}
```

---

## 7. Alert Data Format

```json
{
  "alert": {
    "alertId": "ALERT-UUID",
    "timestamp": "ISO8601",
    "severity": "critical|high|medium|low",
    "type": "health|geofence|battery|device|behavior",
    "category": {
      "health": ["elevated_heart_rate", "fever", "low_activity", "abnormal_breathing"],
      "geofence": ["zone_exit", "zone_entry", "escape"],
      "battery": ["low_battery", "critical_battery", "charging"],
      "device": ["connection_lost", "sensor_error", "firmware_update"],
      "behavior": ["excessive_scratching", "prolonged_rest", "unusual_activity"]
    },
    "metric": {
      "name": "heartRate",
      "value": 180,
      "unit": "bpm",
      "normalRange": "60-120"
    },
    "duration": 320,
    "durationUnit": "seconds",
    "context": {
      "activityLevel": "moderate",
      "ambientTemperature": 28,
      "recentActivity": "running"
    },
    "recommendation": "Monitor closely. Contact vet if sustained > 30 minutes.",
    "acknowledged": false,
    "acknowledgedAt": null,
    "resolvedAt": null
  }
}
```

---

## 8. Battery and Device Status Format

```json
{
  "deviceStatus": {
    "timestamp": "ISO8601",
    "battery": {
      "level": 87,
      "levelUnit": "percent",
      "voltage": 4.05,
      "voltageUnit": "volts",
      "charging": false,
      "timeToEmpty": 9.5,
      "timeToFull": null,
      "timeUnit": "days",
      "health": 95,
      "healthUnit": "percent",
      "cycles": 123,
      "temperature": 25,
      "temperatureUnit": "celsius"
    },
    "connectivity": {
      "bluetooth": {
        "connected": true,
        "signalStrength": -65,
        "signalUnit": "dBm"
      },
      "wifi": {
        "connected": false,
        "ssid": null,
        "signalStrength": null
      },
      "cellular": {
        "connected": true,
        "signalStrength": -85,
        "signalUnit": "dBm",
        "operator": "AT&T",
        "technology": "LTE"
      },
      "gps": {
        "available": true,
        "satellites": 12,
        "accuracy": 5.0
      }
    },
    "firmware": {
      "version": "2.3.1",
      "buildNumber": "20251215-abc123",
      "lastUpdate": "ISO8601"
    },
    "sensors": {
      "accelerometer": "operational|degraded|failed",
      "heartRateSensor": "operational|degraded|failed",
      "temperatureSensor": "operational|degraded|failed",
      "gpsModule": "operational|degraded|failed"
    }
  }
}
```

---

## 9. Data Type Specifications

### 9.1 Timestamp Format

**ISO 8601 UTC format required:**
```
YYYY-MM-DDTHH:MM:SS.sssZ
```

Examples:
- `2025-12-25T10:30:00.000Z`
- `2025-01-15T14:22:15.123Z`

### 9.2 Numeric Precision

| Data Type | Precision | Range |
|-----------|-----------|-------|
| Heart Rate | Integer | 0-300 bpm |
| Temperature | Float (1 decimal) | 35.0-42.0 °C |
| Respiratory Rate | Integer | 0-100 breaths/min |
| GPS Coordinates | Float (6 decimals) | Lat: -90 to 90, Lng: -180 to 180 |
| Distance | Float (2 decimals) | 0-999.99 km |
| Battery | Integer | 0-100% |

---

## 10. Error Handling

### 10.1 Missing Data Representation

Use `null` for missing optional fields:

```json
{
  "temperature": {
    "value": null,
    "unit": "celsius",
    "quality": 0,
    "measurementSite": "collar_contact"
  }
}
```

### 10.2 Invalid Data Flags

Use quality score of 0 for invalid measurements:

```json
{
  "heartRate": {
    "value": 95,
    "unit": "bpm",
    "quality": 0,
    "context": "sensor_error"
  }
}
```

---

## 11. Compression and Optimization

### 11.1 Field Omission

Optional fields may be omitted if not applicable:

```json
{
  "healthData": {
    "heartRate": { ... },
    "temperature": { ... }
    // respiratory rate omitted if not measured
  }
}
```

### 11.2 Batch Data Format

For bulk transfers, use array format:

```json
{
  "batchData": {
    "schemaVersion": "1.0.0",
    "deviceId": "PW-DOG-12345",
    "dataType": "health",
    "samples": [
      {"timestamp": "...", "heartRate": {...}},
      {"timestamp": "...", "heartRate": {...}},
      ...
    ]
  }
}
```

---

## 12. Validation Rules

### 12.1 Required Field Validation

All `required: true` fields must be present and non-null.

### 12.2 Range Validation

Numeric values must fall within specified ranges. Out-of-range values should be rejected.

### 12.3 Format Validation

- Timestamps must be valid ISO 8601
- Device IDs must match pattern: `^PW-[A-Z]+-[0-9A-Z]+$`
- Email addresses must be valid RFC 5322

---

## 13. Version Compatibility

### 13.1 Forward Compatibility

Parsers must ignore unknown fields:

```json
{
  "heartRate": {...},
  "newMetricInV2": {...}  // Ignored by v1.0 parsers
}
```

### 13.2 Backward Compatibility

New versions must not remove required fields from previous versions.

---

## Appendix A: Complete Example

```json
{
  "schemaVersion": "1.0.0",
  "standard": "WIA-PET-007",
  "deviceId": "PW-DOG-12345",
  "timestamp": "2025-12-25T10:30:00.000Z",
  "petProfile": {
    "petId": "PET-ABC-789",
    "name": "Max",
    "species": "dog",
    "breed": "Golden Retriever",
    "birthdate": "2020-03-15",
    "gender": "male",
    "weight": 30.5,
    "weightUnit": "kg"
  },
  "metadata": {
    "firmwareVersion": "2.3.1",
    "batteryLevel": 87,
    "signalStrength": -65
  },
  "healthData": {
    "timestamp": "2025-12-25T10:30:00.000Z",
    "heartRate": {
      "value": 95,
      "unit": "bpm",
      "quality": 92,
      "context": "resting"
    },
    "temperature": {
      "value": 38.5,
      "unit": "celsius",
      "quality": 88,
      "measurementSite": "collar_contact"
    }
  },
  "locationData": {
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "accuracy": 5.0
    },
    "source": "gps",
    "satellites": 12
  },
  "activityData": {
    "date": "2025-12-25",
    "summary": {
      "totalSteps": 8523,
      "distance": {"value": 6.2, "unit": "kilometers"},
      "calories": {"value": 245, "unit": "kcal"}
    }
  }
}
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA  
WIA-PET-007 PHASE 1 Specification
