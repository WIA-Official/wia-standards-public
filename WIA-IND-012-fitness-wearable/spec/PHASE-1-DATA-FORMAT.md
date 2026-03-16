# WIA-IND-012 PHASE 1: DATA FORMAT SPECIFICATION
## Fitness Wearable Standard - 弘益人間 (Benefit All Humanity)

**Version:** 1.0
**Status:** Active
**Category:** IND (Industrial)
**Last Updated:** 2025-01-15

---

## 1. Introduction

Phase 1 of the WIA-IND-012 standard establishes comprehensive data format specifications for fitness wearable devices. The principle of **弘益人間** (Benefit All Humanity) guides this specification, ensuring that health data is accessible, interoperable, and beneficial to all users regardless of device manufacturer or geographic location.

### 1.1 Scope

This specification covers:
- Sensor data schemas for all biometric measurements
- Activity log formats and structures
- Health metrics standardization
- Data serialization and encoding
- Metadata requirements
- Versioning and compatibility

### 1.2 Design Principles

- **Interoperability:** Data must be portable across devices and platforms
- **Privacy:** Formats support granular access control and encryption
- **Extensibility:** Schema can evolve without breaking compatibility
- **Human-readability:** JSON-based formats for transparency
- **Efficiency:** Optimized for storage and transmission

---

## 2. Base Data Schema

All fitness wearable data follows a common base schema:

```json
{
  "standard": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "string",
  "timestamp": "ISO8601",
  "deviceId": "string",
  "userId": "string (hashed)",
  "measurement": {},
  "metadata": {},
  "quality": {}
}
```

### 2.1 Required Fields

- `standard`: Always "WIA-IND-012"
- `version`: Schema version (semantic versioning)
- `philosophy`: "弘益人間" - affirming commitment to benefit humanity
- `dataType`: Type identifier (heart_rate, steps, sleep, etc.)
- `timestamp`: ISO 8601 format with timezone
- `deviceId`: Unique device identifier
- `userId`: Privacy-preserving user identifier (SHA-256 hash)

### 2.2 Optional Fields

- `measurement`: Type-specific measurement data
- `metadata`: Device and environmental context
- `quality`: Data quality indicators

---

## 3. Heart Rate Data Format

### 3.1 Real-Time Heart Rate

```json
{
  "standard": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "heart_rate",
  "timestamp": "2025-01-15T10:30:45.123Z",
  "deviceId": "WIA-DEVICE-001-ABC123",
  "userId": "a1b2c3d4e5f6...",
  "measurement": {
    "bpm": 75,
    "confidence": 0.95,
    "sensor": "PPG",
    "location": "wrist",
    "wavelength_nm": 525
  },
  "metadata": {
    "firmware_version": "2.1.0",
    "battery_level": 0.75,
    "activity_state": "walking"
  },
  "quality": {
    "signal_quality": 0.92,
    "motion_artifact_level": 0.15,
    "contact_quality": 0.98
  }
}
```

### 3.2 Heart Rate Variability (HRV)

```json
{
  "standard": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "hrv",
  "timestamp": "2025-01-15T07:00:00.000Z",
  "deviceId": "WIA-DEVICE-001-ABC123",
  "userId": "a1b2c3d4e5f6...",
  "measurement": {
    "sdnn_ms": 65.4,
    "rmssd_ms": 42.3,
    "pnn50_percent": 25.6,
    "lf_power": 450.2,
    "hf_power": 320.8,
    "lf_hf_ratio": 1.40,
    "rr_intervals_ms": [820, 850, 830, 840, 810, 825, 815, 835]
  },
  "metadata": {
    "measurement_duration_seconds": 300,
    "position": "supine",
    "breathing_pattern": "controlled"
  },
  "quality": {
    "artifact_percentage": 2.1,
    "valid_beats": 365,
    "total_beats": 373
  }
}
```

---

## 4. Activity Data Format

### 4.1 Step Count

```json
{
  "standard": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "steps",
  "timestamp": "2025-01-15T23:59:59.999Z",
  "deviceId": "WIA-DEVICE-001-ABC123",
  "userId": "a1b2c3d4e5f6...",
  "measurement": {
    "steps": 10247,
    "distance_km": 7.82,
    "calories": 520,
    "active_minutes": 145,
    "floors_climbed": 12
  },
  "metadata": {
    "algorithm_version": "3.2.1",
    "calibration_status": "personalized"
  },
  "quality": {
    "confidence": 0.94,
    "wear_time_minutes": 1342
  }
}
```

### 4.2 Workout Session

```json
{
  "standard": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "workout",
  "timestamp": "2025-01-15T18:30:00.000Z",
  "deviceId": "WIA-DEVICE-001-ABC123",
  "userId": "a1b2c3d4e5f6...",
  "measurement": {
    "activity_type": "running",
    "duration_minutes": 45,
    "distance_km": 8.5,
    "avg_heart_rate": 152,
    "max_heart_rate": 178,
    "calories": 580,
    "avg_pace_min_per_km": 5.29,
    "elevation_gain_m": 85,
    "training_effect": 4.2
  },
  "metadata": {
    "auto_detected": true,
    "gps_enabled": true,
    "weather_temp_c": 18,
    "weather_condition": "cloudy"
  },
  "quality": {
    "gps_accuracy_m": 4.2,
    "hr_data_completeness": 0.98
  },
  "segments": [
    {
      "segment_id": 1,
      "type": "warmup",
      "duration_minutes": 5,
      "avg_heart_rate": 125
    },
    {
      "segment_id": 2,
      "type": "interval_work",
      "duration_minutes": 30,
      "avg_heart_rate": 165
    },
    {
      "segment_id": 3,
      "type": "cooldown",
      "duration_minutes": 10,
      "avg_heart_rate": 135
    }
  ]
}
```

---

## 5. Sleep Data Format

```json
{
  "standard": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "sleep",
  "timestamp": "2025-01-15T07:30:00.000Z",
  "deviceId": "WIA-DEVICE-001-ABC123",
  "userId": "a1b2c3d4e5f6...",
  "measurement": {
    "sleep_start": "2025-01-14T23:15:00.000Z",
    "sleep_end": "2025-01-15T07:30:00.000Z",
    "total_sleep_minutes": 465,
    "sleep_efficiency": 0.88,
    "light_sleep_minutes": 230,
    "deep_sleep_minutes": 115,
    "rem_sleep_minutes": 120,
    "awake_minutes": 55,
    "awakenings_count": 3,
    "sleep_score": 85
  },
  "metadata": {
    "bedtime_consistency_score": 0.75,
    "room_temperature_c": 19
  },
  "quality": {
    "data_completeness": 0.97,
    "sensor_confidence": 0.91
  },
  "stages": [
    {"time": "2025-01-14T23:15:00Z", "stage": "light", "duration_min": 15},
    {"time": "2025-01-14T23:30:00Z", "stage": "deep", "duration_min": 45},
    {"time": "2025-01-15T00:15:00Z", "stage": "light", "duration_min": 30},
    {"time": "2025-01-15T00:45:00Z", "stage": "rem", "duration_min": 30}
  ]
}
```

---

## 6. GPS Location Data Format

```json
{
  "standard": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "gps_track",
  "timestamp": "2025-01-15T18:30:00.000Z",
  "deviceId": "WIA-DEVICE-001-ABC123",
  "userId": "a1b2c3d4e5f6...",
  "measurement": {
    "track_points": [
      {
        "latitude": 37.7749,
        "longitude": -122.4194,
        "altitude_m": 52.3,
        "timestamp": "2025-01-15T18:30:00.000Z",
        "accuracy_m": 4.5,
        "speed_mps": 3.2
      }
    ],
    "total_distance_km": 8.5,
    "elevation_gain_m": 85,
    "elevation_loss_m": 78
  },
  "metadata": {
    "satellites_used": ["GPS", "GLONASS", "Galileo"],
    "avg_hdop": 1.2,
    "fix_type": "3D"
  },
  "quality": {
    "position_accuracy_avg_m": 4.8,
    "data_completeness": 0.99
  }
}
```

---

## 7. Biometric Sensor Data Formats

### 7.1 Blood Oxygen (SpO2)

```json
{
  "standard": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "spo2",
  "timestamp": "2025-01-15T10:30:00.000Z",
  "measurement": {
    "spo2_percent": 97,
    "confidence": 0.92,
    "wavelengths_nm": [660, 940]
  }
}
```

### 7.2 Skin Temperature

```json
{
  "standard": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "temperature",
  "timestamp": "2025-01-15T10:30:00.000Z",
  "measurement": {
    "temperature_c": 33.2,
    "location": "wrist",
    "ambient_temp_c": 22.0
  }
}
```

### 7.3 Blood Pressure (Estimated)

```json
{
  "standard": "WIA-IND-012",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "blood_pressure",
  "timestamp": "2025-01-15T10:30:00.000Z",
  "measurement": {
    "systolic_mmhg": 120,
    "diastolic_mmhg": 80,
    "method": "pulse_transit_time",
    "confidence": 0.75
  },
  "metadata": {
    "calibration_date": "2025-01-10T00:00:00.000Z",
    "requires_medical_validation": true
  }
}
```

---

## 8. Data Quality Standards

### 8.1 Quality Metrics

All measurements must include quality indicators:

- **Signal Quality:** 0.0-1.0 scale indicating sensor signal clarity
- **Confidence:** Algorithm confidence in measurement accuracy
- **Contact Quality:** Sensor-skin contact quality
- **Motion Artifact:** Level of motion interference
- **Data Completeness:** Percentage of expected data points collected

### 8.2 Quality Thresholds

Minimum acceptable quality thresholds:
- Signal Quality: ≥ 0.70
- Confidence: ≥ 0.80
- Contact Quality: ≥ 0.75
- Data Completeness: ≥ 0.85

Measurements below thresholds should be flagged for user review.

---

## 9. Privacy and Security

### 9.1 Data Encryption

All data at rest must be encrypted using:
- AES-256 for stored data
- TLS 1.3 for data in transit
- End-to-end encryption for cloud synchronization

### 9.2 Privacy-Preserving Identifiers

- User IDs must be cryptographic hashes (SHA-256 minimum)
- Device IDs should not contain personally identifiable information
- Location data supports privacy zones (home/work obfuscation)

### 9.3 Data Minimization

Only collect and store data necessary for stated purposes. Support:
- Granular data retention policies
- User-configurable data deletion
- Right to data portability (export in standard formats)

---

## 10. Versioning and Compatibility

### 10.1 Semantic Versioning

Schema versions follow semantic versioning (MAJOR.MINOR.PATCH):
- MAJOR: Breaking changes incompatible with previous versions
- MINOR: Backward-compatible additions
- PATCH: Backward-compatible bug fixes

### 10.2 Forward Compatibility

Parsers must:
- Ignore unknown fields (forward compatibility)
- Validate required fields only
- Support multiple schema versions simultaneously

### 10.3 Deprecation Policy

- Deprecated fields marked 6 months before removal
- Migration guides provided for breaking changes
- Legacy support maintained for minimum 24 months

---

## 11. Implementation Requirements

### 11.1 Mandatory Data Types

All WIA-IND-012 compliant devices must support:
- Heart rate (real-time)
- Step count (daily aggregate)
- Activity sessions (workout tracking)
- Sleep tracking (nightly analysis)

### 11.2 Optional Data Types

Devices may optionally support:
- HRV measurements
- GPS tracking
- SpO2 monitoring
- Temperature sensing
- Blood pressure estimation

### 11.3 Data Export

Devices must provide:
- Standard format export (JSON)
- Batch export capabilities
- Automated export via API
- User-friendly export interface

---

## 12. Validation and Testing

### 12.1 Schema Validation

Implementations must validate against JSON Schema definitions provided in the WIA-IND-012 repository.

### 12.2 Test Data Sets

Reference test datasets available for:
- Unit testing implementations
- Integration testing
- Performance benchmarking
- Compatibility verification

### 12.3 Certification

Devices seeking WIA-IND-012 certification must:
- Pass schema validation tests
- Demonstrate data quality standards compliance
- Verify privacy and security implementations
- Provide interoperability evidence

---

## 13. Conclusion

Phase 1 of WIA-IND-012 establishes the data format foundation enabling interoperable, privacy-respecting, high-quality fitness wearable ecosystems. By adhering to these specifications, manufacturers contribute to the **弘益人間** vision of technology benefiting all humanity through universal health data accessibility and portability.

---

**Document Control**

- Version: 1.0
- Effective Date: 2025-01-15
- Next Review: 2026-01-15
- Contact: standards@wia.org

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
