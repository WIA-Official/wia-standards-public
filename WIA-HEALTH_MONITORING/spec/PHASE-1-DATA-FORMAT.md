# WIA-HEALTH_MONITORING: Phase 1 - Data Format Specification

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2026-01-12
**Authors**: WIA Technical Committee

---

## 1. Overview

This document defines the standardized data formats for health monitoring systems, including wearable devices, continuous glucose monitors, heart rate monitors, SpO2 sensors, and sleep trackers.

### 1.1 Scope

- Biometric sensor data formats
- Continuous and discrete health metrics
- Temporal data structures
- Device metadata and capabilities
- Data quality indicators

### 1.2 Design Principles

1. **Interoperability**: Compatible with EMR/EHR systems
2. **Precision**: Medical-grade accuracy requirements
3. **Extensibility**: Support for future sensors
4. **Privacy**: Built-in data protection
5. **Real-time**: Low-latency streaming support

---

## 2. Core Data Structures

### 2.1 Health Metric Base Format

All health metrics inherit from this base structure:

```json
{
  "metric_id": "uuid-v4",
  "metric_type": "HEART_RATE | BLOOD_GLUCOSE | SPO2 | BLOOD_PRESSURE | TEMPERATURE | STEPS | SLEEP | ECG | RESPIRATORY_RATE",
  "timestamp": "ISO8601 datetime with timezone",
  "device_id": "unique device identifier",
  "user_id": "anonymized user identifier",
  "value": "metric-specific data structure",
  "unit": "standard unit of measurement",
  "quality_score": 0.0-1.0,
  "metadata": {
    "device_model": "string",
    "firmware_version": "string",
    "sensor_position": "WRIST | CHEST | FINGER | ARM | EAR",
    "activity_context": "RESTING | ACTIVE | SLEEPING | EXERCISING",
    "confidence_interval": [lower, upper]
  }
}
```

### 2.2 Heart Rate Data Format

```json
{
  "metric_type": "HEART_RATE",
  "value": {
    "bpm": 72,
    "variability_ms": 45.3,
    "resting_hr": 65,
    "max_hr_24h": 145,
    "min_hr_24h": 58,
    "irregular_rhythm_detected": false,
    "raw_intervals_ms": [850, 840, 855, 845]
  },
  "unit": "beats_per_minute",
  "quality_score": 0.98,
  "metadata": {
    "measurement_duration_sec": 60,
    "ppg_signal_quality": "EXCELLENT | GOOD | FAIR | POOR"
  }
}
```

### 2.3 Continuous Glucose Monitoring (CGM) Format

```json
{
  "metric_type": "BLOOD_GLUCOSE",
  "value": {
    "glucose_mg_dl": 105,
    "glucose_mmol_l": 5.83,
    "trend_direction": "RISING_RAPIDLY | RISING | STABLE | FALLING | FALLING_RAPIDLY",
    "trend_rate_mg_dl_min": 0.5,
    "time_in_range_percent": 78.5,
    "prediction_30min": {
      "predicted_value": 110,
      "confidence": 0.85
    }
  },
  "unit": "mg/dL",
  "quality_score": 0.95,
  "metadata": {
    "sensor_age_days": 7,
    "calibration_status": "CALIBRATED | NEEDS_CALIBRATION",
    "alert_thresholds": {
      "low": 70,
      "high": 180
    }
  }
}
```

### 2.4 Blood Oxygen (SpO2) Format

```json
{
  "metric_type": "SPO2",
  "value": {
    "oxygen_saturation_percent": 98,
    "perfusion_index": 4.5,
    "pulse_rate": 72,
    "pleth_variability_index": 12.5
  },
  "unit": "percent",
  "quality_score": 0.97,
  "metadata": {
    "measurement_method": "CONTINUOUS | SPOT_CHECK",
    "ambient_light_level": "LOW | MODERATE | HIGH",
    "motion_artifact_detected": false
  }
}
```

### 2.5 Blood Pressure Format

```json
{
  "metric_type": "BLOOD_PRESSURE",
  "value": {
    "systolic_mmhg": 120,
    "diastolic_mmhg": 80,
    "mean_arterial_pressure": 93,
    "pulse_pressure": 40,
    "measurement_position": "SITTING | STANDING | LYING"
  },
  "unit": "mmHg",
  "quality_score": 0.92,
  "metadata": {
    "cuff_size": "STANDARD | LARGE | SMALL",
    "measurement_location": "UPPER_ARM | WRIST",
    "repeated_measurement": 1
  }
}
```

### 2.6 Sleep Tracking Format

```json
{
  "metric_type": "SLEEP",
  "value": {
    "sleep_start": "ISO8601 datetime",
    "sleep_end": "ISO8601 datetime",
    "total_sleep_minutes": 456,
    "sleep_stages": [
      {
        "stage": "AWAKE | LIGHT | DEEP | REM",
        "start_time": "ISO8601 datetime",
        "duration_minutes": 85,
        "quality_score": 0.88
      }
    ],
    "sleep_efficiency_percent": 92.5,
    "wake_episodes": 2,
    "restlessness_score": 15,
    "sleep_score": 85
  },
  "unit": "minutes",
  "quality_score": 0.91,
  "metadata": {
    "detection_method": "ACCELEROMETER | HR_BASED | MULTI_SENSOR",
    "environmental_factors": {
      "temperature_celsius": 21.5,
      "noise_level_db": 35
    }
  }
}
```

### 2.7 Activity/Steps Format

```json
{
  "metric_type": "STEPS",
  "value": {
    "step_count": 8542,
    "distance_meters": 6834,
    "calories_burned": 342,
    "active_minutes": 67,
    "floors_climbed": 12,
    "intensity_distribution": {
      "sedentary_minutes": 720,
      "light_minutes": 180,
      "moderate_minutes": 45,
      "vigorous_minutes": 15
    }
  },
  "unit": "steps",
  "quality_score": 0.96,
  "metadata": {
    "stride_length_cm": 80,
    "activity_type": "WALKING | RUNNING | CYCLING | SWIMMING | OTHER"
  }
}
```

### 2.8 Body Temperature Format

```json
{
  "metric_type": "TEMPERATURE",
  "value": {
    "temperature_celsius": 36.8,
    "temperature_fahrenheit": 98.2,
    "measurement_site": "ORAL | TYMPANIC | TEMPORAL | AXILLARY | SKIN"
  },
  "unit": "celsius",
  "quality_score": 0.94,
  "metadata": {
    "baseline_temperature": 36.7,
    "trend_7day_avg": 36.75
  }
}
```

### 2.9 ECG/EKG Format

```json
{
  "metric_type": "ECG",
  "value": {
    "heart_rate": 72,
    "rhythm_classification": "SINUS_RHYTHM | ATRIAL_FIBRILLATION | INCONCLUSIVE",
    "waveform_data": {
      "sampling_rate_hz": 512,
      "lead_configuration": "SINGLE_LEAD | 12_LEAD",
      "samples": [/* array of voltage values */],
      "duration_seconds": 30
    },
    "qrs_duration_ms": 95,
    "qt_interval_ms": 380,
    "pr_interval_ms": 160
  },
  "unit": "millivolts",
  "quality_score": 0.89,
  "metadata": {
    "analysis_algorithm_version": "v2.3.1",
    "medical_review_required": false
  }
}
```

### 2.10 Respiratory Rate Format

```json
{
  "metric_type": "RESPIRATORY_RATE",
  "value": {
    "breaths_per_minute": 16,
    "breathing_pattern": "REGULAR | IRREGULAR",
    "breath_intervals_ms": [3750, 3800, 3725]
  },
  "unit": "breaths_per_minute",
  "quality_score": 0.86,
  "metadata": {
    "detection_method": "CHEST_MOVEMENT | NASAL_FLOW | IMPEDANCE"
  }
}
```

---

## 3. Composite Data Structures

### 3.1 Health Summary

```json
{
  "summary_id": "uuid-v4",
  "user_id": "anonymized user identifier",
  "period_start": "ISO8601 datetime",
  "period_end": "ISO8601 datetime",
  "summary_type": "DAILY | WEEKLY | MONTHLY",
  "metrics": {
    "heart_rate": {
      "avg": 72,
      "min": 58,
      "max": 145,
      "resting": 65
    },
    "sleep": {
      "avg_duration_minutes": 450,
      "avg_efficiency": 91.2,
      "total_deep_sleep_minutes": 680
    },
    "activity": {
      "total_steps": 68420,
      "avg_daily_steps": 9774,
      "total_distance_km": 54.7
    },
    "glucose": {
      "avg_mg_dl": 108,
      "time_in_range_percent": 82.5,
      "variability_coefficient": 18.5
    }
  },
  "alerts": [
    {
      "alert_type": "HIGH_HEART_RATE | LOW_GLUCOSE | IRREGULAR_RHYTHM | POOR_SLEEP",
      "timestamp": "ISO8601 datetime",
      "severity": "INFO | WARNING | CRITICAL",
      "message": "Human-readable alert message"
    }
  ]
}
```

### 3.2 Batch Upload Format

For efficient data transmission:

```json
{
  "batch_id": "uuid-v4",
  "device_id": "unique device identifier",
  "user_id": "anonymized user identifier",
  "upload_timestamp": "ISO8601 datetime",
  "data_points": [
    /* array of metric objects */
  ],
  "compression": "NONE | GZIP | BROTLI",
  "checksum": "SHA256 hash",
  "total_count": 1250
}
```

---

## 4. Data Quality Standards

### 4.1 Quality Score Calculation

Quality scores (0.0-1.0) are calculated based on:

| Factor | Weight | Description |
|--------|--------|-------------|
| Signal Strength | 30% | Sensor signal quality |
| Motion Artifact | 25% | Movement interference level |
| Sensor Calibration | 20% | Calibration status and age |
| Environmental Factors | 15% | Ambient conditions |
| Device Battery | 10% | Power level (low battery affects accuracy) |

### 4.2 Minimum Quality Thresholds

| Metric Type | Minimum Quality Score | Critical Use |
|-------------|----------------------|--------------|
| Blood Glucose | 0.90 | Insulin dosing decisions |
| ECG | 0.85 | Arrhythmia detection |
| SpO2 | 0.85 | Respiratory monitoring |
| Blood Pressure | 0.80 | Hypertension management |
| Heart Rate | 0.75 | General fitness |
| Sleep | 0.70 | Sleep analysis |
| Steps | 0.65 | Activity tracking |

---

## 5. Timestamp and Timezone Handling

### 5.1 Timestamp Format

All timestamps MUST use ISO 8601 format with timezone:

```
2026-01-12T14:35:22.123+00:00
```

### 5.2 Timezone Requirements

- Store all data in UTC
- Include original device timezone in metadata
- Support timezone conversion for user display
- Handle daylight saving time transitions

---

## 6. Units and Conversions

### 6.1 Standard Units

| Metric | Primary Unit | Alternative Units |
|--------|-------------|-------------------|
| Heart Rate | beats per minute (bpm) | - |
| Blood Glucose | mg/dL | mmol/L |
| SpO2 | percent (%) | - |
| Blood Pressure | mmHg | - |
| Temperature | Celsius (°C) | Fahrenheit (°F) |
| Distance | meters (m) | kilometers, miles |
| Weight | kilograms (kg) | pounds |

### 6.2 Conversion Formulas

```javascript
// Glucose
mg_dl = mmol_l * 18.018
mmol_l = mg_dl / 18.018

// Temperature
celsius = (fahrenheit - 32) * 5/9
fahrenheit = celsius * 9/5 + 32

// Distance
miles = kilometers * 0.621371
kilometers = miles * 1.60934
```

---

## 7. Data Validation Rules

### 7.1 Range Validation

| Metric | Minimum | Maximum | Invalid Action |
|--------|---------|---------|----------------|
| Heart Rate | 30 bpm | 220 bpm | Flag for review |
| Blood Glucose | 20 mg/dL | 600 mg/dL | Flag for review |
| SpO2 | 70% | 100% | Flag for review |
| Systolic BP | 60 mmHg | 250 mmHg | Flag for review |
| Diastolic BP | 40 mmHg | 150 mmHg | Flag for review |
| Temperature | 32°C | 42°C | Flag for review |

### 7.2 Relationship Validation

- Diastolic BP must be less than Systolic BP
- Sleep end time must be after start time
- Pulse rate from SpO2 should correlate with heart rate (±10%)
- Activity calories should align with intensity and duration

---

## 8. Privacy and Security

### 8.1 Data Anonymization

- User IDs MUST be anonymized/pseudonymized
- No personally identifiable information (PII) in metric data
- Location data requires explicit consent
- Support for data deletion requests (GDPR compliance)

### 8.2 Data Encryption

- All data in transit: TLS 1.3 minimum
- Data at rest: AES-256 encryption
- End-to-end encryption for sensitive metrics (glucose, ECG)

---

## 9. Example: Complete Health Metric

```json
{
  "metric_id": "550e8400-e29b-41d4-a716-446655440000",
  "metric_type": "HEART_RATE",
  "timestamp": "2026-01-12T14:35:22.123+00:00",
  "device_id": "apple-watch-series-9-abc123",
  "user_id": "anon-user-7f8d9e0a",
  "value": {
    "bpm": 72,
    "variability_ms": 45.3,
    "resting_hr": 65,
    "max_hr_24h": 145,
    "min_hr_24h": 58,
    "irregular_rhythm_detected": false,
    "raw_intervals_ms": [850, 840, 855, 845, 860, 835]
  },
  "unit": "beats_per_minute",
  "quality_score": 0.98,
  "metadata": {
    "device_model": "Apple Watch Series 9",
    "firmware_version": "10.2.1",
    "sensor_position": "WRIST",
    "activity_context": "RESTING",
    "confidence_interval": [70, 74],
    "measurement_duration_sec": 60,
    "ppg_signal_quality": "EXCELLENT",
    "battery_level": 0.82,
    "device_timezone": "America/New_York"
  }
}
```

---

## 10. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-12 | Initial specification |

---

## 11. References

- ISO/IEEE 11073 - Health informatics—Personal health device communication
- HL7 FHIR Observation Resource
- Apple HealthKit Data Types
- Google Fit Data Types
- Bluetooth SIG Health Device Profile (HDP)

---

**弘益人間 (홍익인간)** - Benefit All Humanity

© 2026 WIA (World Certification Industry Association)
