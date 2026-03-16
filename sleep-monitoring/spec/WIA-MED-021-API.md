# WIA-MED-021 API Specification
## Sleep Data Exchange Format

### Data Types

#### 1. Sleep Session

```json
{
  "session_id": "uuid-v4",
  "user_id": "uuid-v4",
  "start_time": "2025-01-15T22:00:00Z",
  "end_time": "2025-01-16T06:30:00Z",
  "timezone": "Asia/Seoul",
  "device": {
    "manufacturer": "Example Corp",
    "model": "SleepTracker Pro",
    "firmware_version": "2.1.0",
    "wia_certification": "WIA-MED-021-Class-II"
  },
  "metrics": {
    "total_sleep_time_minutes": 420,
    "sleep_efficiency_percent": 88.4,
    "sleep_onset_latency_minutes": 12,
    "wake_after_sleep_onset_minutes": 35,
    "num_awakenings": 4,
    "time_in_bed_minutes": 510
  },
  "stages": {
    "wake_minutes": 55,
    "light_minutes": 240,
    "deep_minutes": 90,
    "rem_minutes": 90
  },
  "epochs": [
    {
      "timestamp": "2025-01-15T22:00:00Z",
      "stage": "wake",
      "confidence": 0.95
    }
  ]
}
```

#### 2. Respiratory Events (Sleep Apnea)

```json
{
  "session_id": "uuid-v4",
  "events": [
    {
      "start_time": "2025-01-16T02:15:30Z",
      "duration_seconds": 15,
      "type": "obstructive_apnea",
      "spo2_drop_percent": 4,
      "arousal": true
    }
  ],
  "metrics": {
    "ahi": 12.5,
    "odi": 10.2,
    "min_spo2_percent": 88,
    "t90_percent": 2.1
  }
}
```

#### 3. Heart Rate Variability

```json
{
  "session_id": "uuid-v4",
  "metrics": {
    "rmssd_ms": 45.2,
    "sdnn_ms": 52.1,
    "hf_power_ms2": 1250,
    "lf_power_ms2": 850,
    "lf_hf_ratio": 0.68
  },
  "time_series": [
    {
      "timestamp": "2025-01-15T22:00:00Z",
      "rmssd_ms": 42.1
    }
  ]
}
```

### RESTful API Endpoints

#### Authentication
```
POST /api/v1/auth/login
POST /api/v1/auth/refresh
```

#### Sleep Sessions
```
GET    /api/v1/sleep/sessions
POST   /api/v1/sleep/sessions
GET    /api/v1/sleep/sessions/{id}
PUT    /api/v1/sleep/sessions/{id}
DELETE /api/v1/sleep/sessions/{id}
```

#### Export
```
GET /api/v1/sleep/sessions/{id}/export?format=edf
GET /api/v1/sleep/sessions/{id}/export?format=csv
```

### Data Quality Indicators

```json
{
  "signal_quality": {
    "accelerometer": {
      "valid_epochs_percent": 98.5,
      "artifacts_detected": 3
    },
    "ppg": {
      "valid_epochs_percent": 95.2,
      "motion_artifacts": 12
    }
  }
}
```

---

© 2025 WIA · MIT License
