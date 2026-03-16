# WIA-IND-002: Smart Textile Standard
## PHASE 1: DATA FORMAT SPECIFICATION
### 弘益人間 - Benefit All Humanity

---

## 1. Overview

This document defines the data format specifications for WIA-IND-002 Smart Textile Standard. All smart textile devices compliant with this standard MUST implement these data schemas to ensure interoperability across manufacturers, platforms, and applications.

**Version:** 1.0  
**Status:** Final  
**Last Updated:** 2025-12-27

## 2. Core Data Schema

### 2.1 Sensor Data Packet Format

All sensor data packets MUST conform to the following JSON schema:

```json
{
  "deviceId": "string (UUID v4)",
  "timestamp": "ISO 8601 datetime",
  "standard": "WIA-IND-002",
  "philosophy": "弘益人間",
  "version": "1.0",
  "sensors": {
    "temperature": { /* Temperature Data */ },
    "pressure": { /* Pressure Data */ },
    "moisture": { /* Moisture Data */ },
    "strain": { /* Strain Data */ },
    "biometric": { /* Biometric Data */ },
    "environmental": { /* Environmental Data */ }
  },
  "metadata": {
    "batteryLevel": "number (0-100)",
    "signalStrength": "number (dBm)",
    "firmwareVersion": "string (semver)",
    "calibrationDate": "ISO 8601 date"
  }
}
```

### 2.2 Temperature Sensor Data

Temperature sensors MUST report data in the following format:

```json
{
  "temperature": {
    "value": 36.5,
    "unit": "celsius",
    "accuracy": 0.1,
    "sensorType": "thermistor|thermocouple|fiber_optic",
    "location": "chest|arm|leg|torso",
    "calibrated": true,
    "calibrationDate": "2025-12-27T00:00:00Z"
  }
}
```

**Requirements:**
- Value range: -40°C to +85°C
- Accuracy: ±0.1°C for medical applications, ±0.5°C for fitness
- Sampling rate: Minimum 1 Hz, maximum 100 Hz
- Resolution: 0.01°C minimum

### 2.3 Pressure Sensor Data

Pressure sensors MUST report data in the following format:

```json
{
  "pressure": {
    "value": 120.5,
    "unit": "kPa",
    "sensorType": "piezoresistive|capacitive|piezoelectric",
    "location": "foot_left|foot_right|seat|joint",
    "area": 10.0,
    "areaUnit": "cm2",
    "distributionMap": [[/* 2D array of pressure values */]]
  }
}
```

**Requirements:**
- Range: 0-500 kPa for body applications
- Accuracy: ±2% of full scale
- Sampling rate: 10-200 Hz
- Spatial resolution: Minimum 1 cm² for pressure mapping

### 2.4 Moisture Sensor Data

Moisture sensors MUST report data in the following format:

```json
{
  "moisture": {
    "relativeHumidity": 65.5,
    "unit": "percent",
    "skinWetness": 0.45,
    "sensorType": "resistive|capacitive|optical",
    "location": "back|chest|underarm",
    "sweatRate": 0.5,
    "sweatRateUnit": "g/min"
  }
}
```

**Requirements:**
- Humidity range: 0-100% RH
- Accuracy: ±3% RH
- Response time: <5 seconds
- Sweat rate accuracy: ±10%

### 2.5 Strain Sensor Data

Strain sensors MUST report data in the following format:

```json
{
  "strain": {
    "value": 15.5,
    "unit": "percent",
    "sensorType": "resistive|capacitive|fiber_optic",
    "location": "knee_left|elbow_right|spine",
    "baselineLength": 10.0,
    "currentLength": 11.55,
    "lengthUnit": "cm",
    "velocity": 2.5,
    "velocityUnit": "cm/s"
  }
}
```

**Requirements:**
- Strain range: 0-100% elongation
- Accuracy: ±1% of measured value
- Sampling rate: 50-500 Hz for motion capture
- Hysteresis: <5%

### 2.6 Biometric Sensor Data

#### 2.6.1 ECG Data

```json
{
  "biometric": {
    "ecg": {
      "leadConfiguration": "Lead_I|Lead_II|Lead_III|12-lead",
      "samplingRate": 250,
      "samples": [/* array of voltage values in mV */],
      "heartRate": 72,
      "heartRateVariability": {
        "sdnn": 45.5,
        "rmssd": 38.2,
        "pnn50": 12.5
      },
      "quality": "excellent|good|fair|poor"
    }
  }
}
```

**Requirements:**
- Sampling rate: 250-1000 Hz
- Resolution: 24-bit ADC minimum
- Input range: ±5 mV
- Common-mode rejection: >80 dB

#### 2.6.2 Heart Rate Data

```json
{
  "biometric": {
    "heartRate": {
      "value": 72,
      "unit": "bpm",
      "method": "ecg|ppg|acoustic",
      "confidence": 0.95,
      "rrIntervals": [/* array of R-R intervals in ms */]
    }
  }
}
```

#### 2.6.3 Respiration Data

```json
{
  "biometric": {
    "respiration": {
      "rate": 16,
      "unit": "breaths_per_minute",
      "method": "impedance|strain|thermal",
      "tidalVolume": 500,
      "tidalVolumeUnit": "ml",
      "pattern": "normal|shallow|deep|irregular"
    }
  }
}
```

### 2.7 Environmental Sensor Data

```json
{
  "environmental": {
    "airQuality": {
      "pm25": 12.5,
      "pm10": 25.0,
      "no2": 15.0,
      "o3": 30.0,
      "co": 0.5,
      "voc": 100,
      "units": {
        "particulates": "µg/m³",
        "gases": "ppb",
        "voc": "ppb"
      },
      "aqi": 35,
      "category": "good|moderate|unhealthy_sensitive|unhealthy|very_unhealthy|hazardous"
    },
    "uv": {
      "index": 6.5,
      "uva": 2.5,
      "uvb": 1.2,
      "unit": "mW/cm²",
      "exposure": 150,
      "exposureUnit": "J/m²"
    },
    "temperature": 22.5,
    "humidity": 55.0,
    "pressure": 1013.25
  }
}
```

## 3. Data Quality Indicators

All sensor readings SHOULD include quality indicators:

```json
{
  "quality": {
    "signalToNoiseRatio": 35.5,
    "signalToNoiseUnit": "dB",
    "contactQuality": "excellent|good|fair|poor",
    "artifactLevel": "none|low|medium|high",
    "confidence": 0.95,
    "calibrationStatus": "valid|expired|required"
  }
}
```

## 4. Calibration Data Format

Calibration records MUST be stored in the following format:

```json
{
  "calibration": {
    "sensorType": "temperature|pressure|moisture|etc",
    "calibrationDate": "2025-12-27T10:00:00Z",
    "expiryDate": "2026-12-27T10:00:00Z",
    "calibrationMethod": "factory|user|auto",
    "referenceDevice": "string",
    "calibrationPoints": [
      {"reference": 20.0, "measured": 20.1, "correctionFactor": 0.995},
      {"reference": 37.0, "measured": 37.2, "correctionFactor": 0.995}
    ],
    "certificateId": "string (UUID)"
  }
}
```

## 5. Textile Properties Data

Smart textile material properties MUST be documented:

```json
{
  "textileProperties": {
    "fabricType": "cotton|polyester|nylon|blend",
    "threadCount": 200,
    "thickness": 0.5,
    "thicknessUnit": "mm",
    "conductivity": {
      "resistance": 10.0,
      "resistanceUnit": "ohm/cm",
      "type": "silver|copper|carbon"
    },
    "washability": {
      "maxWashCycles": 50,
      "maxTemperature": 40,
      "temperatureUnit": "celsius",
      "testStandard": "ISO 6330"
    },
    "flexibility": {
      "bendRadius": 5.0,
      "bendRadiusUnit": "mm",
      "stretchability": 30,
      "stretchabilityUnit": "percent"
    },
    "breathability": {
      "airPermeability": 150,
      "airPermeabilityUnit": "cm³/cm²/s",
      "moistureTransport": "good|fair|poor"
    }
  }
}
```

## 6. Measurement Standards

### 6.1 Accuracy Requirements

| Sensor Type | Medical Grade | Fitness Grade | Environmental |
|-------------|---------------|---------------|---------------|
| Temperature | ±0.1°C | ±0.5°C | ±1.0°C |
| Heart Rate | ±1 bpm | ±5 bpm | N/A |
| Blood Oxygen | ±2% | ±3% | N/A |
| Pressure | ±2% FS | ±5% FS | ±10% FS |
| Motion | ±1% | ±3% | ±5% |

### 6.2 Sampling Rate Requirements

| Application | Minimum Rate | Recommended Rate | Maximum Rate |
|-------------|--------------|------------------|--------------|
| ECG | 250 Hz | 500 Hz | 1000 Hz |
| PPG | 50 Hz | 100 Hz | 200 Hz |
| Motion | 50 Hz | 100 Hz | 500 Hz |
| Temperature | 1 Hz | 10 Hz | 100 Hz |
| Environment | 0.1 Hz | 1 Hz | 10 Hz |

## 7. Data Transmission Format

### 7.1 Real-Time Streaming

For real-time applications, data SHOULD be transmitted using compact binary formats or compressed JSON:

```json
{
  "stream": {
    "sessionId": "UUID",
    "sequenceNumber": 12345,
    "compression": "gzip|lz4|none",
    "encoding": "json|msgpack|protobuf",
    "data": "base64 encoded payload"
  }
}
```

### 7.2 Batch Upload

For periodic uploads, data MAY be batched:

```json
{
  "batch": {
    "uploadId": "UUID",
    "deviceId": "UUID",
    "startTime": "ISO 8601",
    "endTime": "ISO 8601",
    "recordCount": 1000,
    "records": [/* array of sensor data packets */]
  }
}
```

## 8. Error Handling

Error conditions MUST be reported using standard error codes:

```json
{
  "error": {
    "code": "SENSOR_DISCONNECTED|CALIBRATION_EXPIRED|BATTERY_LOW|etc",
    "severity": "critical|warning|info",
    "timestamp": "ISO 8601",
    "message": "Human-readable description",
    "sensorAffected": "temperature|pressure|etc",
    "recommendedAction": "recalibrate|recharge|contact_support"
  }
}
```

## 9. Data Privacy and Security

### 9.1 Personal Identifiable Information

PII MUST be encrypted and separated from sensor data:

```json
{
  "user": {
    "userId": "hashed UUID",
    "encrypted": true,
    "encryptionMethod": "AES-256-GCM",
    "piiFields": ["name", "email", "phone"],
    "anonymized": true
  }
}
```

### 9.2 Data Retention

Metadata for data retention policies:

```json
{
  "retention": {
    "category": "medical|fitness|research",
    "retentionPeriod": 2555,
    "retentionUnit": "days",
    "deleteAfter": "2032-12-27T00:00:00Z",
    "complianceStandards": ["HIPAA", "GDPR", "CCPA"]
  }
}
```

## 10. Compliance and Validation

All data formats MUST be validated against JSON Schema definitions available at:
- https://wia.org/standards/IND-002/schemas/v1.0/

Validation tools and reference implementations:
- GitHub: https://github.com/WIA-Official/wia-standards
- NPM: `npm install @wia/ind-002-validator`
- Python: `pip install wia-ind-002`

---

**Philosophy:** 弘益人間 - This standard ensures that smart textile data is accessible, interoperable, and beneficial to all humanity by establishing open, well-documented formats that enable innovation while protecting privacy and ensuring quality.

**License:** This specification is released under Creative Commons BY-SA 4.0

**Contact:** standards@wia.org
