# WIA-AGRI-027: Hydroponics Standard
## Phase 1: Data Format & Structure

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Emoji:** 💧

---

## 1. Overview

This document defines the standardized data formats and structures for the WIA-AGRI-027 Hydroponics Standard. All data exchanged between sensors, controllers, databases, and applications must conform to these specifications.

### 1.1 Philosophy

弘益人間 (홍익인간) - Benefit All Humanity

This standard aims to make hydroponic farming accessible, efficient, and sustainable for all stakeholders.

### 1.2 Scope

- Sensor data formats
- System configuration structures
- Nutrient solution specifications
- Alert and logging formats
- Historical data archival

---

## 2. Core Data Types

### 2.1 Sensor Reading Format

All sensor readings must follow this JSON structure:

```json
{
  "deviceId": "string",
  "deviceType": "ph_sensor|ec_sensor|do_sensor|temperature_sensor|level_sensor",
  "location": "string",
  "timestamp": "ISO8601",
  "value": "number",
  "unit": "string",
  "quality": "excellent|good|fair|poor",
  "calibration": {
    "lastCalibrated": "ISO8601",
    "nextCalibration": "ISO8601",
    "status": "valid|due|overdue"
  }
}
```

**Example:**

```json
{
  "deviceId": "SENSOR-PH-001",
  "deviceType": "ph_sensor",
  "location": "reservoir_1",
  "timestamp": "2025-12-26T10:30:00Z",
  "value": 6.2,
  "unit": "pH",
  "quality": "excellent",
  "calibration": {
    "lastCalibrated": "2025-12-20T08:00:00Z",
    "nextCalibration": "2026-01-20T08:00:00Z",
    "status": "valid"
  }
}
```

### 2.2 Nutrient Solution Format

```json
{
  "formulaId": "string",
  "formulaName": "string",
  "version": "string",
  "cropType": "string",
  "growthStage": "seedling|vegetative|flowering|fruiting",
  "targetPH": "number",
  "targetEC": "number",
  "macronutrients": {
    "N": "number (ppm)",
    "P": "number (ppm)",
    "K": "number (ppm)",
    "Ca": "number (ppm)",
    "Mg": "number (ppm)",
    "S": "number (ppm)"
  },
  "micronutrients": {
    "Fe": "number (ppm)",
    "Mn": "number (ppm)",
    "Zn": "number (ppm)",
    "Cu": "number (ppm)",
    "B": "number (ppm)",
    "Mo": "number (ppm)"
  },
  "additives": [
    {
      "name": "string",
      "concentration": "number",
      "unit": "string"
    }
  ]
}
```

### 2.3 System Configuration Format

```json
{
  "systemId": "string",
  "systemName": "string",
  "systemType": "NFT|DWC|EBB_FLOW|DRIP|AEROPONIC|DUTCH_BUCKET",
  "location": {
    "facility": "string",
    "zone": "string",
    "coordinates": {
      "latitude": "number",
      "longitude": "number"
    }
  },
  "reservoir": {
    "capacity": "number (liters)",
    "currentLevel": "number (liters)",
    "material": "string"
  },
  "sensors": [
    {
      "deviceId": "string",
      "deviceType": "string",
      "status": "active|inactive|maintenance"
    }
  ],
  "actuators": [
    {
      "deviceId": "string",
      "type": "pump|valve|doser|heater|cooler",
      "status": "active|inactive|maintenance"
    }
  ],
  "thresholds": {
    "ph": {
      "min": "number",
      "max": "number",
      "target": "number"
    },
    "ec": {
      "min": "number",
      "max": "number",
      "target": "number"
    },
    "temperature": {
      "min": "number",
      "max": "number",
      "target": "number"
    },
    "dissolvedOxygen": {
      "min": "number",
      "target": "number"
    }
  }
}
```

---

## 3. Alert & Event Format

### 3.1 Alert Structure

```json
{
  "alertId": "string (UUID)",
  "timestamp": "ISO8601",
  "severity": "critical|warning|info",
  "category": "sensor|nutrient|system|environmental",
  "source": {
    "systemId": "string",
    "deviceId": "string",
    "location": "string"
  },
  "parameter": "string",
  "currentValue": "number",
  "thresholdValue": "number",
  "message": "string",
  "recommendedAction": "string",
  "acknowledged": "boolean",
  "acknowledgedBy": "string",
  "acknowledgedAt": "ISO8601",
  "resolved": "boolean",
  "resolvedAt": "ISO8601"
}
```

**Example:**

```json
{
  "alertId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-26T14:15:00Z",
  "severity": "warning",
  "category": "nutrient",
  "source": {
    "systemId": "HYDRO-SYS-001",
    "deviceId": "SENSOR-PH-001",
    "location": "reservoir_1"
  },
  "parameter": "pH",
  "currentValue": 7.2,
  "thresholdValue": 6.5,
  "message": "pH level exceeds maximum threshold",
  "recommendedAction": "Add pH down solution to reduce pH to target range",
  "acknowledged": true,
  "acknowledgedBy": "operator@farm.com",
  "acknowledgedAt": "2025-12-26T14:16:00Z",
  "resolved": false,
  "resolvedAt": null
}
```

---

## 4. Historical Data Format

### 4.1 Time-Series Data

```json
{
  "systemId": "string",
  "parameter": "string",
  "unit": "string",
  "aggregation": "raw|hourly|daily|weekly",
  "startTime": "ISO8601",
  "endTime": "ISO8601",
  "dataPoints": [
    {
      "timestamp": "ISO8601",
      "value": "number",
      "quality": "string"
    }
  ],
  "statistics": {
    "min": "number",
    "max": "number",
    "average": "number",
    "median": "number",
    "stdDev": "number"
  }
}
```

### 4.2 Harvest Log Format

```json
{
  "harvestId": "string (UUID)",
  "systemId": "string",
  "cropType": "string",
  "variety": "string",
  "plantingDate": "ISO8601",
  "harvestDate": "ISO8601",
  "quantity": "number",
  "unit": "kg|units",
  "quality": "premium|grade_a|grade_b|grade_c",
  "batchNumber": "string",
  "operator": "string",
  "notes": "string",
  "nutrientHistory": {
    "formulaId": "string",
    "avgPH": "number",
    "avgEC": "number"
  },
  "environmentalConditions": {
    "avgTemperature": "number",
    "avgHumidity": "number",
    "totalLightHours": "number"
  }
}
```

---

## 5. Data Validation Rules

### 5.1 pH Validation

- **Range:** 4.0 - 8.0
- **Precision:** 0.1
- **Type:** Number (float)

### 5.2 EC Validation

- **Range:** 0.5 - 4.0 mS/cm
- **Precision:** 0.1
- **Type:** Number (float)

### 5.3 Temperature Validation

- **Range:** 10.0 - 30.0 °C
- **Precision:** 0.1
- **Type:** Number (float)

### 5.4 Dissolved Oxygen Validation

- **Range:** 5.0 - 15.0 mg/L
- **Precision:** 0.1
- **Type:** Number (float)

### 5.5 Timestamp Validation

- **Format:** ISO 8601 (YYYY-MM-DDTHH:MM:SSZ)
- **Timezone:** UTC
- **Example:** 2025-12-26T10:30:00Z

---

## 6. Data Storage Recommendations

### 6.1 Database Schema

**Sensor Readings Table:**

```sql
CREATE TABLE sensor_readings (
  id BIGSERIAL PRIMARY KEY,
  device_id VARCHAR(50) NOT NULL,
  device_type VARCHAR(50) NOT NULL,
  location VARCHAR(100),
  timestamp TIMESTAMPTZ NOT NULL,
  value DECIMAL(10,2) NOT NULL,
  unit VARCHAR(20),
  quality VARCHAR(20),
  created_at TIMESTAMPTZ DEFAULT NOW(),
  INDEX idx_device_timestamp (device_id, timestamp),
  INDEX idx_location_timestamp (location, timestamp)
);
```

**Nutrient Formulas Table:**

```sql
CREATE TABLE nutrient_formulas (
  formula_id VARCHAR(50) PRIMARY KEY,
  formula_name VARCHAR(100) NOT NULL,
  version VARCHAR(20),
  crop_type VARCHAR(50),
  growth_stage VARCHAR(50),
  target_ph DECIMAL(3,1),
  target_ec DECIMAL(3,1),
  nutrients JSONB,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW()
);
```

### 6.2 Data Retention Policies

- **Raw sensor data:** 90 days
- **Hourly aggregates:** 1 year
- **Daily aggregates:** 5 years
- **Alert logs:** 2 years
- **Harvest records:** Indefinite
- **System configurations:** Version controlled, indefinite

---

## 7. Data Exchange Formats

### 7.1 CSV Export Format

For bulk data export, use the following CSV structure:

```csv
timestamp,device_id,device_type,location,parameter,value,unit,quality
2025-12-26T10:30:00Z,SENSOR-PH-001,ph_sensor,reservoir_1,pH,6.2,pH,excellent
2025-12-26T10:30:00Z,SENSOR-EC-001,ec_sensor,reservoir_1,EC,2.1,mS/cm,excellent
```

### 7.2 XML Format (Legacy Support)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<SensorReading>
  <DeviceId>SENSOR-PH-001</DeviceId>
  <DeviceType>ph_sensor</DeviceType>
  <Location>reservoir_1</Location>
  <Timestamp>2025-12-26T10:30:00Z</Timestamp>
  <Value>6.2</Value>
  <Unit>pH</Unit>
  <Quality>excellent</Quality>
</SensorReading>
```

---

## 8. Compliance & Standards

### 8.1 Related Standards

- **ISO 8601:** Date and time format
- **IEEE 754:** Floating-point arithmetic
- **UTF-8:** Character encoding
- **JSON Schema:** Data validation

### 8.2 Version Control

This specification uses semantic versioning (MAJOR.MINOR.PATCH):

- **MAJOR:** Incompatible changes
- **MINOR:** Backward-compatible additions
- **PATCH:** Backward-compatible fixes

Current version: **1.0.0**

---

## 9. Conclusion

The WIA-AGRI-027 Phase 1 data format specification provides a robust foundation for interoperable hydroponic systems. Adherence to these standards ensures data consistency, reliability, and seamless integration across platforms.

---

**Document Information:**

- **Standard ID:** WIA-AGRI-027
- **Phase:** 1 - Data Format & Structure
- **Status:** Active
- **Maintained by:** WIA (World Certification Industry Association)
- **Contact:** standards@wia.org

弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
