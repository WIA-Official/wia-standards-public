# WIA-AGRI-005 PHASE 1: Data Format

## Overview
Phase 1 establishes the foundational data formats and structures for the WIA-AGRI-005 Soil Sensor Standard, including core sensor data models, measurement units, and basic data quality requirements.

## Core Data Model

### Sensor Profile
```json
{
  "sensorId": "SOIL-001",
  "name": "Field A North Sector",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 45,
    "depth": 15,
    "timezone": "Asia/Seoul"
  },
  "sensorType": "multi-parameter",
  "manufacturer": "AgriTech Solutions",
  "model": "NPK-pH-EC-2024",
  "installDate": "2025-01-15",
  "calibrationDate": "2025-01-15",
  "nextCalibration": "2025-07-15",
  "standard": "WIA-AGRI-005",
  "complianceLevel": 1
}
```

### Soil Sensor Data Format

#### Complete Reading
```json
{
  "sensorId": "SOIL-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "depth": 15
  },
  "npk": {
    "nitrogen": 85,
    "phosphorus": 35,
    "potassium": 150,
    "unit": "mg/kg"
  },
  "ph": {
    "value": 6.8,
    "unit": "pH",
    "method": "glass-electrode"
  },
  "ec": {
    "value": 1.2,
    "unit": "dS/m",
    "temperature": 25
  },
  "moisture": {
    "volumetric": 35,
    "gravimetric": 28,
    "unit": "%",
    "method": "capacitance"
  },
  "temperature": {
    "value": 22,
    "unit": "celsius",
    "depth": 15
  },
  "organicMatter": {
    "value": 2.8,
    "unit": "%",
    "method": "walkley-black"
  },
  "texture": {
    "sand": 45,
    "silt": 35,
    "clay": 20,
    "classification": "loam"
  },
  "battery": 87,
  "signalStrength": -75,
  "standard": "WIA-AGRI-005",
  "version": "1.0"
}
```

## Data Points Specification

### Required Measurements (Level 1)

#### NPK Nutrients
- **Nitrogen (N)**: 0-200 mg/kg
  - Accuracy: ±5 mg/kg or ±10% (whichever is greater)
  - Resolution: 1 mg/kg
  - Method: Nitrate + Ammonium extraction

- **Phosphorus (P)**: 0-100 mg/kg
  - Accuracy: ±3 mg/kg or ±10%
  - Resolution: 1 mg/kg
  - Method: Bray-1 or Olsen extraction

- **Potassium (K)**: 0-300 mg/kg
  - Accuracy: ±10 mg/kg or ±10%
  - Resolution: 1 mg/kg
  - Method: Ammonium acetate extraction

#### pH Measurement
- **Range**: 4.0-9.0 pH
- **Accuracy**: ±0.2 pH units
- **Resolution**: 0.1 pH units
- **Calibration**: Required at pH 4, 7, 10 buffer solutions

#### Electrical Conductivity (EC)
- **Range**: 0-4 dS/m
- **Accuracy**: ±5% or ±0.1 dS/m
- **Resolution**: 0.1 dS/m
- **Temperature Compensation**: Required (25°C reference)

#### Soil Moisture
- **Range**: 0-100% volumetric
- **Accuracy**: ±3% volumetric
- **Resolution**: 1%
- **Method**: Capacitance, TDR, or FDR

#### Soil Temperature
- **Range**: -10°C to +50°C
- **Accuracy**: ±0.5°C
- **Resolution**: 0.1°C
- **Depth**: Must be specified (cm)

### Optional Measurements (Level 2+)

#### Organic Matter
- **Range**: 0-10%
- **Method**: Walkley-Black or Loss-on-Ignition
- **Unit**: % by weight

#### Soil Texture
- **Sand**: 0-100% (2mm - 0.05mm)
- **Silt**: 0-100% (0.05mm - 0.002mm)
- **Clay**: 0-100% (<0.002mm)
- **Classification**: USDA Soil Texture Triangle

#### Micronutrients (Level 3)
- Iron (Fe), Manganese (Mn), Zinc (Zn), Copper (Cu)
- Boron (B), Molybdenum (Mo)
- Unit: mg/kg or ppm

## Measurement Intervals

### Level 1 Compliance
- **Continuous sensors**: 15-minute intervals minimum
- **Periodic sensors**: 1-hour intervals minimum
- **Laboratory analysis**: Weekly minimum for NPK

### Data Retention
- **Real-time data**: 30 days minimum
- **Hourly averages**: 1 year minimum
- **Daily summaries**: Sensor lifetime
- **Calibration records**: Sensor lifetime + 2 years

## Data Quality Requirements

### Sensor Validation
```json
{
  "dataQuality": {
    "status": "valid",
    "flags": [],
    "lastCalibration": "2025-01-15T10:00:00Z",
    "nextCalibration": "2025-07-15T10:00:00Z",
    "calibrationStatus": "current",
    "sensorHealth": "good",
    "batteryLevel": 87,
    "signalQuality": "good"
  }
}
```

### Quality Flags
- `CALIBRATION_DUE`: Calibration date approaching or past
- `OUT_OF_RANGE`: Reading outside expected range
- `BATTERY_LOW`: Battery below 20%
- `SIGNAL_WEAK`: Signal strength below -90 dBm
- `SENSOR_FAULT`: Hardware malfunction detected
- `DATA_SUSPICIOUS`: Reading differs significantly from historical pattern

### Outlier Detection
- Statistical: Readings beyond 3 standard deviations
- Physical: Values outside physically possible ranges
- Temporal: Sudden changes exceeding plausible rates

## Unit Standardization

### NPK Nutrients
- **Standard**: mg/kg (ppm)
- **Alternative**: lb/acre (conversion: mg/kg × 2.0 = lb/acre)

### pH
- **Standard**: pH units (0-14 scale)
- **No conversion**: Universal standard

### Electrical Conductivity
- **Standard**: dS/m (deci-Siemens per meter)
- **Alternative**: mS/cm (1 dS/m = 1 mS/cm)
- **Alternative**: μS/cm (1 dS/m = 1000 μS/cm)

### Moisture
- **Standard**: % volumetric (θv)
- **Alternative**: % gravimetric (θg)
- **Conversion**: θv = θg × bulk_density / water_density

### Temperature
- **Standard**: Celsius (°C)
- **Alternative**: Fahrenheit (°F = °C × 1.8 + 32)
- **Alternative**: Kelvin (K = °C + 273.15)

## Batch Data Format

For multiple sensors or locations:

```json
{
  "batchId": "BATCH-2025-12-26-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "fieldId": "FIELD-A1",
  "standard": "WIA-AGRI-005",
  "readings": [
    {
      "sensorId": "SOIL-001",
      "location": { "lat": 37.5665, "lon": 126.9780, "depth": 15 },
      "npk": { "n": 85, "p": 35, "k": 150 },
      "ph": 6.8,
      "ec": 1.2,
      "moisture": 35,
      "temperature": 22
    },
    {
      "sensorId": "SOIL-002",
      "location": { "lat": 37.5670, "lon": 126.9785, "depth": 15 },
      "npk": { "n": 92, "p": 38, "k": 145 },
      "ph": 7.0,
      "ec": 1.1,
      "moisture": 38,
      "temperature": 21
    }
  ]
}
```

## Implementation Checklist

- [ ] Define sensor profile with all required fields
- [ ] Configure NPK measurement with proper calibration
- [ ] Set up pH monitoring with buffer calibration
- [ ] Install EC sensor with temperature compensation
- [ ] Configure moisture sensor with appropriate method
- [ ] Implement temperature monitoring at specified depth
- [ ] Set up data quality validation rules
- [ ] Configure measurement intervals
- [ ] Implement data retention policies
- [ ] Test data format compliance
- [ ] Validate unit conversions
- [ ] Document calibration procedures

## Compliance Levels

### Level 1: Basic Monitoring
- NPK, pH, EC, Moisture, Temperature
- 15-minute intervals for continuous sensors
- Basic quality flags
- 30-day data retention

### Level 2: Enhanced Monitoring
- All Level 1 requirements
- Organic matter analysis
- Soil texture classification
- Advanced quality validation
- 1-year data retention

### Level 3: Precision Agriculture
- All Level 2 requirements
- Micronutrient analysis
- Real-time data streaming
- Predictive analytics
- Lifetime data retention
- Multi-depth profiling

## 弘益人間 (Benefit All Humanity)
Phase 1 ensures that soil sensor data is standardized, accurate, and accessible—enabling farmers worldwide to make informed decisions for healthier crops and sustainable agriculture.

---
© 2025 SmileStory Inc. / WIA
