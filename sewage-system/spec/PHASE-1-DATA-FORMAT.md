# WIA-SOC-009 Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines standardized data formats for smart sewage systems, including sensor readings, water quality parameters, flow metrics, and system status. All data MUST use JSON-LD format for semantic interoperability.

## 2. Core Data Types

### 2.1 System Identity

```json
{
  "@context": "https://wiastandards.com/soc-009/v1",
  "@type": "SewageSystem",
  "systemId": "SYS-2025-XXXX-YYYY",
  "municipality": "string",
  "operator": "string",
  "location": {
    "latitude": "float",
    "longitude": "float",
    "elevation": "float (meters)"
  },
  "capacity": "float (m³/day)",
  "servingPopulation": "integer",
  "installDate": "ISO8601 date",
  "capabilities": ["array", "of", "capability", "strings"]
}
```

### 2.2 System State

```json
{
  "@type": "SystemState",
  "timestamp": "ISO8601 datetime",
  "flowRate": {
    "value": "float (m³/s)",
    "location": "string",
    "direction": "inflow|outflow|bypass"
  },
  "waterQuality": {
    "pH": 0-14,
    "dissolvedOxygen": "float (mg/L)",
    "temperature": "float (°C)",
    "turbidity": "float (NTU)",
    "conductivity": "float (µS/cm)",
    "BOD": "float (mg/L)",
    "COD": "float (mg/L)",
    "TSS": "float (mg/L)",
    "ammoniaNitrogen": "float (mg/L)",
    "totalPhosphorus": "float (mg/L)"
  },
  "treatmentStatus": {
    "stage": "primary|secondary|tertiary",
    "efficiency": 0-100,
    "chemicalUsage": "float (kg/day)"
  },
  "alerts": ["array", "of", "AlertEvent"]
}
```

### 2.3 Sensor Data Format

```json
{
  "@type": "SensorReading",
  "sensorId": "UUID",
  "sensorType": "flow|quality|level|pressure|gas",
  "timestamp": "ISO8601 datetime",
  "location": {
    "latitude": "float",
    "longitude": "float",
    "zone": "string"
  },
  "reading": {
    "value": "float",
    "unit": "string",
    "quality": "good|fair|poor|invalid",
    "confidence": 0-1
  },
  "calibration": {
    "lastCalibrated": "ISO8601 datetime",
    "nextDue": "ISO8601 datetime"
  }
}
```

### 2.4 Water Quality Report

```json
{
  "@type": "WaterQualityReport",
  "reportId": "UUID",
  "timestamp": "ISO8601 datetime",
  "location": "string",
  "sampleType": "grab|composite|continuous",
  "parameters": {
    "physical": {
      "temperature": "float (°C)",
      "turbidity": "float (NTU)",
      "color": "float (Pt-Co units)",
      "odor": "string"
    },
    "chemical": {
      "pH": "float",
      "DO": "float (mg/L)",
      "BOD5": "float (mg/L)",
      "COD": "float (mg/L)",
      "TSS": "float (mg/L)",
      "TDS": "float (mg/L)",
      "ammonia": "float (mg/L)",
      "nitrate": "float (mg/L)",
      "phosphate": "float (mg/L)",
      "chloride": "float (mg/L)",
      "sulfate": "float (mg/L)"
    },
    "biological": {
      "fecalColiform": "float (CFU/100ml)",
      "EColi": "float (CFU/100ml)",
      "totalColiform": "float (CFU/100ml)"
    },
    "heavyMetals": {
      "lead": "float (µg/L)",
      "mercury": "float (µg/L)",
      "cadmium": "float (µg/L)",
      "chromium": "float (µg/L)",
      "arsenic": "float (µg/L)"
    }
  },
  "compliance": {
    "regulatoryLimit": "object",
    "status": "compliant|warning|violation"
  }
}
```

### 2.5 Flow Event Log

```json
{
  "@type": "FlowEvent",
  "eventId": "UUID",
  "timestamp": "ISO8601 datetime",
  "eventType": "overflow|bypass|surge|blockage|leak",
  "location": {
    "latitude": "float",
    "longitude": "float",
    "description": "string"
  },
  "severity": "low|medium|high|critical",
  "flowData": {
    "peakFlow": "float (m³/s)",
    "duration": "integer (seconds)",
    "volume": "float (m³)",
    "estimatedContaminantLoad": "float (kg)"
  },
  "response": {
    "alertSent": "ISO8601 datetime",
    "responseTime": "integer (minutes)",
    "actionTaken": "string",
    "resolvedAt": "ISO8601 datetime"
  },
  "environmentalImpact": {
    "receivingWater": "string",
    "estimatedImpact": "minimal|moderate|severe"
  }
}
```

## 3. Equipment Data Formats

### 3.1 Pump Status

```json
{
  "@type": "PumpStatus",
  "pumpId": "UUID",
  "location": "string",
  "status": "running|stopped|maintenance|fault",
  "flowRate": "float (m³/s)",
  "power": "float (kW)",
  "current": "float (A)",
  "voltage": "float (V)",
  "vibration": "float (mm/s)",
  "temperature": "float (°C)",
  "runtime": "integer (hours)",
  "cycleCount": "integer",
  "maintenance": {
    "lastService": "ISO8601 datetime",
    "nextDue": "ISO8601 datetime",
    "predictedFailure": "ISO8601 datetime (optional)"
  }
}
```

### 3.2 Treatment Process Data

```json
{
  "@type": "TreatmentProcess",
  "processId": "UUID",
  "stage": "screening|grit_removal|primary|secondary|tertiary|disinfection",
  "timestamp": "ISO8601 datetime",
  "influent": {
    "flow": "float (m³/s)",
    "quality": "WaterQualityReport"
  },
  "effluent": {
    "flow": "float (m³/s)",
    "quality": "WaterQualityReport"
  },
  "efficiency": {
    "BODremoval": "float (%)",
    "SSSremoval": "float (%)",
    "pathogenReduction": "float (log reduction)"
  },
  "chemicals": [
    {
      "type": "coagulant|flocculant|disinfectant|pH_adjuster",
      "name": "string",
      "dosage": "float (mg/L)",
      "costPerUnit": "float ($/kg)"
    }
  ],
  "energy": {
    "consumption": "float (kWh)",
    "cost": "float ($)"
  }
}
```

## 4. Alert and Event Formats

```json
{
  "@type": "SystemAlert",
  "alertId": "UUID",
  "timestamp": "ISO8601 datetime",
  "severity": "info|warning|error|critical",
  "category": "water_quality|flow|equipment|environmental|safety",
  "message": "string",
  "location": "string",
  "parameters": {
    "threshold": "float",
    "actual": "float",
    "deviation": "float (%)"
  },
  "actionRequired": "boolean",
  "acknowledgedBy": "string (optional)",
  "acknowledgedAt": "ISO8601 datetime (optional)",
  "resolvedAt": "ISO8601 datetime (optional)"
}
```

## 5. Environmental Discharge Format

```json
{
  "@type": "DischargeEvent",
  "dischargeId": "UUID",
  "timestamp": "ISO8601 datetime",
  "location": {
    "outfall": "string",
    "receivingWater": "string",
    "latitude": "float",
    "longitude": "float"
  },
  "discharge": {
    "volume": "float (m³)",
    "duration": "integer (minutes)",
    "flowRate": "float (m³/s)"
  },
  "quality": "WaterQualityReport",
  "permit": {
    "permitNumber": "string",
    "limits": "object",
    "compliance": "boolean"
  },
  "conditions": {
    "weatherEvent": "dry|rain|storm",
    "upstreamFlow": "float (m³/s)",
    "dilutionFactor": "float"
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
7. Quality parameters MUST include measurement uncertainty when available
8. Location data MUST use WGS84 coordinate system

## 7. Data Quality Indicators

```json
{
  "@type": "DataQuality",
  "accuracy": "float (±units)",
  "precision": "float",
  "completeness": "float (%)",
  "timeliness": "float (seconds delay)",
  "validity": "boolean",
  "qualityFlags": ["calibration_due", "sensor_drift", "anomaly_detected"]
}
```

## 8. Extensibility

Implementations MAY add custom fields prefixed with "x_" to avoid conflicts with future standard additions.

Example:
```json
{
  "@type": "SystemState",
  "flowRate": 4.2,
  "x_customParameter": "vendor-specific data",
  "x_localRegulation": "municipality-specific field"
}
```

---

© 2025 WIA · MIT License
