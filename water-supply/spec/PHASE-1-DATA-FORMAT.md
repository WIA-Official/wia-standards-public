# WIA-SOC-008 Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines standardized data formats for water supply systems, including water quality metrics, flow data, sensor readings, network topology, and operational logs. All data MUST use JSON-LD format for semantic interoperability.

## 2. Core Data Types

### 2.1 System Identity

```json
{
  "@context": "https://wiastandards.com/soc-008/v1",
  "@type": "WaterSupplySystem",
  "systemId": "WS-2025-XXXX-YYYY",
  "utility": "string",
  "jurisdiction": "string",
  "operator": "string",
  "serviceArea": "string",
  "population": "integer",
  "networkLength": "float (km)",
  "dailyCapacity": "float (m³/day)",
  "certificationLevel": "BASIC|STANDARD|ADVANCED"
}
```

### 2.2 Water Quality Data

```json
{
  "@type": "WaterQuality",
  "timestamp": "ISO8601 datetime",
  "location": {
    "id": "UUID",
    "type": "source|treatment|distribution|tap",
    "coordinates": { "lat": "float", "lon": "float" }
  },
  "parameters": {
    "pH": { "value": "float", "unit": "pH", "status": "normal|warning|critical" },
    "turbidity": { "value": "float", "unit": "NTU", "status": "normal|warning|critical" },
    "chlorine": { "value": "float", "unit": "mg/L", "status": "normal|warning|critical" },
    "temperature": { "value": "float", "unit": "°C" },
    "conductivity": { "value": "float", "unit": "µS/cm" },
    "dissolvedOxygen": { "value": "float", "unit": "mg/L" },
    "TDS": { "value": "float", "unit": "mg/L" },
    "ORP": { "value": "float", "unit": "mV" },
    "hardness": { "value": "float", "unit": "mg/L CaCO₃" },
    "alkalinity": { "value": "float", "unit": "mg/L CaCO₃" }
  },
  "compliance": "compliant|non-compliant",
  "alerts": ["array", "of", "alert", "strings"]
}
```

### 2.3 Flow and Pressure Data

```json
{
  "@type": "HydraulicData",
  "timestamp": "ISO8601 datetime",
  "sensorId": "UUID",
  "location": {
    "coordinates": { "lat": "float", "lon": "float" },
    "zone": "string",
    "pipeId": "string"
  },
  "flow": {
    "rate": "float (L/min)",
    "velocity": "float (m/s)",
    "direction": "inbound|outbound"
  },
  "pressure": {
    "value": "float (bar)",
    "min": "float (bar)",
    "max": "float (bar)",
    "average": "float (bar)"
  },
  "anomalyDetected": "boolean",
  "confidence": "0-1"
}
```

### 2.4 Leak Detection Data

```json
{
  "@type": "LeakEvent",
  "eventId": "UUID",
  "detectionTime": "ISO8601 datetime",
  "location": {
    "coordinates": { "lat": "float", "lon": "float" },
    "address": "string",
    "accuracy": "float (meters)"
  },
  "severity": "low|medium|high|critical",
  "estimatedLoss": {
    "rate": "float (L/day)",
    "volume": "float (L)"
  },
  "detectionMethod": "acoustic|pressure|flow|visual|AI",
  "confidence": "0-100 (%)",
  "status": "detected|verified|repairing|resolved",
  "assignedTeam": "string",
  "responseTime": "integer (minutes)",
  "repairTime": "integer (minutes)"
}
```

### 2.5 Network Topology

```json
{
  "@type": "NetworkTopology",
  "version": "string",
  "lastUpdated": "ISO8601 datetime",
  "nodes": [
    {
      "id": "UUID",
      "type": "source|pump|tank|valve|junction",
      "coordinates": { "lat": "float", "lon": "float" },
      "elevation": "float (m)",
      "capacity": "float (m³)",
      "status": "active|inactive|maintenance"
    }
  ],
  "pipes": [
    {
      "id": "UUID",
      "startNode": "UUID",
      "endNode": "UUID",
      "length": "float (m)",
      "diameter": "float (mm)",
      "material": "cast_iron|PVC|steel|copper|HDPE",
      "installDate": "ISO8601 date",
      "condition": "excellent|good|fair|poor|critical"
    }
  ],
  "zones": [
    {
      "id": "UUID",
      "name": "string",
      "polygon": [[lat, lon], ...],
      "population": "integer",
      "consumption": "float (m³/day)"
    }
  ]
}
```

### 2.6 Consumption Data

```json
{
  "@type": "ConsumptionRecord",
  "meterId": "UUID",
  "customerId": "string",
  "timestamp": "ISO8601 datetime",
  "reading": {
    "value": "float (m³)",
    "cumulative": "float (m³)",
    "interval": "hourly|daily|monthly"
  },
  "flowRate": {
    "average": "float (L/min)",
    "peak": "float (L/min)",
    "minimum": "float (L/min)"
  },
  "anomaly": {
    "detected": "boolean",
    "type": "leak|burst|unusual_pattern|meter_error",
    "confidence": "0-100 (%)"
  },
  "billing": {
    "tier": "residential|commercial|industrial",
    "rate": "float ($/m³)",
    "amount": "float ($)"
  }
}
```

## 3. Data Quality Requirements

### 3.1 Temporal Requirements

| Data Type | Minimum Frequency | Maximum Latency | Retention Period |
|-----------|------------------|-----------------|------------------|
| Water Quality | 15 minutes | 5 minutes | 10 years |
| Flow/Pressure | 5 minutes | 1 minute | 5 years |
| Leak Detection | Real-time | 30 seconds | Permanent |
| Smart Meter | 1 hour | 15 minutes | 7 years |
| Network Topology | On change | 1 hour | Permanent |

### 3.2 Accuracy Requirements

| Parameter | Accuracy | Precision | Calibration Interval |
|-----------|----------|-----------|---------------------|
| pH | ±0.1 pH | 0.01 pH | 3 months |
| Turbidity | ±2% | 0.01 NTU | 6 months |
| Chlorine | ±0.05 mg/L | 0.01 mg/L | 3 months |
| Pressure | ±0.25% FS | 0.01 bar | 12 months |
| Flow Rate | ±1% | 1 L/min | 12 months |

### 3.3 Data Validation

All data MUST include validation metadata:

```json
{
  "validation": {
    "timestamp": "ISO8601 datetime",
    "status": "valid|suspicious|invalid",
    "checks": {
      "rangeCheck": "passed|failed",
      "rateOfChangeCheck": "passed|failed",
      "consistencyCheck": "passed|failed",
      "calibrationCheck": "passed|failed"
    },
    "qualityScore": "0-100 (%)",
    "flags": ["array", "of", "flag", "strings"]
  }
}
```

## 4. Data Exchange Formats

### 4.1 Supported Formats

- **Primary**: JSON-LD (REQUIRED)
- **Legacy**: CSV with metadata (OPTIONAL)
- **Binary**: Protocol Buffers for high-frequency data (OPTIONAL)
- **Time Series**: InfluxDB line protocol (OPTIONAL)

### 4.2 Compression

- Gzip compression RECOMMENDED for large payloads
- Maximum uncompressed payload: 10 MB
- Maximum compressed payload: 5 MB

### 4.3 Encoding

- Character encoding: UTF-8 (REQUIRED)
- Timestamps: ISO 8601 with timezone (REQUIRED)
- Coordinates: WGS84 decimal degrees (REQUIRED)
- Units: SI units preferred, conversions documented

## 5. Security and Privacy

### 5.1 Data Classification

| Classification | Examples | Access Control |
|---------------|----------|----------------|
| Public | Aggregate statistics, compliance reports | Open |
| Internal | Operational data, network maps | Authenticated |
| Confidential | Customer data, billing info | Encrypted + Authorized |
| Restricted | Security vulnerabilities, incident logs | Encrypted + MFA |

### 5.2 Personal Data

Customer data MUST comply with GDPR, CCPA, and local privacy regulations:

- Anonymization for analytics
- Right to access
- Right to deletion
- Right to portability
- Explicit consent for data sharing

### 5.3 Encryption

- Data at rest: AES-256
- Data in transit: TLS 1.3+
- Key management: Hardware security modules (HSM)
- Rotation: Quarterly minimum

## 6. Compliance and Certification

### 6.1 Testing Requirements

Implementations MUST pass automated compliance tests:

- Data format validation (100% pass rate)
- Schema validation (JSON Schema)
- Semantic validation (JSON-LD context)
- Performance benchmarks (throughput, latency)

### 6.2 Certification Levels

**BASIC**: Core formats only (water quality, flow, pressure)  
**STANDARD**: All formats including smart metering  
**ADVANCED**: Full implementation with extensions

### 6.3 Version Compatibility

- Backward compatibility REQUIRED for minor versions
- Migration path REQUIRED for major versions
- Deprecation notice: Minimum 12 months
- Support period: 3 years after deprecation

## 7. Example Implementation

```json
{
  "@context": "https://wiastandards.com/soc-008/v1",
  "@type": "WaterSupplyDataPackage",
  "version": "1.0.0",
  "systemId": "WS-2025-CITY-001",
  "timestamp": "2025-12-26T14:32:15Z",
  "data": [
    {
      "@type": "WaterQuality",
      "timestamp": "2025-12-26T14:30:00Z",
      "location": {
        "id": "SENSOR-001",
        "type": "distribution",
        "coordinates": { "lat": 37.5665, "lon": 126.9780 }
      },
      "parameters": {
        "pH": { "value": 7.3, "unit": "pH", "status": "normal" },
        "turbidity": { "value": 0.8, "unit": "NTU", "status": "normal" },
        "chlorine": { "value": 0.5, "unit": "mg/L", "status": "normal" }
      },
      "compliance": "compliant"
    },
    {
      "@type": "HydraulicData",
      "timestamp": "2025-12-26T14:32:00Z",
      "sensorId": "PRESSURE-042",
      "pressure": { "value": 4.2, "min": 3.8, "max": 4.5, "average": 4.1 },
      "flow": { "rate": 2450, "velocity": 1.2, "direction": "outbound" },
      "anomalyDetected": false
    }
  ]
}
```

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc. · MIT License
