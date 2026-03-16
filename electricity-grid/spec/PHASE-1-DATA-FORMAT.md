# PHASE 1: Data Format Specification

**WIA-SOC-010 Electricity Grid Standard**
Version: 1.0
Status: Draft
Last Updated: 2025-12-26

---

## 1. Overview

This document defines the data formats and structures for the WIA-SOC-010 Electricity Grid Standard. All implementations MUST conform to these specifications to ensure interoperability across different systems, vendors, and jurisdictions.

## 2. Data Exchange Format

### 2.1 Primary Format: JSON-LD

All data exchanges SHALL use JSON-LD (JSON for Linked Data) as the primary format. JSON-LD enables:
- Human-readable data structures
- Machine-processable linked data
- Semantic interoperability
- Extensibility through vocabularies

**Example:**
```json
{
  "@context": "https://wia-official.org/standards/soc-010/context.jsonld",
  "@type": "GridStatusReport",
  "@id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-26T14:30:00Z",
  "gridOperator": {
    "@type": "Organization",
    "name": "Metropolitan Power Grid",
    "id": "urn:grid:operator:mpg-001"
  },
  "systemLoad": {
    "@type": "PowerMeasurement",
    "value": 2450,
    "unit": "MW",
    "timestamp": "2025-12-26T14:30:00Z"
  },
  "renewableGeneration": {
    "@type": "PowerMeasurement",
    "value": 1028,
    "unit": "MW",
    "percentage": 42.0
  }
}
```

### 2.2 Alternative Formats

Implementations MAY support additional formats for specific use cases:
- **XML**: For legacy system integration
- **Protocol Buffers**: For high-performance applications
- **CSV**: For bulk data exchange
- **Parquet**: For big data analytics

However, JSON-LD MUST always be supported as the canonical format.

## 3. Core Data Models

### 3.1 Grid Status

```json
{
  "@context": "https://wia-official.org/standards/soc-010/context.jsonld",
  "@type": "GridStatus",
  "id": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "region": "string",
  "frequency": {
    "value": "number",
    "unit": "Hz",
    "deviation": "number"
  },
  "voltage": {
    "nominal": "number",
    "actual": "number",
    "unit": "kV"
  },
  "currentLoad": {
    "value": "number",
    "unit": "MW"
  },
  "capacity": {
    "total": "number",
    "available": "number",
    "unit": "MW"
  },
  "loadFactor": "number (0-1)",
  "status": "enum (normal|warning|critical|emergency)"
}
```

### 3.2 Renewable Energy Data

```json
{
  "@type": "RenewableGeneration",
  "id": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "sources": [
    {
      "type": "enum (solar|wind|hydro|biomass|geothermal)",
      "capacity": "number (MW)",
      "generation": "number (MW)",
      "availabilityFactor": "number (0-1)",
      "location": {
        "latitude": "number",
        "longitude": "number",
        "altitude": "number (optional)"
      },
      "forecast": {
        "horizon": "ISO 8601 duration",
        "predictions": [
          {
            "timestamp": "ISO 8601 datetime",
            "expectedGeneration": "number (MW)",
            "confidenceInterval": {
              "lower": "number",
              "upper": "number",
              "level": "number (0-1)"
            }
          }
        ]
      }
    }
  ],
  "totalGeneration": "number (MW)",
  "penetrationRate": "number (0-1)"
}
```

### 3.3 Energy Storage

```json
{
  "@type": "EnergyStorage",
  "id": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "technology": "enum (lithium-ion|flow-battery|pumped-hydro|compressed-air|other)",
  "capacity": {
    "power": "number (MW)",
    "energy": "number (MWh)",
    "duration": "number (hours)"
  },
  "stateOfCharge": "number (0-100 %)",
  "powerFlow": {
    "value": "number (MW, positive=charging, negative=discharging)",
    "direction": "enum (charging|discharging|idle)"
  },
  "efficiency": {
    "roundTrip": "number (0-1)",
    "charging": "number (0-1)",
    "discharging": "number (0-1)"
  },
  "cycleCount": "integer",
  "healthStatus": {
    "stateOfHealth": "number (0-100 %)",
    "estimatedRemainingLife": "ISO 8601 duration"
  }
}
```

### 3.4 Demand Response Event

```json
{
  "@type": "DemandResponseEvent",
  "id": "string (UUID)",
  "programType": "enum (dynamic-pricing|direct-control|interruptible|emergency)",
  "startTime": "ISO 8601 datetime",
  "endTime": "ISO 8601 datetime",
  "targetLoadReduction": "number (MW)",
  "actualLoadReduction": "number (MW)",
  "participants": {
    "enrolled": "integer",
    "active": "integer",
    "opted-out": "integer"
  },
  "pricing": {
    "baseline": "number (currency/kWh)",
    "event": "number (currency/kWh)",
    "currency": "ISO 4217 code"
  },
  "incentives": {
    "total": "number (currency)",
    "perParticipant": "number (currency)",
    "currency": "ISO 4217 code"
  },
  "status": "enum (scheduled|active|completed|canceled)"
}
```

### 3.5 Power Quality Metrics

```json
{
  "@type": "PowerQualityMetrics",
  "id": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "location": "string (substation/feeder ID)",
  "frequency": {
    "nominal": "number (Hz)",
    "actual": "number (Hz)",
    "deviation": "number (Hz)",
    "stability": "number (0-1)"
  },
  "voltage": {
    "nominal": "number (kV)",
    "actual": "number (kV)",
    "sags": "integer (count)",
    "swells": "integer (count)",
    "interruptions": "integer (count)"
  },
  "harmonics": {
    "thd": "number (0-100 %)",
    "individual": [
      {
        "harmonic": "integer (2, 3, 5, 7, ...)",
        "magnitude": "number (% of fundamental)"
      }
    ]
  },
  "powerFactor": "number (-1 to 1)",
  "flicker": {
    "pst": "number",
    "plt": "number"
  }
}
```

### 3.6 Smart Meter Reading

```json
{
  "@type": "MeterReading",
  "meterId": "string",
  "timestamp": "ISO 8601 datetime",
  "customerId": "string",
  "location": {
    "address": "string",
    "coordinates": {
      "latitude": "number",
      "longitude": "number"
    }
  },
  "consumption": {
    "energy": "number (kWh)",
    "interval": "ISO 8601 duration",
    "imported": "number (kWh)",
    "exported": "number (kWh, for prosumers)"
  },
  "demand": {
    "peak": "number (kW)",
    "average": "number (kW)"
  },
  "voltage": {
    "min": "number (V)",
    "max": "number (V)",
    "average": "number (V)"
  },
  "events": [
    {
      "type": "enum (outage|voltage-sag|voltage-swell|tamper|...)",
      "timestamp": "ISO 8601 datetime",
      "duration": "ISO 8601 duration",
      "severity": "enum (info|warning|critical)"
    }
  ]
}
```

## 4. Data Quality Requirements

### 4.1 Timestamp Precision
- All timestamps MUST use ISO 8601 format with timezone
- Precision MUST be at least 1 second
- For synchrophasor data, precision MUST be 1 millisecond or better
- UTC timezone is RECOMMENDED for all data exchange

### 4.2 Measurement Accuracy
- Voltage measurements: ±0.5%
- Current measurements: ±1%
- Power measurements: ±1.5%
- Frequency measurements: ±0.01 Hz
- Energy measurements: ±2%

### 4.3 Data Completeness
- Missing data MUST be explicitly indicated (null or NaN)
- Estimated values MUST be flagged with quality indicator
- Data gaps exceeding 15 minutes MUST trigger alarms

### 4.4 Validation Rules
1. Frequency MUST be within ±5% of nominal (57-63 Hz for 60 Hz systems)
2. Voltage MUST be within ±10% of nominal under normal conditions
3. Power factor MUST be between -1 and 1
4. State of charge MUST be 0-100%
5. Efficiency values MUST be 0-1 (0-100%)

## 5. Schema Versioning

### 5.1 Version Format
- Semantic versioning: MAJOR.MINOR.PATCH
- Example: 1.2.3

### 5.2 Compatibility Rules
- MAJOR: Breaking changes (backward incompatible)
- MINOR: New features (backward compatible)
- PATCH: Bug fixes (backward compatible)

### 5.3 Version Negotiation
```json
{
  "@context": "https://wia-official.org/standards/soc-010/context.jsonld",
  "schemaVersion": "1.0.0",
  "supportedVersions": ["1.0.0", "1.0.1", "1.1.0"]
}
```

## 6. Compression and Encoding

### 6.1 Compression
- Implementations SHOULD support gzip compression
- Large datasets (>1 MB) SHOULD be compressed
- HTTP Content-Encoding: gzip header MUST be used

### 6.2 Character Encoding
- UTF-8 MUST be used for all text data
- Binary data MUST be Base64 encoded when included in JSON

## 7. Security Considerations

### 7.1 Sensitive Data
Personal identifiable information (PII) MUST be:
- Encrypted at rest
- Encrypted in transit (TLS 1.3+)
- Anonymized when possible
- Subject to access controls

### 7.2 Data Integrity
- Digital signatures SHOULD be used for critical data
- Checksums MUST be provided for large datasets
- Tamper detection mechanisms MUST be implemented

## 8. Metadata

All data objects SHOULD include metadata:
```json
{
  "metadata": {
    "source": "string (system ID)",
    "quality": "enum (raw|validated|estimated|derived)",
    "reliability": "number (0-1)",
    "latency": "ISO 8601 duration",
    "processingTimestamp": "ISO 8601 datetime"
  }
}
```

---

**End of PHASE 1 Specification**

For API endpoints and protocols, see PHASE-2-API.md
For communication protocols, see PHASE-3-PROTOCOL.md
For integration patterns, see PHASE-4-INTEGRATION.md
