# WIA Environmental Sensor Standard - Phase 1: Data Format
## Version 1.0.0 | WIA-ENE-027-PHASE-1

**Status:** Final
**Published:** 2025-01-01
**Organization:** World Certification Industry Association (WIA)
**License:** MIT License

---

## 1. Overview

Phase 1 of the WIA Environmental Sensor Standard (WIA-ENE-027) defines standardized data formats for environmental sensor measurements. This specification establishes JSON schemas, field definitions, data types, units, validation rules, and metadata requirements that enable interoperability across diverse sensor types and manufacturers.

### 1.1 Purpose

The primary purpose of Phase 1 is to ensure that environmental sensor data can be:
- **Exchanged** between different systems without format conversion
- **Interpreted** correctly through standardized field names and units
- **Validated** automatically using defined schemas and rules
- **Compared** meaningfully across sensors and deployments
- **Archived** in consistent formats for long-term value

### 1.2 Scope

Phase 1 covers:
- Core data model structure common to all sensor types
- Sensor-specific schemas for air quality, water quality, soil, meteorological, radiation, and noise sensors
- Metadata specifications for sensor description and data quality
- Validation rules and quality assurance frameworks
- JSON Schema definitions for automated validation

Phase 1 does NOT cover:
- API interfaces (see Phase 2)
- Communication protocols (see Phase 3)
- System integration patterns (see Phase 4)

### 1.3 Normative References

- **RFC 8259**: The JavaScript Object Notation (JSON) Data Interchange Format
- **JSON Schema Draft 2020-12**: JSON Schema Specification
- **ISO 8601:2019**: Date and time format
- **ISO 19115**: Geographic information - Metadata

---

## 2. Core Data Model

All WIA-compliant environmental sensor data messages share a common core structure providing essential context and metadata.

### 2.1 Mandatory Fields

Every data message MUST include the following fields:

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| `version` | string | WIA standard version (semantic versioning) | `"1.0.0"` |
| `standard` | string | WIA standard identifier | `"WIA-ENE-027"` |
| `deviceId` | string | Globally unique device identifier | `"ENV-AIR-001"` |
| `timestamp` | string | ISO 8601 timestamp with timezone | `"2024-12-26T10:30:00.000Z"` |
| `sensorType` | string | Sensor category identifier | `"air_quality"` |
| `readings` | object | Sensor-specific measurements | See section 3 |

### 2.2 Optional Recommended Fields

The following fields are optional but STRONGLY RECOMMENDED:

#### 2.2.1 Location

```json
"location": {
  "latitude": number,    // Decimal degrees, WGS84
  "longitude": number,   // Decimal degrees, WGS84
  "altitude": number,    // Meters above sea level (optional)
  "accuracy": number,    // Location uncertainty in meters (optional)
  "datum": string        // Coordinate system, default "WGS84"
}
```

#### 2.2.2 Metadata

```json
"metadata": {
  "manufacturer": string,     // Device manufacturer
  "model": string,            // Device model number
  "firmware": string,         // Firmware version
  "battery": number,          // Battery level (0-100%)
  "signalStrength": number,   // RSSI in dBm
  "uptime": number            // Seconds since last restart
}
```

#### 2.2.3 Quality

```json
"quality": {
  "overall": string,    // "good" | "suspect" | "bad" | "missing"
  "flags": [string],    // Array of quality flags
  "confidence": number  // Confidence score 0.0-1.0 (optional)
}
```

#### 2.2.4 Calibration

```json
"calibration": {
  "lastCalibration": string,     // ISO 8601 timestamp
  "nextCalibration": string,     // ISO 8601 timestamp
  "method": string,              // Calibration method
  "parameters": object,          // Method-specific parameters
  "certificateId": string        // Calibration certificate reference
}
```

---

## 3. Sensor-Specific Schemas

### 3.1 Air Quality Sensors

**Sensor Type:** `"air_quality"`

#### 3.1.1 Particulate Matter

```json
{
  "pm1_0": {
    "value": number,          // Particulate matter concentration
    "unit": "μg/m³",          // Micrograms per cubic meter
    "method": string,         // "laser_scattering" | "beta_attenuation"
    "uncertainty": number,    // Measurement uncertainty (±)
    "averaged": number        // Averaging period in seconds (optional)
  },
  "pm2_5": { /* same structure */ },
  "pm4_0": { /* same structure */ },
  "pm10": { /* same structure */ }
}
```

**Valid Ranges:**
- PM1.0: 0-500 μg/m³
- PM2.5: 0-500 μg/m³
- PM10: 0-1000 μg/m³

#### 3.1.2 Gaseous Pollutants

```json
{
  "co2": {
    "value": number,          // Carbon dioxide concentration
    "unit": "ppm",            // Parts per million
    "method": "NDIR",         // Detection method
    "uncertainty": number
  },
  "co": {
    "value": number,
    "unit": "ppm",
    "method": "electrochemical",
    "uncertainty": number
  },
  "no2": {
    "value": number,
    "unit": "ppb",            // Parts per billion
    "method": "electrochemical",
    "uncertainty": number
  },
  "o3": {
    "value": number,
    "unit": "ppb",
    "method": "electrochemical" | "uv_absorption",
    "uncertainty": number
  },
  "so2": {
    "value": number,
    "unit": "ppb",
    "method": "electrochemical",
    "uncertainty": number
  },
  "voc": {
    "value": number,
    "unit": "ppb",
    "method": "metal_oxide" | "pid",
    "uncertainty": number,
    "compound": string        // Specific VOC if measured (optional)
  }
}
```

#### 3.1.3 Air Quality Index

```json
{
  "aqi": {
    "value": number,          // 0-500 scale
    "category": string,       // "good" | "moderate" | "unhealthy_sensitive"
                              // | "unhealthy" | "very_unhealthy" | "hazardous"
    "pollutant": string,      // Primary pollutant determining AQI
    "standard": string        // "US_EPA" | "EU_CAQI" | "CN_AQI" etc.
  }
}
```

### 3.2 Water Quality Sensors

**Sensor Type:** `"water_quality"`

```json
{
  "ph": {
    "value": number,              // pH value 0-14
    "unit": "pH",
    "method": "glass_electrode" | "ion_selective",
    "temperature_compensated": boolean,
    "uncertainty": number
  },
  "dissolved_oxygen": {
    "value": number,              // Dissolved oxygen concentration
    "unit": "mg/L",               // Milligrams per liter
    "method": "optical" | "electrochemical",
    "saturation": number,         // Percent saturation (optional)
    "uncertainty": number
  },
  "turbidity": {
    "value": number,
    "unit": "NTU",                // Nephelometric Turbidity Units
    "method": "nephelometric" | "turbidimetric",
    "uncertainty": number
  },
  "conductivity": {
    "value": number,
    "unit": "μS/cm",              // Microsiemens per centimeter
    "temperature": number,        // Temperature at measurement (°C)
    "specific_conductance": number, // Temperature-compensated to 25°C
    "uncertainty": number
  },
  "temperature": {
    "value": number,
    "unit": "°C",
    "uncertainty": number
  },
  "orp": {
    "value": number,              // Oxidation-Reduction Potential
    "unit": "mV",                 // Millivolts
    "uncertainty": number
  }
}
```

**Additional Water Quality Parameters (Optional):**
- Chlorophyll: µg/L (algae monitoring)
- Total Dissolved Solids (TDS): mg/L
- Salinity: ppt (parts per thousand)
- Specific ions: nitrate, ammonium, chloride (mg/L or ppm)

### 3.3 Soil Sensors

**Sensor Type:** `"soil"`

```json
{
  "moisture": {
    "value": number,              // Volumetric water content
    "unit": "%VWC",               // Percent volumetric water content
    "method": "capacitance" | "tdr" | "fdr",
    "depth": number,              // Measurement depth in meters
    "uncertainty": number
  },
  "temperature": {
    "value": number,
    "unit": "°C",
    "depth": number,
    "uncertainty": number
  },
  "electrical_conductivity": {
    "value": number,
    "unit": "μS/cm",              // Soil salinity indicator
    "depth": number,
    "uncertainty": number
  },
  "nutrients": {
    "nitrogen": {
      "value": number,
      "unit": "ppm",
      "form": "nitrate_NO3" | "ammonium_NH4" | "total_N",
      "method": string,
      "uncertainty": number
    },
    "phosphorus": {
      "value": number,
      "unit": "ppm",
      "form": "available_P" | "total_P",
      "uncertainty": number
    },
    "potassium": {
      "value": number,
      "unit": "ppm",
      "form": "exchangeable_K" | "total_K",
      "uncertainty": number
    }
  }
}
```

### 3.4 Meteorological Sensors

**Sensor Type:** `"meteorological"`

```json
{
  "temperature": {
    "value": number,
    "unit": "°C",
    "uncertainty": number
  },
  "humidity": {
    "value": number,              // Relative humidity
    "unit": "%RH",
    "uncertainty": number
  },
  "pressure": {
    "value": number,
    "unit": "hPa",                // Hectopascals
    "type": "station" | "sea_level",
    "uncertainty": number
  },
  "wind_speed": {
    "value": number,
    "unit": "m/s",
    "gust": number,               // Peak gust speed (optional)
    "uncertainty": number
  },
  "wind_direction": {
    "value": number,              // 0-360 degrees from north
    "unit": "degrees",
    "uncertainty": number
  },
  "precipitation": {
    "value": number,
    "unit": "mm",
    "type": "cumulative" | "rate",
    "period": number              // Time period in seconds
  },
  "solar_radiation": {
    "value": number,
    "unit": "W/m²",               // Watts per square meter
    "type": "global" | "direct" | "diffuse"
  },
  "uv_index": {
    "value": number,              // 0-11+ scale
    "unit": "index"
  }
}
```

---

## 4. Validation Rules

### 4.1 Range Validation

All measurements MUST fall within physically plausible ranges:

| Parameter | Min | Max | Unit |
|-----------|-----|-----|------|
| Temperature (ambient) | -50 | 60 | °C |
| Relative Humidity | 0 | 100 | %RH |
| PM2.5 | 0 | 1000 | μg/m³ |
| PM10 | 0 | 2000 | μg/m³ |
| pH | 0 | 14 | pH |
| Dissolved Oxygen | 0 | 20 | mg/L |

### 4.2 Temporal Consistency

- Timestamps MUST be in ISO 8601 format with timezone
- Future timestamps (beyond current time + 60 seconds) are INVALID
- Rate-of-change limits prevent unrealistic rapid changes

### 4.3 Cross-Parameter Validation

- PM2.5 SHOULD be ≤ PM10
- Dew point MUST be ≤ air temperature
- Specific conductance relates to temperature and conductivity

---

## 5. Quality Flags

Standard quality flags indicate data conditions:

| Flag | Meaning | Action |
|------|---------|--------|
| `calibration_due` | Sensor calibration overdue | Use with caution |
| `out_of_range` | Value exceeds valid range | Investigate sensor |
| `rapid_change` | Unrealistic rate of change | Potential malfunction |
| `low_battery` | Battery below 20% | Replace soon |
| `maintenance_required` | Service needed | Schedule maintenance |
| `estimated` | Value estimated/interpolated | Lower confidence |

---

## 6. JSON Schema

Complete JSON Schema available at:
`https://wiastandards.com/schemas/env-sensor-v1.0.json`

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*

---

## Annex A — Conformance Tier Matrix

WIA conformance for environmental-sensor is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/environmental-sensor/api/` — TypeScript SDK skeleton
- `wia-standards/standards/environmental-sensor/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/environmental-sensor/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
