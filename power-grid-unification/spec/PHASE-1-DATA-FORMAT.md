# WIA-UNI-007 - Phase 1: Data Format

**Version:** 1.0.0  
**Status:** Active  
**Last Updated:** 2025-12-25

## 1. Overview

Phase 1 of the WIA-UNI-007 Power Grid Unification Standard defines comprehensive data formats for representing power grid infrastructure, energy flows, and operational metrics across the Korean Peninsula. Using JSON-LD for semantic interoperability, these formats enable consistent data exchange between grid operators, renewable energy providers, and trading platforms.

### 1.1 Objectives

- Establish common data schemas for power grid assets
- Enable real-time energy flow monitoring
- Support renewable energy integration and tracking
- Provide extensibility for emerging technologies
- Ensure data quality through validation rules

### 1.2 Scope

This specification covers:

- Power grid node metadata (substations, HVDC terminals)
- Real-time power flow measurements
- Renewable energy generation data
- Energy storage system status
- Grid topology and connectivity
- Asset lifecycle and maintenance records

## 2. Core Data Schema

### 2.1 Power Grid Node Format

```json
{
  "@context": "https://wiastandards.com/contexts/uni-007/v1",
  "@type": "PowerGridNode",
  "id": "wia:grid:{unique-id}",
  "nodeType": "substation|hvdc-terminal|solar-farm|wind-farm|hydro-plant|storage|transmission-line",
  "name": "string",
  "location": {
    "latitude": "number",
    "longitude": "number",
    "elevation": "number (optional)",
    "address": "string",
    "region": "north|south|dmz"
  },
  "capacity": {
    "value": "number",
    "unit": "MW|GW",
    "ratedVoltage": {
      "value": "number",
      "unit": "kV"
    }
  },
  "operationalStatus": "operational|planned|under-construction|maintenance|decommissioned",
  "grid": {
    "operator": "string",
    "region": "string",
    "connectionType": "AC|HVDC"
  },
  "standards": ["IEC-61850", "WIA-UNI-007"],
  "certificationStatus": "certified|pending|expired",
  "metadata": {
    "createdAt": "ISO 8601 datetime",
    "updatedAt": "ISO 8601 datetime",
    "version": "string"
  }
}
```

### 2.2 Power Flow Data Format

```json
{
  "@context": "https://wiastandards.com/contexts/uni-007/v1",
  "@type": "PowerFlowMeasurement",
  "nodeId": "wia:grid:{node-id}",
  "timestamp": "ISO 8601 datetime (millisecond precision)",
  "activePower": {
    "value": "number",
    "unit": "MW",
    "direction": "import|export|balanced"
  },
  "reactivePower": {
    "value": "number",
    "unit": "MVAr"
  },
  "voltage": {
    "value": "number",
    "unit": "kV"
  },
  "frequency": {
    "value": "number",
    "unit": "Hz"
  },
  "lineLoading": {
    "percentage": "number (0-100)",
    "temperature": "number (celsius)"
  }
}
```

### 2.3 Renewable Energy Metrics

```json
{
  "@context": "https://wiastandards.com/contexts/uni-007/v1",
  "@type": "RenewableEnergyMetrics",
  "sourceId": "wia:grid:{source-id}",
  "sourceType": "solar|wind|hydro|tidal|geothermal",
  "timestamp": "ISO 8601 datetime",
  "generation": {
    "current": "number (MW)",
    "forecast24h": "array of numbers",
    "capacityFactor": "number (0-1)"
  },
  "environmental": {
    "carbonAvoided": "number (tons CO2)",
    "renewablePercentage": "number (0-100)"
  },
  "weather": {
    "windSpeed": "number (m/s)",
    "solarIrradiance": "number (W/m²)",
    "temperature": "number (celsius)"
  }
}
```

## 3. Validation Rules

- All coordinates must use WGS84 datum
- Timestamps must be UTC with millisecond precision
- Power values must be non-negative
- Frequency must be within ±2 Hz of nominal (60Hz)
- All required fields must be present
- Unit consistency must be enforced

## 4. Extensibility

Custom fields can be added under the "extensions" key while maintaining core schema compatibility.

