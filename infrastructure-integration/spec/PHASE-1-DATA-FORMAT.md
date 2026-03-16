# WIA-UNI-005 - Phase 1: Data Format

**Version:** 1.0.0  
**Status:** Active  
**Last Updated:** 2025-12-25

## 1. Overview

Phase 1 of the WIA-UNI-005 Infrastructure Integration Standard defines comprehensive data formats for representing infrastructure projects, assets, and operations across the Korean Peninsula. Using JSON-LD for semantic interoperability, these formats enable consistent data exchange between diverse systems.

### 1.1 Objectives

- Establish common data schemas for all infrastructure types
- Enable semantic interoperability through linked data
- Support both human readability and machine processing
- Provide extensibility for domain-specific requirements
- Ensure data quality through validation rules

### 1.2 Scope

This specification covers:

- Infrastructure project metadata
- Physical asset descriptions
- Geospatial data representation
- Operational and sensor data
- Maintenance and inspection records
- Environmental monitoring data

## 2. Core Data Schema

### 2.1 Infrastructure Project Format

```json
{
  "@context": "https://wiastandards.com/contexts/uni-005/v1",
  "@type": "InfrastructureProject",
  "id": "wia:uni:proj:{unique-id}",
  "projectType": "railway|highway|bridge|tunnel|pipeline|power-grid|telecom",
  "name": "string",
  "description": "string",
  "location": {
    "startPoint": {
      "latitude": "number",
      "longitude": "number",
      "elevation": "number (optional)",
      "name": "string (optional)"
    },
    "endPoint": {
      "latitude": "number",
      "longitude": "number",
      "elevation": "number (optional)",
      "name": "string (optional)"
    },
    "corridor": "string (optional)",
    "jurisdiction": ["array of jurisdiction codes"]
  },
  "budget": {
    "total": "number",
    "currency": "ISO 4217 currency code",
    "breakdown": {
      "construction": "number",
      "equipment": "number",
      "environmental": "number",
      "contingency": "number"
    }
  },
  "timeline": {
    "planningStart": "ISO 8601 date",
    "constructionStart": "ISO 8601 date",
    "targetCompletion": "ISO 8601 date",
    "actualCompletion": "ISO 8601 date (optional)"
  },
  "stakeholders": [
    {
      "organization": "string",
      "role": "primary|implementing|funding|regulatory",
      "contactInfo": "object (optional)"
    }
  ],
  "standards": ["array of applicable standards"],
  "certificationStatus": "pending|in-progress|certified|expired",
  "metadata": {
    "createdBy": "string",
    "createdAt": "ISO 8601 datetime",
    "modifiedBy": "string",
    "modifiedAt": "ISO 8601 datetime",
    "version": "string"
  }
}
```

### 2.2 Asset Registration Format

```json
{
  "@context": "https://wiastandards.com/contexts/uni-005/v1",
  "@type": "InfrastructureAsset",
  "id": "wia:uni:asset:{unique-id}",
  "assetType": "bridge|tunnel|track-section|power-station|pipeline-segment",
  "name": "string",
  "description": "string",
  "projectId": "wia:uni:proj:{project-id}",
  "location": {
    "geometry": {
      "type": "Point|LineString|Polygon",
      "coordinates": "GeoJSON coordinates array",
      "crs": "EPSG:4326"
    },
    "address": "string (optional)",
    "jurisdiction": "jurisdiction code"
  },
  "technicalSpecs": {
    "// Type-specific fields": "varies by assetType"
  },
  "operationalStatus": "planned|under-construction|operational|maintenance|decommissioned",
  "commissionDate": "ISO 8601 date",
  "expectedLifespan": "number (years)",
  "certifications": [
    {
      "type": "safety|structural|environmental",
      "issuer": "string",
      "issueDate": "ISO 8601 date",
      "expiryDate": "ISO 8601 date",
      "certificateId": "string"
    }
  ]
}
```

## 3. Geospatial Standards

### 3.1 Coordinate System

- **Primary CRS:** WGS84 (EPSG:4326)
- **Precision:** Minimum 6 decimal places (±0.11m accuracy)
- **Elevation:** Meters above mean sea level (MSL)
- **Format:** GeoJSON-compatible

### 3.2 Geometry Types

| Type | Use Case | Example |
|------|----------|---------|
| Point | Stations, facilities | Power station location |
| LineString | Railways, pipelines | Track section route |
| Polygon | Zones, areas | Construction site boundary |
| MultiPoint | Sensor arrays | Distributed sensors |
| MultiLineString | Complex routes | Multi-track sections |

## 4. Operational Data Format

### 4.1 Sensor Reading Schema

```json
{
  "@type": "SensorReading",
  "sensorId": "wia:uni:sensor:{id}",
  "assetId": "wia:uni:asset:{id}",
  "timestamp": "ISO 8601 datetime with timezone",
  "measurementType": "temperature|pressure|vibration|flow|voltage|current",
  "value": "number",
  "unit": "SI unit string",
  "quality": "good|suspect|bad",
  "calibrationDate": "ISO 8601 date"
}
```

### 4.2 Real-time Status Update

```json
{
  "@type": "StatusUpdate",
  "assetId": "wia:uni:asset:{id}",
  "timestamp": "ISO 8601 datetime",
  "status": "operational|degraded|offline|emergency",
  "metrics": {
    "utilization": "number (0-1)",
    "efficiency": "number (0-1)",
    "errorCount": "number"
  },
  "alerts": [
    {
      "severity": "info|warning|critical",
      "message": "string",
      "code": "string"
    }
  ]
}
```

## 5. Maintenance Record Format

```json
{
  "@type": "MaintenanceRecord",
  "id": "wia:uni:maint:{unique-id}",
  "assetId": "wia:uni:asset:{id}",
  "recordType": "inspection|repair|upgrade|certification",
  "scheduledDate": "ISO 8601 date",
  "completionDate": "ISO 8601 date",
  "performedBy": {
    "organization": "string",
    "technician": "string",
    "certification": "string"
  },
  "findings": [
    {
      "component": "string",
      "condition": "excellent|good|fair|poor|critical",
      "notes": "string",
      "actionRequired": "boolean"
    }
  ],
  "workPerformed": "string",
  "partsReplaced": [
    {
      "partId": "string",
      "description": "string",
      "quantity": "number",
      "serialNumbers": ["array"]
    }
  ],
  "cost": {
    "labor": "number",
    "materials": "number",
    "total": "number",
    "currency": "ISO 4217 code"
  },
  "nextScheduledMaintenance": "ISO 8601 date"
}
```

## 6. Environmental Monitoring Data

```json
{
  "@type": "EnvironmentalReading",
  "monitoringStationId": "wia:uni:envmon:{id}",
  "location": {
    "latitude": "number",
    "longitude": "number"
  },
  "timestamp": "ISO 8601 datetime",
  "airQuality": {
    "pm25": "number (μg/m³)",
    "pm10": "number (μg/m³)",
    "no2": "number (ppb)",
    "so2": "number (ppb)",
    "co": "number (ppm)",
    "o3": "number (ppb)",
    "aqi": "number (0-500)"
  },
  "waterQuality": {
    "ph": "number",
    "dissolvedOxygen": "number (mg/L)",
    "turbidity": "number (NTU)",
    "temperature": "number (°C)",
    "pollutants": {}
  },
  "noise": {
    "level": "number (dBA)",
    "measurementPeriod": "string"
  },
  "carbonEmissions": {
    "co2": "number (tonnes)",
    "calculationMethod": "string"
  }
}
```

## 7. Validation Rules

### 7.1 Required Fields

All documents must include:
- `@context` and `@type` fields
- Unique `id` following WIA URI scheme
- Timestamp fields in ISO 8601 format
- Valid references to related entities

### 7.2 Data Quality Constraints

| Field Type | Constraint | Validation |
|------------|-----------|------------|
| Coordinates | Valid WGS84 | Lat: -90 to 90, Lon: -180 to 180 |
| Dates | ISO 8601 | Valid date format with timezone |
| Currency | ISO 4217 | Valid 3-letter currency code |
| Enumerations | Defined values | Must match specified options |
| Numbers | Range checks | Within physically possible bounds |

### 7.3 Referential Integrity

- Asset references must point to existing assets
- Project references must point to existing projects
- Sensor IDs must be registered in asset registry
- Stakeholder organizations must be in directory

## 8. Extensibility

### 8.1 Custom Fields

Custom fields are permitted using namespaced prefixes:

```json
{
  "custom:organizationSpecificField": "value",
  "ext:domainExtension": {}
}
```

### 8.2 Extension Registration

Organizations may register extension schemas with WIA:

- Submit extension proposal to Technical Committee
- Provide JSON Schema definition
- Document use cases and examples
- Receive assigned namespace prefix

## 9. Implementation Requirements

### 9.1 JSON-LD Processing

Implementations must:
- Support JSON-LD 1.1 processing
- Resolve @context URIs
- Handle compaction and expansion
- Support RDF triple extraction

### 9.2 Validation

All data must pass:
- JSON Schema validation
- Custom business rule validation
- Referential integrity checks
- Data quality constraints

### 9.3 Storage and Exchange

- UTF-8 encoding required
- Support for compression (gzip, brotli)
- Maximum document size: 10MB
- Streaming support for large datasets

## 10. Certification Criteria

To achieve Phase 1 certification, implementations must:

1. Generate valid JSON-LD documents for all entity types
2. Pass automated validation test suite (100% pass rate)
3. Demonstrate proper @context resolution
4. Support all required fields and data types
5. Implement extensibility mechanisms correctly
6. Provide comprehensive documentation

---

**© 2025 SmileStory Inc. / WIA**  
**弘益人間 (홍익인간) · Benefit All Humanity**
