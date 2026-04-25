# WIA Ecosystem Monitoring Standard - Phase 1: Data Format Specification v1.0

**Status:** Official Release  
**Version:** 1.0.0  
**Date:** December 26, 2025  
**Authors:** WIA Standards Committee  
**License:** CC BY 4.0

## 1. Introduction

This specification defines standardized data formats for ecosystem monitoring, enabling interoperability across monitoring systems, organizations, and platforms. Phase 1 establishes JSON-based schemas for observations, measurements, and metadata.

### 1.1 Scope

This specification covers:
- Species observation records
- Environmental sensor data
- Water quality measurements
- Air quality measurements
- Soil health assessments
- Carbon flux measurements
- Metadata documentation standards

### 1.2 Normative References

- RFC 8259: JSON Data Interchange Format
- ISO 8601: Date and time format
- ISO 19115: Geographic information metadata
- Darwin Core: Biodiversity data standard
- ENVO: Environment Ontology

## 2. Base Schema Structure

All WIA ecosystem monitoring records share a common base schema:

```json
{
  "wia_version": "1.0",
  "schema_type": "string (required)",
  "record_id": "UUID or unique identifier (required)",
  "timestamp": "ISO 8601 datetime (required)",
  "location": {
    "latitude": "number (required)",
    "longitude": "number (required)",
    "elevation": "number (optional)",
    "datum": "string (default: WGS84)",
    "precision": "number in meters (optional)",
    "location_name": "string (optional)"
  },
  "observer": {
    "id": "string (required)",
    "name": "string (optional)",
    "organization": "string (optional)",
    "email": "string (optional)"
  },
  "quality": {
    "validation_status": "enum (required)",
    "quality_flags": "array of strings (optional)",
    "confidence_level": "number 0-1 (optional)"
  }
}
```

### 2.1 Required Fields

- `wia_version`: Standard version (currently "1.0")
- `schema_type`: Type of record (e.g., "species-observation", "water-quality")
- `record_id`: Globally unique identifier
- `timestamp`: Date and time in ISO 8601 format with timezone
- `location.latitude`: Decimal degrees (-90 to 90)
- `location.longitude`: Decimal degrees (-180 to 180)
- `observer.id`: Observer unique identifier
- `quality.validation_status`: One of: "unvalidated", "in_review", "validated", "expert_verified", "questionable", "invalid"

### 2.2 Optional Fields

- `location.elevation`: Meters above sea level
- `location.datum`: Coordinate reference system (default: WGS84)
- `location.precision`: Location precision in meters
- `quality.confidence_level`: 0.0 (no confidence) to 1.0 (complete confidence)

## 3. Species Observation Schema

### 3.1 Schema Type: "species-observation"

Extended fields for biodiversity observations:

```json
{
  "taxon": {
    "scientific_name": "string (required)",
    "common_name": "string (optional)",
    "kingdom": "string (optional)",
    "phylum": "string (optional)",
    "class": "string (optional)",
    "order": "string (optional)",
    "family": "string (optional)",
    "genus": "string (optional)",
    "species": "string (optional)",
    "subspecies": "string (optional)",
    "taxon_authority": "string (required)",
    "taxon_id": "string (optional)"
  },
  "detection_method": "enum (required)",
  "occurrence_status": "enum (required)",
  "abundance": "number or string (optional)",
  "life_stage": "enum (optional)",
  "sex": "enum (optional)",
  "behavior": "string (optional)",
  "reproductive_condition": "string (optional)",
  "habitat_type": "string using ENVO (optional)",
  "associated_taxa": "array of taxon objects (optional)",
  "environmental_conditions": "object (optional)"
}
```

### 3.2 Controlled Vocabularies

**detection_method** (required):
- "visual_survey" - Direct observation
- "camera_trap" - Motion-activated camera
- "acoustic_monitoring" - Audio recording
- "edna" - Environmental DNA sampling
- "telemetry" - Radio/GPS tracking
- "mark_recapture" - Capture-mark-recapture
- "remote_sensing" - Satellite/aerial imagery
- "specimen" - Physical specimen collection

**occurrence_status** (required):
- "present" - Organism detected
- "absent" - Target survey found no organism
- "trace" - Evidence but no direct observation

**life_stage** (optional):
- "adult", "juvenile", "larva", "egg", "seed", "seedling", "sapling"

**sex** (optional):
- "male", "female", "hermaphrodite", "unknown"

## 4. Environmental Sensor Data Schema

### 4.1 Schema Type: "sensor-timeseries"

```json
{
  "sensor_id": "string (required)",
  "sensor_metadata": {
    "sensor_type": "string (required)",
    "manufacturer": "string (optional)",
    "model": "string (optional)",
    "serial_number": "string (optional)",
    "measurement_parameter": "string (required)",
    "measurement_unit": "string using QUDT (required)",
    "precision": "number (optional)",
    "accuracy": "number (optional)",
    "detection_limit": "number (optional)",
    "calibration_date": "ISO 8601 date (required)",
    "calibration_certificate": "string or URL (optional)"
  },
  "deployment": {
    "deployment_date": "ISO 8601 datetime (required)",
    "location": "location object (required)",
    "height_above_ground": "number in meters (optional)",
    "depth_below_surface": "number in meters (optional)",
    "environment": "string (optional)"
  },
  "data": {
    "start_time": "ISO 8601 datetime (required)",
    "end_time": "ISO 8601 datetime (required)",
    "interval_seconds": "number (required)",
    "readings": "array of reading objects (required)"
  }
}
```

### 4.2 Reading Object Structure

```json
{
  "timestamp": "ISO 8601 datetime (required)",
  "value": "number (required)",
  "qc_flag": "enum (required)",
  "qc_notes": "string (optional)"
}
```

**qc_flag** values:
- "good" - Passes all QC checks
- "questionable" - Marginally acceptable
- "bad" - Fails QC, exclude from analysis
- "missing" - Expected but not collected
- "estimated" - Value interpolated/modeled

## 5. Water Quality Schema

### 5.1 Schema Type: "water-quality-sample"

```json
{
  "sample_id": "string (required)",
  "waterbody_name": "string (optional)",
  "site_id": "string (optional)",
  "sampling_depth_meters": "number (optional)",
  "sampling_method": "enum (required)",
  "parameters": {
    "temperature_c": "number (optional)",
    "ph": "number (optional)",
    "dissolved_oxygen_mgl": "number (optional)",
    "turbidity_ntu": "number (optional)",
    "conductivity_uscm": "number (optional)",
    "total_nitrogen_mgl": "number (optional)",
    "total_phosphorus_mgl": "number (optional)",
    "chlorophyll_a_ugl": "number (optional)",
    "salinity_ppt": "number (optional)"
  },
  "laboratory": {
    "lab_name": "string (optional)",
    "analysis_date": "ISO 8601 date (optional)",
    "methods": "object mapping parameter to method (optional)",
    "detection_limits": "object (optional)"
  }
}
```

**sampling_method** values:
- "grab_sample" - Single discrete sample
- "composite" - Multiple samples combined
- "in_situ" - Measured in place with sensor
- "continuous" - Automated continuous monitoring

## 6. Air Quality Schema

### 6.1 Schema Type: "air-quality-measurement"

```json
{
  "station_id": "string (required)",
  "parameters": {
    "pm25_ugm3": "number (optional)",
    "pm10_ugm3": "number (optional)",
    "o3_ppb": "number (optional)",
    "no2_ppb": "number (optional)",
    "so2_ppb": "number (optional)",
    "co_ppm": "number (optional)",
    "co2_ppm": "number (optional)",
    "temperature_c": "number (optional)",
    "relative_humidity_percent": "number (optional)",
    "pressure_mb": "number (optional)",
    "wind_speed_ms": "number (optional)",
    "wind_direction_degrees": "number (optional)"
  },
  "aqi_value": "number (optional)",
  "aqi_category": "enum (optional)"
}
```

## 7. Metadata Standards

Every dataset MUST include comprehensive metadata following these requirements:

### 7.1 Dataset-Level Metadata

```json
{
  "title": "string (required)",
  "abstract": "string (required)",
  "keywords": "array of strings (required)",
  "authors": "array of person objects (required)",
  "contacts": "array of contact objects (required)",
  "funding_sources": "array of strings (optional)",
  "temporal_coverage": {
    "start_date": "ISO 8601 date (required)",
    "end_date": "ISO 8601 date (optional)"
  },
  "geographic_coverage": {
    "bounding_box": "array [minLon, minLat, maxLon, maxLat] (required)",
    "description": "string (optional)"
  },
  "taxonomic_coverage": "array of taxon objects (optional)",
  "methods": {
    "description": "string (required)",
    "protocol_url": "URL (optional)",
    "sampling_design": "string (optional)"
  },
  "quality_assurance": {
    "description": "string (required)",
    "validation_procedures": "string (optional)"
  },
  "access": {
    "license": "string (required)",
    "restrictions": "string (optional)",
    "citation": "string (required)"
  },
  "related_resources": {
    "publications": "array of URLs or DOIs (optional)",
    "related_datasets": "array of URLs or DOIs (optional)"
  }
}
```

## 8. Validation Rules

All WIA-compliant data MUST pass these validation checks:

1. **Schema Validation**: Data structure matches JSON schema
2. **Required Fields**: All required fields present and non-null
3. **Data Types**: Values match specified types
4. **Range Checks**: Numeric values within valid ranges
5. **Coordinate Validation**: Latitude -90 to 90, longitude -180 to 180
6. **Temporal Validation**: Dates not in future, end >= start
7. **Taxonomic Validation**: Scientific names resolve to authority
8. **Quality Flags**: QC flags from controlled vocabulary

## 9. File Format Conventions

### 9.1 Single Record Files
- Encoding: UTF-8
- Format: JSON
- Extension: .json
- Compression: gzip optional (.json.gz)

### 9.2 Multi-Record Files
- Newline-delimited JSON (NDJSON)
- One record per line
- Extension: .ndjson or .jsonl
- Compression: gzip recommended for large files

### 9.3 Batch Submission
- JSON array of records
- Maximum 10,000 records per file recommended
- Use NDJSON for larger datasets

## 10. Version Control

This specification uses semantic versioning (MAJOR.MINOR.PATCH):
- MAJOR: Breaking changes to schema structure
- MINOR: Backward-compatible additions
- PATCH: Clarifications and corrections

Current version: 1.0.0
