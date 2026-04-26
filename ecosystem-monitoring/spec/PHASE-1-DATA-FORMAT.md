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


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.
