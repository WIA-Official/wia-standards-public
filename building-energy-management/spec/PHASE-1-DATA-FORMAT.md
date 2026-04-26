# WIA Building Energy Management Standard
## Phase 1: Data Format Specification
### Version 1.0

---

## Document Information

- **Standard**: WIA-BEMS (Building Energy Management Standard)
- **Phase**: 1 - Data Format
- **Version**: 1.0.0
- **Status**: Published
- **Date**: January 2025
- **Organization**: WIA (World Certification Industry Association)
- **License**: Open Standard (Freely Implementable)

---

## 1. Introduction

Phase 1 of the WIA-BEMS standard establishes comprehensive data formats and schemas for all building energy management information. These standardized formats enable interoperability between devices, systems, and platforms from different manufacturers, eliminating the vendor lock-in and integration complexity that plague traditional building energy management systems.

### 1.1 Objectives

- Define canonical data structures for all building energy data types
- Establish consistent units, semantics, and metadata requirements
- Enable aggregation and analysis across diverse systems
- Provide validation mechanisms for data quality assurance
- Support both real-time and historical data representations

### 1.2 Scope

Phase 1 covers data formats for:
- Energy consumption and demand measurements
- Environmental conditions (temperature, humidity, air quality)
- Occupancy and space utilization
- Equipment status and performance
- Renewable energy generation
- Energy storage systems
- Control commands and responses

---

## 2. Core Principles

### 2.1 JSON as Base Format

All WIA-BEMS data uses JSON (JavaScript Object Notation) as the serialization format, with formal schemas defined using JSON Schema Draft 7 or later.

**Rationale**: JSON provides human-readable, language-agnostic data representation with widespread tool support and excellent performance characteristics.

### 2.2 ISO Standards Compliance

- **Timestamps**: ISO 8601 format with UTC or explicit timezone
- **Units**: SI units as primary, with conversion specifications for alternatives
- **Language codes**: ISO 639-1 for multi-language support
- **Currency codes**: ISO 4217 for financial data

### 2.3 Semantic Consistency

Field names, value ranges, and enumeration values maintain consistent meaning across all schemas. The standard uses Project Haystack-compatible naming conventions where applicable for maximum compatibility with existing BMS deployments.

---

## 3. Energy Measurement Schema

### 3.1 Energy Consumption

**Schema URI**: `https://wia.org/bems/v1/schemas/energy-consumption.json`

**Required Fields**:
- `measurement_id` (string, UUID): Unique identifier
- `timestamp_start` (string, ISO 8601): Period start
- `timestamp_end` (string, ISO 8601): Period end
- `location` (object): Building, floor, zone, meter identification
- `energy` (object): Consumption value, unit, type
- `quality` (object): Data quality indicators

**Optional Fields**:
- `cost` (object): Associated cost information
- `metadata` (object): Additional context

**Units**: Primary unit is kilowatt-hour (kWh). Alternative units (BTU, therms, joules) must include `unit` field with conversion factors.

### 3.2 Power Demand

**Schema URI**: `https://wia.org/bems/v1/schemas/power-demand.json`

**Required Fields**:
- `measurement_id` (string, UUID): Unique identifier
- `timestamp` (string, ISO 8601): Measurement time
- `location` (object): Measurement point identification
- `power` (object): Demand value, unit, power factor
- `quality` (object): Accuracy, validation status

**Units**: Primary unit is kilowatt (kW). Includes power factor (0.0-1.0) for AC electrical systems.

---

## 4. Environmental Data Schema

### 4.1 Temperature Measurement

**Schema URI**: `https://wia.org/bems/v1/schemas/temperature.json`

**Measurement Types**:
- `ambient_air`: General space air temperature
- `surface`: Building envelope or equipment surface
- `supply_air`: HVAC supply air temperature
- `return_air`: HVAC return air temperature
- `outdoor`: External ambient conditions

**Primary Unit**: Celsius (°C)
**Accuracy Requirement**: Must specify ±X°C accuracy
**Calibration**: Sensors must be calibrated annually minimum

### 4.2 Air Quality

**Schema URI**: `https://wia.org/bems/v1/schemas/air-quality.json`

**Parameters**:
- `co2_ppm`: Carbon dioxide concentration (parts per million)
- `relative_humidity_percent`: Relative humidity (0-100%)
- `pm25_ugm3`: Particulate matter 2.5μm (micrograms per cubic meter)
- `pm10_ugm3`: Particulate matter 10μm
- `tvoc_ppb`: Total volatile organic compounds (parts per billion)

**Thresholds**: Each parameter includes warning and critical threshold values

---

## 5. Occupancy Schema

**Schema URI**: `https://wia.org/bems/v1/schemas/occupancy.json`

### 5.1 Privacy-Preserving Occupancy

**Detection Methods**:
- `pir_sensor`: Passive infrared (binary occupied/vacant)
- `ultrasonic`: Ultrasonic motion detection
- `co2_based`: Estimated from CO2 levels
- `camera_vision`: Computer vision (with consent)
- `wifi_device_count`: Connected device proxy

**Data Fields**:
- `count`: Number of occupants (integer or null if only binary)
- `confidence`: Statistical confidence in measurement (0.0-1.0)
- `privacy_preserved`: Boolean indicating anonymization
- `detection_method`: How occupancy was determined

---

## 6. Equipment Status Schema

### 6.1 HVAC Equipment

**Schema URI**: `https://wia.org/bems/v1/schemas/hvac-status.json`

**Equipment Types**:
- `air_handling_unit`: AHU status and performance
- `fan_coil_unit`: FCU operation
- `variable_air_volume`: VAV box status
- `chiller`: Chiller performance metrics
- `boiler`: Boiler operation and efficiency
- `heat_pump`: Heat pump status

**Standard Fields**:
- `operational_status`: running | stopped | standby | alarm
- `mode`: heating | cooling | ventilating | economizer
- `performance`: Type-specific performance metrics
- `setpoints`: Current control setpoints
- `maintenance`: Runtime hours, maintenance indicators

### 6.2 Lighting Systems

**Schema URI**: `https://wia.org/bems/v1/schemas/lighting-status.json`

**Fields**:
- `status`: on | off | dimmed
- `brightness_percent`: 0-100
- `color_temperature_k`: Kelvin temperature for tunable white
- `control_mode`: manual | scheduled | occupancy | daylight
- `power_w`: Current power consumption

---

## 7. Renewable Energy Schema

### 7.1 Solar PV Generation

**Schema URI**: `https://wia.org/bems/v1/schemas/solar-generation.json`

**Measurement Points**:
- `power_kw`: Instantaneous generation
- `energy_kwh_today`: Daily cumulative generation
- `performance_ratio`: Actual vs. theoretical (0.0-1.0)
- `irradiance_wm2`: Solar irradiance at array
- `module_temp_c`: PV module temperature

**Efficiency Metrics**:
- `inverter_efficiency`: DC to AC conversion (0.0-1.0)
- `system_efficiency`: Overall system (typically 0.15-0.22)

### 7.2 Energy Storage

**Schema URI**: `https://wia.org/bems/v1/schemas/energy-storage.json`

**State Information**:
- `state_of_charge_percent`: Current charge level (0-100)
- `state_of_health_percent`: Battery health (0-100)
- `operating_mode`: charging | discharging | standby | grid_support
- `power_kw`: Positive for charging, negative for discharging

**Performance Metrics**:
- `round_trip_efficiency`: Charge/discharge efficiency
- `cycles_total`: Cumulative charge/discharge cycles
- `throughput_kwh_total`: Lifetime energy throughput

---

## 8. Data Quality Framework

### 8.1 Quality Indicators

Every measurement must include quality indicators:

```json
{
  "quality": {
    "validation_status": "verified | estimated | questionable | invalid",
    "completeness": 0-100,
    "accuracy": numeric_value,
    "estimated": true | false,
    "confidence": 0.0-1.0
  }
}
```

### 8.2 Validation Status Definitions

- **verified**: Measured data passing all validation checks
- **estimated**: Gap-filled or calculated data
- **questionable**: Data outside normal ranges but not definitively invalid
- **invalid**: Data failed validation, should not be used

### 8.3 Missing Data Representation

Missing values are represented as `null` with explanation in quality metadata:

```json
{
  "temperature": {
    "value": null,
    "unit": "celsius"
  },
  "quality": {
    "validation_status": "sensor_failure",
    "error_code": "NO_RESPONSE",
    "error_description": "Sensor not responding to queries",
    "last_valid_measurement": "2025-01-15T12:45:00Z"
  }
}
```

---

## 9. Versioning and Extensions

### 9.1 Schema Versioning

Schemas use semantic versioning (MAJOR.MINOR.PATCH):
- **MAJOR**: Breaking changes, incompatible with previous versions
- **MINOR**: New optional fields, backward compatible
- **PATCH**: Bug fixes, clarifications

### 9.2 Extension Mechanism

Implementations may add custom fields under `x_vendor_` prefix:

```json
{
  "standard_field": "value",
  "x_acme_custom_field": "vendor specific data"
}
```

Extensions must not conflict with standard fields and should be ignored by implementations that don't recognize them.

---

## 10. Implementation Requirements

### 10.1 JSON Schema Validation

All implementations must validate data against published JSON schemas before storage or transmission. Validation libraries exist for all major programming languages.

### 10.2 Time Zone Handling

- Store all timestamps in UTC internally
- Convert to local time only for display
- Always include timezone information in ISO 8601 format
- Handle daylight saving time transitions correctly

### 10.3 Numeric Precision

Report values with precision appropriate to measurement accuracy:
- Temperature: 0.1°C typical precision
- Energy: 0.1 kWh for sub-meters, 1 kWh for whole-building
- Power: 0.1 kW typical precision

Avoid false precision (e.g., reporting 22.375849°C when accuracy is ±0.5°C).

---

## 11. Conformance and Testing

### 11.1 Conformance Levels

- **Level 1 - Basic**: Energy consumption and power demand schemas
- **Level 2 - Environmental**: Addition of temperature and air quality
- **Level 3 - Complete**: All schemas including equipment, renewables, storage

### 11.2 Test Suite

WIA provides open-source test harness for validation:
- Schema validation tests
- Data quality verification
- Boundary condition testing
- Error handling verification

---

## 12. References

- JSON Schema Specification: https://json-schema.org/
- ISO 8601 Date/Time: https://www.iso.org/iso-8601-date-and-time-format.html
- Project Haystack: https://project-haystack.org/
- ASHRAE Standard 201: Facility Smart Grid Information Model

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (Hongik Ingan) - Benefit All Humanity**

This specification is freely implementable. No licensing fees or royalties are required.


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
