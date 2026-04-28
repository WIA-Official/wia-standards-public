# WIA-AGRI-007: Smart Irrigation Standard
## Phase 1: Data Format Specification

### 1.1 Overview

Phase 1 establishes the foundational data formats and structures for smart irrigation systems. This phase defines how irrigation data is represented, stored, and exchanged between components.

**Duration**: 3-6 months
**Key Outcome**: Standardized data schemas for irrigation system information

### 1.2 Core Data Structures

#### 1.2.1 Irrigation System Schema

```json
{
  "system_id": "string (UUID)",
  "system_name": "string",
  "installation_date": "ISO 8601 datetime",
  "location": {
    "latitude": "number (degrees)",
    "longitude": "number (degrees)",
    "elevation": "number (meters)",
    "timezone": "string (IANA)"
  },
  "irrigation_type": "enum [drip, sprinkler, micro, subsurface, pivot]",
  "total_area_ha": "number",
  "zones": ["array of Zone objects"],
  "total_flow_rate_lpm": "number",
  "water_source": "enum [well, surface, municipal, reclaimed, rainwater]",
  "system_status": "enum [operational, maintenance, offline]",
  "last_updated": "ISO 8601 datetime"
}
```

#### 1.2.2 Zone Configuration

```json
{
  "zone_id": "string (unique identifier)",
  "zone_name": "string",
  "area_ha": "number",
  "crop_type": "string",
  "soil_type": "enum [clay, loam, sand, silt, mixed]",
  "emitter_type": "string",
  "emitter_spacing_m": "number",
  "emitter_flow_rate_lph": "number",
  "pressure_kpa": "number",
  "valve_id": "string",
  "valve_status": "enum [open, closed, modulating]",
  "flow_rate_lpm": "number",
  "last_irrigated": "ISO 8601 datetime",
  "next_scheduled": "ISO 8601 datetime"
}
```

#### 1.2.3 Sensor Data Format

```json
{
  "sensor_id": "string (unique identifier)",
  "sensor_type": "enum [soil_moisture, flow_meter, pressure, temperature, humidity]",
  "zone_id": "string (reference to Zone)",
  "timestamp": "ISO 8601 datetime",
  "readings": {
    "soil_moisture_percent": "number (0-100)",
    "soil_temperature_c": "number",
    "volumetric_water_content": "number (0-1)",
    "tension_kpa": "number",
    "salinity_ecw": "number (electrical conductivity)"
  },
  "battery_level": "number (0-100)",
  "signal_strength": "number (-120 to 0 dBm)",
  "calibration_date": "ISO 8601 date"
}
```

#### 1.2.4 Flow Meter Data

```json
{
  "meter_id": "string (unique identifier)",
  "zone_id": "string",
  "timestamp": "ISO 8601 datetime",
  "flow_rate_lpm": "number",
  "total_volume_m3": "number (cumulative)",
  "pressure_kpa": "number",
  "valve_position": "number (0-100 percent)",
  "alert_status": "enum [normal, low_pressure, high_pressure, leak_detected, no_flow]"
}
```

### 1.3 Water Balance Data

#### 1.3.1 ET Data Structure

```json
{
  "date": "ISO 8601 date",
  "zone_id": "string",
  "reference_et_mm": "number (ETo)",
  "crop_coefficient": "number (Kc)",
  "crop_et_mm": "number (ETc = ETo × Kc)",
  "effective_rainfall_mm": "number",
  "irrigation_applied_mm": "number",
  "soil_moisture_depletion_mm": "number",
  "calculation_method": "enum [penman_monteith, hargreaves, priestley_taylor]"
}
```

#### 1.3.2 Irrigation Event

```json
{
  "event_id": "string (UUID)",
  "zone_id": "string",
  "start_time": "ISO 8601 datetime",
  "end_time": "ISO 8601 datetime",
  "duration_minutes": "number",
  "target_depth_mm": "number",
  "actual_depth_mm": "number",
  "flow_rate_lpm": "number",
  "total_volume_m3": "number",
  "uniformity_coefficient": "number (0-1)",
  "efficiency_percent": "number (0-100)",
  "trigger_source": "enum [scheduled, manual, soil_sensor, weather, system]",
  "notes": "string (optional)"
}
```

### 1.4 Weather Data Format

#### 1.4.1 Current Weather

```json
{
  "location_id": "string",
  "timestamp": "ISO 8601 datetime",
  "temperature_c": "number",
  "temperature_min_c": "number (24h)",
  "temperature_max_c": "number (24h)",
  "humidity_percent": "number (0-100)",
  "wind_speed_ms": "number",
  "wind_direction_degrees": "number (0-360)",
  "solar_radiation_mj_m2": "number",
  "precipitation_mm": "number (cumulative)",
  "atmospheric_pressure_kpa": "number",
  "dew_point_c": "number"
}
```

#### 1.4.2 Weather Forecast

```json
{
  "location_id": "string",
  "forecast_date": "ISO 8601 date",
  "forecast_hours": "number (24, 48, 72, etc.)",
  "temperature_high_c": "number",
  "temperature_low_c": "number",
  "precipitation_probability": "number (0-100)",
  "precipitation_amount_mm": "number",
  "wind_speed_avg_ms": "number",
  "humidity_avg_percent": "number",
  "conditions": "string (description)",
  "reference_et_mm": "number (predicted)"
}
```

### 1.5 System Alerts & Diagnostics

#### 1.5.1 Alert Structure

```json
{
  "alert_id": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "severity": "enum [info, warning, critical, emergency]",
  "alert_type": "enum [leak, pressure_drop, sensor_malfunction, valve_stuck, low_battery, communication_loss]",
  "zone_id": "string (if zone-specific)",
  "device_id": "string",
  "message": "string (human-readable)",
  "recommended_action": "string",
  "acknowledged": "boolean",
  "resolved": "boolean",
  "resolved_timestamp": "ISO 8601 datetime (if resolved)"
}
```

### 1.6 Water Usage Reporting

#### 1.6.1 Daily Water Report

```json
{
  "date": "ISO 8601 date",
  "system_id": "string",
  "total_water_used_m3": "number",
  "zones": [
    {
      "zone_id": "string",
      "water_used_m3": "number",
      "irrigation_events": "number (count)",
      "duration_minutes": "number (total)",
      "efficiency_percent": "number"
    }
  ],
  "cost_estimate_usd": "number (optional)",
  "water_saved_vs_conventional_m3": "number",
  "savings_percent": "number"
}
```

### 1.7 Data Quality & Validation

#### 1.7.1 Validation Rules

- **Timestamps**: Must be in ISO 8601 format (UTC or with timezone offset)
- **Sensor Readings**: Must be within physically plausible ranges
  - Soil moisture: 0-100%
  - Temperature: -40°C to +60°C
  - Pressure: 0-1000 kPa
  - Flow rate: 0-10000 L/min
- **Data Integrity**: All foreign keys (zone_id, system_id) must reference existing entities
- **Temporal Consistency**: end_time must be after start_time
- **Numerical Precision**:
  - Volumes: 2 decimal places (m³)
  - Rates: 1 decimal place (L/min)
  - Percentages: 1 decimal place

#### 1.7.2 Data Retention

- **Real-time data**: Retained for 7 days at 1-minute intervals
- **Hourly aggregates**: Retained for 90 days
- **Daily summaries**: Retained for 5 years
- **Event logs**: Retained for 3 years
- **Alerts**: Retained for 2 years

### 1.8 File Formats

#### 1.8.1 Supported Formats

- **JSON**: Primary format for real-time data exchange
- **CSV**: For bulk data export and reporting
- **XML**: For legacy system integration (optional)
- **Binary**: For high-frequency sensor data (custom format)

#### 1.8.2 CSV Export Format

```csv
timestamp,zone_id,flow_rate_lpm,pressure_kpa,soil_moisture_percent,temperature_c,irrigation_status
2025-01-15T08:00:00Z,zone-1,125.5,250,32.5,22.1,active
2025-01-15T08:01:00Z,zone-1,124.8,248,32.6,22.1,active
```

### 1.9 Metadata Standards

#### 1.9.1 System Metadata

```json
{
  "metadata_version": "1.0",
  "schema_version": "WIA-AGRI-007-v1.0",
  "created_by": "string (organization/person)",
  "created_date": "ISO 8601 date",
  "last_modified": "ISO 8601 datetime",
  "data_classification": "enum [public, internal, confidential]",
  "geographic_coverage": "string (region/country)",
  "language": "string (ISO 639-1 code)",
  "units_system": "enum [metric, imperial]"
}
```

### 1.10 Implementation Checklist

- [ ] Define all data schemas in JSON Schema format
- [ ] Implement validation rules for all data types
- [ ] Create sample datasets for testing
- [ ] Document all field definitions and units
- [ ] Establish data retention policies
- [ ] Implement data quality checks
- [ ] Create data migration tools (if upgrading existing systems)
- [ ] Publish schema documentation
- [ ] Provide code examples in major programming languages
- [ ] Set up schema versioning and change management

---

**Next Phase**: [Phase 2: API Interface](PHASE-2-API-INTERFACE.md)

© 2025 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity

---

## Z.1 Audit transport and observability hooks (Phase 1)

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `smart-irrigation` and `wia.standard.phase` =
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 1)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-smart-irrigation-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1)

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1)

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 1)

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.
