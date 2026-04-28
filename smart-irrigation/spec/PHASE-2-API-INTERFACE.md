# WIA-AGRI-007: Smart Irrigation Standard
## Phase 2: API Interface Specification

### 2.1 Overview

Phase 2 defines the RESTful API interfaces for smart irrigation systems, enabling communication between controllers, sensors, weather services, and management platforms.

**Duration**: 4-8 months
**Key Outcome**: Standardized API endpoints and communication protocols

### 2.2 API Architecture

#### 2.2.1 Base URL Structure

```
https://api.irrigation-system.example.com/v1
```

#### 2.2.2 Authentication

All API requests require authentication using OAuth 2.0 or API keys.

```http
Authorization: Bearer {access_token}
X-API-Key: {api_key}
```

### 2.3 System Management APIs

#### 2.3.1 Get System Information

**Endpoint**: `GET /systems/{system_id}`

**Response**:
```json
{
  "system_id": "sys-12345",
  "system_name": "North Field Irrigation",
  "status": "operational",
  "total_zones": 8,
  "active_zones": 3,
  "total_area_ha": 12.5,
  "water_usage_today_m3": 45.2,
  "last_updated": "2025-01-15T14:30:00Z"
}
```

#### 2.3.2 List All Systems

**Endpoint**: `GET /systems`

**Query Parameters**:
- `status`: Filter by status (operational, maintenance, offline)
- `location`: Filter by geographic area
- `page`: Page number for pagination
- `limit`: Results per page (default: 20, max: 100)

**Response**:
```json
{
  "systems": [...],
  "total_count": 25,
  "page": 1,
  "limit": 20
}
```

#### 2.3.3 Update System Configuration

**Endpoint**: `PATCH /systems/{system_id}`

**Request Body**:
```json
{
  "system_name": "Updated Name",
  "status": "maintenance",
  "settings": {
    "auto_irrigation_enabled": true,
    "weather_adjustment_enabled": true
  }
}
```

### 2.4 Zone Management APIs

#### 2.4.1 Get Zone Details

**Endpoint**: `GET /systems/{system_id}/zones/{zone_id}`

**Response**:
```json
{
  "zone_id": "zone-1",
  "zone_name": "Tomato Field A",
  "area_ha": 1.5,
  "crop_type": "tomato",
  "soil_type": "loam",
  "valve_status": "closed",
  "soil_moisture_percent": 35.2,
  "last_irrigated": "2025-01-15T06:00:00Z",
  "next_scheduled": "2025-01-17T06:00:00Z",
  "water_balance_mm": -8.5
}
```

#### 2.4.2 List Zones

**Endpoint**: `GET /systems/{system_id}/zones`

**Response**:
```json
{
  "zones": [
    {
      "zone_id": "zone-1",
      "zone_name": "Tomato Field A",
      "valve_status": "closed",
      "soil_moisture_percent": 35.2
    },
    ...
  ]
}
```

### 2.5 Irrigation Control APIs

#### 2.5.1 Start Irrigation

**Endpoint**: `POST /systems/{system_id}/zones/{zone_id}/irrigate`

**Request Body**:
```json
{
  "duration_minutes": 30,
  "target_depth_mm": 10,
  "flow_rate_percent": 100,
  "start_time": "2025-01-15T18:00:00Z",
  "priority": "normal"
}
```

**Response**:
```json
{
  "event_id": "evt-789",
  "status": "scheduled",
  "start_time": "2025-01-15T18:00:00Z",
  "estimated_end_time": "2025-01-15T18:30:00Z"
}
```

#### 2.5.2 Stop Irrigation

**Endpoint**: `POST /systems/{system_id}/zones/{zone_id}/stop`

**Response**:
```json
{
  "status": "stopped",
  "actual_duration_minutes": 25,
  "actual_volume_m3": 5.2,
  "stopped_at": "2025-01-15T18:25:00Z"
}
```

#### 2.5.3 Emergency Shutoff

**Endpoint**: `POST /systems/{system_id}/emergency-stop`

**Request Body**:
```json
{
  "reason": "Pipe burst detected",
  "stop_all_zones": true
}
```

**Response**:
```json
{
  "status": "emergency_stopped",
  "zones_stopped": ["zone-1", "zone-2", "zone-3"],
  "timestamp": "2025-01-15T14:45:00Z"
}
```

### 2.6 Sensor Data APIs

#### 2.6.1 Get Latest Sensor Readings

**Endpoint**: `GET /systems/{system_id}/zones/{zone_id}/sensors/latest`

**Response**:
```json
{
  "timestamp": "2025-01-15T14:30:00Z",
  "sensors": [
    {
      "sensor_id": "sms-001",
      "sensor_type": "soil_moisture",
      "depth_cm": 30,
      "readings": {
        "soil_moisture_percent": 35.2,
        "soil_temperature_c": 22.1
      }
    }
  ]
}
```

#### 2.6.2 Get Historical Sensor Data

**Endpoint**: `GET /systems/{system_id}/zones/{zone_id}/sensors/{sensor_id}/history`

**Query Parameters**:
- `start_date`: ISO 8601 datetime
- `end_date`: ISO 8601 datetime
- `interval`: Aggregation interval (1min, 5min, 1hour, 1day)

**Response**:
```json
{
  "sensor_id": "sms-001",
  "interval": "1hour",
  "data_points": [
    {
      "timestamp": "2025-01-15T00:00:00Z",
      "soil_moisture_percent": 32.1,
      "soil_temperature_c": 20.5
    },
    ...
  ]
}
```

### 2.7 Weather Integration APIs

#### 2.7.1 Get Current Weather

**Endpoint**: `GET /weather/current`

**Query Parameters**:
- `location_id`: System location identifier
- `lat`: Latitude (if location_id not used)
- `lon`: Longitude (if location_id not used)

**Response**:
```json
{
  "location_id": "loc-123",
  "timestamp": "2025-01-15T14:30:00Z",
  "temperature_c": 26.5,
  "humidity_percent": 55,
  "wind_speed_ms": 3.2,
  "precipitation_mm": 0,
  "solar_radiation_mj_m2": 22.5,
  "reference_et_mm": 6.2
}
```

#### 2.7.2 Get Weather Forecast

**Endpoint**: `GET /weather/forecast`

**Query Parameters**:
- `location_id`: System location identifier
- `days`: Number of days (1-10)

**Response**:
```json
{
  "location_id": "loc-123",
  "forecast": [
    {
      "date": "2025-01-16",
      "temperature_high_c": 32,
      "temperature_low_c": 19,
      "precipitation_probability": 10,
      "precipitation_mm": 0,
      "reference_et_mm": 6.5
    },
    ...
  ]
}
```

### 2.8 ET Calculation APIs

#### 2.8.1 Calculate Reference ET

**Endpoint**: `POST /calculate/et-reference`

**Request Body**:
```json
{
  "date": "2025-01-15",
  "location": {
    "latitude": 38.5,
    "longitude": -121.5,
    "elevation": 15
  },
  "weather": {
    "temperature_avg_c": 25,
    "temperature_min_c": 18,
    "temperature_max_c": 32,
    "humidity_percent": 60,
    "wind_speed_ms": 2.5,
    "solar_radiation_mj_m2": 25
  },
  "method": "penman_monteith"
}
```

**Response**:
```json
{
  "date": "2025-01-15",
  "reference_et_mm": 6.24,
  "method": "penman_monteith",
  "calculation_details": {
    "delta": 0.189,
    "gamma": 0.067,
    "saturation_vapor_pressure_kpa": 3.17,
    "actual_vapor_pressure_kpa": 1.90
  }
}
```

#### 2.8.2 Calculate Crop Water Requirement

**Endpoint**: `POST /calculate/crop-water-requirement`

**Request Body**:
```json
{
  "reference_et_mm": 6.24,
  "crop_coefficient": 1.0,
  "area_ha": 1.5,
  "irrigation_efficiency": 0.85
}
```

**Response**:
```json
{
  "crop_et_mm": 6.24,
  "irrigation_requirement_mm": 7.34,
  "total_volume_m3": 110.1,
  "recommended_duration_hours": 2.5
}
```

### 2.9 Scheduling APIs

#### 2.9.1 Get Irrigation Schedule

**Endpoint**: `GET /systems/{system_id}/schedule`

**Query Parameters**:
- `start_date`: ISO 8601 date
- `end_date`: ISO 8601 date

**Response**:
```json
{
  "schedule": [
    {
      "schedule_id": "sch-456",
      "zone_id": "zone-1",
      "start_time": "2025-01-16T06:00:00Z",
      "duration_minutes": 45,
      "status": "pending",
      "trigger": "scheduled"
    },
    ...
  ]
}
```

#### 2.9.2 Create Schedule

**Endpoint**: `POST /systems/{system_id}/schedule`

**Request Body**:
```json
{
  "zone_id": "zone-1",
  "recurrence": "daily",
  "start_time": "06:00:00",
  "duration_minutes": 45,
  "days_of_week": [1, 3, 5],
  "enabled": true,
  "weather_adjustment": true
}
```

### 2.10 Reporting APIs

#### 2.10.1 Get Water Usage Report

**Endpoint**: `GET /systems/{system_id}/reports/water-usage`

**Query Parameters**:
- `start_date`: ISO 8601 date
- `end_date`: ISO 8601 date
- `granularity`: daily, weekly, monthly

**Response**:
```json
{
  "report_period": {
    "start_date": "2025-01-01",
    "end_date": "2025-01-15"
  },
  "total_water_used_m3": 2450.5,
  "cost_estimate_usd": 1225.25,
  "zones": [
    {
      "zone_id": "zone-1",
      "water_used_m3": 450.2,
      "irrigation_events": 15,
      "efficiency_percent": 87.5
    },
    ...
  ]
}
```

### 2.11 Alert & Notification APIs

#### 2.11.1 Get Active Alerts

**Endpoint**: `GET /systems/{system_id}/alerts`

**Query Parameters**:
- `severity`: info, warning, critical, emergency
- `status`: active, acknowledged, resolved

**Response**:
```json
{
  "alerts": [
    {
      "alert_id": "alert-789",
      "severity": "warning",
      "alert_type": "low_pressure",
      "zone_id": "zone-2",
      "message": "Pressure dropped below threshold",
      "timestamp": "2025-01-15T14:25:00Z",
      "acknowledged": false
    },
    ...
  ]
}
```

#### 2.11.2 Acknowledge Alert

**Endpoint**: `POST /systems/{system_id}/alerts/{alert_id}/acknowledge`

**Request Body**:
```json
{
  "acknowledged_by": "user-123",
  "notes": "Investigating pressure issue"
}
```

### 2.12 Webhook APIs

#### 2.12.1 Register Webhook

**Endpoint**: `POST /webhooks`

**Request Body**:
```json
{
  "url": "https://your-server.com/irrigation-webhook",
  "events": ["irrigation_started", "irrigation_completed", "alert_created"],
  "secret": "webhook_secret_key"
}
```

**Webhook Payload Example**:
```json
{
  "event": "irrigation_started",
  "timestamp": "2025-01-15T18:00:00Z",
  "data": {
    "event_id": "evt-789",
    "zone_id": "zone-1",
    "duration_minutes": 30
  }
}
```

### 2.13 Rate Limiting

- **Standard tier**: 100 requests/minute
- **Premium tier**: 1000 requests/minute
- **Real-time data**: Separate quota, 1 request/second per sensor

**Rate Limit Headers**:
```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1642262400
```

### 2.14 Error Handling

**Standard Error Response**:
```json
{
  "error": {
    "code": "INVALID_ZONE_ID",
    "message": "The specified zone does not exist",
    "details": {
      "zone_id": "zone-999"
    },
    "timestamp": "2025-01-15T14:30:00Z"
  }
}
```

**HTTP Status Codes**:
- 200: Success
- 201: Created
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 429: Too Many Requests
- 500: Internal Server Error

---

**Previous Phase**: [Phase 1: Data Format](PHASE-1-DATA-FORMAT.md)
**Next Phase**: [Phase 3: Protocol](PHASE-3-PROTOCOL.md)

© 2025 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity

---

## Z.1 Audit transport and observability hooks (Phase 2)

Every Phase 2 envelope SHOULD emit a structured log line at the
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
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2)

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

## Z.3 Capabilities discovery and SemVer (Phase 2)

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2)

Phase 2 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2)

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

## Z.6 Supply-chain envelope per SLSA (Phase 2)

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
