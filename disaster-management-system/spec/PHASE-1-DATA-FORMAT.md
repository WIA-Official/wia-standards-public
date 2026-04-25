# WIA-SOC-006 Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines the standardized data formats for disaster management systems, including disaster event representation, alert structures, resource allocation data, and incident logs. All data MUST use JSON-LD format for semantic interoperability and cross-agency compatibility.

## 2. Core Data Types

### 2.1 Disaster Event

```json
{
  "@context": "https://wiastandards.com/soc-006/v1",
  "@type": "DisasterEvent",
  "eventId": "EVT-2025-XXXX-YYYY",
  "disasterType": "tornado|earthquake|flood|wildfire|hurricane|tsunami|pandemic",
  "severity": 1-10,
  "status": "watch|advisory|warning|emergency|critical|resolved",
  "affectedArea": {
    "type": "Polygon",
    "coordinates": [[[lon, lat], ...]],
    "boundingBox": {
      "north": "float",
      "south": "float",
      "east": "float",
      "west": "float"
    }
  },
  "startTime": "ISO8601 datetime",
  "endTime": "ISO8601 datetime (optional)",
  "estimatedAffectedPopulation": "integer",
  "confirmedCasualties": "integer",
  "description": "string"
}
```

### 2.2 Alert Message

```json
{
  "@type": "AlertMessage",
  "alertId": "UUID",
  "timestamp": "ISO8601 datetime",
  "eventId": "UUID (reference to DisasterEvent)",
  "priority": "info|low|medium|high|critical",
  "category": "geo|met|safety|security|rescue|fire|health|env|transport|infra|cbrne|other",
  "urgency": "immediate|expected|future|past",
  "severity": "extreme|severe|moderate|minor|unknown",
  "certainty": "observed|likely|possible|unlikely|unknown",
  "scope": "public|restricted|private",
  "headline": "string (max 160 chars)",
  "description": "string",
  "instruction": "string",
  "web": "URL (optional)",
  "contact": "string (optional)",
  "area": {
    "description": "string",
    "polygon": [[lat, lon], ...],
    "circle": {"lat": float, "lon": float, "radius": float},
    "geocode": {"name": "string", "value": "string"}
  },
  "resources": ["UUID array of allocated resources"],
  "expiresAt": "ISO8601 datetime"
}
```

### 2.3 Resource Definition

```json
{
  "@type": "EmergencyResource",
  "resourceId": "UUID",
  "type": "medical_team|rescue_team|fire_unit|police_unit|shelter|supplies|equipment|vehicle",
  "name": "string",
  "agency": "string",
  "status": "available|deployed|standby|maintenance|offline",
  "location": {
    "lat": "float",
    "lon": "float",
    "altitude": "float (optional)",
    "address": "string"
  },
  "capacity": {
    "personnel": "integer",
    "equipment": "object",
    "supplies": "object",
    "shelter_capacity": "integer (if applicable)"
  },
  "capabilities": ["array", "of", "capability", "strings"],
  "availability": {
    "24_7": "boolean",
    "schedule": "object (optional)"
  },
  "contact": {
    "primary": "string (phone)",
    "secondary": "string (phone)",
    "radio": "string (frequency)",
    "email": "string"
  }
}
```

### 2.4 Deployment Record

```json
{
  "@type": "ResourceDeployment",
  "deploymentId": "UUID",
  "eventId": "UUID",
  "resourceId": "UUID",
  "deployedAt": "ISO8601 datetime",
  "recalledAt": "ISO8601 datetime (optional)",
  "assignedArea": "Polygon",
  "mission": "search_rescue|medical_aid|evacuation|fire_suppression|security|logistics|assessment",
  "status": "en_route|on_site|active|returning|complete",
  "personnel": [
    {
      "id": "UUID",
      "name": "string",
      "role": "string",
      "certifications": ["array"]
    }
  ],
  "progress": {
    "people_rescued": "integer",
    "area_covered": "float (km²)",
    "tasks_completed": "integer"
  }
}
```

### 2.5 Evacuation Order

```json
{
  "@type": "EvacuationOrder",
  "orderId": "UUID",
  "eventId": "UUID",
  "issuedAt": "ISO8601 datetime",
  "issuedBy": "string (authority)",
  "type": "mandatory|recommended|voluntary",
  "evacuationZone": "Polygon",
  "estimatedPopulation": "integer",
  "evacuationRoutes": [
    {
      "routeId": "UUID",
      "name": "string",
      "waypoints": [[lat, lon], ...],
      "capacity": "integer (vehicles/hour)",
      "status": "open|congested|closed"
    }
  ],
  "shelters": ["UUID array"],
  "deadline": "ISO8601 datetime (optional)",
  "instructions": "string",
  "specialNeeds": {
    "medical": "boolean",
    "pets": "boolean",
    "mobility_assistance": "boolean"
  }
}
```

## 3. Sensor Data Formats

### 3.1 Weather Sensor Data

```json
{
  "@type": "WeatherData",
  "timestamp": "ISO8601 datetime",
  "sensorId": "UUID",
  "location": {"lat": float, "lon": float},
  "temperature": "float (°C)",
  "humidity": "float (%)",
  "pressure": "float (hPa)",
  "windSpeed": "float (m/s)",
  "windDirection": "float (degrees)",
  "precipitation": "float (mm)",
  "visibility": "float (meters)",
  "conditions": "string"
}
```

### 3.2 Seismic Data

```json
{
  "@type": "SeismicData",
  "timestamp": "ISO8601 datetime",
  "stationId": "UUID",
  "location": {"lat": float, "lon": float, "depth": float},
  "magnitude": "float",
  "scale": "richter|moment|jma",
  "epicenter": {"lat": float, "lon": float},
  "depth": "float (km)",
  "intensity": "integer (1-12 MMI scale)",
  "waveforms": {
    "p_wave": "base64 data",
    "s_wave": "base64 data"
  }
}
```

### 3.3 Water Level Sensor

```json
{
  "@type": "WaterLevelData",
  "timestamp": "ISO8601 datetime",
  "sensorId": "UUID",
  "location": {"lat": float, "lon": float},
  "waterLevel": "float (meters)",
  "flowRate": "float (m³/s)",
  "floodStage": "float (meters)",
  "status": "normal|watch|minor|moderate|major",
  "trend": "rising|falling|stable"
}
```

## 4. Communication Formats

### 4.1 Inter-Agency Message

```json
{
  "@type": "InterAgencyMessage",
  "messageId": "UUID",
  "timestamp": "ISO8601 datetime",
  "fromAgency": "string",
  "toAgency": "string|array",
  "priority": "routine|priority|immediate|flash",
  "subject": "string",
  "body": "string",
  "attachments": [
    {
      "type": "document|image|video|data",
      "url": "string",
      "checksum": "SHA-256"
    }
  ],
  "requiresResponse": "boolean",
  "expiresAt": "ISO8601 datetime (optional)"
}
```

## 5. Incident Log Format

```json
{
  "@type": "IncidentLog",
  "logId": "UUID",
  "eventId": "UUID",
  "timestamp": "ISO8601 datetime",
  "severity": "info|warning|error|critical",
  "category": "alert|deployment|evacuation|casualty|damage|resource|communication",
  "actor": "string (person/system)",
  "action": "string",
  "location": {"lat": float, "lon": float},
  "details": {
    "arbitrary": "key-value pairs"
  },
  "metadata": {
    "source": "string",
    "version": "string"
  }
}
```

## 6. Damage Assessment Format

```json
{
  "@type": "DamageAssessment",
  "assessmentId": "UUID",
  "eventId": "UUID",
  "timestamp": "ISO8601 datetime",
  "assessor": "string",
  "location": {"lat": float, "lon": float},
  "structureType": "residential|commercial|industrial|infrastructure|agricultural",
  "damageLevel": "none|minor|moderate|severe|destroyed",
  "estimatedCost": "float (USD)",
  "hazards": ["structural|fire|flood|chemical|electrical|other"],
  "habitability": "safe|unsafe|unknown",
  "photosUrls": ["string array"],
  "notes": "string"
}
```

## 7. Validation Rules

1. All timestamps MUST use ISO 8601 format with UTC timezone
2. All coordinates MUST use WGS84 datum (EPSG:4326)
3. All measurements MUST use SI units
4. All UUIDs MUST be version 4
5. Required fields MUST NOT be null
6. Enum values MUST match specification exactly
7. Polygons MUST be closed (first point = last point)
8. All text fields MUST use UTF-8 encoding

## 8. Data Retention

- **Active Events**: Indefinite retention until resolved + 30 days
- **Historical Events**: 10 years minimum
- **Sensor Data**: 5 years minimum
- **Communication Logs**: 7 years minimum
- **Personal Data**: Subject to privacy regulations (GDPR, CCPA)

## 9. Extensibility

Implementations MAY add custom fields prefixed with "x_" to avoid conflicts with future standard additions.

Example:
```json
{
  "@type": "DisasterEvent",
  "eventId": "EVT-2025-001",
  "x_localCode": "vendor-specific identifier",
  "x_customField": "additional data"
}
```

## 10. Data Exchange Formats

### Supported Formats

- **Primary**: JSON-LD (application/ld+json)
- **Alternative**: GeoJSON for spatial data
- **Legacy**: CAP (Common Alerting Protocol) XML
- **Binary**: Protocol Buffers for high-volume streaming

### Compression

- GZIP compression RECOMMENDED for data > 1KB
- Brotli compression SUPPORTED for modern clients

## 11. Security Requirements

- All sensitive data MUST be encrypted at rest (AES-256)
- All data in transit MUST use TLS 1.3 or higher
- PII (Personally Identifiable Information) MUST be anonymized when shared publicly
- Access logs MUST be maintained for all data access

---

© 2025 WIA · MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for disaster-management-system is evaluated across three tiers:

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

- `wia-standards/standards/disaster-management-system/api/` — TypeScript SDK skeleton
- `wia-standards/standards/disaster-management-system/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/disaster-management-system/simulator/` — interactive browser-based simulator for the PHASE protocol

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
