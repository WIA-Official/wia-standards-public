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
