# WIA-ENE-060: Wetland Conservation
## PHASE 3 - PROTOCOL SPECIFICATION

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25

---

## Overview

This specification defines the communication protocols, data exchange standards, and interoperability mechanisms for wetland conservation monitoring systems. It ensures seamless integration between field sensors, monitoring stations, databases, and reporting platforms.

## Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (HTTPS/WSS)    │
├─────────────────────────────────────┤
│   Data Format Layer (JSON/GeoJSON) │
├─────────────────────────────────────┤
│   Security Layer (TLS 1.3 / OAuth) │
├─────────────────────────────────────┤
│   Transport Layer (TCP/UDP/MQTT)   │
└─────────────────────────────────────┘
```

## 1. Real-Time Monitoring Protocol

### MQTT Topics Structure

```
wia/wetland/{wetland_id}/water-quality
wia/wetland/{wetland_id}/biodiversity
wia/wetland/{wetland_id}/vegetation
wia/wetland/{wetland_id}/hydrology
wia/wetland/{wetland_id}/alerts
wia/wetland/{wetland_id}/status
```

### Message Format

```json
{
  "protocol_version": "1.0.0",
  "wetland_id": "WL-OKAV-2025",
  "sensor_id": "SENSOR-WQ-001",
  "timestamp": "2025-12-25T10:00:00Z",
  "message_type": "water_quality_reading",
  "data": {
    "ph": 7.2,
    "dissolved_oxygen_mg_l": 8.5,
    "temperature_celsius": 24.5
  },
  "metadata": {
    "battery_level": 85,
    "signal_strength": -65,
    "location": {
      "latitude": -19.2833,
      "longitude": 22.7333
    }
  },
  "checksum": "sha256:abc123..."
}
```

### QoS Levels

- **QoS 0** (At most once): Non-critical sensor readings
- **QoS 1** (At least once): Standard monitoring data
- **QoS 2** (Exactly once): Critical alerts and alarms

## 2. Sensor Integration Protocol

### Sensor Registration

```http
POST /api/v1/sensors/register
```

**Request:**
```json
{
  "sensor_type": "water_quality",
  "manufacturer": "AquaSense Pro",
  "model": "AS-2000",
  "serial_number": "AS2000-12345",
  "wetland_id": "WL-OKAV-2025",
  "location": {
    "latitude": -19.2833,
    "longitude": 22.7333,
    "elevation_m": 945
  },
  "capabilities": {
    "parameters": ["ph", "dissolved_oxygen", "temperature", "turbidity"],
    "sampling_interval_seconds": 300,
    "battery_powered": true
  },
  "calibration_date": "2025-12-01"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "sensor_id": "SENSOR-WQ-001",
    "mqtt_credentials": {
      "client_id": "wia-sensor-wq-001",
      "username": "sensor_wq_001",
      "password": "generated_secure_token",
      "broker": "mqtt.wia.org:8883"
    },
    "publishing_topics": [
      "wia/wetland/WL-OKAV-2025/water-quality"
    ],
    "command_topic": "wia/wetland/WL-OKAV-2025/sensors/SENSOR-WQ-001/commands"
  }
}
```

### Sensor Heartbeat

Sensors must send heartbeat messages every 5 minutes:

```json
{
  "sensor_id": "SENSOR-WQ-001",
  "timestamp": "2025-12-25T10:00:00Z",
  "status": "operational",
  "battery_level": 85,
  "last_reading": "2025-12-25T09:55:00Z",
  "next_calibration": "2026-06-01"
}
```

### Remote Commands

Commands sent to sensors via MQTT:

```json
{
  "command_id": "CMD-20251225-001",
  "command_type": "configure",
  "timestamp": "2025-12-25T10:00:00Z",
  "parameters": {
    "sampling_interval_seconds": 600,
    "enable_alerts": true,
    "alert_thresholds": {
      "ph_min": 6.5,
      "ph_max": 8.5,
      "do_min": 5.0
    }
  }
}
```

## 3. Data Synchronization Protocol

### Offline Data Collection

For areas with intermittent connectivity:

```json
{
  "batch_id": "BATCH-20251225-001",
  "sensor_id": "SENSOR-WQ-001",
  "wetland_id": "WL-OKAV-2025",
  "collection_period": {
    "start": "2025-12-24T00:00:00Z",
    "end": "2025-12-25T00:00:00Z"
  },
  "readings": [
    {
      "timestamp": "2025-12-24T00:05:00Z",
      "ph": 7.1,
      "dissolved_oxygen_mg_l": 8.3,
      "temperature_celsius": 23.8
    },
    {
      "timestamp": "2025-12-24T00:10:00Z",
      "ph": 7.2,
      "dissolved_oxygen_mg_l": 8.4,
      "temperature_celsius": 23.9
    }
  ],
  "checksum": "sha256:def456..."
}
```

### Conflict Resolution

When multiple sources provide conflicting data:

1. **Priority Order:**
   - Calibrated sensors (highest)
   - Manual observations by certified observers
   - Community reports (lowest)

2. **Timestamp Resolution:**
   - Use server-side timestamp if client timestamp differs by >5 minutes
   - Log timestamp conflicts

3. **Value Validation:**
   - Apply range checks based on wetland type
   - Flag outliers for manual review

## 4. Alert Protocol

### Alert Severity Levels

- **CRITICAL**: Immediate threat to wetland health
- **HIGH**: Significant degradation detected
- **MEDIUM**: Concerning trend identified
- **LOW**: Minor anomaly detected
- **INFO**: Informational notification

### Alert Message Format

```json
{
  "alert_id": "ALT-20251225-001",
  "wetland_id": "WL-OKAV-2025",
  "severity": "HIGH",
  "alert_type": "water_quality_degradation",
  "timestamp": "2025-12-25T10:00:00Z",
  "description": "Dissolved oxygen levels below critical threshold",
  "data": {
    "parameter": "dissolved_oxygen_mg_l",
    "current_value": 4.2,
    "threshold": 5.0,
    "normal_range": "6.0-9.0"
  },
  "location": {
    "latitude": -19.2833,
    "longitude": 22.7333,
    "zone": "northern_sector"
  },
  "recommended_actions": [
    "Investigate potential pollution sources",
    "Increase monitoring frequency",
    "Alert local environmental authorities"
  ],
  "notification_channels": ["email", "sms", "push", "webhook"]
}
```

### Alert Escalation

```
LOW → Wait 24h → Still active? → Escalate to MEDIUM
MEDIUM → Wait 6h → Still active? → Escalate to HIGH
HIGH → Immediate notification to all stakeholders
CRITICAL → Immediate notification + Emergency response
```

## 5. GIS Integration Protocol

### GeoJSON Format for Wetland Boundaries

```json
{
  "type": "Feature",
  "id": "WL-OKAV-2025",
  "geometry": {
    "type": "Polygon",
    "coordinates": [
      [
        [22.7333, -19.2833],
        [22.8000, -19.2833],
        [22.8000, -19.3500],
        [22.7333, -19.3500],
        [22.7333, -19.2833]
      ]
    ]
  },
  "properties": {
    "name": "Okavango Delta",
    "type": "riverine",
    "area_hectares": 1500000,
    "health_score": 92,
    "last_monitored": "2025-12-25T10:00:00Z"
  }
}
```

### Spatial Query Protocol

```http
POST /api/v1/wetlands/spatial-query
```

**Request:**
```json
{
  "query_type": "within_radius",
  "center": {
    "latitude": -19.2833,
    "longitude": 22.7333
  },
  "radius_km": 50,
  "filters": {
    "health_score_min": 80,
    "ramsar_site": true
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "wetlands": [
      {
        "id": "WL-OKAV-2025",
        "name": "Okavango Delta",
        "distance_km": 0,
        "health_score": 92
      }
    ],
    "total_count": 1
  }
}
```

## 6. Satellite Data Integration

### Sentinel Hub Protocol

```http
POST /api/v1/wetlands/{id}/satellite-analysis
```

**Request:**
```json
{
  "satellite": "Sentinel-2",
  "date_range": {
    "start": "2025-12-01",
    "end": "2025-12-25"
  },
  "analysis_type": "vegetation_index",
  "cloud_coverage_max": 20,
  "resolution_m": 10
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "analysis_id": "SAT-20251225-001",
    "tiles": [
      {
        "tile_id": "T33UXP",
        "date": "2025-12-20",
        "cloud_coverage": 5,
        "ndvi_mean": 0.72,
        "ndwi_mean": 0.45,
        "preview_url": "https://storage.wia.org/sat/preview.jpg"
      }
    ],
    "vegetation_change": "+2.3%",
    "water_extent_change": "-1.5%"
  }
}
```

## 7. Mobile App Protocol

### Citizen Science Observations

```http
POST /api/v1/wetlands/{id}/citizen-observations
```

**Request:**
```json
{
  "observer_id": "CITIZEN-001",
  "observation_date": "2025-12-25T08:30:00Z",
  "observation_type": "species_sighting",
  "location": {
    "latitude": -19.2833,
    "longitude": 22.7333,
    "accuracy_meters": 5
  },
  "species": {
    "common_name": "Wattled Crane",
    "scientific_name": "Grus carunculata",
    "count": 12,
    "behavior": "feeding"
  },
  "media": [
    {
      "type": "photo",
      "base64": "data:image/jpeg;base64,/9j/4AAQ...",
      "timestamp": "2025-12-25T08:30:15Z"
    }
  ],
  "notes": "Large flock feeding in shallow water"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "observation_id": "OBS-20251225-001",
    "verification_status": "pending",
    "contribution_points": 10,
    "badge_earned": null,
    "similar_observations": 3
  }
}
```

## 8. Blockchain Verification Protocol

### Verifiable Credentials

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.org/credentials/wetland/v1"
  ],
  "type": ["VerifiableCredential", "WetlandMonitoringRecord"],
  "issuer": {
    "id": "did:wia:ene-060",
    "name": "WIA Wetland Conservation Standard"
  },
  "issuanceDate": "2025-12-25T12:00:00Z",
  "credentialSubject": {
    "id": "did:wia:wetland:WL-OKAV-2025",
    "wetland_name": "Okavango Delta",
    "monitoring_period": {
      "start": "2025-01-01",
      "end": "2025-12-31"
    },
    "health_score": 92,
    "compliance": {
      "ramsar_convention": true,
      "monitoring_frequency": "monthly",
      "data_quality_score": 95
    }
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-25T12:00:00Z",
    "verificationMethod": "did:wia:ene-060#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z58DAdFfa9SkqZMVPxAQpic7ndSayn1PzZs6ZjWp1CktyGesjuTSwRdoWhAfGFCF5bppETSTojQCrfFPP2oumHKtz"
  }
}
```

### Verification Endpoint

```http
POST /api/v1/verify-credential
```

**Request:**
```json
{
  "credential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "type": ["VerifiableCredential"],
    "proof": {...}
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "verified": true,
    "issuer_verified": true,
    "signature_valid": true,
    "not_revoked": true,
    "verification_timestamp": "2025-12-25T14:00:00Z"
  }
}
```

## 9. Data Export Protocol

### Bulk Data Export

```http
GET /api/v1/wetlands/{id}/export
```

**Query Parameters:**
- `format`: csv | json | geojson | shapefile
- `start_date`: ISO 8601 date
- `end_date`: ISO 8601 date
- `data_types`: water_quality,biodiversity,vegetation
- `compression`: zip | gzip | none

**Response Headers:**
```
Content-Type: application/zip
Content-Disposition: attachment; filename="WL-OKAV-2025-export.zip"
X-Export-Id: EXP-20251225-001
X-Record-Count: 8640
```

## 10. Interoperability Standards

### Ramsar Convention Integration

```json
{
  "ramsar_integration": {
    "site_id": "RS1234",
    "data_mapping": {
      "wetland_id": "WL-OKAV-2025",
      "ramsar_criteria": ["1", "2", "3", "4"],
      "management_authority": "Government of Botswana",
      "last_ris_update": "2024-06-15",
      "sync_frequency": "quarterly"
    },
    "shared_data": [
      "area",
      "biodiversity_summary",
      "threats",
      "conservation_measures"
    ]
  }
}
```

### eBird Integration

```json
{
  "ebird_integration": {
    "hotspot_id": "L1234567",
    "sync_observations": true,
    "species_mapping": {
      "wia_format": "scientific_name",
      "ebird_format": "species_code"
    },
    "observation_sync": {
      "direction": "bidirectional",
      "frequency": "hourly"
    }
  }
}
```

## Security Protocols

### Authentication

- **OAuth 2.0** for user authentication
- **API Keys** for sensor authentication
- **JWT tokens** for session management
- **Mutual TLS** for sensor-to-server communication

### Encryption

- **TLS 1.3** for all HTTPS connections
- **AES-256** for data at rest
- **End-to-end encryption** for sensitive observations

### Data Privacy

- **Anonymization** of observer personal data
- **Location fuzzing** for endangered species (±1km)
- **GDPR compliance** for EU data subjects

## Performance Requirements

- **Latency**: <100ms for real-time sensor data
- **Throughput**: 10,000 messages/second
- **Availability**: 99.9% uptime
- **Data retention**: 10 years minimum

---

**Standard:** WIA-ENE-060
**Category:** Energy & Environment
**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
