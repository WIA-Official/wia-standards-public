# WIA Ocean Acidification Response Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red - ENE Category)
**Standard ID**: WIA-ENE-054

---

## Table of Contents

1. [Overview](#overview)
2. [Communication Protocols](#communication-protocols)
3. [Data Exchange Standards](#data-exchange-standards)
4. [Real-time Monitoring Protocol](#real-time-monitoring-protocol)
5. [Alert & Response Protocol](#alert-response-protocol)
6. [Quality Assurance Protocol](#quality-assurance-protocol)
7. [Security & Privacy](#security-privacy)
8. [Interoperability Standards](#interoperability-standards)

---

## Overview

### 1.1 Purpose

The WIA Ocean Acidification Response Protocol Standard defines the communication protocols, data exchange mechanisms, and operational procedures for ocean acidification monitoring systems. This ensures consistent, reliable, and secure data flow between monitoring stations, research institutions, and environmental agencies worldwide.

**Key Objectives**:
- Standardize real-time ocean data transmission
- Ensure data quality and reliability
- Enable global coordination of monitoring efforts
- Support rapid response to critical acidification events
- Facilitate interoperability with existing ocean monitoring networks

### 1.2 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (WIA-ENE-054)  │
├─────────────────────────────────────┤
│   Message Protocol (MQTT/HTTP/WS)  │
├─────────────────────────────────────┤
│   Security Layer (TLS 1.3)         │
├─────────────────────────────────────┤
│   Transport Layer (TCP/UDP)        │
├─────────────────────────────────────┤
│   Network Layer (IPv4/IPv6)        │
└─────────────────────────────────────┘
```

---

## Communication Protocols

### 2.1 Supported Protocols

| Protocol | Use Case | Port | Priority |
|----------|----------|------|----------|
| HTTPS/REST | API access, data queries | 443 | High |
| MQTT | Real-time sensor data streaming | 8883 | High |
| WebSocket | Live dashboard updates | 443 | Medium |
| FTP/SFTP | Bulk data transfer | 22 | Low |
| AMQP | Message queuing for processing | 5671 | Medium |

### 2.2 MQTT Protocol for Real-time Data

**MQTT Topics Structure**:
```
wia/ocean-acidification/v1/{region}/{station_id}/{data_type}

Examples:
wia/ocean-acidification/v1/pacific/PACIFIC-NW-001/ph
wia/ocean-acidification/v1/atlantic/ATLANTIC-001/species
wia/ocean-acidification/v1/alerts/critical
```

**MQTT Message Format**:
```json
{
  "topic": "wia/ocean-acidification/v1/pacific/PACIFIC-NW-001/ph",
  "qos": 1,
  "retain": false,
  "payload": {
    "standard": "WIA-ENE-054",
    "timestamp": "2025-01-15T10:30:00Z",
    "station_id": "PACIFIC-NW-001",
    "measurements": {
      "ph": 8.05,
      "temperature_celsius": 12.5,
      "salinity_psu": 33.5
    }
  }
}
```

**QoS Levels**:
- QoS 0: At most once (routine monitoring)
- QoS 1: At least once (important measurements)
- QoS 2: Exactly once (critical alerts)

### 2.3 WebSocket Protocol

**Connection Endpoint**:
```
wss://realtime.wia.org/ocean-acidification/v1/stream
```

**Subscription Message**:
```json
{
  "action": "subscribe",
  "channels": [
    "station:PACIFIC-NW-001",
    "alerts:critical",
    "predictions:daily"
  ],
  "auth_token": "wia_live_abc123"
}
```

**Server Push Message**:
```json
{
  "channel": "station:PACIFIC-NW-001",
  "event": "ph_update",
  "data": {
    "timestamp": "2025-01-15T10:30:00Z",
    "ph": 8.05,
    "trend": "declining"
  }
}
```

---

## Data Exchange Standards

### 3.1 Data Formats

**Primary**: JSON (application/json)
**Secondary**: XML, CSV, NetCDF, HDF5

**JSON-LD for Linked Data**:
```json
{
  "@context": "https://wia.org/contexts/ocean-acidification/v1",
  "@type": "OceanMeasurement",
  "@id": "https://data.wia.org/measurements/abc123",
  "standard": "WIA-ENE-054",
  "station": {
    "@id": "https://data.wia.org/stations/PACIFIC-NW-001",
    "@type": "MonitoringStation"
  },
  "measurement": {
    "ph": 8.05,
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 3.2 NetCDF Format (Scientific Data)

```python
# NetCDF structure for ocean acidification data
dimensions:
    time = UNLIMITED
    depth = 100
    lat = 1
    lon = 1

variables:
    double time(time)
        units = "seconds since 1970-01-01 00:00:00"
        standard_name = "time"

    float ph(time, depth)
        units = "pH_scale"
        standard_name = "sea_water_ph_reported_on_total_scale"
        precision = 0.001

    float temperature(time, depth)
        units = "celsius"
        standard_name = "sea_water_temperature"

    float carbonate_ion(time, depth)
        units = "umol/kg"
        standard_name = "mole_concentration_of_carbonate_ion_in_sea_water"

global attributes:
    Conventions = "CF-1.8"
    standard = "WIA-ENE-054"
    institution = "WIA Ocean Monitoring Network"
    philosophy = "弘益人間 - Benefit All Humanity"
```

### 3.3 Data Compression

**Supported Compression**:
- gzip (HTTP Content-Encoding)
- brotli (HTTP Content-Encoding)
- zlib (NetCDF internal)

**Request with compression**:
```http
GET /api/v1/ocean/ph/historical
Accept-Encoding: gzip, br
Authorization: Bearer wia_live_abc123
```

---

## Real-time Monitoring Protocol

### 4.1 Sensor Data Transmission

**Transmission Frequency**:
| Parameter | Frequency | Protocol |
|-----------|-----------|----------|
| pH | 1 minute | MQTT |
| Temperature | 1 minute | MQTT |
| Salinity | 5 minutes | MQTT |
| Carbonate chemistry | 1 hour | HTTPS |
| Species observations | Daily | HTTPS |

**Data Packet Format**:
```json
{
  "header": {
    "standard": "WIA-ENE-054",
    "version": "1.0.0",
    "packet_id": "pkt_abc123",
    "timestamp": "2025-01-15T10:30:00Z",
    "sequence_number": 12345
  },
  "source": {
    "station_id": "PACIFIC-NW-001",
    "sensor_id": "SENSOR-PH-001",
    "location": {
      "latitude": 47.6062,
      "longitude": -122.3321,
      "depth_meters": 50
    }
  },
  "measurements": {
    "ph": 8.05,
    "temperature_celsius": 12.5,
    "quality_flag": 0
  },
  "metadata": {
    "calibration_date": "2025-01-01",
    "measurement_method": "spectrophotometric"
  }
}
```

### 4.2 Heartbeat Protocol

**Heartbeat Interval**: 60 seconds

```json
{
  "type": "heartbeat",
  "station_id": "PACIFIC-NW-001",
  "timestamp": "2025-01-15T10:30:00Z",
  "status": "online",
  "sensors": {
    "ph_sensor": "active",
    "temperature_sensor": "active",
    "salinity_sensor": "active"
  },
  "battery_level_percent": 85,
  "last_calibration": "2025-01-01T00:00:00Z"
}
```

### 4.3 Time Synchronization

**Protocol**: NTP (Network Time Protocol)
**NTP Servers**:
- time.wia.org
- time.nist.gov

**Time Accuracy Requirement**: ±1 second

---

## Alert & Response Protocol

### 5.1 Alert Severity Levels

| Level | pH Range | Response Time | Actions |
|-------|----------|---------------|---------|
| INFO | 8.1 - 8.3 | N/A | Log only |
| WARNING | 8.0 - 8.1 | 1 hour | Notify researchers |
| CRITICAL | 7.9 - 8.0 | 15 minutes | Alert agencies |
| EMERGENCY | < 7.9 | Immediate | Emergency response |

### 5.2 Alert Message Format

```json
{
  "alert_type": "ocean_acidification_alert",
  "standard": "WIA-ENE-054",
  "alert_id": "alert_abc123",
  "severity": "CRITICAL",
  "timestamp": "2025-01-15T10:30:00Z",
  "station_id": "PACIFIC-NW-001",
  "location": {
    "latitude": 47.6062,
    "longitude": -122.3321,
    "ocean_zone": "coastal"
  },
  "trigger": {
    "parameter": "ph",
    "current_value": 7.88,
    "threshold": 7.9,
    "duration_minutes": 120
  },
  "ecosystem_impact": {
    "species_at_risk": ["Crassostrea gigas", "Mytilus edulis"],
    "estimated_damage": "high"
  },
  "recommended_actions": [
    "Deploy emergency pH buffers",
    "Increase monitoring frequency",
    "Notify local fisheries",
    "Activate mitigation protocols"
  ]
}
```

### 5.3 Alert Distribution Channels

```json
{
  "distribution": {
    "webhook": {
      "url": "https://emergency.oceanagency.gov/alerts",
      "method": "POST",
      "retry_attempts": 3,
      "timeout_seconds": 30
    },
    "email": {
      "recipients": [
        "ocean-team@noaa.gov",
        "research@woods-hole.edu"
      ],
      "priority": "high"
    },
    "sms": {
      "numbers": ["+1-555-0100"],
      "provider": "twilio"
    },
    "push_notification": {
      "apps": ["WIA Ocean Monitor"],
      "devices": ["mobile", "desktop"]
    }
  }
}
```

---

## Quality Assurance Protocol

### 6.1 Data Quality Checks

**Automated Quality Control**:

```python
# Pseudo-code for quality checks
def quality_control_ph(measurement):
    checks = []

    # Range check
    if not (6.0 <= measurement.ph <= 9.0):
        checks.append({"flag": 3, "reason": "pH out of valid range"})

    # Rate of change check
    if abs(measurement.ph - previous_ph) > 0.5:
        checks.append({"flag": 2, "reason": "Excessive pH change rate"})

    # Temperature consistency
    if not (-2 <= measurement.temperature <= 40):
        checks.append({"flag": 3, "reason": "Temperature out of range"})

    # Salinity consistency
    if not (0 <= measurement.salinity <= 42):
        checks.append({"flag": 3, "reason": "Salinity out of range"})

    return assign_quality_flag(checks)
```

### 6.2 Calibration Protocol

**Calibration Frequency**:
- pH sensors: Weekly
- Temperature sensors: Monthly
- Salinity sensors: Monthly

**Calibration Record Format**:
```json
{
  "calibration": {
    "calibration_id": "cal_abc123",
    "sensor_id": "SENSOR-PH-001",
    "timestamp": "2025-01-01T08:00:00Z",
    "technician": "John Doe",
    "standards_used": [
      {"ph": 4.00, "lot": "CAL-2024-001"},
      {"ph": 7.00, "lot": "CAL-2024-002"},
      {"ph": 10.00, "lot": "CAL-2024-003"}
    ],
    "calibration_curve": {
      "slope": -58.2,
      "intercept": 414.1,
      "r_squared": 0.9998
    },
    "pass": true
  }
}
```

### 6.3 Data Validation Workflow

```
┌──────────────┐
│ Data Arrives │
└──────┬───────┘
       │
       v
┌──────────────────┐
│ Format Validation│ ──> FAIL ──> Reject & Log
└──────┬───────────┘
       │ PASS
       v
┌──────────────────┐
│ Range Checks     │ ──> FLAG ──> Mark for Review
└──────┬───────────┘
       │ PASS
       v
┌──────────────────┐
│ Consistency Checks│ ──> FLAG ──> Mark for Review
└──────┬───────────┘
       │ PASS
       v
┌──────────────────┐
│ Accept & Store   │
└──────────────────┘
```

---

## Security & Privacy

### 7.1 Encryption Standards

**Data in Transit**:
- TLS 1.3 minimum
- HTTPS for all API endpoints
- MQTT over TLS (port 8883)
- WebSocket Secure (WSS)

**Data at Rest**:
- AES-256 encryption
- Encrypted database fields for sensitive data
- Secure key management (AWS KMS, Azure Key Vault)

### 7.2 Authentication & Authorization

**API Authentication Methods**:
1. API Key (for programmatic access)
2. OAuth 2.0 (for institutional access)
3. JWT tokens (for web applications)

**Authorization Scopes**:
```json
{
  "scopes": {
    "ocean.read": "Read ocean monitoring data",
    "ocean.write": "Submit measurement data",
    "ocean.alerts.manage": "Configure and manage alerts",
    "ocean.admin": "Full administrative access"
  }
}
```

### 7.3 Data Privacy

**Personally Identifiable Information (PII)**:
- Researcher names and contact info
- Institutional affiliations
- Location data (if near sensitive areas)

**Privacy Measures**:
- PII encryption
- Access control lists (ACLs)
- Data anonymization for public datasets
- GDPR compliance for EU users

---

## Interoperability Standards

### 8.1 Integration with Global Networks

**Supported Standards**:

| Network | Standard | Protocol |
|---------|----------|----------|
| GOOS | IOC/GOOS | HTTPS/OPeNDAP |
| Argo | Argo NetCDF | FTP/HTTPS |
| GLODAP | CF-1.8 | OPeNDAP |
| SOCAT | SOCAT v2023 | HTTPS |

### 8.2 Data Export Formats

**Supported Export Formats**:
```json
{
  "export_formats": [
    {
      "format": "json",
      "mime_type": "application/json",
      "use_case": "API integration"
    },
    {
      "format": "netcdf",
      "mime_type": "application/x-netcdf",
      "use_case": "Scientific analysis"
    },
    {
      "format": "csv",
      "mime_type": "text/csv",
      "use_case": "Spreadsheet analysis"
    },
    {
      "format": "geojson",
      "mime_type": "application/geo+json",
      "use_case": "GIS mapping"
    }
  ]
}
```

### 8.3 Cross-Standard Mapping

**WIA-ENE-054 ↔ GOOS Mapping**:
```json
{
  "field_mapping": {
    "WIA-ENE-054.measurements.ph": "GOOS.sea_water_ph_reported_on_total_scale",
    "WIA-ENE-054.measurements.temperature_celsius": "GOOS.sea_water_temperature",
    "WIA-ENE-054.measurements.salinity_psu": "GOOS.sea_water_practical_salinity",
    "WIA-ENE-054.location.latitude": "GOOS.latitude",
    "WIA-ENE-054.location.longitude": "GOOS.longitude"
  }
}
```

---

**Philosophy**: 弘益人間 (弘益人間) - Benefit All Humanity

© 2025 WIA Standards Committee | MIT License
