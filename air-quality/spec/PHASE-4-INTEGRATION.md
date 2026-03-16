# WIA-ENE-017: Air Quality Monitoring Standard
## PHASE 4: INTEGRATION

**Document Version:** 1.0
**Status:** Active
**Last Updated:** December 25, 2025
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 4 of the WIA-ENE-017 standard defines requirements and specifications for this critical component of air quality monitoring systems. Building on the data format defined in Phase 1, this phase ensures systems can effectively communicate, integrate, and scale.

### 1.1 Objectives

- Define technical architecture and protocols
- Ensure interoperability between diverse systems
- Establish security and privacy requirements
- Provide implementation guidelines and best practices
- Enable scalable deployments from local to global networks

---

## 2. Technical Architecture

### 2.1 System Components

The WIA-ENE-017 architecture consists of four main layers:

1. **Sensor Layer:** Physical monitoring devices and data acquisition
2. **Communication Layer:** Data transmission and network protocols
3. **Processing Layer:** Real-time analytics and data management
4. **Application Layer:** User interfaces and integration endpoints

### 2.2 Communication Protocols

#### MQTT v5.0 Configuration

Topic Structure:
```
wia/airquality/{country}/{city}/{station_id}/readings
wia/airquality/{country}/{city}/{station_id}/status
wia/airquality/{country}/{city}/{station_id}/alerts
```

QoS Levels:
- QoS 0: Historical bulk data transfer
- QoS 1: Real-time measurements (default)
- QoS 2: Critical health alerts

#### HTTP/REST API

Base URL: `https://api.airquality.wia.org/v1`

Key Endpoints:
- `GET /stations` - List all monitoring stations
- `GET /stations/{id}/current` - Current readings
- `GET /stations/{id}/readings` - Historical data
- `POST /readings` - Submit new measurements
- `GET /aqi/calculate` - AQI conversion service

---

## 3. Security Requirements

### 3.1 Transport Security

- TLS 1.3 minimum for all communications
- Certificate pinning for critical infrastructure
- Perfect forward secrecy (PFS) cipher suites only

### 3.2 Authentication

Two authentication methods supported:

1. **OAuth 2.0** (recommended for interactive applications)
   - Authorization Code flow for web applications
   - Client Credentials flow for backend services
   - Token expiration: 1 hour (access), 30 days (refresh)

2. **API Keys** (for legacy systems)
   - Rotation policy: Every 90 days
   - Key format: `wia_aq_{32_character_hex}`
   - Header: `Authorization: Bearer {api_key}`

### 3.3 Authorization

Role-Based Access Control (RBAC):

| Role | Permissions |
|------|-------------|
| Public | Read current and historical data |
| Contributor | Submit sensor readings |
| Operator | Manage station metadata |
| Administrator | Full system access |

---

## 4. Data Quality Standards

### 4.1 Quality Tiers

Systems must declare their quality tier:

- **Tier 1 (Regulatory):** ±5% precision, ±10% accuracy, ≥85% completeness
- **Tier 2 (Research):** ±10% precision, ±15% accuracy, ≥75% completeness
- **Tier 3 (Informational):** ±20% precision, ±30% accuracy, ≥60% completeness
- **Tier 4 (Indicative):** ±40% precision, ±50% accuracy, ≥50% completeness

### 4.2 Validation Rules

```python
# Range validation
def validate_range(pollutant, value):
    ranges = {
        'pm25': (0, 1000), 'pm10': (0, 2000),
        'o3': (0, 500), 'no2': (0, 500),
        'so2': (0, 500), 'co': (0, 50)
    }
    min_val, max_val = ranges[pollutant]
    return min_val <= value <= max_val

# Spatial consistency
def check_spatial_consistency(station_readings, max_distance_km=10):
    nearby = get_stations_within(station_readings, max_distance_km)
    median_value = calculate_median(nearby)
    for reading in nearby:
        deviation = abs(reading.value - median_value)
        if deviation > 50:  # μg/m³ threshold
            flag_for_review(reading)
```

---

## 5. Performance Requirements

### 5.1 System Metrics

| Metric | Tier 1 | Tier 2 | Tier 3-4 |
|--------|--------|--------|----------|
| Uptime | ≥99.5% | ≥99.0% | ≥95.0% |
| Data Latency | <30s (95%ile) | <60s | <300s |
| API Response Time | <200ms | <500ms | <1s |
| Data Completeness | ≥85% | ≥75% | ≥60% |

### 5.2 Scalability

Systems must support:
- Minimum 1,000 concurrent API clients
- 100,000 sensor readings per minute ingestion
- 10 million historical records query
- Geographic distribution across multiple regions

---

## 6. Integration Interfaces

### 6.1 External System Integration

**Health Systems:**
- Real-time alerts when AQI exceeds thresholds
- FHIR-compatible patient risk assessment
- Hospital admission correlation data

**Emergency Services:**
- Automated notifications during pollution events
- Integration with emergency alert systems
- First responder dashboards

**Public Information:**
- Open data APIs for third-party applications
- RSS/Atom feeds for news aggregation
- Social media automation (Twitter, Facebook)

**Smart City Platforms:**
- Traffic signal optimization
- Building HVAC control
- Public transportation routing

### 6.2 Data Exchange Formats

Supported formats:
- JSON (default, WIA-ENE-017 schema)
- CSV (simplified for spreadsheets)
- XML (legacy system compatibility)
- NetCDF (scientific research datasets)

---

## 7. Implementation Examples

### 7.1 Sample API Request

```http
GET /api/v1/stations/WIA-AQ-KR-Seoul-001/readings?start=2025-12-25T00:00:00Z&end=2025-12-25T23:59:59Z&pollutant=pm25&interval=1h
Authorization: Bearer wia_aq_abc123...
Accept: application/json

Response: 200 OK
{
  "status": "success",
  "data": {
    "station_id": "WIA-AQ-KR-Seoul-001",
    "pollutant": "pm25",
    "interval": "1hour",
    "readings": [
      {"timestamp": "2025-12-25T00:00:00Z", "value": 28.5, "qc_flag": "valid"},
      {"timestamp": "2025-12-25T01:00:00Z", "value": 31.2, "qc_flag": "valid"}
    ],
    "metadata": {
      "count": 24,
      "average": 32.8,
      "max": 45.2,
      "min": 18.3
    }
  }
}
```

### 7.2 MQTT Publish Example

```python
import paho.mqtt.client as mqtt
import json

client = mqtt.Client()
client.username_pw_set("station_001", "secure_password")
client.connect("mqtt.airquality.wia.org", 8883, 60)

reading = {
    "station_id": "WIA-AQ-KR-Seoul-001",
    "timestamp": "2025-12-25T14:30:00Z",
    "measurements": {
        "pm25": {"value": 35.4, "unit": "ug/m3", "qc_flag": "valid"}
    }
}

client.publish(
    "wia/airquality/KR/Seoul/001/readings",
    json.dumps(reading),
    qos=1
)
```

---

## 8. Compliance and Certification

### 8.1 Certification Requirements

To achieve Phase 4 certification:

1. Technical documentation submitted
2. Security audit completed
3. Performance testing passed
4. Interoperability demonstration
5. Data quality validation (30-day co-location study for Tier 1)

### 8.2 Ongoing Compliance

- Quarterly performance reports
- Annual security audits
- Bi-annual calibration verification
- Continuous uptime monitoring

---

## 9. Glossary

- **AQI:** Air Quality Index
- **MQTT:** Message Queuing Telemetry Transport
- **OAuth:** Open Authorization framework
- **QC:** Quality Control
- **QoS:** Quality of Service
- **RBAC:** Role-Based Access Control
- **TLS:** Transport Layer Security

---

## 10. References

1. MQTT Version 5.0 (OASIS Standard)
2. OAuth 2.0 Authorization Framework (RFC 6749)
3. TLS 1.3 (RFC 8446)
4. ISO/IEC 27001:2013 Information Security Management
5. US EPA Air Quality System Technical Documentation

---

**Document Status:** ACTIVE
**Effective Date:** December 25, 2025
**Review Date:** December 25, 2027

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
