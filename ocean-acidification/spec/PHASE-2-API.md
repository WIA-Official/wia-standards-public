# WIA Ocean Acidification Response API Standard
## Phase 2 Specification

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
2. [API Architecture](#api-architecture)
3. [Authentication](#authentication)
4. [Endpoints](#endpoints)
5. [pH Monitoring API](#ph-monitoring-api)
6. [Marine Species API](#marine-species-api)
7. [Prediction API](#prediction-api)
8. [Alert & Notification API](#alert-notification-api)
9. [Error Handling](#error-handling)
10. [Rate Limiting](#rate-limiting)

---

## Overview

### 1.1 Purpose

The WIA Ocean Acidification Response API provides programmatic access to ocean pH monitoring data, marine ecosystem health indicators, acidification predictions, and real-time alerts. This API enables researchers, environmental agencies, and monitoring stations to integrate ocean acidification data into their systems.

**Key Features**:
- Real-time ocean pH monitoring data access
- Marine species impact tracking
- Acidification trend predictions and forecasting
- Alert system for critical pH threshold breaches
- Integration with global ocean monitoring networks
- Historical data analysis and trend visualization

### 1.2 Base URL

```
Production: https://api.wia.org/ocean-acidification/v1
Staging: https://api-staging.wia.org/ocean-acidification/v1
```

### 1.3 Data Format

All API requests and responses use JSON format with UTF-8 encoding.

**Request Headers**:
```http
Content-Type: application/json
Accept: application/json
Authorization: Bearer <API_KEY>
X-WIA-Standard: WIA-ENE-054
```

---

## API Architecture

### 2.1 RESTful Design

The API follows REST principles:

| Method | Purpose | Idempotent |
|--------|---------|------------|
| GET | Retrieve data | Yes |
| POST | Create new records | No |
| PUT | Update existing records | Yes |
| PATCH | Partial update | No |
| DELETE | Remove records | Yes |

### 2.2 Versioning

API versioning is managed through the URL path:
```
/v1/ocean/ph/monitor
/v2/ocean/ph/monitor (future)
```

---

## Authentication

### 3.1 API Key Authentication

```http
GET /api/v1/ocean/ph/monitor
Authorization: Bearer wia_live_abc123xyz789
X-WIA-Standard: WIA-ENE-054
```

### 3.2 OAuth 2.0

For institutional access:

```bash
# Request access token
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "ocean.read ocean.write"
}
```

**Response**:
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "ocean.read ocean.write"
}
```

---

## Endpoints

### 4.1 Endpoint Overview

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/ocean/ph/monitor` | GET, POST | Real-time pH monitoring |
| `/ocean/ph/historical` | GET | Historical pH data |
| `/ocean/species/impact` | GET, POST | Marine species impact data |
| `/ocean/predictions` | GET, POST | Acidification predictions |
| `/ocean/alerts` | GET, POST, PUT | Alert management |
| `/ocean/stations` | GET | Monitoring station information |
| `/ocean/zones` | GET | Ocean zone data |

---

## pH Monitoring API

### 5.1 Get Current pH Data

**Endpoint**: `GET /api/v1/ocean/ph/monitor`

**Query Parameters**:
```
?station_id=PACIFIC-NW-001
&latitude=47.6062
&longitude=-122.3321
&radius_km=100
&start_date=2025-01-01T00:00:00Z
&end_date=2025-01-15T23:59:59Z
&limit=100
```

**Example Request**:
```bash
curl -X GET "https://api.wia.org/ocean-acidification/v1/ocean/ph/monitor?station_id=PACIFIC-NW-001&limit=10" \
  -H "Authorization: Bearer wia_live_abc123" \
  -H "X-WIA-Standard: WIA-ENE-054"
```

**Response** (200 OK):
```json
{
  "standard": "WIA-ENE-054",
  "version": "1.0.0",
  "timestamp": "2025-01-15T10:30:00Z",
  "count": 10,
  "data": [
    {
      "station_id": "PACIFIC-NW-001",
      "location": {
        "latitude": 47.6062,
        "longitude": -122.3321,
        "depth_meters": 50
      },
      "measurements": {
        "timestamp": "2025-01-15T10:00:00Z",
        "ph": 8.05,
        "temperature_celsius": 12.5,
        "salinity_psu": 33.5,
        "dissolved_co2_ppm": 410,
        "carbonate_ion_umol_kg": 185,
        "aragonite_saturation": 2.2
      },
      "quality_flag": 0
    }
  ],
  "pagination": {
    "next": "/api/v1/ocean/ph/monitor?station_id=PACIFIC-NW-001&offset=10&limit=10",
    "total": 1520
  }
}
```

### 5.2 Submit pH Measurement

**Endpoint**: `POST /api/v1/ocean/ph/monitor`

**Request Body**:
```json
{
  "station_id": "PACIFIC-NW-001",
  "location": {
    "latitude": 47.6062,
    "longitude": -122.3321,
    "depth_meters": 50,
    "ocean_zone": "coastal"
  },
  "measurements": {
    "timestamp": "2025-01-15T10:30:00Z",
    "ph": 8.05,
    "temperature_celsius": 12.5,
    "salinity_psu": 33.5,
    "dissolved_co2_ppm": 410,
    "measurement_method": "spectrophotometric",
    "sensor_id": "SENSOR-001",
    "calibration_date": "2025-01-01"
  }
}
```

**Response** (201 Created):
```json
{
  "status": "success",
  "message": "pH measurement recorded",
  "id": "measurement_abc123",
  "timestamp": "2025-01-15T10:30:00Z",
  "quality_check": "passed"
}
```

### 5.3 Get Historical Trends

**Endpoint**: `GET /api/v1/ocean/ph/historical`

**Query Parameters**:
```
?station_id=PACIFIC-NW-001
&start_date=2020-01-01
&end_date=2025-01-15
&aggregation=monthly
```

**Response** (200 OK):
```json
{
  "standard": "WIA-ENE-054",
  "station_id": "PACIFIC-NW-001",
  "period": {
    "start": "2020-01-01",
    "end": "2025-01-15"
  },
  "aggregation": "monthly",
  "data": [
    {
      "month": "2020-01",
      "avg_ph": 8.15,
      "min_ph": 8.10,
      "max_ph": 8.20,
      "std_dev": 0.02,
      "sample_count": 720
    },
    {
      "month": "2020-02",
      "avg_ph": 8.14,
      "min_ph": 8.09,
      "max_ph": 8.19,
      "std_dev": 0.02,
      "sample_count": 696
    }
  ],
  "trend": {
    "slope": -0.01,
    "direction": "declining",
    "significance": 0.001
  }
}
```

---

## Marine Species API

### 6.1 Get Species Impact Data

**Endpoint**: `GET /api/v1/ocean/species/impact`

**Query Parameters**:
```
?zone_id=CORAL-REEF-001
&species_type=coral
&start_date=2025-01-01
```

**Response** (200 OK):
```json
{
  "standard": "WIA-ENE-054",
  "zone_id": "CORAL-REEF-001",
  "species_data": [
    {
      "species_type": "coral",
      "species_name": "Acropora cervicornis",
      "observations": {
        "timestamp": "2025-01-15T10:00:00Z",
        "population_density_per_km2": 800,
        "calcification_rate_change_percent": -18.5,
        "vulnerability_score": 8,
        "health_status": "poor",
        "bleaching_events_count": 3
      }
    }
  ],
  "ecosystem_health": {
    "biodiversity_index": 0.68,
    "coral_coverage_percent": 35,
    "trend": "declining"
  }
}
```

### 6.2 Submit Species Observation

**Endpoint**: `POST /api/v1/ocean/species/impact`

**Request Body**:
```json
{
  "zone_id": "CORAL-REEF-001",
  "species_observation": {
    "timestamp": "2025-01-15T10:00:00Z",
    "species_type": "coral",
    "species_name": "Porites lobata",
    "population_density_per_km2": 1200,
    "calcification_rate_change_percent": -12.0,
    "health_status": "fair",
    "observer_id": "RESEARCHER-001"
  },
  "environmental_context": {
    "ph": 8.05,
    "temperature_celsius": 26.5,
    "light_intensity_umol_m2_s": 800
  }
}
```

**Response** (201 Created):
```json
{
  "status": "success",
  "observation_id": "obs_xyz789",
  "timestamp": "2025-01-15T10:00:00Z"
}
```

---

## Prediction API

### 7.1 Get pH Predictions

**Endpoint**: `GET /api/v1/ocean/predictions`

**Query Parameters**:
```
?station_id=PACIFIC-NW-001
&prediction_years=10
&scenario=rcp45
```

**Response** (200 OK):
```json
{
  "standard": "WIA-ENE-054",
  "station_id": "PACIFIC-NW-001",
  "current_state": {
    "ph": 8.05,
    "timestamp": "2025-01-15T10:00:00Z"
  },
  "predictions": {
    "scenario": "RCP4.5",
    "forecast": [
      {
        "year": 2026,
        "predicted_ph": 8.03,
        "confidence_interval_95": [8.01, 8.05]
      },
      {
        "year": 2030,
        "predicted_ph": 7.98,
        "confidence_interval_95": [7.94, 8.02]
      },
      {
        "year": 2035,
        "predicted_ph": 7.92,
        "confidence_interval_95": [7.86, 7.98]
      }
    ]
  },
  "ecosystem_impact_forecast": {
    "tipping_point_year": 2033,
    "species_at_risk_count": 18,
    "risk_level": "high"
  }
}
```

### 7.2 Generate Custom Prediction

**Endpoint**: `POST /api/v1/ocean/predictions`

**Request Body**:
```json
{
  "station_id": "PACIFIC-NW-001",
  "prediction_config": {
    "years": 20,
    "scenarios": ["rcp26", "rcp45", "rcp85"],
    "include_ecosystem_impact": true,
    "mitigation_strategies": [
      {
        "type": "co2_reduction",
        "target_percent": 30,
        "timeline_years": 10
      }
    ]
  }
}
```

**Response** (200 OK):
```json
{
  "status": "success",
  "prediction_id": "pred_abc123",
  "scenarios": {
    "rcp26": { },
    "rcp45": { },
    "rcp85": { }
  },
  "mitigation_impact": {
    "ph_improvement": 0.08,
    "species_saved": 12
  }
}
```

---

## Alert & Notification API

### 8.1 Configure Alert

**Endpoint**: `POST /api/v1/ocean/alerts`

**Request Body**:
```json
{
  "station_id": "PACIFIC-NW-001",
  "alert_config": {
    "name": "Critical pH Alert",
    "conditions": [
      {
        "parameter": "ph",
        "operator": "less_than",
        "threshold": 7.9,
        "duration_minutes": 60
      }
    ],
    "notification_methods": [
      {
        "type": "webhook",
        "url": "https://your-system.com/alerts",
        "headers": {
          "X-API-Key": "your_key"
        }
      },
      {
        "type": "email",
        "recipients": ["ocean-team@example.com"]
      }
    ]
  }
}
```

**Response** (201 Created):
```json
{
  "status": "success",
  "alert_id": "alert_xyz789",
  "active": true
}
```

### 8.2 Get Active Alerts

**Endpoint**: `GET /api/v1/ocean/alerts/active`

**Response** (200 OK):
```json
{
  "active_alerts": [
    {
      "alert_id": "alert_abc123",
      "station_id": "PACIFIC-NW-001",
      "triggered_at": "2025-01-15T10:30:00Z",
      "condition": "pH below 7.9",
      "current_value": 7.88,
      "severity": "critical",
      "notification_sent": true
    }
  ]
}
```

---

## Error Handling

### 9.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_PH_VALUE",
    "message": "pH value must be between 6.0 and 9.0",
    "details": {
      "provided_value": 12.5,
      "valid_range": [6.0, 9.0]
    },
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 9.2 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET request |
| 201 | Created | Successful POST request |
| 400 | Bad Request | Invalid input data |
| 401 | Unauthorized | Missing or invalid API key |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

### 9.3 Common Error Codes

| Error Code | Description |
|------------|-------------|
| `INVALID_PH_VALUE` | pH value out of valid range |
| `MISSING_COORDINATES` | Location coordinates required |
| `INVALID_STATION_ID` | Station ID not found |
| `QUALITY_CHECK_FAILED` | Data quality validation failed |
| `RATE_LIMIT_EXCEEDED` | Too many requests |

---

## Rate Limiting

### 10.1 Rate Limits

| Tier | Requests/Hour | Requests/Day |
|------|---------------|--------------|
| Free | 100 | 1,000 |
| Research | 1,000 | 10,000 |
| Institutional | 10,000 | 100,000 |
| Enterprise | Unlimited | Unlimited |

### 10.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1642252800
```

---

**Philosophy**: 弘益人間 (弘益人間) - Benefit All Humanity

© 2025 WIA Standards Committee | MIT License
