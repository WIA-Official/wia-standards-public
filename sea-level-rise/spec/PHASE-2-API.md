# WIA-ENE-055: Sea Level Rise Response
## PHASE 2 - API Specification

**Version:** 1.0.0
**Status:** Standard
**Category:** Energy & Environment (ENE)

---

## Overview

This specification defines the RESTful API for sea level monitoring, flood risk assessment, and coastal adaptation planning. The API enables real-time data access, predictive modeling, and integration with emergency management systems.

---

## 1. API Base Configuration

### 1.1 Base URL

```
Production: https://api.wia.global/ene-055/v1
Sandbox: https://sandbox-api.wia.global/ene-055/v1
```

### 1.2 Authentication

All API requests require authentication using API keys or OAuth 2.0:

```http
Authorization: Bearer {API_KEY}
Content-Type: application/json
```

### 1.3 Rate Limiting

- **Free Tier**: 100 requests/hour
- **Standard**: 1,000 requests/hour
- **Enterprise**: Unlimited

Rate limit headers:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1735142400
```

---

## 2. Sea Level Monitoring Endpoints

### 2.1 Get Current Sea Level

**Endpoint:** `GET /sea-level/{station_id}`

**Description:** Retrieve current sea level data from a specific tide gauge station.

**Parameters:**
- `station_id` (required): Tide gauge station identifier

**Response:**
```json
{
  "status": "success",
  "data": {
    "station": "NOAA-8638610",
    "location": {
      "name": "Virginia Key, FL",
      "latitude": 25.7314,
      "longitude": -80.1606
    },
    "current_level": 1.847,
    "timestamp": "2025-12-25T14:30:00Z",
    "trend": {
      "rate": 0.0031,
      "unit": "m/year",
      "confidence": 0.95
    },
    "quality": "good"
  }
}
```

**Error Response:**
```json
{
  "status": "error",
  "error": {
    "code": "STATION_NOT_FOUND",
    "message": "Station ID not found in database"
  }
}
```

---

### 2.2 Get Historical Data

**Endpoint:** `GET /sea-level/{station_id}/history`

**Description:** Retrieve historical sea level measurements.

**Query Parameters:**
- `start_date` (required): ISO8601 start date
- `end_date` (required): ISO8601 end date
- `interval` (optional): hourly|daily|monthly (default: daily)
- `datum` (optional): MLLW|MSL|NAVD88 (default: MSL)

**Request Example:**
```http
GET /sea-level/NOAA-8638610/history?start_date=2025-01-01&end_date=2025-12-25&interval=monthly
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "station": "NOAA-8638610",
    "datum": "MSL",
    "interval": "monthly",
    "measurements": [
      {
        "timestamp": "2025-01-01T00:00:00Z",
        "mean_level": 1.823,
        "min_level": 1.654,
        "max_level": 2.012
      },
      {
        "timestamp": "2025-02-01T00:00:00Z",
        "mean_level": 1.831,
        "min_level": 1.667,
        "max_level": 2.024
      }
    ],
    "count": 12
  }
}
```

---

### 2.3 Submit Monitoring Data

**Endpoint:** `POST /sea-level/monitor`

**Description:** Submit new sea level measurements from monitoring stations.

**Request Body:**
```json
{
  "station_id": "COASTAL-2025-A",
  "measurements": [
    {
      "timestamp": "2025-12-25T14:00:00Z",
      "water_level": 1.82,
      "datum": "MLLW",
      "quality_flag": "good"
    },
    {
      "timestamp": "2025-12-25T15:00:00Z",
      "water_level": 1.85,
      "datum": "MLLW",
      "quality_flag": "good"
    }
  ],
  "metadata": {
    "instrument_type": "acoustic",
    "calibration_date": "2025-06-15"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "records_created": 2,
    "station_id": "COASTAL-2025-A",
    "message": "Measurements recorded successfully"
  }
}
```

---

## 3. Flood Risk Assessment Endpoints

### 3.1 Assess Flood Risk

**Endpoint:** `POST /flood-risk/assess`

**Description:** Calculate flood risk for a specific location based on elevation and sea level projections.

**Request Body:**
```json
{
  "location": {
    "latitude": 25.7314,
    "longitude": -80.1606,
    "elevation_msl": 1.5
  },
  "scenario": "RCP8.5",
  "timeframe": {
    "start_year": 2025,
    "end_year": 2050
  },
  "include_storm_surge": true
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "assessment_id": "RISK-2025-001",
    "location": {
      "latitude": 25.7314,
      "longitude": -80.1606,
      "elevation_msl": 1.5
    },
    "risk_metrics": {
      "risk_level": "HIGH",
      "flood_probability": 0.78,
      "projected_inundation": 2.3,
      "annual_flood_chance": 0.031,
      "first_flood_year": 2035
    },
    "recommendations": [
      {
        "type": "elevation",
        "description": "Elevate structure to minimum 3.5m MSL",
        "estimated_cost": 85000,
        "risk_reduction": 0.65
      },
      {
        "type": "flood_barrier",
        "description": "Install deployable flood barriers",
        "estimated_cost": 45000,
        "risk_reduction": 0.45
      }
    ]
  }
}
```

---

### 3.2 Batch Risk Assessment

**Endpoint:** `POST /flood-risk/batch`

**Description:** Assess flood risk for multiple locations simultaneously.

**Request Body:**
```json
{
  "locations": [
    {
      "id": "LOC-001",
      "latitude": 25.7314,
      "longitude": -80.1606,
      "elevation_msl": 1.5
    },
    {
      "id": "LOC-002",
      "latitude": 25.7850,
      "longitude": -80.1298,
      "elevation_msl": 1.2
    }
  ],
  "scenario": "RCP8.5",
  "year": 2050
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "batch_id": "BATCH-2025-001",
    "results": [
      {
        "location_id": "LOC-001",
        "risk_level": "HIGH",
        "flood_probability": 0.78
      },
      {
        "location_id": "LOC-002",
        "risk_level": "EXTREME",
        "flood_probability": 0.92
      }
    ],
    "summary": {
      "total_assessed": 2,
      "high_risk_count": 1,
      "extreme_risk_count": 1
    }
  }
}
```

---

## 4. Sea Level Projection Endpoints

### 4.1 Get Projections

**Endpoint:** `GET /projections/{region}`

**Description:** Retrieve sea level rise projections for a specific region.

**Query Parameters:**
- `scenario` (optional): RCP2.6|RCP4.5|RCP8.5 (default: all)
- `years` (optional): Comma-separated list of years

**Request Example:**
```http
GET /projections/southeast-florida?scenario=RCP8.5&years=2030,2050,2100
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "region": "Southeast Florida",
    "baseline_year": 2020,
    "scenario": "RCP8.5",
    "projections": [
      {
        "year": 2030,
        "sea_level_rise_m": 0.12,
        "confidence_interval": [0.08, 0.18]
      },
      {
        "year": 2050,
        "sea_level_rise_m": 0.35,
        "confidence_interval": [0.25, 0.48]
      },
      {
        "year": 2100,
        "sea_level_rise_m": 1.02,
        "confidence_interval": [0.73, 1.45]
      }
    ],
    "data_source": "IPCC AR6"
  }
}
```

---

### 4.2 Custom Projection Model

**Endpoint:** `POST /projections/calculate`

**Description:** Calculate custom sea level projections with user-specified parameters.

**Request Body:**
```json
{
  "location": {
    "latitude": 25.7617,
    "longitude": -80.1918
  },
  "parameters": {
    "thermal_expansion_rate": 0.0021,
    "glacier_contribution": 0.0008,
    "greenland_contribution": 0.0012,
    "antarctica_contribution": 0.0006,
    "acceleration_factor": 1.2
  },
  "projection_years": [2030, 2040, 2050, 2075, 2100]
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "projection_id": "CUSTOM-2025-001",
    "projections": [
      {
        "year": 2030,
        "total_rise": 0.115,
        "breakdown": {
          "thermal": 0.063,
          "glaciers": 0.024,
          "greenland": 0.036,
          "antarctica": 0.018
        }
      }
    ]
  }
}
```

---

## 5. Adaptation Planning Endpoints

### 5.1 Generate Adaptation Plan

**Endpoint:** `POST /adaptation/plan`

**Description:** Generate coastal adaptation strategies based on risk assessment.

**Request Body:**
```json
{
  "region": "Miami-Dade County",
  "infrastructure_types": ["roads", "buildings", "utilities"],
  "budget": 50000000,
  "timeline_years": 10,
  "priority": "cost-effective|maximum-protection|rapid-deployment"
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "plan_id": "ADAPT-2025-001",
    "total_cost": 48500000,
    "strategies": [
      {
        "id": "STRAT-001",
        "type": "seawall",
        "description": "Construct 2.5km seawall along waterfront",
        "cost": 20000000,
        "timeline_months": 24,
        "protection_level": 0.85,
        "properties_protected": 850
      },
      {
        "id": "STRAT-002",
        "type": "elevation",
        "description": "Elevate critical infrastructure",
        "cost": 15000000,
        "timeline_months": 18,
        "protection_level": 0.75,
        "facilities_upgraded": 12
      },
      {
        "id": "STRAT-003",
        "type": "drainage",
        "description": "Upgrade stormwater drainage system",
        "cost": 13500000,
        "timeline_months": 15,
        "protection_level": 0.60,
        "area_covered_km2": 8.5
      }
    ],
    "overall_risk_reduction": 0.73
  }
}
```

---

### 5.2 Evaluate Adaptation Strategy

**Endpoint:** `POST /adaptation/evaluate`

**Description:** Evaluate effectiveness and cost-benefit of a proposed adaptation strategy.

**Request Body:**
```json
{
  "strategy": {
    "type": "managed_retreat",
    "properties_affected": 250,
    "relocation_cost_per_property": 150000,
    "timeline_years": 5
  },
  "current_risk": {
    "flood_probability": 0.85,
    "annual_damage_cost": 5000000
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "evaluation_id": "EVAL-2025-001",
    "total_cost": 37500000,
    "annual_savings": 4250000,
    "payback_period_years": 8.8,
    "risk_reduction": 0.95,
    "cost_benefit_ratio": 3.2,
    "recommendation": "RECOMMENDED",
    "notes": "High cost-effectiveness with substantial long-term savings"
  }
}
```

---

## 6. Alert & Notification Endpoints

### 6.1 Subscribe to Alerts

**Endpoint:** `POST /alerts/subscribe`

**Description:** Subscribe to sea level and flood alerts for specific locations.

**Request Body:**
```json
{
  "locations": [
    {
      "id": "WATCH-001",
      "latitude": 25.7617,
      "longitude": -80.1918,
      "alert_threshold_m": 2.0
    }
  ],
  "notification_channels": ["email", "sms", "webhook"],
  "contact": {
    "email": "alerts@example.com",
    "phone": "+1-305-555-0100",
    "webhook_url": "https://example.com/api/alerts"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "subscription_id": "SUB-2025-001",
    "locations_count": 1,
    "active_from": "2025-12-25T15:00:00Z"
  }
}
```

---

### 6.2 Get Active Alerts

**Endpoint:** `GET /alerts/active`

**Description:** Retrieve currently active sea level and flood alerts.

**Query Parameters:**
- `region` (optional): Filter by region
- `severity` (optional): LOW|MODERATE|HIGH|EXTREME

**Response:**
```json
{
  "status": "success",
  "data": {
    "alerts": [
      {
        "alert_id": "ALERT-2025-123",
        "type": "king_tide",
        "severity": "HIGH",
        "location": "Miami Beach",
        "issued_at": "2025-12-25T10:00:00Z",
        "valid_until": "2025-12-25T20:00:00Z",
        "predicted_level": 2.4,
        "message": "King tide event expected. Coastal flooding likely."
      }
    ],
    "count": 1
  }
}
```

---

## 7. Data Export Endpoints

### 7.1 Export Sea Level Data

**Endpoint:** `POST /export/sea-level`

**Description:** Export sea level data in various formats for analysis.

**Request Body:**
```json
{
  "stations": ["NOAA-8638610", "NOAA-8723214"],
  "date_range": {
    "start": "2020-01-01",
    "end": "2025-12-25"
  },
  "format": "csv|json|netcdf",
  "include_metadata": true
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "export_id": "EXPORT-2025-001",
    "download_url": "https://api.wia.global/downloads/EXPORT-2025-001.csv",
    "expires_at": "2025-12-26T15:00:00Z",
    "file_size_mb": 45.2
  }
}
```

---

## 8. WebSocket Real-Time Updates

### 8.1 WebSocket Connection

**Endpoint:** `wss://api.wia.global/ene-055/v1/stream`

**Description:** Real-time streaming of sea level measurements.

**Connection Example:**
```javascript
const ws = new WebSocket('wss://api.wia.global/ene-055/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    stations: ['NOAA-8638610'],
    interval: 'realtime'
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Sea level update:', data);
};
```

**Message Format:**
```json
{
  "type": "measurement",
  "station": "NOAA-8638610",
  "timestamp": "2025-12-25T15:30:00Z",
  "water_level": 1.852,
  "trend": "rising"
}
```

---

## 9. Error Codes

| Code | Description |
|------|-------------|
| 400 | Bad Request - Invalid parameters |
| 401 | Unauthorized - Invalid API key |
| 403 | Forbidden - Insufficient permissions |
| 404 | Not Found - Resource doesn't exist |
| 429 | Too Many Requests - Rate limit exceeded |
| 500 | Internal Server Error |
| 503 | Service Unavailable - Temporary outage |

---

## 10. SDK Support

Official SDKs available for:

- **JavaScript/TypeScript**: `npm install @wia/ene-055`
- **Python**: `pip install wia-sea-level`
- **Go**: `go get github.com/wia-official/ene-055-go`
- **Java**: Maven/Gradle support

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
