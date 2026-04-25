# Chapter 5: API Interfaces and Services

## RESTful Services for Drought Data Access and Management

---

## 5.1 RESTful API Architecture Design

### Design Principles

The WIA-ENV-003 API follows REST architectural principles adapted for geospatial and temporal data:

| Principle | Implementation | Benefit |
|-----------|---------------|---------|
| Resource-Oriented | Resources mapped to drought data entities | Intuitive URL structure |
| Stateless | No server-side session state | Scalability, reliability |
| Cacheable | HTTP caching headers | Performance, reduced load |
| Uniform Interface | Consistent patterns across endpoints | Ease of use |
| Layered System | Load balancers, CDN support | Scalability |

### API Base URL Structure

```
Production:    https://api.drought-monitor.org/v1
Staging:       https://staging-api.drought-monitor.org/v1
Development:   https://dev-api.drought-monitor.org/v1
```

### Resource Hierarchy

```
/v1
├── /indices
│   ├── /pdsi
│   ├── /spi
│   ├── /spei
│   ├── /soil-moisture
│   └── /ndvi
├── /locations
│   ├── /points
│   ├── /regions
│   └── /polygons
├── /timeseries
├── /alerts
│   ├── /subscriptions
│   └── /notifications
├── /forecasts
├── /maps
└── /admin
    ├── /health
    └── /status
```

### HTTP Methods Usage

| Method | Purpose | Idempotent | Safe |
|--------|---------|------------|------|
| GET | Retrieve resources | Yes | Yes |
| POST | Create resources, complex queries | No | No |
| PUT | Full resource update | Yes | No |
| PATCH | Partial resource update | No | No |
| DELETE | Remove resources | Yes | No |

### Response Format Standards

All API responses follow a consistent envelope structure:

```json
{
  "meta": {
    "request_id": "uuid-string",
    "timestamp": "2025-01-15T12:00:00Z",
    "api_version": "1.0.0",
    "standard_id": "WIA-ENV-003",
    "processing_time_ms": 145
  },
  "data": {
    // Response payload
  },
  "pagination": {
    "total_count": 1250,
    "page": 1,
    "page_size": 100,
    "total_pages": 13,
    "next": "https://api.../indices/pdsi?page=2",
    "prev": null
  },
  "links": {
    "self": "https://api.../indices/pdsi?location=US-KS-CD05",
    "related": {
      "spi": "https://api.../indices/spi?location=US-KS-CD05",
      "soil_moisture": "https://api.../indices/soil-moisture?location=US-KS-CD05"
    }
  }
}
```

### Error Response Format

```json
{
  "meta": {
    "request_id": "uuid-string",
    "timestamp": "2025-01-15T12:00:00Z"
  },
  "error": {
    "code": "INVALID_LOCATION",
    "message": "The specified location ID does not exist",
    "details": {
      "provided_value": "US-XX-CD99",
      "valid_format": "US-{state}-CD{division}"
    },
    "documentation_url": "https://docs.../errors/INVALID_LOCATION"
  }
}
```

---

## 5.2 Drought Index Query Endpoints

### PDSI Endpoint

**Endpoint:** `GET /v1/indices/pdsi`

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| location | string | Yes* | Location ID or coordinates |
| lat | number | Yes* | Latitude (-90 to 90) |
| lon | number | Yes* | Longitude (-180 to 180) |
| date | string | No | Specific date (YYYY-MM-DD) |
| start_date | string | No | Period start date |
| end_date | string | No | Period end date |
| variant | string | No | PDSI, SC-PDSI, PHDI, PMDI |
| format | string | No | json, geojson, csv |

*Either location OR lat/lon required

**Example Request:**
```http
GET /v1/indices/pdsi?location=US-KS-CD05&start_date=2024-01-01&end_date=2025-01-31
Accept: application/json
Authorization: Bearer {token}
```

**Example Response:**
```json
{
  "meta": {
    "request_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
    "timestamp": "2025-01-15T12:00:00Z",
    "api_version": "1.0.0"
  },
  "data": {
    "index_type": "SC-PDSI",
    "location": {
      "location_id": "US-KS-CD05",
      "name": "Kansas Climate Division 5",
      "coordinates": {"latitude": 38.5, "longitude": -98.5}
    },
    "period": {
      "start_date": "2024-01-01",
      "end_date": "2025-01-31"
    },
    "values": [
      {
        "date": "2024-01-31",
        "value": -1.25,
        "classification": {"category": "D1", "label": "Moderate Drought"},
        "quality": {"flag": 0, "confidence": 0.95}
      },
      {
        "date": "2024-02-29",
        "value": -1.89,
        "classification": {"category": "D1", "label": "Moderate Drought"},
        "quality": {"flag": 0, "confidence": 0.94}
      }
      // Additional monthly values...
    ],
    "statistics": {
      "mean": -2.15,
      "min": -3.45,
      "max": -0.85,
      "std_dev": 0.78
    }
  }
}
```

### SPI Endpoint

**Endpoint:** `GET /v1/indices/spi`

**Additional Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| time_scale | integer | No | Months (1-48), default: 3 |
| distribution | string | No | gamma, pearson-III |
| baseline_start | integer | No | Baseline start year |
| baseline_end | integer | No | Baseline end year |

**Multi-Scale Query Example:**
```http
GET /v1/indices/spi?location=US-KS-CD05&time_scale=1,3,6,12&date=2025-01-31
```

**Response:**
```json
{
  "data": {
    "index_type": "SPI",
    "location": {"location_id": "US-KS-CD05"},
    "date": "2025-01-31",
    "multi_scale": [
      {"time_scale_months": 1, "value": 0.45, "classification": "Near Normal"},
      {"time_scale_months": 3, "value": -1.23, "classification": "Moderately Dry"},
      {"time_scale_months": 6, "value": -1.87, "classification": "Severely Dry"},
      {"time_scale_months": 12, "value": -2.15, "classification": "Extremely Dry"}
    ]
  }
}
```

### Soil Moisture Endpoint

**Endpoint:** `GET /v1/indices/soil-moisture`

**Additional Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| depth | string | Depth range (e.g., "0-10cm", "0-100cm") |
| variable | string | volumetric, percentile, anomaly |
| source | string | in_situ, satellite, model, blended |

**Response Structure:**
```json
{
  "data": {
    "location": {"location_id": "SCAN-2001"},
    "timestamp": "2025-01-15T06:00:00Z",
    "source": "blended",
    "layers": [
      {
        "depth": "0-10cm",
        "volumetric_water_content": 0.18,
        "percentile": 15,
        "anomaly_mm": -8.2,
        "classification": "D2"
      },
      {
        "depth": "0-100cm",
        "volumetric_water_content": 0.25,
        "percentile": 20,
        "anomaly_mm": -46.0,
        "classification": "D2"
      }
    ]
  }
}
```

### NDVI Endpoint

**Endpoint:** `GET /v1/indices/ndvi`

**Additional Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| satellite | string | MODIS, Landsat, Sentinel-2 |
| composite_days | integer | Compositing period |
| product | string | ndvi, ndvi_anomaly, vci |

---

## 5.3 Geospatial Data Services

### Point Query Service

**Endpoint:** `GET /v1/locations/points/{point_id}`

**Endpoint:** `POST /v1/locations/points/query`

```json
// POST body for multiple point query
{
  "points": [
    {"id": "P1", "lat": 38.5, "lon": -98.5},
    {"id": "P2", "lat": 39.0, "lon": -99.0},
    {"id": "P3", "lat": 37.5, "lon": -97.5}
  ],
  "indices": ["pdsi", "spi-3", "soil_moisture"],
  "date": "2025-01-15"
}
```

### Region Query Service

**Endpoint:** `GET /v1/locations/regions`

**Available Region Types:**

| Type | Example | Description |
|------|---------|-------------|
| climate_division | US-KS-CD05 | NOAA climate divisions |
| county | US-KS-177 | County FIPS codes |
| state | US-KS | State/province codes |
| huc | 10270104 | Hydrologic unit codes |
| agricultural_district | KS-60 | USDA agricultural districts |
| custom | USER-001 | User-defined regions |

**Region Statistics Request:**
```http
GET /v1/locations/regions/US-KS/indices/pdsi?date=2025-01-31&include_statistics=true
```

**Response:**
```json
{
  "data": {
    "region": {
      "region_id": "US-KS",
      "name": "Kansas",
      "type": "state",
      "area_km2": 213100
    },
    "index_type": "PDSI",
    "date": "2025-01-31",
    "spatial_statistics": {
      "mean": -2.35,
      "median": -2.18,
      "min": -4.12,
      "max": -0.45,
      "std_dev": 0.95,
      "area_in_drought": {
        "D0_percent": 95.2,
        "D1_percent": 78.4,
        "D2_percent": 45.6,
        "D3_percent": 12.3,
        "D4_percent": 0.0
      }
    },
    "sub_region_values": [
      {"region_id": "US-KS-CD01", "value": -1.85},
      {"region_id": "US-KS-CD02", "value": -2.45}
      // Additional divisions...
    ]
  }
}
```

### Polygon Query Service

**Endpoint:** `POST /v1/locations/polygons/query`

```json
// POST body for custom polygon query
{
  "geometry": {
    "type": "Polygon",
    "coordinates": [[
      [-99.0, 38.0],
      [-99.0, 39.0],
      [-98.0, 39.0],
      [-98.0, 38.0],
      [-99.0, 38.0]
    ]]
  },
  "indices": ["pdsi", "ndvi_anomaly"],
  "date": "2025-01-15",
  "statistics": ["mean", "min", "max", "percentiles"]
}
```

**Polygon Response:**
```json
{
  "data": {
    "geometry_area_km2": 8234.5,
    "query_date": "2025-01-15",
    "results": {
      "pdsi": {
        "mean": -2.78,
        "min": -3.45,
        "max": -2.15,
        "percentiles": {
          "p10": -3.35,
          "p25": -3.10,
          "p50": -2.75,
          "p75": -2.45,
          "p90": -2.25
        }
      },
      "ndvi_anomaly": {
        "mean": -0.12,
        "min": -0.28,
        "max": 0.05
      }
    }
  }
}
```

---

## 5.4 Time Series Analysis APIs

### Historical Time Series

**Endpoint:** `GET /v1/timeseries`

**Query Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| location | string | Location identifier |
| index | string | Index type |
| start_date | string | Series start date |
| end_date | string | Series end date |
| interval | string | daily, weekly, monthly |
| include_statistics | boolean | Include summary statistics |
| include_trend | boolean | Include trend analysis |

**Response:**
```json
{
  "data": {
    "timeseries": {
      "location_id": "US-KS-CD05",
      "index_type": "PDSI",
      "interval": "monthly",
      "start_date": "2020-01-01",
      "end_date": "2025-01-31",
      "values": [
        {"date": "2020-01-31", "value": -0.85, "quality": 0},
        {"date": "2020-02-29", "value": -1.23, "quality": 0}
        // ... additional values
      ]
    },
    "statistics": {
      "count": 61,
      "mean": -1.78,
      "median": -1.65,
      "std_dev": 1.12,
      "min": {"value": -4.25, "date": "2022-08-31"},
      "max": {"value": 1.85, "date": "2020-05-31"}
    },
    "trend": {
      "method": "Mann-Kendall",
      "slope": -0.024,
      "p_value": 0.032,
      "significance": "significant",
      "interpretation": "Statistically significant drying trend"
    }
  }
}
```

### Comparative Analysis

**Endpoint:** `POST /v1/timeseries/compare`

```json
// Compare multiple locations or indices
{
  "comparisons": [
    {"location": "US-KS-CD05", "index": "pdsi"},
    {"location": "US-KS-CD06", "index": "pdsi"},
    {"location": "US-NE-CD07", "index": "pdsi"}
  ],
  "start_date": "2020-01-01",
  "end_date": "2025-01-31",
  "analysis": ["correlation", "lag_analysis"]
}
```

### Anomaly Detection

**Endpoint:** `GET /v1/timeseries/anomalies`

**Response:**
```json
{
  "data": {
    "anomalies_detected": [
      {
        "period": {"start": "2022-06-01", "end": "2022-09-30"},
        "type": "flash_drought",
        "severity": "extreme",
        "metrics": {
          "pdsi_change": -3.2,
          "duration_months": 4,
          "percentile_rank": 2.3
        }
      }
    ],
    "drought_periods": [
      {
        "start_date": "2021-03-01",
        "end_date": "2023-02-28",
        "duration_months": 24,
        "peak_severity": -4.25,
        "peak_date": "2022-08-31",
        "classification": "multi-year_drought"
      }
    ]
  }
}
```

---

## 5.5 Alert and Notification Services

### Alert Threshold Configuration

**Endpoint:** `POST /v1/alerts/thresholds`

```json
{
  "threshold_id": "user-threshold-001",
  "name": "Kansas Severe Drought Alert",
  "location": {
    "type": "region",
    "region_id": "US-KS"
  },
  "conditions": [
    {
      "index": "pdsi",
      "operator": "less_than",
      "value": -3.0,
      "duration_months": 2
    },
    {
      "index": "soil_moisture_percentile",
      "operator": "less_than",
      "value": 10,
      "conjunction": "AND"
    }
  ],
  "severity": "high",
  "active": true
}
```

### Alert Subscription Management

**Endpoint:** `POST /v1/alerts/subscriptions`

```json
{
  "subscription_id": "sub-001",
  "threshold_ids": ["user-threshold-001", "user-threshold-002"],
  "notification_channels": [
    {
      "type": "email",
      "address": "alerts@example.com",
      "frequency": "immediate"
    },
    {
      "type": "webhook",
      "url": "https://example.com/drought-alerts",
      "method": "POST",
      "headers": {"Authorization": "Bearer xxx"}
    },
    {
      "type": "sms",
      "phone": "+1-555-123-4567",
      "frequency": "daily_digest"
    }
  ],
  "active": true
}
```

### Alert Notification Format

```json
{
  "notification": {
    "notification_id": "notif-abc123",
    "timestamp": "2025-01-15T14:30:00Z",
    "alert": {
      "threshold_id": "user-threshold-001",
      "threshold_name": "Kansas Severe Drought Alert",
      "triggered_conditions": [
        {
          "index": "pdsi",
          "current_value": -3.45,
          "threshold_value": -3.0,
          "condition_met": true
        }
      ]
    },
    "location": {
      "region_id": "US-KS",
      "name": "Kansas"
    },
    "current_status": {
      "pdsi": -3.45,
      "spi_3": -1.85,
      "soil_moisture_percentile": 8,
      "drought_category": "D2"
    },
    "trend": {
      "direction": "worsening",
      "rate": -0.15,
      "forecast": "Continued deterioration expected"
    },
    "actions": {
      "view_details": "https://dashboard.../alert/abc123",
      "acknowledge": "https://api.../alerts/abc123/acknowledge",
      "suppress": "https://api.../alerts/abc123/suppress"
    }
  }
}
```

### Alert History Query

**Endpoint:** `GET /v1/alerts/history`

| Parameter | Type | Description |
|-----------|------|-------------|
| location | string | Filter by location |
| severity | string | Filter by severity level |
| start_date | string | History period start |
| end_date | string | History period end |
| acknowledged | boolean | Filter by acknowledgment status |

---

## 5.6 Authentication and Authorization

### Authentication Methods

The API supports multiple authentication mechanisms:

| Method | Use Case | Security Level |
|--------|----------|----------------|
| API Key | Simple integration, development | Basic |
| OAuth 2.0 | User-facing applications | Standard |
| JWT Bearer | Machine-to-machine, microservices | High |
| mTLS | Enterprise, government | Very High |

### API Key Authentication

```http
GET /v1/indices/pdsi?location=US-KS-CD05
X-API-Key: your-api-key-here
```

### OAuth 2.0 Flow

```
Authorization Endpoint: https://auth.drought-monitor.org/authorize
Token Endpoint:         https://auth.drought-monitor.org/token
Scopes: read:indices, read:alerts, write:alerts, admin
```

**Token Request:**
```http
POST /token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=your_client_id&
client_secret=your_client_secret&
scope=read:indices read:alerts
```

### JWT Bearer Token

```http
GET /v1/indices/pdsi?location=US-KS-CD05
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**JWT Claims Structure:**
```json
{
  "iss": "https://auth.drought-monitor.org",
  "sub": "user-12345",
  "aud": "https://api.drought-monitor.org",
  "exp": 1705320000,
  "iat": 1705316400,
  "scope": "read:indices read:alerts",
  "tier": "professional"
}
```

### Authorization Levels

| Tier | Rate Limit | Features | Cost |
|------|------------|----------|------|
| Public | 100 req/day | Basic indices, limited regions | Free |
| Developer | 10,000 req/day | All indices, all regions | Free |
| Professional | 100,000 req/day | + Alerts, + Historical | $99/mo |
| Enterprise | Unlimited | + SLA, + Support, + Custom | Contact |

---

## 5.7 Rate Limiting and Quota Management

### Rate Limit Headers

All API responses include rate limit information:

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 10000
X-RateLimit-Remaining: 9523
X-RateLimit-Reset: 1705363200
X-RateLimit-Tier: developer
```

### Rate Limit Response (429)

```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "API rate limit exceeded",
    "details": {
      "limit": 10000,
      "window": "day",
      "reset_at": "2025-01-16T00:00:00Z",
      "upgrade_url": "https://drought-monitor.org/pricing"
    }
  }
}
```

### Quota Management Endpoints

**Check Current Usage:**
```http
GET /v1/admin/usage
Authorization: Bearer {token}
```

**Response:**
```json
{
  "data": {
    "tier": "developer",
    "period": {
      "start": "2025-01-01T00:00:00Z",
      "end": "2025-01-31T23:59:59Z"
    },
    "usage": {
      "api_calls": {
        "limit": 10000,
        "used": 4523,
        "remaining": 5477
      },
      "data_export_mb": {
        "limit": 1000,
        "used": 234.5,
        "remaining": 765.5
      },
      "alert_subscriptions": {
        "limit": 10,
        "used": 3,
        "remaining": 7
      }
    }
  }
}
```

### Burst Handling

The API allows temporary bursts above sustained limits:

| Tier | Sustained Rate | Burst Limit | Burst Duration |
|------|---------------|-------------|----------------|
| Public | 100/day | 10/minute | 1 minute |
| Developer | 10k/day | 100/minute | 5 minutes |
| Professional | 100k/day | 500/minute | 10 minutes |
| Enterprise | Custom | Custom | Custom |

---

## 5.8 SDK Implementation Guidelines

### Official SDK Structure

```
drought-monitoring-sdk/
├── src/
│   ├── client.ts           # Main client class
│   ├── indices/
│   │   ├── pdsi.ts
│   │   ├── spi.ts
│   │   └── soil-moisture.ts
│   ├── locations/
│   │   ├── points.ts
│   │   ├── regions.ts
│   │   └── polygons.ts
│   ├── alerts/
│   │   └── subscriptions.ts
│   ├── types/
│   │   └── index.ts
│   └── utils/
│       ├── auth.ts
│       └── errors.ts
├── package.json
└── README.md
```

### TypeScript SDK Example

```typescript
import { DroughtMonitorClient } from '@wia/drought-monitoring-sdk';

// Initialize client
const client = new DroughtMonitorClient({
  apiKey: process.env.DROUGHT_API_KEY,
  baseUrl: 'https://api.drought-monitor.org/v1',
  timeout: 30000,
  retryConfig: {
    maxRetries: 3,
    backoffMs: 1000
  }
});

// Query PDSI
async function getPDSI() {
  try {
    const response = await client.indices.pdsi.get({
      location: 'US-KS-CD05',
      startDate: '2024-01-01',
      endDate: '2025-01-31'
    });

    console.log(`Mean PDSI: ${response.statistics.mean}`);
    console.log(`Classification: ${response.values[0].classification.label}`);

    return response;
  } catch (error) {
    if (error.code === 'RATE_LIMIT_EXCEEDED') {
      console.log(`Rate limited. Retry after: ${error.details.reset_at}`);
    }
    throw error;
  }
}

// Subscribe to alerts
async function subscribeToAlerts() {
  const subscription = await client.alerts.subscriptions.create({
    thresholdIds: ['threshold-001'],
    channels: [{
      type: 'webhook',
      url: 'https://myapp.com/drought-webhook',
      method: 'POST'
    }]
  });

  return subscription;
}

// Query polygon
async function queryPolygon() {
  const polygon = {
    type: 'Polygon' as const,
    coordinates: [[
      [-99.0, 38.0],
      [-99.0, 39.0],
      [-98.0, 39.0],
      [-98.0, 38.0],
      [-99.0, 38.0]
    ]]
  };

  const results = await client.locations.polygons.query({
    geometry: polygon,
    indices: ['pdsi', 'ndvi_anomaly'],
    date: '2025-01-15'
  });

  return results;
}
```

### Python SDK Example

```python
from drought_monitoring import DroughtMonitorClient
from datetime import date

# Initialize client
client = DroughtMonitorClient(
    api_key=os.environ['DROUGHT_API_KEY'],
    base_url='https://api.drought-monitor.org/v1'
)

# Query PDSI time series
pdsi_data = client.indices.pdsi.get(
    location='US-KS-CD05',
    start_date=date(2024, 1, 1),
    end_date=date(2025, 1, 31)
)

# Access data
for value in pdsi_data.values:
    print(f"{value.date}: {value.value} ({value.classification.label})")

# Multi-scale SPI query
spi_data = client.indices.spi.get(
    location='US-KS-CD05',
    time_scales=[1, 3, 6, 12],
    date=date(2025, 1, 31)
)

# Polygon query
from shapely.geometry import Polygon
polygon = Polygon([
    (-99.0, 38.0), (-99.0, 39.0),
    (-98.0, 39.0), (-98.0, 38.0)
])

polygon_results = client.locations.polygons.query(
    geometry=polygon,
    indices=['pdsi', 'soil_moisture'],
    date=date(2025, 1, 15)
)
```

---

## 5.9 Review Questions and Key Takeaways

### Review Questions

1. **API Design**: Explain why the WIA-ENV-003 API uses a resource-oriented REST design rather than RPC-style endpoints. What are the benefits for drought monitoring applications?

2. **Multi-Scale Queries**: A user needs to understand both recent precipitation recovery (1-month SPI) and long-term drought conditions (12-month SPI). Design an API query that retrieves both efficiently.

3. **Geospatial Queries**: Compare the use cases for point queries, region queries, and polygon queries. When would each be most appropriate?

4. **Alert Configuration**: Design an alert threshold configuration for detecting flash drought (rapid onset drought). What indices, thresholds, and temporal conditions would you specify?

5. **Authentication Selection**: A farm management SaaS application needs to access drought data on behalf of thousands of farmers. Which authentication method would be most appropriate and why?

6. **Rate Limit Strategy**: A research team needs to download 10 years of daily PDSI data for 100 locations. How should they structure their API calls to avoid rate limiting while completing the download efficiently?

7. **Error Handling**: Write pseudocode for an SDK method that handles common error scenarios: rate limiting, authentication failure, invalid location, and network timeout.

8. **SDK Design**: What are the key considerations when designing an SDK for the drought monitoring API? How should the SDK handle pagination, rate limiting, and error recovery?

### Key Takeaways

1. **RESTful Design**: The API follows REST principles with resource-oriented URLs, standard HTTP methods, and consistent response formats enabling intuitive integration.

2. **Comprehensive Index Coverage**: Endpoints exist for all major drought indices (PDSI, SPI, SPEI, soil moisture, NDVI) with flexible query parameters for customization.

3. **Geospatial Flexibility**: Point, region, and polygon query services support diverse spatial analysis needs from field-scale to continental.

4. **Time Series Analysis**: Dedicated endpoints support historical analysis, trend detection, and anomaly identification essential for drought monitoring.

5. **Alerting Infrastructure**: Threshold configuration, subscription management, and multi-channel notifications enable proactive drought response.

6. **Security Layers**: Multiple authentication methods (API key, OAuth, JWT, mTLS) accommodate different security requirements and use cases.

7. **Fair Usage Management**: Rate limiting with tiered quotas ensures system stability while providing adequate access for different user types.

8. **SDK Support**: Official SDKs in TypeScript and Python simplify integration, handle common patterns, and abstract API complexity.

9. **Response Consistency**: Standard envelope structure with metadata, pagination, and links provides predictable responses across all endpoints.

10. **Error Transparency**: Detailed error responses with codes, messages, and documentation links support efficient debugging and integration.

---

## Chapter Summary

This chapter has detailed the API interface specifications for the WIA-ENV-003 Drought Monitoring Standard. The RESTful API design provides comprehensive access to drought indices, geospatial services, time series analysis, and alerting capabilities.

The API architecture follows established REST principles—resource orientation, statelessness, cacheability—adapted for the specific requirements of geospatial and temporal drought data. Consistent URL patterns, query parameters, and response formats enable intuitive integration.

Index query endpoints support all major drought indices with flexible filtering by location, time, and index variant. Geospatial services enable queries from single points to complex polygons, supporting diverse application needs. Time series endpoints provide historical data access with statistical analysis and trend detection.

The alerting system enables proactive drought monitoring through configurable thresholds, subscription management, and multi-channel notifications. Organizations can define complex alert conditions combining multiple indices and temporal requirements.

Authentication and authorization support ranges from simple API keys for development to enterprise-grade mTLS for sensitive applications. Rate limiting with tiered quotas ensures fair access while maintaining system performance.

Official SDKs abstract API complexity, handling authentication, pagination, rate limiting, and error recovery. TypeScript and Python examples demonstrate common integration patterns.

These API specifications provide the foundation for interoperable drought monitoring applications, enabling consistent data access across implementations worldwide.

---

**Next Chapter: [Chapter 6: Protocols and Algorithms](06-protocol.md)**
