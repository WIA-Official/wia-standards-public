# WIA Drought Monitoring Standard - Phase 2: API Interface Specification v1.0

## Overview

Phase 2 defines RESTful API interfaces for accessing WIA-compliant drought data programmatically. This enables automated integration, real-time monitoring, and alert subscription services.

**Version:** 1.0.0
**Status:** Published
**Last Updated:** 2025-12-26

## Base URL Structure

All WIA-compliant APIs MUST follow this URL pattern:

```
https://{domain}/wia/drought/v1/{resource}
```

**Components:**
- `{domain}`: Service provider domain
- `/wia/drought`: WIA drought standard namespace (REQUIRED)
- `/v1`: API version
- `{resource}`: Specific endpoint (pdsi, spi, ndvi, etc.)

## Authentication

### Supported Methods

1. **API Key (Required for all implementations)**
```http
GET /wia/drought/v1/pdsi?lat=40.7128&lon=-74.0060
Headers:
  X-WIA-API-Key: your-api-key-here
```

2. **OAuth 2.0 (Optional)**
```http
GET /wia/drought/v1/pdsi?lat=40.7128&lon=-74.0060
Headers:
  Authorization: Bearer {access_token}
```

### Access Tiers

| Tier | Authentication | Rate Limit | Features |
|------|---------------|------------|----------|
| Public | None | 100 req/day | Basic current data |
| Registered | API Key | 10,000 req/day | Historical data, alerts |
| Premium | OAuth 2.0 | 100,000 req/day | All features |
| Research/Gov | OAuth 2.0 | Unlimited | Bulk export, streaming |

## Core Endpoints

### Get Current PDSI

```http
GET /wia/drought/v1/pdsi

Query Parameters:
  lat (required): Latitude (-90 to 90)
  lon (required): Longitude (-180 to 180)
  date (optional): ISO 8601 date

Response: 200 OK
{
  "wia_version": "1.0",
  "data_type": "pdsi",
  "timestamp": "2025-12-26T00:00:00Z",
  "location": {...},
  "pdsi": {...},
  "metadata": {...}
}
```

### Get Current SPI

```http
GET /wia/drought/v1/spi

Query Parameters:
  lat (required): Latitude
  lon (required): Longitude
  scale (required): Time scale (1, 3, 6, 12, 24 months)
  date (optional): ISO 8601 date
```

### Get Soil Moisture

```http
GET /wia/drought/v1/soil-moisture

Query Parameters:
  lat (required): Latitude
  lon (required): Longitude
  depth (optional): Depth in cm or "all"
  date (optional): ISO 8601 date
```

### Get NDVI

```http
GET /wia/drought/v1/ndvi

Query Parameters:
  bbox (required): Bounding box [west,south,east,north]
  date (optional): ISO 8601 date
  resolution (optional): Resolution in meters (250, 500, 1000)
```

## Historical Data

### Time Series Query

```http
GET /wia/drought/v1/{index}/timeseries

Query Parameters:
  lat (required): Latitude
  lon (required): Longitude
  start_date (required): ISO 8601 date
  end_date (required): ISO 8601 date
  interval (optional): daily, weekly, monthly (default: monthly)

Response:
{
  "wia_version": "1.0",
  "data_type": "pdsi_timeseries",
  "location": {...},
  "time_series": [
    {
      "timestamp": "2024-01-01T00:00:00Z",
      "pdsi": {"value": 1.2, "classification": "slightly_wet"}
    },
    ...
  ],
  "statistics": {
    "count": 24,
    "mean": -0.45,
    "trend": "declining"
  }
}
```

## Alert Subscriptions

### Create Subscription

```http
POST /wia/drought/v1/alerts/subscribe

Request Body:
{
  "locations": [
    {"lat": 40.7128, "lon": -74.0060, "name": "Field A"}
  ],
  "indices": ["pdsi", "soil_moisture"],
  "thresholds": {
    "pdsi": {"warning": -2.0, "alert": -3.0}
  },
  "notification_methods": [
    {"type": "webhook", "url": "https://app.com/alert"},
    {"type": "email", "address": "user@example.com"}
  ],
  "frequency": "daily"
}

Response: 201 Created
{
  "subscription_id": "sub_a1b2c3",
  "status": "active",
  "created_at": "2025-12-26T14:30:00Z"
}
```

### Webhook Alert Format

```http
POST {webhook_url}

Request Body:
{
  "alert_id": "alert_x1y2z3",
  "subscription_id": "sub_a1b2c3",
  "timestamp": "2025-12-27T08:15:00Z",
  "location": {"name": "Field A", "lat": 40.7128, "lon": -74.0060},
  "trigger": {
    "index": "pdsi",
    "value": -3.2,
    "threshold": "alert"
  },
  "severity": "high",
  "recommended_actions": [...]
}
```

## Error Handling

### Standard Error Format

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable description",
    "details": {...},
    "request_id": "req_xyz789",
    "documentation_url": "https://docs.wia.org/errors#ERROR_CODE"
  }
}
```

### HTTP Status Codes

| Status | Error Code | Meaning | Action |
|--------|-----------|---------|--------|
| 400 | INVALID_PARAMETERS | Bad request parameters | Fix parameters |
| 401 | UNAUTHORIZED | Missing/invalid auth | Provide valid credentials |
| 404 | DATA_NOT_FOUND | No data available | Try different location/date |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests | Wait before retry |
| 500 | INTERNAL_ERROR | Server error | Contact support |

## Rate Limiting

All responses MUST include rate limit headers:

```http
X-RateLimit-Limit: 10000
X-RateLimit-Remaining: 9847
X-RateLimit-Reset: 1735257600
X-RateLimit-Window: daily
```

## Implementation Requirements

Phase 2 compliant systems MUST:

1. Implement base URL structure `/wia/drought/v1/`
2. Support API key authentication
3. Provide at least PDSI and SPI endpoints
4. Return standardized error responses
5. Include rate limit headers
6. Support CORS for web applications
7. Provide OpenAPI/Swagger documentation

## Conformance Checklist

- [ ] Base URL follows `/wia/drought/v1/` pattern
- [ ] API key authentication working
- [ ] Core endpoints implemented (PDSI, SPI)
- [ ] Error responses match standard format
- [ ] Rate limiting implemented
- [ ] CORS enabled
- [ ] OpenAPI documentation available
- [ ] Load test: 100 requests/second sustained

## References

- REST API Design Best Practices
- OpenAPI Specification 3.0
- OAuth 2.0 RFC 6749

---

© 2025 SmileStory Inc. / WIA
弘益人間
