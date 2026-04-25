# WIA Ecosystem Monitoring Standard - Phase 2: API Interface Specification v1.0

**Status:** Official Release  
**Version:** 1.0.0  
**Date:** December 26, 2025  
**License:** CC BY 4.0

## 1. Introduction

Phase 2 specifies RESTful and real-time APIs for accessing ecosystem monitoring data programmatically.

## 2. RESTful API Specification

### 2.1 Base URL Structure
```
https://api.{domain}/v1/
```

### 2.2 Core Endpoints

**GET /observations**
Query species observations with filtering parameters.

Request parameters:
- `taxon` (string): Scientific name
- `start_date` (ISO 8601): Filter by date range
- `end_date` (ISO 8601): Filter by date range
- `bbox` (string): Bounding box "minLon,minLat,maxLon,maxLat"
- `limit` (integer): Maximum records returned (default: 100, max: 1000)
- `offset` (integer): Pagination offset
- `format` (string): Response format - "json", "csv", "geojson"

Response format:
```json
{
  "status": "success",
  "api_version": "1.0",
  "request_id": "string",
  "timestamp": "ISO 8601 datetime",
  "query": { /* echoed query parameters */ },
  "pagination": {
    "total_records": 1000,
    "returned_records": 100,
    "page": 1,
    "total_pages": 10,
    "next_page": "URL"
  },
  "data": [ /* array of observation objects */ ]
}
```

**POST /observations**
Submit new observation(s).

Request body: Single observation object or array
Response: Created observation ID(s) with validation results

**GET /observations/{id}**
Retrieve specific observation by ID.

**GET /sensors**
List available sensors.

**GET /sensors/{id}/data**
Retrieve sensor time series data.

Parameters:
- `start_time` (ISO 8601): Start of time range
- `end_time` (ISO 8601): End of time range
- `aggregation` (enum): "raw", "hourly", "daily", "monthly"
- `format` (string): "json", "csv", "netcdf"

**GET /sites**
Query monitoring sites.

**GET /datasets**
Discover available datasets.

### 2.3 Authentication

Supported methods:
1. **API Key**: `Authorization: Bearer YOUR_API_KEY`
2. **OAuth 2.0**: Standard OAuth 2.0 flow
3. **JWT**: JSON Web Tokens with claims

### 2.4 Rate Limiting

Headers returned:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1735228800
```

Default limits:
- Anonymous: 100 requests/hour
- Authenticated: 1000 requests/hour
- Premium: 10000 requests/hour

### 2.5 Error Responses

```json
{
  "status": "error",
  "error_code": "INVALID_PARAMETER",
  "message": "Human-readable error description",
  "details": { /* additional error context */ },
  "request_id": "string",
  "timestamp": "ISO 8601 datetime"
}
```

HTTP Status Codes:
- 200: Success
- 201: Created
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 429: Too Many Requests
- 500: Internal Server Error

## 3. Real-Time Streaming API

### 3.1 WebSocket Protocol

Connection:
```javascript
ws://api.{domain}/stream
wss://api.{domain}/stream  // SSL/TLS
```

Subscribe message:
```json
{
  "action": "subscribe",
  "sensors": ["SENSOR-001", "SENSOR-002"],
  "filters": {
    "quality_min": 0.8
  }
}
```

Data messages:
```json
{
  "type": "sensor_reading",
  "sensor_id": "SENSOR-001",
  "timestamp": "ISO 8601",
  "value": 12.3,
  "unit": "celsius",
  "qc_flag": "good"
}
```

### 3.2 MQTT Protocol

Topics:
```
sensors/{sensor_id}/data
sensors/{sensor_id}/status
sensors/{sensor_id}/alerts
observations/{site_id}/species
```

QoS Levels:
- 0: At most once
- 1: At least once (recommended for data)
- 2: Exactly once (for critical alerts)

## 4. Bulk Data Access

### 4.1 Asynchronous Queries

**POST /bulk-query**
Submit large query for asynchronous processing.

**GET /jobs/{job_id}**
Check job status.

**GET /jobs/{job_id}/download**
Download completed results.

### 4.2 Data Dumps

Pre-generated datasets available at `/dumps/`:
- Current year observations: Updated daily
- Sensor data: Updated hourly
- Complete archive: Updated monthly

## 5. OpenAPI Specification

Complete OpenAPI 3.0 specification available at:
```
https://api.{domain}/openapi.json
```

## 6. SDK Support

Official client libraries:
- JavaScript/TypeScript: npm package `@wia/ecosystem-monitoring`
- Python: pip package `wia-ecosystem-monitoring`
- R: CRAN package `wiaR`
- Java: Maven artifact `org.wia:ecosystem-monitoring`

## 7. Versioning

API versions in URL path: `/v1/`, `/v2/`
Minimum 12-month support for deprecated versions.
