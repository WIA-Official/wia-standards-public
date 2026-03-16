# WIA Precision Agriculture API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - Agriculture)

---

## Table of Contents

1. [Overview](#overview)
2. [Authentication](#authentication)
3. [Field Management API](#field-management-api)
4. [Zone Management API](#zone-management-api)
5. [VRT Prescription API](#vrt-prescription-api)
6. [GPS Guidance API](#gps-guidance-api)
7. [Yield Mapping API](#yield-mapping-api)
8. [Soil Data API](#soil-data-api)
9. [Weather Integration API](#weather-integration-api)
10. [Real-Time Telemetry API](#real-time-telemetry-api)
11. [Error Codes](#error-codes)
12. [Rate Limits](#rate-limits)
13. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Precision Agriculture API Interface Standard defines RESTful HTTP APIs for managing precision agriculture data, enabling real-time field operations, VRT prescriptions, GPS guidance, and farm management system integration.

**Base URL**: `https://api.precision-ag.wiastandards.com/v1`

**Supported Protocols**:
- REST/HTTP (Primary)
- WebSocket (Real-time telemetry)
- MQTT (IoT sensor data)
- gRPC (High-performance operations)

### 1.2 API Design Principles

1. **RESTful**: Resource-based URLs, standard HTTP methods
2. **JSON**: All requests/responses use JSON format
3. **Stateless**: Each request contains all necessary information
4. **Versioned**: API version in URL path (`/v1`)
5. **Paginated**: Large datasets support pagination
6. **Real-time**: WebSocket for live field operations

---

## Authentication

### 2.1 API Key Authentication

```http
GET /v1/fields HTTP/1.1
Host: api.precision-ag.wiastandards.com
Authorization: Bearer {API_KEY}
Content-Type: application/json
```

### 2.2 OAuth 2.0

```http
POST /oauth/token HTTP/1.1
Host: api.precision-ag.wiastandards.com
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={CLIENT_ID}
&client_secret={CLIENT_SECRET}
&scope=field:read field:write vrt:read vrt:write
```

**Response**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "field:read field:write vrt:read vrt:write"
}
```

### 2.3 Scopes

| Scope | Description |
|-------|-------------|
| `field:read` | Read field data |
| `field:write` | Create/update fields |
| `zone:read` | Read zone data |
| `zone:write` | Create/update zones |
| `vrt:read` | Read VRT prescriptions |
| `vrt:write` | Create/update VRT prescriptions |
| `yield:read` | Read yield data |
| `yield:write` | Upload yield data |
| `telemetry:read` | Read real-time telemetry |

---

## Field Management API

### 3.1 List Fields

```http
GET /v1/fields?farm_id={FARM_ID}&page=1&limit=20
Authorization: Bearer {TOKEN}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "fields": [
      {
        "field_id": "FLD-2025-001",
        "field_name": "North Field",
        "farm_id": "FARM-001",
        "total_area_ha": 50.5,
        "center_point": { "lat": 37.567, "lng": 126.9785 },
        "current_crop": "CORN",
        "created_at": "2025-01-15T08:00:00Z"
      }
    ],
    "pagination": {
      "page": 1,
      "limit": 20,
      "total": 1,
      "total_pages": 1
    }
  }
}
```

### 3.2 Get Field Details

```http
GET /v1/fields/{FIELD_ID}
Authorization: Bearer {TOKEN}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "field_id": "FLD-2025-001",
    "field_name": "North Field",
    "farm_id": "FARM-001",
    "total_area_ha": 50.5,
    "boundary": {
      "type": "Polygon",
      "coordinates": [[[126.977, 37.566], [126.980, 37.566], [126.980, 37.568], [126.977, 37.568], [126.977, 37.566]]]
    },
    "center_point": { "lat": 37.567, "lng": 126.9785 },
    "elevation_m": 125.5,
    "slope_percent": 2.3,
    "soil_type_primary": "LOAM",
    "current_crop": {
      "crop_type": "CORN",
      "variety": "Pioneer P1197",
      "planting_date": "2025-04-15"
    }
  }
}
```

### 3.3 Create Field

```http
POST /v1/fields
Authorization: Bearer {TOKEN}
Content-Type: application/json

{
  "field_name": "South Field",
  "farm_id": "FARM-001",
  "total_area_ha": 35.0,
  "boundary": {
    "type": "Polygon",
    "coordinates": [[[126.975, 37.564], [126.978, 37.564], [126.978, 37.566], [126.975, 37.566], [126.975, 37.564]]]
  },
  "soil_type_primary": "CLAY_LOAM"
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "field_id": "FLD-2025-002",
    "field_name": "South Field",
    "created_at": "2025-01-15T10:00:00Z"
  }
}
```

### 3.4 Update Field

```http
PUT /v1/fields/{FIELD_ID}
Authorization: Bearer {TOKEN}
Content-Type: application/json

{
  "current_crop": {
    "crop_type": "SOYBEAN",
    "variety": "Pioneer 93Y20",
    "planting_date": "2025-05-10"
  }
}
```

### 3.5 Delete Field

```http
DELETE /v1/fields/{FIELD_ID}
Authorization: Bearer {TOKEN}
```

**Response**:
```json
{
  "status": "success",
  "message": "Field FLD-2025-001 deleted successfully"
}
```

---

## Zone Management API

### 4.1 List Zones

```http
GET /v1/fields/{FIELD_ID}/zones
Authorization: Bearer {TOKEN}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "field_id": "FLD-2025-001",
    "zones": [
      {
        "zone_id": "Z1",
        "zone_name": "High Productivity Zone",
        "area_ha": 15.5,
        "classification": "HIGH_YIELD",
        "ndvi_index": 0.78,
        "soil_type": "LOAM",
        "avg_yield_tons_ha": 10.5
      },
      {
        "zone_id": "Z2",
        "zone_name": "Medium Zone",
        "area_ha": 20.0,
        "classification": "MEDIUM_YIELD",
        "ndvi_index": 0.65,
        "soil_type": "CLAY_LOAM",
        "avg_yield_tons_ha": 9.2
      }
    ]
  }
}
```

### 4.2 Create Zone

```http
POST /v1/fields/{FIELD_ID}/zones
Authorization: Bearer {TOKEN}
Content-Type: application/json

{
  "zone_name": "High Yield Area",
  "area_ha": 12.5,
  "boundary": {
    "type": "Polygon",
    "coordinates": [[[126.977, 37.566], [126.978, 37.566], [126.978, 37.567], [126.977, 37.567], [126.977, 37.566]]]
  },
  "classification": "HIGH_YIELD",
  "soil_type": "LOAM"
}
```

### 4.3 Update Zone

```http
PUT /v1/fields/{FIELD_ID}/zones/{ZONE_ID}
Authorization: Bearer {TOKEN}
Content-Type: application/json

{
  "ndvi_index": 0.82,
  "soil_characteristics": {
    "ph": 6.8,
    "organic_matter_percent": 3.5
  }
}
```

---

## VRT Prescription API

### 5.1 Create VRT Prescription

```http
POST /v1/fields/{FIELD_ID}/prescriptions
Authorization: Bearer {TOKEN}
Content-Type: application/json

{
  "application_type": "FERTILIZER",
  "product_name": "Urea 46-0-0",
  "application_date": "2025-04-15",
  "base_rate_kg_ha": 150.0,
  "zones": [
    { "zone_id": "Z1", "rate_kg_ha": 180.0 },
    { "zone_id": "Z2", "rate_kg_ha": 150.0 },
    { "zone_id": "Z3", "rate_kg_ha": 120.0 }
  ]
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "prescription_id": "VRT-2025-001",
    "field_id": "FLD-2025-001",
    "application_type": "FERTILIZER",
    "total_product_kg": 7750.0,
    "savings_vs_uniform": {
      "product_saved_kg": 450.0,
      "cost_saved": 225.0,
      "currency": "USD"
    },
    "created_at": "2025-04-10T08:00:00Z"
  }
}
```

### 5.2 Get Prescription Map (Shapefile)

```http
GET /v1/fields/{FIELD_ID}/prescriptions/{PRESCRIPTION_ID}/shapefile
Authorization: Bearer {TOKEN}
Accept: application/zip
```

**Response**: ZIP file containing `.shp`, `.shx`, `.dbf`, `.prj` files

### 5.3 Get Prescription Map (ISOXML)

```http
GET /v1/fields/{FIELD_ID}/prescriptions/{PRESCRIPTION_ID}/isoxml
Authorization: Bearer {TOKEN}
Accept: application/xml
```

**Response**: ISO 11783-10 TaskData XML

### 5.4 Calculate Optimal VRT Rates

```http
POST /v1/vrt/calculate
Authorization: Bearer {TOKEN}
Content-Type: application/json

{
  "field_id": "FLD-2025-001",
  "application_type": "FERTILIZER",
  "product_name": "Urea 46-0-0",
  "target_yield_tons_ha": 10.5,
  "soil_test_results": [ /* zone soil data */ ],
  "yield_history": [ /* historical yield data */ ]
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "recommended_prescription": {
      "zones": [
        { "zone_id": "Z1", "rate_kg_ha": 185.0, "rationale": "High yield potential" },
        { "zone_id": "Z2", "rate_kg_ha": 150.0, "rationale": "Base rate" },
        { "zone_id": "Z3", "rate_kg_ha": 115.0, "rationale": "Lower N requirement" }
      ],
      "total_savings_percent": 12.5,
      "confidence_score": 0.92
    }
  }
}
```

---

## GPS Guidance API

### 6.1 Create AB Line

```http
POST /v1/fields/{FIELD_ID}/ab-lines
Authorization: Bearer {TOKEN}
Content-Type: application/json

{
  "point_a": { "lat": 37.566, "lng": 126.977 },
  "point_b": { "lat": 37.568, "lng": 126.980 },
  "swath_width_m": 12.0,
  "offset_m": 0.0
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "ab_line_id": "AB-001",
    "field_id": "FLD-2025-001",
    "heading_degrees": 45.5,
    "swath_width_m": 12.0,
    "created_at": "2025-04-15T07:00:00Z"
  }
}
```

### 6.2 Get GPS Guidance Path

```http
GET /v1/fields/{FIELD_ID}/guidance-path?ab_line_id={AB_LINE_ID}&current_lat=37.567&current_lng=126.978
Authorization: Bearer {TOKEN}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "next_waypoint": { "lat": 37.5672, "lng": 126.9782 },
    "distance_to_waypoint_m": 15.5,
    "heading_degrees": 45.8,
    "cross_track_error_m": 0.05,
    "steering_correction_degrees": 0.3
  }
}
```

### 6.3 Real-Time GPS Tracking (WebSocket)

```javascript
const ws = new WebSocket('wss://api.precision-ag.wiastandards.com/v1/gps/track');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    field_id: 'FLD-2025-001',
    vehicle_id: 'TRACTOR-001'
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('GPS Update:', data);
  // { lat: 37.567, lng: 126.978, speed_km_h: 8.5, heading: 45.5 }
};
```

---

## Yield Mapping API

### 7.1 Upload Yield Data

```http
POST /v1/fields/{FIELD_ID}/yield-data
Authorization: Bearer {TOKEN}
Content-Type: application/json

{
  "harvest_date": "2025-10-22",
  "crop_type": "CORN",
  "moisture_percent": 15.5,
  "yield_points": [
    {
      "gps_location": { "lat": 37.5668, "lng": 126.9782 },
      "yield_kg_ha": 10500,
      "timestamp": "2025-10-22T14:23:45Z"
    },
    {
      "gps_location": { "lat": 37.5669, "lng": 126.9783 },
      "yield_kg_ha": 10800,
      "timestamp": "2025-10-22T14:23:50Z"
    }
  ]
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "harvest_id": "HARVEST-2025-001",
    "field_id": "FLD-2025-001",
    "points_uploaded": 2,
    "avg_yield_kg_ha": 10650,
    "total_yield_kg": 538575
  }
}
```

### 7.2 Get Yield Map

```http
GET /v1/fields/{FIELD_ID}/yield-map?harvest_id={HARVEST_ID}&format=geojson
Authorization: Bearer {TOKEN}
```

**Response**: GeoJSON with yield data

### 7.3 Analyze Yield Variability

```http
POST /v1/yield/analyze
Authorization: Bearer {TOKEN}
Content-Type: application/json

{
  "field_id": "FLD-2025-001",
  "harvest_id": "HARVEST-2025-001"
}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "avg_yield_kg_ha": 10500,
    "min_yield_kg_ha": 8200,
    "max_yield_kg_ha": 12400,
    "std_deviation": 850,
    "coefficient_variation": 8.1,
    "zones_analysis": [
      { "zone_id": "Z1", "avg_yield": 11800, "performance": "+12.4%" },
      { "zone_id": "Z2", "avg_yield": 10500, "performance": "0%" },
      { "zone_id": "Z3", "avg_yield": 9200, "performance": "-12.4%" }
    ],
    "recommendations": [
      "Zone Z3 underperforming - check soil compaction",
      "Zone Z1 high performance - replicate management practices"
    ]
  }
}
```

---

## Soil Data API

### 8.1 Upload Soil Sample

```http
POST /v1/fields/{FIELD_ID}/soil-samples
Authorization: Bearer {TOKEN}
Content-Type: application/json

{
  "zone_id": "Z1",
  "gps_location": { "lat": 37.5668, "lng": 126.9782 },
  "sample_date": "2025-03-01",
  "depth_cm": 30,
  "ph": 6.5,
  "organic_matter_percent": 3.2,
  "nutrients": {
    "nitrogen_ppm": 25.5,
    "phosphorus_ppm": 42.0,
    "potassium_ppm": 185.0
  }
}
```

### 8.2 Get Soil Map

```http
GET /v1/fields/{FIELD_ID}/soil-map?property=ph
Authorization: Bearer {TOKEN}
```

**Response**: Interpolated soil property map (GeoJSON)

---

## Weather Integration API

### 9.1 Get Field Weather

```http
GET /v1/weather/field/{FIELD_ID}?days=7
Authorization: Bearer {TOKEN}
```

**Response**:
```json
{
  "status": "success",
  "data": {
    "field_id": "FLD-2025-001",
    "current": {
      "temperature_c": 22.5,
      "humidity_percent": 65,
      "wind_speed_km_h": 12.0,
      "precipitation_mm": 0.0
    },
    "forecast": [
      {
        "date": "2025-04-16",
        "temp_high_c": 24.0,
        "temp_low_c": 15.0,
        "precipitation_mm": 5.2,
        "wind_speed_km_h": 15.0
      }
    ],
    "growing_degree_days": {
      "accumulated": 485.5,
      "base_temp_c": 10
    }
  }
}
```

---

## Real-Time Telemetry API

### 10.1 Stream Tractor Telemetry (WebSocket)

```javascript
const ws = new WebSocket('wss://api.precision-ag.wiastandards.com/v1/telemetry/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    vehicle_id: 'TRACTOR-001',
    data_types: ['gps', 'speed', 'fuel', 'application_rate']
  }));
};

ws.onmessage = (event) => {
  const telemetry = JSON.parse(event.data);
  console.log(telemetry);
  /*
  {
    timestamp: '2025-04-15T10:23:45Z',
    vehicle_id: 'TRACTOR-001',
    gps: { lat: 37.567, lng: 126.978, accuracy_cm: 2.0 },
    speed_km_h: 8.5,
    fuel_level_percent: 75.5,
    application_rate_kg_ha: 180.0,
    implement_status: 'ACTIVE'
  }
  */
};
```

### 10.2 Publish Sensor Data (MQTT)

```
Topic: wia/precision-ag/field/{FIELD_ID}/sensor/{SENSOR_ID}

Payload:
{
  "timestamp": "2025-04-15T10:25:00Z",
  "sensor_id": "SOIL-SENSOR-001",
  "zone_id": "Z1",
  "gps_location": { "lat": 37.5668, "lng": 126.9782 },
  "moisture_percent": 22.5,
  "temperature_c": 18.5,
  "battery_percent": 85
}
```

---

## Error Codes

### 11.1 HTTP Status Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Missing/invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

### 11.2 Application Error Codes

```json
{
  "status": "error",
  "error": {
    "code": "E-FIELD-001",
    "message": "Field area must be between 0.1 and 100,000 hectares",
    "field": "total_area_ha",
    "value": 150000
  }
}
```

| Code | Description |
|------|-------------|
| `E-FIELD-001` | Invalid field area |
| `E-FIELD-002` | Invalid boundary polygon |
| `E-ZONE-001` | Zone areas do not match field area |
| `E-VRT-001` | Invalid application rate |
| `E-GPS-001` | Invalid GPS coordinates |
| `E-AUTH-001` | Invalid API key |
| `E-RATE-001` | Rate limit exceeded |

---

## Rate Limits

### 12.1 Standard Limits

| Tier | Requests/minute | Requests/day |
|------|----------------|--------------|
| Free | 60 | 10,000 |
| Standard | 300 | 100,000 |
| Premium | 1,000 | 1,000,000 |
| Enterprise | Unlimited | Unlimited |

**Rate Limit Headers**:
```http
X-RateLimit-Limit: 300
X-RateLimit-Remaining: 285
X-RateLimit-Reset: 1642341600
```

---

## Examples

### 13.1 Complete Workflow Example

```javascript
// 1. Create field
const field = await fetch('https://api.precision-ag.wiastandards.com/v1/fields', {
  method: 'POST',
  headers: {
    'Authorization': 'Bearer ' + token,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    field_name: 'North Field',
    farm_id: 'FARM-001',
    total_area_ha: 50.5,
    boundary: { /* GeoJSON polygon */ }
  })
});

// 2. Create management zones
const zone = await fetch(`https://api.precision-ag.wiastandards.com/v1/fields/${fieldId}/zones`, {
  method: 'POST',
  headers: {
    'Authorization': 'Bearer ' + token,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    zone_name: 'High Yield Zone',
    area_ha: 15.5,
    classification: 'HIGH_YIELD'
  })
});

// 3. Create VRT prescription
const prescription = await fetch(`https://api.precision-ag.wiastandards.com/v1/fields/${fieldId}/prescriptions`, {
  method: 'POST',
  headers: {
    'Authorization': 'Bearer ' + token,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    application_type: 'FERTILIZER',
    product_name: 'Urea 46-0-0',
    zones: [
      { zone_id: 'Z1', rate_kg_ha: 180.0 },
      { zone_id: 'Z2', rate_kg_ha: 150.0 }
    ]
  })
});

// 4. Download prescription shapefile
const shapefile = await fetch(`https://api.precision-ag.wiastandards.com/v1/fields/${fieldId}/prescriptions/${prescriptionId}/shapefile`, {
  headers: { 'Authorization': 'Bearer ' + token }
});
```

---

**© 2025 WIA Standards - MIT License**
**弘益人間 (Hongik Ingan) - Benefit All Humanity**
