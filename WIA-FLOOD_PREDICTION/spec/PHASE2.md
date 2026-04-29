# WIA-FLOOD_PREDICTION Specification - PHASE 2

**Version:** 1.0.0
**Last Updated:** 2026-01-11

## API Design & Data Models

This phase defines the REST API endpoints, WebSocket protocols, data models, authentication, and integration patterns for the WIA-FLOOD_PREDICTION system.

---

## Table of Contents

1. [API Overview](#api-overview)
2. [Data Models](#data-models)
3. [REST API Endpoints](#rest-api-endpoints)
4. [WebSocket Protocol](#websocket-protocol)
5. [External API Integrations](#external-api-integrations)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)

---

## API Overview

### Base URL

```
Production:  https://api.wia-flood.org/v1
Staging:     https://api-staging.wia-flood.org/v1
Development: http://localhost:8000/v1
```

### Authentication

All API requests require authentication via API key or OAuth 2.0:

**API Key (Header)**:
```http
Authorization: Bearer wia_live_abc123xyz456...
```

**OAuth 2.0**:
```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### Request Format

- **Content-Type**: `application/json`
- **Accept**: `application/json` or `application/geo+json` (for spatial data)
- **Encoding**: UTF-8

### Response Format

**Success (200)**:
```json
{
  "status": "success",
  "data": { ... },
  "metadata": {
    "timestamp": "2026-01-11T14:30:00Z",
    "request_id": "req_abc123",
    "api_version": "v1"
  }
}
```

**Error (4xx/5xx)**:
```json
{
  "status": "error",
  "error": {
    "code": "INVALID_LOCATION",
    "message": "Latitude must be between -90 and 90",
    "details": { "field": "lat", "value": 95.0 }
  },
  "metadata": {
    "timestamp": "2026-01-11T14:30:00Z",
    "request_id": "req_abc123"
  }
}
```

---

## Data Models

### 1. FloodPrediction

Primary model for flood forecasts.

```typescript
interface FloodPrediction {
  id: string;                    // Unique prediction ID (UUID)
  location: GeoPoint;             // Geographic coordinates
  forecast_date: string;          // ISO 8601 datetime (UTC)
  issued_at: string;              // When prediction was generated
  risk_level: RiskLevel;          // Categorical risk assessment
  probability: number;            // 0.0 - 1.0 (flood probability)
  confidence: number;             // 0.0 - 1.0 (model confidence)

  // Physical predictions
  predicted_depth_meters: number; // Maximum flood depth
  predicted_velocity_ms: number;  // Flow velocity (m/s)
  affected_area_km2: number;      // Inundation extent
  peak_time: string;              // ISO 8601 (when peak occurs)

  // Metadata
  model_version: string;          // e.g., "LSTM-v2.1_CNN-v1.8"
  data_sources: string[];         // ["Sentinel-1", "NOAA-GFS", ...]
  uncertainty_range: {
    depth_min: number;
    depth_max: number;
  };

  // Associated data
  alerts?: FloodAlert[];
  historical_comparison?: HistoricalEvent;
}

type RiskLevel = 'low' | 'medium' | 'high' | 'extreme';

interface GeoPoint {
  lat: number;   // Latitude (-90 to 90)
  lng: number;   // Longitude (-180 to 180)
  elevation_m?: number;
}
```

**Example**:
```json
{
  "id": "pred_2026011114_38.9072_-77.0369",
  "location": {"lat": 38.9072, "lng": -77.0369, "elevation_m": 5.2},
  "forecast_date": "2026-01-18T12:00:00Z",
  "issued_at": "2026-01-11T14:30:00Z",
  "risk_level": "high",
  "probability": 0.87,
  "confidence": 0.82,
  "predicted_depth_meters": 2.4,
  "predicted_velocity_ms": 1.2,
  "affected_area_km2": 3.7,
  "peak_time": "2026-01-18T18:00:00Z",
  "model_version": "LSTM-v2.1_CNN-v1.8_Ensemble",
  "data_sources": ["Sentinel-1", "Sentinel-2", "NOAA-GFS", "USGS-01646500"],
  "uncertainty_range": {
    "depth_min": 1.8,
    "depth_max": 3.1
  }
}
```

### 2. RiverGauge

Real-time and forecasted river water levels.

```typescript
interface RiverGauge {
  id: string;                     // USGS site code (e.g., "01646500")
  name: string;                   // Gauge name
  location: GeoPoint;
  river_name: string;

  // Current conditions
  current_level_m: number;        // Water level above datum
  current_discharge_m3s: number;  // Flow rate
  measured_at: string;            // ISO 8601 timestamp

  // Thresholds
  action_stage_m: number;         // Level requiring action
  flood_stage_m: number;          // Minor flood level
  moderate_flood_stage_m: number;
  major_flood_stage_m: number;

  // Forecast
  forecast: GaugeForecast[];

  // Status
  status: 'normal' | 'action' | 'minor_flood' | 'moderate_flood' | 'major_flood';
  data_quality: 'good' | 'fair' | 'poor';
}

interface GaugeForecast {
  timestamp: string;              // ISO 8601
  level_m: number;
  discharge_m3s: number;
  confidence: number;
}
```

**Example**:
```json
{
  "id": "01646500",
  "name": "POTOMAC RIVER AT LITTLE FALLS PUMP STATION",
  "location": {"lat": 38.9492, "lng": -77.1281, "elevation_m": 11.56},
  "river_name": "Potomac River",
  "current_level_m": 2.13,
  "current_discharge_m3s": 142.5,
  "measured_at": "2026-01-11T14:15:00Z",
  "action_stage_m": 3.05,
  "flood_stage_m": 4.88,
  "moderate_flood_stage_m": 6.10,
  "major_flood_stage_m": 7.92,
  "forecast": [
    {"timestamp": "2026-01-12T00:00:00Z", "level_m": 2.35, "discharge_m3s": 165.0, "confidence": 0.95},
    {"timestamp": "2026-01-13T00:00:00Z", "level_m": 3.20, "discharge_m3s": 245.0, "confidence": 0.88},
    {"timestamp": "2026-01-14T00:00:00Z", "level_m": 5.10, "discharge_m3s": 520.0, "confidence": 0.75}
  ],
  "status": "normal",
  "data_quality": "good"
}
```

### 3. FloodAlert

Real-time alerts for imminent flooding.

```typescript
interface FloodAlert {
  id: string;
  alert_type: AlertType;
  severity: 'advisory' | 'watch' | 'warning' | 'emergency';

  // Location
  affected_area: GeoJSON.Polygon;  // GeoJSON polygon
  affected_counties: string[];     // FIPS codes or names
  population_at_risk: number;

  // Timing
  issued_at: string;
  effective_at: string;
  expires_at: string;

  // Content
  headline: string;
  description: string;
  instructions: string;

  // Associated data
  prediction_id?: string;          // Link to FloodPrediction
  gauge_ids?: string[];            // Affected gauges

  // Distribution
  channels: AlertChannel[];
  recipients: number;              // Total recipients
}

type AlertType =
  | 'flash_flood'
  | 'river_flood'
  | 'coastal_flood'
  | 'dam_failure'
  | 'levee_breach';

type AlertChannel =
  | 'sms'
  | 'email'
  | 'push_notification'
  | 'websocket'
  | 'emergency_broadcast'
  | 'sirens';
```

**Example**:
```json
{
  "id": "alert_ff_dc_20260118",
  "alert_type": "flash_flood",
  "severity": "warning",
  "affected_area": {
    "type": "Polygon",
    "coordinates": [[[-77.12, 38.88], [-77.01, 38.88], [-77.01, 38.96], [-77.12, 38.96], [-77.12, 38.88]]]
  },
  "affected_counties": ["District of Columbia", "Montgomery County, MD"],
  "population_at_risk": 47500,
  "issued_at": "2026-01-11T14:45:00Z",
  "effective_at": "2026-01-11T15:00:00Z",
  "expires_at": "2026-01-12T03:00:00Z",
  "headline": "Flash Flood Warning for Northwest DC and Bethesda, MD until 3:00 AM EST",
  "description": "Heavy rainfall of 2-3 inches expected in the next 6 hours. Rapid flooding of low-lying areas, roads, and small streams likely.",
  "instructions": "Move to higher ground immediately. Do not drive through flooded areas. Turn around, don't drown.",
  "prediction_id": "pred_2026011114_38.9072_-77.0369",
  "gauge_ids": ["01646500", "01647850"],
  "channels": ["sms", "email", "push_notification", "websocket"],
  "recipients": 12500
}
```

### 4. WeatherData

Meteorological inputs for flood prediction.

```typescript
interface WeatherData {
  id: string;
  location: GeoPoint;
  timestamp: string;
  source: 'NOAA-GFS' | 'ECMWF-IFS' | 'HRRR' | 'observed';

  // Precipitation
  precipitation_mm: number;       // Accumulated (hourly/daily)
  precipitation_probability: number;

  // Temperature & pressure
  temperature_c: number;
  dew_point_c: number;
  pressure_hpa: number;

  // Wind
  wind_speed_ms: number;
  wind_direction_deg: number;

  // Forecast horizon
  forecast_hours?: number;        // null if observed data
}
```

### 5. SatelliteImage

Metadata for satellite imagery.

```typescript
interface SatelliteImage {
  id: string;
  satellite: 'Sentinel-1' | 'Sentinel-2' | 'MODIS' | 'VIIRS';
  product_type: string;           // e.g., "GRD", "L2A"

  // Spatial
  bbox: GeoJSON.BBox;             // [minLng, minLat, maxLng, maxLat]
  footprint: GeoJSON.Polygon;
  resolution_m: number;

  // Temporal
  acquisition_date: string;
  processing_date: string;

  // Technical
  bands: string[];                // e.g., ["VV", "VH"] or ["B3", "B8"]
  cloud_cover_pct?: number;

  // Storage
  storage_url: string;            // S3/MinIO URL
  thumbnail_url: string;
  file_size_mb: number;

  // Derived products
  water_mask_url?: string;        // Binary water/no-water
  ndwi_url?: string;
}
```

### 6. HistoricalEvent

Past flood events for validation and comparison.

```typescript
interface HistoricalEvent {
  id: string;
  name: string;                   // e.g., "Hurricane Ida Remnants 2021"
  event_type: 'flash_flood' | 'river_flood' | 'coastal_storm' | 'hurricane';

  // Timing
  start_date: string;
  end_date: string;

  // Location
  affected_area: GeoJSON.Polygon;
  affected_regions: string[];

  // Impact
  max_depth_m: number;
  inundated_area_km2: number;
  fatalities?: number;
  economic_damage_usd?: number;

  // Data
  satellite_images: string[];     // Image IDs
  gauge_records: string[];        // Gauge IDs

  // Analysis
  causes: string[];               // ["heavy_rainfall", "dam_release", ...]
  lessons_learned?: string;
}
```

---

## REST API Endpoints

### Predictions

#### `GET /predictions`

Get flood predictions for a location or area.

**Query Parameters**:
- `lat` (required): Latitude (-90 to 90)
- `lng` (required): Longitude (-180 to 180)
- `radius_km` (optional): Search radius (default: 10, max: 100)
- `start_date` (optional): Filter by forecast date (ISO 8601)
- `end_date` (optional): Filter end (ISO 8601)
- `risk_level` (optional): Filter by risk (`low`, `medium`, `high`, `extreme`)
- `min_probability` (optional): Minimum flood probability (0.0-1.0)

**Example Request**:
```bash
curl -X GET "https://api.wia-flood.org/v1/predictions?lat=38.9072&lng=-77.0369&radius_km=20&min_probability=0.5" \
  -H "Authorization: Bearer wia_live_abc123"
```

**Response (200)**:
```json
{
  "status": "success",
  "data": {
    "predictions": [
      {
        "id": "pred_2026011114_38.9072_-77.0369",
        "location": {"lat": 38.9072, "lng": -77.0369},
        "forecast_date": "2026-01-18T12:00:00Z",
        "risk_level": "high",
        "probability": 0.87,
        "predicted_depth_meters": 2.4
      }
    ],
    "count": 1
  },
  "metadata": {
    "timestamp": "2026-01-11T14:30:00Z",
    "request_id": "req_abc123"
  }
}
```

#### `GET /predictions/{id}`

Get detailed prediction by ID.

**Response (200)**:
```json
{
  "status": "success",
  "data": {
    "prediction": { /* Full FloodPrediction object */ }
  }
}
```

#### `POST /predictions/batch`

Get predictions for multiple locations at once.

**Request Body**:
```json
{
  "locations": [
    {"lat": 38.9072, "lng": -77.0369},
    {"lat": 40.7128, "lng": -74.0060},
    {"lat": 29.7604, "lng": -95.3698}
  ],
  "forecast_date": "2026-01-18T00:00:00Z"
}
```

**Response (200)**:
```json
{
  "status": "success",
  "data": {
    "predictions": [ /* Array of FloodPrediction */ ],
    "count": 3
  }
}
```

### River Gauges

#### `GET /gauges`

List river gauges within an area.

**Query Parameters**:
- `lat`, `lng`, `radius_km` (spatial filter)
- `status` (optional): Filter by status (`normal`, `action`, `flood`, etc.)
- `river_name` (optional): Filter by river name

**Example Request**:
```bash
curl "https://api.wia-flood.org/v1/gauges?lat=38.9072&lng=-77.0369&radius_km=50"
```

#### `GET /gauges/{id}`

Get detailed gauge data with current conditions and forecast.

**Response (200)**:
```json
{
  "status": "success",
  "data": {
    "gauge": { /* Full RiverGauge object */ }
  }
}
```

#### `GET /gauges/{id}/history`

Get historical water level data.

**Query Parameters**:
- `start_date` (required)
- `end_date` (required)
- `interval` (optional): `15min`, `hourly`, `daily` (default: `hourly`)

**Response (200)**:
```json
{
  "status": "success",
  "data": {
    "gauge_id": "01646500",
    "time_series": [
      {"timestamp": "2026-01-01T00:00:00Z", "level_m": 1.85, "discharge_m3s": 120.5},
      {"timestamp": "2026-01-01T01:00:00Z", "level_m": 1.87, "discharge_m3s": 122.0}
    ]
  }
}
```

### Alerts

#### `GET /alerts`

Get active flood alerts.

**Query Parameters**:
- `lat`, `lng`, `radius_km` (spatial filter)
- `severity` (optional): `advisory`, `watch`, `warning`, `emergency`
- `alert_type` (optional): `flash_flood`, `river_flood`, etc.
- `active_only` (optional): boolean (default: true)

**Response (200)**:
```json
{
  "status": "success",
  "data": {
    "alerts": [ /* Array of FloodAlert */ ],
    "count": 2
  }
}
```

#### `POST /alerts/subscribe`

Subscribe to alerts for a location.

**Request Body**:
```json
{
  "location": {"lat": 38.9072, "lng": -77.0369},
  "radius_km": 10,
  "channels": ["email", "sms", "websocket"],
  "contact": {
    "email": "user@example.com",
    "phone": "+12025551234"
  },
  "filters": {
    "min_severity": "watch",
    "alert_types": ["flash_flood", "river_flood"]
  }
}
```

**Response (201)**:
```json
{
  "status": "success",
  "data": {
    "subscription_id": "sub_abc123",
    "status": "active"
  }
}
```

### Weather

#### `GET /weather/forecast`

Get weather forecast for a location.

**Query Parameters**:
- `lat`, `lng` (required)
- `hours` (optional): Forecast horizon (default: 48, max: 384)
- `source` (optional): `NOAA-GFS`, `ECMWF-IFS`, `HRRR`

**Response (200)**:
```json
{
  "status": "success",
  "data": {
    "location": {"lat": 38.9072, "lng": -77.0369},
    "forecast": [ /* Array of WeatherData */ ]
  }
}
```

### Satellite Imagery

#### `GET /satellite/images`

Search for satellite images.

**Query Parameters**:
- `bbox` (required): Bounding box `minLng,minLat,maxLng,maxLat`
- `start_date`, `end_date` (required)
- `satellite` (optional): `Sentinel-1`, `Sentinel-2`, `MODIS`, `VIIRS`
- `max_cloud_cover` (optional): Maximum cloud cover % (default: 30)

**Example Request**:
```bash
curl "https://api.wia-flood.org/v1/satellite/images?bbox=-77.2,38.8,-77.0,39.0&start_date=2026-01-01&end_date=2026-01-11&satellite=Sentinel-2"
```

**Response (200)**:
```json
{
  "status": "success",
  "data": {
    "images": [ /* Array of SatelliteImage */ ],
    "count": 5
  }
}
```

#### `GET /satellite/images/{id}/download`

Download satellite image (redirects to storage URL).

**Response (302)**: Redirect to signed S3/MinIO URL

---

## WebSocket Protocol

Real-time streaming of flood predictions and alerts.

### Connection

**Endpoint**: `wss://api.wia-flood.org/v1/ws`

**Authentication**: Send API key in first message after connection.

```javascript
const ws = new WebSocket('wss://api.wia-flood.org/v1/ws');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    api_key: 'wia_live_abc123'
  }));
};
```

### Message Types

#### 1. Subscribe to Location

```json
{
  "type": "subscribe",
  "channel": "predictions",
  "location": {"lat": 38.9072, "lng": -77.0369},
  "radius_km": 10
}
```

#### 2. Subscribe to Alerts

```json
{
  "type": "subscribe",
  "channel": "alerts",
  "location": {"lat": 38.9072, "lng": -77.0369},
  "radius_km": 50,
  "min_severity": "watch"
}
```

#### 3. Prediction Update

Server → Client when new prediction is available:

```json
{
  "type": "prediction_update",
  "timestamp": "2026-01-11T14:30:00Z",
  "data": { /* FloodPrediction object */ }
}
```

#### 4. Alert Notification

Server → Client when new alert is issued:

```json
{
  "type": "alert_notification",
  "timestamp": "2026-01-11T14:45:00Z",
  "data": { /* FloodAlert object */ }
}
```

#### 5. Heartbeat

Sent every 30 seconds to keep connection alive:

```json
{
  "type": "heartbeat",
  "timestamp": "2026-01-11T14:30:00Z"
}
```

---

## External API Integrations

### 1. Copernicus Sentinel Hub

**Purpose**: Download Sentinel-1/2 imagery

**API Endpoint**: `https://services.sentinel-hub.com/api/v1/`

**Authentication**: OAuth 2.0

**Example Request** (Sentinel-2 NDWI):
```python
import requests

response = requests.post(
    'https://services.sentinel-hub.com/api/v1/process',
    headers={'Authorization': f'Bearer {oauth_token}'},
    json={
        'input': {
            'bounds': {'bbox': [-77.2, 38.8, -77.0, 39.0]},
            'data': [{'type': 'sentinel-2-l2a', 'dataFilter': {'timeRange': {'from': '2026-01-01', 'to': '2026-01-11'}}}]
        },
        'output': {'responses': [{'identifier': 'default', 'format': {'type': 'image/tiff'}}]},
        'evalscript': 'return [(B03-B08)/(B03+B08)]'  # NDWI
    }
)
```

### 2. USGS Water Services

**Purpose**: Real-time river gauge data

**API Endpoint**: `https://waterservices.usgs.gov/rest/`

**Example Request**:
```python
import requests

response = requests.get(
    'https://waterservices.usgs.gov/nwis/iv/',
    params={
        'sites': '01646500',
        'parameterCd': '00065,00060',  # Stage & discharge
        'format': 'json',
        'siteStatus': 'all'
    }
)
data = response.json()
```

### 3. NOAA GFS Data

**Purpose**: Weather forecasts (precipitation, temperature)

**Data Source**: NOAA NOMADS server

**Example Request** (via Python):
```python
from siphon.catalog import TDSCatalog

cat = TDSCatalog('https://thredds.ucar.edu/thredds/catalog/grib/NCEP/GFS/Global_0p25deg/latest.xml')
dataset = cat.datasets[0]
ncss = dataset.subset()

query = ncss.query()
query.lonlat_point(-77.0369, 38.9072)
query.variables('Precipitation_rate_surface', 'Temperature_surface')
query.time_range(start, end)

data = ncss.get_data(query)
```

### 4. NASA SMAP (Soil Moisture)

**Purpose**: Soil moisture data for hydrological models

**API**: NASA Earthdata Search

**Example Request**:
```bash
curl -H "Authorization: Bearer {earthdata_token}" \
  "https://n5eil01u.ecs.nsidc.org/SMAP/SPL3SMP.008/2026.01.11/SMAP_L3_SM_P_20260111_R18290_001.h5"
```

### 5. FEMA IPAWS (Integrated Public Alert & Warning System)

**Purpose**: Push flood alerts to emergency systems

**Protocol**: CAP (Common Alerting Protocol) XML over HTTPS POST

**Example CAP Message**:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<alert xmlns="urn:oasis:names:tc:emergency:cap:1.2">
  <identifier>WIA-FF-DC-20260111-001</identifier>
  <sender>flood-prediction@wia-flood.org</sender>
  <sent>2026-01-11T14:45:00-05:00</sent>
  <status>Actual</status>
  <msgType>Alert</msgType>
  <scope>Public</scope>
  <info>
    <category>Met</category>
    <event>Flash Flood Warning</event>
    <urgency>Immediate</urgency>
    <severity>Severe</severity>
    <certainty>Likely</certainty>
    <areaDesc>Northwest Washington DC, Montgomery County MD</areaDesc>
    <polygon>38.88,-77.12 38.88,-77.01 38.96,-77.01 38.96,-77.12 38.88,-77.12</polygon>
  </info>
</alert>
```

---

## Error Handling

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_API_KEY` | 401 | API key missing or invalid |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INVALID_LOCATION` | 400 | Invalid lat/lng coordinates |
| `PREDICTION_NOT_FOUND` | 404 | Prediction ID doesn't exist |
| `GAUGE_NOT_FOUND` | 404 | Gauge ID doesn't exist |
| `DATA_UNAVAILABLE` | 503 | Upstream data source unavailable |
| `MODEL_ERROR` | 500 | ML model inference failed |
| `INVALID_DATE_RANGE` | 400 | Invalid start/end dates |

### Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "INVALID_LOCATION",
    "message": "Latitude must be between -90 and 90",
    "details": {
      "field": "lat",
      "value": 95.0,
      "constraint": "min: -90, max: 90"
    }
  },
  "metadata": {
    "timestamp": "2026-01-11T14:30:00Z",
    "request_id": "req_abc123",
    "documentation_url": "https://docs.wia-flood.org/errors#INVALID_LOCATION"
  }
}
```

---

## Rate Limiting

### Tiers

| Tier | Requests/Hour | WebSocket Connections | Cost |
|------|---------------|----------------------|------|
| **Free** | 100 | 1 | $0 |
| **Developer** | 1,000 | 5 | $29/mo |
| **Professional** | 10,000 | 25 | $149/mo |
| **Enterprise** | Unlimited | Unlimited | Custom |

### Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1704988800
```

### Exceeding Limits

**Response (429)**:
```json
{
  "status": "error",
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit of 1000 requests/hour exceeded",
    "retry_after": 1342
  }
}
```

---

## Next Steps (PHASE 3)

- Security & authentication details
- Compliance (GDPR, SOC 2, FedRAMP)
- Performance benchmarks
- Disaster response integration workflows

---

**© 2026 WIA | 弘益人間 (Benefit All Humanity)**
