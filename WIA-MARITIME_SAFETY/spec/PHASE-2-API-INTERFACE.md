# WIA-MARITIME_SAFETY: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document defines the REST API interfaces for Maritime Safety systems, enabling vessel tracking, weather monitoring, safety alerts, and emergency response coordination.

## 2. Base Configuration

### 2.1 API Endpoint
```
Production: https://api.wia.org/maritime-safety/v1
Staging: https://staging-api.wia.org/maritime-safety/v1
Development: http://localhost:8080/api/v1
```

### 2.2 Authentication

All API requests require authentication using API keys:

```http
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json
```

### 2.3 Rate Limiting

| Tier | Requests/minute | Requests/day |
|------|----------------|--------------|
| Free | 60 | 10,000 |
| Standard | 600 | 100,000 |
| Enterprise | 6,000 | 1,000,000 |
| Emergency | Unlimited | Unlimited |

## 3. Vessel Tracking API

### 3.1 Get Vessel Position

**Endpoint:** `GET /vessels/{mmsi}/position`

**Request:**
```http
GET /vessels/367123450/position HTTP/1.1
Host: api.wia.org
Authorization: Bearer YOUR_API_KEY
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "mmsi": "367123450",
    "vesselName": "PACIFIC GLORY",
    "timestamp": "2026-01-12T14:30:00Z",
    "position": {
      "latitude": 37.8044,
      "longitude": -122.4162,
      "accuracy": true
    },
    "navigation": {
      "status": "Under way using engine",
      "speedOverGround": 12.5,
      "courseOverGround": 285.3,
      "heading": 287
    }
  }
}
```

### 3.2 Track Multiple Vessels

**Endpoint:** `POST /vessels/track`

**Request:**
```json
{
  "mmsiList": ["367123450", "431234567", "235012345"],
  "fields": ["position", "navigation", "identity"],
  "realtime": true
}
```

**Response:**
```json
{
  "status": "success",
  "data": [
    {
      "mmsi": "367123450",
      "position": {...},
      "navigation": {...}
    },
    {
      "mmsi": "431234567",
      "position": {...},
      "navigation": {...}
    }
  ],
  "metadata": {
    "totalVessels": 3,
    "foundVessels": 2,
    "lastUpdate": "2026-01-12T14:30:00Z"
  }
}
```

### 3.3 Search Vessels by Area

**Endpoint:** `POST /vessels/search`

**Request:**
```json
{
  "area": {
    "type": "circle",
    "center": {
      "latitude": 35.6762,
      "longitude": 139.6503
    },
    "radius": 50
  },
  "filters": {
    "vesselTypes": ["cargo", "tanker"],
    "minLength": 100,
    "flags": ["US", "JP", "UK"]
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "vessels": [
      {
        "mmsi": "431234567",
        "vesselName": "OCEAN STAR",
        "vesselType": "cargo",
        "position": {...},
        "distance": 12.5
      }
    ],
    "totalFound": 15,
    "page": 1,
    "pageSize": 10
  }
}
```

## 4. Weather & Environment API

### 4.1 Get Marine Weather

**Endpoint:** `GET /weather`

**Parameters:**
- `latitude` (required): Latitude in decimal degrees
- `longitude` (required): Longitude in decimal degrees
- `forecast` (optional): Include forecast (true/false)

**Request:**
```http
GET /weather?latitude=35.6762&longitude=139.6503&forecast=true HTTP/1.1
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "timestamp": "2026-01-12T14:30:00Z",
    "position": {
      "latitude": 35.6762,
      "longitude": 139.6503
    },
    "current": {
      "windSpeed": 15.5,
      "windDirection": 270,
      "waveHeight": 2.5,
      "wavePeriod": 8,
      "visibility": 10,
      "seaTemperature": 18.5,
      "airTemperature": 20.2,
      "barometricPressure": 1013.2
    },
    "forecast": {
      "validUntil": "2026-01-13T00:00:00Z",
      "warnings": [
        "Strong wind warning: NW winds 20-25 knots expected"
      ],
      "hourly": [...]
    }
  }
}
```

### 4.2 Get Route Weather

**Endpoint:** `POST /weather/route`

**Request:**
```json
{
  "waypoints": [
    {"latitude": 35.6762, "longitude": 139.6503},
    {"latitude": 37.8044, "longitude": -122.4162}
  ],
  "departureTime": "2026-01-12T18:00:00Z",
  "speed": 15
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "route": {
      "distance": 4536.2,
      "duration": 302.4,
      "segments": [
        {
          "from": 0,
          "to": 1,
          "weather": {
            "avgWindSpeed": 18.5,
            "maxWaveHeight": 3.2,
            "warnings": []
          }
        }
      ]
    },
    "recommendations": [
      "Consider delaying departure by 6 hours for better conditions"
    ]
  }
}
```

## 5. Safety Alert API

### 5.1 Create Alert

**Endpoint:** `POST /alerts`

**Request:**
```json
{
  "severity": "critical",
  "category": "collision",
  "position": {
    "latitude": 35.6762,
    "longitude": 139.6503
  },
  "vessel": {
    "mmsi": "431234567",
    "name": "OCEAN STAR"
  },
  "description": "Collision risk with nearby vessel",
  "autoNotify": true
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "alertId": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": "2026-01-12T14:30:00Z",
    "status": "active",
    "notifiedParties": [
      "Coast Guard Station Tokyo",
      "Vessel MMSI 431234568"
    ]
  }
}
```

### 5.2 Get Active Alerts

**Endpoint:** `GET /alerts`

**Parameters:**
- `severity` (optional): Filter by severity
- `category` (optional): Filter by category
- `area` (optional): Geographic area filter
- `status` (optional): Alert status

**Response:**
```json
{
  "status": "success",
  "data": {
    "alerts": [
      {
        "id": "550e8400-e29b-41d4-a716-446655440000",
        "timestamp": "2026-01-12T14:30:00Z",
        "severity": "critical",
        "category": "collision",
        "position": {...},
        "vessel": {...},
        "status": "active"
      }
    ],
    "total": 5,
    "page": 1
  }
}
```

### 5.3 Update Alert Status

**Endpoint:** `PATCH /alerts/{alertId}`

**Request:**
```json
{
  "status": "acknowledged",
  "responder": {
    "type": "coast_guard",
    "identifier": "USCG-SF-001"
  },
  "notes": "Units dispatched, ETA 15 minutes"
}
```

## 6. Route Planning API

### 6.1 Calculate Safe Route

**Endpoint:** `POST /routes/calculate`

**Request:**
```json
{
  "origin": {
    "latitude": 35.6762,
    "longitude": 139.6503
  },
  "destination": {
    "latitude": 37.8044,
    "longitude": -122.4162
  },
  "vessel": {
    "type": "cargo",
    "length": 200,
    "draft": 12.5,
    "maxSpeed": 20
  },
  "preferences": {
    "avoidWeather": true,
    "avoidTraffic": true,
    "minimizeFuel": true,
    "departureTime": "2026-01-13T06:00:00Z"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "route": {
      "id": "route-550e8400",
      "waypoints": [
        {"latitude": 35.6762, "longitude": 139.6503, "eta": "2026-01-13T06:00:00Z"},
        {"latitude": 37.8044, "longitude": -122.4162, "eta": "2026-01-25T18:30:00Z"}
      ],
      "distance": 4536.2,
      "duration": 302.5,
      "fuelEstimate": 1250.5,
      "warnings": [],
      "alternateRoutes": [...]
    }
  }
}
```

## 7. Port & Berth API

### 7.1 Get Port Information

**Endpoint:** `GET /ports/{portId}`

**Response:**
```json
{
  "status": "success",
  "data": {
    "portId": "USOAK",
    "name": "Port of Oakland",
    "position": {
      "latitude": 37.7955,
      "longitude": -122.2801
    },
    "facilities": {
      "maxDraft": 15.2,
      "maxLength": 366,
      "berths": 35,
      "services": ["bunkering", "repairs", "provisions"]
    },
    "currentConditions": {
      "berthsAvailable": 12,
      "averageWaitTime": 4.5,
      "restrictions": []
    }
  }
}
```

### 7.2 Request Berth Assignment

**Endpoint:** `POST /ports/{portId}/berth-request`

**Request:**
```json
{
  "vessel": {
    "mmsi": "367123450",
    "name": "PACIFIC GLORY",
    "length": 200,
    "draft": 12.5
  },
  "eta": "2026-01-15T08:00:00Z",
  "serviceRequired": ["bunkering", "provisions"],
  "cargoType": "containers",
  "stayDuration": 24
}
```

## 8. Error Handling

### 8.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "INVALID_POSITION",
    "message": "Position coordinates out of valid range",
    "details": {
      "field": "latitude",
      "value": 95.0,
      "constraint": "Must be between -90 and 90"
    }
  },
  "timestamp": "2026-01-12T14:30:00Z"
}
```

### 8.2 Common Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_MMSI | 400 | Invalid MMSI format |
| VESSEL_NOT_FOUND | 404 | Vessel not found in database |
| INVALID_POSITION | 400 | Position coordinates invalid |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| UNAUTHORIZED | 401 | Invalid or missing API key |
| FORBIDDEN | 403 | Insufficient permissions |
| INTERNAL_ERROR | 500 | Server error |

## 9. WebSocket API (Real-time)

### 9.1 Connection

```javascript
const ws = new WebSocket('wss://api.wia.org/maritime-safety/v1/stream');
ws.send(JSON.stringify({
  action: 'subscribe',
  channels: ['vessels', 'alerts', 'weather'],
  filters: {
    area: {
      type: 'circle',
      center: {latitude: 35.6762, longitude: 139.6503},
      radius: 50
    }
  }
}));
```

### 9.2 Real-time Updates

```json
{
  "channel": "vessels",
  "event": "position_update",
  "data": {
    "mmsi": "367123450",
    "position": {...},
    "timestamp": "2026-01-12T14:30:05Z"
  }
}
```

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
