# WIA Cryo-Transport API Standard
## Phase 2 Specification

**Version**: 1.0.0  
**Status**: Complete  
**License**: MIT

---

## API Endpoints

### 1. Real-Time Tracking

**GET /api/v1/transports/{transportId}/status**

Response:
```json
{
  "transportId": "TR-2025-001",
  "status": "IN_TRANSIT",
  "temperature": -195.2,
  "ln2Level": 87,
  "location": {"lat": 39.8561, "lon": -104.6737},
  "eta": "2025-01-15T16:30:00Z"
}
```

### 2. Alert Notifications

**POST /api/v1/alerts**

Webhook payload for temperature alerts:
```json
{
  "alertId": "ALT-001",
  "transportId": "TR-2025-001",
  "type": "TEMPERATURE_WARNING",
  "severity": "WARNING",
  "timestamp": "2025-01-15T14:30:00Z",
  "temperature": -189.2,
  "threshold": -190.0
}
```

### 3. Route Optimization

**POST /api/v1/routes/optimize**

Request:
```json
{
  "origin": {"lat": 33.4942, "lon": -111.9261},
  "destination": {"lat": 42.5896, "lon": -82.9199},
  "transportMode": "AIR",
  "subjectType": "WHOLE_BODY",
  "constraints": {
    "maxDuration": 24,
    "backupFacilitySpacing": 200
  }
}
```

Response:
```json
{
  "routeId": "RT-2025-001",
  "distance": 2650,
  "estimatedDuration": 6.5,
  "waypoints": [...],
  "backupFacilities": [...],
  "routeScore": 0.95
}
```

---

## Authentication

All API requests require JWT authentication:
```
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

---

© 2025 WIA
