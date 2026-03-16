# Phase 2: Pet Tracking API Interface Specification

## WIA-PET-TRACKING API Standard

**Version**: 1.0.0  
**Date**: 2025-12-25  
**Status**: Active  
**Standard ID**: WIA-PET-008-PHASE2  
**Primary Color**: #F59E0B (Amber)

---

## 1. API Overview

### 1.1 REST API Principles

The WIA-PET-TRACKING API follows RESTful design principles:

- **Resource-oriented URLs**
- **HTTP methods**: GET, POST, PUT, PATCH, DELETE
- **JSON request/response bodies**
- **Standard HTTP status codes**
- **Stateless authentication**
- **Versioned endpoints**

### 1.2 Base URL

```
https://api.example.com/v1
```

### 1.3 Authentication

**Method**: Bearer Token (JWT)

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

---

## 2. Location Tracking Endpoints

### 2.1 POST /tracking/update

Update tracker location.

**Request**:
```http
POST /api/v1/tracking/update
Content-Type: application/json
Authorization: Bearer {token}

{
  "trackerId": "TRK-ABC123",
  "timestamp": "2025-12-25T10:30:45.123Z",
  "location": {
    "latitude": 37.774929,
    "longitude": -122.419418,
    "accuracy": 8.5
  },
  "battery": {
    "level": 78,
    "estimatedHours": 36
  }
}
```

**Response (200 OK)**:
```json
{
  "status": "accepted",
  "sequenceNumber": 12847,
  "nextUpdate": "2025-12-25T10:31:15.000Z"
}
```

### 2.2 GET /tracking/location

Get current location.

**Request**:
```http
GET /api/v1/tracking/location?trackerId=TRK-ABC123
Authorization: Bearer {token}
```

**Response (200 OK)**:
```json
{
  "trackerId": "TRK-ABC123",
  "timestamp": "2025-12-25T10:30:45.123Z",
  "location": {
    "latitude": 37.774929,
    "longitude": -122.419418,
    "accuracy": 8.5
  },
  "status": "active"
}
```

### 2.3 GET /tracking/history

Retrieve location history.

**Query Parameters**:
- `trackerId` (required): Tracker ID
- `start` (optional): ISO 8601 start time
- `end` (optional): ISO 8601 end time
- `limit` (optional): Max results (default: 100)
- `offset` (optional): Pagination offset

**Response**:
```json
{
  "trackerId": "TRK-ABC123",
  "period": {
    "start": "2025-12-24T00:00:00Z",
    "end": "2025-12-25T00:00:00Z"
  },
  "points": [...],
  "totalCount": 2880,
  "hasMore": true
}
```

---

## 3. Geofence Management

### 3.1 POST /geofences

Create new geofence.

**Request**:
```json
{
  "name": "Home",
  "type": "circular",
  "center": {
    "latitude": 37.774929,
    "longitude": -122.419418
  },
  "radius": 100,
  "triggers": {
    "entry": true,
    "exit": true
  }
}
```

**Response (201 Created)**:
```json
{
  "geofenceId": "GEO-HOME-001",
  "status": "active",
  "created": "2025-12-25T10:00:00Z"
}
```

### 3.2 GET /geofences

List all geofences.

### 3.3 PUT /geofences/{id}

Update geofence.

### 3.4 DELETE /geofences/{id}

Delete geofence.

---

## 4. Lost Pet Alerts

### 4.1 POST /lost-pet/activate

Activate lost pet mode.

**Request**:
```json
{
  "trackerId": "TRK-ABC123",
  "petId": "PET-789XYZ",
  "lastKnownLocation": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "timestamp": "2025-12-25T10:00:00Z"
  },
  "contactInfo": {
    "phone": "+1-555-0123",
    "email": "owner@example.com"
  },
  "recoveryNetwork": {
    "local": true,
    "regional": true
  }
}
```

**Response (200 OK)**:
```json
{
  "alertId": "LOST-20251225-ABC123",
  "status": "activated",
  "estimatedReach": 706,
  "config": {
    "updateInterval": 10
  }
}
```

### 4.2 POST /lost-pet/sighting-report

Report a sighting.

### 4.3 POST /lost-pet/found

Mark pet as found.

---

## 5. Device Management

### 5.1 GET /devices

List tracked devices.

### 5.2 GET /devices/{id}

Get device details.

### 5.3 PATCH /devices/{id}/config

Update device configuration.

**Request**:
```json
{
  "updateInterval": 30,
  "batteryMode": "normal",
  "gpsMode": "multi-gnss"
}
```

---

## 6. WebSocket Real-time Updates

### 6.1 Connection

```javascript
const ws = new WebSocket('wss://api.example.com/ws/tracking');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    trackerIds: ['TRK-ABC123'],
    token: 'Bearer ...'
  }));
};

ws.onmessage = (event) => {
  const update = JSON.parse(event.data);
  console.log('Location update:', update);
};
```

### 6.2 Message Types

**Location Update**:
```json
{
  "type": "location",
  "trackerId": "TRK-ABC123",
  "timestamp": "2025-12-25T10:30:45Z",
  "location": {...}
}
```

**Geofence Alert**:
```json
{
  "type": "geofence",
  "trackerId": "TRK-ABC123",
  "geofenceId": "GEO-HOME",
  "event": "exit",
  "timestamp": "2025-12-25T10:32:10Z"
}
```

---

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_DATA",
    "message": "Location coordinates are invalid",
    "details": {
      "field": "latitude",
      "value": 95.0,
      "expected": "-90.0 to 90.0"
    },
    "timestamp": "2025-12-25T10:30:00Z"
  }
}
```

### 7.2 HTTP Status Codes

| Code | Meaning |
|------|---------|
| 200 | OK |
| 201 | Created |
| 204 | No Content |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 429 | Too Many Requests |
| 500 | Internal Server Error |
| 503 | Service Unavailable |

---

## 8. Rate Limiting

### 8.1 Limits

- Location updates: 100 requests/minute per tracker
- API queries: 1000 requests/hour per user
- Geofence management: 60 requests/minute

### 8.2 Headers

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1735122645
```

---

## 9. Pagination

```http
GET /api/v1/tracking/history?limit=50&offset=100
```

**Response**:
```json
{
  "data": [...],
  "pagination": {
    "limit": 50,
    "offset": 100,
    "totalCount": 2880,
    "hasMore": true,
    "nextOffset": 150
  }
}
```

---

**弘益人間 · Benefit All Humanity**  
© 2025 WIA - World Certification Industry Association | MIT License
