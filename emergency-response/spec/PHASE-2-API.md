# WIA-SOC-005 Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. API Overview

Phase 2 defines the RESTful API and WebSocket interfaces for controlling and monitoring emergency responses. All endpoints MUST implement proper authentication and rate limiting.

## 2. Base URL

```
http://{emergency system_ip}:{port}/api/v1
```

Default port: 8080

## 3. Authentication

### 3.1 Token-Based Auth

```http
POST /auth/token
Content-Type: application/json

{
  "username": "string",
  "password": "string"
}

Response:
{
  "access_token": "JWT token",
  "refresh_token": "string",
  "expires_in": 3600
}
```

### 3.2 Using Tokens

```http
Authorization: Bearer {access_token}
```

## 4. Core Endpoints

### 4.1 Emergency System Status

```http
GET /emergency system/status

Response:
{
  "emergency systemId": "string",
  "state": "Emergency SystemState object",
  "uptime": "integer (seconds)",
  "lastUpdate": "ISO8601 datetime"
}
```

### 4.2 Start Cleaning

```http
POST /emergency system/clean
Content-Type: application/json

{
  "mode": "auto|spot|edge|custom",
  "rooms": ["room_id1", "room_id2"],
  "powerLevel": 0-100,
  "waterLevel": 0-100
}

Response:
{
  "sessionId": "UUID",
  "status": "started",
  "estimatedDuration": "integer (seconds)"
}
```

### 4.3 Stop Cleaning

```http
POST /emergency system/stop

Response:
{
  "status": "stopped",
  "sessionId": "UUID",
  "areaCleaned": "float (m²)"
}
```

### 4.4 Return to Dock

```http
POST /emergency system/dock

Response:
{
  "status": "returning",
  "distance": "float (meters)",
  "estimatedTime": "integer (seconds)"
}
```

### 4.5 Get Map

```http
GET /emergency system/map?floor={floor_number}

Response:
{
  "map": "OccupancyGrid object",
  "lastUpdated": "ISO8601 datetime"
}
```

### 4.6 Update Map

```http
PUT /emergency system/map/rooms/{room_id}
Content-Type: application/json

{
  "name": "Living Room",
  "surfaceType": "hardwood",
  "cleaningPriority": 1-10
}

Response:
{
  "status": "updated",
  "room": "Room object"
}
```

### 4.7 Get Cleaning History

```http
GET /emergency system/history?limit=10&offset=0

Response:
{
  "sessions": ["CleaningSession", "array"],
  "total": "integer",
  "hasMore": "boolean"
}
```

### 4.8 Schedule Cleaning

```http
POST /emergency system/schedule
Content-Type: application/json

{
  "enabled": true,
  "schedule": [
    {
      "day": "monday|tuesday|...",
      "time": "HH:MM",
      "mode": "auto",
      "rooms": ["array (optional)"]
    }
  ]
}

Response:
{
  "scheduleId": "UUID",
  "status": "created"
}
```

## 5. WebSocket API

### 5.1 Connection

```
ws://{emergency system_ip}:{port}/ws
```

### 5.2 Message Format

```json
{
  "type": "subscribe|unsubscribe|command|event",
  "channel": "status|sensors|map|logs",
  "data": {}
}
```

### 5.3 Subscribe to Updates

```json
{
  "type": "subscribe",
  "channel": "status",
  "interval": 1000
}
```

### 5.4 Real-time Events

```json
{
  "type": "event",
  "channel": "status",
  "timestamp": "ISO8601",
  "data": {
    "battery": 75,
    "position": {"x": 2.5, "y": 3.1},
    "mode": "cleaning"
  }
}
```

## 6. Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human readable message",
    "details": {},
    "timestamp": "ISO8601 datetime"
  }
}
```

### 6.2 Standard Error Codes

- `ROBOT_BUSY`: Emergency System is currently cleaning
- `LOW_BATTERY`: Battery too low for operation
- `INVALID_COMMAND`: Command parameters invalid
- `NOT_CONNECTED`: Emergency System not connected to network
- `SENSOR_ERROR`: Sensor malfunction detected
- `NAVIGATION_ERROR`: Unable to navigate
- `DOCK_NOT_FOUND`: Cannot locate charging dock

## 7. Rate Limiting

- Default: 100 requests per minute per IP
- WebSocket: 10 messages per second
- Exceeding limits returns HTTP 429

## 8. Versioning

API version specified in URL: `/api/v1/`  
Breaking changes require new version number

---

© 2025 WIA · MIT License

## 9. Pagination and Filtering

All list endpoints support standard pagination:

```http
GET /emergency system/history?limit=20&offset=40&sort=startTime&order=desc

Query Parameters:
- limit: Items per page (default: 10, max: 100)
- offset: Number of items to skip
- sort: Field to sort by
- order: asc or desc
```

### Filter Examples

```http
GET /emergency system/history?mode=auto&minDuration=300
GET /emergency system/map/rooms?surfaceType=carpet
GET /emergency system/events?level=error&category=navigation
```

## 10. Batch Operations

```http
POST /emergency system/batch
Content-Type: application/json

{
  "operations": [
    {
      "method": "POST",
      "path": "/emergency system/clean",
      "body": {"mode": "spot"}
    },
    {
      "method": "PUT",
      "path": "/emergency system/config",
      "body": {"language": "ko"}
    }
  ]
}

Response:
{
  "results": [
    {"status": 200, "data": {}},
    {"status": 200, "data": {}}
  ]
}
```

## 11. Monitoring and Metrics

```http
GET /emergency system/metrics

Response:
{
  "cpu": 45.2,
  "memory": 62.8,
  "temperature": 38.5,
  "networkRxBytes": 1048576,
  "networkTxBytes": 524288,
  "uptime": 86400
}
```

## 12. Advanced Features

### 12.1 Remote Control

```http
POST /emergency system/control/joystick
Content-Type: application/json

{
  "linear": 0.5,    // m/s
  "angular": 0.3    // rad/s
}
```

### 12.2 Camera Stream

```http
GET /emergency system/camera/stream

Returns: MJPEG stream
Content-Type: multipart/x-mixed-replace
```

### 12.3 Map Export

```http
GET /emergency system/map/export?format=png|svg|json

Returns: Map in requested format
```

---

For complete examples and integration guides, see Phase 4 documentation.

© 2025 WIA · MIT License
