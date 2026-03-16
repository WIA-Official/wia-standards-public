# WIA-SEMI-019 - Phase 2: Communication API

> **Version:** 1.0  
> **Status:** Active  
> **Last Updated:** 2025-12-26

## 1. Overview

Phase 2 defines modern HTTP/REST and WebSocket APIs for equipment communication, complementing traditional SECS/GEM protocols. These APIs enable easier integration with web-based applications, cloud platforms, and modern software architectures.

## 2. REST API Specification

### 2.1 Base URL Structure

```
https://{equipment-host}/api/v1/{resource}
```

- All APIs use HTTPS (TLS 1.3)
- Version included in URL path (`v1`)
- RESTful resource-based design
- JSON request/response format

### 2.2 Core Endpoints

#### Equipment Information
```
GET /api/v1/equipment/info
Response: Equipment specification (Phase 1 format)
```

#### Equipment Status
```
GET /api/v1/equipment/status
Response: Current equipment state and status
```

#### Parameters
```
GET /api/v1/parameters
GET /api/v1/parameters/{id}
POST /api/v1/parameters/{id}/subscribe
DELETE /api/v1/parameters/{id}/subscribe
```

#### Commands
```
POST /api/v1/command
GET /api/v1/command/{id}/status
```

#### Alarms
```
GET /api/v1/alarms
GET /api/v1/alarms/{id}
POST /api/v1/alarms/{id}/acknowledge
```

#### Events
```
GET /api/v1/events
GET /api/v1/events/{id}
```

#### Recipes
```
GET /api/v1/recipes
GET /api/v1/recipes/{id}
POST /api/v1/recipes
PUT /api/v1/recipes/{id}
DELETE /api/v1/recipes/{id}
```

## 3. Authentication

### 3.1 API Key Authentication

```http
GET /api/v1/parameters
Authorization: Bearer wia_apikey_...
```

### 3.2 OAuth 2.0

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=...&
client_secret=...&
scope=equipment.read equipment.control
```

### 3.3 Role-Based Access Control

| Role | Permissions |
|------|-------------|
| Viewer | Read status, parameters, alarms |
| Operator | Viewer + Send basic commands |
| Engineer | Operator + Recipe management |
| Administrator | All permissions |

## 4. WebSocket Streaming

### 4.1 Connection

```javascript
ws://equipment-host/api/v1/stream
wss://equipment-host/api/v1/stream  // Secure
```

### 4.2 Protocol

```json
// Subscribe
{
  "action": "subscribe",
  "parameters": ["substrate_temp_celsius", "chamber_pressure_pascal"],
  "frequency_hz": 100,
  "format": "json"
}

// Data Stream
{
  "timestamp": "2025-12-26T15:30:45.123Z",
  "sequence": 12345,
  "data": {
    "substrate_temp_celsius": 425.3,
    "chamber_pressure_pascal": 133.2
  }
}

// Unsubscribe
{
  "action": "unsubscribe",
  "parameters": ["chamber_pressure_pascal"]
}
```

## 5. Error Handling

### 5.1 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful request |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid format |
| 401 | Unauthorized | Missing credentials |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | State conflict |
| 500 | Internal Error | Equipment error |
| 503 | Service Unavailable | Maintenance mode |

### 5.2 Error Response Format

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Parameter 'invalid_param' does not exist",
    "timestamp": "2025-12-26T15:30:45Z",
    "request_id": "req_123456"
  }
}
```

## 6. Performance Requirements

| Metric | Requirement |
|--------|-------------|
| API Response Time (95th percentile) | < 200ms |
| WebSocket Latency | < 10ms |
| Throughput (req/s) | > 100 |
| WebSocket Message Rate | 1000 Hz |
| Concurrent Connections | > 50 |

## 7. Silver Certification Requirements

1. Implement all core REST API endpoints
2. Support WebSocket streaming at ≥100Hz
3. Implement API key and OAuth 2.0 authentication
4. Role-based access control (4 roles minimum)
5. Meet performance requirements
6. Provide OpenAPI 3.0 specification
7. Support TLS 1.3

---

**弘益人間 · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 MIT License*
