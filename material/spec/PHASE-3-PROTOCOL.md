# WIA Material Protocol Specification v1.0.0
# Phase 3: Communication Protocol Standard

---

**Version**: 1.0.0
**Date**: 2025-12-14
**Status**: Draft
**Authors**: Claude Code (Opus 4.5)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Protocol Overview](#2-protocol-overview)
3. [Transport Layer](#3-transport-layer)
4. [Message Format](#4-message-format)
5. [Message Types](#5-message-types)
6. [REST API Specification](#6-rest-api-specification)
7. [WebSocket Specification](#7-websocket-specification)
8. [Authentication](#8-authentication)
9. [Error Handling](#9-error-handling)
10. [OPTIMADE Compatibility](#10-optimade-compatibility)
11. [Examples](#11-examples)

---

## 1. Introduction

### 1.1 Purpose

The WIA Material Protocol defines a standard communication protocol for exchanging material science data between clients and servers. This specification enables:

- Querying material databases
- Real-time data streaming from instruments
- Interoperability with existing material science APIs
- Secure and authenticated data exchange

### 1.2 Scope

This specification covers:

- Transport protocols (HTTP/HTTPS, WebSocket)
- Message format and serialization
- REST API endpoints
- WebSocket events
- Authentication mechanisms
- Error handling

### 1.3 Conformance

Implementations MUST support:
- REST API over HTTPS
- JSON message format

Implementations SHOULD support:
- WebSocket for real-time streaming
- MessagePack binary format

### 1.4 Terminology

| Term | Definition |
|------|------------|
| **Client** | Application that initiates requests |
| **Server** | Application that responds to requests |
| **Material** | Data object conforming to Phase 1 schema |
| **Stream** | Continuous flow of material data |
| **Session** | Authenticated connection context |

---

## 2. Protocol Overview

### 2.1 Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      WIA Material Protocol                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐                    ┌──────────────────────┐   │
│  │    Client    │                    │       Server         │   │
│  │              │                    │                      │   │
│  │  ┌────────┐  │   REST/WebSocket   │  ┌────────────────┐  │   │
│  │  │Protocol│  │ ◄────────────────► │  │   Protocol     │  │   │
│  │  │ Layer  │  │                    │  │    Layer       │  │   │
│  │  └────────┘  │                    │  └────────────────┘  │   │
│  │       │      │                    │         │            │   │
│  │  ┌────────┐  │                    │  ┌────────────────┐  │   │
│  │  │Transport│ │                    │  │   Transport    │  │   │
│  │  │ Layer  │  │                    │  │    Layer       │  │   │
│  │  └────────┘  │                    │  └────────────────┘  │   │
│  │       │      │                    │         │            │   │
│  └───────┼──────┘                    └─────────┼────────────┘   │
│          │                                     │                 │
│          └──────────── HTTPS/WSS ──────────────┘                │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Protocol Layers

| Layer | Responsibility |
|-------|---------------|
| **Protocol Layer** | Message encoding/decoding, type handling |
| **Transport Layer** | HTTP/WebSocket connection management |
| **Security Layer** | TLS, authentication, authorization |

### 2.3 Version Negotiation

Protocol version is specified in:
- REST: `Accept` header or URL path
- WebSocket: Initial `handshake` message

```http
Accept: application/vnd.wia-material.v1+json
```

---

## 3. Transport Layer

### 3.1 HTTPS (Required)

All REST API communications MUST use HTTPS (TLS 1.2+).

**Base URL Format**:
```
https://{host}/api/v1/materials
```

**Headers**:
```http
Content-Type: application/json
Accept: application/json
X-WIA-API-Key: {api-key}
```

### 3.2 WebSocket Secure (WSS)

Real-time streaming uses WebSocket over TLS.

**Connection URL**:
```
wss://{host}/api/v1/stream
```

**Subprotocol**:
```
Sec-WebSocket-Protocol: wia-material-v1
```

### 3.3 Connection Management

#### 3.3.1 Timeouts

| Operation | Timeout |
|-----------|---------|
| Connection | 30 seconds |
| Request | 60 seconds |
| Idle (WebSocket) | 120 seconds |

#### 3.3.2 Keep-Alive

WebSocket connections use ping/pong frames:
- Client sends `ping` every 30 seconds
- Server responds with `pong` within 5 seconds
- Connection closed if no pong received

#### 3.3.3 Reconnection

Clients SHOULD implement exponential backoff:
```
delay = min(base * 2^attempt, max_delay)
base = 1 second
max_delay = 60 seconds
max_attempts = 10
```

---

## 4. Message Format

### 4.1 Envelope Structure

All messages use a common envelope:

```json
{
  "protocol": "wia-material",
  "version": "1.0.0",
  "message_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-14T12:00:00.000Z",
  "type": "query",
  "correlation_id": "optional-request-id",
  "payload": {}
}
```

### 4.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `protocol` | string | Yes | Always "wia-material" |
| `version` | string | Yes | Protocol version (semver) |
| `message_id` | string | Yes | UUID v4 unique identifier |
| `timestamp` | string | Yes | ISO 8601 UTC timestamp |
| `type` | string | Yes | Message type enum |
| `correlation_id` | string | No | Links response to request |
| `payload` | object | Yes | Type-specific data |

### 4.3 Serialization

#### 4.3.1 JSON (Default)

```json
{
  "protocol": "wia-material",
  "version": "1.0.0",
  "message_id": "...",
  "timestamp": "2025-12-14T12:00:00Z",
  "type": "data",
  "payload": {
    "material_type": "superconductor",
    "material_id": "wia-mat-001",
    "...": "..."
  }
}
```

Content-Type: `application/json`

#### 4.3.2 MessagePack (Binary)

For high-performance scenarios, MessagePack encoding is supported.

Content-Type: `application/msgpack`

---

## 5. Message Types

### 5.1 Request Types (Client → Server)

| Type | Description |
|------|-------------|
| `query` | Query materials by filter |
| `get` | Get single material by ID |
| `create` | Create new material |
| `update` | Update existing material |
| `delete` | Delete material |
| `subscribe` | Subscribe to data stream |
| `unsubscribe` | Unsubscribe from stream |
| `command` | Execute command |
| `ping` | Connection health check |

### 5.2 Response Types (Server → Client)

| Type | Description |
|------|-------------|
| `query_response` | Query results |
| `get_response` | Single material data |
| `create_response` | Creation confirmation |
| `update_response` | Update confirmation |
| `delete_response` | Deletion confirmation |
| `subscribe_ack` | Subscription confirmed |
| `unsubscribe_ack` | Unsubscription confirmed |
| `data` | Streamed material data |
| `command_ack` | Command acknowledgment |
| `pong` | Health check response |
| `error` | Error response |

### 5.3 Message Schemas

#### 5.3.1 Query Request

```json
{
  "type": "query",
  "payload": {
    "material_type": "superconductor",
    "filter": {
      "field": "properties.superconductor.critical_temperature.value",
      "operator": "gt",
      "value": 77
    },
    "sort": {
      "field": "timestamp",
      "order": "desc"
    },
    "pagination": {
      "offset": 0,
      "limit": 100
    },
    "fields": ["material_id", "identity", "properties"]
  }
}
```

#### 5.3.2 Query Response

```json
{
  "type": "query_response",
  "correlation_id": "original-request-id",
  "payload": {
    "data": [
      { "material_id": "...", "identity": {...}, "properties": {...} }
    ],
    "meta": {
      "total_count": 150,
      "returned_count": 100,
      "offset": 0,
      "has_more": true
    }
  }
}
```

#### 5.3.3 Subscribe Request

```json
{
  "type": "subscribe",
  "payload": {
    "channel": "measurements",
    "material_type": "superconductor",
    "filter": {
      "field": "identity.name",
      "operator": "contains",
      "value": "YBCO"
    }
  }
}
```

#### 5.3.4 Data Stream

```json
{
  "type": "data",
  "payload": {
    "channel": "measurements",
    "sequence": 12345,
    "material": {
      "material_type": "superconductor",
      "material_id": "wia-mat-001",
      "timestamp": "2025-12-14T12:00:00.123Z",
      "...": "..."
    }
  }
}
```

#### 5.3.5 Error Response

```json
{
  "type": "error",
  "correlation_id": "original-request-id",
  "payload": {
    "code": "MATERIAL_NOT_FOUND",
    "message": "Material with ID 'wia-mat-999' not found",
    "details": {
      "material_id": "wia-mat-999"
    }
  }
}
```

---

## 6. REST API Specification

### 6.1 Base URL

```
https://{host}/api/v1
```

### 6.2 Endpoints

#### 6.2.1 List Materials

```http
GET /materials
GET /materials?type={material_type}
GET /materials?filter={json_filter}
```

**Query Parameters**:
| Parameter | Type | Description |
|-----------|------|-------------|
| `type` | string | Filter by material type |
| `filter` | JSON | Advanced filter object |
| `sort` | string | Sort field (prefix `-` for desc) |
| `offset` | int | Pagination offset |
| `limit` | int | Max results (default: 100, max: 1000) |
| `fields` | string | Comma-separated field list |

**Response**:
```json
{
  "data": [...],
  "meta": {
    "total_count": 1000,
    "returned_count": 100,
    "offset": 0,
    "limit": 100
  },
  "links": {
    "self": "/api/v1/materials?offset=0&limit=100",
    "next": "/api/v1/materials?offset=100&limit=100"
  }
}
```

#### 6.2.2 Get Material

```http
GET /materials/{material_id}
```

**Response**:
```json
{
  "data": {
    "material_type": "superconductor",
    "material_id": "wia-mat-001",
    "...": "..."
  },
  "links": {
    "self": "/api/v1/materials/wia-mat-001"
  }
}
```

#### 6.2.3 Create Material

```http
POST /materials
Content-Type: application/json

{
  "material_type": "superconductor",
  "identity": {...},
  "structure": {...},
  "properties": {...}
}
```

**Response** (201 Created):
```json
{
  "data": {
    "material_id": "wia-mat-002",
    "...": "..."
  },
  "links": {
    "self": "/api/v1/materials/wia-mat-002"
  }
}
```

#### 6.2.4 Update Material

```http
PUT /materials/{material_id}
PATCH /materials/{material_id}
```

#### 6.2.5 Delete Material

```http
DELETE /materials/{material_id}
```

**Response** (204 No Content)

### 6.3 Filter Syntax

Filter objects support these operators:

| Operator | Description | Example |
|----------|-------------|---------|
| `eq` | Equals | `{"field": "type", "operator": "eq", "value": "superconductor"}` |
| `ne` | Not equals | `{"field": "status", "operator": "ne", "value": "draft"}` |
| `gt` | Greater than | `{"field": "tc", "operator": "gt", "value": 77}` |
| `gte` | Greater or equal | `{"field": "tc", "operator": "gte", "value": 77}` |
| `lt` | Less than | `{"field": "tc", "operator": "lt", "value": 273}` |
| `lte` | Less or equal | `{"field": "tc", "operator": "lte", "value": 273}` |
| `contains` | Contains string | `{"field": "name", "operator": "contains", "value": "YBCO"}` |
| `in` | In list | `{"field": "type", "operator": "in", "value": ["sc", "ti"]}` |
| `exists` | Field exists | `{"field": "properties.gap", "operator": "exists", "value": true}` |

Compound filters:

```json
{
  "and": [
    {"field": "type", "operator": "eq", "value": "superconductor"},
    {"field": "tc", "operator": "gt", "value": 77}
  ]
}
```

---

## 7. WebSocket Specification

### 7.1 Connection

```javascript
const ws = new WebSocket('wss://host/api/v1/stream', 'wia-material-v1');
```

### 7.2 Handshake

After connection, client sends handshake:

```json
{
  "type": "handshake",
  "payload": {
    "protocol_version": "1.0.0",
    "client_id": "client-uuid",
    "auth_token": "jwt-token-or-api-key"
  }
}
```

Server responds:

```json
{
  "type": "handshake_ack",
  "payload": {
    "session_id": "session-uuid",
    "server_version": "1.0.0",
    "capabilities": ["query", "subscribe", "command"]
  }
}
```

### 7.3 Subscription Channels

| Channel | Description |
|---------|-------------|
| `materials` | All material updates |
| `measurements` | Real-time measurement data |
| `instruments` | Instrument status updates |
| `alerts` | System alerts |

### 7.4 Flow Control

#### 7.4.1 Backpressure

Client can request flow control:

```json
{
  "type": "flow_control",
  "payload": {
    "channel": "measurements",
    "action": "pause"  // or "resume"
  }
}
```

#### 7.4.2 Rate Limiting

Server enforces rate limits:
- Messages: 100/second per channel
- Subscriptions: 10 concurrent per client

---

## 8. Authentication

### 8.1 API Key Authentication

```http
X-WIA-API-Key: wia_live_xxxxxxxxxxxx
```

Key format: `wia_{env}_{32-char-random}`
- `env`: `live`, `test`, `dev`

### 8.2 JWT Authentication

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIs...
```

JWT Claims:
```json
{
  "sub": "user-id",
  "aud": "wia-material-api",
  "iat": 1702560000,
  "exp": 1702563600,
  "scope": ["read", "write", "stream"]
}
```

### 8.3 Scopes

| Scope | Permissions |
|-------|-------------|
| `read` | GET requests only |
| `write` | POST, PUT, PATCH, DELETE |
| `stream` | WebSocket subscriptions |
| `admin` | Management operations |

---

## 9. Error Handling

### 9.1 HTTP Status Codes

| Code | Meaning |
|------|---------|
| 200 | Success |
| 201 | Created |
| 204 | No Content |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 409 | Conflict |
| 422 | Validation Error |
| 429 | Rate Limited |
| 500 | Server Error |
| 503 | Service Unavailable |

### 9.2 Error Codes

| Code | HTTP | Description |
|------|------|-------------|
| `INVALID_REQUEST` | 400 | Malformed request |
| `VALIDATION_ERROR` | 422 | Schema validation failed |
| `AUTH_REQUIRED` | 401 | Authentication needed |
| `AUTH_INVALID` | 401 | Invalid credentials |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `MATERIAL_NOT_FOUND` | 404 | Material ID not found |
| `DUPLICATE_ID` | 409 | Material ID already exists |
| `RATE_LIMITED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |
| `SERVICE_UNAVAILABLE` | 503 | Temporary unavailable |

### 9.3 Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Request validation failed",
    "details": [
      {
        "field": "properties.superconductor.critical_temperature",
        "message": "Required field missing"
      }
    ],
    "request_id": "req-uuid"
  }
}
```

---

## 10. OPTIMADE Compatibility

### 10.1 Mapping

WIA Material structures can be converted to/from OPTIMADE format:

| WIA Material | OPTIMADE |
|--------------|----------|
| `material_id` | `id` |
| `identity.name` | `attributes.chemical_formula_descriptive` |
| `structure.crystal_system` | `attributes.space_group` |
| `structure.lattice_parameters` | `attributes.lattice_vectors` |
| `structure.composition` | `attributes.elements`, `attributes.nelements` |

### 10.2 OPTIMADE Endpoint

```http
GET /api/v1/optimade/structures
GET /api/v1/optimade/structures/{id}
```

These endpoints return OPTIMADE-compliant responses.

### 10.3 Filter Translation

WIA filters can include OPTIMADE syntax:

```http
GET /api/v1/materials?optimade_filter=elements HAS "Fe" AND nelements=2
```

---

## 11. Examples

### 11.1 Query Superconductors

```bash
curl -X GET \
  'https://api.example.com/api/v1/materials?type=superconductor&filter={"field":"properties.superconductor.critical_temperature.value","operator":"gt","value":77}' \
  -H 'X-WIA-API-Key: wia_live_xxx'
```

### 11.2 Create Material

```bash
curl -X POST \
  'https://api.example.com/api/v1/materials' \
  -H 'Content-Type: application/json' \
  -H 'X-WIA-API-Key: wia_live_xxx' \
  -d '{
    "material_type": "superconductor",
    "identity": {
      "name": "YBCO",
      "formula": "YBa2Cu3O7"
    },
    "properties": {
      "superconductor": {
        "critical_temperature": {
          "value": 92,
          "unit": "K"
        }
      }
    }
  }'
```

### 11.3 WebSocket Subscription

```javascript
const ws = new WebSocket('wss://api.example.com/api/v1/stream');

ws.onopen = () => {
  // Handshake
  ws.send(JSON.stringify({
    type: 'handshake',
    payload: { auth_token: 'wia_live_xxx' }
  }));
};

ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);

  if (msg.type === 'handshake_ack') {
    // Subscribe to measurements
    ws.send(JSON.stringify({
      type: 'subscribe',
      payload: {
        channel: 'measurements',
        material_type: 'superconductor'
      }
    }));
  }

  if (msg.type === 'data') {
    console.log('New measurement:', msg.payload.material);
  }
};
```

---

## Appendix A: JSON Schema

### A.1 Message Envelope Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/protocol/message.json",
  "type": "object",
  "required": ["protocol", "version", "message_id", "timestamp", "type", "payload"],
  "properties": {
    "protocol": {
      "type": "string",
      "const": "wia-material"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "message_id": {
      "type": "string",
      "format": "uuid"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "type": {
      "type": "string",
      "enum": [
        "query", "query_response",
        "get", "get_response",
        "create", "create_response",
        "update", "update_response",
        "delete", "delete_response",
        "subscribe", "subscribe_ack",
        "unsubscribe", "unsubscribe_ack",
        "data", "command", "command_ack",
        "ping", "pong", "error",
        "handshake", "handshake_ack",
        "flow_control"
      ]
    },
    "correlation_id": {
      "type": "string"
    },
    "payload": {
      "type": "object"
    }
  }
}
```

---

## Appendix B: Protocol Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-14 | Initial release |

---

<div align="center">

**WIA Material Protocol v1.0.0**

---

弘益人間 🤟

</div>
