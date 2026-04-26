# WIA-AI-016 Phase 2: API Specification

> 弘益人間 (홍익인간) · Benefit All Humanity

## Overview

Phase 2 defines RESTful APIs, WebSocket protocols, and service interfaces for WIA-AI-016 compliant multi-agent systems.

## Base URL

```
https://api.example.com/v1/agents
```

## Authentication

All API requests require authentication via Bearer token:

```http
Authorization: Bearer <token>
```

## Agent Management API

### Register Agent

**POST** `/agents`

```json
{
  "name": "agent-001",
  "capabilities": ["temperature-sensing", "data-logging"],
  "publicKey": "-----BEGIN PUBLIC KEY-----\n...",
  "metadata": {
    "type": "sensor",
    "location": "room-101"
  }
}
```

**Response 201 Created**

```json
{
  "agentId": "agent-001",
  "address": "wia://api.example.com/agents/agent-001",
  "registeredAt": "2025-12-25T10:00:00Z",
  "status": "active"
}
```

### Get Agent

**GET** `/agents/{agentId}`

**Response 200 OK**

```json
{
  "agentId": "agent-001",
  "name": "agent-001",
  "address": "wia://api.example.com/agents/agent-001",
  "capabilities": ["temperature-sensing"],
  "status": "active",
  "reputation": 0.85,
  "registeredAt": "2025-12-25T10:00:00Z"
}
```

### Update Agent

**PATCH** `/agents/{agentId}`

```json
{
  "status": "idle | busy | offline",
  "capabilities": ["new-capability"]
}
```

### Delete Agent

**DELETE** `/agents/{agentId}`

**Response 204 No Content**

## Messaging API

### Send Message

**POST** `/agents/{agentId}/messages`

```json
{
  "performative": "request",
  "receiver": ["agent-002"],
  "content": {
    "action": "measure-temperature",
    "location": "room-101"
  },
  "protocol": "fipa-request",
  "conversationId": "conv-001"
}
```

**Response 202 Accepted**

```json
{
  "messageId": "msg-001",
  "status": "queued",
  "timestamp": "2025-12-25T10:00:00Z"
}
```

### Get Messages

**GET** `/agents/{agentId}/messages?status=unread&limit=50`

**Response 200 OK**

```json
{
  "messages": [
    {
      "messageId": "msg-001",
      "performative": "inform",
      "sender": "agent-002",
      "content": {"temperature": 25},
      "receivedAt": "2025-12-25T10:01:00Z",
      "status": "unread"
    }
  ],
  "total": 1,
  "page": 1
}
```

### Mark Message Read

**PATCH** `/agents/{agentId}/messages/{messageId}`

```json
{
  "status": "read"
}
```

## Task API

### Create Task

**POST** `/tasks`

```json
{
  "type": "measurement",
  "priority": 5,
  "requiredSkills": ["temperature-sensing"],
  "deadline": "2025-12-25T12:00:00Z",
  "parameters": {
    "location": "room-101"
  }
}
```

**Response 201 Created**

```json
{
  "taskId": "task-001",
  "status": "pending",
  "createdAt": "2025-12-25T10:00:00Z"
}
```

### Get Task

**GET** `/tasks/{taskId}`

**Response 200 OK**

```json
{
  "taskId": "task-001",
  "type": "measurement",
  "status": "in-progress",
  "assignedTo": "agent-001",
  "createdAt": "2025-12-25T10:00:00Z",
  "deadline": "2025-12-25T12:00:00Z"
}
```

### Assign Task

**POST** `/tasks/{taskId}/assign`

```json
{
  "agentId": "agent-001"
}
```

### Complete Task

**POST** `/tasks/{taskId}/complete`

```json
{
  "result": {
    "temperature": 25,
    "timestamp": "2025-12-25T10:30:00Z"
  },
  "quality": 0.95
}
```

## Discovery API

### Search Agents

**GET** `/agents/search?capability=temperature-sensing&status=idle`

**Response 200 OK**

```json
{
  "agents": [
    {
      "agentId": "agent-001",
      "capabilities": ["temperature-sensing"],
      "status": "idle",
      "reputation": 0.85
    }
  ],
  "total": 1
}
```

### Get Capabilities

**GET** `/capabilities`

**Response 200 OK**

```json
{
  "capabilities": [
    {
      "name": "temperature-sensing",
      "type": "sensing",
      "agentCount": 15
    }
  ]
}
```

## WebSocket Protocol

### Connection

```
wss://api.example.com/v1/agents/{agentId}/ws
```

### Authentication

Send immediately after connection:

```json
{
  "type": "auth",
  "token": "Bearer <token>"
}
```

### Message Format

```json
{
  "type": "message | event | command",
  "data": {}
}
```

### Real-time Messages

**Server → Client**

```json
{
  "type": "message",
  "data": {
    "messageId": "msg-001",
    "performative": "request",
    "sender": "agent-002",
    "content": {}
  }
}
```

**Client → Server**

```json
{
  "type": "message",
  "data": {
    "performative": "inform",
    "receiver": ["agent-002"],
    "content": {}
  }
}
```

### Events

```json
{
  "type": "event",
  "data": {
    "eventType": "task-assigned | agent-joined | system-alert",
    "timestamp": "2025-12-25T10:00:00Z",
    "details": {}
  }
}
```

### Heartbeat

**Client → Server** (every 30 seconds)

```json
{
  "type": "ping"
}
```

**Server → Client**

```json
{
  "type": "pong",
  "timestamp": "2025-12-25T10:00:00Z"
}
```

## Error Responses

### 400 Bad Request

```json
{
  "error": "invalid_request",
  "message": "Missing required field: performative",
  "details": {
    "field": "performative",
    "expected": "string"
  }
}
```

### 401 Unauthorized

```json
{
  "error": "unauthorized",
  "message": "Invalid or expired token"
}
```

### 404 Not Found

```json
{
  "error": "not_found",
  "message": "Agent not found: agent-999"
}
```

### 429 Too Many Requests

```json
{
  "error": "rate_limit_exceeded",
  "message": "Too many requests",
  "retryAfter": 60
}
```

## Rate Limits

- **Standard Agents**: 100 requests/minute
- **Verified Agents**: 1000 requests/minute
- **WebSocket Messages**: 10 messages/second

## Pagination

```
GET /agents?page=2&limit=50
```

**Response Headers**

```
X-Total-Count: 150
X-Page: 2
X-Per-Page: 50
Link: </agents?page=3&limit=50>; rel="next"
```

## Versioning

API version in URL and header:

```
GET /v1/agents

Accept: application/vnd.wia.v1+json
```

## Discovery (Agent Card Endpoint)

Every WIA-AI-016 host MUST expose its Agent Card at a stable well-known URL:

```
GET /.well-known/wia-agent-card
Accept: application/json
```

**Response 200 OK** — body matches the Agent Card schema in Phase 1. Hosts that front multiple agents MUST also expose:

```
GET /.well-known/wia-agent-index
Accept: application/json
```

```json
{
  "schemaVersion": "wia.ai-016.agent-index/1",
  "agents": [
    {"agentId": "agent-001", "agentCardUrl": "https://api.example.com/agents/agent-001/card"},
    {"agentId": "agent-002", "agentCardUrl": "https://api.example.com/agents/agent-002/card"}
  ],
  "next": null
}
```

Discovery responses MUST set `Cache-Control: max-age=300` or shorter; clients MUST honour `ETag`/`If-None-Match` per RFC 9111.

## JSON-RPC Tools Endpoint (MCP-compatible)

In addition to the REST surface, every agent SHOULD expose a JSON-RPC 2.0 endpoint for direct tool invocation by LLM-aware peers. This endpoint adopts the Model Context Protocol method shapes so MCP hosts can connect without a translator.

```
POST /agents/{agentId}/jsonrpc
Content-Type: application/json
```

### `tools/list`

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "tools/list",
  "params": {"cursor": null}
}
```

**Response**

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "tools": [
      {
        "name": "measure-temperature",
        "title": "Measure room temperature",
        "description": "Returns the current temperature for a named zone in degrees Celsius.",
        "inputSchema": {"$ref": "../schemas/measure-temperature.input.json"},
        "outputSchema": {"$ref": "../schemas/measure-temperature.output.json"},
        "annotations": {"readOnlyHint": true, "idempotentHint": true}
      }
    ],
    "nextCursor": null
  }
}
```

### `tools/call`

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/call",
  "params": {
    "name": "measure-temperature",
    "arguments": {"zone": "room-101"}
  }
}
```

**Response**

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "result": {
    "content": [
      {"type": "text", "text": "Temperature in room-101 is 25.0 °C."}
    ],
    "structuredContent": {
      "zone": "room-101",
      "celsius": 25.0,
      "measuredAt": "2025-12-25T10:00:00Z"
    },
    "isError": false
  }
}
```

JSON-RPC error replies MUST follow JSON-RPC 2.0 error semantics. Reserved error codes:

| Code | Meaning |
|------|---------|
| -32700 | Parse error |
| -32600 | Invalid Request |
| -32601 | Method not found |
| -32602 | Invalid params |
| -32603 | Internal error |
| -32001 | Tool execution error (WIA-AI-016 reserved) |
| -32002 | Tool not authorized (WIA-AI-016 reserved) |

### `notifications/tools/list_changed`

Servers MAY send the JSON-RPC notification `notifications/tools/list_changed` (no `id`) over a long-lived transport when their tool catalog changes. Clients receiving this notification SHOULD re-issue `tools/list`.

## Server-Sent Events (Streaming Tasks)

For tasks whose results emerge over time, agents MUST offer an SSE stream alongside the create response:

```
GET /tasks/{taskId}/events
Accept: text/event-stream
```

Each event uses the `text/event-stream` media type (HTML Living Standard `EventSource` semantics):

```
event: task.update
data: {"taskId":"task-001","status":"in-progress","progress":0.4}

event: task.completed
data: {"taskId":"task-001","status":"completed","result":{"celsius":25.0}}
```

Streams MUST emit a heartbeat comment line (`:keepalive`) at least every 30 seconds.

## Webhook Events

Agents that prefer push delivery to long-lived sockets MAY register webhooks:

**POST** `/agents/{agentId}/webhooks`

```json
{
  "url": "https://callbacks.example.com/wia",
  "events": ["task.completed", "agent-state-changed"],
  "secret": "<base64 random ≥ 32 bytes>",
  "ttl": "30d"
}
```

Each delivery MUST include:

```
POST /wia
Content-Type: application/json
WIA-Event-Id: 01HG...
WIA-Event-Type: task.completed
WIA-Signature: t=1735128000,v1=hexdigest
```

Receivers MUST recompute `HMAC-SHA-256(secret, "{t}.{body}")` and compare in constant time. Replays older than 5 minutes MUST be rejected.

## Bulk Operations

To reduce request fan-out, agents MAY accept batched requests at:

**POST** `/agents/{agentId}/messages:batch`

```json
{
  "messages": [
    {"performative": "inform", "receiver": ["agent-002"], "content": {"x": 1}},
    {"performative": "inform", "receiver": ["agent-002"], "content": {"x": 2}}
  ]
}
```

Batches MUST contain ≤ 100 entries and ≤ 5 MB total. Each entry returns an individual status:

```json
{
  "results": [
    {"index": 0, "status": "queued", "messageId": "msg-001"},
    {"index": 1, "status": "rejected", "error": "rate_limit_exceeded"}
  ]
}
```

## Idempotency

State-changing requests SHOULD accept the header `Idempotency-Key: <ULID>`. The server MUST cache the response body and status for at least 24 hours and replay them when the same key reappears, even if the original request is retried. Keys are scoped to (agentId, route, key).

## Problem Details (RFC 9457)

All error responses MUST be encoded as `application/problem+json` per RFC 9457:

```json
{
  "type": "https://standards.wia.example/multi-agent-system/errors/rate-limit",
  "title": "Rate limit exceeded",
  "status": 429,
  "detail": "Standard agents are limited to 100 requests/minute.",
  "instance": "/agents/agent-001/messages",
  "retryAfter": 60
}
```

Vendor-specific fields MUST be added under unique top-level keys; never override the core RFC 9457 members.

## OpenAPI Conformance

A normative OpenAPI 3.1 document for this Phase is published at:

```
https://standards.wia.example/multi-agent-system/openapi.yaml
```

Implementations MUST pass the conformance suite at `cli/conformance.sh`, which exercises every operation listed above against a deployed endpoint and asserts schema-valid responses.

## Normative References

- JSON-RPC 2.0 — JSON-RPC Working Group
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9111 — HTTP Caching
- IETF RFC 6585 — Additional HTTP Status Codes
- IETF RFC 7235 — HTTP/1.1 Authentication
- IETF RFC 8615 — Well-Known URIs
- WHATWG HTML — Server-Sent Events (`EventSource`)
- OpenAPI Specification 3.1 — OpenAPI Initiative

---

**WIA-AI-016 Phase 2 Specification v1.0**
© 2025 SmileStory Inc. / World Certification Industry Association
弘益人間 (홍익인간) · Benefit All Humanity
