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

---

**WIA-AI-016 Phase 2 Specification v1.0**
© 2025 SmileStory Inc. / World Certification Industry Association
弘益人間 (홍익인간) · Benefit All Humanity
