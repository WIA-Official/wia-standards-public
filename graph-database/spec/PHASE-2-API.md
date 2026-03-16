# WIA-DATA-015: Graph Database Standard
## PHASE 2: API Specification

**Version:** 1.0  
**Status:** Draft  
**Last Updated:** 2025-01-15

---

## 1. Overview

This document defines the API specifications for the WIA-DATA-015 Graph Database Standard.

## 2. RESTful API Endpoints

### 2.1 Base URL Structure

```
https://api.wia-data-015.org/v1
```

### 2.2 Authentication

#### Bearer Token Authentication
```http
GET /nodes HTTP/1.1
Host: api.wia-data-015.org
Authorization: Bearer <access_token>
Content-Type: application/json
```

#### API Key Authentication
```http
GET /nodes HTTP/1.1
Host: api.wia-data-015.org
X-API-Key: <api_key>
Content-Type: application/json
```

## 3. Node Operations

### 3.1 Create Node

**Request:**
```http
POST /nodes
Content-Type: application/json

{
  "labels": ["Person"],
  "properties": {
    "name": "Alice",
    "email": "alice@example.com",
    "age": 30
  }
}
```

**Response:**
```json
{
  "id": "node_12345",
  "labels": ["Person"],
  "properties": {
    "name": "Alice",
    "email": "alice@example.com",
    "age": 30
  },
  "metadata": {
    "created": "2024-01-15T10:30:00Z",
    "version": 1
  }
}
```

### 3.2 Get Node

```http
GET /nodes/{nodeId}
```

### 3.3 Update Node

```http
PUT /nodes/{nodeId}
PATCH /nodes/{nodeId}
```

### 3.4 Delete Node

```http
DELETE /nodes/{nodeId}
```

## 4. Edge Operations

### 4.1 Create Edge

```http
POST /edges

{
  "type": "KNOWS",
  "source": "node_12345",
  "target": "node_67890",
  "properties": {
    "since": "2020-01-01"
  }
}
```

### 4.2 Query Edges

```http
GET /nodes/{nodeId}/edges?direction=out&type=KNOWS
```

## 5. Query API

### 5.1 Execute Query

```http
POST /query

{
  "query": "MATCH (n:Person) WHERE n.age > $age RETURN n",
  "parameters": {"age": 25},
  "limit": 100
}
```

### 5.2 Batch Queries

```http
POST /query/batch

{
  "queries": [
    {"query": "MATCH (n:Person) RETURN count(n)"},
    {"query": "MATCH ()-[r]->() RETURN count(r)"}
  ]
}
```

## 6. Graph Algorithms API

### 6.1 Shortest Path

```http
POST /algorithms/shortestPath

{
  "start": "node_123",
  "end": "node_456",
  "relationshipType": "KNOWS",
  "maxDepth": 5
}
```

### 6.2 PageRank

```http
POST /algorithms/pageRank

{
  "nodeLabel": "Person",
  "relationshipType": "FOLLOWS",
  "iterations": 20,
  "dampingFactor": 0.85
}
```

## 7. Bulk Operations

### 7.1 Bulk Import

```http
POST /bulk/import
Content-Type: application/json

{
  "nodes": [...],
  "edges": [...],
  "mode": "create"
}
```

### 7.2 Bulk Export

```http
GET /bulk/export?format=json&nodeLabel=Person
```

## 8. Schema Management

### 8.1 Get Schema

```http
GET /schema
```

### 8.2 Create Constraint

```http
POST /schema/constraints

{
  "type": "unique",
  "nodeLabel": "User",
  "property": "email"
}
```

### 8.3 Create Index

```http
POST /schema/indexes

{
  "nodeLabel": "Person",
  "property": "name",
  "type": "btree"
}
```

## 9. Error Handling

### 9.1 Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid node properties",
    "details": {
      "field": "email",
      "issue": "Invalid email format"
    },
    "timestamp": "2024-01-15T10:30:00Z",
    "requestId": "req_abc123"
  }
}
```

### 9.2 HTTP Status Codes

- 200: Success
- 201: Created
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 409: Conflict
- 429: Too Many Requests
- 500: Internal Server Error

## 10. Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1640000000
```

## 11. Pagination

```http
GET /nodes?limit=100&offset=0
GET /nodes?limit=100&cursor=eyJpZCI6MTIzNDV9
```

## 12. WebSocket API

### 12.1 Real-time Updates

```javascript
ws://api.wia-data-015.org/ws

// Subscribe to node changes
{
  "action": "subscribe",
  "nodeLabel": "Person",
  "events": ["create", "update", "delete"]
}
```

---

**License:** CC BY 4.0  
**Contact:** standards@wia-official.org
