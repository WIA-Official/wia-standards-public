# WIA-DATA-008: Data Lineage Standard
## PHASE 2: API Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

This phase defines the RESTful API for submitting, querying, and managing data lineage metadata.

## 1. API Design Principles

- **RESTful**: Follow REST architectural style
- **JSON**: Request and response bodies in JSON
- **Versioned**: API version in URL path
- **Stateless**: Each request contains all necessary information
- **Idempotent**: Safe to retry requests
- **Paginated**: Large result sets use pagination

## 2. Base URL

```
https://api.lineage.example.com/v1
```

## 3. Authentication

### 3.1 API Key

```http
Authorization: Bearer <api_key>
```

### 3.2 OAuth 2.0

Support for OAuth 2.0 client credentials flow:

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret"
}
```

## 4. Core Endpoints

### 4.1 Submit Lineage Event

**Endpoint:** `POST /lineage/events`

Submit a single lineage event (OpenLineage compatible).

**Request:**
```http
POST /v1/lineage/events
Authorization: Bearer <token>
Content-Type: application/json

{
  "eventType": "COMPLETE",
  "eventTime": "2025-12-26T10:00:00Z",
  "run": {
    "runId": "550e8400-e29b-41d4-a716-446655440000"
  },
  "job": {
    "namespace": "airflow",
    "name": "customer_etl"
  },
  "inputs": [...],
  "outputs": [...]
}
```

**Response:**
```json
{
  "status": "accepted",
  "event_id": "evt_123456",
  "received_at": "2025-12-26T10:00:01Z"
}
```

### 4.2 Batch Submit Events

**Endpoint:** `POST /lineage/events/batch`

Submit multiple events in a single request.

**Request:**
```json
{
  "events": [
    {...event1...},
    {...event2...},
    {...event3...}
  ]
}
```

**Response:**
```json
{
  "status": "accepted",
  "accepted_count": 3,
  "rejected_count": 0,
  "event_ids": ["evt_123", "evt_124", "evt_125"]
}
```

### 4.3 Get Node

**Endpoint:** `GET /lineage/nodes/{node_id}`

Retrieve a specific node.

**Response:**
```json
{
  "node_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "node_type": "table",
  "namespace": "postgres://warehouse:5432",
  "name": "customers",
  "qualified_name": "postgres://warehouse:5432/public.customers",
  "metadata": {...},
  "schema": {...}
}
```

### 4.4 Search Nodes

**Endpoint:** `GET /lineage/nodes`

Search for nodes.

**Query Parameters:**
- `q`: Search query
- `type`: Filter by node type
- `namespace`: Filter by namespace
- `limit`: Results per page (default: 20, max: 100)
- `offset`: Pagination offset

**Example:**
```http
GET /v1/lineage/nodes?q=customer&type=table&limit=20
```

**Response:**
```json
{
  "total": 47,
  "limit": 20,
  "offset": 0,
  "nodes": [
    {...node1...},
    {...node2...}
  ]
}
```

### 4.5 Get Upstream Lineage

**Endpoint:** `GET /lineage/nodes/{node_id}/upstream`

Get upstream dependencies (sources).

**Query Parameters:**
- `depth`: Maximum depth to traverse (default: 3, max: 10)
- `include_columns`: Include column-level lineage (boolean)

**Response:**
```json
{
  "root_node_id": "a1b2c3d4-...",
  "depth": 3,
  "nodes": [...],
  "edges": [...]
}
```

### 4.6 Get Downstream Lineage

**Endpoint:** `GET /lineage/nodes/{node_id}/downstream`

Get downstream consumers.

**Query Parameters:**
- `depth`: Maximum depth to traverse
- `include_columns`: Include column-level lineage

**Response:** Same structure as upstream

### 4.7 Get Complete Lineage

**Endpoint:** `GET /lineage/nodes/{node_id}/complete`

Get both upstream and downstream lineage.

**Response:**
```json
{
  "root_node_id": "a1b2c3d4-...",
  "upstream_depth": 3,
  "downstream_depth": 3,
  "nodes": [...],
  "edges": [...]
}
```

### 4.8 Get Column Lineage

**Endpoint:** `GET /lineage/columns`

Get column-level lineage.

**Query Parameters:**
- `table`: Qualified table name
- `column`: Column name

**Response:**
```json
{
  "table": "warehouse.fact_sales",
  "column": "revenue",
  "upstream": [
    {
      "table": "staging.transactions",
      "column": "amount",
      "transformation": "SUM(amount)"
    }
  ],
  "downstream": [
    {
      "table": "analytics.monthly_revenue",
      "column": "total_revenue",
      "transformation": "SUM(revenue)"
    }
  ]
}
```

## 5. Impact Analysis

### 5.1 Analyze Impact

**Endpoint:** `POST /lineage/impact`

Analyze impact of potential changes.

**Request:**
```json
{
  "node_id": "a1b2c3d4-...",
  "change_type": "schema_change | deprecation | modification",
  "details": {
    "column": "email",
    "action": "rename",
    "new_value": "contact_email"
  }
}
```

**Response:**
```json
{
  "impact_analysis_id": "impact_123",
  "affected_nodes": 23,
  "criticality": "high",
  "downstream_assets": [
    {
      "node_id": "...",
      "name": "customer_summary",
      "impact_level": "high",
      "reason": "Direct column reference"
    }
  ],
  "recommendations": [
    "Add deprecation period of 30 days",
    "Notify 3 downstream owners"
  ]
}
```

## 6. Root Cause Analysis

### 6.1 Trace Issue

**Endpoint:** `POST /lineage/trace`

Trace data quality issue to root cause.

**Request:**
```json
{
  "node_id": "a1b2c3d4-...",
  "issue": {
    "type": "data_quality",
    "description": "Revenue 30% lower than expected",
    "started_at": "2025-12-19T00:00:00Z"
  }
}
```

**Response:**
```json
{
  "trace_id": "trace_123",
  "root_cause": {
    "node_id": "...",
    "name": "external.payment_api",
    "probable_cause": "API filter changed on 2025-12-19",
    "confidence": 0.87
  },
  "path": [
    {"node": "analytics.revenue", "status": "issue_present"},
    {"node": "staging.sales", "status": "issue_present"},
    {"node": "raw.transactions", "status": "issue_detected"},
    {"node": "external.payment_api", "status": "root_cause"}
  ]
}
```

## 7. Administrative Endpoints

### 7.1 Health Check

**Endpoint:** `GET /health`

**Response:**
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2025-12-26T10:00:00Z"
}
```

### 7.2 Get Statistics

**Endpoint:** `GET /lineage/stats`

**Response:**
```json
{
  "total_nodes": 15247,
  "total_edges": 23891,
  "node_types": {
    "table": 8934,
    "view": 2341,
    "report": 3972
  },
  "last_updated": "2025-12-26T09:45:00Z"
}
```

## 8. Webhooks

### 8.1 Register Webhook

**Endpoint:** `POST /webhooks`

Register a webhook for lineage events.

**Request:**
```json
{
  "url": "https://your-service.com/webhook",
  "events": ["node.created", "node.updated", "edge.created"],
  "secret": "your_webhook_secret"
}
```

**Response:**
```json
{
  "webhook_id": "wh_123",
  "status": "active"
}
```

### 8.2 Webhook Payload

When an event occurs:

```json
{
  "event_type": "node.created",
  "event_id": "evt_456",
  "timestamp": "2025-12-26T10:00:00Z",
  "data": {
    "node_id": "...",
    "node_type": "table",
    "name": "new_table"
  }
}
```

## 9. Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "RESOURCE_NOT_FOUND",
    "message": "Node with ID 'xyz' not found",
    "details": {
      "node_id": "xyz"
    },
    "request_id": "req_789"
  }
}
```

### Error Codes

- `400 BAD_REQUEST`: Invalid request format
- `401 UNAUTHORIZED`: Missing or invalid authentication
- `403 FORBIDDEN`: Insufficient permissions
- `404 NOT_FOUND`: Resource not found
- `409 CONFLICT`: Resource conflict
- `422 UNPROCESSABLE_ENTITY`: Validation error
- `429 RATE_LIMIT_EXCEEDED`: Too many requests
- `500 INTERNAL_SERVER_ERROR`: Server error
- `503 SERVICE_UNAVAILABLE`: Service temporarily unavailable

## 10. Rate Limiting

- **Default**: 1000 requests per hour per API key
- **Batch**: 100 batch submissions per hour
- **Headers**:
  ```
  X-RateLimit-Limit: 1000
  X-RateLimit-Remaining: 847
  X-RateLimit-Reset: 1640520000
  ```

## 11. Pagination

For endpoints returning lists:

**Request:**
```http
GET /v1/lineage/nodes?limit=20&offset=40
```

**Response:**
```json
{
  "total": 247,
  "limit": 20,
  "offset": 40,
  "has_more": true,
  "next": "/v1/lineage/nodes?limit=20&offset=60",
  "data": [...]
}
```

## 12. Filtering and Sorting

**Filtering:**
```http
GET /v1/lineage/nodes?type=table&namespace=postgres://warehouse
```

**Sorting:**
```http
GET /v1/lineage/nodes?sort=name&order=asc
```

## 13. SDKs

Official SDKs available:

- **Python**: `pip install wia-lineage`
- **JavaScript/TypeScript**: `npm install @wia/lineage`
- **Java**: Maven/Gradle dependency
- **Go**: `go get github.com/wia-official/lineage-go`

**Python Example:**
```python
from wia_lineage import LineageClient

client = LineageClient(api_key="your_api_key")

# Submit event
client.submit_event({
    "eventType": "COMPLETE",
    "run": {...},
    "job": {...}
})

# Query lineage
upstream = client.get_upstream("node_id", depth=3)
```

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**
