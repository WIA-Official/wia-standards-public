# WIA-DATA-010: PHASE 2 - API Interface Specification

**Version:** 1.0.0  
**Status:** Complete  
**Last Updated:** 2025-01-15

---

## Overview

This document specifies the API interfaces for WIA-DATA-010 Data Integration Standard, including REST APIs, GraphQL, gRPC, and webhook specifications.

## 1. REST API Specification

### 1.1 Base URL Structure
```
https://api.{service}.wia-standards.io/v1
```

### 1.2 Authentication
- **Method:** Bearer Token (OAuth 2.0)
- **Header:** `Authorization: Bearer {access_token}`
- **Token Format:** JWT (JSON Web Token)

**Example Request:**
```http
GET /pipelines HTTP/1.1
Host: api.data-integration.wia-standards.io
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json
```

### 1.3 Standard Headers

| Header | Required | Description |
|--------|----------|-------------|
| `Authorization` | Yes | Bearer token for authentication |
| `Content-Type` | Yes (for POST/PUT) | `application/json` |
| `Accept` | No | Response format (default: `application/json`) |
| `X-Request-ID` | No | Unique request identifier for tracing |
| `X-API-Version` | No | Specific API version (overrides URL version) |

### 1.4 HTTP Methods

| Method | Usage | Idempotent | Safe |
|--------|-------|------------|------|
| GET | Retrieve resources | Yes | Yes |
| POST | Create resources | No | No |
| PUT | Update/replace resources | Yes | No |
| PATCH | Partial update | No | No |
| DELETE | Remove resources | Yes | No |

### 1.5 Response Codes

| Code | Meaning | When to Use |
|------|---------|-------------|
| 200 OK | Success | GET, PUT, PATCH requests succeeded |
| 201 Created | Resource created | POST request succeeded |
| 204 No Content | Success, no response body | DELETE succeeded |
| 400 Bad Request | Invalid request | Validation errors, malformed JSON |
| 401 Unauthorized | Authentication failed | Missing or invalid token |
| 403 Forbidden | Authorization failed | Valid token, insufficient permissions |
| 404 Not Found | Resource not found | Resource doesn't exist |
| 409 Conflict | Resource conflict | Duplicate resource, version conflict |
| 422 Unprocessable Entity | Semantic errors | Business logic validation failed |
| 429 Too Many Requests | Rate limit exceeded | Client exceeded rate limit |
| 500 Internal Server Error | Server error | Unexpected server error |
| 503 Service Unavailable | Service down | Maintenance, overload |

### 1.6 Error Response Format
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid pipeline configuration",
    "details": [
      {
        "field": "source.type",
        "issue": "Unsupported source type 'foobar'",
        "allowed_values": ["postgresql", "mysql", "mongodb", "kafka"]
      }
    ],
    "request_id": "req_a1b2c3d4e5f6",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 1.7 Pagination
**Query Parameters:**
- `limit`: Number of items per page (default: 20, max: 100)
- `offset`: Number of items to skip (default: 0)
- `cursor`: Cursor-based pagination token (alternative to offset)

**Response Format:**
```json
{
  "data": [...],
  "pagination": {
    "total": 1250,
    "limit": 20,
    "offset": 40,
    "has_more": true,
    "next_cursor": "cursor_xyz123"
  }
}
```

### 1.8 Filtering and Sorting
**Filtering:**
```
GET /pipelines?status=running&created_after=2025-01-01
```

**Sorting:**
```
GET /pipelines?sort=-created_at,name
```
- Prefix `-` for descending order
- Comma-separated for multiple fields

### 1.9 Field Selection (Sparse Fieldsets)
```
GET /pipelines?fields=id,name,status
```

## 2. API Endpoints

### 2.1 Pipeline Management

#### Create Pipeline
```http
POST /pipelines
Content-Type: application/json

{
  "name": "customer-sync",
  "description": "Sync customers from Salesforce to Snowflake",
  "source": {
    "type": "salesforce",
    "config": {
      "instance_url": "https://example.salesforce.com",
      "api_version": "v55.0"
    }
  },
  "destination": {
    "type": "snowflake",
    "config": {
      "account": "xyz12345",
      "warehouse": "COMPUTE_WH",
      "database": "ANALYTICS",
      "schema": "STAGING"
    }
  },
  "schedule": "0 */6 * * *",
  "transformations": [...]
}
```

**Response (201 Created):**
```json
{
  "pipeline_id": "pip_a1b2c3d4e5f6",
  "name": "customer-sync",
  "status": "created",
  "created_at": "2025-01-15T10:30:00Z",
  "updated_at": "2025-01-15T10:30:00Z"
}
```

#### Get Pipeline
```http
GET /pipelines/{pipeline_id}
```

**Response (200 OK):**
```json
{
  "pipeline_id": "pip_a1b2c3d4e5f6",
  "name": "customer-sync",
  "status": "running",
  "last_run": {
    "run_id": "run_xyz789",
    "started_at": "2025-01-15T12:00:00Z",
    "completed_at": "2025-01-15T12:15:30Z",
    "records_processed": 125000,
    "status": "success"
  },
  "next_run": "2025-01-15T18:00:00Z"
}
```

#### List Pipelines
```http
GET /pipelines?status=running&limit=20
```

#### Update Pipeline
```http
PUT /pipelines/{pipeline_id}
PATCH /pipelines/{pipeline_id}
```

#### Delete Pipeline
```http
DELETE /pipelines/{pipeline_id}
```

### 2.2 Run Management

#### Trigger Run
```http
POST /pipelines/{pipeline_id}/runs

{
  "trigger_type": "manual",
  "parameters": {
    "full_refresh": false
  }
}
```

#### Get Run Status
```http
GET /runs/{run_id}
```

#### List Runs
```http
GET /pipelines/{pipeline_id}/runs?status=failed&limit=50
```

#### Get Run Logs
```http
GET /runs/{run_id}/logs?level=error&limit=1000
```

### 2.3 Connector Management

#### List Connectors
```http
GET /connectors
```

**Response:**
```json
{
  "connectors": [
    {
      "id": "postgresql",
      "name": "PostgreSQL",
      "type": "source",
      "category": "database",
      "versions": ["12", "13", "14", "15"]
    },
    {
      "id": "snowflake",
      "name": "Snowflake",
      "type": "destination",
      "category": "warehouse"
    }
  ]
}
```

#### Get Connector Schema
```http
GET /connectors/{connector_id}/schema
```

### 2.4 Monitoring & Metrics

#### Get Pipeline Metrics
```http
GET /pipelines/{pipeline_id}/metrics?period=24h
```

**Response:**
```json
{
  "metrics": {
    "records_processed": 1250000,
    "avg_runtime_seconds": 930,
    "success_rate": 0.98,
    "error_count": 15,
    "throughput_records_per_sec": 1344
  },
  "period": "24h",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

## 3. GraphQL API

### 3.1 Schema
```graphql
type Pipeline {
  id: ID!
  name: String!
  description: String
  status: PipelineStatus!
  source: Source!
  destination: Destination!
  schedule: String
  createdAt: DateTime!
  updatedAt: DateTime!
  runs(first: Int, status: RunStatus): [Run!]!
}

type Run {
  id: ID!
  pipeline: Pipeline!
  status: RunStatus!
  startedAt: DateTime!
  completedAt: DateTime
  recordsProcessed: Int!
  errorCount: Int!
  logs(level: LogLevel): [LogEntry!]!
}

enum PipelineStatus {
  CREATED
  RUNNING
  PAUSED
  FAILED
}

enum RunStatus {
  PENDING
  RUNNING
  SUCCESS
  FAILED
}

type Query {
  pipeline(id: ID!): Pipeline
  pipelines(status: PipelineStatus, limit: Int): [Pipeline!]!
  run(id: ID!): Run
}

type Mutation {
  createPipeline(input: CreatePipelineInput!): Pipeline!
  updatePipeline(id: ID!, input: UpdatePipelineInput!): Pipeline!
  deletePipeline(id: ID!): Boolean!
  triggerRun(pipelineId: ID!): Run!
}
```

### 3.2 Example Queries
```graphql
query {
  pipeline(id: "pip_a1b2c3d4e5f6") {
    id
    name
    status
    runs(first: 10, status: SUCCESS) {
      id
      startedAt
      recordsProcessed
    }
  }
}
```

## 4. gRPC API

### 4.1 Service Definition (Protobuf)
```protobuf
syntax = "proto3";

package wia.data.integration.v1;

service PipelineService {
  rpc CreatePipeline(CreatePipelineRequest) returns (Pipeline);
  rpc GetPipeline(GetPipelineRequest) returns (Pipeline);
  rpc ListPipelines(ListPipelinesRequest) returns (ListPipelinesResponse);
  rpc DeletePipeline(DeletePipelineRequest) returns (DeletePipelineResponse);
  rpc TriggerRun(TriggerRunRequest) returns (Run);
}

message Pipeline {
  string pipeline_id = 1;
  string name = 2;
  string description = 3;
  PipelineStatus status = 4;
  Source source = 5;
  Destination destination = 6;
  string schedule = 7;
  int64 created_at = 8;
  int64 updated_at = 9;
}

enum PipelineStatus {
  PIPELINE_STATUS_UNSPECIFIED = 0;
  PIPELINE_STATUS_CREATED = 1;
  PIPELINE_STATUS_RUNNING = 2;
  PIPELINE_STATUS_PAUSED = 3;
  PIPELINE_STATUS_FAILED = 4;
}
```

## 5. Webhooks

### 5.1 Webhook Events
- `pipeline.created`
- `pipeline.updated`
- `pipeline.deleted`
- `run.started`
- `run.completed`
- `run.failed`

### 5.2 Webhook Payload
```json
{
  "event_id": "evt_xyz123",
  "event_type": "run.completed",
  "timestamp": "2025-01-15T12:15:30Z",
  "data": {
    "run_id": "run_xyz789",
    "pipeline_id": "pip_a1b2c3d4e5f6",
    "status": "success",
    "records_processed": 125000,
    "started_at": "2025-01-15T12:00:00Z",
    "completed_at": "2025-01-15T12:15:30Z"
  }
}
```

### 5.3 Webhook Signature
- **Header:** `X-WIA-Signature`
- **Algorithm:** HMAC-SHA256
- **Format:** `sha256=<hex_digest>`

**Verification:**
```python
import hmac
import hashlib

def verify_webhook(payload, signature, secret):
    expected = hmac.new(
        secret.encode(),
        payload.encode(),
        hashlib.sha256
    ).hexdigest()
    return hmac.compare_digest(f"sha256={expected}", signature)
```

## 6. Rate Limiting

### 6.1 Rate Limit Headers
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1610723400
```

### 6.2 Rate Limit Tiers
| Tier | Requests/minute | Burst |
|------|-----------------|-------|
| Free | 100 | 120 |
| Standard | 1000 | 1200 |
| Professional | 5000 | 6000 |
| Enterprise | Custom | Custom |

## 7. Versioning

### 7.1 URL Versioning
```
https://api.example.com/v1/pipelines
https://api.example.com/v2/pipelines
```

### 7.2 Header Versioning
```http
X-API-Version: 2.0
```

### 7.3 Deprecation
- Announce deprecation 6 months in advance
- Support old version for 12 months after new version release
- Include `Sunset` header in responses from deprecated APIs

```http
Sunset: Sat, 15 Jul 2025 23:59:59 GMT
```

---

**Previous Phase:** [PHASE 1 - Data Format](PHASE1-DATA-FORMAT.md)  
**Next Phase:** [PHASE 3 - Protocol](PHASE3-PROTOCOL.md)

**Status:** ✅ Complete  
**Compliance:** WIA-DATA-010 v1.0.0
