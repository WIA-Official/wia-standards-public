# WIA-SOC-012 Telecommunications Infrastructure Standard
## Phase 2: API Specification

> **Version**: 1.0.0  
> **Status**: Stable  
> **Last Updated**: 2025

---

## 1. Overview

Phase 2 defines RESTful API endpoints for managing telecommunications infrastructure. This specification enables standardized programmatic access to infrastructure data, configuration, and monitoring.

### 1.1 Design Principles

- **RESTful**: Standard HTTP methods and status codes
- **Stateless**: No server-side session management
- **Versioned**: API version in URL path
- **Secure**: Authentication and authorization required
- **Performant**: Pagination, filtering, caching support

---

## 2. Base URL and Versioning

\`\`\`
Base URL: https://api.wiastandards.com/v1/telecom-infra
Version: v1 (current)
\`\`\`

### 2.1 API Versioning Strategy

- Major version in URL path (/v1/, /v2/)
- Semantic versioning for documentation
- Minimum 12-month support for deprecated versions
- Deprecation headers in responses

---

## 3. Authentication

### 3.1 API Key Authentication

\`\`\`http
GET /infrastructure HTTP/1.1
Host: api.wiastandards.com
Authorization: Bearer <api_key>
\`\`\`

### 3.2 OAuth 2.0

Supported flows:
- Authorization Code (for user-facing applications)
- Client Credentials (for server-to-server)
- Refresh Token

\`\`\`http
POST /oauth/token HTTP/1.1
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret"
}
\`\`\`

---

## 4. Core API Endpoints

### 4.1 Infrastructure Resources

#### GET /infrastructure
List all infrastructure elements

**Query Parameters:**
- `type` (string): Filter by type (cell_tower, fiber_node, etc.)
- `location` (string): Filter by geographic area (bbox or radius)
- `operator_id` (string): Filter by operator
- `status` (string): Filter by status (operational, maintenance, offline)
- `page` (integer): Page number (default: 1)
- `limit` (integer): Items per page (default: 50, max: 500)

**Example Request:**
\`\`\`http
GET /v1/telecom-infra/infrastructure?type=cell_tower&status=operational&page=1&limit=100
\`\`\`

**Example Response:**
\`\`\`json
{
  "data": [
    {
      "infra_id": "550e8400-e29b-41d4-a716-446655440000",
      "type": "cell_tower",
      "location": {
        "latitude": 37.7749,
        "longitude": -122.4194
      },
      "status": "operational"
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 100,
    "total": 1523,
    "pages": 16
  },
  "links": {
    "self": "/v1/telecom-infra/infrastructure?page=1",
    "next": "/v1/telecom-infra/infrastructure?page=2",
    "last": "/v1/telecom-infra/infrastructure?page=16"
  }
}
\`\`\`

#### GET /infrastructure/{id}
Retrieve specific infrastructure element

**Path Parameters:**
- `id` (uuid): Infrastructure element ID

**Example Response:**
\`\`\`json
{
  "infra_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-15T14:30:00Z",
  "version": "1.0.0",
  "type": "cell_tower",
  "location": {...},
  "specifications": {...},
  "telemetry": {...}
}
\`\`\`

#### POST /infrastructure
Create new infrastructure element

**Request Body:** Full infrastructure object (as per Phase 1)

**Example Response:**
\`\`\`json
{
  "infra_id": "new-uuid",
  "status": "created",
  "message": "Infrastructure element created successfully"
}
\`\`\`

#### PUT /infrastructure/{id}
Update infrastructure element

**Request Body:** Updated infrastructure object

#### PATCH /infrastructure/{id}
Partial update of infrastructure element

**Request Body:** Fields to update
\`\`\`json
{
  "status": "maintenance",
  "telemetry": {
    "power": {
      "consumption_watts": 3200
    }
  }
}
\`\`\`

#### DELETE /infrastructure/{id}
Delete infrastructure element (logical delete)

**Response:**
\`\`\`json
{
  "infra_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "deleted",
  "deleted_at": "2025-01-15T15:00:00Z"
}
\`\`\`

### 4.2 Telemetry Endpoints

#### GET /infrastructure/{id}/telemetry
Get real-time telemetry data

**Query Parameters:**
- `metrics` (string): Comma-separated metrics (e.g., "throughput,latency,power")
- `start_time` (string): ISO 8601 timestamp
- `end_time` (string): ISO 8601 timestamp
- `interval` (string): Aggregation interval (1m, 5m, 1h, 1d)

**Example Response:**
\`\`\`json
{
  "infra_id": "550e8400-e29b-41d4-a716-446655440000",
  "time_range": {
    "start": "2025-01-15T00:00:00Z",
    "end": "2025-01-15T23:59:59Z"
  },
  "data": [
    {
      "timestamp": "2025-01-15T14:00:00Z",
      "metrics": {
        "throughput_mbps": 2500,
        "latency_ms": 10,
        "power_consumption_watts": 3500
      }
    }
  ]
}
\`\`\`

#### POST /infrastructure/{id}/telemetry
Submit telemetry data

**Request Body:**
\`\`\`json
{
  "timestamp": "2025-01-15T14:30:00Z",
  "telemetry": {
    "performance": {...},
    "power": {...},
    "environmental": {...}
  }
}
\`\`\`

### 4.3 Network Topology

#### GET /topology
Get network topology graph

**Query Parameters:**
- `bbox` (string): Bounding box (lon_min,lat_min,lon_max,lat_max)
- `max_depth` (integer): Maximum hops from starting nodes

**Response:**
\`\`\`json
{
  "nodes": [...],
  "links": [...],
  "metadata": {
    "node_count": 250,
    "link_count": 420
  }
}
\`\`\`

### 4.4 Coverage Analysis

#### GET /coverage/analyze
Analyze coverage for a geographic area

**Query Parameters:**
- `bbox` (string): Bounding box
- `technology` (string): Filter by technology (4G, 5G)
- `operator_id` (string): Specific operator

**Response:**
\`\`\`json
{
  "coverage": {
    "area_km2": 100,
    "outdoor_coverage_percent": 98.5,
    "indoor_coverage_percent": 85.2,
    "average_signal_strength_dbm": -75,
    "technology_breakdown": {
      "5G": 45.2,
      "4G": 98.5,
      "3G": 99.8
    }
  },
  "quality_metrics": {
    "average_throughput_mbps": 150,
    "average_latency_ms": 15
  }
}
\`\`\`

### 4.5 Spectrum Management

#### GET /spectrum/allocations
List spectrum allocations

**Query Parameters:**
- `frequency_band` (string): E.g., "3.5 GHz", "28 GHz"
- `operator_id` (string): Filter by operator
- `license_type` (string): exclusive, shared, unlicensed

#### POST /spectrum/allocations
Create new spectrum allocation

#### GET /spectrum/utilization
Get spectrum utilization metrics

---

## 5. HTTP Status Codes

### 5.1 Success Codes
- `200 OK`: Request successful
- `201 Created`: Resource created
- `202 Accepted`: Async operation accepted
- `204 No Content`: Success, no response body

### 5.2 Client Error Codes
- `400 Bad Request`: Invalid request
- `401 Unauthorized`: Authentication required
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `409 Conflict`: Resource conflict
- `422 Unprocessable Entity`: Validation error
- `429 Too Many Requests`: Rate limit exceeded

### 5.3 Server Error Codes
- `500 Internal Server Error`: Server error
- `502 Bad Gateway`: Upstream service error
- `503 Service Unavailable`: Temporary unavailability
- `504 Gateway Timeout`: Upstream timeout

---

## 6. Error Response Format

\`\`\`json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid infrastructure data",
    "details": [
      {
        "field": "location.latitude",
        "message": "Must be between -90 and 90"
      }
    ],
    "request_id": "req_abc123",
    "timestamp": "2025-01-15T14:30:00Z"
  }
}
\`\`\`

---

## 7. Rate Limiting

### 7.1 Rate Limits
- Standard tier: 100 requests/minute
- Premium tier: 1000 requests/minute
- Enterprise tier: Custom limits

### 7.2 Headers
\`\`\`
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 75
X-RateLimit-Reset: 1642265400
\`\`\`

---

## 8. Pagination

### 8.1 Cursor-Based Pagination
Recommended for large datasets:

\`\`\`http
GET /infrastructure?cursor=eyJpZCI6MTIzfQ&limit=100
\`\`\`

### 8.2 Offset-Based Pagination
Simple but less efficient:

\`\`\`http
GET /infrastructure?page=2&limit=100
\`\`\`

---

## 9. Filtering and Sorting

### 9.1 Filtering
Multiple filter formats supported:
\`\`\`
?type=cell_tower
?status[]=operational&status[]=maintenance
?location[near]=37.7749,-122.4194&location[radius]=10km
\`\`\`

### 9.2 Sorting
\`\`\`
?sort=created_at:desc,location.latitude:asc
\`\`\`

---

## 10. Webhooks

### 10.1 Event Types
- `infrastructure.created`
- `infrastructure.updated`
- `infrastructure.deleted`
- `telemetry.threshold_exceeded`
- `alert.triggered`

### 10.2 Webhook Payload
\`\`\`json
{
  "event_id": "evt_abc123",
  "event_type": "infrastructure.updated",
  "timestamp": "2025-01-15T14:30:00Z",
  "data": {
    "infra_id": "550e8400-e29b-41d4-a716-446655440000",
    "changes": {
      "status": {
        "old": "operational",
        "new": "maintenance"
      }
    }
  }
}
\`\`\`

---

## 11. SDK Support

Official SDKs available for:
- TypeScript/JavaScript (npm: @wia/telecom-infra-sdk)
- Python (pip: wia-telecom-infra)
- Java (Maven: com.wia:telecom-infra-sdk)
- Go (go get github.com/wia/telecom-infra-go)

---

**WIA-SOC-012 Phase 2 v1.0**  
© 2025 SmileStory Inc. / WIA
