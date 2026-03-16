# WIA-DATA-014: PHASE 2 - API Interface Specification

**Version:** 1.0.0
**Status:** Complete
**Date:** 2025-12-26

---

## 1. Overview

PHASE 2 defines standard REST API interfaces for time-series data ingestion, querying, and management.

### 1.1 Goals

- **RESTful Design:** Follow REST principles for predictable API behavior
- **Consistency:** Uniform API across different TSDB implementations
- **Performance:** Efficient bulk operations and streaming
- **Security:** Built-in authentication and authorization

---

## 2. Base URL and Versioning

### 2.1 Base URL

```
https://api.example.com/v1/timeseries
```

### 2.2 API Versioning

- Version in URL path: `/v1/`, `/v2/`
- Backward compatibility within major versions
- Deprecation notices 6 months before removal

---

## 3. Authentication

### 3.1 API Key Authentication

```http
Authorization: Bearer YOUR_API_KEY
```

### 3.2 OAuth 2.0

```http
Authorization: Bearer OAUTH_TOKEN
```

### 3.3 Rate Limiting

Response headers:
```http
X-RateLimit-Limit: 10000
X-RateLimit-Remaining: 9523
X-RateLimit-Reset: 1735208400
```

---

## 4. Write Operations

### 4.1 Write Single Point

**Endpoint:** `POST /write`

**Request:**
```json
{
  "database": "metrics",
  "point": {
    "timestamp": "2025-12-26T10:30:00Z",
    "measurement": "temperature",
    "tags": {"location": "server1"},
    "fields": {"value": 23.5}
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "points_written": 1,
  "timestamp": "2025-12-26T10:30:01Z"
}
```

### 4.2 Write Batch

**Endpoint:** `POST /write/batch`

**Request:**
```json
{
  "database": "metrics",
  "points": [
    {
      "timestamp": "2025-12-26T10:30:00Z",
      "measurement": "cpu",
      "tags": {"host": "server1"},
      "fields": {"usage": 75.2}
    },
    {
      "timestamp": "2025-12-26T10:30:00Z",
      "measurement": "memory",
      "tags": {"host": "server1"},
      "fields": {"used": 4096}
    }
  ]
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "points_written": 2,
  "errors": []
}
```

---

## 5. Query Operations

### 5.1 Query by Time Range

**Endpoint:** `GET /query`

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `database` | string | Yes | Database name |
| `measurement` | string | Yes | Measurement name |
| `start` | ISO8601 | Yes | Start timestamp |
| `end` | ISO8601 | Yes | End timestamp |
| `tags` | object | No | Tag filters |
| `fields` | array | No | Field selection |
| `limit` | integer | No | Max results (default: 1000) |

**Example Request:**
```http
GET /query?database=metrics&measurement=temperature&start=2025-12-26T00:00:00Z&end=2025-12-26T23:59:59Z&tags[location]=server1
```

**Response (200 OK):**
```json
{
  "measurement": "temperature",
  "results": [
    {
      "timestamp": "2025-12-26T10:00:00Z",
      "tags": {"location": "server1"},
      "fields": {"value": 22.5}
    },
    {
      "timestamp": "2025-12-26T11:00:00Z",
      "tags": {"location": "server1"},
      "fields": {"value": 23.5}
    }
  ],
  "count": 24
}
```

### 5.2 Aggregation Query

**Endpoint:** `POST /query/aggregate`

**Request:**
```json
{
  "database": "metrics",
  "measurement": "cpu",
  "start": "2025-12-26T00:00:00Z",
  "end": "2025-12-26T23:59:59Z",
  "aggregation": {
    "function": "mean",
    "interval": "1h",
    "field": "usage"
  },
  "tags": {"host": "server1"}
}
```

**Response:**
```json
{
  "results": [
    {"timestamp": "2025-12-26T00:00:00Z", "mean": 45.2},
    {"timestamp": "2025-12-26T01:00:00Z", "mean": 52.1},
    {"timestamp": "2025-12-26T02:00:00Z", "mean": 38.7}
  ]
}
```

---

## 6. Anomaly Detection

### 6.1 Detect Anomalies

**Endpoint:** `POST /anomaly/detect`

**Request:**
```json
{
  "database": "metrics",
  "measurement": "cpu",
  "start": "2025-12-26T00:00:00Z",
  "end": "2025-12-26T23:59:59Z",
  "method": "zscore",
  "threshold": 3.0,
  "field": "usage"
}
```

**Response:**
```json
{
  "anomalies": [
    {
      "timestamp": "2025-12-26T14:23:15Z",
      "value": 98.5,
      "zscore": 3.2,
      "severity": "high"
    }
  ],
  "total_anomalies": 5,
  "method": "zscore",
  "threshold": 3.0
}
```

---

## 7. Forecasting

### 7.1 Generate Forecast

**Endpoint:** `POST /forecast`

**Request:**
```json
{
  "database": "metrics",
  "measurement": "network_traffic",
  "history_start": "2025-12-20T00:00:00Z",
  "history_end": "2025-12-26T00:00:00Z",
  "forecast_periods": 24,
  "interval": "1h",
  "model": "arima",
  "field": "bytes_in"
}
```

**Response:**
```json
{
  "forecast": [
    {
      "timestamp": "2025-12-27T00:00:00Z",
      "predicted_value": 156.3,
      "confidence_interval": [142.1, 170.5],
      "confidence_level": 0.95
    }
  ],
  "model": "arima",
  "accuracy_metrics": {
    "mape": 5.2,
    "rmse": 12.3
  }
}
```

---

## 8. Management Operations

### 8.1 List Measurements

**Endpoint:** `GET /measurements`

**Response:**
```json
{
  "measurements": [
    {"name": "temperature", "count": 1000000},
    {"name": "cpu", "count": 5000000},
    {"name": "memory", "count": 5000000}
  ]
}
```

### 8.2 Delete Data

**Endpoint:** `DELETE /data`

**Parameters:**
```http
DELETE /data?database=metrics&measurement=temperature&start=2025-01-01T00:00:00Z&end=2025-01-31T23:59:59Z
```

**Response:**
```json
{
  "status": "success",
  "points_deleted": 2592000
}
```

### 8.3 Retention Policy

**Endpoint:** `POST /retention`

**Request:**
```json
{
  "database": "metrics",
  "name": "one_week",
  "duration": "7d",
  "replication": 1,
  "default": false
}
```

---

## 9. Error Handling

### 9.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_TIMESTAMP",
    "message": "Timestamp must be in ISO8601 format",
    "details": {
      "field": "timestamp",
      "value": "invalid-date"
    }
  }
}
```

### 9.2 HTTP Status Codes

| Code | Meaning | Use Case |
|------|---------|----------|
| 200 | OK | Successful query |
| 201 | Created | Data written |
| 400 | Bad Request | Invalid parameters |
| 401 | Unauthorized | Invalid API key |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

---

## 10. Best Practices

### 10.1 Batch Writes
- Write 1,000-10,000 points per request
- Use compression (gzip)
- Implement retry logic with exponential backoff

### 10.2 Query Optimization
- Always specify time ranges
- Use tags for filtering (indexed)
- Limit result sets
- Cache frequently accessed data

### 10.3 Rate Limiting
- Implement client-side rate limiting
- Use retry-after header
- Distribute load across time

---

**License:** MIT
**Copyright:** © 2025 WIA Standards
**弘益人間 (Benefit All Humanity)**
