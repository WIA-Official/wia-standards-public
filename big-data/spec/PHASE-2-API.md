# WIA-DATA-001: Phase 2 - API Interface Specification

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

Phase 2 defines standardized RESTful APIs, client SDKs, and query languages for big data systems.

## 2. RESTful API

### 2.1 Base URL Structure

```
https://api.wia-data.com/v1/
```

### 2.2 Core Endpoints

#### 2.2.1 Datasets

```http
GET    /api/v1/datasets                 # List all datasets
POST   /api/v1/datasets                 # Create new dataset
GET    /api/v1/datasets/{id}            # Get dataset metadata
PUT    /api/v1/datasets/{id}            # Update dataset
DELETE /api/v1/datasets/{id}            # Delete dataset
GET    /api/v1/datasets/{id}/data       # Get dataset data
POST   /api/v1/datasets/{id}/data       # Insert data
POST   /api/v1/datasets/{id}/query      # Query dataset
```

### 2.3 Request/Response Format

**Create Dataset Request:**
```json
{
  "name": "user_events",
  "schema": {...},
  "partitioning": {
    "type": "time",
    "field": "timestamp",
    "granularity": "day"
  },
  "compression": "snappy"
}
```

**Create Dataset Response:**
```json
{
  "id": "ds-12345",
  "name": "user_events",
  "created_at": "2025-01-01T00:00:00Z",
  "status": "active",
  "row_count": 0,
  "byte_size": 0
}
```

## 3. Query Language

### 3.1 WIA Query Syntax

```json
{
  "select": ["user_id", "COUNT(*) as event_count"],
  "where": {
    "timestamp": {
      "gte": "2025-01-01",
      "lt": "2025-02-01"
    },
    "event_type": "click"
  },
  "groupBy": ["user_id"],
  "orderBy": ["event_count DESC"],
  "limit": 100
}
```

### 3.2 Filter Operations

| Operation | Syntax | Example |
|-----------|--------|---------|
| Equality | `{"field": value}` | `{"status": "active"}` |
| Greater Than | `{"field": {"gt": value}}` | `{"age": {"gt": 18}}` |
| Less Than | `{"field": {"lt": value}}` | `{"price": {"lt": 100}}` |
| In List | `{"field": {"in": [...]}}` | `{"status": {"in": ["active", "pending"]}}` |
| Not Null | `{"field": {"notNull": true}}` | `{"email": {"notNull": true}}` |
| Like | `{"field": {"like": "pattern"}}` | `{"name": {"like": "%john%"}}` |

### 3.3 Aggregation Functions

- `COUNT(*)` - Count rows
- `SUM(field)` - Sum values
- `AVG(field)` - Average values
- `MIN(field)` - Minimum value
- `MAX(field)` - Maximum value
- `DISTINCT(field)` - Unique values

## 4. Authentication and Authorization

### 4.1 API Key Authentication

```http
Authorization: Bearer YOUR_API_KEY
```

### 4.2 OAuth 2.0

Supported flows:
- Authorization code
- Client credentials
- Refresh token

### 4.3 Permissions Model

```json
{
  "permissions": {
    "datasets:read": true,
    "datasets:write": true,
    "datasets:delete": false,
    "schemas:manage": false
  }
}
```

## 5. Client SDKs

### 5.1 Python SDK

```python
from wia import WIAClient

client = WIAClient(api_key="your-key")

# Create dataset
dataset = client.create_dataset(
    name="events",
    schema=schema
)

# Insert data
dataset.insert([
    {"id": "1", "value": 42},
    {"id": "2", "value": 38}
])

# Query data
results = dataset.query()
    .select("id", "value")
    .where(value__gt=40)
    .execute()
```

### 5.2 Java SDK

```java
WIAClient client = new WIAClient("your-key");

Dataset dataset = client.createDataset(
    DatasetConfig.builder()
        .name("events")
        .schema(schema)
        .build()
);

dataset.insert(records);

QueryResult results = dataset.query()
    .select("id", "value")
    .where(Where.gt("value", 40))
    .execute();
```

## 6. Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640995200
```

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_SCHEMA",
    "message": "Schema validation failed",
    "details": {
      "field": "user_id",
      "reason": "Required field missing"
    }
  }
}
```

### 7.2 HTTP Status Codes

| Code | Status | Meaning |
|------|--------|---------|
| 200 | OK | Request successful |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid request |
| 401 | Unauthorized | Authentication required |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

---

**© 2025 WIA - World Certification Industry Association**
*弘益人間 · Benefit All Humanity*
