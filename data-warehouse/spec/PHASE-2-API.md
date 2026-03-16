# WIA Data Warehouse API Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #10B981 (Emerald - DATA domain)

---

## Table of Contents

1. [Overview](#overview)
2. [API Architecture](#api-architecture)
3. [Authentication](#authentication)
4. [Endpoints](#endpoints)
5. [Request/Response Formats](#requestresponse-formats)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)
8. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Data Warehouse API Standard defines RESTful APIs for interacting with data warehouses, enabling programmatic access to dimensional data, metadata, and analytics functions.

**Core Capabilities**:
- Query fact and dimension tables
- Execute OLAP operations (slice, dice, drill-down, roll-up)
- Manage ETL pipelines
- Access metadata and data lineage
- Monitor warehouse performance

### 1.2 API Design Principles

1. **RESTful**: Follow REST architectural constraints
2. **Stateless**: Each request contains all necessary information
3. **Versioned**: Support multiple API versions
4. **Secure**: HTTPS, authentication, authorization
5. **Performant**: Pagination, caching, compression

---

## API Architecture

### 2.1 Base URL

```
https://api.datawarehouse.{organization}.com/v1
```

### 2.2 API Versioning

Version is specified in the URL path:
- `/v1/` - Version 1 (current)
- `/v2/` - Version 2 (future)

### 2.3 Content Type

All requests and responses use JSON:
```
Content-Type: application/json
Accept: application/json
```

---

## Authentication

### 3.1 API Key Authentication

Include API key in request header:
```http
Authorization: Bearer {api_key}
```

### 3.2 OAuth 2.0

For user-specific operations:
```http
Authorization: Bearer {oauth_token}
```

### 3.3 Service Account

For service-to-service:
```http
Authorization: ServiceAccount {service_account_key}
```

---

## Endpoints

### 4.1 Query Endpoints

#### Execute Query
```http
POST /v1/query
Content-Type: application/json

{
  "sql": "SELECT * FROM fact_sales WHERE date_key = 20250115 LIMIT 100",
  "format": "json|csv|parquet",
  "use_cache": true
}
```

**Response**:
```json
{
  "query_id": "q-12345-abc",
  "status": "completed",
  "rows_returned": 100,
  "execution_time_ms": 234,
  "data": [
    {
      "fact_sales_key": 1,
      "date_key": 20250115,
      "sales_amount": 1250.00
    }
  ],
  "metadata": {
    "columns": [
      {"name": "fact_sales_key", "type": "bigint"},
      {"name": "date_key", "type": "int"},
      {"name": "sales_amount", "type": "decimal"}
    ]
  }
}
```

#### Get Query Status
```http
GET /v1/query/{query_id}/status
```

### 4.2 Dimension Endpoints

#### Get Dimension Data
```http
GET /v1/dimensions/{dimension_name}?limit=100&offset=0
```

#### Get Dimension by Natural Key
```http
GET /v1/dimensions/{dimension_name}/{natural_key}
```

#### Create Dimension Record (SCD Type 1)
```http
POST /v1/dimensions/{dimension_name}

{
  "customer_id": "CUST-12345",
  "customer_name": "John Doe",
  "email": "john@example.com",
  "city": "Seoul"
}
```

#### Update Dimension Record (SCD Type 2)
```http
PUT /v1/dimensions/{dimension_name}/{natural_key}

{
  "email": "newemail@example.com",
  "scd_type": 2
}
```

### 4.3 Fact Endpoints

#### Insert Fact Records
```http
POST /v1/facts/{fact_table_name}

{
  "records": [
    {
      "date_key": 20250115,
      "product_key": 1001,
      "store_key": 5,
      "customer_key": 10234,
      "sales_amount": 1250.00,
      "quantity_sold": 3
    }
  ]
}
```

#### Get Fact Aggregations
```http
POST /v1/facts/{fact_table_name}/aggregate

{
  "measures": ["sum(sales_amount)", "count(*)"],
  "dimensions": ["date_key", "product_key"],
  "filters": {
    "date_key": {"gte": 20250101, "lte": 20250131},
    "product_key": {"in": [1001, 1002, 1003]}
  },
  "group_by": ["date_key", "product_key"],
  "order_by": ["sum(sales_amount) DESC"]
}
```

### 4.4 Metadata Endpoints

#### Get Schema Information
```http
GET /v1/metadata/schemas
GET /v1/metadata/schemas/{schema_name}/tables
GET /v1/metadata/tables/{table_name}
```

#### Get Data Lineage
```http
GET /v1/metadata/lineage/{table_name}
```

**Response**:
```json
{
  "table_name": "fact_sales",
  "upstream": [
    {
      "source_system": "ERP",
      "source_table": "orders",
      "transformation": "ETL-001"
    }
  ],
  "downstream": [
    {
      "target": "sales_summary_mv",
      "type": "materialized_view"
    }
  ]
}
```

### 4.5 ETL Endpoints

#### Trigger ETL Job
```http
POST /v1/etl/jobs/{job_name}/run

{
  "parameters": {
    "start_date": "2025-01-01",
    "end_date": "2025-01-31"
  }
}
```

#### Get ETL Job Status
```http
GET /v1/etl/jobs/{job_id}/status
```

---

## Request/Response Formats

### 5.1 Pagination

```http
GET /v1/dimensions/dim_customer?limit=100&offset=200
```

**Response**:
```json
{
  "data": [...],
  "pagination": {
    "total_rows": 10000,
    "limit": 100,
    "offset": 200,
    "has_more": true
  }
}
```

### 5.2 Filtering

```json
{
  "filters": {
    "field_name": {"operator": "value"},
    "city": {"eq": "Seoul"},
    "sales_amount": {"gte": 1000, "lte": 5000},
    "product_category": {"in": ["Electronics", "Computers"]}
  }
}
```

**Supported Operators**:
- `eq`: Equal
- `ne`: Not equal
- `gt`: Greater than
- `gte`: Greater than or equal
- `lt`: Less than
- `lte`: Less than or equal
- `in`: In list
- `like`: Pattern match

---

## Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_QUERY",
    "message": "Syntax error in SQL query",
    "details": "Line 1: Unexpected token 'FORM'",
    "request_id": "req-abc123"
  }
}
```

### 6.2 HTTP Status Codes

| Code | Meaning | Use Case |
|------|---------|----------|
| 200 | OK | Successful request |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid input |
| 401 | Unauthorized | Missing/invalid auth |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Maintenance mode |

---

## Rate Limiting

### 7.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 997
X-RateLimit-Reset: 1640995200
```

### 7.2 Default Limits

- **Standard tier**: 1000 requests/hour
- **Premium tier**: 10000 requests/hour
- **Enterprise tier**: Unlimited

---

## Examples

### 8.1 Complete Query Example

```bash
curl -X POST https://api.datawarehouse.example.com/v1/query \
  -H "Authorization: Bearer api_key_12345" \
  -H "Content-Type: application/json" \
  -d '{
    "sql": "SELECT d.year, d.month, p.category, SUM(f.sales_amount) as total_sales FROM fact_sales f JOIN dim_date d ON f.date_key = d.date_key JOIN dim_product p ON f.product_key = p.product_key WHERE d.year = 2025 GROUP BY d.year, d.month, p.category ORDER BY total_sales DESC LIMIT 10"
  }'
```

### 8.2 Dimension Update (SCD Type 2)

```bash
curl -X PUT https://api.datawarehouse.example.com/v1/dimensions/dim_customer/CUST-12345 \
  -H "Authorization: Bearer api_key_12345" \
  -H "Content-Type: application/json" \
  -d '{
    "email": "newemail@example.com",
    "city": "Busan",
    "scd_type": 2
  }'
```

---

**License**: MIT
**Copyright**: © 2025 WIA Standards Committee
**Philosophy**: 弘益人間 (Benefit All Humanity)
