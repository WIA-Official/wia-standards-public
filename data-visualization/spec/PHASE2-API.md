# WIA-DATA-011: PHASE 2 - API Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26

## Overview

This specification defines the standard APIs for data visualization services in the WIA-DATA-011 standard. It covers REST APIs, GraphQL endpoints, and WebSocket protocols for real-time data visualization.

## Table of Contents

1. [REST API Specification](#rest-api-specification)
2. [GraphQL API](#graphql-api)
3. [Authentication & Authorization](#authentication--authorization)
4. [Rate Limiting](#rate-limiting)
5. [Error Handling](#error-handling)
6. [Versioning](#versioning)

---

## 1. REST API Specification

### 1.1 Base URL

All API endpoints are relative to the base URL:

```
https://api.example.com/v1/viz
```

### 1.2 Core Endpoints

#### Get Visualization Data

```
GET /viz/data/{dataset_id}
```

**Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `dataset_id` | string | Yes | Dataset identifier |
| `format` | string | No | Output format (json, csv, parquet) |
| `filter` | string | No | Filter expression |
| `limit` | integer | No | Maximum number of records (default: 1000) |
| `offset` | integer | No | Pagination offset (default: 0) |

**Response:**

```json
{
  "status": "success",
  "data": {
    "metadata": {
      "dataset_id": "sales-2025",
      "total_records": 10000,
      "returned_records": 1000,
      "has_more": true
    },
    "schema": {
      "fields": [...]
    },
    "data": [...]
  }
}
```

#### Create Visualization

```
POST /viz/create
```

**Request Body:**

```json
{
  "type": "line",
  "data_source": "dataset_id",
  "encoding": {
    "x": { "field": "date", "type": "temporal" },
    "y": { "field": "sales", "type": "quantitative" }
  },
  "config": {
    "width": 800,
    "height": 400,
    "theme": "dark"
  }
}
```

**Response:**

```json
{
  "status": "success",
  "viz_id": "viz-12345",
  "url": "https://api.example.com/v1/viz/viz-12345",
  "embed_url": "https://api.example.com/v1/viz/viz-12345/embed",
  "preview_url": "https://api.example.com/v1/viz/viz-12345/preview.png"
}
```

#### Update Visualization

```
PUT /viz/{viz_id}
```

#### Delete Visualization

```
DELETE /viz/{viz_id}
```

#### Get Chart Recommendations

```
POST /viz/recommend
```

**Request Body:**

```json
{
  "data_sample": [...],
  "user_intent": "show trend over time",
  "preferences": {
    "chart_types": ["line", "bar", "area"],
    "max_results": 5
  }
}
```

**Response:**

```json
{
  "recommendations": [
    {
      "type": "line",
      "confidence": 0.92,
      "reason": "Time series data with temporal patterns",
      "config": {...}
    },
    {
      "type": "area",
      "confidence": 0.78,
      "reason": "Shows cumulative trend effectively",
      "config": {...}
    }
  ]
}
```

### 1.3 Data Aggregation Endpoints

```
GET /viz/data/{dataset_id}/aggregate
```

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `group_by` | string | Grouping fields (comma-separated) |
| `metrics` | string | Aggregation metrics (sum, avg, count, etc.) |
| `interval` | string | Time interval (1h, 1d, 1w, 1m) |

**Example:**

```
GET /viz/data/sales-2025/aggregate?group_by=region&metrics=sum(revenue),avg(revenue)&interval=1d
```

### 1.4 Export Endpoints

```
GET /viz/{viz_id}/export
```

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `format` | string | Export format (png, svg, pdf, json) |
| `width` | integer | Image width in pixels |
| `height` | integer | Image height in pixels |
| `dpi` | integer | DPI for raster formats |

---

## 2. GraphQL API

### 2.1 Schema

```graphql
type Query {
  dataset(id: ID!): Dataset
  datasets(filter: DatasetFilter, limit: Int, offset: Int): [Dataset!]!
  visualization(id: ID!): Visualization
  visualizations(filter: VizFilter): [Visualization!]!
  recommendations(input: RecommendationInput!): [ChartRecommendation!]!
}

type Mutation {
  createVisualization(input: CreateVizInput!): Visualization!
  updateVisualization(id: ID!, input: UpdateVizInput!): Visualization!
  deleteVisualization(id: ID!): Boolean!
  uploadDataset(file: Upload!): Dataset!
}

type Subscription {
  dataUpdated(datasetId: ID!): Dataset!
  vizUpdated(vizId: ID!): Visualization!
}

type Dataset {
  id: ID!
  name: String!
  description: String
  schema: Schema!
  data(limit: Int, offset: Int): [DataPoint!]!
  metadata: Metadata!
  createdAt: DateTime!
  updatedAt: DateTime!
}

type Visualization {
  id: ID!
  type: ChartType!
  dataset: Dataset!
  encoding: Encoding!
  config: Config!
  embedUrl: String!
  previewUrl: String!
}

type ChartRecommendation {
  type: ChartType!
  confidence: Float!
  reason: String!
  config: Config!
}

enum ChartType {
  LINE
  BAR
  SCATTER
  PIE
  AREA
  HEATMAP
  TREEMAP
  NETWORK
}
```

### 2.2 Example Queries

#### Get Dataset

```graphql
query GetDataset($id: ID!) {
  dataset(id: $id) {
    id
    name
    schema {
      fields {
        name
        type
        description
      }
    }
    data(limit: 100) {
      ...dataFields
    }
  }
}
```

#### Create Visualization

```graphql
mutation CreateViz($input: CreateVizInput!) {
  createVisualization(input: $input) {
    id
    embedUrl
    previewUrl
  }
}
```

#### Subscribe to Data Updates

```graphql
subscription DataUpdates($datasetId: ID!) {
  dataUpdated(datasetId: $datasetId) {
    id
    updatedAt
    data(limit: 10) {
      ...dataFields
    }
  }
}
```

---

## 3. Authentication & Authorization

### 3.1 API Key Authentication

```
Authorization: Bearer YOUR_API_KEY
```

### 3.2 OAuth 2.0

Supported flows:
- Authorization Code
- Client Credentials
- Resource Owner Password

**Token Endpoint:**

```
POST /oauth/token
```

**Request:**

```json
{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "viz:read viz:write"
}
```

### 3.3 Scopes

| Scope | Description |
|-------|-------------|
| `viz:read` | Read visualizations and data |
| `viz:write` | Create and update visualizations |
| `viz:delete` | Delete visualizations |
| `data:read` | Read datasets |
| `data:write` | Upload and modify datasets |
| `admin` | Full administrative access |

---

## 4. Rate Limiting

### 4.1 Limits

| Tier | Requests/Minute | Requests/Day |
|------|----------------|--------------|
| Free | 60 | 10,000 |
| Pro | 600 | 100,000 |
| Enterprise | Custom | Custom |

### 4.2 Headers

Response headers include rate limit information:

```
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1735210200
```

### 4.3 Rate Limit Exceeded

```json
{
  "status": "error",
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "API rate limit exceeded",
    "retry_after": 60
  }
}
```

---

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {
      "field": "field_name",
      "constraint": "validation_rule"
    },
    "documentation_url": "https://docs.example.com/errors/ERROR_CODE"
  }
}
```

### 5.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Malformed request |
| `UNAUTHORIZED` | 401 | Missing or invalid authentication |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `VALIDATION_ERROR` | 422 | Input validation failed |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |
| `SERVICE_UNAVAILABLE` | 503 | Service temporarily unavailable |

---

## 6. Versioning

### 6.1 URL Versioning

```
https://api.example.com/v1/viz/...
https://api.example.com/v2/viz/...
```

### 6.2 Header Versioning

```
Accept: application/vnd.wia.viz.v1+json
```

### 6.3 Deprecation

Deprecated endpoints include a warning header:

```
Deprecation: true
Sunset: Sat, 31 Dec 2025 23:59:59 GMT
Link: <https://docs.example.com/migration>; rel="deprecation"
```

---

## Appendix A: HTTP Status Codes

| Status | Description |
|--------|-------------|
| 200 | Success |
| 201 | Created |
| 204 | No Content |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 422 | Unprocessable Entity |
| 429 | Too Many Requests |
| 500 | Internal Server Error |
| 503 | Service Unavailable |

---

**© 2025 WIA (World Certification Industry Association)**
**Standard:** WIA-DATA-011
**Phase:** 2 - API Specification
