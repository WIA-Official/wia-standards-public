# WIA-DATA-005: Data Quality - Phase 2 API Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

This document defines the RESTful API specification for the WIA-DATA-005 Data Quality standard. It provides standardized endpoints for data quality profiling, validation, monitoring, and governance.

## Base URL

```
https://api.example.com/wia/data-quality/v1
```

## Authentication

All API requests require authentication using OAuth 2.0 Bearer tokens or API keys.

```http
Authorization: Bearer <access_token>
```

or

```http
X-API-Key: <api_key>
```

## Common Response Format

### Success Response

```json
{
  "status": "success",
  "data": {},
  "metadata": {
    "requestId": "uuid",
    "timestamp": "ISO8601"
  }
}
```

### Error Response

```json
{
  "status": "error",
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {}
  },
  "metadata": {
    "requestId": "uuid",
    "timestamp": "ISO8601"
  }
}
```

## API Endpoints

### 1. Data Profiling

#### Profile Dataset

Generate quality profile for a dataset.

```http
POST /datasets/{datasetId}/profile
```

**Request Body:**
```json
{
  "sampleSize": 10000,
  "columns": ["col1", "col2"],
  "includeDistributions": true,
  "detectPatterns": true
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "profileId": "uuid",
    "status": "completed",
    "profile": {
      // DataQualityProfile object
    }
  }
}
```

#### Get Profile

Retrieve existing profile.

```http
GET /profiles/{profileId}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "profile": {
      // DataQualityProfile object
    }
  }
}
```

### 2. Validation Rules

#### Create Validation Rule

```http
POST /rules
```

**Request Body:**
```json
{
  "ruleName": "Email Format Check",
  "description": "Validates email format",
  "ruleType": "format",
  "severity": "high",
  "target": {
    "dataset": "customers",
    "column": "email"
  },
  "condition": {
    "operator": "regex",
    "pattern": "^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\\.[a-zA-Z]{2,}$"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "ruleId": "uuid",
    "rule": {
      // ValidationRule object
    }
  }
}
```

#### List Validation Rules

```http
GET /rules?dataset={dataset}&active={true|false}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "rules": [
      // Array of ValidationRule objects
    ],
    "pagination": {
      "total": 100,
      "page": 1,
      "pageSize": 20
    }
  }
}
```

#### Update Validation Rule

```http
PUT /rules/{ruleId}
```

#### Delete Validation Rule

```http
DELETE /rules/{ruleId}
```

### 3. Validation Execution

#### Run Validation

Execute validation rules against dataset.

```http
POST /validations
```

**Request Body:**
```json
{
  "datasetId": "customers",
  "ruleIds": ["uuid-1", "uuid-2"],
  "sampleSize": 1000,
  "stopOnFirstFailure": false
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "validationId": "uuid",
    "status": "running",
    "estimatedCompletion": "ISO8601"
  }
}
```

#### Get Validation Results

```http
GET /validations/{validationId}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "validationId": "uuid",
    "status": "completed",
    "results": [
      // Array of ValidationResult objects
    ],
    "summary": {
      "totalRules": 10,
      "passed": 8,
      "failed": 2,
      "passRate": 80.0
    }
  }
}
```

### 4. Quality Metrics

#### Submit Metrics

```http
POST /metrics
```

**Request Body:**
```json
{
  "metrics": [
    {
      "metricName": "completeness",
      "datasetId": "customers",
      "dimension": "completeness",
      "value": 95.5,
      "unit": "percentage",
      "timestamp": "2025-12-26T10:00:00Z"
    }
  ]
}
```

#### Query Metrics

```http
GET /metrics?datasetId={dataset}&dimension={dimension}&from={ISO8601}&to={ISO8601}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "metrics": [
      // Array of QualityMetric objects
    ]
  }
}
```

### 5. Issues Management

#### Create Issue

```http
POST /issues
```

**Request Body:**
```json
{
  "issueType": "invalid",
  "severity": "high",
  "description": "Invalid email format",
  "location": {
    "dataset": "customers",
    "column": "email",
    "row": 12345
  }
}
```

#### List Issues

```http
GET /issues?status={status}&severity={severity}&assignedTo={userId}
```

#### Update Issue

```http
PUT /issues/{issueId}
```

**Request Body:**
```json
{
  "status": "resolved",
  "resolution": "Fixed via data cleansing script",
  "assignedTo": "user-uuid"
}
```

### 6. Data Cleansing

#### Create Cleansing Job

```http
POST /cleansing-jobs
```

**Request Body:**
```json
{
  "datasetId": "customers",
  "operations": [
    {
      "type": "trim",
      "columns": ["name", "email"]
    },
    {
      "type": "standardize",
      "column": "email",
      "transformation": "lowercase"
    }
  ]
}
```

#### Get Cleansing Job Status

```http
GET /cleansing-jobs/{jobId}
```

### 7. Dashboards and Reports

#### Get Quality Dashboard

```http
GET /dashboards/quality?datasetId={dataset}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "overallScore": 92.5,
    "dimensions": {
      "accuracy": 95.0,
      "completeness": 90.0,
      "consistency": 92.0,
      "timeliness": 94.0,
      "validity": 93.0,
      "uniqueness": 91.0
    },
    "topIssues": [
      // Top 5 quality issues
    ]
  }
}
```

#### Generate Quality Report

```http
POST /reports
```

**Request Body:**
```json
{
  "reportType": "quality-scorecard",
  "datasetId": "customers",
  "period": {
    "from": "2025-12-01",
    "to": "2025-12-26"
  },
  "format": "pdf"
}
```

## Webhooks

Subscribe to quality events.

```http
POST /webhooks
```

**Request Body:**
```json
{
  "url": "https://your-app.com/webhooks/quality",
  "events": ["validation.failed", "issue.created", "threshold.exceeded"],
  "secret": "webhook-secret"
}
```

**Webhook Payload:**
```json
{
  "event": "validation.failed",
  "timestamp": "ISO8601",
  "data": {
    "validationId": "uuid",
    "ruleId": "uuid",
    "failedRecords": 150
  },
  "signature": "HMAC-SHA256 signature"
}
```

## Rate Limiting

- 1000 requests per minute per API key
- 10,000 requests per hour per API key
- Rate limit headers included in responses:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640534400
```

## Pagination

All list endpoints support pagination:

```http
GET /rules?page=2&pageSize=50
```

## Error Codes

| Code | Message | Description |
|------|---------|-------------|
| 400 | Bad Request | Invalid request parameters |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource conflict |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Service temporarily unavailable |

## SDK Support

Official SDKs available for:
- Python
- JavaScript/TypeScript
- Java
- Go
- C#

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
