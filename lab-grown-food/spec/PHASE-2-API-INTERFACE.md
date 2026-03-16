# WIA-AGRI-019: Lab-Grown Food
## Phase 2: API Interface Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## 1. Introduction

This specification defines the RESTful API interfaces for cellular agriculture and lab-grown food production systems. It enables standardized communication between bioreactors, laboratory information management systems (LIMS), quality control systems, and production management platforms.

### 1.1 Scope

This API covers:
- Cell line management and tracking
- Culture batch operations (CRUD)
- Real-time bioreactor monitoring and control
- Quality assurance workflows
- Production analytics and reporting
- Regulatory compliance documentation

### 1.2 Base URL

All API endpoints are relative to the base URL:
```
https://api.wia.org/v1/agri-019
```

For development and testing:
```
https://sandbox.wia.org/v1/agri-019
```

---

## 2. Authentication

### 2.1 API Key Authentication

All requests MUST include an API key in the header:

```http
GET /cultures HTTP/1.1
Host: api.wia.org
X-API-Key: your-api-key-here
Content-Type: application/json
```

### 2.2 OAuth 2.0 (Recommended for Production)

For enhanced security, use OAuth 2.0 with JWT tokens:

```http
GET /cultures HTTP/1.1
Host: api.wia.org
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json
```

**Token Endpoint:**
```
POST https://auth.wia.org/oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your-client-id
&client_secret=your-client-secret
&scope=agri-019:read agri-019:write
```

### 2.3 Rate Limiting

- Standard tier: 1000 requests/hour
- Premium tier: 10000 requests/hour
- Enterprise tier: Unlimited

Rate limit headers included in all responses:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 856
X-RateLimit-Reset: 1642345678
```

---

## 3. Cell Line Management

### 3.1 List Cell Lines

```http
GET /cell-lines
```

**Query Parameters:**
- `species` (optional): Filter by species (bovine, porcine, chicken, fish)
- `cellType` (optional): Filter by cell type
- `page` (optional): Page number (default: 1)
- `limit` (optional): Results per page (default: 50, max: 100)

**Response:** 200 OK
```json
{
  "data": [
    {
      "id": "BOVINE-SAT-V2",
      "name": "Bovine Muscle Satellite Cells v2",
      "species": "Bos taurus",
      "tissueType": "Skeletal muscle",
      "cellType": "satellite-cell",
      "passage": 12,
      "immortalized": false,
      "validation": {
        "mycoplasma": "negative",
        "lastTested": "2025-01-10T00:00:00Z"
      }
    }
  ],
  "pagination": {
    "total": 45,
    "page": 1,
    "pages": 1,
    "limit": 50
  }
}
```

### 3.2 Get Cell Line Details

```http
GET /cell-lines/{id}
```

**Response:** 200 OK
```json
{
  "id": "BOVINE-SAT-V2",
  "name": "Bovine Muscle Satellite Cells v2",
  "species": "Bos taurus",
  "tissueType": "Skeletal muscle",
  "cellType": "satellite-cell",
  "source": {
    "type": "biopsy",
    "origin": "Netherlands",
    "biopsyDate": "2024-06-15T00:00:00Z"
  },
  "characteristics": {
    "morphology": "Spindle-shaped adherent cells",
    "growthRate": 1.2,
    "doublingTime": 24,
    "maxPassage": 25,
    "immortalized": false
  },
  "storage": {
    "location": "Cryobank A, Shelf 3, Box 12",
    "cryopreservationDate": "2024-07-01T00:00:00Z",
    "vialCount": 15,
    "passage": 12
  },
  "validation": {
    "karyotype": "Normal diploid (2n=60)",
    "mycoplasma": "negative",
    "crossContamination": false,
    "lastTested": "2025-01-10T00:00:00Z"
  },
  "genealogy": {
    "parent": "BOVINE-SAT-V1",
    "children": [],
    "derivationDate": "2024-08-01T00:00:00Z"
  }
}
```

### 3.3 Create Cell Line

```http
POST /cell-lines
Content-Type: application/json
```

**Request Body:**
```json
{
  "name": "Porcine Adipocyte Line A1",
  "species": "Sus scrofa",
  "tissueType": "Adipose tissue",
  "cellType": "adipocyte",
  "source": {
    "type": "biopsy",
    "origin": "Denmark"
  },
  "characteristics": {
    "immortalized": false,
    "doublingTime": 36
  }
}
```

**Response:** 201 Created
```json
{
  "id": "PORCINE-ADIP-A1",
  "name": "Porcine Adipocyte Line A1",
  "createdAt": "2025-01-15T10:30:00Z"
}
```

### 3.4 Update Cell Line

```http
PATCH /cell-lines/{id}
Content-Type: application/json
```

**Request Body:**
```json
{
  "storage": {
    "passage": 13,
    "vialCount": 12
  },
  "validation": {
    "mycoplasma": "negative",
    "lastTested": "2025-01-15T00:00:00Z"
  }
}
```

**Response:** 200 OK

---

## 4. Culture Batch Operations

### 4.1 Create Culture Batch

```http
POST /cultures
Content-Type: application/json
```

**Request Body:**
```json
{
  "cellLineId": "BOVINE-SAT-V2",
  "bioreactorId": "BR-A1",
  "operator": "Dr. Jane Smith",
  "protocol": {
    "id": "PROTO-BOVINE-STD",
    "version": "2.1"
  },
  "parameters": {
    "pH": 7.2,
    "temperature": 37.0,
    "dissolvedOxygen": 30,
    "glucose": 4.5
  },
  "targets": {
    "cellDensity": 1e7,
    "duration": 14,
    "yieldKg": 5.0
  }
}
```

**Response:** 201 Created
```json
{
  "id": "BATCH-2025-001",
  "cellLineId": "BOVINE-SAT-V2",
  "bioreactorId": "BR-A1",
  "status": "inoculation",
  "phase": "phase-1",
  "startDate": "2025-01-15T08:00:00Z",
  "estimatedCompletion": "2025-01-29T08:00:00Z"
}
```

### 4.2 Get Batch Status

```http
GET /cultures/{batchId}
```

**Response:** 200 OK
```json
{
  "id": "BATCH-2025-001",
  "cellLineId": "BOVINE-SAT-V2",
  "bioreactorId": "BR-A1",
  "status": "proliferation",
  "phase": "phase-2",
  "startDate": "2025-01-15T08:00:00Z",
  "daysElapsed": 5,
  "currentMetrics": {
    "cellDensity": 6.2e6,
    "viability": 94.8,
    "pH": 7.18,
    "temperature": 37.1
  },
  "targets": {
    "cellDensity": 1e7,
    "duration": 14
  },
  "progress": 62
}
```

### 4.3 List Culture Batches

```http
GET /cultures
```

**Query Parameters:**
- `status` (optional): Filter by status
- `bioreactorId` (optional): Filter by bioreactor
- `startDate` (optional): ISO 8601 date
- `endDate` (optional): ISO 8601 date
- `page` (optional): Page number
- `limit` (optional): Results per page

**Response:** 200 OK
```json
{
  "data": [
    {
      "id": "BATCH-2025-001",
      "cellLineId": "BOVINE-SAT-V2",
      "status": "proliferation",
      "startDate": "2025-01-15T08:00:00Z",
      "progress": 62
    }
  ],
  "pagination": {
    "total": 23,
    "page": 1,
    "pages": 1,
    "limit": 50
  }
}
```

### 4.4 Update Batch Status

```http
PATCH /cultures/{batchId}
Content-Type: application/json
```

**Request Body:**
```json
{
  "status": "differentiation",
  "phase": "phase-3",
  "notes": "Switched to differentiation medium"
}
```

**Response:** 200 OK

### 4.5 Terminate Batch

```http
POST /cultures/{batchId}/terminate
Content-Type: application/json
```

**Request Body:**
```json
{
  "reason": "Contamination detected",
  "timestamp": "2025-01-20T14:30:00Z"
}
```

**Response:** 200 OK

---

## 5. Bioreactor Monitoring & Control

### 5.1 Get Real-Time Telemetry

```http
GET /bioreactors/{bioreactorId}/telemetry
```

**Query Parameters:**
- `interval` (optional): Time interval (1min, 5min, 15min, 1hour)
- `limit` (optional): Number of data points (default: 100)

**Response:** 200 OK
```json
{
  "bioreactorId": "BR-A1",
  "batchId": "BATCH-2025-001",
  "lastUpdate": "2025-01-20T14:30:00Z",
  "current": {
    "pH": 7.22,
    "temperature": 37.0,
    "dissolvedOxygen": 28.5,
    "agitation": 80,
    "cellDensity": 8.5e6,
    "viability": 95.2
  },
  "nutrients": {
    "glucose": 2.8,
    "glutamine": 2.1,
    "lactate": 12.5,
    "ammonia": 1.2
  },
  "alarms": []
}
```

### 5.2 Stream Telemetry (WebSocket)

```javascript
const ws = new WebSocket('wss://api.wia.org/v1/agri-019/ws/bioreactors/BR-A1');

ws.onmessage = (event) => {
  const telemetry = JSON.parse(event.data);
  console.log('pH:', telemetry.parameters.pH.value);
  console.log('Cell density:', telemetry.cellMetrics.viableCellDensity.value);
};
```

**Message Format:**
```json
{
  "timestamp": "2025-01-20T14:30:05Z",
  "bioreactorId": "BR-A1",
  "batchId": "BATCH-2025-001",
  "parameters": {
    "pH": {"value": 7.22, "unit": "pH"},
    "temperature": {"value": 37.0, "unit": "celsius"}
  },
  "cellMetrics": {
    "viableCellDensity": {"value": 8.5e6, "unit": "cells/mL"},
    "viability": {"value": 95.2, "unit": "percent"}
  }
}
```

### 5.3 Control Bioreactor Parameters

```http
POST /bioreactors/{bioreactorId}/control
Content-Type: application/json
```

**Request Body:**
```json
{
  "action": "adjust-pH",
  "targetValue": 7.2,
  "method": "CO2-injection"
}
```

**Response:** 200 OK
```json
{
  "success": true,
  "message": "pH adjustment initiated",
  "estimatedTime": 300
}
```

**Available Actions:**
- `adjust-pH` - Modify pH level
- `adjust-temperature` - Modify temperature
- `adjust-DO` - Modify dissolved oxygen
- `adjust-agitation` - Modify stirring speed
- `feed-glucose` - Add glucose
- `feed-glutamine` - Add glutamine
- `media-change` - Partial media replacement
- `emergency-stop` - Emergency shutdown

### 5.4 Get Historical Data

```http
GET /bioreactors/{bioreactorId}/history
```

**Query Parameters:**
- `startDate` (required): ISO 8601 timestamp
- `endDate` (required): ISO 8601 timestamp
- `parameters` (optional): Comma-separated list (pH,temperature,DO)
- `interval` (optional): Data aggregation (raw, 1min, 5min, 1hour)

**Response:** 200 OK
```json
{
  "bioreactorId": "BR-A1",
  "startDate": "2025-01-15T00:00:00Z",
  "endDate": "2025-01-20T00:00:00Z",
  "interval": "1hour",
  "data": [
    {
      "timestamp": "2025-01-15T00:00:00Z",
      "pH": 7.20,
      "temperature": 37.0,
      "dissolvedOxygen": 30.0,
      "cellDensity": 5e5
    },
    {
      "timestamp": "2025-01-15T01:00:00Z",
      "pH": 7.19,
      "temperature": 37.1,
      "dissolvedOxygen": 29.5,
      "cellDensity": 5.2e5
    }
  ]
}
```

---

## 6. Quality Control

### 6.1 Submit Quality Test

```http
POST /quality/tests
Content-Type: application/json
```

**Request Body:**
```json
{
  "batchId": "BATCH-2025-001",
  "testType": "microbiological",
  "laboratory": "QC Lab A",
  "testDate": "2025-01-29T10:00:00Z",
  "results": {
    "totalViableCount": {
      "value": 850,
      "unit": "CFU/g"
    },
    "pathogens": [
      {
        "organism": "E. coli",
        "detected": false,
        "method": "PCR"
      },
      {
        "organism": "Salmonella",
        "detected": false,
        "method": "ELISA"
      }
    ]
  }
}
```

**Response:** 201 Created
```json
{
  "id": "QC-TEST-2025-045",
  "batchId": "BATCH-2025-001",
  "status": "pass",
  "createdAt": "2025-01-29T10:30:00Z"
}
```

### 6.2 Get Quality Test Results

```http
GET /quality/tests/{testId}
```

**Response:** 200 OK
```json
{
  "id": "QC-TEST-2025-045",
  "batchId": "BATCH-2025-001",
  "testType": "microbiological",
  "status": "pass",
  "results": {
    "totalViableCount": {
      "value": 850,
      "unit": "CFU/g",
      "limit": 10000,
      "status": "pass"
    }
  },
  "certifiedBy": "Dr. Emily Chen",
  "certificationDate": "2025-01-29T11:00:00Z"
}
```

### 6.3 Run Automated Quality Check

```http
POST /quality/auto-check
Content-Type: application/json
```

**Request Body:**
```json
{
  "batchId": "BATCH-2025-001",
  "checks": [
    "parameter-compliance",
    "contamination-risk",
    "nutrient-balance",
    "growth-rate"
  ]
}
```

**Response:** 200 OK
```json
{
  "batchId": "BATCH-2025-001",
  "overallStatus": "pass",
  "checks": {
    "parameter-compliance": {
      "status": "pass",
      "score": 98
    },
    "contamination-risk": {
      "status": "pass",
      "risk": "low"
    },
    "nutrient-balance": {
      "status": "warning",
      "message": "Glucose level approaching threshold"
    },
    "growth-rate": {
      "status": "pass",
      "actual": 1.18,
      "expected": 1.20
    }
  },
  "timestamp": "2025-01-20T14:35:00Z"
}
```

---

## 7. Analytics & Reporting

### 7.1 Get Production Statistics

```http
GET /analytics/production
```

**Query Parameters:**
- `startDate` (required)
- `endDate` (required)
- `groupBy` (optional): day, week, month

**Response:** 200 OK
```json
{
  "period": {
    "start": "2025-01-01T00:00:00Z",
    "end": "2025-01-31T00:00:00Z"
  },
  "summary": {
    "totalBatches": 23,
    "successfulBatches": 21,
    "failedBatches": 2,
    "successRate": 91.3,
    "totalYield": 115.5,
    "averageYield": 5.02
  },
  "byWeek": [
    {
      "week": 1,
      "batches": 5,
      "yield": 24.5
    }
  ]
}
```

### 7.2 Get Cost Analysis

```http
GET /analytics/costs/{batchId}
```

**Response:** 200 OK
```json
{
  "batchId": "BATCH-2025-001",
  "totalCost": 1847.50,
  "breakdown": {
    "media": 650.00,
    "growthFactors": 420.00,
    "consumables": 180.00,
    "utilities": 215.00,
    "labor": 382.50
  },
  "costPerKg": 369.50,
  "currency": "USD"
}
```

---

## 8. Error Handling

### 8.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "pH value must be between 6.5 and 7.8",
    "field": "parameters.pH",
    "timestamp": "2025-01-20T14:40:00Z",
    "requestId": "req_abc123xyz"
  }
}
```

### 8.2 HTTP Status Codes

- `200 OK` - Success
- `201 Created` - Resource created
- `400 Bad Request` - Invalid input
- `401 Unauthorized` - Missing/invalid authentication
- `403 Forbidden` - Insufficient permissions
- `404 Not Found` - Resource not found
- `409 Conflict` - Resource conflict
- `429 Too Many Requests` - Rate limit exceeded
- `500 Internal Server Error` - Server error
- `503 Service Unavailable` - Temporary unavailability

### 8.3 Common Error Codes

- `INVALID_PARAMETER` - Invalid request parameter
- `MISSING_FIELD` - Required field missing
- `RESOURCE_NOT_FOUND` - Requested resource doesn't exist
- `DUPLICATE_RESOURCE` - Resource already exists
- `INVALID_CREDENTIALS` - Authentication failed
- `RATE_LIMIT_EXCEEDED` - Too many requests
- `BIOREACTOR_UNAVAILABLE` - Bioreactor offline
- `BATCH_TERMINATED` - Cannot modify terminated batch

---

## 9. Webhooks

### 9.1 Register Webhook

```http
POST /webhooks
Content-Type: application/json
```

**Request Body:**
```json
{
  "url": "https://your-server.com/webhook",
  "events": [
    "batch.created",
    "batch.completed",
    "batch.failed",
    "alarm.critical"
  ],
  "secret": "your-webhook-secret"
}
```

**Response:** 201 Created

### 9.2 Webhook Events

Example webhook payload:
```json
{
  "event": "alarm.critical",
  "timestamp": "2025-01-20T14:45:00Z",
  "data": {
    "bioreactorId": "BR-A1",
    "batchId": "BATCH-2025-001",
    "alarm": {
      "code": "PH_OUT_OF_RANGE",
      "severity": "critical",
      "message": "pH dropped to 6.5"
    }
  }
}
```

---

## 10. SDK Examples

### 10.1 TypeScript

```typescript
import { LabGrownFoodAPI } from '@wia/agri-019';

const client = new LabGrownFoodAPI({
  apiKey: process.env.WIA_API_KEY
});

const batch = await client.cultures.create({
  cellLineId: 'BOVINE-SAT-V2',
  bioreactorId: 'BR-A1',
  parameters: { pH: 7.2, temperature: 37.0 }
});
```

### 10.2 Python

```python
from wia_agri_019 import LabGrownFoodClient

client = LabGrownFoodClient(api_key="your-api-key")

batch = client.cultures.create(
    cell_line_id="BOVINE-SAT-V2",
    bioreactor_id="BR-A1",
    parameters={"pH": 7.2, "temperature": 37.0}
)
```

---

**© 2025 SmileStory Inc. / WIA**
弘익人間 (홍익인간) · Benefit All Humanity
