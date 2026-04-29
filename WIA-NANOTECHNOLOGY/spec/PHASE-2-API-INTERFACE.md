# WIA-NANOTECHNOLOGY: PHASE 2 - API INTERFACE SPECIFICATION

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2026-01-12
**Philosophy**: 弘益人間 (Benefit All Humanity)

## 1. Overview

This specification defines RESTful API interfaces for nanomaterial databases, characterization data management, synthesis planning, and simulation services in the WIA-NANOTECHNOLOGY ecosystem.

## 2. Base URL and Versioning

```
Base URL: https://api.wia-nanotechnology.org/v1
API Version: 1.0
Authentication: Bearer token (JWT)
Content-Type: application/json
```

## 3. Material Database API

### 3.1 Create Material

```http
POST /materials
Authorization: Bearer {token}
Content-Type: application/json

Request Body:
{
  "name": "string",
  "type": "CNT | graphene | quantum_dot | nanoparticle | nanocomposite | nanofilm",
  "composition": {
    "elements": [...],
    "formula": "string"
  },
  "dimensions": {...},
  "properties": {...}
}

Response: 201 Created
{
  "materialId": "uuid",
  "name": "string",
  "createdAt": "ISO 8601 timestamp",
  "status": "active"
}
```

### 3.2 Get Material

```http
GET /materials/{materialId}
Authorization: Bearer {token}

Response: 200 OK
{
  // Full material object as per PHASE-1 format
}

Errors:
404 Not Found - Material does not exist
403 Forbidden - Insufficient permissions
```

### 3.3 Search Materials

```http
GET /materials/search
Authorization: Bearer {token}

Query Parameters:
- type: string (material type filter)
- elements: string (comma-separated element symbols)
- minSize: number (minimum dimension in nm)
- maxSize: number (maximum dimension in nm)
- properties: string (property filters in JSON format)
- page: number (default: 1)
- limit: number (default: 20, max: 100)
- sort: string (field to sort by)
- order: asc | desc

Response: 200 OK
{
  "materials": [
    {
      "materialId": "uuid",
      "name": "string",
      "type": "string",
      "composition": {...},
      "dimensions": {...},
      "properties": {...}
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 150,
    "pages": 8
  }
}
```

### 3.4 Update Material

```http
PATCH /materials/{materialId}
Authorization: Bearer {token}
Content-Type: application/json

Request Body:
{
  "properties": {
    "physical": {
      "surfaceArea": 1500
    }
  }
}

Response: 200 OK
{
  "materialId": "uuid",
  "updatedAt": "ISO 8601 timestamp",
  "changes": ["properties.physical.surfaceArea"]
}
```

### 3.5 Delete Material

```http
DELETE /materials/{materialId}
Authorization: Bearer {token}

Response: 204 No Content

Errors:
404 Not Found
409 Conflict - Material has associated data
```

## 4. Characterization Data API

### 4.1 Upload Characterization Results

```http
POST /characterization
Authorization: Bearer {token}
Content-Type: multipart/form-data

Form Data:
- materialId: string
- technique: string
- metadata: JSON (instrument, conditions, etc.)
- dataFiles: File[] (images, spectra, raw data)

Response: 201 Created
{
  "characterizationId": "uuid",
  "materialId": "uuid",
  "technique": "string",
  "uploadedAt": "ISO 8601 timestamp",
  "fileCount": 5,
  "status": "processing"
}
```

### 4.2 Get Characterization Results

```http
GET /characterization/{characterizationId}
Authorization: Bearer {token}

Response: 200 OK
{
  "characterizationId": "uuid",
  "materialId": "uuid",
  "technique": "TEM",
  "timestamp": "ISO 8601 timestamp",
  "results": {
    // Technique-specific results per PHASE-1
  },
  "files": [
    {
      "fileId": "uuid",
      "type": "image | spectrum | raw_data",
      "url": "string",
      "size": "number (bytes)"
    }
  ]
}
```

### 4.3 List Characterizations for Material

```http
GET /materials/{materialId}/characterizations
Authorization: Bearer {token}

Query Parameters:
- technique: string (filter by technique)
- dateFrom: ISO 8601 timestamp
- dateTo: ISO 8601 timestamp
- page: number
- limit: number

Response: 200 OK
{
  "characterizations": [
    {
      "characterizationId": "uuid",
      "technique": "string",
      "timestamp": "ISO 8601 timestamp",
      "operator": "string",
      "summary": {...}
    }
  ],
  "pagination": {...}
}
```

### 4.4 Analyze Characterization Data

```http
POST /characterization/{characterizationId}/analyze
Authorization: Bearer {token}
Content-Type: application/json

Request Body:
{
  "analysisType": "particle_size | phase_identification | roughness | composition",
  "parameters": {
    "threshold": 0.5,
    "algorithm": "otsu | watershed | ML"
  }
}

Response: 200 OK
{
  "analysisId": "uuid",
  "results": {
    "particleSizeDistribution": {...},
    "statistics": {...}
  },
  "processingTime": "number (ms)"
}
```

## 5. Synthesis Planning API

### 5.1 Create Synthesis Plan

```http
POST /synthesis/plan
Authorization: Bearer {token}
Content-Type: application/json

Request Body:
{
  "targetMaterialId": "uuid",
  "method": "CVD | sol_gel | hydrothermal | ...",
  "constraints": {
    "maxTemperature": 1200,
    "availablePrecursors": ["string"],
    "targetYield": 0.8
  }
}

Response: 201 Created
{
  "planId": "uuid",
  "method": "string",
  "estimatedYield": 0.85,
  "estimatedCost": {
    "value": 250,
    "currency": "USD"
  },
  "steps": [
    {
      "stepNumber": 1,
      "description": "string",
      "duration": "number (seconds)",
      "parameters": {...}
    }
  ],
  "warnings": ["string"],
  "recommendations": ["string"]
}
```

### 5.2 Record Synthesis Execution

```http
POST /synthesis/execute
Authorization: Bearer {token}
Content-Type: application/json

Request Body:
{
  "planId": "uuid (optional)",
  "materialId": "uuid",
  "method": "string",
  "precursors": [...],
  "processSteps": [...],
  "yield": {...},
  "quality": {...}
}

Response: 201 Created
{
  "synthesisId": "uuid",
  "materialId": "uuid",
  "timestamp": "ISO 8601 timestamp",
  "status": "completed"
}
```

### 5.3 Get Synthesis History

```http
GET /materials/{materialId}/synthesis
Authorization: Bearer {token}

Response: 200 OK
{
  "syntheses": [
    {
      "synthesisId": "uuid",
      "method": "string",
      "timestamp": "ISO 8601 timestamp",
      "yield": {...},
      "quality": {...}
    }
  ]
}
```

## 6. Simulation API

### 6.1 Submit Simulation Job

```http
POST /simulation
Authorization: Bearer {token}
Content-Type: application/json

Request Body:
{
  "type": "molecular_dynamics | DFT | finite_element | monte_carlo",
  "materialId": "uuid (optional)",
  "system": {
    "numberOfAtoms": 10000,
    "dimensions": {...}
  },
  "parameters": {
    "timeStep": 1.0,
    "totalTime": 1000,
    "temperature": 300
  },
  "computeResources": {
    "cores": 16,
    "memory": "32GB",
    "wallTime": "24h"
  }
}

Response: 202 Accepted
{
  "simulationId": "uuid",
  "status": "queued",
  "estimatedStartTime": "ISO 8601 timestamp",
  "estimatedDuration": "number (seconds)",
  "queuePosition": 3
}
```

### 6.2 Get Simulation Status

```http
GET /simulation/{simulationId}
Authorization: Bearer {token}

Response: 200 OK
{
  "simulationId": "uuid",
  "status": "queued | running | completed | failed",
  "progress": 0.65,
  "startedAt": "ISO 8601 timestamp",
  "estimatedCompletion": "ISO 8601 timestamp",
  "results": {...} // Only present if completed
}
```

### 6.3 Get Simulation Results

```http
GET /simulation/{simulationId}/results
Authorization: Bearer {token}

Response: 200 OK
{
  "simulationId": "uuid",
  "type": "string",
  "completedAt": "ISO 8601 timestamp",
  "results": {
    "energy": {...},
    "trajectory": {
      "url": "string (download link)",
      "format": "xyz | pdb | dcd",
      "size": "number (bytes)"
    },
    "properties": {...}
  },
  "computeStats": {
    "coreHours": 384,
    "peakMemory": "28GB",
    "wallTime": "number (seconds)"
  }
}
```

### 6.4 Cancel Simulation

```http
DELETE /simulation/{simulationId}
Authorization: Bearer {token}

Response: 200 OK
{
  "simulationId": "uuid",
  "status": "cancelled",
  "cancelledAt": "ISO 8601 timestamp"
}
```

## 7. Property Prediction API

### 7.1 Predict Material Properties

```http
POST /properties/predict
Authorization: Bearer {token}
Content-Type: application/json

Request Body:
{
  "materialId": "uuid",
  "properties": ["bandgap", "conductivity", "thermalConductivity"],
  "method": "ML | DFT | empirical"
}

Response: 200 OK
{
  "predictionId": "uuid",
  "materialId": "uuid",
  "predictions": {
    "bandgap": {
      "value": 1.42,
      "unit": "eV",
      "confidence": 0.92,
      "method": "ML"
    },
    "conductivity": {
      "value": 1.5e6,
      "unit": "S/m",
      "confidence": 0.88,
      "method": "ML"
    }
  },
  "timestamp": "ISO 8601 timestamp"
}
```

## 8. Safety and Compliance API

### 8.1 Get Safety Information

```http
GET /materials/{materialId}/safety
Authorization: Bearer {token}

Response: 200 OK
{
  "materialId": "uuid",
  "hazardClassification": {...},
  "exposureLimits": {...},
  "toxicology": {...},
  "handling": {...},
  "sds": {
    "url": "string (Safety Data Sheet)",
    "lastUpdated": "ISO 8601 timestamp"
  }
}
```

### 8.2 Check Regulatory Compliance

```http
POST /compliance/check
Authorization: Bearer {token}
Content-Type: application/json

Request Body:
{
  "materialId": "uuid",
  "jurisdiction": "US | EU | ISO",
  "regulations": ["REACH", "TSCA", "ISO/TS 80004"]
}

Response: 200 OK
{
  "materialId": "uuid",
  "compliant": true,
  "details": [
    {
      "regulation": "REACH",
      "status": "compliant",
      "registrationNumber": "string",
      "expiresAt": "ISO 8601 timestamp"
    }
  ],
  "warnings": ["string"],
  "requiredActions": ["string"]
}
```

## 9. Batch Operations API

### 9.1 Batch Create Materials

```http
POST /materials/batch
Authorization: Bearer {token}
Content-Type: application/json

Request Body:
{
  "materials": [
    {
      "name": "string",
      "type": "string",
      // ... material definition
    }
  ]
}

Response: 201 Created
{
  "batchId": "uuid",
  "created": 45,
  "failed": 2,
  "results": [
    {
      "index": 0,
      "status": "success",
      "materialId": "uuid"
    },
    {
      "index": 1,
      "status": "error",
      "error": "Duplicate material name"
    }
  ]
}
```

## 10. Webhooks

### 10.1 Register Webhook

```http
POST /webhooks
Authorization: Bearer {token}
Content-Type: application/json

Request Body:
{
  "url": "https://your-server.com/webhook",
  "events": ["simulation.completed", "characterization.uploaded"],
  "secret": "string (for signature verification)"
}

Response: 201 Created
{
  "webhookId": "uuid",
  "url": "string",
  "events": ["string"],
  "active": true
}
```

### 10.2 Webhook Payload Example

```json
{
  "eventId": "uuid",
  "event": "simulation.completed",
  "timestamp": "ISO 8601 timestamp",
  "data": {
    "simulationId": "uuid",
    "status": "completed",
    "results": {...}
  },
  "signature": "HMAC-SHA256 signature"
}
```

## 11. Rate Limiting

- Anonymous: 100 requests/hour
- Authenticated: 1000 requests/hour
- Premium: 10000 requests/hour
- Batch operations count as 1 request per item

## 12. Error Responses

```json
{
  "error": {
    "code": "string (ERROR_CODE)",
    "message": "string (human-readable)",
    "details": {...},
    "timestamp": "ISO 8601 timestamp",
    "requestId": "uuid"
  }
}
```

Common error codes:
- `INVALID_REQUEST` (400)
- `UNAUTHORIZED` (401)
- `FORBIDDEN` (403)
- `NOT_FOUND` (404)
- `CONFLICT` (409)
- `RATE_LIMITED` (429)
- `INTERNAL_ERROR` (500)

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
