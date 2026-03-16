# WIA-CRYO-010 PHASE 3: PROTOCOLS

**Standard**: WIA-CRYO-010  
**Phase**: 3 - API and Communication Protocols  
**Version**: 1.0.0  
**Date**: January 2025  
**Status**: Active  

## Overview

Phase 3 defines REST API specifications, WebSocket protocols, and data exchange mechanisms for cryopreservation research systems.

## 3.1 RESTful API Specification

### Base URL
```
https://api.wia.org/cryo-research/v1
```

### Authentication
All requests require OAuth 2.0 Bearer token:
```
Authorization: Bearer {access_token}
```

### Endpoints

#### GET /experiments
Query experiments with filters
```http
GET /experiments?cell_type=hepatocyte&viability_min=80&limit=50&offset=0
```

Response (200 OK):
```json
{
  "results": [...],
  "total": 145,
  "page": 1,
  "pageSize": 50,
  "links": {
    "self": "/experiments?page=1",
    "next": "/experiments?page=2"
  }
}
```

#### GET /experiments/{id}
Retrieve specific experiment
```http
GET /experiments/exp-2025-001
```

#### POST /experiments
Submit new experiment
```http
POST /experiments
Content-Type: application/ld+json

{experiment JSON-LD}
```

Response (201 Created):
```json
{
  "status": "created",
  "experimentId": "exp-2025-456",
  "location": "/experiments/exp-2025-456"
}
```

#### PUT /experiments/{id}
Update experiment
```http
PUT /experiments/exp-2025-001
```

#### DELETE /experiments/{id}
Archive experiment (soft delete)

### Error Responses
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Required field 'principalInvestigator' missing",
    "details": {...}
  }
}
```

Status codes:
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 409: Conflict
- 422: Unprocessable Entity
- 500: Internal Server Error

## 3.2 WebSocket Protocol

Real-time data streaming for monitoring.

### Connection
```javascript
ws://api.wia.org/cryo-research/ws/v1
```

### Message Format
```json
{
  "type": "temperature_update",
  "experimentId": "exp-2025-001",
  "timestamp": "2025-01-21T10:15:23.456Z",
  "data": {
    "probe": "T001",
    "temperature": -125.3,
    "unit": "°C"
  }
}
```

### Subscription
```json
{
  "action": "subscribe",
  "channel": "experiment.exp-2025-001.temperature"
}
```

## 3.3 Data Export Formats

### CSV Export
```http
GET /experiments/exp-2025-001/export?format=csv
```

Header row required, UTF-8 encoding.

### Bulk Export
```http
POST /bulk-export
{
  "experimentIds": ["exp-2025-001", "exp-2025-002"],
  "format": "json-ld",
  "compression": "gzip"
}
```

Returns job ID for async processing.

## 3.4 Pagination

Cursor-based pagination for large datasets:
```http
GET /experiments?cursor=eyJpZCI6MTIzfQ&limit=50
```

Response includes next cursor:
```json
{
  "results": [...],
  "nextCursor": "eyJpZCI6MTczfQ",
  "hasMore": true
}
```

## 3.5 Rate Limiting

- 1000 requests/hour per API key
- 100 concurrent WebSocket connections

Headers:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1642780800
```

## 3.6 Webhooks

Register webhooks for event notifications:
```http
POST /webhooks
{
  "url": "https://myapp.com/webhook",
  "events": ["experiment.completed", "viability.low_alert"],
  "secret": "shared_secret_for_hmac"
}
```

Webhook payload:
```json
{
  "event": "experiment.completed",
  "experimentId": "exp-2025-001",
  "timestamp": "2025-01-21T15:30:00Z",
  "data": {...}
}
```

HMAC signature in header:
```
X-WIA-Signature: sha256=abc123...
```

## 3.7 GraphQL API (Optional)

```graphql
query {
  experiments(filter: {cellType: "hepatocyte", viabilityMin: 80}) {
    experimentId
    title
    results {
      viability
      recovery
    }
  }
}
```

## 3.8 FHIR Integration

For clinical cryopreservation data, support FHIR resources:
- Specimen
- Procedure
- Observation

## 3.9 Data Validation

All submissions validated against JSON Schema before acceptance.
Validation errors returned with 422 status.

---

**Previous**: [PHASE-2: Algorithms](PHASE-2.md)  
**Next**: [PHASE-4: Integration](PHASE-4.md)

© 2025 SmileStory Inc. / WIA  
弘益人間 (Hongik Ingan) · Benefit All Humanity
