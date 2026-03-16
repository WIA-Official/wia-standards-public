# WIA-IND-007 Phase 2: API Interface Specification
## Food Safety Traceability Standard v1.0

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

---

## Overview

Phase 2 defines RESTful API interfaces enabling seamless data exchange between supply chain participants. These APIs support traceability queries, event submissions, recall notifications, and regulatory reporting while maintaining security and data privacy.

## 1. API Architecture

### 1.1 Design Principles

**RESTful Standards:**
- Resource-based URLs
- Standard HTTP methods (GET, POST, PUT, DELETE)
- Stateless operations
- JSON request/response bodies
- HTTPS/TLS 1.3 encryption mandatory

**Base URL Format:**
```
https://api.traceability.{organization}.com/v1
```

### 1.2 Versioning Strategy

APIs use URL path versioning:
- `/v1/` - Current stable version
- `/v2/` - Next major version (backward incompatible)
- Header-based versioning for minor updates

## 2. Authentication and Authorization

### 2.1 OAuth 2.0 Implementation

**Grant Types Supported:**
- Client Credentials (machine-to-machine)
- Authorization Code (user delegation)
- Refresh Token (token renewal)

**Token Endpoint:**
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={CLIENT_ID}
&client_secret={CLIENT_SECRET}
&scope=traceability:read traceability:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "traceability:read traceability:write"
}
```

### 2.2 API Key Authentication

Alternative authentication for simpler integrations:

**Header:**
```
X-API-Key: your-api-key-here
```

## 3. Traceability Query APIs

### 3.1 Product Lookup

Retrieve complete product traceability information:

**Endpoint:**
```
GET /v1/products/{gtin}/batch/{batchNumber}
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "product": {
      "gtin": "01234567890128",
      "batchNumber": "BATCH-2025-001",
      "productName": "Organic Lettuce",
      "currentLocation": "Distribution Center A",
      "status": "in-transit"
    },
    "origin": {
      "producerId": "FARM-ABC-001",
      "location": "Farm ABC, California, USA",
      "harvestDate": "2025-12-20"
    },
    "supplyChainEvents": [...],
    "temperatureLog": [...],
    "certifications": [...]
  },
  "philosophy": "弘益人間 - Benefit All Humanity"
}
```

### 3.2 Backward Tracing

Trace product back to origin:

**Endpoint:**
```
GET /v1/trace/backward?batch={batchNumber}&depth=all
```

**Response includes:**
- All upstream suppliers
- Ingredient source batches
- Processing facilities
- Complete custody chain

### 3.3 Forward Tracing

Identify all products and locations downstream:

**Endpoint:**
```
GET /v1/trace/forward?batch={batchNumber}
```

**Use Case:** Identify distribution of potentially contaminated batch

## 4. Event Submission APIs

### 4.1 Supply Chain Event Creation

Submit handling events:

**Endpoint:**
```
POST /v1/events
Content-Type: application/json
Authorization: Bearer {access_token}
```

**Request Body:**
```json
{
  "eventType": "TRANSPORT",
  "timestamp": "2025-12-20T14:00:00Z",
  "batchNumber": "BATCH-2025-001",
  "fromLocation": "FACILITY-001",
  "toLocation": "DISTRIBUTION-CENTER-A",
  "transportDetails": {
    "vehicleId": "TRUCK-123",
    "driverId": "DRIVER-456",
    "departureTime": "2025-12-20T14:00:00Z",
    "estimatedArrival": "2025-12-20T18:00:00Z"
  },
  "temperatureAtDeparture": 4.2,
  "sealNumber": "SEAL-789"
}
```

**Response:**
```json
{
  "status": "success",
  "eventId": "EVT-2025-12345",
  "message": "Event recorded successfully",
  "blockchainTransaction": "0x1234567890abcdef..."
}
```

### 4.2 Temperature Reading Submission

Bulk temperature data upload:

**Endpoint:**
```
POST /v1/temperature-logs/{batchNumber}
```

**Request:**
```json
{
  "readings": [
    {
      "timestamp": "2025-12-20T14:00:00Z",
      "temperature": 4.2,
      "sensorId": "TEMP-001"
    },
    {
      "timestamp": "2025-12-20T14:15:00Z",
      "temperature": 4.3,
      "sensorId": "TEMP-001"
    }
  ]
}
```

## 5. Recall Management APIs

### 5.1 Recall Initiation

Trigger product recall process:

**Endpoint:**
```
POST /v1/recalls
Authorization: Bearer {access_token}
```

**Request:**
```json
{
  "recallType": "voluntary",
  "severity": "Class I",
  "affectedProducts": [
    {
      "gtin": "01234567890128",
      "batchNumbers": ["BATCH-2025-001", "BATCH-2025-002"]
    }
  ],
  "contaminationType": "salmonella",
  "detectionPoint": "Retail Store 123",
  "reason": "Potential Salmonella contamination detected",
  "recallDate": "2025-12-27",
  "contactInformation": {
    "name": "Quality Manager",
    "phone": "+1-555-0123",
    "email": "quality@company.com"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "recallId": "RECALL-2025-001",
  "affectedUnits": 1250,
  "distributionCenters": 47,
  "retailLocations": 230,
  "notificationsSent": 277,
  "estimatedRetrievalTime": "24 hours"
}
```

### 5.2 Recall Status Tracking

Monitor recall progress:

**Endpoint:**
```
GET /v1/recalls/{recallId}/status
```

**Response:**
```json
{
  "recallId": "RECALL-2025-001",
  "status": "in-progress",
  "initiatedAt": "2025-12-27T10:00:00Z",
  "progress": {
    "notificationRate": "100%",
    "retrievalRate": "67%",
    "unitsRetrieved": 837,
    "unitsOutstanding": 413
  },
  "timeline": [...]
}
```

## 6. Regulatory Reporting APIs

### 6.1 Compliance Report Generation

Generate regulatory reports:

**Endpoint:**
```
POST /v1/reports/regulatory
```

**Request:**
```json
{
  "reportType": "FSMA-204",
  "dateRange": {
    "start": "2025-01-01",
    "end": "2025-12-31"
  },
  "products": ["01234567890128"],
  "format": "pdf"
}
```

**Response:**
```json
{
  "status": "success",
  "reportId": "RPT-2025-001",
  "downloadUrl": "https://reports.company.com/RPT-2025-001.pdf",
  "expiresAt": "2025-12-28T10:00:00Z"
}
```

### 6.2 Audit Trail Export

Export complete audit trails:

**Endpoint:**
```
GET /v1/audit-trail?batch={batchNumber}&format=json
```

## 7. Integration APIs

### 7.1 ERP System Webhooks

Configure webhooks for event notifications:

**Endpoint:**
```
POST /v1/webhooks
```

**Request:**
```json
{
  "url": "https://erp.company.com/webhooks/traceability",
  "events": ["event.created", "recall.initiated", "temperature.alert"],
  "authentication": {
    "type": "bearer",
    "token": "webhook-secret-token"
  }
}
```

### 7.2 Blockchain Verification

Verify data against blockchain records:

**Endpoint:**
```
GET /v1/blockchain/verify?eventId={eventId}
```

**Response:**
```json
{
  "verified": true,
  "transactionHash": "0x1234567890abcdef...",
  "blockNumber": 15234567,
  "timestamp": "2025-12-20T14:00:00Z",
  "network": "Ethereum Mainnet"
}
```

## 8. Error Handling

### 8.1 Standard Error Responses

**Error Format:**
```json
{
  "error": {
    "code": "INVALID_BATCH_NUMBER",
    "message": "Batch number BATCH-2025-999 not found",
    "details": "The specified batch number does not exist in the system",
    "timestamp": "2025-12-27T10:00:00Z",
    "requestId": "req-12345"
  }
}
```

### 8.2 HTTP Status Codes

- `200 OK` - Successful request
- `201 Created` - Resource created
- `400 Bad Request` - Invalid input
- `401 Unauthorized` - Authentication failed
- `403 Forbidden` - Insufficient permissions
- `404 Not Found` - Resource not found
- `429 Too Many Requests` - Rate limit exceeded
- `500 Internal Server Error` - Server error

## 9. Rate Limiting

### 9.1 Rate Limit Headers

**Response Headers:**
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 997
X-RateLimit-Reset: 1640000000
```

### 9.2 Tier-Based Limits

- **Free Tier:** 100 requests/hour
- **Standard Tier:** 1,000 requests/hour
- **Premium Tier:** 10,000 requests/hour
- **Enterprise Tier:** Unlimited (with SLA)

## 10. Data Privacy and Security

### 10.1 Field-Level Encryption

Sensitive fields encrypted at rest and in transit:
- Personal worker identifications
- Proprietary formulations
- Pricing information
- Supplier contracts

### 10.2 Access Control Lists

Fine-grained permissions:

**Roles:**
- `traceability:read` - View traceability data
- `traceability:write` - Submit events
- `traceability:admin` - Full access
- `recall:initiate` - Trigger recalls
- `reports:generate` - Create reports

---

## Implementation Guidelines

### SDK Availability

Official SDKs provided for:
- JavaScript/TypeScript (Node.js, Browser)
- Python
- Java
- C#/.NET
- Go
- Ruby

### Testing Environment

Sandbox API available at:
```
https://api.sandbox.traceability.wia.org/v1
```

### Support and Documentation

- Interactive API Documentation: https://docs.wia.org/api
- Developer Forum: https://forum.wia.org
- Technical Support: support@wia.org

---

**Document Version:** 1.0
**Last Updated:** 2025-12-27
**Status:** Official Release
**弘益人間 - Benefit All Humanity through Connected Food Systems**

© 2025 SmileStory Inc. / WIA (World Certification Industry Association)
