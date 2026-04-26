# WIA-AGRI-015: Food Safety Standard
## Phase 2 - API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines RESTful API endpoints, GraphQL schema, and WebSocket interfaces for food safety management systems.

### 1.1 Base URLs

```
Production:  https://api.wia.org/food-safety/v1
Staging:     https://api-staging.wia.org/food-safety/v1
Sandbox:     https://sandbox.wia.org/food-safety/v1
```

### 1.2 Authentication

All API requests require authentication via:
- **OAuth 2.0** (recommended for web/mobile apps)
- **API Key** (for server-to-server)
- **DID Authentication** (for decentralized identity)

```http
Authorization: Bearer <access_token>
X-API-Key: <api_key>
X-WIA-DID: did:wia:manufacturer:12345
```

---

## 2. REST API Endpoints

### 2.1 Product Management

#### POST /products
Create a new food product record.

**Request:**
```json
{
  "productName": "Fresh Strawberries",
  "category": "fresh-produce",
  "batchNumber": "FARM-20250115-001",
  "productionDate": "2025-01-15T06:00:00Z",
  "expiryDate": "2025-01-22T23:59:59Z",
  "manufacturer": {
    "id": "MFR-12345",
    "name": "Sunshine Farms"
  },
  "allergens": {
    "contains": [],
    "free": ["Gluten", "Dairy"]
  }
}
```

**Response (201 Created):**
```json
{
  "productId": "FS-2025-STR-001",
  "status": "created",
  "timestamp": "2025-01-15T08:00:00Z",
  "qrCode": "https://wia.org/verify/FS-2025-STR-001"
}
```

#### GET /products/{productId}
Retrieve product details.

**Response (200 OK):**
```json
{
  "productId": "FS-2025-STR-001",
  "productName": "Fresh Strawberries",
  "category": "fresh-produce",
  "batchNumber": "FARM-20250115-001",
  "safetyStatus": "safe",
  "certifications": ["HACCP", "Organic-USDA"],
  "traceability": {
    "origin": "California, USA",
    "chainOfCustody": 4
  }
}
```

#### PATCH /products/{productId}
Update product information (e.g., new test results).

**Request:**
```json
{
  "safety_monitoring": {
    "contamination_tests": [
      {
        "testType": "microbial",
        "parameter": "Salmonella",
        "result": "negative",
        "status": "pass"
      }
    ]
  }
}
```

#### DELETE /products/{productId}
Mark product as recalled/withdrawn.

---

### 2.2 Safety Monitoring

#### POST /monitoring
Submit real-time safety monitoring data.

**Request:**
```json
{
  "productId": "FS-2025-STR-001",
  "timestamp": "2025-01-15T10:30:00Z",
  "location": "storage",
  "temperature": {
    "current": 4.2,
    "criticalLimit": 7.0
  },
  "humidity": {
    "current": 85
  }
}
```

**Response (200 OK):**
```json
{
  "monitoringId": "MON-2025-001",
  "status": "safe",
  "alerts": []
}
```

#### GET /monitoring/{productId}/latest
Get latest monitoring data for a product.

#### GET /monitoring/alerts
Get active safety alerts.

**Response (200 OK):**
```json
{
  "alerts": [
    {
      "alertId": "ALERT-2025-001",
      "productId": "FS-2025-STR-002",
      "severity": "high",
      "type": "temperature_deviation",
      "message": "Temperature exceeded critical limit for 3 hours",
      "timestamp": "2025-01-15T09:00:00Z"
    }
  ]
}
```

---

### 2.3 HACCP Compliance

#### POST /haccp/ccp
Record Critical Control Point monitoring.

**Request:**
```json
{
  "productId": "FS-2025-STR-001",
  "ccpNumber": 1,
  "description": "Receiving Temperature",
  "criticalLimit": "≤ 4°C",
  "measuredValue": 3.8,
  "status": "within-limit",
  "verifiedBy": "did:wia:inspector:jane-doe"
}
```

#### GET /haccp/{productId}/audit-trail
Get complete HACCP audit trail for a product.

---

### 2.4 Contamination Testing

#### POST /testing/submit
Submit lab test results.

**Request:**
```json
{
  "productId": "FS-2025-STR-001",
  "testType": "microbial",
  "parameter": "E.coli",
  "result": "negative",
  "labId": "LAB-12345",
  "testDate": "2025-01-15T14:00:00Z",
  "certifiedBy": "did:wia:lab:certified-labs"
}
```

#### GET /testing/{productId}/results
Get all test results for a product.

---

### 2.5 Recall Management

#### POST /recalls
Initiate a product recall.

**Request:**
```json
{
  "severity": "class-I",
  "reason": "Salmonella contamination detected",
  "affectedProducts": [
    {
      "productId": "FS-2025-STR-001",
      "batchNumbers": ["FARM-20250115-001", "FARM-20250115-002"]
    }
  ],
  "geographicArea": ["USA", "Canada"],
  "consumerActions": "Do not consume. Return to place of purchase for full refund."
}
```

**Response (201 Created):**
```json
{
  "recallId": "RECALL-2025-001",
  "status": "active",
  "notificationsSent": 15432,
  "mediaReleased": true
}
```

#### GET /recalls/{recallId}
Get recall details.

#### PATCH /recalls/{recallId}/status
Update recall status.

---

### 2.6 Traceability

#### GET /traceability/{productId}
Get complete chain of custody.

**Response (200 OK):**
```json
{
  "productId": "FS-2025-STR-001",
  "chainOfCustody": [
    {
      "stepNumber": 1,
      "entity": "Sunshine Farms",
      "role": "producer",
      "location": "California, USA",
      "timestamp": {
        "received": "2025-01-15T06:00:00Z",
        "shipped": "2025-01-15T08:00:00Z"
      },
      "temperature_log": [
        {"timestamp": "2025-01-15T06:00:00Z", "temperature": 4.1},
        {"timestamp": "2025-01-15T07:00:00Z", "temperature": 4.3}
      ]
    },
    {
      "stepNumber": 2,
      "entity": "FreshCo Distributors",
      "role": "distributor",
      "location": "Los Angeles, CA",
      "timestamp": {
        "received": "2025-01-15T10:00:00Z",
        "shipped": "2025-01-15T14:00:00Z"
      }
    }
  ],
  "blockchain_proof": {
    "merkleRoot": "0x1234abcd...",
    "verified": true
  }
}
```

---

## 3. GraphQL Schema

### 3.1 Query

```graphql
type Query {
  product(id: ID!): Product
  products(filter: ProductFilter): [Product!]!
  monitoring(productId: ID!): [MonitoringData!]!
  recalls(status: RecallStatus): [Recall!]!
  traceability(productId: ID!): TraceabilityChain
}

type Product {
  productId: ID!
  productName: String!
  category: ProductCategory!
  batchNumber: String!
  productionDate: DateTime!
  expiryDate: DateTime!
  safetyStatus: SafetyStatus!
  certifications: [String!]!
  allergens: AllergenInfo!
  traceability: TraceabilityChain
}

enum SafetyStatus {
  SAFE
  WARNING
  CRITICAL
  RECALLED
}
```

### 3.2 Mutation

```graphql
type Mutation {
  createProduct(input: ProductInput!): Product!
  submitMonitoring(input: MonitoringInput!): MonitoringData!
  submitTestResult(input: TestResultInput!): TestResult!
  initiateRecall(input: RecallInput!): Recall!
}
```

---

## 4. WebSocket Interface

### 4.1 Real-time Safety Alerts

```javascript
const ws = new WebSocket('wss://api.wia.org/food-safety/v1/alerts');

ws.onmessage = (event) => {
  const alert = JSON.parse(event.data);
  console.log('Safety Alert:', alert);
  // {
  //   "alertId": "ALERT-2025-001",
  //   "severity": "high",
  //   "productId": "FS-2025-STR-001",
  //   "message": "Temperature deviation detected"
  // }
};
```

### 4.2 Live Temperature Monitoring

```javascript
ws.send(JSON.stringify({
  "action": "subscribe",
  "channel": "temperature",
  "productId": "FS-2025-STR-001"
}));
```

---

## 5. Error Codes

| Code | Message | Description |
|------|---------|-------------|
| 400 | Bad Request | Invalid input data |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Product/resource not found |
| 409 | Conflict | Duplicate batch number |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |

**Error Response Format:**
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Expiry date must be after production date",
    "field": "expiryDate",
    "timestamp": "2025-01-15T10:00:00Z"
  }
}
```

---

## 6. Rate Limiting

| Tier | Requests/minute | Requests/day |
|------|----------------|--------------|
| Free | 60 | 1,000 |
| Standard | 300 | 10,000 |
| Enterprise | 1,000 | 100,000 |

**Rate Limit Headers:**
```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1642248000
```

---

## 7. Webhooks

Subscribe to events:
- `product.created`
- `product.recalled`
- `monitoring.alert`
- `test.failed`
- `recall.initiated`

**Webhook Payload:**
```json
{
  "event": "monitoring.alert",
  "timestamp": "2025-01-15T10:00:00Z",
  "data": {
    "productId": "FS-2025-STR-001",
    "alertType": "temperature_deviation",
    "severity": "high"
  }
}
```

---

**弘益人間 (Hongik Ingan) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
