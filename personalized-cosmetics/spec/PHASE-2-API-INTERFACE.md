# WIA-IND-006 Phase 2: API Interface Specification
## Personalized Cosmetics Standard - API Protocols and Endpoints

**Version:** 1.0
**Last Updated:** 2025-01-15
**Status:** Active
**Philosophy:** 弘익人間 (Benefit All Humanity)

---

## Overview

Phase 2 defines RESTful and GraphQL API interfaces for personalized cosmetics systems, enabling interoperability between skin analysis platforms, formulation engines, manufacturing systems, and consumer applications.

## 1. API Architecture

### 1.1 Base URL Structure

```
https://api.{provider}.com/wia-ind-006/v1/
```

### 1.2 Authentication

All API requests require authentication via OAuth 2.0 or API keys.

**Headers:**
```
Authorization: Bearer {access_token}
X-API-Key: {api_key}
X-WIA-Standard: IND-006-v1.0
Content-Type: application/json
```

### 1.3 Rate Limiting

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642521600
```

## 2. Profile Management APIs

### 2.1 Create User Profile

**Endpoint:** `POST /profiles`

**Request:**
```json
{
  "userId": "user_12345",
  "demographicData": {
    "age": 28,
    "location": {
      "country": "US",
      "climate": "temperate"
    }
  },
  "preferences": {
    "budget": "mid-range",
    "texture": "serum"
  }
}
```

**Response:** `201 Created`
```json
{
  "profileId": "prof_abc123",
  "userId": "user_12345",
  "createdAt": "2025-01-15T10:30:00Z",
  "status": "active",
  "version": "1.0"
}
```

### 2.2 Get Profile

**Endpoint:** `GET /profiles/{profileId}`

**Response:** `200 OK`
```json
{
  "profileId": "prof_abc123",
  "userId": "user_12345",
  "skinAnalysis": { },
  "preferences": { },
  "createdAt": "2025-01-15T10:30:00Z",
  "updatedAt": "2025-01-15T14:20:00Z"
}
```

### 2.3 Update Profile

**Endpoint:** `PATCH /profiles/{profileId}`

**Request:**
```json
{
  "preferences": {
    "texture": "cream"
  }
}
```

**Response:** `200 OK`

### 2.4 Delete Profile

**Endpoint:** `DELETE /profiles/{profileId}`

**Response:** `204 No Content`

## 3. Skin Analysis APIs

### 3.1 Submit Analysis

**Endpoint:** `POST /analysis`

**Request:**
```json
{
  "profileId": "prof_abc123",
  "analysisType": "imaging",
  "images": [
    {
      "type": "visible",
      "data": "base64_encoded_image",
      "timestamp": "2025-01-15T10:00:00Z"
    }
  ],
  "sensorData": {
    "moisture": [
      {"location": "forehead", "value": 45}
    ]
  }
}
```

**Response:** `202 Accepted`
```json
{
  "sessionId": "sess_xyz789",
  "status": "processing",
  "estimatedCompletion": "2025-01-15T10:05:00Z",
  "statusUrl": "/analysis/sess_xyz789"
}
```

### 3.2 Get Analysis Results

**Endpoint:** `GET /analysis/{sessionId}`

**Response:** `200 OK`
```json
{
  "sessionId": "sess_xyz789",
  "profileId": "prof_abc123",
  "status": "completed",
  "timestamp": "2025-01-15T10:04:32Z",
  "results": {
    "skinType": "combination",
    "moistureLevel": 45,
    "concerns": [
      {"type": "acne", "severity": 3},
      {"type": "aging", "severity": 2}
    ],
    "aiAnalysis": {
      "wrinkleCount": 12,
      "confidenceScore": 0.94
    }
  },
  "recommendations": {
    "suggestedProducts": ["serum", "moisturizer"],
    "keyIngredients": ["niacinamide", "hyaluronic-acid"]
  }
}
```

## 4. Personalization APIs

### 4.1 Generate Formulation

**Endpoint:** `POST /formulations/generate`

**Request:**
```json
{
  "profileId": "prof_abc123",
  "productType": "serum",
  "primaryGoal": "hydration",
  "constraints": {
    "budget": "mid-range",
    "excludeIngredients": ["fragrance", "parabens"]
  }
}
```

**Response:** `201 Created`
```json
{
  "formulationId": "form_def456",
  "profileId": "prof_abc123",
  "productType": "serum",
  "ingredients": [
    {
      "inciName": "Hyaluronic Acid",
      "concentration": 2.0,
      "unit": "%",
      "function": "active"
    },
    {
      "inciName": "Niacinamide",
      "concentration": 5.0,
      "unit": "%",
      "function": "active"
    }
  ],
  "properties": {
    "pH": 5.5,
    "texture": "lightweight gel"
  },
  "estimatedPrice": 45.00,
  "matchScore": 0.92
}
```

### 4.2 Optimize Formulation

**Endpoint:** `POST /formulations/{formulationId}/optimize`

**Request:**
```json
{
  "feedback": {
    "satisfaction": 4,
    "irritation": false,
    "efficacy": "good",
    "texture": "too light"
  },
  "adjustments": {
    "increaseHydration": true
  }
}
```

**Response:** `200 OK`
```json
{
  "formulationId": "form_def456_v2",
  "changes": [
    "Increased Hyaluronic Acid from 2.0% to 3.0%",
    "Added Glycerin at 2.0%"
  ],
  "expectedImprovements": ["enhanced hydration", "richer texture"]
}
```

### 4.3 Get Formulation

**Endpoint:** `GET /formulations/{formulationId}`

**Response:** `200 OK`

## 5. Manufacturing APIs

### 5.1 Submit Production Order

**Endpoint:** `POST /manufacturing/orders`

**Request:**
```json
{
  "formulationId": "form_def456",
  "quantity": 1,
  "priority": "normal",
  "deliveryAddress": {
    "name": "Jane Doe",
    "street": "123 Main St",
    "city": "New York",
    "state": "NY",
    "zip": "10001",
    "country": "US"
  },
  "packaging": {
    "type": "standard",
    "labelCustomization": "Happy Birthday Jane!"
  }
}
```

**Response:** `201 Created`
```json
{
  "orderId": "ord_ghi789",
  "formulationId": "form_def456",
  "status": "queued",
  "estimatedCompletion": "2025-01-16T14:00:00Z",
  "estimatedShipping": "2025-01-17T09:00:00Z",
  "trackingUrl": "/manufacturing/orders/ord_ghi789"
}
```

### 5.2 Get Order Status

**Endpoint:** `GET /manufacturing/orders/{orderId}`

**Response:** `200 OK`
```json
{
  "orderId": "ord_ghi789",
  "status": "manufacturing",
  "timeline": [
    {"stage": "queued", "timestamp": "2025-01-15T15:00:00Z"},
    {"stage": "formulating", "timestamp": "2025-01-16T08:00:00Z"},
    {"stage": "manufacturing", "timestamp": "2025-01-16T10:00:00Z", "current": true}
  ],
  "qualityChecks": {
    "ingredientVerification": "passed",
    "phTest": "pending"
  }
}
```

### 5.3 Update Order

**Endpoint:** `PATCH /manufacturing/orders/{orderId}`

**Request:**
```json
{
  "priority": "high",
  "deliveryAddress": {
    "street": "456 New Address"
  }
}
```

**Response:** `200 OK`

## 6. Quality Control APIs

### 6.1 Submit QC Results

**Endpoint:** `POST /quality-control/results`

**Request:**
```json
{
  "orderId": "ord_ghi789",
  "batchNumber": "BATCH-2025-A123",
  "checks": {
    "ingredientVerification": "pass",
    "phTest": {"result": 5.5, "status": "pass"},
    "stabilityTest": "pass",
    "contaminationScreen": "pass"
  },
  "inspector": "QC-007",
  "timestamp": "2025-01-16T12:00:00Z",
  "certificateNumber": "CERT-2025-001234"
}
```

**Response:** `201 Created`

### 6.2 Get QC Certificate

**Endpoint:** `GET /quality-control/certificates/{certificateNumber}`

**Response:** `200 OK`

## 7. Ingredient Database APIs

### 7.1 Search Ingredients

**Endpoint:** `GET /ingredients?search=hyaluronic&category=active`

**Response:** `200 OK`
```json
{
  "results": [
    {
      "ingredientId": "ing_123",
      "inciName": "Hyaluronic Acid",
      "commonNames": ["Hyaluronan"],
      "category": "active",
      "functions": ["moisturizing", "anti-aging"],
      "safetyData": {
        "maxConcentration": 5.0,
        "commonAllergen": false
      }
    }
  ],
  "count": 1,
  "page": 1,
  "totalPages": 1
}
```

### 7.2 Get Ingredient Details

**Endpoint:** `GET /ingredients/{ingredientId}`

**Response:** `200 OK`

## 8. Verification and Authentication APIs

### 8.1 Generate QR Code

**Endpoint:** `POST /verification/qr-code`

**Request:**
```json
{
  "productId": "prod_jkl012",
  "formulationId": "form_def456",
  "orderId": "ord_ghi789",
  "type": "authentication"
}
```

**Response:** `201 Created`
```json
{
  "qrCodeId": "qr_mno345",
  "imageUrl": "https://cdn.wia.com/qr/qr_mno345.png",
  "dataUrl": "data:image/png;base64,...",
  "verificationUrl": "https://verify.wia-ind-006.org/qr_mno345"
}
```

### 8.2 Verify Product

**Endpoint:** `GET /verification/products/{productId}`

**Response:** `200 OK`
```json
{
  "productId": "prod_jkl012",
  "authentic": true,
  "formulationId": "form_def456",
  "manufactureDate": "2025-01-16",
  "expiryDate": "2026-01-16",
  "certifications": ["organic", "cruelty-free"],
  "blockchain": {
    "verified": true,
    "txHash": "0x..."
  }
}
```

### 8.3 Issue Verifiable Credential

**Endpoint:** `POST /verification/credentials`

**Request:**
```json
{
  "productId": "prod_jkl012",
  "credentialType": "ProductAuthenticity",
  "issuerDID": "did:wia:ind006:manufacturer:001"
}
```

**Response:** `201 Created`

## 9. Analytics and Reporting APIs

### 9.1 Get User Analytics

**Endpoint:** `GET /analytics/users/{userId}`

**Response:** `200 OK`
```json
{
  "userId": "user_12345",
  "metrics": {
    "profileUpdates": 5,
    "analysesCompleted": 3,
    "formulationsGenerated": 7,
    "ordersPlaced": 4,
    "averageSatisfaction": 4.5
  },
  "skinProgress": {
    "moistureImprovement": 15,
    "concernsResolved": 2
  }
}
```

### 9.2 Get System Metrics

**Endpoint:** `GET /analytics/system`

**Response:** `200 OK`
```json
{
  "totalProfiles": 50000,
  "activeUsers": 12000,
  "formulationsGenerated": 75000,
  "averageMatchScore": 0.89,
  "systemUptime": 99.9
}
```

## 10. Webhook Events

### 10.1 Webhook Registration

**Endpoint:** `POST /webhooks`

**Request:**
```json
{
  "url": "https://your-app.com/webhooks/wia-ind-006",
  "events": ["analysis.completed", "formulation.ready", "order.shipped"],
  "secret": "your_webhook_secret"
}
```

### 10.2 Event Payloads

**analysis.completed:**
```json
{
  "event": "analysis.completed",
  "timestamp": "2025-01-15T10:05:00Z",
  "data": {
    "sessionId": "sess_xyz789",
    "profileId": "prof_abc123",
    "status": "completed"
  }
}
```

## 11. Error Handling

### 11.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_PROFILE",
    "message": "Profile not found",
    "details": "No profile exists with ID prof_invalid",
    "timestamp": "2025-01-15T10:00:00Z",
    "requestId": "req_xyz123"
  }
}
```

### 11.2 HTTP Status Codes

- 200: Success
- 201: Created
- 202: Accepted (async processing)
- 204: No Content
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 429: Too Many Requests
- 500: Internal Server Error
- 503: Service Unavailable

## 12. GraphQL API

### 12.1 Schema Endpoint

```
POST https://api.{provider}.com/wia-ind-006/graphql
```

### 12.2 Example Query

```graphql
query GetUserProfile($profileId: ID!) {
  profile(id: $profileId) {
    profileId
    userId
    skinAnalysis {
      skinType
      moistureLevel
      concerns {
        type
        severity
      }
    }
    formulations {
      formulationId
      productType
      matchScore
    }
  }
}
```

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 - Benefit All Humanity**

---

## Annex A — Conformance Tier Matrix

WIA conformance for personalized-cosmetics is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/personalized-cosmetics/api/` — TypeScript SDK skeleton
- `wia-standards/standards/personalized-cosmetics/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/personalized-cosmetics/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
