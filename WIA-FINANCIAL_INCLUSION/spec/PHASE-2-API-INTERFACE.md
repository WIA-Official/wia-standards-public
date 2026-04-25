# WIA-FINANCIAL_INCLUSION: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the API interface requirements for FINANCIAL INCLUSION. All implementations MUST provide these endpoints to ensure interoperability.

## 2. REST API Endpoints

### 2.1 Base URL
```
https://api.wia.org/v1/wia-financial-inclusion/
```

### 2.2 Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | /records | List all records |
| GET | /records/{id} | Get specific record |
| POST | /records | Create new record |
| PUT | /records/{id} | Update record |
| DELETE | /records/{id} | Delete record |
| GET | /status | Service health check |
| GET | /schema | Get data schema |

### 2.3 Request/Response Format

#### Create Record (POST /records)
```http
POST /v1/wia-financial-inclusion/records HTTP/1.1
Content-Type: application/json
Authorization: Bearer <token>

{
  "category": "standard",
  "value": "data content",
  "metadata": {}
}
```

#### Response
```http
HTTP/1.1 201 Created
Content-Type: application/json

{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "created",
  "timestamp": "2025-01-01T00:00:00Z"
}
```

## 3. TypeScript SDK

### 3.1 Installation
```bash
npm install @wia/wia-financial-inclusion-sdk
```

### 3.2 Client Interface
```typescript
interface WIAClient {
  // Record operations
  createRecord(data: RecordInput): Promise<Record>;
  getRecord(id: string): Promise<Record>;
  updateRecord(id: string, data: Partial<RecordInput>): Promise<Record>;
  deleteRecord(id: string): Promise<void>;
  listRecords(options?: ListOptions): Promise<RecordList>;
  
  // Utility methods
  validate(data: unknown): ValidationResult;
  getSchema(): Promise<JSONSchema>;
  healthCheck(): Promise<HealthStatus>;
}

interface RecordInput {
  category: string;
  value: any;
  metadata?: Record<string, unknown>;
}

interface Record extends RecordInput {
  id: string;
  type: string;
  version: string;
  timestamp: string;
  signature?: string;
}
```

### 3.3 Usage Example
```typescript
import { createClient } from '@wia/wia-financial-inclusion-sdk';

const client = createClient({
  apiKey: process.env.WIA_API_KEY,
  baseUrl: 'https://api.wia.org/v1'
});

// Create a new record
const record = await client.createRecord({
  category: 'standard',
  value: 'example data',
  metadata: { source: 'application' }
});

// Get record by ID
const retrieved = await client.getRecord(record.id);
```

## 4. Authentication

### 4.1 API Key Authentication
```http
Authorization: Bearer <api_key>
```

### 4.2 OAuth 2.0
```http
Authorization: Bearer <access_token>
```

### 4.3 Token Scopes
| Scope | Description |
|-------|-------------|
| read | Read-only access |
| write | Create and update |
| delete | Delete records |
| admin | Full access |

## 5. Error Handling

### 5.1 Error Response Format
```json
{
  "error": {
    "code": "E001",
    "message": "Validation failed",
    "details": [
      { "field": "category", "issue": "required" }
    ]
  }
}
```

### 5.2 HTTP Status Codes
| Code | Meaning |
|------|---------|
| 200 | Success |
| 201 | Created |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 429 | Rate Limited |
| 500 | Server Error |

## 6. Rate Limiting

### 6.1 Limits
| Tier | Requests/Minute | Requests/Day |
|------|----------------|--------------|
| Free | 60 | 1,000 |
| Standard | 600 | 50,000 |
| Enterprise | 6,000 | Unlimited |

### 6.2 Rate Limit Headers
```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1704067200
```

## 7. Webhooks

### 7.1 Event Types
| Event | Description |
|-------|-------------|
| record.created | New record created |
| record.updated | Record updated |
| record.deleted | Record deleted |

### 7.2 Webhook Payload
```json
{
  "event": "record.created",
  "timestamp": "2025-01-01T00:00:00Z",
  "data": { "id": "...", "type": "..." }
}
```

---

## 8. Reference Standards Alignment

The Phase 2 API surface is layered above well-established financial-services and IT primitives.

| Concern | Reference |
|---------|-----------|
| HTTP semantics | RFC 9110 |
| HTTP/1.1 | RFC 9112 |
| HTTP/2 | RFC 9113 |
| HTTP/3 over QUIC | RFC 9114 / RFC 9000 |
| TLS 1.3 | RFC 8446 |
| Certificate format | RFC 5280 (X.509 v3) |
| OpenAPI description | OpenAPI Specification 3.1 |
| JSON | RFC 8259 |
| Errors | RFC 9457 (Problem Details for HTTP APIs) |
| Pagination linking | RFC 8288 (Web Linking) |
| Bearer tokens | RFC 6750 |
| OAuth 2.0 | RFC 6749 + RFC 7636 (PKCE) |
| Mutual TLS | RFC 8705 |
| JWT | RFC 7519 |
| Ed25519 | RFC 8032 |
| ECDSA | NIST FIPS 186-5 |
| Hash | FIPS 180-4, FIPS 202 |
| WebSocket | RFC 6455 |
| Trace context | W3C Trace Context Recommendation |
| Locale | BCP 47 (RFC 5646), Unicode CLDR |
| Time | ISO 8601:2019 |
| Financial-services messaging | ISO 20022 |
| Information security | ISO/IEC 27001:2022 |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

## 9. Conformance

A Phase 2 implementation is conformant when:

1. The OpenAPI 3.1 description publishes every endpoint with request and response schemas.
2. Authentication accepts at least the `bearerAuth` (JWT) scheme.
3. Errors use RFC 9457 problem-detail responses.
4. Cryptographic primitives match §8 with explicit algorithm identifiers in tokens and signatures.
5. Where the deployment integrates with traditional financial-services infrastructure, ISO 20022 messages are produced or consumed under the deployment's documented profile.

## 10. Implementation Appendix

### 10.1 Idempotency

Endpoints that create resources accept an `Idempotency-Key` request header containing a UUID. The server stores the response for at least 24 hours and returns the cached response on retry, guarding against duplicate transactions when the network is unreliable.

### 10.2 Trace Context

Every request carries a W3C Trace Context `traceparent` so distributed traces span the customer-facing app, the WIA-FINANCIAL_INCLUSION service, downstream payment processors, and regulatory reporting systems.

### 10.3 Rate-limit Headers

Rate-limit headers follow the IETF rate-limit-headers conventions in addition to the legacy `X-RateLimit-*` headers documented in §6.2 above.

### 10.4 Cross-Border Disclosures

Where the implementation processes cross-border financial transactions, it conforms to the deploying jurisdictions' AML/CFT rules, FATF Recommendations, and the rule-defined recordkeeping retention. Disclosure to regulators follows the rule-defined demand process and is subject to prior legal review.

### 10.5 Accessibility

Customer-facing surfaces conform to W3C WCAG 2.2 Level AA so that users with disabilities can transact without external accommodation. Conformance evidence is part of the deployment's compliance artefacts.

### 10.6 Idempotency and Replay

Money-moving endpoints accept an `Idempotency-Key` header carrying a UUID. The server stores the response for at least 24 hours and returns the cached response on retry. This is critical for financial-inclusion deployments where unreliable connectivity makes duplicate-submission risk material.

### 10.7 Pagination and Search

List endpoints use cursor-based pagination via the `Link` header per RFC 8288. Search endpoints support filtering by transaction category, time range, party, currency, and amount range. Filters compose; the server returns a `Vary` header listing the request fields that affect caching.

### 10.8 Webhook Conformance

Webhook delivery follows the three-rule contract: HMAC-SHA-256 signatures over the payload, idempotent delivery identifiers in the `X-Wia-Delivery-Id` header, and exponential-backoff retries up to a configurable maximum. Receivers respond with 2xx within 15 seconds for acknowledgement.

### 10.9 OpenAPI Description

The full machine-readable interface description is published as an OpenAPI 3.1.0 document at `/api/v1/openapi.json`. Schema validation uses JSON Schema draft 2020-12. The reference deployment reviews the OpenAPI document on every release to ensure that no endpoint deviates from the published contract.

### 10.10 Capability Discovery

Clients discover server capabilities through the `/.well-known/wia-financial-inclusion/capabilities` endpoint. The response includes supported currencies, supported transaction categories, supported identity-credential issuers, and the active rate-limit policy.

### 10.11 Versioning Policy

Path versioning (`/api/v1`, `/api/v2`) follows semantic-versioning principles. Minor versions remain wire-compatible with the prior minor of the same major. Major bumps coexist with the prior major for at least 18 months before deprecation, in keeping with the long-tail nature of financial-inclusion deployments.

---

**弘益人間 (Benefit All Humanity)**



## 11. Operational Considerations

### 11.1 Network Resilience

Underbanked populations frequently operate over unreliable networks. The Phase 2 API surface is designed for graceful degradation:

- All money-moving endpoints accept idempotency keys.
- All endpoints support resumable operations through documented state machines.
- The reference SDK retries failed requests with jittered exponential backoff.
- Server responses include `Retry-After` for transient errors so clients back off in a coordinated manner.

### 11.2 Battery and Bandwidth

Mobile clients in low-resource environments cannot afford constant connectivity. The reference SDK supports:

- Batched submission of cached transactions when connectivity is restored.
- Differential synchronisation that transfers only the records that have changed since the last successful sync.
- CBOR alternate encoding (RFC 8949) negotiated via `Accept: application/cbor` for bandwidth-constrained links.

### 11.3 Multi-Tenant Operations

Operating organisations that host services for multiple distinct cohorts (community organisations, microfinance institutions, NGOs) document their tenant-isolation guarantees in the deployment's compliance evidence. The reference programme follows ISO/IEC 27017:2015 cloud-services controls for shared infrastructure.

### 11.4 Disaster Recovery

Recovery objectives align with ISO/IEC 27031:2011. The reference RTO is 15 minutes for the API tier and 5 minutes for telemetry-driven dashboards. Quarterly DR drills are mandatory and reviewed by the operating organisation's quality officer.

### 11.5 Sustainability

Operating organisations may publish sustainability reports following ISO 14064-1:2018. For financial-inclusion deployments, energy efficiency on the customer side (battery consumption per transaction) is a meaningful metric and is included in the conformance evidence where measured.

### 11.6 Deprecation Discipline

API endpoints are deprecated through a documented procedure: announcement on the deprecation registry, deprecation notice in the OpenAPI document, sunset header per RFC 8594 in responses to deprecated endpoints, monitoring of remaining traffic, and final removal after the announced sunset window has elapsed. The reference deprecation window is 18 months for endpoints actively used by financial-inclusion deployments, recognising the long-tail nature of the user base.

### 11.7 Audit and Examination Endpoints

Read-only audit endpoints expose the deployment's configuration, the active conformance status, and the recent operational metrics for inspection by independent auditors. Audit access uses dedicated credentials with read-only scope and is logged separately from production traffic so that auditor activity is distinguishable from operations.

### 11.8 Test Vectors

Conformance test vectors are published as an open package. Each test vector documents the request, the expected response, and the active wire-protocol version so that any implementation can self-assess against the same baseline prior to a formal review.

---

*© 2025 WIA - World Certification Industry Association*
*MIT License*
