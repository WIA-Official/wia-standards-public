# WIA-DATA_WAREHOUSE: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the API interface requirements for DATA WAREHOUSE. All implementations MUST provide these endpoints to ensure interoperability.

## 2. REST API Endpoints

### 2.1 Base URL
```
https://api.wia.org/v1/wia-data-warehouse/
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
POST /v1/wia-data-warehouse/records HTTP/1.1
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
npm install @wia/wia-data-warehouse-sdk
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
import { createClient } from '@wia/wia-data-warehouse-sdk';

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

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-DATA_WAREHOUSE is evaluated across three tiers:

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

- `wia-standards/standards/WIA-DATA_WAREHOUSE/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-DATA_WAREHOUSE/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-DATA_WAREHOUSE/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex G — Document Index

This PHASE document is one of four PHASE artifacts that together describe this WIA standard. The full set of related artifacts is:

- `spec/PHASE-1-DATA-FORMAT.md` — normative data formats and schema definitions.
- `spec/PHASE-2-API.md` — normative SDK and Client API contract that consumes the PHASE 1 data formats.
- `spec/PHASE-3-PROTOCOL.md` — normative real-time communication protocol bindings (where applicable).
- `spec/PHASE-4-INTEGRATION.md` — normative ecosystem integration patterns (LIMS, paging, federation, reporting).

Readers SHOULD consult all four PHASE documents in sequence when planning a deployment. The OpenAPI document published alongside this standard reflects the §4 endpoints in machine-readable form and is updated synchronously with this PHASE.

This index is informative; the normative content is in the body of each PHASE document.
