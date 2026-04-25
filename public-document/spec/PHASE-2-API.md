# WIA-SOC-013 PHASE 2: API SPECIFICATION

**Public Document Standard - RESTful API and SDK**

Version: 1.0
Date: 2025-01-15
Status: Final

---

## 1. API Endpoints

### Base URL

```
https://api.wiastandards.gov/v1/documents
```

### 1.1 Document Operations

#### Create Document
```http
POST /documents
Content-Type: application/json
Authorization: Bearer {token}

{
  "type": "birthCertificate",
  "data": {...},
  "language": "en"
}

Response: 201 Created
{
  "documentId": "doc:wia:soc013:birth:US:2025:abc123",
  "status": "created",
  "url": "/documents/doc:wia:soc013:birth:US:2025:abc123"
}
```

#### Get Document
```http
GET /documents/{documentId}
Authorization: Bearer {token}

Response: 200 OK
{
  "documentId": "...",
  "type": "...",
  "data": {...}
}
```

#### Verify Document
```http
POST /documents/{documentId}/verify
Content-Type: application/json

{
  "checkRevocation": true,
  "validateSignature": true,
  "verifyBlockchain": true
}

Response: 200 OK
{
  "valid": true,
  "signatureValid": true,
  "notRevoked": true,
  "blockchainValid": true
}
```

### 1.2 Signature Operations

```http
POST /documents/{documentId}/sign
Authorization: Bearer {token}

{
  "algorithm": "ECDSA-SHA256",
  "certificateChain": true
}

Response: 200 OK
{
  "signature": "...",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 1.3 Metadata Operations

```http
PUT /documents/{documentId}/metadata
Content-Type: application/json

{
  "dublinCore": {...},
  "premis": {...}
}

Response: 200 OK
```

## 2. SDK Libraries

### 2.1 JavaScript/TypeScript

```typescript
import { WiaPublicDocument } from 'wia-soc-013';

const document = await WiaPublicDocument.create({
  type: 'birthCertificate',
  data: birthData
});

await document.sign({ privateKey: key });
await document.publish();

const verification = await document.verify();
```

### 2.2 Python

```python
from wia_soc_013 import WiaPublicDocument

document = WiaPublicDocument.create(
    type='birthCertificate',
    data=birth_data
)

document.sign(private_key=key)
document.publish()

verification = document.verify()
```

## 3. Authentication

OAuth 2.0 with OpenID Connect:

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={client_id}
&client_secret={client_secret}
&scope=documents:read documents:write

Response:
{
  "access_token": "...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

## 4. Rate Limiting

- 1000 requests per minute per API key
- 50 MB total upload size per minute
- Retry-After header on 429 responses

## 5. Error Codes

| Code | Description |
|------|-------------|
| 400 | Bad Request - Invalid parameters |
| 401 | Unauthorized - Invalid credentials |
| 403 | Forbidden - Insufficient permissions |
| 404 | Not Found - Document doesn't exist |
| 409 | Conflict - Document already exists |
| 429 | Too Many Requests - Rate limit exceeded |
| 500 | Internal Server Error |

## 6. Webhooks

Register webhooks for events:

```http
POST /webhooks
{
  "url": "https://your-app.com/webhook",
  "events": ["document.created", "document.verified", "document.revoked"]
}
```

Webhook payload:
```json
{
  "event": "document.created",
  "documentId": "...",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {...}
}
```

---

© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for public-document is evaluated across three tiers:

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

- `wia-standards/standards/public-document/api/` — TypeScript SDK skeleton
- `wia-standards/standards/public-document/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/public-document/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-2-API

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API.

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



---

## Annex F — Operations and lifecycle notes

This informative annex captures operational guidance that has emerged from reference implementations and is expected to migrate into the normative body in a future minor revision.

### F.1 Deprecation policy

When a member of a canonical schema is deprecated, the schema MUST continue to accept and emit the member for at least 12 months from the publication of the deprecation notice. During the deprecation window, the implementation SHOULD emit a `Deprecation` HTTP header per the IETF deprecation-header draft for any response that contains the deprecated member, and SHOULD provide a `Sunset` header indicating the planned removal date per IETF RFC 8594.

### F.2 Backwards-compatible extensions

Vendors who extend a canonical schema with their own members MUST namespace those members with a reverse-DNS prefix, MUST treat the extension as opt-in, and MUST NOT shadow any normative member name reserved for future minor revisions of the standard.

### F.3 Operational telemetry

Every conformant deployment SHOULD expose a small set of operational metrics aligned with the OpenTelemetry semantic conventions. The recommended metric names are `wia.<slug>.requests.duration`, `wia.<slug>.requests.errors`, and `wia.<slug>.records.in_flight`.

### F.4 Logging

Logs MUST NOT contain unredacted authentication tokens, raw evidence pointers that the deployment does not own, or any plaintext personal data. Implementations SHOULD adopt structured JSON logging and SHOULD include the W3C Trace Context `trace_id` in every log line so that logs can be joined to the distributed-tracing graph.


---

## Annex G — Conformance attestation template

This informative annex offers a recommended template that conformant deployments may use when publishing their conformance attestation. The template is JSON Schema 2020-12 and is published under the WIA-Official catalogue.

```json
{
  "$id": "https://wiastandards.com/templates/conformance-attestation.json",
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["standard", "tier", "issued_at", "valid_until", "attesting_party", "evidence"],
  "properties": {
    "standard": { "type": "string", "format": "uri" },
    "tier": { "enum": ["tier-1", "tier-2", "tier-3"] },
    "issued_at": { "type": "string", "format": "date-time" },
    "valid_until": { "type": "string", "format": "date-time" },
    "attesting_party": { "type": "string", "format": "uri" },
    "evidence": { "type": "array", "minItems": 1, "items": { "type": "string", "format": "uri" } },
    "remarks": { "type": "string" }
  }
}
```

The template intentionally omits any sector-specific fields. Sector profiles SHOULD extend the template via JSON Schema composition (`allOf`) rather than redefinition.
