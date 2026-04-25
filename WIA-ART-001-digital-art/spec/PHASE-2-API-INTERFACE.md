# WIA-ART-001: Phase 2 - API Interface Specification

**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 specifies the programmatic interfaces (APIs) for digital-art creation, validation, signing, transformation, and distribution under WIA-ART-001. The API surface is published as an OpenAPI 3.1.0 document and follows the conventions documented in §10 (Reference Standards Alignment) for transport, encoding, authentication, and error handling.

The Phase 2 design treats interoperability as a first-class concern: any conformant client SHOULD be able to drive any conformant server without bespoke integration work, given a standard authentication credential and an OpenAPI document.

---

## 2. TypeScript SDK Interface

```typescript
export interface DigitalArtConfig {
  apiKey?: string;
  endpoint?: string;
  version?: string;
  acceptLanguage?: string;
  timeoutMs?: number;
}

export interface ArtworkPayload {
  title: string;
  description?: string;
  medium: 'raster' | 'vector' | 'three_d' | 'video' | 'audio' | 'mixed';
  format: string;          // canonical format identifier (PNG, SVG, glTF, WebM, ...)
  bytes?: Uint8Array | string; // optional inline payload
  uri?: string;            // optional fetch-by-URI
  metadata?: ArtworkMetadata;
}

export interface ArtworkMetadata {
  authors: AuthorIdentity[];
  creation_time_iso8601: string;  // ISO 8601:2019
  rights?: RightsClaim;
  provenance?: ProvenanceLink[];
  language?: string;       // BCP 47 (RFC 5646)
}

export interface AuthorIdentity {
  display_name: string;
  did?: string;            // W3C DID Core 1.0
  verified?: boolean;
}

export interface RightsClaim {
  license_id: string;       // SPDX identifier where applicable
  attribution_required: boolean;
  commercial_use_allowed: boolean;
  derivatives_allowed: boolean;
}

export interface ValidationResult {
  ok: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
}

export interface DigitalArtSDK {
  create(payload: ArtworkPayload): Promise<{ id: string; manifest_uri: string }>;
  validate(payload: ArtworkPayload): Promise<ValidationResult>;
  sign(id: string, key_ref: string): Promise<{ signature: string; manifest_uri: string }>;
  verify(id: string): Promise<{ valid: boolean; details: object }>;
  get(id: string): Promise<ArtworkPayload>;
  search(query: ArtworkSearchQuery): Promise<ArtworkSearchPage>;
  export(id: string, options: ExportOptions): Promise<{ uri: string; format: string }>;
}
```

The SDK is published in TypeScript with parallel implementations in Python, Go, Rust, and Swift. SDK behaviour is defined by the OpenAPI document; SDKs are convenience wrappers and not normative.

---

## 3. REST API Endpoints

### 3.1 Artwork lifecycle

| Method | Path | Purpose |
|--------|------|---------|
| POST | `/api/v1/artworks` | Create artwork |
| GET | `/api/v1/artworks/{id}` | Retrieve artwork |
| PATCH | `/api/v1/artworks/{id}` | Update artwork (limited fields) |
| DELETE | `/api/v1/artworks/{id}` | Soft-delete artwork |
| POST | `/api/v1/artworks/validate` | Validate artwork without persisting |
| POST | `/api/v1/artworks/{id}/sign` | Sign artwork manifest |
| GET | `/api/v1/artworks/{id}/verify` | Verify signature and provenance |
| GET | `/api/v1/artworks/{id}/manifest` | Retrieve canonical signed manifest |
| GET | `/api/v1/artworks/{id}/export` | Export to specified format |

Create example:

```http
POST /api/v1/artworks HTTP/1.1
Host: api.wia-art.org
Content-Type: application/json
Authorization: Bearer eyJhbGciOiJFZERTQSIsImtpZCI6ImtleS0xIn0...

{
  "title": "Phase Drift",
  "medium": "raster",
  "format": "image/png",
  "uri": "https://cdn.example.com/art/phase-drift.png",
  "metadata": {
    "authors": [{"display_name": "Maya Tanaka", "did": "did:web:tanaka.art"}],
    "creation_time_iso8601": "2026-04-26T08:14:32Z",
    "rights": {
      "license_id": "CC-BY-4.0",
      "attribution_required": true,
      "commercial_use_allowed": true,
      "derivatives_allowed": true
    },
    "language": "en"
  }
}
```

### 3.2 Search and discovery

```http
GET /api/v1/artworks?q=phase+drift&medium=raster&limit=50&cursor=eyJ... HTTP/1.1
```

Discovery responses follow the W3C DCAT v3 vocabulary when consumed as RDF and the schema.org `VisualArtwork` / `CreativeWork` types when consumed as JSON-LD.

### 3.3 Provenance and history

```http
GET /api/v1/artworks/{id}/provenance HTTP/1.1
```

Provenance responses follow the W3C PROV-O ontology so that downstream tools can reuse standard graph queries.

### 3.4 Bulk operations

Bulk creation supports up to 100 artworks per request:

```http
POST /api/v1/artworks/bulk HTTP/1.1
Content-Type: application/json
Idempotency-Key: 1c9f...
```

Bulk responses include per-item success status so the client can retry only the failures.

---

## 4. Authentication and Authorisation

The API supports three authentication schemes:

| Scheme | Reference | Use case |
|--------|-----------|----------|
| `bearerAuth` | RFC 6750 (Bearer Token) | API gateway tokens |
| `oauth2` | RFC 6749, with PKCE per RFC 7636 | Federated client identity |
| `mutualTLS` | RFC 8705 | Studio device attestation |

Bearer tokens follow JWT (RFC 7519). The signing algorithm is Ed25519 (RFC 8032) by default; ECDSA P-256 (NIST FIPS 186-5) is acceptable for legacy hardware. Required JWT claims: `iss`, `sub`, `exp`, `iat`. WIA-specific claims include `wia.role` and `wia.scope`.

OAuth 2.0 flows: Authorization Code with PKCE (RFC 7636) and Client Credentials (RFC 6749 §4.4) are supported. The Resource Owner Password Credentials grant is not supported for new deployments.

---

## 5. Error Handling

All error responses follow RFC 9457 *Problem Details for HTTP APIs*:

```json
{
  "type": "https://wia-art.org/errors/format-not-supported",
  "title": "Format Not Supported",
  "status": 415,
  "detail": "Format 'image/heic' is not supported by this endpoint.",
  "instance": "/api/v1/artworks/validate"
}
```

The Content-Type for error responses is `application/problem+json`.

---

## 6. Rate Limiting

Rate-limit headers follow IETF rate-limit-headers conventions:

```
RateLimit-Limit: 1000, 1000;w=60
RateLimit-Remaining: 947
RateLimit-Reset: 53
```

Throttled requests return HTTP 429 with `Retry-After` per RFC 9110.

---

## 7. Pagination

List endpoints use cursor-based pagination with the `Link` header per RFC 8288:

```
Link: <https://api.wia-art.org/api/v1/artworks?cursor=...&limit=50>; rel="next"
```

---

## 8. Locale

The `Accept-Language` request header (BCP 47, RFC 5646) selects the locale for any human-readable strings in responses. Server-side locale data follows the Unicode CLDR.

---

## 9. WebSocket and Server-Sent Events

For real-time updates (e.g., live collaborative sessions), the WebSocket transport per RFC 6455 is used. Subprotocol negotiation uses `Sec-WebSocket-Protocol: wia-art.v1`. Server-Sent Events per the W3C EventSource specification are an acceptable alternative for one-way event streams.

---

## 10. Reference Standards Alignment

| Concern | Reference |
|---------|-----------|
| HTTP semantics | RFC 9110 |
| HTTP/1.1 | RFC 9112 |
| HTTP/2 | RFC 9113 |
| HTTP/3 over QUIC | RFC 9114 / RFC 9000 |
| TLS 1.3 | RFC 8446 |
| JSON | RFC 8259 |
| OpenAPI | OpenAPI Specification 3.1 |
| Errors | RFC 9457 |
| Pagination linking | RFC 8288 |
| Bearer tokens | RFC 6750 |
| OAuth 2.0 | RFC 6749 + RFC 7636 (PKCE) |
| Mutual TLS | RFC 8705 |
| JWT | RFC 7519 |
| Ed25519 | RFC 8032 |
| ECDSA | NIST FIPS 186-5 |
| Hash | FIPS 180-4, FIPS 202 |
| WebSocket | RFC 6455 |
| Provenance | W3C PROV-O |
| Identity | W3C DID Core 1.0 |
| Verifiable credentials | W3C VC Data Model 2.0 |
| Catalog | W3C DCAT v3 |
| Locale | BCP 47 (RFC 5646), Unicode CLDR |
| Time | ISO 8601:2019 |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

---

## 11. Conformance

A Phase 2 implementation is conformant when:

1. The OpenAPI 3.1 description publishes every endpoint listed in §3 with request and response schemas.
2. Authentication accepts at least the `bearerAuth` scheme.
3. Errors use RFC 9457 problem-detail responses.
4. Pagination uses `Link` headers per RFC 8288.
5. Cryptographic primitives match §4 with explicit algorithm identifiers in tokens and signatures.

## 12. Implementation Appendix

### 12.1 Idempotency

Endpoints that create resources accept an `Idempotency-Key` request header containing a UUID. The server stores the response for at least 24 hours and returns the cached response on retry. This guards against duplicate artwork creation when a client's network connection is unreliable.

### 12.2 ETags and Conditional Requests

Mutating endpoints accept `If-Match` headers per RFC 9110 §13.1.1 carrying the artwork's manifest ETag, so concurrent updates do not silently overwrite each other. ETags are computed deterministically from the canonical manifest hash and are stable across replicas.

### 12.3 CORS

CORS responses follow the WHATWG Fetch Living Standard. The reference deployment publishes the allow-list of origins, methods, and headers and reviews it quarterly.

### 12.4 Header registry

Custom WIA headers are prefixed `X-Wia-` and registered:

- `X-Wia-Manifest-Uri` — URI of the canonical signed manifest for the response.
- `X-Wia-Manifest-Hash` — SHA-256 hash of the canonical manifest.
- `X-Wia-Trace-Id` — Trace identifier echoed for distributed tracing.

Implementations MUST NOT introduce additional `X-Wia-` headers without coordinating with the WIA registry custodian.

### 12.5 Streaming uploads

Large artwork uploads (video, high-resolution raster, multi-gigabyte 3D scenes) use chunked transfer per RFC 9112, with resumable upload semantics inspired by the tus.io resumable-upload protocol. Resumable uploads use a per-upload token that the server accepts on subsequent PUT requests until the upload is complete.

### 12.6 SDK discovery

SDK packages publish their metadata at well-known URLs so that clients can detect SDK version, supported endpoints, and known issues:

- `/.well-known/wia-art-001/sdk-info`
- `/.well-known/wia-art-001/openapi.json`

### 12.7 Backwards compatibility

When the API minor version increments, existing clients continue to operate against the prior minor's contract for at least 12 months. New optional fields appear in responses and are gracefully ignored by older clients per RFC 9110 §6.5 forward-compatibility expectations.

Major-version increments (`v1` → `v2`) coexist with the prior major for at least 18 months before deprecation. The deprecation calendar is published in the OpenAPI document and on the WIA standards-deprecation registry.

### 12.8 Webhook Conformance

When the API supports outbound webhooks (manifest-signed events, marketplace listings, catalog ingest), webhook conformance follows the same three-rule contract as Phase 2 API in the cryogenic-transport standard: HMAC-SHA-256 signatures over the payload, idempotent delivery identifiers, and exponential-backoff retries up to a configurable maximum. Receivers respond with 2xx within 15 seconds for acknowledgement.

---

**弘益人間 (Benefit All Humanity)**
*© 2025 WIA*
