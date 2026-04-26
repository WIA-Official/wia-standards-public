# WIA-ENE-057: Desertification Prevention
## PHASE 3 - Protocol Specification

**Version:** 1.0.0
**Status:** Standard
**Last Updated:** 2025-12-25

---

## Overview

This document defines the communication protocols, data exchange standards, and interoperability requirements for the WIA-ENE-057 Desertification Prevention system. It ensures seamless integration between monitoring sensors, data platforms, and global conservation networks.

**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

---

## 1. Core Protocol Architecture

### 1.1 Protocol Stack

```
┌─────────────────────────────────────────┐
│   Application Layer (RESTful API)      │
├─────────────────────────────────────────┤
│   Transport Layer (HTTPS/WSS)          │
├─────────────────────────────────────────┤
│   Data Format Layer (JSON/GeoJSON)     │
├─────────────────────────────────────────┤
│   Authentication Layer (OAuth 2.0/JWT)  │
├─────────────────────────────────────────┤
│   Network Layer (TCP/IP)                │
└─────────────────────────────────────────┘
```

### 1.2 Protocol Principles

1. **Stateless Communication:** Each request contains all necessary information
2. **Idempotency:** Repeated requests produce the same result
3. **Versioning:** API version included in URL path
4. **Content Negotiation:** Support for multiple response formats
5. **Error Resilience:** Graceful degradation and retry mechanisms

---

## 2. Data Exchange Protocols

### 2.1 RESTful HTTP Protocol

**Base Principles:**
- **Resource-Oriented:** URLs represent resources, not actions
- **HTTP Methods:** GET (read), POST (create), PUT (update), DELETE (remove)
- **Status Codes:** Standard HTTP status codes for responses
- **HATEOAS:** Hypermedia links for API navigation

**Example Resource URL:**
```
/api/v1/monitoring/vegetation/{locationId}
/api/v1/assessment/desertification/{locationId}
/api/v1/restoration/plan/{planId}
```

**Request Headers:**
```http
GET /api/v1/monitoring/vegetation/SAHEL-REGION-001 HTTP/1.1
Host: api.wia-ene-057.org
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json
Accept-Encoding: gzip, deflate
User-Agent: WIA-ENE-057-Client/1.0.0
X-Request-ID: req-20251225-abc123
```

**Response Headers:**
```http
HTTP/1.1 200 OK
Content-Type: application/json; charset=utf-8
Content-Length: 1234
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-Request-ID: req-20251225-abc123
Cache-Control: max-age=3600
ETag: "33a64df551425fcc55e4d42a148795d9f25f89d4"
```

### 2.2 WebSocket Protocol (Real-time Streaming)

**Connection Handshake:**
```http
GET /v1/stream HTTP/1.1
Host: api.wia-ene-057.org
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
```

**Authentication Message:**
```json
{
  "type": "auth",
  "version": "1.0.0",
  "token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

**Subscription Message:**
```json
{
  "type": "subscribe",
  "subscriptionId": "sub-20251225-001",
  "locations": ["SAHEL-REGION-001", "SAHEL-REGION-002"],
  "dataTypes": ["vegetation", "soil", "alerts"],
  "filters": {
    "minSeverity": "medium",
    "metrics": ["ndvi", "soilMoisture"]
  }
}
```

**Data Stream Message:**
```json
{
  "type": "data",
  "messageId": "msg-20251225-001",
  "timestamp": "2025-12-25T10:00:00Z",
  "subscriptionId": "sub-20251225-001",
  "dataType": "vegetation",
  "locationId": "SAHEL-REGION-001",
  "payload": {
    "ndvi": 0.35,
    "evi": 0.28,
    "change": -0.02,
    "trend": "declining"
  }
}
```

**Heartbeat Protocol:**
```json
{
  "type": "ping",
  "timestamp": "2025-12-25T10:00:00Z"
}
```

**Response:**
```json
{
  "type": "pong",
  "timestamp": "2025-12-25T10:00:01Z"
}
```

---

## 3. Sensor Data Ingestion Protocol

### 3.1 IoT Device Protocol (MQTT)

**Topic Structure:**
```
wia-ene-057/{locationId}/{sensorType}/{metric}

Examples:
wia-ene-057/SAHEL-REGION-001/soil/moisture
wia-ene-057/SAHEL-REGION-001/weather/rainfall
wia-ene-057/SAHEL-REGION-001/vegetation/ndvi
```

**Message Format (JSON):**
```json
{
  "deviceId": "SENSOR-SOIL-001",
  "locationId": "SAHEL-REGION-001",
  "timestamp": "2025-12-25T10:00:00Z",
  "sensorType": "soil-moisture",
  "readings": {
    "moisture": 12.5,
    "temperature": 28.5,
    "depth": "0-10cm"
  },
  "metadata": {
    "battery": 85,
    "signalStrength": -65,
    "firmware": "1.2.3"
  }
}
```

**QoS Levels:**
- **QoS 0:** At most once (telemetry data)
- **QoS 1:** At least once (important measurements)
- **QoS 2:** Exactly once (critical alerts)

### 3.2 Satellite Data Ingestion (GeoTIFF/NetCDF)

**Protocol:** HTTPS with multipart/form-data

**Request:**
```http
POST /api/v1/ingest/satellite
Content-Type: multipart/form-data; boundary=----WebKitFormBoundary
Authorization: Bearer YOUR_API_KEY

------WebKitFormBoundary
Content-Disposition: form-data; name="metadata"
Content-Type: application/json

{
  "locationId": "SAHEL-REGION-001",
  "satellite": "MODIS",
  "instrument": "MODIS-Terra",
  "product": "MOD13Q1",
  "acquisitionDate": "2025-12-25T10:30:00Z",
  "resolution": "250m",
  "bands": ["NDVI", "EVI"],
  "cloudCover": 5.2,
  "crs": "EPSG:4326"
}
------WebKitFormBoundary
Content-Disposition: form-data; name="file"; filename="SAHEL-001-20251225.tif"
Content-Type: image/tiff

[Binary GeoTIFF data]
------WebKitFormBoundary--
```

---

## 4. Interoperability Protocols

### 4.1 OGC Web Services

**WMS (Web Map Service) - GetCapabilities:**
```http
GET /wms?
  service=WMS&
  version=1.3.0&
  request=GetCapabilities
```

**WMS - GetMap:**
```http
GET /wms?
  service=WMS&
  version=1.3.0&
  request=GetMap&
  layers=desertification-risk&
  styles=&
  crs=EPSG:4326&
  bbox=14.0,-5.0,15.0,-3.0&
  width=800&
  height=600&
  format=image/png
```

**WFS (Web Feature Service) - GetFeature:**
```http
GET /wfs?
  service=WFS&
  version=2.0.0&
  request=GetFeature&
  typeName=wia-ene-057:vegetation-monitoring&
  outputFormat=application/json&
  cql_filter=locationId='SAHEL-REGION-001'
```

**WCS (Web Coverage Service) - GetCoverage:**
```http
GET /wcs?
  service=WCS&
  version=2.0.1&
  request=GetCoverage&
  coverageId=ndvi-coverage&
  subset=Lat(14.0,15.0)&
  subset=Long(-5.0,-3.0)&
  format=image/tiff
```

### 4.2 GeoJSON Feature Protocol

**Response Format:**
```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "id": "SAHEL-REGION-001",
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [-4.0, 14.0],
            [-3.0, 14.0],
            [-3.0, 15.0],
            [-4.0, 15.0],
            [-4.0, 14.0]
          ]
        ]
      },
      "properties": {
        "locationId": "SAHEL-REGION-001",
        "name": "Sahel Test Region",
        "area": 10000,
        "areaUnit": "hectares",
        "desertificationRisk": 68.5,
        "riskCategory": "high-risk",
        "vegetationCover": 45.2,
        "ndvi": 0.35,
        "lastUpdate": "2025-12-25T10:00:00Z"
      }
    }
  ],
  "crs": {
    "type": "name",
    "properties": {
      "name": "EPSG:4326"
    }
  }
}
```

---

## 5. Security Protocols

### 5.1 TLS/SSL Protocol

**Minimum Requirements:**
- **TLS Version:** TLS 1.2 or higher
- **Cipher Suites:** Strong ciphers only (AES-256, ChaCha20)
- **Certificate:** Valid X.509 certificate from trusted CA
- **Perfect Forward Secrecy:** Required (ECDHE, DHE)

**Example Configuration:**
```
TLSv1.2+
ECDHE-RSA-AES256-GCM-SHA384
ECDHE-RSA-CHACHA20-POLY1305
DHE-RSA-AES256-GCM-SHA384
```

### 5.2 OAuth 2.0 Protocol Flow

**Authorization Code Flow:**

1. **Authorization Request:**
```http
GET /oauth/authorize?
  response_type=code&
  client_id=YOUR_CLIENT_ID&
  redirect_uri=https://your-app.com/callback&
  scope=monitoring:read restoration:write&
  state=xyz123
```

2. **Authorization Response:**
```http
HTTP/1.1 302 Found
Location: https://your-app.com/callback?
  code=AUTH_CODE&
  state=xyz123
```

3. **Token Request:**
```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded
Authorization: Basic BASE64(client_id:client_secret)

grant_type=authorization_code&
code=AUTH_CODE&
redirect_uri=https://your-app.com/callback
```

4. **Token Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "scope": "monitoring:read restoration:write"
}
```

### 5.3 JWT Token Format

**Header:**
```json
{
  "alg": "RS256",
  "typ": "JWT",
  "kid": "wia-ene-057-key-1"
}
```

**Payload:**
```json
{
  "iss": "https://auth.wia-ene-057.org",
  "sub": "user-12345",
  "aud": "https://api.wia-ene-057.org",
  "exp": 1735128000,
  "iat": 1735124400,
  "scope": "monitoring:read restoration:write",
  "organization": "Great Green Wall Foundation",
  "role": "project-manager"
}
```

---

## 6. Data Synchronization Protocol

### 6.1 Event-Driven Synchronization

**Event Types:**
- `data.created` - New data record created
- `data.updated` - Existing data updated
- `data.deleted` - Data record deleted
- `alert.triggered` - Alert condition met
- `assessment.completed` - Risk assessment finished

**Event Message Format:**
```json
{
  "eventId": "evt-20251225-001",
  "eventType": "data.created",
  "timestamp": "2025-12-25T10:00:00Z",
  "source": "wia-ene-057-monitoring",
  "version": "1.0.0",
  "data": {
    "resourceType": "vegetation-monitoring",
    "resourceId": "VEG-20251225-001",
    "locationId": "SAHEL-REGION-001",
    "action": "created",
    "payload": {
      "ndvi": 0.35,
      "evi": 0.28,
      "timestamp": "2025-12-25T10:00:00Z"
    }
  },
  "metadata": {
    "organizationId": "org-12345",
    "userId": "user-67890",
    "correlationId": "corr-abc123"
  }
}
```

### 6.2 Webhook Delivery Protocol

**Webhook Registration:**
```json
{
  "webhookUrl": "https://your-domain.com/webhooks/wia-ene-057",
  "events": ["data.created", "alert.triggered"],
  "filters": {
    "locationIds": ["SAHEL-REGION-001"],
    "severity": ["high", "critical"]
  },
  "secret": "webhook-secret-key-for-signature"
}
```

**Webhook Delivery:**
```http
POST /webhooks/wia-ene-057 HTTP/1.1
Host: your-domain.com
Content-Type: application/json
X-WIA-Signature: sha256=abc123...
X-WIA-Event: alert.triggered
X-WIA-Delivery: delivery-20251225-001

{
  "eventId": "evt-20251225-001",
  "eventType": "alert.triggered",
  ...
}
```

**Signature Verification:**
```javascript
const crypto = require('crypto');

function verifyWebhookSignature(payload, signature, secret) {
  const hmac = crypto.createHmac('sha256', secret);
  const digest = 'sha256=' + hmac.update(payload).digest('hex');
  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(digest)
  );
}
```

---

## 7. Batch Processing Protocol

### 7.1 Bulk Data Upload

**Multipart Upload Initiation:**
```http
POST /api/v1/bulk/upload/initiate
Content-Type: application/json

{
  "dataType": "vegetation-monitoring",
  "recordCount": 10000,
  "estimatedSize": 52428800,
  "format": "json"
}
```

**Response:**
```json
{
  "uploadId": "upload-20251225-001",
  "chunkSize": 5242880,
  "expiresAt": "2025-12-26T10:00:00Z",
  "uploadUrl": "/api/v1/bulk/upload/upload-20251225-001/chunk"
}
```

**Chunk Upload:**
```http
POST /api/v1/bulk/upload/upload-20251225-001/chunk
Content-Type: application/octet-stream
X-Chunk-Number: 1
X-Chunk-Total: 10

[Binary chunk data]
```

**Upload Completion:**
```http
POST /api/v1/bulk/upload/upload-20251225-001/complete
Content-Type: application/json

{
  "uploadId": "upload-20251225-001",
  "chunksReceived": 10,
  "checksums": ["sha256-chunk1", "sha256-chunk2", ...]
}
```

### 7.2 Batch Processing Status

**Status Check:**
```http
GET /api/v1/bulk/upload/upload-20251225-001/status
```

**Response:**
```json
{
  "uploadId": "upload-20251225-001",
  "status": "processing",
  "progress": {
    "totalRecords": 10000,
    "processed": 7500,
    "successful": 7450,
    "failed": 50,
    "percentage": 75.0
  },
  "errors": [
    {
      "recordNumber": 245,
      "error": "Invalid NDVI value: 1.5"
    }
  ],
  "estimatedCompletion": "2025-12-25T11:00:00Z"
}
```

---

## 8. Caching Protocol

### 8.1 HTTP Cache Headers

**Cache-Control Directives:**
```http
Cache-Control: public, max-age=3600, s-maxage=7200
Cache-Control: private, no-cache, must-revalidate
Cache-Control: no-store
```

**ETag and Conditional Requests:**
```http
GET /api/v1/monitoring/vegetation/SAHEL-REGION-001
If-None-Match: "33a64df551425fcc55e4d42a148795d9f25f89d4"

Response:
HTTP/1.1 304 Not Modified
ETag: "33a64df551425fcc55e4d42a148795d9f25f89d4"
```

**Last-Modified:**
```http
GET /api/v1/assessment/desertification/SAHEL-REGION-001
If-Modified-Since: Wed, 25 Dec 2025 09:00:00 GMT

Response:
HTTP/1.1 304 Not Modified
Last-Modified: Wed, 25 Dec 2025 09:00:00 GMT
```

---

## 9. Error Handling Protocol

### 9.1 Retry Strategy

**Exponential Backoff:**
```
Retry Delay = min(max_delay, base_delay * 2^attempt)

Example:
Attempt 1: 1 second
Attempt 2: 2 seconds
Attempt 3: 4 seconds
Attempt 4: 8 seconds
Attempt 5: 16 seconds
```

**Retry Headers:**
```http
HTTP/1.1 503 Service Unavailable
Retry-After: 120

HTTP/1.1 429 Too Many Requests
Retry-After: Wed, 25 Dec 2025 11:00:00 GMT
```

### 9.2 Circuit Breaker Pattern

**States:**
1. **Closed:** Normal operation, requests flow through
2. **Open:** Failures exceeded threshold, requests fail fast
3. **Half-Open:** Test request to check if service recovered

**Configuration:**
```json
{
  "failureThreshold": 5,
  "timeout": 30000,
  "resetTimeout": 60000,
  "monitoringPeriod": 10000
}
```

---

## 10. Versioning Protocol

### 10.1 API Versioning Strategy

**URL Path Versioning:**
```
https://api.wia-ene-057.org/v1/monitoring/vegetation
https://api.wia-ene-057.org/v2/monitoring/vegetation
```

**Header Versioning:**
```http
GET /api/monitoring/vegetation
Accept: application/vnd.wia-ene-057.v1+json
```

**Deprecation Notice:**
```http
HTTP/1.1 200 OK
Deprecation: true
Sunset: Wed, 31 Dec 2026 23:59:59 GMT
Link: </api/v2/monitoring/vegetation>; rel="alternate"
```

---

## 11. Compliance & Standards

### 11.1 Protocol Standards Compliance

- **HTTP/1.1:** RFC 7230-7235 ✓
- **HTTP/2:** RFC 7540 ✓
- **WebSocket:** RFC 6455 ✓
- **OAuth 2.0:** RFC 6749 ✓
- **JWT:** RFC 7519 ✓
- **GeoJSON:** RFC 7946 ✓
- **MQTT:** MQTT v3.1.1, v5.0 ✓

### 11.2 Security Standards

- **TLS:** RFC 8446 (TLS 1.3) ✓
- **OAuth 2.0 Security:** RFC 6749, RFC 8252 ✓
- **CORS:** W3C CORS Specification ✓
- **CSP:** Content Security Policy Level 3 ✓

---

**Document Control:**
- **Author:** WIA Standards Committee
- **Review Date:** 2025-12-25
- **Next Review:** 2026-06-25
- **Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
