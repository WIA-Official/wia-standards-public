# WIA-SPACE-003 (satellite-communication) — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 2 of 4 (API Interface)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This phase defines the network-facing API of WIA-SPACE-003. Two complementary surfaces are specified:

- **HTTP/REST** — for management plane operations (catalog, contact-window planning, link-budget queries, service-plan administration), built on RFC 9110 (HTTP semantics) and RFC 9112 / RFC 9113 / RFC 9114, with TLS 1.3 (RFC 8446) as the only permitted carrier.
- **CoAP** — for constrained user terminals (IoT-class VSATs, hand-held terminals), built on RFC 7252 with OBSERVE (RFC 7641), block transfer (RFC 7959), and DTLS 1.3 (RFC 9147) or OSCORE (RFC 8613) for security.

OpenAPI 3.1 documents the HTTP/REST surface. The CoAP surface uses a CoRE Resource Directory descriptor (RFC 9176).

### 1.1 Authentication and authorization

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Token format | JWT with PoP, or CWT for constrained nodes | RFC 7519, RFC 7800; RFC 8392 |
| Authorization framework | OAuth 2.1 | RFC 9700 (BCP) |
| Constrained-device flow | ACE-OAuth | RFC 9200, RFC 9202 (DTLS), RFC 9203 (OSCORE) |
| Channel security (HTTP) | TLS 1.3 | RFC 8446 |
| Channel security (CoAP) | DTLS 1.3 or OSCORE | RFC 9147; RFC 8613 |
| At-rest token integrity | COSE_Sign1 | RFC 9052 |

Tokens MUST carry an `aud` claim equal to the canonical deployment URI from Phase 1 §2.1, and a `scope` string composed of one or more verbs from §4 below.

---

## 2. HTTP/REST Surface

### 2.1 Base URL

```
https://<host>/wia-space-003/v1
```

Servers MUST advertise version 1 of this specification through the `Server` header and the `/.well-known/wia-space` document (RFC 8615 Well-Known URIs).

### 2.2 Resource map

| Resource | Methods | Purpose |
|----------|---------|---------|
| `/deployments` | GET | List deployments visible to caller |
| `/deployments/{id}` | GET, PATCH | Deployment descriptor |
| `/deployments/{id}/spacecraft` | GET, POST | Spacecraft catalog |
| `/deployments/{id}/spacecraft/{scId}` | GET, PATCH, DELETE | Spacecraft descriptor |
| `/deployments/{id}/spacecraft/{scId}/orbit` | GET, PATCH | Orbit elements |
| `/deployments/{id}/spacecraft/{scId}/contact-windows` | GET | Future contact windows for a station |
| `/deployments/{id}/ground-stations` | GET, POST | Ground-station catalog |
| `/deployments/{id}/ground-stations/{gsId}` | GET, PATCH, DELETE | Ground-station descriptor |
| `/deployments/{id}/user-terminals` | GET, POST | User-terminal catalog |
| `/deployments/{id}/links` | GET, POST | Link catalog |
| `/deployments/{id}/links/{linkId}/budget` | GET, POST | Link-budget calculation |
| `/deployments/{id}/bundle-sessions` | GET, POST | Bundle sessions |
| `/deployments/{id}/bundle-sessions/{bsId}/bundles` | GET, POST | Bundle traffic |
| `/deployments/{id}/telemetry-sessions` | GET, POST | Telemetry / tele-command sessions |
| `/deployments/{id}/service-plans` | GET, POST | Service plans |
| `/deployments/{id}/audit` | GET | Audit log |
| `/deployments/{id}/health` | GET | Liveness / readiness |

`PATCH` requests use JSON Merge Patch (RFC 7396) by default and JSON Patch (RFC 6902) when `Content-Type: application/json-patch+json`.

### 2.3 Conditional requests

ETag/If-Match/If-None-Match per RFC 9110 §13. Strong validators required for any state-changing request.

### 2.4 Errors

Errors use *Problem Details for HTTP APIs* (RFC 9457). The minimum problem types:

| Code | HTTP | Meaning |
|------|------|---------|
| `space/forbidden-zone` | 403 | Caller lacks IEC 62443 zone privilege |
| `space/spacecraft-out-of-coverage` | 409 | No contact window in the requested horizon |
| `space/link-budget-infeasible` | 422 | Operator policy threshold not met |
| `space/regulatory-filing-missing` | 422 | Required regulatory record absent |
| `space/service-plan-quota-exceeded` | 429 | Tenant exceeded contracted plan |
| `space/bundle-lifetime-expired` | 410 | Bundle expired before delivery |

### 2.5 Pagination

Cursor-based pagination is mandatory. Servers return `Link` headers per RFC 8288 with `rel="next"` and `rel="prev"`.

### 2.6 Long-running operations

Bulk imports, link-budget Monte Carlo runs, and large contact-window scans are long-running. They follow `202 Accepted` + status-URI pattern, with optional `Prefer: respond-async` (RFC 7240).

---

## 3. CoAP Surface

### 3.1 URI scheme

```
coaps://<gateway-host>/wia-space/v1/{path}
```

Resource paths mirror the HTTP surface but elide `deployments/{id}` because each gateway is bound to a single deployment.

### 3.2 Methods and options

CoAP methods (RFC 7252 §5.8) map directly: GET, POST, PUT, DELETE. Resource representations use CBOR (RFC 8949) with COSE_Sign1 / COSE_Encrypt0 wrappers (RFC 9052 / 9053) when end-to-end protection is required.

OBSERVE (RFC 7641) MUST be supported for telemetry resources:

```
GET coaps://gw/wia-space/v1/spacecraft/sc-001/state
Observe: 0
```

Block-wise transfers (RFC 7959) MUST be supported for any payload exceeding 1024 bytes.

### 3.3 Resource Directory

Gateways MUST register their resource set with a CoRE Resource Directory (RFC 9176). Resource types:

- `rt="wia.s.spacecraft"` — spacecraft endpoint
- `rt="wia.s.terminal"` — user-terminal endpoint
- `rt="wia.s.link"` — link endpoint
- `if="wia.s.observe"` — telemetry observe interface

---

## 4. Verbs and Scopes

| Verb | Capability | Surface(s) |
|------|------------|------------|
| `space:read` | Read catalog | HTTP, CoAP |
| `space:author-spacecraft` | Add/edit spacecraft | HTTP |
| `space:author-ground-station` | Add/edit ground stations | HTTP |
| `space:author-user-terminal` | Add/edit user terminals | HTTP |
| `space:plan-contact` | Compute contact windows | HTTP |
| `space:author-link-budget` | Compute / persist link budgets | HTTP |
| `space:bundle-send` | Submit bundles for transmission | HTTP, CoAP |
| `space:bundle-receive` | Receive bundles | HTTP, CoAP |
| `space:tc-send` | Send tele-commands | HTTP |
| `space:tm-receive` | Receive telemetry | HTTP, CoAP |
| `space:audit-read` | Read audit log | HTTP |
| `space:plan-administration` | Manage service plans | HTTP |

Each verb MUST be enforced at every surface. Tokens missing a required verb produce HTTP 403 / CoAP 4.03 with the `space/forbidden-zone` problem code.

---

## 5. Sample Operations

### 5.1 Compute a link budget (HTTP)

```
POST /wia-space-003/v1/deployments/dp-1/links/lk-12/budget HTTP/1.1
Host: gateway.example.org
Authorization: DPoP <token>
Content-Type: application/json

{"weather":"CLEAR","atmosphericLossDb":1.2,"rainAttenuationDb":0.0}
```

Response:

```
HTTP/1.1 200 OK
ETag: "BUD-2026-0432"
Content-Type: application/json

{"cnrDb": 12.3, "marginDb": 4.7, "feasible": true}
```

### 5.2 Compute contact windows

```
POST /wia-space-003/v1/deployments/dp-1/spacecraft/sc-001/contact-windows:compute HTTP/1.1
Content-Type: application/json

{"groundStationId":"gs-seoul-1","horizon":{"from":"2026-04-26T00:00Z","to":"2026-04-27T00:00Z"},"minElevationDeg":10}
```

### 5.3 Submit a bundle (HTTP)

```
POST /wia-space-003/v1/deployments/dp-1/bundle-sessions/bs-7/bundles HTTP/1.1
Content-Type: application/cbor
```

The body is a CBOR representation of an RFC 9171 BPv7 bundle. The response carries an audit reference and the eventual transmission window.

---

## 6. Conformance

A WIA-SPACE-003 v1 *gateway* MUST implement:

- HTTP/REST surface in §2 with TLS 1.3.
- Verbs from §4 with audit logging on every state-changing call.
- Problem details from §2.4.
- BPv7 (RFC 9171) ingress and egress when the deployment uses bundle convergence.

A WIA-SPACE-003 v1 *client* MUST:

- Honour ETag-based concurrency.
- Tolerate unknown problem-detail codes per RFC 9457 §4.4.
- Handle CoAP block-wise transfers (RFC 7959) for any payload exceeding 1024 bytes.

---

## 7. Operational Notes

### 7.1 Idempotency

Mutating verbs accept `Idempotency-Key`. Servers retain the response per key for 24 hours.

### 7.2 Rate limiting

Token-bucket rate limiting with the IETF `RateLimit-*` headers. Tele-command rate limits MUST be configured per spacecraft and per operator policy; the surface MUST refuse tele-commands that would exceed the spacecraft's safe-rate envelope.

### 7.3 Discovery

`/deployments/{id}/.well-known/wia-space` returns supported verbs, supported encodings, supported convergence layers, and links to OpenAPI / CoAP-RD endpoints.

### 7.4 Health endpoint

`/deployments/{id}/health` MUST return `{state, version, uptimeSeconds, spacecraft:{total,reachable}, links:{active,faulted}, bundles:{queued,inFlight,deliveredLast24h}, audit:{lastEntryAt,queueDepth}}`. The `state` and `version` fields MUST be served unauthenticated; full detail requires `space:read`.

---

## 8. References

1. RFC 6902; RFC 7396 — *JSON Patch / Merge Patch.*
2. RFC 7240 — *Prefer Header for HTTP.*
3. RFC 7252; RFC 7641; RFC 7959 — *CoAP family.*
4. RFC 7519; RFC 7800; RFC 8392 — *JWT, PoP, CWT.*
5. RFC 8259; RFC 8610; RFC 8615; RFC 8949 — *JSON, CDDL, well-known URIs, CBOR.*
6. RFC 8288 — *Web Linking.*
7. RFC 8446; RFC 9147 — *TLS 1.3, DTLS 1.3.*
8. RFC 8613 — *OSCORE.*
9. RFC 9052; RFC 9053 — *COSE.*
10. RFC 9110; RFC 9112; RFC 9113; RFC 9114 — *HTTP family.*
11. RFC 9171; RFC 9173 — *BPv7, BPv7 default security context.*
12. RFC 9176 — *CoRE Resource Directory.*
13. RFC 9200; RFC 9202; RFC 9203 — *ACE-OAuth and profiles.*
14. RFC 9457 — *Problem Details for HTTP APIs.*
15. RFC 9700 — *OAuth 2.1.*
16. ISO/IEC 27001:2022; ISO/IEC 27002:2022.
17. IEC 62443-3-3:2013.

---

## 9. Discovery Document Schema

The `/.well-known/wia-space` discovery document MUST conform to the following JSON schema fragment:

```json
{
  "version": "string (semver)",
  "supportedVerbs": ["array of verb tokens from §4"],
  "supportedEncodings": ["json", "cbor"],
  "supportedConvergenceLayers": ["TCPCL", "UDPCL", "LTPCL"],
  "supportedBands": ["array of band labels from Phase 1 §7"],
  "openApiUrl": "string (URI)",
  "coreResourceDirectoryUrl": "string (URI) | null",
  "bundleProtocolVersion": 7,
  "conformanceTags": ["array of conformance tags from Phase 4 §10"]
}
```

The discovery document SHOULD be cached for 60 seconds by clients; the engine MUST emit `Cache-Control: max-age=60`.

---

## 10. Resumable Uploads and Downloads

Bundle traffic and large telemetry archives use resumable uploads and downloads:

- Uploads: PUT with `Upload-Offset` and `Upload-Length` headers per the IETF *Resumable Uploads* draft profile. Servers MAY also accept a single multipart/form-data POST when the body is small.
- Downloads: GET with `Range` headers per RFC 9110 §14. Servers MUST advertise `Accept-Ranges: bytes`.

Both directions support the IANA *Tus protocol* values where the server has registered them; the surface itself is media-type-agnostic.

---

## 11. Versioning Policy

Major version bumps require a parallel HTTP path (`/wia-space-003/v2`). Minor and patch revisions are backwards-compatible within `/v1`. Clients MUST tolerate unknown fields and unknown problem-detail extensions per RFC 9457 §4.4. Breaking changes to bundle ingress / egress formats require a major version bump even when the catalog formats are unchanged.

---

## 12. Backpressure Profile

Servers MUST publish a backpressure profile describing how the surface behaves under load:

- Audit queue depth threshold (HTTP 503 + `Retry-After`).
- Bundle queue depth threshold per session (HTTP 429 + `Retry-After`).
- Tele-command rate limit per spacecraft (HTTP 429 + `Retry-After`).

The profile lives at `/.well-known/wia-space-backpressure` and MUST be observable by tenants for capacity planning.
