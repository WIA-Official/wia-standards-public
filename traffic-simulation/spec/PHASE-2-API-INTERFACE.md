# WIA-CITY-017 (traffic-simulation) — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 2 of 4 (API Interface)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This phase defines the network-facing API of WIA-CITY-017. Two surfaces are specified:

- **HTTP/REST surface** — for project, network, scenario, run, and result management. Built on RFC 9110 (HTTP semantics) and RFC 9112 / RFC 9113 / RFC 9114, with TLS 1.3 (RFC 8446) as the only permitted carrier.
- **Streaming surface** — for live SPaT/MAP message replay or real-time engine telemetry. Streaming is delivered over HTTP/2 server-pushed events or RFC 8895-style server-sent events; for constrained roadside modules, the surface mirrors the ISO/TS 19091 SPaT message stream over CoAP (RFC 7252) with OBSERVE (RFC 7641).

OpenAPI 3.1 documents the HTTP/REST surface.

### 1.1 Authentication and authorization

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Token format | JWT with PoP, or CWT for constrained nodes | RFC 7519, RFC 7800; RFC 8392 |
| Authorization framework | OAuth 2.1 | RFC 9700 (BCP) |
| Constrained-device flow | ACE-OAuth | RFC 9200, RFC 9202, RFC 9203 |
| Channel security (HTTP) | TLS 1.3 | RFC 8446 |
| Channel security (CoAP) | DTLS 1.3 or OSCORE | RFC 9147; RFC 8613 |

Tokens MUST carry an `aud` claim equal to the canonical project URI from Phase 1 §2.1, and a `scope` string composed of one or more verbs from §4.

---

## 2. HTTP/REST Surface

### 2.1 Base URL

```
https://<host>/wia-city-017/v1
```

Servers MUST advertise the version through the `Server` header and the `/.well-known/wia-traffic` document (RFC 8615).

### 2.2 Resource map

| Resource | Methods | Purpose |
|----------|---------|---------|
| `/projects` | GET, POST | List / create projects |
| `/projects/{projectId}` | GET, PATCH, DELETE | Project descriptor |
| `/projects/{projectId}/networks` | GET, POST | Network catalog |
| `/projects/{projectId}/networks/{networkId}` | GET, PATCH, DELETE | Network |
| `/projects/{projectId}/networks/{networkId}:import-gdf` | POST | Import ISO 14825 GDF document |
| `/projects/{projectId}/networks/{networkId}:export-spat-map` | POST | Export ISO/TS 19091 messages |
| `/projects/{projectId}/intersections/{intersectionId}` | GET, PATCH | Signalised-intersection descriptor |
| `/projects/{projectId}/demand` | GET, POST | Demand catalog |
| `/projects/{projectId}/scenarios` | GET, POST | Scenario catalog |
| `/projects/{projectId}/scenarios/{scenarioId}:run` | POST | Launch a run |
| `/projects/{projectId}/runs/{runId}` | GET | Run descriptor |
| `/projects/{projectId}/runs/{runId}/results` | GET | Result summary |
| `/projects/{projectId}/runs/{runId}/trace` | GET | Vehicle-trace bundle |
| `/projects/{projectId}/runs/{runId}/spat-stream` | GET (SSE) | Live SPaT replay |
| `/projects/{projectId}/audit` | GET | Audit log |
| `/projects/{projectId}/health` | GET | Liveness / readiness |

`PATCH` requests use JSON Merge Patch (RFC 7396) by default and JSON Patch (RFC 6902) when `Content-Type: application/json-patch+json`.

### 2.3 Conditional requests

ETag/If-Match/If-None-Match per RFC 9110 §13. Strong validators required for any state-changing request.

### 2.4 Errors

Errors use *Problem Details for HTTP APIs* (RFC 9457). The minimum problem types:

| Code | HTTP | Meaning |
|------|------|---------|
| `traffic/forbidden-project` | 403 | Caller lacks project privilege |
| `traffic/network-incompatible-gdf` | 422 | GDF import failed ISO 14825 conformance |
| `traffic/intersection-spat-incompatible` | 422 | SPaT/MAP export failed ISO/TS 19091 conformance |
| `traffic/demand-out-of-horizon` | 422 | Demand timeline out of scenario horizon |
| `traffic/run-cancelled` | 409 | Run was cancelled before completion |
| `traffic/license-required` | 403 | Engine licensing constraint |

### 2.5 Pagination and long operations

Cursor pagination with `Link` headers per RFC 8288. Long-running runs use `202 Accepted` with `Location` header pointing to the run descriptor.

---

## 3. Streaming Surface

### 3.1 SPaT replay over SSE

```
GET /wia-city-017/v1/projects/p-1/runs/r-12/spat-stream HTTP/1.1
Accept: text/event-stream
```

Each SSE event has the JSON body:

```json
{
  "tSeconds": 12.45,
  "intersectionId": "isx-001",
  "spatBytes": "<base64 ISO/TS 19091 SPaT message>"
}
```

### 3.2 SPaT replay over CoAP

```
GET coaps://gw/wia-traffic/v1/runs/r-12/spat-stream
Observe: 0
Accept: 60   ; application/cbor
```

CoAP notifications carry the CBOR-encoded version of the SSE event body.

### 3.3 Engine telemetry

Engine-internal telemetry (queue lengths, simulation step rate, RNG state hash) is exposed as a separate SSE stream gated by the `traffic:read-engine-telemetry` verb.

---

## 4. Verbs and Scopes

| Verb | Capability |
|------|------------|
| `traffic:read` | Read project, networks, demand, scenarios, runs |
| `traffic:author-network` | Create/edit networks, import GDF |
| `traffic:author-demand` | Create/edit demand |
| `traffic:author-scenario` | Create/edit scenarios |
| `traffic:run` | Launch a scenario run |
| `traffic:cancel-run` | Cancel an in-flight run |
| `traffic:export` | Export SPaT/MAP / GDF / vehicle-trace bundles |
| `traffic:read-engine-telemetry` | Subscribe to engine telemetry |
| `traffic:audit-read` | Read audit log |

Tokens missing a required verb produce HTTP 403 with the `traffic/forbidden-project` problem code.

---

## 5. Sample Operations

### 5.1 Launch a run

```
POST /wia-city-017/v1/projects/p-1/scenarios/sc-rush-am:run HTTP/1.1
Content-Type: application/json
Authorization: DPoP <token>

{"rngSeed": 19770425, "engineCapabilities": ["MICROSCOPIC"]}
```

Response:

```
HTTP/1.1 202 Accepted
Location: /wia-city-017/v1/projects/p-1/runs/r-12
ETag: "R-12-INIT"
```

### 5.2 Export SPaT/MAP (HTTP)

```
POST /wia-city-017/v1/projects/p-1/networks/net-cbd:export-spat-map HTTP/1.1
Accept: application/x-iso19091+cbor
```

The response is a CBOR bundle of ISO/TS 19091 MAP and SPaT messages corresponding to the project's intersections. The export is signed with COSE_Sign1 (RFC 9052) using the project's signing key.

### 5.3 Subscribe to live SPaT (CoAP)

See §3.2.

---

## 6. Conformance

A WIA-CITY-017 v1 *engine* MUST implement:

- HTTP/REST surface in §2.
- ISO 14825 GDF import path under `/networks/{id}:import-gdf`.
- ISO/TS 19091 export path under `/networks/{id}:export-spat-map`.
- Verbs from §4 with audit logging on every state-changing call.

A WIA-CITY-017 v1 *client* MUST:

- Honour ETag-based concurrency.
- Parse RFC 9457 problem details, including unknown extension fields.
- Validate exported bundles against COSE_Sign1 detached signatures.

---

## 7. Operational Notes

### 7.1 Idempotency

Mutating verbs accept `Idempotency-Key`. Servers retain the response per key for 24 hours.

### 7.2 Rate limiting

Token-bucket rate limiting with the IETF `RateLimit-*` headers. Run-launch verbs MAY be subject to a per-account burst limit; engine-telemetry subscription is rate-limited per project.

### 7.3 Discovery

`/projects/{projectId}/.well-known/wia-traffic` returns supported verbs, supported encodings, supported export formats, and links to OpenAPI / SPaT-stream endpoints.

### 7.4 Resumable uploads

Network or demand uploads exceeding 50 MB SHOULD use the IETF *Resumable Uploads* draft profile (PUT with `Upload-Offset` and `Upload-Length` headers); short uploads MAY use a single multipart/form-data POST.

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
11. RFC 9176 — *CoRE Resource Directory.*
12. RFC 9200; RFC 9202; RFC 9203 — *ACE-OAuth and profiles.*
13. RFC 9457 — *Problem Details for HTTP APIs.*
14. RFC 9700 — *OAuth 2.1.*
15. RFC 9562 — *Universally Unique IDentifiers (UUIDs).*
16. ISO 14817-1:2015; ISO 14817-2:2015; ISO 14817-3:2017 — *ITS data dictionaries.*
17. ISO 14825:2011 — *Geographic Data Files (GDF).*
18. ISO 17572 (all parts) — *Location referencing.*
19. ISO/TS 19091:2017/2019 — *V2I / I2V signalised intersections.*
20. ISO 19082:2025 — *Roadside-module / signal-controller data frames.*
21. ISO 21217:2020 — *CALM architecture.*
22. ISO/IEC 27001:2022; ISO/IEC 27701:2019 — *ISMS / PIM.*

---

## 9. Engine Federation Notes

Engines that share runs across organisational boundaries MUST:

- Use a federation document (Phase 4 §9.1) listing partner project URIs.
- Restrict the verb set advertised to partners (typically `traffic:read`, `traffic:export`).
- Sign exported runs with a federation-grade COSE key distinct from the operator key.

This federation contract is enforced at the audience and scope layer of every issued token.

---

## 10. Health Endpoint Contract

The `/projects/{projectId}/health` endpoint MUST return a JSON document with at least:

- `state` ∈ `{OK, DEGRADED, CRITICAL, OFFLINE}`
- `version` — implementation semver
- `uptimeSeconds` — non-negative integer
- `engines` — `{total, healthy, runningJobs}`
- `runs` — `{queued, inFlight, completedLast24h, failedLast24h}`
- `dataStore` — `{capacityTb, freeTb, oldestTraceAt}`
- `timeSync` — `{source, accuracyMs, lastCheck}`
- `audit` — `{lastEntryAt, queueDepth}`

The endpoint MUST be served without authentication for the `state` and `version` fields only; full detail requires `traffic:read`.

---

## 11. Backpressure and Quotas

Engines MUST implement per-project quotas covering:

- Concurrent runs.
- Active live-replay subscriptions.
- Trace storage capacity.
- Outbound federation bandwidth.

Quota enforcement uses the IETF `RateLimit-*` headers on the HTTP surface and CoAP option `MaxAge` semantics on the CoAP surface. Quota exhaustion returns HTTP 429 / CoAP 4.29 with a `Retry-After` header and a problem-detail of type `traffic/quota-exceeded`.

---

## 12. Versioning Policy

Major version bumps require a parallel HTTP path (`/wia-city-017/v2`). Minor and patch revisions are backwards-compatible within `/v1`. Clients MUST tolerate unknown fields and unknown problem-detail extensions per RFC 9457 §4.4. Breaking changes to vehicle-trace bundle structure require a major version bump even when the network and scenario formats are unchanged.

---

## 13. Discovery Document Schema

The `/.well-known/wia-traffic` discovery document MUST conform to the following JSON schema fragment:

```json
{
  "version": "string (semver)",
  "supportedVerbs": ["array of verb tokens from §4"],
  "supportedEncodings": ["json", "cbor"],
  "supportedExportFormats": ["application/x-iso19091+cbor", "application/x-gdf+xml", "application/wia-trace+cbor"],
  "supportedSPaTCadenceMs": "number",
  "openApiUrl": "string (URI)",
  "spatStreamUrl": "string (URI)",
  "federationDocumentUrl": "string (URI) | null",
  "conformanceTags": ["array of conformance tags from Phase 4 §12"]
}
```

The discovery document SHOULD be cached for 60 seconds by clients; the engine MUST emit `Cache-Control: max-age=60` accordingly.
