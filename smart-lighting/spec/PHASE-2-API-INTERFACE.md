# WIA-CITY-009 (smart-lighting) — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 2 of 4 (API Interface)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This phase defines the network-facing API of WIA-CITY-009. Two complementary surfaces are specified:

- **HTTP/REST surface**, suitable for management plane operations (commissioning, scenes, schedules, reporting), built on RFC 9110 (HTTP semantics) and RFC 9112 (HTTP/1.1) with carrier upgrade to RFC 9113 (HTTP/2) or RFC 9114 (HTTP/3).
- **CoAP surface**, suitable for low-power data-plane operations (state changes, telemetry observe), built on RFC 7252 (Constrained Application Protocol) with the OBSERVE option of RFC 7641, block transfer of RFC 7959, and security via DTLS 1.3 (RFC 9147).

Both surfaces share an OpenAPI 3.1 (HTTP) and a CoRE Resource Directory (RFC 9176) discovery descriptor (CoAP). Resource identifiers and payload schemas come from Phase 1.

### 1.1 Authentication and authorization

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Token format | JWT with PoP, or CWT | RFC 7519, RFC 7800; RFC 8392 |
| Authorization framework | OAuth 2.1 | RFC 9700 (Best Current Practice) |
| Constrained-device flow | ACE-OAuth | RFC 9200 (ACE framework), RFC 9202 (DTLS profile), RFC 9203 (OSCORE profile) |
| Channel security (HTTP) | TLS 1.3 | RFC 8446 |
| Channel security (CoAP) | DTLS 1.3 or OSCORE | RFC 9147; RFC 8613 |
| At-rest token integrity | COSE_Sign1 | RFC 9052 |

Tokens MUST carry an audience (`aud`) claim equal to the canonical site URI defined in Phase 1 §2.1, and a scope (`scope`) string containing one or more verbs from §4 below.

---

## 2. HTTP/REST Surface

### 2.1 Base URL

```
https://<site-host>/wia-city-009/v1
```

Servers MUST advertise version 1 of this specification through the `Server` header and the `/.well-known/wia-lighting` document (RFC 8615 Well-Known URIs).

### 2.2 Resource map

| Resource | Methods | Purpose |
|----------|---------|---------|
| `/sites` | GET | List sites visible to caller |
| `/sites/{siteId}` | GET, PATCH | Site descriptor |
| `/sites/{siteId}/zones` | GET, POST | Zone collection |
| `/sites/{siteId}/zones/{zoneId}` | GET, PATCH, DELETE | Zone descriptor |
| `/sites/{siteId}/luminaires` | GET, POST | Luminaire collection |
| `/sites/{siteId}/luminaires/{luminaireId}` | GET, PATCH, DELETE | Luminaire descriptor |
| `/sites/{siteId}/luminaires/{luminaireId}:command` | POST | Issue an IEC 62386-102 command |
| `/sites/{siteId}/sensors` | GET, POST | Sensor collection |
| `/sites/{siteId}/sensors/{sensorId}/teds` | GET | TEDS document (ISO/IEC/IEEE 21451) |
| `/sites/{siteId}/scenes` | GET, POST, PUT | Scene collection |
| `/sites/{siteId}/scenes/{sceneId}:recall` | POST | Recall a scene |
| `/sites/{siteId}/groups/{groupId}:command` | POST | Issue a group-broadcast command |
| `/sites/{siteId}/events` | GET (SSE) | Stream events (RFC 8895 *server-sent events* style; HTML Living Standard) |
| `/sites/{siteId}/health` | GET | Liveness/readiness, IEC 62443-3-3 SR 6.1 reporting |

`PATCH` requests use JSON Merge Patch (RFC 7396) by default and JSON Patch (RFC 6902) when `Content-Type: application/json-patch+json`.

### 2.3 Conditional requests

Resource freshness uses `ETag`/`If-Match`/`If-None-Match` per RFC 9110 §13.1. Strong validators are required for any PATCH or DELETE.

### 2.4 Errors

Error responses use the *Problem Details for HTTP APIs* media type defined by RFC 9457. The `type` URI of WIA-CITY-009 problem types is `https://wiastandards.org/problems/lighting/<code>`. The minimum problem types are:

| Code | HTTP | Meaning |
|------|------|---------|
| `lighting/forbidden-zone` | 403 | Caller lacks IEC 62443 zone privilege |
| `lighting/dali-unaddressable` | 409 | Short-address conflict (IEC 62386-102 §11.2) |
| `lighting/scene-out-of-range` | 422 | Scene number outside 0..15 |
| `lighting/photobiological-violation` | 422 | Spectrum violates IEC 62471 risk-group |
| `lighting/teds-incompatible` | 422 | TEDS not parseable per ISO/IEC/IEEE 21451-2 |

### 2.5 Pagination

Cursor-based pagination is mandatory. Servers return `Link` headers per RFC 8288 with `rel="next"` and `rel="prev"`. Clients MUST NOT depend on numeric offsets.

### 2.6 Long-running operations

Commissioning, firmware update, and bulk scene assignment use the *RESTful long-running operation* pattern: `202 Accepted` returns a `Location` header to a status URI, which serves a JSON document with `{"state":"<RUNNING|DONE|ERROR>","progress":<0..100>,...}`. Clients SHOULD use `Prefer: respond-async` per RFC 7240.

---

## 3. CoAP Surface

### 3.1 URI scheme

```
coaps://<gateway-host>/wia-l/v1/{path}
```

Resource paths mirror the HTTP surface but elide `sites/{siteId}` because each gateway is bound to a single site (IEC 62443-3-3 segmentation requirement).

### 3.2 Methods and options

CoAP methods (RFC 7252 §5.8) map directly: GET, POST, PUT, DELETE. Resource representations use CBOR (RFC 8949) media-type `application/cbor` with COSE_Sign1 / COSE_Encrypt0 wrappers (RFC 9052/9053) when end-to-end protection is required.

OBSERVE (RFC 7641) MUST be supported for telemetry resources:

```
GET coaps://gw/wia-l/v1/luminaires/lum-1015/state
Observe: 0
```

The server emits notifications with monotonically increasing 24-bit Observe values. Clients use the OBSERVE cancellation procedure of RFC 7641 §3.6.

Block-wise transfers (RFC 7959) MUST be supported for commissioning payloads larger than the path MTU.

### 3.3 Resource Directory

Gateways MUST register their resource set with a CoRE Resource Directory (RFC 9176). Discovery is performed via:

```
GET coap://[FF02::FD]/.well-known/core?rt=core.rd
```

Lighting-specific *resource types* are:

- `rt="wia.l.luminaire"` — luminaire endpoint
- `rt="wia.l.sensor"` — sensor endpoint
- `rt="wia.l.zone"` — zone endpoint
- `rt="wia.l.scene"` — scene endpoint
- `if="wia.l.command"` — command interface
- `if="wia.l.observe"` — telemetry observe interface

### 3.4 Group communication

Multicast group addressing follows RFC 7390 (group communication for CoAP). Lighting *groups* (Phase 1 §2.4 `addressing.groups`) are bound to IPv6 multicast groups in the FF35::/16 site-local range. Reliable group fan-out uses RFC 9176 publish-subscribe brokerage as a fallback.

---

## 4. Verbs and Scopes

The following verbs MUST be present in any access token used against either surface. Each verb maps to a coarse-grained capability and to one or more concrete operations.

| Verb | Capability | Surface(s) |
|------|------------|------------|
| `lighting:read` | Read site, zones, luminaires, sensors, scenes, events | HTTP, CoAP |
| `lighting:control` | Issue commands, recall scenes | HTTP, CoAP |
| `lighting:commission` | Add/remove devices, set short addresses | HTTP |
| `lighting:configure` | Edit zones, scenes, schedules | HTTP |
| `lighting:safety-override` | Override DALI device-type 1 (IEC 62386-202) emergency mode | HTTP |
| `lighting:audit` | Read IEC 62443 audit log | HTTP |

A token that omits the verb required for a request MUST be rejected with HTTP 403 / CoAP 4.03 and the problem code `lighting/forbidden-zone`.

---

## 5. Sample Operations

### 5.1 Recall a scene (HTTP)

```
POST /wia-city-009/v1/sites/hq/scenes/scene-evening:recall HTTP/1.1
Host: gateway.example.org
Authorization: DPoP <token>
Content-Type: application/json

{"transitionMillis": 4000}
```

Response:

```
HTTP/1.1 200 OK
ETag: "S-2025-0432"
Content-Type: application/json

{"state":"OK","appliedAt":"2026-04-26T08:13:42+09:00"}
```

### 5.2 Subscribe to a luminaire (CoAP)

```
GET coaps://gw.local/wia-l/v1/luminaires/lum-1015/state
Observe: 0
Token: 0x9A
Accept: 60   ; application/cbor
```

Successive notifications carry the CBOR-encoded luminaire state object from Phase 1 §2.3.

### 5.3 Bulk commission (HTTP, async)

```
POST /wia-city-009/v1/sites/hq/luminaires:bulk-commission HTTP/1.1
Prefer: respond-async
Content-Type: application/json
```

Response:

```
HTTP/1.1 202 Accepted
Location: /wia-city-009/v1/sites/hq/operations/op-7421
```

The status URI returns the long-running operation document defined in §2.6.

---

## 6. Conformance

A WIA-CITY-009 v1 *gateway* MUST implement:

- Either the HTTP surface or the CoAP surface, or both.
- All verbs from §4.
- Problem details from §2.4.
- TEDS retrieval (`/sensors/{id}/teds`) when sensors are present.
- IEC 62443-3-3 SR 1.5 / SR 1.7 audit logging on every state-changing call.

A WIA-CITY-009 v1 *client* MUST:

- Honour ETag-based concurrency.
- Tolerate unknown problem-detail codes per RFC 9457 §4.4.
- Handle CoAP block-wise transfers (RFC 7959) for any payload exceeding 1024 bytes.

---

## 7. References

1. RFC 7252 — *The Constrained Application Protocol (CoAP).*
2. RFC 7390 — *Group Communication for the Constrained Application Protocol (CoAP).*
3. RFC 7396 — *JSON Merge Patch.*
4. RFC 6902 — *JavaScript Object Notation (JSON) Patch.*
5. RFC 7519 — *JSON Web Token (JWT).*
6. RFC 7641 — *Observing Resources in CoAP.*
7. RFC 7800 — *Proof-of-Possession Key Semantics for JSON Web Tokens.*
8. RFC 7959 — *Block-Wise Transfers in CoAP.*
9. RFC 7240 — *Prefer Header for HTTP.*
10. RFC 8288 — *Web Linking.*
11. RFC 8392 — *CBOR Web Token (CWT).*
12. RFC 8446 — *The Transport Layer Security (TLS) Protocol Version 1.3.*
13. RFC 8613 — *Object Security for Constrained RESTful Environments (OSCORE).*
14. RFC 8615 — *Well-Known Uniform Resource Identifiers.*
15. RFC 8949 — *Concise Binary Object Representation (CBOR).*
16. RFC 9052 — *CBOR Object Signing and Encryption (COSE) — Structures and Process.*
17. RFC 9053 — *CBOR Object Signing and Encryption (COSE) — Initial Algorithms.*
18. RFC 9110 — *HTTP Semantics.*
19. RFC 9112 — *HTTP/1.1.*
20. RFC 9113 — *HTTP/2.*
21. RFC 9114 — *HTTP/3.*
22. RFC 9147 — *The Datagram Transport Layer Security (DTLS) Protocol Version 1.3.*
23. RFC 9176 — *Constrained RESTful Environments (CoRE) Resource Directory.*
24. RFC 9200 — *Authentication and Authorization for Constrained Environments using OAuth 2.0 (ACE-OAuth).*
25. RFC 9202 — *DTLS Profile of ACE-OAuth.*
26. RFC 9203 — *OSCORE Profile of ACE-OAuth.*
27. RFC 9457 — *Problem Details for HTTP APIs.*
28. RFC 9562 — *Universally Unique IDentifiers (UUIDs).*
29. RFC 9700 — *OAuth 2.1 — Best Current Practice for OAuth 2.0 Security.*
30. IEC 62386-102 ed. 3.0 — *DALI control-gear commands and scenes.*
31. IEC 62386-202 — *Self-contained emergency lighting (device type 1).*
32. IEC 62386-209 — *Colour control gear (device type 8).*
33. IEC 62443-3-3:2013 — *System security requirements and security levels.*
34. IEC 62471:2006 — *Photobiological safety of lamps and lamp systems.*
35. ISO/IEC/IEEE 21451-2:2010 — *Wired transducer interface.*

---

## 8. Operational Notes

### 8.1 Idempotency keys

Mutating verbs (`recall`, command POSTs, scene PUTs) MUST accept an `Idempotency-Key` request header. Servers retain the response associated with a key for 24 hours and replay it on duplicate requests within the window. The header value is an opaque RFC 9562 UUID generated by the client. Idempotency processing is layered above ETag concurrency control and does not bypass it; a cached idempotent response with a stale ETag is invalidated when the underlying resource changes.

### 8.2 Rate limiting

Servers SHOULD apply a token-bucket rate limit per access-token subject. Limit excess MUST be signalled with HTTP 429 / CoAP 4.29 and the headers `RateLimit-Limit`, `RateLimit-Remaining`, `RateLimit-Reset` from the IETF *RateLimit Header Fields* draft profile. Rate limits MUST NOT apply to safety-related verbs (`lighting:safety-override`).

### 8.3 Discovery

A site root document `/sites/{siteId}/.well-known/wia-lighting` returns a JSON object with the version, supported verbs, supported encodings (`json`, `cbor`), and links to the OpenAPI document and the CoAP Resource Directory. Discovery responses MUST be served unauthenticated to allow client bootstrapping; subsequent calls require authenticated access.
