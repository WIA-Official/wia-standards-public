# WIA-CITY-014 (security-system-city) — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 2 of 4 (API Interface)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This phase defines the network-facing API of WIA-CITY-014. It exposes three coordinated surfaces:

- **HTTP/REST** — for management plane operations (catalog, configuration, health, evidence retrieval), built on RFC 9110 (HTTP semantics) with HTTPS over TLS 1.3 (RFC 8446) as the only permitted carrier.
- **RTP/RTSP video plane** — IEC 62676-2-3 RTP-based interoperability for live and recorded video; RTSP follows RFC 7826.
- **Event/alert bus** — RFC 8895-style server-sent events on the management plane and CoAP-OBSERVE on the constrained side (RFC 7252, RFC 7641).

OpenAPI 3.1 documents the HTTP/REST surface. The RTSP and RTP profiles defer to IEC 62676-2-x for media-specific elements.

### 1.1 Authentication and authorization

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Token format | JWT with PoP, or CWT for constrained nodes | RFC 7519, RFC 7800; RFC 8392 |
| Authorization framework | OAuth 2.1 | RFC 9700 (BCP) |
| Constrained-device flow | ACE-OAuth | RFC 9200, RFC 9202 (DTLS), RFC 9203 (OSCORE) |
| Channel security (HTTP) | TLS 1.3 | RFC 8446 |
| Channel security (CoAP) | DTLS 1.3 or OSCORE | RFC 9147; RFC 8613 |
| At-rest token integrity | COSE_Sign1 | RFC 9052 |

Tokens MUST carry an `aud` claim equal to the canonical site URI from Phase 1 §2.1, and a `scope` string composed of one or more verbs from §4.

---

## 2. HTTP/REST Surface

### 2.1 Base URL

```
https://<site-host>/wia-city-014/v1
```

Servers MUST advertise version 1 of this specification through the `Server` header and the `/.well-known/wia-security` document (RFC 8615 Well-Known URIs).

### 2.2 Resource map

| Resource | Methods | Purpose |
|----------|---------|---------|
| `/sites` | GET | List sites visible to caller |
| `/sites/{siteId}` | GET, PATCH | Site descriptor |
| `/sites/{siteId}/zones` | GET, POST | Zone collection |
| `/sites/{siteId}/cameras` | GET, POST | Camera collection |
| `/sites/{siteId}/cameras/{cameraId}` | GET, PATCH, DELETE | Camera descriptor |
| `/sites/{siteId}/cameras/{cameraId}:ptz` | POST | PTZ command (when supported) |
| `/sites/{siteId}/cameras/{cameraId}/streams` | GET | Live stream descriptors (RTP profile) |
| `/sites/{siteId}/cameras/{cameraId}/recordings` | GET, POST | Recorded segments |
| `/sites/{siteId}/access-points` | GET, POST | Access points |
| `/sites/{siteId}/access-points/{apId}:command` | POST | Door command (lock, unlock, momentary, lockdown) |
| `/sites/{siteId}/sensors` | GET, POST | Sensor catalog |
| `/sites/{siteId}/alarms` | GET, POST | Alarm collection |
| `/sites/{siteId}/alarms/{alarmId}:acknowledge` | POST | Acknowledge an alarm |
| `/sites/{siteId}/alarms/{alarmId}:clear` | POST | Clear (with justification) |
| `/sites/{siteId}/alarms/{alarmId}/evidence` | GET | Bundled evidence package (ISO/IEC 27037) |
| `/sites/{siteId}/recorders` | GET, POST | Recorder/NVR catalog |
| `/sites/{siteId}/audit` | GET | Audit log per IEC 62443-3-3 SR 6.1 |
| `/sites/{siteId}/health` | GET | Liveness/readiness, capability summary |

`PATCH` requests use JSON Merge Patch (RFC 7396) by default and JSON Patch (RFC 6902) when `Content-Type: application/json-patch+json`.

### 2.3 Conditional requests

ETag/If-Match/If-None-Match per RFC 9110 §13. Strong validators required for any state-changing request.

### 2.4 Errors

Errors use *Problem Details for HTTP APIs* (RFC 9457). The minimum problem types:

| Code | HTTP | Meaning |
|------|------|---------|
| `security/forbidden-zone` | 403 | Caller lacks IEC 62443 zone privilege |
| `security/grade-mismatch` | 409 | Component grade below zone requirement |
| `security/operational-category-not-met` | 422 | Camera does not meet IEC 62676-4 category |
| `security/credential-revoked` | 403 | Credential on RFC 5280 CRL or RFC 6960 OCSP fail |
| `security/evidence-bundle-tampered` | 422 | ISO/IEC 27037 chain-of-custody verify failed |
| `security/lockdown-active` | 423 | Door command refused while site lockdown active |

### 2.5 Pagination

Cursor-based pagination is mandatory. Servers return `Link` headers per RFC 8288 with `rel="next"` and `rel="prev"`.

### 2.6 Long-running operations

Bulk imports, mass firmware updates, and lockdown propagation are long-running. They follow `202 Accepted` + status-URI pattern, with optional `Prefer: respond-async` (RFC 7240).

---

## 3. RTP / RTSP Video Plane

### 3.1 Conformance

Live and recorded video distribution MUST conform to IEC 62676-2-3 *IP interoperability based on RTP services*. The RTSP control plane follows RFC 7826 (RTSP 2.0); legacy RTSP 1.0 (RFC 2326) is deprecated for new deployments.

### 3.2 Stream descriptors

`GET /cameras/{cameraId}/streams` returns:

```json
{
  "streams": [
    {
      "kind": "live",
      "rtspUrl": "rtsps://gateway/live/cam-lobby-01",
      "codec": "ISO/IEC 23008-2",
      "profile": "Main",
      "resolution": "3840x2160",
      "fps": 30
    },
    {
      "kind": "playback",
      "rtspUrl": "rtsps://gateway/playback/cam-lobby-01?from=...&to=...",
      "codec": "ISO/IEC 23008-2",
      "profile": "Main"
    }
  ]
}
```

### 3.3 Transport security

Live and recorded video MUST be transported over RTSPS (TLS 1.3, RFC 8446) or over a SRTP profile per IETF specifications. Plain RTSP / unencrypted RTP MUST NOT be used outside of an isolated test bench.

### 3.4 Operational category attestation

Each stream descriptor MUST include a self-asserted IEC 62676-4 operational performance category and a reference to the calibration record that justifies the assertion. Mismatch between assertion and observation triggers the `security/operational-category-not-met` problem type.

---

## 4. Verbs and Scopes

| Verb | Capability | Surface(s) |
|------|------------|------------|
| `security:read` | Read catalog, view live/recorded video metadata | HTTP |
| `security:view-live` | Pull live stream | RTP |
| `security:view-recorded` | Pull recorded segment | RTP |
| `security:control-ptz` | Issue PTZ command | HTTP |
| `security:control-door` | Lock/unlock/momentary on access point | HTTP |
| `security:lockdown` | Site- or zone-level lockdown | HTTP |
| `security:acknowledge-alarm` | Acknowledge alarm | HTTP |
| `security:clear-alarm` | Clear alarm with justification | HTTP |
| `security:audit-read` | Read audit log | HTTP |
| `security:evidence-export` | Bundle and export ISO/IEC 27037 evidence | HTTP |

Each verb MUST be enforced at every surface. Tokens missing a required verb produce HTTP 403 / CoAP 4.03 with the `security/forbidden-zone` problem code.

---

## 5. Sample Operations

### 5.1 Acknowledge an alarm (HTTP)

```
POST /wia-city-014/v1/sites/hq/alarms/al-2026-04-26-0173:acknowledge HTTP/1.1
Host: gateway.example.org
Authorization: DPoP <token>
Content-Type: application/json

{"comment": "Patrol team dispatched"}
```

Response:

```
HTTP/1.1 200 OK
ETag: "AL-7421"
Content-Type: application/json

{"state":"ACKNOWLEDGED","acknowledgedAt":"2026-04-26T08:15:42+09:00"}
```

### 5.2 Door momentary unlock (HTTP)

```
POST /wia-city-014/v1/sites/hq/access-points/ap-main:command HTTP/1.1
Content-Type: application/json

{"verb":"momentary","durationSeconds":5,"justification":"VIP arrival"}
```

### 5.3 Live stream (RTSP)

```
RTSP/2.0 DESCRIBE rtsps://gateway/live/cam-lobby-01 RTSP/2.0
CSeq: 1
Authorization: Bearer <token>
```

The DESCRIBE response is an SDP (RFC 8866) document referencing the IEC 62676-2-3 RTP profile.

### 5.4 Evidence export

```
GET /wia-city-014/v1/sites/hq/alarms/al-2026-04-26-0173/evidence HTTP/1.1
Accept: application/wia-evidence+zip
```

The response is a ZIP container with:

- `manifest.json` — ISO/IEC 27037 chain-of-custody manifest, COSE_Sign1 signed.
- `clip.mp4` — relevant video clip, ISO/IEC 14496-10 or 23008-2.
- `events.jsonl` — newline-delimited Phase-1 events.
- `signatures/` — detached COSE signatures.

---

## 6. Conformance

A WIA-CITY-014 v1 *gateway* MUST implement:

- The HTTP/REST surface in §2.
- The RTP/RTSP profile of §3 with TLS-protected transport.
- Verbs from §4 with audit logging on every state-changing call.
- Problem details from §2.4.

A WIA-CITY-014 v1 *client* MUST:

- Honour ETag-based concurrency.
- Parse RFC 9457 problem details, including unknown extension fields.
- Validate evidence bundles against COSE_Sign1 detached signatures.
- Refuse to render a stream descriptor whose asserted IEC 62676-4 category does not match the calibration record (when accessible).

---

## 7. Operational Notes

### 7.1 Idempotency

Mutating verbs accept `Idempotency-Key`. Servers retain the response per key for 24 hours. Idempotent processing is layered above ETag concurrency control.

### 7.2 Rate limiting

Token-bucket rate limiting with the IETF `RateLimit-*` headers. Safety verbs (`security:control-door` *unlock*, `security:lockdown`) MUST NOT be rate-limited below operational thresholds.

### 7.3 Discovery

`/sites/{siteId}/.well-known/wia-security` returns supported verbs, supported encodings, supported codecs, and links to OpenAPI / RTSP descriptors.

---

## 8. References

1. RFC 2326; RFC 7826 — *Real Time Streaming Protocol (RTSP).*
2. RFC 5280 — *Internet X.509 PKI Certificate and CRL Profile.*
3. RFC 6749; RFC 9700 — *OAuth 2.0 / 2.1.*
4. RFC 6902; RFC 7396 — *JSON Patch / Merge Patch.*
5. RFC 6960 — *X.509 Online Certificate Status Protocol (OCSP).*
6. RFC 7240 — *Prefer Header for HTTP.*
7. RFC 7252 — *Constrained Application Protocol (CoAP).*
8. RFC 7519; RFC 7800; RFC 8392 — *JWT, PoP, CWT.*
9. RFC 7641; RFC 7959 — *CoAP OBSERVE / Block-wise.*
10. RFC 8259; RFC 8610; RFC 8615; RFC 8949 — *JSON, CDDL, well-known URIs, CBOR.*
11. RFC 8288 — *Web Linking.*
12. RFC 8446; RFC 9147 — *TLS 1.3, DTLS 1.3.*
13. RFC 8613 — *OSCORE.*
14. RFC 8866 — *Session Description Protocol.*
15. RFC 9052; RFC 9053 — *COSE Structures and Algorithms.*
16. RFC 9110; RFC 9111; RFC 9112; RFC 9113; RFC 9114 — *HTTP family.*
17. RFC 9176 — *CoRE Resource Directory.*
18. RFC 9200; RFC 9202; RFC 9203 — *ACE-OAuth and profiles.*
19. RFC 9457 — *Problem Details for HTTP APIs.*
20. RFC 9562 — *Universally Unique IDentifiers (UUIDs).*
21. IEC 62676-1:2013; IEC 62676-2-1:2013; IEC 62676-2-3:2013; IEC 62676-4:2025 — *Video surveillance.*
22. IEC 60839-1:2014; IEC 60839-11-1:2013 — *Alarm and electronic access-control systems.*
23. IEC 62443-3-3:2013 — *System security requirements and security levels.*
24. ISO/IEC 14496-10; ISO/IEC 23008-2 — *AVC, HEVC.*
25. ISO/IEC 27001:2022; ISO/IEC 27037:2012 — *ISMS, digital evidence.*

---

## 9. Operational Considerations

### 9.1 Health endpoint contract

The `/sites/{siteId}/health` endpoint MUST return a JSON document conformant to the Phase-1 schema with at least the following keys:

- `state` ∈ `{OK, DEGRADED, CRITICAL, OFFLINE}`
- `version` — implementation semver
- `uptimeSeconds` — non-negative integer
- `cameras` — `{total, reachable, byCategory}`
- `accessPoints` — `{total, reachable, byGrade}`
- `recorders` — `{total, healthy, totalCapacityTb, freeCapacityTb}`
- `lockdown` — `{active, scope, since}`
- `audit` — `{lastEntryAt, queueDepth}`

The endpoint MUST be served without authentication for the `state` and `version` fields only; full detail requires `security:read`.

### 9.2 Backpressure

Servers MUST implement backpressure on long polling and event subscription. When the audit queue depth exceeds the configured threshold, new event subscriptions return HTTP 503 with the `Retry-After` header per RFC 9110 §10.2.3 until queue depth recovers.

### 9.3 Versioning

Major version bumps require a parallel HTTP path (`/wia-city-014/v2`). Minor and patch revisions are backwards-compatible within `/v1`. Clients MUST tolerate unknown fields and unknown problem-detail extensions per RFC 9457 §4.4.
