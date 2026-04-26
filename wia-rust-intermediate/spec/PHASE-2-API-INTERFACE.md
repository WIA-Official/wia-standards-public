# WIA Rust Intermediate — Phase 2: API Interface

**Standard**: WIA Rust Intermediate
**Phase**: 2 of 4 — API Interface
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 2 specifies the HTTP surface that an academy exposes for the data
families defined in Phase 1: curriculum manifests, exercises, learner
records, and assessment results. Out of scope: cross-academy federation
state (Phase 3), WIA-family bridges (Phase 4).

---

## 2. Transport

* HTTPS only, TLS 1.3 (IETF RFC 8446) or later.
* HTTP/2 RECOMMENDED, HTTP/1.1 SHALL be supported as fallback.
* `Strict-Transport-Security` per IETF RFC 6797.
* Compression: `gzip`, `br`.

---

## 3. Discovery

```
GET https://<academy>/.well-known/wia-rust-intermediate
```

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "academy_id": "did:wia:academy:bonghwa",
  "endpoints": {
    "manifest":   "https://academy.example/wri/manifest",
    "exercise":   "https://academy.example/wri/exercise",
    "learner":    "https://academy.example/wri/learner",
    "assessment": "https://academy.example/wri/assessment",
    "handshake":  "https://academy.example/wri/handshake"
  },
  "supported_signatures": ["Ed25519"],
  "supported_languages": ["en", "ko"],
  "rate_limits": {
    "anonymous":     { "rps": 1,  "burst": 5 },
    "authenticated": { "rps": 20, "burst": 60 }
  }
}
```

Discovery responses are cacheable for 300 s.

---

## 4. Curriculum Manifest Endpoints

### 4.1 Read

```
GET /wri/manifest/{manifest_id}
```

Returns the manifest JSON. Anonymous reads are permitted.

### 4.2 Publish

```
PUT /wri/manifest/{manifest_id}
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"
```

The body is a complete manifest. The academy MUST verify both the HTTP
message signature and the manifest's detached signature.

### 4.3 Withdraw

```
DELETE /wri/manifest/{manifest_id}
Authorization: WIA-Sig …
```

Withdrawn manifests remain in audit storage for 7 years (default).
Subsequent reads return `410 Gone`.

---

## 5. Exercise Endpoints

```
GET /wri/exercise/{exercise_id}
GET /wri/exercise?module_id={n}&difficulty={band}
```

Filter combinations supported: by module, by difficulty band, by author.
Cursor pagination as in Phase 2 §10.

---

## 6. Learner Record Endpoints

### 6.1 Read

```
GET /wri/learner/{learner_id}
```

The learner's own DID grants full read. Other callers receive a redacted
record showing modules passed and level attained but no submission
artefacts.

### 6.2 Update

```
PUT /wri/learner/{learner_id}
Authorization: WIA-Sig …
```

Authorised callers: the learner (their own record), the academy that
graded the most recent assessment, or a delegated agent (Phase 3 §6).

### 6.3 Withdraw

```
DELETE /wri/learner/{learner_id}
```

Withdrawal hides public reads but retains the record in audit storage
for 7 years per academic record retention norms.

---

## 7. Assessment Endpoints

### 7.1 Submit

```
POST /wri/assessment
Content-Type: application/json
Idempotency-Key: <opaque>
```

The body is an assessment_result envelope. Submissions are immutable; a
correction is appended as a new envelope referencing the original via
`corrects`.

### 7.2 Read

```
GET /wri/assessment/{result_id}
GET /wri/assessment?learner_id={did}&exercise_id={id}
```

---

## 8. Authentication

Writes require HTTP Message Signatures per IETF RFC 9421. The signature
MUST cover the request method, target URI, `Date`, `Content-Digest` and
`Content-Length`. Reads MAY be anonymous subject to the discovery rate
limit.

---

## 9. Errors

Errors follow IETF RFC 9457 problem details. Reserved error types live
under `https://wiastandards.com/wia-rust-intermediate/errors/`.

```json
{
  "type": "https://wiastandards.com/wia-rust-intermediate/errors/manifest-version-unsupported",
  "title": "Manifest version not supported",
  "status": 400,
  "detail": "wia_rust_intermediate_version=2.0 not implemented"
}
```

---

## 10. Pagination

Cursor-based pagination as in WIA-SOCIAL Phase 2 §6.1. Responses include
`next_cursor`, `approx_total`, and `page_kind` fields.

---

## 11. Conformance

A Phase 2 conformant academy MUST:

1. Publish `/.well-known/wia-rust-intermediate` with all endpoints.
2. Implement manifest, exercise, learner, assessment routes.
3. Honour Idempotency-Key on all writes.
4. Emit problem-detail JSON for 4xx/5xx responses.

---

## 12. References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 6797 — HSTS
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 9457 — Problem Details for HTTP APIs
* JSON Schema Draft 2020-12

---

## Appendix A — Reference Request / Response Pairs

### A.1 Read a manifest

```http
GET /wri/manifest/cm_01HZB… HTTP/1.1
Host: academy.example
Accept: application/json

HTTP/1.1 200 OK
Content-Type: application/json
Cache-Control: public, max-age=300

{ "wia_rust_intermediate_version":"1.0.0", "type":"curriculum_manifest", "...":"..." }
```

### A.2 Submit an assessment with idempotency

```http
POST /wri/assessment HTTP/1.1
Host: academy.example
Content-Type: application/json
Idempotency-Key: 7e3a9b6c-49d4-4d0a-8e0d-…
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_rust_intermediate_version":"1.0.0", "type":"assessment_result", "...":"..." }

HTTP/1.1 201 Created
Location: /wri/assessment/res_01HZA…
RateLimit-Limit: 20
RateLimit-Remaining: 19
RateLimit-Reset: 53
```

### A.3 Update a learner record

```http
PUT /wri/learner/did%3Awia%3Alearner%3A01HZA%E2%80%A6 HTTP/1.1
Host: academy.example
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_rust_intermediate_version":"1.0.0", "type":"learner_record",
  "modules_passed":[{"module_id":1,"passed_at":"2026-02-15","band":"Merit"}], "...":"..." }

HTTP/1.1 200 OK
```

## Appendix B — Conformance Test Suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-rust-intermediate-conformance` and
walks through:

1. Discovery document round-trip.
2. Manifest publish, read, replace, withdraw.
3. Exercise listing with filter combinations.
4. Learner record update with concurrent grader writes (idempotency).
5. Assessment submission and corrections chain.
6. Rate-limit header presence and exhaustion behaviour.

## Appendix C — Reserved Endpoint Paths

Implementations MUST NOT serve unrelated content under the following
prefixes; future minor versions reserve them for additional features.

| Prefix | Reserved for |
|--------|--------------|
| `/wri/cohort/`     | Phase 3 cohort grouping for synchronous courses |
| `/wri/transcript/` | Future transcript export bundle |
| `/wri/badge/`      | Future Open Badges export bridge |
| `/wri/integrity/`  | Future cryptographic transparency log |

Hosts that need to surface unrelated tooling on the same origin MUST use
a distinct subdomain or path prefix outside the reserved namespace.

## Appendix D — Pagination and Cursor Semantics

`GET /wri/exercise?…` and `GET /wri/assessment?…` use opaque cursors.
The response shape is:

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "items": [ /* … */ ],
  "next_cursor": "AAAB",
  "approx_total": 312,
  "page_kind": "forward"
}
```

* Cursors are opaque base64url; clients MUST NOT parse them.
* Pagination is monotonic by server-side write order.
* Hosts SHOULD honour cursors for at least 7 days; stale cursors return
  `410 Gone` with a problem document of type `…/cursor-expired`.
* Maximum page size is 200 items. Larger requests are silently capped.

## Appendix E — Operational Recommendations

* Academies MUST persist incoming writes durably before acknowledging.
* Academies SHOULD provide a separate read-replica endpoint for high-cardinality
  reads (exercise listings, assessment audit) so write paths remain
  responsive to grader submissions during exam windows.
* Academies SHOULD expose a `/wri/health` liveness probe outside the
  public rate-limit accounting; tooling consumes it but it MUST NOT be
  advertised in `/.well-known/wia-rust-intermediate`.
* Academies SHOULD retain problem-detail responses emitted to identified
  peers for 30 days to support cross-academy debugging during disputes.

## Appendix F — Backwards Compatibility Promise

Within the 1.x line every endpoint listed in §4–§7 MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional query parameters, response fields, new
endpoints, or media types. Hosts MUST NOT remove or repurpose existing
ones. Breaking changes ride a major version bump and MUST be preceded by
a 12-month deprecation window per RFC 8594 / RFC 9745; deprecated routes
MUST emit `Deprecation` and `Sunset` headers throughout the window.

## Appendix G — CORS Policy

* Public read endpoints (`GET /wri/manifest/…`, `GET /wri/exercise/…`)
  MUST send `Access-Control-Allow-Origin: *` so unauthenticated tooling
  (course catalogue aggregators, public registries) can fetch them
  cross-origin.
* Authenticated write endpoints MUST validate the request `Origin`
  against an allow-list configured by the academy operator. The
  allow-list MUST be explicit (no `*` for credentialed requests).
* Pre-flight responses MUST cache for 600 seconds via
  `Access-Control-Max-Age` to avoid repeated OPTIONS overhead from busy
  grader UIs.

## Appendix H — Operational Telemetry

A conformant academy SHOULD expose Prometheus-style metrics on a separate
port (default 9091) so SRE dashboards have a uniform shape across
academies.

| Metric | Type | Labels |
|--------|------|--------|
| `wia_ri_manifests_published_total` | counter | `outcome` |
| `wia_ri_assessments_received_total` | counter | `band` |
| `wia_ri_handshakes_completed_total` | counter | `role`, `outcome` |
| `wia_ri_request_duration_seconds` | histogram | `route`, `status` |
| `wia_ri_seen_nonce_cache_size` | gauge | none |

Telemetry MUST NOT include learner identifiers as label values; doing so
would violate the privacy promise of Phase 3 §8. Operators that need
per-learner debugging SHOULD use the per-peer 30-day problem-detail
retention from Appendix E rather than baking identities into telemetry.

## Appendix I — Reference Health and Readiness Probes

Academies SHOULD expose three liveness signals on a separate port:

| Probe | Path | Payload |
|-------|------|---------|
| Liveness  | `/healthz`  | `{"status":"ok"}` |
| Readiness | `/readyz`   | `{"deps":{"db":"ok","queue":"ok"}}` |
| Startup   | `/startupz` | `{"phase":"warming","ready_at":"2026-…"}` |

These MUST NOT be advertised in `/.well-known/wia-rust-intermediate` and
MUST NOT be subject to public rate limits. Internal observability tooling
consumes them; nothing external should rely on their presence.

## Appendix J — Worked Discovery Document (Full profile)

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "academy_id": "did:wia:academy:bonghwa",
  "endpoints": {
    "manifest":   "https://academy.bonghwa.example/wri/manifest",
    "exercise":   "https://academy.bonghwa.example/wri/exercise",
    "learner":    "https://academy.bonghwa.example/wri/learner",
    "assessment": "https://academy.bonghwa.example/wri/assessment",
    "handshake":  "https://academy.bonghwa.example/wri/handshake",
    "cohort":     "https://academy.bonghwa.example/wri/cohort"
  },
  "supported_signatures": ["Ed25519"],
  "supported_languages": ["en", "ko"],
  "bridge_profile": "Full",
  "rate_limits": {
    "anonymous":     { "rps": 1,  "burst": 5 },
    "authenticated": { "rps": 20, "burst": 60 }
  },
  "trust_anchor": {
    "url": "https://academy.bonghwa.example/.well-known/wia-trust-anchor",
    "fingerprint": "sha256:7d8f2c…"
  }
}
```

Discovery responses MUST be cacheable for at least 300 seconds via
`Cache-Control: public, max-age=300`. Clients MUST honour
`must-revalidate` when present.

弘益人間 — Benefit All Humanity.
