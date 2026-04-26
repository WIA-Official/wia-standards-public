# WIA-SOCIAL — Phase 2: API Interface Specification

**Standard**: WIA-SOCIAL
**Phase**: 2 of 4 — API Interface
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 2 specifies how a WIA-SOCIAL service exposes its operations over HTTP.
Phase 1 fixed the on-the-wire shapes; this phase fixes the verbs, paths,
status codes, content negotiation, idempotency rules and discovery flow.

Out of scope: federation handshake state machine (Phase 3), legacy network
bridges (Phase 4).

---

## 2. Transport

* Transport MUST be HTTPS over TLS 1.3 (IETF RFC 8446) or later. Plaintext
  HTTP MUST be rejected with `426 Upgrade Required`.
* Servers SHOULD enable HTTP/2 (IETF RFC 9113) and SHOULD also support
  HTTP/1.1 fallback for older clients.
* Servers MUST set `Strict-Transport-Security: max-age=31536000;
  includeSubDomains` per IETF RFC 6797.
* Compression: `gzip`, `br` per the negotiated `Accept-Encoding`.

---

## 3. Discovery

Every WIA-SOCIAL host MUST publish a discovery document at:

```
GET https://<host>/.well-known/wia-social
```

```json
{
  "wia_social_version": "1.0.0",
  "endpoints": {
    "bundle":    "https://example.com/wia-social/bundle",
    "activity":  "https://example.com/wia-social/activity",
    "feed":      "https://example.com/wia-social/feed",
    "handshake": "https://example.com/wia-social/handshake",
    "receipt":   "https://example.com/wia-social/receipt",
    "schema":    "https://example.com/wia-social/schema"
  },
  "supported_signatures": ["Ed25519"],
  "supported_networks":   ["instagram", "twitter", "tiktok", "mastodon"],
  "rate_limits": {
    "anonymous":    { "rps": 1,  "burst": 5  },
    "authenticated":{ "rps": 10, "burst": 20 }
  }
}
```

Discovery responses MUST be cacheable for at least 300 seconds via
`Cache-Control: public, max-age=300`. Clients MUST honour `must-revalidate`
when present.

---

## 4. Identity Bundle Endpoints

### 4.1 Read

```
GET /wia-social/bundle/{subject}
Accept: application/json
```

| Status | Body |
|--------|------|
| `200 OK` | Identity bundle JSON |
| `404 Not Found` | Subject is unknown to this host |
| `410 Gone` | Subject existed but the latest bundle is `revoked` |
| `429 Too Many Requests` | See `Retry-After` |

### 4.2 Publish

```
PUT /wia-social/bundle/{subject}
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…",ts="…"
```

The body is a complete identity bundle. The server MUST verify:

1. The `Authorization` header signature, per IETF RFC 9421 (HTTP Message
   Signatures).
2. The bundle's own detached signature.
3. The signing key matches the bundle's `key.public`.

| Status | Meaning |
|--------|---------|
| `201 Created` | New bundle accepted (first publication) |
| `200 OK` | Existing bundle replaced |
| `400 Bad Request` | Bundle fails JSON Schema validation |
| `401 Unauthorized` | Missing or malformed `Authorization` |
| `403 Forbidden` | Signature does not match the bundle key |
| `409 Conflict` | `previous_signature` does not point at the current head |

### 4.3 Revoke

```
DELETE /wia-social/bundle/{subject}
Authorization: WIA-Sig …
```

Revocation MUST be signed by the same key that signed the bundle being
revoked. Successful revocation returns `204 No Content` and causes future
reads to return `410 Gone` until a fresh bundle is published.

---

## 5. Activity Endpoints

### 5.1 Submit

```
POST /wia-social/activity
Content-Type: application/json
Idempotency-Key: <opaque>
```

* The body is one activity envelope (Phase 1 §6).
* `Idempotency-Key` MUST be honoured per IETF draft `idempotency-key-header`.
  Repeating the same key with the same body yields the same status; with a
  different body MUST yield `422 Unprocessable Entity`.
* Servers SHOULD complete the request within 5 seconds. Long fan-outs MUST
  return `202 Accepted` with a `Location:` pointing to a status resource.

| Status | Meaning |
|--------|---------|
| `201 Created` | Activity accepted and indexed |
| `202 Accepted` | Accepted, fan-out in progress |
| `409 Conflict` | Same `activity_id` already exists |
| `413 Payload Too Large` | Activity exceeds host limits (default 1 MiB) |

### 5.2 Read

```
GET /wia-social/activity/{activity_id}
```

Returns the activity, or `410 Gone` if revoked.

### 5.3 Stream

```
GET /wia-social/activity?since={cursor}
Accept: text/event-stream
```

Returns activities newer than `cursor` as Server-Sent Events
(WHATWG HTML Living Standard §9.2). Each event's `id` field is the cursor
value the client uses for the next request.

---

## 6. Feed Endpoints

### 6.1 Compose Unified Feed

```
GET /wia-social/feed/{subject}?networks=twitter,mastodon&limit=50
```

* `networks` filters which network claims contribute. Default is all.
* `limit` is bounded at 200; servers MUST cap silently if exceeded.
* Response is a cursor-paginated list:

```json
{
  "wia_social_version": "1.0.0",
  "items": [ { "type": "activity", "...": "..." } ],
  "next_cursor": "MTcwNTMxMjAwMA==",
  "approx_total": 312
}
```

### 6.2 Subscribe (WebSub)

A subscriber MAY register a callback per W3C WebSub:

```
POST /wia-social/feed/{subject}/subscribe
Content-Type: application/x-www-form-urlencoded

hub.callback=https://my.app/cb&hub.mode=subscribe&hub.topic=…
```

The host MUST perform the WebSub verification of intent before delivering
activities to the callback.

---

## 7. Federation Handshake

The handshake bootstraps trust between two peers. Phase 3 specifies the
state machine; Phase 2 only specifies the HTTP wire layer.

```
POST /wia-social/handshake
Content-Type: application/json
```

```json
{
  "wia_social_version": "1.0.0",
  "from_peer": "https://alpha.example/wia-social",
  "to_peer":   "https://beta.example/wia-social",
  "ephemeral_public_key": "MCowBQYDK2VwAyEA…",
  "nonce": "AAEC03…",
  "timestamp": "2025-01-15T10:30:00Z",
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

Successful response returns a federation receipt (Phase 1 §5).

---

## 8. Authentication

### 8.1 Subject Authentication

Identity-bundle writes (`PUT`/`DELETE` on `/bundle`) require an HTTP Message
Signature using the subject's signing key. The signature MUST cover the
request method, target URI, `Date`, `Content-Digest` and `Content-Length`
headers per IETF RFC 9421.

### 8.2 Peer Authentication

Federation requests (`POST /handshake`, `POST /activity` from a remote host)
MUST be signed by the peer's published key, fetched from the peer's
discovery document.

### 8.3 Anonymous Reads

`GET` endpoints MAY be served anonymously, subject to the discovery rate
limit for `anonymous`.

---

## 9. Errors

Errors use IETF RFC 9457 problem details:

```http
HTTP/1.1 422 Unprocessable Entity
Content-Type: application/problem+json

{
  "type": "https://wiastandards.com/social/errors/invalid-claim",
  "title": "Claim failed validation",
  "status": 422,
  "detail": "claim[2].handle must match @[A-Za-z0-9_]+",
  "instance": "/wia-social/bundle/acct:bob@example.org"
}
```

The `type` URI MUST resolve to a human-readable description of the error
and SHOULD be stable across versions.

---

## 10. Rate Limiting & Quotas

* Servers MUST advertise per-class limits in the discovery document.
* Servers MUST emit the response headers `RateLimit-Limit`,
  `RateLimit-Remaining`, `RateLimit-Reset` per IETF draft
  `draft-ietf-httpapi-ratelimit-headers`.
* On exhaustion: `429 Too Many Requests` with `Retry-After` (RFC 7231 §7.1.3).
* Hosts MAY apply per-subject quotas; quota exceedance returns `403 Forbidden`
  with a problem document of type `…/quota-exceeded`.

---

## 11. Versioning & Deprecation

* The HTTP path is unversioned (`/wia-social/…`); versioning lives inside
  the JSON via `wia_social_version`.
* When a future major version is introduced, hosts MUST keep the previous
  major reachable for at least 12 months and MUST emit `Deprecation` and
  `Sunset` headers per IETF RFC 9745 / RFC 8594.

---

## 12. CORS

* Public read endpoints MUST send `Access-Control-Allow-Origin: *`.
* Authenticated write endpoints MUST validate `Origin` against an allow-list
  configured by the host operator.
* Pre-flight responses MUST cache for 600 seconds via `Access-Control-Max-Age`.

---

## 13. Operational Telemetry

A conformant host SHOULD expose Prometheus-style metrics on a separate port:

| Metric | Type | Description |
|--------|------|-------------|
| `wia_social_bundles_published_total` | counter | Lifetime publishes |
| `wia_social_activities_received_total` | counter | Inbound activities |
| `wia_social_handshakes_completed_total` | counter | Successful handshakes |
| `wia_social_request_duration_seconds` | histogram | Per-route latency |

Telemetry MUST NOT include subject identifiers in label values; doing so
would violate the privacy promise of Phase 1 §4.3.

---

## 14. Conformance Checklist

A Phase 2 conformant host MUST:

1. Serve `/.well-known/wia-social` with all required endpoints.
2. Implement `GET/PUT/DELETE` on `/bundle/{subject}` with the listed status
   codes and signature checks.
3. Implement `POST/GET` on `/activity` and honour `Idempotency-Key`.
4. Implement `/feed/{subject}` with cursor pagination.
5. Emit problem-detail JSON for all 4xx/5xx responses.
6. Advertise rate limits and emit RateLimit-* headers.

The companion simulator at `simulator/index.html` exercises each of these
surfaces interactively.

---

## 15. References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 9113 — HTTP/2
* IETF RFC 6797 — HSTS
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 9457 — Problem Details for HTTP APIs
* IETF RFC 9745 — HTTP Deprecation header
* IETF RFC 8594 — HTTP Sunset header
* IETF RFC 7231 — HTTP/1.1 Semantics
* W3C WebSub
* WHATWG HTML — Server-Sent Events
* JSON Schema Draft 2020-12

---

## Appendix A — Reference Request / Response Pairs

### A.1 Read a public bundle

```http
GET /wia-social/bundle/acct%3Aalice%40example.org HTTP/1.1
Host: example.com
Accept: application/json

HTTP/1.1 200 OK
Content-Type: application/json
Cache-Control: public, max-age=300
ETag: "sha256:7d8f2c…"

{ "wia_social_version":"1.0.0", "type":"identity_bundle", "...":"..." }
```

### A.2 Submit an activity with idempotency

```http
POST /wia-social/activity HTTP/1.1
Host: example.com
Content-Type: application/json
Idempotency-Key: 7e3a9b6c-49d4-4d0a-8e0d-…
Authorization: WIA-Sig keyid="…",signature="…",ts="…"

{ "wia_social_version":"1.0.0", "type":"activity", "verb":"post", "...":"..." }

HTTP/1.1 201 Created
Location: /wia-social/activity/act_01HX…
```

### A.3 Subscribe via WebSub

```http
POST /wia-social/feed/acct%3Aalice%40example.org/subscribe HTTP/1.1
Host: example.com
Content-Type: application/x-www-form-urlencoded

hub.callback=https%3A%2F%2Fmy.app%2Fcb&hub.mode=subscribe&hub.topic=https%3A%2F%2Fexample.com%2Fwia-social%2Ffeed%2Facct%3Aalice%40example.org

HTTP/1.1 202 Accepted
```

The subscriber's callback then receives an HTTP `GET` from the hub asking
it to confirm intent; the subscriber MUST echo `hub.challenge`.

## Appendix B — Conformance Test Suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-social-conformance`. The suite walks
through:

1. Discovery document round-trip.
2. Bundle publish, read, replace, revoke.
3. Activity submit with idempotency key collision.
4. Feed cursor pagination across 1 000 items.
5. WebSub intent verification.
6. Rate-limit header presence and exhaustion behaviour.
7. Problem-detail format on every 4xx/5xx path.

Hosts publishing a `bridge_profile=Full` SHOULD additionally pass the
suite's bridge-extension tests for at least one network.

## Appendix C — Operational Notes

* Hosts MUST persist incoming activities and bundle writes durably before
  acknowledging them; an at-most-once acknowledgement followed by storage
  loss would silently break federation.
* Hosts SHOULD maintain a 30-day retention for problem-detail responses
  emitted to identified peers, to support cross-host debugging.
* Hosts SHOULD expose a `/wia-social/health` endpoint returning `200 OK`
  with `{"status":"ok"}` for load-balancer probes; the endpoint MUST NOT
  be advertised in `/.well-known/wia-social` and MUST NOT count toward
  rate limits.

弘益人間 — Benefit All Humanity.
