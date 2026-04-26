# WIA-SPACE-014 — Phase 2: API Interface Specification

**Standard**: WIA-SPACE-014 (Space Tourism)
**Phase**: 2 of 4 — API Interface
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 2 specifies the HTTP surface that operators expose for the data
families defined in Phase 1: passenger records, vehicle manifests, mission
profiles, safety events and insurance records. Out of scope: protocol
state across operators (Phase 3), regulator integration (Phase 4).

---

## 2. Transport

* HTTPS only, TLS 1.3 (IETF RFC 8446) or later.
* HTTP/2 RECOMMENDED, HTTP/1.1 SHALL be supported as fallback.
* `Strict-Transport-Security` per IETF RFC 6797.
* Compression: `gzip`, `br`.
* Operators MUST publish their API base URL via DNS SRV record
  `_wia-space-tourism._tcp` for tooling discovery.

---

## 3. Discovery

```
GET https://<operator>/.well-known/wia-space-tourism
```

```json
{
  "wia_space_tourism_version": "1.0.0",
  "operator_id": "did:wia:operator:lev",
  "endpoints": {
    "passenger": "https://lev.example/wst/passenger",
    "manifest":  "https://lev.example/wst/manifest",
    "mission":   "https://lev.example/wst/mission",
    "safety":    "https://lev.example/wst/safety",
    "insurance": "https://lev.example/wst/insurance",
    "telemetry": "https://lev.example/wst/telemetry"
  },
  "supported_signatures": ["Ed25519"],
  "fitness_classes": ["Class-1 Sub-orbital", "Class-2 Sub-orbital", "Class-3 Orbital"],
  "rate_limits": {
    "anonymous":     { "rps": 1,  "burst": 5 },
    "authenticated": { "rps": 20, "burst": 60 }
  }
}
```

Discovery responses are cacheable for 300 s.

---

## 4. Passenger Endpoints

### 4.1 Read

```
GET /wst/passenger/{passenger_id}
```

Returns the passenger record. Personally identifying fields are redacted
unless the caller presents a credential allowing the disclosure (Phase 3
§9 audience controls).

### 4.2 Submit

```
PUT /wst/passenger/{passenger_id}
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"
```

Operators MUST verify signatures using the issuer's published key.
Idempotency-Key MUST be honoured. Body must be a valid passenger_record
per Phase 1 §3.

### 4.3 Withdraw

```
DELETE /wst/passenger/{passenger_id}
```

Withdrawn records remain in audit storage for the regulatory retention
period (default 7 years for sub-orbital, 25 years for orbital and beyond).
Reads after withdrawal return `410 Gone`.

---

## 5. Vehicle Manifest Endpoints

### 5.1 Compose

```
POST /wst/manifest
Content-Type: application/json
Idempotency-Key: <opaque>
```

The body is a complete `vehicle_manifest`. Operators MUST verify that
each `occupant_id` references an active passenger record.

### 5.2 Amend

```
PATCH /wst/manifest/{manifest_id}
Content-Type: application/json-patch+json
```

Patches use IETF RFC 6902 JSON Patch. Amendments are accepted up to T-2
hours before launch unless overridden by operator policy.

### 5.3 Read

```
GET /wst/manifest/{manifest_id}?include=insurance
```

The `include` parameter requests inline expansion of related records.
Allowed values: `insurance`, `passenger`, `mission`.

---

## 6. Mission Profile Endpoints

```
GET /wst/mission/{mission_id}
PUT /wst/mission/{mission_id}
```

Mission profiles are immutable once `frozen_at` is set. Frozen profiles
MUST be re-published under a new `mission_id` if changes are required.

---

## 7. Safety Event Endpoints

```
POST /wst/safety
GET  /wst/safety/{event_id}
GET  /wst/safety?mission_id={mission_id}
```

* Operators MUST accept `safety_event` envelopes signed by any
  on-mission crew member, ground controller, or passenger.
* Operators MUST NOT delete safety events. Corrections are appended as
  follow-up events with `corrects: <event_id>`.

---

## 8. Insurance Endpoints

```
GET  /wst/insurance/{policy_id}
PUT  /wst/insurance/{policy_id}
POST /wst/insurance/{policy_id}/claim
```

Claim submissions return `202 Accepted` with a claim id; the underwriter
endpoint at `claims_endpoint` (Phase 1 §7) handles further processing.

---

## 9. Telemetry Stream

```
GET /wst/telemetry/{mission_id}?since={cursor}
Accept: text/event-stream
```

Server-Sent Events (WHATWG HTML §9.2) carry per-second telemetry frames.
Frames include flight phase, altitude, velocity, vehicle G-load and
selected ECLSS gauges. The stream is throttled to 10 Hz for public
audiences and unlimited for ground-control audiences.

---

## 10. Authentication

* Operator-internal calls use HTTP Message Signatures per IETF RFC 9421.
* Cross-operator calls use Phase 3 federation receipts.
* Public read of mission summary endpoints is permitted; safety events
  marked `severity ≥ incident` MUST be public unless under regulator
  embargo.

---

## 11. Errors

Errors follow IETF RFC 9457 problem details. Reserved error types live
under `https://wiastandards.com/space-tourism/errors/`.

```json
{
  "type": "https://wiastandards.com/space-tourism/errors/passenger-medical-expired",
  "title": "Passenger medical certificate expired",
  "status": 422,
  "detail": "passenger.medical.last_examined_at is older than 12 months for Class-2",
  "instance": "/wst/passenger/did:wia:passenger:01HXY…"
}
```

---

## 12. Rate Limiting

`RateLimit-Limit`, `RateLimit-Remaining`, `RateLimit-Reset` headers MUST
be emitted. `429 Too Many Requests` is returned when exhausted, with
`Retry-After`.

---

## 13. Conformance Checklist

A Phase 2 conformant operator MUST:

1. Publish `/.well-known/wia-space-tourism` with all endpoints.
2. Implement passenger / manifest / mission / safety / insurance routes.
3. Stream telemetry over SSE.
4. Honour Idempotency-Key for all writes.
5. Emit problem-detail JSON for 4xx/5xx.

The companion simulator at `simulator/index.html` exercises every route.

---

## 14. References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 6797 — HSTS
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 9457 — Problem Details for HTTP APIs
* IETF RFC 6902 — JSON Patch
* WHATWG HTML — Server-Sent Events
* JSON Schema Draft 2020-12

---

## Appendix A — Reference Request / Response Pairs

### A.1 Reserve a manifest seat

```http
POST /wst/manifest/reserve HTTP/1.1
Host: lev.example
Content-Type: application/json
Idempotency-Key: 7e3a9b6c-49d4-4d0a-8e0d-…
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_space_tourism_version":"1.0.0",
  "passenger_id":"did:wia:passenger:01HYG…",
  "mission_id":"msn_2026-05-12-ORB07-01",
  "preferred_seat":"window" }

HTTP/1.1 201 Created
Location: /wst/manifest/vmfst_01HYG…/seats/S2
RateLimit-Limit: 20
RateLimit-Remaining: 19
RateLimit-Reset: 53

{ "seat_id":"S2", "manifest_id":"vmfst_01HYG…", "hold_until":"2026-02-01T01:00:00Z" }
```

### A.2 Submit a safety event

```http
POST /wst/safety HTTP/1.1
Host: lev.example
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_space_tourism_version":"1.0.0",
  "type":"safety_event",
  "mission_id":"msn_2026-04-27-LEV03-01",
  "captured_at":"2026-04-27T09:33:12Z",
  "severity":"anomaly",
  "category":"ECLSS",
  "description":"CO2 scrubber bank A drew 12% above predicted current.",
  "auto_actions_taken":["bank-B switched in","alert dispatched"],
  "narrative_followup_required":true }

HTTP/1.1 201 Created
Location: /wst/safety/evt_01HXY…
```

### A.3 Stream telemetry

```http
GET /wst/telemetry/msn_2026-04-27-LEV03-01?since=cur_AAAA HTTP/1.1
Host: lev.example
Accept: text/event-stream

HTTP/1.1 200 OK
Content-Type: text/event-stream
Cache-Control: no-store

id: cur_AAAB
event: telemetry
data: {"phase":"ascent","t":42.0,"alt_km":17.3,"vel_kms":1.41,"g":3.6,"o2_kpa":34.2}

id: cur_AAAC
event: telemetry
data: {"phase":"ascent","t":43.0,"alt_km":18.7,"vel_kms":1.49,"g":3.7,"o2_kpa":34.1}
```

### A.4 Withdraw a passenger record

```http
DELETE /wst/passenger/did:wia:passenger:01HYG… HTTP/1.1
Host: lev.example
Authorization: WIA-Sig keyid="…",signature="…"

HTTP/1.1 204 No Content
```

Subsequent reads return `410 Gone` for the audit retention period.

## Appendix B — Conformance Test Suite

Black-box tests live at
`https://github.com/WIA-Official/wia-space-tourism-conformance` and walk
through:

1. Discovery document round-trip.
2. Passenger record publish, read, withdraw.
3. Manifest reservation with idempotency key collision.
4. Manifest patch and freeze transition.
5. Mission lifecycle transitions per Phase 3 §4.
6. Safety event submission for each severity level.
7. Telemetry stream cursor recovery after reconnect.
8. Insurance claim happy path and denied path.
9. Rate limit headers and exhaustion behaviour.

Operators publishing `integration_profile=Full` SHOULD additionally pass
the suite's regulator and SAR adapter tests.

## Appendix C — Operational Recommendations

* Operators MUST persist incoming writes durably before acknowledging.
* Hosts SHOULD emit a `/wst/health` liveness probe outside the public
  rate-limit accounting.
* Hosts SHOULD provide a separate read-replica endpoint for
  high-cardinality reads (telemetry, safety events) so write paths remain
  responsive.
* Hosts SHOULD expose a 30-day retention of problem-detail responses
  emitted to identified peers, for cross-host debugging.

## Appendix D — Reserved Endpoint Paths

Implementations MUST NOT serve unrelated content under the following
prefixes; future minor versions reserve them for additional WIA-SPACE-014
features.

| Prefix | Reserved for |
|--------|--------------|
| `/wst/regulator/` | Phase 3 regulator inbound feeds and licence checks |
| `/wst/sar/`       | Phase 3 SAR contingency notices |
| `/wst/range/`     | Phase 4 range-safety bridges |
| `/wst/biometric/` | Future passenger biometrics endpoint (consent-gated) |
| `/wst/integrity/` | Future cryptographic transparency log |

Hosts that must surface unrelated tooling on the same origin MUST use a
distinct subdomain or path prefix outside the reserved namespace.

## Appendix E — Pagination and Cursor Semantics

`GET /wst/manifest?since=…` and `GET /wst/safety?since=…` use opaque
cursors. The response shape is:

```json
{
  "wia_space_tourism_version": "1.0.0",
  "items": [ /* … */ ],
  "next_cursor": "AAAB",
  "approx_total": 312,
  "page_kind": "forward"
}
```

* Cursors are opaque base64url; clients MUST NOT parse them.
* The pagination is monotonic by server-side write order.
* Clients that reconnect MAY pass the most recent `next_cursor` to resume.
* Hosts SHOULD honour cursors for at least 7 days; stale cursors return
  `410 Gone` with a problem document of type `…/cursor-expired`.

The maximum page size is 200 items. Larger requests are silently capped.

## Appendix F — Health and Readiness Probes

Operators SHOULD expose three liveness signals on a separate port:

| Probe | Path | Payload |
|-------|------|---------|
| Liveness  | `/healthz`  | `{"status":"ok"}` |
| Readiness | `/readyz`   | `{"deps":{"db":"ok","queue":"ok"}}` |
| Startup   | `/startupz` | `{"phase":"warming","ready_at":"2026-…"}` |

These MUST NOT be advertised in `/.well-known/wia-space-tourism` and MUST
NOT be subject to public rate limits. Internal observability tooling
consumes them; nothing external should rely on their presence.

## Appendix G — Backwards Compatibility Promise

Within the 1.x line every endpoint listed in §4–§9 MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add new optional query parameters, response fields,
endpoints, or media types. Hosts MUST NOT remove or repurpose existing
ones. Breaking changes ride a major version bump and MUST be preceded by
a 12-month deprecation window per §11; deprecated routes MUST emit
`Deprecation` and `Sunset` headers throughout the window.

弘益人間 — Benefit All Humanity.
