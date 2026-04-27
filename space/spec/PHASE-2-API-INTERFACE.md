# WIA-SPACE — Phase 2: API Interface

**Standard**: WIA Space (general spaceflight operations)
**Phase**: 2 of 4 — API Interface
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 2 specifies the HTTP surface that a space-operations host (mission
control, ground station, satellite operator, ground-segment provider)
exposes for the data families defined in Phase 1: vehicles, missions,
manifests, telemetry, conjunction warnings, and incident reports. Out of
scope: federation between operators (Phase 3), regulator and SAR bridges
(Phase 4).

---

## 2. Transport

* HTTPS only, TLS 1.3 (IETF RFC 8446) or later. Plaintext HTTP MUST be
  rejected with `426 Upgrade Required`.
* HTTP/2 RECOMMENDED (IETF RFC 9113); HTTP/1.1 SHALL be supported as
  fallback for legacy ground systems.
* `Strict-Transport-Security: max-age=31536000; includeSubDomains` per
  IETF RFC 6797.
* Compression: `gzip`, `br` per the negotiated `Accept-Encoding`.
* Operators MAY publish a DNS SRV record `_wia-space._tcp` for tooling
  discovery in addition to the discovery document below.

---

## 3. Discovery

Every WIA Space host MUST publish a discovery document at:

```
GET https://<operator>/.well-known/wia-space
```

```json
{
  "wia_space_version": "1.0.0",
  "operator_id": "did:wia:operator:ksc-launch",
  "endpoints": {
    "vehicle":     "https://ops.example/space/vehicle",
    "mission":     "https://ops.example/space/mission",
    "manifest":    "https://ops.example/space/manifest",
    "telemetry":   "https://ops.example/space/telemetry",
    "conjunction": "https://ops.example/space/conjunction",
    "incident":    "https://ops.example/space/incident"
  },
  "supported_signatures": ["Ed25519"],
  "supported_mission_classes": ["sub-orbital", "orbital", "lunar", "deep-space"],
  "rate_limits": {
    "anonymous":     { "rps": 1,  "burst": 5 },
    "authenticated": { "rps": 30, "burst": 100 }
  }
}
```

Discovery responses MUST be cacheable for at least 300 seconds via
`Cache-Control: public, max-age=300`. Clients MUST honour
`must-revalidate` when present.

---

## 4. Vehicle Endpoints

### 4.1 Read

```
GET /space/vehicle/{vehicle_id}
Accept: application/json
```

Returns the vehicle record (Phase 1 §Vehicle).

### 4.2 Publish / Update

```
PUT /space/vehicle/{vehicle_id}
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"
```

The host MUST verify both the HTTP message signature and the vehicle
record's detached signature.

### 4.3 Withdraw

```
DELETE /space/vehicle/{vehicle_id}
Authorization: WIA-Sig …
```

Withdrawn vehicles remain in audit storage for the regulatory retention
period (default 25 years for orbital and beyond, 7 years for sub-orbital).
Reads after withdrawal return `410 Gone`.

---

## 5. Mission Endpoints

```
POST  /space/mission             — create new mission
GET   /space/mission/{mission_id}
PATCH /space/mission/{mission_id} — JSON Patch (RFC 6902)
GET   /space/mission?vehicle_id={id}&since={cursor}
```

Mission state transitions (DRAFT → REVIEWED → FROZEN → LAUNCHED → RECOVERED
→ ARCHIVED, or ABORTED) ride as PATCH operations and MUST be signed by a
delegate with the `mission_state` scope.

---

## 6. Manifest Endpoints

```
POST  /space/manifest
GET   /space/manifest/{manifest_id}?include=insurance
PATCH /space/manifest/{manifest_id}     — until T-2h before launch
```

The `include` parameter requests inline expansion of related records.
Allowed values: `insurance`, `passenger`, `mission`, `vehicle`.

---

## 7. Telemetry Stream

```
GET /space/telemetry/{mission_id}?since={cursor}
Accept: text/event-stream
```

Server-Sent Events (WHATWG HTML §9.2) carry per-second telemetry frames.
Frames include flight phase, position, velocity, vehicle G-load, ECLSS
gauges (when applicable), and propulsion telemetry. The stream is
throttled to 1 Hz for public audiences and unlimited for authenticated
mission-control audiences.

Reconnecting clients pass the most recent event id as `Last-Event-Id`
per WHATWG HTML §9.2; the host resumes from the next available frame
or emits a `gap_marker` event for the missing range.

---

## 8. Conjunction Warning Endpoints

```
POST /space/conjunction              — submit a conjunction warning
GET  /space/conjunction/{warning_id}
GET  /space/conjunction?vehicle_id={id}&since={timestamp}
```

Conjunction warnings carry orbital-debris encounter probabilities for
satellite operators. Phase 4 §SAR specifies how these flow to space
situational awareness providers.

---

## 9. Incident Endpoints

```
POST  /space/incident
GET   /space/incident/{incident_id}
GET   /space/incident?mission_id={id}&severity={s}
PATCH /space/incident/{incident_id}  — disposition update (signed)
```

Incidents follow the severity ladder from Phase 1 §Severity:
`observation`, `anomaly`, `incident`, `accident`. Operators MUST publish
at least an `observation`-level summary within 30 days of every flight;
`incident` and above MUST be pushed to the regulator within 24 hours.

---

## 10. Authentication

* Operator-internal calls use HTTP Message Signatures per IETF RFC 9421
  covering method, target URI, `Date`, `Content-Digest`, `Content-Length`.
* Cross-operator calls use Phase 3 federation receipts.
* Public read of mission summary endpoints is permitted; safety incidents
  marked `severity ≥ incident` MUST be public unless under regulator
  embargo.

---

## 11. Errors

Errors follow IETF RFC 9457 problem details. Reserved error types live
under `https://wiastandards.com/space/errors/`.

```json
{
  "type": "https://wiastandards.com/space/errors/mission-frozen",
  "title": "Mission profile is frozen",
  "status": 409,
  "detail": "mission state is FROZEN; create a new mission for changes",
  "instance": "/space/mission/msn_2026-04-27"
}
```

---

## 12. Rate Limiting

`RateLimit-Limit`, `RateLimit-Remaining`, `RateLimit-Reset` headers MUST
be emitted per IETF draft `draft-ietf-httpapi-ratelimit-headers`.
`429 Too Many Requests` is returned when exhausted, with `Retry-After`
per IETF RFC 7231 §7.1.3. Telemetry streams MAY apply per-stream
concurrency caps; the discovery document advertises the cap.

---

## 13. Conformance Checklist

A Phase 2 conformant operator MUST:

1. Publish `/.well-known/wia-space` with all endpoints listed above.
2. Implement vehicle / mission / manifest / telemetry / conjunction /
   incident routes with the documented status codes.
3. Stream telemetry over SSE with `Last-Event-Id` recovery.
4. Honour `Idempotency-Key` for all writes per IETF draft
   `idempotency-key-header`.
5. Emit problem-detail JSON for all 4xx/5xx responses.

The companion simulator at `simulator/index.html` exercises representative
flows for each surface.

---

## 14. References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 9113 — HTTP/2
* IETF RFC 6797 — HSTS
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 9457 — Problem Details for HTTP APIs
* IETF RFC 6902 — JSON Patch
* IETF RFC 7231 — HTTP/1.1 Semantics
* WHATWG HTML — Server-Sent Events
* JSON Schema Draft 2020-12

---

## Appendix A — Reference Request / Response Pairs

### A.1 Patch a mission state

```http
PATCH /space/mission/msn_2026-04-27 HTTP/1.1
Host: ops.example
Content-Type: application/json-patch+json
Authorization: WIA-Sig keyid="…",signature="…"

[{ "op": "replace", "path": "/state", "value": "FROZEN" }]

HTTP/1.1 200 OK
Content-Type: application/json
RateLimit-Limit: 30
RateLimit-Remaining: 29

{ "wia_space_version":"1.0.0", "type":"mission", "state":"FROZEN", "...":"..." }
```

### A.2 Stream telemetry

```http
GET /space/telemetry/msn_2026-04-27?since=cur_AAAA HTTP/1.1
Host: ops.example
Accept: text/event-stream
Authorization: WIA-Sig keyid="…",signature="…"

HTTP/1.1 200 OK
Content-Type: text/event-stream
Cache-Control: no-store

id: cur_AAAB
event: telemetry
data: {"phase":"ascent","t":42.0,"alt_km":17.3,"vel_kms":1.41,"g":3.6}

id: cur_AAAC
event: telemetry
data: {"phase":"ascent","t":43.0,"alt_km":18.7,"vel_kms":1.49,"g":3.7}
```

### A.3 Submit an incident

```http
POST /space/incident HTTP/1.1
Host: ops.example
Content-Type: application/json
Idempotency-Key: 7e3a9b6c-49d4-4d0a-8e0d-…
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_space_version":"1.0.0", "type":"incident",
  "mission_id":"msn_2026-04-27", "captured_at":"2026-04-27T09:33:12Z",
  "severity":"anomaly", "description":"CO2 scrubber bank A drew 12% above predicted current.",
  "signature":{ "alg":"Ed25519", "value":"…" } }

HTTP/1.1 201 Created
Location: /space/incident/inc_01HXY…
```

## Appendix B — Conformance Test Suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-space-conformance` and walks
through:

1. Discovery document round-trip.
2. Vehicle publish, read, withdraw.
3. Mission lifecycle transitions per Phase 3.
4. Manifest reservation with idempotency key collision.
5. Telemetry stream cursor recovery after a forced disconnect.
6. Conjunction warning submission and listing.
7. Incident submission for each severity level and disposition update.
8. Rate-limit headers and exhaustion behaviour.

Operators publishing `integration_profile=Full` SHOULD additionally pass
the suite's regulator and SAR adapter tests for at least one peer.

弘益人間 — Benefit All Humanity.
