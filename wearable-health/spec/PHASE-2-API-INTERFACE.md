# WIA-MED-020 Wearable Health — Phase 2: API Interface

**Standard**: WIA-MED-020 Wearable Health Monitoring
**Phase**: 2 of 4 — API Interface
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 2 specifies the HTTP surface that a wearable health host
(typically a vendor cloud, a clinical-monitoring platform, or a hospital
RPM bridge) exposes for the data families defined in Phase 1.

Out of scope: federation between vendors (Phase 3), HealthKit / Google
Fit / FHIR bridges (Phase 4).

---

## 2. Transport

* HTTPS only, TLS 1.3 (IETF RFC 8446) or later.
* HTTP/2 RECOMMENDED, HTTP/1.1 SHALL be supported as fallback for
  legacy device sync paths.
* `Strict-Transport-Security` per IETF RFC 6797.
* Compression: `gzip`, `br`.

---

## 3. Discovery

```
GET https://<host>/.well-known/wia-wearable-health

{
  "wia_wearable_health_version": "1.0.0",
  "host_id": "did:wia:wearable-host:hospital-rpm",
  "endpoints": {
    "device":      "https://wh.example/wh/device",
    "sync":        "https://wh.example/wh/sync",
    "measurement": "https://wh.example/wh/measurement",
    "session":     "https://wh.example/wh/session",
    "alert":       "https://wh.example/wh/alert",
    "calibration": "https://wh.example/wh/calibration"
  },
  "supported_signatures": ["Ed25519"],
  "supported_measurement_types": ["heart_rate","step_count","sleep_stage","spo2","ecg","glucose","sbp","dbp","temp","weight"],
  "rate_limits": {
    "anonymous":     { "rps": 1,  "burst": 5 },
    "authenticated": { "rps": 50, "burst": 200 }
  }
}
```

Discovery responses are cacheable for 300 s.

---

## 4. Device Endpoints

```
PUT  /wh/device/{device_id}     (signed; first publication or update)
GET  /wh/device/{device_id}
GET  /wh/device?patient_id={id}
DELETE /wh/device/{device_id}   (signed; tombstone, audit retention)
```

The host MUST verify the HTTP message signature and the device record's
detached signature on every write.

---

## 5. Sync Endpoint

```
POST /wh/sync                    (signed; bulk measurement upload)
Content-Type: application/json
Idempotency-Key: <opaque>
```

The body is a sync envelope carrying many measurements at once,
typical of a periodic device sync operation:

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "sync",
  "device_id": "did:wia:device:apple-watch-S9-01HXY",
  "synced_at": "2026-04-27T10:00:00Z",
  "measurements": [
    { "measurement_type": "heart_rate", "captured_at": "2026-04-27T09:55:00Z", "value": 72, "unit": "/min" },
    { "measurement_type": "step_count", "captured_at": "2026-04-27T09:55:00Z", "value": 3450, "unit": "1" }
  ],
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

The host MUST honour `Idempotency-Key` per IETF draft
`idempotency-key-header`. Repeating the same key with the same body
yields the same status; with a different body MUST yield
`422 Unprocessable Entity`.

---

## 6. Measurement Endpoints

```
GET /wh/measurement/{measurement_id}
GET /wh/measurement?patient_id={id}&type={t}&from={ts}&to={ts}&since={cursor}
```

Streaming variant uses Server-Sent Events:

```
GET /wh/measurement/stream/{device_id}
Accept: text/event-stream
```

Reconnecting clients pass the most recent event id as `Last-Event-Id`
per WHATWG HTML §9.2.

---

## 7. Session Endpoints

```
POST  /wh/session                (signed)
GET   /wh/session/{session_id}
PATCH /wh/session/{session_id}   (signed; summary updates allowed until 24h after end)
GET   /wh/session?patient_id={id}&kind={k}
```

---

## 8. Alert Endpoints

```
POST /wh/alert                   (signed)
GET  /wh/alert/{alert_id}
GET  /wh/alert?patient_id={id}&severity={s}&since={cursor}
```

Critical alerts MUST be persisted before acknowledgement; the host
MUST page on-call clinicians per the patient's consent envelope
within 60 seconds of receipt for `severity = critical`.

---

## 9. Calibration Endpoints

```
PUT  /wh/calibration/{calibration_id}     (signed)
GET  /wh/calibration/{calibration_id}
GET  /wh/calibration?device_id={id}
GET  /wh/calibration/due?within={ISO-8601-duration}&clinical_grade=true
```

The `due?within=` query is the typical morning-stand-up query for the
clinical-grade device fleet.

---

## 10. Authentication

* Writes require HTTP Message Signatures per IETF RFC 9421.
* Reads MAY be anonymous subject to the discovery rate limit; PHI-bearing
  reads (any payload tied to a patient identifier) MUST be authenticated.
* Cross-vendor reads use Phase 3 federation receipts.

---

## 11. Errors

Errors follow IETF RFC 9457 problem details. Reserved error types live
under `https://wiastandards.com/wearable-health/errors/`.

```json
{
  "type": "https://wiastandards.com/wearable-health/errors/clinical-grade-without-clearance",
  "title": "Clinical grade claim without regulatory clearance reference",
  "status": 422,
  "detail": "device.clinical_grade=true but no fda_510k_number / ce_mdr_number / kfda_number",
  "instance": "/wh/device/did:wia:device:cgm-G7-01HXY"
}
```

---

## 12. Conformance

A Phase 2 conformant host MUST:

1. Publish `/.well-known/wia-wearable-health` with all endpoints.
2. Implement device / sync / measurement / session / alert /
   calibration routes with the documented status codes.
3. Stream measurements over SSE with `Last-Event-Id` recovery.
4. Honour `Idempotency-Key` for all writes.
5. Refuse `clinical_grade=true` device claims without clearance proof.
6. Emit problem-detail JSON for 4xx/5xx responses.

---

## 13. References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 6797 — HSTS
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 9457 — Problem Details for HTTP APIs
* WHATWG HTML — Server-Sent Events
* HL7 FHIR R5 — Observation resource
* JSON Schema Draft 2020-12

---

## Appendix A — Reference Request / Response Pairs

### A.1 Bulk sync from a fitness tracker

```http
POST /wh/sync HTTP/1.1
Host: wh.example
Content-Type: application/json
Idempotency-Key: 7e3a9b6c-…
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_wearable_health_version":"1.0.0", "type":"sync", "device_id":"…",
  "synced_at":"2026-04-27T10:00:00Z",
  "measurements":[ { "measurement_type":"step_count", "value":12345, "unit":"1" } ],
  "signature":{ "alg":"Ed25519", "value":"…" } }

HTTP/1.1 201 Created
Location: /wh/sync/sync_01HXY
RateLimit-Limit: 50
RateLimit-Remaining: 49
```

### A.2 Stream from a continuous glucose monitor

```http
GET /wh/measurement/stream/did%3Awia%3Adevice%3Acgm-G7-01HXY HTTP/1.1
Host: wh.example
Accept: text/event-stream
Authorization: WIA-Sig keyid="…",signature="…"

HTTP/1.1 200 OK
Content-Type: text/event-stream

id: msr_01
event: measurement
data: { "measurement_type":"glucose", "value":120, "unit":"mg/dL", "...":"..." }

id: msr_02
event: measurement
data: { "measurement_type":"glucose", "value":118, "unit":"mg/dL", "...":"..." }
```

### A.3 Submit a critical alert

```http
POST /wh/alert HTTP/1.1
Host: wh.example
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_wearable_health_version":"1.0.0", "type":"alert",
  "patient_id":"did:wia:patient:01HXY", "severity":"critical",
  "rule":"glucose < 54 mg/dL", "evidence_measurement_ids":["msr_07"],
  "actions_taken":["notify_patient"], "actions_pending":["page_oncall"],
  "signature":{ "alg":"Ed25519", "value":"…" } }

HTTP/1.1 201 Created
Location: /wh/alert/alt_01HXY
```

## Appendix B — Conformance Test Suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-wearable-health-conformance` and
walks through:

1. Discovery document round-trip.
2. Device publish, read, withdraw with clearance refusal path.
3. Bulk sync with idempotency key collision.
4. Measurement query with type and time-window filters.
5. Stream subscribe / reconnect with `Last-Event-Id`.
6. Session create + summary patch within 24h window.
7. Alert submission with critical-severity paging assertion.
8. Calibration `due?within=` query.
9. Rate-limit headers and exhaustion behaviour.

## Appendix C — Operational Recommendations

* Hosts MUST persist incoming writes durably before acknowledging.
* Hosts SHOULD provide a separate read-replica endpoint for high-rate
  measurement queries so writes from devices stay responsive.
* Hosts SHOULD expose a `/wh/health` liveness probe outside the public
  rate-limit accounting; tooling consumes it but it MUST NOT be
  advertised in `/.well-known/wia-wearable-health`.
* Hosts SHOULD retain problem-detail responses emitted to identified
  peers for 30 days to support cross-host debugging during disputes.

## Appendix D — Reserved Endpoint Paths

Implementations MUST NOT serve unrelated content under the following
prefixes; future minor versions reserve them for additional features.

| Prefix | Reserved for |
|--------|--------------|
| `/wh/audit/`     | Future audit-log read endpoint |
| `/wh/integrity/` | Future cryptographic transparency log |
| `/wh/regulator/` | Future regulator inbound feed for adverse-event reporting |
| `/wh/research/`  | Future de-identified research export |
| `/wh/family/`    | Future family / caregiver view endpoint |

Hosts that need to surface unrelated tooling on the same origin MUST
use a distinct subdomain or path prefix outside the reserved namespace.

## Appendix E — Operational Telemetry

A conformant host SHOULD expose Prometheus-style metrics on a separate
port:

| Metric | Type | Labels |
|--------|------|--------|
| `wia_wh_devices_registered_total` | counter | `manufacturer` |
| `wia_wh_measurements_received_total` | counter | `type`, `clinical_grade` |
| `wia_wh_alerts_emitted_total` | counter | `severity` |
| `wia_wh_alert_paging_latency_seconds` | histogram | `severity` |
| `wia_wh_consent_violations_total` | counter | `attempted_audience` |
| `wia_wh_calibration_overdue_devices` | gauge | none |
| `wia_wh_request_duration_seconds` | histogram | `route`, `status` |

Telemetry MUST NOT include patient identifiers, device serials, or
raw measurement values as label values.

## Appendix F — Backwards-Compatibility Promise

Within the 1.x line every endpoint listed in this document MUST remain
reachable and MUST continue to honour the documented status codes and
content shapes. Hosts MAY add optional query parameters, response
fields, new endpoints, or media types. Hosts MUST NOT remove or
repurpose existing ones. Breaking changes ride a major version bump
and MUST be preceded by a 12-month deprecation window per IETF
RFC 8594 and RFC 9745; deprecated routes MUST emit `Deprecation` and
`Sunset` headers throughout the window.

## Appendix G — Health and Readiness Probes

A conformant host SHOULD expose three liveness signals on a separate
port (default 9091, distinct from the public service port):

| Probe | Path | Payload |
|-------|------|---------|
| Liveness  | `/healthz`  | `{"status":"ok"}` |
| Readiness | `/readyz`   | `{"deps":{"db":"ok","queue":"ok","alert_pager":"ok"}}` |
| Startup   | `/startupz` | `{"phase":"warming","ready_at":"2026-…"}` |

These MUST NOT be advertised in `/.well-known/wia-wearable-health`
and MUST NOT be subject to public rate limits. Internal observability
tooling consumes them; nothing external should rely on their presence.

The readiness endpoint's `deps` map MUST include every external
dependency that, if down, would cause the host to fail to serve
authenticated writes — typically the measurement store, the alert
queue, the consent store, and (for clinical hosts) the on-call
paging integration.

## Appendix H — CORS Policy

* Public read endpoints (`GET /wh/measurement/...` for aggregate
  data) MUST send `Access-Control-Allow-Origin: *` so unauthenticated
  tooling can fetch them cross-origin.
* Authenticated write endpoints MUST validate the request `Origin`
  against an allow-list configured by the host operator; the
  allow-list MUST be explicit (no wildcards for credentialed
  requests).
* Pre-flight responses MUST cache for 600 seconds via
  `Access-Control-Max-Age` to avoid repeated OPTIONS overhead from
  busy clinical dashboards refreshing measurement views.

弘益人間 — Benefit All Humanity.
