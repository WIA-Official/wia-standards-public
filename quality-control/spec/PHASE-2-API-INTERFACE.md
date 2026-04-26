# WIA-IND-025 Quality Control — Phase 2: API Interface

**Standard**: WIA-IND-025 (Quality Control)
**Phase**: 2 of 4 — API Interface
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 2 specifies the HTTP surface that a quality-system host (typically
a manufacturing site's QMS service) exposes for the data families
defined in Phase 1: inspection plans, inspection results, SPC samples,
defect records, calibration records, NCR/CAPA, and audit findings. Out
of scope: federation between sites (Phase 3), ERP/MES bridges (Phase 4).

---

## 2. Transport

* HTTPS only, TLS 1.3 (IETF RFC 8446) or later.
* HTTP/2 RECOMMENDED, HTTP/1.1 SHALL be supported as fallback.
* `Strict-Transport-Security` per IETF RFC 6797.
* Compression: `gzip`, `br`.

---

## 3. Discovery

Every host MUST publish a discovery document at:

```
GET https://<host>/.well-known/wia-quality-control
```

```json
{
  "wia_quality_control_version": "1.0.0",
  "site_id": "did:wia:site:bonghwa-line-A",
  "endpoints": {
    "plan":        "https://qc.example/qc/plan",
    "result":      "https://qc.example/qc/result",
    "spc":         "https://qc.example/qc/spc",
    "defect":      "https://qc.example/qc/defect",
    "calibration": "https://qc.example/qc/calibration",
    "ncr":         "https://qc.example/qc/ncr",
    "capa":        "https://qc.example/qc/capa",
    "audit":       "https://qc.example/qc/audit"
  },
  "supported_signatures": ["Ed25519"],
  "qms_standards": ["ISO 9001", "IATF 16949"],
  "rate_limits": {
    "anonymous":     { "rps": 1,  "burst": 5 },
    "authenticated": { "rps": 50, "burst": 200 }
  }
}
```

Discovery responses are cacheable for 300 s.

---

## 4. Inspection Plan Endpoints

### 4.1 Read

```
GET /qc/plan/{plan_id}
```

### 4.2 Publish

```
PUT /qc/plan/{plan_id}
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"
Idempotency-Key: <opaque>
```

The host MUST verify both the HTTP message signature and the plan's
detached signature.

### 4.3 Withdraw

```
DELETE /qc/plan/{plan_id}
```

Withdrawn plans remain in audit storage for the regulatory retention
period (7 years default for ISO 9001 contexts; 15 years for medical
device contexts under ISO 13485). Reads after withdrawal return
`410 Gone`.

---

## 5. Inspection Result Endpoints

```
POST /qc/result            — submit a new inspection result
GET  /qc/result/{result_id}
GET  /qc/result?lot_id={id}
```

* Submissions are immutable; corrections are appended as a new envelope
  whose `corrects` references the original.
* Hosts MUST persist before acknowledging.
* Idempotency-Key MUST be honoured per IETF draft `idempotency-key-header`.

---

## 6. SPC Sample Endpoints

```
POST /qc/spc                — submit one subgroup sample
GET  /qc/spc/{sample_id}
GET  /qc/spc?chart_id={id}&since={cursor}
```

Streaming variant uses Server-Sent Events:

```
GET /qc/spc/stream?chart_id={id}
Accept: text/event-stream
```

Each event payload is a `spc_sample` envelope. Reconnecting clients pass
the most recent event id as `Last-Event-Id` per WHATWG HTML §9.2.

---

## 7. Defect Endpoints

```
POST /qc/defect
GET  /qc/defect/{defect_id}
GET  /qc/defect?lot_id={id}&category={cat}&severity={sev}
```

Filters compose with logical AND. Cursor pagination as in Appendix D.

---

## 8. Calibration Endpoints

```
PUT  /qc/calibration/{record_id}
GET  /qc/calibration/{record_id}
GET  /qc/calibration?equipment_id={id}
GET  /qc/calibration/due?within={ISO-8601-duration}
```

The `due?within=` query returns records due within the requested window.
This is the typical morning-stand-up query for the calibration team.

---

## 9. NCR / CAPA Endpoints

```
POST /qc/ncr
GET  /qc/ncr/{ncr_id}
PATCH /qc/ncr/{ncr_id}     — disposition / closure (signed)
POST /qc/capa
GET  /qc/capa/{capa_id}
PATCH /qc/capa/{capa_id}   — effectiveness check signed update
```

NCR and CAPA closure require a signature from a role with the
`quality_manager` scope (Phase 3 §4).

---

## 10. Audit Endpoints

```
POST /qc/audit/finding
GET  /qc/audit/finding/{finding_id}
GET  /qc/audit/finding?audit_id={id}&status={s}
PATCH /qc/audit/finding/{finding_id}  — status update
```

Audit findings transition `open → in_progress → verification → closed`
or `open → withdrawn`.

---

## 11. Authentication

* Writes require HTTP Message Signatures per IETF RFC 9421 covering
  method, target URI, `Date`, `Content-Digest`, `Content-Length`.
* Reads MAY be anonymous subject to the discovery rate limit.
* Cross-site reads use Phase 3 federation receipts.

---

## 12. Errors

Errors follow IETF RFC 9457 problem details. Reserved error types live
under `https://wiastandards.com/quality-control/errors/`.

```json
{
  "type": "https://wiastandards.com/quality-control/errors/checkpoint-out-of-tolerance",
  "title": "Checkpoint observation out of tolerance",
  "status": 422,
  "detail": "checkpoint cp-001 value 10.071 exceeds tol_plus 0.05 from nominal 10.000",
  "instance": "/qc/result/res_01HZA"
}
```

---

## 13. Conformance

A Phase 2 conformant host MUST:

1. Publish `/.well-known/wia-quality-control` with all endpoints.
2. Implement plan / result / spc / defect / calibration / ncr / capa /
   audit routes.
3. Stream SPC samples over SSE.
4. Honour `Idempotency-Key` on all writes.
5. Emit problem-detail JSON for 4xx/5xx responses.

The companion simulator at `simulator/index.html` exercises representative
flows for each surface.

---

## 14. References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 6797 — HSTS
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 9457 — Problem Details for HTTP APIs
* WHATWG HTML — Server-Sent Events
* JSON Schema Draft 2020-12

---

## Appendix A — Reference Request / Response Pairs

### A.1 Submit an inspection result

```http
POST /qc/result HTTP/1.1
Host: qc.example
Content-Type: application/json
Idempotency-Key: 7e3a9b6c-49d4-4d0a-8e0d-…
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_quality_control_version":"1.0.0",
  "type":"inspection_result",
  "plan_id":"plan_01HZA…",
  "lot_id":"lot_2026-04-A",
  "started_at":"2026-04-01T09:30:00Z",
  "completed_at":"2026-04-01T10:05:00Z",
  "inspector_id":"did:wia:inspector:09…",
  "observations":[
    { "checkpoint_id":"cp-001", "value":10.012, "unit":"mm", "verdict":"pass" }
  ],
  "verdict":"pass",
  "signature":{ "alg":"Ed25519", "value":"…" } }

HTTP/1.1 201 Created
Location: /qc/result/res_01HZA…
RateLimit-Limit: 50
RateLimit-Remaining: 49
RateLimit-Reset: 53
```

### A.2 Stream SPC samples

```http
GET /qc/spc/stream?chart_id=chart_A-014_outer-dim HTTP/1.1
Host: qc.example
Accept: text/event-stream

HTTP/1.1 200 OK
Content-Type: text/event-stream
Cache-Control: no-store

id: spc_01HZA01
event: sample
data: { "wia_quality_control_version":"1.0.0", "type":"spc_sample", "...":"..." }

id: spc_01HZA02
event: sample
data: { "wia_quality_control_version":"1.0.0", "type":"spc_sample", "...":"..." }
```

### A.3 Open an NCR

```http
POST /qc/ncr HTTP/1.1
Host: qc.example
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_quality_control_version":"1.0.0",
  "type":"ncr",
  "opened_at":"2026-04-01T12:00:00Z",
  "detected_at":"in-process",
  "severity":"major",
  "description":"Threading depth out of spec on lot L-2026-04-A",
  "evidence_inspection_ids":["res_01HZA…"],
  "containment_action":"Quarantine lot L-2026-04-A; pause line A.",
  "capa_required":true,
  "signature":{ "alg":"Ed25519", "value":"…" } }

HTTP/1.1 201 Created
Location: /qc/ncr/ncr_01HZA…
```

## Appendix B — Conformance Test Suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-quality-control-conformance` and
walks through:

1. Discovery document round-trip.
2. Plan publish, read, withdraw.
3. Inspection result submit + auto-NCR triggering on first `fail`.
4. SPC sample stream subscribe / reconnect with `Last-Event-Id`.
5. Defect filter combinations.
6. Calibration `due?within=…` query.
7. NCR open → disposition → close flow.
8. CAPA effectiveness check transition.
9. Audit finding open → closed flow.
10. Rate-limit headers and exhaustion behaviour.

## Appendix C — Operational Recommendations

* Hosts MUST persist incoming writes durably before acknowledging.
* Hosts SHOULD provide a separate read-replica endpoint for
  high-cardinality reads (SPC sample history, audit evidence) so write
  paths stay responsive during shift change.
* Hosts SHOULD expose a `/qc/health` liveness probe outside the public
  rate-limit accounting; tooling consumes it but it MUST NOT be
  advertised in `/.well-known/wia-quality-control`.
* Hosts SHOULD retain problem-detail responses emitted to identified
  peers for 30 days to support cross-site debugging during disputes.

## Appendix D — Pagination Semantics

Cursor-based pagination as in WIA-SOCIAL Phase 2 §6.1. Responses include
`next_cursor`, `approx_total`, and `page_kind` fields. Cursors are opaque
base64url; clients MUST NOT parse them. Hosts SHOULD honour cursors for
at least 7 days; stale cursors return `410 Gone` with a problem document
of type `…/cursor-expired`.

## Appendix E — Reserved Endpoint Paths

Implementations MUST NOT serve unrelated content under the following
prefixes; future minor versions reserve them for additional features.

| Prefix | Reserved for |
|--------|--------------|
| `/qc/incoming/`     | Phase 3 supplier-side incoming material certificates |
| `/qc/coc/`          | Phase 3 outgoing certificate of conformance endpoints |
| `/qc/certifications/` | Phase 4 third-party certification lookup |
| `/qc/integrity/`    | Future cryptographic transparency log |
| `/qc/regulator/`    | Future regulator inbound feed |

Hosts that need to surface unrelated tooling on the same origin MUST use
a distinct subdomain or path prefix outside the reserved namespace.

## Appendix F — Operational Telemetry

A conformant host SHOULD expose Prometheus-style metrics on a separate
port:

| Metric | Type | Labels |
|--------|------|--------|
| `wia_qc_inspections_received_total` | counter | `verdict` |
| `wia_qc_spc_samples_received_total` | counter | `chart_id` |
| `wia_qc_ncr_opened_total` | counter | `severity` |
| `wia_qc_ncr_closed_total` | counter | `disposition` |
| `wia_qc_capa_effectiveness_total` | counter | `result` |
| `wia_qc_request_duration_seconds` | histogram | `route`, `status` |
| `wia_qc_seen_nonce_cache_size` | gauge | none |

Telemetry MUST NOT include personal identifiers as label values; doing
so would violate the privacy promise of Phase 3 §8. Operators that need
per-inspector debugging SHOULD use the per-peer 30-day problem-detail
retention from Appendix C rather than baking identities into telemetry.

## Appendix G — Backwards Compatibility Promise

Within the 1.x line every endpoint listed in §4–§10 MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional query parameters, response fields, new
endpoints, or media types. Hosts MUST NOT remove or repurpose existing
ones. Breaking changes ride a major version bump and MUST be preceded by
a 12-month deprecation window per IETF RFC 8594 and RFC 9745;
deprecated routes MUST emit `Deprecation` and `Sunset` headers
throughout the window. Hosts SHOULD additionally publish a deprecation
calendar on their press page (`press/deprecations.md`) so downstream
integrators can plan migration windows in advance and avoid surprise
cut-overs in the middle of a production run.

弘益人間 — Benefit All Humanity.
