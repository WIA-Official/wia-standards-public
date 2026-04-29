# WIA-LANG-005 PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-LANG-005
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a hydrogen-
energy programme exposes for the records defined in PHASE-1.
Consumers include guarantee-of-origin schemes, midstream
operators, refuelling-station owners, industrial offtakers,
gas-network operators, regulators, and the operator's own
analytics and audit platforms.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- ISO 14687:2019 (hydrogen fuel quality)
- ISO 19880-1:2020 (HRS general requirements)
- SAE J2601 / J2719 / J2799

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The OpenAPI
3.1 document at `/v1/openapi.json` is canonical.

Spectrum-class device-to-device protocols (HRS-to-vehicle SAE
J2799 IRDA / IR communication, on-station IIoT field bus)
are not exposed through this API; this API is the
metadata, evidence, and inter-operator coordination facade.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-LANG-005",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":          "/v1/programmes",
    "production":          "/v1/production",
    "qualityRecords":      "/v1/quality-records",
    "storageInventories":  "/v1/storage-inventories",
    "refuellingEvents":    "/v1/refuelling-events",
    "transportEvents":     "/v1/transport-events",
    "safetyIncidents":     "/v1/safety-incidents",
    "evidence":            "/v1/evidence",
    "openapi":             "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes              — register a programme
GET    /v1/programmes/{pid}        — retrieve programme
PATCH  /v1/programmes/{pid}/status — advance status
PATCH  /v1/programmes/{pid}/guarantee-of-origin
                                    — register or refresh
                                       CertifHy-equivalent
                                       guarantee-of-origin
                                       binding
```

Programme status transitions follow PHASE-1 §2; invalid
transitions return `422` with type
`urn:wia:WIA-LANG-005:status-transition`.

## §4 Production Records

```
POST   /v1/programmes/{pid}/production       — append a
                                                production
                                                record
GET    /v1/production/{pid}                  — retrieve
                                                production
                                                record
GET    /v1/programmes/{pid}/production?
       facility={f}&from={t}&to={t}            — query window
```

Production records that claim a renewable or low-carbon
guarantee-of-origin require an attached source-mix breakdown
(PHASE-1 §3 `electricitySourceMix`); submissions without
the breakdown that nevertheless claim the guarantee return
`422` with type
`urn:wia:WIA-LANG-005:source-mix-required`.

## §5 Hydrogen Quality

```
POST   /v1/production/{pid}/quality-records  — append a
                                                quality record
GET    /v1/quality-records/{qid}             — retrieve
                                                quality record
GET    /v1/production/{pid}/quality-records  — list quality
                                                records for a
                                                production
                                                batch
```

Quality records cite an ISO/IEC 17025-accredited laboratory;
submissions citing a non-accredited laboratory return `409`
with type
`urn:wia:WIA-LANG-005:laboratory-not-accredited`.

## §6 Storage Inventory

```
POST   /v1/facilities/{fid}/storage-inventories
                                              — register an
                                                inventory
                                                snapshot
GET    /v1/storage-inventories/{iid}         — retrieve
                                                snapshot
GET    /v1/facilities/{fid}/storage-inventories?
       from={t}&to={t}                          — query window
```

## §7 Refuelling Events (HRS)

```
POST   /v1/hrs/{fid}/refuelling-events       — append a
                                                refuelling
                                                event
GET    /v1/refuelling-events/{eid}           — retrieve event
GET    /v1/hrs/{fid}/refuelling-events?
       from={t}&to={t}                          — query window
```

Refuelling events MUST cite a quality certificate per ISO
19880-8 / SAE J2719; submissions without the cite return
`409` with type
`urn:wia:WIA-LANG-005:quality-certificate-required`.

## §8 Transport Events

```
POST   /v1/programmes/{pid}/transport-events  — register a
                                                transport
                                                event
PATCH  /v1/transport-events/{eid}/unloaded    — record
                                                unloading
GET    /v1/transport-events/{eid}             — retrieve
                                                event
```

Transport unloading records that show a mass discrepancy
beyond the operator's per-route tolerance trigger an
investigation through the operator's loss-control workflow.

## §9 Safety Incidents

```
POST   /v1/programmes/{pid}/safety-incidents  — register an
                                                incident
PATCH  /v1/safety-incidents/{iid}/severity    — update
                                                severity
PATCH  /v1/safety-incidents/{iid}/root-cause  — attach
                                                root-cause
                                                reference
GET    /v1/safety-incidents/{iid}             — retrieve
                                                incident
```

Incidents at severity `major` or `critical` automatically
escalate to the operating jurisdiction's industrial-safety
regulator through the integration described in PHASE-4 §6.

## §10 Errors

All error responses are `application/problem+json` per RFC
9457. Defined types include those above plus:

- `urn:wia:WIA-LANG-005:hydrogen-quality-out-of-grade`
- `urn:wia:WIA-LANG-005:cryogenic-temperature-excursion`
- `urn:wia:WIA-LANG-005:guarantee-of-origin-mismatch`
- `urn:wia:WIA-LANG-005:evidence-mismatch`

## §11 Authentication

Mutually-authenticated TLS for guarantee-of-origin schemes,
midstream operators, regulators, and partner operators.
Public read-only endpoints (production summaries, safety-
incident summaries above informational, refuelling-station
service-level statistics) are reachable without a client
certificate.

## §12 Caching, Concurrency, Audit

Stable resources (closed transport events, completed
refuelling events, signed evidence packages) are cacheable
with `Cache-Control: max-age=31536000, immutable`. Live
inventory and in-flight transport events are cacheable for
60 seconds. ETags are mandatory on every PATCH endpoint.
Audit logs carry `programmeId`, `traceId`, the issuing
client certificate's subject, and the operator's clock skew
vs the operating jurisdiction's NTP service.

## §13 Worked Example: Production to Refuelling

1. The electrolyser plant records production with the full
   electricity source-mix breakdown.
2. The plant's accredited laboratory ingests a quality
   record citing the production batch and the ISO 14687
   Type I Grade D test certificate.
3. Compressed hydrogen ships to an HRS via tube-trailer; the
   transport event records loaded mass and unloads at the
   destination.
4. The HRS dispenses hydrogen to a vehicle per SAE J2601;
   the refuelling event cites the quality certificate and
   records the precooled-temperature envelope.
5. The guarantee-of-origin scheme consumes the production
   and quality records to issue the per-batch guarantee
   that downstream offtakers cite in their LCA reporting.

## §14 Bulk and Pagination

```
POST   /v1/bulk/production              — batched production
                                            ingest from facility
                                            historians
POST   /v1/bulk/refuelling-events       — batched HRS event
                                            ingest
POST   /v1/bulk/storage-inventories     — batched inventory
                                            snapshots
GET    /v1/bulk/{operationId}           — operation status
```

Cursor-based pagination uses the `cursor` query parameter
and `Link` headers (RFC 8288); cursors persist for at least
24 hours.

## §15 Streaming Subscription

Consumers subscribe via Server-Sent Events at:

- `/v1/programmes/{pid}/events` — programme-wide events
  (carbon-intensity drift alerts, GoO issuance and
  retirement, safety incident escalations).
- `/v1/hrs/{fid}/events` — refuelling-station events
  (dispenser fault, quality certificate refresh, abort-fill
  notifications).

Subscribers reconnect via the `Last-Event-ID` header (W3C
EventSource semantics).

## §16 Provenance and Aggregation

```
GET    /v1/provenance/{recordId}    — provenance entry for
                                       any PHASE-1 record
GET    /v1/aggregate/production-by-pathway?period=...
GET    /v1/aggregate/carbon-intensity-trend?path=...&period=...
GET    /v1/aggregate/hrs-availability?period=...
```

Aggregate consumers fetch population-level statistics; per-
batch attribution requires the operator's authorisation.

## §17 Carrier Conversion and Embrittlement Endpoints

```
POST   /v1/facilities/{fid}/carrier-conversions
                                          — register a carrier
                                            conversion event
GET    /v1/carrier-conversions/{cid}      — retrieve event
POST   /v1/pipelines/{pid}/embrittlement-coupons
                                          — register a coupon
                                            installation
PATCH  /v1/embrittlement-coupons/{eid}/destructive-test
                                          — attach test report
                                            and verdict
GET    /v1/embrittlement-coupons/{eid}    — retrieve coupon
```

Embrittlement-coupon test verdicts of `remediation-required`
emit a notification to the operator's pipeline-integrity
team and freeze the affected pipeline section's blending
percentage until remediation completes.

## §18 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an
OpenAPI 3.1 document, signs evidence packages per RFC 9421,
and rejects refuelling-event submissions without the per-
batch quality certificate.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-LANG-005
- **Last Updated:** 2026-04-28
