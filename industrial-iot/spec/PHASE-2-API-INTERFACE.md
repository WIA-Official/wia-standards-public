# WIA-industrial-iot PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-industrial-iot
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
industrial-IoT programme exposes for the records defined in
PHASE-1. Consumers include MES and APM services, asset-performance
analytics platforms, ERP integrations, regulator and accreditation
bodies, and citation tools that resolve published reliability or
production reports to their underlying records.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 / 9111 / 9457 / 6901 / 6902 / 8259 / 8288 / 9421
- IETF RFC 5789 (PATCH method)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- IEC 62264 (ISA-95)
- IEC 62443 (security for industrial automation and control
  systems)
- OPC Unified Architecture (UA)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments. The OpenAPI 3.1
document at `/v1/openapi.json` is canonical.

The API is a metadata, configuration, and aggregation facade.
High-rate field-bus traffic flows through OPC UA or vendor field-
bus protocols on isolated networks per IEC 62443; the WIA API
exposes references to historised data rather than streaming the
raw bus traffic.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-industrial-iot",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "sites":            "/v1/sites",
    "assets":           "/v1/assets",
    "telemetry":        "/v1/telemetry",
    "controlLoops":     "/v1/control-loops",
    "alarmEvents":      "/v1/alarm-events",
    "maintenance":      "/v1/maintenance",
    "cyberPosture":     "/v1/cyber-posture",
    "productionOrders": "/v1/production-orders",
    "evidence":         "/v1/evidence",
    "openapi":          "/v1/openapi.json"
  }
}
```

## §3 Sites and Assets

```
POST   /v1/sites                          — register a site
GET    /v1/sites/{sid}                    — retrieve site record
PATCH  /v1/sites/{sid}/zone               — update IEC 62443 zone
                                              classification
POST   /v1/sites/{sid}/assets             — register an asset
GET    /v1/assets/{aid}                   — retrieve asset record
PATCH  /v1/assets/{aid}/firmware          — update firmware
                                              version (records the
                                              update event)
PATCH  /v1/assets/{aid}/decommission      — record decommission
```

Asset registrations whose `iec14224Class` does not resolve in the
ISO 14224 dictionary (when the asset class is in scope of the
dictionary) return `422 Unprocessable Entity` with type
`urn:wia:industrial-iot:iec14224-class-unresolved`.

## §4 Telemetry

```
POST   /v1/assets/{aid}/telemetry         — append telemetry sample
POST   /v1/bulk/telemetry                 — batched append
GET    /v1/telemetry/{tid}                — retrieve sample
GET    /v1/assets/{aid}/telemetry?from={t}&to={t}
                                          — query window
GET    /v1/sites/{sid}/telemetry/aggregate?signal={ref}&period={p}
                                          — aggregated signal
```

Telemetry submissions whose timestamp regresses against the
historian's latest sample for the same signal return `422` with
type `urn:wia:industrial-iot:timestamp-regress`. The historian
preserves the original timestamp for forensic continuity even
when the operator's clock drift triggers a regression.

## §5 Control Loops

```
POST   /v1/sites/{sid}/control-loops      — register a loop
GET    /v1/control-loops/{lid}            — retrieve loop record
PATCH  /v1/control-loops/{lid}/mode       — change control mode
POST   /v1/control-loops/{lid}/tunings    — register a tuning
                                              revision
GET    /v1/control-loops/{lid}/tunings    — list historic tunings
```

Mode transitions follow the safety-engineering matrix the operator
maintains; transitions that violate the matrix (e.g. `auto` →
`out-of-service` without a safety lock) return `409 Conflict`
with type `urn:wia:industrial-iot:mode-transition-violation`.

## §6 Alarms and Events

```
POST   /v1/assets/{aid}/alarm-events      — register an event
PATCH  /v1/alarm-events/{eid}/acknowledge — acknowledge
PATCH  /v1/alarm-events/{eid}/clear       — clear
GET    /v1/sites/{sid}/alarm-events?priority={1-4}&since={t}
                                          — query active alarms
```

Alarm priority follows ISA-18.2 conventions; the API rejects
priority values outside 1-4 with type
`urn:wia:industrial-iot:alarm-priority-out-of-range`.

## §7 Maintenance

```
POST   /v1/assets/{aid}/maintenance       — register a record
GET    /v1/maintenance/{mid}              — retrieve record
GET    /v1/assets/{aid}/maintenance?from={t}
                                          — query history
GET    /v1/assets/{aid}/reliability-kpi?period={p}
                                          — derived KPIs (MTBF,
                                              MTTR per ISO 14224
                                              conventions)
```

Reliability KPIs are computed server-side from maintenance and
alarm history; programmes that publish externally cited KPIs
include the underlying record set in the evidence package so that
downstream consumers can recompute the KPIs.

## §8 Cybersecurity Posture

```
POST   /v1/sites/{sid}/cyber-posture      — register posture
GET    /v1/cyber-posture/{pid}            — retrieve posture
PATCH  /v1/cyber-posture/{pid}/review     — append review event
```

## §9 Production Orders

```
POST   /v1/sites/{sid}/production-orders  — register an order
GET    /v1/production-orders/{oid}        — retrieve order
PATCH  /v1/production-orders/{oid}/status — advance order status
PATCH  /v1/production-orders/{oid}/lots   — append lot bindings
GET    /v1/production-orders/{oid}/trace  — trace from order to
                                              telemetry / alarm
                                              records
```

The `trace` endpoint resolves the production order through every
related record (telemetry windows, alarm events, maintenance
records of the work-cell during the order) so that recall and
quality-deviation investigations can reconstruct order-bound
context without polling.

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:industrial-iot:iec14224-class-unresolved`
- `urn:wia:industrial-iot:timestamp-regress`
- `urn:wia:industrial-iot:mode-transition-violation`
- `urn:wia:industrial-iot:alarm-priority-out-of-range`
- `urn:wia:industrial-iot:zone-conduit-violation`
- `urn:wia:industrial-iot:evidence-mismatch`

## §11 Authentication

The API uses mutually-authenticated TLS for MES, APM, ERP, and
regulator client certificates. Field-bus controllers authenticate
through the operator's industrial-security broker before reaching
the WIA API.

## §12 Caching

Stable resources (closed alarms, completed maintenance records,
signed evidence packages) are cacheable with `Cache-Control:
max-age=31536000, immutable`. Mutable resources (active alarms,
in-progress production orders) are cacheable for 60 seconds.

## §13 Streaming Subscriptions

Consumers subscribe via Server-Sent Events at
`/v1/sites/{sid}/events`. Topics include alarm-state transitions,
control-mode changes, asset firmware updates, production-order
status transitions, and cyber-posture review outcomes.

## §14 Worked Example: From Trip to Trace

1. Process anomaly produces a priority-1 alarm; the SCADA
   front-end POSTs the event.
2. Operator acknowledges the alarm via PATCH; the loop's mode
   transitions to `manual`.
3. Reliability engineer queries the asset's recent telemetry and
   alarm history through the `trace` endpoint.
4. Maintenance is performed; ISO 14224 failure code is recorded.
5. Loop returns to `auto`; tuning revision is registered if any
   change was made.
6. Reliability KPIs are recomputed; deviations are surfaced to
   APM.

## §15 Bulk Operations and Pagination

Bulk endpoints accept arrays of telemetry samples and event
records for high-volume submissions:

```
POST   /v1/bulk/telemetry              — batched telemetry
POST   /v1/bulk/alarm-events           — batched event submission
POST   /v1/bulk/maintenance            — batched maintenance
                                          import (legacy CMMS
                                          ingest)
GET    /v1/bulk/{operationId}          — operation status
GET    /v1/bulk/{operationId}/items    — per-item status
```

List endpoints use cursor-based pagination via the `cursor` query
parameter and `Link` headers (RFC 8288). Cursors persist for at
least 24 hours.

## §16 Privacy-Preserving Aggregation

Aggregate consumers (industry analysts, supply-chain auditors,
public-health analysts for occupational-exposure trends) fetch
population-level statistics through aggregation endpoints:

```
GET    /v1/aggregate/mtbf?asset-class=...&period=...
GET    /v1/aggregate/alarm-rate?priority=...&period=...
GET    /v1/aggregate/zone-violations?period=...
```

Out-of-policy queries (cohort below threshold, scope outside the
consumer's authorisation) return `403 Forbidden` with type
`urn:wia:industrial-iot:cohort-too-small`.

## §17 Audit and Observability

Every endpoint emits structured logs with `siteId`, `assetId`,
`traceId`, the issuing client certificate's subject, and the
historian's clock skew vs the reference NTP source.

## §18 Energy / Sustainability Endpoints

```
POST   /v1/sites/{sid}/energy            — append energy sample
GET    /v1/energy/{eid}                  — retrieve sample
GET    /v1/sites/{sid}/energy?from={t}&to={t}
                                          — query window
GET    /v1/sites/{sid}/sustainability?period={p}
                                          — period rollup (Scope 1
                                              and Scope 2 totals
                                              per the operator's
                                              GHG protocol)
```

Operator submissions whose `intervalDurationS` is below the
operator's declared minimum reporting cadence return `422` with
type `urn:wia:industrial-iot:reporting-cadence-mismatch`.

## §19 Configuration Snapshot Endpoints

```
POST   /v1/assets/{aid}/configuration-snapshots — register a
                                                  snapshot
GET    /v1/configuration-snapshots/{cid}        — retrieve
                                                  snapshot
GET    /v1/assets/{aid}/configuration-snapshots?from={t}
                                                — query series
GET    /v1/configuration-snapshots/{cid}/diff?against={cid2}
                                                — diff against
                                                  another snapshot
                                                  (server-rendered
                                                  unified-diff or
                                                  structured-diff
                                                  per Accept header)
```

The diff endpoint is consumed by change-board reviewers and post-
incident investigators; submissions whose two snapshots are not
on the same asset return `422` with type
`urn:wia:industrial-iot:snapshot-asset-mismatch`.

## §20 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, and signs evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-industrial-iot
- **Last Updated:** 2026-04-27
