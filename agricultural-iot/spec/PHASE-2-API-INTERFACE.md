# WIA-agricultural-iot PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-agricultural-iot
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an
agricultural-IoT operator exposes for the records defined
in PHASE-1. Consumers include FMIS vendors, equipment OEMs
that consume task-data round-trips, agronomy advisor
platforms, livestock-traceability authorities, and the
operator's own analytics platforms.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- IETF RFC 7252 (CoAP — for the device-side facade only)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- OGC SensorThings API 1.1
- ISO 11783 series
- AgGateway ADAPT
- OASIS MQTT v5.0 / MQTT-SN

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The OpenAPI
3.1 document at `/v1/openapi.json` is canonical.

The operator-side API is the FMIS-facing facade. The
device-side facade speaks CoAP / MQTT / MQTT-SN / LoRaWAN /
ISOBUS-CAN directly and is documented by the underlying
specifications; this PHASE does not redefine those
device-side protocols.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-agricultural-iot",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "operations":            "/v1/operations",
    "geoReferences":         "/v1/geo-references",
    "devices":               "/v1/devices",
    "observations":          "/v1/observations",
    "isobusTasks":           "/v1/isobus-tasks",
    "animals":               "/v1/animals",
    "irrigationPlans":       "/v1/irrigation-plans",
    "soilClassifications":   "/v1/soil-classifications",
    "evidence":              "/v1/evidence",
    "openapi":               "/v1/openapi.json"
  }
}
```

## §3 Operation Lifecycle

```
POST   /v1/operations               — register an operation
GET    /v1/operations/{oid}         — retrieve operation
PATCH  /v1/operations/{oid}/status  — advance status
```

## §4 Geo-References

```
POST   /v1/operations/{oid}/geo-references
                                          — register a feature
GET    /v1/geo-references/{rid}           — retrieve feature
GET    /v1/operations/{oid}/geo-references?
       refKind={k}                          — list features
```

Geometry submissions cite the CRS (PHASE-1 §3 `crsRef`);
submissions without the CRS return `422` with type
`urn:wia:agricultural-iot:crs-required`.

## §5 Devices

```
POST   /v1/operations/{oid}/devices       — register a device
PATCH  /v1/devices/{did}/status           — advance status
PATCH  /v1/devices/{did}/firmware         — update firmware
                                              reference
GET    /v1/devices/{did}                  — retrieve device
```

ISOBUS controllers (deviceClass with `isobus-` prefix)
require an SSN/SOSA profile reference; submissions without
the reference return `422` with type
`urn:wia:agricultural-iot:ssn-sosa-profile-required`.

## §6 Observations (OGC SensorThings Aligned)

```
POST   /v1/devices/{did}/observations     — append a single
                                              observation
POST   /v1/bulk/observations              — batched ingest
GET    /v1/observations/{oid}             — retrieve
                                              observation
GET    /v1/devices/{did}/observations?
       property={p}&from={t}&to={t}         — query window
```

Observations whose `qualityFlag` is `device-fault` are
quarantined from analytics-aggregation feeds until the
operator's QA team approves the value.

## §7 ISOBUS Task Round-Trip

```
POST   /v1/operations/{oid}/isobus-tasks  — register a task
PATCH  /v1/isobus-tasks/{tid}/start       — record actual
                                              start
PATCH  /v1/isobus-tasks/{tid}/end         — record actual end
                                              and as-applied
                                              map reference
GET    /v1/isobus-tasks/{tid}             — retrieve task
```

Variable-rate tasks require an attached prescription map at
registration; submissions without the map for
`fertiliser-application` / `spraying` / `planting` tasks
return `422` with type
`urn:wia:agricultural-iot:prescription-map-required`.

## §8 Livestock Animals

```
POST   /v1/operations/{oid}/animals       — register an
                                              animal
PATCH  /v1/animals/{aid}/status           — advance
                                              registration
                                              status
GET    /v1/animals/{aid}                  — retrieve animal
GET    /v1/operations/{oid}/animals?
       species={s}&breed={b}                — query animals
```

Animal-record submissions whose `rfidTag` does not conform
to ISO 11784 / 11785 return `422` with type
`urn:wia:agricultural-iot:rfid-tag-malformed`.

## §9 Irrigation Plans

```
POST   /v1/operations/{oid}/irrigation-plans
                                          — register a plan
PATCH  /v1/irrigation-plans/{pid}/applied — record application
GET    /v1/irrigation-plans/{pid}         — retrieve plan
GET    /v1/operations/{oid}/irrigation-plans?
       zone={z}&from={t}&to={t}             — query plans
```

Plans that exceed the operator's water-rights allocation
return `409` with type
`urn:wia:agricultural-iot:water-rights-exceeded` and
require operator authorisation before resubmission.

## §10 Soil Classifications

```
POST   /v1/operations/{oid}/soil-classifications
                                          — register a
                                              classification
GET    /v1/soil-classifications/{cid}     — retrieve
                                              classification
GET    /v1/operations/{oid}/soil-classifications?
       scheme={s}                           — query by
                                              taxonomy scheme
```

## §11 Errors

All error responses are `application/problem+json` per RFC
9457. Defined types include those above plus:

- `urn:wia:agricultural-iot:isobus-protocol-violation`
- `urn:wia:agricultural-iot:agrovoc-uri-not-resolvable`
- `urn:wia:agricultural-iot:cohort-too-small`
- `urn:wia:agricultural-iot:evidence-mismatch`

## §12 Authentication

Mutually-authenticated TLS for FMIS, OEM, advisor, and
regulator consumers. Public read-only endpoints (operation
summary, soil-classification per zone, anonymised aggregate
yield-driver statistics) are reachable without a client
certificate.

## §13 Caching, Concurrency, Audit

Stable resources (closed ISOBUS tasks with as-applied maps,
classified soil samples, signed evidence packages) are
cacheable with `Cache-Control: max-age=31536000, immutable`.
Live observations are cacheable for 30 seconds. ETags are
mandatory on every PATCH endpoint. Audit logs carry
`operationId`, `traceId`, the issuing client certificate's
subject, and the operator's clock skew vs the operating
jurisdiction's NTP service.

## §14 Streaming Subscription

Consumers subscribe via Server-Sent Events at:

- `/v1/operations/{oid}/events` — operation-wide events
  (water-rights breach warnings, ISOBUS task completions,
  livestock RFID-anomaly notifications).
- `/v1/devices/{did}/events` — device-scoped events
  (fault, firmware update completion, threshold alarms).

Subscribers reconnect via the `Last-Event-ID` header (W3C
EventSource semantics).

## §15 Bulk and Pagination

```
POST   /v1/bulk/devices                   — batched device
                                              registration
POST   /v1/bulk/observations              — batched ingest
POST   /v1/bulk/animals                   — batched animal
                                              import
GET    /v1/bulk/{operationId}             — operation status
```

Cursor-based pagination uses the `cursor` query parameter
and `Link` headers (RFC 8288); cursors persist for at least
24 hours.

## §16 Provenance and Aggregation

```
GET    /v1/provenance/{recordId}    — provenance entry for
                                       any PHASE-1 record
GET    /v1/aggregate/observation-volume?period=...
GET    /v1/aggregate/yield-zone-coverage?period=...
GET    /v1/aggregate/water-application-by-zone?period=...
```

## §17 Worked Example: ISOBUS Task Round-Trip

1. The operator authors an ISOBUS task with a variable-rate
   prescription map (ISOXML PRESCRIPTION-MAP wrapped in
   AgGateway ADAPT) and registers the task.
2. The tractor's task controller receives the task through
   the OEM's FMIS sync; the operator records actual-start
   on tractor engagement.
3. Field execution records as-applied measurements
   continuously through the ISOBUS task controller.
4. On completion, the operator records actual-end and
   attaches the as-applied map.
5. FMIS analytics consume the as-applied vs prescription
   delta for season-end input-cost reconciliation.

## §18 Drift-Card and Soil-Sample Endpoints

```
POST   /v1/isobus-tasks/{tid}/drift-cards    — register drift
                                                card placement
                                                + post-task
                                                assessment
GET    /v1/isobus-tasks/{tid}/drift-cards    — retrieve drift
                                                card history
POST   /v1/operations/{oid}/soil-samples     — register a soil
                                                sample (sample
                                                metadata; lab
                                                results flow
                                                into PHASE-1 §9
                                                soilClassification
                                                separately)
GET    /v1/soil-samples/{sid}                — retrieve soil
                                                sample metadata
```

Drift card submissions whose `windSpeedMs` exceeds the
product label's spraying threshold return `409` with type
`urn:wia:agricultural-iot:wind-speed-exceeded` and the
operator's spraying-task workflow halts the application.

## §19 Pesticide Application and Animal Health Endpoints

```
POST   /v1/isobus-tasks/{tid}/pesticide-applications
                                          — register product
                                            application detail
                                            for a spraying task
GET    /v1/pesticide-applications/{aid}   — retrieve detail
POST   /v1/animals/{anid}/health-events   — register a health
                                            event
GET    /v1/animal-health-events/{eid}     — retrieve event
GET    /v1/animals/{anid}/health-events?
       kind={k}&from={t}                    — query history
```

Animal-health events of kind `treatment-administered` carry
a withdrawal period; subsequent events of kind `slaughter`
or milk-use within the withdrawal window return `409` with
type `urn:wia:agricultural-iot:withdrawal-period-active`
and the operator's livestock-management workflow halts the
intended action.

## §20 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an
OpenAPI 3.1 document, signs evidence packages per RFC 9421,
and rejects ISOBUS task submissions that violate the
ISO 11783-10 task envelope.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-agricultural-iot
- **Last Updated:** 2026-04-28
