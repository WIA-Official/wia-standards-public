# WIA-inventory-management PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-inventory-management
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an
inventory-management programme exposes for the records
defined in PHASE-1. Consumers include WMS / ERP integrations,
3PL provider gateways, trading-partner EDI bridges, demand-
planning dashboards, and the operator's own analytics and
audit platforms.

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
- GS1 EPCIS 2.0 / CBV 2.0
- GS1 General Specifications

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The OpenAPI
3.1 document at `/v1/openapi.json` is canonical.

The API supports both bidirectional partner integrations
(trading-partner inbound ASN consumption, outbound ASN
emission) and read-only consumer integrations (analytics
dashboards, executive reports).

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-inventory-management",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "items":              "/v1/items",
    "locations":          "/v1/locations",
    "stockBalances":      "/v1/stock-balances",
    "lots":               "/v1/lots",
    "serials":            "/v1/serials",
    "movementEvents":     "/v1/movement-events",
    "receiving":          "/v1/receiving",
    "shipping":           "/v1/shipping",
    "inventoryCounts":    "/v1/inventory-counts",
    "evidence":           "/v1/evidence",
    "openapi":            "/v1/openapi.json"
  }
}
```

## §3 Item Master and Location

```
POST   /v1/programmes/{pid}/items         — register an item
GET    /v1/items/{iid}                    — retrieve item
PATCH  /v1/items/{iid}/status             — advance status
POST   /v1/programmes/{pid}/locations     — register a
                                              location
GET    /v1/locations/{lid}                — retrieve location
PATCH  /v1/locations/{lid}/capacity       — update capacity
                                              profile
```

Item submissions whose `itemControl=controlled-substance`
must reference a regulator-issued controlled-substance
licence; submissions without the reference return `409`
with type
`urn:wia:inventory-management:controlled-substance-licence-required`.

## §4 Stock Balances

```
POST   /v1/items/{iid}/stock-balances     — record a balance
                                              snapshot
GET    /v1/stock-balances/{bid}           — retrieve balance
GET    /v1/items/{iid}/stock-balances?
       location={lid}&from={t}&to={t}      — query window
```

Stock balances for lot-controlled items that omit the
`lotRef` and `expirationDate` return `422` with type
`urn:wia:inventory-management:lot-fields-required`.

## §5 Lot and Serial Lifecycle

```
POST   /v1/items/{iid}/lots               — register a lot
PATCH  /v1/lots/{lid}/qa-release          — attach QA release
                                              certificate
POST   /v1/items/{iid}/serials            — register a serial
PATCH  /v1/serials/{sid}/warranty         — update warranty
                                              window
GET    /v1/lots/{lid}                     — retrieve lot
GET    /v1/serials/{sid}                  — retrieve serial
```

Regulated-product items (pharmaceutical, medical-device,
food, cosmetic) require an attached QA release before stock
balances under the lot can transition to disposition
`available_for_sale`.

## §6 Movement Events (EPCIS 2.0)

```
POST   /v1/programmes/{pid}/movement-events
                                          — register an EPCIS
                                              event
POST   /v1/bulk/movement-events           — batched event
                                              ingest
GET    /v1/movement-events/{eid}          — retrieve event
GET    /v1/programmes/{pid}/movement-events?
       bizStep={s}&disposition={d}&from={t}&to={t}
                                          — query events
```

Movement-event submissions that violate the EPCIS 2.0
schema or use a CBV 2.0 vocabulary term not in the published
profile return `422` with type
`urn:wia:inventory-management:epcis-violation` and the
operator's reconciliation workflow runs against the source
system.

## §7 Receiving and Shipping

```
POST   /v1/programmes/{pid}/receiving     — register a
                                              receiving record
PATCH  /v1/receiving/{rid}/discrepancy    — append a
                                              discrepancy
PATCH  /v1/receiving/{rid}/putaway-completed
                                          — close receiving
POST   /v1/programmes/{pid}/shipping      — register a
                                              shipping record
PATCH  /v1/shipping/{sid}/proof-of-delivery
                                          — attach POD
                                              reference
GET    /v1/receiving/{rid}                — retrieve receiving
GET    /v1/shipping/{sid}                 — retrieve shipping
```

Receiving discrepancies above the operator's threshold
trigger an automatic supplier-side notification through the
trading-partner integration described in PHASE-4 §5.

## §8 Cycle-Count and Physical-Inventory

```
POST   /v1/programmes/{pid}/inventory-counts
                                          — register a count
PATCH  /v1/inventory-counts/{cid}/results — append count
                                              results
PATCH  /v1/inventory-counts/{cid}/approve — approve variance
                                              posting
GET    /v1/inventory-counts/{cid}         — retrieve count
```

Variance posting requires an approval reference for any
adjustment beyond the operator's variance-tolerance
threshold; submissions without approval return `409` with
type
`urn:wia:inventory-management:variance-approval-required`.

## §9 Errors

All error responses are `application/problem+json` per RFC
9457. Defined types include those above plus:

- `urn:wia:inventory-management:expired-lot-shipment-blocked`
- `urn:wia:inventory-management:cold-chain-excursion`
- `urn:wia:inventory-management:hazmat-segregation-violation`
- `urn:wia:inventory-management:evidence-mismatch`

## §10 Authentication

Mutually-authenticated TLS for trading-partner, 3PL provider,
and regulator consumers. Internal operator-to-operator
endpoints (analytics dashboards, internal QA tools)
authenticate through the operator's IDP.

## §11 Caching, Concurrency, Audit

Stable resources (closed receiving / shipping records,
approved inventory counts, signed evidence packages) are
cacheable with `Cache-Control: max-age=31536000, immutable`.
Stock balance and movement event mutable resources are
cacheable for 30 seconds — short to support near-real-time
allocate-to-promise workflows. ETags are mandatory on every
PATCH endpoint. Audit logs carry `programmeId`, `traceId`,
the issuing client certificate's subject, and the operator's
clock skew vs the operating jurisdiction's NTP service.

## §12 Streaming Subscription

Consumers subscribe via Server-Sent Events at:

- `/v1/programmes/{pid}/events` — programme-wide events
  (cold-chain excursion alerts, hazmat segregation
  violations, controlled-substance discrepancy escalations).
- `/v1/items/{iid}/events` — item-scoped events (stock-out
  warnings, low-stock alerts, expiration-approaching
  notifications).
- `/v1/receiving/{rid}/events` — receiving-scoped events
  (discrepancy notifications, put-away completion).

Subscribers reconnect via the `Last-Event-ID` header.

## §13 Worked Example: Receipt to Sale

1. The supplier transmits an ASN; the operator registers a
   receiving record referencing the ASN.
2. The dock receives the trailer, scans SSCCs, and emits
   `ObjectEvent`s with `bizStep=receiving`,
   `disposition=in_progress`.
3. The WMS plans put-away; movement events emit
   `bizStep=arriving` and `bizStep=storing` against active
   pick-face locations.
4. Stock balances refresh against the new on-hand at the
   pick-face locations.
5. An outbound order picks against the pick-face locations,
   emitting `ObjectEvent`s with `bizStep=picking` and a
   shipping record on completion.

## §14 Bulk and Pagination

```
POST   /v1/bulk/items                    — batched item master
                                            sync from ERP
POST   /v1/bulk/movement-events          — batched event ingest
POST   /v1/bulk/stock-balances           — batched balance
                                            snapshot
GET    /v1/bulk/{operationId}            — operation status
```

Cursor-based pagination uses the `cursor` query parameter and
`Link` headers (RFC 8288). Bulk operations honour the per-
ingest backpressure protocol so that a high-volume EPCIS
event stream from a busy distribution centre does not exceed
the operator's per-tenant rate limit.

## §15 Available-to-Promise Endpoint

```
GET    /v1/items/{iid}/atp?location={lid}&horizon={d}
                                          — server-computed
                                            available-to-
                                            promise quantity
                                            for the requested
                                            horizon, accounting
                                            for inbound,
                                            allocated, and
                                            committed
                                            inventory
```

ATP responses are cacheable for 30 seconds; storefront
integrations honour the cache TTL so that shopper-facing
availability does not lag the actual stock state by more
than the cache window.

## §16 Provenance and Aggregate Endpoints

```
GET    /v1/provenance/{recordId}    — provenance entry for
                                       any PHASE-1 record
GET    /v1/aggregate/inventory-turns?period=...
GET    /v1/aggregate/cycle-count-variance?period=...
GET    /v1/aggregate/cold-chain-excursion-count?period=...
```

## §17 Recall Endpoints

```
POST   /v1/programmes/{pid}/recalls       — initiate a recall
                                            event
PATCH  /v1/recalls/{rid}/status           — advance recall
                                            status
PATCH  /v1/recalls/{rid}/affected-stock   — append affected
                                            stock identifiers
GET    /v1/recalls/{rid}/trace            — server-rendered
                                            trace graph from
                                            affected lots /
                                            serials to
                                            current locations
                                            and customer-side
                                            destinations
GET    /v1/recalls/{rid}                  — retrieve recall
```

Recall trace queries respect the operator's per-customer
data-protection policy; per-end-customer detail is returned
only to authorised regulator and operator-internal
consumers.

## §18 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI
3.1 document, signs evidence packages per RFC 9421, and
rejects EPCIS 2.0 submissions that violate the schema.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-inventory-management
- **Last Updated:** 2026-04-28
