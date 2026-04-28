# WIA-inter-korean-data-exchange PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-inter-korean-data-exchange
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an authorised
inter-Korean exchange operator exposes for the records defined
in PHASE-1. Consumers include the Ministry of Unification, the
Inter-Korean Exchange and Cooperation Bureau, the Korea Red
Cross, registered humanitarian NGOs, and the operator-side
analytics and audit platforms.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- IETF RFC 5322 (correspondence envelope)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- UN OCHA Common Operational Datasets

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operator.
Versioning uses `/v1/` path segments. The OpenAPI 3.1 document
at `/v1/openapi.json` is canonical.

The API is consumed only on the south side. North-side
counterparts receive paper artefacts, sealed-envelope memory
media, or video conferencing handoffs through the inter-Korean
liaison channel; the API never directly addresses north-side
systems.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-inter-korean-data-exchange",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":               "/v1/programmes",
    "identityTokens":           "/v1/identity-tokens",
    "familyReunions":           "/v1/family-reunions",
    "aidManifests":             "/v1/aid-manifests",
    "liaisonCorrespondences":   "/v1/liaison-correspondences",
    "jointVentureInventories":  "/v1/joint-venture-inventories",
    "evidence":                 "/v1/evidence",
    "openapi":                  "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes               — register a programme
GET    /v1/programmes/{pid}         — retrieve programme
PATCH  /v1/programmes/{pid}/status  — advance status
```

Programme status `suspended` reflects an Inter-Korean Exchange
and Cooperation Bureau notification of suspension (typically
during periods of escalated political tension). Suspension
freezes all in-flight exchange artefacts; the operator may
not advance any record past its current status while suspended.

## §4 Identity Tokens

```
POST   /v1/identity-tokens            — register a token (no
                                          PII permitted in body)
GET    /v1/identity-tokens/{tid}      — retrieve token metadata
PATCH  /v1/identity-tokens/{tid}/eligibility
                                       — update reunion
                                          eligibility
```

Submissions whose body contains personal-identifying fields
return `422` with type
`urn:wia:inter-korean-data-exchange:pii-in-body`; the operator's
CRM holds personal identity, never the API.

## §5 Family-Reunion Lifecycle

```
POST   /v1/programmes/{pid}/family-reunions  — register a reunion
GET    /v1/family-reunions/{rid}             — retrieve reunion
PATCH  /v1/family-reunions/{rid}/match       — record north-side
                                                 match confirmation
PATCH  /v1/family-reunions/{rid}/outcome     — close reunion
                                                 with outcome
```

Reunion-round identifier (PHASE-1 §4) is registered separately
through the operator's reunion-administration process; the API
records the round identifier against each reunion but does not
manage round scheduling. Reunion-round scheduling is governed
by political-level inter-Korean negotiations, not by this API.

## §6 Humanitarian-Aid Manifests

```
POST   /v1/programmes/{pid}/aid-manifests       — register a
                                                   manifest
GET    /v1/aid-manifests/{mid}                  — retrieve
                                                   manifest
PATCH  /v1/aid-manifests/{mid}/shipped          — record
                                                   shipment
PATCH  /v1/aid-manifests/{mid}/received         — record
                                                   north-side
                                                   receipt
                                                   confirmation
GET    /v1/programmes/{pid}/aid-manifests?
       beneficiary={c}&period={p}                — query
                                                   manifests
```

Manifest submissions whose `cargoLines` include a hazard class
not authorised by the Ministry of Unification's per-shipment
authorisation return `422` with type
`urn:wia:inter-korean-data-exchange:hazard-class-not-authorised`.

## §7 Liaison Correspondence

```
POST   /v1/programmes/{pid}/liaison-correspondences
                                       — register
                                          correspondence
                                          (south-to-north or
                                          north-to-south)
GET    /v1/liaison-correspondences/{cid}
                                       — retrieve
                                          correspondence
PATCH  /v1/liaison-correspondences/{cid}/received
                                       — record
                                          acknowledgement
```

Correspondence body URIs (`bodyRef`) point to the operator's
secure document store; the API never returns the body inline,
so that downstream consumers must hold an authorisation to
fetch the body separately under the operator's correspondence-
classification policy.

## §8 Joint-Venture Inventory

```
POST   /v1/programmes/{pid}/joint-venture-inventories
                                       — register an inventory
                                          snapshot
GET    /v1/joint-venture-inventories/{iid}
                                       — retrieve inventory
GET    /v1/programmes/{pid}/joint-venture-inventories?
       venture={r}&from={t}&to={t}      — query history
```

Joint-venture records have been infrequent since the Mt Kumgang
tour suspension (2008) and the Kaesong Industrial Complex
closure (2016); the API supports historical inventory
reconstruction for archival purposes and for any future
re-opening under the Inter-Korean Exchange and Cooperation Act.

## §9 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:inter-korean-data-exchange:pii-in-body`
- `urn:wia:inter-korean-data-exchange:authorisation-revoked`
- `urn:wia:inter-korean-data-exchange:hazard-class-not-authorised`
- `urn:wia:inter-korean-data-exchange:correspondence-classification-restricted`
- `urn:wia:inter-korean-data-exchange:evidence-mismatch`

## §10 Authentication and Authorisation

Mutually-authenticated TLS for Ministry of Unification, Korea
Red Cross, KOICA, and registered NGO consumers. Public read-
only endpoints (aggregate aid-manifest statistics, anonymised
reunion-round summaries) are reachable without a client
certificate.

## §11 Caching and Concurrency

Stable resources (closed reunions, received aid manifests,
acknowledged correspondence, signed evidence packages) are
cacheable with `Cache-Control: max-age=31536000, immutable`.
Mutable resources (in-flight reunions, in-transit aid manifests)
are cacheable for 60 seconds. ETags are mandatory on every
PATCH endpoint.

## §12 Audit and Observability

Every endpoint emits structured logs with `programmeId`,
`traceId`, the issuing client certificate's subject, and the
operator's clock skew vs the Korea Standards Time reference
(KRISS UTC+9 service). Audit logs retain per the records-
retention policy in PHASE-3 §8.

## §13 Worked Example: Authorisation to Receipt for Aid Shipment

1. The operator obtains a Ministry of Unification shipment
   authorisation and registers an aid manifest with the
   authorisation reference.
2. Cargo lines are populated against the UN OCHA cluster
   classification and the per-line UN/CEFACT unit codes.
3. On shipment, the operator records `shippedAt` and the route
   code; the route code reflects the actually used corridor
   (the rail / road / maritime / third-country routes that
   have operated at different periods).
4. On north-side receipt confirmation through the inter-Korean
   liaison channel, the operator records `receivedAt`.
5. The evidence package is generated for downstream auditors
   (Ministry of Unification, the operator's funder, the
   public-disclosure register).

## §14 Bulk and Pagination

Bulk endpoints accept arrays for high-volume reunion list
import and historical aid manifest backfill:

```
POST   /v1/bulk/family-reunions      — batched reunion import
POST   /v1/bulk/aid-manifests        — batched manifest backfill
GET    /v1/bulk/{operationId}        — operation status
```

Cursor-based pagination uses the `cursor` query parameter and
`Link` headers (RFC 8288); cursors persist for at least 24
hours so that reconciliation tooling that connects intermittently
across the operator's authorisation window does not lose its
place.

## §15 Streaming Subscription

Consumers subscribe via Server-Sent Events at:

- `/v1/programmes/{pid}/events` — programme-wide events
  (authorisation amendments, suspension events, sanctions
  refresh outcomes).
- `/v1/aid-manifests/{mid}/events` — shipment-status events
  for an aid manifest.
- `/v1/family-reunions/{rid}/events` — reunion-lifecycle
  events for a registered reunion.

Subscribers reconnect via the `Last-Event-ID` header (W3C
EventSource semantics).

## §16 Privacy-Preserving Aggregation

Aggregate consumers (Ministry research divisions, academic
researchers under operator-vetted programmes) fetch
population-level statistics through aggregation endpoints
that emit counts only:

```
GET    /v1/aggregate/reunion-completion-rate?period=...
GET    /v1/aggregate/aid-volume-by-cluster?period=...
```

Out-of-policy queries (cohort below threshold, requests for
per-applicant detail) return `403 Forbidden` with type
`urn:wia:inter-korean-data-exchange:cohort-too-small`.

## §17 Cultural-Exchange Endpoints

```
POST   /v1/programmes/{pid}/cultural-exchanges
                                       — register a cultural
                                          exchange
GET    /v1/cultural-exchanges/{cid}    — retrieve exchange
PATCH  /v1/cultural-exchanges/{cid}/end
                                       — close exchange with
                                          completion summary
```

Cultural exchanges of kind `heritage-repatriation` require an
`artefactCatalogueRef` at registration; submissions that omit
the catalogue return `422` with type
`urn:wia:inter-korean-data-exchange:heritage-catalogue-required`.

## §18 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                       for any PHASE-1 record
```

Provenance entries trace a record to its parents (programme,
authorisation, sanctions sweep, manifest, reunion, exchange)
so that auditors can walk the chain end-to-end.

## §19 Distribution Evidence Endpoints

```
POST   /v1/aid-manifests/{mid}/distribution-evidence
                                       — register distribution
                                          evidence
GET    /v1/distribution-evidence/{eid} — retrieve evidence
PATCH  /v1/distribution-evidence/{eid}/diversion-flag
                                       — record diversion
                                          concern
```

Evidence submissions whose `beneficiaryClassification` does
not match the manifest's declared classification return `422`
with type
`urn:wia:inter-korean-data-exchange:beneficiary-classification-mismatch`.

## §20 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI
3.1 document, signs evidence packages per RFC 9421, and rejects
PII in any DATA-FORMAT field that this PHASE marks as opaque.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-inter-korean-data-exchange
- **Last Updated:** 2026-04-28
