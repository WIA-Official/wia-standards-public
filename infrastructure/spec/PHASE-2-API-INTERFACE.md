# WIA-infrastructure PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-infrastructure
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited civil-
infrastructure asset programme exposes for the records defined in
PHASE-1. Consumers include partner asset owners, inspection
contractors, BIM coordinators, condition-rating analytics services,
maintenance-management systems, regulators (transportation
ministries, water-quality authorities, dam-safety regulators), and
citation tools that resolve published condition reports to their
underlying records.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / JSON Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO 19650-2 (BIM information delivery)
- ISO 16739 (IFC)
- ISO 55001 (asset-management-system requirements)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
asset programme. Versioning uses `/v1/` path segments and Semantic
Versioning 2.0.0. The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical.

The API is a control-plane and metadata facade; large BIM
federation files, NDT scan archives, and high-resolution
photographic evidence flow over content-addressed storage that the
API references but does not stream in-line.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-infrastructure",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "assets":             "/v1/assets",
    "components":         "/v1/components",
    "spatialIdentities":  "/v1/spatial-identities",
    "inspections":        "/v1/inspections",
    "conditionAssessments": "/v1/condition-assessments",
    "workOrders":         "/v1/work-orders",
    "handOvers":          "/v1/hand-overs",
    "decommissionings":   "/v1/decommissionings",
    "evidence":           "/v1/evidence",
    "openapi":            "/v1/openapi.json"
  }
}
```

## §3 Asset Endpoints

```
POST   /v1/assets                       — register an asset
GET    /v1/assets/{aid}                 — retrieve asset record
PATCH  /v1/assets/{aid}/classification  — append a classification
                                          reference
GET    /v1/assets/{aid}/components      — list components
GET    /v1/assets/{aid}/timeline        — chronological timeline
                                          of inspection,
                                          assessment, work-order,
                                          and rehabilitation
                                          events
```

Asset registrations whose `classifierRefs` cite an unknown IFC4
entity return `422` with type
`urn:wia:infrastructure:classification-unknown`.

## §4 Component Endpoints

```
POST   /v1/assets/{aid}/components       — register a component
GET    /v1/components/{cid}              — retrieve component record
PATCH  /v1/components/{cid}/retrofit     — register a retrofit;
                                           emits successor
                                           component record
GET    /v1/components/{cid}/predecessors — walk the predecessor
                                           chain back to the
                                           original component
```

Retrofit submissions MUST cite the governing design code edition
(`AASHTO LRFD 9`, `EN 1992-1-1:2004+A1:2014`, `ACI 318-19`, etc.).
Submissions that omit the governing edition return `422` with type
`urn:wia:infrastructure:design-code-missing`.

## §5 Spatial-Identity Endpoints

```
POST   /v1/assets/{aid}/spatial-identities  — register a spatial
                                              identity
GET    /v1/spatial-identities/{sid}         — retrieve spatial
                                              identity
GET    /v1/assets/{aid}/spatial-identities?at={iso8601}
                                            — resolve geometry as
                                              of a specific date
```

The "as-of" query is critical for inspection-report consumers:
historical inspection reports must resolve to the geometry that
was current at the time of inspection, not the current geometry.

## §6 Inspection Endpoints

```
POST   /v1/assets/{aid}/inspections      — register an inspection
GET    /v1/inspections/{iid}             — retrieve inspection
PATCH  /v1/inspections/{iid}/sign        — append inspector
                                           signature; locks the
                                           record
POST   /v1/inspections/{iid}/correction  — emit a successor
                                           inspection record with
                                           a stated correction
                                           reason
GET    /v1/inspections/{iid}/observations — list observations
```

Once an inspection record is signed (PHASE-1 §5) further mutation
is rejected; the API returns `409` with type
`urn:wia:infrastructure:inspection-locked` to PATCH attempts on
locked inspections.

## §7 Condition-Assessment Endpoints

```
POST   /v1/condition-assessments              — register an
                                                 assessment
GET    /v1/condition-assessments/{caid}       — retrieve
                                                 assessment
PATCH  /v1/condition-assessments/{caid}/remaining-service-life
                                              — update the
                                                 remaining-service-
                                                 life estimate
                                                 with a new method
                                                 reference
GET    /v1/assets/{aid}/condition-assessments?from={iso8601}
                                              — query assessments
                                                 produced after a
                                                 date
```

Assessment submissions MUST cite a `ratingScheme` value that maps
to a known scheme (AASHTO general appraisal, ASCE condition
index, EN 1990 reliability index, owner-specific ladder).
Submissions whose scheme is unknown return `422` with type
`urn:wia:infrastructure:rating-scheme-unknown`.

## §8 Work-Order Endpoints

```
POST   /v1/assets/{aid}/work-orders          — register a work
                                                order
GET    /v1/work-orders/{woid}                — retrieve work order
PATCH  /v1/work-orders/{woid}/execution      — record execution
                                                outcomes
GET    /v1/assets/{aid}/work-orders?type={type}
                                             — query by work type
```

Non-routine work-order submissions (corrective-minor,
corrective-structural, rehabilitation, replacement, emergency)
MUST carry a `triggeringAssessmentRef` that resolves to a
condition assessment whose component ratings include the work-
order's component refs.

## §9 Hand-Over Endpoints

```
POST   /v1/assets/{aid}/hand-overs        — register a hand-over
GET    /v1/hand-overs/{hoid}              — retrieve hand-over
GET    /v1/hand-overs/{hoid}/federation   — fetch the IFC4.3
                                             federation-file
                                             content-address
PATCH  /v1/hand-overs/{hoid}/defect-liability
                                          — extend the defect
                                             liability period in
                                             the event of a
                                             contractual change
```

The federation-file content-address MUST validate as ISO 16739
IFC4.3 STEP-formatted text or the HDF5-encoded variant; otherwise
the API returns `422` with type
`urn:wia:infrastructure:federation-file-invalid`.

## §10 Decommissioning Endpoints

```
POST   /v1/assets/{aid}/decommissionings   — register a
                                              decommissioning
GET    /v1/decommissionings/{did}          — retrieve record
GET    /v1/assets/{aid}/successor          — resolve to the
                                              successor asset
                                              (if any)
```

Decommissioning records require an `archivalDepositRef` that
resolves to a recognised long-term archive (PHASE-4 §6); without
the deposit the API returns `422` with type
`urn:wia:infrastructure:archive-deposit-required`.

## §11 Evidence Package Endpoints

```
POST   /v1/assets/{aid}/evidence           — request package
                                              generation
GET    /v1/evidence/{packageId}            — retrieve package
GET    /v1/evidence/{packageId}/manifest   — manifest only
```

The evidence-package format is governed by PHASE-4 §3 and contains
the asset record, the component decomposition, the spatial-
identity history, the inspection set, the condition-assessment
set, the work-order register, the hand-over record, and the
signed manifest.

## §12 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:infrastructure:classification-unknown`
- `urn:wia:infrastructure:design-code-missing`
- `urn:wia:infrastructure:inspection-locked`
- `urn:wia:infrastructure:rating-scheme-unknown`
- `urn:wia:infrastructure:federation-file-invalid`
- `urn:wia:infrastructure:archive-deposit-required`
- `urn:wia:infrastructure:evidence-mismatch`

## §13 Authentication

Mutually-authenticated TLS for owner-to-contractor, owner-to-
regulator, and owner-to-archive connections. Public read-only
endpoints (asset register summaries, regulator-published condition
indices) are reachable without a client certificate.

## §14 Caching

Stable resources (signed inspections, completed condition
assessments, executed work orders, hand-over federation files)
are cacheable with `Cache-Control: max-age=31536000, immutable`.
Mutable resources (work orders in flight, draft assessments) are
cacheable for 60 seconds; ETags are mandatory on every PATCH
endpoint with `If-Match` conditional requests.

## §15 Streaming Subscriptions

Consumers subscribe via Server-Sent Events at
`/v1/assets/{aid}/events`. Topics include inspection sign events,
new condition assessments, work-order execution events,
rehabilitation completion, and decommissioning notification.

## §16 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                       for any PHASE-1 record
```

Provenance entries trace an asset's evidence package back to its
parents (asset record, hand-over federation file, the chain of
inspections that produced each condition assessment, the work-
order chain through rehabilitation events) so that auditors can
walk the chain end-to-end without inferring it from filename
conventions.

## §17 Bulk Operations

Long inspection campaigns and large condition-assessment sweeps
produce many records that are exchanged in bulk:

```
POST   /v1/bulk/inspections          — submit a bulk inspection
                                        sweep
POST   /v1/bulk/condition-assessments — submit a bulk assessment
                                        sweep
POST   /v1/bulk/work-orders          — submit a bulk work-order
                                        register import
GET    /v1/bulk/{operationId}        — operation status
```

Bulk operations are idempotent on the operation identifier;
retried submissions resolve to the same operation identifier and
return the prior outcome rather than producing duplicate records.

## §18 Audit and Observability

Every endpoint emits structured logs with `assetId`, `traceId`,
the issuing client certificate's subject, and the operator's
clock skew vs the reference NTP source. Audit logs are
immutable and sealed daily into the operator's transparency log
so that downstream auditors can verify that no audit event was
silently retracted.

## §19 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI
3.1 document, and signs evidence packages per RFC 9421.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-infrastructure
- **Last Updated:** 2026-04-28
