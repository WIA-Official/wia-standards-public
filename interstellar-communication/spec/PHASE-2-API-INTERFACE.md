# WIA-interstellar-communication PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-interstellar-communication
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an
interstellar-communication operator exposes for the records
defined in PHASE-1. Consumers include partner observatories
collaborating on follow-up observations, the IAA SETI
Permanent Committee post-detection adjudication network,
operating-jurisdiction regulators, partner agencies for
precursor probe operations, and the operator's own analytics
and audit platforms.

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
- IVOA ObsCore / ObsTAP
- ITU-R RA.769

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The OpenAPI
3.1 document at `/v1/openapi.json` is canonical.

Observation data archives, candidate signal time-series, and
probe telemetry archives are content-addressed and reachable
through the operator's content store; this API returns
metadata and content-addresses, not large data payloads.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-interstellar-communication",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "targets":                 "/v1/targets",
    "observations":            "/v1/observations",
    "candidates":              "/v1/candidates",
    "rfiEntries":              "/v1/rfi-entries",
    "probeTelemetryLinks":     "/v1/probe-telemetry-links",
    "deliberateTransmissions": "/v1/deliberate-transmissions",
    "evidence":                "/v1/evidence",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes               — register a programme
GET    /v1/programmes/{pid}         — retrieve programme
PATCH  /v1/programmes/{pid}/status  — advance status
```

Programme `kind=deliberate-transmission` requires a governance
review reference (PHASE-3 §6) before the API accepts any
deliberate-transmission record under the programme.

## §4 Target Catalogue

```
POST   /v1/programmes/{pid}/targets        — register a target
GET    /v1/targets/{tid}                   — retrieve target
GET    /v1/programmes/{pid}/targets?
       within-pc={d}&type={s}                — query targets
```

Target submissions cite an external catalogue identifier
where one exists (HD, Hipparcos, Gaia DR3, TIC); the API
verifies the cite against a registered cross-walk and rejects
unverifiable cites with type
`urn:wia:interstellar-communication:catalogue-cite-not-resolved`.

## §5 Observations

```
POST   /v1/programmes/{pid}/observations   — register an
                                              observation
PATCH  /v1/observations/{oid}/end          — record observation
                                              end and archive
                                              reference
GET    /v1/observations/{oid}              — retrieve observation
GET    /v1/programmes/{pid}/observations?
       target={tid}&band={k}&from={t}        — query observations
```

Observation submissions whose `observationDigest` disagrees
with the archive content return `422` with type
`urn:wia:interstellar-communication:observation-digest-mismatch`.

## §6 Candidate Signal Lifecycle

```
POST   /v1/observations/{oid}/candidates   — register a
                                              candidate
PATCH  /v1/candidates/{cid}/reproducibility-state
                                            — update state
                                              per the post-
                                              detection
                                              protocol
PATCH  /v1/candidates/{cid}/post-detection-escalate
                                            — escalate to the
                                              IAA SETI Permanent
                                              Committee
GET    /v1/candidates/{cid}                — retrieve candidate
```

Candidates with significance above the operator's escalation
threshold automatically advance to
`reproducibility-state=follow-up-pending` upon registration;
manual override requires authorisation and is audited.

## §7 RFI Catalogue

```
POST   /v1/programmes/{pid}/rfi-entries    — register an RFI
                                              entry
GET    /v1/rfi-entries/{rid}               — retrieve entry
GET    /v1/programmes/{pid}/rfi-entries?
       frequency={hz}&kind={k}               — query entries
```

The RFI catalogue feeds the operator's signal-detection
pipeline so that subsequent candidates falling near a
catalogued RFI source are pre-classified as RFI without
analyst time.

## §8 Probe Telemetry Links

```
POST   /v1/programmes/{pid}/probe-telemetry-links
                                            — register a link
GET    /v1/probe-telemetry-links/{lid}     — retrieve link
GET    /v1/programmes/{pid}/probe-telemetry-links?
       probe={r}&from={t}                    — query links
```

Probe-telemetry links feed the operator's archive of long-
duration interstellar-precursor measurements (Voyager-class
spacecraft now in the heliosheath and beyond).

## §9 Deliberate Transmissions

```
POST   /v1/programmes/{pid}/deliberate-transmissions
                                            — register a
                                              transmission
                                              proposal
PATCH  /v1/deliberate-transmissions/{tid}/governance-review
                                            — attach governance
                                              review outcome
PATCH  /v1/deliberate-transmissions/{tid}/approval-state
                                            — advance state
GET    /v1/deliberate-transmissions/{tid}  — retrieve record
```

Approval-state advances to `approved` require an attached
governance-review outcome of `approved` (PHASE-3 §6);
mismatched submissions return `409 Conflict` with type
`urn:wia:interstellar-communication:governance-not-approved`.

## §10 Errors

All error responses are `application/problem+json` per RFC
9457. Defined types include those above plus:

- `urn:wia:interstellar-communication:significance-threshold-mismatch`
- `urn:wia:interstellar-communication:rfi-already-cataloged`
- `urn:wia:interstellar-communication:post-detection-restricted`
- `urn:wia:interstellar-communication:evidence-mismatch`

## §11 Authentication

Mutually-authenticated TLS for partner observatories,
post-detection adjudication network members, regulators, and
partner agencies. Public read-only endpoints (programme
summaries, post-detection-cleared candidates, public
observation catalogues) are reachable without a client
certificate.

## §12 Caching, Concurrency, Audit

Stable resources (closed observations, classified candidates,
post-detection-cleared records, signed evidence packages)
are cacheable with `Cache-Control: max-age=31536000,
immutable`. Mutable resources (in-flight observations,
under-review candidates) are cacheable for 60 seconds.
ETags are mandatory on every PATCH endpoint. Audit logs
carry `programmeId`, `traceId`, the issuing client
certificate's subject, and the operator's clock skew vs the
IAU SOFA / UT1 reference.

## §13 Worked Example: Search to Adjudication

1. The operator schedules a passive radio observation of a
   G-type target within 100 pc, registers the observation,
   and uploads the archive on completion.
2. The signal-detection pipeline emits five candidates; four
   are pre-classified as RFI from the catalogue, one is
   advanced to `follow-up-pending` based on significance.
3. A follow-up observation re-points the dish at the same
   target during a different epoch; the candidate fails to
   reproduce, advancing to `rfi-classified` with an updated
   RFI catalogue entry.
4. (In a hypothetical positive case) The candidate
   reproduces; the operator escalates to the post-detection
   adjudication network, which initiates the IAA SETI
   Permanent Committee protocol.

## §14 Bulk and Pagination

Bulk endpoints accept arrays for high-volume candidate
ingest (a single observation can produce thousands of
candidates that the pipeline emits in batches), RFI catalogue
import, and observation backfill from legacy records:

```
POST   /v1/bulk/candidates           — batched candidate
                                       ingest
POST   /v1/bulk/rfi-entries          — batched RFI import
GET    /v1/bulk/{operationId}        — operation status
```

Cursor-based pagination uses the `cursor` query parameter and
`Link` headers (RFC 8288); cursors persist for at least 24
hours.

## §15 Streaming Subscription Topics

Consumers subscribe via Server-Sent Events at:

- `/v1/programmes/{pid}/events` — programme-wide events
  (above-threshold candidate emissions, post-detection
  escalations, deliberate-transmission approvals).
- `/v1/candidates/{cid}/events` — candidate-scoped events
  (state transitions, follow-up confirmations, RFI
  classification).
- `/v1/observations/{oid}/events` — observation-scoped
  events (archive-availability, pipeline completion).

Subscribers reconnect via the `Last-Event-ID` header (W3C
EventSource semantics).

## §16 Provenance and Aggregation

```
GET    /v1/provenance/{recordId}    — provenance entry for
                                       any PHASE-1 record
GET    /v1/aggregate/observation-hours-by-band?period=...
GET    /v1/aggregate/candidate-rate-by-pipeline?period=...
GET    /v1/aggregate/rfi-density-by-frequency?period=...
```

## §17 Synthetic Injection and Campaign Endpoints

```
POST   /v1/programmes/{pid}/synthetic-injections
                                            — register an
                                              injection test
GET    /v1/synthetic-injections/{iid}      — retrieve
                                              injection
POST   /v1/programmes/{pid}/campaigns      — register a
                                              campaign
PATCH  /v1/campaigns/{caid}/end            — close a campaign
                                              with summary
                                              counts
GET    /v1/campaigns/{caid}                — retrieve campaign
```

Campaign-end submissions whose summary counts disagree with
the per-observation aggregation return `422` with type
`urn:wia:interstellar-communication:campaign-summary-mismatch`
and the operator must reconcile via the per-observation
endpoint before closing the campaign.

## §18 Privacy-Preserving Aggregation

Aggregate consumers (research-funder reporting, observatory
operations metrics, public-engagement dashboards) fetch
population-level statistics through endpoints that emit
counts only, never per-target attribution that would reveal
candidate-investigation pre-clearance:

```
GET    /v1/aggregate/observation-hours-by-facility?period=...
GET    /v1/aggregate/campaign-target-coverage?period=...
GET    /v1/aggregate/cleared-candidate-count?period=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:interstellar-communication:cohort-too-small` or
`urn:wia:interstellar-communication:pre-cleared-not-disclosed`
depending on the scope.

## §19 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an
OpenAPI 3.1 document, signs evidence packages per RFC 9421,
and rejects deliberate-transmission approval without the
governance-review outcome.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-interstellar-communication
- **Last Updated:** 2026-04-28
