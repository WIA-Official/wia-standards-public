# WIA-pet-cloning PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-pet-cloning
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited pet-cloning
programme exposes for the records defined in PHASE-1 (DATA-FORMAT). The
contract is consumed by reference laboratories, breed registries, regulators,
and post-natal verification services. It is not intended for owner-facing
consumer applications; consumer access flows through the operator's CRM,
which mediates between the owner and this API.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 7807 (Predecessor — kept here only because some legacy auditor
  tools still emit RFC 7807; emitting RFC 9457 is REQUIRED for new
  implementations)
- IETF RFC 6901 (JSON Pointer)
- IETF RFC 6902 (JSON Patch)
- IETF RFC 7234 (originally defined cache semantics, now superseded by 9111)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 5789 (PATCH method)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context (correlation propagation)

---

## §1 Scope and Versioning

The API is a JSON-over-HTTPS interface served on a domain published by the
operating programme. Versioning uses URL path segments (`/v1/`) and follows
Semantic Versioning 2.0.0 at the major-version level: a non-additive change
to a resource shape MUST trigger a new major path segment. Additive changes
(new optional fields, new enum values that do not displace existing ones) are
permitted in-place.

All response bodies are RFC 8259 JSON in UTF-8 with `application/json`
content type; problem responses use `application/problem+json` per RFC 9457.

## §2 Base URL and Root Discovery

The API root publishes machine-readable discovery as a single JSON document.
A conformant client SHOULD bootstrap from this document rather than hard-
coding URLs.

```
GET /v1/
```

Response (abbreviated):

```json
{
  "standard": "WIA-pet-cloning",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "cases": "/v1/cases",
    "donors": "/v1/donors",
    "samples": "/v1/samples",
    "oocytes": "/v1/oocytes",
    "reconstructions": "/v1/reconstructions",
    "embryos": "/v1/embryos",
    "recipients": "/v1/recipients",
    "verifications": "/v1/verifications",
    "evidence": "/v1/evidence",
    "openapi": "/v1/openapi.json"
  }
}
```

Servers MUST expose `/v1/openapi.json` as the canonical OpenAPI 3.1 document.

## §3 Cases

```
POST   /v1/cases                       — create a case
GET    /v1/cases/{caseId}              — retrieve a case
PATCH  /v1/cases/{caseId}              — update mutable fields (status,
                                          programme, crossSpecies)
GET    /v1/cases?status={status}&since={ts}  — list cases (cursor pagination)
```

`POST /v1/cases` accepts the case-shaped body from PHASE-1 §2. Servers MUST
generate `caseId` server-side as a UUIDv7 and MUST reject client-provided
identifiers with `409 Conflict`. The response carries the canonical case
record and the location header points to the case URL.

Status transitions are gated server-side. A case MAY move from `draft` to
`active`, from `active` to `transferred` or `ceased`, and from `transferred`
to `verified`. All other transitions return `422 Unprocessable Entity` with a
problem document whose `type` is `urn:wia:pet-cloning:status-transition`.

## §4 Donors and Samples

```
POST   /v1/cases/{caseId}/donors            — register a donor
GET    /v1/donors/{donorId}                 — retrieve donor record
POST   /v1/donors/{donorId}/samples         — record a tissue sample
GET    /v1/samples/{sampleId}               — retrieve sample record
POST   /v1/samples/{sampleId}/custody       — append a custody event
```

Donor records are append-only with respect to identity fields. The
`euthanasiaStatus` field MAY change from `alive` to `post-mortem` exactly
once; subsequent attempts return `409 Conflict`.

The custody-event endpoint accepts a single CustodyEvent (PHASE-1 §9).
Servers MUST persist the event with monotonically increasing sequence
numbers per resource and MUST publish the sequence in the response so that
clients can detect lost events on reconnection.

## §5 Oocytes, Reconstruction, and Embryos

```
POST   /v1/cases/{caseId}/oocytes           — register an oocyte
POST   /v1/cases/{caseId}/reconstructions   — record an SCNT event
POST   /v1/reconstructions/{rid}/embryos    — record a derived embryo
PATCH  /v1/embryos/{embryoId}               — update fate or culture log
```

Reconstruction submissions reference an existing `sampleId` and an existing
`oocyteId`; the server validates referential integrity and enforces that
both belong to the same case. Cross-case combinations return `422` with a
problem document whose `type` is `urn:wia:pet-cloning:cross-case-reference`.

## §6 Recipients and Pregnancy Updates

```
POST   /v1/cases/{caseId}/recipients              — register a recipient
PATCH  /v1/recipients/{rid}/pregnancy             — update pregnancy diagnosis
PATCH  /v1/recipients/{rid}/parturition           — record parturition outcome
GET    /v1/recipients/{rid}                       — retrieve recipient record
```

Pregnancy updates use RFC 6902 JSON Patch documents so that incremental
veterinary observations can be applied without race conditions. Servers MUST
require `If-Match` against the recipient ETag to apply a patch.

## §7 Post-Natal Verification

The verification resource carries the genetic identity confirmation for a
live offspring. It is the keystone of the standard: registries and regulators
treat the verification report as the externally-citable artefact for the
case.

```
POST   /v1/cases/{caseId}/verifications      — submit a verification report
GET    /v1/verifications/{vid}               — retrieve a verification report
GET    /v1/verifications/{vid}/witnesses     — list independent witness labs
```

A verification report references the donor sample, the recipient, the live
offspring's RFID, and a list of STR (short tandem repeat) loci comparison
results. The report is signed by the issuing reference laboratory and by at
least one independent witness laboratory. Servers MUST refuse to publish a
verification with fewer than the configured minimum number of witnesses for
the operating jurisdiction (default: one witness; programmes that elect to
publish externally citable reports SHOULD configure two or more).

## §8 Evidence Package

The evidence package endpoint produces an exportable archive containing all
records, custody logs, instrument register entries, signed manifests, and
the verification report for a case. The package format is governed by
PHASE-4.

```
POST   /v1/cases/{caseId}/evidence            — request package generation
GET    /v1/evidence/{packageId}               — retrieve a package
GET    /v1/evidence/{packageId}/manifest      — retrieve manifest only
```

Evidence package generation is asynchronous; the POST response carries a
`202 Accepted` and a `Location` header pointing to the package resource,
which transitions through `pending` → `building` → `ready` (or `failed`).

## §9 Authentication and Authorization

The API uses mutually-authenticated TLS for laboratory-to-laboratory and
laboratory-to-registry connections, with client certificates issued by an
accreditation root that is operated by the certifying body. Owner-facing CRM
intermediaries use their own session model; the CRM is the only consumer
that translates owner identity into the opaque `ownerReference` defined in
PHASE-1 §3.

Programmes MUST configure rate limits per client certificate and SHOULD
publish the resulting limits in `Retry-After` headers when the limit is
reached.

## §10 Errors and Problem Details

All error responses are `application/problem+json` per RFC 9457. The `type`
field is a URN under the `urn:wia:pet-cloning` namespace. The defined types
are listed in the catalogue at PHASE-2 Annex A and include:

- `urn:wia:pet-cloning:status-transition` — invalid status change.
- `urn:wia:pet-cloning:cross-case-reference` — referencing across cases.
- `urn:wia:pet-cloning:custody-gap` — custody log has a missing event.
- `urn:wia:pet-cloning:viability-window` — sample exceeds species window.
- `urn:wia:pet-cloning:incomplete-evidence` — evidence build aborted.

Each problem response carries `traceId` (W3C Trace Context propagation)
and `evidenceLink` pointing to the evidence package URL when one exists.

## §11 Caching, Concurrency, and ETag Discipline

GET responses are cacheable per RFC 9111 with `Cache-Control: max-age=60`
unless the resource is a verification or evidence package, in which case
the response is immutable and carries `Cache-Control: max-age=31536000,
immutable`. ETags are mandatory on every PATCH endpoint; conditional
requests use `If-Match`. Servers MUST return `412 Precondition Failed`
when ETags do not match, with a problem document containing the current
ETag in the `serverETag` extension.

## §12 Audit and Observability

Every endpoint emits structured logs with `caseId`, `traceId`, the issuing
client certificate's subject, and the operator's clock skew vs the reference
NTP source. Audit logs are retained for at least seven calendar years from
the last access of the case (PHASE-3 §7), in line with ISO/IEC 27001:2022
A.5.33 expectations for the records concerned.

## §13 Worked Example: From Sample Registration to Verification

The following sequence illustrates the canonical flow for a single case as
exercised against a conformant server. Identifiers are abbreviated for
readability.

1. **Open a case.** A `POST /v1/cases` with `caseAcquiredAt` set to the
   moment the donor consent was countersigned returns the new case in
   `draft` status with a server-assigned `caseId` such as
   `01913a7b-...`. The Location header points to `/v1/cases/01913a7b-...`.
2. **Register the donor.** `POST /v1/cases/{caseId}/donors` with the donor
   record from PHASE-1 §3 transitions the case to `active` if the
   donor's `consentHash` resolves and the regulatory authorisation is
   present (jurisdictions vary, see PHASE-3 §3).
3. **Record the biopsy and freeze.** `POST /v1/donors/{donorId}/samples`
   with the sample shape from PHASE-1 §4 captures the biopsy. A custody
   event is appended via `POST /v1/samples/{sampleId}/custody` for each
   shipment, thaw, and re-freeze.
4. **Source oocytes and reconstruct.** Oocyte records are posted under
   the case; reconstruction events reference the chosen sample and
   oocyte. The server validates referential integrity per §5.
5. **Cleavage and transfer.** Embryo records are appended with cleavage
   stage observations; the recipient's pregnancy and parturition
   updates are applied with JSON Patch under the recipient resource per
   §6.
6. **Verify and publish.** The verification report is signed by the
   issuing reference laboratory and the witness laboratory under §7,
   the case advances to `verified`, and the evidence package is
   produced under §8.

A conformant server completes this flow without error for the canonical
positive vector under `tests/phase-vectors/phase-2-api-interface/`.

## §14 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/` and emits an OpenAPI document
that round-trips the documented resource shapes. The test vectors include
positive and negative cases for every endpoint listed in this document and
for every problem type listed in §10. Conformance is evidenced by the test
matrix submitted as part of the PHASE-4 evidence package.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-pet-cloning
- **Last Updated:** 2026-04-27
