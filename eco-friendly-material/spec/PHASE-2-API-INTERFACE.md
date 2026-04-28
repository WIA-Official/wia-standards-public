# WIA-eco-friendly-material PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-eco-friendly-material
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that
an eco-friendly-material operator (material
producer, EPD programme operator, Type I scheme
operator, Type II self-declarant, verification
body, organisation-level or project-level GHG
reporter) exposes for the records defined in
PHASE-1. The contract carries the material-
registration, life-cycle assessment upload,
environmental-product-declaration publication,
Type II self-declared claim attestation,
organisation- and project-level GHG report
ingestion, and chain-of-custody anchoring
endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 8288
  (Web Linking), RFC 6901 / 6902 (JSON Pointer /
  Patch), RFC 8259 (JSON), RFC 4122 (UUID), RFC
  9421 (HTTP Message Signatures), RFC 8615 (well-
  known URIs)
- W3C Trace Context
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015, ISO/IEC
  17065:2012
- ISO 14040:2006/Amd 1:2020 and ISO 14044:2006/Amd
  1:2017/Amd 2:2020 (LCA principles, requirements)
- ISO 14025:2006 (Type III EPD), ISO 14021:2016
  (Type II claims), ISO 14024:2018 (Type I labels)
- ISO 14067:2018 (carbon footprint of products)
- ISO 14064-1:2018 / 14064-2:2019 / 14064-3:2019
  and ISO 14065:2020
- EN 15804:2012+A2:2019, EN 15978:2011, ISO
  21930:2017
- EU Construction Products Regulation (EU)
  305/2011 and EU Carbon Border Adjustment
  Mechanism Regulation (EU) 2023/956 (the
  declaration fields carried by the registration
  endpoint in §3)

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by
the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical for the WIA
endpoints declared in this PHASE. Schema changes
follow the non-breaking conventions in PHASE-1 §2
(additive fields, additive enum values, no
semantic re-use of existing field names). Every
endpoint carries a per-request signature using HTTP
Message Signatures (RFC 9421) anchored to the
operator's accreditation reference (where the
operator is a verification body), to the
operator's manufacturer registration (where the
operator is a Type II self-declarant), or to the
EPD programme operator's published instructions
(where the operator publishes Type III EPDs); the
signature key set is published at
`/.well-known/wia/eco-friendly-material/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-eco-friendly-material",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "materials":          "/v1/materials",
    "lcaRecords":         "/v1/lca-records",
    "epdRecords":         "/v1/epd-records",
    "type2Claims":        "/v1/type2-claims",
    "ghgRecords":         "/v1/ghg-records",
    "custody":            "/v1/custody-events",
    "openapi":            "/v1/openapi.json",
    "wellKnown":          "/.well-known/wia/eco-friendly-material"
  }
}
```

## §3 Material Registration Endpoints

### §3.1 Register a material

```
POST /v1/materials
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from PHASE-1
(materialFamily, ecoAttribute, declaredUnit,
productCategoryRule). The server returns `201
Created` with the canonical resource URL at
`/v1/materials/{materialId}` and validates the
declared `ecoAttribute` set against the
`materialFamily`: a `compostable` attribute
declared on a `metal` material is rejected with
`422 Unprocessable Entity` carrying an RFC 9457
problem document at `/problems/eco-attribute-
material-family-mismatch` and the position of the
offending field expressed as a JSON Pointer (RFC
6901).

### §3.2 Retrieve a material

```
GET /v1/materials/{materialId}
Accept: application/json
```

The response carries the registered material
record, the link set covering its LCA records,
EPDs, Type II claims, and GHG inventories. A `Link`
header (RFC 8288) points at the related resources.

### §3.3 Search the material registry

```
GET /v1/materials?family={family}&ecoAttribute={attr}
&pcr={pcrId}&jurisdiction={iso3166}
&page={cursor}&size={size}
```

The response is an RFC 8288 `Link`-paginated
collection.

## §4 LCA Upload Endpoints

### §4.1 Submit an LCA

```
POST /v1/lca-records
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature>
```

The multipart body carries one JSON part with the
§4 record from PHASE-1 (systemBoundary,
modulesCovered, impactCategories, inventoryDataset,
cutoffRules, allocationRules, uncertaintyAnalysis)
and one or more file parts holding the LCI dataset
files referenced by `inventoryDataset`. The server
stores the dataset files under a content-addressable
URI (the SHA-256 hex digest is the path segment)
so that PHASE-3 §6 can later reference the artefact
by hash.

The server enforces the modules-covered
enumeration against the system-boundary
declaration: a `cradle-to-gate` boundary with a
`B7` module is rejected with `422 Unprocessable
Entity` and a problem document explaining that the
B-modules belong to the use stage, which is outside
a `cradle-to-gate` boundary.

### §4.2 Retrieve an LCA

```
GET /v1/lca-records/{lcaId}
Accept: application/json
```

The response carries the LCA envelope and the
links to the dataset file at
`/v1/lca-records/{lcaId}/dataset` and to the
sibling LCA records carried by the same material.

## §5 EPD Publication Endpoints

### §5.1 Publish an EPD

```
POST /v1/epd-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §5 record from PHASE-1.
The server verifies that the `programmeRef` is a
known EPD programme operator under ISO 14025 §7,
that the `pcrIdentifier` resolves to a published
PCR, and that the `verificationType` and
`verifierReference` match the declared
`verificationType` enumeration. A `verificationType`
of `third-party-individual` requires
`verifierReference.iso14065AccreditationRef` to be
present; missing accreditation returns `422
Unprocessable Entity` at `/problems/iso14025-
verification-incomplete`.

### §5.2 Retrieve an EPD

```
GET /v1/epd-records/{epdId}
Accept: application/json
```

The response carries the EPD envelope, the
machine-readable summary suitable for ingestion
into a building-level EN 15978 assessment tool,
and the verification-report reference.

### §5.3 Search EPDs

```
GET /v1/epd-records?materialRef={id}
&pcr={pcrId}&validBetween={iso8601}/{iso8601}
&verificationType={type}
&page={cursor}&size={size}
```

## §6 Type II Self-Declared Claim Endpoints

### §6.1 Attest a Type II claim

```
POST /v1/type2-claims
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §6 record from PHASE-1.
The server enforces the ISO 14021 §7 evaluation-
method binding: a claim of `recyclable` whose
`evaluationMethod` is `ISO-14021-§7-recycled-
content-evaluation` is rejected with `422
Unprocessable Entity` at `/problems/iso14021-
evaluation-method-mismatch`.

The server also enforces the ISO 14021 §7
evidence requirement: a `recycled-content` claim
whose `evidenceArtefacts` does not include a mass-
balance attestation is rejected.

### §6.2 Retrieve a Type II claim

```
GET /v1/type2-claims/{claimId}
Accept: application/json
```

## §7 GHG Report Endpoints

### §7.1 Submit a GHG report

```
POST /v1/ghg-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §7 record from PHASE-1.
The server validates the scope envelope against
the `reportLevel` declaration: an `organisation-
iso-14064-1` report MUST carry Scope 1, Scope 2,
and at least one of the Scope 3 categories per
ISO 14064-1 §6.4; a `project-iso-14064-2` report
MUST carry the project baseline and project
emissions per ISO 14064-2 §5.4.

### §7.2 Verify a GHG report

```
POST /v1/ghg-records/{ghgId}/verifications
Content-Type: application/json
Signature: <RFC 9421 signature from a verifier
            with an ISO 14065:2020 accreditation>
```

The verifier's accreditation reference is checked
against the issuing accreditation body's published
register; the `verificationLevel` declared in the
report (`limited-iso-14064-3` or `reasonable-iso-
14064-3`) is bound to the verifier's report
envelope.

## §8 Chain-of-Custody Endpoint

### §8.1 Anchor a custody event

```
POST /v1/custody-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §8 record from PHASE-1.
The server appends the event to the per-artefact
custody chain. PHASE-3 §6 specifies how the chain
is anchored to a public transparency log.

## §9 Error Reporting

Errors are returned using RFC 9457 Problem
Details. The problem-type identifiers are stable
strings rooted at `/problems/` and are documented
in the OpenAPI document. Validation errors carry a
`pointer` field whose value is a JSON Pointer (RFC
6901) into the offending request body. The server
emits a per-request `traceparent` header (W3C
Trace Context) so that a downstream call chain
can be reconstructed for post-incident review.

## §10 Programme Operator Reciprocal Recognition

```
GET /v1/programmes/{programmeId}/recognised-peers
Accept: application/json
```

ISO 14025 §7.6 encourages mutual recognition
between EPD programme operators. The operator
publishes the list of programme operators whose
EPDs the operator accepts as equivalent under a
mutual-recognition agreement. The list carries
the peer programme operator's identifier, the
peer's published instructions URI, and the date
the mutual-recognition agreement was signed. A
downstream consumer running a query that filters
by `mutualRecognition: peer-programme-id` returns
the union of the operator's own EPDs and the
peer's EPDs that satisfy the query.

## §11 Bulk EPD Export

```
GET /v1/epd-records:bulk
Accept: application/x-ndjson
```

A buyer running a portfolio-level analysis (a
construction-project designer integrating many
products into a single building-level
assessment, or a public-procurement authority
running a bulk environmental analysis) requests
the operator's EPD register as a newline-
delimited JSON stream. The endpoint streams the
EPDs in publication-date order; a consumer
resuming the stream provides an `If-Resume-After`
header carrying the last-received publication
timestamp.

## §12 Verifier Webhook

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A verifier registers a webhook to receive a push
notification when a publication assigned to the
verifier is uploaded. The webhook envelope
carries the operator's signing key reference, the
event type (EPD upload, GHG report upload, Type
II claim upload), and the resource identifier;
the verifier's webhook receiver verifies the
signature against the operator's public-key set
before queuing the work item.

## §13 Schema-Validation Note

The OpenAPI 3.1 document at `/v1/openapi.json`
carries JSON Schema 2020-12 schemas for every
request and response envelope. Schemas are
published with a `$schema` declaration and an
`additionalProperties: false` setting on the
top-level objects so that an unknown field
returned by a malformed implementation is
detected at the consumer side. The Schema URIs
are stable across non-breaking schema additions;
breaking schema changes are introduced under a
new major version with a parallel base path
`/v2/`.

## §14 Cache and Concurrency Discipline

Every retrieval endpoint emits an `ETag` header
(RFC 9110 §8.8.3) computed over the canonical
representation of the resource. Conditional
requests (`If-Match`, `If-None-Match`) are honoured
on update endpoints so that a concurrent
publication does not overwrite a peer's update.
The server returns `412 Precondition Failed`
where the conditional request does not match,
together with an RFC 9457 problem document
carrying the current ETag so that the consumer
can refresh and retry.
