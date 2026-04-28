# WIA-fire-resistant-material PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-fire-resistant-material
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that
a fire-resistant-material operator (material
producer, fire-testing laboratory, notified body,
certification body, Authority Having Jurisdiction,
or public-procurement authority) exposes for the
records defined in PHASE-1. The contract carries
the material-registration, reaction-to-fire test
upload, resistance-to-fire test upload,
Declaration of Performance publication, AHJ
audit ingestion, and chain-of-custody anchoring
endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110, RFC 9111, RFC 9457, RFC 8288,
  RFC 6901 / 6902, RFC 8259, RFC 4122, RFC
  9421, RFC 8615
- W3C Trace Context
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012, ISO/IEC 17025:2017
- EN 13501-1:2018+A1:2019, EN 13501-2:2023,
  EN 13823:2020+A1:2022, EN ISO 11925-2:2020
- ISO 1182:2020, ISO 1716:2018, ISO 5660-1:2015,
  ISO 5660-2:2002, ISO 9239-1:2010, ISO 13943:
  2017
- ASTM E84-23, UL 723:2018, ASTM E119-23a,
  ASTM E136-22, ASTM E2257-22
- NFPA 251 / 252 / 257 / 259 / 268
- EN 1363-1:2020, EN 1364 series, EN 1365 series,
  EN 1366 series, EN 1634-1
- KS F 2271:2016, KS F 2257-1:2019
- EU Construction Products Regulation (EU)
  305/2011, EU Decision 2000/147/EC, EU Decision
  2000/367/EC

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by
the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical. Schema changes
follow the non-breaking conventions in PHASE-1
§2. Every endpoint carries a per-request
signature using HTTP Message Signatures (RFC
9421) anchored to the operator's ISO/IEC 17025
laboratory accreditation, the operator's ISO/IEC
17065 product-certification body accreditation,
or the operator's notified-body designation
under EU CPR Article 39; the signature key set
is published at
`/.well-known/wia/fire-resistant-material/keys.
json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-fire-resistant-material",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "materials":          "/v1/materials",
    "rtfTestRecords":     "/v1/rtf-test-records",
    "rfTestRecords":      "/v1/rf-test-records",
    "dopRecords":         "/v1/dop-records",
    "ahjAuditRecords":    "/v1/ahj-audits",
    "custody":            "/v1/custody-events",
    "openapi":            "/v1/openapi.json",
    "wellKnown":          "/.well-known/wia/fire-resistant-material"
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

Request body carries the §3 record from PHASE-1.
The server validates the `materialFamily` against
the `intendedUse`: a `flame-retardant-polymer`
declared with `intendedUse: structural-beam-
column` is rejected with `422 Unprocessable
Entity` at `/problems/cpr-intended-use-family-
mismatch` and the offending field expressed as a
JSON Pointer (RFC 6901).

### §3.2 Retrieve a material

```
GET /v1/materials/{materialId}
Accept: application/json
```

### §3.3 Search materials

```
GET /v1/materials?family={family}
&intendedUse={use}&euroclass={class}
&fireResistanceClass={REI}
&dopExpiringBefore={iso8601}
&page={cursor}&size={size}
```

## §4 Reaction-to-Fire Test Endpoints

### §4.1 Upload a reaction-to-fire test

```
POST /v1/rtf-test-records
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature from a fire-
            testing laboratory's accreditation
            certificate>
```

The multipart body carries one JSON part with the
§4 record from PHASE-1 (testStandard,
testLaboratory, measurementResult,
classification) and one or more file parts
holding the test report, the calibration record,
the photographic evidence (per EN 13823 §7), and
the raw sensor data file. The server stores the
report files under content-addressable URIs (the
SHA-256 hex digest is the path segment).

The server enforces the test-method-to-
classification mapping table:

- A `classification.primaryClass` of `A1`
  REQUIRES test reports under both ISO 1182 and
  ISO 1716 in the underlying record set.
- A `classification.primaryClass` of `A2`
  REQUIRES the ISO 1716 test result and the EN
  13823 SBI test result, with the additional
  ISO 1182 result where the producer claims the
  homogeneous-product route per EN 13501-1
  Annex B.
- A `classification.primaryClass` of `B`, `C`,
  or `D` REQUIRES the EN 13823 SBI test and
  the EN ISO 11925-2 small-flame test.
- A `classification.primaryClass` of `E`
  REQUIRES the EN ISO 11925-2 small-flame test.

A mismatch returns `422 Unprocessable Entity`
at `/problems/en13501-1-test-set-incomplete`.

### §4.2 Retrieve a reaction-to-fire test

```
GET /v1/rtf-test-records/{rtfTestId}
Accept: application/json
```

## §5 Resistance-to-Fire Test Endpoints

### §5.1 Upload a resistance-to-fire test

```
POST /v1/rf-test-records
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature>
```

The multipart body carries the §5 record from
PHASE-1 (testStandard, testLaboratory,
measurementResult, classification) and the
supporting evidence (the EN 1363-1 furnace
calibration, the test specimen design, the
heating-curve record).

The server validates the EN 1363-1 failure
criteria envelope: a record claiming `REI 60`
classification MUST carry passing time-to-
failure values for all three R, E, and I
criteria at or above 60 minutes. A record
claiming `EI` only (no load-bearing) MUST omit
the R criterion.

### §5.2 Retrieve a resistance-to-fire test

```
GET /v1/rf-test-records/{rfTestId}
Accept: application/json
```

## §6 Declaration-of-Performance Endpoints

### §6.1 Publish a Declaration of Performance

```
POST /v1/dop-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §6 record from PHASE-1.
The server validates the `harmonisedSpec`
reference (the harmonised standard or European
Assessment Document) against the EU CPR
hENs / EAD register, and the `notifiedBodyRef`
against the EU NANDO database for notified
bodies. A withdrawn or expired notified-body
reference returns `410 Gone`.

The server enforces the conformity-assessment-
system mapping under EU CPR Annex V: a fire-
performance product family in scope of system
1+ requires the notified body to issue a
certificate of constancy of performance and to
conduct continuing surveillance per EU CPR
Article 39.

### §6.2 Retrieve a Declaration of Performance

```
GET /v1/dop-records/{dopId}
Accept: application/json
```

## §7 AHJ Audit Endpoints

### §7.1 Submit an AHJ audit outcome

```
POST /v1/ahj-audits
Content-Type: application/json
Signature: <RFC 9421 signature from the AHJ's
            authorised inspector>
```

Request body carries the §7 record from PHASE-1.
The server links the audit to the building
project and to the material record so that the
audit trail is preserved across building life-
cycle events.

### §7.2 Retrieve an AHJ audit

```
GET /v1/ahj-audits/{auditId}
Accept: application/json
```

## §8 Custody and Error Reporting

### §8.1 Anchor a custody event

```
POST /v1/custody-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

### §8.2 Error envelope

Errors are returned using RFC 9457 Problem
Details. Validation errors carry a `pointer`
field (RFC 6901). The server emits a per-request
`traceparent` header (W3C Trace Context).

## §9 Concurrency and Cache

Every retrieval endpoint emits an `ETag` header
(RFC 9110 §8.8.3). Conditional requests
(`If-Match`, `If-None-Match`) are honoured on
update endpoints. The server returns `412
Precondition Failed` where the conditional
request does not match.

## §10 Bulk Export for Building-Code Audits

```
GET /v1/dop-records:bulk
Accept: application/x-ndjson
```

A building-code authority running a bulk audit
of installed materials in a portfolio of
buildings requests the operator's DoP register
as a newline-delimited JSON stream. The endpoint
streams the DoP records in publication-date
order; a consumer resuming the stream provides
an `If-Resume-After` header.

## §11 Notified-Body Surveillance Endpoint

```
GET /v1/programmes/{programmeId}/surveillance
Accept: application/json
Signature: <RFC 9421 signature from a notified
            body>
```

A notified body conducting continuing
surveillance under EU CPR Article 39 requests
the operator's manufacturing-process records,
the per-batch test result set, and the per-
batch DoP reference. The endpoint authenticates
the requester's notified-body designation
identifier against the EU NANDO database.

## §12 Building-Information-Modelling Integration

```
GET /v1/materials/{materialId}/bim-summary
Accept: application/json
```

The operator publishes a machine-readable
summary suitable for ingestion into a Building
Information Modelling tool. The summary carries
the material's geometry-and-composition
declaration, the reaction-to-fire and
resistance-to-fire classifications, and the per-
classification time-to-failure value so that the
BIM tool can parameterise the building-level
fire-protection design.

## §13 Schema-Validation Note

The OpenAPI 3.1 document at `/v1/openapi.json`
carries JSON Schema 2020-12 schemas for every
request and response envelope. Schemas are
published with `additionalProperties: false` on
top-level objects so that an unknown field
returned by a malformed implementation is
detected at the consumer side. The Schema URIs
are stable across non-breaking schema additions.

## §14 Webhook Endpoint for Notified-Body
       Notifications

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A notified body registers a webhook to receive
a push notification when a manufacturer publishes
a new DoP, a corrective-action announcement, or
a withdrawal under EU CPR Article 56. The webhook
envelope carries the operator's signing-key
reference, the event type, and the resource
identifier; the notified body's webhook receiver
verifies the signature before queuing the work
item.

## §15 Conformance Test-Vector Endpoint

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation
against this specification. Each vector carries
the request, the expected response, the test-
report identifier from PHASE-1 §4 / §5, and the
classification outcome that the test-of-record
example reproduces. A laboratory or notified
body running an interoperability check uses the
endpoint to confirm that the operator's API
reproduces the published classification logic
deterministically across the EN 13501-1 / EN
13501-2 / ASTM E84 / ASTM E119 enumeration set.

## §16 Bilingual Public Retrieval

```
GET /v1/dop-records/{dopId}?lang={lang}
Accept: application/json
```

The operator's public retrieval endpoint accepts
a `lang` query parameter (`en`, `ko`, `de`,
`fr`, `es`, `ja`, `zh`) and returns the public
fields with the harmonised-standard reference
in the requested language. The classification
codes (Euroclass, REI, EI) are language-
neutral and carried in the canonical form
across all retrievals.
