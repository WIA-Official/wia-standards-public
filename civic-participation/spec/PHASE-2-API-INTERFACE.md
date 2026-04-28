# WIA-civic-participation PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-civic-participation
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that
a public-administration body operating a citizen-
engagement platform exposes for the records
defined in PHASE-1. The contract carries the
consultation publication, citizen-submission
ingestion, deliberation record publication,
outcome announcement, engagement-KPI publication,
and chain-of-custody anchoring endpoints. The
API is the canonical interoperability layer
between the operator's platform, the citizen
client, the elected representative's analysis
workspace, the transparency observer's audit
client, and the smart-city KPI dashboard.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details for HTTP
  APIs), RFC 8288 (Web Linking), RFC 6901 / 6902
  (JSON Pointer / Patch), RFC 8259 (JSON), RFC
  4122 (UUID), RFC 9421 (HTTP Message Signatures),
  RFC 8615 (well-known URIs)
- W3C Trace Context, W3C Data Catalog Vocabulary
  (DCAT) v3, W3C Open Digital Rights Language
  (ODRL) 2.2 Information Model and Vocabulary
- W3C Decentralised Identifiers (DIDs) v1.0 and
  W3C Verifiable Credentials Data Model v2.0
- ISO/IEC 27001:2022 (information-security
  management)
- ITU-T Y.4900/L.1600 (smart sustainable cities
  KPI overview)
- ISO 37120:2018, ISO 37122:2019, ISO 37123:2019
- ISO 18091:2019 (local government QMS)
- OECD Recommendation of the Council on Open
  Government (OECD/LEGAL/0438)
- UN DESA E-Government Survey
- World Bank GovTech Maturity Index
- EU Single Digital Gateway Regulation (EU)
  2018/1724 and EU Interoperable Europe Act
  Regulation (EU) 2024/903 (cited where the
  operator participates in the SDG or the
  Interoperable Europe Portal)
- EU GDPR (Regulation (EU) 2016/679) Articles
  6 / 9 / 12-22 (the processing-basis envelope
  carried by the submission endpoint)
- KR 행정기본법 / KR 정보공개법

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by
the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical for the WIA
endpoints declared in this PHASE. Schema changes
follow the non-breaking conventions in PHASE-1
§2 (additive fields, additive enum values).
Every endpoint carries a per-request signature
using HTTP Message Signatures (RFC 9421)
anchored to the operator's e-government
identifier (the SDG operator identifier where
the operator participates in the EU Single
Digital Gateway, the operator's national
government registry identifier otherwise); the
signature key set is published at
`/.well-known/wia/civic-participation/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-civic-participation",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "consultations":      "/v1/consultations",
    "submissions":        "/v1/submissions",
    "deliberations":      "/v1/deliberations",
    "outcomes":           "/v1/outcomes",
    "kpiRecords":         "/v1/kpi-records",
    "custody":            "/v1/custody-events",
    "openapi":            "/v1/openapi.json",
    "wellKnown":          "/.well-known/wia/civic-participation"
  }
}
```

## §3 Consultation Endpoints

### §3.1 Publish a consultation

```
POST /v1/consultations
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from PHASE-1
(consultationType, engagementLadder, subject,
rightsExpression, openingDate, closingDate,
responseSchedule, jurisdictionalScope). The
server validates the `engagementLadder` against
the `consultationType`: a `binding-referendum`
consultation MUST declare an `engagementLadder`
of `empower`, while an `e-petition` consultation
MUST declare `inform` or `consult` per the
operator's published petition rules. A mismatch
returns `422 Unprocessable Entity` with an RFC
9457 problem document at `/problems/oecd-
engagement-ladder-mismatch` and the offending
field expressed as a JSON Pointer (RFC 6901).

The `rightsExpression` is parsed as a W3C ODRL
2.2 policy and validated against the operator's
ODRL profile published in the discovery
endpoint.

### §3.2 Retrieve a consultation

```
GET /v1/consultations/{consultationId}
Accept: application/json
```

The response carries the consultation envelope
together with a `Link` header (RFC 8288)
pointing at the submissions, deliberations, and
outcomes already published.

### §3.3 Search consultations

```
GET /v1/consultations?type={type}
&jurisdictionalScope={scope}&openBetween={iso8601}/{iso8601}
&engagementLadder={ladder}
&page={cursor}&size={size}
```

The response is an RFC 8288 `Link`-paginated
collection.

## §4 Submission Endpoints

### §4.1 Submit a citizen contribution

```
POST /v1/consultations/{consultationId}/submissions
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature where the
            submitter is institutional, omitted
            where the submitter is anonymous or
            pseudonymous>
```

The multipart body carries one JSON part with the
§4 record from PHASE-1 (submitter, submissionType,
body, consentDirective) and zero or more
attachments. The server stores the attachments
under a content-addressable URI (the SHA-256 hex
digest is the path segment).

The server enforces the consultation's submission-
type enumeration: a consultation that declares
`submissionType: vote` rejects a `free-text`
submission with `422 Unprocessable Entity` at
`/problems/consultation-submission-type-mismatch`.

Where the submitter declares a verified-citizen
identity via a W3C Verifiable Credential issued
by a national identity provider, the server
verifies the credential's issuer signature and
the credential's revocation status before
accepting the submission.

### §4.2 Acknowledge a submission

```
POST /v1/submissions/{submissionId}/acknowledgement
Content-Type: application/json
Signature: <RFC 9421 signature from the operator>
```

The operator publishes a per-submission
acknowledgement at the cadence declared in the
consultation's `responseSchedule`. The
acknowledgement carries the submission's
position in the consultation's intake queue,
the expected processing-completion date, and
the contact-channel for follow-up.

### §4.3 Retrieve a submission

```
GET /v1/submissions/{submissionId}
Accept: application/json
Authorization: <bearer token from the submitter
                 or from a transparency observer
                 with audit scope>
```

The response carries the submission envelope.
The submitter's identity envelope is redacted
according to the submitter's declared identity
disclosure scope (anonymous, pseudonymous,
verified-citizen with name redacted, verified-
citizen with name disclosed).

### §4.4 Search submissions

```
GET /v1/consultations/{consultationId}/submissions
?submissionType={type}&submittedAfter={iso8601}
&submittedBefore={iso8601}
&page={cursor}&size={size}
```

## §5 Deliberation Endpoints

### §5.1 Publish a deliberation record

```
POST /v1/deliberations
Content-Type: application/json
Signature: <RFC 9421 signature from the
            operator's deliberative-forum
            secretariat>
```

Request body carries the §5 record from PHASE-1.
The server enforces the participant-set
discipline: a `citizen-panel-sortition`
deliberation MUST declare a participant set
selected through a sortition algorithm with the
algorithm's parameters carried in
`participantSet[].selectionMethod`.

### §5.2 Retrieve a deliberation record

```
GET /v1/deliberations/{deliberationId}
Accept: application/json
```

## §6 Outcome Endpoints

### §6.1 Publish an outcome

```
POST /v1/outcomes
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §6 record from PHASE-1.
The server validates that the publication date
is on or before the consultation's declared
`responseSchedule.publishOutcomeBy` date; a late
publication is published as the outcome
envelope with a `publishedLate: true` flag and
the underlying reason recorded in the
chain-of-custody record (PHASE-3 §6).

### §6.2 Retrieve an outcome

```
GET /v1/outcomes/{outcomeId}
Accept: application/json
```

### §6.3 Subscribe to outcome notifications

```
POST /v1/consultations/{consultationId}/outcome-webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A submitter, a transparency observer, or an
external aggregator registers a webhook to
receive a push notification when the outcome is
published. The webhook envelope carries the
operator's signing-key reference, the event
type, and the resource identifier.

## §7 Engagement-KPI Endpoint

### §7.1 Publish a KPI bundle

```
POST /v1/kpi-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

The KPI bundle is published per the cadence
declared in the operator's reporting plan
(annual, biennial). The server validates the
KPI envelope against the operator's declared
KPI bundle (ITU-T Y.4900 + ISO 37120/37122 +
UN DESA + OECD).

### §7.2 Retrieve KPI history

```
GET /v1/programmes/{programmeId}/kpi-history
?since={iso8601}&until={iso8601}
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
Details. The problem-type identifiers are
stable strings rooted at `/problems/` and are
documented in the OpenAPI document. Validation
errors carry a `pointer` field whose value is a
JSON Pointer (RFC 6901). The server emits a
per-request `traceparent` header (W3C Trace
Context).

## §9 Concurrency and Cache Discipline

Every retrieval endpoint emits an `ETag` header
(RFC 9110 §8.8.3). Conditional requests
(`If-Match`, `If-None-Match`) are honoured on
update endpoints. The server returns `412
Precondition Failed` where the conditional
request does not match.

## §10 Bulk Export for Transparency Observers

```
GET /v1/consultations/{consultationId}/submissions:bulk
Accept: application/x-ndjson
Authorization: <bearer token from a transparency
                 observer with audit scope>
```

A transparency observer running a portfolio-
level audit requests the consultation's
submission corpus as a newline-delimited JSON
stream. The export is gated on the observer's
audit-scope authorisation declared in the
operator's open-data publication policy. A
consumer resuming the stream provides an
`If-Resume-After` header carrying the last-
received submission timestamp.

## §11 Single Digital Gateway Federation

```
GET /v1/consultations/{consultationId}/sdg-summary
Accept: application/json
```

A consultation in scope of the EU Single Digital
Gateway publishes a machine-readable summary
suitable for ingestion into the SDG's cross-
border-citizen-engagement layer. The summary
carries the consultation's title in each EU
official language declared by the operator, the
opening and closing dates, and the per-language
submission endpoint reference.

## §12 Webhook Endpoint for KPI Publication

```
POST /v1/programmes/{programmeId}/kpi-webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A smart-city dashboard or a transparency
observer registers a webhook to receive a push
notification when a KPI bundle is published.

## §13 Schema-Validation and Conformance

The OpenAPI 3.1 document at `/v1/openapi.json`
carries JSON Schema 2020-12 schemas for every
request and response envelope. Schemas are
published with `additionalProperties: false` on
top-level objects so that an unknown field is
detected at the consumer side.

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation
against this specification. Each vector
references the relevant ITU-T Y.4900 / ISO
37120 / OECD CITIZEN-ENGAGEMENT clause.
