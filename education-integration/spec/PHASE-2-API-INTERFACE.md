# WIA-education-integration PHASE 2 — API Interface Specification

**Standard:** WIA-education-integration
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that
an education-integration operator (national
education ministry, multilateral programme
secretariat, international school, education-
development agency, recognition authority,
education-data warehouse) exposes for the records
defined in PHASE-1. The contract carries the
programme registration, ISCED 2011 classification
upload, curriculum publication, learner
registration, recognition-decision issuance,
mobility-flow recording, and chain-of-custody
anchoring endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics), RFC 9111
  (HTTP Caching), RFC 9457 (Problem Details),
  RFC 8288 (Web Linking), RFC 6901 / 6902
  (JSON Pointer / Patch), RFC 8259 (JSON), RFC
  4122 (UUID), RFC 9421 (HTTP Message
  Signatures), RFC 8615 (well-known URIs)
- W3C Trace Context
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012
- ISO 21001:2018 (educational organisations
  management system requirements)
- ISO/IEC 19796-1:2005 (e-learning quality
  management)
- IMS Global LTI 1.3 / Advantage, OneRoster
  1.2, Caliper Analytics 1.2, QTI 3.0
- IMS Open Badges 3.0, W3C Verifiable
  Credentials Data Model v2.0
- UNESCO ISCED 2011, ISCED-F 2013

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published
by the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical for the WIA
endpoints declared in this PHASE. Schema
changes follow the non-breaking conventions in
PHASE-1 §2. Every endpoint carries a per-
request signature using HTTP Message Signatures
(RFC 9421) anchored to the operator's
accreditation reference (the ISO 21001
certification, the Lisbon / Tokyo / Global
Convention competent-authority designation,
the multilateral programme's secretariat
designation); the signature key set is
published at
`/.well-known/wia/education-integration/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-education-integration",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":          "/v1/programmes",
    "iscedRecords":        "/v1/isced-records",
    "curricula":           "/v1/curricula",
    "learners":            "/v1/learners",
    "recognitionDecisions": "/v1/recognition-decisions",
    "mobilityRecords":     "/v1/mobility-records",
    "custody":             "/v1/custody-events",
    "openapi":             "/v1/openapi.json",
    "wellKnown":           "/.well-known/wia/education-integration"
  }
}
```

## §3 ISCED Programme Endpoints

### §3.1 Register an ISCED programme

```
POST /v1/isced-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from
PHASE-1 (iscedLevel, iscedField, duration,
awardName, nationalFrameworkRef). The server
returns `201 Created` with the canonical
resource URL at
`/v1/isced-records/{iscedRecordId}` and
validates the declared `iscedLevel` against
the ISCED 2011 level table: a programme
awarding a "Bachelor of Science" with
`iscedLevel: 5` (short-cycle tertiary) is
rejected with `422 Unprocessable Entity`
carrying an RFC 9457 problem document at
`/problems/isced-level-award-mismatch` and the
position of the offending field expressed as a
JSON Pointer (RFC 6901), since a Bachelor
degree is ISCED Level 6.

### §3.2 Retrieve an ISCED record

```
GET /v1/isced-records/{iscedRecordId}
Accept: application/json
```

The response carries the registered ISCED
record, the link set covering its curricula,
its enrolled learners, and its issued
recognition decisions. A `Link` header (RFC
8288) carries pagination cursors when the
linked-resource collection exceeds the per-
collection page size declared at the
`/v1/openapi.json` document.

### §3.3 Search ISCED records

```
GET /v1/isced-records?level={level}
&field={field}&country={iso3166-alpha3}
&page={cursor}&size={size}
```

The response is an RFC 8288 `Link`-paginated
collection of ISCED records matching the
filter. The `level` filter accepts the ISCED
2011 levels 0 through 8; the `field` filter
accepts the ISCED-F 2013 narrow-field codes.

## §4 Curriculum Endpoints

### §4.1 Publish a curriculum

```
POST /v1/curricula
Content-Type: application/json
Signature: <RFC 9421 signature from the
            operator's quality-assurance
            authority>
```

Request body carries the §4 record from
PHASE-1. The server validates that the sum of
the per-module `creditValue` is consistent
with the declared `totalCredits` value within
the operator's tolerance threshold (a 0.5-
credit tolerance is the default per the
operator's declared discipline).

### §4.2 Retrieve a curriculum

```
GET /v1/curricula/{curriculumId}
Accept: application/json
```

### §4.3 LTI integration

```
GET /v1/curricula/{curriculumId}/lti-launch
```

Where the operator integrates with an LMS via
IMS LTI 1.3, the endpoint returns the LTI
launch payload bound to the requesting LMS
context.

## §5 Learner Endpoints

### §5.1 Register a learner

```
POST /v1/learners
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §5 record from
PHASE-1. The server uses a pseudonymous
learner identifier; the linkage between the
pseudonym and the directly-identifying record
is held in the operator's identity vault
(PHASE-3 §8).

### §5.2 Retrieve a learner record

```
GET /v1/learners/{learnerId}
Accept: application/json
```

### §5.3 Per-learner transcript

```
GET /v1/learners/{learnerId}/transcript
Accept: application/json
```

The transcript carries the learner's per-
programme enrolment, the per-programme credit
accumulation, the per-mobility credit
recognition, and the per-decision recognition
record. The transcript is signed by the
operator using the operator's signing-key set.

## §6 Recognition-Decision Endpoints

### §6.1 Publish a recognition decision

```
POST /v1/recognition-decisions
Content-Type: application/json
Signature: <RFC 9421 signature from the
            recognition authority>
```

Request body carries the §6 record from
PHASE-1. The server enforces the convention-
to-authority binding: a `Lisbon-Recognition-
Convention-1997` decision MUST be issued by
an authority designated under that convention.
A mismatch returns `403 Forbidden` at
`/problems/recognition-authority-not-
designated`.

### §6.2 Retrieve a recognition decision

```
GET /v1/recognition-decisions/{recognitionId}
Accept: application/json
```

### §6.3 Appeal a recognition decision

```
POST /v1/recognition-decisions/{recognitionId}/appeals
Content-Type: application/json
Signature: <RFC 9421 signature from the
            applicant>
```

Per Lisbon Article III.5, the applicant
lodges an appeal against the recognition
decision. The appeal carries the substantive
grounds under the convention's substantive
criteria (absence of substantial difference,
fair procedure, transparency).

## §7 Mobility Endpoints

### §7.1 Record a mobility flow

```
POST /v1/mobility-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §7 record from
PHASE-1. The server enforces that the
`fromCountry` and `toCountry` are different
(except where the operator declares an intra-
country exchange) and that the
`learningAgreementRef` URL carries a
countersigned learning agreement signed by
the sending and receiving institutions.

### §7.2 Retrieve a mobility record

```
GET /v1/mobility-records/{mobilityId}
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
(RFC 6901). The server emits a per-request
`traceparent` header (W3C Trace Context).

## §9 Concurrency and Cache

Every retrieval endpoint emits an `ETag`
header (RFC 9110 §8.8.3). Conditional
requests (`If-Match`, `If-None-Match`) are
honoured.

## §10 Bulk Export for Education-Statistics

```
GET /v1/programmes:bulk
Accept: application/x-ndjson
```

A multilateral education-statistics body
(UNESCO Institute for Statistics, OECD
Education Directorate) requests the operator's
programme register as a newline-delimited JSON
stream. The endpoint streams the programme
records in registration-date order; a consumer
resuming the stream provides an
`If-Resume-After` header carrying the last-
received registration timestamp.

## §11 Caliper Analytics Endpoint

```
POST /v1/caliper-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Where the operator integrates with an LMS via
IMS Caliper 1.2, the endpoint receives Caliper
events from the LMS for the operator's
analytics-and-reporting programme.

## §12 Verifiable-Credentials Re-Issuance

```
GET /v1/learners/{learnerId}/credentials
Accept: application/json
```

A learner's diploma or transcript is re-
issuable as a W3C Verifiable Credential signed
by the operator's signing-key set so that a
downstream recognition authority can verify
the credential without contacting the issuing
operator directly.

## §13 Schema-Validation and Conformance

The OpenAPI 3.1 document at
`/v1/openapi.json` carries JSON Schema 2020-12
schemas for every request and response
envelope. Schemas are published with
`additionalProperties: false` on top-level
objects.

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation
against this specification.

## §14 Webhook Endpoint for Recognition Notifications

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A learner, a sending institution, or a
receiving institution registers a webhook to
receive a push notification when a recognition
decision is issued, a mobility record is
recorded, or a transcript is re-issued.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §15 Subscriptions for the Education Statistics

```
GET /v1/programmes/{programmeId}/statistics
?period={iso8601}/{iso8601}
Accept: application/json
```

A multilateral education-statistics body
(UNESCO Institute for Statistics, OECD
Education Directorate) subscribes to the
operator's per-period statistics envelope
covering the per-ISCED-level enrolment count,
the per-ISCED-field graduate count, and the
per-jurisdictional intake-and-outflow mobility
flow. The response is signed by the operator's
public-key set so that the downstream
multilateral consumer can validate the
envelope offline.

## §16 Pseudonymisation Discipline

The operator's API does not surface the
learner's directly-identifying record. The
learner-identity vault is an operator-internal
service accessed only through a separate
authenticated endpoint subject to the
operator's role-based access-control policy
declared in PHASE-3 §9.

## §17 Multi-Language Programme Surface

```
GET /v1/programmes/{programmeId}?lang={lang}
Accept: application/json
```

The operator's programme record is published
in each operator-declared language with the
programme's title, the qualification's award
name, the curriculum's per-module title, and
the recognition decision's substantive
reasoning translated. The classification codes
(ISCED level, ISCED field, EQF level) are
language-neutral and carried in the canonical
form across all retrievals.
