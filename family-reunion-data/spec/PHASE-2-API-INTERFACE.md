# WIA-family-reunion-data PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-family-reunion-data
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract
that a humanitarian operator (Red Cross or Red
Crescent national society, UNHCR field office,
IOM field office, Ministry of the Interior
missing-persons register, child-protection
authority, inter-country adoption central
authority, or war-graves commission) exposes
for the records defined in PHASE-1. The
contract carries the person registration,
tracing-request lodgement, identification-
anchor publication, inter-agency transfer
publication, HXL coordination feed
publication, and chain-of-custody anchoring
endpoints. The API is the canonical inter-
agency interoperability layer between the
operator, the partner agency, the inter-agency
coordination dashboard, and the per-case
beneficiary.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110, RFC 9111, RFC 9457, RFC 8288,
  RFC 6901 / 6902, RFC 8259, RFC 4122, RFC
  9421, RFC 8615, RFC 6234 (SHA-256), RFC
  8032 (Ed25519)
- W3C Trace Context, W3C ODRL 2.2, W3C VC v2.0
- ISO/IEC 27001:2022, ISO/IEC 27018:2019
- ISO 19115-1/-2 (geographic metadata), ISO
  6709 (geographic point), ISO 3166-1/-2, ISO
  5218, ISO 639, ISO 4217
- ICRC RFL Strategy and ICRC Professional
  Standards for Protection Work
- UNHCR ProGres v4 schema and UNHCR Personal
  Data Policy
- IOM DTM Methodological Framework and IOM
  Data Protection Manual
- HXL Standard maintained by the OCHA Centre
  for Humanitarian Data
- 1949 Geneva Conventions and 1977 Additional
  Protocols
- 1989 CRC, 1993 Hague ICA, 1980 Hague ICCA,
  1963 VCCR Article 36, 1961 Statelessness
  Convention, 1951 Refugee Convention and
  1967 Protocol
- EU GDPR Articles 6 / 9 / 12-22 / 46-49 / 89
- EU Council Directive 2003/86/EC (right to
  family reunification)
- KR 출입국관리법 / KR 난민법

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published
by the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical. Schema changes
follow the non-breaking conventions in PHASE-1
§2. Every endpoint carries a per-request
signature using HTTP Message Signatures (RFC
9421) anchored to the operator's institutional
identifier (the ICRC RFL coordination identifier,
the UNHCR partnership-agreement reference, the
IOM operational identifier, or the relevant
national-authority registration); the signature
key set is published at
`/.well-known/wia/family-reunion-data/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-family-reunion-data",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "personRecords":      "/v1/person-records",
    "tracingRequests":    "/v1/tracing-requests",
    "identificationAnchors": "/v1/identification-anchors",
    "transfers":          "/v1/transfers",
    "hxlFeeds":           "/v1/hxl-feeds",
    "custody":            "/v1/custody-events",
    "openapi":            "/v1/openapi.json",
    "wellKnown":          "/.well-known/wia/family-reunion-data"
  }
}
```

## §3 Person Registration Endpoints

### §3.1 Register a person

```
POST /v1/person-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from PHASE-1
(registrationContext, identifierBindings,
givenName, familyName, dateOfBirth,
placeOfBirth, sexCode, nationalityCodes,
spokenLanguages, protectionStatus,
vulnerabilityFlags, consentDirective). The
server validates the per-context required
fields:

- A `unhcr-progres-individual` registration
  MUST carry the UNHCR ProGres individual
  identifier in `identifierBindings`.
- An `icca-1993-adoption` registration MUST
  carry the central-authority's case
  identifier and the consent of both the
  receiving and originating central
  authorities.
- A `separated-minor-unaccompanied` registration
  MUST carry an `appointedGuardianRef` field
  identifying the appointed temporary
  guardian under the operator's local
  child-protection law.

A missing per-context field returns `422
Unprocessable Entity` at `/problems/per-
context-required-field-missing` with the
offending field's JSON Pointer (RFC 6901).

### §3.2 Retrieve a person record

```
GET /v1/person-records/{personId}
Accept: application/json
Authorization: <bearer token from the operator's
                 protection officer or the
                 person's authenticated agent>
```

The response is redacted per the person's
consent directive. A response that omits a
field due to consent declares the omission with
a `redacted: true` flag for that field so that
a downstream consumer can detect the redaction
without inferring it from missing data.

### §3.3 Search person records

```
GET /v1/person-records?registrationContext={ctx}
&jurisdiction={iso3166}&protectionStatus={status}
&page={cursor}&size={size}
```

Search results are restricted to the
authenticated agent's authorisation scope
under the operator's documented data-access
discipline.

## §4 Tracing-Request Endpoints

### §4.1 Lodge a tracing request

```
POST /v1/tracing-requests
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §4 record from PHASE-1
(requesterRef, soughtPersonRef,
relationshipDeclared, separationContext,
tracingChannels). Where the requester is not
already registered, the request body carries
the requester's declared identity envelope; the
operator's protection officer reviews the
envelope and creates a registration record per
§3.1 if the requester consents.

### §4.2 Update a tracing request status

```
PATCH /v1/tracing-requests/{tracingId}
Content-Type: application/json-patch+json
Signature: <RFC 9421 signature from the
            operator's protection officer>
```

The PATCH body carries an RFC 6902 JSON Patch
that updates the `rfStatus` field. A status
change is recorded as a chain-of-custody
event.

### §4.3 Retrieve a tracing request

```
GET /v1/tracing-requests/{tracingId}
Accept: application/json
Authorization: <bearer token>
```

## §5 Identification-Anchor Endpoints

### §5.1 Anchor an identification

```
POST /v1/identification-anchors
Content-Type: application/json
Signature: <RFC 9421 signature from the
            operator's supervising protection
            officer>
```

Request body carries the §5 record from PHASE-1
(partyAref, partyBref, matchEvidence,
attestation). The server validates that both
party references resolve to the operator's
person register or to a partner operator's
person register reachable through a documented
data-sharing agreement.

The `matchEvidence` envelope is reviewed by
the supervising protection officer per the
ICRC Professional Standards for Protection
Work; the per-anchor attestation declares the
review outcome.

### §5.2 Retrieve an identification anchor

```
GET /v1/identification-anchors/{anchorId}
Accept: application/json
Authorization: <bearer token>
```

## §6 Inter-Agency Transfer Endpoints

### §6.1 Publish a transfer

```
POST /v1/transfers
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §6 record from PHASE-1
(caseRef, fromOperator, toOperator, legalBasis,
consentRef). The server validates the
`legalBasis` against the operator's documented
data-sharing arrangement with the receiving
operator. A transfer to a non-EU country
carries the GDPR Article 46-49 appropriate
safeguard reference.

### §6.2 Retrieve a transfer

```
GET /v1/transfers/{transferId}
Accept: application/json
Authorization: <bearer token>
```

## §7 HXL Coordination Endpoints

### §7.1 Publish an HXL feed

```
POST /v1/hxl-feeds
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §7 record from PHASE-1
(coordinationContext, hxlTagVersion,
datasetUri, refreshFrequency). The HXL feed
URI returns a per-row HXL-tagged dataset that
is consumed by the inter-agency coordination
dashboard.

### §7.2 Retrieve an HXL feed

```
GET /v1/hxl-feeds/{hxlId}
Accept: application/json | text/csv
```

The CSV response carries HXL-tagged columns per
the HXL Standard.

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
field (RFC 6901). The server emits a per-
request `traceparent` header (W3C Trace
Context). The error message is redacted to
avoid disclosing personal data of the affected
person.

## §9 Concurrency and Cache Discipline

Every retrieval endpoint emits an `ETag`
header (RFC 9110 §8.8.3). Conditional requests
are honoured. The cache TTL is set short
(typically under 60 seconds) for person and
tracing endpoints so that consent withdrawals
propagate rapidly.

## §10 Restricted Bulk Export for Protection Audits

```
GET /v1/programmes/{programmeId}/cases:bulk
Accept: application/x-ndjson
Authorization: <bearer token from a designated
                 protection auditor>
```

A protection auditor (a designated member of
the operator's supervisory board, an external
ICRC RFL coordinator, an ombudsperson under
the operator's accountability framework) runs
a bulk audit. The export is gated on the
auditor's declared scope.

## §11 Webhook Endpoint for Tracing Outcomes

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A partner agency or a designated relative
registers a webhook to receive a push
notification when a tracing request reaches
a `positive-identification` outcome.

## §12 Schema-Validation and Conformance

The OpenAPI 3.1 document at `/v1/openapi.json`
carries JSON Schema 2020-12 schemas. The
schemas embed the UNHCR ProGres v4 sub-schema
and the HXL Standard sub-schema where
applicable.

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation.
Each vector references the relevant ICRC RFL,
UNHCR Personal Data Policy, IOM DTM, HXL
Standard, or 1989 CRC clause.

## §13 Multi-Language Support

```
GET /v1/person-records/{personId}?lang={lang}
Accept: application/json
```

The operator's API supports per-record content
in multiple languages. The `lang` parameter
selects the preferred language for the
displayed labels and the per-vulnerability
note narrative; structural fields (codes,
dates, identifiers) are language-neutral.

## §14 Data-Minimisation Discipline at Read Time

The retrieval endpoints honour the data-
minimisation principle of GDPR Article 5(1)(c):
the response carries the minimum subset of
fields necessary for the requesting agent's
declared purpose. The operator's API enforces
purpose-binding through the bearer token's
declared purpose claim.
