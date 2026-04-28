# WIA-digital-time-capsule PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-digital-time-capsule
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract
that a digital-preservation operator (national
archive, library preservation department,
research-data repository, IIPC web archive,
institutional repository, or personal-capsule
service) exposes for the records defined in
PHASE-1. The contract carries the OAIS
Information Package ingestion, PREMIS metadata
publication, format-identification ingestion,
BagIt transfer-package endpoint, web-archive
WARC ingestion, trustworthy-digital-repository
audit ingestion, capsule-seal-and-release
endpoint, and chain-of-custody anchoring
endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110, RFC 9111, RFC 9457, RFC 8288,
  RFC 6901 / 6902, RFC 8259, RFC 4122, RFC
  9421, RFC 8615
- IETF RFC 6234 (SHA-256), RFC 8032 (Ed25519),
  RFC 8493 (BagIt)
- W3C Trace Context, W3C ODRL 2.2, W3C VC Data
  Model v2.0
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015
- ISO 14721:2012 (OAIS), ISO 16363:2012, ISO
  16919:2020, ISO 14641:2018
- PREMIS Data Dictionary v3.0
- ISO 28500:2017 (WARC), ISO 19005-1/-2/-3/-4
  (PDF/A), ISO 32000-2:2020 (PDF 2.0)
- ISO 15836-1:2017 / 15836-2:2019 (Dublin Core)
- METS / MODS (Library of Congress)
- IIPC WARC and CDX format guidelines
- EU Regulation (EU) 910/2014 (eIDAS) and EU
  Regulation (EU) 2024/1183 (eIDAS-2)
- KR 공공기록물 관리에 관한 법률 / KR 전자문서법

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published
by the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical. Schema changes
follow the non-breaking conventions in PHASE-1
§2. Every endpoint carries a per-request
signature using HTTP Message Signatures (RFC
9421) anchored to the operator's institutional
identifier and / or the operator's qualified-
electronic-signature certificate under EU
eIDAS-2 where the package crosses an EU
jurisdiction; the signature key set is published
at
`/.well-known/wia/digital-time-capsule/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-digital-time-capsule",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "oaisPackages":       "/v1/oais-packages",
    "premisRecords":      "/v1/premis-records",
    "formatRecords":      "/v1/format-records",
    "bagitPackages":      "/v1/bagit-packages",
    "warcIngestion":      "/v1/warc-ingestion",
    "tdrAudits":          "/v1/tdr-audits",
    "capsuleSeals":       "/v1/capsule-seals",
    "custody":            "/v1/custody-events",
    "openapi":            "/v1/openapi.json",
    "wellKnown":          "/.well-known/wia/digital-time-capsule"
  }
}
```

## §3 OAIS Package Ingestion Endpoints

### §3.1 Submit a SIP

```
POST /v1/oais-packages
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature>
```

The multipart body carries one JSON part with
the §3 record from PHASE-1 (packageType,
contentInformation,
preservationDescriptionInformation,
packageDescription) and one or more file parts
carrying the data objects. The server validates
that the SIP carries the OAIS Mandatory
Properties — Reference Information, Provenance,
Context, Fixity, Access Rights — and refuses a
SIP that omits a Mandatory Property with `422
Unprocessable Entity` at `/problems/oais-
mandatory-property-missing`.

### §3.2 Generate an AIP

```
POST /v1/oais-packages/{packageId}/aip
Signature: <RFC 9421 signature from the
            preservation engineer>
```

The server transforms the SIP into an AIP per
the OAIS §4.1 transformation discipline. The
AIP carries the SIP's content together with
the per-event PREMIS Event records produced
during ingestion (validation, fixity-check,
format-identification, virus-check).

### §3.3 Generate a DIP

```
POST /v1/oais-packages/{packageId}/dip
Content-Type: application/json
```

The server transforms the AIP into a DIP per
the consumer's request envelope (a per-format
DIP for a researcher running an analytics
workflow, a presentation DIP for a public-
access viewer).

### §3.4 Retrieve a package

```
GET /v1/oais-packages/{packageId}
Accept: application/json
```

## §4 PREMIS Endpoints

### §4.1 Publish a PREMIS record

```
POST /v1/premis-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §4 record from PHASE-1
(objectEntity, eventEntity, agentEntity,
rightsEntity).

### §4.2 Retrieve a PREMIS record

```
GET /v1/premis-records/{premisId}
Accept: application/json | application/xml
```

The XML serialisation follows the PREMIS XML
schema published by the Library of Congress.

### §4.3 Append an event

```
POST /v1/premis-records/{premisId}/events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A per-event PREMIS Event record is appended to
the existing PREMIS record. The append is
recorded as a chain-of-custody event so that
the event-history is preserved.

## §5 Format Identification Endpoints

### §5.1 Submit a format-identification report

```
POST /v1/format-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §5 record from PHASE-1
(formatIdentifier, validationOutcome,
validationReport, preservationFormat). The
server validates the format identifier against
the Library of Congress Sustainability of
Digital Formats register and refuses an
unknown identifier.

### §5.2 Retrieve a format record

```
GET /v1/format-records/{formatRecordId}
Accept: application/json
```

## §6 BagIt Transfer Endpoints

### §6.1 Submit a BagIt package

```
POST /v1/bagit-packages
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature over the
            manifest digest>
```

The multipart body carries a `bag-info.txt` part,
a `manifest-{algorithm}.txt` part, a
`tagmanifest-{algorithm}.txt` part, and the
`data/` payload parts. The server validates the
manifest digest against each payload file and
refuses a package whose declared digest does
not match the recomputed digest (RFC 6234
SHA-256 or SHA-512).

### §6.2 Verify a BagIt package

```
GET /v1/bagit-packages/{bagitId}/verification
Accept: application/json
```

The endpoint returns the per-payload digest-
verification report.

## §7 WARC Ingestion Endpoints

### §7.1 Submit a WARC file

```
POST /v1/warc-ingestion
Content-Type: application/warc
Signature: <RFC 9421 signature>
```

The WARC file is ingested per ISO 28500:2017
WARC File Format. The server generates the
per-WARC CDX index per the IIPC CDX format
guideline.

### §7.2 Retrieve a WARC record

```
GET /v1/warc-ingestion/{warcId}/records/{warcRecordId}
Accept: application/warc
```

## §8 Trustworthy-Digital-Repository Audit Endpoints

### §8.1 Submit an audit report

```
POST /v1/tdr-audits
Content-Type: application/json
Signature: <RFC 9421 signature from the audit
            certification body>
```

Request body carries the §7 record from PHASE-1
(auditFramework, auditScope, auditOutcome,
auditBody). The server validates the audit
body's ISO 16919:2020 accreditation against
the issuing accreditation body's register.

### §8.2 Retrieve an audit report

```
GET /v1/tdr-audits/{auditId}
Accept: application/json
```

## §9 Capsule Seal and Release Endpoints

### §9.1 Seal a capsule

```
POST /v1/capsule-seals
Content-Type: application/json
Signature: <RFC 9421 signature from the
            depositor + a co-signer per the
            operator's seal policy>
```

Request body carries the capsule envelope —
the depositor identity, the named beneficiary
(individual, group, public, or "open at" date),
the declared opening date, the per-capsule
release-condition envelope, and the rights-
expression for the post-opening re-use. The
server records the seal event in the chain-of-
custody record.

### §9.2 Open a capsule

```
POST /v1/capsule-seals/{sealId}/open
Signature: <RFC 9421 signature from the
            named beneficiary or from the
            operator's date-trigger>
```

The server verifies the opening conditions:
- For a `time-trigger` capsule, the current
  date is on or after the declared opening
  date.
- For a `beneficiary-trigger` capsule, the
  named beneficiary's identity is verified
  against the W3C Verifiable Credential
  declared in the seal envelope.
- For an `event-trigger` capsule, the trigger
  event is verified against the operator's
  declared event-source.

A premature opening attempt returns `403
Forbidden` at `/problems/capsule-opening-
condition-not-met`.

## §10 Custody and Error Reporting

### §10.1 Anchor a custody event

```
POST /v1/custody-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

### §10.2 Error envelope

Errors are returned using RFC 9457 Problem
Details. Validation errors carry a `pointer`
field (RFC 6901). The server emits a per-
request `traceparent` header (W3C Trace
Context).

## §11 Concurrency and Cache Discipline

Every retrieval endpoint emits an `ETag` header
(RFC 9110 §8.8.3). Conditional requests
(`If-Match`, `If-None-Match`) are honoured.

## §12 Bulk Export for Audit Inspection

```
GET /v1/programmes/{programmeId}/aips:bulk
Accept: application/x-ndjson
Authorization: <bearer token from an audit
                 certification body>
```

An audit body running an ISO 16363 audit
ingests the operator's AIP register as a
newline-delimited JSON stream.

## §13 Webhook Endpoint for Capsule Trigger Notifications

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A named beneficiary or an event-trigger source
registers a webhook to receive a push
notification when a capsule's trigger
condition is met.

## §14 Schema-Validation and Conformance

The OpenAPI 3.1 document at `/v1/openapi.json`
carries JSON Schema 2020-12 schemas. The PREMIS
endpoints follow the PREMIS XML / JSON schemas.
The WARC endpoints follow ISO 28500:2017.

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation.
Each vector references the relevant OAIS,
PREMIS, ISO 16363, BagIt, or WARC clause.
