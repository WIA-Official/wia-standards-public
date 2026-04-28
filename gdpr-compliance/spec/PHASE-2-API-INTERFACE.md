# WIA-gdpr-compliance PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-gdpr-compliance
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a
controller (or, where applicable, a processor) exposes
for the records defined in PHASE-1. Consumers include
the controller's data-protection officer, the lead
supervisory authority and other supervisory authorities
under the one-stop-shop mechanism, the European Data
Protection Board, the data subjects exercising rights
under Articles 15 to 22, the controller's joint
controllers and processors, and the controller's
external auditors.

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
- ISO/IEC 27701:2019 (privacy information management)
- W3C Trace Context
- EU GDPR + UK GDPR
- EDPB Guidelines on transparency and on data subject
  rights

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
controller. Versioning uses `/v1/` path segments. The
OpenAPI 3.1 document at `/v1/openapi.json` is canonical.

This API is the controller-facing facade for GDPR
compliance records. End-user channels through which
data subjects exercise rights (web forms, email
correspondence, helpline scripts) are operated by the
controller's product surface; this API records and
mediates those interactions.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-gdpr-compliance",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":               "/v1/programmes",
    "processingActivities":     "/v1/processing-activities",
    "dataSubjectRequests":      "/v1/data-subject-requests",
    "consentRecords":           "/v1/consent-records",
    "internationalTransfers":   "/v1/international-transfers",
    "dpiaRecords":              "/v1/dpia-records",
    "breachNotifications":      "/v1/breach-notifications",
    "saCorrespondence":         "/v1/sa-correspondence",
    "controllerArrangements":   "/v1/controller-arrangements",
    "evidence":                 "/v1/evidence",
    "openapi":                  "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes              — register a programme
GET    /v1/programmes/{pid}        — retrieve programme
PATCH  /v1/programmes/{pid}/status — advance status
PATCH  /v1/programmes/{pid}/lead-supervisory-authority
                                   — record lead DPA
                                     designation per
                                     Article 56
PATCH  /v1/programmes/{pid}/representative
                                   — record Article 27
                                     EU representative
```

Programmes that declare cross-border processing without
a `leadSupervisoryAuthority` field return `409` with type
`urn:wia:gdpr-compliance:lead-sa-required`. Programmes
processing in the EU from outside the EU without a
`representative` field return `409` with type
`urn:wia:gdpr-compliance:art-27-representative-required`.

## §4 Records of Processing Activities

```
POST   /v1/programmes/{pid}/processing-activities
                                   — register a record
                                     of processing
                                     activity per
                                     Article 30
PATCH  /v1/processing-activities/{aid}
                                   — amend a record
GET    /v1/processing-activities/{aid}
                                   — retrieve a record
GET    /v1/programmes/{pid}/processing-activities
                                   — list records (the
                                     controller's
                                     Article 30(4)
                                     register that the
                                     supervisory
                                     authority can
                                     request)
```

Activity submissions whose `legalBasis` is
`art-6-1-a-consent` and whose `personalDataCategory`
matches the controller's special-category data
dictionary without a `specialCategoryBasis` field return
`409` with type
`urn:wia:gdpr-compliance:art-9-basis-required`.

## §5 Data Subject Requests

```
POST   /v1/programmes/{pid}/data-subject-requests
                                   — register a request
                                     under Articles 15
                                     to 22
PATCH  /v1/data-subject-requests/{rid}/extension
                                   — declare an Article
                                     12(3) two-month
                                     extension with
                                     documented reason
PATCH  /v1/data-subject-requests/{rid}/response
                                   — record the
                                     controller's
                                     response
GET    /v1/data-subject-requests/{rid}
                                   — retrieve request
GET    /v1/data-subject-requests/{rid}/response-artefact
                                   — fetch the response
                                     payload (Article 15
                                     access export,
                                     Article 20
                                     portability export)
```

Responses cite the policy version under which the
request is processed; responses delivered after the
Article 12(3) deadline without an extension entry
return `409` on `PATCH /response` with type
`urn:wia:gdpr-compliance:art-12-3-deadline-elapsed-no-
extension`.

## §6 Consent Records

```
POST   /v1/programmes/{pid}/consent-records
                                   — register consent
                                     capture
PATCH  /v1/consent-records/{cid}/withdrawal
                                   — record withdrawal
                                     per Article 7(3)
GET    /v1/consent-records/{cid}   — retrieve consent
GET    /v1/programmes/{pid}/consent-records?dataSubject={dsid}
                                   — query by data
                                     subject
```

Consent submissions whose `granularPurposes` array is
empty (i.e. bundled consent) return `422` with type
`urn:wia:gdpr-compliance:bundled-consent-prohibited`,
mirroring the EDPB consent guidelines' prohibition on
bundled consent.

## §7 International Transfers

```
POST   /v1/programmes/{pid}/international-transfers
                                   — register a Chapter
                                     V transfer
PATCH  /v1/international-transfers/{tid}/transfer-impact-assessment
                                   — record TIA per
                                     EDPB Recommendations
GET    /v1/international-transfers/{tid}
                                   — retrieve transfer
GET    /v1/international-transfers/{tid}/scc-instrument
                                   — fetch the executed
                                     SCC instrument (
                                     Commission
                                     Implementing
                                     Decision (EU)
                                     2021/914 module)
```

Transfer submissions citing `art-46-2-c-scc-*` modules
without a TIA reference return `409` with type
`urn:wia:gdpr-compliance:tia-required-for-scc-module`.

## §8 DPIA Records

```
POST   /v1/processing-activities/{aid}/dpia-records
                                   — register a DPIA per
                                     Article 35
PATCH  /v1/dpia-records/{did}/prior-consultation
                                   — record Article 36
                                     prior consultation
                                     correspondence
                                     reference
GET    /v1/dpia-records/{did}      — retrieve DPIA
```

DPIA submissions whose `triggeringFactor` is
`art-35-3-b-large-scale-special-category` without a
`dpoConsultedAt` field return `409` with type
`urn:wia:gdpr-compliance:art-35-2-dpo-consultation-
missing`.

## §9 Breach Notifications

```
POST   /v1/programmes/{pid}/breach-notifications
                                   — register a breach
PATCH  /v1/breach-notifications/{bid}/sa-notification
                                   — record Article 33
                                     supervisory-
                                     authority
                                     notification
PATCH  /v1/breach-notifications/{bid}/data-subject-notification
                                   — record Article 34
                                     data-subject
                                     notification
PATCH  /v1/breach-notifications/{bid}/remediation
                                   — record post-
                                     incident remediation
GET    /v1/breach-notifications/{bid}
                                   — retrieve breach
                                     record
```

Breach notifications submitted with an `awareAt` more
than 72 hours before `saNotifiedAt` and without an
`saNotificationDelayReason` return `409` with type
`urn:wia:gdpr-compliance:art-33-1-72h-breach-no-reason`.

## §10 Supervisory-Authority Correspondence

```
POST   /v1/programmes/{pid}/sa-correspondence
                                   — register a piece of
                                     correspondence
PATCH  /v1/sa-correspondence/{cid}/response
                                   — record the
                                     controller's
                                     response
PATCH  /v1/sa-correspondence/{cid}/outcome
                                   — record the
                                     supervisory
                                     authority's
                                     outcome under
                                     Article 58(2)
GET    /v1/sa-correspondence/{cid}
                                   — retrieve
                                     correspondence
```

Article 60 one-stop-shop correspondence is mirrored to
the lead supervisory authority's identifier; mirror
failures return `502` with type
`urn:wia:gdpr-compliance:art-60-mirror-failure`.

## §11 Errors, Authentication, Caching, Audit

Errors: `application/problem+json` per RFC 9457 with the
types named above plus
`urn:wia:gdpr-compliance:evidence-mismatch`.
Authentication: mutually-authenticated TLS for
supervisory-authority, DPO, joint-controller, processor,
and auditor consumers; the data subject's Article 15
access-export endpoint requires the controller's
identity-verification flow before delivery.
Caching: stable resources (closed sa correspondence,
fulfilled data subject requests, superseded consent
records, archived programmes) cacheable with
`Cache-Control: max-age=31536000, immutable`.
Audit logs carry `programmeId`, `requestId`, `traceId`,
the issuing client certificate's subject, and the
controller's clock skew vs the operating jurisdiction's
NTP service.

## §12 Streaming Subscription, Bulk, Pagination, Provenance

SSE at `/v1/programmes/{pid}/events` for programme-wide
events (breach declared, sa correspondence opened,
data subject request received, transfer mechanism
suspended). Subscribers reconnect via `Last-Event-ID`.
Bulk endpoints: `/v1/bulk/processing-activities`,
`/v1/bulk/consent-records`, `/v1/bulk/data-subject-
requests`. Cursor-based pagination via `cursor` and
`Link` headers. Provenance via
`/v1/provenance/{recordId}` emits the in-toto
attestation chain for any record.

## §13 Worked Example: Article 33 Breach Notification

1. Controller's incident-response team detects
   suspicious access to a customer database at
   `detectedAt`.
2. Investigation confirms unauthorised access at
   `awareAt`; the Article 33 72-hour clock starts.
3. Controller registers the breach via
   `POST /breach-notifications`.
4. Within 72 hours, controller files the supervisory-
   authority notification via
   `PATCH /sa-notification` with the affected-data-
   subject count and breach kind.
5. Where Article 34 high-risk threshold met, controller
   files the data-subject notification via
   `PATCH /data-subject-notification` and updates
   transparency materials.
6. Lessons-learned narrative is filed via
   `PATCH /remediation` and feeds the controller's
   Article 32 security-measure review cycle.

## §14 Article 22 Automated-Decision Review Endpoint

```
POST   /v1/data-subject-requests/{rid}/art-22-review
                                   — register review of
                                     an automated
                                     decision per
                                     Article 22(3)
GET    /v1/data-subject-requests/{rid}/art-22-review
                                   — retrieve review
                                     outcome
```

Article 22 reviews record the human-reviewer identifier,
the new decision, and the rationale narrative reference;
the data subject's right to express their point of view
and to contest the decision is preserved by the
controller's review discipline.

## §15 Aggregate and Provenance Endpoints

```
GET    /v1/provenance/{recordId}    — provenance entry for
                                       any PHASE-1 record
GET    /v1/aggregate/data-subject-request-volume?period=...&kind=...
GET    /v1/aggregate/breach-volume?period=...&kind=...
GET    /v1/aggregate/consent-withdrawal-rate?period=...
```

## §16 Conformance

A conformant server passes the test vectors published
under `tests/phase-vectors/phase-2-api-interface/`,
emits an OpenAPI 3.1 document, signs evidence packages
per RFC 9421, refuses bundled consent submissions, and
enforces the Article 33 72-hour breach clock as a
machine-verifiable invariant rather than as a manual
discipline.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-gdpr-compliance
- **Last Updated:** 2026-04-28
