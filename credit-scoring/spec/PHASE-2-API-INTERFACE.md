# WIA-credit-scoring PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-credit-scoring
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a
credit-scoring operator (lender, credit-bureau, or
scoring provider) exposes for the records defined in
PHASE-1. The contract is consumed by the operator's
underwriting front-end, the credit-bureau-supplied
data feed, the consumer-facing portal, the supervisory
authority's examination tooling, and the operator's
model-governance and monitoring systems.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web Linking),
  RFC 8259 (JSON), RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022, ISO/IEC 27701:2019
- W3C Trace Context
- US ECOA Regulation B (12 CFR Part 1002), FCRA
  Regulation V (12 CFR Part 1022), TILA Regulation Z
  (12 CFR Part 1026)
- EU CCD recast (Directive (EU) 2023/2225) Articles
  6, 9, 18, 26
- EU AI Act 2024 Annex III §5(b)
- EU GDPR Articles 12 to 22
- KR Credit Information Use and Protection Act, KR
  금융소비자보호법
- ISO/IEC 42001:2023 AI management system

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The
OpenAPI 3.1 document at `/v1/openapi.json` is canonical.

The API is the operator-facing facade for the credit-
scoring lifecycle. End-user channels through which the
consumer applies for credit, receives adverse-action
notices, exercises FCRA dispute rights, or exercises
GDPR Article 22(3) right-to-explanation are served by
the operator's product surface and by the consumer-
portal subset of this API.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-credit-scoring",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":               "/v1/programmes",
    "consumers":                "/v1/consumers",
    "tradelines":               "/v1/tradelines",
    "featureVectors":           "/v1/feature-vectors",
    "models":                   "/v1/models",
    "scores":                   "/v1/scores",
    "creditworthinessAssessments": "/v1/creditworthiness-assessments",
    "adverseActionNotices":     "/v1/adverse-action-notices",
    "consentRecords":           "/v1/consent-records",
    "consumerPortal":           "/v1/consumer-portal",
    "openapi":                  "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
GET    /v1/programmes
GET    /v1/programmes/{programmeId}
POST   /v1/programmes
PATCH  /v1/programmes/{programmeId}
```

Programme creation requires the operator's model-risk
committee approval reference; transitions between
`design`, `validated`, `operating`, and `wind-down` are
audited.

## §4 Consumer and Tradeline Endpoints

```
GET    /v1/consumers/{consumerId}
POST   /v1/consumers
GET    /v1/tradelines?consumer={consumerId}
GET    /v1/tradelines/{tradelineId}
POST   /v1/tradelines/{tradelineId}/disputes  (FCRA
                                              1681i)
```

The dispute endpoint records the consumer's FCRA
dispute and triggers the operator's furnisher-
investigation workflow per FCRA 15 USC 1681i. The
operator MUST complete the investigation within the
30-day window (15 USC 1681i(a)(1)(A)) extended by 15
days where the consumer supplies relevant additional
information.

## §5 Feature-Vector and Score Endpoints

```
POST   /v1/feature-vectors          (compute features
                                     for a consumer)
GET    /v1/feature-vectors/{vectorId}
POST   /v1/scores                   (compute a score
                                     from a feature
                                     vector)
GET    /v1/scores/{scoreId}
```

The score response carries the score value, the score
band, the principal reason codes, the model reference,
and a signed statement of the model version and the
time of computation; the consumer-portal endpoint
exposes the same response with additional plain-
language explanations satisfying GDPR Article 22(3)
and ECOA Reg B 12 CFR 1002.9.

## §6 Creditworthiness-Assessment Endpoints

```
POST   /v1/creditworthiness-assessments
GET    /v1/creditworthiness-assessments/{assessmentId}
PATCH  /v1/creditworthiness-assessments/{assessmentId}
        (manual-review override; carries the human
         reviewer reference)
```

The endpoint produces the decision, the score-and-
reason-code record, and — for declines — the adverse-
action notice. The lender's policy-decision point
applies the assessment basis declared in the request
(US ECOA / TILA, EU CCD / MCD, KR 금융소비자보호법
적합성, etc.).

## §7 Adverse-Action Notice Endpoints

```
GET    /v1/adverse-action-notices?consumer={consumerId}
GET    /v1/adverse-action-notices/{noticeId}
POST   /v1/adverse-action-notices
```

The notice is delivered through the consumer's chosen
channel (email, postal mail, in-portal); the operator
records the delivery time-stamp and the consumer's
acknowledgement. The notice carries the principal
reasons, the consumer's right to a free credit-report
copy under FCRA 15 USC 1681j(a), and the redress-
channel URI.

## §8 Consumer-Portal Endpoints

```
GET    /v1/consumer-portal/me/scores
GET    /v1/consumer-portal/me/adverse-action-notices
GET    /v1/consumer-portal/me/score-explanation/{scoreId}
GET    /v1/consumer-portal/me/credit-report-copy
POST   /v1/consumer-portal/me/disputes
POST   /v1/consumer-portal/me/article-22-3-review
```

The consumer's identity is bound to the bearer token
through the operator's IdP (consumer-side OAuth 2.1).
The Article 22(3) review endpoint records the
consumer's request for human review of the automated
decision; the operator's response SLA is documented in
the operator's GDPR Article 22 procedure.

## §9 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 with audiences
declared per surface (operator-facing, consumer-
portal, supervisory-authority examination). Scopes
follow the operator's role-and-permission catalogue
(`scoring:read`, `scoring:write`, `disputes:write`,
`adverse-action:read`, `examination:read`). The
supervisory-authority examination scope (US CFPB, EU
NCA, KR FSC / FSS) provides read-only access to the
inspection-record surface.

## §10 HTTP Status Codes

- `200 OK` — read or search success
- `201 Created` — create success (Location header
  carries the new resource URL)
- `204 No Content` — delete or successful PATCH
  response success
- `400 Bad Request` — malformed payload (Problem
  Details body)
- `401 Unauthorized` — missing or invalid bearer token
- `403 Forbidden` — discipline-rejection (the Problem
  Details references the rejecting discipline — fair-
  lending, model-risk, consent, or examination scope)
- `404 Not Found` — resource not registered with the
  operator's records
- `409 Conflict` — version-mismatch on update (`If-
  Match`)
- `422 Unprocessable Content` — validation failure
  with Problem Details issue details
- `429 Too Many Requests` — rate-limit exceeded
- `503 Service Unavailable` — downstream credit-
  bureau or model-server unavailable

## §11 Caching and Trace-Context

`ETag` carries the resource's version-id. Cacheable
PHI / NPI is cached only with `Cache-Control: private,
no-store` for the consumer-portal endpoints; FCRA
permissible-purpose caching restrictions apply. Trace-
context (`traceparent`) is propagated through the
operator's underwriting pipeline, the credit-bureau-
supplied data feed, the model-server, and the audit
log so that an end-to-end reconstruction of the
decision is possible.

## §12 Webhook and Event Surface

The operator publishes the credit-decision lifecycle
events through a webhook channel registered by the
LOS, the servicing system, and the consumer-portal:

- `score.computed` — a score has been computed for a
  consumer, carrying the score-record reference.
- `assessment.decided` — an assessment has reached a
  decision (approve, decline, counter-offer, manual-
  review).
- `notice.delivered` — an adverse-action notice has
  been delivered through the chosen channel.
- `dispute.received` — a consumer dispute has been
  recorded and forwarded to the relevant credit-
  bureau.
- `dispute.resolved` — a dispute investigation has
  closed; the resolution outcome (updated, no-change,
  consumer-supplied-information-incorporated) is
  carried.
- `article-22-3-review.received` — a GDPR Article
  22(3) review request has been recorded; the
  operator's human-review SLA begins.
- `article-22-3-review.resolved` — the human-review
  outcome has been delivered to the consumer.

Webhook signatures use HTTP Message Signatures (RFC
9421) so that the receiving system can verify that
the event originated from the operator. Retry
discipline follows the operator's published retry
budget for transient receiver failures.

## §13 Examination Surface

The operating jurisdiction's supervisory authority
exercises its examination authority through:

```
GET    /v1/examination/programmes
GET    /v1/examination/models
GET    /v1/examination/decisions?from={iso}&to={iso}
GET    /v1/examination/disparate-impact-reports?period={period}
GET    /v1/examination/audit-events?from={iso}&to={iso}
```

Examination-scope tokens are read-only and carry the
supervisory authority's identity binding. The CFPB
Examination Manual (US), the EU Member-State NCA
inspection authority, and the KR FSC / FSS oversight
discipline are all served from this surface; the
operator's response SLA is documented in the
operator's compliance procedure.

## §14 Bulk-Export and Reproducibility Surface

The operator exposes a bulk-export endpoint for
internal model-validation and external-audit reviewers:

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/status
GET    /v1/bulk-export/{exportId}/manifest
```

Bulk exports carry the model reference, the time-window,
and the score-decision sample. The export's manifest
declares the cryptographic digest of each NDJSON file
produced so that the receiving reviewer can verify the
export's integrity. Bulk exports are governed by the
operator's data-egress policy; PHI / NPI is protected
under HIPAA-equivalent disposal discipline.

## §15 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the consumer-portal
surface for the right-of-access and right-to-
explanation responses, expose the supervisory-
authority examination surface, and emit the audit-
event for every action against PHI / NPI.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-credit-scoring
- **Last Updated:** 2026-04-28
