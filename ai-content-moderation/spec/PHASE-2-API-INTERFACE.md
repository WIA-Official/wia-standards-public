# WIA-ai-content-moderation PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-ai-content-moderation
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an
AI-content-moderation operator exposes for the records
defined in PHASE-1. Consumers include trusted-flagger
civil-society organisations, content-provenance
authentication services, regulators (EU DSA Coordinators,
US FTC equivalents, UK Ofcom under the Online Safety Act,
KR KCC), out-of-court dispute-resolution bodies, and the
operator's own audit and analytics platforms.

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
- C2PA Content Credentials
- EU DSA + EU AI Act 2024
- NIST AI RMF + AI 600-1 GenAI Profile

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The
OpenAPI 3.1 document at `/v1/openapi.json` is canonical.

This API is the operator-facing facade for moderation
records, transparency reports, and statutory escalations.
It does NOT expose end-user content distribution APIs;
those are operated by the platform's product surface.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-ai-content-moderation",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":             "/v1/programmes",
    "policyVersions":         "/v1/policy-versions",
    "contentItems":           "/v1/content-items",
    "classifierOutputs":      "/v1/classifier-outputs",
    "reviewerDecisions":      "/v1/reviewer-decisions",
    "appeals":                "/v1/appeals",
    "statutoryEscalations":   "/v1/statutory-escalations",
    "transparencyReports":    "/v1/transparency-reports",
    "evidence":               "/v1/evidence",
    "openapi":                "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes              — register a programme
GET    /v1/programmes/{pid}        — retrieve programme
PATCH  /v1/programmes/{pid}/status — advance status
PATCH  /v1/programmes/{pid}/vlop-designation
                                    — record VLOP
                                       designation per EU
                                       DSA Article 33
```

Programmes that are VLOP-designated must register the
risk-assessment cadence per DSA Article 34/35; submissions
without the cadence return `409` with type
`urn:wia:ai-content-moderation:vlop-risk-assessment-required`.

## §4 Policy Versions

```
POST   /v1/programmes/{pid}/policy-versions
                                          — register a
                                              policy version
PATCH  /v1/policy-versions/{pvid}/superseded-by
                                          — record successor
GET    /v1/policy-versions/{pvid}        — retrieve policy
                                              metadata
GET    /v1/policy-versions/{pvid}/content
                                          — fetch policy
                                              text in the
                                              operator's
                                              supported
                                              languages
```

Policy-version submissions require an `approvedBy`
reference (operator's policy-governance committee
approval).

## §5 Content Items

```
POST   /v1/programmes/{pid}/content-items — register a
                                              content item
                                              for moderation
GET    /v1/content-items/{cid}            — retrieve item
                                              metadata
GET    /v1/content-items/{cid}/c2pa-manifest
                                          — retrieve C2PA
                                              manifest if
                                              present
```

Content submissions whose `contentDigest` does not match
the artefact bytes return `422` with type
`urn:wia:ai-content-moderation:content-digest-mismatch`.

## §6 Classifier Outputs

```
POST   /v1/content-items/{cid}/classifier-outputs
                                          — register
                                              classifier
                                              output for an
                                              item
GET    /v1/classifier-outputs/{coid}     — retrieve output
GET    /v1/content-items/{cid}/classifier-outputs
                                          — list outputs
                                              (multiple
                                              classifier
                                              versions
                                              retained for
                                              audit)
```

Classifier-output submissions whose `recommendedAction`
is `remove-and-escalate-statutory` automatically queue the
content for the statutory-escalation workflow per §8.

## §7 Reviewer Decisions

```
POST   /v1/content-items/{cid}/reviewer-decisions
                                          — register human
                                              reviewer
                                              decision
GET    /v1/reviewer-decisions/{did}      — retrieve decision
PATCH  /v1/reviewer-decisions/{did}/wellness-flag
                                          — update reviewer
                                              wellness flag
                                              (operator's
                                              moderator-
                                              wellness
                                              pipeline
                                              integration)
```

Decisions cite the policy version (PHASE-1 §3) under
which the decision is grounded; submissions citing a
superseded policy return `409` with type
`urn:wia:ai-content-moderation:policy-version-superseded`.

## §8 Appeals

```
POST   /v1/content-items/{cid}/appeals    — file an appeal
                                              (per DSA
                                              Article 20
                                              internal
                                              complaint-
                                              handling)
PATCH  /v1/appeals/{aid}/resolution      — record resolution
PATCH  /v1/appeals/{aid}/escalation      — escalate to
                                              out-of-court
                                              dispute body
GET    /v1/appeals/{aid}                 — retrieve appeal
```

Appeals filed beyond the operator's published per-decision
appeal window return `409` with type
`urn:wia:ai-content-moderation:appeal-window-elapsed`.

## §9 Statutory Escalations

```
POST   /v1/content-items/{cid}/statutory-escalations
                                          — register a
                                              statutory
                                              escalation
PATCH  /v1/statutory-escalations/{seid}/acknowledgement
                                          — record authority
                                              acknowledgement
GET    /v1/statutory-escalations/{seid}  — retrieve
                                              escalation
                                              record
```

CSAM escalations (`escalationKind=csam-mandatory-report`)
follow the operator's PHASE-3 §8 statutory discipline;
the API enforces immediate escalation routing without
operator-discretionary delay.

## §10 Transparency Reports

```
POST   /v1/programmes/{pid}/transparency-reports
                                          — register a
                                              report
GET    /v1/transparency-reports/{trid}   — retrieve report
                                              metadata
GET    /v1/transparency-reports/{trid}/content
                                          — fetch report
                                              artefact (
                                              public — no
                                              client cert
                                              required for
                                              EU DSA Article
                                              15 reports)
```

Reports of regime `EU-DSA-Article-15` follow the EU DSA
transparency-report template; submissions citing the
regime that omit required template fields return `422`
with type
`urn:wia:ai-content-moderation:dsa-template-incomplete`.

## §11 Errors, Authentication, Caching, Audit

Errors: `application/problem+json` per RFC 9457 with the
types named above plus
`urn:wia:ai-content-moderation:evidence-mismatch`.
Authentication: mutually-authenticated TLS for trusted-
flagger, regulator, and partner consumers; transparency
reports are public read-only. Caching: stable resources
(superseded policy versions, resolved appeals,
acknowledged statutory escalations, signed evidence
packages) cacheable with `Cache-Control: max-age=
31536000, immutable`. Audit logs carry `programmeId`,
`contentId`, `traceId`, the issuing client certificate's
subject, and the operator's clock skew vs the operating
jurisdiction's NTP service.

## §12 Streaming Subscription, Bulk, Pagination, Provenance

SSE at `/v1/programmes/{pid}/events` for programme-wide
events (policy revision, statutory-escalation acknowledged,
transparency report published) and
`/v1/content-items/{cid}/events` for content-scoped events.
Subscribers reconnect via `Last-Event-ID`. Bulk endpoints:
`/v1/bulk/content-items`, `/v1/bulk/classifier-outputs`,
`/v1/bulk/reviewer-decisions`. Cursor-based pagination
via `cursor` and `Link` headers. Provenance via
`/v1/provenance/{recordId}`.

## §13 Worked Example: Submission to Statutory Escalation

1. Platform user submits content; the operator registers
   the content item.
2. Classifier produces output: confidence above CSAM
   indicator threshold triggers `remove-and-escalate-
   statutory` recommended action.
3. Content is removed from distribution and queued for
   the operator's statutory-escalation workflow.
4. Statutory escalation registers per §9 with
   `escalationKind=csam-mandatory-report` to NCMEC
   CyberTipline; acknowledgement received within the
   authority's published SLA.
5. Quarterly transparency report aggregates CSAM
   escalation counts (no per-content disclosure to
   protect investigation integrity).

## §14 Hash-Database Submission Endpoint

```
POST   /v1/content-items/{cid}/hash-submissions
                                          — submit content
                                            hash to a hash-
                                            sharing database
                                            (GIFCT, PhotoDNA,
                                            StopNCII, etc.)
GET    /v1/hash-submissions/{hsid}        — retrieve
                                            submission record
GET    /v1/programmes/{pid}/hash-submissions?
       database={d}                         — query submissions
                                            by database
```

Hash submissions to CSAM databases (PhotoDNA-NCMEC) follow
the operator's PHASE-3 §7 statutory discipline; the API
routes the submission through the operator's CSAM
escalation team, not through general moderation pipelines.

## §15 Aggregate and Provenance Endpoints

```
GET    /v1/provenance/{recordId}    — provenance entry for
                                       any PHASE-1 record
GET    /v1/aggregate/moderation-volume?period=...&kind=...
GET    /v1/aggregate/appeal-reversal-rate?period=...
GET    /v1/aggregate/classifier-accuracy?model=...&period=...
```

## §16 Conformance

A conformant server passes the test vectors published
under `tests/phase-vectors/phase-2-api-interface/`, emits
an OpenAPI 3.1 document, signs evidence packages per RFC
9421, and rejects superseded-policy citations in reviewer
decisions.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-ai-content-moderation
- **Last Updated:** 2026-04-28
