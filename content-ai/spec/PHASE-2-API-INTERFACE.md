# WIA-content-ai PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-content-ai
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that a
content-platform operator (social-media platform,
video-sharing platform, news publisher, search engine,
chat / messaging service, AI-content studio,
aggregator) exposes for the records defined in
PHASE-1. The contract is consumed by the operator's
trust-and-safety function, the user-facing notice-
and-action surface, the trusted-flagger network, the
out-of-court dispute settlement bodies, the
supervisory authority's examination tooling, the
operator's compliance-and-audit function, and the
public-disclosure surface for transparency reports.

References (CITATION-POLICY ALLOW only):

- C2PA Content Credentials specification v1.4
- ETSI TS 104 224 (AI in media)
- IPTC Photo Metadata 2024
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web
  Linking), RFC 8259 (JSON), RFC 9421 (HTTP Message
  Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022, ISO/IEC 42001:2023
- W3C Trace Context, W3C Verifiable Credentials Data
  Model 2.0
- EU AI Act Articles 50, 51 to 55
- EU DSA Articles 14, 16, 17, 20, 21, 22, 24, 25,
  26, 27, 28, 33 to 43

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by
the operator. Versioning uses `/v1/` path segments.
The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface. The C2PA Content
Credentials manifest schema and the IPTC Photo
Metadata 2024 schema are canonical for their
respective embedded-metadata surfaces.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-content-ai",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "contentTranscripts":      "/v1/content-transcripts",
    "contentCredentials":      "/v1/content-credentials",
    "disclosures":             "/v1/disclosures",
    "moderationDecisions":     "/v1/moderation-decisions",
    "notices":                 "/v1/notices",
    "appeals":                 "/v1/appeals",
    "trustedFlaggers":         "/v1/trusted-flaggers",
    "transparencyReports":     "/v1/transparency-reports",
    "childSafety":             "/v1/child-safety",
    "accessibility":           "/v1/accessibility",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 Content Transcript Endpoints

```
GET    /v1/content-transcripts?content={contentId}
GET    /v1/content-transcripts/{transcriptId}
POST   /v1/content-transcripts
```

The transcript endpoint records the AI-assistance
provenance for content the operator publishes; the
record is used in audit and in the response to user
queries about how content was produced.

## §4 C2PA Content Credentials Endpoints

```
POST   /v1/content-credentials/issue
                              (issue a manifest for
                               an AI-generated /
                               modified content
                               asset)
POST   /v1/content-credentials/verify
                              (verify a third-party
                               manifest)
GET    /v1/content-credentials/{credentialId}
GET    /v1/content-credentials?content={contentId}
```

Manifest issuance follows C2PA v1.4 — the manifest
carries the producer claim, AI-generated declaration,
the model reference (where applicable), the parent
manifest reference (for derived content), and the
operator's signature. Verification returns the
manifest's chain-of-trust evaluation against the
C2PA trust list.

## §5 Synthetic / Deepfake Disclosure Endpoints

```
GET    /v1/disclosures?content={contentId}
POST   /v1/disclosures
GET    /v1/disclosures/{disclosureId}
```

The disclosure endpoint records the EU AI Act
Article 50(1) / 50(2) / 50(3) / 50(4) transparency-
disclosure that accompanies the affected content.
The visual-or-auditory presentation reference is
retained for examination.

## §6 Moderation Decision Endpoints (DSA Article 17)

```
GET    /v1/moderation-decisions?content={contentId}
GET    /v1/moderation-decisions/{decisionId}
POST   /v1/moderation-decisions
GET    /v1/moderation-decisions/{decisionId}/statement-of-reasons
       (the DSA Article 17 statement of reasons,
        delivered to the affected user)
```

The DSA Article 17 statement-of-reasons format
follows the DSA Transparency Database (DSA-TDB)
schema published by the European Commission; the
operator's submissions to the DSA-TDB are emitted
on the operator's published cadence (real-time for
VLOPs / VLOSEs).

## §7 Notice-and-Action Endpoints (DSA Article 16)

```
POST   /v1/notices                     (user or
                                        trusted-
                                        flagger
                                        submits a
                                        notice)
GET    /v1/notices/{noticeId}
PATCH  /v1/notices/{noticeId}/decision (operator
                                        records the
                                        decision)
GET    /v1/notices?from={iso}&to={iso}
```

## §8 Appeal and Out-of-Court Dispute Endpoints
       (DSA Articles 20 + 21)

```
POST   /v1/appeals                  (user lodges an
                                     appeal)
GET    /v1/appeals/{appealId}
PATCH  /v1/appeals/{appealId}/internal-review
                                    (record internal-
                                     review outcome)
POST   /v1/appeals/{appealId}/escalate-to-oods
                                    (escalate to a
                                     certified out-of-
                                     court dispute
                                     settlement body)
```

## §9 Trusted-Flagger Endpoints (DSA Article 22)

```
GET    /v1/trusted-flaggers
POST   /v1/trusted-flaggers           (register a
                                       certified
                                       trusted
                                       flagger)
PATCH  /v1/trusted-flaggers/{flaggerId}/suspend
                                      (suspend per
                                       DSA Article
                                       23)
```

## §10 Transparency-Report Endpoints (DSA Article 24)

```
GET    /v1/transparency-reports
POST   /v1/transparency-reports
GET    /v1/transparency-reports/{reportId}
GET    /v1/transparency-reports/{reportId}/public-archive
```

## §11 Child-Safety and Accessibility Endpoints

```
GET    /v1/child-safety/age-assurance
POST   /v1/child-safety/age-assurance
GET    /v1/accessibility/conformance-report
POST   /v1/accessibility/conformance-report
```

## §12 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/moderation-decisions
GET    /v1/examination/notices
GET    /v1/examination/appeals
GET    /v1/examination/transparency-reports
GET    /v1/examination/audit-events
GET    /v1/examination/risk-assessment        (DSA
                                              Article
                                              34
                                              VLOP /
                                              VLOSE
                                              risk
                                              assessment)
GET    /v1/examination/independent-audit-report
                                              (DSA
                                              Article
                                              37 VLOP
                                              independent
                                              audit
                                              report)
```

The examination scope is read-only and bound to the
authority's identity (Member-State Digital Services
Coordinator and the Commission for VLOP / VLOSE
operators in EU; FTC for COPPA / CalOPPA / UDAAP
enforcement in US; KCC + KR PIPC for KR-jurisdiction
operators; UK Ofcom for UK Online Safety Act).

## §13 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1; per-surface
audiences distinguish user-facing, trust-and-safety,
trusted-flagger, OODS, and examination scopes. The
operator's trust-and-safety operations role accesses
moderation decisions; the user-facing scope is
limited to the user's own content and decisions.

## §14 HTTP Status Codes

Standard codes apply (200 / 201 / 202 / 400 / 401 /
403 / 404 / 409 / 422 / 429 / 503) with Problem
Details bodies. Decisions delivered to affected users
are accompanied by `Cache-Control: no-store` so that
private user data does not enter shared caches.

## §15 Webhook and Event Surface

The operator publishes lifecycle events through a
webhook channel:

- `content.published`, `content.modified`,
  `content.removed`.
- `content-credentials.issued`,
  `content-credentials.verified`.
- `moderation-decision.recorded`.
- `notice.received`, `notice.resolved`.
- `appeal.lodged`, `appeal.decided`.
- `trusted-flagger.registered`,
  `trusted-flagger.suspended`.
- `transparency-report.published`.

Webhook signatures use HTTP Message Signatures
(RFC 9421).

## §16 Bulk-Export and DSA Vetted-Researcher Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/status
GET    /v1/vetted-researcher/datasets   (DSA Article
                                         40 vetted-
                                         researcher
                                         dataset
                                         catalogue)
POST   /v1/vetted-researcher/access-request
```

Bulk exports support the supervisory authority's
data calls (DSA Article 73 Commission requests for
information; DSC examination data calls). The
vetted-researcher surface implements the DSA Article
40 access-to-data programme, sharing data with
researchers vetted by the operator's home Member-
State DSC.

## §17 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the content-credentials
issuance and verification surface, expose the DSA
Article 14-28 surfaces where the operator is in
scope, expose the supervisory examination surface,
and propagate trace-context across the publish-to-
moderation chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-content-ai
- **Last Updated:** 2026-04-28
