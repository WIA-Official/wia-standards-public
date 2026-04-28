# WIA-financial-inclusion PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-financial-inclusion
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a
financial-inclusion programme exposes for the records
defined in PHASE-1. Consumers include the programme
operator, the operating jurisdiction's financial-
inclusion strategy coordinator, the operating
jurisdiction's central bank or financial-inclusion
authority, the operating jurisdiction's consumer-
protection regulator, the operating jurisdiction's
AML/CFT supervisor, the operating jurisdiction's
alternative dispute-resolution body, civil-society
partner organisations, and the programme's external
evaluators.

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
- ISO 20022 (financial-services messaging)
- ISO 4217 (currency codes)
- W3C Trace Context
- World Bank UFA-2020 + Findex Database
- FATF 40 Recommendations
- EU PAD 2014/92/EU + EU PSD2 2015/2366 + EU CCD 2023/2225

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
programme. Versioning uses `/v1/` path segments. The
OpenAPI 3.1 document at `/v1/openapi.json` is canonical.

This API is the programme-facing facade for financial-
inclusion records. Customer-facing channels (mobile-
money apps, branch counters, mobile-money-agent
networks, web-based account-opening flows) flow
through the programme's product surface; this API
records the artefacts of regulator-and-funder reporting
significance.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-financial-inclusion",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":             "/v1/programmes",
    "basicAccountAccess":     "/v1/basic-account-access",
    "consumerDisclosures":    "/v1/consumer-disclosures",
    "feeSchedules":           "/v1/fee-schedules",
    "disputeResolutions":     "/v1/dispute-resolutions",
    "creditDecisions":        "/v1/credit-decisions",
    "programmeOutcomes":      "/v1/programme-outcomes",
    "educationActivities":    "/v1/education-activities",
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
PATCH  /v1/programmes/{pid}/national-strategy
                                   — record National
                                     Financial
                                     Inclusion
                                     Strategy
                                     reference
PATCH  /v1/programmes/{pid}/consumer-protection-authority
                                   — record consumer-
                                     protection
                                     authority binding
```

## §4 Basic-Account Access

```
POST   /v1/programmes/{pid}/basic-account-access
                                   — register a basic
                                     account opening
PATCH  /v1/basic-account-access/{aid}/upgrade-pathway
                                   — record customer
                                     upgrade to
                                     standard CDD
                                     when threshold
                                     exceeded
GET    /v1/basic-account-access/{aid}
                                   — retrieve record
GET    /v1/programmes/{pid}/basic-account-access?accountKind={k}
                                   — query by account
                                     kind
```

Account-access submissions whose `simplifiedCddBasis`
is non-`no-simplified-cdd-applied` without
`thresholdLimits` return `409` with type
`urn:wia:financial-inclusion:simplified-cdd-thresholds-
required`.

## §5 Consumer-Protection Disclosures

```
POST   /v1/programmes/{pid}/consumer-disclosures
                                   — register a
                                     consumer-
                                     protection
                                     disclosure
GET    /v1/consumer-disclosures/{did}
                                   — retrieve record
```

Disclosure submissions whose `disclosureKind` is
`eu-payment-accounts-directive-fee-information`
without an attached fee-schedule reference (PHASE-1
§5) return `422` with type
`urn:wia:financial-inclusion:eu-pad-fee-information-
attachment-required`.

## §6 Fee Schedules

```
POST   /v1/programmes/{pid}/fee-schedules
                                   — register a fee
                                     schedule
PATCH  /v1/fee-schedules/{fid}/superseded-by
                                   — record successor
GET    /v1/fee-schedules/{fid}     — retrieve schedule
GET    /v1/fee-schedules/{fid}/glossary
                                   — fetch the fee-
                                     glossary
                                     publication
GET    /v1/fee-schedules/{fid}/fee-information-document
                                   — fetch the EU PAD
                                     Fee Information
                                     Document
```

Fee schedules introducing a new fee category mid-
period without the operating jurisdiction's pre-
notification deadline observed (e.g. EU PSD2 Article
54(2) two-month change notice, US Reg DD § 1030.5
30-day notice) return `409` with type
`urn:wia:financial-inclusion:fee-change-notice-
deadline-required`.

## §7 Dispute Resolutions

```
POST   /v1/programmes/{pid}/dispute-resolutions
                                   — register a
                                     dispute filing
PATCH  /v1/dispute-resolutions/{did}/internal-resolution
                                   — record internal
                                     resolution
                                     decision
PATCH  /v1/dispute-resolutions/{did}/external-escalation
                                   — record external
                                     escalation to
                                     ADR body
GET    /v1/dispute-resolutions/{did}
                                   — retrieve dispute
```

Dispute filings that miss the operating jurisdiction's
internal resolution deadline (PHASE-1 §6
`internalResolutionDeadlineRef`) without resolution
return `409` on `PATCH /external-escalation` with
type
`urn:wia:financial-inclusion:internal-resolution-
deadline-elapsed-must-escalate`.

## §8 Credit Decisions

For programmes extending credit:

```
POST   /v1/programmes/{pid}/credit-decisions
                                   — register a credit
                                     decision
PATCH  /v1/credit-decisions/{cid}/adverse-action-notice
                                   — attach adverse-
                                     action notice
                                     reference
GET    /v1/credit-decisions/{cid}  — retrieve decision
```

Credit-decision submissions whose `decisionOutcome` is
`denied` or `counter-offer` without an
`adverseActionNoticeRef` return `409` with type
`urn:wia:financial-inclusion:adverse-action-notice-
required`. Decisions made without a fair-lending
framework reference return `422` with type
`urn:wia:financial-inclusion:fair-lending-framework-
required`.

## §9 Programme Outcomes

```
POST   /v1/programmes/{pid}/programme-outcomes
                                   — register a
                                     reporting-period
                                     outcome aggregate
GET    /v1/programme-outcomes/{oid}
                                   — retrieve outcome
GET    /v1/programmes/{pid}/programme-outcomes?period=...
                                   — query outcomes
                                     by period
```

Outcome aggregates carry only aggregate counts; per-
customer data does not appear in outcome payloads,
mirroring the operating jurisdiction's privacy
discipline for customer-level reporting.

## §10 Education Activities

```
POST   /v1/programmes/{pid}/education-activities
                                   — register a
                                     financial-literacy
                                     education
                                     activity
GET    /v1/education-activities/{eid}
                                   — retrieve activity
```

Education activities that cross-walk to WIA-digital-
citizenship records carry the cross-walk reference
in the curriculum-reference field.

## §11 Errors, Authentication, Caching, Audit

Errors: `application/problem+json` per RFC 9457 with
the types named above plus
`urn:wia:financial-inclusion:evidence-mismatch`.
Authentication: mutually-authenticated TLS for
financial-inclusion-strategy-coordinator, central-
bank, consumer-protection-regulator, AML/CFT
supervisor, ADR-body, civil-society-partner, and
external-evaluator consumers. Caching: stable
resources (superseded fee schedules, resolved
disputes, archived programmes) cacheable with
`Cache-Control: max-age=31536000, immutable`. Audit
logs carry `programmeId`, `customerId` (where
applicable, gated to authorised consumers), `traceId`,
the issuing client certificate's subject, and the
programme's clock skew vs the operating jurisdiction's
NTP service.

## §12 Streaming Subscription, Bulk, Pagination, Provenance

SSE at `/v1/programmes/{pid}/events` for programme-
wide events (basic-account opening, fee-schedule
revision, dispute filed, dispute escalated, credit-
decision-denial pattern flagged for fair-lending
review). Subscribers reconnect via `Last-Event-ID`.
Bulk endpoints: `/v1/bulk/basic-account-access`,
`/v1/bulk/credit-decisions`, `/v1/bulk/programme-
outcomes`. Cursor-based pagination via `cursor` and
`Link` headers. Provenance via
`/v1/provenance/{recordId}` emits the in-toto
attestation chain for any record.

## §13 Worked Example: Tier-1 Mobile-Money Onboarding

1. Mobile-money operator registers a programme with
   `operatorClass=mobile-money-operator` and
   governing-frameworks including the operating
   jurisdiction's mobile-money licensing regime and
   `FATF-Recs-with-Simplified-CDD`.
2. Customer presents national ID at a mobile-money
   agent; `POST /basic-account-access` registered
   with `accountKind=mobile-money-tier-1`,
   `simplifiedCddBasis=tier-based-mobile-money`,
   and the threshold limits the operating
   jurisdiction's regulator imposes on tier-1
   accounts (e.g. per-day, per-month aggregate, and
   per-balance caps).
3. Pre-account fee schedule and consumer-protection
   disclosure registered via `POST /consumer-
   disclosures`.
4. Customer transactions flow through the operator's
   product surface; transaction monitoring runs
   (cross-walked with WIA-anti-money-laundering).
5. When the customer's transactions approach the
   tier-1 thresholds, the operator triggers the
   upgrade pathway via `PATCH /basic-account-access/
   upgrade-pathway` to standard CDD (full KYC), at
   which point full WIA-anti-money-laundering CDD
   discipline applies.
6. Dispute filings flow through `POST /dispute-
   resolutions`; escalation to the operating
   jurisdiction's ADR body (e.g. central-bank
   consumer redress mechanism) via `PATCH /external-
   escalation`.

## §14 Aggregate and Provenance Endpoints

```
GET    /v1/provenance/{recordId}
GET    /v1/aggregate/new-account-volume?period=...&accountKind=...
GET    /v1/aggregate/active-account-rate?period=...
GET    /v1/aggregate/dispute-volume?period=...&kind=...
GET    /v1/aggregate/credit-decision-denial-rate?period=...
```

## §15 Conformance

A conformant server passes the test vectors published
under `tests/phase-vectors/phase-2-api-interface/`,
emits an OpenAPI 3.1 document, signs evidence packages
per RFC 9421, refuses simplified-CDD account openings
without threshold-limit declarations, refuses adverse-
action credit decisions without notice references,
and refuses fee-schedule revisions without observance
of the operating jurisdiction's pre-notification
deadline.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-financial-inclusion
- **Last Updated:** 2026-04-28
