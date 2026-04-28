# WIA-anti-money-laundering PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-anti-money-laundering
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an
obliged entity exposes for the records defined in
PHASE-1. Consumers include the obliged entity's
compliance officer / MLRO, the operating jurisdiction's
financial-intelligence unit (FIU), the operating
jurisdiction's AML/CFT supervisor, the obliged entity's
external auditors, and law enforcement under the
operating jurisdiction's information-sharing rules.

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
- ISO 17442 (LEI)
- W3C Trace Context
- FATF 40 Recommendations
- FATF Recommendation 16 ("Travel Rule") for cross-
  border wire transfers

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
obliged entity. Versioning uses `/v1/` path segments.
The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical.

This API is the obliged-entity-facing facade for AML
records. It does NOT expose end-user banking APIs;
those are operated by the obliged entity's product
surface. STR/SAR filings flow from this API through
the FIU's prescribed channel (e.g. FinCEN BSA E-Filing
for US, KoFIU's electronic reporting channel for KR,
the operating Member State's FIU portal for EU,
NCA-UKFIU for UK).

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-anti-money-laundering",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":               "/v1/programmes",
    "cddRecords":               "/v1/cdd-records",
    "sanctionsScreenings":      "/v1/sanctions-screenings",
    "transactions":             "/v1/transactions",
    "suspiciousReports":        "/v1/suspicious-reports",
    "ctrRecords":               "/v1/ctr-records",
    "correspondentBanking":     "/v1/correspondent-banking",
    "investigationCases":       "/v1/investigation-cases",
    "evidence":                 "/v1/evidence",
    "openapi":                  "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes              — register a
                                     programme
GET    /v1/programmes/{pid}        — retrieve programme
PATCH  /v1/programmes/{pid}/status — advance status
PATCH  /v1/programmes/{pid}/mlro   — record MLRO
                                     designation per
                                     FATF Recommendation
                                     18 / equivalent
                                     national rule
PATCH  /v1/programmes/{pid}/fiu-binding
                                   — record FIU binding
                                     for STR/SAR filings
```

Programmes operating in a jurisdiction without an MLRO
designation field return `409` with type
`urn:wia:anti-money-laundering:mlro-required`.

## §4 CDD Records

```
POST   /v1/programmes/{pid}/cdd-records
                                   — register a customer
                                     CDD record per
                                     FATF Recommendation
                                     10
PATCH  /v1/cdd-records/{cid}/risk-rating
                                   — update risk rating
                                     (typically on
                                     periodic refresh
                                     or event trigger)
PATCH  /v1/cdd-records/{cid}/beneficial-owners
                                   — update beneficial-
                                     ownership chain
GET    /v1/cdd-records/{cid}       — retrieve CDD record
GET    /v1/cdd-records/{cid}/edd-annotation
                                   — retrieve EDD
                                     annotation if
                                     applicable
```

CDD submissions whose `customerKind` is `legal-person`
or `trust` without a `beneficialOwners` array return
`422` with type
`urn:wia:anti-money-laundering:beneficial-ownership-
required`. CDD submissions whose `pepStatus` is non-
`not-pep` without an `eddAnnotation.seniorManagement
ApprovalRef` return `409` with type
`urn:wia:anti-money-laundering:pep-senior-management-
approval-required`.

## §5 Sanctions Screening

```
POST   /v1/cdd-records/{cid}/sanctions-screenings
                                   — register a customer
                                     sanctions screening
POST   /v1/transactions/{tid}/sanctions-screenings
                                   — register a
                                     transaction-level
                                     sanctions screening
PATCH  /v1/sanctions-screenings/{sid}/decision
                                   — record reviewer
                                     decision
GET    /v1/sanctions-screenings/{sid}
                                   — retrieve screening
```

Screenings with `matchKind=potential-match-pending-
review` outstanding longer than the obliged entity's
published review SLA emit a programme-level event so
that the compliance officer can re-prioritise the
queue.

## §6 Transactions

```
POST   /v1/programmes/{pid}/transactions
                                   — register a
                                     transaction
GET    /v1/transactions/{tid}      — retrieve
                                     transaction
GET    /v1/transactions/{tid}/travel-rule-fields
                                   — retrieve
                                     Recommendation 16
                                     wire-transfer
                                     information
                                     (originator and
                                     beneficiary fields)
GET    /v1/transactions/{tid}/case
                                   — retrieve linked
                                     case if applicable
```

Cross-border transactions above the operating
jurisdiction's de minimis threshold whose
`travelRuleFields` are incomplete return `422` with
type
`urn:wia:anti-money-laundering:travel-rule-fields-
incomplete`.

## §7 Suspicious-Transaction Reports (STR / SAR)

```
POST   /v1/programmes/{pid}/suspicious-reports
                                   — file an STR / SAR
PATCH  /v1/suspicious-reports/{rid}/fiu-acknowledgement
                                   — record FIU
                                     acknowledgement
GET    /v1/suspicious-reports/{rid}
                                   — retrieve report
```

STR / SAR submissions whose `tippingOffPrecaution` is
not true return `409` with type
`urn:wia:anti-money-laundering:tipping-off-precaution-
required`. STR / SAR records are gated to the
compliance officer / MLRO and external auditors; the
records are NEVER exposed to the customer or to
unrelated branches per FATF Recommendation 21 tipping-
off prohibition.

## §8 Currency-Transaction Reports (CTR)

```
POST   /v1/transactions/{tid}/ctr-records
                                   — file a CTR for a
                                     transaction above
                                     the operating
                                     jurisdiction's
                                     threshold
GET    /v1/ctr-records/{cid}       — retrieve CTR
```

CTRs for transactions structured to evade the
threshold (multiple deposits below threshold by the
same customer within a defined window, the
"structuring" pattern under US BSA 31 USC 5324) are
flagged by the obliged entity's transaction-monitoring
rules and routed through the STR / SAR endpoint
instead of the CTR endpoint.

## §9 Correspondent Banking

```
POST   /v1/programmes/{pid}/correspondent-banking
                                   — register a
                                     correspondent-
                                     banking
                                     relationship
PATCH  /v1/correspondent-banking/{rid}/respondent-aml-
attestation
                                   — record respondent
                                     AML attestation
                                     review
PATCH  /v1/correspondent-banking/{rid}/review
                                   — record periodic
                                     relationship
                                     review
GET    /v1/correspondent-banking/{rid}
                                   — retrieve
                                     relationship
```

Correspondent-banking submissions whose `shellBankPolicy`
is `respondent-may-permit-shell-use` return `409` with
type
`urn:wia:anti-money-laundering:shell-bank-correspondent-
prohibited`, mirroring FATF Recommendation 13.

## §10 Investigation Cases

```
POST   /v1/programmes/{pid}/investigation-cases
                                   — open an investigation
                                     case
PATCH  /v1/investigation-cases/{cid}/status
                                   — advance case status
PATCH  /v1/investigation-cases/{cid}/exit-decision
                                   — record customer-
                                     exit decision
GET    /v1/investigation-cases/{cid}
                                   — retrieve case
```

Cases closed with `closed-customer-exited` automatically
trigger downstream workflow (account closure,
relationship termination per the obliged entity's
contracted-out workflow, downstream notifications to
joint-account holders per the operating jurisdiction's
consumer-protection rules).

## §11 Errors, Authentication, Caching, Audit

Errors: `application/problem+json` per RFC 9457 with
the types named above plus
`urn:wia:anti-money-laundering:evidence-mismatch`.
Authentication: mutually-authenticated TLS for all
consumers; the customer's own access (Article 15 GDPR
right of access where applicable) is gated through the
operating jurisdiction's tipping-off discipline (some
records — open STR / SAR and ongoing investigations —
are not subject to disclosure even on Article 15
request, under Article 23 derogations). Caching:
stable resources (closed cases, acknowledged STR / SAR
filings, archived programmes) cacheable with
`Cache-Control: max-age=31536000, immutable`. Audit
logs carry `programmeId`, `customerId`, `transactionId`,
`traceId`, the issuing client certificate's subject,
and the obliged entity's clock skew vs the operating
jurisdiction's NTP service.

## §12 Streaming Subscription, Bulk, Pagination, Provenance

SSE at `/v1/programmes/{pid}/events` for programme-
wide events (sanctions match confirmed, transaction-
monitoring rule triggered, STR / SAR filed, FIU
acknowledgement received). Subscribers reconnect via
`Last-Event-ID`. Bulk endpoints: `/v1/bulk/cdd-records`,
`/v1/bulk/transactions`, `/v1/bulk/sanctions-
screenings`. Cursor-based pagination via `cursor` and
`Link` headers. Provenance via
`/v1/provenance/{recordId}` emits the in-toto
attestation chain for any record.

## §13 Worked Example: Travel-Rule Cross-Border Wire

1. Customer requests a cross-border wire above the
   operating jurisdiction's Travel-Rule de minimis
   threshold.
2. Obliged entity registers the transaction via
   `POST /transactions` with full `travelRuleFields`
   (originator + beneficiary name, account number /
   wallet address, physical address).
3. Sanctions screening runs against UN, EU, OFAC, and
   the operating jurisdiction's national list via
   `POST /sanctions-screenings`; on `confirmed-match-
   blocked` the transaction is held and the case is
   opened via `POST /investigation-cases`.
4. On `no-match`, the wire is dispatched on the
   chosen rail (SWIFT MT 103 or ISO 20022 pacs.008
   per the corridor's adoption status).
5. Transaction-monitoring rules retrospectively
   evaluate the transaction; if a rule fires, an
   investigation case opens.
6. If the case closes with `closed-str-filed`, an
   STR / SAR is filed via `POST /suspicious-reports`.

## §14 Politically-Exposed-Person Refresh Endpoint

```
POST   /v1/cdd-records/{cid}/pep-refresh
                                   — refresh PEP status
                                     against an
                                     operator-bound PEP
                                     data source (the
                                     obliged entity's
                                     contracted PEP
                                     screening
                                     provider)
GET    /v1/cdd-records/{cid}/pep-history
                                   — retrieve PEP-
                                     status change
                                     history (a
                                     domestic PEP that
                                     becomes a foreign
                                     PEP on relocation
                                     triggers reassessment)
```

## §15 Aggregate and Provenance Endpoints

```
GET    /v1/provenance/{recordId}
GET    /v1/aggregate/transaction-volume?period=...&rail=...
GET    /v1/aggregate/sanctions-match-volume?period=...&list=...
GET    /v1/aggregate/str-volume?period=...&trigger=...
```

## §16 Conformance

A conformant server passes the test vectors published
under `tests/phase-vectors/phase-2-api-interface/`,
emits an OpenAPI 3.1 document, signs evidence packages
per RFC 9421, refuses shell-bank correspondent
relationships, refuses incomplete Travel-Rule fields
above threshold, and refuses STR / SAR submissions
without the tipping-off precaution flag.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-anti-money-laundering
- **Last Updated:** 2026-04-28
