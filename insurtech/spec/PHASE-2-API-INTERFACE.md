# WIA-insurtech PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-insurtech
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an accredited
insurtech operator exposes for the records defined in PHASE-1.
Consumers include brokers, distribution partners, reinsurers,
regulators (solvency, market-conduct, prudential), and the
operator's own consumer-facing portals and analytics platforms.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 4217 (currency codes)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- ACORD Reference Architecture (insurance industry data model)
- IFRS 17 (Insurance Contracts)

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operator.
Versioning uses `/v1/` path segments. The OpenAPI 3.1 document
at `/v1/openapi.json` is canonical.

ACORD XML envelopes are accepted at the legacy `/acord/` prefix
for partner integrations that have not yet migrated; the legacy
prefix is documented in the operator's broker-portal integration
guide and is sunset on the cadence the operator publishes.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-insurtech",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":           "/v1/programmes",
    "parties":              "/v1/parties",
    "riskObjects":          "/v1/risk-objects",
    "coverages":            "/v1/coverages",
    "policies":             "/v1/policies",
    "underwritingDecisions": "/v1/underwriting-decisions",
    "claims":               "/v1/claims",
    "cessions":             "/v1/cessions",
    "consumerDisclosures":  "/v1/consumer-disclosures",
    "premiumReceipts":      "/v1/premium-receipts",
    "evidence":             "/v1/evidence",
    "openapi":              "/v1/openapi.json"
  }
}
```

## §3 Programme and Party

```
POST   /v1/programmes               — register a programme
GET    /v1/programmes/{pid}         — retrieve programme
PATCH  /v1/programmes/{pid}/status  — advance status
POST   /v1/parties                  — register a party token
GET    /v1/parties/{prid}           — retrieve party (no PII)
PATCH  /v1/parties/{prid}/kyc       — update KYC state
```

KYC state transitions follow the operator's KYC vendor's outcome
codes; transitions that fail policy (e.g. `passed` to
`not-required`) return `422` with type
`urn:wia:insurtech:kyc-transition`.

## §4 Risk Objects, Coverages, Policies

```
POST   /v1/policies/{pid}/risk-objects   — register risk object
GET    /v1/risk-objects/{roid}           — retrieve risk object
POST   /v1/policies/{pid}/coverages      — register coverage
GET    /v1/coverages/{cvid}              — retrieve coverage
POST   /v1/programmes/{pid}/policies     — register a policy
PATCH  /v1/policies/{pid}/status         — advance status
PATCH  /v1/policies/{pid}/endorse        — emit an endorsement
                                            (creates a new policy
                                             revision; old revision
                                             remains addressable)
GET    /v1/policies/{pid}                — retrieve policy
```

Endorsements are content-addressed; the new revision references
the prior revision through `endorsementChainRef` (PHASE-1 §6).

## §5 Underwriting Decisions

```
POST   /v1/policies/{pid}/underwriting-decisions
                                          — register a decision
GET    /v1/underwriting-decisions/{did}   — retrieve decision
GET    /v1/underwriting-decisions/{did}/rationale
                                          — retrieve rendered
                                            rationale (the
                                            consumer-facing
                                            adverse-action text)
```

Decisions of outcome `decline` automatically trigger emission of
a consumer disclosure (PHASE-1 §10) of kind
`adverse-action-notice`; emission failure raises an audit alert.

## §6 Claims Lifecycle

```
POST   /v1/policies/{pid}/claims         — register a claim
PATCH  /v1/claims/{cid}/reserve          — update reserve
PATCH  /v1/claims/{cid}/status           — advance status
POST   /v1/claims/{cid}/payments         — record a payment
POST   /v1/claims/{cid}/fraud-investigation
                                          — open a fraud
                                            investigation
GET    /v1/claims/{cid}                  — retrieve claim
```

Claim payments that exceed the reserve trigger a reserve
re-strengthening alert; the operator's actuary reviews the alert
through the integration described in PHASE-4 §6.

## §7 Reinsurance Cessions

```
POST   /v1/cessions                      — register a cession
GET    /v1/cessions/{ceid}               — retrieve cession
POST   /v1/cessions/{ceid}/bordereau     — emit a bordereau
                                            artefact for the
                                            reporting period
GET    /v1/cessions/{ceid}/bordereau     — retrieve bordereau
```

Bordereau artefacts are content-addressed; reinsurer consumers
pin the manifest digest of the consumed bordereau in their own
ledgers so that retrospective audits can resolve the exact
artefact consumed.

## §8 Consumer Disclosures

```
POST   /v1/policies/{pid}/disclosures    — register a disclosure
GET    /v1/disclosures/{dsid}            — retrieve disclosure
PATCH  /v1/disclosures/{dsid}/acknowledged
                                          — record acknowledgement
```

Disclosures of kind `key-facts` for new business and
`renewal-notice` for renewals are mandatory; policies that reach
`bound` or `in-force` without the corresponding disclosure
emission are flagged in the operator's market-conduct dashboard.

## §9 Premium Receipts

```
POST   /v1/policies/{pid}/premium-receipts
                                          — register a receipt
GET    /v1/premium-receipts/{rcid}       — retrieve receipt
GET    /v1/policies/{pid}/premium-receipts?period=...
                                          — list receipts for a
                                            policy in a window
```

Receipt allocations that disagree with the bound coverage's
premium component return `422` with type
`urn:wia:insurtech:premium-allocation-mismatch`; reconciliation
flows through the operator's accounting system.

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:insurtech:status-transition`
- `urn:wia:insurtech:kyc-transition`
- `urn:wia:insurtech:reserve-overrun`
- `urn:wia:insurtech:premium-allocation-mismatch`
- `urn:wia:insurtech:disclosure-required`
- `urn:wia:insurtech:cession-overlap`
- `urn:wia:insurtech:evidence-mismatch`

## §11 Authentication and Authorisation

The API uses mutually-authenticated TLS for broker, MGA,
reinsurer, and regulator connections. Consumer-facing portals
authenticate consumers through the operator's identity provider
and translate consumer requests into authenticated API calls
under the operator's service-account credentials.

## §12 Caching and Concurrency

Stable resources (issued policies, settled claims, signed
bordereaux, signed evidence packages) are cacheable with
`Cache-Control: max-age=31536000, immutable`. Mutable resources
(open claims, pending underwriting decisions) are cacheable for
60 seconds. ETags are mandatory on every PATCH endpoint.

## §13 Audit and Observability

Every endpoint emits structured logs with `policyId` or
`claimId`, `traceId`, the issuing client certificate's subject,
and the operator's clock skew vs the reference NTP source.

## §14 Worked Example: Quote to Bind to Claim

1. Broker submits a quote request through `/v1/programmes/{pid}/
   policies` in `quoted` status with risk objects and coverages
   attached.
2. The operator's underwriting engine emits an underwriting
   decision via `/v1/policies/{pid}/underwriting-decisions`.
3. On `accept`, the broker advances the policy to `bound`; on
   first-premium receipt, the policy advances to `in-force`.
4. A claim is later registered via `/v1/policies/{pid}/claims`;
   the operator's adjuster updates reserve, captures evidence
   in the operator's claims-administration system, and on
   resolution advances claim status to `settled`.
5. The reinsurance cession's bordereau picks up the claim in the
   next reporting cycle.

## §15 Bulk Operations and Pagination

Bulk endpoints accept arrays for high-volume policy migration,
claim batch import, and bordereau ingest. Cursor-based
pagination uses the `cursor` query parameter and `Link` headers
(RFC 8288); cursors persist for at least 24 hours.

```
POST   /v1/bulk/policies               — batched policy import
POST   /v1/bulk/claims                 — batched claim import
POST   /v1/bulk/premium-receipts       — batched receipt ingest
GET    /v1/bulk/{operationId}          — operation status
```

## §16 Streaming Subscription Topics

Consumers subscribe via Server-Sent Events at:

- `/v1/policies/{pid}/events` — policy-scoped events
  (endorsements, status changes, renewal notices).
- `/v1/claims/{cid}/events` — claim-scoped events (reserve
  changes, payments, status changes, fraud-investigation
  opening / closing).
- `/v1/programmes/{pid}/events` — programme-wide events
  (regulator submissions, sanctions sweep completions,
  complaint escalations, model-risk-management alerts).

Subscribers reconnect via the `Last-Event-ID` header (W3C
EventSource semantics) so that brokered partners do not lose
visibility of priority-1 events during reconnection windows.

## §17 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                       for any PHASE-1 record
```

Provenance entries trace a record to its parents (policy,
underwriting decision, claim, treaty, disclosure) so that
auditors can walk the chain end-to-end without bespoke joins.

## §18 Privacy-Preserving Aggregation

Aggregate consumers (research collaboratives, industry
benchmarking services, public-policy analysts) fetch
population-level statistics through aggregation endpoints that
emit counts, means, and dispersions:

```
GET    /v1/aggregate/loss-ratio?line=...&period=...
GET    /v1/aggregate/persistency?line=...&cohort=...
GET    /v1/aggregate/complaint-rate?period=...&channel=...
```

Out-of-policy queries (cohort below threshold, per-policy
detail requested without authorisation) return `403 Forbidden`
with type `urn:wia:insurtech:cohort-too-small`.

## §19 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI
3.1 document, signs evidence packages per RFC 9421, and rejects
PII in any DATA-FORMAT field that this PHASE marks as opaque.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-insurtech
- **Last Updated:** 2026-04-28
