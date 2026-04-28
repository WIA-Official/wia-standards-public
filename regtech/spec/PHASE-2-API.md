# WIA-FIN-004 — Phase 2: API Interface

> RegTech canonical Phase 2: API surface (events + screening + risk + travel-rule + alerts + audit).

# WIA-FIN-004: RegTech Standard Specification v1.0

**Version:** 1.0.0
**Status:** Final
**Date:** January 2025
**Authors:** WIA Standards Committee
**Organization:** World Certification Industry Association (WIA)

---

## Abstract

This specification defines the WIA-FIN-004 RegTech Standard for regulatory technology and compliance automation in financial services. It provides comprehensive data formats, API specifications, security protocols, and integration guidelines for implementing automated compliance, AML/KYC verification, and real-time monitoring systems.

**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Data Formats](#5-data-formats)
6. [API Specifications](#6-api-specifications)
7. [Security Requirements](#7-security-requirements)
8. [Integration Guidelines](#8-integration-guidelines)
9. [Compliance](#9-compliance)
10. [Examples](#10-examples)

---


## 6. API Specifications

### 6.1 RESTful API Endpoints

#### 6.1.1 Compliance Check

**Endpoint:** `POST /api/v1/compliance/check`

**Request:**
```json
{
  "transactionId": "string",
  "amount": "number",
  "currency": "string",
  "customerRiskProfile": "low | medium | high",
  "jurisdiction": "string"
}
```

**Response:**
```json
{
  "riskScore": "number (0-100)",
  "status": "pass | review | fail",
  "requiresSAR": "boolean",
  "requiresCTR": "boolean",
  "recommendation": "string",
  "timestamp": "ISO 8601 string"
}
```

#### 6.1.2 KYC Verification

**Endpoint:** `POST /api/v1/kyc/verify`

**Request:**
```json
{
  "customerId": "string",
  "documents": [{
    "type": "passport | drivers_license | national_id",
    "documentNumber": "string",
    "issuingCountry": "string",
    "image": "base64 string (optional)"
  }]
}
```

**Response:**
```json
{
  "status": "verified | pending | rejected",
  "confidence": "number (0-100)",
  "documentValid": "boolean",
  "identityMatch": "boolean",
  "nextAction": "string"
}
```

#### 6.1.3 AML Screening

**Endpoint:** `POST /api/v1/aml/screen`

**Request:**
```json
{
  "entity": {
    "name": "string",
    "type": "person | organization",
    "dateOfBirth": "string (optional)",
    "nationality": "string (optional)"
  }
}
```

**Response:**
```json
{
  "sanctionsMatch": "boolean",
  "pepStatus": "boolean",
  "watchList": "boolean",
  "riskLevel": "low | medium | high",
  "actionRequired": "string"
}
```

#### 6.1.4 Report Submission

**Endpoint:** `POST /api/v1/reports/submit`

**Request:**
```json
{
  "reportType": "sar | ctr | eft | atr",
  "reportId": "string",
  "data": "object",
  "narrative": "string",
  "jurisdiction": "string"
}
```

**Response:**
```json
{
  "status": "submitted | rejected",
  "confirmationNumber": "string",
  "timestamp": "ISO 8601 string"
}
```

### 6.2 Authentication

All API requests MUST include authentication using one of:

- **OAuth 2.0 Bearer Token:** `Authorization: Bearer <token>`
- **API Key:** `X-API-Key: <key>`

### 6.3 Rate Limiting

API endpoints are subject to rate limiting:

- Standard tier: 1,000 requests per hour
- Premium tier: 10,000 requests per hour
- Enterprise tier: Unlimited

Rate limit headers are included in responses:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640995200
```

---




---

## A.1 Endpoint reference

```http
POST /regtech/v1/events                         # register compliance event
GET  /regtech/v1/events/{id}                    # fetch event
POST /regtech/v1/screening/sanctions            # screen entity against sanctions
POST /regtech/v1/screening/pep                  # screen entity for PEP status
POST /regtech/v1/risk/score                     # compute risk score
POST /regtech/v1/travel-rule/exchange           # exchange travel-rule envelope
GET  /regtech/v1/audit/{eventId}                # audit trail
WS   /regtech/v1/alerts/stream                  # real-time alert stream
```

Every endpoint follows the discovery convention at `/.well-known/wia-regtech`.

## A.2 Event-and-screening API

`POST /events` accepts the Phase 1 §A.1 envelope and returns a stable `eventId`. `POST /screening/sanctions` and `POST /screening/pep` accept entity descriptors (name, date-of-birth, address, identifiers) and return a canonical match-result envelope with per-list per-match score and the recommended triage action. The endpoint validates list-version freshness against the operator's policy threshold (typical 24-hour stale tolerance for OFAC SDN; 7-day for commercial PEP lists).

## A.3 Risk-scoring API

`POST /risk/score` runs the operator's deployed risk-scoring model against the requested entity / transaction / counterparty and returns the score plus the explainability envelope. The endpoint enforces model-version-of-record (the response carries `X-Model-Version`), the input-validation envelope (PII fields validated per the input schema), and the per-tenant rate-limit. Model deprecation triggers an envelope-level deprecation header so consumers can migrate to the successor model within the documented retirement window.

## A.4 Travel-rule API

`POST /travel-rule/exchange` accepts the Phase 1 §A.4 envelope and returns the per-VASP delivery confirmation. The endpoint enforces the IVMS 101 schema, the per-jurisdiction threshold envelope, and the secure-data-exchange envelope (recipient VASP's public-key envelope from the operator's trust list; per-message Ed25519 signature). Failed deliveries are retried per the operator's policy and quarantined-then-escalated when retry-budget is exhausted.

## A.5 Alerts WebSocket and audit API

The alerts-stream WebSocket multiplexes per-tenant alert events: new SAR/STR drafted, new sanctions-list update with the per-tenant impact summary, new high-risk transaction routed for review, model-drift threshold crossed. Subscribers can filter by alert-class. Audit endpoint `GET /audit/{eventId}` returns the immutable audit trail signed at every state transition; the audit-integrity envelope is anchored in the Merkle tree per Phase 1 §A.5.

## A.6 Rate-limit envelope

100 req/h unauthenticated, 1000 req/h authenticated, 10000 req/h trusted-partner with the per-jurisdiction rate-limit overrides. Bulk-export (`GET /events/export?after=cursor&limit=N`) max 1000 per page with the cursor-based pagination. Search endpoints (sanctions / PEP screening) are subject to a stricter look-to-block ratio per the operator's anti-abuse policy.

## A.7 SupTech regulator-side endpoints

Regulator-facing endpoints (gated by the regulator's mTLS certificate plus the operator-side authorisation envelope) include: `GET /supervisor/events?regulator={id}` for the regulator-scoped event feed (per-regulator authority filter applied server-side); `POST /supervisor/inquiries` for regulator-initiated inquiries; `GET /supervisor/dashboards/{id}` for the regulator-side aggregated dashboards. Per-regulator data-scope envelopes are negotiated at on-boarding and signed by both the operator and the regulator's certificate authority.

## A.8 Reporting and filing endpoints

Filing endpoints follow the per-jurisdiction submission protocol: `POST /filings/sar` accepts the SAR/STR envelope per Phase 1 §A.7 and returns the filing-confirmation receipt with the regulator-issued reference number (FinCEN BSA reference; goAML XML reference; KOFIU receipt-of-record); `POST /filings/ctr` accepts the currency-transaction-report envelope; `POST /filings/cmir` accepts the cross-border-currency-and-monetary-instruments envelope; `POST /filings/fbar` for FBAR (FinCEN Form 114) where applicable. Submission failures trigger a retry within the operator's documented SLA with escalation to the operator's compliance officer when the retry budget is exhausted.

## A.9 Webhook delivery for compliance events

Operators can subscribe to webhook deliveries on compliance-event lifecycle (created, triaged, escalated, resolved, filed-with-regulator, regulator-acknowledged) and on sanctions-list update events. Webhook payloads carry the same envelope shape as the API GET response, signed by the WIA tenant key, and retried with exponential backoff up to 24 h before the broker logs a permanent-failure event for operator review. Subscriber endpoints MUST validate the webhook signature against the tenant key chain before processing.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/regtech/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-regtech-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/regtech-host:1.0.0` ships every regtech envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/regtech.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Regtech deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
