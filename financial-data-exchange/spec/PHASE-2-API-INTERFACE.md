# WIA-financial-data-exchange PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-financial-data-exchange
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that a
financial-data-exchange operator exposes for the
records defined in PHASE-1. Four complementary
surfaces are described: the ISO 20022 messaging
surface; the FIX 5.0 SP2 trading surface; the
SWIFT MT/MX legacy surface; and the HTTPS / JSON
RESTful surface for open-banking (PSD2 / Open Banking
UK / FDX / KR 마이데이터) and operational visibility.

References (CITATION-POLICY ALLOW only):

- ISO 20022 message implementation guidelines + the
  SWIFT MT-to-ISO-20022 industry coexistence
  cutover specification
- FIX 5.0 SP2 + FIX FAST + FIXatdl 1.1 + FIX
  Orchestra
- SWIFT MT + MX + GPI + CCT Inst
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web
  Linking), RFC 8259 (JSON), RFC 9421 (HTTP Message
  Signatures)
- IETF RFC 6749 (OAuth 2.0), RFC 8252 (OAuth for
  Native Apps), RFC 9126 (Pushed Authorization
  Requests), RFC 9396 (RAR Rich Authorization
  Requests), RFC 9449 (DPoP)
- OpenID Foundation FAPI 2.0 Security Profile +
  FAPI 2.0 Message Signing + FAPI-CIBA Profile
- OpenID Connect 1.0
- ISO 8601, ISO 9362, ISO 13616, ISO 17442
- ISO/IEC 27001:2022
- W3C Trace Context

---

## §1 Scope and Versioning

The operator exposes:

- The ISO 20022 messaging endpoint (SWIFT InterAct
  + bilateral channels).
- The FIX 5.0 SP2 endpoint for order entry and
  execution-report exchange.
- The SWIFT MT / MX endpoint over the SWIFT
  network.
- The HTTPS / JSON RESTful surface served from a
  domain published by the operator under `/v1/`.
- The open-banking endpoints for PSD2 / Open
  Banking UK / FDX / KR 마이데이터.

The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface; the FIX Orchestra
document, the ISO 20022 implementation guidelines,
and the relevant open-banking specification (PSD2
RTS / OBIE / FDX / KR 마이데이터) are canonical for
their respective surfaces.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-financial-data-exchange",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "parties":                 "/v1/parties",
    "accounts":                "/v1/accounts",
    "instruments":             "/v1/instruments",
    "iso20022":                "/iso20022/",
    "fix":                     "/fix/",
    "swiftMt":                 "/swift-mt/",
    "openBanking":             "/open-banking/",
    "fpml":                    "/v1/fpml-trades",
    "corporateActions":        "/v1/corporate-actions",
    "consents":                "/v1/consents",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 ISO 20022 Surface

Per business area:

```
POST   /iso20022/pacs/{messageId}    (pacs.008
                                      customer credit
                                      transfer · pacs.
                                      009 FI credit
                                      transfer · pacs.
                                      002 status report)
POST   /iso20022/pain/{messageId}    (pain.001 customer
                                      credit transfer
                                      initiation · pain.
                                      002 status report)
POST   /iso20022/camt/{messageId}    (camt.054 / camt.029
                                      / camt.056)
POST   /iso20022/setr/{messageId}    (setr.010 fund-
                                      order)
POST   /iso20022/seev/{messageId}    (corporate-actions
                                      seev.031~037)
```

## §4 FIX 5.0 SP2 Surface

```
FIX session (TCP / TLS) — order entry, modify,
cancel, execution-report exchange (D / G / F / 8)
FIX FAST / FIX-Stream over UDP for market-data
FIX Orchestra metadata served at /fix-orchestra/
descriptor.xml
```

## §5 SWIFT MT / MX Surface

```
SWIFTNet FIN — MT 103 / 202 / 202 COV legacy
customer / FI / cover-payment messages
SWIFTNet InterAct — MX wrapped ISO 20022 messages
SWIFT GPI tracker — UETR-keyed status updates
SWIFT GPI CCT Inst — cross-border instant
```

## §6 Open-Banking Surface (PSD2 / Open Banking UK /
       FDX / KR 마이데이터)

```
GET    /open-banking/aisp/v3/accounts             (PSD2
                                                   account-
                                                   information)
GET    /open-banking/aisp/v3/accounts/{accountId}/transactions
POST   /open-banking/pisp/v3/payments             (PSD2
                                                   payment-
                                                   initiation)
GET    /open-banking/cbpii/v3/funds-confirmations (PSD2
                                                   funds-
                                                   confirmation)
GET    /fdx/core/v6/accounts                      (FDX
                                                   v6 API)
GET    /fdx/core/v6/accounts/{accountId}/payments
GET    /fdx/core/v6/accounts/{accountId}/statements
GET    /마이데이터/v1/bank/accounts                 (KR
                                                   마이데이터)
GET    /마이데이터/v1/bank/accounts/{id}/transactions
```

The OAuth 2.0 client-authentication uses FAPI 2.0
Security Profile (mTLS or private_key_jwt + DPoP);
Pushed Authorization Requests (RFC 9126) and Rich
Authorization Requests (RFC 9396) carry the
account-set + permission-set scope per consent.

## §7 Party, Account, Instrument Endpoints

```
GET    /v1/parties?lei={lei}
GET    /v1/parties/{partyId}
POST   /v1/parties
GET    /v1/accounts?iban={iban}
GET    /v1/accounts/{accountId}
GET    /v1/instruments?isin={isin}
GET    /v1/instruments/{instrumentId}
```

## §8 FpML and Corporate-Actions Endpoints

```
GET    /v1/fpml-trades?uti={uti}
POST   /v1/fpml-trades
GET    /v1/fpml-trades/{tradeId}
GET    /v1/corporate-actions?instrument={isin}
POST   /v1/corporate-actions
GET    /v1/corporate-actions/{actionId}
```

## §9 Consent and SCA Endpoints

```
GET    /v1/consents?party={partyId}
POST   /v1/consents               (SCA-protected)
PATCH  /v1/consents/{consentId}/withdraw  (PSD2
                                           Art 64
                                           customer-
                                           withdrawal)
GET    /v1/consents/{consentId}/sca-history
```

## §10 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/parties
GET    /v1/examination/accounts
GET    /v1/examination/messages?from={iso}&to={iso}
GET    /v1/examination/fpml-trades
GET    /v1/examination/consents
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to the
authority's identity (US SEC + CFTC + FRB + OCC +
CFPB; EU EBA + ESMA + ECB + Member-State NCA; UK
FCA + PRA; KR FSC + FSS + FIU).

## §11 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 baseline; FAPI
2.0 Security Profile is the open-banking baseline.
Internal subsystem-to-subsystem calls use mutual
TLS with the operator's internal certificate
authority. The supervisory examination scope uses
read-only tokens bound to the authority's identity.

## §12 HTTP Status Codes

Standard codes apply (200 / 201 / 202 / 400 / 401 /
403 / 404 / 409 / 422 / 429 / 503) with Problem
Details bodies. Open-banking error responses follow
the operating jurisdiction's published error-code
schema.

## §13 Webhook and Event Surface

Lifecycle events:

- `message.sent`, `message.received`,
  `message.acknowledged`
- `payment.initiated`, `payment.executed`,
  `payment.rejected`
- `consent.captured`, `consent.withdrawn`,
  `consent.expired`
- `sca.completed`, `sca.exempted`
- `corporate-action.announced`,
  `corporate-action.payment-due`
- `fpml-trade.executed`,
  `fpml-trade.confirmed-clearing`

Webhook signatures use HTTP Message Signatures
(RFC 9421); FAPI 2.0 Message Signing applies to
open-banking webhooks.

## §14 Bulk-Export Surface

```
POST   /v1/bulk-export
GET    /v1/bulk-export/{exportId}/manifest
GET    /v1/examination/audit-events.csv
```

Supports the supervisory authority's MiFIR Article
26 transaction-report retrieval, EMIR trade-
repository retrieval, and FFIEC IT Exam data calls.

## §15 Trade-Repository and Reporting Surface

For OTC-derivative reporting and MiFIR transaction
reporting:

```
POST   /v1/reporting/emir-trade
POST   /v1/reporting/dodd-frank-swap
POST   /v1/reporting/mifir-transaction
GET    /v1/reporting/{reportId}/status
GET    /v1/reporting/{reportId}/acknowledgement
```

The reporting endpoints forward the report to the
operator's chosen ARM / SDR / TR per the operating
jurisdiction's rules. UPI + UTI + LEI references
are validated before submission.

## §16 GPI Tracker and UETR Surface

```
GET    /v1/gpi/tracker/{uetr}/status
GET    /v1/gpi/tracker/{uetr}/timeline
POST   /v1/gpi/tracker/update            (carrier-
                                          published
                                          status
                                          update)
```

The UETR is the canonical end-to-end correlation
identifier for SWIFT-routed customer-credit
transfers; the tracker exposes the per-hop status
chain for the originator's reconciliation.

## §17 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the ISO 20022 / FIX /
SWIFT / open-banking surfaces relevant to the
operator's role, expose the supervisory examination
surface, exercise the FAPI 2.0 Security Profile on
the open-banking surface, and propagate trace-
context across the message-to-settlement chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-financial-data-exchange
- **Last Updated:** 2026-04-29
