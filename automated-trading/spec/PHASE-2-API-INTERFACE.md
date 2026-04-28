# WIA-automated-trading PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-automated-trading
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that an
automated-trading operator (investment firm,
broker-dealer, proprietary-trading firm, futures
commission merchant, or trading venue) exposes for
the records defined in PHASE-1. Three complementary
surfaces are described: the FIX 5.0 SP2 messaging
surface — the operating wire format for order entry,
market-data, and trading-venue interaction; the ISO
20022 surface for post-trade reporting and reference-
data; and the HTTPS / JSON RESTful surface for
operational visibility, the firm's compliance-and-
audit functions, the venue's market-surveillance
endpoint, and the supervisory examination scope.

References (CITATION-POLICY ALLOW only):

- FIX 5.0 SP2 + FIX FAST + FIXatdl 1.1 + FIX
  Orchestra (the canonical wire format for order
  entry and market-data dissemination)
- ISO 20022 (the financial industry message scheme
  for post-trade and reference-data messaging)
- ISO 10383 (Market Identifier Code, MIC)
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web
  Linking), RFC 8259 (JSON), RFC 9421 (HTTP Message
  Signatures)
- ISO 8601, ISO 4217, ISO 3166-1, ISO 9362, ISO
  17442
- W3C Trace Context
- ISO/IEC 27001:2022
- EU MiFID II Articles 17, 47, 48, 50; MiFIR Articles
  26, 27; RTS 6, RTS 7, RTS 25
- US SEC Reg SCI Rules 1000–1006; SEC Rule 15c3-5;
  Reg ATS; Reg NMS; FINRA Rule 3110, 4511, 5210,
  6140; CFTC 17 CFR 1.81

---

## §1 Scope and Versioning

The operator exposes:

- The FIX 5.0 SP2 endpoint for order entry, modify,
  cancel, and execution-report exchange with the
  trading venue (or with the operator's clients
  routed to the venue).
- The ISO 20022 endpoint for post-trade transaction
  reporting (MiFIR Article 26 ARM / APA channels)
  and for reference-data exchange.
- The HTTPS / JSON RESTful surface served from a
  domain published by the operator under `/v1/`.

The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface. The FIX Orchestra
document published by the operator is canonical for
the FIX surface; the operator's ISO 20022 conformance
declaration is canonical for the ISO 20022 surface.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-automated-trading",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":              "/v1/programmes",
    "algorithms":              "/v1/algorithms",
    "orders":                  "/v1/orders",
    "executions":              "/v1/executions",
    "riskControls":            "/v1/risk-controls",
    "killSwitch":              "/v1/kill-switch",
    "transactionReports":      "/v1/transaction-reports",
    "surveillanceAlerts":      "/v1/surveillance-alerts",
    "conformanceTests":        "/v1/conformance-tests",
    "resilienceDrills":        "/v1/resilience-drills",
    "examination":             "/v1/examination",
    "openapi":                 "/v1/openapi.json"
  }
}
```

## §3 FIX 5.0 SP2 Order-Entry Surface

The FIX 5.0 SP2 message types in use:

- `D` (NewOrderSingle) — order submission.
- `G` (OrderCancelReplaceRequest) — modify.
- `F` (OrderCancelRequest) — cancel.
- `8` (ExecutionReport) — order acknowledgement,
  partial-fill, fill, cancel-confirmation, reject.
- `j` (BusinessMessageReject).
- `9` (OrderCancelReject).
- `H` (OrderStatusRequest) and `s` (OrderStatusReport).

Each FIX session is established with the venue's
SenderCompID / TargetCompID negotiation; the
operator's session-level resilience follows MiFID II
RTS 7 trading-venue resilience requirements
(re-connection discipline, sequence-number recovery).

## §4 Algorithm and Risk-Control Endpoints

```
GET    /v1/algorithms
GET    /v1/algorithms/{algorithmId}
POST   /v1/algorithms                 (register a new
                                       algorithm)
PATCH  /v1/algorithms/{algorithmId}   (parameter
                                       update — the
                                       firm's
                                       governance
                                       committee
                                       reference is
                                       required)
GET    /v1/risk-controls
GET    /v1/risk-controls/{controlId}
POST   /v1/risk-controls
PATCH  /v1/risk-controls/{controlId}/threshold
       (threshold update — RTS 6 Article 11 mandates
       documentation of every change)
```

## §5 Kill-Switch Endpoint

```
POST   /v1/kill-switch                (engage the
                                       kill switch —
                                       the rationale
                                       URI is required)
PATCH  /v1/kill-switch/{engagementId}/release
                                      (release after
                                       remediation;
                                       four-eyes
                                       discipline
                                       enforced)
GET    /v1/kill-switch/active
```

The kill-switch endpoint accepts the trader, the
supervisor, the risk officer, and (for automated
breaches) the firm's risk engine as authenticated
callers; engagement is recorded in PHASE-1 §7 with
the trigger kind, scope, and rationale narrative.

## §6 Transaction-Report Endpoints

```
POST   /v1/transaction-reports
GET    /v1/transaction-reports/{reportId}
GET    /v1/transaction-reports?from={iso}&to={iso}
```

The transaction-report endpoint forwards the report
to the operator's chosen ARM (Approved Reporting
Mechanism) or APA (Approved Publication Arrangement)
under MiFIR Article 26 / 27; for US-side flows the
report feeds the SEC Consolidated Audit Trail (CAT)
under SEC Rule 613 and the FINRA TRACE channel for
fixed-income transactions. KR-side flows feed the
KRX 거래보고 channel under 자본시장법.

## §7 Surveillance Endpoints

```
GET    /v1/surveillance-alerts
GET    /v1/surveillance-alerts/{alertId}
PATCH  /v1/surveillance-alerts/{alertId}     (record
                                              reviewer
                                              outcome)
POST   /v1/surveillance-alerts/{alertId}/stor
       (file the suspicious-transaction-and-order
        report to the Member-State NCA per MAR
        Article 16)
```

## §8 Conformance-Test and Drill Endpoints

```
GET    /v1/conformance-tests
POST   /v1/conformance-tests           (request a
                                        venue
                                        conformance
                                        test slot)
GET    /v1/conformance-tests/{testId}
GET    /v1/resilience-drills
POST   /v1/resilience-drills           (record a
                                        completed
                                        drill)
```

## §9 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/algorithms
GET    /v1/examination/orders?from={iso}&to={iso}
GET    /v1/examination/executions?from={iso}&to={iso}
GET    /v1/examination/risk-controls
GET    /v1/examination/kill-switch
GET    /v1/examination/surveillance-alerts
GET    /v1/examination/transaction-reports
```

The examination scope is read-only and bound to the
authority's identity (ESMA + Member-State NCA in EU,
SEC + FINRA + CFTC in US, KR FSC + FSS in KR, the
trading venue's market-surveillance function).

## §10 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 with audiences
declared per surface. FIX session-level security
follows the venue's published TLS / MAC discipline.
Internal subsystem-to-subsystem calls use mutual TLS
with the operator's internal certificate authority.
The kill-switch endpoint requires elevated scope and
enforces four-eyes discipline on release.

## §11 HTTP Status Codes

- `200 OK` — read or search success
- `201 Created` — create success (Location header
  carries the new resource URL)
- `202 Accepted` — order accepted for downstream
  routing
- `400 Bad Request` — malformed payload (Problem
  Details body)
- `401 Unauthorized` — missing or invalid bearer
  token
- `403 Forbidden` — discipline rejection (the
  Problem Details references the rejecting
  discipline — risk-control, restricted-list,
  algorithm-not-approved, kill-switch-engaged)
- `404 Not Found` — resource not registered with the
  operator's records
- `409 Conflict` — version-mismatch on update
- `422 Unprocessable Content` — validation failure
  with Problem Details issue details
- `429 Too Many Requests` — rate-limit exceeded
- `503 Service Unavailable` — venue or upstream
  market-data feed unavailable

## §12 Caching, Conditional Requests and Trace-Context

`ETag` carries the resource's version-id. Order-entry
responses are not cacheable — `Cache-Control: no-
store`. Trace-context (`traceparent`) is propagated
across the operator's pipeline so that the order-to-
execution chain (parent → child → venue → execution
report) can be reconstructed end to end for the
surveillance and examination surfaces.

## §13 Webhook and Event Surface

The operator publishes lifecycle events through a
webhook channel:

- `order.acknowledged`, `order.executed`,
  `order.cancelled`, `order.rejected`.
- `algorithm.deployed`, `algorithm.retired`.
- `risk-control.threshold-breached`,
  `risk-control.updated`.
- `kill-switch.engaged`, `kill-switch.released`.
- `surveillance-alert.detected`,
  `surveillance-alert.escalated`.
- `transaction-report.submitted`,
  `transaction-report.acknowledged`.

Webhook signatures use HTTP Message Signatures (RFC
9421).

## §14 Examination Bulk-Export Surface

```
POST   /v1/examination/bulk-export
GET    /v1/examination/bulk-export/{exportId}/status
GET    /v1/examination/bulk-export/{exportId}/manifest
```

Bulk exports support the supervisory authority's
periodic data calls (MiFIR Article 26 transaction-
report retrieval, SEC CAT data calls, FINRA OTS-
equivalent reviews). The export manifest declares
the cryptographic digest of each NDJSON file produced
so the receiving authority can verify integrity.

## §15 Conformance

Implementations claiming PHASE-2 conformance publish
the FIX Orchestra document, the OpenAPI document,
expose the kill-switch endpoint with the four-eyes
release discipline, expose the transaction-report
endpoint to the operator's chosen ARM / APA / CAT /
TRACE channel, expose the supervisory examination
surface, and propagate trace-context across the
order-to-execution chain.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-automated-trading
- **Last Updated:** 2026-04-28
