# WIA-cross-border-payment PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-cross-border-payment
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the API contract that a cross-
border payment operator exposes for the records defined
in PHASE-1. Two complementary surfaces are described:
the ISO 20022 / SWIFT FIN messaging surface — the
canonical wire format for cross-border value transfer —
and the HTTPS / JSON RESTful surface for operational
visibility, customer-channel interaction, and the
operator's compliance-and-audit functions.

References (CITATION-POLICY ALLOW only):

- ISO 20022 (the universal financial industry
  message scheme; the canonical wire format)
- SWIFT FIN, SWIFTNet, SWIFT GPI tracking
- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web Linking),
  RFC 8259 (JSON), RFC 9421 (HTTP Message Signatures)
- ISO 8601, ISO 4217, ISO 3166-1, ISO 9362, ISO
  13616, ISO 17442
- W3C Trace Context
- ISO/IEC 27001:2022
- CPMI Principles for Financial Market
  Infrastructures (the safety-and-efficiency
  principles for the operator's interaction with
  payment systems)

---

## §1 Scope and Versioning

The operator exposes:

- The ISO 20022 messaging endpoint for the canonical
  pacs.008 / pacs.009 / pacs.002 / camt.054 / camt.029
  exchange with the operator's correspondents.
- The SWIFT FIN endpoint for legacy MT 103 / 202 /
  202 COV exchange, retained where the operator's
  correspondent has not migrated.
- The HTTPS / JSON RESTful surface for customer
  channels, status visibility, and compliance
  surfaces, served from a domain published by the
  operator under `/v1/`.

The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical for the JSON surface; the operator's ISO
20022 conformance profile and SWIFT capability
declaration are canonical for the messaging surface.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-cross-border-payment",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":           "/v1/programmes",
    "instructions":         "/v1/instructions",
    "wireMessages":         "/v1/wire-messages",
    "screening":            "/v1/screening",
    "cbddq":                "/v1/cbddq",
    "settlements":          "/v1/settlements",
    "investigations":       "/v1/investigations",
    "sarRecords":           "/v1/sar-records",
    "customerPortal":       "/v1/customer-portal",
    "openapi":              "/v1/openapi.json"
  }
}
```

## §3 Instruction Submission and Lifecycle

```
POST   /v1/instructions
GET    /v1/instructions/{instructionId}
GET    /v1/instructions?endToEndId={id}&uetr={uetr}
PATCH  /v1/instructions/{instructionId}/cancel
```

The submission endpoint accepts the originating
customer's payment instruction; the operator's policy-
decision point evaluates the travel-rule completeness,
the screening outcome, and the operator's correspondent
relationship before generating the outbound wire
message. The cancel endpoint records a cancellation
request that the operator forwards along the chain via
camt.056 (FIToFIPaymentCancellationRequest).

## §4 Wire-Message Endpoints

```
GET    /v1/wire-messages?instruction={instructionId}
GET    /v1/wire-messages/{messageId}
POST   /v1/wire-messages/inbound      (correspondent
                                       inbound — used
                                       by partners
                                       that do not
                                       use SWIFT)
```

The wire-message endpoint exposes the canonical pacs
/ camt / MT message persisted in PHASE-1 §5 along
with its cryptographic digest. Inbound posting from a
non-SWIFT correspondent is authenticated through the
operator's bilateral channel agreement.

## §5 Status and SWIFT GPI Tracking Endpoints

```
GET    /v1/instructions/{instructionId}/status
GET    /v1/instructions/{instructionId}/timeline
```

The status endpoint surfaces the SWIFT GPI tracking
status and the equivalent ISO 20022 pacs.002 status
report:

- `ACSP` accepted-settlement-in-process
- `ACCP` accepted-customer-profile
- `ACSC` accepted-settlement-completed
- `RJCT` rejected
- `PDNG` pending

The timeline endpoint returns the per-hop chronology
of the correspondent chain so that the originator and
the beneficiary can both reconcile the lifecycle.

## §6 Screening and CBDDQ Endpoints

```
GET    /v1/screening?party={partyId}
POST   /v1/screening              (run a screening
                                   pass against a
                                   party reference)
GET    /v1/cbddq
GET    /v1/cbddq/{cbddqId}
POST   /v1/cbddq                  (record a received
                                   CBDDQ for review)
```

## §7 Investigation and Exception Endpoints

```
POST   /v1/investigations
GET    /v1/investigations/{investigationId}
PATCH  /v1/investigations/{investigationId}    (record
                                                 resolution)
```

The investigation endpoint records the camt.029
resolution-of-investigation request and its outcome.

## §8 Customer-Portal Endpoints

```
GET    /v1/customer-portal/me/instructions
GET    /v1/customer-portal/me/instructions/{instructionId}/status
POST   /v1/customer-portal/me/investigations
```

The customer's identity is bound to the bearer token
through the operator's IdP. The portal exposes the
status and the allowed-to-disclose subset of the
correspondent timeline (the FATF Recommendation 21
tipping-off discipline restricts what the operator
discloses where a SAR / STR is open).

## §9 SAR / STR Filing Endpoints

```
POST   /v1/sar-records
GET    /v1/sar-records/{sarId}
```

The SAR / STR filing endpoint forwards the operator's
suspicious-activity report to the operating
jurisdiction's FIU (US FinCEN, KR KoFIU, UK NCA-UKFIU,
EU AMLA-cross-border) using the FIU's published wire
format. FATF Recommendation 21 tipping-off discipline
applies — the operator does not disclose the SAR /
STR to the customer or the counterparty.

## §10 Authentication and Authorisation

Bearer tokens conform to OAuth 2.1 with audiences
declared per surface. The operator's bilateral
correspondent channel uses mutual TLS with the
counterparty's trust-anchor certificate authority.
The customer-portal surface uses consumer-OAuth flows.
The SAR / STR filing surface is locked to the
operator's compliance-officer role and is exempt from
customer-portal disclosure.

## §11 HTTP Status Codes

- `200 OK` — read or search success
- `201 Created` — instruction or record created
  (Location header carries the new resource URL)
- `202 Accepted` — instruction accepted for
  processing; the status endpoint will surface the
  evolving state
- `400 Bad Request` — malformed payload (Problem
  Details body)
- `401 Unauthorized` — missing or invalid bearer
  token
- `403 Forbidden` — discipline-rejection (the
  Problem Details references the rejecting
  discipline — sanctions screening, travel-rule
  completeness, CBDDQ-pending, correspondent-not-
  approved)
- `404 Not Found` — resource not registered with the
  operator's records
- `409 Conflict` — version-mismatch on update (`If-
  Match`)
- `422 Unprocessable Content` — validation failure
  with Problem Details issue details
- `429 Too Many Requests` — rate-limit exceeded
- `503 Service Unavailable` — downstream payment
  system or correspondent unavailable

## §12 Caching, Conditional Requests, and Trace-Context

`ETag` carries the resource's version-id. NPI-bearing
responses use `Cache-Control: private, no-store` —
travel-rule data and SAR / STR data are not cached
client-side. Trace-context (`traceparent`) is
propagated across the operator's pipeline; UETR is
the canonical end-to-end correlation across the
correspondent chain.

## §13 Webhook and Event Surface

The operator publishes the cross-border payment
lifecycle events through a webhook channel registered
by enterprise customers, the operator's compliance
function, and partner correspondents:

- `instruction.submitted` — the originator's
  instruction has been recorded.
- `screening.cleared` — the sanctions and adverse-
  media screening has cleared (or been cleared after
  manual review).
- `wire-message.dispatched` — the outbound pacs.008
  / pacs.009 / MT 103 / MT 202 has been transmitted
  to the next correspondent.
- `gpi.tracker-updated` — the SWIFT GPI tracker has
  posted a new status for the UETR.
- `settlement.completed` — settlement has reached
  finality on the relevant payment system.
- `investigation.opened` — a camt.029 resolution-of-
  investigation request has been raised.
- `investigation.resolved` — an investigation has
  been resolved.

Webhook signatures use HTTP Message Signatures (RFC
9421) so that the receiving system can verify that
the event originated from the operator. Retry
discipline follows the operator's published retry
budget for transient receiver failures.

## §14 Examination Surface

Supervisory authorities and external auditors with
the operator's examination scope read the operator's
records through:

```
GET    /v1/examination/programmes
GET    /v1/examination/instructions?from={iso}&to={iso}
GET    /v1/examination/screening?from={iso}&to={iso}
GET    /v1/examination/cbddq
GET    /v1/examination/sar-records          (FIU and
                                              supervisor
                                              scope)
GET    /v1/examination/audit-events?from={iso}&to={iso}
```

The examination scope is read-only and bound to the
authority's identity. The SAR / STR examination
endpoint is gated by the Recommendation 21 tipping-
off discipline — the endpoint is exposed to the FIU
and to the prudential supervisor under their
respective statutory authorities, not to general
external auditors.

## §15 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the customer-portal and
operator surfaces described above, exercise the
screening and travel-rule disciplines at submission
time, and propagate the UETR end-to-end.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-cross-border-payment
- **Last Updated:** 2026-04-28
