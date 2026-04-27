# WIA-payment-system PHASE 2 — API Interface Specification

**Standard:** WIA-payment-system
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a payment-system boundary exposes
for instruction origination, status query, settlement reporting,
sanctions screening, fraud-flag publication, dispute lifecycle, and
authorisation flow. The shape is HTTP/JSON for modern API consumers
and ISO 20022 XML for legacy clearing-system interconnect; both
share the same underlying records.

References (CITATION-POLICY ALLOW only):
- ISO 20022 — Universal Financial Industry message scheme
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 8259 (JSON)
- EMVCo Tokenisation Framework v2.x
- PCI DSS v4.0 — segmentation requirements influence the API surface
- ISO 8583 — message format for legacy card-network interconnect

---

## §1 Instruction origination (credit transfer)

```
POST /instructions HTTP/1.1
Host: pay.bank-k.example
Authorization: Bearer eyJhbGciOiJFUzI1NiIsImtpZCI6ImsxIn0...
Content-Type: application/wia-pay+json
Accept: application/wia-pay+json

{
  "messageType": "pacs.008.001.10",
  "groupHeader": {"messageId": "MSGID-2026-04-27-091500-0001", "creationDateTime": "2026-04-27T09:15:00+09:00"},
  "creditTransfer": {
    "paymentId": {"endToEndId": "E2E-7e2-91a7", "uetr": "..."},
    "interbankSettlementAmount": {"currency": "KRW", "amount": "1000000"},
    "chargeBearer": "SHAR",
    "debtor": {"name": "...", "identification": {"iban": "..."}},
    "debtorAgent": {"bicfi": "BANKKRSE"},
    "creditor": {"name": "...", "identification": {"iban": "..."}},
    "creditorAgent": {"bicfi": "BANKKRSE"},
    "remittanceInformation": {"unstructured": ["INV 2026-04-27-1234"]},
    "purposeOfPayment": {"code": "GDDS"}
  }
}
```

The boundary validates the structure against the ISO 20022 schema
the deployment honours, applies sanctions screening (PHASE 1 §7),
validates the originator's signing key, and emits an AuditEvent
keyed by the UETR. On acceptance the response carries the assigned
clearing-system reference and the next-step status URI.

## §2 Status query

```
GET /instructions/<uetr> HTTP/1.1
Authorization: Bearer ...
Accept: application/wia-pay+json
```

The response carries the most recent status transition from
`accepted` → `accepted-for-settlement` → `settled` (or
`returned` / `rejected` / `cancelled`). Status streams are also
available via Server-Sent Events on `/instructions/<uetr>/$watch`.

The boundary applies the same release-authority gate as instruction
origination; instructions the requester is not party to (not
debtor, creditor, debtor-agent, creditor-agent, or regulator) are
refused with `urn:wia:pay:problem:not-a-party`.

## §3 Settlement reporting

```
GET /reports/camt053?from=2026-04-27T00:00:00+09:00&to=2026-04-27T23:59:59+09:00 HTTP/1.1
Authorization: Bearer ...
```

Returns a camt.053 Bank-to-Customer Statement covering the requested
window. Statements are signed by the issuing bank and timestamped;
their integrity is verifiable against the bank's published JWKS.
Camt.054 Debit/Credit Notifications are pushed to subscribers in
near-real-time via Server-Sent Events on `/reports/camt054/$subscribe`.

## §4 Sanctions screening

```
POST /screening HTTP/1.1
{
  "instruction": {...},
  "screeningSources": ["un", "ofac-sdn", "eu-consolidated", "kr-mof-blacklist"]
}
```

The boundary runs the named lists against the instruction's
debtor/creditor/intermediary names, returning a screening evidence
record (PHASE 1 §7). Hits and partial matches require reviewer
disposition before the underlying instruction can be released.
The reviewer's disposition is itself a signed record so the
audit trail captures who decided and when.

## §5 Fraud-flag publication

```
POST /fraud/flags HTTP/1.1
{
  "subjectInstruction": "uetr:91a7...",
  "flagType": "velocity-anomaly",
  "severityBand": "advisory",
  "originPrincipal": "urn:wia:pay:fraud-engine:bank-k.realtime"
}
```

The boundary records the flag, links it to the underlying
instruction, and emits an AuditEvent. Promotion to `blocking`
happens via `POST /fraud/flags/<id>/$promote` with a signed
authorisation from the fraud-policy authority. Demotion is
analogous.

## §6 Card authorisation

Card authorisations flow over the card network; this PHASE exposes
the boundary-side ingest surface for non-network sources (e.g.,
e-commerce gateway POSTing an authorisation request to be routed):

```
POST /card/authorisations HTTP/1.1
{
  "tokenisedPAN": "TOK-...",
  "panSuffix": "4242",
  "cardScheme": "visa",
  "merchantId": "M-...",
  "terminalId": "T-...",
  "mcc": "5411",
  "transactionType": "00",
  "amount": {"currency": "KRW", "amount": "12000"},
  "cvm": "no-cvm",
  "terminalEntryMode": "contactless-emv"
}
```

The boundary forwards to the relevant card network using ISO 8583
(legacy) or ISO 20022 (where the network supports it) and returns
the network's response. PCI-scope segmentation applies: this
endpoint is *only* reachable from PCI-zone callers; non-PCI
callers receive `urn:wia:pay:problem:pci-zone-required`.

## §7 Dispute lifecycle

```
POST /disputes HTTP/1.1
{
  "subjectTransaction": "card-tx:91a7",
  "chargebackReasonCode": "13.1",  // visa: services not received
  "disputedAmount": {"currency": "KRW", "amount": "12000"},
  "evidenceRefs": [...]
}
```

Subsequent state transitions use `PUT /disputes/<id>` with the
new state. The boundary refuses transitions that violate the
scheme's chargeback workflow (e.g., `arbitration` cannot follow
`inquiry` directly).

## §8 Refund

```
POST /refunds HTTP/1.1
{
  "subjectTransaction": "card-tx:91a7",
  "amount": {"currency": "KRW", "amount": "12000"},
  "reason": "merchant-initiated-customer-request"
}
```

Refund flows are *new* card transactions referencing the original;
the original is preserved unchanged. Refund routing follows the
same network as the original authorisation.

## §9 Errors and warnings

| URI                                          | Status | Meaning                                       |
|----------------------------------------------|-------:|-----------------------------------------------|
| `urn:wia:pay:problem:malformed-iso20022`     | 400    | message fails schema validation               |
| `urn:wia:pay:problem:currency-mismatch`      | 422    | InterbankSettlementAmount vs InstructedAmount conflict without FX block |
| `urn:wia:pay:problem:sanctions-hit`          | 403    | sanctions hit awaiting reviewer disposition   |
| `urn:wia:pay:problem:fraud-block`            | 403    | fraud flag promoted to blocking               |
| `urn:wia:pay:problem:not-a-party`            | 403    | requester is not a party to the instruction   |
| `urn:wia:pay:problem:duplicate-uetr`         | 409    | UETR already in use                           |
| `urn:wia:pay:problem:pci-zone-required`      | 403    | endpoint reachable only from PCI-scope        |
| `urn:wia:pay:problem:audit-unavailable`      | 503    | audit chain write failed                      |

Warnings (200-OK with content caveats) use `Warning:` headers per
RFC 7234 §5.5 with codes namespaced under `wia-pay-`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked sanctions-screening sequence (informative)

```
POST /screening HTTP/1.1
Host: pay.bank-k.example
Authorization: Bearer ...
Content-Type: application/wia-pay+json

{
  "instruction": {...pacs.008...},
  "screeningSources": ["un", "ofac-sdn", "eu-consolidated", "kr-mof-blacklist"]
}
```

Response (clean):

```
200 OK
Content-Type: application/wia-pay+json

{
  "screeningId": "urn:wia:pay:screening:s-91a7",
  "result": "clear",
  "screenedSources": ["un", "ofac-sdn", "eu-consolidated", "kr-mof-blacklist"],
  "screeningTimestamp": "2026-04-27T09:15:01.230+09:00",
  "signature": "<JWS detached>"
}
```

Response (partial-match requires reviewer disposition):

```
202 Accepted
Content-Type: application/wia-pay+json

{
  "screeningId": "urn:wia:pay:screening:s-91a8",
  "result": "partial-match",
  "matchEvidence": [{"source": "ofac-sdn", "matchScore": "moderate", "matchedName": "...", "reasoning": "phonetic + Latin transliteration"}],
  "reviewerQueueRef": "/screening/queue/s-91a8"
}
```

A reviewer with `wia_role: compliance-officer` retrieves the queue
entry, reviews the match, and disposition signs to release or hold
the underlying instruction.

## Annex B — Capability advertisement (informative)

```
GET /.well-known/wia/payment-system/capabilities HTTP/1.1
```

Response advertises the deployment's supported ISO 20022 message
profiles, supported card networks, supported clearing rails,
supported jurisdictions, federation peers (correspondent banks),
and PCI DSS v4 conformance year. Capability documents are signed.

## Annex C — Pagination and rate limiting (informative)

Statement and report queries paginate at ≤ 1000 entries per page.
Per-token rate limits default to 100 instruction-origination calls
per second per institution and 1000 status-query calls per second.
Rate-limit refusals carry `urn:wia:pay:problem:rate-limited`.

## Annex D — 3DS authentication flow (informative)

For card-not-present (CNP) e-commerce flows, 3-D Secure authentication
runs before authorisation:

```
POST /3ds/authenticate HTTP/1.1
{
  "tokenisedPAN": "TOK-...",
  "amount": {"currency": "KRW", "amount": "12000"},
  "merchantId": "M-...",
  "deviceFingerprint": {...}
}
```

Response:

```
{
  "transactionStatus": "Y",   // Y / N / U / R / A / C / D
  "authenticationValue": "<EMVCo CAVV>",
  "ecommerceIndicator": "05",
  "dsTransactionId": "...",
  "acsUrl": null              // null when frictionless; URL when challenge required
}
```

A challenge flow returns an `acsUrl` to which the cardholder is
redirected for issuer challenge (OTP, biometric, or risk-based).
On challenge completion the merchant calls back into the
authorisation endpoint with the challenge result.

## Annex E — Capability advertisement (informative)

```
GET /.well-known/wia/payment-system/capabilities HTTP/1.1
```

Response advertises supported ISO 20022 message profiles, supported
card networks, supported clearing rails, supported jurisdictions,
correspondent-banking peers, PCI DSS v4 conformance year, and
3DS version support. Capability documents are signed.

## Annex F — Idempotency keys (informative)

POST endpoints accept an `Idempotency-Key` header (per the conventional
HTTP idempotency pattern) so retries from the originator do not create
duplicate instructions. The boundary stores the key for 24 hours; a
retry within that window with the same key returns the original response.
A retry with a different body but the same key is rejected with
`urn:wia:pay:problem:idempotency-conflict`. UETRs alone are not enough
because they're embedded in the instruction body and a retry that
fails at the originator before commit may not have a UETR yet.
