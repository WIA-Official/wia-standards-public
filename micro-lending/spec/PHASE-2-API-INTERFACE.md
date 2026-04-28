# WIA-micro-lending PHASE 2 — API Interface Specification

**Standard:** WIA-micro-lending
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the resource-oriented API surface
for micro-lending operations: customer onboarding and
KYC, group registration, loan-product catalogue,
application intake and decisioning, disbursement,
repayment, restructuring, write-off, customer-
protection disclosure, regulatory reporting, and
agent-network operations. The API is engineered to
operate over both bank-rail (ISO 20022) and mobile-
money rails (operator-specific) with a unified record
shape.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 9530 (Content-Digest)
- IETF RFC 6749, RFC 7636 (PKCE), RFC 8414 (OAuth Authorization Server Metadata)
- ISO 20022 — payment messages (pacs.008, pain.001, camt.054)
- BIS — payment-systems oversight principles
- FATF Recommendation 16 — wire transfers
- ITU-T X.1216 — security framework for digital financial services
- US Reg E (12 CFR 1005), Reg Z (12 CFR 1026); EU PSD2 (2015/2366)
- Open Banking (UK / EU NextGenPSD2) for sponsor-bank rails

---

## §1 Endpoint root

API root is implementation-controlled. All endpoints
are TLS 1.3 (RFC 8446). Sponsor-to-sponsor flows use
mutual TLS; agent-network and field-officer flows use
device-attested OAuth 2 with sponsor-issued client
certificates.

## §2 Customer endpoints

```
POST   /v1/customers                     register customer
GET    /v1/customers/{ref}               retrieve
PATCH  /v1/customers/{ref}               amend non-identity fields
POST   /v1/customers/{ref}/kyc           append KYC verification event
POST   /v1/customers/{ref}/screen        run sanctions / PEP screen
GET    /v1/customers?status=&risk=       list / filter
DELETE /v1/customers/{ref}               soft delete (closed account chain)
```

KYC verification events sign with the verifying
operator's key; screen events record outcome and the
list version (sanctions-list reference) consulted.

## §3 Group endpoints

```
POST   /v1/groups                        register group
GET    /v1/groups/{ref}                  retrieve
PATCH  /v1/groups/{ref}/members          add / remove members
POST   /v1/groups/{ref}/meetings         record meeting event (attendance,
                                         decisions, savings-deposit)
GET    /v1/groups/{ref}/loans            outstanding group-bound loans
```

Meeting records bind to the group's collective
guarantee chain so an inspector can reconstruct
attendance and decisions.

## §4 Product endpoints

```
GET    /v1/products                      list catalogue
POST   /v1/products                      publish new product
GET    /v1/products/{ref}                retrieve
PATCH  /v1/products/{ref}                amend (versioned)
GET    /v1/products/{ref}/disclosure     associated disclosure templates
```

Pricing changes trigger a new product version and a
fresh disclosure template; outstanding loans operate
on the product version they originated under.

## §5 Application endpoints

```
POST   /v1/applications                  intake application
POST   /v1/applications/{ref}/score      run credit-scoring
POST   /v1/applications/{ref}/decide     record decision
GET    /v1/applications/{ref}            retrieve
GET    /v1/applications?status=&channel= list / filter
```

Score requests pin the model version; the scoring
output is recorded with per-feature contribution where
the model is explainable (PHASE 3 §5).

## §6 Disbursement endpoints

```
POST   /v1/disbursements                 instruct disbursement
GET    /v1/disbursements/{ref}           retrieve
GET    /v1/disbursements?account=&date=  list / filter
POST   /v1/disbursements/{ref}/cancel    pre-execution cancellation
```

Instructions emit the appropriate rail message (ISO
20022 pacs.008 for bank rail; mobile-money operator
B2C messages for mobile rail). The implementation
records the rail-side identifier returned by the
operator.

## §7 Repayment endpoints

```
POST   /v1/repayments                    record repayment
GET    /v1/repayments/{ref}              retrieve
GET    /v1/repayments?account=&date=     list / filter
POST   /v1/repayments/$reconcile         end-of-day reconciliation
```

Reconciliation matches operator-side message reports
(camt.054 / mobile-money daily statement) with the
sponsor's ledger; mismatches open stewardship tasks.

## §8 Restructuring and write-off endpoints

```
POST   /v1/restructurings                propose restructuring
POST   /v1/restructurings/{ref}/approve  internal approval
GET    /v1/restructurings/{ref}          retrieve
POST   /v1/write-offs                    record write-off
POST   /v1/recoveries                    record post-write-off recovery
```

Approvals are role-segregated (originator ≠ approver)
per BIS Core Principles risk-management requirements.

## §9 Customer-protection disclosure endpoints

```
POST   /v1/disclosures                   issue disclosure to customer
POST   /v1/disclosures/{ref}/accept      record acceptance
GET    /v1/disclosures/{ref}             retrieve (gated by customer consent)
GET    /v1/customers/{ref}/disclosures   list per customer
```

Acceptance events sign with the customer's
authentication factor (PIN / OTP / biometric / wet-
ink-scan-with-witness) per the loan-agreement
disclosure.

## §10 Regulatory-report endpoints

```
POST   /v1/reports                       generate report (PAR / STR / CTR / etc)
GET    /v1/reports/{ref}                 retrieve
POST   /v1/reports/{ref}/submit          file with regulator gateway
GET    /v1/reports?type=&period=         list / filter
```

Submissions emit gateway-specific messages (e.g. FIU
goAML for STRs, central-bank reporting endpoints for
PAR / large-exposure reports).

## §11 Agent-network endpoints

```
POST   /v1/agents                        register agent
POST   /v1/agents/{ref}/till             open / close till session
POST   /v1/agents/{ref}/transactions     transaction list (cash-in / cash-out)
GET    /v1/agents/{ref}                  retrieve agent profile
```

Agent operations are audited per the rail-operator's
agent-management requirements (e.g. M-PESA / Airtel
Money agent SOPs).

## §12 Error model (RFC 9457)

```json
{
  "type":   "urn:wia:ml:problem:disclosure-not-accepted",
  "title":  "Customer has not accepted the disclosure",
  "status": 409,
  "detail": "Customer cu-2026-001 has not accepted disclosure d-2026-001",
  "instance": "/v1/disbursements"
}
```

Common type URIs:

| Type URI suffix              | HTTP | Meaning                                       |
|------------------------------|-----:|-----------------------------------------------|
| `disclosure-not-accepted`    | 409  | disbursement blocked without acceptance       |
| `kyc-incomplete`             | 403  | onboarding does not satisfy CDD               |
| `screening-blocking`         | 403  | sanctions / PEP hit blocking transaction      |
| `over-indebtedness`          | 422  | customer fails over-indebtedness check        |
| `dual-control-required`      | 403  | originator attempted approval                 |
| `rail-operator-error`        | 502  | upstream payment rail returned an error       |
| `idempotency-conflict`       | 409  | conflicting request under same idempotency key |

## §13 Bulk export

```
GET  /v1/$export?_type=Customer,LoanAccount,Repayment,Disclosure
GET  /v1/$status/{exportId}
GET  /v1/$result/{exportId}/{file}
```

Output is NDJSON.

## §14 Audit headers

| Header                  | Meaning                                       |
|-------------------------|-----------------------------------------------|
| `X-Request-Id`          | client-set, echoed                            |
| `X-Audit-Event-Id`      | server-set, links to PHASE 3 audit chain      |
| `Content-Digest`        | RFC 9530 SHA-256 of the response body         |
| `Idempotency-Key`       | RFC draft `idempotency-key-header`             |

## Annex A — OpenAPI reference

A canonical OpenAPI 3.1 description is published at
`api/openapi-3.1.yaml`.

## Annex B — Worked disbursement (informative)

```http
POST /v1/disbursements HTTP/1.1
Authorization: Bearer ...
Content-Type: application/json
WIA-ML-Schema-Version: 1.0
Idempotency-Key: 7c0d...

{
  "loanAccountRef": "la-2026-04-12-007",
  "amount": 50000,
  "currency": "KES",
  "disbursementChannel": "mobile-money",
  "paymentInstrumentRef": "+254712345678"
}
```

Response 201 returns the disbursement record with the
operator-side rail reference and the queued ISO 20022
message identifier.

## Annex C — Webhook surface

Implementations expose webhooks for `disbursement-
settled`, `repayment-received`, `par-bucket-changed`,
`screening-flag`, and `regulatory-report-submitted`
events. Payloads sign with RFC 7515 JWS; receivers
verify against `/.well-known/wia-ml-keys.json`.
Delivery is at-least-once; receivers are expected to
be idempotent on `eventId`.

## Annex D — Conformance disclosure

Implementations declare the OpenAPI revision served,
the ISO 20022 messages supported, the rail-operator
bindings (M-PESA / Airtel / MTN MoMo / etc), the
sanctions-list updates honoured, and the FIU
gateway profile.

## Annex E — Idempotency and replay

The API tolerates client retries through the
`Idempotency-Key` header. The implementation persists
the request hash + key for a 24-hour minimum so
retried requests return the original response without
side effects. Repayment endpoints are particularly
sensitive: an operator-rail timeout that retries may
otherwise post a duplicate payment.
