# WIA-privacy PHASE 2 — API Interface Specification

**Standard:** WIA-privacy
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a privacy-control
deployment exposes for personal-data inventory queries,
RoPA management, consent capture and withdrawal, DSR
intake and fulfillment, DPIA management, breach-notification
handling, cross-border-transfer registration, sub-processor
disclosure, and privacy-notice publication.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 27701:2019 — PIMS API conventions referenced where applicable
- ISO/IEC 29184:2020 — online privacy notice + consent surfaces
- ISO 31700-1:2023 — Privacy by Design surfaces
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS)
- W3C DPV (Data Privacy Vocabulary) for purpose / lawful-basis vocabulary
- WIA-identity-management PHASE 2 — for DSR identity-verification flow

---

## §1 Inventory + RoPA endpoints

```
GET /inventory/{inventoryId} HTTP/1.1
GET /inventory?dataCategory=health-info&controller=...
POST /inventory HTTP/1.1
PUT /inventory/{inventoryId}
DELETE /inventory/{inventoryId}    (with audit + replacement reference)
```

Inventory mutations require DPO / CPO authorisation.

```
GET /ropa/{ropaId}
GET /ropa?controller=...&purpose=direct-marketing-email
POST /ropa HTTP/1.1
PUT /ropa/{ropaId}/state           (active / superseded / cancelled)
```

RoPA exports for regulator inspection:

```
GET /ropa/$export?format=csv&controller=...&since=...
GET /ropa/$export?format=ms-excel&controller=...
GET /ropa/$export?format=jsonl&controller=...
```

The exporter signs the export bundle so regulators can verify
authenticity offline.

## §2 Consent capture and withdrawal

```
POST /consents HTTP/1.1
Content-Type: application/json
```

Body is a PHASE 1 §4 consent record. The boundary verifies
the noticeTextRef is current and signs the resulting consent
with the controller's signing key.

```
GET /consents?dataSubjectRef=...&active=true
PUT /consents/{consentId}/withdraw
```

Withdrawal is a first-class operation (Art. 7(3) GDPR / K-PIPA
§22(7)): the boundary timestamps the withdrawal, propagates
to subscribed downstream systems via webhook, and audit-chains
the event. Withdrawal MUST be as easy as the original consent
(K-PIPA §22(8) 동의 방법과 동일한 방법으로 철회).

```
GET /consents/{consentId}/evidence
```

Returns the evidence-artifact reference (signed) so auditors
can verify the consent was captured against the notice version
shown.

## §3 DSR intake + fulfillment

```
POST /dsr HTTP/1.1
Content-Type: application/json
```

Body is a PHASE 1 §5 DSR record. The boundary:

1. Acknowledges receipt within the regime's time bound
   (GDPR: undue delay; K-PIPA: 즉시; CCPA: 10 days for
   acknowledgement)
2. Initiates identity-verification proportionate to the
   request type (cross-reference WIA-identity-management
   PHASE 2)
3. Sets the regulatory deadline timer
4. Routes the request to the controller's DSR fulfillment
   workflow

```
GET /dsr/{dsrId}
PUT /dsr/{dsrId}/state
PUT /dsr/{dsrId}/extension      (with reason; per regime allowance)
POST /dsr/{dsrId}/fulfillment   (uploads fulfillment evidence)
POST /dsr/{dsrId}/refusal       (uploads refusal record with legal basis)
```

Refusal records cite the specific provision (e.g., GDPR Art. 15(4)
third-party rights, K-PIPA §35(4) 다른 사람의 권리 침해).

```
GET /dsr/$export?format=jsonl&controller=...
```

Returns DSR records for regulator inspection (subject-PII
redacted unless the requester has the regulator role).

## §4 DPIA management

```
POST /dpia HTTP/1.1
GET /dpia/{dpiaId}
PUT /dpia/{dpiaId}/state
POST /dpia/{dpiaId}/regulator-consultation
```

State transitions: `draft` → `under-review` → `approved` →
`active` → `superseded`. Material RoPA changes flag bound
DPIAs as `requires-review`.

```
GET /dpia?triggeringRopaRef=...&active=true
```

## §5 Breach-notification

```
POST /breaches HTTP/1.1
```

Body is a PHASE 1 §7 breach record. The boundary:

1. Starts the regime-specific notification timer (GDPR 72h,
   K-PIPA 72h KISA + 5일 정보주체, US states per applicable
   state)
2. Routes to the regulator-notification workflow
3. Triggers subject-notification preparation if Art. 34
   threshold met or jurisdiction requires

```
PUT /breaches/{breachId}/state
POST /breaches/{breachId}/regulator-notification
POST /breaches/{breachId}/subject-notification
GET /breaches/{breachId}/timeline
```

Timeline endpoint returns the full chronology with each
event signed by the responsible authority.

## §6 Cross-border-transfer registry

```
POST /transfers HTTP/1.1
GET /transfers?dataExporter=...&active=true
PUT /transfers/{transferId}/state
POST /transfers/{transferId}/tia      (Transfer Impact Assessment)
```

The boundary refuses processing operations whose declared
transfer mechanism is invalid or expired (e.g., SCC 2010/87
deprecated since 2022-12-27).

## §7 Sub-processor disclosure

```
GET /subprocessors?parentProcessor=...
POST /subprocessors HTTP/1.1
PUT /subprocessors/{subprocessorId}/state
```

Webhook subscriptions for sub-processor changes:

```
POST /subscriptions HTTP/1.1
{
  "events": ["subprocessor-added", "subprocessor-removed", "subprocessor-changed"],
  "webhookUrl": "https://customer-x.example/wia-privacy-webhook"
}
```

The boundary signs webhook payloads (detached JWS in
`Wia-Signature` header) so customers verify provenance.

## §8 Privacy-notice publication

```
POST /notices HTTP/1.1
GET /notices/{noticeId}
GET /notices?language=ko-KR&active=true
PUT /notices/{noticeId}/state
```

Notice mutations are versioned; consents reference the
notice version active at consent-capture time.

```
GET /notices/{noticeId}/translations?format=icu
```

Returns ICU-format translation bundle for the notice (where
multi-language deployments offer localised notice presentation).

## §9 Data-subject self-service portal endpoints

For deployments offering self-service:

```
GET /me/consents      (subject-token bound)
GET /me/processing    (subject's RoPA participation overview)
POST /me/dsr          (self-service DSR submission)
GET /me/dsr/{dsrId}/status
```

Self-service portal endpoints are gated by the subject's
authenticated session (cross-reference WIA-identity-management
PHASE 2 §3).

## §10 Capability discovery

```
GET /.well-known/wia/privacy HTTP/1.1
```

Returns the capability document:

```json
{
  "wia.standardVersion": "1.0",
  "wia.implementationVersion": "controller-x-3.2.0",
  "supportedRegimes": ["gdpr", "k-pipa", "ccpa", "lgpd"],
  "primaryAuthority": "urn:wia:org:dpo:controller-x",
  "ropaExportFormats": ["jsonl", "csv", "ms-excel"],
  "dsrChannels": ["web-form", "email", "phone", "paper"],
  "subprocessorPublic": true,
  "manifest": "https://controller-x.example/.well-known/wia/privacy/manifest.jws"
}
```

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Idempotency

Write endpoints accept `Idempotency-Key`. Boundary stores
keys for 30 days. Replays return the original response.

## Annex B — Pagination

List endpoints support cursor pagination. Cursors are signed
by the boundary, valid for 30 minutes.

## Annex C — Negative-test vectors (informative)

| Stimulus                                               | Expected response                          |
|--------------------------------------------------------|--------------------------------------------|
| Consent POST without noticeTextRef                     | 422 + `notice-reference-missing`           |
| DSR for unverified identity (sensitive request)        | 422 + `identity-verification-required`     |
| DSR fulfillment past regulatory deadline               | 200 with `delayed=true` + audit warning   |
| Cross-border POST with expired SCC reference           | 422 + `transfer-mechanism-invalid`         |
| Breach POST after 72h without regulator notification   | accepts; emits `late-notification` flag   |
| Subprocessor mutation without DPA reference            | 422 + `dpa-reference-missing`              |

## Annex D — Webhook subscriptions

Event classes: `consent-given`, `consent-withdrawn`,
`dsr-received`, `dsr-fulfilled`, `dsr-refused`,
`breach-opened`, `breach-state-changed`,
`subprocessor-added`, `subprocessor-removed`,
`notice-version-published`, `transfer-mechanism-expired`.

## Annex E — Authorities and roles

| Role                        | Scope                                                   |
|-----------------------------|---------------------------------------------------------|
| `data-subject`              | own data (self-service consent, DSR submission)         |
| `dpo` / `cpo`               | full controller-side operations                          |
| `processor-administrator`   | processor-bound operations                               |
| `subprocessor`              | scope per parent-processor authorisation                 |
| `regulator`                 | read-only across deployment scope                        |
| `auditor`                   | read-only across engagement scope                        |
| `customer-administrator`    | customer-tenant operations (B2B SaaS)                    |

## Annex F — Worked DSR fulfillment record (informative)

```json
{
  "fulfillmentId": "urn:wia:priv:dsr-fulfillment:controller-x:f-014",
  "dsrRef": "urn:wia:priv:dsr:controller-x:d-014",
  "fulfilledAt": "2026-04-30T10:00:00+09:00",
  "fulfilledBy": "urn:wia:hr:dpo-staff:s-7e2c",
  "deliveryChannel": "secure-portal-download",
  "evidenceArtifactRef": "urn:wia:priv:evidence:f-014-export.json.signed",
  "verificationMethod": "subject-account-with-mfa",
  "completionSignature": "<jws-detached>"
}
```

## Annex G — Capability versioning

`wia.standardVersion` and `wia.implementationVersion` declared
in the capability document. Standard-version mismatch is a
hard refusal; implementation-version mismatch is logged.

## Annex H — Audit-chain replay endpoint

```
GET /audit/chain?since=...&kind=consent-given,dsr-received,breach-opened
Accept: application/x-ndjson
```

For regulators / auditors. Restricted kinds (e.g., breach
detail) require the requester's role to authorise.

## Annex I — Regulator submission endpoints

For deployments operating in jurisdictions that require
direct regulatory submissions:

```
POST /regulator/submissions
```

Body declares the regulator URN, the submission kind
(annual report, breach notification, RoPA inspection
response), and the artifact bundle reference. The boundary
records the submission and the regulator's acknowledgement.

## Annex J — Bulk-export gating

Bulk exports (`/ropa/$export`, `/dsr/$export`) are gated by
the deployment's bulk-quota policy and the requester's role.
DPO / regulator / auditor roles have higher quotas; processor
roles are scoped to processor-bound records.
