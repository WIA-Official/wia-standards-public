# WIA-payment-system PHASE 4 — Integration Specification

**Standard:** WIA-payment-system
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a payment-system deployment integrates the
data, APIs, and protocols from PHASEs 1–3 with adjacent systems:
core banking, card networks, instant-payment rails, fraud and
sanctions services, dispute platforms, regulators, and customer
channels. It is non-prescriptive about specific vendors; it
specifies the integration *contracts* a deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- ISO 20022 Migration Programme (SWIFT — corresponding banking)
- BIS CPMI Principles for Financial Market Infrastructures (PFMI)
- PCI DSS v4.0 — segmentation, key management, Requirement 10 audit
- EMVCo Book 3 — online authorisation flows
- FATF Recommendations 10, 16, 32 — CDD, wire transfers, cross-border
- Domestic schemes: KFTC (KR), NACHA + Federal Reserve (US),
  Pay.UK / OBIE (UK), EBA Clearing (EU), JBA Zengin (JP)

---

## §1 Core-banking integration

The deployment's core-banking system is the system of record for
account balances, customer master data, and accounting. The
boundary integration contract:

- Account look-ups (by IBAN, domestic account number, alias) flow
  through a thin adapter that hides core-banking specifics
- Balance checks are performed at instruction-acceptance time;
  insufficient-funds rejection is a structured response (the
  underlying core-banking error code is mapped to the boundary's
  `urn:wia:pay:problem:insufficient-funds` URI)
- Posting (debit/credit) is committed only on settlement finality;
  tentative postings (instant-payment hold) are explicit in the
  core-banking ledger so reconciliation is unambiguous

The core-banking system never reaches outside the boundary
directly; all external traffic flows through the boundary so the
audit chain captures every external interaction.

## §2 Card-network integration

For card flows, the deployment connects to one or more card networks
(Visa, Mastercard, AmEx, JCB, Discover, UnionPay, BC카드, KEB하나카드,
RuPay, etc.). Integration:

- A separate processor connection per network, each in its own
  CDE-segment subnet
- ISO 8583 (legacy) or ISO 20022 (newer) message exchange per
  network's protocol
- Token-vault segregation: each network's tokens live in a
  network-specific vault to honour scheme isolation
- Settlement files (clearing day) are reconciled into the boundary's
  audit chain on receipt; reconciliation discrepancies are surfaced
  as incidents

Each card network has its own scheme rules (interchange, fees,
chargeback reason codes, refund-window). The deployment maintains
a per-network rules engine so the boundary applies the correct
rules per transaction.

## §3 Instant-payment rails

Connections to instant-payment rails (FedNow, SEPA Instant, KFTC
신속이체, UPI in IN, PromptPay in TH, etc.) require:

- 24×7 operational availability (these rails are 24×7)
- Per-rail message profile compliance (often a national variant of
  ISO 20022 pacs.008)
- Sub-10-second end-to-end SLA enforcement
- Distinct reconciliation per rail

A failed instant-payment instruction is reversed on the rail's
return path; partial-success states do not exist for instant
finality rails.

## §4 ACH / batch-clearing integration

For batch-clearing rails (NACHA in US, KFTC bulk in KR, BACS in UK):

- Bulk file generation per scheme format (NACHA has a fixed-width
  format; SEPA Direct Debit uses ISO 20022 pain.008)
- Batch sign-off requires reviewer authorisation in the deployment's
  approval workflow
- Return windows per scheme (NACHA NSF window, SEPA SDD return
  window) are tracked per transaction and surfaced to operations
  as expiry approaches

Returns are *new* instructions referencing the original; the
original is preserved unchanged.

## §5 Fraud and sanctions services

Fraud engines (real-time scoring, behavioural-anomaly, network-rule)
and sanctions-screening services run as adjacent micro-services
behind the boundary. Integration:

- Each service authenticates with mTLS + signed responses; the
  boundary refuses to honour unsigned fraud / screening responses
- Service unavailability has tier-specific consequences:
  - Sanctions screening unavailable: cross-border flows queued;
    domestic continues with deferred screening flagged
  - Fraud engine unavailable: instructions accepted with `pending
    fraud review` flag for retroactive analysis
- Service decisions are recorded as signed records in the chain;
  retrospective edits are detectable

## §6 Dispute platform

Disputes flow through a separate dispute-platform service that
implements the per-network chargeback workflow:

- The platform consumes the boundary's transaction stream and
  surfaces customer-disputed transactions to operations
- Transitions (issue chargeback, accept, represent, escalate to
  arbitration) call back into the boundary's PHASE 2 §7 endpoints
- Evidence packages (receipts, AVS/CVV2 results, 3DS authentication
  evidence) attach to dispute records as referenced documents

The dispute platform never bypasses the boundary; every transition
is auditable.

## §7 Regulator interface

Regulators (KR FSS / FIU, US OFAC / FinCEN, EU national CAs, UK FCA,
JP FSA) request audit windows during investigations. The integration
contract:

- Regulator requests are signed; the boundary's audit chain records
  the request
- The boundary returns the AuditEvents intersecting the request,
  the daily roots, and inclusion proofs
- The boundary emits an AuditEvent recording the regulator's
  request itself so regulator-initiated audits are auditable

CTR / SAR (Currency Transaction Report / Suspicious Activity
Report) submissions follow the jurisdiction's reporting workflow;
the boundary tracks submission status (submitted / acknowledged /
queried) so the deployment knows where each report is in the
regulator's workflow.

## §8 Customer-facing channels

Mobile banking, internet banking, branch tellers, and ATM networks
all consume the boundary's API. Integration:

- Each channel authenticates with channel-specific credentials
  (mobile: device-bound JWT; teller: smart-card + PIN; ATM: card
  + EMV CDCVM)
- Channel-specific UI state is *not* in scope of this PHASE; only
  the resulting payment instruction crosses the boundary
- Receipts (ATM slips, mobile push notifications, e-statements)
  are derived projections of the canonical record; their fidelity
  is verifiable against the chain

## §9 Acceptance criteria

A deployment claims conformance when:

1. Every instruction in the past quarter has a matching audit
   chain entry with verifiable inclusion proof
2. Every cross-border instruction has FATF Rec 16 originator and
   beneficiary information complete
3. Every card transaction in the past quarter is in the relevant
   network's settlement file (no orphan authorisations)
4. PCI DSS v4 segmentation has been re-validated within the past
   year by an authorised QSA
5. Sanctions screening evidence is on file for every cross-border
   instruction
6. ASV scans for the CDE pass quarterly
7. Regulatory submissions (CTR/SAR/equivalent) are current
8. Quarterly compliance report has no integrity-check failures

A deployment failing any of these reports the gap in its compliance
package rather than concealing it.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Operational SLAs (informative)

| Concern                                          | Default SLA                  |
|--------------------------------------------------|------------------------------|
| Instruction-origination p95 added latency        | ≤ 200 ms (retail)            |
|                                                  | ≤ 50 ms (instant payment)    |
| Sanctions screening turnaround                   | ≤ 500 ms (clean cases)       |
|                                                  | manual disposition window per scheme (hits) |
| Card authorisation network round-trip            | ≤ 2 s (95th)                 |
| Settlement finality posting after acceptance     | rail-specific (RTGS: minutes;|
|                                                  | instant: seconds; ACH: clearing day) |
| Audit chain entry available after operation      | ≤ 1 s                        |
| Statement generation (camt.053)                  | ≤ 24 h after value date       |
| Dispute lifecycle transitions (Visa example)     | per Visa Core Rules windows  |

Tighter SLAs negotiable; loosening requires operational sign-off.

## Annex B — Decommissioning (informative)

When a deployment is decommissioned:

1. Outstanding instructions are either settled, returned, or
   recorded as undeliverable with structured reasons
2. Open disputes transfer to the receiving custodian under a
   bilateral handover signed by both parties
3. Card-network connections are terminated per the network's
   off-boarding procedures; merchant accounts migrate per the
   acquirer's transfer procedures
4. Final daily root is sealed and the chain is exported to the
   receiving custodian

The decommissioning manifest is itself an audit event in the final
chain root, signed by both outgoing and incoming custodians.

## Annex C — Common pitfalls (informative)

- **UETR collision** — UETRs are UUIDs and collisions are
  cryptographically improbable, but a faulty UUID generator that
  reuses values would surface as `urn:wia:pay:problem:duplicate-uetr`.
  The deployment SHOULD audit its UUID generator at provisioning
- **Time skew across rails** — RTGS (millisecond), instant payment
  (sub-second), card clearing (second), ACH (clearing day): the
  deployment's time discipline must satisfy the tightest rail in
  use, not the average
- **Currency-fractional drift** — a producer treating KRW as 2-decimal
  introduces a × 100 amount inflation; boundary's `malformed-iso20022`
  catches the well-formed-amount mismatch but not all such bugs
- **Sanctions-list cache staleness** — sanctions lists update daily
  to weekly; the deployment's screening cache MUST refresh at least
  every 24h to avoid releasing instructions that the latest list
  would have caught
- **Token-vault scaling** — a token vault that exceeds its provisioning
  size silently denies new token issuance; deployments SHOULD monitor
  token-vault utilisation and provision capacity proactively

## Annex D — Settlement reconciliation (informative)

Daily settlement reconciliation flows:

1. The clearing system publishes a settlement file at end-of-cycle
2. The deployment fetches the file, validates the signature, and
   reconciles each line against the boundary's transaction records
3. Discrepancies (line in settlement file but no matching record;
   record marked settled but no settlement-file entry) surface as
   reconciliation incidents for operations review
4. The reconciled settlement file plus the boundary's records
   together feed the core-banking nightly close

A reconciliation discrepancy that persists past the deployment's
SLA escalates to the clearing-system's operations contact.
