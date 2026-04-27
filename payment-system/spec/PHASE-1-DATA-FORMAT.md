# WIA-payment-system PHASE 1 — Data Format Specification

**Standard:** WIA-payment-system
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for retail and
wholesale payment-system records: account identifiers, payment
instructions, clearing and settlement records, dispute records,
sanctions screening evidence, fraud-flag records, and the cross-
references that bind these together. The shape interoperates with
ISO 20022 financial messaging so that an existing payment scheme,
RTGS, or instant-payments rail can adopt this PHASE without
inventing parallel data models.

References (CITATION-POLICY ALLOW only):
- ISO 20022 — Financial Services Universal Financial Industry message scheme
- ISO 20022 message catalogue — pacs.008 (FI-to-FI Customer Credit Transfer),
  pacs.009 (FI Credit Transfer), pacs.002 (Status Report), pacs.004 (Return),
  camt.053 (Bank-to-Customer Statement), camt.054 (Bank-to-Customer Debit/Credit Notification)
- ISO 13616 — IBAN; ISO 9362 — BIC; ISO 4217 — Currency code
- PCI DSS v4.0 — Payment Card Industry Data Security Standard
- PCI Software Security Framework — Secure Software Standard / Secure SLC
- EMVCo Book 2 / Book 3 — EMV Integrated Circuit Card specifications
- EMVCo Tokenisation Framework v2.x
- FATF Recommendations 10 (CDD), 16 (Wire Transfers), 32 (Cross-border)
- IETF RFC 7515 (JWS), RFC 8785 (JCS), RFC 9162 (Certificate Transparency 2.0 pattern)

---

## §1 Scope

This PHASE applies to systems that originate, route, clear, settle,
or report retail and wholesale payments: bank-to-bank credit
transfers, card-network authorisations and clearings, instant-
payment rails, payment-initiation services, and the controllers'
audit and compliance records arising from those flows. It addresses
the *shape* of records; transport protocols are addressed in
PHASE 3, integration with existing rails in PHASE 4.

The standard is jurisdiction-aware: an implementation MUST declare
which regulatory regime its records fall under (one of: BIS RTGS
member-bank rails, EU SEPA / TARGET2, US Fedwire / FedNow / NACHA,
KR HOFINET / 한은금융망 / KFTC 신속이체, JP BOJ-NET / Zengin, etc.)
so that downstream auditors apply the correct interpretation of
finality, return windows, and sanctions screening.

In scope: credit transfers (single and bulk), card authorisations
and clearings, refund records, sanctions evidence, fraud flags,
dispute and chargeback records. Out of scope: securities settlement
(addressed by ISO 20022 securities messages, separate WIA standard
in plan), cryptocurrency native protocols (separate WIA standard).

## §2 Account identifier model

Accounts carry a *structured* identifier:

| Identifier kind         | Source standard                                         |
|-------------------------|---------------------------------------------------------|
| IBAN                    | ISO 13616 — Europe / Saudi Arabia / others              |
| BIC (SWIFT)             | ISO 9362 — Bank Identifier Code                          |
| Domestic account number | per national scheme (KR 계좌번호, US ABA + DDA, JP 銀行・支店・口座) |
| Card primary account number (PAN) | ISO/IEC 7812 BIN + check-digit                |
| Tokenised PAN           | EMVCo Tokenisation Framework v2.x                       |
| Phone-number proxy      | ITU-T E.164 (used by some instant-payment rails)         |
| Alias                   | scheme-defined alias (e.g., KR 토스ID, JP J-Coin tag)    |

PAN-bearing fields are *in scope of PCI DSS v4* — they MUST be
truncated, masked, or tokenised in any record that crosses the
boundary into a non-PCI environment. This PHASE never carries a
full PAN in audit trails; the audit trail references the tokenised
PAN or the masked PAN ("************4242").

## §3 Payment instruction record (ISO 20022-aligned)

A payment instruction follows the ISO 20022 pacs.008 message profile:

| Field group           | Source / Binding                                                 |
|-----------------------|------------------------------------------------------------------|
| GroupHeader           | MessageId (UETR-bound), CreationDateTime, NumberOfTransactions   |
| CreditTransferTransactionInformation | Per-payment block:                                |
|   PaymentId           | InstructionId, EndToEndId, UETR (ISO 20022 unique end-to-end txn id), ClearingSystemReference |
|   InterbankSettlementAmount | currency (ISO 4217) + amount + fractional digits per currency |
|   ChargeBearer        | DEBT, CRED, SHAR, SLEV (ISO 20022 closed enum)                   |
|   Debtor / Creditor   | Name, postal address, identification (BIC/IBAN/local)            |
|   DebtorAgent / CreditorAgent | BIC + clearing-system-id where applicable                |
|   RemittanceInformation | structured (RemittanceLocation) or unstructured (free text)    |
|   PurposeOfPayment    | ExternalPurposeCode (ISO 20022 ExternalPurposeCode list)         |

The instruction is signed using JWS (RFC 7515) by the originating
institution. The detached signature is held alongside the ISO 20022
canonical XML/JSON so the message remains canonical for downstream
processing.

## §4 Currency and amount handling

Amounts are represented as fixed-point decimal with precision per
ISO 4217 fractional-digit rules:

| Currency | Fractional digits | Examples |
|----------|------------------|----------|
| KRW, JPY | 0                | ₩100,000 / ¥10,000 |
| USD, EUR, GBP, KRW… | 2     | $123.45 / €99.99 |
| BHD, KWD, OMR, JOD | 3     | three-digit-fractional |

Amounts MUST NEVER be stored as binary floats; the wire form is
ISO 20022 numeric (string of digits with a decimal-point position
defined by the currency). Currency mismatch between the
InterbankSettlementAmount currency and the InstructedAmount
currency is permitted only when an FX block (ExchangeRateInformation)
is present and signed.

## §5 UETR and end-to-end traceability

Every transaction carries a Universally-Unique End-to-End
Transaction Reference (UETR) — a 36-character UUID per ISO 20022.
The UETR is preserved across every leg of a multi-bank transfer
so that auditors can reconstruct the full lifecycle from origination
through settlement.

This PHASE binds the UETR to the boundary's audit chain: every
status change (accepted, accepted-for-settlement, settled, returned,
rejected, cancelled) emits an AuditEvent referencing the UETR.
Reconstruction of a transaction lifecycle is therefore a query
against AuditEvents filtered by UETR.

## §6 Card authorisation record (EMVCo / PCI DSS-aligned)

A card authorisation carries:

- `transactionId` — URN of form `urn:wia:pay:cardauth:<acquirer>:<seq>`
- `tokenisedPAN` — EMVCo token (PAN never stored in clear post-
  authorisation in the deployment's non-PCI scope)
- `panSuffix` — last four digits of the underlying PAN (clear text,
  permitted by PCI DSS for receipts)
- `cardScheme` — Visa, Mastercard, AmEx, JCB, Discover, UnionPay,
  RuPay, BC카드, KEB하나카드, etc.
- `merchantId` — acquirer-assigned MID
- `terminalId` — POS or e-commerce terminal identifier
- `mcc` — ISO 18245 Merchant Category Code
- `transactionType` — 00 (purchase), 01 (cash advance), 09 (purchase
  with cashback), 20 (refund) per ISO 8583 / EMV book 3
- `cvm` — Cardholder Verification Method (PIN, signature, no-CVM,
  CDCVM device-side biometric)
- `terminalEntryMode` — chip-and-PIN, contactless EMV, magstripe,
  manual, e-commerce CNP (ISO 8583 PoS Entry Mode)
- `authResult` — approved, declined, referral, partial-approval
- `authNetworkId` — ISO 8583 RRN / authorisation code (returned by
  the card network)

PAN-equivalent data (CVV2, full magstripe Track 2, PIN block) are
*never* persisted past the authorisation transaction — PCI DSS v4
Requirement 3 forbids it.

## §7 Sanctions screening evidence

Every cross-border instruction (and any domestic instruction that
exceeds the deployment's screening threshold) carries sanctions
screening evidence:

- `screeningId` — URN
- `instructionRef` — UETR or card transaction ID
- `screeningSource[]` — list of screened-against lists (UN, OFAC SDN,
  EU consolidated, UK HMT, KR 외환관리법 별표, JP MOF, etc.)
- `result` — clear, hit, partial-match
- `matchEvidence` — name normalisation method, fuzzy-match score
  band (none / low / moderate / high), reviewer disposition for
  hits / partial-matches
- `screeningTimestamp` — RFC 3339 with offset

Screening evidence is signed with the screening service's key.
A transaction without screening evidence at the time of release
MUST not be released; the boundary refuses unscreened cross-border
flows.

## §8 Fraud-flag record

Fraud-flag records bind to instructions or card authorisations:

- `flagId` — URN
- `subjectInstruction` — UETR or card transaction ID
- `flagType` — coded type (velocity-anomaly, geolocation-mismatch,
  device-anomaly, behavioural-anomaly, network-rule-flag, scheme-
  blacklist-hit)
- `severityBand` — informational, advisory, blocking
- `originPrincipal` — fraud engine URN
- `flagTimestamp` — RFC 3339 with offset

Flags are *advisory* until promoted to a block by the deployment's
fraud-policy engine. Promotions and demotions emit AuditEvents
linked to the flag and the underlying transaction.

## §9 Dispute and chargeback record

Disputes follow the relevant scheme's chargeback workflow with
state machine:

- `disputeId` — URN
- `subjectTransaction` — original card transaction ID
- `chargebackReasonCode` — scheme-defined reason code
- `state` — one of `inquiry`, `chargeback-issued`, `representment`,
  `pre-arbitration`, `arbitration`, `accepted`, `withdrawn`,
  `expired`
- `transitions[]` — each transition: from-state, to-state, timestamp,
  signing principal, evidence references
- `disputedAmount` — currency + amount (may be partial of original)

Disputes are append-only; revisions issue a new state-transition
record referencing the prior. The chain ties to the original
transaction's UETR so the full lifecycle (authorisation → clearing
→ dispute → resolution) is reconstructable.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked example: pacs.008 credit transfer (informative)

A fully populated cross-border credit transfer:

```json
{
  "messageType": "pacs.008.001.10",
  "groupHeader": {
    "messageId": "MSGID-2026-04-27-091500-0001",
    "creationDateTime": "2026-04-27T09:15:00+09:00",
    "numberOfTransactions": "1",
    "settlementInformation": {"settlementMethod": "INDA", "settlementAccount": {...}}
  },
  "creditTransfer": {
    "paymentId": {
      "instructionId": "INSID-91a7",
      "endToEndId": "E2E-7e2-91a7",
      "uetr": "550e8400-e29b-41d4-a716-446655440000",
      "clearingSystemReference": "BOK-WIRE-2026-04-27-91a7"
    },
    "interbankSettlementAmount": {"currency": "KRW", "amount": "1000000"},
    "interbankSettlementDate": "2026-04-27",
    "chargeBearer": "SHAR",
    "instructedAmount": {"currency": "USD", "amount": "750.00"},
    "exchangeRateInformation": {
      "exchangeRate": 1333.33,
      "rateType": "spot",
      "contractIdentification": "FX-91a7"
    },
    "debtor": {"name": "Hong Gildong", "identification": {"organisationId": {"other": [{"id": "880101-1******"}]}}},
    "debtorAccount": {"identification": {"iban": "GB29NWBK60161331926819"}},
    "debtorAgent": {"financialInstitutionIdentification": {"bicfi": "BANKKRSE"}},
    "creditor": {"name": "Acme Inc"},
    "creditorAccount": {"identification": {"iban": "DE89370400440532013000"}},
    "creditorAgent": {"financialInstitutionIdentification": {"bicfi": "DEUTDEFF"}},
    "remittanceInformation": {"unstructured": ["INV 2026-04-27-1234"]},
    "purposeOfPayment": {"code": "GDDS"}
  }
}
```

The boundary signs the canonical XML/JSON form (RFC 7515 detached
JWS). All FATF Rec 16 fields are present (originator name, account,
identification; beneficiary name, account); the boundary refuses
the instruction otherwise.

## Annex B — Conformance disclosure

Implementations declare per-section conformance in their published
capability document. Sections marked `partial` reference the
deployment policy explaining the gap; sections marked `excluded`
carry a justification citing the controlling jurisdiction's
allowance. A deployment that is `partial` or `excluded` on §3
(Payment instruction), §6 (Card authorisation), §7 (Sanctions),
or §9 (Dispute) is non-conformant overall and cannot claim Deep v3.

## Annex C — Currency-precision pitfalls (informative)

KRW and JPY are zero-fractional currencies; storing "1000.00" as
KRW is a malformed amount per ISO 4217. The boundary normalises
on ingest:

- KRW "1,000,000" → store as "1000000" (no decimal)
- JPY "10000" → store as "10000"
- USD "123.45" → store as "123.45" (two decimals)
- BHD "1.234" → store as "1.234" (three decimals)

A producer that emits a fractional-digit count inconsistent with
the currency is rejected with `urn:wia:pay:problem:malformed-iso20022`.

## Annex D — Cross-domain references (informative)

| Reference                       | Use site                                                |
|---------------------------------|---------------------------------------------------------|
| WIA-mobile-payment              | wallet-tokenised authorisation flowing through this PHASE |
| WIA-open-banking                | PSD2/FAPI Payment Initiation invoking PHASE 2 §1         |
| WIA-medical-data-privacy        | medical-bill insurance-payment integration               |
| WIA-insurtech                   | insurance-claim disbursement payouts                    |

The boundary verifies the cross-domain reference exists at the
referenced standard's boundary before delivery so downstream
readers do not see dangling references.

## Annex E — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. ISO 20022 message profiles bump independently from this PHASE; the deployment policy maps each PHASE version to the ISO 20022 profile version it honours so correspondent banks know what to expect. Deprecation enters a 12-month sunset window with migration notes recorded in the audit chain.
