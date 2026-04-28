# WIA-financial-data-exchange PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-financial-data-exchange
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-financial-data-exchange. The standard
covers persistent record shapes for the lifecycle of
a financial-data exchange operator — the institution
identification record (LEI + BIC + jurisdictional
identifiers); the customer / counterparty record;
the account-and-instrument record (IBAN + ISIN +
RIC); the message-and-transaction record under ISO
20022 + FIX 5.0 SP2 + SWIFT MT/MX; the open-banking
account-information / payment-initiation record
under PSD2 / PSD3 / Open Banking UK / FDX; the
derivative-trade record under FpML; the corporate-
actions record; the consent record under PSD2 SCA +
KR 마이데이터 + US §1033; the audit-and-traceability
record; and the supervisory and oversight
correspondence record. Records are consumed by the
operator's institutional clients (banks, asset
managers, fintech aggregators), the operator's
internal compliance-and-risk function, the operating
jurisdiction's supervisory authority (US SEC + CFTC
+ FRB + OCC + CFPB; EU EBA + ESMA + ECB + Member-
State NCAs; UK FCA + PRA; KR FSC + FSS + FIU; ISO
20022 Registration Authority through SWIFT), and
external auditors.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- ISO 20022 (Universal financial industry message
  scheme — the operator publishes the per-business-
  area message implementation guidelines): pacs
  (Payments Clearing and Settlement), pain
  (Payments Initiation), camt (Cash Management),
  acmt (Account Management), reda (Reference Data),
  remt (Remittance Advice), seev (Securities
  Events), semt (Securities Management), setr
  (Securities Trade), tsmt (Trade Services
  Management), auth (Authorities), head
  (Application Header)
- ISO 20022 message versions through 2024 release
  (the SWIFT MT-to-ISO-20022 industry coexistence
  cutover programme governs migration deadlines)
- ISO 9362 (Business Identifier Code, BIC)
- ISO 13616 (IBAN)
- ISO 17442:2020 (Legal Entity Identifier, LEI)
- ISO 6166 (Securities Identification Numbering
  System, ISIN)
- ISO 10962 (Classification of Financial Instruments,
  CFI)
- ISO 11649 (RF Creditor Reference)
- ISO 4217 (Currency codes)
- ISO 3166-1 (Country codes)
- ISO 10383 (Market Identifier Code, MIC)
- ISO 18774 (Financial Instrument Short Name, FISN)
- FIX 5.0 SP2 + FIX FAST + FIXatdl 1.1 + FIX
  Orchestra (the operating wire format for trade
  execution and post-trade reporting; ITCH /
  OUCH / SoupBin native protocols layered above)
- SWIFT MT messages (MT 103 / 202 / 202 COV / 304
  / 320 / 540 / 541 / 542 / 543 / 544 / 545 / 546
  / 547 / 548 / 549 / 564 / 566 / 567 / 568 / 900
  / 910 / 940 / 950) and SWIFT MX (the SWIFTNet
  InterAct ISO 20022 wrapper)
- SWIFT GPI (Global Payments Innovation) tracker +
  CCT Inst customer-credit-transfer-instant
- FpML (Financial products Markup Language) 5.x
  for OTC derivatives
- FDX (Financial Data Exchange) API specification
  v6.0 + FDX Common Standard (the US-led open-
  banking-and-finance API)
- EU PSD2 (Directive (EU) 2015/2366) + Commission
  Delegated Regulation (EU) 2018/389 RTS on Strong
  Customer Authentication and Common and Secure
  Open Standards of Communication
- EU PSD3 proposal + EU PSR (Payment Services
  Regulation) proposal (cited where the operator's
  forward-looking implementation roadmap addresses
  PSD3)
- EU EBA RTS on SCA + EBA Guidelines on PSD2 +
  EBA RTS on Strong Customer Authentication v2 +
  EBA Final Guidelines on outsourcing
- EU FIDA (Financial Data Access framework
  proposal) — cited as the operator's forward-
  looking horizon for non-payment data access
- UK Open Banking Implementation Entity (OBIE)
  Read/Write API specifications + UK Open Finance
  Long-Term Regulatory Framework
- US CFPB §1033 final rule (Personal Financial
  Data Rights, 2024) + 12 CFR Part 1033
- US FFIEC IT Examination Handbook
- KR 신용정보의 이용 및 보호에 관한 법률 + KR
  마이데이터 표준 API + KR 금융위원회 + 금융보안원
  (FSI) + KR 신용정보원 + 한국은행 BOK-Wire+
- ITU-T X.509 certificates + IETF RFC 5280 PKIX
- ISO 22301:2019 (Business continuity management)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts a financial-data-exchange operator (a bank,
a payment service provider, an account information
service provider AISP, a payment initiation service
provider PISP, a fintech aggregator, an institutional
exchange, a market-data vendor, a corporate-treasury
operator) maintains:

- The institution identification record.
- The customer / counterparty record.
- The account-and-instrument record.
- The ISO 20022 / FIX / SWIFT message record.
- The open-banking AISP / PISP record.
- The FpML derivative-trade record.
- The corporate-actions record.
- The consent and SCA record.
- The audit-and-traceability record.
- The supervisory-correspondence record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("bank-credit-
                       institution" | "payment-
                       service-provider-psp" |
                       "account-information-service
                       -provider-aisp" | "payment-
                       initiation-service-provider
                       -pisp" | "card-issuer" |
                       "card-acquirer" | "asset-
                       manager" | "broker-dealer" |
                       "investment-bank" |
                       "exchange-trading-venue" |
                       "ccp-clearing" | "csd-
                       settlement" | "fmi-other" |
                       "fintech-aggregator" |
                       "market-data-vendor" |
                       "corporate-treasury" |
                       "user-defined")
operatorBic          : string (ISO 9362)
operatorLei          : string (ISO 17442:2020)
operatorJurisdiction : array of string (ISO 3166-1)
governingFrameworks  : array of enum ("ISO-20022"
                       | "ISO-9362-BIC" |
                       "ISO-13616-IBAN" |
                       "ISO-17442-LEI-2020" |
                       "ISO-6166-ISIN" |
                       "ISO-10962-CFI" |
                       "ISO-11649-RF" |
                       "ISO-10383-MIC" |
                       "ISO-18774-FISN" |
                       "FIX-5-0-SP2" |
                       "FIX-FAST" | "FIX-ATDL" |
                       "FIX-ORCHESTRA" |
                       "SWIFT-MT-LEGACY" |
                       "SWIFT-MX-INTERACT" |
                       "SWIFT-GPI" |
                       "SWIFT-MT-TO-ISO20022-
                       COEXISTENCE" |
                       "FPML-5-X" |
                       "FDX-API-V6" |
                       "EU-PSD2-2015-2366" |
                       "EU-PSD2-RTS-SCA-2018-389" |
                       "EU-EBA-OUTSOURCING" |
                       "EU-PSD3-PROPOSAL" |
                       "EU-FIDA-PROPOSAL" |
                       "UK-OBIE-RW-API" |
                       "UK-OPEN-FINANCE-LTRF" |
                       "US-CFPB-1033-2024" |
                       "US-FFIEC-IT-EXAM" |
                       "KR-신용정보법" |
                       "KR-마이데이터-표준-API" |
                       "KR-FSC-FSS-FSI" |
                       "ITU-T-X-509" |
                       "ISO-22301-2019-BCMS" |
                       "user-defined")
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Customer / Counterparty Record

```
partyRecord:
  partyId            : string (uuidv7)
  partyKind          : enum ("natural-person" |
                       "legal-entity" | "trust" |
                       "fund" | "government-
                       authority" | "other")
  legalName          : string
  lei                : string (ISO 17442; required
                       for legal-entity counter-
                       parties)
  jurisdiction       : string (ISO 3166-1)
  taxIdentifier      : object (per-jurisdiction
                       identifier — US TIN / EU
                       VAT / EORI / KR 사업자등록
                       번호 / KR 주민등록번호
                       encrypted at rest)
  address            : object
  pepStatus          : enum ("not-pep" | "domestic-
                       pep" | "foreign-pep" |
                       "international-organization-
                       pep" | "unknown")
  sanctionsScreeningRef : string (the operator's
                       sanctions-screening record
                       reference)
```

## §4 Account-and-Instrument Record

```
accountRecord:
  accountId          : string (uuidv7)
  iban               : string (ISO 13616; absent
                       for non-IBAN jurisdictions)
  accountIdentifier  : string (jurisdictional
                       account identifier)
  accountKind        : enum ("current-payment" |
                       "savings" | "card-funding"
                       | "loan" | "investment-
                       custody" | "investment-
                       cash" | "trust" | "user-
                       defined")
  currency           : string (ISO 4217)
  accountServicerBic : string (ISO 9362 of the
                       account-servicing institution)
  ownerPartyRef      : string (PHASE-1 §3)

instrumentRecord:
  instrumentId       : string (uuidv7)
  isin               : string (ISO 6166; required
                       for transferable securities)
  cfi                : string (ISO 10962)
  fisn               : string (ISO 18774)
  marketIdentifier   : string (ISO 10383 MIC for
                       the listing venue)
  instrumentName     : string
  issuerLei          : string
  currency           : string (ISO 4217)
```

## §5 ISO 20022 / FIX / SWIFT Message Record

```
messageRecord:
  messageId          : string (uuidv7)
  messageBusinessArea : enum ("iso-20022-pacs-
                       payments-clearing-settlement"
                       | "iso-20022-pain-payments-
                       initiation" | "iso-20022-
                       camt-cash-management" |
                       "iso-20022-acmt-account-
                       management" | "iso-20022-
                       reda-reference-data" |
                       "iso-20022-remt-remittance-
                       advice" | "iso-20022-seev-
                       securities-events" |
                       "iso-20022-semt-securities-
                       management" | "iso-20022-
                       setr-securities-trade" |
                       "iso-20022-tsmt-trade-
                       services-management" |
                       "iso-20022-auth-authorities"
                       | "iso-20022-head" |
                       "fix-5-0-sp2-trade" |
                       "swift-mt-legacy" |
                       "swift-mx" | "fpml-otc-
                       derivatives" | "user-defined")
  messageId4DLetter  : string (e.g. "pacs.008.
                       001.13" for ISO 20022)
  direction          : enum ("inbound" | "outbound")
  counterpartyBic    : string
  sentAt             : string (ISO 8601)
  receivedAt         : string (ISO 8601)
  cryptographicDigest : string (SHA-256 of the
                       canonical payload)
  payloadRef         : string (URI of the
                       canonical payload — encrypted
                       at rest)
```

## §6 Open-Banking AISP / PISP Record

```
openBankingRecord:
  recordId           : string (uuidv7)
  serviceKind        : enum ("aisp-account-
                       information" | "pisp-payment-
                       initiation" | "cbpii-card-
                       based-payment-instrument-
                       issuer" | "fdx-financial-
                       data-exchange" | "kr-
                       마이데이터-개인신용정보-전송"
                       | "user-defined")
  authorisationKind  : enum ("psd2-rts-redirect" |
                       "psd2-rts-decoupled" |
                       "psd2-rts-embedded-with-
                       app2app" | "fdx-oauth-2-1-
                       fapi-2" | "kr-마이데이터-
                       전송요구권" | "us-cfpb-1033-
                       authorized-data-provider" |
                       "user-defined")
  consentRef         : string (PHASE-1 §9)
  scopeOfAccess      : array of string (account-
                       set + permission-set)
  effectiveFrom      : string (ISO 8601)
  effectiveUntil     : string (ISO 8601; PSD2 RTS
                       Article 10 caps consent at
                       180 days unless renewed via
                       SCA)
```

## §7 FpML Derivative-Trade Record

```
fpmlTradeRecord:
  tradeId            : string (uuidv7)
  fpmlVersion        : enum ("fpml-5-11" | "fpml-
                       5-12" | "user-defined")
  productClass       : enum ("interest-rate-swap"
                       | "credit-default-swap" |
                       "fx-spot" | "fx-forward" |
                       "fx-swap" | "fx-option" |
                       "equity-swap" | "equity-
                       option" | "commodity-swap" |
                       "commodity-option" |
                       "structured-note" |
                       "user-defined")
  ccpClearedRef      : string (the CCP's clearing
                       account / position
                       reference; absent for
                       bilaterally-cleared trades)
  isdaMasterAgreementRef : string
  collateralCsaRef   : string (the ISDA Credit
                       Support Annex reference)
  uti                : string (Unique Trade
                       Identifier per CPMI-IOSCO)
  upi                : string (Unique Product
                       Identifier per ISO 4914)
  payloadRef         : string (URI of the FpML
                       trade payload)
```

## §8 Corporate-Actions Record

```
corporateAction:
  actionId           : string (uuidv7)
  affectedInstrumentRef : string
  actionKind         : enum ("dividend-cash" |
                       "dividend-stock" | "stock-
                       split" | "reverse-split" |
                       "rights-issue" | "stock-
                       distribution" | "merger" |
                       "tender-offer" | "interest-
                       payment" | "redemption" |
                       "user-defined")
  recordDate         : string (ISO 8601 date)
  exDate             : string (ISO 8601 date)
  paymentDate        : string (ISO 8601 date)
  iso20022SeevMessageRef : string (URI of the seev
                       message — typically seev.
                       031 / .032 / .033 / .034 /
                       .035 / .036 / .037 / .038)
```

## §9 Consent and Strong-Customer-Authentication Record

```
consentRecord:
  consentId          : string (uuidv7)
  partyRef           : string
  scaTriggered       : boolean
  scaFactorsApplied  : array of enum ("knowledge"
                       | "possession" | "inherence"
                       | "user-defined")
                       (PSD2 RTS Article 4 SCA
                       requires at least two
                       independent factors;
                       dynamic-linking per Article
                       5 for payment SCA)
  authenticationMethod : enum ("psd2-rts-redirect"
                       | "fido2-webauthn" |
                       "passkey" | "tokenised-
                       app-token" | "user-defined")
  capturedAt         : string (ISO 8601)
  expiresAt          : string (ISO 8601; PSD2 RTS
                       Article 10 180-day cap)
  withdrawalAt       : string (ISO 8601; absent
                       until withdrawn — PSD2
                       Article 64 customer right
                       to withdraw)
  exemptionApplied   : enum ("low-value-payment-
                       art-16" | "trusted-
                       beneficiaries-art-13" |
                       "recurring-transaction-art
                       -14" | "transaction-risk-
                       analysis-art-18" |
                       "corporate-payment-art-17"
                       | "n/a-no-exemption" |
                       "user-defined")
```

## §10 Audit-and-Traceability Record

```
auditEvent:
  eventId            : string (uuidv7)
  occurredAt         : string (ISO 8601 instant
                       with millisecond precision)
  eventKind          : enum ("message-sent" |
                       "message-received" |
                       "consent-captured" |
                       "consent-withdrawn" |
                       "sca-completed" |
                       "trade-executed" |
                       "settlement-completed" |
                       "data-access-by-aisp" |
                       "data-shared-via-fdx" |
                       "user-defined")
  actorRef           : string (the system identity
                       or the authenticated user)
  outcomeKind        : enum ("success" | "minor-
                       failure" | "serious-failure")
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
each of the records defined above for every customer,
account, instrument, message, and trade the operator
handles, satisfy the ISO 20022 message-version
discipline, exercise the PSD2 / PSD3 / Open Banking /
FDX / KR 마이데이터 / US §1033 obligations applicable
to the operator's role, and preserve the records
under the operating jurisdiction's recordkeeping
discipline (US SEC 17a-4 + FFIEC IT Exam retention;
EU UCC / PSD2 5-year + 10-year retention; KR
신용정보법 5-year retention).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-financial-data-exchange
- **Last Updated:** 2026-04-29
