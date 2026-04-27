# WIA-open-banking PHASE 1 — Data Format Specification

**Standard:** WIA-open-banking
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for open-banking
records: customer consent records, account-information access
profiles, payment-initiation records, fund-confirmation records,
strong-customer-authentication evidence, third-party-provider (TPP)
identity records, and the cross-references that bind these
together. The shape interoperates with PSD2 RTS, OBIE Open Banking
UK, KR 마이데이터 표준 API, and Berlin Group NextGenPSD2 so existing
TPP and ASPSP implementations adopt this PHASE without inventing
parallel data models.

References (CITATION-POLICY ALLOW only):
- EU PSD2 (Directive 2015/2366) and the EBA Regulatory Technical Standards on SCA and CSC
- OpenID Foundation FAPI 2.0 (Financial-grade API Security Profile, Baseline + Advanced)
- OpenID Connect Core 1.0
- IETF OAuth 2.0 (RFC 6749), OAuth 2.1 draft, RFC 7515 (JWS), RFC 7519 (JWT), RFC 7636 (PKCE), RFC 9101 (JAR), RFC 9126 (PAR), RFC 8705 (mTLS), RFC 8628 (Device Auth)
- OBIE Open Banking UK — Read/Write API Specification v3.x
- Berlin Group NextGenPSD2 XS2A Framework
- KR 금융위원회·금융보안원 마이데이터 표준 API v1.x (KFSC MyData Standard API)
- ISO 20022 — payment-initiation pacs profiles for downstream clearing
- ISO 13616 (IBAN), ISO 9362 (BIC), ISO 4217 (currency)

---

## §1 Scope

This PHASE applies to systems that participate in regulated open-
banking flows: account-information service providers (AISPs),
payment-initiation service providers (PISPs), card-based payment-
instrument-issuer service providers (CBPIIs / equivalent),
account-servicing payment-service providers (ASPSPs), and the
identity and consent infrastructure that connects them.

The standard is jurisdiction-aware: an implementation MUST declare
which open-banking regime its records originate under (EU PSD2,
UK OBIE, KR 마이데이터, BR Open Finance, AU CDR, JP 全銀協 JBA Standard
API, etc.) so downstream consumers apply the correct interpretation
of consent windows, SCA exemptions, and dispute pathways.

In scope: TPP registration, consent records, account-information
flow records, payment-initiation records, fund-confirmation, SCA
evidence, decoupled-authentication evidence, fraud-flag interaction
with consent. Out of scope: clearing / settlement (handled by
WIA-payment-system), card-side flows (handled by WIA-mobile-payment),
non-PSD2 banking APIs (out of regulatory regime).

## §2 TPP identity record

A TPP carries:

| Field                   | Source / Binding                                           |
|-------------------------|------------------------------------------------------------|
| `tppId`                 | URN of form `urn:wia:ob:tpp:<authority>:<id>`              |
| `legalName`             | TPP's registered legal name                                 |
| `licenceAuthority`      | issuing authority URN (e.g., FCA UK, BaFin DE, FSC KR)     |
| `licenceNumber`         | the regulator-issued licence ID                             |
| `licenceRoles[]`        | subset of {AISP, PISP, CBPII} authorised by the licence     |
| `eIDASCertificate`      | (EU/UK) PSD2-eIDAS Qualified Web Authentication Certificate (QWAC) and/or Qualified Seal Certificate (QSealC) per RTS Article 34 |
| `clientCertificateFp`   | mTLS client certificate fingerprint                         |
| `redirectURIs[]`        | allowed redirect URIs (validated at authorisation time)     |
| `softwareStatement`     | OIDC software-statement JWT (per OBIE Dynamic Client Registration) |
| `notificationEndpoints[]`| TPP's webhooks for asynchronous events (e.g., consent-revoked) |

A TPP whose licence is suspended or revoked has all its consents
suspended at the boundary; existing in-flight calls complete but
new operations are refused. The licence-status check is performed
at every call (cached at the deployment's policy interval).

## §3 Consent record

Every access to a customer's account flows under a consent. Consent
records carry:

- `consentId` — URN
- `customerRef` — pseudonymous customer identifier
- `tppRef` — TPP URN
- `aspsRef` — ASPSP URN (the bank holding the account)
- `consentScope` — closed enum: `account-information`,
  `payment-initiation`, `funds-confirmation`, `card-based-payment-
  instrument` (per PSD2 / OBIE)
- `permissions[]` — per-scope permissions (for AIS:
  `ReadAccountsBasic`, `ReadAccountsDetail`, `ReadBalances`,
  `ReadTransactionsBasic`, `ReadTransactionsDetail`,
  `ReadTransactionsCredits`, `ReadTransactionsDebits`,
  `ReadStandingOrdersBasic`, `ReadStatementsBasic`,
  `ReadDirectDebits`, `ReadProducts`, `ReadOffers`,
  `ReadPartyPSU`, `ReadParty`, `ReadParties`, `ReadScheduledPayments`)
- `transactionFromDateTime` / `transactionToDateTime` (AIS only) —
  consent window for transaction history
- `expirationDateTime` — consent expiry (PSD2 RTS Article 36(3): max 180 days for AIS without explicit re-confirmation)
- `authorisedAt` — RFC 3339 SCA completion timestamp
- `policy` — URI of the consent's privacy policy (per ISO 29184)

Consents are signed by the customer's SCA-bound credential; the
ASPSP records the signature alongside the consent so that audit
reconstruction confirms the customer authorised exactly the
declared scope and permissions.

## §4 Strong Customer Authentication evidence

Per PSD2 RTS, every payment-initiation and (every 180 days) every
account-information access flows under SCA evidence:

- `scaId` — URN
- `consentRef` — consent that this SCA authorised
- `customerRef` — customer identifier
- `factors[]` — the two factors used; closed enum drawn from PSD2
  RTS Article 4: `knowledge` (password, PIN, secret answer),
  `possession` (mobile device, hardware token, SIM-bound),
  `inherence` (biometric)
- `factorEvidence[]` — per factor: factor type, evidence reference
  (e.g., FIDO2 assertion ID for inherence/possession, OTP submission
  hash for possession-via-OTP)
- `dynamicLinking` — per Article 5: structured fields binding the
  authentication to the specific transaction (amount, payee) so a
  replay against a different transaction fails
- `scaTimestamp` — RFC 3339 with offset

Decoupled-authentication evidence (the customer authenticates on a
separate channel, e.g., banking app push notification) carries a
`decoupledChannelRef` field naming the channel.

## §5 Account-information access record

For each AIS API call:

- `accessId` — URN
- `consentRef` — consent
- `tppRef` — TPP making the call
- `requestedResources[]` — per-resource access (Account,
  Transaction, Balance, etc.)
- `accessTimestamp` — RFC 3339 with offset

The boundary applies the consent's permission gate before
returning data; resources outside the consent's permissions are
silently filtered (the ASPSP MUST NOT signal the existence of
unconsented data).

## §6 Payment-initiation record

A payment initiation carries:

- `pisId` — URN
- `consentRef` — consent (PIS scope)
- `paymentInstruction` — the underlying payment in ISO 20022 pain
  format (or OBIE OBWriteDomesticConsent / OBWriteDomesticScheduledConsent
  / OBWriteInternationalConsent for UK)
- `risk` — risk-information block (per OBIE: PaymentContextCode,
  MerchantCategoryCode, MerchantCustomerIdentification,
  DeliveryAddress for goods-delivery)
- `confirmationOfFundsRef` — (optional) reference to a CoF response
- `executionDateTime` — RFC 3339 with offset

The ASPSP routes the underlying instruction into the relevant
clearing rail (WIA-payment-system) and returns the resulting UETR
back to the PISP for status query.

## §7 Funds-confirmation record

A CBPII (or PISP performing pre-execution check) requests funds
confirmation:

- `cofId` — URN
- `consentRef` — consent (CoF scope)
- `instructedAmount` — currency + amount
- `availability` — `available` or `not-available`
- `cofTimestamp` — RFC 3339 with offset

CoF responses are *binary*: the ASPSP MUST NOT return the actual
balance. PSD2 RTS limits CoF responses to a yes/no answer.

## §8 Korean MyData specifics

For KR 마이데이터 deployments, the consent record carries
KFSC MyData-specific fields:

- `transmissionPurpose` — closed enum per KFSC Standard API
  (e.g., `이체-마이데이터`, `자산조회-마이데이터`, `종합자산관리`)
- `dataItems[]` — per-data-item permissions drawn from KFSC's
  data-item catalogue
- `dataPeriod` — per-data-item period (e.g., 1 year for transaction
  history)
- `정보주체동의서명` — customer's signed consent receipt per KFSC
  template
- `상호인증서` — cross-authentication certificate per KFSC PKI

KR consents have shorter default expiry windows (90 days) than EU
(180 days) per FSC guidance.

## §9 Notification record

Asynchronous notifications (consent revoked, transaction status
updated, fraud-flag promoted) flow as signed records to the TPP's
webhook:

- `notificationId` — URN
- `notificationType` — closed enum
- `subjectRef` — the consent / transaction / flag the notification
  is about
- `notificationTimestamp` — RFC 3339 with offset
- `signature` — JWS by the ASPSP

The TPP acknowledges receipt; un-acknowledged notifications retry
per the deployment's retry budget. Notifications are a one-way
event stream from ASPSP to TPP; the TPP does not write back over
the notification channel.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked example: AIS consent (informative)

A fully populated UK OBIE-style AIS consent record:

```json
{
  "consentId": "urn:wia:ob:consent:aspsp-bank-uk:c-91a7-2026-04-27",
  "customerRef": "urn:wia:mdp:subject:f4c2-9bd1-7a05-3e8e",
  "tppRef": "urn:wia:ob:tpp:fca-uk:tpp-7e2c",
  "aspsRef": "urn:wia:ob:aspsp:obie:bank-uk-001",
  "consentScope": "account-information",
  "permissions": [
    "ReadAccountsBasic", "ReadAccountsDetail",
    "ReadBalances",
    "ReadTransactionsBasic", "ReadTransactionsDetail", "ReadTransactionsCredits", "ReadTransactionsDebits",
    "ReadStandingOrdersBasic", "ReadDirectDebits"
  ],
  "transactionFromDateTime": "2025-10-27T00:00:00Z",
  "transactionToDateTime": "2026-04-27T23:59:59Z",
  "expirationDateTime": "2026-10-24T23:59:59Z",
  "authorisedAt": "2026-04-27T09:15:00+01:00",
  "policy": "https://bank-uk.example/privacy/openbanking-2026",
  "regime": "UK-OBIE-v3.1.10"
}
```

The consent is signed by the customer's SCA-bound credential (FIDO2
inherence + possession). The signature is held alongside the
consent record so audit reconstruction confirms exactly which
permissions were authorised at which time.

## Annex B — Conformance disclosure

Implementations declare per-section conformance in their published
capability document. Sections marked `partial` reference the
deployment policy explaining the gap; `excluded` carries a
justification citing the controlling jurisdiction's allowance. A
deployment that is `partial` or `excluded` on §3 (Consent), §4
(SCA), §6 (Payment initiation), §7 (Funds confirmation), or §8
(Korean MyData specifics, when applicable) is non-conformant
overall.

## Annex C — Cross-domain references (informative)

| Reference                 | Use site                                                    |
|---------------------------|-------------------------------------------------------------|
| WIA-payment-system        | clearing handoff for PIS-initiated payments                 |
| WIA-mobile-payment        | wallet-bound SCA via FIDO2/WebAuthn                          |
| WIA-medical-data-privacy  | medical-bill PIS payments referencing patient consent        |
| WIA-insurtech             | insurance-premium PIS subscription payments                  |

The boundary verifies the cross-domain reference exists at the
referenced standard's boundary before delivery.

## Annex D — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. Each regime's API
profile (OBIE v3.1.x, Berlin Group XS2A, KFSC MyData v1.x) bumps
independently from this PHASE; the deployment policy maps each
PHASE version to the regime profile it honours so TPPs know
what to expect. Deprecation enters a 12-month sunset window with
migration notes recorded in the audit chain.

## Annex E — Per-regime consent windows (informative)

| Regime               | AIS consent default expiry | PIS authorisation lifetime           | Re-confirmation cadence      |
|----------------------|----------------------------|--------------------------------------|------------------------------|
| EU PSD2              | 180 days max (RTS 36(3))   | per-instruction (single-use unless multi-PIS) | every 180 days for AIS |
| UK OBIE              | 90 days SCA cycle           | per-instruction or recurring         | every 90 days SCA-renewal    |
| KR 마이데이터         | 90 days default             | per-instruction                      | every 90 days re-consent     |
| BR Open Finance      | 12 months max               | per-instruction                      | per-instruction SCA          |
| AU CDR               | 12 months max               | n/a (data sharing only)              | per-CDR rules                |
| JP JBA Standard API  | per scheme                  | per-instruction                      | per JBA guideline             |

Deployments operating in multiple regimes hold separate consent
records per regime; cross-regime consent merging is explicitly
out of scope.

## Annex F — Cross-regime mapping (informative)

| WIA term              | EU PSD2 / Berlin Group       | UK OBIE                              | KR 마이데이터              |
|-----------------------|------------------------------|--------------------------------------|----------------------------|
| TPP                   | TPP (AISP/PISP/CBPII)        | TPP (AISP/PISP/CBPII)                | 본인신용정보관리회사       |
| ASPSP                 | ASPSP                        | ASPSP                                | 정보제공자 (financial institution) |
| Consent               | Consent                      | Consent (OBWriteConsent)             | 본인동의서                 |
| SCA                   | SCA (RTS Art. 4)             | SCA                                  | 강한 인증 (간편인증 / 공동인증서) |
| eIDAS QWAC            | required for mTLS            | required for mTLS                    | 상호인증서                 |
| eIDAS QSealC          | required for body-signing    | required for body-signing            | n/a (별도 서명체계)         |

## Annex G — Notification fidelity

Notification records reflect the ASPSP's view of the subject (consent or transaction). Discrepancies between notification and canonical record indicate either an ASPSP-side bug or a delivery-injection attack; reconciliation surfaces the divergence.
