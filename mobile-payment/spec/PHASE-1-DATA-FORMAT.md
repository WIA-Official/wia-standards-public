# WIA-mobile-payment PHASE 1 — Data Format Specification

**Standard:** WIA-mobile-payment
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for mobile-payment
records: wallet provisioning records, tokenised PAN bindings, NFC
contactless transaction records, HCE-emulation card data, QR-code
payment payloads, FIDO2 device attestations, transaction receipts,
and the cross-references that bind these together. The shape
interoperates with EMVCo Tokenisation, NFC Forum, and FIDO2 so
existing wallet implementations adopt this PHASE without rebuilding
data models.

References (CITATION-POLICY ALLOW only):
- EMVCo Tokenisation Framework v2.x — token request, provisioning, lifecycle
- EMVCo Contactless Specifications for Payment Systems Book A/B/C/D
- EMVCo Cloud-Based Payments / EMVCo Mobile Payments
- ISO/IEC 14443 — proximity card air-interface (NFC contactless)
- ISO/IEC 7816 — smart-card command/response APDU
- NFC Forum NDEF (NFC Data Exchange Format), Type 4 Tag Operation
- FIDO Alliance Universal Authentication Framework / FIDO2 (CTAP2 + WebAuthn)
- W3C Payment Request API; W3C Web Payments
- EMVCo QR Code Specification (Merchant-Presented Mode / Consumer-Presented Mode)
- ISO 20022 (referenced via WIA-payment-system for downstream clearing)
- IETF RFC 7515 (JWS), RFC 7517 (JWK), RFC 9162 (Certificate Transparency 2.0 pattern)

---

## §1 Scope

This PHASE applies to systems that originate or accept mobile
payments: wallet apps (Apple Pay, Google Wallet, Samsung Pay,
Naver Pay, Kakao Pay, J-Coin, Alipay+, etc.), Host Card Emulation
(HCE) wallets, NFC contactless POS terminals, QR-code merchant
acceptance, and FIDO2-secured device-bound payment authorisation.

In scope: wallet token lifecycle, contactless EMV transaction
records, HCE cryptogram records, QR-code payment payloads, FIDO2
device attestation evidence, on-device CDCVM (consumer device
cardholder verification method) records, transaction receipts.
Out of scope: clearing and settlement (handled by WIA-payment-system),
PSD2/FAPI flow (handled by WIA-open-banking), card-issuance and
card-network internals (handled by EMVCo specs).

The standard is wallet-platform-aware: a single implementation
MUST declare which wallet-platform certification scheme it
honours (Apple Pay, Google Pay, Samsung Wallet, KFTC Mobile FastPay,
JBA J-Coin, etc.) so downstream consumers know which trust roots
apply.

## §2 Wallet-account identifier model

A wallet account carries:

| Identifier kind   | Source / Binding                                                |
|-------------------|-----------------------------------------------------------------|
| `walletAccountRef`| URN of form `urn:wia:mp:wallet:<provider>:<opaque>` (≥ 128 bit entropy) |
| `tokenisedPAN`    | EMVCo token (Domain Restriction Controls bound to wallet)      |
| `panSuffix`       | last four digits of underlying PAN, permitted in clear         |
| `cardScheme`      | Visa, Mastercard, AmEx, JCB, BC카드, KEB하나카드, etc.          |
| `deviceRef`       | URN of the bound device (FIDO2 attestation key reference)      |
| `walletProvider`  | Apple, Google, Samsung, Naver, Kakao, J-Coin, etc.             |

Full PAN never crosses the boundary into the wallet's runtime
environment; the wallet holds the EMVCo token plus the per-device
domain restrictions. Re-binding (the cardholder receives a re-issued
card) issues a new token under EMVCo Tokenisation Framework §
"Token Lifecycle Management".

## §3 Wallet provisioning record

Wallet provisioning binds a card to a wallet on a specific device:

- `provisioningId` — URN
- `walletAccountRef` — wallet-account URN
- `tokenisedPAN` — EMVCo token issued by the network's TSP
- `tokenRequestor` — TR ID assigned by the network to the wallet
  provider (per EMVCo Tokenisation Framework §3.2)
- `deviceRef` — bound device URN
- `provisioningChannel` — one of `in-app-issuer`, `manual-pan-entry`,
  `push-from-issuer`, `network-pull`
- `idAndVbinding` — issuer's identification-and-verification (ID&V)
  result band: `green` (low risk, frictionless), `yellow` (challenge
  required), `red` (declined)
- `cardholderConsent` — signed consent record naming the wallet,
  the device, and the card
- `provisioningTimestamp` — RFC 3339 with offset

Each provisioning emits an audit record. Re-provisioning (token
rotation, device replacement) creates a new provisioning record
referencing the prior; the prior is preserved for audit.

## §4 Contactless EMV transaction record

NFC contactless transactions follow EMVCo Contactless Specifications:

- `transactionId` — URN
- `walletAccountRef` — issuing wallet
- `tokenisedPAN` — token used
- `cryptogram` — Application Cryptogram (ARQC for online, TC for
  offline-approved); Cryptogram Information Data (CID) per EMV Book 3
- `atc` — Application Transaction Counter (per EMV Book 3 §6.5.5);
  monotonically increasing per token
- `terminalEntryMode` — `contactless-emv-chip` (the only one this
  PHASE recognises for tokenised flows)
- `transactionType` — 00 (purchase), 09 (purchase-with-cashback),
  20 (refund) per ISO 8583
- `cvm` — `cdcvm` (on-device biometric/PIN), `no-cvm` (low-value),
  `signature` (rare on contactless), or `online-pin` (for ATM)
- `terminalRef` — POS terminal URN
- `merchantId` — acquirer-assigned MID
- `mcc` — ISO 18245 Merchant Category Code
- `amount` — ISO 4217 currency + amount
- `transactionTimestamp` — RFC 3339 with offset

Cryptogram + ATC together prove transaction freshness; replay
attempts (ATC reuse) are rejected at the issuer.

## §5 Host Card Emulation (HCE) record

HCE wallets emulate a payment card in software on the mobile
device. HCE-specific fields:

- `hceMode` — `cloud-based` (per EMVCo CBP) or `device-resident`
- `tokenDomainRestriction` — `single-token` (one token per
  transaction) or `limited-use-key` (a key valid for a small
  number of transactions before refresh)
- `keyRefreshRef` — URN of the most recent key-refresh event from
  the TSP
- `appletId` — for SE-resident applets, the applet's AID; for HCE,
  the routing AID

HCE flows that exhaust their limited-use-key budget without
refresh from the TSP are refused; the wallet surfaces a
"refresh-required" state to the cardholder.

## §6 QR-code payment payload (EMVCo QRCPS)

QR-code payments use the EMVCo Merchant-Presented Mode (MPM) or
Consumer-Presented Mode (CPM) format:

- **MPM**: merchant displays a static or dynamic QR; the wallet
  scans, builds a transaction, and submits to the acquirer
- **CPM**: wallet displays a one-time QR containing the tokenised
  PAN + cryptogram; merchant scans and submits

The payload follows EMVCo QR Code Specification with TLV (tag-
length-value) encoding. Mandatory tags: 00 (Payload Format
Indicator), 52 (Merchant Category Code), 53 (Currency), 54
(Amount, optional for static QR), 58 (Country Code), 59 (Merchant
Name), 60 (Merchant City), 63 (CRC checksum). Dynamic QRs include
tag 62 (Additional Data Field Template) for transaction-specific
fields.

CRC is CCITT-CRC16 over the TLV bytes excluding the CRC tag itself.
A QR with incorrect CRC is refused at the wallet.

## §7 FIDO2 device attestation record

Device attestation binds a wallet account to a specific device:

- `attestationId` — URN
- `walletAccountRef` — wallet account
- `deviceRef` — device URN
- `attestationFormat` — one of `packed`, `tpm`, `android-key`,
  `android-safetynet`, `apple`, `apple-app-attest` (per W3C
  WebAuthn / FIDO2 attestation statement formats)
- `aaguid` — Authenticator Attestation GUID (per WebAuthn)
- `publicKey` — COSE-encoded public key
- `signature` — attestation signature over the wallet+device binding
- `attestationCertChain[]` — X.509 chain to a recognised root
  (Apple, Google, Microsoft, vendor)
- `attestationTimestamp` — RFC 3339 with offset

Re-attestation (device firmware update, jailbreak/root remediation)
issues a new attestation referencing the prior. An attestation
chain that fails verification fails the wallet binding; the wallet
account is suspended pending review.

## §8 CDCVM (on-device verification) record

CDCVM (Consumer Device Cardholder Verification Method) — biometric
or device-PIN performed on-device — produces a record:

- `cdcvmId` — URN
- `walletAccountRef` — wallet account
- `cdcvmMethod` — one of `face`, `fingerprint`, `iris`, `device-pin`,
  `device-passcode`
- `liveness` — boolean (whether liveness check passed; required for
  biometric methods on supported platforms)
- `cdcvmTimestamp` — RFC 3339 with offset

CDCVM records are device-resident only; the boundary records that
CDCVM occurred (with method and result) but never receives the
biometric template.

## §9 Transaction receipt

Each completed transaction emits a receipt for the cardholder:

- `receiptId` — URN
- `transactionRef` — transaction URN
- `merchantName`, `merchantCity`
- `amount` — currency + amount
- `panSuffix` — last four digits in clear (PCI DSS-permitted)
- `transactionTimestamp` — RFC 3339 with offset
- `acquirerReference` — RRN from the network
- `signature` — JWS by the wallet provider

Receipts are derived projections; the canonical record is the
transaction (PHASE 1 §4). Receipt forgery would not match the
canonical record, so audit reconstruction surfaces forgery.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked example: Apple Pay contactless transaction (informative)

```json
{
  "transactionId": "urn:wia:mp:tx:apple:91a7-2026-04-27",
  "walletAccountRef": "urn:wia:mp:wallet:apple:opaque-7e2c-...",
  "tokenisedPAN": "TOK-VTS-V-...",
  "panSuffix": "4242",
  "cardScheme": "visa",
  "cryptogram": "3F8A21B0E40C2D9E",
  "cid": "80",
  "atc": 1234,
  "terminalEntryMode": "contactless-emv-chip",
  "transactionType": "00",
  "cvm": "cdcvm",
  "amount": {"currency": "KRW", "amount": "12000"},
  "merchantId": "M-7e2c",
  "terminalRef": "urn:wia:mp:terminal:t-91a7",
  "transactionTimestamp": "2026-04-27T09:31:14+09:00",
  "metadata": {
    "wallet": "apple-pay",
    "deviceModel": "iPhone (Secure Enclave)",
    "tspNetwork": "Visa Token Service",
    "cdcvmMethod": "face"
  }
}
```

The cryptogram is derived from the tokenised PAN (DPAN), the
limited-use key, the ATC, and the transaction-context inputs per
EMVCo Book 2 cryptogram derivation. The issuer verifies on receipt;
replay (ATC reuse) is rejected.

## Annex B — Conformance disclosure

Implementations declare per-section conformance in their published
capability document. Sections marked `partial` reference the
deployment policy explaining the gap; `excluded` carries a
justification citing the wallet platform's allowance. A deployment
that is `partial` or `excluded` on §3 (Provisioning), §4
(Contactless EMV), §7 (FIDO2 attestation), or §9 (Receipt) is
non-conformant overall and cannot claim Deep v3 status.

## Annex C — Cross-domain references (informative)

| Reference                      | Use site                                                     |
|--------------------------------|--------------------------------------------------------------|
| WIA-payment-system             | clearing handoff post-authorisation (PHASE 4 §7)              |
| WIA-open-banking               | PSD2 Payment Initiation invoking wallet authentication        |
| WIA-medical-data-privacy       | health-FSA / HSA card-bound mobile payments                   |
| WIA-public-transportation      | transit-pass binding to wallet (Korea T-money in wallet)      |

The boundary verifies the cross-domain reference exists at the
referenced standard's boundary before delivery so downstream readers
do not see dangling references.

## Annex D — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. EMVCo specifications
bump independently from this PHASE; the deployment policy maps each
PHASE version to the EMVCo Books / Tokenisation Framework version
it honours. Deprecated PHASE versions enter a 12-month sunset
window with migration notes recorded in the audit chain. Legacy
tokens issued under deprecated versions remain verifiable through
the prior key in the JWKS for the full token lifecycle window.

## Annex E — Platform-specific notes (informative)

- **Apple Pay** stores tokens in the Secure Enclave; CDCVM is Touch
  ID, Face ID, or device passcode; cryptograms are generated by
  Apple's Secure Element pathway
- **Google Wallet** stores tokens in StrongBox where available
  (Android 9+) or Keystore otherwise; CDCVM is fingerprint, face,
  or device PIN
- **Samsung Wallet** stores tokens in Knox-protected storage; CDCVM
  is fingerprint, face, iris (where available), or device PIN
- **Naver / Kakao Pay** use platform-specific HCE wallets gated by
  KISA-recognised attestation roots; CDCVM follows the platform's
  biometric stack

## Annex F — Receipt fidelity

Receipts SHOULD reproduce all consumer-visible transaction details. A receipt that diverges from the canonical record (PHASE 1 §4) indicates either a wallet-side bug or a forgery; reconciliation surfaces such divergence.
