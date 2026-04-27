# WIA-mobile-payment PHASE 3 — Protocol Specification

**Standard:** WIA-mobile-payment
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format (PHASE 1)
to the API surface (PHASE 2): NFC contactless air-interface,
HCE routing, FIDO2 device-binding handshake, EMVCo Tokenisation
provisioning protocol, the device-side cryptogram generation flow,
QR-code consumer/merchant presentation flows, and the cross-domain
handoff to the clearing layer (WIA-payment-system).

References (CITATION-POLICY ALLOW only):
- ISO/IEC 14443 — proximity card air-interface
- ISO/IEC 7816-4 — smart-card APDU command/response
- EMVCo Contactless Specifications Books A/B/C/D
- EMVCo Cloud-Based Payments — limited-use-key model
- EMVCo Tokenisation Framework v2.x — token request, provisioning
- W3C WebAuthn Level 3 / FIDO2 CTAP2.1 — registration and authentication
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 9162 (Certificate Transparency 2.0 pattern)
- NFC Forum Type 4 Tag Operation Specification

---

## §1 Authentication

Wallet apps and merchant terminals authenticate to the boundary
using mTLS (TLS 1.3 client certificates) plus signed message
payloads:

- mTLS authenticates the *connection*: wallet provider's certificate
  identifies the wallet, terminal vendor's certificate identifies
  the terminal
- JWS authenticates the *message*: each transaction submission is
  signed by the principal that constructed it (wallet on the device,
  acquirer on the terminal-side)

Token-based bearer auth is permitted for non-sensitive operations
(receipt retrieval, capability discovery).

## §2 Token format and signing

Tokens are JWS-signed JWTs (RFC 7515 + RFC 7519). Default signature
algorithm is ES256 (P-256 ECDSA). FIDO2 attestation signatures use
the algorithm declared in the COSE-encoded public key (typically
ES256, RS256, or EdDSA Ed25519 per device).

EMVCo cryptograms (ARQC, TC) are produced by the issuer-controlled
crypto material on the wallet, not by the boundary. The boundary
only carries the cryptogram and ATC values to the network for
verification.

## §3 NFC contactless air-interface

NFC contactless transactions use ISO/IEC 14443 Type A or Type B
air-interface at 13.56 MHz, with EMVCo Books A–D defining the
contactless EMV protocol on top:

- Anti-collision and PICC selection per ISO 14443-3
- T=CL transmission protocol per ISO 14443-4
- EMV SELECT AID, GET PROCESSING OPTIONS (GPO), READ RECORD,
  GENERATE AC commands per EMV Book 3
- Issuer / wallet decides cryptogram type (ARQC online or TC offline)
  based on terminal risk-management parameters and the wallet's
  Card Risk Management

The boundary does not see the air-interface; it sees the resulting
cryptogram + ATC + transaction context delivered by the terminal-
side (acquirer) submission.

## §4 HCE routing

HCE wallets register a routing AID (per Android Card Emulation
"Host-based Card Emulation" architecture) so NFC-routed APDUs reach
the wallet app on the device. The wallet processes APDUs in
software, generates the cryptogram from a limited-use key
(per EMVCo CBP), and returns the response to the terminal.

Limited-use keys are short-lived: typically a budget of ≤ 50
transactions or ≤ 7 days, whichever comes first, before refresh
from the TSP. The wallet refuses transactions when the budget is
exhausted; the cardholder UI surfaces "refresh required".

For SIM/eSIM-resident or eSE-resident applets (legacy SIM-based
mobile wallets, KR USIM-NFC), the applet implements the contactless
EMV protocol directly without HCE routing; the cryptogram is
generated inside the secure element using a permanent issuer-
controlled key.

## §5 FIDO2 device-binding handshake

FIDO2 registration follows W3C WebAuthn Level 3:

1. Wallet provider's relying party (RP) initiates registration:
   `navigator.credentials.create()` with attestation request
2. Authenticator (TPM, Apple Secure Enclave, Android StrongBox,
   FIDO U2F key) generates a key pair, signs an attestation
   statement over the wallet+device binding
3. Wallet posts the attestation to PHASE 2 §6
4. Boundary verifies the attestation chain to a recognised root
5. On success, the wallet account is bound to the device

Subsequent authentication (PHASE 2 §7 CDCVM) uses
`navigator.credentials.get()` against the registered credential.
The boundary verifies the resulting assertion against the public
key recorded at registration.

## §6 EMVCo Tokenisation provisioning protocol

Token provisioning follows EMVCo Tokenisation Framework §3:

1. Wallet (token requestor, TR) sends a token request to the network's
   TSP via the boundary, including the underlying PAN (encrypted
   to the TSP) and the ID&V evidence
2. TSP performs ID&V (often via the issuer's API), assigns an ID&V
   band (green / yellow / red), and on green/yellow returns a
   token bound to the wallet+device
3. Boundary persists the provisioning record (PHASE 1 §3) and
   returns the token reference to the wallet
4. Wallet stores the token + Domain Restriction Controls in
   device-local secure storage (Secure Enclave, StrongBox, eSE)

Yellow ID&V results require challenge (issuer-initiated OTP, in-app
issuer call, etc.) before the token is provisioned.

## §7 Time discipline

Wallet device clocks synchronise via the device's standard NTP
configuration. Drift exceeding 30 s suspends FIDO2 attestation
(timestamps in attestation statements must be fresh) and CDCVM
verification (replay protection). The boundary timestamps each
transaction with its own authoritative clock and reconciles drift
with the device's reported timestamp.

## §8 QR-code presentation flow (CPM and MPM)

Consumer-Presented Mode:
1. Wallet generates a one-time QR per EMVCo QRCPS (PHASE 1 §6) with
   the tokenised PAN + per-transaction cryptogram
2. Merchant scans, builds a contactless-equivalent transaction record,
   submits to the acquirer
3. Acquirer routes to the network for online authorisation

Merchant-Presented Mode:
1. Merchant displays a static or dynamic QR
2. Wallet scans, validates the CRC and merchant identity, constructs
   a transaction with the merchant's MID + amount + tokenised PAN
3. Wallet submits to the boundary via PHASE 2 §3

QR-code payments inherit the same cryptogram and ATC discipline as
NFC contactless; replay (ATC reuse) is rejected at the issuer.

## §9 Failure modes

| Failure                                | Behaviour                                              |
|----------------------------------------|--------------------------------------------------------|
| TSP unreachable                        | Wallet provisioning queued; in-flight tokens continue   |
| FIDO2 attestation chain unrecognised   | Wallet binding refused                                  |
| Device firmware modified post-attestation | Re-attestation required; existing transactions allowed until next CDCVM |
| Limited-use-key budget exhausted       | Wallet surfaces "refresh required"; transactions refused |
| ATC replay detected                    | Transaction refused at issuer                           |
| Audit chain write failure              | Operation rejected (consistency w/ payment-system §9)   |
| QR CRC mismatch                        | Wallet refuses scan                                     |
| Time drift > 30 s                      | FIDO2 + CDCVM operations refused                         |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

| Concern                  | Default                                | Notes                              |
|--------------------------|----------------------------------------|------------------------------------|
| Token signing (boundary) | ES256 (P-256 ECDSA)                    | EdDSA Ed25519 also accepted        |
| WebAuthn / FIDO2 sig     | per device's COSE key                  | typically ES256 / RS256 / Ed25519  |
| EMV cryptogram           | per network (Visa: VAC; Mastercard: M/Chip) | cryptographic suite per network |
| TLS                      | 1.3 (RFC 8446)                         | 1.2 explicit-list only             |
| Symmetric at rest        | AES-256-GCM                            |                                    |

## Annex B — Wallet-platform attestation roots (informative)

| Platform              | Root authority                                                  |
|-----------------------|-----------------------------------------------------------------|
| Apple                 | Apple Inc Root CA (attestation), Apple App Attest                |
| Google                | Google Hardware Attestation Roots, SafetyNet (deprecated)        |
| Microsoft             | Microsoft TPM Root Authority                                    |
| Samsung               | Samsung Knox Attestation Root                                    |
| Vendor-specific TPM   | vendor-issued root in the FIDO Metadata Service                  |
| KR domestic           | KISA-recognised KR domestic roots                                |

The deployment maintains the recognised-roots list and refreshes
quarterly; root revocations are surfaced as security incidents.

## Annex C — HCE limited-use-key budget worked example (informative)

A typical HCE wallet on Android:

- Refresh budget: 50 transactions or 7 days, whichever first
- On each transaction the budget decrements
- When budget is at 5, the wallet proactively refreshes from TSP
- When budget is exhausted before refresh succeeds, transactions
  refused; cardholder UI shows "refresh required"

The 5-transaction headroom allows the wallet to recover from
intermittent network without cardholder impact. The headroom value
is per-deployment configurable.

## Annex D — Cross-domain references (informative)

| Reference                     | Use site                                                |
|-------------------------------|---------------------------------------------------------|
| WIA-payment-system            | clearing handoff for authorised transactions             |
| WIA-open-banking              | PSD2 Payment Initiation invoking wallet authentication   |
| WIA-public-transportation     | transit-pass binding (Korea T-money in wallet)           |
| WIA-medical-data-privacy      | HSA/FSA card-bound mobile payments                       |

The boundary verifies the cross-domain reference exists at the
referenced standard's boundary before delivery.

## Annex E — Time-precision worked example (informative)

For mobile-payment timestamps:

- Wallet-side transactions: millisecond precision
- FIDO2 attestation: second precision (per WebAuthn spec)
- CDCVM events: millisecond precision
- Receipt timestamps: second precision (consumer-readable)

Drift between wallet device clock and boundary clock greater than
30 s causes FIDO2 + CDCVM operations to refuse; the deployment's
acceptance window is bounded by the most-restrictive operation in
the call chain.

## Annex F — EMVCo cryptogram derivation (informative)

The application cryptogram (ARQC for online; TC for offline-approved)
is derived per EMVCo Book 2:

1. The terminal sends GET PROCESSING OPTIONS (GPO) with terminal
   transaction data (amount, terminal country code, transaction
   type, etc.)
2. The wallet (or SE applet) collects a CDOL1 input string per the
   issuer-defined Card Risk Management
3. Cryptogram input = SAD || ATC || UN || CDOL1 (where SAD = static
   application data, UN = unpredictable number from terminal)
4. Cryptogram = MAC over input under the issuer-controlled DAK
   (Derived ARQC Key) via 3DES or AES per the network's profile
5. Wallet returns ARQC + CID + ATC in the GENERATE AC response

Issuer verifies ARQC at authorisation time; replay protection comes
from the ATC counter, freshness from the unpredictable number.

## Annex G — Conformance levels (informative)

| Level     | Scope                                                           |
|-----------|-----------------------------------------------------------------|
| Surface   | data formats accepted; self-attested; no audit transparency     |
| Verified  | annual third-party audit (PCI DSS QSA + EMVCo certification)    |
| Anchored  | continuous evidence package per audit chain transparency        |

Implementations declare their level in PHASE 2 Annex D (capabilities).
Wallet platforms typically require Verified or Anchored from the
issuing institution before accepting their cards into the wallet.
