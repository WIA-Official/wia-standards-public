# WIA-mobile-payment PHASE 2 — API Interface Specification

**Standard:** WIA-mobile-payment
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a mobile-payment boundary exposes
for wallet provisioning, token lifecycle management, transaction
submission, FIDO2 attestation, CDCVM event recording, QR-code
payload generation/consumption, and receipt retrieval.

References (CITATION-POLICY ALLOW only):
- EMVCo Tokenisation Framework v2.x — token request, provisioning, lifecycle
- EMVCo Cloud-Based Payments
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 8259 (JSON)
- W3C WebAuthn Level 3 / FIDO2 CTAP2.1
- W3C Payment Request API
- EMVCo QR Code Specification

---

## §1 Wallet provisioning

```
POST /wallets/<walletAccountRef>/provisioning HTTP/1.1
Host: mp.wallet-provider.example
Authorization: Bearer eyJhbGciOiJFUzI1NiIsImtpZCI6ImsxIn0...
Content-Type: application/wia-mp+json

{
  "tokenRequestor": "TR-12345678",
  "underlyingPanRef": "encrypted-pan-blob-from-issuer-app",
  "deviceRef": "urn:wia:mp:device:apple-attestation:...",
  "provisioningChannel": "in-app-issuer",
  "cardholderConsent": {"signed": "<JWS detached>"}
}
```

The boundary forwards to the network's TSP, receives the EMVCo
token, persists the provisioning record (PHASE 1 §3), and returns
the tokenised PAN reference. Failures route to the relevant
problem URI (`urn:wia:mp:problem:idv-declined`,
`urn:wia:mp:problem:device-attestation-failed`, etc.).

## §2 Token lifecycle

```
POST /tokens/<tokenisedPAN>/$suspend HTTP/1.1
POST /tokens/<tokenisedPAN>/$resume HTTP/1.1
DELETE /tokens/<tokenisedPAN> HTTP/1.1
```

These three endpoints implement the EMVCo Tokenisation Framework
lifecycle states. Suspension freezes the token (no new transactions
authorised), resume reactivates, deletion is permanent. Re-binding
to a new PAN (cardholder receives re-issued card) issues a new
token via PHASE 2 §1 referencing the prior token in the
provisioning record.

## §3 Contactless EMV transaction submission

For online authorisation (cryptogram type ARQC):

```
POST /transactions HTTP/1.1
Content-Type: application/wia-mp+json

{
  "walletAccountRef": "urn:wia:mp:wallet:apple:...",
  "tokenisedPAN": "TOK-...",
  "cryptogram": "<base64 ARQC>",
  "cid": "80",
  "atc": 1234,
  "terminalEntryMode": "contactless-emv-chip",
  "transactionType": "00",
  "cvm": "cdcvm",
  "amount": {"currency": "KRW", "amount": "12000"},
  "merchantId": "M-...",
  "terminalRef": "urn:wia:mp:terminal:..."
}
```

The boundary forwards to the relevant card network for online
authorisation, records the network's response, and emits an
AuditEvent. Offline-approved transactions (cryptogram type TC)
are submitted in batched clearing-files at end-of-day.

## §4 HCE cryptogram refresh

For HCE wallets using limited-use keys:

```
POST /hce/keys/$refresh HTTP/1.1
{
  "walletAccountRef": "urn:wia:mp:wallet:google:...",
  "tokenisedPAN": "TOK-...",
  "deviceRef": "urn:wia:mp:device:android-key:..."
}
```

The boundary calls the network's TSP for new limited-use keys and
returns them encrypted under the device's attestation key. The
wallet decrypts on-device and stores the keys in its Android
Keystore / iOS Secure Enclave.

## §5 QR-code payload generation

For Consumer-Presented Mode QR (wallet displays QR):

```
POST /qr/cpm HTTP/1.1
{
  "walletAccountRef": "urn:wia:mp:wallet:naver:...",
  "tokenisedPAN": "TOK-...",
  "amount": {"currency": "KRW", "amount": "12000"}
}
```

Response carries the EMVCo QRCPS payload as a TLV-encoded string
plus the rendered QR-code image. The CRC is included in the
payload; merchants verify the CRC on scan.

For Merchant-Presented Mode (merchant displays QR), the merchant
generates a static or dynamic QR per the same EMVCo specification;
this PHASE provides a `POST /qr/mpm/$validate` endpoint for the
wallet to verify the merchant's QR before constructing the
transaction.

## §6 FIDO2 attestation

```
POST /attestations HTTP/1.1
Content-Type: application/wia-mp+json

{
  "walletAccountRef": "urn:wia:mp:wallet:samsung:...",
  "deviceRef": "urn:wia:mp:device:tpm-attestation:...",
  "attestationFormat": "tpm",
  "attestationStatement": {...},
  "aaguid": "...",
  "publicKey": {...COSE...},
  "signature": "..."
}
```

The boundary verifies the attestation chain to a recognised root
(Apple, Google, Microsoft, vendor-specific TPM CAs, KISA-recognised
KR roots), records the attestation, and binds the device to the
wallet account.

## §7 CDCVM event recording

CDCVM happens on-device; the wallet posts a record afterwards:

```
POST /cdcvm/events HTTP/1.1
{
  "walletAccountRef": "...",
  "cdcvmMethod": "face",
  "liveness": true,
  "transactionRef": "...",
  "cdcvmTimestamp": "2026-04-27T09:31:14+09:00"
}
```

The boundary persists the event and links it to the transaction.
Biometric templates are NEVER posted; only the method, the result,
and the device's signed assertion that CDCVM occurred.

## §8 Receipt retrieval

```
GET /receipts/<transactionRef> HTTP/1.1
Authorization: Bearer ...
```

Returns the transaction receipt (PHASE 1 §9). Receipts are signed
by the wallet provider; the requester verifies the signature
against the provider's published JWKS before accepting.

## §9 Errors and warnings

| URI                                              | Status | Meaning                                       |
|--------------------------------------------------|-------:|-----------------------------------------------|
| `urn:wia:mp:problem:device-attestation-failed`   | 403    | FIDO2 attestation chain fails verification    |
| `urn:wia:mp:problem:idv-declined`                | 403    | issuer ID&V declined wallet provisioning      |
| `urn:wia:mp:problem:atc-replay`                  | 409    | ATC value already seen for this token         |
| `urn:wia:mp:problem:limited-use-key-exhausted`   | 422    | HCE key budget exhausted; refresh required    |
| `urn:wia:mp:problem:qr-checksum-invalid`         | 400    | EMVCo QRCPS CRC mismatch                       |
| `urn:wia:mp:problem:cdcvm-required`              | 422    | transaction requires CDCVM but none recorded  |
| `urn:wia:mp:problem:token-suspended`             | 403    | tokenised PAN suspended via §2                |
| `urn:wia:mp:problem:audit-unavailable`           | 503    | audit chain write failed                      |

Warnings (200-OK with content caveats) use `Warning:` headers per
RFC 7234 §5.5 with codes namespaced under `wia-mp-`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Capability advertisement (informative)

```
GET /.well-known/wia/mobile-payment/capabilities HTTP/1.1
```

Response advertises supported wallet platforms, supported card
networks, supported attestation roots, supported QR formats
(EMVCo QRCPS, KR Zero Pay, JP J-Coin), supported FIDO2 attestation
formats, and the deployment's W3C Payment Request integration
status. Capability documents are signed.

## Annex B — Idempotency keys (informative)

Sensitive POST endpoints accept an `Idempotency-Key` header so retries
do not duplicate transactions. The boundary stores the key for 24
hours; a retry within that window with the same key returns the
original response. UETR alone is not enough because mobile retries
may not have a UETR before the boundary assigns one.

## Annex C — Worked WebAuthn registration sequence (informative)

The wallet RP (relying party) initiates registration:

```js
navigator.credentials.create({
  publicKey: {
    challenge: <issued by RP>,
    rp: {id: "wallet-provider.example", name: "WalletProvider"},
    user: {id: walletAccountRefBytes, name: cardholderEmail, displayName: "..."},
    pubKeyCredParams: [{type: "public-key", alg: -7}],   // ES256
    authenticatorSelection: {userVerification: "required", residentKey: "preferred"},
    attestation: "direct"
  }
})
```

The resulting attestation statement posts to PHASE 2 §6.

## Annex D — Capability discovery worked example (informative)

```
GET /.well-known/wia/mobile-payment/capabilities HTTP/1.1
Accept: application/json
```

```
200 OK
Content-Type: application/json

{
  "wia.walletPlatforms": ["apple-pay", "google-wallet", "samsung-wallet", "naver-pay", "kakao-pay"],
  "wia.cardNetworks": ["visa", "mastercard", "amex", "jcb", "unionpay", "bccard", "kebhana"],
  "wia.attestationRoots": ["apple", "google-android-key", "samsung-knox", "kisa-kr-roots"],
  "wia.qrFormats": ["emvco-cpm", "emvco-mpm", "kr-zero-pay", "jp-jcoin"],
  "wia.fidoAttestationFormats": ["packed", "tpm", "android-key", "apple", "apple-app-attest"],
  "wia.w3cPaymentRequest": true,
  "wia.signature": "<JWS detached>"
}
```

Clients verify the signature against the deployment's JWKS before
honouring any advertised capability. Cached capability documents
expire on the deployment's session-token lifetime.

## Annex E — Pagination and rate limiting (informative)

Wallet-account list and transaction-history queries paginate at
≤ 1000 entries per page. Per-token rate-limit defaults: 100
provisioning calls per minute per wallet provider, 1000 transaction
submissions per second per wallet provider. Rate-limit refusals
carry `urn:wia:mp:problem:rate-limited`.

## Annex F — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. EMVCo Tokenisation
Framework versions bump independently from this PHASE; the
deployment policy maps each PHASE version to the EMVCo TSP version
it honours. Deprecation enters a 12-month sunset window with
migration notes recorded in the audit chain.

## Annex G — Acceptance flow worked example (informative)

A typical Apple Pay + Visa contactless transaction at a KR convenience
store:

1. Customer opens Apple Wallet, selects card; CDCVM (Face ID) runs;
   wallet presents the EMVCo cryptogram via NFC contactless
2. Terminal builds an ISO 8583 0100 authorisation request, routes
   to acquirer, acquirer routes to Visa
3. VTS detokenises tokenised PAN to underlying PAN, forwards to
   issuer for online authorisation
4. Issuer verifies cryptogram, ATC freshness, fund availability,
   fraud screening; returns approval
5. Apple Pay receives `transactionStatus: approved` via the network
   callback; wallet displays "Done", emits CDCVM event + transaction
   record into PHASE 1 §4 chain
6. Receipt emitted; clearing handoff to WIA-payment-system at
   end-of-day batch

## Annex H — Domain-restricted token (DRC) refresh

The Domain Restriction Controls bound to a token are managed by the
network's TSP. The wallet refreshes DRC values opportunistically:

- On wallet app start (cold-start)
- Daily during background refresh
- On-demand when a transaction fails with `urn:wia:mp:problem:drc-mismatch`

DRC mismatches at point-of-sale are rare but recoverable; the wallet
surfaces a "tap again" prompt while it refreshes in the background.

## Annex I — Wallet-account discovery

```
GET /wallets?owner=<authenticated-self> HTTP/1.1
Authorization: Bearer ...
```

Returns the wallet accounts owned by the authenticated cardholder
across all bound devices, with for each: walletAccountRef,
panSuffix, cardScheme, deviceRef, walletProvider, current state
(active/suspended). Cardholder-initiated suspension via
`POST /wallets/<ref>/$suspend` and resume via
`POST /wallets/<ref>/$resume` are gated by FIDO2 CDCVM. Lost-device
deletion via `DELETE /wallets/<ref>` requires an additional reviewer
sign-off if the wallet's last-active timestamp is recent (suspected
device theft is a higher-stakes operation than routine deletion).
