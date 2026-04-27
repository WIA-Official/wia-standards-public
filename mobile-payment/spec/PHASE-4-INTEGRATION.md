# WIA-mobile-payment PHASE 4 — Integration Specification

**Standard:** WIA-mobile-payment
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how a mobile-payment deployment integrates the
data, APIs, and protocols from PHASEs 1–3 with adjacent systems:
issuing banks, card networks, acquirers, POS terminal vendors,
device-attestation services, wallet platforms, and the downstream
clearing layer (WIA-payment-system). It is non-prescriptive about
specific vendors; it specifies the integration *contracts* a
deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- EMVCo Tokenisation Framework v2.x
- EMVCo Cloud-Based Payments
- W3C Payment Request API; W3C Web Payments
- W3C WebAuthn Level 3 / FIDO2 CTAP2.1
- WIA-payment-system (PHASE 1–4) — for clearing handoff
- KR FSC e-finance regulations (referenced for KR deployments)
- JP METI Cashless Payment Guidelines (referenced for JP deployments)

---

## §1 Issuer integration

The issuer (the bank that issued the underlying card) participates
in:

- **ID&V** during provisioning (PHASE 3 §6) — issuer's API receives
  the wallet provider's request and returns green/yellow/red
- **Online authorisation** for ARQC-cryptogram transactions —
  network forwards authorisation requests; issuer authorises or
  declines based on funds, fraud, and risk
- **Token lifecycle events** — issuer-initiated suspension or
  deletion (e.g., card lost / stolen) propagates to the wallet via
  the network

The integration contract: issuer responses are signed; the boundary
refuses to honour unsigned issuer responses. Issuer-initiated token
state changes are recorded as audit events in the wallet's chain
plus the issuer's own systems.

## §2 Card network integration

Mobile-payment flows route through the card network for online
authorisation and clearing. Per-network specifics:

- **Visa Token Service** for Visa
- **Mastercard Digital Enablement Service (MDES)** for Mastercard
- **JCB Token Service** for JCB
- **AmEx Token Service** for AmEx
- **UnionPay Tokenization Service** for UnionPay
- **BC카드/KEB하나카드/KFTC TSP** for KR-issued cards

Each network maintains its own TSP; this PHASE references the
relevant TSP per card network at provisioning time. Networks do
not share token vaults; cross-network token portability is not
in scope of this PHASE.

## §3 Acquirer / POS terminal integration

Acquirers connect POS terminals into the card networks. Integration
specifics:

- POS terminal firmware versions are tracked by the acquirer; an
  out-of-date firmware (failing PCI PTS / EMVCo Level 1/2/3)
  is suspended from production routing
- Contactless EMV transactions flow from terminal → acquirer
  switch → card network. The boundary intersects on the wallet
  side (PHASE 2 §3) for transaction origination from the wallet
  itself; terminal-side flows are out of this PHASE's scope
- QR-code merchant acceptance follows the acquirer's QR profile
  (KR Zero Pay, JP J-Coin Pay 加盟店 QR, Alipay+, etc.)

## §4 Wallet platform integration

The deployment integrates with one or more wallet platforms:

- **Apple Pay** — uses Apple's PassKit / WalletKit, requires Apple
  attestation, uses Apple's Secure Enclave for token storage
- **Google Pay / Wallet** — uses Google's PaymentsPro, requires
  Android Key / SafetyNet attestation, uses StrongBox where available
- **Samsung Wallet / Pay** — uses Samsung Knox attestation, supports
  MST (Magnetic Secure Transmission) on legacy terminals
- **Naver Pay / Kakao Pay** (KR) — KR-domestic wallet platforms with
  KISA-recognised attestation roots
- **J-Coin Pay** (JP) — JBA JP-Coin tag plus EMVCo QRCPS

Each platform's certification scheme has its own requirements;
the deployment policy enumerates which platforms it supports.

## §5 Device attestation services

Device attestation services include:

- **Apple Attestation Service** — for iOS / iPadOS / watchOS
- **Google Play Integrity API** + Android Key Attestation — for Android
- **Microsoft Attestation Service** — for Windows TPM-backed wallets
- **Samsung Knox Attestation** — for Knox-enabled Samsung devices
- **Vendor-specific TPM / TEE attestation** — for less-common platforms

The boundary maintains a recognised-roots list per platform. A
device whose attestation chains to an unrecognised root is refused
at PHASE 2 §6.

## §6 W3C Payment Request integration

For browser-based checkout (W3C Payment Request API), the deployment
exposes a payment-handler that bridges from the browser API into
this PHASE's surface:

- Browser invokes `PaymentRequest({methodData, details, options})`
- Payment-handler authenticates with the wallet's credentials
  (FIDO2 CDCVM)
- Payment-handler constructs a contactless-equivalent transaction
  record using the tokenised PAN + cryptogram
- Boundary submits to the card network via PHASE 2 §3

W3C Payment Request integration is opt-in per deployment; not all
wallet providers expose a Payment Handler.

## §7 Clearing handoff (WIA-payment-system)

Authorised transactions hand off to the clearing layer (WIA-payment-
system) for batched clearing-day settlement:

- Each transaction's record references the resulting WIA-payment-
  system clearing entry
- Refunds initiated through PHASE 2 §3 (transactionType 20) flow
  to WIA-payment-system as new card transactions
- Disputes initiated through WIA-payment-system PHASE 2 §7 reference
  the originating mobile-payment transaction record

The two standards share the audit chain so end-to-end reconstruction
is possible from CDCVM event through clearing settlement.

## §8 Operational SLAs

| Concern                                          | Default SLA              |
|--------------------------------------------------|--------------------------|
| Wallet provisioning end-to-end                   | ≤ 30 s (frictionless)    |
|                                                  | manual challenge per issuer |
| Transaction submission p95 added latency         | ≤ 100 ms                 |
| Online authorisation network round-trip          | ≤ 2 s                    |
| HCE limited-use-key refresh                      | ≤ 5 s                    |
| FIDO2 attestation verification                   | ≤ 500 ms                 |
| Receipt availability after authorisation         | ≤ 5 s                    |
| Audit chain entry available after operation      | ≤ 1 s                    |

## §9 Acceptance criteria

A deployment claims conformance when:

1. Every fielded wallet account has a valid device attestation on file
2. Every transaction in the past quarter has a matching audit chain
   entry
3. Every transaction's ATC value is monotonically increasing per
   token (no replay)
4. Limited-use-key refresh activity is within the network's policy
5. CDCVM rate is consistent with the wallet's policy (most wallets
   require CDCVM on every transaction or after the first idle window)
6. Token lifecycle events (issuer-initiated suspensions, lost-card
   deletions) propagate within the operational SLA
7. Quarterly compliance report has no integrity-check failures

A deployment failing any of these reports the gap in its compliance
package rather than concealing it.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Common pitfalls (informative)

- **Attestation root drift** — wallet-platform attestation roots
  rotate periodically; the deployment's recognised-roots list MUST
  refresh quarterly to avoid rejecting freshly-attested devices
- **Limited-use-key starvation** — HCE wallets that aggressively
  consume their key budget without timely refresh experience
  recurring "refresh required" UI; the deployment SHOULD monitor
  per-wallet refresh frequency and tune the headroom
- **Token-vault scaling** — TSP token vaults that approach capacity
  silently throttle new provisioning; the deployment SHOULD monitor
  vault utilisation and provision capacity proactively
- **EMVCo cryptogram replay across schemes** — different schemes
  use different cryptogram derivations; a wallet binding multiple
  scheme tokens must keep per-token ATC counters separate
- **CDCVM bypass via terminal "no-CVM" path** — some terminals
  configured for low-value contactless skip CDCVM altogether; the
  deployment policy SHOULD set a per-merchant low-value threshold
  consistent with the issuer's policy

## Annex B — Decommissioning (informative)

When a wallet platform is decommissioned, all bound wallets are
suspended and tokens are deleted via PHASE 2 §2. Active provisioning
records are retained per the wallet platform's audit retention
period; transaction history is retained per the underlying
WIA-payment-system retention rules. The decommissioning manifest
is itself an audit event in the chain, signed by both outgoing
and incoming custodians.

## Annex C — Quarterly compliance report (informative)

The boundary emits a quarterly compliance report covering:

- Total provisioning attempts and outcomes (green/yellow/red)
- Total contactless transactions by network and outcome
- ATC-replay refusals (count; 0 expected)
- Limited-use-key refresh rate per wallet platform
- CDCVM rate per wallet platform per merchant category
- Token-vault utilisation per network
- Attestation-root recognition status
- Audit-chain integrity check results

The report is signed and is itself in scope for the audit chain so
report tampering would surface in the chain.

## Annex D — Wallet-platform decommissioning (informative)

When a wallet platform exits the deployment:

1. All bound wallets on that platform are suspended via PHASE 2 §2
2. Tokens are deleted via PHASE 2 §2 (DELETE)
3. Audit records are preserved per WIA-payment-system retention
4. The platform's attestation root is moved from "active" to
   "deprecated" in the recognised-roots list; existing transactions
   continue verifying historic attestations from the deprecated
   root, but new bindings are refused
5. Cardholders are notified via the platform's standard channels;
   their cards remain usable on other platforms or the physical
   card

The decommissioning manifest is itself an audit event in the chain.

## Annex E — Lessons-learned register (informative)

Recurring lessons-learned across wallet-platform certifications:

- **Domain Restriction Controls drift** — networks update DRC
  configurations; wallets that cache DRC values stale-out and
  reject legitimate transactions until cache refresh
- **Liveness-detection false negatives** — biometric liveness on
  some platforms produces false negatives in low-light or
  dark-skin conditions; the deployment SHOULD monitor per-method
  rejection rates and surface anomalies
- **Cross-network token portability** — tokens are not portable
  between networks; a Visa token cannot be re-binding to a
  Mastercard PAN. Wallet UX should make this clear to the cardholder
- **HCE budget exhaustion at point-of-sale** — recovering from
  exhaustion mid-checkout requires a network call; deployments
  SHOULD pre-warm the budget when wallet enters foreground

## Annex F — Wallet-platform certification (informative)

Apple Pay, Google Wallet, Samsung Wallet, and the KR-domestic
wallets each maintain their own certification programmes. A
deployment seeking acceptance into a wallet:

1. Submits the deployment's ID&V flow + attestation handling for
   the platform's review
2. Passes the platform's compatibility tests (provisioning success
   rate, transaction success rate, customer-support pathway)
3. Signs the platform's commercial agreement (out of scope of this
   PHASE)
4. Receives platform-issued credentials for its TSP integration

The platform reviews the deployment at quarterly intervals or after
a material change. Failure to maintain certification suspends new
provisioning while permitting existing wallets to continue
transacting until token expiry.
