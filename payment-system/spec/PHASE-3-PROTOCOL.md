# WIA-payment-system PHASE 3 — Protocol Specification

**Standard:** WIA-payment-system
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format (PHASE 1)
to the API surface (PHASE 2): authentication of originating banks,
acquirers, payment-initiation services, and merchants; the cross-
clearing handshake; PCI DSS v4 segmentation; the audit chain
construction for payment lifecycles; and the cross-jurisdiction
release-authority handshake for cross-border flows.

References (CITATION-POLICY ALLOW only):
- ISO 20022 — message exchange semantics
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK), RFC 9162
  (Certificate Transparency 2.0 pattern)
- PCI DSS v4.0 — segmentation, encryption, key-management requirements
- PCI Software Security Framework — Secure Software Standard
- EMVCo Book 2 / Book 3 — chip authentication and online authorisation flows
- EMVCo Tokenisation Framework v2.x — token request, provisioning, lifecycle
- FATF Recommendation 16 — Wire Transfers (originator and beneficiary information)

---

## §1 Authentication

Originating institutions and PSPs (Payment Service Providers)
authenticate using mTLS (TLS 1.3 client certificates) plus JWS-
signed message-level signatures. The two layers serve different
purposes:

- mTLS authenticates the *connection*: the client certificate
  identifies the institution holding the TLS session
- JWS authenticates the *message*: each payment instruction is
  signed by the originating principal, distinguishing the
  institution (TLS) from the originating user/system (JWS)

Token-based bearer auth (JWT) is permitted for non-message
operations (status query, statement retrieval) where mTLS is
insufficient.

## §2 Token format and signing

Tokens are JWS-signed JWTs (RFC 7515 + RFC 7519). Default signature
algorithm is ES256 (P-256 ECDSA); high-value flows (interbank
wholesale) SHOULD use ES384. Mandatory claims:

| Claim     | Source                                                       |
|-----------|--------------------------------------------------------------|
| `iss`     | issuing authority (institution's KA)                         |
| `aud`     | resource server (clearing system or counterparty)             |
| `sub`     | originating principal URN                                    |
| `iat` / `exp` | issuance/expiry per RFC 7519                              |
| `wia_role` | one of: bank, psp, acquirer, processor, regulator, merchant |
| `wia_jur`  | declared jurisdiction (KR / US / EU / JP / ...)              |
| `wia_party_role[]` | scoped roles (e.g., debtor-agent, creditor-agent)    |

Tokens MUST be sent over TLS 1.3.

## §3 Key management

Each institution publishes a JWKS at a fixed well-known URI.
Signing keys rotate at least every 365 days for routine traffic
and every 180 days for high-value wholesale flows. Prior keys
remain in the JWKS for ≥ 730 days (longer than the longest
disputable transaction window) so signatures on long-lived
instructions remain verifiable.

Private keys live in HSMs (FIPS 140-3 Level 3 for high-value
wholesale; Level 2 minimum for retail). For KR deployments KCMVP
is acceptable. EMVCo card-network signing keys live in the card
network's own HSM topology, separate from the deployment's general
signing key.

## §4 Audit chain construction

Every instruction acceptance, status transition, screening
decision, fraud flag, dispute transition, refund, and authorisation
emits an AuditEvent. AuditEvents form a per-deployment hash chain:

```
chain_input  = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

Canonicalisation uses RFC 8785 JSON Canonicalisation Scheme. The
chain root is sealed once per UTC day. Sealed roots MAY be published
to a transparency log (RFC 9162 pattern) for jurisdictions that
require public auditability.

For high-volume retail payment rails the chain is sharded by UETR
hash prefix to keep sealing throughput within operational SLAs;
the sharding is itself audited so an after-the-fact reshard is
detectable.

## §5 Cross-clearing handshake (FATF Rec 16)

Cross-border instructions carry FATF Recommendation 16 originator
and beneficiary information end-to-end:

- Originator: full name, account number (or unique transaction ref),
  address (or national ID, customer ID, place and date of birth)
- Beneficiary: full name, account number (or unique transaction ref)

Intermediary institutions MUST relay this information unchanged.
The boundary refuses to relay an instruction missing required Rec 16
fields with `urn:wia:pay:problem:fatf-rec16-incomplete`. Stripping
or anonymising Rec 16 information is itself a regulatory violation.

## §6 PCI DSS v4 segmentation

The boundary respects PCI DSS v4 segmentation:

- The *cardholder data environment* (CDE) holds full PAN, magstripe
  Track 1/2, CVV2, PIN block. CDE is a network-segmented enclave
  with TLS 1.3 mTLS, FIPS-validated crypto, and quarterly ASV scans
- Tokenisation services map full PAN ↔ tokenised PAN at the CDE
  boundary; tokens cross into non-CDE zones, full PAN does not
- The audit chain stores tokenised PAN only; full PAN is held in
  CDE-internal audit only, governed by PCI DSS v4 Requirement 10

A non-CDE caller that requests a full-PAN-bearing operation is
refused with `urn:wia:pay:problem:pci-zone-required`. A CDE
caller that attempts to export full PAN to a non-CDE zone is
refused at the CDE egress filter.

## §7 Time discipline

Clocks synchronise to authoritative sources (NTPv4 stratum-2 to
GNSS-disciplined; for high-value RTGS the deployment uses BIPM-
traceable time references). Drift exceeding 50 ms suspends new
instruction issuance; drift exceeding 1 s suspends settlement
finality posting because finality is timestamp-bound.

All record timestamps are RFC 3339 with offset; UTC is preferred
on transit; local time + offset is permitted on receipts where
the receiving institution's regulatory regime requires local
time on the printed record.

## §8 Settlement finality

Settlement finality is per the operating rail's finality model:

| Rail            | Finality model                                                |
|-----------------|---------------------------------------------------------------|
| RTGS (TARGET2, Fedwire, BOK-Wire+) | settled = final; no return    |
| Instant payment (FedNow, SEPA Instant, KFTC 신속이체) | settled = final within seconds |
| Card network    | clearing-day cycle; chargebacks are *new* transactions, not unwinds |
| ACH (NACHA US, KFTC bulk KR)       | NSF return windows per scheme  |

The boundary records the operating rail's finality model in each
instruction so downstream auditors interpret return windows
correctly.

## §9 Failure modes

| Failure                              | Behaviour                                              |
|--------------------------------------|--------------------------------------------------------|
| KA JWKS unreachable                  | Cached keys honoured until cache expiry                |
| Sanctions screening offline          | Cross-border instructions queued; domestic continues   |
| Fraud engine offline                 | Instructions accepted; flagged for retroactive review  |
| CDE tokenisation offline             | Card flows refused with `tokenisation-unavailable`      |
| Audit chain write failure            | Operations rejected on the boundary; no silent loss     |
| Time drift > 1 s                     | Settlement finality posting suspended                   |
| Counterparty BIC unreachable         | Instruction held in queue; retry per scheme rules       |
| FATF Rec 16 incomplete               | Cross-border instruction rejected at originating bank   |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

| Concern                  | Default                                | Notes                              |
|--------------------------|----------------------------------------|------------------------------------|
| Token signing            | ES256 (P-256 ECDSA)                    | wholesale prefers ES384            |
| Daily-root signing       | ES384 (P-384 ECDSA)                    |                                    |
| Audit hash               | SHA-256 (RFC 6234)                     |                                    |
| TLS                      | 1.3 (RFC 8446)                         | 1.2 explicit-list only             |
| Symmetric at rest (audit)| AES-256-GCM                            |                                    |
| Card PAN encryption (CDE)| AES-256-GCM with HSM-protected DEK    | PCI DSS Requirement 3.5            |
| Token format (PAN tokens)| EMVCo Tokenisation Framework v2.x      | network-specific token vault       |
| 3DS authentication crypto| EMVCo 3DS 2.x                          | for CNP authentication             |

Quantum-resistance migration is tracked separately as NIST PQ
standards reach deployment-grade financial HSM support.

## Annex B — PCI DSS v4 zone discipline (informative)

The CDE is a tightly-segmented network zone:

- Inbound: only authenticated, encrypted, allowlisted traffic
- Outbound: only to specific authorised destinations (card networks,
  HSM, regulator interfaces); blocked by default
- Audit: every CDE-internal action emits a Requirement 10 log entry
  preserved for the PCI-required retention period
- Personnel: only background-checked operators with PCI-trained
  duty assignments may access CDE-internal systems
- Tokenisation services straddle the CDE boundary: full PAN inside,
  tokens outside

A breach of zone discipline is itself an incident under PCI DSS
Requirement 12.10 (incident response).

## Annex C — Cross-border release-authority handshake (informative)

Cross-border instructions may pass through correspondent banking
relationships. The release-authority handshake:

1. Originating bank signs the instruction with full FATF Rec 16
   originator/beneficiary information
2. Each correspondent bank in the chain verifies the prior signature,
   adds its own signature (and its own `chargeBearer` adjustments
   per the chain), and forwards
3. The receiving bank verifies the entire signature chain plus
   the FATF Rec 16 information before posting the credit

A correspondent bank that cannot relay the FATF Rec 16 information
unchanged is non-conformant; instructions through that path are
diverted by the originating bank to a different correspondent.

## Annex D — Tokenisation lifecycle (informative)

EMVCo Tokenisation Framework v2.x defines the lifecycle:

1. **Token request** — wallet provider or merchant requests a token
   from the network's Token Service Provider (TSP) bound to a
   specific PAN
2. **Token provisioning** — TSP issues a token; the wallet stores the
   token (+ optional Domain Restriction Controls)
3. **Token authorisation** — at transaction time, the token + a
   transaction-time cryptogram (DPAN cryptogram) are presented; the
   network detokenises at the issuer side and authorises against
   the underlying PAN
4. **Token lifecycle events** — suspension, deletion, re-binding to
   a new PAN (when the cardholder receives a re-issued card)

This PHASE records token lifecycle events as `urn:wia:pay:token:<...>`
in the audit chain. The full PAN never leaves the TSP / issuer HSM
boundary.

## Annex E — Time-precision worked example (informative)

For RTGS rails the timestamp precision is millisecond:

```
"creationDateTime": "2026-04-27T09:15:00.123+09:00"
```

For instant-payment rails the precision is microsecond where the
underlying clock supports it:

```
"creationDateTime": "2026-04-27T09:15:00.123456+09:00"
```

For ACH / card clearing the precision is second; sub-second values
are tolerated but not required.

## Annex F — Settlement-finality interpretation (informative)

Different rails interpret "final" differently:

- **RTGS** (BOK-Wire+, TARGET2, Fedwire) — credit posted is final
  on the central bank's books; no return possible. The boundary
  must not advertise tentative state on RTGS messages
- **Instant payment** (KFTC 신속이체, FedNow, SEPA Instant, UPI) —
  final within seconds; the rail's reverse-payment is a fresh
  instruction, not an unwind
- **Card networks** — authorisation is a contractual lock on funds,
  but clearing happens daily; chargebacks are *new* card transactions
  initiated by the issuer through the network's chargeback workflow,
  not unwinds of the original authorisation
- **ACH / batch clearing** — NSF return windows define a settlement-
  reversal possibility; the boundary tracks the return window per
  transaction and surfaces expiring windows to operations

The boundary records the operating rail's finality model in each
instruction so downstream auditors interpret return possibilities
correctly.
