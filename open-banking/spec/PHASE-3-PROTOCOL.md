# WIA-open-banking PHASE 3 — Protocol Specification

**Standard:** WIA-open-banking
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format (PHASE 1)
to the API surface (PHASE 2): SCA flows (redirect, decoupled,
embedded), FAPI 2.0 message-signature obligations, eIDAS PSD2
certificate handling, audit-chain construction, dynamic linking
under PSD2 RTS Article 5, and the cross-jurisdiction handling for
deployments that operate under multiple regimes.

References (CITATION-POLICY ALLOW only):
- OpenID Foundation FAPI 2.0 (Baseline + Advanced)
- OpenID Connect Core 1.0
- IETF OAuth 2.0 (RFC 6749), OAuth 2.1 draft, RFC 7515 (JWS),
  RFC 7517 (JWK), RFC 7519 (JWT), RFC 7636 (PKCE), RFC 9101 (JAR),
  RFC 9126 (PAR), RFC 8705 (mTLS, certificate-bound tokens),
  RFC 8628 (Device Authorisation)
- IETF RFC 8446 (TLS 1.3), RFC 9162 (Certificate Transparency 2.0 pattern)
- EU PSD2 RTS on SCA and CSC
- EBA Guidelines on the conditions of the SCA exemptions (EBA/GL/2018/04)
- EU eIDAS Regulation (910/2014) — QWAC and QSealC

---

## §1 Strong Customer Authentication

PSD2 RTS Article 4 requires SCA combining at least two of:
knowledge, possession, inherence — independent factors. The
deployment supports:

- **Redirect SCA** (preferred under FAPI 2.0): TPP redirects user
  to ASPSP's authorisation endpoint; user authenticates in the
  ASPSP's UI; ASPSP redirects back with an authorisation_code
- **Decoupled SCA**: ASPSP authenticates the user on a separate
  channel (e.g., mobile-banking app push); TPP polls for completion
- **Embedded SCA** (rare in PSD2/FAPI 2.0): SCA UI rendered inline;
  generally avoided because it conflicts with FAPI 2.0's
  redirect-and-PKCE preference

Each SCA invocation produces an SCA evidence record (PHASE 1 §4).
Records are signed by the customer's SCA-bound credential plus the
ASPSP's signing key.

## §2 Dynamic linking (PSD2 RTS Article 5)

For payment-initiation flows, the SCA evidence MUST be dynamically
linked to the specific transaction:

- The amount and the payee of the transaction are bound into the
  authentication code
- A modification of either field after SCA invalidates the SCA
  evidence; the boundary refuses the payment

Dynamic-linking implementation: the JAR JWT (RFC 9101) carrying the
PAR includes the `authorization_details` block (RAR) with the
exact transaction; the SCA-bound credential signs over a hash of
this block plus the authorisation_code.

## §3 FAPI 2.0 message-signature obligations

FAPI 2.0 Advanced profile requires:

- **mTLS-bound access tokens** per RFC 8705
- **Pushed Authorisation Requests** (PAR, RFC 9126) — the TPP MUST
  push the authorisation request to the ASPSP before redirect
- **JWT-Secured Authorisation Requests** (JAR, RFC 9101) — the
  authorisation request body is signed
- **Detached JWS** (`x-jws-signature` header) on payment-initiation
  request bodies and on notification deliveries
- **Replay protection** via `iat` / `nbf` / `exp` JWT claims plus
  per-call `x-fapi-interaction-id`

A request that omits these signatures or fails signature
verification is rejected with the relevant problem URI.

## §4 eIDAS PSD2 certificates (EU/UK)

EU/UK PSD2 deployments use eIDAS-grade certificates:

- **QWAC** (Qualified Web Authentication Certificate) — used as
  the TPP's mTLS client certificate; the QWAC includes the TPP's
  PSD2 role attributes (PSP_AS, PSP_PI, PSP_AI, PSP_IC) per
  ETSI TS 119 495
- **QSealC** (Qualified Seal Certificate) — used to sign request
  bodies (JWS detached); the QSealC binds the message to the legal
  entity, distinct from the connection-level QWAC

The boundary verifies certificates against the ETSI Trust Service
Status List (TSL) of the issuing nation. A QWAC/QSealC whose
issuer is not in the TSL is rejected.

## §5 KR MyData certificates

KR 마이데이터 deployments use KFSC-issued certificates:

- **상호인증서** — TPP's mTLS client certificate issued by the
  KFSC PKI
- **공동인증서 (formerly 공인인증서)** — customer's authentication
  certificate (legacy SCA path; modern KR MyData uses 간편인증)

KR consent records carry the customer's signed consent in the
KFSC standard template; the boundary verifies signatures against
KFSC's published JWKS and the KFSC's certificate-revocation list.

## §6 Audit chain construction

Every consent setup, SCA invocation, account-information access,
payment initiation, funds-confirmation, notification delivery, and
licence-status check emits an AuditEvent. AuditEvents form a per-
ASPSP hash chain:

```
chain_input  = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

Canonicalisation uses RFC 8785 JSON Canonicalisation Scheme. The
chain root is sealed once per UTC day. Sealed roots MAY be published
to a transparency log (RFC 9162 pattern) for jurisdictions that
require public auditability (UK FCA's Open Banking initiative
encourages such publishing).

## §7 Time discipline

Clocks synchronise to NTPv4 stratum-2 sources. Drift exceeding 100 ms
causes the boundary to refuse new SCA invocations because dynamic
linking depends on agreed time. Drift exceeding 5 s suspends new
token issuance.

All record timestamps are RFC 3339 with offset; UTC is preferred
on the wire; SCA evidence carries the local time-zone offset of
the user's authenticating device for forensic reconstruction.

## §8 Cross-regime handling

A deployment operating in multiple regimes (e.g., EU PSD2 + UK
OBIE + KR 마이데이터) holds a separate boundary instance per regime
because the consent semantics and certificate roots differ. Cross-
regime traffic does not exist in this PHASE; a customer with
accounts in two regimes engages each regime's TPPs separately.

## §9 Failure modes

| Failure                              | Behaviour                                                      |
|--------------------------------------|----------------------------------------------------------------|
| TPP licence revoked                  | All consents suspended; in-flight calls completed; new refused |
| eIDAS TSL fetch fails                | Cached TSL honoured until cache expiry; new TPP-onboarding refused |
| SCA channel unreachable (decoupled)  | TPP poll times out per deployment policy; SCA marked failed    |
| Dynamic-linking mismatch             | Payment refused with `urn:wia:ob:problem:dynamic-link-mismatch`|
| Audit chain write failure            | Operation rejected (consistency w/ payment-system §9)          |
| Time drift > 5 s                     | Token issuance + SCA invocation suspended                      |
| Notification webhook unreachable     | Retry per deployment policy; persistent failure surfaces incident |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

| Concern                       | Default                                | Notes                                   |
|-------------------------------|----------------------------------------|-----------------------------------------|
| JAR signing                   | PS256 or ES256                         | per FAPI 2.0 Advanced                   |
| Detached JWS body signature   | PS256 or ES256                         |                                         |
| OIDC ID token signing         | PS256 (FAPI 2.0 Baseline)              | RS256 not permitted                     |
| TLS                           | 1.3 (RFC 8446)                         | 1.2 explicit-list only                  |
| mTLS client cert              | RSA 2048 / ECC P-256                   | EU/UK: eIDAS QWAC                       |
| Token format                  | mTLS-bound JWT (RFC 8705)              | bearer + cnf claim with thumbprint      |
| PKCE                          | S256                                   | plain method not permitted              |

Quantum-resistance migration is tracked separately as NIST PQ
standards reach deployment-grade financial HSM support.

## Annex B — SCA exemption regime (informative)

EBA Guidelines GL/2018/04 enumerate SCA exemptions:

- Trusted Beneficiary list (RTS Article 13)
- Recurring transactions (RTS Article 14)
- Low-value payments (≤ €30; ≤ 5 cumulative or ≤ €100 cumulative
  per RTS Article 16)
- Transaction Risk Analysis (TRA) exemption (RTS Article 18) —
  permitted only when the deployment's fraud rate is below the
  threshold per Article 19

Each exemption invocation is itself an audit event so regulators
can review TRA-exemption usage against the deployment's fraud
rate over the relevant window.

## Annex C — Worked dynamic-linking flow (informative)

1. Customer enters £100 to "Bob's Plumbing"
2. TPP submits PAR with `authorization_details: [{type:
   "payment_initiation", instructedAmount: {currency: "GBP",
   amount: "100.00"}, creditorName: "Bob's Plumbing", ...}]`
3. ASPSP renders the SCA UI showing exactly £100 to "Bob's Plumbing"
4. Customer authenticates; SCA-bound credential signs a hash of the
   `authorization_details` block
5. ASPSP issues authorization_code; the SCA evidence record carries
   the signed hash
6. At payment-initiation time, the boundary recomputes the hash
   from the request body's Initiation block; mismatch refuses the
   payment with `urn:wia:ob:problem:dynamic-link-mismatch`

A modification of amount or payee after SCA fails the dynamic-
linking check; the customer is protected from man-in-the-middle
modification.

## Annex D — Conformance levels (informative)

| Level     | Scope                                                              |
|-----------|--------------------------------------------------------------------|
| Surface   | data formats accepted; self-attested; no FAPI 2.0 conformance test |
| Verified  | annual third-party audit + FAPI 2.0 conformance test pass          |
| Anchored  | continuous evidence package per audit chain transparency            |

Implementations declare their level in the OpenID Connect
discovery document. Coalition operations (cross-jurisdiction PIS)
typically require Verified or Anchored from all parties.

## Annex E — Time-precision worked example (informative)

For open-banking:

- Consent timestamps: second precision; SCA timestamps millisecond
- Payment-initiation timestamps: millisecond precision; clearing
  rails inherit their own precision (sub-second to second)
- Notification timestamps: millisecond precision

Drift between TPP, ASPSP, and clearing rail clocks must stay within
the most-restrictive operation's tolerance (SCA dynamic linking
~ 100 ms).

## Annex F — Federation manifest

Cross-jurisdictional flow (rare) uses a federation manifest signed by both regimes' authorities. Without a federation manifest, cross-regime traffic is refused.

## Annex G — Audit transparency option (informative)

UK OBIE encourages public publishing of daily audit roots to a
transparency log so customers and TPPs can independently verify the
ASPSP's audit chain. KR 마이데이터 currently does not require public
publishing. EU PSD2 leaves the choice to each ASPSP. The deployment
policy enumerates which transparency-log mode is in effect.
