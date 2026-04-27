# WIA-pq-crypto PHASE 3 — Protocol Specification

**Standard:** WIA-pq-crypto
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format
(PHASE 1) to the API surface (PHASE 2) and to operational
cryptographic exchanges: authentication of crypto-officers,
HSM control planes, and partners; the HSM control-plane
binary protocol; the TLS 1.3 hybrid handshake configuration;
hybrid CMS / JWS signature construction; key-rotation
overlap discipline; audit-chain construction; time
discipline; and the migration-phase advance/rollback
protocol.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK),
  RFC 9162 (Certificate Transparency 2.0)
- IETF draft-ietf-tls-hybrid-design — TLS 1.3 hybrid key exchange
- IETF RFC 9258 — Composite signatures in CMS
- NIST FIPS 203/204/205 — for primitive references
- NIST SP 800-208 — Stateful hash-based signature scheme considerations
- NSA CNSA 2.0 — for transition cadence

---

## §1 Authentication

Crypto-officers, key-holders, HSM control planes, partners,
and auditors authenticate using JWS-signed JWTs issued by
the deployment's identity authority. Token claims:

- `iss`, `sub`, `aud`, `iat`, `exp`
- `wia.role` — one of the roles in PHASE 2 Annex F
- `wia.scope[]` — operation-class scopes
- `wia.holderRef` — for key-holder tokens, the authority
  whose keys may be managed
- `wia.kmRef` — for HSM control-plane tokens, the KM URN

Crypto-officer write tokens are short-lived (typically 5
minutes) and require a hardware-token-backed user signature;
the audit chain records the user's hardware-token attestation
on every officer action.

## §2 HSM control-plane binary protocol

HSM control-plane interactions use a length-prefixed CBOR
stream over a mutual-TLS 1.3 connection with the HSM as
client and the boundary as server (or vice versa per
deployment topology). Each frame:

- 4-byte length prefix
- CBOR-encoded operation envelope (request, response, or
  attestation push)
- 32-byte per-frame MAC (HMAC-SHA-256 with rotated key)

Operation envelopes include:

- `op` — operation name (e.g., `keypair-create`,
  `keypair-attest`, `rotation-prepare`, `rotation-commit`,
  `compromise-flag`)
- `params` — operation-specific
- `requestId` — opaque request identifier
- `clientSignature` — client signature over the canonical
  encoding

The boundary persists frame-level acknowledgements; a
client may resume on reconnect using the boundary's
last-acknowledged cursor.

## §3 TLS 1.3 hybrid handshake configuration

For migration phase `hybrid`, the deployment's TLS 1.3
endpoints configure:

- Supported groups list including a hybrid group
  (e.g., `x25519_kyber768`, conventionally referenced via
  the IANA registry alias for ML-KEM-768)
- ClientHello signature_algorithms list including hybrid
  signature options (per draft-ietf-tls-hybrid-design)
- Cipher suites with PQ-secure AEAD parameters

The boundary's capability document publishes the supported
hybrid groups so partners verify negotiation feasibility
before exchange. Partners that cannot negotiate a hybrid
group fall back per the deployment's documented fallback
policy.

## §4 Hybrid CMS / JWS signature construction

For document and code signing in `hybrid` phase, signatures
are constructed per RFC 9258 composite-signature pattern:

- Two underlying signatures: classical (e.g., Ed25519) +
  PQ (e.g., ML-DSA-65)
- Composite signature object identifies both algorithms via
  `algorithmRef` URN and includes both signature blobs
- Verifiers verify both signatures independently; failure of
  either is a verification failure

Signing-keys for hybrid signatures are typically two
separate keypairs in the registry (PHASE 1 §3); the
hybrid-binding record (PHASE 1 §5) names the pair.

## §5 Key-rotation overlap discipline

During rotation overlap windows:

- Both predecessor and successor keys are valid for
  verification
- New artifacts are signed with the successor key
- Verifiers honouring the deployment's policy accept either
  key during the overlap

Rotation discipline rules:

- Overlap window must not be shorter than the deployment-
  declared minimum (typically 24 hours for routine rotations,
  zero for compromise rotations)
- Compromise rotations include a `compromise-flag`
  attestation from the HSM control plane confirming the
  key has been destroyed
- Successor key MUST exist before predecessor `validNotAfter`

## §6 Time discipline

All record timestamps use RFC 3339 with explicit offset.
Boundary clock is disciplined to a national-laboratory time
reference; drift outside declared bound triggers a
`boundary-clock-degraded` capability flag and tags subsequent
records `provisional` until recovered.

## §7 Audit chain

Every boundary state transition is appended to a Merkle
audit log:

- `entryId`, `parent`, `at`, `actor`, `kind`, `payloadHash`,
  `signature`
- `kind` enum: `algorithm-registered`, `algorithm-deprecated`,
  `keypair-registered`, `keypair-rotated`, `keypair-retired`,
  `compromise-rotation-flagged`, `phase-advanced`,
  `phase-rolled-back`, `evidence-published`,
  `attestation-recorded`, `partner-roster-mutated`

Anchored deployments mirror the audit chain to a regulator-
trusted witness on a declared cadence; regulators receiving
phase-advance and compromise-rotation entries on a
push subscription.

## §8 Migration-phase advance/rollback protocol

Phase advance follows a documented protocol:

1. Crypto-officer drafts the phase declaration
2. Audit committee reviews the deployment's roadmap
   compliance and risk file
3. Boundary stages the new phase declaration in test mode
   with all subscribed partners notified
4. Partner acknowledgement window (typically 30 days)
5. Activation: phase declaration becomes current; the
   boundary refreshes capability document and notifies all
   subscribers

Rollback is the inverse; rollback within the deployment-
declared safety window does not require partner
acknowledgement, but emergency rollback (e.g., critical
algorithm vulnerability) requires crypto-officer counter-
signature plus an incident-record reference.

## §9 Replay protection

All write endpoints accept `Idempotency-Key`; the boundary
stores keys for 30 days. Replays return the original
response; conflicts return
`urn:wia:pqc:problem:idempotency-conflict`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cryptographic signature suite

Default boundary signatures use Ed25519 in classical phase
and ML-DSA-65 in PQ-only phase; hybrid phase uses both
under the composite-signature pattern of §4.

## Annex B — Compromise-rotation handling

Compromise rotations bypass the standard overlap window:

1. HSM control plane emits `compromise-flag` attestation
2. Successor key is created and signed with the highest
   priority
3. Predecessor key's `validNotAfter` is set to current epoch
4. Boundary refuses any further use of the predecessor for
   signing (verification of historical artifacts continues
   per deployment policy)
5. Audit-chain entry at `kind=compromise-rotation-flagged`
   carries an incident-record reference for the cause
6. All subscribers are notified within the deployment-
   declared latency

## Annex C — Cipher-suite floors

The deployment publishes per-protocol cipher-suite floors in
the capability document. Floors are tightened over time as
algorithms are deprecated; partners verify floor compatibility
on each exchange.

## Annex D — Negative-test vectors for protocol layer

| Stimulus                                              | Expected outcome                              |
|-------------------------------------------------------|-----------------------------------------------|
| Keypair-create without HSM attestation                | refused; recorded                              |
| Rotation with successor not yet attested              | refused; recorded                              |
| Phase rollback without elevation                      | refused with rollback-elevation-required      |
| Hybrid CMS signature missing PQ component             | verification fails                             |
| Compromise rotation without HSM compromise-flag       | refused; recorded                              |

## Annex E — Algorithm registry maintenance cadence

The algorithm registry is reviewed on a documented cadence
(typically quarterly). Reviews consider:

- New NIST publications
- Cryptanalytic advances in the literature
- CNSA 2.0 transition guidance updates
- Partner roster's preferred algorithms

Registry updates emit audit-chain entries and notify
subscribers.

## Annex F — Boundary-clock health

The boundary publishes a clock-health record in the
capability document including the primary and secondary time
sources, the most recent successful sync, and the current
drift estimate.

## Annex G — Hardware-token-backed officer signatures

Crypto-officer actions require a hardware-token-backed user
signature (e.g., FIDO2 / WebAuthn-class attestation). The
deployment's identity authority records the hardware token's
attestation in the audit chain alongside the officer action.
A token marked compromised or lost is revoked immediately
and outstanding tokens it backed are refused at the boundary.

## Annex H — Worked TLS 1.3 hybrid negotiation (informative)

A worked hybrid handshake:

1. Client offers `supported_groups` including
   `x25519_kyber768` (hybrid) and `x25519` (classical
   fallback)
2. Server selects `x25519_kyber768`
3. Both endpoints derive shared secret per the hybrid KDF
4. Application data encrypted under the derived secret

The boundary records the negotiated group on each TLS
session for audit; persistent classical-fallback exceeds
the deployment's tolerance triggers a roadmap review.

## Annex I — Out-of-band partner-key bootstrap

Initial partner enrolment uses an out-of-band exchange (e.g.,
diplomatic-pouch, in-person hardware-token swap, signed
postal package via two-person courier). The boundary
verifies the bootstrap channel's evidence record before
admitting the partner's key into the registry. Subsequent
key rotations of a bootstrapped partner re-use the
established trust path; loss of the bootstrap-evidence
record forces a fresh out-of-band exchange.

## Annex J — Disaster-recovery audit-chain reconstruction

If the audit chain experiences a recoverable corruption
(disk failure, partial power loss), reconstruction draws on:

- The most recent regulator-witness mirror snapshot
- Hash-chained backup copies held by partner deployments
  per a documented mutual-attestation arrangement
- Per-actor signed archives of write operations they
  originated

The boundary publishes a reconstruction-evidence record
and reissues a phase-rolled-back marker if the
reconstruction excludes any phase-advance event.
Partners are notified within the deployment-declared
recovery window and re-verify capability documents
before resuming exchanges.

## Annex K — Cross-region replication discipline

For deployments that span multiple regions (DR pairs,
multi-jurisdiction operations), inter-region replication
respects the migration-phase declaration of each region:

- A region in `pq-only` MUST NOT import classical-only
  artifacts from a peer in `classical` phase except via
  hybrid re-signing at the boundary
- Cross-region phase advances are coordinated; a unilateral
  phase-advance in one region without partner readiness
  is logged as an audit-chain warning
- Replication of the audit chain itself is per-region with
  cross-region cross-references for incidents requiring
  multi-region coordination

Partners verify the per-region capability document of the
peer they are exchanging with; capability mismatches
between sibling regions are surfaced explicitly.
