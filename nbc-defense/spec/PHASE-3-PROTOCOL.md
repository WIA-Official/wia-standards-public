# WIA-nbc-defense PHASE 3 — Protocol Specification

**Standard:** WIA-nbc-defense
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols that bind the data format
(PHASE 1) to the API surface (PHASE 2): authentication for sensors
and operators, the bandwidth-constrained tactical encoding, audit
chain construction, time discipline, sample chain-of-custody
integrity, and the cross-organisation release-authority handshake.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 6066 (TLS extensions, SNI)
- IETF RFC 7515 (JWS), RFC 7519 (JWT), RFC 7517 (JWK)
- IETF RFC 9162 (Certificate Transparency 2.0) — pattern for hash-chain transparency
- STANAG 5066 — bandwidth-constrained tactical data exchange (HF radio)
- STANAG 4694 — NATO Common Data Exchange (CBRN-specific subset)
- STANAG 4677 — Dismounted Soldier Reference Architecture (sensor on-soldier integration)
- ISO 17025:2017 — testing-and-calibration laboratory competence

---

## §1 Authentication

Sensors and operators authenticate using JWS-signed JWTs. The
authoritative identity provider is the deployment's key authority
(KA), which signs identity assertions and publishes its JWKS at a
fixed URI. Token claim layout:

| Claim       | Source                                                    |
|-------------|-----------------------------------------------------------|
| `iss`       | KA URL                                                    |
| `aud`       | the boundary's URL                                         |
| `sub`       | sensor URN, operator URN, or service URN                   |
| `iat`/`exp` | issuance/expiry per RFC 7519                              |
| `wia_role`  | one of: sensor, operator, plume-modeller, lab-analyst, c2 |
| `wia_doctrine` | declared doctrine (NATO, KR, US, JP, etc.)             |
| `wia_clear` | release-authority clearance (handling caveat)             |

Sensor tokens are short-lived (≤ 1 hour) and carry the sensor's
specific permissions. Operator tokens are session-scoped (≤ 8 hours)
and carry the operator's role at the rate the C2 system grants.

## §2 Token format and signing

Tokens are JWS-signed JWTs (RFC 7515 + RFC 7519). The default
signature algorithm is ES256 (P-256 ECDSA). Higher classification
levels SHOULD use ES384 or, in legacy environments, RS256. Tokens
are rejected when:

- Their `wia_clear` is below the resource's required handling caveat
- Their `wia_doctrine` does not match the deployment's federation
  agreement with the issuing nation's KA
- Their issuance time exceeds the deployment's clock-skew budget
  (PHASE 3 §6)

Tokens MUST be sent over TLS 1.3; legacy tactical links carrying
TLS 1.2 are permitted only on legs explicitly listed in the
deployment policy.

## §3 Key management

The KA publishes a JWKS at a fixed well-known URI. Signing keys
rotate at least every 90 days for routine traffic and at least
every 30 days for tactical deployments. Prior keys remain in the
JWKS for ≥ 180 days so that long-lived audit signatures remain
verifiable. Private keys live in HSMs (FIPS 140-3 Level 2 or
national-equivalent for KR/JP/EU deployments).

Cross-coalition deployments register peer KAs in a federation
manifest signed by both parties. A peer's tokens are accepted only
when the peer KA's JWKS resolves and the federation manifest is
current.

## §4 Audit-chain construction

Every event ingest, query, promotion, plume run, work-order action,
triage record creation, and report rendering emits an AuditEvent.
AuditEvents form a per-deployment hash chain:

```
chain_input  = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

Canonicalisation uses RFC 8785 JSON Canonicalisation Scheme. The
chain root is sealed once per UTC day by signing the root with the
deployment's signing key; sealed roots MAY be published to a
transparency log following the RFC 9162 pattern when the deployment
policy enables external auditability.

A reconstruction failure (proof mismatch, missing event, missing
daily root) is itself a security incident that triggers a parallel
incident-response workflow.

## §5 Bandwidth-constrained tactical encoding

Tactical links (HF radio, narrow-band satellite) cannot carry the
full JSON payloads of PHASE 2. The protocol defines a constrained-
binary encoding for the most common message shapes:

- NBC-1 initial detection — fixed-size 96-byte packet over STANAG 5066
- NBC-3 plume forecast contour — variable-size polygon with
  Douglas-Peucker simplification at the boundary

The constrained encoding is *projection only*: the canonical record
remains the JSON record at the boundary, and the constrained packet
includes a hash reference back to the canonical record so that the
recipient can request the full record over a higher-bandwidth link
when available.

Constrained packets are signed using a shorter EdDSA signature
(Ed25519, 64-byte signature) so that the air time penalty is
acceptable.

## §6 Time discipline

Clocks are synchronised to GNSS time (GPS + Galileo + BeiDou +
GLONASS multi-constellation receiver) where available, falling
back to NTPv4 stratum-2 for fixed-installation sensors. Drift
exceeding 500 ms is treated as an operational incident; drift
exceeding 5 s suspends new token issuance for the affected sensor
because token validity windows depend on agreed time.

All record timestamps are RFC 3339 with offset; the deployment
expresses the local time-zone offset explicitly, never assumes
UTC, so that fielded operators can read times without mental
arithmetic.

## §7 Sample chain-of-custody integrity

The chain-of-custody log defined in PHASE 1 §5 is hash-chained at
the per-sample level: each transfer event references the prior
event's hash. Sealed-bag identifiers are physical-paper QR codes
backed by the same chain so that a misplaced bag is detectable
post hoc.

The receiving laboratory verifies the chain on receipt and records
the verification result as a chain entry. A verification failure
(broken seal, hash mismatch, missing transfer) demotes the sample
back to `presumptive` evidence weight; a `validated` event derived
from a broken-custody sample is a downgrading incident that the
boundary surfaces to the operations team.

## §8 Release-authority handshake

Cross-organisation release of NBC records (e.g., from a national
defence force to a public-health authority, or from a coalition
partner to its national capital) follows a handshake:

1. The requesting organisation submits a release request naming
   the records, the receiving organisation, the purpose, and the
   release authority's signature
2. The originating organisation's release officer signs the request
3. Both signatures are recorded in the audit chain
4. The boundary releases the records under the named purpose

A release without both signatures is refused with
`urn:wia:nbc:problem:release-not-authorised`. Coalition operations
delegate release authority per the standing federation manifest;
ad-hoc release outside the manifest requires both nations' release
officers to sign individually.

## §9 Failure modes

| Failure                                | Behaviour                                        |
|----------------------------------------|--------------------------------------------------|
| KA JWKS unreachable                    | Cached keys honoured until cache expiry          |
| Tactical link drops mid-NBC-1 packet   | Sender retries on next contact; no duplicate event created |
| Audit chain write failure              | Operation rejected (consistency w/ medical-data-privacy §9) |
| Plume model run failure                | Run marked `failed`; client polls and retries with adjusted inputs |
| Time drift > 5 s                       | Token issuance suspended for affected sensor     |
| Federation manifest expired            | Peer tokens refused; manual renewal required     |
| Sample seal broken at lab              | Sample demoted; boundary records demotion event   |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

| Concern               | Default                                | Legacy permitted   |
|-----------------------|----------------------------------------|--------------------|
| Token signature       | ES256 (P-256 ECDSA)                    | RS256              |
| Daily-root signature  | ES384 (P-384 ECDSA)                    | RS256              |
| Tactical packet sig   | EdDSA Ed25519 (64-byte signature)      | n/a                |
| Audit hash            | SHA-256 (RFC 6234)                     | n/a                |
| TLS                   | 1.3 (RFC 8446)                         | 1.2 explicit-list  |
| Symmetric at rest     | AES-256-GCM                            | AES-128-GCM        |

Quantum-resistance is out of scope until NIST PQ standards are
ready for deployment-grade tactical HSMs.

## Annex B — Federation manifest shape (informative)

```yaml
federation:
  parties:
    - {orgRef: "urn:wia:org:nation-A.cbrn-cmd", kid: "ka-A-2026"}
    - {orgRef: "urn:wia:org:nation-B.cbrn-cmd", kid: "ka-B-2026"}
  flows:
    - {from: "nation-A", to: "nation-B", purposes: ["operational"], records: ["events", "plume", "work-orders"]}
    - {from: "nation-B", to: "nation-A", purposes: ["operational"], records: ["events"]}
  expiry: "2027-04-27"
  signatures: [<JWS-by-A>, <JWS-by-B>]
```

A federation manifest with one signature missing is rejected by
both peers; the manifest is the live agreement, not a draft.

## Annex C — Air-gap mode (informative)

Some deployments operate without persistent network connectivity
(submarine-based labs, isolated forward-deployed teams). The
protocol supports air-gap mode:

- Tokens are issued at long lifetimes (≤ 7 days) signed by an
  embedded KA local to the platform
- Records accumulate in the local audit chain; the chain root is
  signed locally
- On reaching a connected node, the local chain is exported with
  its signed root; the receiving deployment merges the segment as
  a federated chain segment with a manifest signed by both parties
- Air-gap segments are tagged so that auditors can identify them
  and apply heightened scrutiny on temporal claims (the local
  chain has no transparency log during the air-gap window)

Air-gap mode is a deployment-policy choice, not a default. A
deployment in air-gap mode SHOULD limit the duration to the
shortest window necessary so that audit visibility is restored
quickly.

## Annex D — Coalition-grade key strength (informative)

Coalition operations involving multi-national tokens prefer the
strongest mutually-supported algorithm. Defaults:

- Within a single-nation deployment: ES256
- Within a NATO-aligned coalition: ES384
- Within a deployment that includes a nation requiring national-
  algorithm conformance (KR KCMVP, JP CRYPTREC, FR ANSSI): the
  deployment policy enumerates the alternative algorithm; tokens
  carry a `wia_alg` claim naming the algorithm so that downstream
  verifiers know which national HSM is required for verification

Keys for coalition signatures are escrowed only when the federation
manifest names an escrow agent; default is no escrow.

## Annex E — Algorithm and key-rotation summary (informative)

The protocol's default cryptographic posture is summarised below.
Quantum-resistance migration is tracked separately as new NIST PQ
standards reach deployment-grade tactical HSM support.

| Concern                  | Default                        | Rotation cadence                  |
|--------------------------|--------------------------------|-----------------------------------|
| Token signing            | ES256                           | every 90 days (routine)           |
|                          |                                 | every 30 days (tactical)          |
| Daily-root signing       | ES384                           | every 180 days                    |
| Tactical packet signing  | EdDSA Ed25519                   | every 30 days                     |
| TLS server certificate   | RSA 2048 or ECC P-256           | every 365 days                    |
| Symmetric at rest        | AES-256-GCM                     | per data-class deployment policy  |

Rotation events are themselves AuditEvents and form a pinned record
in the chain so that auditors can confirm the timeline of which key
was active when.

## Annex F — Tactical timing budget (informative)

For STANAG 5066 narrowband links, the protocol allows for slow
round-trip but requires that the sequence be deterministic:

| Step                                             | Budget                  |
|--------------------------------------------------|-------------------------|
| Sensor-side packet preparation                   | ≤ 200 ms                |
| Modem on-air time for 96-byte NBC-1 packet       | per modem capability    |
| Receiver acknowledgement                         | ≤ 5 s after on-air end  |
| Boundary canonicalisation and audit emission     | ≤ 1 s                   |
| Operator-visible rendering                       | ≤ 30 s end-to-end       |

Deployments tighten these timings where modem capability allows;
loosening them requires sign-off from operational command.
