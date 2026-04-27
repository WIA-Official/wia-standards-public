# WIA-military-communication PHASE 3 — Protocol Specification

**Standard:** WIA-military-communication
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format (PHASE 1)
to the API surface (PHASE 2): authentication of operators and
terminals, the bandwidth-constrained tactical encoding, audit-chain
construction, time discipline, classification handling integrity,
and the cross-coalition release-authority handshake. Where an allied
protocol (STANAG 5066, MMHS) already covers a behaviour, this
standard defers; gaps and the privacy/audit overlay are specified
here.

References (CITATION-POLICY ALLOW only):
- STANAG 4406 — Military Message Handling System
- STANAG 5066 — bandwidth-constrained tactical data exchange
- STANAG 4774 / 4778 — Confidentiality and Information labelling
- MIL-STD-188-141C — interoperability and performance standards for ALE / HF
- MIL-STD-188-110D — interoperability for HF radio modems
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK), RFC 9162 (Certificate Transparency 2.0 pattern)
- ITU-R Recommendations for HF / VHF / UHF / SATCOM frequency planning

---

## §1 Authentication

Operators and terminals authenticate using JWS-signed JWTs issued
by the deployment's identity provider (KA). Identity assertions
carry classification clearance and releasability scope so that a
single token can be evaluated at message-origination time without
extra round trips. Token claim layout:

| Claim          | Source                                                            |
|----------------|-------------------------------------------------------------------|
| `iss`          | KA URL                                                            |
| `aud`          | the boundary URL                                                  |
| `sub`          | terminal URN, operator URN, service URN                           |
| `iat` / `exp`  | issuance/expiry per RFC 7519                                      |
| `wia_role`     | one of: terminal, operator, network-mgmt, gateway, c2             |
| `wia_doctrine` | declared doctrine (NATO, KR, US, JP, FVEY, …)                     |
| `wia_clearance`| classification clearance (e.g., `secret`)                          |
| `wia_releasability[]` | coalition releasabilities the principal can act under       |

A token presented to an endpoint whose declared classification
exceeds `wia_clearance` is rejected with
`urn:wia:milcomms:problem:clearance-insufficient`.

## §2 Token format and signing

Tokens are JWS-signed JWTs (RFC 7515 + RFC 7519). Default signature
algorithm is ES256 (P-256 ECDSA); higher classification levels
SHOULD use ES384 (P-384). Tactical token issuance MAY use Ed25519
to reduce signature size on narrowband links.

Tokens are short-lived: ≤ 8 hours for operator sessions, ≤ 1 hour
for terminals on tactical networks (modulated by drift budget and
roaming requirements). Tokens carry the issuing KA's `kid` so the
boundary can resolve the verification key against the KA's JWKS.

## §3 Key management

The KA publishes a JWKS at a fixed well-known URI. Signing keys
rotate at least every 90 days for routine traffic, every 30 days
for tactical deployments, and on-demand after a personnel-change
event in the issuing authority. Prior keys remain in the JWKS for
≥ 180 days so that long-lived audit signatures remain verifiable.

Cross-coalition deployments register peer KAs in a federation
manifest signed by both parties. A peer's tokens are accepted only
when the peer KA's JWKS resolves and the federation manifest is
current; expired manifests suspend cross-coalition flow.

Private keys live in HSMs (FIPS 140-3 Level 2 or national
equivalent — KCMVP for KR, CRYPTREC for JP, ANSSI for FR). Keys
never leave the HSM; signing is a remote operation.

## §4 Audit chain construction

Every message-origination, classification-validation, addressing
lookup, spectrum allocation, link-state observation aggregation,
bridge event, and EMCON change emits an AuditEvent. AuditEvents
form a per-deployment hash chain:

```
chain_input  = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

Canonicalisation uses RFC 8785 JSON Canonicalisation Scheme. The
chain root is sealed once per UTC day by signing the root with the
deployment's signing key. Sealed roots MAY be published to a
transparency log following the RFC 9162 pattern when the deployment
policy enables external auditability.

Reconstruction failure (proof mismatch, missing event, missing
daily root) is itself a security incident triggering an incident-
response workflow.

## §5 Bandwidth-constrained tactical encoding

Tactical narrowband links cannot carry the full JSON envelope of
PHASE 2. The protocol defines constrained encodings:

- **MMHS short message** — STANAG 4406 envelope with a fixed-size
  classification block; suitable for narrowband HF
- **VMF tactical message** — MIL-STD-2045-47001D Variable Message
  Format with a hash reference back to the canonical record at the
  boundary
- **STANAG 5066 packet** — sub-network access with assured-delivery
  service for short ack-required messages

Constrained encodings are *projection only*; the canonical record
remains the JSON record at the boundary. The constrained payload
includes a hash reference back to the canonical record so the
receiver can request the full record when bandwidth allows.

Constrained payloads are signed using EdDSA Ed25519 (64-byte
signature) so the air-time penalty stays acceptable.

## §6 Time discipline

Clocks synchronise to GNSS (multi-constellation receiver) where
available and fall back to NTPv4 stratum-2 for fixed-installation
terminals. Drift exceeding 500 ms is an operational incident; drift
exceeding 5 s suspends new token issuance for affected terminals
because token validity windows depend on agreed time.

Tactical narrowband terminals carry an embedded time-of-day clock
disciplined by GPS PPS (1 PPS pulse-per-second) where possible.
Terminals without PPS may operate in degraded-time mode with a
recorded warning.

## §7 Classification handling integrity

The boundary enforces classification handling at every operation:

- A message addressed to an endpoint outside the message's
  releasability is refused; no partial delivery
- A bridge between waveforms of different classification ceilings
  is refused unless the bridge has an explicit downgrade
  authorisation signed by the release authority and visible in the
  audit chain
- A read of an audit entry referencing a classified message uses
  the same gate; auditors above the message's classification level
  see the full record, auditors below see a redacted record showing
  only the metadata permitted at their level

The boundary records every classification-gate decision with the
input clearance, the message marking, the gate result, and the
deciding policy version so that auditors can reproduce the
decision.

## §8 Release-authority handshake

Cross-coalition release of military communications follows a
handshake mirroring WIA-nbc-defense PHASE 3 §8:

1. Requesting organisation submits a release request naming
   the records, the receiving organisation, the purpose, and
   the release authority's signature
2. Originating organisation's release officer signs the request
3. Both signatures are recorded in the audit chain
4. The boundary releases the records under the named purpose

A release without both signatures is refused with
`urn:wia:milcomms:problem:release-not-authorised`. Coalition
operations delegate release authority per the standing federation
manifest; ad-hoc release outside the manifest requires both nations'
release officers to sign individually.

## §9 Failure modes

| Failure                              | Behaviour                                         |
|--------------------------------------|---------------------------------------------------|
| KA JWKS unreachable                  | Cached keys honoured until cache expiry           |
| Tactical link drops mid-MMHS packet  | Sender retries; no duplicate event created        |
| Audit chain write failure            | Operation rejected (consistency w/ medical-data-privacy §9) |
| Time drift > 5 s                     | Token issuance suspended for affected terminal    |
| Federation manifest expired          | Peer tokens refused; manual renewal required      |
| Spectrum allocation conflict         | Allocation rejected; conflict surfaced            |
| Classification gate mismatch         | Operation rejected with structured reason         |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

| Concern                     | Default                            | Legacy permitted                |
|-----------------------------|------------------------------------|---------------------------------|
| Token signing               | ES256                              | RS256                           |
| Daily-root signing          | ES384                              | RS256                           |
| Tactical packet signing     | EdDSA Ed25519 (64-byte)            | n/a                             |
| Audit hash                  | SHA-256 (RFC 6234)                 | n/a                             |
| TLS                         | 1.3 (RFC 8446)                     | 1.2 explicit-list               |
| Symmetric at rest (audit)   | AES-256-GCM                        | AES-128-GCM                     |
| HF voice crypto suite       | per national crypto plan           | per national legacy plan         |

Quantum-resistance migration is tracked separately as new NIST
PQ standards reach deployment-grade tactical HSM support.

## Annex B — Federation manifest shape (informative)

```yaml
federation:
  parties:
    - {orgRef: "urn:wia:org:nation-A.signal-cmd", kid: "ka-A-2026"}
    - {orgRef: "urn:wia:org:nation-B.signal-cmd", kid: "ka-B-2026"}
  flows:
    - {from: "nation-A", to: "nation-B", purposes: ["operational"], records: ["messages", "spectrum", "links"]}
    - {from: "nation-B", to: "nation-A", purposes: ["operational"], records: ["messages"]}
  bridgePolicy:
    - {sourceWaveform: "link-16-ja", destWaveform: "link-22-bridged", lossy: false}
  expiry: "2027-04-27"
  signatures: [<JWS-by-A>, <JWS-by-B>]
```

## Annex C — Cross-domain release-handshake worked example (informative)

A coalition partner requests release of a milcomms message that
references an NBC validated event:

1. Partner's release officer signs a request naming both the
   milcomms message ID and the underlying NBC event ID
2. Originating organisation's release officer signs the request,
   acknowledging both the milcomms-side releasability and the
   NBC-side release authority
3. The milcomms boundary verifies the NBC boundary's signature
   on the underlying event before releasing the wrapping message
4. Both signatures are recorded in the audit chain; the cross-
   referenced NBC event's chain is also updated with a release
   notification

The handshake guarantees that a release of a wrapping message
implicitly verifies the underlying-domain release; partners cannot
exfiltrate underlying-domain content by wrapping it in a milcomms
message.

## Annex D — Tactical narrowband budget (informative)

Approximate air-time budget for a 96-byte NBC-1 packet over
STANAG 5066 at 2400 bps:

- payload + signature: 96 + 64 = 160 bytes = 1280 bits → 0.53 s
- ARQ acknowledgement: ~16 bits → ~7 ms
- propagation: variable per HF skywave path

A handful of such packets per minute are operationally feasible;
high-rate streaming is not. Voice-grade waveforms (e.g., MELP at
2400 bps) coexist on the same band by ALE-driven channel selection.

## Annex E — Time discipline summary (informative)

| Source                | Drift target        | Notes                                |
|-----------------------|---------------------|--------------------------------------|
| GNSS multi-constellation | ± 100 ns (PPS)   | preferred for tactical               |
| NTPv4 stratum-2        | ≤ 50 ms              | preferred for fixed installations    |
| Local oscillator only  | < 5 s rated         | degraded mode, recorded warning      |

A terminal that lacks any time source MUST not originate timestamps;
its messages are quarantined at the boundary until time is re-acquired.
