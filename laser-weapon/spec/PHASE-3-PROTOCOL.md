# WIA-laser-weapon PHASE 3 — Protocol Specification

**Standard:** WIA-laser-weapon
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format (PHASE 1)
to the API surface (PHASE 2): authentication of weapon systems,
operators, and range-control authorities; the constrained binary
fire-control encoding (Link 16 J11/J12 mapping); audit-chain
construction; time discipline; eye-safety-zone enforcement
integrity; and the cross-coalition release-authority handshake.

References (CITATION-POLICY ALLOW only):
- STANAG 5516 — Tactical Data Link 16 (J-series messages)
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK), RFC 9162
  (Certificate Transparency 2.0 pattern)
- IEC 60825-1 — eye-safety classification
- ANSI Z136.1 — laser hazard zones
- ISO 11146 — M² measurement
- ISO 13694 — power-density distribution measurement
- WGS-84 — geodetic reference frame

---

## §1 Authentication

Weapon systems, operators, range-control authorities, and integrating
C2 systems authenticate using JWS-signed JWTs issued by the deployment's
identity provider (KA). Token claim layout:

| Claim          | Source                                                      |
|----------------|-------------------------------------------------------------|
| `iss`          | KA URL                                                      |
| `aud`          | the boundary URL                                            |
| `sub`          | weapon-system URN, operator URN, range-control URN          |
| `iat` / `exp`  | issuance/expiry per RFC 7519                                 |
| `wia_role`     | one of: weapon, operator, range-control, c2, fire-control   |
| `wia_doctrine` | declared doctrine                                           |
| `wia_clearance`| classification clearance                                    |
| `wia_roe_scope[]` | for fire-control / engagement authorities, RoE categories the principal may invoke |

Weapon-system tokens are short-lived (≤ 1 hour, refreshed during
warm-up); range-control tokens are session-scoped (≤ 8 hours).

## §2 Token format and signing

Tokens are JWS-signed JWTs (RFC 7515 + RFC 7519). Default signature
algorithm is ES256 (P-256 ECDSA); higher classification levels SHOULD
use ES384. Tactical fire-control loops MAY use Ed25519 for shorter
signatures on narrowband links.

Tokens MUST be sent over TLS 1.3.

## §3 Key management

The KA publishes a JWKS at a fixed well-known URI. Signing keys
rotate at least every 90 days for routine traffic, every 30 days for
operational deployments. Prior keys remain in the JWKS for ≥ 180 days
for verifiability of long-lived audit signatures.

Cross-coalition deployments register peer KAs in a federation manifest
signed by both parties. Private keys live in HSMs (FIPS 140-3 Level 2
or national equivalent, KCMVP for KR).

## §4 Audit chain construction

Every engagement-command intake, eye-safety-zone-check, beam-emission
lifecycle event, thermal-limit excursion, beam-quality publication,
atmospheric record, and range-test schedule emits an AuditEvent.
AuditEvents form a per-deployment hash chain:

```
chain_input  = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

Canonicalisation uses RFC 8785 JSON Canonicalisation Scheme. The chain
root is sealed once per UTC day. Sealed roots MAY be published to a
transparency log following the RFC 9162 pattern when the deployment
policy enables external auditability.

## §5 Tactical bandwidth-constrained encoding

For Link 16 fire-control loops, this PHASE maps onto J-series:

| WIA record           | J-series mapping                              |
|----------------------|-----------------------------------------------|
| Engagement command   | J11.x weapon-direction (DEW class)            |
| Beam-emission start  | J12.x weapon-status (firing)                  |
| Beam-emission end    | J12.x weapon-status (cooldown)                |
| Atmospheric channel  | J3.x amplifying remarks (lightly profiled)    |

The mapping is *projection only*; the canonical record remains the
JSON record at the boundary. The J-series payload includes a hash
reference back to the canonical record for higher-bandwidth retrieval.

## §6 Time discipline

Clocks synchronise to GNSS (multi-constellation receiver). Tactical
fire-control loops require sub-millisecond timing because beam-pointing
inverse-kinematics depends on track-state freshness. Drift exceeding
100 ms suspends new engagement commands; drift exceeding 1 s suspends
weapon-status reporting.

Beam-emission timestamps use microsecond precision where the underlying
clock supports it; all record timestamps are RFC 3339 with offset.

## §7 Eye-safety-zone enforcement integrity

The eye-safety-zone check (PHASE 2 §2) runs as the last gate before
fire instruction. The check incorporates:

- The commanded beam geometry (azimuth, elevation, divergence)
- The weapon's IEC 60825-1 class and aperture power
- The atmospheric channel observation (visibility class)
- The deployment's human-rated zones and friendly-asset zones

A `clear` response is signed by the safety-check service; the boundary
refuses to issue fire instruction without a fresh signed `clear`. The
signed result is recorded in the audit chain so any after-the-fact
deletion or alteration is detectable.

## §8 Release-authority handshake

Cross-coalition release of laser-weapon records (e.g., engagement
outcomes shared with a NATO partner) follows the same handshake as
WIA-missile-defense PHASE 3 §8: dual signatures (originating
release authority + receiving release authority) recorded in the
audit chain. Engagement-decision records released across coalition
borders include the eye-safety-zone-check evidence so partners can
verify safety due-diligence.

## §9 Failure modes

| Failure                                | Behaviour                                       |
|----------------------------------------|-------------------------------------------------|
| KA JWKS unreachable                    | Cached keys honoured until cache expiry          |
| Eye-safety-zone-check service offline  | New engagement commands refused                  |
| Atmospheric channel data missing       | Engagement permitted with `unknown` channel; flagged for review |
| Audit chain write failure              | Operation rejected (consistency w/ missile-defense §9) |
| Time drift > 1 s                       | Weapon-status reporting suspended                |
| Federation manifest expired            | Cross-coalition release refused                  |
| Thermal limits exceeded mid-emission   | Automatic cooldown; emission terminated; flagged |

## Annex A — Algorithm choices (informative)

| Concern                  | Default                        | Notes                          |
|--------------------------|--------------------------------|--------------------------------|
| Token signing            | ES256 (P-256 ECDSA)            | Ed25519 for tactical narrowband|
| Daily-root signing       | ES384 (P-384 ECDSA)            |                                |
| Audit hash               | SHA-256 (RFC 6234)             |                                |
| TLS                      | 1.3 (RFC 8446)                 | 1.2 explicit-list only         |
| Eye-safety-check signature | ES256                        | service-specific signing key   |
| Symmetric at rest (audit)| AES-256-GCM                    |                                |

## Annex B — Conformance levels

| Level     | Scope                                                            |
|-----------|------------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                              |
| Verified  | annual third-party audit                                          |
| Anchored  | continuous evidence package per audit chain transparency          |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex C — Federation manifest worked example (informative)

```yaml
federation:
  parties:
    - {orgRef: "urn:wia:org:rok-army.adcc", kid: "ka-rok-2026"}
    - {orgRef: "urn:wia:org:us-army.380adabde", kid: "ka-us-2026"}
  flows:
    - {from: "rok", to: "us", purposes: ["operational"], records: ["emissions", "outcomes"]}
    - {from: "us", to: "rok", purposes: ["operational"], records: ["engagement-decisions"]}
  weaponClassScope:
    - {fromParty: "us", weaponClass: "HEL-50kW", crossUseAllowed: false}
  expiry: "2027-04-27"
  signatures: [<JWS-by-rok>, <JWS-by-us>]
```

## Annex D — Conformance disclosure path

Implementations declare per-section conformance in their published
capability document (PHASE 2 Annex B). Sections marked `partial` or
`excluded` reference the deployment policy explaining the gap. A
deployment that is `partial` or `excluded` on §1 (Authentication),
§4 (Audit chain), or §7 (Eye-safety-zone enforcement) is non-conformant
overall and cannot claim Deep v3 status.

## Annex E — Time-precision worked example (informative)

| Concern                          | Precision required                             |
|----------------------------------|------------------------------------------------|
| Token issuance time              | second precision                                |
| Eye-safety-zone-check timestamp  | millisecond precision                           |
| Beam-emission start/end          | microsecond precision (laser-pointing kinematics)|
| Thermal sample stream            | millisecond precision (10 Hz)                   |
| Atmospheric record                | second precision                                |

Drift between weapon-system clock and boundary clock greater than
100 ms suspends new engagement-command issuance because beam-pointing
inverse-kinematics depends on track-state freshness at this scale.

## Annex F — Cross-domain reference indexing (informative)

The audit chain indexes cross-domain references for forensic replay:

| Source standard                | Reference field in laser-weapon record    |
|--------------------------------|-------------------------------------------|
| WIA-missile-defense            | `decisionRef`, `targetRef`                |
| WIA-military-communication     | `releaseAuthorityCorrespondence`          |
| WIA-nbc-defense                | `casualtyCoordinationEventRef` (when relevant)|
| range-control                  | `rangeAuthorisationRef`                   |

Forensic replay across two domains requires both audit chains to
be available; the deployment's incident-response procedure SHOULD
include a cross-chain replay drill at least annually.

## Annex G — Service identity (eye-safety-zone planner) (informative)

The eye-safety-zone planner is a separate service identity from the
boundary itself. The planner's signing key is held in its own HSM;
the boundary stores the planner's public key in its trust set.

Operational rotation: the planner rotates its signing key on the
same cadence as the boundary (every 30 days for tactical, every 90
days for strategic). Rotation events are themselves audit events
and the boundary updates its trust set on receipt of the rotation
notification (signed by the prior key, naming the next key).

## Annex H — TLS profile

Tactical fire-control loops use TLS 1.3 with mutual authentication; the deployment's PKI issues both client and server certificates with rotation per Annex A. ChaCha20-Poly1305 is preferred for tactical platforms with limited AES hardware support.

## Annex I — Audit chain throughput

Per-deployment chain throughput SHOULD support the highest engagement-rate scenario; sharding by emissionId hash prefix is permitted at high tempo.
