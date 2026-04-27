# WIA-military-robot PHASE 3 — Protocol Specification

**Standard:** WIA-military-robot
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format (PHASE 1)
to the API surface (PHASE 2): authentication of platforms, GCSs,
operators, release authorities; the constrained binary tactical
encoding for tele-operation and mission control; audit-chain
construction; time discipline (with special handling for UUV
acoustic-modem latency); autonomy-envelope enforcement; weapon-
release dual-signature handshake; and the cross-coalition release-
authority handshake.

References (CITATION-POLICY ALLOW only):
- STANAG 4856 — UGV common architecture
- STANAG 4677 — Dismounted Soldier Reference Architecture
- STANAG 5516 — Tactical Data Link 16
- IEEE P1872 — Robotics ontologies
- IEC 61508 — Safety Integrity Levels
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK), RFC 9162 (Certificate Transparency 2.0 pattern)
- WGS-84 — geodetic reference frame

---

## §1 Authentication

Platforms, GCSs, operators, release authorities, and integrating C2
systems authenticate using JWS-signed JWTs. Token claim layout is
identical to WIA-military-drone PHASE 3 §1 with `wia_role` extended
to `{ platform, gcs, operator, release-authority, c2,
autonomy-supervisor }`.

Platform tokens are short-lived (≤ 1 hour). UUV deployments may
issue longer-lived tokens (≤ 24 hours) because acoustic-modem
re-authentication round-trips are expensive; the longer lifetime is
explicitly declared in the deployment policy.

## §2 Token format and signing

Tokens are JWS-signed JWTs. Default ES256 (P-256 ECDSA); higher
classification levels SHOULD use ES384. UUV acoustic-modem links
SHOULD use Ed25519 for shorter signatures.

Tokens MUST be sent over TLS 1.3 on RF/wired links; UUV acoustic
links use a per-link encryption profile out of scope of this PHASE
that wraps the JWS payload.

## §3 Key management

The KA publishes a JWKS at a fixed well-known URI. Signing keys
rotate at least every 90 days for routine traffic, every 30 days for
operational deployments. Prior keys remain in the JWKS for ≥ 180
days. Private keys live in HSMs (FIPS 140-3 Level 2 or national
equivalent).

## §4 Audit chain construction

Every mission-plan submission, platform state event, sensor observation,
weapon-release authorisation, weapon-release event, geofence evidence
record, autonomy declaration, mishap record, and lost-link event emits
an AuditEvent. Hash-chain construction follows the same shape as
WIA-military-drone PHASE 3 §4.

Late-arriving events from acoustic-modem-buffered UUV uplinks are
inserted at recorded timestamps; events whose timestamp falls inside
an already-sealed daily root are routed to the late-arrival queue and
surface to operations for review (per the WIA-missile-defense pattern).

## §5 Tactical bandwidth-constrained encoding

Tele-operation links carry compact command/feedback packets:

- **Wired-fibre tether** (EOD robots): full PHASE 2 stream over the
  fibre (high bandwidth, low latency)
- **RF-LOS** (route-clearance, sentry): compact tele-op packets with
  full PHASE 2 records pushed every 100 ms or on state change
- **Acoustic-modem** (UUV): packed binary with time-stamped commands,
  acknowledgements piggybacked; per-deployment bandwidth budget
  determines packet density

Constrained encodings are *projection*; the canonical record at the
boundary is the JSON record from PHASE 2.

## §6 Time discipline

Land/surface platforms discipline to GNSS PPS. UUVs operating below
GNSS reception use inertial navigation system (INS) drift-corrected
on each surface or on each acoustic-modem time-pulse from a surface
ship; the deployment policy enumerates the acceptable INS drift
budget per UUV class (typically ≤ 10 m / hour of dive).

All record timestamps are RFC 3339 with offset; sub-surface UUVs
record both local INS time and the most-recent GNSS-disciplined
reference so post-mission reconstruction can correct drift.

## §7 Autonomy-envelope enforcement

The autonomy-envelope check runs on every platform action that could
exceed the declared envelope:

- The boundary holds the signed declaration (PHASE 1 §8) for the
  current mission
- The platform's onboard supervisor cross-checks each commanded /
  autonomous action against the envelope before execution
- Exceedances either auto-escalate to the human-on-loop operator (if
  declared `bounded-autonomy`) or auto-abort (if declared
  `fully-autonomous` with no human-on-loop)

A platform that violates its envelope without auto-escalation is
treated as a safety-critical fault and triggers mishap reporting.

## §8 Weapon-release dual-signature handshake

Identical structure to WIA-military-drone PHASE 3 §8: operator +
release-authority signatures, RoE authorisation check, plus an
additional autonomy-level check (the platform's declared autonomy
level must permit weapon-release of the requested category — fully-
autonomous lethal release is out of scope of this PHASE in any
configuration).

## §9 Failure modes

| Failure                              | Behaviour                                              |
|--------------------------------------|--------------------------------------------------------|
| KA JWKS unreachable                  | Cached keys honoured until cache expiry                 |
| Tactical link drops                  | Lost-link behaviour from mission plan invoked           |
| Acoustic-modem window missed (UUV)   | Platform retries on next surface or per dive plan       |
| Autonomy-envelope violation          | Auto-escalate or auto-abort per declaration             |
| Audit chain write failure            | Mission-plan acceptance refused; in-flight ops continue with local persistence |
| Time drift (UUV INS) > policy        | Mission auto-aborted unless surface re-fix possible     |

## Annex A — Algorithm choices (informative)

| Concern                  | Default                       | Notes                              |
|--------------------------|-------------------------------|------------------------------------|
| Token signing            | ES256 (P-256 ECDSA)           | Ed25519 for UUV acoustic links     |
| Daily-root signing       | ES384 (P-384 ECDSA)           |                                    |
| Audit hash               | SHA-256 (RFC 6234)            |                                    |
| TLS                      | 1.3 (RFC 8446)                | RF/wired only; acoustic uses per-link profile |
| Symmetric at rest (audit)| AES-256-GCM                   |                                    |

## Annex B — UUV-specific time discipline (informative)

A worked UUV time-discipline cycle:

1. Surface platform launches UUV with GNSS-disciplined clock; INS
   at zero drift
2. Dive begins at t=0; INS drift accumulates per platform spec
3. At t=2 hours, surface ship pings acoustic time-pulse; UUV resyncs
   INS clock to acoustic-pulse-derived time (with bounded acoustic-
   propagation correction)
4. UUV continues dive; drift restarts from new reference
5. UUV surfaces at t=8 hours; GNSS resynchronises clock; logged INS
   drift over the dive is recorded for post-mission analysis

Records timestamped during a dive carry an `inferredFromInsAtTimeRef`
field naming the most-recent reference + estimated drift uncertainty.

## Annex C — Conformance disclosure

Implementations declare per-section conformance in their published
capability document. Sections marked `partial` or `excluded` reference
the deployment policy. A deployment that is `partial` or `excluded`
on §1 (Authentication), §4 (Audit chain), §6 (Time discipline), §7
(Autonomy-envelope enforcement), or §8 (Dual-signature handshake)
is non-conformant overall.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex D — Federation manifest worked example (informative)

```yaml
federation:
  parties:
    - {orgRef: "urn:wia:org:rok-army.adcc", kid: "ka-rok-2026"}
    - {orgRef: "urn:wia:org:us-army.380adabde", kid: "ka-us-2026"}
  flows:
    - {from: "rok", to: "us", purposes: ["operational"], records: ["state", "observations", "releases"]}
    - {from: "us", to: "rok", purposes: ["operational"], records: ["state", "observations"]}
  weaponReleaseScope:
    - {fromParty: "us", platformDomain: "rok-ugv-eod", crossUseAllowed: false}
  expiry: "2027-04-27"
  signatures: [<JWS-by-rok>, <JWS-by-us>]
```

A manifest with one signature missing is rejected by both peers.

## Annex E — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. STANAG 4856 / 4677
versions bump independently. Deprecation enters a 12-month sunset
window with migration notes recorded in the audit chain.

## Annex F — Time-precision worked example (informative)

| Concern                          | Precision required                             |
|----------------------------------|------------------------------------------------|
| Mission-plan timestamp           | second precision                                |
| Platform state stream (ground)   | millisecond precision                           |
| Platform state stream (UUV)      | per acoustic window + INS drift annotation     |
| Sensor observation                | sub-second precision (millisecond-level)        |
| Weapon-release event              | millisecond (slower than UAS — closer ranges,  |
|                                  | smaller dwell budgets)                          |
| Mishap incidentTimestamp          | millisecond                                     |

UUV records timestamped during a dive carry both the local INS time
and the most-recent GNSS-disciplined reference; post-mission
reconstruction corrects drift using the recorded reference.

## Annex G — Cross-domain reference indexing (informative)

| Source standard                | Reference field in robot record                |
|--------------------------------|------------------------------------------------|
| WIA-missile-defense            | `decisionRef`, `targetRef`                     |
| WIA-military-communication     | `releaseAuthorityCorrespondence`               |
| WIA-nbc-defense                | NBC-sensor observation cross-ingest            |
| WIA-medical-data-privacy       | casualty-evacuation reconnaissance             |
| WIA-military-drone             | shared GCS infrastructure                      |

## Annex H — Federation manifest signing

Federation manifests are signed by both parties' KAs. The boundary
caches the manifest at the deployment-policy interval (typically 1 hour);
expired manifests suspend cross-coalition flow. Renewal is signed and
recorded as an AuditEvent visible to both parties.

## Annex I — Conformance levels (informative)

| Level     | Scope                                                                |
|-----------|----------------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                                  |
| Verified  | annual third-party audit (STANAG 4856 / 4677 + IEC 61508 SIL)         |
| Anchored  | continuous evidence package per audit chain transparency              |

Coalition operations typically require Verified or Anchored from all
parties; cross-coalition weapon-release authority requires Anchored.

## Annex J — Audit-chain throughput

For high-tempo ops the chain is sharded by platformRef hash prefix; sharding is itself audited so an after-the-fact reshard is detectable.

## Annex K — TLS profile

RF/wired links use TLS 1.3 with mutual authentication; ChaCha20-Poly1305 is preferred on tactical radios with limited AES hardware. UUV acoustic links use a per-link encryption profile out of scope of this PHASE.
