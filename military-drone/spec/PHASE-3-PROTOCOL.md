# WIA-military-drone PHASE 3 — Protocol Specification

**Standard:** WIA-military-drone
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format (PHASE 1)
to the API surface (PHASE 2): authentication of airframes, ground
control stations (GCS), operators, and release authorities; the
constrained binary tactical encoding (Link 16 J-series mapping +
STANAG 4586 UCS interfaces); audit-chain construction; time discipline;
geofence enforcement integrity; weapon-release dual-signature
handshake; and the cross-coalition release-authority handshake.

References (CITATION-POLICY ALLOW only):
- STANAG 4586 — UAV Control System interfaces
- STANAG 5516 — Tactical Data Link 16 (J-series)
- MISB ST 0601, ST 0903 — UAS metadata
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK), RFC 9162 (Certificate Transparency 2.0 pattern)
- WGS-84 — geodetic reference frame
- ICAO Doc 4444 — air-traffic management procedures

---

## §1 Authentication

Airframes, ground control stations, operators, and release authorities
authenticate using JWS-signed JWTs issued by the deployment's identity
provider (KA). Token claim layout:

| Claim          | Source                                                         |
|----------------|----------------------------------------------------------------|
| `iss`          | KA URL                                                         |
| `aud`          | the boundary URL                                               |
| `sub`          | airframe URN, GCS URN, operator URN, release-authority URN     |
| `iat` / `exp`  | issuance/expiry per RFC 7519                                    |
| `wia_role`     | one of: airframe, gcs, operator, release-authority, c2          |
| `wia_doctrine` | declared doctrine                                              |
| `wia_clearance`| classification clearance                                       |
| `wia_roe_scope[]` | RoE categories the principal may invoke                      |

Airframe tokens are short-lived (≤ 1 hour) and refresh during
flight by the GCS. Release-authority tokens are session-scoped
(≤ 8 hours) and require fresh issuance per operational shift.

## §2 Token format and signing

Tokens are JWS-signed JWTs (RFC 7515 + RFC 7519). Default signature
algorithm is ES256 (P-256 ECDSA); higher classification levels SHOULD
use ES384. Tactical narrowband links MAY use Ed25519.

Tokens MUST be sent over TLS 1.3.

## §3 Key management

The KA publishes a JWKS at a fixed well-known URI. Signing keys
rotate at least every 90 days for routine traffic, every 30 days for
operational deployments. Prior keys remain in the JWKS for ≥ 180 days.

Cross-coalition deployments register peer KAs in a federation
manifest signed by both parties. Private keys live in HSMs (FIPS 140-3
Level 2 or national equivalent, KCMVP for KR).

## §4 Audit chain construction

Every mission-plan submission, airframe state event, payload event,
sensor observation, weapon-release authorisation, weapon-release event,
geofence evidence record, lost-link event, and recovery record emits
an AuditEvent. AuditEvents form a per-deployment hash chain:

```
chain_input  = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

Canonicalisation uses RFC 8785 JSON Canonicalisation Scheme. The chain
root is sealed once per UTC day. Sealed roots MAY be published to a
transparency log following RFC 9162 when the deployment policy
enables external auditability.

For high-tempo ISR operations (multiple airframes, 10 Hz state
updates) the chain is sharded by airframeRef hash prefix; sharding
is itself audited so an after-the-fact reshard is detectable.

## §5 STANAG 4586 UCS interfaces

The deployment's GCS speaks STANAG 4586 to the airframe and to the
boundary. The protocol maps STANAG 4586 message classes into PHASE 2:

| STANAG 4586 message class    | PHASE 2 endpoint                          |
|------------------------------|-------------------------------------------|
| Mission planning             | POST /missions, PUT /missions/<>/$retask  |
| Vehicle control              | tactical link to airframe (out of scope of this PHASE; the boundary records the resulting state events) |
| Payload control              | airframe state stream (PHASE 2 §2) + observation publication (PHASE 2 §3) |
| Operator messaging            | sideband, out of scope                    |
| Mission monitoring            | GET /airframes/<>/state/$subscribe        |

Mapping is *projection* — the canonical record at the boundary is
the JSON record from PHASE 2; STANAG 4586 messages on the tactical
link reference the canonical record by URN.

## §6 Tactical bandwidth-constrained encoding (Link 16)

For Link 16 fire-control loops:

| WIA record                     | J-series mapping                                |
|--------------------------------|-------------------------------------------------|
| Airframe state                 | J3.x position-and-track of the airframe         |
| Sensor observation             | J3.x track of the observed object               |
| Weapon-release authorisation   | J11.x weapon-direction (UAS class)              |
| Weapon-release event           | J12.x weapon-status (released)                  |
| Recovery record                | J17.x amplifying remarks                         |

Constrained encoding is *projection only*; the canonical record
remains at the boundary. The J-series payload includes a hash
reference back to the canonical record.

## §7 Time discipline

Airframe clocks discipline to GNSS PPS (multi-constellation receiver:
GPS + Galileo + BeiDou + GLONASS) where available, falling back to
oscillator-only with a recorded warning. Drift exceeding 100 ms
suspends new mission-plan acceptance because route timing depends on
agreed time. Drift exceeding 5 s suspends weapon-release
authorisation because release-condition timing depends on tight time
discipline.

All record timestamps are RFC 3339 with offset; tactical loops use
microsecond precision where the underlying clock supports it.

## §8 Weapon-release dual-signature handshake

A weapon release flows under dual signatures:

1. The operator (in the GCS) signs the release request with their
   token's signing key
2. The release authority (in the operations centre, at a higher
   command level) signs the release request with their token's
   signing key
3. The boundary verifies both signatures + the RoE authorisation
   (`wia_roe_scope` covering the engagement category) before issuing
   the fire instruction to the airframe over the tactical link
4. The airframe acknowledges receipt; the airframe's release event
   is signed by the airframe's signing key

A release event with fewer than two signatures (operator + release
authority) is rejected at the boundary with
`urn:wia:md-uas:problem:dual-signature-required`. An airframe that
attempts to fire without a fresh signed authorisation is treated as
a safety-override event and triggers the deployment's incident
response.

## §9 Failure modes

| Failure                              | Behaviour                                          |
|--------------------------------------|----------------------------------------------------|
| KA JWKS unreachable                  | Cached keys honoured until cache expiry             |
| Tactical link drops mid-mission      | Lost-link behaviour from mission plan invoked       |
| Weapon-release authorisation expires | Release refused; new authorisation required         |
| Civil-airspace coordination revoked  | Airframe diverted to closest approved airspace      |
| Audit chain write failure            | Mission-plan acceptance refused; in-flight ops continue with local persistence |
| Time drift > 5 s                     | Release authorisation suspended                     |
| Federation manifest expired          | Cross-coalition release refused                     |

## Annex A — Algorithm choices (informative)

| Concern                  | Default                       | Notes                              |
|--------------------------|-------------------------------|------------------------------------|
| Token signing            | ES256 (P-256 ECDSA)           | Ed25519 for tactical narrowband    |
| Daily-root signing       | ES384 (P-384 ECDSA)           |                                    |
| Tactical packet signing  | EdDSA Ed25519 (64-byte)       | air-time minimisation               |
| Audit hash               | SHA-256 (RFC 6234)            |                                    |
| TLS                      | 1.3 (RFC 8446)                | 1.2 explicit-list only             |
| Symmetric at rest (audit)| AES-256-GCM                   |                                    |

## Annex B — Federation manifest worked example (informative)

```yaml
federation:
  parties:
    - {orgRef: "urn:wia:org:rok-army.adcc", kid: "ka-rok-2026"}
    - {orgRef: "urn:wia:org:us-army.380adabde", kid: "ka-us-2026"}
  flows:
    - {from: "rok", to: "us", purposes: ["operational"], records: ["state", "observations", "releases"]}
    - {from: "us", to: "rok", purposes: ["operational"], records: ["state", "observations"]}
  weaponReleaseScope:
    - {fromParty: "us", airframeClass: "rok-tactical", crossUseAllowed: false}
  expiry: "2027-04-27"
  signatures: [<JWS-by-rok>, <JWS-by-us>]
```

## Annex C — Time-precision worked example (informative)

| Concern                          | Precision                      |
|----------------------------------|--------------------------------|
| Mission-plan timestamp           | second                         |
| Airframe state stream             | millisecond (1 Hz to 10 Hz)    |
| Sensor observation                | sub-second (millisecond-level) |
| KLV metadata stream               | per-frame (typically 30 Hz)    |
| Weapon-release event              | microsecond (fire-control)     |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex D — Lost-link recovery worked example (informative)

1. Airframe loses BLOS-SATCOM link at 09:31:14; LOS-RF was already
   beyond range
2. Onboard logic invokes `lostLinkBehaviour: return-to-base` per
   mission plan
3. Airframe persists state events locally to flash-storage during
   the lost-link window
4. At 09:43:07 LOS-RF is re-acquired as the airframe approaches base
5. Airframe pushes its locally-persisted state events to the boundary
   via PHASE 2 §2; the boundary inserts them into the chain at the
   recorded timestamps and emits a `lost-link-recovery` audit event
6. Recovery record (PHASE 1 §9) is created at landing

Late-arrival events with timestamps inside an already-sealed daily
root are routed to the late-arrival queue and surface to operations
for review.

## Annex E — Conformance levels (informative)

| Level     | Scope                                                                 |
|-----------|-----------------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                                   |
| Verified  | annual third-party audit (STANAG 4586 conformance + RoE compliance)    |
| Anchored  | continuous evidence package per audit chain transparency               |

Coalition operations typically require Verified or Anchored from all
parties; cross-coalition weapon-release authority requires Anchored.

## Annex F — Tactical narrowband budget (informative)

For STANAG 4586 narrowband TC links to a low-bandwidth airframe:

| Message class                | Approx. on-air time @ 9.6 kbps |
|------------------------------|--------------------------------|
| Vehicle-control command      | 50-200 ms                      |
| Mission-plan dispatch (small)| 1-5 s                          |
| State stream (compact)       | 10-50 ms / sample              |
| KLV metadata (frame)         | per-vendor compression         |

Higher-bandwidth airframes (SATCOM-equipped MALE/HALE) carry the
full PHASE 2 §3 KLV stream over the dedicated wideband link; the
boundary's tactical link still carries the canonical state events.

## Annex G — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. STANAG 4586 versions
bump independently from this PHASE; the deployment policy maps each
PHASE version to the STANAG 4586 version it honours so coalition
partners' GCSs interoperate. Deprecation enters a 12-month sunset
window with migration notes recorded in the audit chain.

## Annex H — Conformance disclosure

Implementations declare per-section conformance in their published capability document. Sections marked partial or excluded reference the deployment policy. A deployment that is partial or excluded on §1 (Authentication), §4 (Audit chain), §7 (Time discipline), or §8 (Dual-signature handshake) is non-conformant overall.
