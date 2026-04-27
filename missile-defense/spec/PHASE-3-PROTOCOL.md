# WIA-missile-defense PHASE 3 — Protocol Specification

**Standard:** WIA-missile-defense
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format (PHASE 1)
to the API surface (PHASE 2): authentication of sensors, fusion
engines, engagement authorities, and weapon systems; the constrained
binary tactical encoding for fire-control loops; audit-chain
construction and time discipline; the Link 16 / Link 22 mapping;
and the cross-coalition release-authority handshake.

References (CITATION-POLICY ALLOW only):
- STANAG 5516 — Tactical Data Link 16 (J-series messages)
- STANAG 5522 — Link 22
- MIL-STD-6016 — J-series message realisation
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK), RFC 9162 (Certificate Transparency 2.0 pattern)
- WGS-84 (NIMA TR8350.2) — geodetic reference frame
- ITU-R Recommendations for tactical-link spectrum

---

## §1 Authentication

Every node (sensor, fusion engine, engagement authority, weapon
system) authenticates using JWS-signed JWTs issued by the
deployment's identity provider (KA). Token claim layout:

| Claim          | Source                                                           |
|----------------|------------------------------------------------------------------|
| `iss`          | KA URL                                                            |
| `aud`          | the boundary URL                                                  |
| `sub`          | sensor URN, fusion-engine URN, weapon-system URN, operator URN    |
| `iat` / `exp`  | issuance/expiry per RFC 7519                                      |
| `wia_role`     | one of: sensor, fusion, engagement-authority, weapon, c2          |
| `wia_doctrine` | declared doctrine                                                  |
| `wia_clearance`| classification clearance                                           |
| `wia_releasability[]` | releasability scope                                         |
| `wia_engagement_scope[]` | for engagement authorities, scope of engagement       |

The `wia_engagement_scope` claim is critical: an engagement
authority can decide only on threats inside its declared scope
(geographic and threat-class). A decision outside scope is
rejected with `urn:wia:md:problem:engagement-authority-lacking`.

## §2 Token format and signing

Tokens are JWS-signed JWTs. Default signature algorithm is ES256
(P-256 ECDSA); higher classification levels SHOULD use ES384.
Tactical token issuance for fire-control loops MAY use Ed25519
to reduce signature size. Tokens are short-lived (≤ 4 hours for
operator sessions, ≤ 1 hour for fielded weapon systems).

Tokens carry the issuing KA's `kid` so the boundary can resolve
the verification key against the KA's JWKS.

## §3 Key management

The KA publishes a JWKS at a fixed well-known URI. Signing keys
rotate at least every 90 days for routine traffic, every 30 days
for operationally-deployed networks. Prior keys remain in the
JWKS for ≥ 180 days for long-lived audit signature verification.

Cross-coalition deployments register peer KAs in a federation
manifest signed by both parties. Private keys live in HSMs
(FIPS 140-3 Level 2 or national equivalent).

## §4 Audit chain construction

Every observation, track creation/update, threat assessment,
engagement decision, weapon-status transition, intercept outcome,
and federation correlation emits an AuditEvent. AuditEvents form
a per-deployment hash chain:

```
chain_input  = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

Canonicalisation uses RFC 8785 JSON Canonicalisation Scheme. The
chain root is sealed once per UTC day and signed by the deployment
signing key. Sealed roots MAY be published to a transparency log
(RFC 9162 pattern) when the deployment policy enables external
auditability.

For fire-control loops the audit emission is asynchronous (the
weapon system MUST not delay launch waiting for audit), but the
weapon system MUST persist the audit-event payload locally before
state transition so that no event can be silently dropped. The
boundary reconciles persisted local events with the central chain
within the deployment's operational SLA (PHASE 4 §1).

## §5 Tactical bandwidth-constrained encoding

For Link 16 / Link 22 fire-control loops, the protocol maps WIA
records onto J-series messages:

| WIA record           | J-series mapping                                             |
|----------------------|--------------------------------------------------------------|
| Sensor observation   | J3.x track-position messages                                 |
| Fused track          | J3.0 reference-point + identification messages              |
| Threat assessment    | J7.x threat warning                                          |
| Engagement decision  | J11.x weapon-direction message                               |
| Weapon status        | J12.x weapon-status message                                  |
| Intercept outcome    | J14.x kill assessment                                        |

The mapping is *projection only* — the canonical record remains
the JSON record at the boundary. The J-series payload includes a
hash reference back to the canonical record so the receiver can
request the full record over a higher-bandwidth link when available.

## §6 Time discipline

Clocks synchronise to GNSS (multi-constellation receiver) where
available; tactical fire-control loops require sub-millisecond time
discipline because track propagation is sensitive to timing. Drift
exceeding 100 ms suspends new engagement decisions for the affected
authority; drift exceeding 1 s suspends sensor observation ingest
for the affected sensor.

All record timestamps are RFC 3339 with offset; tactical loops use
microsecond-precision timestamps where the underlying clock supports
it.

## §7 Engagement authority handshake

An engagement decision flows:

1. Threat assessment surfaces (PHASE 1 §5)
2. Engagement authority's tablet receives the assessment and
   surfaces the recommended decision per the doctrine
3. Engagement authority signs the decision with their token
4. The boundary validates the authority's `wia_engagement_scope`
   against the assessment's geo and threat-class
5. The decision is hashed into the audit chain and pushed to the
   weapon system

A decision without a current authority signature is refused; a
decision outside the authority's scope is refused; a decision
referring to an assessment whose confidence band is below the
doctrine threshold is refused.

## §8 Cross-coalition federation

Multi-coalition track correlation and engagement coordination
follow a federation manifest signed by all participating parties.
The manifest enumerates which records flow which direction, which
weapon systems each party may task on the other's tracks, and the
release authorities for cross-party content.

A federation manifest expiry suspends cross-coalition decisions;
manifest renewal is recorded as an AuditEvent visible to all
parties.

## §9 Failure modes

| Failure                              | Behaviour                                                  |
|--------------------------------------|------------------------------------------------------------|
| KA JWKS unreachable                  | Cached keys honoured until cache expiry                    |
| Tactical link drops mid-J11.x        | Engagement authority retransmits; weapon refuses duplicate via decisionRef matching |
| Audit chain write failure (boundary) | Operations rejected on the boundary; weapon-system local persistence continues so loops can recover |
| Time drift > 1 s on sensor           | Sensor observations rejected until drift recovers           |
| Federation manifest expired          | Cross-coalition decisions rejected                          |
| Covariance malformed                 | Observation rejected; sensor flagged for review             |
| Weapon system unhealthy              | Engagement decisions referring to that weapon refused       |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

| Concern                  | Default                                | Notes                              |
|--------------------------|----------------------------------------|------------------------------------|
| Token signing            | ES256 (P-256 ECDSA)                    | tactical may use Ed25519           |
| Daily-root signing       | ES384 (P-384 ECDSA)                    |                                    |
| Tactical packet signing  | EdDSA Ed25519 (64-byte)                | air-time minimisation              |
| Audit hash               | SHA-256 (RFC 6234)                     |                                    |
| TLS                      | 1.3 (RFC 8446)                         | 1.2 explicit-list only             |
| Symmetric at rest        | AES-256-GCM                            |                                    |
| Time discipline          | GNSS multi-constellation, < 1 ms drift | tactical fire-control sensitivity  |

## Annex B — Engagement-authority scope worked example (informative)

A scope claim:

```json
{
  "wia_engagement_scope": [
    {
      "geo": {"type": "Polygon", "coordinates": [...]},
      "kinematicClasses": ["tactical-ballistic", "medium-range-ballistic"],
      "weaponSystems": ["urn:wia:md:ws:patriot-pac3-bty-7", "urn:wia:md:ws:thaad-bty-1"]
    }
  ]
}
```

A decision is acceptable only if the assessment's predicted-impact
area falls inside the scope geo, the kinematicClass is in the list,
and the tasked weapon system is in the list. All three checks must
pass; partial matches are rejected.

## Annex C — Local-persistence reconciliation (informative)

Weapon systems persist their own state-transition records locally.
The reconciliation protocol with the boundary:

1. On reconnection after outage, the weapon system pushes its
   local-buffered records in order
2. The boundary verifies each record's signature, hashes it into
   the chain at the appropriate position (using the recorded
   timestamp as the chain order key)
3. If a record's timestamp falls inside an already-sealed daily
   root, the record is routed to the late-arrival queue and surfaces
   to the operations team for review rather than silently shifting
   the chain
4. The weapon system keeps its local buffer until the boundary
   acknowledges each record; this guarantees no record is silently
   dropped

The protocol is defensive: integrity is preferred over silent loss.

## Annex D — Spectrum coordination crosswalk (informative)

Missile-defence systems share spectrum with civil and other military
users. Coordination flows through WIA-military-communication PHASE 2
§4 (spectrum allocations); this PHASE references the resulting
allocations and refuses observations at frequencies outside the
deployment's coordinated allocations.

The crosswalk:

| Role           | Source standard                                                |
|----------------|----------------------------------------------------------------|
| Allocations    | WIA-military-communication PHASE 1 §6 (spectrum allocations)   |
| Conflict alerts | WIA-military-communication PHASE 2 §4                          |
| Sensor radar   | WIA-missile-defense PHASE 1 §3 (sensor observation)            |
| Frequency band | sensor metadata declares operating band; boundary cross-checks |

A sensor whose declared band falls outside the deployment's
spectrum allocations is suspended pending coordination correction.

## Annex E — Conformance levels (informative)

| Level     | Scope                                                        |
|-----------|--------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                          |
| Verified  | annual third-party audit                                      |
| Anchored  | continuous evidence package per audit chain transparency      |

Implementations declare their level in the capability advertisement
(PHASE 2 §9). Coalition operations typically require Verified or
Anchored from all parties; a deployment claiming Surface only is
not eligible for cross-coalition engagement decisions.

## Annex F — Handover at shift change (informative)

Engagement-authority shift changes are themselves auditable. The
outgoing authority signs a hand-over record naming the assessments
in flight, the active decisions, and the active weapon systems.
The incoming authority signs an acknowledgement of the hand-over
record before assuming the engagement-authority role. The boundary
honours the new authority's signed decisions only after the
acknowledgement is committed; before then the prior authority
remains the binding signer.
