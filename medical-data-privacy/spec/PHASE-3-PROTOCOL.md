# WIA-medical-data-privacy PHASE 3 — Protocol Specification

**Standard:** WIA-medical-data-privacy
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the protocols that bind the data format (PHASE 1)
to the API interface (PHASE 2): authentication, key management, audit
chain construction, identity-broker rotation, retention enforcement,
and the cross-controller disclosure handshake. Where a healthcare-
specific protocol exists (SMART App Launch, IHE ATNA), this standard
defers to it; where one does not, this PHASE specifies the gap.

References (CITATION-POLICY ALLOW only):
- HL7 SMART App Launch 2.2 — clinician/app authentication and scope strings
- IHE ATNA — Audit Trail and Node Authentication (BITS-1.0 transport)
- IETF RFC 7515 (JWS), RFC 7519 (JWT), RFC 7517 (JWK), RFC 7518 (JWA)
- IETF RFC 8446 (TLS 1.3), RFC 6066 (TLS extensions, SNI)
- IETF RFC 9162 (Certificate Transparency 2.0) — pattern for hash-chain transparency
- ISO/IEC 27001:2022 §A.5–§A.8 — information security controls
- ISO/IEC 27018:2019 — controller-to-processor PII flows in cloud
- NIST SP 800-66r2 — implementing the HIPAA Security Rule (informational mapping)
- KISA "보건의료 정보 보호 가이드" — for K-PIPA implementations (informational mapping)

---

## §1 Authentication

Clinician applications authenticate using SMART App Launch 2.2:

- `launch-ehr` flow for in-EHR launch (clinician identity carried by the EHR session)
- `launch-standalone` for external apps (full OAuth 2.0 authorisation_code with PKCE)
- `client_credentials` for server-to-server (research pipelines, regulator gateways)

Scope strings combine the SMART resource scope with the PHASE 1 §4
purpose vocabulary. Examples:

- `patient/Observation.rs` (SMART) + `purpose:TREAT` (this standard)
- `system/Patient.rs purpose:HRESCH`

A token MUST carry both forms; missing the purpose suffix is treated
as an unscoped token and rejected at the edge. Tokens are short-lived
(≤15 minutes for clinician sessions, ≤5 minutes for system tokens).

## §2 Token format

Tokens are JWS-signed JWTs (RFC 7515 + RFC 7519). The header uses
ES256 or ES384 (P-256 / P-384 ECDSA). RS256 is permitted for
backward compatibility but new deployments SHOULD use ECDSA. Mandatory
claims:

| Claim     | Source                                                    |
|-----------|-----------------------------------------------------------|
| `iss`     | the issuing authorisation server's URL                    |
| `aud`     | the resource server's FHIR base URL                       |
| `sub`     | the principal (clinician URN or service URN)              |
| `iat`     | issuance time (RFC 7519 §4.1.6)                           |
| `exp`     | expiry time                                               |
| `scope`   | space-separated SMART scopes including `purpose:<X>`      |
| `wia_org` | the controller organisation ID                            |
| `wia_jur` | the jurisdiction declared by the controller (PHASE 1 §1)  |

Tokens MUST be sent over TLS 1.3 (RFC 8446); TLS 1.2 is permitted only
for legacy interfaces explicitly listed in the deployment policy.

## §3 Key management

Each controller publishes a JSON Web Key Set (RFC 7517) at a fixed
well-known URI: `/.well-known/jwks.json`. The signing key for
consent receipts and AuditEvent rollups MUST appear in the JWKS
with `use: sig` and `kid` matching the receipt's `kid` header.

Keys rotate at least every 90 days. After rotation, the prior key
remains in the JWKS for ≥180 days so that signatures on long-lived
receipts can still be verified. Rotation evidence (the prior key's
fingerprint, the new key's fingerprint, the rotation timestamp) is
itself an AuditEvent.

Private keys live in an HSM (FIPS 140-3 Level 2 or equivalent
national certification — for KR deployments KCMVP is acceptable).
Keys never leave the HSM; signing is a remote operation.

## §4 Audit-chain construction

AuditEvents form a per-controller hash chain. For each event:

```
chain_input  = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

Canonicalisation uses RFC 8785 JSON Canonicalisation Scheme on the
FHIR JSON (with FHIR's element ordering preserved). The chain root
is sealed once per UTC day (`23:59:59.999Z`) by signing the root
with the controller's signing key.

The signed daily root MAY be published to a transparency log if the
deployment policy enables it. If it is, the publishing protocol
follows the pattern of RFC 9162 Certificate Transparency 2.0
(append-only Merkle tree, signed tree head, inclusion proofs). If
it is not, the daily root is held in a controller-internal
append-only store with offline backups.

Auditors reconstructing a window of activity request:

1. The signed daily root for each day in the window
2. The Merkle inclusion proof for each event of interest
3. The events themselves

Reconstruction failure (proof mismatch, missing event, missing
daily root) is itself an incident requiring breach-notification
review per PHASE 1 §8.

## §5 Identity broker

Re-identification operations (mapping `subjectRef` to MRN, or
linking two `subjectRef` values per PHASE 1 §5) flow through the
identity broker. The broker exposes a private API reachable only
from the controller's network, never from the open internet.

The broker holds:
- the MRN ↔ `subjectRef` table (encrypted at rest with a key
  separate from the FHIR store)
- the `subjectRef` ↔ `subjectRef` linkage table (encrypted likewise)
- the linkage authorisation records (PHASE 1 §5)

Broker operations emit AuditEvents into the same chain as the
FHIR store so that auditors see read patterns on identifiers
alongside reads on records.

## §6 Identity rotation

Subject identifiers rotate when:

1. A linkage authorisation expires (the linked identifier is
   discarded; subsequent calls re-mint a fresh per-purpose
   identifier).
2. A controller is dissolved or transferred (the new controller
   re-mints all identifiers under its own URN namespace).
3. A subject exercises a "withdraw and re-enrol" right under
   the controlling jurisdiction.

Rotation issues a Provenance resource (PHASE 1 §5) so that
historic AuditEvents can be reattributed to the new
identifier with explicit audit visibility.

## §7 Retention enforcement

Each resource carries a retention policy derived from its
jurisdiction and resource type. Examples:

| Jurisdiction | Resource          | Retention                       |
|--------------|-------------------|---------------------------------|
| HIPAA        | Designated Record | 6 years (45 CFR §164.530(j))    |
| K-PIPA       | 의료기록           | 10 years for medical records (의료법 §22)    |
| GDPR         | Most clinical     | "no longer than necessary" — set by deployment policy |

The retention engine runs daily, marks records past retention as
`tombstoned` (not deleted), and serves them as `410 Gone` with a
problem URI of `urn:wia:mdp:problem:retention-expired`. Tombstones
are themselves retained for the audit-trail retention period
(typically 6 years longer than the underlying record).

Hard deletion of tombstones is a privileged operation requiring
two-person integrity; a single-principal deletion is rejected.

## §8 Cross-controller disclosure

When a record crosses controller boundaries (e.g., from a hospital
to a research consortium), the sending controller emits a
*disclosure manifest*:

- the consent receipt that authorises the disclosure
- the resource set (FHIR Bundle hash)
- the receiving controller's identifier
- the purpose
- the period during which the receiver may retain the data
- the receiver's signature acknowledging the manifest

The manifest is signed by both controllers and is stored in both
audit chains. A receiver that uses the data outside the manifest's
purpose has, by construction, a falsified audit trail — the breach
is detectable.

## §9 Failure modes

If an AuditEvent cannot be persisted, the underlying operation
MUST fail (the response is `503 Service Unavailable` with a
problem URI of `urn:wia:mdp:problem:audit-unavailable`). The
deployment SHOULD NOT relax this rule; a successful disclosure
without an audit record is the exact pattern that creates an
unprovable breach later.

If the signing key is unavailable, consent issuance and audit
sealing fail. Reads against existing records continue, gated by
the cached consent JWS signatures (verified against the prior
key still in the JWKS per §3).

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

The protocol defaults below are intended for new deployments. Existing
deployments SHOULD migrate at the next planned key rotation; legacy
algorithms remain permitted only for backward-compatibility with
data created before the upgrade.

| Concern              | Default                         | Legacy permitted |
|----------------------|---------------------------------|------------------|
| Token signature      | ES256 (P-256, SHA-256)          | RS256, RS384     |
| Receipt signature    | ES384 (P-384, SHA-384)          | RS256            |
| Daily-root signature | ES384                           | RS256            |
| Audit hash           | SHA-256 (RFC 6234)              | n/a              |
| TLS                  | 1.3 (RFC 8446)                  | 1.2 explicit-list|
| Symmetric at rest    | AES-256-GCM                     | AES-128-GCM      |
| Key wrapping         | RSA-OAEP-SHA-256 or ECDH-ES     | n/a              |

Quantum-resistance is out of scope for this version; a migration plan
will be addressed in a future minor version when NIST post-quantum
signature standards are stable enough for production HSM support.

## Annex B — Failure-mode matrix (informative)

| Failure                          | Behaviour                                               |
|----------------------------------|---------------------------------------------------------|
| AuditEvent persistence fails     | reject the originating request (PHASE 3 §9)             |
| Signing key unavailable          | deny new consents and audit sealing; allow existing reads using cached signatures |
| Identity broker unreachable      | deny linkage operations; allow purpose-internal reads   |
| JWKS endpoint unreachable        | deny new tokens; allow existing tokens until expiry     |
| Transparency log unreachable     | continue local chain; backfill at next reachable window |
| Time skew > 5 min from authoritative source | reject token issuance; alert ops               |

## Annex C — Daily-root sealing (informative)

The following pseudocode illustrates the daily-root sealing
operation. Production implementations use a properly tested HSM
client; the snippet is for clarity only.

```
events = audit_store.fetch_window(day_start, day_end)
prev   = audit_store.previous_root(day_start)

root = prev
for ev in events:                    # ordered by seq
    canon = jcs_canonicalise(ev.body)        # RFC 8785
    root  = sha256(root || canon)

signed = hsm_sign(controller_kid, root)      # ES384 over root
audit_store.write_daily_root(day_end, root, signed)

if policy.transparency_log_enabled:
    log_handle = transparency_log.append(signed)
    audit_store.bind_log_handle(day_end, log_handle)
```

A re-derivation of the day's root from the persisted events MUST
match the stored signed root; mismatch is a chain-integrity incident
and triggers the breach-investigation flow (PHASE 1 §8).

The seal happens once per UTC day; it never re-seals retrospectively.
A late-arriving event with a timestamp inside an already-sealed day
is rejected and routed to the late-arrival queue, which is itself
auditable.

## Annex D — Time discipline

Clocks are synchronised by NTPv4 with stratum-2 or better authoritative
servers. Time drift exceeding 500 ms from the authoritative source
is treated as an operational incident. Token issuance is suspended
when drift exceeds 5 s because token validity windows depend on
agreed time. The stratum-2 source itself is documented in the
deployment policy so that auditors can replicate the time reference.
