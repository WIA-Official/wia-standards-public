# WIA-network-security PHASE 3 — Protocol Specification

**Standard:** WIA-network-security
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format
(PHASE 1) to the API surface (PHASE 2): authentication of
detectors, analysts, partners, and asset owners; the
streaming binary projection for high-volume event ingestion;
audit-chain construction; time discipline; TLP marking
enforcement; cryptographic signing; and post-quantum
migration.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK),
  RFC 9162 (Certificate Transparency 2.0)
- OASIS TAXII 2.1 — for partner-exchange transport
- OASIS Sigma — for correlation-rule signing
- IETF RFC 6545 (RID) — for cross-organisation incident notification
- WIA-pq-crypto PHASE 3 — for ML-KEM/ML-DSA migration profiles

---

## §1 Authentication

Detectors, analysts, asset owners, partners, and regulator
roles authenticate using JWS-signed JWTs issued by the
deployment's identity authority. Token claims:

- `iss` — issuing authority URN
- `sub` — operator/system URN
- `aud` — boundary URN
- `iat`, `exp` — RFC 3339 with offset
- `wia.role` — one of the roles in PHASE 2 Annex F
- `wia.scope[]` — operation-class scopes
- `wia.detectorRef` — for detector tokens, the URN of the
  detector the token speaks for

Analyst tokens are short-lived (typically 15 minutes for
write-capable, 60 minutes for read-only). Detector tokens
may be longer-lived (24 hours) where rotated by the
deployment's secrets pipeline; long-lived static credentials
are forbidden for any human role.

## §2 Streaming binary projection

For very-high-volume event ingestion (millions of events per
second across a fleet), a binary projection is offered. The
projection is a length-prefixed CBOR stream over a mutual-TLS
1.3 connection. Each frame:

- 4-byte length prefix
- CBOR-encoded event record (PHASE 1 §3 with canonical key
  ordering)
- 32-byte per-frame MAC (HMAC-SHA-256 with rotated key)

The boundary acknowledges in batches; consumers retransmit
unacknowledged events on reconnect using the boundary's
last-known-good cursor. The binary projection is opt-in;
deployments without high-volume needs use the JSON streaming
endpoint of PHASE 2 §2.

## §3 Audit chain

Every boundary state transition is appended to a Merkle
audit log:

- `entryId` — URN
- `parent` — prior `entryId` SHA-256
- `at` — RFC 3339 with offset
- `actor` — authority URN making the transition
- `kind` — closed enum: `asset-registered`, `event-ingested`,
  `ioc-published`, `tio-published`, `vuln-published`,
  `vuln-remediated`, `incident-opened`, `incident-state-changed`,
  `incident-closed`, `alert-state-changed`, `action-executed`,
  `action-outcome-recorded`, `tlp-marking-changed`,
  `regulator-witnessed`, `risk-accepted`
- `payloadHash` — SHA-256 of the canonical JSON payload
- `signature` — JWS by the actor

Anchored deployments mirror the audit chain to a regulator-
trusted witness on a declared cadence. Specific entry classes
(`risk-accepted`, `regulator-witnessed`) are mandatory-mirror
under regulator MoUs.

## §4 TLP marking enforcement

TLP markings flow with IOCs, TIOs, and incident-related
artifacts. The boundary enforces:

- Re-distribution refused if target's clearance is below
  marking
- Marking downgrade requires a signed downgrade record
  (PHASE 1 §7 incident `lessonsLearnedRef` typically
  references the downgrade rationale)
- Marking upgrade is permitted unilaterally and propagated
  to subscribers via the webhook surface

Refusals are audit-chained at `kind=tlp-marking-refusal` with
the partner identity that was refused.

## §5 Time discipline

All record timestamps use RFC 3339 with explicit offset.
Boundary clock is disciplined to a national-laboratory time
reference; drift outside declared bound triggers a
`boundary-clock-degraded` capability flag and tags subsequent
records `provisional` until recovered. Detector clocks are
expected to be NTP-disciplined; events with timestamps
outside the boundary's declared `clockSlackBudget` are
refused.

## §6 Transport security

All endpoints require TLS 1.3 (RFC 8446) with a deployment-
declared cipher-suite list. Mutual TLS is required for
detector, partner-TAXII, regulator, and binary-projection
endpoints. Certificate revocation is published through the
deployment's revocation surface; aligned with the
deployment-declared revocation cadence.

## §7 RID cross-organisation notification

For incidents requiring cross-organisation notification (e.g.,
infections traced to a partner's network), the boundary
emits an IETF RFC 6545 RID message:

- Iodef-bound XML payload (or JSON projection where the
  partner supports it)
- Signed by the deployment's incident-notification key
- Delivered over the partner's documented RID endpoint or
  via TAXII as a fallback

Receipt acknowledgements are audit-chained.

## §8 OpenC2 action authentication

Response actions (PHASE 1 §9) carry an OpenC2-signed
authorisation token in addition to the analyst's JWT. The
token identifies the action's policy basis and is verified
by the executing control-plane component before execution.
Unauthorised actions are refused at the control plane and
the refusal is audit-chained.

## §9 Replay protection

Event POST, IOC POST, and action POST require
`Idempotency-Key`; the boundary stores keys for 24 hours.
Replays within that window with the same body return the
original response; different bodies return
`urn:wia:nsec:problem:idempotency-conflict`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cryptographic signature suite

Default signature algorithms are ECDSA P-256 with SHA-256 for
analyst and detector tokens, and EdDSA (Ed25519) for audit-
chain entries. Deployments may declare alternative suites in
the capability document; partners verify suite compatibility
on initial connection.

## Annex B — Post-quantum migration

The standard supports a phased PQC migration aligned with
WIA-pq-crypto PHASE 3:

- Phase A — classical-only (current default)
- Phase B — hybrid (classical + ML-KEM, classical + ML-DSA)
- Phase C — PQ-only

Detectors in unmaintained appliances may remain on Phase A
under documented exception; the exception is recorded in the
capability document and tracked for end-of-support
remediation.

## Annex C — Cipher-suite floors

Endpoints accept only TLS 1.3 cipher suites with forward
secrecy. The cipher-suite floor is published in the
capability document; partners verify compatibility before
exchange. The detector-onboarding playbook explicitly lists
the floor so detector vendors can verify support before
integration.

## Annex D — Negative-test vectors for protocol layer

| Stimulus                                              | Expected outcome                              |
|-------------------------------------------------------|-----------------------------------------------|
| Detector token without `wia.detectorRef`              | 422 + detector-token-malformed                |
| Event with timestamp outside clock-slack budget       | 422 + clock-discipline-violation              |
| TAXII partner request with insufficient TLP clearance | 403 + tlp-distribution-denied                 |
| OpenC2 action without authorisation token             | 403 + action-authorisation-missing            |
| Audit-chain entry with broken parent hash             | rejected at append; boundary alerts           |

## Annex E — Algorithm registry

The deployment maintains an algorithm registry naming the
cipher and signature algorithms in use per role class. The
registry is published in the capability document and tracked
across PQ migration phases for partner verification.

## Annex F — Boundary-clock health

The boundary publishes a clock-health record in the
capability document including the primary and secondary time
sources, the most recent successful sync, and the current
drift estimate. Partners verify clock health before
accepting time-sensitive products.

## Annex G — Worked correlation-rule signing

Sigma-aligned correlation rules are signed for distribution:

1. SOC-author drafts the rule and submits it via the SIEM's
   rule-management interface
2. The deployment's rule-review workflow signs off; the
   signed rule package is stored with its `correlationRuleRef`
3. Rule package signatures are verified at SIEM load time;
   unverified rules are not loaded into production
4. Rule mutations enter a versioned chain; the rule library's
   Merkle root is published in the capability document

For partner-shared rules (e.g., a TLP:GREEN community rule),
the partner's signature is verified against their published
key before promotion.

## Annex H — Action-blast-radius limits

Response actions (PHASE 1 §9) carry implicit blast-radius
limits enforced at protocol level:

- `block-network-flow` — limited to /24 by default for
  Tier-1 analysts; broader scopes require Tier-2 or above
- `quarantine-host` — limited to non-tier-1-criticality assets
  for Tier-1; tier-1 assets require SOC-lead approval
- `disable-account` — non-privileged accounts only for
  Tier-1; privileged accounts require IAM-lead approval
- `kill-process` — single host scope for Tier-1; fleet-wide
  process kill requires Tier-2 or above

The deployment publishes its action-blast-radius matrix in
the capability document.

## Annex I — Anonymisation envelope

Where TLP and PPI controls require anonymisation before
re-disclosure, the boundary applies a documented anonymisation
envelope:

- IPs in PPI:USER class are k-anonymised (k declared per
  policy) for partner sharing
- Usernames are replaced with deterministic per-partner
  pseudonyms so the partner can correlate across messages
  without learning identity
- Free-text fields are scanned for PII and redacted; the
  redactor's policy version is included in the envelope

The anonymisation envelope is itself a versioned artifact;
changes to the envelope require a privacy-officer counter-
signature and emit a `privacy-envelope-mutated` audit-chain
entry.

## Annex J — RID notification template

A worked RID-over-TAXII notification:

```json
{
  "ridId": "urn:wia:nsec:rid:soc-x:rid-2026-04-27-014",
  "incidentRef": "urn:wia:nsec:incident:soc-x:i-2026-04-27-001",
  "partnerRef": "urn:wia:auth:partner-y",
  "kind": "infection-sourced-from-partner",
  "evidenceRefs": ["urn:wia:nsec:event:soc-x:e-991","urn:wia:nsec:ioc:soc-x:i-2-001"],
  "tlpMarking": "amber",
  "originatorSignature": "<jws-detached>"
}
```

Partner receipt acknowledgement:

```json
{
  "ackId": "urn:wia:nsec:rid-ack:partner-y:a-2026-04-27-014",
  "ridRef": "urn:wia:nsec:rid:soc-x:rid-2026-04-27-014",
  "receivedAt": "2026-04-27T22:45:00+09:00",
  "partnerSignature": "<jws-detached>"
}
```

## Annex K — JWT claim cookbook

Worked claim sets per role:

```json
{
  "iss": "urn:wia:auth:soc-x-identity",
  "sub": "urn:wia:auth:soc-x-analyst-3",
  "aud": "urn:wia:nsec:boundary:soc-x",
  "iat": "2026-04-27T22:30:00+09:00",
  "exp": "2026-04-27T22:45:00+09:00",
  "wia.role": "analyst",
  "wia.scope": ["events:read","alerts:write","incidents:write"],
  "wia.tier": 1
}
```

Detector token claims include `wia.detectorRef` and a
narrower scope (`events:write` only). Partner-TAXII tokens
include `wia.tlpClearance` declaring the highest TLP marking
the partner is cleared to receive on the collection.

## Annex L — Detector-onboarding handshake

Detector onboarding follows a documented handshake:

1. Detector vendor presents a signed detector-bundle manifest
   declaring `detectorRef`, supported event schemas, and
   confidence calibration
2. Boundary verifies the manifest's signature against the
   vendor's published key
3. Initial test events are submitted in a sandbox lane
4. SOC lead reviews and approves promotion to production
5. Production token is issued; the detector is recorded in
   the capability document

A detector promoted to production is monitored for false-
positive rate and confidence calibration; persistent drift
triggers a re-onboarding review.
