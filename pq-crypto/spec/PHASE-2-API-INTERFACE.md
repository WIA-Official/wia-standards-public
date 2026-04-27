# WIA-pq-crypto PHASE 2 — API Interface Specification

**Standard:** WIA-pq-crypto
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a deployment exposes for
algorithm-registry management, key-pair lifecycle (creation,
rotation, deprecation), hybrid-mode binding management,
migration-phase declaration, conformance-evidence
publication, and HSM-attestation exchange. The shape is
HTTP/JSON; HSM control-plane interactions use the binary
protocol in PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS),
  RFC 7517 (JWK), RFC 7518 (JWA)
- NIST FIPS 203/204/205 — for algorithm reference
- WIA-network-security PHASE 2 — for cipher-suite floor coordination
- WIA-supply-chain PHASE 2 — for code-signing key sharing

---

## §1 Algorithm registry endpoints

```
POST /algorithms HTTP/1.1
Authorization: Bearer <jws-crypto-officer-jwt>
Content-Type: application/json
```

Body is a PHASE 1 §2 algorithm record. Successful intake
returns 201 Created with the boundary's URN binding. Updates
that change `acceptedFor` or `deprecationDate` use PUT.

```
GET /algorithms/{algorithmRef}
GET /algorithms?class=kem&nistCategory=3&acceptedFor=tls-server
```

The default sort orders by `algorithmName`; partners
verifying compatibility query by `acceptedFor` and the
target migration phase.

## §2 Key-pair lifecycle

```
POST /keypairs HTTP/1.1
```

Body is a PHASE 1 §3 keypair record. Pre-checks at intake:

- `algorithmRef` is in the algorithm registry and accepted
  for the declared `purpose`
- `keyManagementRef` is recognised
- `holderRef` is permitted to hold keys with the declared
  algorithm

Refusal responses use RFC 9457 problem details:

| problem URN                                  | meaning                              |
|----------------------------------------------|--------------------------------------|
| `urn:wia:pqc:problem:algorithm-deprecated`   | algorithm past acceptance cutoff     |
| `urn:wia:pqc:problem:algorithm-not-accepted` | algorithm not accepted for purpose   |
| `urn:wia:pqc:problem:keymanagement-unknown`  | KM ref not recognised                |
| `urn:wia:pqc:problem:phase-mismatch`         | declared phase incompatible with deployment phase |
| `urn:wia:pqc:problem:successor-already-set`  | predecessor already has a successor  |

```
GET /keypairs/{keypairRef}
GET /keypairs?holderRef=…&algorithmRef=…&purpose=tls-server
PUT /keypairs/{keypairRef}/validity
```

The PUT updates `validNotBefore` / `validNotAfter`; the
boundary refuses non-monotonic updates that would invalidate
already-published artifacts.

## §3 Key rotation

```
POST /rotations HTTP/1.1
```

Body is a PHASE 1 §4 rotation record. The boundary verifies:

- `predecessorRef` is currently active
- `successorRef` exists, has matching purpose, and is in the
  registry
- `overlapWindowStart` / `overlapWindowEnd` are consistent
- The rotation reason is acceptable for the declared phase

Successful rotation:

- Updates `successorRef` on the predecessor
- Schedules `validNotAfter` on the predecessor at the end of
  the overlap window
- Records HSM attestations in the audit chain
- Notifies subscribers via webhook

## §4 Hybrid-mode binding management

```
POST /hybrid-bindings HTTP/1.1
GET /hybrid-bindings/{bindingId}
GET /hybrid-bindings?protocolRef=…&phase=hybrid
```

POST submits a PHASE 1 §5 hybrid-binding record. The
boundary verifies both `classicalAlgorithmRef` and
`pqAlgorithmRef` are in the registry and acceptable for the
declared `protocolRef`. The boundary refuses:

- Hybrid bindings whose classical algorithm is past
  deprecation
- Hybrid bindings whose PQ algorithm is below the
  deployment's declared minimum NIST category for the
  protocol class

## §5 Migration-phase declaration

```
POST /phase-declarations HTTP/1.1
```

Body is a PHASE 1 §6 phase-declaration record. The boundary
verifies the proposed phase is reachable from the current
phase per the deployment's roadmap; rollback is permitted
but requires elevated authorisation.

```
GET /phase-declarations/current
GET /phase-declarations?since=…
```

Phase advances trigger a fan-out to all keys holding
phase-bound state; partners are notified via webhook so
they can re-verify cipher-suite floors before subsequent
exchanges.

## §6 Algorithm deprecation management

```
POST /deprecations HTTP/1.1
```

Body is a PHASE 1 §7 deprecation record. The boundary
verifies:

- `algorithmRef` is currently in the registry
- `acceptanceCutoffAt` is at least the deployment-declared
  notice ahead of submission (or marked exception with
  cryptographic-officer counter-signature)
- Replacement algorithms are present in the registry

```
GET /deprecations/{deprecationId}
GET /deprecations?effective=true&for=tls-server
```

Effective deprecations (where `acceptanceCutoffAt` has
passed) are surfaced to all consumers; the boundary refuses
new POST /keypairs against deprecated algorithms.

## §7 Conformance-evidence publication

```
POST /evidence HTTP/1.1
```

Body is a PHASE 1 §8 conformance-evidence record. The
boundary verifies:

- `period` is a recognised period
- `phaseDeclarationRef` is the active phase declaration
- All referenced rotation/deprecation/attestation records
  exist
- Auditor signature is verifiable against the auditor's
  registered key

```
GET /evidence/{evidenceId}
GET /evidence?period=2026-Q2
```

Evidence records form the canonical conformance signal for
partners and regulators.

## §8 HSM attestation exchange

```
POST /attestations HTTP/1.1
GET /attestations/{attestationId}
GET /attestations?kmRef=…&since=…
```

POST submits an HSM attestation record. The boundary
verifies the attestation signature against the HSM's
registered key and stores the attestation alongside the
event it attests (e.g., key creation, rotation, destruction).

## §9 Capability discovery

```
GET /.well-known/wia/pq-crypto HTTP/1.1
```

Returns:

```json
{
  "wia.standardVersion": "1.0",
  "wia.implementationVersion": "authority-x-3.0.0",
  "currentPhase": "hybrid",
  "phaseEntryAt": "2026-04-01T00:00:00+09:00",
  "algorithmRosterRef": "urn:wia:pqc:roster:authority-x:r-2026-q2",
  "minNistCategory": 3,
  "acceptedHybridBindings": ["tls-1.3-hybrid-x25519-mlkem768"],
  "manifest": "https://authority-x.example/.well-known/wia/pq-crypto/manifest.jws"
}
```

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Idempotency

Keypair POST, rotation POST, and evidence POST accept
`Idempotency-Key`. Boundary stores keys for 30 days.
Replays return the original response.

## Annex B — Pagination

List endpoints support cursor pagination with cursors signed
by the boundary, valid for 30 minutes. Page envelopes
declare the snapshot epoch.

## Annex C — Negative-test vectors (informative)

| Stimulus                                            | Expected response                              |
|-----------------------------------------------------|------------------------------------------------|
| Keypair POST with deprecated algorithm              | 422 + algorithm-deprecated                     |
| Keypair POST without keyManagementRef               | 422 + keymanagement-required                   |
| Rotation POST with non-monotonic validity           | 422 + validity-non-monotonic                   |
| Hybrid binding with PQ below min NIST category      | 422 + nist-category-below-floor                |
| Phase rollback without elevated authorisation       | 403 + rollback-elevation-required              |
| Evidence POST without auditor signature             | 422 + auditor-signature-missing                |

## Annex D — Bulk export

```
GET /export/keypairs?from=…&to=…
GET /export/rotations?from=…&to=…
```

NDJSON streamed export, gated by the deployment's bulk-quota
policy. Audit-logged with `kind=bulk-export`.

## Annex E — Webhook subscriptions

Push subscriptions for cryptographic-aware consumers
(application identity providers, partner deployments):

```
POST /subscriptions HTTP/1.1

{
  "subscriptionId": "urn:wia:pqc:sub:partner-y:s-001",
  "callbackUrl": "https://partner-y.example/webhooks/pqc",
  "eventClasses": ["rotation-completed","phase-advanced","algorithm-deprecated"],
  "subscriberAuthorityRef": "urn:wia:auth:partner-y"
}
```

Webhook delivery uses TLS 1.3 with detached JWS in
`Wia-Signature`.

## Annex F — Authorities and roles

| Role                  | Scope                                              |
|-----------------------|----------------------------------------------------|
| `crypto-officer`      | algorithm registry, deprecation, phase advances    |
| `key-holder`          | keypair lifecycle for owned holderRef              |
| `auditor`             | evidence sign-off, read-only across in-scope       |
| `partner`             | capability-document and evidence read              |
| `hsm-control-plane`   | attestation submission for managed KM              |

## Annex G — Worked rotation request

```json
{
  "rotationId": "urn:wia:pqc:rotation:authority-x:r-2026-04-27-001",
  "predecessorRef": "urn:wia:pqc:keypair:authority-x:k-tls-2025",
  "successorRef": "urn:wia:pqc:keypair:authority-x:k-tls-2026-q2",
  "rotatedAt": "2026-04-27T22:00:00+09:00",
  "rotationReason": "scheduled",
  "overlapWindowStart": "2026-04-27T22:00:00+09:00",
  "overlapWindowEnd": "2026-05-27T22:00:00+09:00",
  "attestationRefs": ["urn:wia:pqc:att:authority-x:hsm-1:a-2026-04-27-001"],
  "signatures": [/* crypto-officer + key-holder JWS detached */]
}
```

## Annex H — Capability versioning

Capability documents declare both `wia.standardVersion` and
`wia.implementationVersion`. A standard-version mismatch is
a hard refusal; an implementation-version mismatch is logged
but not refusing.

## Annex I — Audit-chain replay

For partners and Anchored auditors, the boundary serves
selective audit-chain replay at `/audit/chain` filtered by
kind and time range. Restricted kinds (e.g.,
`compromise-rotation`) require elevated read authorisation.

## Annex J — Concurrency and stale-write protection

Write endpoints accept `If-Match` with the resource's
current ETag. A stale ETag returns
`urn:wia:pqc:problem:stale-resource` (412 Precondition
Failed). For batch resources, the boundary serializes
mutations per `holderRef` so concurrent rotations on the
same holder do not interleave.

## Annex K — Read-your-writes consistency

After a successful POST/PUT, subsequent GETs from the same
authenticated principal return the new state immediately.
Cross-principal visibility may have a short propagation
delay bounded by the deployment-declared cache TTL; the
capability document publishes the bound so partners can
size their retry windows.

## Annex L — Audit-chain replay subscription

Anchored auditors may subscribe to selective audit-chain
replay via Server-Sent Events:

```
GET /audit/chain/$subscribe?kinds=phase-advanced,compromise-rotation-flagged HTTP/1.1
Authorization: Bearer ...
Accept: text/event-stream
```

Each event carries the entry's hash, parent hash, and
signed payload reference. Subscribers reconstruct the
chain locally and detect missing entries against the
boundary's published chain-root summary.

## Annex M — Authority delegation chains

Cryptographic authorities may delegate sub-scopes to
nested authorities (e.g., a national authority delegates
TLS-server keypair management to an agency authority).
Delegation records carry:

- `delegationId` — URN
- `parentAuthorityRef` — issuing authority
- `childAuthorityRef` — receiving authority
- `delegatedScopes[]` — scope strings inherited
- `notBefore` / `notAfter` — delegation validity
- `revocationConditions` — circumstances forcing
  immediate revocation

The boundary verifies a token's effective scope by walking
the delegation chain to the root authority; revoked
intermediate delegations cut off all descendants.

## Annex N — Capability-document mirror

Partners may host a read-only mirror of the boundary's
capability document for offline verification. The boundary
publishes a daily signed mirror snapshot at
`/.well-known/wia/pq-crypto/mirror/<YYYY-MM-DD>.jws`.
Mirrors verify the daily signature against the boundary's
long-lived signing key and publish their freshness.
