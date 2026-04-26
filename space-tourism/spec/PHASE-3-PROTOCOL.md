# WIA-SPACE-014 — Phase 3: Protocol Specification

**Standard**: WIA-SPACE-014 (Space Tourism)
**Phase**: 3 of 4 — Protocol
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 3 specifies how independent operators, regulators, insurance
underwriters and search-and-rescue authorities exchange records over time:
trust establishment, mission lifecycle state machines, replay defence,
delegation of authority and embargo handling for safety events.

---

## 2. Roles

| Role | Description |
|------|-------------|
| **Operator** | Holds passenger records, manifests, missions, safety events |
| **Underwriter** | Holds insurance records, settles claims |
| **Regulator** | Holds the licence to fly, audits safety events |
| **SAR Authority** | Search-and-rescue endpoint; receives flight plan and contingency notices |
| **Passenger Agent** | Acts on behalf of a passenger (book, cancel, claim) |

A single legal entity MAY play multiple roles. Each role is identified by
a DID; trust is recorded in the operator's signed peer list.

---

## 3. Federation Handshake

```
   ┌─────────────┐
   │   IDLE      │
   └─────┬───────┘
         │ peer presents credential + ephemeral key
         ▼
   ┌─────────────┐
   │  PENDING    │ origin checks credential against role registry
   └─────┬───────┘
         │ valid
         ▼
   ┌─────────────┐
   │  ACCEPTED   │ origin issues federation receipt
   └─────┬───────┘
         │ optional revocation
         ▼
   ┌─────────────┐
   │  REVOKED    │
   └─────────────┘
```

Receipts (Phase 1 of WIA-SOCIAL §5 form is reused) are persisted by both
sides for at least the regulatory retention period.

---

## 4. Mission Lifecycle

```
DRAFT → REVIEWED → FROZEN → LAUNCHED → RECOVERED → ARCHIVED
                              │
                              └─► ABORTED (with safety_event)
```

| State | Allowed transitions | Notes |
|-------|---------------------|-------|
| DRAFT     | REVIEWED, ARCHIVED | Mission profile editable |
| REVIEWED  | FROZEN, DRAFT      | Operator safety officer signs off |
| FROZEN    | LAUNCHED, ABORTED  | Profile immutable; manifest changes locked |
| LAUNCHED  | RECOVERED, ABORTED | Telemetry stream open |
| RECOVERED | ARCHIVED           | Crew and passengers off vehicle |
| ABORTED   | ARCHIVED           | Safety event mandatory |
| ARCHIVED  | (terminal)         | Read-only after retention period |

State changes are signed by the operator and pushed to subscribed peers
(regulator, SAR, underwriter).

---

## 5. Replay Defence

Every signed envelope (state change, safety event, claim) carries a 96-bit
nonce and an RFC 3339 timestamp. Receivers MUST:

1. Reject envelopes with skew > ±300 s.
2. Reject envelopes whose `(signer, nonce)` pair has been seen within the
   last 600 s.
3. Maintain the seen-nonce cache for at least 600 s.

For safety events, the cache MUST be persistent across restarts to avoid
duplicate notification storms after an operator failover.

---

## 6. Delegation

A passenger MAY delegate booking and claim authority to an agent by
publishing a `delegation` envelope:

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "delegation",
  "delegator_id": "did:wia:passenger:01HXY…",
  "delegate_id":  "did:wia:agent:09…",
  "scopes": ["book", "amend_manifest", "submit_claim"],
  "valid_from": "2026-01-01T00:00:00Z",
  "valid_until": "2026-12-31T23:59:59Z",
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Operators MUST refuse delegated actions whose scopes do not cover the
attempted operation.

---

## 7. Safety Event Distribution

* Severity `observation` and `anomaly` MAY be embargoed at operator
  discretion for up to 90 days while internal review completes.
* Severity `incident` and `accident` MUST be pushed to the regulator
  within 24 hours and to the public within 30 days.
* SAR authorities receive `contingency` notices in real time during
  launch, coast, descent and recovery phases.

A safety event MUST NOT be deleted. Corrections are appended as a new
event referencing the original via `corrects: <event_id>`.

---

## 8. Insurance Claim Flow

```
Passenger → Operator: submit claim packet (medical + safety event refs)
Operator  → Underwriter: forward signed packet, attach federation receipt
Underwriter → Operator: claim_status (open / under_review / settled / denied)
Operator  → Passenger: notify, escalate to regulator if disputed > 60 days
```

All transitions are envelope-signed. Disputes age into a regulator queue
automatically when the underwriter has not advanced the claim within
60 days of submission.

---

## 9. Audience Controls

| Audience | Visibility |
|----------|------------|
| `public` | Mission summary, anonymised safety events, redacted manifest |
| `crew`   | Full manifest, full passenger medical |
| `regulator` | All non-personally-identifying records, plus identity if subpoenaed |
| `underwriter` | Insurance record + linked safety events for the policy |
| `passenger` | Own record only |
| `sar` | Live flight plan, contingency notices, recovery zone forecasts |

Operators enforce audience by issuing per-peer credentials at handshake
time and by gating record reads on those credentials.

---

## 10. Cryptographic Suite

| Use | Algorithm | Reference |
|-----|-----------|-----------|
| Identity signing | Ed25519 | IETF RFC 8032 |
| HTTP message signing | Ed25519 over RFC 9421 | RFC 9421 |
| Hashing | SHA-256 | FIPS 180-4 |
| Transport | TLS 1.3 | IETF RFC 8446 |

PQC migration tracks the parent WIA family roadmap.

---

## 11. Conformance

A Phase 3 conformant operator MUST:

1. Implement the federation handshake and mission lifecycle state machines.
2. Persist receipts and seen-nonce caches as specified.
3. Honour delegation envelopes including scope restrictions.
4. Push safety events according to severity timing rules.
5. Enforce audience-based read controls.

---

## 12. References

* IETF RFC 8032 — EdDSA
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* FIPS 180-4 — SHA family
* W3C DID 1.0
* ICAO Annex 13 — Safety event severity ladder

---

## Appendix A — Worked Mission Lifecycle Trace

```
α = Operator (lev.example)
β = Regulator (caa.example)
γ = SAR adapter (sar.example)
δ = Underwriter (ins.example)

T-30d
  α: POST /wst/mission { state:"DRAFT" }
T-21d
  α → β: handshake; β joins regulator role
T-14d
  α: PATCH /wst/mission/{id} state=REVIEWED, signed by safety officer
T-2h
  α: PATCH /wst/mission/{id} state=FROZEN
  α → γ: send pre-launch flight plan, predicted footprint
  α → δ: confirm coverage active for every passenger
T+0
  α: state=LAUNCHED, telemetry SSE opens
  γ: subscribes telemetry, monitors heartbeat
T+33s
  α: POST /wst/safety severity=anomaly
  α → β: forward signed safety_event
T+12m
  α: state=RECOVERED, mission summary published
T+30d
  α: state=ARCHIVED on retention boundary
```

A failed handshake at T-21d yields a problem document of type
`…/handshake-bad-signature` and prevents the mission from leaving DRAFT.

## Appendix B — Replay Cache Sizing

For an operator handling 200 envelopes per second across all roles, the
seen-nonce cache must hold roughly `200 × 600 = 120 000` entries to enforce
§5's 600-second window. With 16-byte nonce keys plus a 4-byte timestamp,
the cache footprint is approximately `120 000 × 24 ≈ 3 MiB`. Operators
SHOULD provision at least double this to absorb mission-window bursts and
SHOULD make the cache persistent across restarts so a failover does not
re-open the window for a duplicated safety event.

## Appendix C — Audience Decision Matrix

| Caller | passenger_record | manifest | safety_event | insurance | telemetry |
|--------|------------------|----------|--------------|-----------|-----------|
| Public                    | ✗ | redacted   | severity ≥ incident | ✗ | 1 Hz throttled |
| Crew                      | ✓ | full       | full              | ✗ | full           |
| Regulator                 | redacted | full | full              | metadata only | full |
| Underwriter (own policy)  | medical only | seat-only | linked events | full | ✗ |
| Passenger (own record)    | own | own seat | own events       | own       | own seat |
| SAR                       | ✗ | flight plan only | contingency only | ✗ | full     |

Operators MUST refuse cross-class enrichment that would defeat the matrix
(for example, joining a `passenger_record` with a `safety_event` on
behalf of an unauthenticated public caller).

## Appendix D — Operator Responsibilities Around Failover

When an operator fails over from primary to standby region, the standby
MUST:

1. Reload the persistent seen-nonce cache before resuming envelope
   processing.
2. Re-issue handshakes to peers whose receipts are not present in the
   standby's storage.
3. Replay missed safety events from the primary's append-only log before
   accepting new events.
4. Notify peers via a `notice` envelope that primary→standby switchover
   has occurred, with an estimated `restore_at` for the primary.

## Appendix E — Privacy Threats and Mitigations

| Threat | Mitigation |
|--------|------------|
| Bulk export of passenger records by compromised peer | Per-claim audience controls per §9; peer downgrade after first detected leak |
| Side-channel timing on signature verify | Constant-time Ed25519 implementations REQUIRED |
| Long-term metadata accumulation by underwriter | Underwriter receives only metadata + linked safety events; full passenger record stays at operator |
| Telemetry stream re-identification of crew | Public stream throttled to 1 Hz and stripped of biometric channels |
| SAR adapter abuse for surveillance | SAR endpoints accept only flight-plan and contingency envelopes; any non-flight queries return `403` |

## Appendix F — Inter-Operator Manifest Exchange

When a passenger reschedules from operator A to operator B mid-cycle, A
generates a `manifest_transfer` envelope:

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "manifest_transfer",
  "from_operator": "did:wia:operator:lev",
  "to_operator":   "did:wia:operator:orb",
  "passenger_id":  "did:wia:passenger:01HYG…",
  "original_manifest_id": "vmfst_01HYG…",
  "delegation_id": "del_01HYG…",
  "reason": "weather-scrub",
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Operator B verifies:

1. The delegation envelope grants `transfer_manifest` scope.
2. Operator A's signature against A's host key.
3. Passenger fitness class still matches operator B's vehicle profile.

If all checks pass, B issues a fresh manifest entry, sends a confirmation
envelope back to A, and A marks its original seat `transferred`. The
seat's history is appended, never overwritten — a passenger or auditor
querying the seat's lineage MUST see all transfers.

## Appendix G — Trust List Maintenance

Each operator maintains a signed trust list:

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "trust_list",
  "operator_id": "did:wia:operator:lev",
  "issued_at": "2026-04-01T00:00:00Z",
  "valid_until": "2026-05-01T00:00:00Z",
  "entries": [
    { "peer_id": "did:wia:regulator:caa-us", "role": "regulator", "score": 1.0 },
    { "peer_id": "did:wia:insurer:09…",      "role": "underwriter", "score": 0.97 },
    { "peer_id": "did:wia:sar:rescue-01",    "role": "sar",        "score": 1.0 }
  ],
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Trust lists are republished at least monthly. Peers fetch the latest list
during handshake and refuse stale lists older than 60 days. A peer may
self-publish a `revocation` envelope to immediately drop trust between
list refresh windows.

弘益人間 — Benefit All Humanity.
