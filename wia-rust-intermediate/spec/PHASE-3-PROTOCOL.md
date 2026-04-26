# WIA Rust Intermediate — Phase 3: Federation Protocol

**Standard**: WIA Rust Intermediate
**Phase**: 3 of 4 — Federation Protocol
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 3 specifies how academies, employers, and learners build trust over
time: handshake state, learner-record portability, replay defence,
delegation, and revocation.

---

## 2. Roles

| Role | Description |
|------|-------------|
| **Academy** | Issues curriculum manifests, grades assessments, holds learner records |
| **Learner** | Subject of records; owns the signing key for their DID |
| **Employer** | Verifies learner credentials |
| **Grader** | Authorised by an academy to grade assessments |
| **Agent** | Delegated by a learner to act on their behalf |

A single legal entity MAY play multiple roles.

---

## 3. Federation Handshake

```
   ┌─────────┐
   │ IDLE    │
   └────┬────┘
        │ peer presents credential + ephemeral key
        ▼
   ┌─────────┐
   │ PENDING │  origin verifies signature
   └────┬────┘
        │ valid
        ▼
   ┌─────────┐
   │ ACCEPTED│  origin issues federation receipt
   └────┬────┘
        │
        ▼
   ┌─────────┐
   │ REVOKED │
   └─────────┘
```

PENDING MUST resolve within 30 seconds. Receipts persist for at least 7
years per academic record retention norms.

---

## 4. Learner Record Portability

A learner moving from academy A to academy B presents:

1. The current learner_record envelope from A, signed by A's host key.
2. The learner's own delegation envelope authorising B to write.
3. A `move` envelope:

   ```json
   {
     "wia_rust_intermediate_version": "1.0.0",
     "type": "move",
     "learner_id": "did:wia:learner:01HZA…",
     "from_academy": "did:wia:academy:bonghwa",
     "to_academy":   "did:wia:academy:silla",
     "signed_at": "2026-04-20T10:00:00Z",
     "signature": { "alg": "Ed25519", "value": "…" }
   }
   ```

Academy B verifies:

1. The signature on the original record (by A's host key).
2. The signature on the delegation (by the learner's key).
3. The signature on the move envelope (by the learner's key).

Once accepted, A returns `301 Moved Permanently` for the learner record
URL for at least 12 months. Modules passed at A are preserved in B's
record under `modules_passed[].academy_id`.

---

## 5. Replay Defence

Each signed envelope (record write, assessment submission, move) carries
a 96-bit nonce and an RFC 3339 timestamp. Receivers MUST:

1. Reject envelopes with skew > ±300 s.
2. Reject envelopes whose `(signer, nonce)` tuple has been seen within
   the last 600 s.
3. Maintain the seen-nonce cache for at least 600 s.

For learner records the cache MUST persist across restarts so failover
does not re-open the window for a duplicated grade.

---

## 6. Delegation

A learner delegates rights to an agent (career coach, accommodations
service, parent for minors) by publishing:

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "delegation",
  "delegator_id": "did:wia:learner:01HZA…",
  "delegate_id":  "did:wia:agent:09…",
  "scopes": ["read_record", "write_record", "submit_assessment"],
  "valid_from": "2026-01-01T00:00:00Z",
  "valid_until": "2026-12-31T23:59:59Z",
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Academies MUST refuse delegated actions whose scopes do not cover the
attempted operation. Delegation envelopes are revocable by the delegator
publishing a new envelope with `valid_until` set to the current time.

---

## 7. Grader Authority

A grader's authority is a delegation envelope from the academy:

```json
{
  "type": "delegation",
  "delegator_id": "did:wia:academy:bonghwa",
  "delegate_id":  "did:wia:grader:09…",
  "scopes": ["grade", "rubric_apply"],
  "valid_until": "2027-04-20T00:00:00Z",
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Academies MUST publish the active grader list on their discovery
document and MUST publish revocations within 60 seconds of decision.

---

## 8. Audience Controls

| Audience | Visibility |
|----------|------------|
| `public`     | level_attained, modules_passed counts (no artefacts) |
| `learner`    | full own record |
| `academy`    | full record for learners enrolled at this academy |
| `employer`   | level_attained + modules_passed list (band only) |
| `regulator`  | full record under formal compliance request |

Academies enforce audience by gating record reads on credentials issued
during handshake.

---

## 9. Cryptographic Suite

| Use | Algorithm | Reference |
|-----|-----------|-----------|
| Identity signing | Ed25519 | IETF RFC 8032 |
| HTTP message signing | Ed25519 over RFC 9421 | RFC 9421 |
| Hashing | SHA-256 | FIPS 180-4 |
| Transport | TLS 1.3 | IETF RFC 8446 |

---

## 10. Revocation

A learner record MAY be revoked by the issuing academy if academic
misconduct is established. Revocation is signed by the academy and
appended to the record's history; the record remains visible but every
read returns the most recent envelope, including the revocation event.

Revocation is reversible: appeals that succeed publish a `restoration`
envelope. The record's chain MUST always read from oldest to newest with
no rewrites.

---

## 11. Conformance

A Phase 3 conformant implementation MUST:

1. Implement the handshake state machine.
2. Honour replay defence bounds.
3. Implement learner-record move with chain preservation.
4. Honour delegation envelopes including scope restrictions.
5. Enforce per-audience read controls.

---

## 12. References

* IETF RFC 8032 — EdDSA
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* FIPS 180-4 — SHA family

---

## Appendix A — Worked Move Trace

```
α = bonghwa academy
β = silla academy
λ = learner did:wia:learner:01HZA…

1. λ → β: POST /wri/handshake (learner credential)
2. β verifies, ACCEPTED, returns receipt
3. λ → β: POST /wri/learner/01HZA… { type:"move", from:α, to:β, signature_λ }
4. β fetches record from α, verifies α's signature
5. β stores record, marks origin chain
6. β → α: notification of move (for α's audit log)
7. α responds 200, sets future reads to 301 → β's URL
8. λ continues at β; new module passes append to existing chain
```

If step 4 fails verification (e.g. α's host key has rotated), β returns a
problem document of type `…/move-source-invalid` and the move is
rolled back.

## Appendix B — Replay Cache Sizing

For an academy receiving 50 envelopes per second across all roles, the
seen-nonce cache must hold roughly `50 × 600 = 30 000` entries to enforce
§5's 600-second window. With 16-byte nonce keys plus a 4-byte timestamp,
the cache footprint is approximately `30 000 × 24 ≈ 720 KiB`. Academies
SHOULD provision at least double this to absorb exam-window bursts and
SHOULD make the cache persistent across restarts.

## Appendix C — Audience Decision Matrix

| Caller | learner_record | manifest | exercise | assessment_result |
|--------|----------------|----------|----------|-------------------|
| Public                        | level + counts | full | full | ✗ |
| Learner (own record)          | full           | full | full | full |
| Academy (host)                | full for own learners | full | full | full |
| Grader (delegated)            | metadata only  | full | full | full for graded items |
| Employer                      | level + bands  | metadata only | metadata only | ✗ |
| Regulator (compliance)        | full under formal request | full | full | full |
| Anonymous reader              | level only     | full | metadata only | ✗ |

Academies MUST refuse cross-class enrichment that defeats the matrix —
for example, joining a learner_record with assessment artefacts on
behalf of an unauthenticated public caller.

## Appendix D — Privacy Threats and Mitigations

| Threat | Mitigation |
|--------|------------|
| Learner record harvested by hostile peer | Per-audience read controls; peer revocation on first detected leak |
| Side-channel timing on signature verify  | Constant-time Ed25519 implementations REQUIRED |
| Long-term metadata accumulation by employer | Employer audience returns level + band only |
| Grader collusion on a learner's submissions | Academy MUST rotate graders or use double-blind for high-stakes assessments |
| Identity correlation across academies | Learners SHOULD use a fresh DID per tier if cross-tier linkage is undesired |

## Appendix E — Trust List Maintenance

Each academy maintains a signed trust list:

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "trust_list",
  "academy_id": "did:wia:academy:bonghwa",
  "issued_at": "2026-04-01T00:00:00Z",
  "valid_until": "2026-05-01T00:00:00Z",
  "entries": [
    { "peer_id": "did:wia:academy:silla",   "role": "academy",    "score": 0.97 },
    { "peer_id": "did:wia:employer:nogada", "role": "employer",   "score": 0.90 },
    { "peer_id": "did:wia:registry:ec",     "role": "registry",   "score": 1.00 }
  ],
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Trust lists are republished at least monthly; peers refuse stale lists
older than 60 days. A peer may self-publish a `revocation` envelope to
immediately drop trust between list refresh windows.

## Appendix F — Operator Responsibilities Around Failover

When an academy fails over from primary to standby region, the standby
MUST:

1. Reload the persistent seen-nonce cache before resuming envelope
   processing — failure to do so re-opens a 600-second window for an
   attacker to replay a previously-seen grade-write envelope.
2. Re-issue handshakes to peers whose receipts are not present in the
   standby's storage. Peers that do not see a handshake within their
   trust-list refresh interval will mark the academy stale and refuse
   downstream operations.
3. Replay any missed assessment_result envelopes from the primary's
   append-only log before accepting new submissions, so grader UIs do
   not silently reorder.
4. Notify peers via a `notice` envelope that primary→standby switchover
   has occurred, with an estimated `restore_at` for the primary. Peers
   show this to learners and graders to set expectations.

## Appendix G — Cohort Grouping for Synchronous Courses

Some academies deliver Intermediate courses to a synchronous cohort
where a group of learners progresses together. The optional cohort
envelope groups them:

```json
{
  "wia_rust_intermediate_version": "1.0.0",
  "type": "cohort",
  "cohort_id": "co_2026Q2-bonghwa-A",
  "academy_id": "did:wia:academy:bonghwa",
  "manifest_id": "cm_01HZB…",
  "starts_at": "2026-04-01T00:00:00Z",
  "ends_at":   "2026-09-30T00:00:00Z",
  "learner_ids": [ "did:wia:learner:01HZA…", "did:wia:learner:02HZA…" ],
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Cohort envelopes are advisory: they do not change record semantics, but
they let academies surface "weekly progress" reports to graders without
needing a separate group concept. Implementations MAY ignore cohorts
entirely without losing conformance.

## Appendix H — Operational Notes for Graders

* Graders SHOULD verify that the assessment_result envelopes they sign
  reference the correct exercise_id and learner_id; submission portals
  pre-fill these fields but mismatches happen during academy migrations
  or LMS bridge re-mappings.
* Graders SHOULD double-check the rubric_id matches the exercise's
  declared rubric_id before signing — a mismatch indicates a stale
  rubric cache and SHOULD trigger a reload before grading.
* Academies SHOULD provide a one-click "rotate grader key" in the
  grader UI; lost grader keys are rotated by the academy publishing a
  fresh delegation envelope from §7 and revoking the prior one.

弘益人間 — Benefit All Humanity.
