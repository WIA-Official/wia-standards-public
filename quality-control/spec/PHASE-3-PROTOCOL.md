# WIA-IND-025 Quality Control — Phase 3: Protocol

**Standard**: WIA-IND-025 (Quality Control)
**Phase**: 3 of 4 — Protocol
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 3 specifies how independent quality-system hosts (sites, suppliers,
customers, certification bodies) build trust over time, exchange
inspection and audit data, define roles and delegation, defend against
replay, and revoke stale records.

---

## 2. Roles

| Role | Description |
|------|-------------|
| **Site** | Holds the canonical inspection plans, results, SPC samples, NCR/CAPA |
| **Supplier** | Sends incoming material certifications |
| **Customer** | Receives outgoing certificates of conformance |
| **Certification body** | Performs external audits, holds findings |
| **Inspector** | Authorised to sign inspection results |
| **Quality manager** | Authorised to disposition NCR and close CAPA |
| **Auditor** | Authorised to file audit findings |

A single legal entity MAY play multiple roles. Each role is identified
by a DID; role grants are recorded in the site's signed peer list.

---

## 3. Federation Handshake

```
   ┌──────────┐
   │  IDLE    │
   └────┬─────┘
        │ peer presents credential + ephemeral key
        ▼
   ┌──────────┐
   │  PENDING │ origin verifies credential against role registry
   └────┬─────┘
        │ valid
        ▼
   ┌──────────┐
   │ ACCEPTED │ origin issues federation receipt (signed)
   └────┬─────┘
        │ optional revocation
        ▼
   ┌──────────┐
   │ REVOKED  │
   └──────────┘
```

Receipts persist for at least the regulatory retention period
(typically 7 years for ISO 9001, 15 years for ISO 13485 medical devices).

---

## 4. Role Delegation

A site delegates rights to a person or service:

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "delegation",
  "delegator_id": "did:wia:site:bonghwa-line-A",
  "delegate_id":  "did:wia:inspector:09…",
  "scopes": ["inspect", "spc_sample"],
  "valid_from": "2026-01-01T00:00:00Z",
  "valid_until": "2026-12-31T23:59:59Z",
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Reserved scopes:

| Scope | Permits |
|-------|---------|
| `inspect`         | Sign inspection results |
| `spc_sample`      | Sign SPC samples |
| `defect_log`      | Sign defect records |
| `calibrate`       | Sign calibration records |
| `ncr_open`        | Open NCR |
| `quality_manager` | Disposition NCR, close CAPA |
| `auditor`         | File audit findings |
| `admin`           | Withdraw any record (audit-only) |

A site MUST refuse a signed envelope whose claimed signer lacks the
required scope. Delegation envelopes are revocable by the delegator
publishing a new envelope with `valid_until` set to the current time.

---

## 5. Replay Defence

Each signed envelope (result, SPC sample, NCR, CAPA, audit finding)
carries a 96-bit nonce and an RFC 3339 timestamp. Receivers MUST:

1. Reject envelopes with skew > ±300 s.
2. Reject envelopes whose `(signer, nonce)` tuple has been seen within
   the last 600 s.
3. Maintain the seen-nonce cache for at least 600 s.

For NCR/CAPA the cache MUST persist across restarts so failover does
not re-open the window for a duplicated disposition write.

---

## 6. Lifecycle State Machines

### 6.1 Inspection Result → NCR

```
inspection_result(verdict=fail) ─auto──► ncr(state=open)
ncr ── disposition assigned ────────────► ncr(state=disposition_set)
ncr ── containment verified ────────────► ncr(state=closed)
ncr(state=open or disposition_set) ── 14d without action ──► auto-escalate
```

A `fail` inspection result MUST auto-create an NCR within 5 seconds of
acknowledgement.

### 6.2 NCR → CAPA

```
ncr(state=closed, severity ∈ {major, critical}) ─auto──► capa(state=open)
capa ── corrective_actions complete ─────────────────────► capa(state=verifying)
capa ── effectiveness_result=effective ──────────────────► capa(state=closed)
capa ── effectiveness_result=ineffective ────────────────► capa(state=re-opened)
re-opened capa SHALL spawn a new capa with supersedes pointer
```

### 6.3 Audit Finding

```
finding(state=open) ── owner accepted ─► state=in_progress
state=in_progress ── action complete ──► state=verification
state=verification ── auditor verified ► state=closed
state=open or in_progress ── due_date passed ─► auto-flag overdue
```

Sites MUST NOT delete audit findings; corrections are appended as new
findings referencing the original via `corrects`.

---

## 7. Supplier and Customer Federation

### 7.1 Incoming Material Certificates

A supplier sends incoming material data via:

```
POST https://customer.example/qc/incoming
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_quality_control_version":"1.0.0",
  "type":"incoming_material_cert",
  "supplier_id":"did:wia:supplier:09…",
  "customer_id":"did:wia:customer:21…",
  "lot_id":"sup-2026-04-001",
  "test_results":[ … ],
  "signature":{ "alg":"Ed25519", "value":"…" } }
```

The customer site verifies the supplier's signature against the supplier's
published key and links the certificate to its own incoming inspection
result.

### 7.2 Outgoing Certificate of Conformance

A site emits a CoC envelope when shipping:

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "certificate_of_conformance",
  "site_id": "did:wia:site:bonghwa-line-A",
  "customer_id": "did:wia:customer:21…",
  "lot_id": "lot_2026-04-A",
  "shipped_at": "2026-04-02T08:00:00Z",
  "evidence_result_ids": ["res_01HZA…"],
  "qms_standards": ["ISO 9001:2015"],
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

CoC envelopes are immutable; revocation is published as a separate
`coc_revocation` envelope referencing the original.

---

## 8. Audience Controls

| Audience | Visibility |
|----------|------------|
| `public` | qms_standards advertised, certifications attested by external bodies |
| `customer` | CoC for shipped lots, plus linked inspection summaries |
| `supplier` | Receipts for incoming material, plus disposition outcomes |
| `auditor` | Full audit findings + linked evidence |
| `regulator` | Full record under formal compliance request |
| `internal` | Full operational data |

Sites MUST refuse cross-class enrichment that defeats the matrix (for
example, joining defect records with employee identifiers on behalf of a
customer audience).

---

## 9. Cryptographic Suite

| Use | Algorithm | Reference |
|-----|-----------|-----------|
| Identity signing | Ed25519 | IETF RFC 8032 |
| HTTP message signing | Ed25519 over RFC 9421 | RFC 9421 |
| Hashing | SHA-256 | FIPS 180-4 |
| Transport | TLS 1.3 | IETF RFC 8446 |

PQC migration tracks the parent WIA family roadmap.

---

## 10. Conformance

A Phase 3 conformant implementation MUST:

1. Implement the federation handshake state machine.
2. Honour replay defence bounds.
3. Auto-create NCR on first failed inspection result per §6.1.
4. Auto-spawn CAPA for major/critical NCR per §6.2.
5. Honour delegation envelopes including scope restrictions.
6. Enforce audience-based read controls.

---

## 11. References

* IETF RFC 8032 — EdDSA
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* FIPS 180-4 — SHA family
* W3C DID 1.0
* ISO 9001:2015, ISO 13485:2016, IATF 16949, AS9100

---

## Appendix A — Worked NCR / CAPA Trace

```
T+0   inspector signs inspection_result with verdict=fail
T+1s  site auto-creates ncr_01HZA, severity=major from rule table
T+5s  site auto-creates capa_01HZA referencing ncr_01HZA
T+10s containment_action recorded by site supervisor
T+1d  quality_manager sets disposition=rework
T+2d  rework completed, ncr_01HZA closed
T+5d  capa corrective_actions complete
T+30d effectiveness_check_at reached; quality_manager records result=effective
T+30d capa_01HZA closed
```

If the effectiveness check returns `ineffective`, a fresh CAPA is created
referencing the prior via `supersedes`. The chain is append-only; no
record in the trace is rewritten.

## Appendix B — Replay Cache Sizing

For a busy site receiving 200 envelopes per second across all roles, the
seen-nonce cache must hold roughly `200 × 600 = 120 000` entries to
enforce §5's 600-second window. With 16-byte nonce keys plus a 4-byte
timestamp, the cache footprint is approximately
`120 000 × 24 ≈ 3 MiB`. Sites SHOULD provision at least double this to
absorb shift-change bursts and SHOULD persist the cache across restarts.

## Appendix C — Privacy Threats and Mitigations

| Threat | Mitigation |
|--------|------------|
| Operator identity correlation across customers | Per-audience read controls; customer audience returns aggregate data only |
| Side-channel timing on signature verify | Constant-time Ed25519 implementations REQUIRED |
| Long-term metadata accumulation by certification body | External auditor receives finding-scoped evidence only, not full record |
| Supplier price disclosure via incoming material data | `incoming_material_cert` MUST NOT carry pricing fields; pricing rides a separate envelope on a separate endpoint |
| Mass-export of inspection records by compromised peer | Sites apply per-source rate limits and revoke peers on first detected anomaly |

## Appendix D — Trust List Maintenance

Each site maintains a signed trust list:

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "trust_list",
  "site_id": "did:wia:site:bonghwa-line-A",
  "issued_at": "2026-04-01T00:00:00Z",
  "valid_until": "2026-05-01T00:00:00Z",
  "entries": [
    { "peer_id": "did:wia:supplier:09…", "role": "supplier",      "score": 0.95 },
    { "peer_id": "did:wia:customer:21…", "role": "customer",      "score": 0.99 },
    { "peer_id": "did:wia:auditor:42…",  "role": "auditor",       "score": 1.00 }
  ],
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Trust lists are republished at least monthly; peers refuse stale lists
older than 60 days. A peer may self-publish a `revocation` envelope to
immediately drop trust between list refresh windows.

## Appendix E — Operator Responsibilities Around Failover

When a site fails over from primary to standby region, the standby MUST:

1. Reload the persistent seen-nonce cache before resuming envelope
   processing — failure to do so re-opens a 600-second window for an
   attacker to replay a previously-seen NCR-disposition envelope and
   forge a closure.
2. Re-issue handshakes to peers whose receipts are not present in the
   standby's storage. Peers that do not see a handshake within their
   trust-list refresh interval will mark the site stale and refuse
   downstream operations.
3. Replay missed inspection_result and SPC envelopes from the primary's
   append-only log before accepting new submissions, so SPC charts do
   not silently reorder during the cut-over window.
4. Notify peers via a `notice` envelope that primary→standby switchover
   has occurred, with an estimated `restore_at` for the primary. Peers
   show this to users to set expectations.

## Appendix F — Worked Customer CoC Trace

```
α = site bonghwa-line-A
β = customer factory-21

T+0   α: ships lot_2026-04-A
T+1m  α: emits certificate_of_conformance referencing res_01HZA…
T+2m  β: GET /qc/coc/lot_2026-04-A from α
        β: verifies α's signature, stores receipt
T+1d  β: incoming inspection on the same lot signs another inspection_result
T+2d  β: discovers an outlier; opens its own ncr in β's QMS
T+3d  β: cross-files the ncr against α via POST /qc/incoming
T+5d  α: opens ncr referencing β's ncr; capa spawned
T+30d capa effectiveness check returns effective; both sides close
```

Throughout this trace, every write is signed and every receipt is
preserved. There is no implicit trust; every cross-site action is an
auditable event.

## Appendix G — Worked Auditor Trace

```
α = site bonghwa-line-A
γ = certification body did:wia:auditor:dnv-001

T-7d  γ → α: handshake (auditor role, scope=audit)
T+0   γ: files audit_finding(severity=major, clause=7.1.5)
T+1d  α: assigns owner, transitions to in_progress
T+30d α: corrective action complete, transitions to verification
T+35d γ: verifies, transitions to closed
T+90d α: surfaces the closed finding to next year's surveillance
        audit as evidence of an effective response.
```

Audit findings are append-only; corrections are new findings linked via
`corrects` rather than rewrites, so an auditor can always reconstruct
the disposition chain.

弘益人間 — Benefit All Humanity.
