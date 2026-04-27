# WIA-MED-020 Wearable Health — Phase 3: Protocol

**Standard**: WIA-MED-020 Wearable Health Monitoring
**Phase**: 3 of 4 — Protocol
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 3 specifies how independent wearable health hosts (vendor cloud,
hospital RPM platform, EHR bridge, research consortium) build trust
over time, exchange device data without merging databases, defend
against replay, and enforce the patient-consent boundary that wearable
data necessarily crosses on its way from a wrist to a clinic.

---

## 2. Roles

| Role | Description |
|------|-------------|
| **Vendor cloud** | Holds raw device data; signs envelopes for downstream consumers |
| **Patient-controlled host** | Holds the patient's portable identity and consent envelopes |
| **Clinical host** | Hospital / clinic RPM platform; receives data under consent |
| **Research aggregator** | Receives de-identified data under explicit research consent |
| **Custodian** | Holds the legally binding consent record (often the patient themselves, or a parent for minors) |

A single legal entity MAY play multiple roles. Trust between roles is
established by federation handshake and recorded in signed receipts.

---

## 3. Federation Handshake

```
   IDLE
     │ peer presents credential + ephemeral key
     ▼
   PENDING (origin verifies signature)
     │ valid
     ▼
   ACCEPTED (origin issues federation receipt; both sides persist)
     │ optional revocation
     ▼
   REVOKED
```

The handshake reuses the WIA-SOCIAL Phase 3 §5 receipt shape so that
vendor implementations can share their federation library across
multiple WIA family standards.

Receipts persist for at least the regulatory retention period
(typically 7 years for HIPAA-bound data, 25 years for paediatric
consent records).

---

## 4. Patient Consent Envelope

The patient's signed consent envelope is the gating control on cross-host
data flow:

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "patient_consent",
  "patient_id": "did:wia:patient:01HXY",
  "custodian_id": "did:wia:patient:01HXY",
  "scopes": ["clinical_dashboard", "ehr_export", "research_aggregate"],
  "audiences": [
    { "audience_id": "did:wia:clinical-host:hospital-A", "scope": "clinical_dashboard" },
    { "audience_id": "did:wia:research:univ-X",           "scope": "research_aggregate" }
  ],
  "valid_from": "2026-04-01T00:00:00Z",
  "valid_until": "2027-04-01T00:00:00Z",
  "revocable": true,
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

Vendor clouds MUST refuse cross-host reads whose patient identifier
lacks an active consent envelope from the custodian, except in declared
emergency scope (`scope = ["emergency"]`) which auto-expires after 24
hours and triggers a mandatory custodian review.

For paediatric patients, the custodian is the legal guardian; the
custodian MUST be cleared via a separate identity-proofing flow
documented in Phase 4 §WIA-OMNI-API.

---

## 5. Replay Defence

Each signed envelope (sync, alert, consent) carries a 96-bit nonce and
an RFC 3339 timestamp. Receivers MUST:

1. Reject envelopes with skew > ±300 s.
2. Reject envelopes whose `(signer, nonce)` tuple has been seen within
   the last 600 s.
3. Maintain the seen-nonce cache for at least 600 s.

For high-volume bulk sync the cache MAY be a Bloom filter; false
positives MUST trigger a re-fetch via the sync endpoint rather than
silent drops.

For critical alerts the cache MUST be persistent across restarts so
that a vendor failover does not re-open the window for a duplicated
critical-alert envelope (which could trigger duplicate clinician pages).

---

## 6. Audience Controls

| Audience | Visibility |
|----------|------------|
| `public` | Aggregate engagement statistics only |
| `patient` | Full own data |
| `caregiver` | Per-scope (typically high-level summaries) |
| `clinician` | Full data for patients under active care |
| `ehr_bridge` | Per-minute aggregates by default; full waveform requires elevated consent |
| `research` | De-identified per HIPAA Safe Harbor §164.514(b)(2), under explicit research consent |
| `emergency_service` | Full data + identifying metadata under emergency consent |

Hosts MUST refuse cross-class enrichment that defeats the matrix.

---

## 7. Cross-Vendor Move

A patient transferring from one vendor cloud to another:

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "patient_move",
  "patient_id": "did:wia:patient:01HXY",
  "from_vendor": "did:wia:vendor:apple-health",
  "to_vendor":   "did:wia:vendor:samsung-health",
  "moved_at": "2026-04-27T10:00:00Z",
  "scope": "full_history",
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

The new vendor fetches the patient's history from the prior vendor over
a federated read, then hosts the data going forward. The old vendor
returns `301 Moved Permanently` for the patient's data URL for at
least 12 months.

---

## 8. Cryptographic Suite

| Use | Algorithm | Reference |
|-----|-----------|-----------|
| Identity signing | Ed25519 | IETF RFC 8032 |
| HTTP message signing | Ed25519 over RFC 9421 | RFC 9421 |
| Hashing | SHA-256 | FIPS 180-4 |
| Transport | TLS 1.3 | IETF RFC 8446 |
| Data-layer (raw payloads) | AES-256-GCM | FIPS 197 / NIST SP 800-38D |

PQC migration tracks the parent WIA family roadmap.

---

## 9. Conformance

A Phase 3 conformant implementation MUST:

1. Implement the federation handshake state machine.
2. Honour replay-defence bounds.
3. Enforce patient consent before serving any non-emergency cross-host
   read.
4. Apply audience-based read controls.
5. Maintain an append-only audit log of every cross-host read signed by
   the requesting peer.

---

## 10. References

* IETF RFC 8032 — EdDSA
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* FIPS 180-4 — SHA family
* FIPS 197 — AES
* NIST SP 800-38D — GCM mode
* HIPAA Safe Harbor §164.514(b)(2)
* HL7 FHIR R5 Consent resource

---

## Appendix A — Worked Federation Trace

```
α = Apple Health (vendor cloud)        did:wia:vendor:apple-health
β = Hospital RPM platform              did:wia:clinical-host:hospital-rpm
λ = patient                            did:wia:patient:01HXY
```

```
T-1d  λ publishes a patient_consent envelope authorising β as
      audience=clinician_dashboard for scope=heart_rate, sleep
T+0   β → α: federation handshake; β presents clinical-host credential
T+5s  α verifies + receipt issued; β stores receipt
T+10s β: GET /wh/measurement?patient_id=λ&type=heart_rate&from=T-7d
T+11s α: verifies consent + receipt; SSE stream of historical + live
      measurements opens
T+1h  λ revokes consent for β
T+1h+1s α emits a notice envelope; β closes the SSE
T+1h+2s β confirms data destruction within scope's redaction_window
        and emits a destruction receipt
```

If consent verification fails at any point, α returns a problem
document of type `…/consent-not-found` and the SSE is closed with
`event: error` per WHATWG HTML §9.2.

## Appendix B — Replay Cache Sizing

For a busy vendor cloud receiving 500 sync envelopes per second
(typical Saturday afternoon spike for fitness tracker users), the
seen-nonce cache must hold roughly `500 × 600 = 300 000` entries.
With 16-byte nonce keys plus a 4-byte timestamp, the strict cache
footprint is approximately `300 000 × 24 ≈ 7.5 MiB`. Vendors at this
scale SHOULD use a Bloom filter sized for 1 % false positive rate
(~ 1 MiB) for sync paths and a strict cache for alert paths.

## Appendix C — Privacy Threats and Mitigations

| Threat | Mitigation |
|--------|------------|
| Vendor lock-in via opaque history | Patient_move envelope (§7) with full_history scope |
| Bulk export by compromised clinical host | Per-audience read controls; revoke peer on first detected anomaly |
| Re-identification of de-identified research export | HIPAA Safe Harbor §164.514(b)(2) plus k-anonymity ≥ 5 |
| Side-channel timing on signature verify | Constant-time Ed25519 implementations REQUIRED |
| Critical alert duplication on vendor failover | Persistent seen-nonce cache for alert path |
| Side-channel via timing on consent verify | Constant-time signature verification of consent envelope |

## Appendix D — Trust List Maintenance

Each host maintains a signed trust list of federated peers (clinical
hosts, research aggregators, custodians). Trust lists are republished
at least monthly; peers refuse stale lists older than 60 days. A peer
may self-publish a `revocation` envelope to immediately drop trust
between list refresh windows.

## Appendix E — Operator Failover Notes

When a vendor cloud fails over from primary to standby region, the
standby MUST:

1. Reload the persistent seen-nonce cache before resuming envelope
   processing — failure to do so re-opens a 600-second window for an
   attacker to replay a previously-seen alert envelope, potentially
   triggering duplicate clinician pages.
2. Re-issue handshakes to peers whose receipts are not present in the
   standby's storage.
3. Replay any missed alert envelopes from the primary's append-only
   log before accepting new alerts. For critical alerts the standby
   MUST NOT skip this step — a missed alert is a clinical-safety event.
4. Notify peers via a `notice` envelope that primary→standby
   switchover has occurred, with an estimated `restore_at` for the
   primary.

## Appendix F — Worked Patient-Move Trace

A patient moving from Apple Health to Samsung Health:

```
T+0    λ: signs patient_move envelope (full_history scope)
T+1s   λ: presents move envelope to Samsung Health
T+2s   Samsung Health: federation handshake with Apple Health
T+5s   Samsung Health: GET /wh/measurement?patient_id=λ&since=earliest
T+1m   Apple Health: streams entire history (typically ~2 GB for 5y)
T+30m  Samsung Health: history fully ingested; sends destination receipt
T+30m+1s Apple Health: marks the patient's data path as moved;
                      future reads return 301 to Samsung Health
T+12mo Apple Health: 301 expires; data may be deleted per
                     local retention policy and patient's deletion request
```

The move is signed by the patient throughout; neither vendor can
move data without the patient's signature, and the audit log of the
move is preserved by both vendors.

## Appendix G — Audience Decision Matrix

| Caller | device | measurement | session | alert | calibration |
|--------|--------|-------------|---------|-------|-------------|
| Public anonymous            | metadata only | aggregate stats | aggregate stats | ✗ | ✗ |
| Patient (own data)          | full | full | full | full | full |
| Caregiver (delegated scope) | full | per-scope summaries | per-scope summaries | per-scope | metadata |
| Clinician (active care)     | full | full | full | full | full |
| EHR bridge                  | metadata | per-minute aggregates | summary | metadata only | metadata only |
| Research aggregator         | de-identified | de-identified per HIPAA Safe Harbor | de-identified | ✗ | ✗ |
| Emergency service           | full | full | recent only | full | metadata |

Hosts MUST refuse cross-class enrichment that defeats the matrix —
for example, joining measurement frames with device serial numbers on
behalf of the research aggregator audience is prohibited even when
both are individually permitted.

## Appendix H — Notes on Paediatric Custodian Flow

For paediatric patients, the custodian (legal guardian) holds the
consent envelope on behalf of the child. The flow:

1. Guardian's identity is proofed via WIA-OMNI-API (typically via
   government-issued ID verification).
2. WIA-OMNI-API issues a signed `guardian_link` claim binding the
   guardian's DID to the child's DID with a documented relationship
   type (parent / legal guardian / foster parent / ward).
3. The guardian signs the patient consent envelope on behalf of the
   child; the consent envelope's `custodian_id` is the guardian's DID.
4. When the child reaches the local age of majority (defined by
   jurisdiction in the WIA-OMNI-API claim), the guardian_link claim
   automatically expires; the child publishes their own consent
   envelopes from that point forward.
5. Hosts MUST NOT serve consent envelopes signed by an expired
   guardian_link, even if the consent envelope's `valid_until` has
   not yet passed.

This flow is intentionally strict: paediatric consent is high-stakes
and the standard treats it with the same rigour as the adult
identity-proofing flow plus an explicit relationship-type assertion.

弘益人間 — Benefit All Humanity.
