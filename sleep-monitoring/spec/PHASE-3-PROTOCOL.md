# WIA-MED-021 Sleep Monitoring — Phase 3: Protocol

**Standard**: WIA-MED-021 Sleep Monitoring
**Phase**: 3 of 4 — Protocol
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 3 specifies how independent sleep monitoring hosts (vendor
cloud, sleep lab, hospital RPM platform, research aggregator)
build trust over time, exchange session data without merging
databases, defend against replay, and enforce the patient-consent
boundary that sleep data necessarily crosses on its way from a
sensor to a clinical chart or a research dataset.

---

## 2. Roles

| Role | Description |
|------|-------------|
| **Recording host** | Holds raw session data; signs envelopes for downstream consumers |
| **Scoring service** | Performs AASM-compliant scoring against a session's raw recording |
| **Clinical host** | Hospital sleep clinic platform; receives scored sessions under consent |
| **Research aggregator** | Receives de-identified session data under explicit research consent |
| **Custodian** | Holds the legally binding consent record (patient, or guardian for minors) |

A single legal entity MAY play multiple roles. Trust is established
by federation handshake and recorded in signed receipts.

---

## 3. Federation Handshake

The handshake reuses the WIA-SOCIAL Phase 3 §5 receipt shape so that
vendor implementations can share their federation library across
multiple WIA family standards.

```
   IDLE → PENDING (origin verifies signature) → ACCEPTED → optional REVOKED
```

Receipts persist for at least 25 years per typical clinical sleep
record retention norms.

---

## 4. Patient Consent Envelope

The patient consent envelope is the gating control on cross-host
data flow:

```json
{
  "wia_sleep_monitoring_version": "1.0.0",
  "type": "patient_consent",
  "patient_id": "did:wia:patient:01HXY",
  "custodian_id": "did:wia:patient:01HXY",
  "scopes": ["clinical_dashboard", "ehr_export", "research_de_identified"],
  "audiences": [
    { "audience_id": "did:wia:clinical-host:hospital-sleep-clinic-A", "scope": "clinical_dashboard" },
    { "audience_id": "did:wia:research:univ-X-sleep-cohort",          "scope": "research_de_identified" }
  ],
  "valid_from": "2026-04-01T00:00:00Z",
  "valid_until": "2027-04-01T00:00:00Z",
  "revocable": true,
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

For paediatric patients (sleep apnoea is increasingly recognised in
children), the custodian is the legal guardian; the custodian MUST be
cleared via the WIA-OMNI-API guardian-link flow.

---

## 5. Replay Defence

Each signed envelope (session, stage batch, event, scoring, consent)
carries a 96-bit nonce and an RFC 3339 timestamp. Receivers MUST:

1. Reject envelopes with skew > ±300 s.
2. Reject envelopes whose `(signer, nonce)` tuple has been seen within
   the last 600 s.
3. Maintain the seen-nonce cache for at least 600 s.

For high-volume bulk stage submission the cache MAY be a Bloom filter;
false positives MUST trigger a re-fetch via the standard's pull
endpoints rather than silent drops.

---

## 6. Audience Controls

| Audience | Visibility |
|----------|------------|
| `public` | Aggregate sleep statistics only |
| `patient` | Full own data |
| `caregiver` | Per-scope summaries (typically stage histograms, not raw events) |
| `clinician` | Full data for patients under active care |
| `ehr_bridge` | Scoring summary only; raw events require elevated consent |
| `research_de_identified` | De-identified per HIPAA Safe Harbor §164.514(b)(2), under explicit research consent, k-anonymity ≥ 5 |

Hosts MUST refuse cross-class enrichment that defeats the matrix.

---

## 7. Cross-Host Session Move

A patient transferring care between sleep clinics signs a
`session_move` envelope. The new clinic fetches the session history
from the prior clinic over a federated read; the old clinic returns
`301 Moved Permanently` for the session URL for at least 12 months.

---

## 8. Scoring Re-Computation Across Hosts

When AASM publishes a Manual update (typically every few years), every
session may need re-scoring under the new rules. The standard supports
this without forcing physical data re-transfer:

1. The recording host keeps the original scoring_metadata.
2. A scoring service (which may be a different organisation) federates
   with the recording host, fetches the EDF recording, scores it under
   the new algorithm, and POSTs a new scoring_metadata back to the
   recording host with `supersedes` referencing the prior.
3. The recording host preserves both scoring_metadata records.
4. Downstream consumers can choose which scoring to trust based on
   their use case (research consortia typically pin to a specific
   algorithm version for cohort comparability).

---

## 9. Cryptographic Suite

| Use | Algorithm | Reference |
|-----|-----------|-----------|
| Identity signing | Ed25519 | IETF RFC 8032 |
| HTTP message signing | Ed25519 over RFC 9421 | RFC 9421 |
| Hashing (incl. EDF integrity) | SHA-256 | FIPS 180-4 |
| Transport | TLS 1.3 | IETF RFC 8446 |
| Data-layer (EDF at rest) | AES-256-GCM | FIPS 197 / NIST SP 800-38D |

---

## 10. Conformance

A Phase 3 conformant implementation MUST:

1. Implement the federation handshake state machine.
2. Honour replay-defence bounds.
3. Enforce patient consent before serving any cross-host read.
4. Apply audience-based read controls.
5. Support scoring re-computation chain via `supersedes`.
6. Maintain an append-only audit log of every cross-host read.

---

## 11. References

* AASM Manual for the Scoring of Sleep and Associated Events
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
α = sleep lab (vendor cloud)        did:wia:vendor:psg-cloud
β = academic research consortium    did:wia:research:univ-X-sleep-cohort
λ = patient                         did:wia:patient:01HXY
```

```
T-1d  λ publishes patient_consent (research_de_identified scope)
T+0   β → α: federation handshake (research aggregator role)
T+5s  α verifies + receipt issued; β stores receipt
T+10s β: GET /sm/session?patient_id=λ&kind=polysomnography
T+11s α: verifies consent + audience scope; returns de-identified
      session list (k-anonymity ≥ 5; no patient_id, only cohort token)
T+1h  λ revokes research consent
T+1h+1s α emits notice envelope; β confirms data destruction within
        the consent's redaction_window (24 hours default)
T+1d  β publishes signed destruction receipt
```

If consent verification fails at any point, α returns a problem
document of type `…/consent-not-found` and the read is denied.

## Appendix B — Replay Cache Sizing

For a typical sleep-lab cloud receiving 50 session-stage batches per
second across all customers (peak post-overnight upload), the
seen-nonce cache must hold roughly `50 × 600 = 30 000` entries.
With 16-byte nonce keys plus a 4-byte timestamp, the strict cache
footprint is approximately `30 000 × 24 ≈ 720 KiB`. Hosts SHOULD
provision at least double this to absorb morning-upload bursts.

## Appendix — Implementation Notes

### Conformance test suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-sleep-monitoring-conformance` and
walks through the public surface of this Phase. Hosts publishing
`bridge_profile=Full` SHOULD additionally pass the suite's extension
tests for at least one supported third-party scoring algorithm and one
vendor wearable platform.

### Reference container

The `wia/sleep-monitoring-host:1.0.0` container image implements every
endpoint specified in this Phase with mock data; integrators exercise
their bridge against it before going to production. The container ships
with a SQLite-backed store suitable for small-to-medium clinical-research
deployments (up to 100 000 sessions) and a built-in AASM-2024-default
scoring algorithm for the consumer-grade portion of the workload.

### Operational considerations

Sleep monitoring infrastructure has three operational considerations
that integrators consistently underestimate. First, EDF file storage:
a single overnight PSG produces 100-500 MB of EDF data; clinics
typically retain raw EDF for 7-25 years per local regulation, so cold
storage cost dominates the deployment budget. Second, scoring re-runs:
when AASM publishes a Manual update, every session may need re-scoring;
the standard's scoring_metadata supersedes chain (Phase 3 §8) lets this
happen without forcing physical data re-transfer. Third, paediatric
patients: the AASM rules for children differ from adults; the
scoring_algorithm field must distinguish (e.g. `AASM-2024-paediatric`)
so adult-rule comparisons against paediatric data are not silently made.

### Backwards-compatibility promise

Within the 1.x line every endpoint and envelope listed in this Phase
MUST remain reachable and MUST continue to honour the documented
status codes and content shapes. Hosts MAY add optional query parameters,
response fields, new endpoints, or media types. Hosts MUST NOT remove
or repurpose existing ones. Breaking changes ride a major version bump
and MUST be preceded by a 12-month deprecation window per IETF
RFC 8594 and RFC 9745.

## Appendix — Trust List + Audit

Each host maintains a signed trust list of federated peers (recording
hosts, scoring services, clinical hosts, research aggregators).
Trust lists are republished at least monthly; peers refuse stale
lists older than 60 days. A peer may self-publish a `revocation`
envelope to immediately drop trust between list refresh windows.

Audit logs MUST capture every cross-host read with the requesting
peer's federation receipt, the consent envelope referenced, the
returned scope, and the timestamp. The audit log is append-only;
operators MUST NOT mutate prior entries. The audit log is the primary
evidence base for HIPAA / GDPR / AASM accreditation reviews.

## Appendix — Operator Failover Notes

When a host fails over from primary to standby region, the standby
MUST: reload the persistent seen-nonce cache before resuming envelope
processing; re-issue handshakes to peers whose receipts are not present
in the standby's storage; replay any missed scoring envelopes from
the primary's append-only log; notify peers via a `notice` envelope
that primary→standby switchover has occurred, with an estimated
`restore_at` for the primary.

For sleep-monitoring data the failover is rarely time-critical (sleep
studies are typically scored within 1-3 days of recording, not
minutes), but the audit-log preservation contract still applies — a
missed scoring envelope is an evidence-trail gap that auditors will
flag.

## Appendix — Reserved Endpoint Paths and Reserved Tokens

Implementations MUST NOT serve unrelated content under the following
endpoint prefixes; future minor versions reserve them for additional
features: `/sm/audit/`, `/sm/integrity/`, `/sm/regulator/`, `/sm/research/`,
`/sm/family/`. Hosts that need to surface unrelated tooling on the same
origin MUST use a distinct subdomain or path prefix outside the reserved
namespace. The reserved-token tables in earlier sections list the
stable enums for each field; future minor versions add tokens but never
remove them, and implementations MUST treat unknown tokens as
`unrecognised` rather than fail the envelope.

## Appendix — Schema Versioning Notes

The `wia_sleep_monitoring_version` field uses semver. Major version
bumps are breaking and require a 12-month deprecation window per IETF
RFC 8594 / RFC 9745. Minor version bumps are additive: new fields,
new enum tokens, new endpoint variants. Patch version bumps fix
documentation, examples, or non-normative clarifications without
changing the wire format.

Receivers MUST refuse a major version they do not implement.
Receivers MUST treat unknown minor-version optional fields as
non-fatal extensions. Receivers SHOULD log a warning when they
encounter an unknown enum token so operators can plan a registry
update; the receiver MUST still process the rest of the envelope
correctly.

弘益人間 — Benefit All Humanity.
