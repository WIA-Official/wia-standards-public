# WIA-MED-021 Sleep Monitoring — Phase 2: API Interface

**Standard**: WIA-MED-021 Sleep Monitoring
**Phase**: 2 of 4 — API Interface
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 2 specifies the HTTP surface that a sleep monitoring host
(typically a vendor cloud, sleep-lab platform, or hospital RPM
system) exposes for the data families defined in Phase 1.

---

## 2. Transport

* HTTPS only, TLS 1.3 (IETF RFC 8446) or later.
* HTTP/2 RECOMMENDED (IETF RFC 9113); HTTP/1.1 SHALL be supported as
  fallback.
* `Strict-Transport-Security` per IETF RFC 6797.
* Compression: `gzip`, `br`.

---

## 3. Discovery

```
GET https://<host>/.well-known/wia-sleep-monitoring

{
  "wia_sleep_monitoring_version": "1.0.0",
  "host_id": "did:wia:sleep-host:hospital-sleep-lab",
  "endpoints": {
    "session":  "https://sm.example/sm/session",
    "stage":    "https://sm.example/sm/stage",
    "event":    "https://sm.example/sm/event",
    "arousal":  "https://sm.example/sm/arousal",
    "scoring":  "https://sm.example/sm/scoring"
  },
  "supported_signatures": ["Ed25519"],
  "supported_session_kinds": ["polysomnography","home_sleep_apnoea_test","consumer_wearable","nap"],
  "supported_scoring_algorithms": ["AASM-2024-default","AASM-2018-default","Vendor-A-v3"],
  "rate_limits": {
    "anonymous":     { "rps": 1,  "burst": 5 },
    "authenticated": { "rps": 30, "burst": 100 }
  }
}
```

Discovery responses are cacheable for 300 s.

---

## 4. Session Endpoints

```
POST   /sm/session                  (signed)
GET    /sm/session/{session_id}
GET    /sm/session?patient_id={id}&kind={k}&from={ts}&to={ts}
PATCH  /sm/session/{session_id}     (signed; ended_at update + scoring_metadata link)
DELETE /sm/session/{session_id}     (signed; tombstone, audit retention 25 years)
```

### 4.1 Session Lifecycle

```
DRAFT → IN_PROGRESS → SCORING → SCORED → ARCHIVED
                              ↓
                          → CORRECTED (new scoring_metadata appended)
```

State transitions ride as PATCH operations and MUST be signed by a
delegate with the appropriate scope (`technician_scoring`,
`physician_review`, `archive_manager`).

---

## 5. Stage Endpoints

```
POST /sm/stage                      (signed; bulk submit per session)
GET  /sm/stage/{stage_id}
GET  /sm/stage?session_id={id}
```

Bulk submission is the typical pattern: a session's 960 epochs (8
hours × 30 second epochs) ride in a single envelope to amortise the
network round trip. Hosts MUST honour Idempotency-Key.

---

## 6. Event Endpoints

```
POST /sm/event                      (signed; respiratory event submission)
GET  /sm/event/{event_id}
GET  /sm/event?session_id={id}&kind={k}
```

Bulk submission supported via `events: [...]` array. Hosts MUST
preserve event ordering by `started_at` for downstream AHI
re-computation.

---

## 7. Arousal Endpoints

```
POST /sm/arousal                    (signed; bulk submit)
GET  /sm/arousal/{arousal_id}
GET  /sm/arousal?session_id={id}
```

Hosts MUST refuse arousal submissions whose `duration_seconds < 3`
per AASM rules.

---

## 8. Scoring Endpoints

```
POST /sm/scoring                    (signed; new scoring_metadata)
GET  /sm/scoring/{scoring_metadata_id}
POST /sm/scoring/{scoring_metadata_id}/recompute  (re-run with new algorithm)
```

Scoring re-computation is the path for retrospective re-scoring with
a new algorithm version. The original scoring_metadata is preserved;
the new scoring_metadata references the prior via `supersedes`.

---

## 9. Authentication

* Writes require HTTP Message Signatures per IETF RFC 9421.
* Reads MAY be anonymous subject to the discovery rate limit; PHI-bearing
  reads MUST be authenticated.
* Cross-vendor reads use Phase 3 federation receipts.

---

## 10. Errors

Errors follow IETF RFC 9457 problem details. Reserved error types live
under `https://wiastandards.com/sleep-monitoring/errors/`.

```json
{
  "type": "https://wiastandards.com/sleep-monitoring/errors/arousal-too-short",
  "title": "Arousal duration below AASM minimum",
  "status": 422,
  "detail": "arousal.duration_seconds=2 violates AASM 3-second minimum",
  "instance": "/sm/arousal/ar_01HXY-23"
}
```

---

## 11. Conformance

A Phase 2 conformant host MUST:

1. Publish `/.well-known/wia-sleep-monitoring` with all endpoints.
2. Implement session / stage / event / arousal / scoring routes with
   the documented status codes.
3. Honour `Idempotency-Key` for all writes.
4. Refuse arousal submissions below the AASM 3-second minimum.
5. Refuse `clinical_grade: true` without regulatory clearance proof.
6. Emit problem-detail JSON for 4xx/5xx responses.

---

## 12. References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 6797 — HSTS
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 9457 — Problem Details for HTTP APIs
* AASM Manual for the Scoring of Sleep and Associated Events
* JSON Schema Draft 2020-12

---

## Appendix A — Reference Request / Response

### A.1 Submit a session

```http
POST /sm/session HTTP/1.1
Host: sm.example
Content-Type: application/json
Idempotency-Key: 7e3a9b6c-...
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_sleep_monitoring_version":"1.0.0", "type":"session",
  "patient_id":"did:wia:patient:01HXY",
  "session_kind":"polysomnography",
  "started_at":"2026-04-26T22:30:00Z",
  "ended_at":"2026-04-27T07:00:00Z",
  "device_ids":["did:wia:device:psg-vendor-A"],
  "clinical_grade":true,
  "edf_url":"https://recordings.example/sessions/ses_01HXY.edf",
  "edf_sha256":"7d8f2c…",
  "signature":{ "alg":"Ed25519", "value":"…" } }

HTTP/1.1 201 Created
Location: /sm/session/ses_01HXY
RateLimit-Limit: 30
RateLimit-Remaining: 29
```

### A.2 Bulk-submit sleep stages

```http
POST /sm/stage HTTP/1.1
Host: sm.example
Content-Type: application/json
Idempotency-Key: 7e3a9b6c-...

{ "wia_sleep_monitoring_version":"1.0.0",
  "type":"stage_batch",
  "session_id":"ses_01HXY",
  "stages":[
    { "epoch_index":0,   "epoch_start":"2026-04-26T22:30:00Z",
      "epoch_duration_seconds":30, "stage":"wake", "confidence":0.99 },
    { "epoch_index":1,   "epoch_start":"2026-04-26T22:30:30Z",
      "epoch_duration_seconds":30, "stage":"n1",   "confidence":0.85 }
    /* ... 960 epochs total ... */
  ],
  "signature":{ "alg":"Ed25519", "value":"…" } }

HTTP/1.1 201 Created
```

## Appendix B — Conformance Test Suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-sleep-monitoring-conformance` and
walks through:

1. Discovery document round-trip.
2. Session create, read, withdraw with clearance refusal path.
3. Stage bulk submit with idempotency.
4. Event submission with arousal-duration validation.
5. Scoring re-computation chain (supersedes).
6. Federation handshake against a mock peer.
7. Rate-limit headers and exhaustion behaviour.

## Appendix C — Operational Recommendations

* Hosts MUST persist incoming writes durably before acknowledging.
* Hosts SHOULD provide a separate read-replica endpoint for query-heavy
  research queries.
* Hosts SHOULD expose a `/sm/health` liveness probe outside the
  public rate-limit accounting.
* Hosts SHOULD retain problem-detail responses to identified peers
  for 30 days to support cross-host debugging.

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
