# WIA-MED-021 Sleep Monitoring — Phase 1: Data Format

**Standard**: WIA-MED-021 Sleep Monitoring
**Phase**: 1 of 4 — Data Format
**Version**: 1.0.0
**Status**: Draft
**Philosophy**: 弘益人間 — Benefit All Humanity

---

## 1. Scope

Phase 1 fixes the JSON shapes that every WIA-MED-021 conformant
device, sleep lab system, EHR bridge, or research aggregator MUST
read and emit.

| Object family | Purpose |
|---------------|---------|
| **Session** | A contiguous sleep recording session (one night, or one nap) |
| **Sleep stage** | One epoch (typically 30 seconds) classified into AASM stages |
| **Respiratory event** | Apnoea, hypopnoea, RERA, oxygen desaturation event |
| **Arousal** | EEG-based or movement-based arousal event |
| **Scoring metadata** | The algorithm + scorer responsible for the session's stage classifications |

Out of scope: HTTP surface (Phase 2), federation across organisations
(Phase 3), HL7 FHIR / EDF / vendor bridges (Phase 4).

---

## 2. Encoding Rules

* UTF-8 JSON per IETF RFC 8259, `snake_case` keys.
* Timestamps RFC 3339 in UTC, `Z` suffix; sub-second precision required
  for events tied to specific waveform samples.
* Identifiers URI-shaped per IETF RFC 3986. Patient identifiers SHOULD
  use `did:wia:patient:…` to keep biometric data out of the wire.
* Sleep stage tokens use the AASM Manual current edition: `wake`,
  `n1`, `n2`, `n3`, `rem`. Implementations MAY also emit `unscored`
  for epochs the algorithm could not classify.
* Numeric fields use IEEE 754 double precision.

### 2.1 Versioning

```json
"wia_sleep_monitoring_version": "1.0.0"
```

A receiver MUST refuse a major version it does not implement.

---

## 3. Session Record

```json
{
  "wia_sleep_monitoring_version": "1.0.0",
  "type": "session",
  "session_id": "ses_01HXY",
  "patient_id": "did:wia:patient:01HXY",
  "session_kind": "polysomnography",
  "started_at": "2026-04-26T22:30:00Z",
  "ended_at":   "2026-04-27T07:00:00Z",
  "device_ids": ["did:wia:device:psg-vendor-A"],
  "clinical_grade": true,
  "edf_url": "https://recordings.example/sessions/ses_01HXY.edf",
  "edf_sha256": "7d8f2c…",
  "scoring_metadata_id": "scm_01HXY",
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

### 3.1 Session Kinds

`polysomnography` (full clinical PSG with EEG + EOG + EMG + airflow +
pulse oximetry + chest/abdomen belts), `home_sleep_apnoea_test`
(typically airflow + SpO2 only), `consumer_wearable` (typically
heart rate + accelerometer derived sleep stages), `nap`. Other
session kinds MAY be registered via the WIA Standards extension
process; receivers MUST treat unknown kinds as `unrecognised`.

### 3.2 Clinical Grade

`clinical_grade: true` MAY be set only when the recording device has
regulatory clearance for the relevant signals (FDA 510(k), EU CE-MDR,
KFDA approval). When `true`, the device record MUST carry the
clearance reference (see WIA-MED-020 Phase 1 §3.2).

### 3.3 EDF / EDF+ Reference

For clinical-grade sessions, the session record MUST reference the
underlying European Data Format (EDF / EDF+) raw biosignal recording
via `edf_url` plus `edf_sha256` integrity hash. The standard does not
require the EDF file to ride inline — the recording is typically
hundreds of megabytes and lives in a separate object store.

---

## 4. Sleep Stage Record

```json
{
  "wia_sleep_monitoring_version": "1.0.0",
  "type": "sleep_stage",
  "session_id": "ses_01HXY",
  "epoch_index": 142,
  "epoch_start": "2026-04-26T23:41:00Z",
  "epoch_duration_seconds": 30,
  "stage": "n2",
  "confidence": 0.92,
  "scoring_algorithm": "AASM-2024-default",
  "signature": null
}
```

Stages are typically scored at 30-second epoch granularity per AASM
guidelines. Consumer wearables MAY use 5-minute epochs at lower
confidence; the `epoch_duration_seconds` field carries the actual
granularity.

`signature` is optional for stage records inside a session because the
session itself is signed and the stages inherit the session's
provenance. Standalone-published stage records (from a community
research aggregator, for example) MUST be signed.

---

## 5. Respiratory Event Record

```json
{
  "wia_sleep_monitoring_version": "1.0.0",
  "type": "respiratory_event",
  "session_id": "ses_01HXY",
  "event_id": "re_01HXY-12",
  "event_kind": "obstructive_apnoea",
  "started_at": "2026-04-26T23:45:12.300Z",
  "duration_seconds": 14,
  "associated_oxygen_desaturation_pct": 3.5,
  "associated_arousal": true,
  "scoring_algorithm": "AASM-2024-default",
  "scorer_id": "did:wia:scorer:tech-09",
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

### 5.1 Event Kinds

`obstructive_apnoea`, `central_apnoea`, `mixed_apnoea`, `hypopnoea`,
`rera` (Respiratory Effort Related Arousal), `oxygen_desaturation`.
Each kind has its own AASM scoring criteria; the standard preserves
the per-event metadata so the AHI (Apnoea-Hypopnoea Index) can be
re-computed by downstream consumers without ambiguity.

### 5.2 AHI / RDI Computation

```
AHI = (apnoeas + hypopnoeas) / sleep_hours
RDI = (apnoeas + hypopnoeas + reras) / sleep_hours
```

Implementations MUST publish the AHI and RDI in the scoring_metadata
record; downstream consumers MUST trust the published values rather
than recomputing from individual events (because consumer wearables
sometimes derive AHI from non-AASM-equivalent inputs).

---

## 6. Arousal Record

```json
{
  "wia_sleep_monitoring_version": "1.0.0",
  "type": "arousal",
  "session_id": "ses_01HXY",
  "arousal_id": "ar_01HXY-23",
  "started_at": "2026-04-26T23:45:25.100Z",
  "duration_seconds": 4,
  "kind": "spontaneous",
  "associated_event_id": null,
  "signature": null
}
```

Kinds: `spontaneous`, `respiratory` (associated with a respiratory
event), `limb_movement`, `external` (noise, light, etc.). Arousals
shorter than 3 seconds per AASM are not scored; implementations MUST
NOT emit sub-3-second arousals.

---

## 7. Scoring Metadata Record

```json
{
  "wia_sleep_monitoring_version": "1.0.0",
  "type": "scoring_metadata",
  "scoring_metadata_id": "scm_01HXY",
  "session_id": "ses_01HXY",
  "scoring_algorithm": "AASM-2024-default",
  "scorer_id": "did:wia:scorer:tech-09",
  "scored_at": "2026-04-27T09:00:00Z",
  "summary": {
    "total_sleep_minutes": 480,
    "rem_minutes": 102,
    "n1_minutes": 35,
    "n2_minutes": 240,
    "n3_minutes": 88,
    "wake_minutes": 25,
    "ahi": 4.5,
    "rdi": 6.0,
    "min_spo2_pct": 88,
    "average_spo2_pct": 95
  },
  "aasm_compliance_level": "full",
  "signature": { "aleg": "Ed25519", "value": "Hd9w…" }
}
```

`aasm_compliance_level` is one of:

* `full` — meets every AASM Manual scoring rule for the current edition.
* `partial` — meets a documented subset of rules; the diff from `full`
  is published at `https://wiastandards.com/sleep-monitoring/aasm-partial/{algorithm_id}.md`.
* `consumer_estimate` — derives stages from non-AASM-equivalent inputs
  (heart rate, accelerometer, etc.); not suitable for clinical decisions.

---

## 8. Schema Files

JSON Schema 2020-12 documents are served from
`https://wiastandards.com/sleep-monitoring/schemas/`. Implementations
SHOULD bundle local copies for offline validation.

---

## 9. Conformance

A Phase 1 conformant implementation MUST:

1. Round-trip every object family byte-identically through encode/decode.
2. Reject objects missing required fields per the JSON Schemas.
3. Treat unknown optional fields as non-fatal.
4. Refuse `clinical_grade: true` without a regulatory-clearance reference.
5. Honour the AASM scoring vocabulary (§2 stage tokens, §5.1 event kinds).
6. Refuse arousals shorter than 3 seconds per AASM rules.

---

## 10. References

* AASM Manual for the Scoring of Sleep and Associated Events (current edition)
* IETF RFC 8259 — JSON
* IETF RFC 3339 — Date/Time
* IETF RFC 3986 — URI Generic Syntax
* IETF RFC 8032 — EdDSA / Ed25519
* European Data Format (EDF) and EDF+ specification
* HL7 FHIR R5 — Observation, Procedure, Patient resources
* JSON Schema Draft 2020-12

---

## Appendix A — Reserved Tokens

| Field | Reserved tokens |
|-------|-----------------|
| `stage` | `wake`, `n1`, `n2`, `n3`, `rem`, `unscored` |
| `event_kind` | `obstructive_apnoea`, `central_apnoea`, `mixed_apnoea`, `hypopnoea`, `rera`, `oxygen_desaturation` |
| `kind` (arousal) | `spontaneous`, `respiratory`, `limb_movement`, `external` |
| `session_kind` | `polysomnography`, `home_sleep_apnoea_test`, `consumer_wearable`, `nap` |
| `aasm_compliance_level` | `full`, `partial`, `consumer_estimate` |

Future minor versions add tokens but never remove them.

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
