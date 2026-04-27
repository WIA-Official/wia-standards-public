# WIA-MED-021 Sleep Monitoring — Phase 4: Integration

**Standard**: WIA-MED-021 Sleep Monitoring
**Phase**: 4 of 4 — Integration
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 4 specifies how WIA-MED-021 integrates with three classes of
existing system:

1. **Clinical sleep medicine standards** — AASM scoring algorithms,
   EDF / EDF+ raw biosignal recordings, HL7 FHIR R5 Observation /
   Procedure resources.
2. **Vendor sleep platforms** — Apple HealthKit (iOS sleep tracking),
   Google Fit (Android sleep tracking), Samsung Health, Fitbit Web API,
   Oura Ring API, Withings.
3. **WIA family standards** — WIA-OMNI-API for credentials,
   WIA-MED-020 (Wearable Health) sister standard, WIA Vital Sign
   Streaming for raw biosignal streams, WIA-AIR-SHIELD for transport
   hardening, WIA-ACCESSIBILITY for patient and clinician interfaces.

---

## 2. AASM Scoring Integration

The AASM Manual for the Scoring of Sleep and Associated Events is the
clinical reference standard. WIA-MED-021 does not re-publish the AASM
rules; it provides the data shape that AASM-compliant scorers consume
and emit.

### 2.1 Algorithm Identification

The `scoring_algorithm` field carries an opaque identifier registered
at `https://wiastandards.com/sleep-monitoring/scoring-algorithms/`.
Reference algorithms in the initial registry:

| Algorithm ID | Description |
|--------------|-------------|
| `AASM-2024-default` | Default scoring per the 2024 edition AASM Manual |
| `AASM-2018-default` | Default scoring per the 2018 edition (preserved for cohort comparability) |
| `AASM-2024-paediatric` | 2024 edition with paediatric adjustments per AASM §IX |
| `Vendor-A-v3` | A reference vendor-specific scoring (consumer wearables) |

### 2.2 Compliance Levels

`aasm_compliance_level` ladder (see Phase 1 §7) lets downstream
consumers filter by appropriateness for their use case. A clinical
sleep clinic typically requires `full`; a research consortium may
accept `partial` with documented rule deviations; a consumer
wellness app accepts `consumer_estimate`.

---

## 3. EDF / EDF+ Bridge

European Data Format (EDF) and EDF+ are the de-facto raw biosignal
formats for sleep medicine. The bridge:

* Reads an EDF / EDF+ file from the recording host's object store.
* Translates the file's per-channel metadata into the WIA session
  envelope (sampling rates, channel names, units, calibration).
* Preserves the EDF file URL plus SHA-256 integrity hash in the
  session envelope so downstream consumers can re-fetch and re-score.
* Writes scoring results back to a parallel WIA scoring_metadata
  envelope without modifying the original EDF file.

The bridge does not transcode the raw biosignal payload. EDF is a
mature, well-supported format; the standard composes with it rather
than replacing it.

---

## 4. HL7 FHIR R5 Bridge

Each WIA session maps to an HL7 FHIR R5 `Procedure` resource
(representing the sleep study) plus one or more `Observation`
resources (representing scoring summary metrics: AHI, RDI, total
sleep time, stage minutes).

Default LOINC mappings:

| WIA scoring summary field | LOINC | Display |
|---------------------------|-------|---------|
| `total_sleep_minutes` | 93832-4 | Sleep duration |
| `ahi`                 | 90562-0 | Apnea hypopnea index |
| `min_spo2_pct`        | 59408-5 | Oxygen saturation in arterial blood by pulse oximetry |
| `n3_minutes`          | 93810-0 | Slow wave sleep duration |
| `rem_minutes`         | 93808-4 | REM sleep duration |

The bridge does not re-publish the EDF raw recording into FHIR;
FHIR `MediaStream` references the EDF URL.

---

## 5. Vendor Platform Bridges

### 5.1 Apple HealthKit

The HealthKit bridge runs as part of an iOS companion app:

* Reads `HKCategoryTypeIdentifierSleepAnalysis` data via HealthKit
  with the user's explicit per-type consent.
* Translates each sleep period into a WIA session envelope with
  `session_kind: consumer_wearable` and `aasm_compliance_level: consumer_estimate`.
* Writes scoring summary back to HealthKit so Apple Health UI shows
  "WIA Sleep Monitoring" as the data source.

### 5.2 Google Fit, Samsung Health, Fitbit, Oura, Withings

Vendor-specific bridge profiles live under
`spec/profiles/<vendor>.md`. Each profile documents OAuth scopes,
field-by-field mapping, polling cadence, and credential management.

---

## 6. WIA Family Integration

### 6.1 WIA-OMNI-API

Patient identity, vendor account credentials, and clinician licence
records live in WIA-OMNI-API. Paediatric custodian relationships
also live there as signed `guardian_link` claims.

### 6.2 WIA-MED-020 (Wearable Health)

WIA-MED-020 and WIA-MED-021 are sister standards. Consumer wearable
sleep data typically arrives at the host as a WIA-MED-020 measurement
stream (heart rate, step count, accelerometer); the host derives a
WIA-MED-021 session by aggregating the measurement stream into
sleep periods and scoring stages from the heart rate / movement
inputs at `consumer_estimate` compliance.

### 6.3 WIA Vital Sign Streaming

Clinical PSG sessions involve continuous high-rate biosignals (EEG at
500 Hz, EMG at 1000 Hz, ECG at 250 Hz, airflow at 25 Hz, SpO2 at 1 Hz).
The session record references these streams via WIA Vital Sign Streaming
URLs; the two standards compose for the full PSG workflow.

### 6.4 WIA-AIR-SHIELD

Transport hardening for cross-organisation federation. Hosts MAY refuse
subscription requests from peers below a configured AIR-SHIELD score.

### 6.5 WIA-ACCESSIBILITY

Patient-facing UI surfaces accommodate accessibility profiles
(large-text mode for elderly patients, high-contrast for low-vision,
voice output for motor-impaired). Sleep clinic results portals use
the patient's accessibility profile for chart rendering and consent UI.

---

## 7. Compliance Mappings

### 7.1 HIPAA

WIA-MED-021 carries PHI; hosts MUST implement HIPAA Security Rule
controls. The patient consent envelope (Phase 3 §4) is the GDPR
Article 7 consent record where applicable.

### 7.2 FDA / CE / KFDA

For clinical-grade PSG hardware, the device record carries the
regulatory-clearance reference per WIA-MED-020 §3.2.

### 7.3 AASM Accreditation

Sleep clinics seeking AASM accreditation can use WIA-MED-021's
audit log as evidence of conformant scoring practice. The standard's
`scorer_id` field ties each session's scoring to a specific
technician's WIA-OMNI-API identity, satisfying the AASM requirement
for traceable scoring provenance.

---

## 8. Migration Paths

### 8.1 From a vendor-locked PSG stack

A sleep clinic currently storing data only in a vendor PSG vendor's
cloud migrates by:

1. Deploying a WIA-MED-021 host alongside the vendor.
2. Importing historical sessions via the vendor's export API
   (typically EDF + vendor-specific scoring XML); the WIA importer
   maps the vendor's scoring vocabulary to AASM tokens.
3. Switching new sessions to direct WIA ingest at the recording
   workstation.
4. Running a 30-day shadow period during which both stacks receive
   new sessions; comparing AHI / total sleep time / REM minutes
   nightly. After three consecutive zero-divergence days, retire
   the vendor stack.

### 8.2 Between hosts

Phase 3 §7 `session_move` envelope handles cross-host migration with
12-month `301` from the old host.

---

## 9. Observability

A conformant host SHOULD expose:

| Metric | Type | Description |
|--------|------|-------------|
| `wia_sm_sessions_received_total{kind}` | counter | per-kind session ingest |
| `wia_sm_scoring_runs_total{algorithm}` | counter | per-algorithm scoring |
| `wia_sm_arousal_too_short_refused_total` | counter | AASM minimum violations |
| `wia_sm_consent_violations_total` | counter | refused cross-host reads |
| `wia_sm_edf_integrity_failures_total` | counter | EDF SHA-256 mismatches |

Labels MUST NOT include patient identifiers.

---

## 10. Conformance Profiles

| Level | Required integrations |
|-------|-----------------------|
| **Minimal** | Phase 1 envelopes, Phase 2 session ingest + query |
| **Core**    | Plus AASM-2024-default scoring, EDF+ bridge, WIA-OMNI-API credentials |
| **Full**    | Plus FHIR R5 export, vendor bridge for ≥1 platform, WIA-MED-020 composition, WIA Vital Sign Streaming for raw EEG/EMG |

Hosts publish their level in `bridge_profile` of the discovery
document.

---

## 11. Worked Example — Clinical PSG Session End-to-End

```
Patient   : did:wia:patient:01HXY
PSG vendor: Vendor-A PSG-001 hardware (FDA 510(k) K123456)
Sleep lab : did:wia:sleep-host:hospital-sleep-clinic-A
Sleep MD  : did:wia:clinician:sleep-md-09
Tech      : did:wia:scorer:tech-09
```

1. Patient consents (clinical_dashboard scope, 1-year validity) for
   the sleep clinic.
2. Recording workstation captures 8-hour overnight PSG; uploads EDF+
   to clinic's WIA-MED-021 host with session record (clinical_grade=true).
3. Tech scores the session manually using AASM-2024-default in vendor
   software; vendor exports scoring results back to WIA host as
   stage_batch + event + arousal envelopes; tech publishes
   scoring_metadata referencing AASM-2024-default.
4. Sleep MD reviews the scoring; if MD requests re-scoring with a
   different algorithm, tech publishes new scoring_metadata with
   `supersedes` reference.
5. EHR bridge writes Procedure + Observation resources to Epic.
6. Patient portal (under WIA-ACCESSIBILITY profile) shows simplified
   summary: AHI, REM percentage, sleep efficiency.
7. After 30-day clinical follow-up, session is archived; data remains
   queryable for 25-year retention per HIPAA + clinical norms.

---

## 12. Security Considerations

* EDF+ raw recordings contain identifying biosignal patterns; hosts
  MUST encrypt EDF at rest with AES-256-GCM and MUST refuse cross-host
  EDF transfer without consent envelope authorisation.
* Scoring services hold AASM expertise; operators MUST use
  WIA-OMNI-API for technician credential storage with periodic
  re-credentialing (typically every 5 years per AASM accreditation).
* Consumer wearable bridges hold OAuth tokens for vendor platforms;
  operators MUST use HSM-backed refresh tokens.

---

## 13. References

* AASM Manual for the Scoring of Sleep and Associated Events (current edition)
* European Data Format (EDF) and EDF+ specification
* HL7 FHIR R5 — Procedure, Observation, Patient resources
* HIPAA Security Rule
* GDPR Article 7
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* FDA 510(k) regulatory pathway
* WIA-OMNI-API standard
* WIA-MED-020 Wearable Health standard
* WIA Vital Sign Streaming standard
* WIA-AIR-SHIELD standard
* WIA-ACCESSIBILITY standard

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

弘益人間 — Benefit All Humanity.
