# WIA-MED-020 Wearable Health — Phase 1: Data Format

**Standard**: WIA-MED-020 Wearable Health Monitoring
**Phase**: 1 of 4 — Data Format
**Version**: 1.0.0
**Status**: Draft
**Philosophy**: 弘益人間 — Benefit All Humanity

---

## 1. Scope

Phase 1 fixes the JSON shapes that every WIA-MED-020 conformant device,
mobile app, hub, dashboard, or EHR bridge MUST be able to read and
emit so that data flows portably across the wearable health ecosystem.

| Object family | Purpose |
|---------------|---------|
| **Device** | Identity and capability descriptor for one wearable device |
| **Measurement** | One observation (heart rate, step count, sleep stage, ECG, glucose, SpO2, blood pressure, …) |
| **Session** | A contiguous activity / sleep / monitoring session bundling many measurements |
| **Alert** | A clinically meaningful event derived from measurements |
| **Calibration** | A signed calibration record tying a device to a traceable reference |

Out of scope: HTTP surface (Phase 2), federation across vendors (Phase 3),
HealthKit / Google Fit / FHIR bridges (Phase 4).

---

## 2. Encoding Rules

* UTF-8 JSON per IETF RFC 8259, `snake_case` keys.
* Timestamps RFC 3339 in UTC, `Z` suffix; sub-second precision required
  for measurements at ≥ 1 Hz.
* Identifiers URI-shaped per IETF RFC 3986. Patient identifiers SHOULD
  use `did:wia:patient:…` to keep biometrics out of the wire.
* Measurement units use UCUM codes (`/min` for bpm, `mg/dL` for
  glucose, `mmHg` for blood pressure, etc.) per the Unified Code for
  Units of Measure standard.
* Numeric fields fit in IEEE 754 double precision.
* Binary payloads (raw ECG samples, accelerometer arrays) use base64url
  per IETF RFC 4648 §5.

### 2.1 Versioning

```json
"wia_wearable_health_version": "1.0.0"
```

A receiver MUST refuse a major version it does not implement and
MUST treat unknown minor-version optional fields as non-fatal.

---

## 3. Device Record

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "device",
  "device_id": "did:wia:device:apple-watch-S9-01HXY",
  "patient_id": "did:wia:patient:01HXY",
  "manufacturer": "Apple",
  "model": "Watch Series 9",
  "firmware_version": "watchOS 11.2",
  "registered_at": "2026-04-27T10:00:00Z",
  "clinical_grade": false,
  "fda_510k_number": null,
  "supported_measurements": ["heart_rate", "ecg", "spo2", "step_count", "sleep_stage"],
  "calibration_chain_id": null,
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

### 3.1 Required fields

`wia_wearable_health_version`, `type`, `device_id`, `patient_id`,
`manufacturer`, `model`, `firmware_version`, `registered_at`,
`clinical_grade`, `supported_measurements`, `signature`.

### 3.2 Clinical grade

`clinical_grade: true` MAY be set only when the device has regulatory
clearance for the relevant measurements (FDA 510(k), EU CE-MDR,
KFDA approval). When `true`, the `fda_510k_number` (or equivalent
`ce_mdr_number` / `kfda_number`) MUST be populated. Implementations
MUST refuse a device record claiming `clinical_grade: true` without
a valid clearance reference.

### 3.3 Calibration chain

`calibration_chain_id` references the most recent `calibration` record
for this device. Clinical-grade devices MUST maintain an unbroken
calibration chain to a national metrology institute reference (NIST,
PTB, KRISS, NMIJ, NIM).

---

## 4. Measurement Record

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "measurement",
  "measurement_id": "msr_01HXY",
  "device_id": "did:wia:device:apple-watch-S9-01HXY",
  "patient_id": "did:wia:patient:01HXY",
  "measurement_type": "heart_rate",
  "captured_at": "2026-04-27T10:00:00Z",
  "value": 72,
  "unit": "/min",
  "context": "rest",
  "clinical_grade": false,
  "raw_payload_url": null,
  "session_id": "ses_01HXY",
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

### 4.1 Reserved measurement types

| Type | Unit (UCUM) | Notes |
|------|-------------|-------|
| `heart_rate` | `/min` | Resting / active context required |
| `step_count` | `1` | Count over a window; window in `context` |
| `distance` | `m` | |
| `calories` | `kcal` | Activity-derived energy expenditure |
| `sleep_stage` | enum | `awake`, `rem`, `light`, `deep` |
| `spo2` | `%` | 0–100 |
| `ecg` | reference | Raw ECG sample stream — `raw_payload_url` MUST be set |
| `glucose` | `mg/dL` | Continuous glucose monitor |
| `sbp` | `mm[Hg]` | Systolic blood pressure |
| `dbp` | `mm[Hg]` | Diastolic blood pressure |
| `temp` | `Cel` | Body temperature in degrees Celsius |
| `weight` | `kg` | |

Implementations MUST treat unknown measurement types as `unrecognised`
rather than fail.

### 4.2 Raw payload reference

For high-rate streams (ECG, accelerometer waveform, raw PPG), the
`raw_payload_url` references an out-of-band binary blob with an
integrity hash. The wearable host MUST fetch and verify before
publishing the measurement to a clinical audience.

---

## 5. Session Record

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "session",
  "session_id": "ses_01HXY",
  "patient_id": "did:wia:patient:01HXY",
  "session_kind": "sleep",
  "started_at": "2026-04-26T22:00:00Z",
  "ended_at":   "2026-04-27T06:30:00Z",
  "device_ids": ["did:wia:device:apple-watch-S9-01HXY"],
  "measurement_ids": ["msr_01", "msr_02", "msr_03"],
  "summary": {
    "total_minutes": 510,
    "rem_minutes": 105,
    "deep_minutes": 95,
    "light_minutes": 290,
    "awake_minutes": 20
  },
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

`session_kind` is one of `activity`, `sleep`, `workout`, `monitoring`,
`fasting`, `recovery`. The summary block is optional but RECOMMENDED;
its shape varies per session kind.

---

## 6. Alert Record

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "alert",
  "alert_id": "alt_01HXY",
  "patient_id": "did:wia:patient:01HXY",
  "captured_at": "2026-04-27T10:30:00Z",
  "severity": "critical",
  "rule": "heart_rate above 130 bpm at rest for > 10 minutes",
  "evidence_measurement_ids": ["msr_01", "msr_02", "msr_03"],
  "actions_taken": ["notify_patient", "log_for_clinician"],
  "actions_pending": ["page_oncall_clinician"],
  "signature": { "alg": "Ed25519", "value": "Vr3w…" }
}
```

Severity ladder: `info`, `warning`, `urgent`, `critical`. Critical
alerts MUST be pushed to the patient and (if consent permits) the
on-call clinician within 60 seconds of detection.

---

## 7. Calibration Record

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "calibration",
  "calibration_id": "cal_01HXY",
  "device_id": "did:wia:device:cgm-G7-01HXY",
  "calibrated_at": "2026-04-27T08:00:00Z",
  "next_due_at":   "2026-05-27T08:00:00Z",
  "method": "fingerstick_reference",
  "reference_value": 120,
  "device_value": 118,
  "delta": -2,
  "traceability": "NIST",
  "previous_calibration_id": "cal_01HWZ",
  "signature": { "alg": "Ed25519", "value": "Mx7q…" }
}
```

Clinical-grade devices MUST publish a calibration record at least
monthly; consumer-grade devices SHOULD publish on first activation
and after any firmware update that affects measurement accuracy.

---

## 8. Schema Files

JSON Schema 2020-12 documents are served from
`https://wiastandards.com/wearable-health/schemas/`. Implementations
SHOULD bundle local copies for offline validation.

---

## 9. Conformance

A Phase 1 conformant implementation MUST:

1. Round-trip every object family byte-identically through encode/decode.
2. Reject objects missing required fields per the JSON Schemas.
3. Treat unknown optional fields as non-fatal.
4. Refuse a device record claiming `clinical_grade: true` without a
   valid regulatory-clearance reference.
5. Fetch and verify integrity for `raw_payload_url` references before
   publishing to a clinical audience.

---

## 10. References

* IETF RFC 8259 — JSON
* IETF RFC 3339 — Date/Time
* IETF RFC 3986 — URI Generic Syntax
* IETF RFC 4648 — base64
* IETF RFC 8032 — EdDSA / Ed25519
* HL7 FHIR R5 — Observation, Patient, Device resources
* IEEE 11073-104xx series — Personal health device profiles
* UCUM — Unified Code for Units of Measure
* JSON Schema Draft 2020-12

---

## Appendix A — Reserved Tokens

| Field | Reserved tokens |
|-------|-----------------|
| `severity` (alert) | `info`, `warning`, `urgent`, `critical` |
| `session_kind` | `activity`, `sleep`, `workout`, `monitoring`, `fasting`, `recovery` |
| `context` (measurement) | `rest`, `active`, `recovery`, `sleep` |
| `sleep_stage` | `awake`, `rem`, `light`, `deep` |
| `traceability` | `NIST`, `PTB`, `KRISS`, `NMIJ`, `NIM`, `BIPM` |
| `method` (calibration) | `fingerstick_reference`, `lab_assay`, `bench_test`, `manufacturer_factory` |

Future minor versions add tokens but never remove them.

## Appendix B — Worked Continuous Glucose Monitor Sequence

A Dexcom G7 sensor publishing measurements every 5 minutes. The
device record:

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "device",
  "device_id": "did:wia:device:cgm-G7-01HXY",
  "patient_id": "did:wia:patient:01HXY",
  "manufacturer": "Dexcom",
  "model": "G7",
  "firmware_version": "1.4.2",
  "registered_at": "2026-04-27T10:00:00Z",
  "clinical_grade": true,
  "fda_510k_number": "K212229",
  "supported_measurements": ["glucose"],
  "calibration_chain_id": "cal_01HXY",
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

A measurement at T+5 min:

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "measurement",
  "measurement_id": "msr_01HXY-T5",
  "device_id": "did:wia:device:cgm-G7-01HXY",
  "patient_id": "did:wia:patient:01HXY",
  "measurement_type": "glucose",
  "captured_at": "2026-04-27T10:05:00Z",
  "value": 118,
  "unit": "mg/dL",
  "context": "rest",
  "clinical_grade": true,
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

A critical alert when glucose drops below 54 mg/dL:

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "alert",
  "alert_id": "alt_01HXY-hypo",
  "patient_id": "did:wia:patient:01HXY",
  "captured_at": "2026-04-27T11:30:00Z",
  "severity": "critical",
  "rule": "glucose < 54 mg/dL (severe hypoglycaemia threshold per ADA Standards of Care)",
  "evidence_measurement_ids": ["msr_01HXY-T80", "msr_01HXY-T85", "msr_01HXY-T90"],
  "actions_taken": ["notify_patient", "log_for_clinician"],
  "actions_pending": ["page_oncall_endocrinologist"],
  "signature": { "alg": "Ed25519", "value": "Vr3w…" }
}
```

The alert references three consecutive 5-minute measurements as
evidence; this avoids triggering on a single sensor noise spike.
Per ADA Standards of Care, severe hypoglycaemia (under 54 mg/dL)
warrants immediate intervention regardless of trending direction;
the rule's evidence requirement is purely a noise filter, not a
delay mechanism.

## Appendix C — Worked Sleep Session

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "session",
  "session_id": "ses_01HXY-sleep-2026-04-26",
  "patient_id": "did:wia:patient:01HXY",
  "session_kind": "sleep",
  "started_at": "2026-04-26T22:30:00Z",
  "ended_at":   "2026-04-27T06:45:00Z",
  "device_ids": ["did:wia:device:apple-watch-S9-01HXY"],
  "measurement_ids": ["msr_sleep_01", "msr_sleep_02", "msr_sleep_03"],
  "summary": {
    "total_minutes": 495,
    "rem_minutes": 102,
    "deep_minutes": 88,
    "light_minutes": 285,
    "awake_minutes": 20,
    "average_heart_rate": 56,
    "minimum_heart_rate": 48,
    "average_spo2": 96
  },
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

## Appendix D — Worked Calibration Chain (clinical-grade scale)

A pharmacy weighing scale traceable to NIST through a reference
weight set. The current calibration:

```json
{
  "wia_wearable_health_version": "1.0.0",
  "type": "calibration",
  "calibration_id": "cal_pharm-scale-2026-04",
  "device_id": "did:wia:device:pharm-scale-A1",
  "calibrated_at": "2026-04-15T08:00:00Z",
  "next_due_at":   "2026-07-15T08:00:00Z",
  "method": "bench_test",
  "reference_value": 100.000,
  "device_value": 100.012,
  "delta": 0.012,
  "delta_unit": "g",
  "as_found_uncertainty_g": 0.020,
  "as_left_uncertainty_g": 0.005,
  "traceability": "NIST",
  "traceability_certificate_url": "https://traceability.example/calibration-certs/cal_pharm-scale-2026-04.pdf",
  "previous_calibration_id": "cal_pharm-scale-2026-01",
  "calibrator_id": "did:wia:calibrator:metrology-lab-A",
  "signature": { "alg": "Ed25519", "value": "Mx7q…" }
}
```

The chain links back through `previous_calibration_id` to every
prior calibration; auditors can walk the chain to the most recent
NIST traceability certificate without trusting any intermediate party.

弘益人間 — Benefit All Humanity.
