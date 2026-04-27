# WIA-MED-020 Wearable Health — Phase 4: Integration

**Standard**: WIA-MED-020 Wearable Health Monitoring
**Phase**: 4 of 4 — Integration
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 4 specifies how WIA-MED-020 integrates with three classes of
existing system:

1. **Vendor health platforms** — Apple HealthKit, Google Fit, Samsung
   Health, Garmin Connect, Fitbit Web API.
2. **Clinical interoperability standards** — HL7 FHIR R5, IEEE 11073,
   IHE PCD-01.
3. **WIA family standards** — WIA-OMNI-API for credentials,
   WIA-AIR-SHIELD for transport hardening, WIA-INTENT for clinician
   intent lowering, WIA-ACCESSIBILITY for patient and clinician
   accommodations, WIA Vital Sign Streaming for high-rate continuous
   waveforms.

The aim is that one WIA-MED-020 host is the canonical record while
every vendor / EHR / research system reads from or writes to it
through bridges, never holding canonical state for the patient's
wearable data.

---

## 2. HL7 FHIR R5 Bridge

### 2.1 Observation Mapping

Each WIA `measurement` envelope maps to an HL7 FHIR R5 `Observation`
resource:

```json
{
  "resourceType": "Observation",
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/observation-category",
      "code": "vital-signs"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "8867-4",
      "display": "Heart rate"
    }]
  },
  "subject": { "reference": "Patient/did:wia:patient:01HXY" },
  "effectiveDateTime": "2026-04-27T10:00:00Z",
  "valueQuantity": {
    "value": 72,
    "unit": "/min",
    "system": "http://unitsofmeasure.org",
    "code": "/min"
  },
  "device": { "reference": "Device/did:wia:device:apple-watch-S9-01HXY" },
  "derivedFrom": [{ "reference": "DocumentReference/wia-mh/msr_01HXY" }]
}
```

### 2.2 Default LOINC Mapping

| WIA measurement_type | LOINC | Display |
|----------------------|-------|---------|
| `heart_rate`  | 8867-4   | Heart rate |
| `step_count`  | 55423-8  | Steps in 24 hours measured |
| `spo2`        | 59408-5  | Oxygen saturation in arterial blood by pulse oximetry |
| `glucose`     | 2339-0   | Glucose Mass/volume in Blood |
| `sbp`         | 8480-6   | Systolic blood pressure |
| `dbp`         | 8462-4   | Diastolic blood pressure |
| `temp`        | 8310-5   | Body temperature |
| `weight`      | 29463-7  | Body weight |
| `sleep_stage` | 93832-4  | Sleep duration |

For ECG raw waveforms, the bridge uses HL7 FHIR `MediaStream` plus a
`derivedFrom` link to a binary representation conforming to the WIA
Vital Sign Streaming standard.

---

## 3. IEEE 11073 Bridge

IEEE 11073-104xx defines per-device profiles for personal health
device communication (Bluetooth GATT, NFC, USB). The WIA bridge:

* Subscribes to the device's IEEE 11073 measurement channel.
* Translates each measurement into the WIA `measurement` envelope.
* Preserves the IEEE 11073 device specialisation code in the WIA
  `device.metadata` field so clinical systems can verify the source
  device class.

| IEEE 11073 specialisation | WIA measurement_type |
|---------------------------|----------------------|
| -10408 Thermometer        | `temp` |
| -10415 Weighing scale     | `weight` |
| -10417 Glucose meter      | `glucose` |
| -10421 Blood pressure monitor | `sbp` + `dbp` |
| -10441 Cardiovascular fitness activity | `heart_rate` + `step_count` |

---

## 4. Vendor Platform Bridges

### 4.1 Apple HealthKit

The HealthKit bridge runs as part of an iOS companion app:

* Reads HealthKit data via `HKObjectTypeIdentifier` queries with the
  user's explicit consent under iOS's per-type permission model.
* Translates each sample into the WIA `measurement` envelope.
* Writes WIA-host-derived measurements back to HealthKit under the
  app's own source identifier (so Apple Health UI shows
  "WIA Wearable Health" as the data source).

### 4.2 Google Fit

The Google Fit bridge runs as part of an Android companion app:

* Subscribes to `DataType` streams via the Fit REST API with the
  user's OAuth consent.
* Translates samples into the WIA envelope.
* Writes back via the bulk insert endpoint under the app's data
  source.

### 4.3 Samsung Health, Garmin, Fitbit

Vendor-specific bridge profiles live under
`spec/profiles/<vendor>.md`. Each profile documents OAuth scopes,
field-by-field mapping, polling cadence, and credential management.

---

## 5. WIA Family Integration

### 5.1 WIA-OMNI-API

Patient identity, vendor account credentials, and provider licence
records live in WIA-OMNI-API. WIA-MED-020 hosts fetch them by DID
rather than holding raw documents. Paediatric custodian relationships
also live in WIA-OMNI-API as signed `guardian_link` claims.

### 5.2 WIA-AIR-SHIELD

Transport hardening (TLS configuration, peer reputation). Hosts MAY
refuse subscription requests from peers whose AIR-SHIELD score is below
a host-set threshold; the threshold is published in the discovery
document.

### 5.3 WIA-INTENT

A clinician's "show me last week of glucose for patient 01HXY" intent
is lowered to:

1. Verify clinician's licence via WIA-OMNI-API.
2. Fetch the patient's consent envelope; verify clinician audience.
3. Issue a WIA-MED-020 measurement query with type=glucose and the
   requested time window.
4. Render the response under the clinician's WIA-ACCESSIBILITY profile.

### 5.4 WIA-ACCESSIBILITY

Patient-facing UI surfaces accommodate accessibility profiles
(large-text mode for elderly patients, high-contrast for low-vision,
voice output for motor-impaired). The patient profile drives chart
rendering, alert presentation, and consent UI.

### 5.5 WIA Vital Sign Streaming

For high-rate continuous waveforms (ECG, full PPG), WIA-MED-020 emits
a `raw_payload_url` pointing to a WIA Vital Sign Streaming resource.
The two standards compose: WIA-MED-020 provides the discrete
measurement framing while WIA Vital Sign Streaming provides the
continuous waveform stream.

---

## 6. Compliance Mappings

### 6.1 HIPAA

WIA-MED-020 carries PHI; hosts MUST implement HIPAA Security Rule
controls (administrative safeguards, physical safeguards, technical
safeguards including audit controls and encryption at rest).

### 6.2 GDPR

The patient consent envelope (Phase 3 §4) is the GDPR Article 7
consent record. The patient's right to revocation (GDPR Article 7(3))
is implemented via the `revocable: true` field; revocation triggers
data destruction across audiences within the scope's
`redaction_window` (default 24 hours).

### 6.3 FDA 510(k)

For clinical-grade devices, the `fda_510k_number` field on the
device record is the binding tie to the regulatory clearance.
Vendor-specific 510(k) databases are accessible via the FDA's public
databases; the bridge MAY pre-validate the number's format before
accepting the device record.

---

## 7. Migration Paths

### 7.1 From a vendor-locked stack

A patient currently storing data only in Apple HealthKit migrates by:

1. Installing the WIA-MED-020 companion iOS app.
2. Granting HealthKit per-type access.
3. App runs an initial historical backfill (typically 2 years of data).
4. App registers each connected device under the patient's WIA identity.
5. Subsequent data flows in real time to the user's chosen WIA host.

### 7.2 Between WIA hosts

Phase 3 §7 `patient_move` envelope handles cross-host migration with
12-month `301` from the old host.

---

## 8. Observability

A conformant host SHOULD expose:

| Metric | Type | Description |
|--------|------|-------------|
| `wia_wh_devices_registered_total{manufacturer}` | counter | per-vendor device registrations |
| `wia_wh_measurements_received_total{type}` | counter | per-type measurement volume |
| `wia_wh_alerts_total{severity}` | counter | per-severity alert counts |
| `wia_wh_consent_violations_total` | counter | refused cross-host reads |
| `wia_wh_calibration_overdue` | gauge | clinical-grade devices overdue today |

Labels MUST NOT include patient identifiers.

---

## 9. Conformance Profiles

| Level | Required integrations |
|-------|-----------------------|
| **Minimal** | FHIR R5 Observation export, WIA-OMNI-API credential fetch |
| **Core**    | Plus IEEE 11073 bridge for ≥1 device class, WIA-ACCESSIBILITY enforcement |
| **Full**    | Plus WIA-AIR-SHIELD scoring, WIA-INTENT lowering, HealthKit + Google Fit bidirectional, WIA Vital Sign Streaming for ECG |

Hosts publish their level in `bridge_profile` of the discovery document.

---

## 10. Worked Example — Continuous Glucose Monitoring

```
Patient   : did:wia:patient:01HXY
Device    : did:wia:device:cgm-G7-01HXY (FDA 510(k) clinical-grade)
Vendor    : Dexcom (FDA 510(k) K212229)
Clinic    : did:wia:clinical-host:endocrine-clinic-A
```

1. Patient consents (clinical_dashboard scope, 1-year validity) for
   the endocrine clinic.
2. Vendor cloud sends measurements at 5-minute intervals via Phase 2
   sync endpoint.
3. Clinic subscribes to the patient's measurement stream via Phase 3
   federation (consent verified each time).
4. When glucose drops below 54 mg/dL, vendor cloud emits a critical
   alert; on-call endocrinologist is paged within 60 seconds.
5. Patient's calibration record is updated weekly; an out-of-tolerance
   reading triggers a re-calibration prompt and a clinician
   notification.
6. EHR bridge writes per-day glucose summary to the clinic's Epic
   instance as FHIR Observation resources.

---

## 11. Security Considerations

* Vendor bridges hold OAuth tokens for the vendor platforms;
  operators MUST use WIA-OMNI-API for credential storage and
  HSM-backed refresh tokens.
* Continuous glucose monitor data is high-stakes (a missed critical
  alert can be fatal); hosts MUST treat alert paths as the highest
  audit priority and MUST page on-call when alert latency exceeds a
  baseline threshold.
* Research aggregators receive de-identified data per the patient's
  research consent envelope; aggregators MUST refuse data whose
  audience does not permit the planned downstream use.

---

## 12. References

* HL7 FHIR R5 — Observation, Patient, Device, Consent resources
* IEEE 11073-104xx series — Personal health device profiles
* IHE PCD-01 — Patient care device profile
* HIPAA Security Rule
* GDPR Article 7
* FDA 510(k) regulatory pathway
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* WIA-OMNI-API standard
* WIA-AIR-SHIELD standard
* WIA-INTENT standard
* WIA-ACCESSIBILITY standard
* WIA Vital Sign Streaming standard

---

## Appendix A — Bridge Configuration File

A reference Apple HealthKit bridge configuration uses TOML:

```toml
[bridge]
vendor = "apple-healthkit"
ios_min_version = "17.0"
host_id = "did:wia:wearable-host:hospital-rpm"

[credentials]
provider = "wia-omni-api"
omni_endpoint = "https://omni.example"

[mapping]
# HealthKit type → WIA measurement_type
heart_rate = "HKQuantityTypeIdentifierHeartRate"
step_count = "HKQuantityTypeIdentifierStepCount"
spo2       = "HKQuantityTypeIdentifierOxygenSaturation"
ecg        = "HKDataTypeIdentifierElectrocardiogram"
glucose    = "HKQuantityTypeIdentifierBloodGlucose"
sbp        = "HKQuantityTypeIdentifierBloodPressureSystolic"
dbp        = "HKQuantityTypeIdentifierBloodPressureDiastolic"
temp       = "HKQuantityTypeIdentifierBodyTemperature"
weight     = "HKQuantityTypeIdentifierBodyMass"

[delivery]
batch_size = 100
sync_interval_seconds = 300
backoff = "exponential"
initial_delay_ms = 1000
max_delay_ms = 60000

[telemetry]
prometheus_port = 9091
```

Operators MAY embed vendor-specific sections; conformant bridges MUST
ignore unknown sections rather than refuse to start.

## Appendix B — Conformance Test Coverage by Profile

| Capability | Minimal | Core | Full |
|------------|---------|------|------|
| FHIR R5 Observation export | ✓ | ✓ | ✓ |
| WIA-OMNI-API credential fetch | ✓ | ✓ | ✓ |
| IEEE 11073 bridge for ≥1 device class | — | ✓ | ✓ |
| WIA-ACCESSIBILITY enforcement | — | ✓ | ✓ |
| Vendor bridge for ≥1 platform (HealthKit / Google Fit / Samsung / Garmin / Fitbit) | — | ✓ | ✓ |
| WIA-AIR-SHIELD scoring | — | — | ✓ |
| WIA-INTENT clinician intent lowering | — | — | ✓ |
| WIA Vital Sign Streaming for ECG | — | — | ✓ |
| Bidirectional sync (write back to vendor) | — | — | ✓ |
| Paediatric custodian flow | optional | optional | ✓ |
| Research aggregator with k-anonymity ≥ 5 | optional | optional | ✓ |

A host publishing `bridge_profile=Full` MUST pass every line in the
Full column and SHOULD pass at least one optional line.

弘益人間 — Benefit All Humanity.
