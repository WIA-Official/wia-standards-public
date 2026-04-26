# WIA-MED-003 Phase 4: Ecosystem Integration Standard

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-15

## Overview

Phase 4 defines how WIA-MED-003 integrates with existing healthcare ecosystems including EHR/EMR systems, telemedicine platforms, and cloud services.

## EHR/EMR Integration

### FHIR Resource Mapping

WIA-MED-003 data maps to FHIR Observation resources:

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
  "valueQuantity": {
    "value": 72,
    "unit": "beats/minute",
    "system": "http://unitsofmeasure.org",
    "code": "/min"
  }
}
```

### Supported EHR Systems

- **Epic:** FHIR R4 API
- **Cerner:** FHIR R4 API
- **Allscripts:** FHIR R4 API
- **athenahealth:** FHIR DSTU2/R4
- **eClinicalWorks:** FHIR R4

## Telemedicine Integration

### Real-time Monitoring During Video Consultation

Vital signs are streamed alongside video:

```javascript
// Twilio Video + WIA-MED-003
twilioRoom.localDataTrack.send(JSON.stringify({
  type: 'vital-sign',
  standard: 'WIA-MED-003',
  data: vitalSignData
}));
```

### Remote Patient Monitoring (RPM)

Continuous monitoring with cloud aggregation:
- Real-time data streaming to provider dashboard
- Automated alert generation
- Trend analysis and reporting
- Integration with care management platforms

## Wearable Device Integration

### Apple HealthKit

Bidirectional sync with iOS Health app:
- Write: Vital signs → HealthKit
- Read: HealthKit → WIA-MED-003 format

### Google Fit

Integration with Android health platform:
- Write: Vital signs → Google Fit
- Read: Google Fit → WIA-MED-003 format

## AI/ML Analytics Integration

### Anomaly Detection

Real-time analysis of vital signs:
- ECG arrhythmia detection
- SpO2 desaturation events
- Blood pressure abnormalities
- HRV stress assessment

### Predictive Analytics

Forecast future health trends:
- Blood pressure trend prediction
- Heart failure risk assessment
- Diabetes complication prediction
- Sleep apnea screening

## Cloud Storage Integration

### AWS HealthLake

FHIR-based data lake for healthcare:
```javascript
await healthlake.startFHIRImportJob({
  DatastoreId: 'datastore-id',
  InputDataConfig: { S3Uri: 's3://bucket/data' }
});
```

### Azure Health Data Services

Azure FHIR service integration:
```javascript
await azureFHIRClient.createObservation(vitalSignData);
```

### Google Cloud Healthcare API

Google's healthcare data platform:
```javascript
await healthcare.projects.locations.datasets.fhirStores
  .fhir.create({
    parent: fhirStoreName,
    type: 'Observation',
    requestBody: observation
  });
```

## Interoperability Standards

WIA-MED-003 is compatible with:

- **HL7 FHIR R4:** Health data exchange
- **DICOM:** Medical imaging (for waveforms)
- **IHE PCD-01:** Patient care device profiles
- **ISO/IEEE 11073:** Medical device communication
- **Continua Design Guidelines:** Personal health devices

## Integration Best Practices

1. **Data Quality:** Validate all data before transmission
2. **Error Handling:** Graceful degradation and retry logic
3. **Performance:** Batch operations when possible
4. **Security:** Always encrypt PHI/PII data
5. **Compliance:** Follow all applicable regulations
6. **Monitoring:** Track integration health and errors
7. **Documentation:** Maintain API integration guides

---

## WIA Family Integration

### WIA-OMNI-API

Clinician identity and credentials (medical licence, role assignments,
training records) live in WIA-OMNI-API. Hosts fetch them by DID rather
than holding raw documents.

```
GET https://omni.example/credential/did:wia:clinician:09…/licence
Authorization: WIA-Sig …
→ 200 { "issuer":"BC College of Physicians", "valid_until":"2027-06-30" }
```

### WIA-ACCESSIBILITY

Clinician and patient accessibility profiles drive UI choices: large-text
mode, high-contrast waveform rendering, audio-cued alerts for visually
impaired clinicians, simplified summary view for cognitively impaired
patients reviewing their own monitoring history.

### WIA-AIR-SHIELD

Transport hardening (TLS configuration, peer reputation). Hosts MAY
refuse subscription requests from peers whose AIR-SHIELD score is
below a host-set threshold; the threshold is published in the
discovery document.

### WIA-INTENT

A clinician's "show me last hour for patient 01HXY" intent is lowered
to:

1. Fetch the patient consent envelope.
2. Resolve the active stream(s) for the patient.
3. Issue a Phase 2 replay request scoped to the requested window.
4. Render the waveform under the clinician's WIA-ACCESSIBILITY
   profile.

### WIA-SOCIAL

Clinical milestones (de-identified) MAY be published via WIA-SOCIAL
bridges with `audience=internal` for departmental announcements
without leaving the EHR perimeter.

## Telemedicine + EHR Worked Example

```
Patient    : did:wia:patient:01HXY
ICU host   : did:wia:operator:icu-A   (producer)
EHR bridge : did:wia:bridge:epic-eu   (consumer)
Clinician  : did:wia:clinician:09…    (consumer)
Custodian  : did:wia:custodian:hospital-A
```

1. Custodian publishes patient consent envelope (Phase 3 §Patient
   Consent) with `scope = ["icu", "remote_monitoring"]`.
2. Clinician's WIA-INTENT lowering issues subscribe to
   `/vss/stream/stream-001`.
3. ICU host verifies clinician's federation receipt + consent →
   accepts SSE subscription.
4. Frames flow at 250 Hz for ECG; Phase 4 EHR bridge concurrently
   subscribes and writes per-minute aggregates as FHIR Observation
   resources to Epic.
5. Patient transfers to general ward; consent scope updated to
   `["general_ward", "remote_monitoring"]`.
6. ICU host emits a `notice` envelope; consumers resubscribe with
   the new scope.

## Migration Paths

### From a Vendor-Locked Bedside Monitor

A hospital migrating from a single-vendor bedside monitor to
WIA-Vital-Sign-Streaming follows:

1. Deploy a WIA host alongside the existing monitor.
2. Run a vendor-bridge adapter that reads the monitor's serial /
   network protocol and emits WIA frames.
3. Run a 14-day shadow period with both surfaces active. Compare
   WIA-derived alerts against vendor-derived alerts daily; any
   divergence > 0 must be explained before retiring the vendor stack.
4. After three consecutive zero-divergence days, switch alerting to
   WIA and demote vendor monitor to backup.

### Between Hospitals

A patient transfer from hospital A to hospital B:

1. A signs a `move` envelope citing the active stream id.
2. B's host fetches the most recent channel descriptor and replays
   the prior 60 minutes from A.
3. B opens its own subscriber stream; A continues to source frames
   for 30 minutes after handover for clinical continuity, then
   closes the source.
4. A's host returns `301 Moved Permanently` for the prior stream
   URL for at least 12 months for audit purposes.

## Observability

A conformant host SHOULD expose:

| Metric | Type | Description |
|--------|------|-------------|
| `wia_vss_frames_received_total{channel}` | counter | Inbound frames |
| `wia_vss_subscribers{stream_id}` | gauge | Active subscribers |
| `wia_vss_alert_emitted_total{level}` | counter | Alerts by level |
| `wia_vss_consent_violations_total` | counter | Refused subscriptions |
| `wia_vss_replay_seconds_served_total` | counter | Replay byte-load |

Labels MUST NOT include patient identifiers. Operators that need
per-patient debugging SHOULD use the per-peer 30-day problem-detail
retention rather than baking identities into telemetry.

## Conformance Profiles

| Level | Required integrations |
|-------|-----------------------|
| **Minimal** | FHIR Observation export, WIA-OMNI-API credential fetch |
| **Core**    | Plus WIA-ACCESSIBILITY enforcement, telemedicine bridge for ≥1 vendor |
| **Full**    | Plus WIA-AIR-SHIELD scoring, WIA-INTENT lowering, AI/ML anomaly bridge, AWS HealthLake/Azure/GCP export |

Hosts publish their level in `bridge_profile` of the discovery
document.

## Security Considerations

* Vendor adapters hold device-side credentials; operators MUST use
  WIA-OMNI-API for credential storage and HSM-backed refresh tokens.
* Streaming endpoints carry PHI; hosts MUST treat them as the highest
  audit priority and MUST page on-call when authentication failures
  spike beyond a baseline threshold.
* AI/ML anomaly bridges receive de-identified or pseudo-identified
  data per the patient's consent envelope; bridges MUST refuse data
  whose audience does not permit the planned downstream use (e.g.
  research training set).

## Appendix A — References

* HL7 FHIR R5 — Observation, Patient, Consent resources
* IEEE 11073 — Personal health device communication
* DICOM Part 15 — Security and System Management
* IHE PCD-01 — Patient care device profile
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* HIPAA Safe Harbor §164.514(b)(2)
* WIA-OMNI-API standard
* WIA-AIR-SHIELD standard
* WIA-INTENT standard
* WIA-ACCESSIBILITY standard
* WIA-SOCIAL standard

---

## Appendix B — Bridge Configuration File

A reference Epic FHIR bridge configuration uses TOML:

```toml
[bridge]
ehr = "epic"
fhir_version = "R5"
operator_id = "did:wia:operator:icu-A"

[credentials]
provider = "wia-omni-api"
omni_endpoint = "https://omni.example"

[mapping]
channel_to_loinc = { ecg = "131329", spo2 = "59408-5", hr = "8867-4", sbp = "8480-6", dbp = "8462-4", temp = "8310-5", rr = "9279-1" }

[delivery]
aggregate_window_seconds = 60
flush_interval_seconds = 30
backoff = "exponential"
initial_delay_ms = 1000
max_delay_ms = 60000

[telemetry]
prometheus_port = 9091
```

Operators MAY embed vendor-specific sections; conformant bridges MUST
ignore unknown sections rather than refuse to start.

## Appendix C — Conformance Test Coverage by Profile

| Capability | Minimal | Core | Full |
|------------|---------|------|------|
| FHIR R5 Observation export | ✓ | ✓ | ✓ |
| WIA-OMNI-API credential fetch | ✓ | ✓ | ✓ |
| WIA-ACCESSIBILITY enforcement | — | ✓ | ✓ |
| Telemedicine vendor bridge | — | ✓ | ✓ |
| WIA-AIR-SHIELD scoring | — | — | ✓ |
| WIA-INTENT lowering | — | — | ✓ |
| AI/ML anomaly bridge | — | — | ✓ |
| AWS HealthLake / Azure / GCP export | — | — | ✓ |
| HealthKit / Google Fit bidirectional sync | optional | optional | ✓ |
| Vendor adapter for ≥1 monitor | optional | ✓ | ✓ |

A host publishing `bridge_profile=Full` MUST pass every line in the
Full column and SHOULD pass at least one optional line.

## Appendix D — LOINC Mapping for Common Channels

The default LOINC mapping that bridges SHOULD apply when emitting
FHIR Observation resources:

| WIA channel | LOINC code | Display |
|-------------|-----------|---------|
| `ecg` (waveform) | 131329 | ECG study |
| `hr`  | 8867-4   | Heart rate |
| `spo2`| 59408-5  | Oxygen saturation in arterial blood by pulse oximetry |
| `sbp` | 8480-6   | Systolic blood pressure |
| `dbp` | 8462-4   | Diastolic blood pressure |
| `map` | 8478-0   | Mean blood pressure |
| `temp`| 8310-5   | Body temperature |
| `rr`  | 9279-1   | Respiratory rate |
| `etco2`| 19889-5 | End tidal carbon dioxide |

For channels not in the default mapping, bridges SHOULD fall back to a
SNOMED CT code from the IPS (International Patient Summary) value
set; the discovery document MAY override the default mapping with a
`channel_loinc` table.

## Appendix E — Worked AI/ML Bridge

A research-tier AI bridge subscribed under the `research` audience:

```
T-1d    custodian publishes patient_consent (scope=research)
T+0     bridge: GET /vss/stream/stream-001 with Audience: research
T+1s    host verifies consent + audience scope; SSE opens with
        de-identified frames (patient_id replaced by k-anon group id)
T+1m    bridge appends frame batch to model training queue
T+1d    bridge emits monthly research summary back to host (aggregate
        only, no per-patient outputs); host appends to audit log
```

If the consent envelope expires, the host emits a `notice` envelope
and the bridge MUST close its SSE; any cached frames in the bridge
queue MUST be redacted within the consent's `redaction_window`
(default 24 h).

---

**Copyright 2025 WIA / SmileStory Inc.**
**License:** MIT
**弘益人間 · Benefit All Humanity**
