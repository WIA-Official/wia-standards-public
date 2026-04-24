# WIA-MED-BE-001 · Bionic Eye — Phase 4: Integration

**Version:** 1.1.0
**Status:** Active
**Last Updated:** 2026-04-25
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 4 specifies how a WIA-compliant visual prosthesis **integrates with the clinical, rehabilitation, regulatory, and daily-life ecosystems**: hospital EHRs (HL7 FHIR R5), ophthalmology imaging (DICOM OP/OCT/OPT), surgical scheduling, low-vision rehabilitation, home-health accessibility hubs, adverse-event reporting, and the hardware emergency-stop chain.

The philosophy is clear: the implant is never a standalone device — it lives in a patient's eye, a clinician's workflow, and a regulator's record. Phase 4 makes each of those attachment points well-defined.

### 1.1 Normative references

- HL7 FHIR R5 — `Device`, `DeviceMetric`, `Procedure`, `Observation`, `AdverseEvent`, `Communication`.
- DICOM PS3.3 — Ophthalmic Visual Field, OP (Ophthalmic Photography), OPT (OCT).
- ICD-10 H35.5 (retinal dystrophies), H54 (low vision).
- IEC 60601-1-8 — Alarm systems in medical equipment.
- IEC 62304 — Medical device software lifecycle.
- ISO 14971 — Risk management.
- FDA 21 CFR 803 (Medical Device Reporting); EU MDR Annex IX — Post-market surveillance.
- WHO ICF — International Classification of Functioning, Disability and Health.

---

## 2. Clinical-workflow touchpoints

### 2.1 Pre-op workflow

1. **Diagnosis**: DICOM OP fundus + OPT OCT + full-field ERG confirms candidacy (e.g., RP, advanced AMD).
2. **Candidacy grading**: `Observation` resource with LOINC `87615-1` documenting BCVA, visual field, dark-adaptation.
3. **Informed consent**: `Consent` resource per HL7 Consent Directive; digitally signed (ES256); versioned and re-confirmed ≤ 30 days before surgery.
4. **Surgical plan**: `PlanDefinition` links device model, electrode array, intra-op imaging protocol, and the anaesthesia team.

### 2.2 Intra-op workflow

- Intra-op OCT images are stored with private DICOM tag `(0071, xxxx)` `WIA Bionic Eye Intra-op Protocol`.
- Electrode array anchoring uses pre-registered fiducials; misalignment > 200 μm is logged to `Observation.note`.
- First impedance sweep captured immediately after skin closure and forms the electrode array *baseline* for Phase 3 § 2.3.

### 2.3 Post-op and fitting

- Fitting appointments scheduled via `Appointment` resource with `serviceType = "visual-prosthesis-fitting"`.
- Each fitting commit (Phase 2 § 4) emits `Procedure` (code `91873-9`) + aggregated `Observation`s (threshold, impedance).
- Percept calibration data is exported to the low-vision therapist's HL7 `DocumentReference`.

### 2.4 Quality-of-life follow-up

Validated instruments:
- Functional Low-Vision Observer Rated Assessment (FLORA).
- Visual Function Questionnaire 25 (VFQ-25).
- Orientation & Mobility Structured Assessment.

Results captured as `QuestionnaireResponse` resources; trend dashboards feed back into preset tuning (Phase 2 § 7).

---

## 3. Emergency stop (hardware + software)

The emergency stop chain is cross-cutting: triggers in Phase 3 link-layer, responses in Phase 2 patient app, and outcomes in this phase's alarm and reporting pipelines.

### 3.1 Automatic triggers

```ts
interface AutoTriggers {
  over_current:         { threshold_uA: number, total_mA: number, response_us: < 100 };
  over_voltage:         { threshold_V: number, response_us: < 100 };
  impedance_anomaly:    { sudden_change_pct: number, open_kOhm: number,
                          short_kOhm: number, response_ms: < 10 };
  thermal_overload:     { delta_C_above_baseline: 1, absolute_C: 42, response_ms: < 50 };
  charge_imbalance:     { dc_leakage_nA: 100, window_ms: 1000 };
  comms_loss:           { timeout_ms: 300 };
  sar_exceeded:         { threshold_W_per_kg: 1.6, window_s: 60 };
  fw_signature_invalid: { lockdown_ms: 300000 };
}
```

### 3.2 Manual triggers

```ts
interface ManualTriggers {
  patient_button:   { location: "frame"|"pendant"|"wristband"|"remote",
                      hold_ms?: 0, confirm?: false };
  voice_command:    { keywords: ["stop","emergency","멈춰","정지","parar"],
                      sensitivity: 0.6, confirm: true };
  caregiver_remote: { range_m: 10, auth: "paired_passkey" };
  app_control:      { auth: "pin" | "biometric" | "both" };
  clinician_cpi:    { requires_scope: "wia.be.safety.override" };
}
```

### 3.3 Three-phase stop sequence

| Phase      | Deadline  | Actions                                                     |
|------------|-----------|-------------------------------------------------------------|
| Immediate  | < 1 ms    | Disable all outputs; short-circuit electrodes; discharge caps|
| Fast       | < 100 ms  | Snapshot last parameters; persist to flash; notify processor |
| Follow-up  | < 1 s     | Alert patient (vibrate/beep/voice); alert caregiver; alert clinic on-call; upload event log |

The timing budget is enforced by the implant's watchdog; missed deadlines latch the device into `safe_state` until a clinician re-enables it.

### 3.4 Safe state

```ts
interface SafeState {
  all_electrodes_disabled: true;
  output_voltage_V: 0;
  capacitor_residual_V: 0;
  dc_leakage_nA: < 10;
  indicators: {
    patient_led: "red_blinking";
    processor_led: "red_solid";
    audio_alert: true;
  };
  communication: "active";     // e-stop does NOT kill telemetry
  autonomy: {
    restart_allowed_automatically: false;
    requires_clinician_clearance: true
  };
}
```

### 3.5 Event logging

Every e-stop emits:

```json
{ "event_id": "01JCG…",
  "timestamp": "2026-04-25T14:30:00.128Z",
  "trigger": {"type":"thermal_overload", "source":"automatic",
              "value_C": 1.4, "threshold_C": 1.0},
  "context": {"active_electrodes":[…], "impedance_kOhm":[…],
              "coil_k":0.14, "session_minutes":38},
  "response": {"stop_latency_us": 640, "phases_completed":["immediate","fast","followup"],
               "alerts_sent":["patient.vibrate","caregiver.app","clinic.on-call"]}
}
```

---

## 4. Restart protocol

### 4.1 Policies per trigger class

| Trigger class           | Auto-restart allowed | Cooldown | Clinician clearance |
|-------------------------|----------------------|----------|---------------------|
| over_current / voltage  | No                   | —        | Required            |
| impedance_anomaly       | No                   | —        | Required            |
| thermal_overload        | After ΔT < 0.5 °C    | 300 s    | Not required        |
| charge_imbalance        | No                   | —        | Required            |
| comms_loss              | Yes                  | 10 s     | Not required        |
| patient_button          | Yes                  | 60 s     | Not required        |
| sar_exceeded            | Yes (duty-cycled)    | 60 s     | Not required        |
| fw_signature_invalid    | No (quarantine)      | —        | Required + manuf.   |

### 4.2 Required checks before re-enabling

- Impedance sweep within normal band for ≥ 95 % of active electrodes.
- DC leakage ≤ 10 nA averaged over 1 s.
- Thermistor within 0.2 °C of pre-session baseline.
- Patient confirmation (PAI or caregiver passkey).
- Audit log acknowledged on the CPI.

---

## 5. Adverse-event reporting

### 5.1 FDA MDR + EU MDR pipelines

Every severity-≥ 2 event is pushed to the manufacturer's PMS system within 24 h. Canonical fields:

- Device ID (serial), firmware version, hardware revision.
- Trigger class + value + threshold.
- Patient demographics (de-identified).
- Outcome: resolved / hospitalised / device-removed / death.
- Root-cause hypothesis + containment plan.

Submissions use the FDA eSubmitter MDR format (US) and the EUDAMED Vigilance module (EU); the WIA standard guarantees the internal record is already structured so both conversions are deterministic.

### 5.2 FHIR AdverseEvent

```json
{
  "resourceType": "AdverseEvent",
  "status": "completed", "actuality": "actual",
  "subject": { "reference": "Patient/pid-0042" },
  "date":   "2026-04-25T14:30:00Z",
  "event": { "coding": [{ "system":"http://snomed.info/sct",
                          "code":"417293001",
                          "display":"Device malfunction – implantable"}]},
  "seriousness": { "coding": [{"code":"non-serious"}]},
  "suspectEntity": [{
     "instance": { "reference":"Device/dev-001" },
     "causality": [{"assessment":{"text":"Probable thermal overload"}}]}]
}
```

---

## 6. Low-vision rehabilitation loop

### 6.1 Assistive pairing

Visual prostheses do not replace other assistive modalities — they augment them. WIA devices MUST interoperate with:

- **Braille displays** (USB-HID, Bluetooth) for OCR text results.
- **Screen readers** (NVDA, JAWS, VoiceOver, TalkBack) — the patient app exposes a standards-compliant ARIA tree.
- **Canes and GPS** — obstacle priorities from Phase 1 § 4.2 are routed to haptic cane hubs.
- **Hearing aids / cochlear implants** — binaural cues synchronised to phosphene priority for audio-visual fusion.

### 6.2 Therapist workflow

A low-vision therapist can load a session summary (`/devices/{id}/fittings/{fid}/log`) into their case file, run structured O&M (orientation & mobility) exercises, and feed graded results back as `QuestionnaireResponse` + `Observation` resources that the CPI uses to retune modes.

### 6.3 Training scenes

Reference training scenarios are bundled and versioned:

- *Doorway traversal* — high-contrast door frames + static obstacles.
- *Curb descent* — simulated kerb at 1.2 m, IMU-validated gait check.
- *Face localisation* — moving face at 1.5–2.5 m with random head pose.
- *Sign reading* — low-/high-contrast signage, different font sizes.

---

## 7. Smart-home and accessibility integrations

### 7.1 Matter / Thread

Emergency stop, ambient-light coupling, and scene presets are exposed as Matter accessory clusters:

- Cluster `0x0500 Visual Prosthesis` (WIA-reserved) with attributes `ActiveMode`, `IntensityScale`, `PatientPresence`, `SafeStateActive`.
- Command `EmergencyStop` — a one-shot command that all Matter fabrics route with `priority=URGENT`.

### 7.2 HomeKit / Google Home / Alexa

A bridge exposes a minimal voice surface:

- "Stop visual aid" → e-stop.
- "Reading mode" → scene preset.
- "Set prosthesis intensity 40 percent" → `intensity` scale.

Voice flows MUST NOT allow restart without the patient's passkey — the microphone path is advisory, not privileged.

---

## 8. Data lifecycle with research and industry

### 8.1 Anonymisation

De-identified trial sets leave the institution only via the RTI surface, with PHI stripped and `patient_id` replaced by a study-specific `subject_id`. Export events are logged with IRB number, destination domain, and purpose.

### 8.2 Federation and consent

Multi-centre trials use federated learning: raw cycles stay in each hospital; only gradient updates cross institutional boundaries. Each site's Consent resource gates participation; withdrawal propagates via FHIR Subscription.

### 8.3 Long-term archive

Clinical fitting records, psychophysics, and adverse events are archived for the longer of: regulator minimum (US 7 y, EU 10 y) or the device's expected service life +5 y. Archives are WORM (write-once, read-many) with per-year integrity hashes anchored on the WIA Chain.

---

## 9. Interoperability with related WIA standards

- **WIA-MED-001 Medical Data** — patient demographics, encounter records.
- **WIA-MED-BCI-001 BCI** — cortical prosthesis protocol overlap (`cortical-60e`).
- **WIA-MED-ASSIST-001 Assistive Wearable** — cane / haptic hub fusion.
- **WIA-MED-AAC-001 AAC + Cognitive AAC** — OCR output → AAC pipeline for patients with combined disabilities.
- **WIA-DATA-001 Big Data** — RTI bulk export uses Arrow Flight SQL (Phase 2 § 8).

---

## 10. Worked scenario

Patient Kim (RP, epiretinal 60-electrode implant) enters an unfamiliar room:

1. Camera captures an approaching colleague (face, confidence 0.93).
2. Processed visual tags the face as `high` priority; saliency bump.
3. Phosphene map projects onto fovea-adjacent electrodes 12, 15, 18, 21.
4. Stimulation driver commands bursts (Phase 1 § 6) at Shannon-safe amplitudes.
5. Implant reports `TELEMETRY` (Phase 3 § 3.3) via inductive link with RSSI −41 dBm, coil-k 0.18.
6. Patient recognises the colleague's gait + outline; CPI logs a successful object recognition event.
7. Ten seconds later, tissue temperature rises 1.1 °C → thermal e-stop fires.
8. Three-phase stop completes in 340 μs; pendant vibrates; caregiver app receives notice.
9. Restart is allowed after 5-minute cooldown; impedance sweep passes; patient taps PAI confirm; session resumes.
10. FHIR AdverseEvent is *not* emitted (thermal-overload is expected and within design); instead, a `DeviceMetric` trend is stored for the clinician's review.

---

## 11. Compliance checklist

- [ ] FHIR Device / Procedure / Observation / AdverseEvent resources emitted.
- [ ] DICOM OP/OPT tags carry WIA Bionic Eye intra-op identifiers.
- [ ] Hardware e-stop meets the three-phase timing budget (§ 3.3).
- [ ] Event log non-repudiable; periodically WIA-Chain-anchored.
- [ ] Adverse-event pipeline reaches FDA eMDR / EUDAMED Vigilance.
- [ ] Low-vision rehab exercises signed off by a licensed therapist.
- [ ] Matter / HomeKit / Alexa e-stop command routes with `URGENT` priority.
- [ ] Federated research honours Consent resource state transitions.
- [ ] Interop verified with WIA-MED-ASSIST-001 (haptic cane) and WIA-MED-BCI-001 (cortical).

---

## 12. References

1. HL7 FHIR R5 — Device, Procedure, Observation, AdverseEvent, Communication, Consent.
2. DICOM PS3.3 — Ophthalmic IODs (OP, OPT, VF).
3. LOINC terminology (codes 87615-1 BCVA; 91873-9 fitting; 91874-7 threshold).
4. IEC 60601-1-8 — Alarm systems.
5. IEC 62304 — Medical device software lifecycle.
6. FDA 21 CFR 803 — MDR; EU MDR Annex IX — Vigilance.
7. WHO ICF — International Classification of Functioning.
8. Matter 1.3 specification.
9. Greenwald SH et al. (2020). Rehabilitation outcomes with retinal prostheses. *Ophthalmology*.

---

**Document Status:** ACTIVE
**Effective Date:** April 25, 2026
**Review Date:** April 25, 2028

© 2026 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
