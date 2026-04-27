# PHASE 1 — Data Format

> Telemedicine canonical envelopes: consultation session, clinical
> encounter, e-prescription, appointment slot, patient consent.
> All envelopes are signed with Ed25519 over the canonical JSON
> form (RFC 8785 JCS) and bind to HL7 FHIR R5 resources where
> applicable.

## 1.1 Consultation session envelope

The `consultation_session` envelope is the smallest unit of a
remote care interaction. It binds a patient and a provider to a
modality (video, audio, chat, asynchronous messaging) and a
scheduled time.

```
{
  "wia_telemedicine_version": "1.0.0",
  "type": "consultation_session",
  "session_id": "ULID",
  "patient_id":  "did:wia:patient:... | FHIR Patient/...",
  "provider_id": "did:wia:provider:... | FHIR Practitioner/...",
  "modality": "video" | "audio" | "chat" | "async_message"
            | "store_and_forward",
  "specialty": "general" | "dermatology" | "psychiatry"
             | "ophthalmology" | "telestroke" | "tele_icu"
             | "second_opinion",
  "scheduled_start": "RFC 3339",
  "scheduled_end":   "RFC 3339",
  "patient_consent_ref": "did:wia:consent:...",
  "jurisdiction":  "ISO 3166-1 alpha-2",
  "language_pref": "BCP 47",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

`patient_consent_ref` is mandatory; a session cannot exist
without an explicit, signed `patient_consent` envelope.

## 1.2 Clinical encounter envelope

The `clinical_encounter` envelope is the post-session record of
the interaction's clinical content. It maps to FHIR Encounter and
carries the diagnostic codes, findings, and follow-up plan.

```
{
  "wia_telemedicine_version": "1.0.0",
  "type": "clinical_encounter",
  "encounter_id": "ULID",
  "session_id":   "ULID",
  "patient_id":   "did:wia:patient:...",
  "started_at": "RFC 3339",
  "ended_at":   "RFC 3339",
  "icd11_codes": [
    { "code": "BA00.Z", "display": "Hypertensive disease unspecified" }
  ],
  "snomed_findings": [
    { "code": "271649006", "display": "Systolic blood pressure" }
  ],
  "vitals_observations": [
    "fhir:Observation/..."
  ],
  "imaging_refs": [ "dicom:wado-rs:..." ],
  "follow_up_required": true,
  "signature_by_provider": "Ed25519"
}
```

## 1.3 e-Prescription envelope

```
{
  "wia_telemedicine_version": "1.0.0",
  "type": "e_prescription",
  "rx_id": "ULID",
  "encounter_id": "ULID",
  "patient_id":   "did:wia:patient:...",
  "prescriber_id": "did:wia:provider:...",
  "rxnorm_code": "...",
  "atc_code":    "...",
  "dose":        "10 mg",
  "route":       "oral",
  "frequency":   "BID",
  "duration_days": 30,
  "refills_authorised": 0,
  "controlled_substance_schedule": null,
  "drug_interaction_checks": [
    { "kind": "ddi", "severity": "minor", "summary": "..." }
  ],
  "patient_allergy_review_completed": true,
  "signature_by_prescriber": "Ed25519"
}
```

The `drug_interaction_checks` and
`patient_allergy_review_completed` fields are mandatory because
the most common preventable harm from teleprescribing is a
drug-drug or drug-allergy interaction that an in-person prescriber
would catch by looking at a paper chart.

## 1.4 Appointment slot envelope

```
{
  "wia_telemedicine_version": "1.0.0",
  "type": "appointment_slot",
  "slot_id": "ULID",
  "provider_id": "did:wia:provider:...",
  "available_from": "RFC 3339",
  "duration_min": 30,
  "modality": "video" | "audio" | "chat" | "async_message",
  "specialty": "...",
  "languages_supported": ["en", "ko"],
  "jurisdictions_licensed": ["KR", "US-CA"],
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

`jurisdictions_licensed` is mandatory because cross-jurisdiction
prescribing has different licensing requirements than
cross-jurisdiction consultation; a patient discovering at booking
time that the provider is not licensed in the patient's
jurisdiction is the worst possible time for that discovery.

## 1.5 Patient consent envelope

The `patient_consent` envelope is signed by the patient (not the
provider) and is the legal basis for every other envelope in the
patient's encounter chain.

```
{
  "wia_telemedicine_version": "1.0.0",
  "type": "patient_consent",
  "consent_id": "ULID",
  "patient_id": "did:wia:patient:...",
  "scope": "telemedicine_consultation"
        | "data_sharing_emr"
        | "data_sharing_research"
        | "recording_session",
  "granted_at": "RFC 3339",
  "valid_until": "RFC 3339",
  "revocable": true,
  "language_of_consent": "BCP 47",
  "signed_by_patient": "Ed25519",
  "witness_signed": "Ed25519 (optional)"
}
```

A `witness_signed` field is required for vulnerable populations
(minors, patients with documented cognitive impairment) per the
applicable jurisdiction's law.

## 1.6 Patient-reported outcome envelope

```
{
  "wia_telemedicine_version": "1.0.0",
  "type": "patient_reported_outcome",
  "pro_id": "ULID",
  "encounter_id": "ULID",
  "instrument": "PHQ-9" | "GAD-7" | "PROMIS-29" | "EQ-5D-5L"
              | "tenant-specific",
  "captured_at": "RFC 3339",
  "responses": [ { "item": "...", "value": 0 } ],
  "score_summary": 0,
  "signed_by_patient": "Ed25519"
}
```

PROs are first-class envelopes because outcome measurement under
remote care is otherwise sparse; instrument-based PROs are the
most reliable signal of treatment efficacy in telemedicine
modalities.

## 1.7 References

- HL7 FHIR R5 — Patient, Practitioner, Encounter, Observation,
  MedicationRequest, Consent
- WHO ICD-11 — diagnostic coding
- SNOMED CT International — clinical findings
- RxNorm + WHO ATC/DDD — medication identification
- DICOM Web (WADO-RS / STOW-RS / QIDO-RS) — imaging exchange
- IHE ITI — cross-enterprise document sharing
- ISO 3166-1 — jurisdiction codes
- BCP 47 — language tags
- RFC 8785 — JSON Canonicalisation Scheme

## 1.8 Vital sign data binding

Vital signs captured during a remote encounter are emitted as FHIR
Observation resources and referenced from the
`clinical_encounter.vitals_observations` array. The standard does
not redefine vital-sign coding; LOINC codes are used for the
observation type (e.g., 85354-9 for blood pressure panel) and UCUM
units for measurements.

## 1.9 Imaging binding

Imaging studies acquired or reviewed during a remote encounter are
referenced as DICOM Web URLs in `clinical_encounter.imaging_refs`.
The standard does not redistribute imaging payloads; the URLs
point to the originating PACS/VNA where access control is
enforced.

弘益人間 — Benefit All Humanity. Telemedicine envelopes exist so that
a remote consultation is as auditable as an in-person one, while
remaining accessible to patients in jurisdictions that do not have
adjacent providers in their specialty.

## 1.10 Recording and retention

Sessions in modalities `video`, `audio`, and `chat` MAY be
recorded. Recording requires a separate `patient_consent` envelope
with `scope: recording_session`. Recordings are stored under the
patient's tenant with retention determined by jurisdiction:

- US (HIPAA) — minimum 6 years from creation
- EU (GDPR + national health code) — patient-controlled retention
- KR (의료법 제22조) — 10 years for medical records
- JP (医療法) — 5 years post-treatment

The standard does not enforce retention; it requires that the
recording's `retention_until` field be set per jurisdiction and
that the recording broker honour the field.

## 1.11 Cross-jurisdictional referral envelope

A `referral` envelope is emitted when a provider in one jurisdiction
refers a patient to a provider in another. The envelope carries
both providers' identities, the clinical rationale, and the
scope of consent the patient has granted to the receiving
provider.

```
{
  "wia_telemedicine_version": "1.0.0",
  "type": "referral",
  "referral_id": "ULID",
  "patient_id": "did:wia:patient:...",
  "from_provider": "did:wia:provider:...",
  "to_provider":   "did:wia:provider:...",
  "rationale_icd11": "...",
  "consent_scope":  "...",
  "signature_by_referring": "Ed25519"
}
```

## 1.12 Audit and tamper evidence

Every envelope's signature covers the canonical JSON form. The
provider tenant SHOULD maintain an append-only Merkle log and
publish a `daily_attestation_root` envelope so that an audit six
years later can verify that an encounter envelope existed in its
current form on its emission date.

弘益人間 — Benefit All Humanity. The envelope is the patient's record,
not the provider's. The patient's signature is what makes it real.

## 1.13 Worked example: psychiatry session lifecycle

The following sequence shows the envelope flow for a 50-minute
psychiatry session with PHQ-9 follow-up:

```
T-7d:   appointment_slot published by provider
T-3d:   patient signs patient_consent (telemedicine_consultation)
        + patient_consent (recording_session)
T-1d:   consultation_session published (modality: video,
        specialty: psychiatry, scheduled 50 min)
T+0:    session opens, WebRTC media established
T+50m:  session closes
T+1h:   clinical_encounter published with ICD-11 6A70.0,
        SNOMED 35489007 (Depressive disorder)
T+1h:   e_prescription published (RxNorm 312036 for sertraline,
        50 mg, daily, 30 days, 2 refills authorised)
T+24h:  patient_reported_outcome (PHQ-9, score 12)
T+30d:  follow-up appointment_slot published; patient books;
        cycle repeats
```

Each envelope is signed; the chain forms a complete
session-to-followup audit trail the patient owns.

## 1.14 Vulnerable population safeguards

For minors, the patient_consent envelope includes a parental or
guardian counter-signature. For patients with documented cognitive
impairment, a witness signature (designated representative) is
required. The standard does not adjudicate which patients are
vulnerable — that is properly a clinical and legal determination —
but requires that when a vulnerable-population designation is in
effect, the additional signatures appear on the consent.

## Implementer's checklist

For organisations preparing a conforming telemedicine
implementation, the following non-exhaustive checklist captures
the operational decisions every implementation has to make
explicitly. The checklist is intentionally pragmatic; it omits
nice-to-have features in favour of the items that, when missed,
have actually caused production incidents in deployed
telemedicine systems:

1. Identify the patient identity assurance level that the
   implementation will accept. Different levels suit different
   risk profiles (mental-health teleconsultation requires a
   higher level than dietary advice).
2. Select the WebRTC media path. Direct peer-to-peer is the
   lowest latency but cannot be recorded; SFU/MCU adds latency
   but allows recording, transcription, and analyst review.
3. Decide the pharmacy back-end before launch. Pharmacy
   integration is the longest-lead-time integration in most
   deployments and is rarely a strength of the platform team.
4. Decide the EHR back-end before launch. FHIR R5 read/write
   parity with the EHR is rare; most EHRs offer read-only FHIR
   and require a vendor-specific channel for write.
5. Select the patient-facing UI accessibility baseline. WCAG
   2.2 AA is the minimum; AAA is the appropriate target for
   adult-care platforms supporting elderly patients.
6. Decide the cross-jurisdiction policy. Federation is powerful
   but unsafe to enable before the consent and identity flows
   are stable in single-jurisdiction operation.
7. Choose the recording retention policy and surface it in the
   consent envelope. The recording retention has direct legal
   exposure for the platform.
8. Build operational procedures for the rare but consequential
   events: emergency exception (§3.9), prescription error
   correction, identity-assurance failure, recording loss.

弘益人間 — Benefit All Humanity. The checklist exists so that
implementers can fail fast on the items that matter, before they
fail slow on the items that matter to patients.

## Telemedicine in low-bandwidth contexts

Many of the patients who would benefit most from telemedicine live
in low-bandwidth contexts: rural broadband, mobile networks in
mountainous regions, satellite links for maritime crew, refugee
camps with intermittent connectivity. The standard accommodates
low-bandwidth operation through three deliberate choices:

- the `async_message` and `store_and_forward` modalities require
  no real-time channel and are eventually-consistent;
- envelope payloads are small (typically under 4 KB unsigned)
  and tolerate aggressive compression on the wire;
- the SSE stream replay-on-connect (Phase 2 §2.6) absorbs the
  short connection drops typical of mobile and satellite networks
  without the patient having to re-fetch full envelope history.

弘익인간 — Benefit All Humanity also means the patient on the worst
connection should still receive a verifiable record.
