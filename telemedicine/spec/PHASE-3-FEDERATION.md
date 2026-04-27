# PHASE 3 — Federation Protocol

> Cross-jurisdiction provider trust, replay defence, patient
> consent flow across borders, and pharmacy / lab federation.

## 3.1 Why federate at all

A patient who travels, who relocates between jurisdictions, or who
needs subspecialty care unavailable locally is the typical
consumer of cross-jurisdiction telemedicine. The federation
protocol allows providers in different jurisdictions to share
clinical envelopes under explicit patient consent without
forcing either provider into a single shared EHR.

## 3.2 The federation handshake

```
{
  "wia_telemedicine_version": "1.0.0",
  "type": "federation_handshake",
  "handshake_id": "ULID",
  "initiator": "did:wia:provider-org:...",
  "counterparty": "did:wia:provider-org:...",
  "patient_consent_ref": "did:wia:consent:...",
  "scope": ["encounter:read", "prescription:read", "imaging:read"],
  "tlp": "AMBER+STRICT",
  "valid_until": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Healthcare federation operates at TLP AMBER+STRICT by default
(receiver may share within the receiving organisation only). RED
is reserved for emergency referrals where the consent does not
permit further sharing.

## 3.3 Replay defence

Federated envelopes carry a 96-bit nonce and are accepted within
±300 seconds wall-clock skew. The receiver maintains a 600-second
replay cache; replayed nonces are silently dropped.

## 3.4 Patient identity binding

Cross-jurisdiction federation relies on the patient identity
having a verifiable mapping across both jurisdictions. The
standard supports four binding mechanisms:

- **eIDAS** — EU member-state identity assurance
- **KFTC** — Korean financial-grade identity (의료마이데이터 binding)
- **NHS Number / Medicare ID / equivalent** — national health
  identifier where available
- **Patient-asserted DID** — a self-sovereign identifier the
  patient curates across jurisdictions

The provider's federation handshake names which binding
mechanism applies for the consent and what proof of binding the
patient produced.

## 3.5 Per-patient consent revocation

The patient revokes federation consent by signing an
`investor_consent_revocation` envelope (using the same envelope
type as Phase 1). The originating provider stops emitting to the
counterparty within 60 seconds; the counterparty stops querying
within 60 seconds. The 60-second SLA is the most time-sensitive
control in the protocol.

## 3.6 Cross-border clinical bundle

```
{
  "wia_telemedicine_version": "1.0.0",
  "type": "cross_border_bundle",
  "bundle_id": "ULID",
  "patient_id": "did:wia:patient:...",
  "from_jurisdiction": "KR",
  "to_jurisdiction":   "US-CA",
  "consent_ref": "...",
  "evidence_refs": [
    "wia:encounter:...",
    "wia:prescription:...",
    "wia:patient_reported_outcome:...",
    "fhir:Bundle/..."
  ],
  "languages_translated": ["en", "ko"],
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Translations are referenced by language; the standard does not
require any specific translation source but does require that the
translation provenance be recorded in the bundle.

## 3.7 Pharmacy federation

A prescription emitted in one jurisdiction may be filled in
another only when (a) the prescriber is licensed in both
jurisdictions or (b) a receiving prescriber re-issues under their
own licence after review. The standard provides a
`prescription_re_issuance` envelope for case (b); the chain
preserves the original prescriber's intent while making the
re-issuing prescriber's licence the legal basis.

## 3.8 Laboratory federation

Lab orders and results flow across providers via FHIR
ServiceRequest and DiagnosticReport resources, referenced from
the federation bundle. The standard does not redefine lab data
exchange; it provides the consent and audit binding around
existing FHIR + IHE LAB / IHE LCC profiles.

## 3.9 Emergency exception

In emergencies (loss of consciousness, life-threatening event)
where the patient cannot give consent, federation MAY proceed
under a `treating_in_loco_paciente` exception envelope signed by
the responding clinician under their professional licence. The
exception is itself signed and audit-trailed; the patient (or
next of kin) reviews the exception after the fact and may
challenge it through normal regulatory channels.

## 3.10 Federation telemetry

Daily `federation_telemetry` envelopes summarise envelope volume,
replay-cache hit rate, revocation latency, and cross-border bundle
count. The telemetry stream is queryable by both peers.

## 3.11 Worked example: telestroke

A hospital in a rural jurisdiction without 24/7 neurology coverage
federates with a tele-stroke service in another jurisdiction. A
patient presents with stroke symptoms; the local emergency
clinician initiates a federation handshake under emergency
exception (§3.9). The remote neurologist reviews imaging and
clinical signs in real time and recommends thrombolysis. The
local clinician administers under their own licence; the
recommendation is recorded as a referral envelope from the remote
neurologist. The patient retroactively counter-signs the consent
during recovery.

## 3.12 Conformance test corpus

A reference test corpus is published at
`https://wiastandards.com/telemedicine/conformance/` covering
valid handshakes, replay defence, cross-border bundle assembly,
pharmacy re-issuance, and emergency exception. Conforming
implementations MUST pass the corpus and publish a
`conformance_attestation` envelope.

## 3.13 Operational considerations

Federation introduces additional latency. Implementations SHOULD
cache the latest signed envelope per patient per peer for at most
60 seconds during business hours and 5 minutes off-hours, with
on-demand refresh.

Federated identity proofing should be re-verified annually for
chronic-care patients to absorb identity-assurance lifecycle
events (passport renewal, insurance switch, address change).

## 3.14 Backwards compatibility

Implementations migrating from IHE XDS / XCA cross-enterprise
document sharing MAY operate parallel envelope and IHE flows
during a transitional window of 18 months. WIA envelopes are
canonical for verification; IHE messages remain canonical for
existing IHE-only consumers.

## 3.15 Reference list

- HL7 FHIR R5 — patient and encounter modelling
- IHE ITI — cross-enterprise document sharing
- IHE PCC — patient care coordination profiles
- WHO ICD-11 — diagnostic coding
- SNOMED CT — clinical terminology
- eIDAS Regulation — EU identity assurance
- KR 의료마이데이터 — Korean medical MyData
- US ONC — information-blocking rules
- RFC 8785 — JSON Canonicalisation
- RFC 9421 — HTTP Message Signatures

## 3.16 Operational coda

Federation increases the surface that must be secured but also
increases the surface that delivers value to the patient. The
balance favours federation when (a) consent is explicit and
revocable, (b) replay defence is operational, and (c) audit
visibility is mutual. Implementations should not enable
federation features before all three conditions are met.

弘益人間 — Benefit All Humanity. Federation lets a rural patient
reach a specialist; the consent envelope makes sure it stays
patient-controlled.

## 3.17 Translation and accessibility

Cross-border federation routinely involves language translation.
Translation provenance is recorded in the
`cross_border_bundle.languages_translated` field; downstream
consumers know which language pairs the bundle has been
translated through. Machine translation is permissible for
patient comprehension aids but MUST NOT replace human translation
for legal documents (consent, prescription instructions,
discharge summaries). The standard's `translation_attestation`
sub-envelope records which fields were machine-translated, which
were human-translated, and the translator's identity.

Accessibility considerations mirror translation: a patient with
visual impairment requires ARIA-described UI; a patient with
hearing impairment requires real-time captioning during video
sessions. The standard does not redefine accessibility but
requires that the consent envelope record the patient's declared
accessibility needs so that downstream session orchestration can
provision the appropriate aids.

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
