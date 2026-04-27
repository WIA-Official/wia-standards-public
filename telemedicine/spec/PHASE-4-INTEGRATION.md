# PHASE 4 — Integration

> Integration with HL7 FHIR R5, IHE profiles, ICD-11, SNOMED CT,
> RxNorm, eIDAS, KFTC OpenBanking-style claims, and pharmacy /
> lab / imaging back ends.

## 4.1 HL7 FHIR R5 binding

The standard binds tightly to FHIR R5 for clinical content
modelling. The mapping is:

```
WIA envelope                     FHIR R5 resource
───────────────────────────────  ─────────────────────────────
consultation_session             Encounter (status: in-progress)
clinical_encounter               Encounter (status: finished) +
                                 Condition + Observation set
e_prescription                   MedicationRequest
appointment_slot                 Slot + Schedule
patient_consent                  Consent
patient_reported_outcome         Observation +
                                 QuestionnaireResponse
referral                         ServiceRequest
cross_border_bundle              Bundle (transaction or document)
```

WIA envelopes carry the Ed25519 signature and the cross-vendor
binding metadata; FHIR resources carry the clinical content. The
two are complementary, not redundant.

## 4.2 IHE profile binding

The standard interoperates with the following IHE profiles:

- **ITI XDS / XCA** — document sharing across enterprises
- **PCC APS / PCC RECON** — care coordination
- **LAB Laboratory Reporting Workflow** — lab flows
- **CARD Cardiology** — ECG, echocardiogram exchange
- **PHARM e-Prescription** — prescription routing
- **MHD Mobile Health Documents** — FHIR-based document exchange

The WIA envelope is referenced from the IHE document metadata
header; downstream IHE consumers can verify the WIA signature
without parsing the envelope itself.

## 4.3 Coding system bindings

```
WIA envelope field                Coding system
────────────────────────────────  ─────────────────────────────
icd11_codes                       WHO ICD-11 (2024 release)
snomed_findings                   SNOMED CT International
rxnorm_code                       US RxNorm + WHO ATC/DDD
loinc_codes (in observations)     LOINC 2.78+
ucum_units                        UCUM
language_pref / consent           BCP 47
jurisdiction                      ISO 3166-1 alpha-2
```

The standard does not redistribute the coding system content;
implementations license SNOMED CT separately under the SNOMED
International Affiliate Licence and obtain ICD-11 / LOINC under
the publishers' respective licences.

## 4.4 Identity binding (eIDAS / KFTC / national)

For patient identity assurance:

- **EU**: eIDAS-conformant national eID (German Personalausweis,
  Italian SPID, Spanish DNIe, etc.)
- **KR**: KFTC OpenBanking-grade identity, 의료마이데이터 binding
- **JP**: My Number (マイナンバー) via the Healthcare
  Information Network identity provider
- **US**: state-issued ID through eIDAS-equivalent providers
  (ID.me, Login.gov)
- **Self-sovereign**: patient-curated DID (last resort)

Provider identity assurance is via WIA-OMNI-API trust fabric and
the provider's professional licensing body (KMA, AMA, GMC,
respective specialty colleges).

## 4.5 EHR integration

Major EHR systems (Epic, Cerner Oracle Health, Meditech, AthenaIDX,
한국전자의무기록 vendors) integrate via FHIR R5 endpoints exposed
by the EHR. The WIA envelope is published in parallel; the EHR
remains the canonical clinical record, the WIA envelope is the
canonical interoperability artefact.

## 4.6 Pharmacy integration

Pharmacy systems consume `e_prescription` envelopes via the
SureScripts (US), KFTC 처방전 전송 (KR), or national e-prescription
gateway (EU member states) protocols. The WIA envelope is the
canonical signed prescription; the gateway protocol is the
transport.

## 4.7 Laboratory integration

Lab systems consume `referral` envelopes (for lab orders) and
publish DiagnosticReport resources back via FHIR. The WIA
envelope around the referral is the patient-consent binding; the
FHIR DiagnosticReport carries the result content.

## 4.8 Imaging integration

PACS / VNA systems integrate via DICOM Web (WADO-RS, STOW-RS,
QIDO-RS). Imaging studies referenced from `clinical_encounter`
envelopes resolve to DICOM Web URLs at the originating PACS where
access control is enforced.

## 4.9 Insurance / payer integration

Payer integration is via the X12 837 (US), KR HIRA 청구, EU
national billing protocols. The WIA envelope is referenced from
the claim as a supporting document; the claim payload is the
payer's native format.

## 4.10 Korean integration notes

- 의료법 제22조 medical-record retention: 10 years
- 의료마이데이터 (MyHealthRecord-KR): patient data portability
- HIRA 심사평가 청구: claim submission via existing HIRA gateway
- 응급의료법 제27조: emergency exception aligns with §3.9

For Korean providers, the WIA envelope stream is consumable by
HIRA, the 건강보험심사평가원 portal, and the patient's 의료마이데이터
view through the bulk export endpoint.

## 4.11 European integration notes

- GDPR Article 9: special category health data processing under
  explicit consent
- EHDS (European Health Data Space) regulation: patient
  primary-use data portability
- eIDAS 2.0: identity wallets
- EU Cyber Resilience Act: medical-device + telemedicine platform
  vulnerability handling commitments

## 4.12 US integration notes

- HIPAA Privacy Rule + Security Rule
- ONC information-blocking rules: data portability (USCDI v3+)
- DEA Ryan Haight Act: controlled-substance teleprescribing
  restrictions
- CMS telehealth reimbursement (POS code 02 / modifier 95)
- State-by-state licensing compacts (IMLC, PSY-PACT)

## 4.13 WIA family integration

This standard plugs into:

- **WIA-OMNI-API** — provider credential storage
- **WIA Money** — copay and self-pay flows
- **WIA-AIR-SHIELD** — transport hardening for federation
- **WIA Vital-Sign Streaming** — continuous biosignal capture
  during home-monitoring telemedicine

## 4.14 Worked example: cross-border second opinion

A patient in Korea seeks a second opinion from a US-based
sub-specialist. The flow:

```
1. Patient signs consent_envelope scope: cross_border_consultation
2. KR provider exports clinical_encounter + imaging refs as FHIR Bundle
3. KR provider initiates federation_handshake to US sub-specialist
4. US sub-specialist reviews under TLP AMBER+STRICT
5. US sub-specialist publishes consultation_session +
   clinical_encounter (second-opinion encounter)
6. Both encounters reference the patient's master record by DID
7. Patient receives both encounter envelopes in their wallet
8. Patient retains durable, signed second-opinion record
```

Three providers, two jurisdictions, one signed envelope chain
the patient owns at the end.

## 4.15 Conformance maturity model

- **Bridge-only** — translates FHIR + IHE flows into envelopes
- **Native** — authors envelopes natively from the EHR
- **Native + Federation** — additionally federates across
  jurisdictions

## 4.16 Operational considerations

Telemedicine integration is dominated by identity-assurance and
licensing complexity, not by technical complexity. Implementers
SHOULD invest in §4.4 (identity binding) and §4.12 (US licensing
compacts) before investing in §4.6 (pharmacy) or §4.10 (Korean
HIRA), because identity and licensing failures at session-start
time poison every downstream envelope.

## 4.17 Backwards compatibility

Pre-standard telemedicine platforms MAY operate proprietary
schemas in parallel with WIA envelopes during a transitional
window. The WIA envelope is canonical for cross-vendor verification;
the proprietary schema remains canonical inside the platform
operator.

## 4.18 Reference list

- HL7 FHIR R5
- IHE ITI / PCC / LAB / CARD / PHARM / MHD profiles
- WHO ICD-11
- SNOMED CT International
- LOINC 2.78+
- UCUM
- RxNorm + WHO ATC/DDD
- DICOM Web
- eIDAS Regulation 910/2014 + eIDAS 2.0
- KR 의료법, KR 의료마이데이터, KR 응급의료법
- HIPAA / HITECH
- US ONC information-blocking rules
- US DEA Ryan Haight Act
- EU EHDS Regulation
- EU GDPR

## 4.19 Operational coda

Telemedicine adoption succeeds when the patient experience is
better than in-person care for at least one dimension (access,
privacy, cost, convenience) and not measurably worse on the
others. The standard's envelope discipline contributes to better
access (cross-jurisdiction federation) and better privacy
(explicit, revocable consent) without compromising clinical
quality (FHIR + IHE binding).

弘益人間 — Benefit All Humanity. The integration surface exists so
that telemedicine reaches the patient who needs it most — the
patient without an adjacent specialist — without becoming a
walled garden of vendor-locked records.

## 4.20 Worked example: chronic-care diabetes management

A patient with type-2 diabetes in Korea uses a CGM (continuous
glucose monitor) at home and consults monthly with their
endocrinologist via telemedicine. The standards in play:

- WIA Vital-Sign Streaming envelopes carry CGM readings
  continuously
- WIA Telemedicine envelopes carry the monthly consultation,
  encounter, and prescription records
- WIA-OMNI-API anchors the patient's identity across both standards
- HIRA receives quarterly claim envelopes derived from the
  encounter stream
- 의료마이데이터 portal exposes the full envelope history to the
  patient

The patient's care record is durable, signed, and patient-owned;
the endocrinologist's workflow is unchanged from in-person
practice apart from the video signalling layer; the regulator
receives standard-format claims; the insurer receives standard-
format encounter evidence.

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
