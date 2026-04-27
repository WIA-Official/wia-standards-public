# PHASE 2 — API Interface

> HTTP and WebRTC signalling surface for telemedicine session
> establishment, EHR query, e-prescription publication, and
> appointment management. Requests carry HTTP Message Signatures
> (RFC 9421); errors use Problem Details (RFC 9457).

## 2.1 Endpoint surface

```
POST /tm/v1/sessions                       Open consultation_session
GET  /tm/v1/sessions/{id}                  Fetch session
POST /tm/v1/sessions/{id}/signalling       WebRTC signalling
POST /tm/v1/sessions/{id}/end              Close session

POST /tm/v1/encounters                     Publish clinical_encounter
GET  /tm/v1/encounters/{id}                Fetch encounter

POST /tm/v1/prescriptions                  Publish e_prescription
GET  /tm/v1/prescriptions/{id}             Fetch prescription

GET  /tm/v1/slots?provider_id=...          Discover appointment slots
POST /tm/v1/slots                          Publish a slot
POST /tm/v1/slots/{id}/book                Book a slot

POST /tm/v1/consents                       Patient consent
POST /tm/v1/consents/{id}/revoke           Revoke consent

POST /tm/v1/pro                            Patient-reported outcome
POST /tm/v1/referrals                      Cross-provider referral

GET  /tm/v1/stream/{patient_id}            SSE: patient envelope stream
GET  /.well-known/wia-telemedicine         Capability advertisement
```

## 2.2 Authentication

All requests carry HTTP Message Signatures (RFC 9421) over the
mandatory `(request-target)`, `@authority`, `content-digest`,
`@created`, `@expires` covered components. Provider keys live in
the WIA-OMNI-API trust fabric; patient keys live in the patient's
hardware-bound passkey or jurisdiction-issued health credential
(eIDAS, KFTC, MyHealthRecord, KRX 의료마이데이터).

## 2.3 Signalling

WebRTC signalling uses the standard offer/answer + ICE candidate
exchange via the `/sessions/{id}/signalling` endpoint. The
endpoint is bidirectional (POST from either side) and is signed
per request. Media is SRTP (AES-128-GCM); signalling is TLS 1.3.

## 2.4 Idempotency

POST endpoints accept `Idempotency-Key` headers (UUIDv7
recommended). A repeated request within 24 hours returns the
original response without re-creating the resource. Idempotency
matters in telemedicine because mobile clients reconnect
frequently and double-creating an encounter is harmful (it
duplicates a billing record).

## 2.5 Pagination

List endpoints use cursor pagination with `Link` headers
(RFC 8288). Page size is capped at 200 envelopes per page.

## 2.6 Streaming semantics

The `/stream/{patient_id}` endpoint emits Server-Sent Events for
the patient's envelopes (sessions, encounters, prescriptions,
follow-up reminders). Access is gated by the patient's signed
consent for the consuming application; family-member access uses
delegated consent.

## 2.7 Capability advertisement

```
GET /.well-known/wia-telemedicine

200 OK
{
  "wia_telemedicine_version": "1.0.0",
  "type": "capability_advertisement",
  "provider_org_id": "did:wia:provider-org:...",
  "endpoint_surface_version": "tm/v1",
  "supported_modalities": ["video", "audio", "chat", "async_message"],
  "supported_specialties": ["general", "psychiatry", "dermatology"],
  "languages": ["en", "ko", "ja"],
  "jurisdictions_licensed": ["KR", "US-CA", "US-NY"],
  "fhir_endpoint": "https://...",
  "rate_limits": { "rpm": 600 },
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## 2.8 Error envelope

```
{
  "type": "https://wiastandards.com/telemedicine/errors/jurisdiction-licence-missing",
  "title": "Provider not licensed for patient jurisdiction",
  "status": 403,
  "detail": "Provider licensed in [KR]; patient in [US-NY]",
  "patient_id": "did:wia:patient:..."
}
```

## 2.9 Versioning

The wire format version is in the envelope; the HTTP surface is
in the URL (`/tm/v1`). Breaking changes get a new URL prefix.

## 2.10 Rate limits

Default: 600 RPM per provider client identity, 100 burst per
second. Patient-facing apps inherit a per-patient limit of 60 RPM
to prevent automated harvesting of consent endpoints.

## 2.11 Content negotiation

JSON by default; CBOR (RFC 8949) negotiated via
`Accept: application/cbor`. The signature is over the canonical
JSON form regardless.

## 2.12 Webhook callbacks

Providers register callbacks for downstream EHR systems and
patient-facing apps. Backoff: 1, 2, 4, 8, 16, 32, 64s, dead-letter
at the 8th attempt. Receivers MUST verify envelope signatures.

## 2.13 Bulk export (patient data portability)

```
POST /tm/v1/exports
{ "patient_id": "...", "from": "...", "to": "..." }
→ 202 Accepted, Location: /tm/v1/exports/{id}
```

The export bundle is FHIR Bundle resource format with WIA
envelopes attached as DocumentReference resources, signed by
the patient. Patient data portability satisfies GDPR Article 20,
KR 의료마이데이터 데이터이동권, and US ONC information-blocking
rules.

## 2.14 Health and observability

```
GET /tm/v1/health    → liveness
GET /tm/v1/ready     → readiness
GET /tm/v1/metrics   → Prometheus exposition
```

## 2.15 Operational considerations

End-to-end latency budget for video signalling SHOULD target
p95 < 250 ms. Re-keying SRTP streams on the hour is the standard
recommendation; mid-call re-keys can introduce audio glitches and
SHOULD be reserved for credential rotation events.

Patient-facing applications MUST display a clear visual indicator
when a session is being recorded, in addition to the consent
envelope record. The visual indicator is a usability requirement,
not a legal substitute for consent.

## 2.16 Backwards compatibility

EHR systems migrating from a vendor-proprietary REST surface MAY
operate a translator that consumes their native format and emits
WIA envelopes. The translator is a publisher in its own right with
its own DID-bound signing key.

## 2.17 Test surface

A read-only test surface is exposed at
`https://sandbox.wiastandards.com/tm/v1` returning realistic
synthetic envelope traffic. Sandbox traffic is signed by the WIA
test key; signatures will not verify against production trust
roots.

## 2.18 Long-poll fallback

For clients that cannot maintain SSE, long-poll is available on
`GET /tm/v1/poll/{patient_id}?since=<envelope_id>` with a 30-second
hold. The standard documents long-poll as a fallback rather than
a primary path; SSE is canonical.

## 2.19 Field-level encryption

Sensitive fields (national identifiers, mental-health diagnostic
codes, HIV/STI status) MAY be field-level encrypted using AES-256-
GCM with per-tenant keys held in the tenant's HSM. The encrypted
form appears in the canonical envelope; signature verification
does not require decryption.

## 2.20 Schema registry

`GET /tm/v1/schemas/{type}/{version}` returns the JSON Schema for
the named envelope type at the named version. Schemas are
immutable; new versions get new identifiers. Conforming clients
MUST validate against the schema version named in the envelope.

## 2.21 Worked example: video consultation

```
1. Patient app: POST /tm/v1/consents (signed by patient passkey)
2. Patient app: POST /tm/v1/sessions (refers consent_id)
3. Provider app: GET  /tm/v1/sessions/{id}  (verifies patient identity)
4. Both apps:    POST /tm/v1/sessions/{id}/signalling (WebRTC offer/answer/ICE)
5. SRTP media:   established, AES-128-GCM
6. Provider app: POST /tm/v1/encounters (after the call ends)
7. Provider app: POST /tm/v1/prescriptions (if applicable)
8. Patient app:  POST /tm/v1/pro (PHQ-9 follow-up)
9. Either:       POST /tm/v1/sessions/{id}/end (signed)
```

The chain of envelopes from initial consent to closing PRO forms
a complete signed audit trail of the encounter, queryable by the
patient at any future date.

## 2.22 Operational coda

Adoption is incremental: a small clinic can begin by publishing
consultation sessions and consents, and add encounter, prescription,
and PRO publication later as workflow allows. The standard is
intentionally additive.

弘益人間 — Benefit All Humanity. The API exists so that the patient,
not the clinic, is the durable owner of the encounter record.

## 2.23 Capability advertisement worked example

```
GET /.well-known/wia-telemedicine

200 OK
{
  "wia_telemedicine_version": "1.0.0",
  "type": "capability_advertisement",
  "provider_org_id": "did:wia:provider-org:seoul-medical-centre",
  "endpoint_surface_version": "tm/v1",
  "supported_modalities": ["video", "audio", "chat", "async_message"],
  "supported_specialties": [
    "general", "psychiatry", "dermatology",
    "endocrinology", "cardiology"
  ],
  "languages": ["ko", "en", "ja"],
  "jurisdictions_licensed": ["KR", "JP"],
  "fhir_endpoint": "https://fhir.smc.example/r5",
  "rate_limits": { "rpm": 600, "burst_per_second": 100 },
  "vulnerable_population_workflows_supported": ["minor", "cognitive"],
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

A consumer integrating with this provider for the first time
reads this single document and learns everything needed to begin:
which modalities, which specialties, which languages, which
jurisdictions, where to talk FHIR, and which vulnerable-population
workflows are honoured.

## 2.24 Webhook callback worked example

```
POST /tm/v1/webhooks
{
  "url": "https://patient-app.example/wia-callback",
  "events": ["clinical_encounter", "e_prescription",
             "patient_reported_outcome"],
  "patient_filter": "did:wia:patient:...",
  "shared_secret_hint": "k_2026q3",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Patient-app callbacks are scoped per-patient via `patient_filter`
so that one webhook configuration cannot leak data across
patients on shared infrastructure.

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
