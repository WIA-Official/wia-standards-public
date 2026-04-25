# WIA-TRADITIONAL-MEDICINE: Phase 3 - Protocol

## 弘益人間 (Benefit All Humanity)

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-01

---

## 1. Overview

Phase 3 defines standardized clinical protocols for traditional medicine practice, ensuring consistency, safety, and quality across different practitioners and settings.

---

## 2. Constitution Assessment Protocol

### 2.1 Pre-Assessment Requirements

| Requirement | Description |
|-------------|-------------|
| Patient Preparation | No food/drink 2 hours before (for tongue diagnosis) |
| Environment | Quiet room, natural or color-corrected lighting |
| Time Allocation | 30-45 minutes for comprehensive assessment |
| Documentation | Digital recording system ready |

### 2.2 Assessment Workflow

```
STEP 1: QUESTIONNAIRE ADMINISTRATION
├── Select appropriate questionnaire
│   ├── TCM: CCMQ (Constitution in Chinese Medicine Questionnaire)
│   ├── Ayurveda: Prakriti Assessment Questionnaire
│   └── Sasang: QSCCII (Questionnaire for Sasang Constitution Classification II)
├── Administer in patient's native language
├── Allow 15-20 minutes for completion
└── Calculate and record raw scores

STEP 2: PHYSICAL EXAMINATION
├── Body type assessment (frame, weight distribution)
├── Facial characteristics analysis
├── Voice quality assessment
├── Gait observation
└── Skin characteristics

STEP 3: AI-ASSISTED ANALYSIS (Optional)
├── Facial image capture (standardized protocol)
├── Voice recording (30-second sample)
├── Integration with questionnaire results
└── Confidence score generation

STEP 4: MULTI-OMICS INTEGRATION (When Available)
├── Pharmacogenomic markers (CYP450, etc.)
├── Metabolomic profile
├── Microbiome assessment
└── Constitutional biomarker panel

STEP 5: SYNTHESIS AND DOCUMENTATION
├── Integrate all data sources
├── Determine primary and secondary constitution types
├── Document confidence levels
└── Generate personalized recommendations
```

---

## 3. Four Examinations Protocol (四診)

### 3.1 Inspection Protocol (望診)

#### 3.1.1 Tongue Diagnosis Standards

**Image Capture Protocol:**

| Parameter | Requirement |
|-----------|-------------|
| Lighting | Natural daylight or 5500K color-corrected |
| Camera | Minimum 12MP, auto white-balance disabled |
| Distance | 15-20 cm from tongue |
| Duration | Capture within 15 seconds |
| Background | Neutral gray card included in frame |
| Patient Position | Seated, tongue extended naturally |
| Timing | Before eating, drinking, or brushing |

**Analysis Parameters:**

| Feature | Options | Significance |
|---------|---------|--------------|
| Body Color | Pale, Light Red, Red, Dark Red, Purple | Reflects Qi, Blood, Yin, Yang status |
| Body Shape | Thin, Normal, Swollen, Teeth-marked | Reflects body fluid and Qi status |
| Coating Color | White, Yellow, Gray, Black | Reflects pathogen and heat level |
| Coating Thickness | Thin, Normal, Thick, Peeled | Reflects pathogen depth |
| Moisture | Dry, Normal, Wet | Reflects body fluid status |

### 3.2 Auscultation Protocol (聞診)

**Voice Analysis:**
- Record 30-second speech sample
- Assess volume, clarity, strength
- Note breathing patterns during speech

**Parameters:**

| Quality | Indication |
|---------|------------|
| Weak, low volume | Qi deficiency |
| Loud, coarse | Heat, excess |
| Hoarse | Phlegm, blood stasis |
| Breathless | Lung Qi deficiency |

### 3.3 Inquiry Protocol (問診)

**Ten Questions Framework (十問):**

1. **Cold/Heat (寒熱):** Temperature preferences, fever patterns
2. **Perspiration (汗):** Spontaneous, night sweats, absence
3. **Head/Body (頭身):** Headache, body aches, dizziness
4. **Chest/Abdomen (胸腹):** Fullness, pain, palpitations
5. **Appetite/Thirst (食慾渴飲):** Hunger, taste, thirst
6. **Bowel/Urination (二便):** Frequency, quality, color
7. **Sleep (睡眠):** Quality, dreams, waking patterns
8. **Ears/Eyes (耳目):** Hearing, vision, discharges
9. **Previous Illness (既往病史):** Medical history
10. **Cause of Illness (發病原因):** Triggers, onset

### 3.4 Palpation Protocol (切診)

**Pulse Diagnosis Standards:**

| Step | Protocol |
|------|----------|
| Patient Preparation | Rest 5 minutes, arm at heart level |
| Position | Practitioner uses 3 fingers (index, middle, ring) |
| Location | Radial artery: Cun, Guan, Chi positions |
| Pressure | Three levels: superficial, middle, deep |
| Duration | Minimum 60 seconds per wrist |
| Documentation | Rate, rhythm, quality for each position |

**Pulse Sensor Protocol (Optional):**

| Parameter | Specification |
|-----------|---------------|
| Sensor Type | Piezoelectric or optical |
| Sampling Rate | ≥1000 Hz |
| Channels | Minimum 3 (one per position) |
| Duration | Minimum 60 seconds |
| Output | Time-domain waveform, frequency spectrum |

---

## 4. Treatment Protocol

### 4.1 Treatment Selection Framework

```
PATTERN DIAGNOSIS
      ↓
DETERMINE TREATMENT PRINCIPLE (治法)
      ↓
SELECT BASE FORMULA
      ↓
MODIFY FOR CONSTITUTION
      ↓
ADJUST FOR INDIVIDUAL FACTORS
      ↓
SAFETY SCREENING
      ↓
PRESCRIBE AND DOCUMENT
```

### 4.2 Dosage Guidelines

| Factor | Adjustment |
|--------|------------|
| Age (child) | 1/4 to 1/2 adult dose |
| Age (elderly) | 2/3 to 3/4 adult dose |
| Constitution (weak) | Start with lower dose |
| Condition (acute) | Higher dose, shorter duration |
| Condition (chronic) | Lower dose, longer duration |

### 4.3 Treatment Duration

| Condition Type | Typical Duration | Review Interval |
|----------------|------------------|-----------------|
| Acute | 3-7 days | Daily |
| Sub-acute | 2-4 weeks | Weekly |
| Chronic | 1-3 months | Bi-weekly |
| Constitutional | Ongoing | Monthly |

---

## 5. Safety Protocol

### 5.1 Pre-Prescription Safety Checks

- [ ] Confirm patient identity
- [ ] Review medical history
- [ ] Check current medications
- [ ] Verify allergies
- [ ] Assess pregnancy/lactation status
- [ ] Run drug-herb interaction check
- [ ] Verify herb quality certification

### 5.2 Contraindication Categories

| Category | Action |
|----------|--------|
| Absolute | Do not prescribe |
| Relative | Weigh risk/benefit, modify dose |
| Conditional | Monitor closely |

### 5.3 Adverse Event Response

```
SUSPECTED ADVERSE EVENT
      ↓
STOP TREATMENT IMMEDIATELY
      ↓
ASSESS PATIENT CONDITION
      ↓
PROVIDE SUPPORTIVE CARE
      ↓
DOCUMENT ALL DETAILS
      ↓
REPORT VIA PHARMACOVIGILANCE SYSTEM
      ↓
FOLLOW UP WITH PATIENT
```

---

## 6. Quality Control Protocol

### 6.1 Herb Quality Standards

| Test | Standard |
|------|----------|
| Identity | DNA barcode, macroscopic, microscopic |
| Heavy Metals | As <3ppm, Pb <10ppm, Hg <0.5ppm, Cd <1ppm |
| Pesticides | Per pharmacopoeia limits |
| Microbial | Total count <10^5 CFU/g |
| Active Content | Per pharmacopoeia specification |

### 6.2 Documentation Standards

All clinical encounters must document:
- Date, time, practitioner ID
- Patient chief complaint
- Four examinations findings
- Pattern diagnosis with ICD-11 TM code
- Treatment principle and prescription
- Patient instructions
- Follow-up plan

---

## 7. Telehealth Protocol

### 7.1 Remote Consultation Requirements

| Component | Requirement |
|-----------|-------------|
| Video Quality | Minimum 720p, stable connection |
| Lighting | Patient well-lit for visual assessment |
| Tongue Photo | Standardized protocol (see 3.1.1) |
| Pulse | Wearable sensor or patient self-report |
| Environment | Private, quiet space for both parties |

### 7.2 Limitations of Remote Assessment

| Examination | Remote Feasibility |
|-------------|-------------------|
| Inspection | High (with good video) |
| Tongue Diagnosis | Moderate (with proper photo) |
| Auscultation | Moderate (audio quality dependent) |
| Inquiry | High |
| Pulse Palpation | Low (sensor required) |
| Abdominal Palpation | Not possible |

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association

## P.3 Protocol Cross-References

The protocol defined here carries the data formats from Phase 1 and the API
operations from Phase 2 across trust boundaries. Phase 4 describes how the
protocol composes with adjacent infrastructure.

### P.3.1 Transport Bindings

| Binding | Default Port | Use |
|---------|-------------:|-----|
| HTTP/2 + TLS 1.3 | 443 | Public, request-response |
| HTTP/3 (QUIC) | 443 | Mobile, lossy networks |
| gRPC | 443 | Service-to-service |
| MQTT 5.0 | 8883 | Constrained / IoT devices |
| AMQP 0-9-1 | 5671 | Backplane / event streams |

### P.3.2 Message Envelope

Every protocol message carries a small envelope independent of payload:

```
+----------------+------------------+--------------------+
| message_id     | UUIDv4           | RFC 4122           |
| trace_id       | 16-byte hex      | W3C Trace Context  |
| span_id        | 8-byte hex       | W3C Trace Context  |
| origin_node    | DNS name or NIN  | RFC 1035           |
| issued_at      | RFC 3339         | UTC required       |
| ttl_seconds    | uint32           | 0 = no expiry      |
| content_type   | media type       | RFC 6838           |
| body           | opaque bytes     | per content_type   |
+----------------+------------------+--------------------+
```

### P.3.3 Reliability Model

The protocol provides at-least-once delivery by default. Receivers
deduplicate by `message_id`. Exactly-once semantics are achieved when both
peers participate in the idempotency contract from Phase 2 §P.2.3.

### P.3.4 Backpressure

Senders MUST honour HTTP `Retry-After`, gRPC `RESOURCE_EXHAUSTED`, or MQTT
flow-control packets. The recommended back-off is full jitter exponential with
cap 30 s and cumulative cap 5 min.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of traditional-medicine so that conformance claims at any
Phase remain unambiguous.*

