# WIA-TRADITIONAL-MEDICINE: Phase 4 - Integration

## 弘益人間 (Benefit All Humanity)

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-01

---

## 1. Overview

Phase 4 defines integration standards for connecting traditional medicine systems with modern healthcare infrastructure, including electronic health records (EHR), laboratory information systems, pharmacovigilance databases, and global health data platforms.

---

## 2. HL7 FHIR Integration

### 2.1 FHIR Resource Mappings

| Traditional Medicine Concept | FHIR Resource | Profile |
|------------------------------|---------------|---------|
| Pattern Diagnosis | Condition | TMPatternDiagnosis |
| Constitution Assessment | Observation | TMConstitutionObservation |
| Tongue Findings | Observation | TMTongueDiagnosis |
| Pulse Findings | Observation | TMPulseDiagnosis |
| Herbal Prescription | MedicationRequest | TMHerbalPrescription |
| Acupuncture Treatment | Procedure | TMAcupunctureProcedure |
| Four Examinations | DiagnosticReport | TMDiagnosticReport |

### 2.2 FHIR Profile Examples

#### Pattern Diagnosis (Condition)

```json
{
  "resourceType": "Condition",
  "id": "tm-pattern-example",
  "meta": {
    "profile": [
      "http://wia.live/fhir/StructureDefinition/TMPatternDiagnosis"
    ]
  },
  "clinicalStatus": {
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/condition-clinical",
      "code": "active"
    }]
  },
  "verificationStatus": {
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/condition-ver-status",
      "code": "confirmed"
    }]
  },
  "category": [{
    "coding": [{
      "system": "http://wia.live/fhir/CodeSystem/tm-category",
      "code": "pattern-diagnosis",
      "display": "Traditional Medicine Pattern Diagnosis"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://id.who.int/icd/entity",
      "code": "TM1:SF32.2",
      "display": "Spleen qi deficiency with dampness"
    }],
    "text": "脾氣虛夾濕 / 비기허협습"
  },
  "subject": {
    "reference": "Patient/example"
  },
  "recordedDate": "2025-01-15",
  "recorder": {
    "reference": "Practitioner/tm-practitioner-001"
  },
  "evidence": [{
    "detail": [{
      "reference": "Observation/tongue-diagnosis-001"
    }, {
      "reference": "Observation/pulse-diagnosis-001"
    }]
  }]
}
```

#### Herbal Prescription (MedicationRequest)

```json
{
  "resourceType": "MedicationRequest",
  "id": "herbal-prescription-example",
  "meta": {
    "profile": [
      "http://wia.live/fhir/StructureDefinition/TMHerbalPrescription"
    ]
  },
  "status": "active",
  "intent": "order",
  "medicationCodeableConcept": {
    "coding": [{
      "system": "http://wia.live/fhir/CodeSystem/tm-formulas",
      "code": "LIU-JUN-ZI-TANG",
      "display": "六君子湯 (Liu Jun Zi Tang)"
    }],
    "text": "Six Gentlemen Decoction"
  },
  "subject": {
    "reference": "Patient/example"
  },
  "requester": {
    "reference": "Practitioner/tm-practitioner-001"
  },
  "reasonReference": [{
    "reference": "Condition/tm-pattern-example"
  }],
  "dosageInstruction": [{
    "text": "Decoct and take warm, twice daily after meals",
    "timing": {
      "repeat": {
        "frequency": 2,
        "period": 1,
        "periodUnit": "d"
      }
    }
  }],
  "dispenseRequest": {
    "quantity": {
      "value": 7,
      "unit": "doses"
    }
  }
}
```

---

## 3. ICD-11 Traditional Medicine Integration

### 3.1 ICD-11 TM Module Structure

```
ICD-11 Supplementary Chapter
└── Traditional Medicine Conditions (TM1)
    ├── TM1:S - Patterns/Syndromes
    │   ├── TM1:SA - Yin Patterns
    │   ├── TM1:SB - Yang Patterns
    │   ├── TM1:SC - Qi Patterns
    │   ├── TM1:SD - Blood Patterns
    │   ├── TM1:SE - Qi Movement Patterns
    │   ├── TM1:SF - Blood Movement Patterns
    │   ├── TM1:SG - Phlegm Patterns
    │   ├── TM1:SH - Heat Patterns
    │   ├── TM1:SI - Cold Patterns
    │   └── TM1:SJ - Dampness Patterns
    └── TM1:D - Disorders by Body System
```

### 3.2 Dual Coding Guidelines

Patients should receive both:
- Primary ICD-11 disease code (Western diagnosis)
- Supplementary TM1 pattern code (Traditional diagnosis)

Example:
```
Primary: 5A11 - Type 2 diabetes mellitus
Pattern: TM1:SA21.0 - Yin deficiency with heat pattern
         TM1:SC30.1 - Spleen qi deficiency pattern
```

---

## 4. WHO TCIM Dashboard Integration

### 4.1 Data Exchange Protocol

| Data Type | Format | Frequency |
|-----------|--------|-----------|
| Policy Updates | JSON | Real-time |
| Practitioner Registry | FHIR Practitioner | Daily sync |
| Adverse Events | ICH E2B(R3) | Real-time |
| Utilization Statistics | HL7 ADT | Monthly aggregate |
| Research Outcomes | CDISC ODM | Per study |

### 4.2 Reporting Requirements

```json
{
  "tcim_report": {
    "country_code": "KR",
    "reporting_period": "2025-Q1",
    "traditional_medicine_type": "korean_medicine",
    "metrics": {
      "registered_practitioners": 25000,
      "patient_encounters": 5000000,
      "herbal_prescriptions": 3500000,
      "acupuncture_procedures": 8000000,
      "adverse_events_reported": 45
    }
  }
}
```

---

## 5. Drug Interaction Database Integration

### 5.1 Connected Databases

| Database | Data Type | Update Frequency |
|----------|-----------|------------------|
| DrugBank | Drug-herb interactions | Weekly |
| Natural Medicines | Evidence reviews | Monthly |
| PharmGKB | Pharmacogenomics | Weekly |
| KEGG | Pathways, compounds | Monthly |
| PubChem | Chemical structures | Real-time |
| ChEMBL | Bioactivity data | Quarterly |

### 5.2 Interaction Query Protocol

```
REQUEST:
  Herbs: [당귀, 천궁, 백작약]
  Medications: [Warfarin, Metformin, Amlodipine]

PROCESS:
  1. Query herb compound database
  2. Map to molecular targets
  3. Cross-reference with drug targets
  4. Check pharmacokinetic interactions (CYP450)
  5. Check pharmacodynamic interactions
  6. Aggregate evidence and severity

RESPONSE:
  Interactions found: 2
  Major: 1 (당귀 ↔ Warfarin)
  Moderate: 1 (인삼 ↔ Warfarin)
  Overall risk: HIGH
```

---

## 6. Laboratory System Integration

### 6.1 Constitutional Biomarker Panel

| Biomarker | Constitution Correlation |
|-----------|-------------------------|
| FTO gene variants | Phlegm-dampness, Kapha |
| PPARG polymorphisms | Metabolic constitution |
| Inflammatory markers (CRP, IL-6) | Heat/cold patterns |
| Thyroid panel | Yang deficiency assessment |
| Cortisol rhythm | Yin deficiency assessment |
| Microbiome diversity | Overall constitutional balance |

### 6.2 LOINC Codes for TM Tests

| Test | LOINC Code | Description |
|------|------------|-------------|
| Constitution questionnaire | 99001-0 | CCMQ Score |
| Tongue color assessment | 99002-8 | AI-assisted analysis |
| Pulse waveform | 99003-6 | Sensor-based pulse |

---

## 7. Telemedicine Platform Integration

### 7.1 Integration Requirements

| Component | Protocol | Standard |
|-----------|----------|----------|
| Video Consultation | WebRTC | SRTP encryption |
| Document Sharing | FHIR DocumentReference | PDF/A, DICOM |
| Prescription | FHIR MedicationRequest | Electronic signature |
| Scheduling | FHIR Appointment | iCalendar sync |

### 7.2 Remote Monitoring Devices

| Device | Data Type | Integration |
|--------|-----------|-------------|
| Pulse sensor wearable | Pulse waveform | Bluetooth → FHIR Observation |
| Tongue camera | High-res image | Cloud upload → AI analysis |
| Smart scale | Weight, body composition | FHIR Observation |
| Activity tracker | Steps, sleep | Apple Health / Google Fit |

---

## 8. Privacy and Security

### 8.1 Compliance Requirements

| Regulation | Jurisdiction | Requirements |
|------------|--------------|--------------|
| GDPR | European Union | Consent, data portability, right to erasure |
| HIPAA | United States | PHI protection, BAA requirements |
| PIPA | South Korea | Personal information protection |
| PIPL | China | Data localization, consent |

### 8.2 Data Security Standards

| Requirement | Standard |
|-------------|----------|
| Encryption at rest | AES-256 |
| Encryption in transit | TLS 1.3 |
| Authentication | OAuth 2.0, OpenID Connect |
| Access control | RBAC with ABAC extensions |
| Audit logging | FHIR AuditEvent |

### 8.3 Traditional Knowledge Protection

| Principle | Implementation |
|-----------|----------------|
| Nagoya Protocol compliance | Benefit sharing agreements |
| Indigenous knowledge protection | Access restrictions on sacred knowledge |
| Community consent | Community-level consent for traditional formulas |
| Attribution | Source community documentation |

---

## 9. Implementation Roadmap

### Phase 4.1: Foundation (Months 1-6)
- Implement FHIR profiles
- Connect to ICD-11 TM module
- Establish security infrastructure

### Phase 4.2: Expansion (Months 7-12)
- Integrate with major EHR vendors
- Connect to pharmacovigilance systems
- Deploy multi-omics integration

### Phase 4.3: Global Rollout (Months 13-24)
- WHO TCIM dashboard connection
- International data exchange agreements
- Full telemedicine integration

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association

## P.4 Integration Cross-References

This Phase describes how the data formats (Phase 1), API surface (Phase 2),
and protocol layer (Phase 3) compose with adjacent infrastructure to form a
production deployment.

### P.4.1 Deployment Topologies

| Topology | When to Use | Trade-off |
|----------|------------|-----------|
| Single-region active-passive | Predictable latency, single-region users | Cold standby cost |
| Multi-region active-active | Global users, regional sovereignty | Conflict resolution complexity |
| Edge fan-out | Low latency at the edge, central system of record | Cache coherence |
| Air-gapped enclave | Regulatory / national security domains | Manual reconciliation |

### P.4.2 Dependency Inventory

Every implementation MUST publish a Software Bill of Materials (SBOM) in
SPDX 2.3 or CycloneDX 1.5 format covering: (a) direct runtime dependencies,
(b) transitive dependencies pinned to specific versions, (c) base container
images, (d) cryptographic libraries.

### P.4.3 Operational Readiness Checklist

- [ ] Health check endpoint returns 200 within 1 s p99
- [ ] Metrics exposed in Prometheus or OTLP format
- [ ] Logs are structured JSON with correlation IDs
- [ ] Traces use W3C Trace Context headers end-to-end
- [ ] Backups verified by quarterly restore drill
- [ ] Runbook published and indexed
- [ ] Disaster recovery RTO / RPO documented
- [ ] On-call rotation defined and acknowledged

### P.4.4 Migration Pathways

Adopters migrating from legacy systems should follow the staged pattern:
(1) shadow read, (2) shadow write, (3) primary write with legacy fallback,
(4) primary read, (5) legacy decommission. Each stage runs for at least one
business cycle before the next.


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

