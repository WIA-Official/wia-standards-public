# WIA-physical-enhancement PHASE 4 — Integration Specification

**Standard:** WIA-physical-enhancement
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how WIA-physical-enhancement
integrates with adjacent regulatory, sport-governance,
clinical, and research-data systems: clinical-trials
registries, EHR systems via FHIR R5, regulator device
and pharmacovigilance pipelines, the WADA ADAMS data-
exchange profile, IPC classification systems, IF medical
commissions, paralympic and Olympic data flows for
adaptive-sport contexts, occupational-safety
authorities, and the IDMP product dictionary. It also
specifies the operational binding to companion WIA
standards.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Procedure, Observation, MedicationStatement, DeviceUseStatement, AdverseEvent, ResearchSubject
- HL7 SMART App Launch 2.0; HL7 v2.x ADT, ORU; HL7 CDA R2 (legacy)
- IDMP — ISO 11615 / 11616 / 11238 / 11239 / 11240
- ICH E2B (R3) — ICSR transmission profile
- ICH M11 — Clinical electronic Structured Harmonised Protocol
- WHO ICTRP — International Clinical Trials Registry Platform
- ClinicalTrials.gov, EU CTIS, EudraCT, jRCT, KCT, ANZCTR, ChiCTR, ISRCTN
- WADA Code — current edition; ADAMS data-exchange profile
- IPC Athlete Classification Code; IPC Sport Technical Rules per discipline
- IF medical-commission rules (per International Federation, e.g. World Athletics, FIBA, FINA, FEI)
- ISO 13482 — personal-care robots safety (exoskeletons)
- IEEE 11073-10101 / IEEE 11073-10406 — health-device nomenclature
- US OSHA Process Safety Management; EU OSHA; ILO Occupational Safety
- 21 CFR Part 11; 21 CFR §312.32 (IND safety); 21 CFR §812.150 (IDE)
- EU CTR (Reg 536/2014); EU MDR (2017/745) — medical-device vigilance
- ISO 8601, ISO 3166, BCP 47
- IETF RFC 9110 (HTTP), RFC 7515 (JWS), RFC 8259 (JSON), RFC 8785 (JCS)

---

## §1 Clinical-trials registries

Every clinical or research protocol registers in at
least one WHO ICTRP primary registry prior to first
subject enrolment.

| Registry            | Operator         | Identifier prefix |
|---------------------|------------------|-------------------|
| ClinicalTrials.gov  | NIH NLM          | NCT               |
| EU CTIS / CTR       | EMA              | EU-CT             |
| EU CTR (legacy)     | EMA              | EudraCT           |
| jRCT                | PMDA             | jRCT              |
| KCT (CRIS)          | KDCA             | KCT               |
| ANZCTR              | Australia / NZ   | ACTRN             |
| ChiCTR              | China            | ChiCTR            |
| ISRCTN              | UK               | ISRCTN            |

## §2 EHR integration via FHIR R5

| WIA record          | FHIR R5 resource                                   |
|---------------------|----------------------------------------------------|
| Subject             | ResearchSubject + Patient                          |
| Intervention (drug) | MedicationStatement + MedicationRequest            |
| Intervention (dev.) | DeviceUseStatement + Procedure + Device            |
| Session             | Procedure (subType = WIA-PE session)               |
| Performance measure | Observation (LOINC for instrument)                 |
| Adverse event       | AdverseEvent                                       |
| Consent             | Consent                                            |
| Ethics approval     | ResearchStudy.protocol.relatedArtifact             |

LOINC codes for performance instruments are pulled from
the published LOINC release (e.g. 89200-7 6-Minute Walk
Test distance, 75328-5 Berg Balance Scale total).

## §3 Device-vigilance pipeline

Device-related adverse events transmit through the
relevant regulator vigilance pipeline:

| Regulator | Device-vigilance route                          |
|-----------|-------------------------------------------------|
| FDA       | MedWatch 3500A; MAUDE for post-market reports   |
| EU        | EUDAMED — UDI / vigilance / clinical-evidence    |
| MFDS      | MD-RM (Medical Device Risk Management) portal    |
| PMDA      | J-AER (medical device adverse-event report)      |
| Health Canada | Mandatory Problem Reports                    |
| TGA       | Australian Black Triangle scheme + DAEN         |

Device events follow EU MDR §87 vigilance procedure for
EU markets; FDA MDR (21 CFR Part 803) governs US
post-market reports.

## §4 Pharmacovigilance pipeline

Medicinal-product adverse events transmit as ICH E2B
(R3) ICSRs through:

| Regulator | Gateway                                          |
|-----------|--------------------------------------------------|
| FDA       | FDA ESG (Electronic Submissions Gateway), 3500A  |
| EMA       | EudraVigilance EVDAS / EVWEB                     |
| PMDA      | J-AER                                            |
| MFDS      | K-PV                                             |
| Health Canada | CIOMS-I + Canada Vigilance                  |
| TGA       | DAEN                                             |

Sponsors maintain an annual DSUR per ICH E2F and
periodic post-market PSUR / PBRER per ICH E2C (R2)
where the intervention is an approved product.

## §5 Sport-governance integration

For sport-context interventions the standard binds to:

| Body / system        | Binding                                       |
|----------------------|-----------------------------------------------|
| WADA ADAMS           | TUE submission, whereabouts, doping-control   |
|                      | events, sample chain-of-custody               |
| IF medical commissions | session ↔ medical-commission notifications  |
|                      | for in-competition AE                         |
| IPC classification   | adaptive-class records and re-classification  |
|                      | events                                        |
| WADA-accredited labs | sample analysis result return                 |
| NADO                 | national-level testing programmes             |

ADAMS submissions follow the WADA ADAMS data-exchange
profile; whereabouts entries follow ISTI Annex I.

## §6 Product identification (IDMP)

Drug interventions resolve to IDMP records:

| IDMP standard | Subject                                     |
|---------------|---------------------------------------------|
| ISO 11615     | Medicinal Product Identification (MPID)     |
| ISO 11616     | Pharmaceutical Product Identification        |
| ISO 11238     | Substance / referential / chemical          |
| ISO 11239     | Pharmaceutical dose forms / units / routes  |
| ISO 11240     | Units of measurement                        |

IDMP identifiers replace legacy product strings on
intervention records, allowing post-market linkage to
FAERS / EVDAS analyses without manual reconciliation.

## §7 Adaptive-sport / paralympic integration

For adaptive-sport contexts:

- IPC athlete identifier and classification record
- IPC competition entry and result records
- IPC-classification protest workflow
- continental committee (e.g. Asian Paralympic Committee,
  European Paralympic Committee) athlete records

The standard's intervention record cross-references the
adaptive-class so a re-classification event triggers a
review of the active intervention's continued
permissibility.

## §8 Occupational-safety integration

For occupational contexts:

- exoskeleton operator training records bind to OSHA /
  EU OSHA / ILO occupational standards
- post-incident AE records mirror to OSHA 300 / 301
  forms (US) or RIDDOR (UK) or analogous national
  registers
- ergonomic-assessment artefacts bind to ISO 6385
  (workplace ergonomic principles) and EN 1005-2 /
  ISO 11226 (force / posture limits)

## §9 Cross-domain WIA bindings

| Companion standard           | Binding purpose                              |
|------------------------------|----------------------------------------------|
| WIA-rehabilitation-device    | orthosis / prosthetic record cross-link      |
| WIA-medical-data-privacy     | special-category data, lawful basis          |
| WIA-clinical-decision-support| performance / outcome interpretation          |
| WIA-emergency-medical-data   | session-emergency escalation                 |
| WIA-medical-iot              | session telemetry pipeline                   |
| WIA-emotion-ai               | affective-state correlated training (where   |
|                              | bound to performance state)                  |

Each binding identifies the consumed PHASE of the
companion standard.

## §10 Long-term archival

| Regulator / body | Retention                                            |
|------------------|------------------------------------------------------|
| FDA (drug)       | 21 CFR §312.62(c) — 2 years post-marketing /         |
|                  | post-discontinuation                                  |
| FDA (device)     | 21 CFR §812.140 — 2 years after later of life-cycle  |
|                  | end or last shipment                                 |
| EMA              | EU CTR Annex I §38 — 25 years for trial master file  |
| WADA             | Article 17 of the WADA Code — 10 years for           |
|                  | doping-control samples per current rules             |
| PMDA             | 5 years post-licence                                 |
| MFDS             | 3 years per Pharmaceutical Affairs Act §7            |

## §11 Reproducibility and conformance test suite

The reference test suite covers, at minimum:

- consent gate before session-open (clinical /
  occupational / sport contexts)
- expired-ethics rejection (clinical / occupational)
- TUE expiry rejection (sport context)
- ICSR propagation on serious unrelated AE
- session-close blocked while open SAE pending
- BIDS export of session physiological-log
- FHIR R5 export of subject + intervention + outcome
- IDMP resolution of drug intervention to MPID
- audit-chain hash continuity
- ADAMS submission cross-walk for sport contexts
- IPC classification event recorded on intervention
  permissibility review

## §12 Internationalisation

User-facing strings (consent forms, instrument prompts,
result interpretation) carry the BCP 47 language tag.
Country-specific regulator paths are resolved by the
study's primary-registry country code (ISO 3166-1
alpha-3).

## §13 Security and privacy posture

- Transport: TLS 1.3 with mutual TLS for sponsor /
  regulator / IF transmissions
- Authentication: SMART on FHIR (clinical); IF / NADO
  mutual-TLS profile (sport); client_credentials with
  key attestation (occupational)
- At-rest: AES-256-GCM with sponsor-controlled KMS;
  per-protocol key wrapping per ISO/IEC 27002 §8.24
- Audit: tamper-evident chain (PHASE 3 §10) exportable
  per ISO/IEC 27037 forensic-evidence guidance
- Privacy: subject identifiers opaque; sport-context
  identifiers separate from clinical identifiers; re-
  linkage table held by sponsor under regulator-
  approved DPIA

## §14 Operational metrics

Sponsors / IFs report (informationally) on the WIA
registry:

- subjects enrolled / withdrawn / completed
- sessions per intervention kind
- AE rate (per 1000 sessions) by severity
- regulator-clock compliance proportion
- sport-context: TUEs granted / pending / refused;
  whereabouts compliance rate

## §15 Recovery and continuity

- API outage — local capture, sync on reconnect
- regulator-gateway outage — queue ICSRs locally;
  transmit on reconnect; clock paused only on regulator-
  declared outage
- ADAMS outage — queue TUE / whereabouts events; replay
  on recovery
- biobank / sample outage — local SHA-256 manifest of
  pending exports; replay on recovery

## Annex A — Worked end-to-end example (informative)

A multi-site occupational exoskeleton deployment in a
warehouse-logistics operator trains 600 operators over
six months. Each operator consents per the occupational
SOP, the deployment is registered as ClinicalTrials.gov
NCT-XXXX, and per-shift telemetry uploads through
PHASE 2 §4. A serious near-miss event (operator slip
under load) generates an AE record; the incident
notifies OSHA 300 within 7 days. The exoskeleton
firmware updates under IEC 62304 lifecycle; the
software-version event extends the audit chain. End-of-
study aggregated results publish to ClinicalTrials.gov
and to the relevant occupational-safety registry.

## Annex B — Conformance disclosure

Implementations declare the registries bound to, the
regulator and IF gateways supported, the FHIR profile
versions exposed, the IDMP catalogue version, and the
ADAMS data-exchange profile version. Disclosure is
machine-readable at `/.well-known/wia-pe-conformance.json`.

## Annex C — Versioning

Adding a new regulator gateway is minor; changing the
ICSR or ADAMS binding format is major.
