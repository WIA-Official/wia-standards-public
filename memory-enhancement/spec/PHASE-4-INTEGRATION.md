# WIA-memory-enhancement PHASE 4 — Integration Specification

**Standard:** WIA-memory-enhancement
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how WIA-memory-enhancement
integrates with adjacent regulatory, clinical, and
research-data systems: clinical-trials registries
(ClinicalTrials.gov, EU CTIS, WHO ICTRP, KCT, jRCT),
electronic health records via HL7 FHIR R5, regulator
pharmacovigilance pipelines (FDA FAERS, EMA EudraVigilance,
PMDA J-AER, MFDS K-PV), the IDMP product dictionary,
neuroimaging biobanks (BIDS, OpenNeuro), and ethics-
oversight networks. It also specifies the operational
binding to WIA companion standards so the longitudinal
memory-enhancement record is usable across the lifecycle
from protocol authorship to long-term post-market
surveillance.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — RESTful API, Bulk Data Access, IPS, US Core, KR Core
- HL7 SMART App Launch 2.0
- HL7 v2.x ADT, ORU; HL7 CDA R2 — for legacy hospital systems
- IHE PCC profiles (Patient Care Coordination)
- IDMP — ISO 11615 / 11616 / 11238 / 11239 / 11240
- ICH E2B (R3) — ICSR transmission profile
- ICH E2D — Post-approval safety data management
- ICH M11 — Clinical electronic Structured Harmonised Protocol (CeSHarP)
- WHO ICTRP — International Clinical Trials Registry Platform
- ClinicalTrials.gov PRS, EU CTIS / CTR public portal, jRCT, KCT (CRIS)
- BIDS 1.9 — Brain Imaging Data Structure (EEG/iEEG/MEG/MRI extensions)
- OpenNeuro — neuroimaging dataset repository (NIH support)
- IEEE 11073-10406 (EEG-PHD), IEEE 11073-10101 nomenclature
- ISO 8601 (date / time), ISO 3166 (country / region), BCP 47 (language)
- IETF RFC 9110 (HTTP), RFC 7515 (JWS), RFC 8259 (JSON), RFC 8785 (JCS)
- ETSI EN 319 411-2 (qualified electronic signatures, eIDAS) and ETSI EN 319 522 (registered electronic delivery)

---

## §1 Clinical-trials registries

Every protocol that runs sessions under this standard
must be registered prior to first subject enrolment in
at least one WHO ICTRP primary registry.

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

Registry binding is recorded in the protocol-version
record (PHASE 1 §3 `protocolRef`). Updates to the
registry record (status changes, results posting) emit
a registry-event in the audit chain (PHASE 3 §8) so
inspectors can correlate registry public state to
sponsor private state.

## §2 EHR integration via FHIR R5

Memory-enhancement records bind to FHIR R5 resources:

| WIA record           | FHIR R5 resource                                 |
|----------------------|--------------------------------------------------|
| Subject              | ResearchSubject + Patient (SMART scope-gated)    |
| Intervention (drug)  | MedicationStatement + MedicationRequest           |
| Intervention (dev.)  | Procedure + Device + DeviceUseStatement           |
| Session              | Procedure (subType = WIA-ME session)              |
| Outcome              | Observation (LOINC code per instrument)           |
| Adverse event        | AdverseEvent                                      |
| Consent              | Consent                                           |
| Ethics approval      | ResearchStudy.protocol.relatedArtifact            |

LOINC codes for cognitive instruments are pulled from
the published LOINC release (e.g. 72172-0 MoCA, 72133-2
MMSE) via Regenstrief; rater-administered scores carry
the appropriate LOINC scale-version code.

## §3 Pharmacovigilance pipeline

Serious adverse events transmit as ICH E2B (R3) ICSRs:

```
WIA-AdverseEvent  →  E2B(R3) ICSR  →  Regulator gateway
```

Regulator gateway endpoints (operationally controlled):

| Regulator | Gateway                                          |
|-----------|--------------------------------------------------|
| FDA       | FDA ESG (Electronic Submissions Gateway), 3500A  |
| EMA       | EudraVigilance EVDAS / EVWEB                     |
| PMDA      | J-AER                                            |
| MFDS      | K-PV                                             |
| Health Canada | CIOMS-I + Canada Vigilance                  |
| TGA       | Australian Black Triangle scheme + DAEN          |

ICSR identifier format follows ICH E2B (R3) `safetyReport
Id` rules (sender + sequence). Sponsors file an annual
DSUR (Development Safety Update Report) covering the
protocol; periodic post-market PSUR / PBRER cycles cover
approved products per ICH E2C (R2).

## §4 Product identification (IDMP)

Drug interventions resolve to IDMP records:

| IDMP standard | Subject                                   |
|---------------|-------------------------------------------|
| ISO 11615     | Medicinal Product Identification (MPID)   |
| ISO 11616     | Pharmaceutical Product Identification (PhPID) |
| ISO 11238     | Substance / referential / chemical / mineral |
| ISO 11239     | Pharmaceutical dose forms / units / routes |
| ISO 11240     | Units of measurement                      |

IDMP identifiers replace legacy product strings in the
intervention `productRef`; this allows post-market
linkage to FAERS / EVDAS analyses without manual
reconciliation.

## §5 Neuroimaging and physiological data

Subjects whose protocol includes neuroimaging or EEG
biomarker capture have those data exported in BIDS 1.9
form and indexed at the sponsor's biobank. EEG capture
follows IEEE 11073-10406 (PHD profile) for live binding
during the session and EDF / EDF+ for archival.

Where the protocol allows, de-identified BIDS datasets
can be published to OpenNeuro under a CC0 / CC-BY
licence per NIH data-sharing policy. Re-identification
risk is evaluated using k-anonymity and l-diversity
analyses on the dataset's metadata before publication.

## §6 Ethics-oversight network

IRB / IEC bodies are recorded with their public
identifier (OHRP IRB Number for US bodies, EU EC number
for European RECs, KAIRB for Korean bodies, PMDA-
registered IRBs for Japan). The ethics-approval record
links to the IRB body record and to the protocol-version
record, allowing reverse audit ("which protocols did
this IRB approve?") and forward audit ("which IRB
approved this session's protocol-version?").

## §7 Cross-domain WIA bindings

| Companion standard          | Binding purpose                                |
|-----------------------------|------------------------------------------------|
| WIA-medical-imaging         | fMRI / MEG biomarker capture                   |
| WIA-medical-data-privacy    | special-category data handling, lawful basis   |
| WIA-clinical-decision-support | outcome interpretation, clinician dashboards |
| WIA-emergency-medical-data  | session-emergency escalation, notification     |
| WIA-bionic-eye / WIA-prosthetic-control | BCI bridging where the same       |
|                             | device platform delivers both                  |
| WIA-explainable-ai          | when SaMD provides cognitive-training planning |
| WIA-medical-iot             | device telemetry path during the session        |

Each binding identifies the companion standard's PHASE
that the binding consumes (e.g. WIA-medical-data-
privacy PHASE 1 record schema, PHASE 3 lawful-basis
declaration).

## §8 Long-term archival

Protocol records, session records, outcome records,
consent records, AE records, and audit-chain events
are retained for the regulator-mandated minimum:

| Regulator | Retention                                                  |
|-----------|------------------------------------------------------------|
| FDA (drug)  | 21 CFR §312.62(c) — 2 years post-marketing or post-discontinuation |
| FDA (device)| 21 CFR §812.140 — 2 years after the later of product life-cycle end or last shipment |
| EMA       | EU CTR Annex I §38 — 25 years for trial master file        |
| PMDA      | 5 years post-licence per Pharmaceutical Affairs Law         |
| MFDS      | 3 years per Pharmaceutical Affairs Act §7                   |

Sponsors retain on the longest applicable clock and
record retention-end events in the audit chain on
disposal.

## §9 Data interoperability profiles

The standard exposes three reference profiles to ease
implementation:

1. **Clinical-Trial profile** — full ICH E6 (R3) GCP
   audit, IRB binding, ICSR pipeline; intended for
   sponsor-led interventional trials.
2. **Post-Market profile** — for approved products in
   real-world data pipelines; trims investigational-only
   fields and adds device-vigilance binding (EU MDR
   §87, FDA MDR 21 CFR Part 803).
3. **Research-Registry profile** — observational-study
   shape used by academic registries (e.g. NIA's NACC,
   UK Biobank cognitive sub-study); read-only ICSR
   binding (registry receives but does not transmit).

## §10 Conformance test suite

A reference test suite covers, at minimum:

- valid-consent gate before session-open
- expired-ethics-approval rejection
- AE serious-and-unexpected propagation to ICSR fixture endpoint
- session close blocked while open SAE pending
- BIDS export of session physiological data
- FHIR R5 export of subject + intervention + outcome
- IDMP resolution of drug intervention to MPID
- audit-chain hash continuity

The suite is published at `tests/conformance/` and
runs against any implementation claiming Core or Full
conformance.

## §11 Internationalisation

User-facing strings (consent forms, instrument
prompts, outcome interpretation summaries) carry the
BCP 47 language tag. Country-specific regulator paths
are resolved by the protocol's primary-registry country
code (ISO 3166-1 alpha-3). Time stamps carry the local
offset and a reference to the time source (NIST, KASI,
PTB) for forensic reconstruction.

## §12 Security and privacy posture

- Transport: TLS 1.3 (RFC 8446) with mutual TLS for
  sponsor-to-regulator transmissions; ETSI EN 319 522
  registered electronic delivery for jurisdictions that
  require it.
- Authentication: SMART on FHIR for clinical contexts;
  client_credentials with key attestation for sponsor
  systems.
- At-rest: AES-256-GCM with sponsor-controlled KMS;
  per-protocol key-wrapping per ISO/IEC 27002 §8.24.
- Access: role-segregated per ICH E6 (R3) — operator,
  rater, sponsor, monitor, regulator; rater independence
  enforced at the API layer (PHASE 2 §5).
- Audit: tamper-evident chain (PHASE 3 §8) exportable
  per ISO/IEC 27037 forensic-evidence guidance.
- Privacy: subject identifiers opaque; re-linkage table
  held by sponsor under regulator-approved DPIA per
  EU GDPR Art. 35 / ICO Code of Practice / KISA
  guidelines.

## §13 Operational metrics

Sponsors report (informationally) on the WIA registry:

- protocols active, suspended, terminated
- subjects enrolled, withdrawn, completed
- session count per intervention kind
- AE rate (per 1000 sessions) by severity
- regulator-clock compliance rate (proportion of SAEs
  reported within window)

## §14 Recovery and continuity

Sponsors maintain a continuity plan covering:

- API outage — local site capture; sync on reconnect
- regulator-gateway outage — queue ICSRs locally;
  transmit on reconnect; clock paused only on regulator-
  declared outage
- biobank outage — local SHA-256 manifest of pending
  exports; replay on recovery

## Annex A — Worked end-to-end example (informative)

A multi-site Phase 2b trial of an investigational
hippocampal prosthesis runs at six US academic medical
centres. Each centre's IRB issues an approval recorded
under §6 of PHASE 1. Subjects consent via electronic
signature per 21 CFR Part 11. Sessions execute under
PHASE 3 §4. Per-session outcomes capture as FHIR R5
Observations bound to LOINC. A serious unexpected AE
at site 3 transmits as an ICSR via FDA ESG within 5
days; the IRB at site 3 issues a notification to the
sponsor and pauses new enrolment locally; the sponsor
DSMB reviews and clears site 3 to resume after one
week. End-of-study results are deposited at
ClinicalTrials.gov and to OpenNeuro (de-identified
BIDS) under a CC-BY licence.

## Annex B — Conformance disclosure

Implementations declare the registries they bind to,
the regulator gateways they support, the FHIR profile
versions they expose, and the IDMP catalogue version
they resolve against. Disclosure is published at the
standard's conformance directory and machine-readable
in the implementation's `/.well-known/wia-me-
conformance.json` document.

## Annex C — Versioning

PHASE 4 follows the standard's semantic-versioning
policy. Adding a new regulator gateway is a minor
revision; changing the ICSR binding format is a major
revision.
