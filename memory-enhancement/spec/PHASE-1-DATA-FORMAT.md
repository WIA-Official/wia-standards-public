# WIA-memory-enhancement PHASE 1 — Data Format Specification

**Standard:** WIA-memory-enhancement
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for memory-
enhancement interventions covering pharmacological agents,
non-invasive neuromodulation (tDCS, tACS, TMS), invasive
neuroprosthetics (hippocampal prosthesis, deep-brain
stimulation), and digital cognitive-training programmes.
The records bind every intervention to a documented
ethics approval, an informed-consent instance, the
subject's neuropsychological baseline, the per-session
outcome, the device or pharmacovigilance binding, and
the adverse-event chain so regulators, investigators,
and downstream registries can trace each enhancement
event end-to-end.

References (CITATION-POLICY ALLOW only):
- ICH E6 (R3) Good Clinical Practice
- ICH E2A / E2B (R3) Pharmacovigilance and ICSR
- ISO 14155:2020 Clinical investigation of medical devices for human subjects
- ISO 14971:2019 Application of risk management to medical devices
- ISO/IEC 27001:2022, ISO/IEC 27701:2019 (privacy extension)
- IEC 60601-1, IEC 60601-2-26 (electroencephalographs), IEC 60601-1-2 EMC
- IEC 62366-1:2015 Usability engineering for medical devices
- IEEE 11073-10101 / IEEE 11073-10406 (EEG personal-health-device profile)
- HL7 FHIR R5 (Procedure, MedicationStatement, Observation, ResearchSubject, AdverseEvent)
- Brain Imaging Data Structure (BIDS) 1.9 — EEG, iEEG, MEG extensions
- European Data Format (EDF / EDF+) — physiological recording exchange
- ICD-11 MMS — Chapter 06 (Mental, behavioural and neurodevelopmental disorders)
- Declaration of Helsinki (2013, 2024 revision) — ethical principles for medical research
- 45 CFR 46 Common Rule — Protection of Human Subjects (US)
- IETF RFC 8259 (JSON), RFC 8785 (JSON Canonicalisation Scheme), RFC 7515 (JWS), RFC 4122 (UUID)

---

## §1 Scope

This PHASE applies to interventions that aim to restore,
sustain, or augment human declarative, working, or
procedural memory. The intervention may be an approved
medicinal product, an investigational drug under an IND
or CTA dossier, an FDA Class II/III neurological device,
a CE-marked medical device under EU MDR, an
investigational BCI under an IDE, or a digital therapeutic
indexed in a national catalogue (FDA Pre-Cert successor,
MFDS Digital Therapeutics, BfArM DiGA, NICE ESF).

In scope: subject record, intervention record, session
record, outcome record, neural-data record, consent
record, ethics-approval record, adverse-event record,
device-binding record, and the cross-references binding
each intervention to its protocol, sponsor, and
pharmacovigilance dossier.

Out of scope: cosmetic memory-related claims that are
not classified as medical interventions in any
regulator's catalogue (handled by general consumer-
product and advertising standards) and military or
intelligence-use cognitive operations (governed by
sovereign defence regimes).

## §2 Subject record

Every enrolled subject carries:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `subjectRef`         | UUID (RFC 4122) opaque to investigators         |
| `pseudonymRef`       | study-local code, never the legal name          |
| `birthYear`          | year only; full date never carried              |
| `sex`                | ISO/IEC 5218 (1=male, 2=female, 9=N/A)          |
| `consentRef`         | active consent record (this PHASE §6)           |
| `ethicsApprovalRef`  | IRB / IEC / REC approval (this PHASE §7)        |
| `enrolmentDate`      | ISO 8601 date                                   |
| `clinicalStatus`     | code from ICD-11 MMS (e.g. 6D71 mild cognitive  |
|                      | impairment, 8A20 Alzheimer's disease)           |
| `eligibilityFlag`    | `meets-protocol`, `screen-fail`, `withdrawn`    |

`subjectRef` is the only invariant identifier; the
pseudonym is rotated on protocol-amendment if the IRB
requires unblinding, and re-linkage is held by the
sponsor under ICH E6 (R3) §5.5.

## §3 Intervention record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `interventionRef`    | URI per §2.4                                    |
| `kind`               | `pharmacological`, `tDCS`, `tACS`, `TMS`,       |
|                      | `DBS`, `hippocampal-prosthesis`, `BCI-recall`,  |
|                      | `digital-training`                              |
| `productRef`         | for drugs: SPL set-id (DailyMed) or EMA dossier;|
|                      | for devices: UDI-DI per IMDRF UDI               |
| `protocolRef`        | ClinicalTrials.gov NCT id or EudraCT or KCT     |
| `armRef`             | active / sham / placebo / open-label            |
| `doseSchedule`       | for drugs: ATC code + dose per WHOCC;           |
|                      | for tDCS/tACS: current (mA), duration (s),      |
|                      | montage (electrode positions per 10–20 system); |
|                      | for TMS: stimulator model, intensity (% RMT),   |
|                      | frequency (Hz), pulses, target by MNI coords;   |
|                      | for DBS: pulse width (μs), frequency (Hz),      |
|                      | amplitude (mA or V), contact configuration      |
| `targetCognitive`    | declarative, working, episodic, procedural,     |
|                      | spatial, prospective                            |
| `riskClassification` | per ISO 14971 — low / medium / high / critical  |

Drug records additionally carry the active substance
INN (WHO INN) and the route (ATC route codes).

## §4 Session record

A session is the smallest replayable unit of intervention.

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `sessionRef`         | UUID, immutable across the session lifecycle    |
| `subjectRef`         | §2                                              |
| `interventionRef`    | §3                                              |
| `startTime`          | ISO 8601 timestamp with timezone offset         |
| `endTime`            | ISO 8601 timestamp with timezone offset         |
| `siteRef`            | facility code, ISO 3166-2 region, room id       |
| `operatorRef`        | credentialed clinician / operator UUID          |
| `deliveredDose`      | actual dose delivered (drug mg, electrical mC,  |
|                      | TMS pulse count, training-task minutes)         |
| `deviationRef`       | protocol-deviation record, if any               |
| `aeRef[]`            | adverse-event records this session triggered    |

Sessions reference the protocol-version under which they
ran; protocol amendments produce a new protocol-version
record and are not retroactive.

## §5 Outcome record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `outcomeRef`         | UUID                                            |
| `sessionRef`         | §4                                              |
| `instrument`         | MMSE, MoCA, RBANS, CDR-SOB, CANTAB-PAL,         |
|                      | RAVLT, WMS-IV, ADAS-Cog, FAQ; one per record    |
| `instrumentVersion`  | publisher version + language localisation       |
| `rawScore`           | numeric raw                                     |
| `standardisedScore`  | z-score with reference population               |
| `assessor`           | rater UUID; kept distinct from `operatorRef`    |
| `assessmentTime`     | ISO 8601 timestamp                              |
| `interpretation`     | `improved`, `stable`, `declined`, `non-eval`    |

Outcome records are bound to the session that preceded
them and to the baseline outcome at enrolment so per-
subject trajectories are reproducible.

## §6 Consent record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `consentRef`         | UUID                                            |
| `subjectRef`         | §2                                              |
| `protocolVersion`    | the version the subject consented to            |
| `documentRef`        | hash of the IRB-approved consent document       |
| `language`           | BCP 47 tag                                      |
| `signatureMethod`    | `wet-ink-scan`, `qualified-electronic`,         |
|                      | `advanced-electronic`, `verbal-witnessed`       |
| `witnessRef`         | for verbal consent (illiterate or vulnerable)   |
| `signedAt`           | ISO 8601                                        |
| `withdrawnAt`        | optional ISO 8601 — withdrawal terminates       |
|                      | further data collection but not data already    |
|                      | de-identified for prior analyses                |
| `proxyRef`           | for legally-authorised representative           |

Electronic-consent signatures conform to eIDAS (EU)
or 21 CFR Part 11 (US) according to jurisdiction.

## §7 Ethics-approval record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `ethicsApprovalRef`  | URI                                             |
| `bodyName`           | IRB / IEC / REC name                            |
| `bodyId`             | OHRP IRB number, EU EC number, KCDC IRB id      |
| `protocolRef`        | study protocol identifier                       |
| `decision`           | `approved`, `approved-with-conditions`,         |
|                      | `deferred`, `disapproved`                       |
| `decisionDate`       | ISO 8601                                        |
| `expiryDate`         | ISO 8601                                        |
| `conditions`         | text reproducing the IRB condition list         |

Approval cannot be assumed transitive across sites;
each site appears in this record as its own approval.

## §8 Adverse-event record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `aeRef`              | UUID                                            |
| `sessionRef`         | §4                                              |
| `subjectRef`         | §2                                              |
| `onsetTime`          | ISO 8601                                        |
| `meddraTerm`         | MedDRA Lowest-Level Term and Preferred Term     |
| `severity`           | mild / moderate / severe / life-threatening /   |
|                      | death (CTCAE 5.0 grade if oncology context)     |
| `causality`          | unrelated / unlikely / possible / probable /    |
|                      | definite (per ICH E2A)                          |
| `serious`            | boolean per ICH E2A SAE definition              |
| `outcome`            | recovered / recovering / not-recovered /        |
|                      | recovered-with-sequelae / fatal                 |
| `expedited`          | boolean — expedited reporting required          |

Serious adverse events bind to the ICSR/E2B(R3) message
exported in PHASE 4 §5 to FDA MedWatch, EMA EudraVigilance,
PMDA, and the MFDS.

## §9 Device-binding record

For non-pharmacological interventions:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `deviceRef`          | UUID                                            |
| `udiDi`              | UDI-DI per IMDRF UDI WG                         |
| `udiPi`              | production identifier (lot, serial, expiry)     |
| `manufacturerRef`    | basic UDI-DI or sponsor id                      |
| `riskClass`          | FDA I / II / III; EU MDR I / IIa / IIb / III    |
| `softwareVersion`    | for software-as-a-medical-device (SaMD)         |
| `lastCalibration`    | ISO 8601 — last performance verification        |

Devices that fail calibration cannot be bound to new
session records until a fresh calibration entry exists.

## §10 Cross-domain references (informative)

- WIA-medical-imaging — for fMRI/MEG used as biomarker
- WIA-clinical-decision-support — for outcome interpretation
- WIA-medical-data-privacy — for special-category data
- WIA-emergency-medical-data — for AE escalation

## Annex A — Conformance disclosure

Implementations declare the JSON-Schema URIs they
support, the canonicalisation form (RFC 8785), and the
key set used to sign session and consent records (RFC
7515 JWS).

## Annex B — Score reference populations (informative)

Standardised cognitive instruments require a reference
population for z-scoring. Reference populations include
NHANES (US), KNHANES (Korea), and the Mayo Clinic Older
Americans Normative Studies. The reference population
URI is recorded in `Outcome.referencePopulation`.

## Annex C — Worked tDCS session record (informative)

```json
{
  "sessionRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "subjectRef": "ME-007",
  "interventionRef": "wia-me://intervention/anodal-tDCS-DLPFC-2mA-20min",
  "startTime": "2026-04-12T09:14:00+09:00",
  "endTime":   "2026-04-12T09:34:00+09:00",
  "deliveredDose": {"current_mA": 2.0, "duration_s": 1200, "charge_mC": 2400},
  "outcomeRef": "outcome:RBANS-list-recall:+0.6sd"
}
```

## Annex D — Versioning

This PHASE follows semantic versioning per the WIA
governance procedure. Field additions are minor; field
removals or semantic redefinition require a major bump.

## Annex E — Conformance level

Conformance is "Core" (subject + intervention + session
+ outcome + consent + ethics-approval + AE) or "Full"
(adds device-binding for device interventions and
pharmacovigilance binding for drug interventions).
