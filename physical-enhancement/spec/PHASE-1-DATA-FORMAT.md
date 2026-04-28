# WIA-physical-enhancement PHASE 1 — Data Format Specification

**Standard:** WIA-physical-enhancement
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for human
physical-enhancement records covering powered exoskeletons
and orthoses, prosthetic limb augmentation, performance-
oriented pharmacology, gene-doping detection assays,
sport-medicine therapeutic-use exemptions (TUEs), and
adaptive-sport classification. Records bind every
intervention to its regulatory framework (medical
device, medicinal product, or sport-governance rule),
to the subject's informed consent and ethics approval
where applicable, and to the chain of evidence that
distinguishes therapeutic indication from sport-context
prohibited use.

References (CITATION-POLICY ALLOW only):
- World Anti-Doping Code (WADA) and the International Standard for the Prohibited List
- WADA International Standard for Therapeutic Use Exemptions (ISTUE)
- WADA International Standard for Laboratories (ISL)
- WADA Athlete Biological Passport — Operating Guidelines
- IPC Athlete Classification Code (Paralympic classification)
- ISO 13482:2014 — Robots and robotic devices — Safety for personal-care robots (exoskeletons)
- ISO 14971:2019 — Risk management for medical devices
- ISO 14155:2020 — Clinical investigation of medical devices
- ISO/IEC TR 63272 — Wearable assistive devices framework
- IEC 60601-1, IEC 60601-2 family — medical electrical equipment safety
- IEC 62366-1:2015 — Usability engineering for medical devices
- IEEE 11073-10101 / IEEE 11073-10406 — personal-health-device nomenclature
- HL7 FHIR R5 — Procedure, MedicationStatement, DeviceUseStatement, Observation, ResearchSubject
- ICH E2A / E2B (R3) — pharmacovigilance (when intervention is a medicinal product)
- ICH E6 (R3) — Good Clinical Practice
- Declaration of Helsinki (2013, 2024 revision)
- 45 CFR 46 Common Rule — protection of human subjects (US)
- IETF RFC 8259 (JSON), RFC 4122 (UUID), RFC 7515 (JWS), RFC 8785 (JCS)

---

## §1 Scope

This PHASE applies to interventions intended to restore,
maintain, or augment human locomotion, strength,
endurance, or fine-motor performance in three distinct
contexts:

1. **clinical** — therapy-led interventions for persons
   with mobility impairment, post-stroke rehabilitation,
   amputees, or chronic neuromuscular disease;
2. **occupational** — supervised use in industrial,
   military, or emergency-response settings;
3. **sport / competition** — under sport-governance
   rules including the WADA Code, IPC Athlete
   Classification Code, and federation-specific
   technical regulations.

In scope: subject record, intervention record, session
record, biomarker / sample record, performance-
measurement record, consent record, ethics-approval
record, sport-governance binding record, adverse-event
record, and device-binding record. Out of scope:
recreational consumer fitness products that make no
medical or sport-governed claim (handled by general
consumer-product standards).

## §2 Subject record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `subjectRef`         | UUID (RFC 4122) opaque identifier               |
| `pseudonymRef`       | study-local code; never the legal name          |
| `birthYear`          | year only                                       |
| `sex`                | ISO/IEC 5218 (1=male, 2=female, 9=N/A)          |
| `consentRef`         | active consent record (this PHASE §6)           |
| `ethicsApprovalRef`  | for clinical / research interventions           |
| `sportContextRef`    | IPC / IF-specific identifier when sport-bound   |
| `clinicalStatus`     | ICD-11 MMS code (e.g. 8B20 stroke, 8B26 spinal- |
|                      | cord injury, FA00.Z osteoarthritis)             |
| `enrolmentDate`      | ISO 8601                                        |
| `eligibilityFlag`    | `meets-protocol`, `screen-fail`, `withdrawn`    |

Sport contexts carry a separate identifier issued by the
relevant International Federation or WADA-recognised
National Anti-Doping Organisation; it is never reused
for clinical identification.

## §3 Intervention record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `interventionRef`    | URI                                             |
| `kind`               | `exoskeleton`, `orthosis`, `prosthesis-augment`,|
|                      | `pharmacological`, `gene-therapy`, `gene-doping`|
|                      | (detection only; never authoring), `nutrition`, |
|                      | `training-protocol`                             |
| `productRef`         | for drugs: SPL set-id (DailyMed) / EMA dossier; |
|                      | for devices: UDI-DI (IMDRF UDI WG)              |
| `regulatoryClass`    | FDA Class I / II / III; EU MDR I / IIa / IIb /  |
|                      | III; for drugs ATC code; for sport-governed:    |
|                      | WADA prohibited / restricted / permitted        |
| `protocolRef`        | ClinicalTrials.gov NCT, EudraCT, KCT,           |
|                      | jRCT identifier                                 |
| `armRef`             | active / sham / placebo / open-label / control  |
| `doseSchedule`       | for drugs: ATC + dose schedule (per WHOCC);     |
|                      | for devices: setpoint, control mode, training   |
|                      | volume, session frequency                       |
| `targetCapability`   | gait, grip, lift, endurance, balance, posture,  |
|                      | sprint, fine-motor                              |
| `riskClassification` | per ISO 14971 — low / medium / high / critical  |

A single subject may carry multiple intervention
records concurrently (e.g. orthosis + training-protocol
+ adjunct medicinal product).

## §4 Session record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `sessionRef`         | UUID                                            |
| `subjectRef`         | §2                                              |
| `interventionRef`    | §3                                              |
| `startTime`          | ISO 8601 with timezone                          |
| `endTime`            | ISO 8601 with timezone                          |
| `siteRef`            | facility code, ISO 3166-2 region                |
| `operatorRef`        | credentialed clinician / coach / PT             |
| `deliveredDose`      | actual dose, repetitions, joint torque          |
|                      | profile, energy-per-stride, training volume     |
| `physiologicalLog`   | URI to per-session telemetry (BIDS-aligned for  |
|                      | EMG / IMU / EEG; CSV / Parquet for kinematics)  |
| `aeRef[]`            | adverse-event records this session triggered    |
| `deviationRef`       | protocol-deviation record, if any               |

## §5 Performance-measurement record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `measurementRef`     | UUID                                            |
| `sessionRef`         | §4                                              |
| `instrument`         | 6-Minute Walk Test, Timed Up-and-Go, FMA,       |
|                      | BBS, ARAT, Box-and-Block, isokinetic            |
|                      | dynamometer, gait-analysis (Vicon, Xsens),      |
|                      | force-plate (AMTI, Kistler), VO₂max protocol    |
| `instrumentVersion`  | publisher / firmware version + language         |
| `rawScore`           | numeric raw                                     |
| `standardisedScore`  | z-score against reference population            |
| `assessor`           | rater UUID (independent of operator)            |
| `assessmentTime`     | ISO 8601                                        |
| `interpretation`     | improved / stable / declined / non-eval         |

## §6 Consent record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `consentRef`         | UUID                                            |
| `subjectRef`         | §2                                              |
| `protocolVersion`    | the version the subject consented to            |
| `documentRef`        | SHA-256 of the IRB-approved consent document    |
| `language`           | BCP 47 tag                                      |
| `signatureMethod`    | `wet-ink-scan`, `qualified-electronic`,         |
|                      | `advanced-electronic`, `verbal-witnessed`       |
| `signedAt`           | ISO 8601                                        |
| `withdrawnAt`        | optional ISO 8601                               |
| `proxyRef`           | for legally-authorised representative           |

For sport contexts the consent record additionally
covers anti-doping testing under the WADA Code and the
TUE-application data-handling regime.

## §7 Sport-governance binding

For interventions running in or adjacent to competition:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `governingBody`      | International Federation (IF) / NADO            |
| `applicableRules`    | WADA Code year, IPC Classification rules,       |
|                      | IF technical regulations                        |
| `tueRef`             | Therapeutic Use Exemption identifier (where     |
|                      | the intervention is otherwise prohibited)       |
| `tueDecisionDate`    | ISO 8601                                        |
| `tueExpiryDate`      | ISO 8601                                        |
| `classRef`           | for adaptive sport: IPC class, sport-specific   |
|                      | class                                           |
| `whereaboutsBinding` | for elite testing-pool athletes                 |

The TUE record links to the WADA ADAMS submission so an
inspector can verify the TUE is current.

## §8 Adverse-event record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `aeRef`              | UUID                                            |
| `sessionRef`         | §4                                              |
| `subjectRef`         | §2                                              |
| `onsetTime`          | ISO 8601                                        |
| `meddraTerm`         | MedDRA LLT + Preferred Term                     |
| `severity`           | mild / moderate / severe / life-threatening /   |
|                      | death                                           |
| `causality`          | unrelated / unlikely / possible / probable /    |
|                      | definite (per ICH E2A)                          |
| `serious`            | boolean per ICH E2A SAE definition              |
| `outcome`            | recovered / recovering / not-recovered /        |
|                      | recovered-with-sequelae / fatal                 |
| `expedited`          | boolean — expedited reporting required          |

Device-related events propagate to the device-vigilance
pipeline (PHASE 4 §3); medicinal-product events
propagate to the pharmacovigilance pipeline (PHASE 4 §4).

## §9 Device-binding record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `deviceRef`          | UUID                                            |
| `udiDi`              | UDI-DI (IMDRF UDI WG)                           |
| `udiPi`              | production identifier                           |
| `riskClass`          | FDA / EU MDR class                              |
| `safetyProfile`      | ISO 13482 personal-care robot annex + IEC       |
|                      | 60601-2-78 powered orthosis profile (where      |
|                      | applicable)                                     |
| `lastCalibration`    | ISO 8601                                        |
| `softwareVersion`    | for SaMD components                             |
| `firmwareVersion`    | for embedded controllers                        |

## §10 Cross-domain references (informative)

- WIA-rehabilitation-device — for orthosis / prosthetic
  cross-binding
- WIA-medical-data-privacy — special-category data
- WIA-emotion-ai — for affective-state correlated training
- WIA-medical-iot — for telemetry pipeline
- WIA-emergency-medical-data — for AE escalation

## Annex A — Worked exoskeleton session record (informative)

```json
{
  "sessionRef": "9af3c1c8-...",
  "subjectRef": "PE-A07",
  "interventionRef": "wia-pe://intervention/exo-stride-assist-200N",
  "startTime": "2026-04-12T10:00:00+09:00",
  "endTime":   "2026-04-12T10:42:00+09:00",
  "deliveredDose": {"steps": 1860, "assist_peak_N": 200, "duty_pct": 38},
  "performanceRef": "performance:6MWT:+62m"
}
```

## Annex B — Reference populations

Standardised performance instruments require a reference
population for z-scoring. Reference populations include
NHANES (US), ENNS (France), KNHANES (Korea). The
reference URI is recorded on the measurement record.

## Annex C — Conformance disclosure

Implementations declare the JSON-Schema URIs they serve,
the sport-governance bodies whose rules they encode,
and the FHIR Bulk Data profile version.

## Annex D — Versioning

Field additions are minor; semantic redefinition or
removal is major.

## Annex E — Conformance level

"Core" (subject + intervention + session + performance +
consent + ethics + AE) and "Full" (adds sport-governance
binding for sport-context standards or device-binding
for device interventions).
