# WIA-medical-robot PHASE 1 — Data Format Specification

**Standard:** WIA-medical-robot
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for medical robotic
systems: surgical robots (laparoscopic, orthopaedic, neurosurgical,
ophthalmic), interventional radiology robots, rehabilitation robots,
pharmacy compounding robots, hospital service robots (delivery,
disinfection), and the mobile autonomous platforms that move through
clinical environments. The shape interoperates with HL7 FHIR R5
Procedure, Device, and Observation resources and with the
ISO/IEC 80601-2 series of particular safety standards for surgical
and rehabilitation robotics.

References (CITATION-POLICY ALLOW only):
- IEC 60601-1:2005/A2:2020 — Medical electrical equipment, general
  requirements for basic safety and essential performance
- IEC 60601-1-2:2014/A1:2020 — Electromagnetic disturbances
- IEC 80601-2-77:2019 — Particular requirements for the basic safety
  and essential performance of robotically assisted surgical equipment
- IEC 80601-2-78:2019 — Particular requirements for medical robots
  for rehabilitation, assessment, compensation or alleviation
- ISO 13482:2014 — Robots and robotic devices — Safety requirements
  for personal care robots
- ISO 13485:2016 — Medical devices — Quality management systems
- ISO 14971:2019 — Risk management
- IEC 62304:2006/A1:2015 — Medical device software life cycle
- IEC 62366-1:2015/A1:2020 — Application of usability engineering
- ISO/IEC 23894:2023 — AI risk management (where AI assistance is
  integrated, e.g., autonomous task selection)
- HL7 FHIR R5 — Procedure, Device, Observation, ServiceRequest,
  DeviceUsage
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 3339

---

## §1 Scope

This PHASE applies to robotic systems that operate in medical
contexts subject to medical-device regulation (US FDA, EU MDR,
K-MFDS, PMDA, ANVISA). It standardises the *shape* of robot
identity, procedure records, motion telemetry, intervention
events, alarm conditions, and the cross-references that bind
robot operations to a clinical record.

Robots are classified by:

- **Regulatory class** — FDA Class II / III, MDR Class IIa / IIb /
  III; the higher class governs verification cadence
- **Software safety class** — IEC 62304 Class A / B / C
- **Particular standard** — IEC 80601-2-77 (surgical),
  IEC 80601-2-78 (rehabilitation), ISO 13482 (personal care);
  out of scope here: industrial-grade ISO 10218 robots used in
  non-clinical contexts (those are addressed by a sibling WIA
  industrial-robot standard)

The classification travels with every record so downstream
systems apply the correct risk handling and audit-cadence.

## §2 Robot identity

Every robotic system carries a structured identifier:

- `robotRef` — URN of form `urn:wia:mrobot:robot:<udi-di>:<udi-pi>`
- `kindRef` — URN denoting the robot kind (e.g.,
  `urn:wia:mrobot:kind:surgical-laparoscopic`,
  `urn:wia:mrobot:kind:rehab-upper-limb`,
  `urn:wia:mrobot:kind:pharmacy-compounding`,
  `urn:wia:mrobot:kind:service-disinfection-uvc`)
- `softwareRef` — IEC 62304 software identifier and signed
  firmware build hash
- `endEffectorRef[]` — for modular surgical robots, the list
  of currently-attached end effectors, each with its own
  UDI-PI and lifetime-counter
- `siteRef` — facility / OR / ward URN where the robot is
  installed or scheduled

A record without a recognised UDI is rejected with
`urn:wia:mrobot:problem:udi-required`. Modular end effectors
beyond their declared use-life are refused with
`urn:wia:mrobot:problem:end-effector-life-exceeded`.

## §3 Procedure record

Each robotic intervention is bound to a FHIR Procedure resource:

| FHIR field            | Binding                                                          |
|-----------------------|------------------------------------------------------------------|
| `status`              | preparation / in-progress / completed / stopped / entered-in-error |
| `category`            | from FHIR ProcedureCategoryCodes                                 |
| `code`                | a SNOMED CT procedure code or ICD-10-PCS where applicable       |
| `subject`             | Reference to the pseudonymous patient                            |
| `performedDateTime`   | RFC 3339 with offset                                            |
| `performer[]`         | clinical principals + the robot, with role coding                |
| `usedReference[]`     | references to instruments, devices, end effectors                |
| `note[]`              | free-text notes captured during the procedure                    |

The robot itself appears as a `performer` with the role
code `assistant` (FHIR ProcedurePerformerRoleCodes); the
human surgeon retains primary responsibility under
IEC 80601-2-77 §201.4.4 (operator control).

## §4 Motion telemetry

For procedures where the robot moves end effectors (surgical,
rehabilitation), motion telemetry is captured as a stream of
samples. Each sample:

- `sampleId` — sequence number within the procedure
- `at` — RFC 3339 with offset, sub-millisecond precision when
  the robot's clock supports it
- `joints[]` — per-joint position, velocity, torque
- `endEffectorPose` — 6-DoF pose (position + orientation) in
  the robot's base frame
- `appliedForce` — for force-controlled tasks, the commanded
  and measured force vectors
- `safetyEnvelope` — `clear` / `warning` / `violation` per
  the robot's collision and force limits

Motion telemetry is high-frequency (typically 500–1000 Hz);
the boundary stores it in a separate time-series store with
references from the procedure record (PHASE 2 §3).

## §5 Intervention event

Significant moments in a procedure are captured as discrete
intervention events:

- `eventId` — URN
- `procedureRef` — parent procedure
- `kind` — closed enum: `tool-engagement`, `tool-disengagement`,
  `force-limit-warning`, `force-limit-violation`,
  `collision-stop`, `operator-pause`, `operator-resume`,
  `mode-change`, `safety-button-press`, `tele-link-loss`,
  `tele-link-restored`, `emergency-stop`
- `at` — RFC 3339
- `triggeredBy` — `operator` / `system` / `safety-supervisor`
- `relatedTelemetryRef` — pointer to the telemetry sample
  range surrounding the event
- `outcome` — closed enum: `proceeded`, `paused`, `aborted`

Intervention events are append-only and form the procedure's
audit trail.

## §6 Alarm condition record

Alarms follow IEC 60601-1-8 priorities. A robot-specific alarm
catalogue extends the standard categories:

- `force-limit-exceeded` — high priority
- `safety-envelope-violation` — high priority
- `tele-link-loss` — high priority
- `end-effector-loose` — high priority
- `pose-tracking-degraded` — medium priority
- `calibration-overdue` — low priority
- `maintenance-due` — low priority

Alarm records carry the same fields as PHASE 1 §5 of the
medical-iot standard (acknowledged-by, escalation log) plus:

- `procedureRef` if the alarm fires during a procedure
- `causalChainRef` — pointer to the intervention event that
  preceded the alarm

## §7 Calibration and verification

Each robot carries:

- **Calibration record** — joint-encoder calibration, end-
  effector tip calibration, force-sensor calibration; updated
  on every successful calibration cycle
- **Verification record** — periodic functional verification
  against a phantom or test pattern; results recorded
  quantitatively
- **End-effector lifetime counter** — actuation cycle count
  and force-cycle count; the manufacturer's UDI-DI declares
  the end-of-life thresholds

A robot whose calibration or verification has lapsed is
ineligible to start new procedures. In-progress procedures
on a robot that crosses a maintenance threshold are not
forcibly interrupted; the boundary instead logs a
`maintenance-overrun` event for retrospective review.

## §8 Patient-association record

Procedures bind the robot to a patient via a procedure-scoped
association:

- `associationId` — URN
- `robotRef` — operating robot
- `subject` — pseudonymous patient (cross-domain reference per §10)
- `procedureRef` — bound procedure
- `period.start` / `period.end` — procedure-bounded
- `consentRef` — cross-reference to WIA-medical-data-privacy
  consent for the procedure's purpose-of-use (`TREAT`)

Tele-surgery scenarios where the surgeon is in a remote
console add a second association entry binding the robot
to the remote-surgeon principal.

## §9 Imaging integration

Many robotic procedures use real-time imaging (intraoperative
ultrasound, fluoroscopy, OCT). Imaging streams are referenced
by DICOM Study Instance UID:

- `imagingRefs[]` — list of `(studyInstanceUid, modality,
  acquisitionPeriod)` tuples linking the procedure to imaging
  acquired during it
- Cross-reference to WIA-medical-imaging for storage,
  release, and de-identification rules

## §10 Cross-domain references

| Reference                  | Use site                                                  |
|----------------------------|-----------------------------------------------------------|
| WIA-medical-data-privacy   | every procedure record references the consent record      |
| WIA-medical-imaging        | imaging studies acquired during procedures                |
| WIA-medical-iot            | non-robotic IoT devices used during the procedure         |
| WIA-network-security       | cipher-suite floor for tele-surgery and console links     |
| WIA-pq-crypto              | post-quantum migration phase                              |

The boundary verifies cross-domain references resolve at the
referenced standard's boundary before delivery.

## §11 Subject identifier scope

Subject identifiers follow WIA-medical-data-privacy `subjectRef`
shape; the medical-robot boundary holds no direct identifiers
in audit chains.

## §12 Conformance levels

| Level     | Scope                                                                |
|-----------|----------------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                                 |
| Verified  | annual third-party audit including IEC 62304 + IEC 80601-2-77/-78    |
| Anchored  | continuous evidence package + IEC 80001-1 risk file                  |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Negative test vectors (informative)

| Stimulus                                                | Expected outcome                                  |
|---------------------------------------------------------|---------------------------------------------------|
| Procedure start without active consent                   | 403 + `no-active-consent`                         |
| Robot with calibration overdue                           | 403 + `calibration-overdue`                       |
| End effector beyond use-life                             | 422 + `end-effector-life-exceeded`                |
| Motion telemetry without robot-clock sub-ms precision    | accepted, flagged `clock-precision-degraded`     |
| Force-limit violation during procedure                   | high-priority alarm + intervention event recorded |
| Tele-link loss during tele-surgery                       | high-priority alarm + safe-state transition       |

## Annex B — Robot kind catalogue (informative)

| Kind URN                                          | Particular standard | Risk class |
|---------------------------------------------------|---------------------|------------|
| `urn:wia:mrobot:kind:surgical-laparoscopic`       | IEC 80601-2-77      | III        |
| `urn:wia:mrobot:kind:surgical-orthopaedic`        | IEC 80601-2-77      | III        |
| `urn:wia:mrobot:kind:surgical-neurosurgical`      | IEC 80601-2-77      | III        |
| `urn:wia:mrobot:kind:surgical-ophthalmic`         | IEC 80601-2-77      | III        |
| `urn:wia:mrobot:kind:rehab-upper-limb`            | IEC 80601-2-78      | IIb        |
| `urn:wia:mrobot:kind:rehab-gait`                  | IEC 80601-2-78      | IIb        |
| `urn:wia:mrobot:kind:pharmacy-compounding`        | ISO 13485 + IEC 60601-1 | IIa     |
| `urn:wia:mrobot:kind:service-disinfection-uvc`    | ISO 13482           | I          |
| `urn:wia:mrobot:kind:service-delivery`            | ISO 13482           | I          |

The catalogue is appendable in future minor versions.
