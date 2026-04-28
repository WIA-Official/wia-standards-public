# WIA-education-robot PHASE 1 — Data Format Specification

**Standard:** WIA-education-robot
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-education-robot. The standard
covers the persistent record shapes that an
education-robot operator (a personal-care
robotic-tutor manufacturer, a programmable
educational-robot kit producer, a school-based
service-robot deployer, an after-school robotics
academy operator, an inclusive-education
robotics-tutor for special-needs learners, a
museum-based robotic docent operator, a
telepresence-robot for hospitalised learners
operator, an ISO 13482 personal-care-robot
manufacturer, an industrial-classroom IEC
62443-compliant robotics deployer) maintains
when registering an education-robot platform,
classifying its safety profile under ISO 13482
or ISO 10218, declaring its educational-content
binding under IEEE Std 1872 ontology, recording
the learner-robot interaction event log, and
tracking the robot's lifecycle and recall
discipline. Records are consumed by the school's
ICT-and-safety committee approving the deployment,
by the parent or guardian providing the consent
for a minor learner's interaction with the robot,
by the manufacturer's quality team running the
post-market surveillance, and by the supervisory
education-or-product-safety regulator (the EU CE
marking authority, the US Consumer Product Safety
Commission CPSC, the KR 산업통상자원부 KATS
agency).

References (CITATION-POLICY ALLOW only):

- ISO 13482:2014 (robots and robotic devices —
  safety requirements for personal care robots
  — covering mobile servant robot, physical
  assistant robot, and person carrier robot)
- ISO 10218-1:2011 / ISO 10218-2:2011 (robots
  and robotic devices — safety requirements for
  industrial robots — Part 1 robot, Part 2
  robot system and integration; the ISO
  10218-1:2011 / 10218-2:2011 are anchored
  under ISO TC 299)
- ISO/TS 15066:2016 (robots and robotic devices
  — collaborative robots — for the
  collaborative-classroom robot binding)
- ISO 9241 series (ergonomics of human-system
  interaction) — ISO 9241-210:2019 (human-
  centred design for interactive systems),
  ISO 9241-110:2020 (interaction principles)
- IEC 62366-1:2015/Amd 1:2020 (medical devices
  — application of usability engineering to
  medical devices, applied as the analogue
  reference for the inclusive-education robot
  binding)
- IEC 61508 series (functional safety of
  electrical / electronic / programmable
  electronic safety-related systems)
- IEEE Std 1872-2015 (Standard Ontologies for
  Robotics and Automation; the CORA — Core
  Ontologies for Robotics and Automation —
  reference)
- IEEE Std 1872.2-2021 (Autonomous Robotics
  ontology, the AuR ontology)
- IEC 62443 series (industrial automation and
  control systems security — for the school-
  network-isolated deployment)
- IEC 60601-1:2005+AMD1:2012+AMD2:2020 (medical
  electrical equipment — used as the analogue
  reference for an inclusive-education robot
  with medical attributes)
- ISO 22989:2022 (information technology —
  artificial intelligence — concepts and
  terminology)
- ISO/IEC 23894:2023 (information technology —
  artificial intelligence — guidance on risk
  management) and ISO/IEC 42001:2023
  (artificial intelligence management system)
- W3C WebRTC 1.0 (Real-Time Communication —
  for the telepresence-robot binding)
- IEEE 11073-10101 (medical-device nomenclature
  — used for the inclusive-education vital-
  sign-correlation binding)
- IEC 60950-1:2005+AMD2:2013 / IEC 62368-1:
  2018+AMD1:2020 (information-technology and
  audio-video safety)
- EU Machinery Directive 2006/42/EC (succeeded
  by EU Machinery Regulation (EU) 2023/1230)
- EU AI Act (Regulation (EU) 2024/1689) — the
  high-risk classification for an education-
  robot operating as a high-risk AI system
- KR 산업안전보건법, KR 학교안전사고 예방 및
  보상에 관한 법률, KR 어린이제품 안전 특별법
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO
  8601 (date-time)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when an education-robot
platform is registered, classified under ISO
13482 / ISO 10218, characterised through IEEE
Std 1872 ontology binding, and tracked across
its lifecycle. Implementations covered include:

- A primary-school programmable robot kit (a
  cubic-style block-based programmable robot
  deployed in a coding-introduction class)
  registered against ISO 8124 toy-safety baseline
  and the EU CE Toy Safety Directive 2009/48/EC.
- A secondary-school robotic competition platform
  (an autonomous mobile robot operating in a
  competition arena) registered against the
  ISO 10218 industrial-robot safety baseline.
- An after-school robotics-academy humanoid
  robot (a desktop social-interaction humanoid)
  registered against ISO 13482 personal-care-
  robot baseline.
- An inclusive-education robot for autism-
  spectrum-disorder learners (a humanoid robot
  delivering social-skill-training sessions
  under therapist supervision) registered
  against the IEC 62366-1 usability discipline
  with the operator's consent and clinical-
  supervision envelope.
- A telepresence robot for a hospitalised
  learner (a remote-presence robot allowing the
  learner to attend the home classroom from the
  hospital ward) registered with the W3C WebRTC
  signalling discipline and the operator's
  privacy envelope.

The mechanical-safety envelope under ISO 13482,
the educational-content binding under IEEE Std
1872, and the privacy-and-consent envelope under
GDPR / KR 개인정보 보호법 receive distinct
encodings in this PHASE; the additional
safeguards required by each deployment context
are encoded in PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — robot
                       manufacturer, school
                       deployer, after-school
                       academy, inclusive-
                       education programme)
operatorRole         : enum ("robot-manufacturer"
                       | "robot-system-integrator"
                       | "school-deployer" |
                       "academy-operator" |
                       "inclusive-education-
                       provider" | "user-defined")
governingFrameworks  : array of enum ("ISO-13482"
                       | "ISO-10218-1" |
                       "ISO-10218-2" |
                       "ISO-TS-15066" |
                       "ISO-9241-210" |
                       "ISO-9241-110" |
                       "IEC-62366-1" |
                       "IEC-61508" |
                       "IEEE-1872" |
                       "IEEE-1872.2" |
                       "IEC-62443-3-3" |
                       "IEC-60601-1" |
                       "ISO-22989" |
                       "ISO-IEC-23894" |
                       "ISO-IEC-42001" |
                       "EU-Machinery-2023-1230" |
                       "EU-AI-Act-2024-1689" |
                       "EU-Toy-Safety-2009-48-EC"
                       | "KR-산업안전보건법" |
                       "KR-학교안전사고-예방-보상-
                       법률" | "KR-어린이제품-안전-
                       특별법" | "user-defined")
accreditationStatus  : object (the ISO/IEC 17025
                       accreditation reference for
                       the testing laboratory, the
                       ISO/IEC 17065 accreditation
                       reference for the product-
                       certification body, the EU
                       notified-body designation
                       reference for the CE-
                       marking conformity
                       assessment)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Robot Platform Record

```
robotRecord:
  robotId            : string (uuidv7)
  identifierBindings : array of object (per-
                       jurisdiction product
                       identifiers — for example
                       the EU CE Declaration of
                       Conformity reference, the
                       UL Recognised Component
                       identifier, the KR 어린이
                       제품 안전인증 번호 — each
                       carrying the issuing
                       authority and the scope
                       of use)
  robotKind          : enum ("programmable-block-
                       robot" | "mobile-servant-
                       robot" | "physical-
                       assistant-robot" |
                       "person-carrier-robot" |
                       "industrial-collaborative-
                       robot" | "humanoid-social-
                       robot" | "telepresence-
                       robot" | "robotic-arm-
                       educational" | "drone-
                       educational" | "user-
                       defined")
  iso13482Class      : enum ("mobile-servant" |
                       "physical-assistant" |
                       "person-carrier" | "n/a-
                       industrial" | "user-
                       defined")
  payload            : object (declared payload
                       in kg, with operating
                       speed in m/s)
  workspaceClass     : enum ("classroom" |
                       "laboratory" | "competition-
                       arena" | "home-deployment"
                       | "hospital-room" |
                       "museum-floor" | "user-
                       defined")
  intendedAgeBand    : enum ("3-5" | "6-9" |
                       "10-13" | "14-18" |
                       "adult" | "user-defined")
  intendedSupervision : enum ("teacher-supervised"
                       | "therapist-supervised" |
                       "self-paced" | "remote-
                       supervised" | "user-
                       defined")
```

## §4 IEEE Std 1872 Ontology Binding Record

```
ontologyRecord:
  ontologyRecordId   : string (uuidv7)
  robotRef           : string (PHASE-1 §3 record
                       reference)
  coraClassBinding   : object (the IEEE Std 1872
                       CORA class binding —
                       Robot, RobotPart, Sensor,
                       Effector, Environment,
                       Position, Orientation —
                       per the standard's
                       ontology declaration)
  aurBehaviour       : object (the IEEE Std
                       1872.2 AuR ontology
                       behaviour binding —
                       autonomous-behaviour
                       declaration, sensor-fusion
                       binding, decision-making
                       envelope)
  educationalOutcome : object (the per-platform
                       educational-outcome
                       declaration — the learning
                       outcome the platform
                       delivers, the per-outcome
                       assessment binding)
```

## §5 Safety-Conformity Record

```
safetyRecord:
  safetyRecordId     : string (uuidv7)
  robotRef           : string (PHASE-1 §3 record
                       reference)
  testStandard       : enum ("ISO-13482" | "ISO-
                       10218-1" | "ISO-10218-2" |
                       "ISO-TS-15066" | "IEC-
                       61508-SIL-2" | "IEC-61508-
                       SIL-3" | "EU-Machinery-
                       2023-1230" | "user-
                       defined")
  testLaboratory     : object (laboratory name +
                       the laboratory's ISO/IEC
                       17025 accreditation
                       reference + the laboratory's
                       safety-test report
                       identifier)
  measurementResult  : object (per-method derived
                       quantities — separation
                       distance per ISO-TS-15066,
                       quasi-static-pressure
                       limit per ISO-TS-15066
                       Annex A, transient-pressure
                       limit, force-and-pressure
                       maps for the collaborative
                       workspace)
  passOrFail         : enum ("pass" | "fail" |
                       "conditional")
```

## §6 Interaction-Event Log

```
interactionEvent:
  eventId            : string (uuidv7)
  robotRef           : string (PHASE-1 §3 record
                       reference)
  learnerRef         : string (pseudonymous
                       learner identifier per
                       §8 privacy discipline)
  sessionStart       : string (ISO 8601 date-time)
  sessionEnd         : string (ISO 8601 date-time)
  sessionContent     : object (the ontology-bound
                       content reference — for
                       example a programming
                       lesson, a social-skill-
                       training session, a
                       remote-presence connection)
  consentRef         : string (the parent-or-
                       guardian-consent envelope
                       reference where the learner
                       is a minor)
  supervisionRef     : string (the supervising
                       teacher or therapist
                       identifier)
```

## §7 Lifecycle and Recall Record

```
lifecycleRecord:
  lifecycleId        : string (uuidv7)
  robotRef           : string (PHASE-1 §3 record
                       reference)
  lifecycleEvent     : enum ("commissioned" |
                       "firmware-update" |
                       "maintenance-performed" |
                       "incident-reported" |
                       "recall-issued" |
                       "decommissioned" |
                       "user-defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity)
  notes              : string (the operator's
                       narrative description of
                       the lifecycle event)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the safety-test,
                       interaction-event, or
                       lifecycle identifier)
  custodyEvent       : enum ("safety-test-
                       conducted" | "ce-marking-
                       applied" | "deployment-
                       commissioned" | "incident-
                       reported" | "recall-
                       executed" | "shipment" |
                       "user-defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex digest)
```

## §9 Manifest

Implementations publish a signed manifest carrying
`standardSlug` (constant value "education-robot"),
`version`, `implementation`, the operator's
`accreditationStatus`, and the `profile`
declaration that selects which of the optional
records (ontology, safety-conformity,
interaction-event, lifecycle) the implementation
supports. The manifest is signed using a key
whose public part is published on the operator's
`.well-known/wia/education-robot/` discovery
endpoint declared in PHASE-2.

弘益人間 (Hongik Ingan) — Benefit All Humanity
