# WIA-education-robot PHASE 3 — Protocol Specification

**Standard:** WIA-education-robot
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
an education-robot operator across the
manufacturer-to-deployer-to-learner-to-regulator
value chain: the ISO 13482 personal-care-robot
hazard-identification discipline, the ISO 10218
industrial-robot risk-assessment discipline, the
ISO/TS 15066 collaborative-robot pressure-and-
force discipline, the IEEE Std 1872 ontology
binding discipline, the IEC 62366-1 usability-
engineering discipline (applied as the analogue
reference for the inclusive-education robot), the
EU AI Act high-risk-AI classification discipline
where the robot processes biometric / behavioural
data, the IEC 62443 industrial-control-system
security discipline for the school-network-
isolated deployment, the parent-or-guardian
consent discipline for a minor learner, and the
post-market-surveillance discipline including
incident reporting and recall propagation.

References (CITATION-POLICY ALLOW only):

- ISO 13482:2014, ISO 10218-1:2011, ISO 10218-
  2:2011, ISO/TS 15066:2016, ISO 12100:2010
  (general principles for design — risk
  assessment and risk reduction)
- ISO 9241-210:2019, ISO 9241-110:2020, IEC
  62366-1:2015/Amd 1:2020
- IEC 61508 series, IEC 62443-3-3:2013
- IEEE Std 1872-2015 (CORA), IEEE Std 1872.2-
  2021 (AuR)
- ISO/IEC 23894:2023, ISO/IEC 42001:2023
- EU Machinery Regulation (EU) 2023/1230, EU AI
  Act (Regulation (EU) 2024/1689), EU Toy
  Safety Directive 2009/48/EC
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012
- IETF RFC 9110, RFC 9421, RFC 9457, RFC 6234,
  RFC 8615, RFC 6962
- W3C Trace Context, W3C WebRTC 1.0
- EU GDPR (Regulation (EU) 2016/679)
- KR 산업안전보건법, KR 학교안전사고 예방 및 보상
  에 관한 법률, KR 어린이제품 안전 특별법, KR
  개인정보 보호법
- US 16 CFR Part 1112 (Children's Product
  Safety Rules), CPSC procedures

---

## §1 ISO 13482 Personal-Care-Robot Discipline

### §1.1 Hazard identification

A personal-care robot deployed in a school or an
after-school academy is bound to the ISO 13482:
2014 hazard-identification discipline. The
operator's safety-test record carries the per-
hazard identification — battery hazards,
mechanical hazards, electrical hazards, hazards
arising from incorrect autonomous decisions,
hazards arising from interaction with humans —
per the standard's risk-assessment table.

### §1.2 Per-class envelope

ISO 13482 defines the personal-care-robot classes
— mobile servant robot, physical assistant
robot, person carrier robot. The operator's API
binds the per-deployment robot to its class and
applies the class-specific safety requirements.

### §1.3 Operating-environment declaration

The operator declares the per-deployment
operating environment (the classroom layout, the
expected human-presence density, the per-segment
floor surface) so that the safety case is
interpretable in the deployment context.

## §2 ISO 10218 Industrial-Robot Discipline

### §2.1 Per-system risk assessment

A robotic-arm or autonomous-mobile-robot platform
deployed in a robotics academy or competition
arena is bound to the ISO 10218-1:2011 / -2:2011
risk-assessment discipline. The operator's safety-
test record carries the per-system risk
assessment per ISO 12100:2010.

### §2.2 Safeguarding strategy

The operator declares the per-deployment
safeguarding strategy — fixed guards, interlocked
movable guards, presence-sensing protective
equipment, light curtains — per the standard's
§4 safeguarding measures.

## §3 ISO/TS 15066 Collaborative-Robot Discipline

A collaborative robot operating in a classroom
laboratory (a desktop robotic-arm sharing the
workspace with a student) is bound to the ISO/TS
15066:2016 collaborative-robot discipline. The
operator's safety-test record carries the per-
body-region quasi-static and transient pressure-
and-force values measured against the standard's
Annex A reference values.

## §4 IEEE Std 1872 Ontology Discipline

### §4.1 CORA class binding

Every robot platform is bound to the IEEE Std
1872-2015 CORA classes — Robot, RobotPart,
Sensor, Effector, Environment, Position,
Orientation. The operator's API rejects an
ontology record that does not bind to at least
the Robot class and at least one of Sensor or
Effector.

### §4.2 AuR ontology binding

An autonomous-behaviour robot is additionally
bound to the IEEE Std 1872.2-2021 AuR ontology.
The operator's API records the per-platform
autonomous-behaviour declaration, the sensor-
fusion binding, and the decision-making
envelope.

### §4.3 Educational-content binding

The operator's per-platform educational-outcome
declaration is bound to the ontology so that a
downstream consumer (a curriculum integrator, a
recognition authority) can deterministically
interpret the platform's educational content.

## §5 IEC 62366-1 Usability Discipline

A robot deployed for inclusive education
(autism-spectrum-disorder learners, learners
with motor-coordination differences) is bound
to the IEC 62366-1:2015/Amd 1:2020 usability-
engineering discipline applied as the analogue
reference. The operator declares the per-
deployment use-specification, runs the
formative usability evaluation, and runs the
summative usability evaluation per the
standard's §5 process.

## §6 IEC 61508 Functional-Safety Discipline

A robot whose autonomous-decision module is
classified as safety-related is bound to the
IEC 61508 functional-safety discipline. The
operator's safety-test record carries the per-
channel safety-integrity-level (SIL) evidence,
the per-channel hardware-fault-tolerance
declaration, and the per-channel diagnostic-
coverage declaration.

## §7 EU AI Act High-Risk-AI Discipline

An education-robot processing biometric
identification, processing emotional inference,
or providing access to educational opportunity
is classified as a high-risk AI system under
the EU AI Act (Regulation (EU) 2024/1689) Annex
III. The operator runs the conformity
assessment per Article 43, registers the system
in the EU AI Act database per Article 49, and
implements the post-market monitoring per
Article 72.

## §8 IEC 62443 Industrial-Control-System Security

A school-deployed robot connected to the school
network is bound to the IEC 62443-3-3:2013
system-security-requirements discipline. The
operator's API publishes the per-deployment
security-zone classification, the per-zone
security-level declaration (SL-1 through SL-4),
and the per-zone hardening declaration.

## §9 Parent-or-Guardian Consent Discipline

### §9.1 Per-minor consent envelope

A learner who is a minor (under 18 in most
jurisdictions; under 13 under US COPPA; under
14 under KR 개인정보 보호법 §22-2) interacts
with the robot only after the parent or
guardian's consent envelope is recorded.

### §9.2 Per-feature consent

The consent envelope is granular per feature —
camera-recording, voice-recording, biometric-
identification, emotional-inference, telepresence
— so that the learner is exposed only to the
features the parent or guardian has consented
to.

### §9.3 Consent revocation

The consent revocation is honoured immediately;
all per-learner data is deleted within the
operator's declared deletion window per the
operator's privacy declaration.

## §10 Privacy and Identity-Vault Discipline

The operator publishes the learner's
pseudonymous identifier on its API surface and
holds the linkage to the directly-identifying
record in an identity vault. Access to the
vault is gated on the operator's role-based
access-control policy. A learner whose data is
processed under EU jurisdiction is processed
per GDPR Article 8 (child consent threshold
13-16 by Member State).

## §11 Post-Market Surveillance Discipline

### §11.1 Incident reporting

Every incident (a learner injured during a
session, a robot malfunctioning during a
session, a robot's autonomous decision causing
harm) is reported per the operator's national
authority's incident-reporting procedure (the
EU CE-marking authority, the US CPSC, the KR
KATS).

### §11.2 Recall propagation

A recall issued by the manufacturer propagates
to all subscribed deployer endpoints via webhook
per PHASE-2 §11. The operator records the per-
deployer acknowledgement per the recall-
authority's reporting requirement.

### §11.3 Per-platform field-safety-corrective-
       action register

The operator maintains the per-platform field-
safety-corrective-action register and publishes
the register to the relevant recall authority
on a frequency declared in the operator's
post-market surveillance plan.

## §12 KR-Jurisdiction Discipline

### §12.1 KR 어린이제품 안전 특별법 binding

A KR-jurisdiction operator selling an
education-robot to a minor learner binds the
robot to the KR 어린이제품 안전 특별법 (Children's
Product Safety Special Act) and to the KR 어린
이제품 안전인증 envelope issued by the Korean
Agency for Technology and Standards.

### §12.2 KR 학교안전사고 binding

A school-deployed robot is bound to the KR 학교
안전사고 예방 및 보상에 관한 법률 (Act on
Prevention of and Compensation for School Safety
Accidents). The operator's incident-reporting
envelope binds to the KR 학교안전공제회 (School
Safety Mutual Aid Association) reference.

### §12.3 KR 산업안전보건법 binding

A KR-jurisdiction industrial-classroom
collaborative-robot is bound to the KR 산업
안전보건법 (Occupational Safety and Health Act)
and the KR Ministry of Employment and Labor's
machinery-safety inspection envelope.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §13 Telepresence-Robot Network Discipline

A telepresence robot deployed for a
hospitalised learner is bound to the W3C
WebRTC 1.0 signalling discipline. The
operator's API publishes the per-session
signalling-server reference, the per-session
ICE-and-TURN configuration, and the per-
session media-stream encryption envelope. The
hospital's network-segregation policy is
honoured — the robot operates on a network
segment isolated from the hospital's clinical-
data network per the IEC 62443-3-3 zone
declaration.

## §14 ISO/IEC 42001 AI Management Discipline

An education-robot whose autonomous-decision
module is classified as an AI system is bound
to the ISO/IEC 42001:2023 AI management system
discipline. The operator declares the per-
platform AI risk-treatment plan, the per-
platform AI impact-assessment, and the per-
platform AI continuous-improvement cycle.

## §15 Post-Incident Review Discipline

After a per-deployment incident (a learner
injured during a session, a robot malfunctioning
during a session), the operator runs a post-
incident review per the operator's documented
procedure. The review's outcome is bound to
the per-incident corrective-action register
and is reported to the relevant supervisory
authority per the operator's reporting plan.
