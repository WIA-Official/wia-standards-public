# WIA-education-robot PHASE 4 — Integration Specification

**Standard:** WIA-education-robot
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This document defines how an education-robot
operator integrates with the systems that
surround the manufacturer-deployer-learner-
regulator value chain: the EU CE-marking
notified body operating the conformity
assessment under the EU Machinery Regulation
and the EU AI Act; the US CPSC operating the
children's-product-safety register; the KR
산업통상자원부 KATS operating the children's-
product-safety mark register; the school
information-and-communication-technology
committee approving the deployment; the
parent-or-guardian information channel; the
inclusive-education clinical supervisor; the
post-market surveillance authority; the recall
authority; and the LMS or curriculum platform
binding the robot's educational content to the
school's curriculum.

References (CITATION-POLICY ALLOW only):

- ISO 13482:2014, ISO 10218-1:2011 / -2:2011,
  ISO/TS 15066:2016, ISO 12100:2010
- IEC 62366-1:2015/Amd 1:2020, IEC 61508 series,
  IEC 62443-3-3:2013, IEC 62368-1:2018+AMD1:
  2020
- IEEE Std 1872-2015 (CORA), IEEE Std 1872.2-
  2021 (AuR)
- ISO/IEC 23894:2023, ISO/IEC 42001:2023, ISO/IEC
  17021-1:2015, ISO/IEC 17065:2012, ISO/IEC
  27001:2022
- EU Machinery Regulation (EU) 2023/1230, EU AI
  Act (Regulation (EU) 2024/1689), EU Toy
  Safety Directive 2009/48/EC, EU GDPR
  (Regulation (EU) 2016/679)
- US 16 CFR Part 1112 (CPSC procedures), US
  COPPA (Children's Online Privacy Protection
  Act)
- KR 어린이제품 안전 특별법, KR 학교안전사고 예방
  및 보상에 관한 법률, KR 산업안전보건법, KR 개인
  정보 보호법
- W3C WebRTC 1.0
- IETF RFC 9110, RFC 9457, RFC 8615, RFC 9421
- IMS Global LTI 1.3 / Advantage

---

## §1 EU CE-Marking Notified-Body Integration

A robot in scope of the EU Machinery Regulation
(EU) 2023/1230 is conformity-assessed by an EU
notified body. The notified body queries the
operator's API for the per-platform safety-
test record and issues the per-platform CE
Declaration of Conformity. The operator
publishes the DoC reference in the robot
record's `identifierBindings`.

A robot classified as a high-risk AI system
under the EU AI Act undergoes the conformity
assessment per Article 43 by an AI-Act-
designated notified body.

## §2 US CPSC Integration

A US-jurisdiction operator marketing an
education-robot to children registers the
product with the US Consumer Product Safety
Commission per US 16 CFR Part 1112. The
operator's API publishes the CPSC registration
reference in the robot record.

## §3 KR KATS Integration

A KR-jurisdiction operator marketing an
education-robot to a minor learner binds the
robot to the KR 어린이제품 안전인증 mark issued
by the Korean Agency for Technology and
Standards (KATS). The operator's API publishes
the per-platform safety-certification number
in the robot record.

## §4 School ICT Committee Integration

A school's ICT-and-safety committee approves
the deployment of an education-robot in the
school. The operator's API publishes the per-
deployment school-approval reference, the
per-deployment teacher-supervision plan, and
the per-deployment incident-response plan so
that the school's committee can review the
deployment.

## §5 Parent-or-Guardian Information Channel

A parent or guardian of a minor learner
receives the per-platform information envelope
through the operator's published information
channel. The envelope carries the platform's
intended use, the platform's data-processing
declaration (the per-feature consent envelope
declared in PHASE-3 §9.2), the platform's
incident-reporting channel, and the platform's
recall procedure.

## §6 Inclusive-Education Clinical Supervisor Integration

An inclusive-education robot session is
supervised by a clinical supervisor (a
behavioural therapist, a speech-and-language
therapist, an occupational therapist). The
operator's API binds the per-session
supervisor identifier and publishes the per-
session clinical-outcome envelope to the
supervisor's clinical-record system per the
operator's data-sharing agreement.

## §7 Post-Market Surveillance Authority Integration

The post-market surveillance authority (the EU
Member State market-surveillance authority
under the EU Machinery Regulation, the US
CPSC, the KR KATS) audits the operator's
post-market surveillance records on demand.
The operator's API publishes the per-period
post-market-surveillance summary to the
authority.

## §8 Recall Authority Integration

A recall is registered with the recall
authority (the EU Safety Gate operated by
the European Commission, the US CPSC SaferProducts.gov,
the KR Consumer Product Safety Information Net
operated by the Korea Consumer Agency). The
operator's API publishes the recall envelope
to the authority's intake endpoint.

## §9 LMS / Curriculum Platform Integration

### §9.1 IMS LTI 1.3 launch

A school's LMS launches the robot's
educational content through the IMS LTI 1.3 /
Advantage launch flow. The operator's API
validates the LTI launch claim and surfaces
the per-session content envelope.

### §9.2 Curriculum binding

The operator's API publishes the per-platform
educational-outcome declaration so that a
curriculum integrator can bind the platform
to the per-curriculum module declared in the
host institution's WIA-education-integration
records.

## §10 W3C WebRTC Telepresence Integration

A telepresence-robot session integrates with
the school's WebRTC signalling-server
infrastructure per the W3C WebRTC 1.0
specification. The operator's API publishes
the per-session signalling-server reference,
the per-session ICE-and-TURN configuration,
and the operator's privacy declaration.

## §11 Audit and Conformity-Assessment Integration

### §11.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system is
audited under ISO/IEC 17021-1 by an accredited
certification body. The audit result is stored
in the operator's audit envelope.

### §11.2 ISO/IEC 17065 product certification

A robot whose route to market includes a
product-certification mark (the CE marking, a
UL Listed mark, the KR 어린이제품 안전인증 mark)
is bound to the certification body's ISO/IEC
17065:2012 accreditation.

## §12 Public Retrieval and Re-Issuance

### §12.1 Public deployment summary

The operator publishes per-period deployment
statistics on the public-portal endpoint
without per-learner identifiers — total
deployments by school type, total interaction
sessions, total incidents reported.

### §12.2 Verifiable-credentials re-issuance

A robot's CE-marking conformity declaration is
re-issuable as a W3C Verifiable Credential
signed by the notified body's signing-key set
so that a downstream procurement authority can
verify the conformity declaration without
contacting the notified body directly.

## §13 KR-Jurisdiction Integration

### §13.1 KR 학교안전공제회 binding

A school-deployed robot is bound to the KR
학교안전공제회 (School Safety Mutual Aid
Association) so that an incident causing
injury to a learner triggers the per-incident
compensation envelope.

### §13.2 KR 교육부 NEIS binding

A KR-jurisdiction primary or secondary school
deploying the robot binds the per-learner
interaction record to the KR Ministry of
Education's NEIS per the operator's national-
education-data-exchange protocol.

### §13.3 KR 보건복지부 inclusive-education

A KR-jurisdiction inclusive-education robot
deployed under the KR Ministry of Health and
Welfare's special-education programme binds
the per-deployment envelope to the ministry's
special-education register.

## §14 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
education-robot standard. Implementations cite
the ISO / IEC / IEEE / EU / US / KR references
by their issuing organisation and the
publication year so that a downstream consumer
can locate the authoritative text. Updates to
a cited standard (for example, an amendment
to ISO 13482, a new ISO/TS 15066 edition, a
new EU AI Act delegated act) trigger an
internal review cycle in the operator's
quality-management discipline before the new
revision is bound into the operator's
enumeration set.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §15 Refugee and Mobile-Learning Programme Integration

A refugee-education programme operator
deploying education-robots in a refugee
camp's learning facility binds the operator's
per-deployment envelope to the UNHCR Connected
Education Strategy. The operator publishes
the per-deployment envelope to the UNHCR
secretariat's intake endpoint per the
secretariat's information-management
discipline.

## §16 EU AI Act Conformity-Assessment Integration

An education-robot classified as a high-risk
AI system under the EU AI Act undergoes the
conformity assessment per Article 43. The
EU-AI-Act-designated notified body queries
the operator's API for the per-platform AI
documentation (the technical documentation
per Annex IV, the data-governance declaration
per Article 10, the human-oversight envelope
per Article 14, the accuracy-and-robustness
declaration per Article 15) and issues the
per-platform conformity certificate.

## §17 Safety-Information-Sharing Cooperative Integration

An operator participating in a safety-
information-sharing cooperative (the EU CIRC,
the IEEE Robotics and Automation Society
safety-information-sharing programme)
publishes the per-platform incident envelope
and the per-platform field-safety-corrective-
action envelope to the cooperative's intake
endpoint per the cooperative's information-
sharing agreement.

## §18 OECD Education Directorate Robotic-Education Indicators

The OECD Education Directorate publishes
indicators on the use of robotic platforms in
schools. The operator publishes the per-period
deployment dataset (per-platform deployment
count, per-platform supervised-session count,
per-platform incident count) to the OECD
Education Directorate's intake endpoint per
the OECD Education at a Glance reporting
cadence.

## §19 UNESCO ICT-in-Education Integration

A UNESCO-member-state operator participating
in the UNESCO ICT-in-Education programme
publishes the per-deployment educational-
outcome envelope to the UNESCO secretariat's
intake endpoint. The envelope carries the
per-deployment learning-outcome attainment,
the per-deployment teacher-supervision
training record, and the per-deployment
inclusion declaration.

## §20 Continuous Improvement Programme

Each operator publishes an annual improvement
plan addressing post-market-surveillance
findings, recall-effectiveness measurements,
inclusive-education clinical-outcome trends,
and the per-deployment teacher-training
register. The programme is open and the
annual report is published on the operator's
public-portal endpoint.
