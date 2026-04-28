# WIA-education-integration PHASE 3 — Protocol Specification

**Standard:** WIA-education-integration
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
an education-integration operator across the
learner-to-institution-to-recognition-authority-
to-statistics value chain: the UNESCO ISCED 2011
classification discipline that anchors every
qualification record, the Lisbon / Tokyo /
Global Recognition Convention discipline that
gates a cross-jurisdictional recognition
decision, the Bologna learning-outcomes
framework that scores a curriculum's outcome
declaration, the ISO 21001 educational-
organisation management discipline that runs
the operator's quality-management cycle, the
ISO/IEC 19796-1 e-learning quality discipline
that gates a digital-curriculum publication, the
chain-of-custody anchoring discipline that
prevents silent mutation of the learner's
academic record, the privacy-discipline anchored
to the EU GDPR / KR 개인정보 보호법 / OECD
Privacy Guidelines, and the appeal-and-review
discipline under the recognition conventions.

References (CITATION-POLICY ALLOW only):

- UNESCO ISCED 2011, ISCED-F 2013
- UNESCO Convention on the Recognition of
  Qualifications concerning Higher Education
  (the Global Convention, 2019), Tokyo
  Convention (2011), Lisbon Recognition
  Convention (1997)
- Bologna Process Working Group on
  Qualifications Framework (the Bologna
  learning-outcomes framework)
- ISO 21001:2018, ISO/IEC 19796-1:2005, ISO/IEC
  23988:2007, ISO/IEC 24751-1:2008
- IMS Global LTI 1.3, OneRoster 1.2, Caliper
  Analytics 1.2, QTI 3.0, Open Badges 3.0
- W3C Verifiable Credentials Data Model v2.0,
  W3C Decentralized Identifiers v1.0
- ADL xAPI 2.0
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012
- IETF RFC 9110, RFC 9421, RFC 9457, RFC 6234,
  RFC 8615, RFC 6962
- W3C Trace Context, W3C ODRL 2.2
- EU GDPR (Regulation (EU) 2016/679), OECD
  Privacy Guidelines 2013, KR 개인정보 보호법
- KR 고등교육법, KR 학점인정 등에 관한 법률, KR
  자격기본법

---

## §1 ISCED 2011 Classification Discipline

### §1.1 Per-programme level binding

Every programme record carried by the
operator's API is bound to a UNESCO ISCED 2011
level (0 through 8). The operator's API
publishes the per-element mapping table from
the operator's national-qualifications-
framework level to the ISCED 2011 level so
that a downstream recognition authority can
deterministically interpret the programme.

### §1.2 Per-programme field binding

Every programme is bound to an ISCED-F 2013
narrow-field-of-education three-digit code.
The operator's API maintains the per-field
crosswalk to the operator's national field
classification.

### §1.3 ISCED revision absorption

UNESCO publishes a periodic revision of the
ISCED classification (the next revision is
under development). The operator subscribes
to UNESCO's publishing endpoint and triggers
an internal review cycle when a revision is
published.

## §2 Recognition Convention Discipline

### §2.1 Convention-to-authority binding

The Global / Tokyo / Lisbon Recognition
Conventions designate competent authorities
empowered to issue recognition decisions. The
operator's API binds every recognition
decision to the issuing authority's
designation reference and rejects a decision
whose authority is not designated under the
applicable convention.

### §2.2 Substantial-difference test

Per Lisbon Article III.5 (and the analogous
articles of the Tokyo and Global Conventions),
the recognition authority applies the
substantial-difference test to the presented
qualification. A finding of substantial
difference must be reasoned in writing and
made available to the applicant.

### §2.3 Fair-procedure discipline

The recognition authority applies the fair-
procedure discipline declared in the
convention — the applicant's right to a
written decision, the applicant's right to
appeal, the authority's reasonable-time-frame
obligation. The operator's API records the
per-step timestamps so that the fair-procedure
audit trail is preserved.

## §3 Bologna Learning-Outcomes Framework Discipline

A curriculum record's per-module
`learningOutcomes` declaration is anchored to
the Bologna learning-outcomes framework. The
operator's API validates that each module
declares at least one outcome under each of
the framework's three dimensions (knowledge,
skills, autonomy and responsibility) at the
module's declared level.

## §4 ISO 21001 Quality-Management Discipline

The operator runs an ISO 21001:2018 educational-
organisation management system covering the
programme registration, the curriculum
publication, the learner enrolment, the
recognition-decision issuance, the mobility-
flow recording, and the chain-of-custody
processes. Internal audits run on a frequency
declared in the quality manual; the
nonconformity register is reviewed in the
ISO 21001 §9.3 management-review cycle.

## §5 ISO/IEC 19796-1 E-Learning Quality Discipline

A digital curriculum is published only after
the per-curriculum ISO/IEC 19796-1 quality-
assurance evaluation is recorded against the
operator's quality-assurance authority. The
evaluation is bound to the per-process
quality reference model — needs analysis,
framework analysis, conception, development,
implementation, and evaluation.

## §6 IMS Global Standards Discipline

### §6.1 LTI 1.3 launch authentication

A curriculum's LTI launch follows the IMS
LTI 1.3 / Advantage authentication flow with
the OAuth 2 client-credentials profile. The
operator's API validates the LTI message
signature against the issuing platform's
public-key set published at the platform's
JWKS endpoint.

### §6.2 OneRoster roster synchronisation

The operator's roster is synchronised with the
host institution via IMS OneRoster 1.2. The
roster carries the per-learner enrolment, the
per-course teacher assignment, and the per-
class section binding.

### §6.3 Caliper analytics envelope

Caliper events flowing from the host LMS to
the operator's analytics endpoint are
enveloped per IMS Caliper 1.2 with the W3C
ld-context binding.

### §6.4 QTI 3.0 assessment binding

A curriculum's per-module assessment is
encoded per IMS QTI 3.0 so that a downstream
LMS can deliver the assessment without
operator-specific transformation.

### §6.5 Open Badges 3.0 issuance

A learner's micro-credential is re-issuable
as an IMS Open Badges 3.0 badge signed by the
operator's signing-key set.

## §7 W3C Verifiable Credentials Discipline

### §7.1 Diploma re-issuance

A learner's diploma is re-issuable as a W3C
Verifiable Credential per the Verifiable
Credentials Data Model v2.0. The credential
carries the issuing operator's identifier
(W3C DID v1.0), the recognition authority's
counter-signature, and the credential's
expiry-and-revocation envelope.

### §7.2 Credential revocation registry

The operator publishes a status list of
revoked credentials. A downstream consumer
verifying a credential queries the status
list to confirm the credential is not revoked.

## §8 Chain-of-Custody Anchoring Discipline

### §8.1 Per-event transparency log

Every chain-of-custody event carried by
PHASE-1 §8 is appended to a per-operator
transparency log modelled on the IETF RFC 6962
Certificate Transparency append-only-log
structure.

### §8.2 Mutation prevention

A custody event cannot be retroactively
edited; an amendment is recorded as a new
event with `previousEventRef` pointing at the
event being amended.

## §9 Privacy and Identity-Vault Discipline

### §9.1 Pseudonym-and-vault separation

The operator publishes the learner's
pseudonymous identifier on its API surface
and holds the linkage to the directly-
identifying record in an identity vault.
Access to the vault is gated on the
operator's role-based access-control policy.

### §9.2 GDPR / KR 개인정보 보호법 binding

A learner whose data is processed under EU
jurisdiction is processed per GDPR Articles 6
(lawful basis), 9 (special-category data
where applicable), 12-22 (data subject
rights), 24-30 (controller obligations),
32-34 (security and breach notification). A
learner whose data is processed under KR
jurisdiction is processed per KR 개인정보
보호법 §15 (lawful basis), §17 (third-party
provision), §18 (out-of-purpose use), §28-2
(pseudonymisation), and §35 (subject access).

## §10 Appeal-and-Review Discipline

### §10.1 Per-decision review right

An applicant contesting a recognition decision
lodges an administrative review under the
recognition convention's review-and-appeal
procedure (Lisbon Article III.5, Tokyo
Article V.5, Global Article XI).

### §10.2 Per-decision binding under review

The contested decision remains binding
pending the review outcome, except where the
recognition authority's national law provides
for a stay.

## §11 KR-Jurisdiction Discipline

### §11.1 KR 고등교육법 binding

A KR-jurisdiction operator binds the
programme's accreditation to the relevant
article of KR 고등교육법 (Higher Education
Act). The KR Ministry of Education publishes
the accreditation register; the operator's
API references the register on each
retrieval.

### §11.2 KR 학점인정 binding

The KR 학점인정 등에 관한 법률 (Act on
Credit-Bank System) governs the recognition
of credits accumulated outside the formal
higher-education system. The operator's API
publishes the per-credit recognition envelope
to the KR Credit-Bank System operated by the
National Institute for Lifelong Education.

### §11.3 KR 자격기본법 binding

The KR 자격기본법 (Framework Act on
Qualifications) governs the national
qualifications framework. The operator binds
the per-programme award to the KR National
Competency Standards (NCS) reference.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §12 Public-Sector Procurement Discipline

A public-sector procurement programme
selecting an education-integration operator
applies the WTO Government Procurement
Agreement (GPA) discipline where the
operator's procurement is in scope. The
operator publishes the per-tender entry with
the procurement timetable, the technical
specifications, and the award criteria so
that a foreign supplier can compete on the
same terms as a domestic supplier.

## §13 Student-Mobility Visa Coordination

A learner crossing a border for a programme
of study is bound by the destination
jurisdiction's student-visa requirement. The
operator's API publishes the per-mobility
visa-letter envelope to the learner so that
the learner can present the visa-letter to the
destination consulate. The visa-letter envelope
is signed by the operator's signing-key set;
the destination consulate verifies the
envelope before issuing the student visa.

## §14 Erasmus Without Paper Discipline

A European-Higher-Education-Area operator
participating in the Erasmus+ programme binds
the operator's mobility records to the
Erasmus Without Paper (EWP) network operated
by the European Commission. The operator's
API publishes the per-mobility envelope to
the EWP endpoint per the EWP technical
specifications so that the receiving
institution can ingest the mobility envelope
without operator-specific transformation.
