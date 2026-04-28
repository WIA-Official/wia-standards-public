# WIA-education-integration PHASE 4 — Integration Specification

**Standard:** WIA-education-integration
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This document defines how an education-integration
operator integrates with the systems that
surround cross-jurisdictional education: the
UNESCO Institute for Statistics collecting the
education-statistics return; the OECD Education
Directorate producing the Education at a Glance
indicator framework; the European Commission's
Erasmus+ programme operating the Europass
platform; the national education ministry
maintaining the national qualifications
framework; the national accreditation authority
operating the higher-education accreditation
register; the Bologna Follow-Up Group monitoring
the Bologna Process implementation; the host
LMS operating the IMS LTI 1.3 launch endpoint;
the supervisory data-protection authority
overseeing the per-learner processing under
GDPR or equivalent; and the inter-institutional
recognition authority issuing the Lisbon /
Tokyo / Global Recognition Convention decision.

References (CITATION-POLICY ALLOW only):

- UNESCO Institute for Statistics (UIS) data
  submission protocol, OECD Education at a
  Glance indicator framework
- European Commission Erasmus+ programme
  (Regulation (EU) 2021/817) and Europass
  platform (Decision (EU) 2018/646)
- UNESCO ISCED 2011, ISCED-F 2013, Lisbon
  Recognition Convention 1997, Tokyo
  Convention 2011, Global Convention 2019
- Bologna Process (the European Higher
  Education Area), Bologna Follow-Up Group
- ISO 21001:2018, ISO/IEC 19796-1:2005, ISO/IEC
  17021-1:2015, ISO/IEC 17065:2012, ISO/IEC
  27001:2022
- IMS Global LTI 1.3, OneRoster 1.2, Caliper
  Analytics 1.2, QTI 3.0, Open Badges 3.0
- W3C Verifiable Credentials Data Model v2.0,
  W3C Decentralized Identifiers v1.0
- IETF RFC 8259, RFC 9457, RFC 8615, RFC 9421
- W3C Trace Context, W3C ODRL 2.2
- EU GDPR (Regulation (EU) 2016/679), KR 개인
  정보 보호법
- KR 고등교육법, KR 학점인정 등에 관한 법률, KR
  자격기본법, KR 평생교육법
- ASEAN Qualifications Reference Framework
  (AQRF)

---

## §1 UNESCO UIS Integration

The operator's national education ministry
submits the operator's per-period education-
statistics return through the UNESCO Institute
for Statistics submission endpoint. The
submission carries the per-ISCED-level
enrolment count, the per-ISCED-field graduate
count, the per-jurisdictional intake-and-
outflow mobility flow, and the per-period
budget allocation envelope.

## §2 OECD Education Directorate Integration

The operator's national education ministry
submits the operator's per-cycle data to the
OECD Education Directorate for the OECD
Education at a Glance indicator framework. The
submission carries the per-indicator dataset —
expected years of schooling, attainment by
ISCED level, expenditure per student,
graduation rate.

## §3 Erasmus+ and Europass Integration

A European-Commission-recognised mobility
programme participating in Erasmus+ binds the
operator's mobility records to the Erasmus+
programme's identifier. The Europass platform
re-issues the learner's credential as an
Europass digital credential signed by the
issuing operator's signing key, anchored to
the Europass platform's public-key set.

## §4 National Accreditation Authority Integration

The operator queries the national
accreditation authority's higher-education
accreditation register on each programme
registration to verify the operator's
accreditation reference. The register is
operated by the national authority designated
under the operator's national education law
(the Korean University Accreditation Council
KUAC, the German Akkreditierungsrat, the US
Council for Higher Education Accreditation
CHEA).

## §5 Bologna Follow-Up Group Integration

A European Higher Education Area participant
publishes the per-period implementation
report to the Bologna Follow-Up Group
endpoint. The report carries the operator's
implementation of the Bologna learning-
outcomes framework, the Diploma Supplement,
the European Credit Transfer and Accumulation
System (ECTS), and the European Standards and
Guidelines for Quality Assurance (ESG-QA).

## §6 Host LMS Integration

### §6.1 LTI 1.3 launch

The host LMS launches the operator's
curriculum through the IMS LTI 1.3 / Advantage
launch flow. The operator's API validates the
LTI launch claim signed by the platform's
JWKS-published key.

### §6.2 OneRoster roster sync

The host LMS synchronises the operator's
roster via IMS OneRoster 1.2 with the per-
learner enrolment and per-class section
binding.

### §6.3 Caliper analytics

The host LMS forwards Caliper events to the
operator's analytics endpoint for the
operator's per-curriculum analytics
programme.

## §7 Supervisory Data-Protection Authority Integration

The supervisory data-protection authority
overseeing the operator's per-learner
processing audits the operator's records of
processing activities on demand. Where the
operator processes personal data under GDPR,
the operator publishes the records of
processing activities per Article 30 to the
authority's endpoint. Where the operator
processes personal data under KR 개인정보 보호
법, the operator publishes the per-period
privacy-impact-assessment summary per §33 to
the KR personal-information-protection
commission's endpoint.

## §8 Inter-Institutional Recognition Authority Integration

A recognition authority designated under the
Lisbon / Tokyo / Global Recognition Convention
queries the operator's API on each
recognition application to verify the
operator's accreditation status, the
applicant's qualification record, and the
applicant's transcript. The authority issues
the recognition decision through the
operator's API per PHASE-2 §6.

## §9 Audit and Conformity-Assessment Integration

### §9.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system
declared in PHASE-3 §4 is audited under
ISO/IEC 17021-1 by an accredited certification
body. The audit result is stored in the
operator's audit envelope and is referenced
from the programme record's
`accreditationStatus`.

### §9.2 ISO/IEC 17065 product certification

A programme whose route to market includes a
product-certification mark (a national
accreditation mark, an Erasmus+ programme
quality label) is bound to the certification
body's ISO/IEC 17065:2012 accreditation. The
certification body's marking, scope of
certification, and the certified programme's
identifier are published so that a downstream
consumer can verify the certification before
relying on it.

## §10 Public Retrieval and Re-Issuance

### §10.1 Public programme summary

The operator publishes per-period programme
statistics on the public-portal endpoint
without per-learner identifiers. The
aggregated dataset is consumed by researchers,
journalists, and policy analysts.

### §10.2 Verifiable-credentials re-issuance

A learner's diploma or transcript is re-
issuable as a W3C Verifiable Credential signed
by the operator's signing-key set so that a
downstream recognition authority can verify
the credential without contacting the issuing
operator directly.

## §11 KR-Jurisdiction Integration

### §11.1 KR 교육부 NEIS integration

A KR-jurisdiction operator's primary and
secondary education record binds to the KR
Ministry of Education's NEIS (National
Education Information System) per the
operator's national-education-data-exchange
protocol.

### §11.2 KR 한국대학교육협의회 integration

A KR-jurisdiction higher-education operator
binds the per-programme accreditation to the
Korean Council for University Education
(KCUE)'s programme register per the council's
recognition register.

### §11.3 KR 학점은행제 integration

A KR-jurisdiction operator participating in
the KR Credit Bank System binds the per-
credit recognition envelope to the National
Institute for Lifelong Education (NILE)'s
credit-bank platform.

### §11.4 KR 평생교육법 integration

A KR-jurisdiction lifelong-education operator
binds the per-programme registration to the
KR 평생교육법 (Lifelong Education Act) and the
KR Lifelong Education Promotion Council
secretariat's register.

## §12 Regional Integration

### §12.1 ASEAN Qualifications Reference Framework

An ASEAN-Member-State operator binds the
operator's per-programme qualification to the
ASEAN Qualifications Reference Framework
(AQRF) reference level so that a downstream
ASEAN-Member-State recognition authority can
deterministically interpret the qualification.

### §12.2 EU EQF binding

A European-Higher-Education-Area participant
binds the operator's per-programme award to
the European Qualifications Framework for
Lifelong Learning (EQF) reference level per
EQF Recommendation 2017/C 189/03.

## §13 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
education-integration standard. Implementations
cite the multilateral and intergovernmental
agreements (UNESCO ISCED, Lisbon / Tokyo /
Global Recognition Conventions, Erasmus+ /
Europass, Bologna Process, ASEAN Qualifications
Reference Framework, EQF) and the ISO / IEC /
IETF / W3C / IMS / EU / KR references by their
issuing organisation and the publication year
so that a downstream consumer can locate the
authoritative text. Updates to a cited
standard (for example, the next ISCED
revision, a new ISO 21001 amendment, a new
IMS LTI release) trigger an internal review
cycle in the operator's quality-management
discipline declared in PHASE-3 §4 before the
new revision is bound into the operator's
enumeration set.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §14 Audit and Inspector Replay Payload

For a national accreditation authority's on-
site audit, the operator publishes the
inspector-replay payload covering the per-
programme registration, the per-curriculum
publication, the per-learner enrolment, the
per-recognition-decision issuance, the per-
mobility-flow recording, the per-credential
issuance, and the chain-of-custody export.
The payload is signed using the operator's
audit-chain JWS key so that the inspector
can verify integrity end-to-end without
trusting the operator at runtime.

## §15 Refugee-Education Programme Integration

A UNHCR-recognised refugee-education
programme operator binds the operator's
per-learner registration to the UNHCR
Connected Education Strategy. The operator
publishes the per-learner credential as a
W3C Verifiable Credential signed by the
UNHCR-recognised authority's signing-key set
so that the refugee learner's qualification
is portable across jurisdictions.
