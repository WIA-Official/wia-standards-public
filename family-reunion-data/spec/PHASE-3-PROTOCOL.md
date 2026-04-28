# WIA-family-reunion-data PHASE 3 — PROTOCOL Specification

**Standard:** WIA-family-reunion-data
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
a humanitarian operator across the registered-
person-to-tracing-request-to-identification-
anchor value chain: the humanitarian-principle
discipline that anchors every operator action
to neutrality, impartiality, independence, and
humanity, the consent-and-data-protection
discipline under GDPR Article 9 / UNHCR
Personal Data Policy / IOM Data Protection
Manual, the special-protection discipline for
unaccompanied or separated minors under the
1989 CRC, the inter-state-transfer discipline
under GDPR Article 46-49 / VCCR Article 36 /
Hague-1980 / Hague-1993, the biometric-data
discipline that gates the use of fingerprints,
iris scans, and face data under the operator's
biometric policy, the chain-of-custody
anchoring discipline that prevents silent
mutation of the case file, the inter-agency
coordination discipline that aligns the
operator's case file with the IASC cluster,
the security-and-confidentiality discipline
that protects the registered person against
adverse re-identification, and the post-case-
closure retention discipline.

References (CITATION-POLICY ALLOW only):

- 1949 Geneva Conventions and 1977 Additional
  Protocols I and II
- 1951 Refugee Convention and 1967 Protocol
- 1989 CRC (especially Articles 7, 8, 9, 10,
  11, 20, 22, 35)
- 1993 Hague Convention on Inter-Country
  Adoption
- 1980 Hague Convention on the Civil Aspects
  of International Child Abduction
- 1963 Vienna Convention on Consular Relations
  Article 36
- 1961 Convention on the Reduction of
  Statelessness
- ICRC Restoring Family Links Strategy and
  ICRC Code of Conduct
- ICRC Professional Standards for Protection
  Work
- UNHCR ProGres v4 schema and UNHCR Policy on
  the Protection of Personal Data of Persons
  of Concern
- IOM Displacement Tracking Matrix
  Methodological Framework and IOM Data
  Protection Manual
- HXL Standard (the OCHA-maintained
  humanitarian-data tagging standard)
- Sphere Handbook
- ISO 19115-1/-2, ISO 6709, ISO 3166-1/-2,
  ISO 5218, ISO 639, ISO 4217
- ISO 9001:2015, ISO/IEC 27001:2022, ISO/IEC
  27018:2019
- ISO/IEC 17021-1:2015
- IETF RFC 9110, RFC 9421, RFC 9457, RFC 6234,
  RFC 8032, RFC 6962
- W3C Trace Context, W3C ODRL 2.2, W3C VC
  v2.0
- EU GDPR Articles 6, 9, 12-22, 25, 32, 33-34,
  46-49, 89
- EU Council Directive 2003/86/EC
- KR 출입국관리법, KR 난민법, KR 아동복지법
  (Child Welfare Act), KR 개인정보보호법

---

## §1 Humanitarian-Principle Discipline

Every operator action carried by the API is
anchored to the four humanitarian principles
declared in the ICRC RFL Strategy: humanity,
impartiality, neutrality, and independence.
The operator's API rejects an operator action
that is conditioned on a discriminatory
attribute (nationality, ethnicity, religious
belief) where the discrimination is unrelated
to the case's protection requirement; the
rejection is recorded as a nonconformity event
under PHASE-3 §10.

## §2 Consent-and-Data-Protection Discipline

### §2.1 GDPR Article 9 special-category basis

Every person record contains GDPR Article 9
special-category data (health-related
vulnerability flags, biometric identifiers
where collected, ethnicity-related fields).
The operator's API records the per-record
Article 9(2) processing basis — typically
Article 9(2)(b) (employment / social
security), Article 9(2)(c) (vital interests),
Article 9(2)(d) (legitimate activity of a
foundation, association, or non-profit body),
or Article 9(2)(g) (substantial public
interest under EU or Member-State law).

### §2.2 UNHCR Personal Data Policy purpose
       binding

A UNHCR-coordinated operation binds the per-
record processing to the per-purpose
declaration under the UNHCR Personal Data
Policy. The operator's API enforces purpose-
binding through the bearer-token's declared
purpose claim and refuses a retrieval request
whose declared purpose is outside the per-
record purpose declaration.

### §2.3 IOM Data Protection Manual binding

An IOM-coordinated operation binds the per-
record processing to the per-purpose
declaration under the IOM Data Protection
Manual.

### §2.4 Privacy-by-design discipline

The operator runs a privacy-impact assessment
on each new processing activity per GDPR
Article 35 and IOM Data Protection Manual
§3. The assessment outcome is recorded in the
operator's audit envelope.

## §3 Special-Protection Discipline for Minors

### §3.1 CRC Article 22 refugee-children
       protection

A `separated-minor-unaccompanied` registration
is bound to the operator's child-protection
authority's case file per CRC Article 22 and
the operator's national child-protection law.

### §3.2 Best-interests-of-the-child assessment

Every action affecting a minor is taken
following a best-interests-of-the-child
assessment per CRC Article 3. The assessment
is recorded in the per-action audit envelope.

### §3.3 Appointed guardian binding

The operator's API records the
`appointedGuardianRef` field for every
unaccompanied minor; the field references the
appointed guardian under the operator's
national child-protection law and is signed
by the appointing authority.

### §3.4 Hague-1993 inter-country adoption
       discipline

Where the case is in scope of the 1993 Hague
Convention, the operator's API records the
both-central-authority consent under the
Convention's Article 17 framework before any
inter-state placement is initiated.

### §3.5 Hague-1980 child-abduction discipline

Where the case involves a cross-border
abduction in scope of the 1980 Hague
Convention, the operator's API binds the
case to the Hague-1980 Central Authority of
the destination state and records the per-
case return-application reference.

## §4 Inter-State-Transfer Discipline

### §4.1 GDPR Article 46-49 appropriate safeguard

A transfer of personal data to a non-adequate
third country is bound to a GDPR Article 46
appropriate safeguard (Standard Contractual
Clauses, Binding Corporate Rules, an approved
Code of Conduct, or an approved Certification
Mechanism), or to an Article 49 derogation
where applicable. The operator's API records
the per-transfer safeguard reference.

### §4.2 VCCR Article 36 consular notification

Where the registered person is a foreign
national entitled to consular notification per
VCCR Article 36, the operator's API records
the per-case notification status (notified,
notification-declined-by-the-registrant,
notification-pending).

### §4.3 Refugee non-refoulement discipline

A transfer of a refugee or asylum-seeker is
gated on the non-refoulement principle of the
1951 Refugee Convention Article 33. The
operator's API rejects a transfer to a state
where the registered person would face a
threat to life or freedom on account of race,
religion, nationality, membership of a
particular social group, or political opinion.

## §5 Biometric-Data Discipline

### §5.1 Operator biometric policy binding

Biometric data (fingerprints, iris scans, face
images) is collected only under the operator's
documented biometric policy. The policy
declares the per-purpose collection basis, the
retention period, and the per-purpose
disclosure scope.

### §5.2 Biometric match-evidence handling

A biometric match used as evidence in an
identification anchor (PHASE-1 §5) is reviewed
by the supervising protection officer; the
biometric match score, the threshold, and the
match-method identifier are recorded in the
attestation envelope.

### §5.3 Biometric-deletion-on-closure rule

Biometric templates are deleted on case
closure per the operator's documented retention
schedule, except where an Article 89 archival
basis or a statutory retention obligation
applies. The deletion is recorded as a chain-
of-custody event.

## §6 Chain-of-Custody Anchoring Discipline

### §6.1 Per-event transparency log

Every chain-of-custody event carried by PHASE-1
§8 is appended to a per-operator transparency
log modelled on the IETF RFC 6962 Certificate
Transparency append-only-log structure. The
log is operated within the operator's secure
processing environment and is not exposed to
external consumers without the protection
officer's authorisation.

### §6.2 Mutation prevention

A custody event cannot be retroactively edited;
an amendment is recorded as a new event with
`previousEventRef` pointing at the event being
amended.

## §7 Inter-Agency Coordination Discipline

### §7.1 IASC cluster binding

Where the operation is in scope of an IASC
cluster (Protection cluster, Camp Coordination
and Camp Management cluster), the operator's
HXL feed is bound to the cluster's coordination
endpoint so that the cluster's information-
management officer can aggregate the
operator's data into the cluster-level dataset.

### §7.2 OCHA 3W coordination

Where the operation contributes to the OCHA
3W (Who-What-Where) operational picture, the
operator's HXL feed is published with the
OCHA-defined HXL tags (`#org`, `#sector`,
`#adm1`, `#adm2`, `#status`).

## §8 Security-and-Confidentiality Discipline

### §8.1 ISO/IEC 27001 + 27018 binding

The operator's information-security management
system is certified under ISO/IEC 27001:2022
and ISO/IEC 27018:2019 (PII protection in
public clouds). The operator's API publishes
the per-deployment certification envelope.

### §8.2 Adverse-re-identification risk

Where a coordination dataset carries fields
that could allow adverse re-identification of
a registered person (the combination of an
ethnic descriptor, a geographic area, and a
date of arrival), the operator's API applies
the operator's documented k-anonymity or
suppression rules before publication.

## §9 Quality-Management Discipline

The operator runs an ISO 9001:2015 quality
management system covering the registration,
tracing, identification, transfer, HXL
coordination, and case-closure processes.
Internal audits run on a frequency declared
in the quality manual.

## §10 Post-Case-Closure Retention Discipline

### §10.1 Per-purpose retention schedule

The operator publishes a per-purpose retention
schedule that declares how long each category
of data is retained after case closure (a
positive-identification case may retain the
case file for the longer of the operator's
declared retention period or any statutory
obligation, while a closed-no-result tracing
request retains the file for a shorter
declared period).

### §10.2 Right-of-erasure interaction

A registered person exercising the GDPR Article
17 right of erasure on a non-essential field
triggers the operator's erasure workflow. The
erasure is honoured except where Article
17(3)(b) (compliance with a legal obligation
under EU or Member-State law) or Article
17(3)(d) (archival processing in the public
interest) applies.

### §10.3 Per-jurisdiction archival basis

Where a case file is retained under an Article
89 archival basis, the file is migrated to the
operator's archival tier and the migration is
recorded as a chain-of-custody event.

## §11 KR-Jurisdiction Discipline

### §11.1 KR 출입국관리법 + KR 난민법 binding

A KR-jurisdiction operator binds the
registration to the relevant article of KR
출입국관리법 (Immigration Control Act) and
KR 난민법 (Refugee Act).

### §11.2 KR 아동복지법 binding

A KR-jurisdiction operator handling an
unaccompanied or separated minor binds the
case file to KR 아동복지법 (Child Welfare
Act) and the appointed guardian's
authorisation envelope under KR 미성년자
보호 법령.

### §11.3 KR 개인정보보호법 sensitive-information
       binding

A KR-jurisdiction operator binds the per-
record processing to KR Personal Information
Protection Act Article 23 sensitive-
information processing basis where the record
includes special-category data.
