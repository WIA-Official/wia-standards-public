# WIA-civic-participation PHASE 3 — PROTOCOL Specification

**Standard:** WIA-civic-participation
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
a public-administration body operating a
citizen-engagement platform across the
operator-to-citizen-to-elected-representative
value chain: the OECD-engagement-ladder
discipline that ties the consultation type to
the citizen's expected influence on the
outcome, the rights-and-policy-expression
discipline that governs how citizen submissions
may be re-used, the deliberation discipline that
governs the operator's documented
decision-making, the outcome-publication
discipline that gates the operator's response
commitment, the privacy-and-consent discipline
that gates the processing of citizen-submission
data under GDPR Article 9 special-category
provisions, the chain-of-custody anchoring
discipline that prevents silent mutation of the
deliberative record, the engagement-KPI
discipline that aligns the operator's reporting
with ITU-T Y.4900 / ISO 37120 / UN DESA / OECD
benchmarks, and the post-decision review
discipline that handles a discovered
nonconformance in the engagement process.

References (CITATION-POLICY ALLOW only):

- ITU-T Recommendation Y.4900/L.1600
- ISO 37120:2018, ISO 37122:2019, ISO 37123:2019,
  ISO 37100:2016, ISO 37101:2016
- ISO 18091:2019 (local government QMS)
- ISO 9001:2015 (quality management systems)
- W3C Open Digital Rights Language (ODRL) 2.2
  Information Model and Vocabulary
- W3C Decentralised Identifiers (DIDs) v1.0
- W3C Verifiable Credentials Data Model v2.0
- W3C Data Catalog Vocabulary (DCAT) v3
- OECD Recommendation of the Council on Open
  Government (OECD/LEGAL/0438) and OECD Citizen
  Engagement: A Guide to Public Engagement
- UN DESA E-Government Survey
- World Bank GovTech Maturity Index
- ISO/IEC 27001:2022 (information-security
  management) and ISO/IEC 17021-1:2015 (audit
  and certification)
- IETF RFC 9110, RFC 9421, RFC 9457, RFC 8615,
  RFC 6962 (the per-event transparency log
  template)
- W3C Trace Context
- EU GDPR (Regulation (EU) 2016/679) Articles
  6, 9, 12-22, 24-30, 32-34
- EU Single Digital Gateway Regulation (EU)
  2018/1724 and EU Interoperable Europe Act
  Regulation (EU) 2024/903
- KR 행정기본법 (Framework Act on
  Administrative Affairs) and KR 정보공개법
  (Official Information Disclosure Act)

---

## §1 OECD Engagement-Ladder Discipline

Every consultation record carries the
`engagementLadder` enumeration declared in
PHASE-1 §3. The operator's API enforces the
consultation-type-to-engagement-ladder mapping:

- `inform` — the operator publishes information
  to the public; no submission is solicited.
- `consult` — the operator solicits citizen
  feedback on a defined policy question.
- `involve` — the operator works with citizens
  to ensure that their concerns are
  consistently understood and considered (a
  public-consultation portal that publishes a
  per-submission rationale-of-non-acceptance).
- `collaborate` — the operator partners with
  citizens in each aspect of the decision
  including the development of alternatives
  (a participatory-budgeting platform where
  citizens propose and rank the projects).
- `empower` — the operator places the final
  decision in the hands of the public (a
  binding referendum, a formal popular vote).

A consultation declared above its actual
engagement scope is rejected at publication
with `422 Unprocessable Entity` at
`/problems/oecd-engagement-ladder-overclaim`.

## §2 Rights-and-Policy-Expression Discipline

### §2.1 W3C ODRL profile

Every consultation record carries a W3C ODRL
2.2 policy expression in `rightsExpression`.
The policy declares the permitted re-use of the
consultation envelope and the per-submission
data — for example, the right to redistribute,
to use for academic research, to publish as
open data after a defined embargo period, to
anonymise after closure. The operator's API
parses the policy and refuses a re-use request
that violates a `prohibition` or `obligation`
clause.

### §2.2 Per-submission sub-policy

A submission may carry a sub-policy that is
more restrictive than the consultation-level
policy (a citizen submitting under their
verified identity may restrict the use to
internal deliberation only). The sub-policy is
reconciled with the consultation policy under
the W3C ODRL evaluation rules; the most-
restrictive policy applies.

## §3 Deliberation Discipline

### §3.1 Documented decision basis

Every deliberation record cites the submission
set that informed the decision. The operator's
API enforces that the cited submissions are
present in the consultation's submission
register; a citation pointing at a submission
that does not exist returns `422 Unprocessable
Entity` at `/problems/deliberation-citation-
unknown`.

### §3.2 Sortition discipline for citizen panels

A `citizen-panel-sortition` deliberation
declares the sortition algorithm parameters in
`participantSet[].selectionMethod`. The
algorithm carries the population frame, the
randomisation seed source (a verifiable
randomness service or a publicly auditable
draw), and the per-participant selection
criteria (age, residence, sortition cohort
size).

### §3.3 Conflict-of-interest discipline

Every deliberation participant declares a
conflict-of-interest envelope per ISO 37001:
2016 anti-bribery management. The conflict-of-
interest envelope is recorded as part of the
participant profile and is reviewed by the
operator's quality manager under PHASE-3 §10
discipline.

## §4 Privacy-and-Consent Discipline

### §4.1 GDPR Article 9 special-category basis

A submission containing health data, political-
opinion data, religious-belief data, or other
GDPR Article 9 special-category data is
processed only under an explicit Article 9(2)
basis. The operator's API records the basis in
`consentDirective.gdprBasis` and refuses a
submission that does not declare a valid basis
where the body's content is detected as
special-category.

### §4.2 Cross-border transfer

Where the submitter's data is transferred
across an EU/EEA boundary to a non-adequate
country, the operator binds the transfer to a
GDPR Article 46 appropriate safeguard (Standard
Contractual Clauses, Binding Corporate Rules,
ad-hoc transfer impact assessment). The
transfer is recorded in the chain-of-custody
record so that the supervisory data-protection
authority can audit the transfer trail.

### §4.3 Right-of-erasure interaction

A citizen submitter exercising the GDPR Article
17 right of erasure triggers the operator's
erasure workflow. Where the submission has
already been cited in a deliberation record,
the erasure is honoured by redacting the
submitter's identity from the public-portal
view but retaining the cited body for the
duration of the operator's record-retention
period under the regulator's retention
discipline; the redaction is recorded as a
chain-of-custody event.

## §5 Outcome-Publication Discipline

### §5.1 Response-schedule honouring

The operator publishes the consultation outcome
on or before the date declared in
`responseSchedule.publishOutcomeBy`. A late
publication is annotated with `publishedLate:
true` in the response envelope and the
underlying delay reason is recorded in the
chain-of-custody record.

### §5.2 Per-submission response

For consultations declared at the `involve` or
`collaborate` engagement level, the operator
publishes a per-submission rationale-of-
non-acceptance where the submission's input
was not adopted. The rationale is bound to the
deliberation record that produced the decision.

## §6 Chain-of-Custody Anchoring Discipline

### §6.1 Per-event transparency log

Every chain-of-custody event carried by PHASE-1
§8 is appended to a per-operator transparency
log modelled on the IETF RFC 6962 Certificate
Transparency append-only-log structure. The
log publishes a signed tree-head every signed-
tree-head period (default 24 h, configurable
per programme); the signed tree-head is
anchored to the operator's public-key set
declared in
`/.well-known/wia/civic-participation/keys.
json`.

### §6.2 Mutation prevention

A custody event cannot be retroactively edited;
an amendment is recorded as a new event with
`previousEventRef` pointing at the event being
amended. The amending event's narrative carries
the reason and is reviewed under the operator's
ISO 9001 §10.2 nonconformity-and-corrective-
action discipline.

## §7 Engagement-KPI Discipline

### §7.1 ITU-T Y.4900 / ISO 37120 binding

The KPI bundle declared in PHASE-1 §7 binds the
operator's engagement performance to the ITU-T
Y.4900 smart-city engagement KPI series and
the ISO 37120/37122 indicator catalogue. The
operator's API enforces that the per-indicator
value falls within the indicator's declared
unit envelope (a percentage value reported in
the [0,100] range, a time-to-respond reported
in days).

### §7.2 UN DESA / OECD benchmark binding

The operator's reporting bundle additionally
carries the UN DESA E-Participation Index sub-
component and the OECD OURdata Index sub-
component for the reporting period so that a
cross-jurisdictional benchmark is preserved.

## §8 Sortition-and-Random-Selection Discipline

A citizen panel selected by sortition uses a
verifiable random-selection mechanism. The
random seed is published in the deliberation
envelope so that the selection can be re-run by
an independent observer. The selection's
demographic profile is published as a
per-cohort summary so that the panel's
representativeness can be audited.

## §9 Disinformation-Resilience Discipline

### §9.1 Bot-detection envelope

The operator runs a bot-detection layer at
submission-intake time. The detection layer's
result (suspected, not-suspected) is recorded
as a chain-of-custody event but does not
unilaterally block the submission; suspect
submissions are flagged for human review by
the operator's secretariat under the operator's
documented review discipline.

### §9.2 Coordinated-inauthentic-behaviour layer

The operator publishes a per-consultation
report on coordinated inauthentic behaviour
detected during the consultation (mass-
template submissions, coordinated voting
clusters). The report is reviewed by the
operator's quality manager and the
transparency observer.

## §10 Quality-Management Discipline

The operator runs an ISO 9001:2015 quality
management system, with the local-government
adaptation per ISO 18091:2019, covering the
consultation-publication, submission-intake,
deliberation, outcome-publication, KPI-
publication, and chain-of-custody processes.
Internal audits run on a frequency declared in
the quality manual; the nonconformity register
is reviewed in the ISO 9001 §9.3 management-
review cycle. The operator's information-
security management system declared under
ISO/IEC 27001:2022 covers the protection of
citizen-submission data.

## §11 Post-Decision Review Discipline

### §11.1 Adverse-finding review

Where a transparency observer's audit produces
an adverse finding (a missing submission, a
decision-record citation pointing at a non-
existent submission, a delayed outcome
publication), the operator's API records the
finding as a nonconformity event and runs the
ISO 9001 §10.2 corrective-action workflow.

### §11.2 Public-record reconciliation

The operator publishes the reconciliation
summary on the public-portal so that a
downstream consumer can see the corrective
action and the date of closure.

## §12 KR-Jurisdiction Discipline

### §12.1 KR 행정기본법 binding

A KR-jurisdiction operator binds the
consultation envelope to the relevant article
of KR 행정기본법 (Framework Act on
Administrative Affairs) and KR 행정절차법
(Administrative Procedure Act) so that the
consultation's procedural validity is
preserved.

### §12.2 KR 정보공개법 disclosure discipline

A KR-jurisdiction operator handles a KR
information-disclosure request under KR 정보
공개법 (Official Information Disclosure Act).
The operator publishes the per-request
disclosure decision and the underlying
documentary record per the act's discipline.
