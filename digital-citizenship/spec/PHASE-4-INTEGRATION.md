# WIA-digital-citizenship PHASE 4 — INTEGRATION Specification

**Standard:** WIA-digital-citizenship
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a digital-citizenship
programme integrates with the systems that surround
it: the operating jurisdiction's education ministry
or programme funder; civil-society partner
organisations; the operating jurisdiction's online-
safety authority (where the programme operates an
escalation channel for online-safety incidents); the
operating jurisdiction's child-protection and
safeguarding agencies; the parents and guardians of
minor learners (through the operator's identity-
verified parent-portal flow); the programme's
external evaluators; the operating jurisdiction's
data-protection authority for GDPR-Article-8 / COPPA
/ Age-Appropriate Design Code privacy regime; the
contracted accessibility auditor for WCAG 2.2 Level
AA conformance; and long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17021-1:2015 (management-system audit and
  certification)
- ISO/IEC 19796-1:2005 (learning, education and
  training — quality management)
- ISO 8601 (date and time)
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Education Ministry / Programme Funder Integration

The operating jurisdiction's education ministry (for
public-school programmes) or programme funder (for
library, NGO, and e-government literacy programmes)
consumes funder reports and audits programme
delivery. Integration carries the funder's identifier,
the per-period reporting endpoint, the per-period
budget-execution submission, the per-grant audit
workflow, and the funder's response SLA.

For EU-jurisdiction programmes funded under the EU
Digital Education Action Plan 2021-2027, integration
carries the European Commission's per-grant
coordinator identifier and the Plan's reporting
template.

## §2 Civil-Society Partner Organisation Integration

Civil-society partner organisations (NGO partners,
academic-research consortia, library consortia)
integrate through cohort-delivery agreements.
Integration carries each partner's identifier, the
per-cohort responsibility-matrix reference, the per-
cohort cooperation record, and the per-cycle review
cadence.

## §3 Online-Safety Authority Integration

For programmes operating in jurisdictions with an
online-safety authority (UK Ofcom under the Online
Safety Act, EU Member State regulators under the
Digital Services Act, US FTC for COPPA enforcement)
integration carries the authority's identifier, the
per-incident escalation endpoint, the per-incident
acknowledgement workflow, and the per-period
aggregate-statistics submission.

## §4 Child-Protection and Safeguarding Agency Integration

The operating jurisdiction's child-protection agency
(US child-protection services per state, UK local-
authority safeguarding boards, EU Member State child-
protection agencies, KR child-protection agency)
consumes safeguarding escalations beyond the
programme's internal-resolution capacity. Integration
carries the agency's identifier, the per-escalation
intake endpoint, the per-case cooperation workflow,
and the per-case status-update channel.

## §5 Parent-Portal Integration

The operator's parent-portal flow gates parents'
access to the per-learner record:

- per-parent identity verification (the operator's
  multi-factor identity-verification flow);
- per-parent access to the learner's enrolment,
  module-completion, and pastoral-record subset
  appropriate for parental review;
- per-parent withdrawal-of-consent flow (parents may
  withdraw consent under COPPA / GDPR Article 7(3)
  / UK Age-Appropriate Design Code at any time);
- per-parent access to the operator's privacy notice
  in plain age-appropriate language and in the
  operating jurisdiction's official languages.

## §6 External Evaluator Integration

External evaluators commissioned by the funder consume
audit-trail exports through dedicated client
certificates. The export carries the API audit logs
for the audit window, the cohort enrolment and
completion records (with learner identities anonymised
per the operator's anonymisation policy), the
inclusive-access provision use, the online-safety-
incident aggregate, and the funder-reporting cycle's
narrative reports.

## §7 Data-Protection Authority Integration

The operating jurisdiction's data-protection authority
(US FTC for COPPA, UK ICO for the Age-Appropriate
Design Code, EU Member State DPAs for GDPR Article 8,
KR PIPC for KR-PIPA) integrates through the
operator's parental-consent and minor-learner-data
discipline. Integration carries the DPA's identifier,
the per-inquiry cooperation workflow, the per-
investigation response workflow, and the per-cycle
voluntary disclosure channel.

## §8 Accessibility Auditor Integration

The contracted accessibility auditor consumes the
programme's curriculum-module and platform artefacts
for per-cycle WCAG 2.2 Level AA conformance audit.
Integration carries the auditor's identifier, the
per-cycle audit-report archive, the per-finding
remediation tracker, and the per-cycle scope
statement.

## §9 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  curriculum-modules/          — module records with
                                  WCAG conformance
                                  evidence
  cohorts/                     — cohort records
  learners/                    — learner records (
                                  anonymised in
                                  external-evaluator
                                  bundle, full in
                                  funder-audit
                                  bundle per the
                                  funder's data-
                                  access agreement)
  module-completions/          — completion records
  programme-outcomes/          — aggregate outcome
                                  reports
  online-safety-incidents/     — incident records (
                                  gated to
                                  safeguarding-lead
                                  and child-protection-
                                  agency consumers)
  e-government-topics/         — topic records (where
                                  applicable)
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is
signed by the programme operator's HTTP-message-
signature key (RFC 9421) and counter-signed by the
programme's safeguarding lead when the package
supports a child-protection-agency submission.

## §10 Manifest and Signatures

Verification tools recompute file digests, compare to
the manifest, and reject the package on mismatch with
type `urn:wia:digital-citizenship:evidence-mismatch`.
Learner-identity-redaction integrity is checked
separately so that public-shareable bundles do not
carry per-learner content the operator has not
authorised for disclosure.

## §11 well-known URI Discovery

A conformant programme exposes a discovery document
at `/.well-known/wia-digital-citizenship` that links
to the API root, the operator's safeguarding-lead
contact, the funder binding, the operating
jurisdiction's online-safety-authority binding (where
applicable), the per-jurisdiction privacy-notice URI,
and the contracted accessibility auditor.

## §12 Long-Term Archive Integration

Programmes designate a long-term archive that holds
aggregate outcome reports, funder-reporting history,
and curriculum-module artefacts beyond the programme's
primary retention horizon. Quarterly deposits round-
trip content-addresses; on programme wind-down,
remaining records transfer to the archive with
content-addresses preserved subject to the operating
jurisdiction's records-retention rules.

## §13 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose attestations (WCAG 2.2
Level AA conformance, ISO/IEC 19796-1 quality-
management adoption, funder-recognised completion of
a digital-citizenship cohort) to consumers of W3C
Verifiable Credentials MAY re-issue the attestations
as Verifiable Credentials under the Data Model 2.0
specification. Re-issuance is optional; the canonical
record remains the JSON evidence-package manifest.

## §14 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds
with `Last-Event-ID` resume support. Subscribers that
disconnect during cohort delivery windows resume from
the last seen event identifier without losing
visibility of priority-1 events (online-safety-incident
escalation, accessibility-finding remediation
deadline, parental-consent withdrawal received).

## §15 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible
with prior-minor clients. Major revisions go through
a deprecation window of at least one full funder-
reporting cycle so that funder, civil-society-partner,
and external-evaluator integrations have time to
migrate.

## §16 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (WIA-
a11y-wiabooks for accessible-content authoring
discipline, WIA-financial-inclusion for financial-
literacy module overlap, WIA-gdpr-compliance for the
privacy overlay across minor-learner records, WIA-
ai-content-moderation for online-safety-incident
classification overlap) emit cross-standard linkage
records.

## §17 Reader Tooling

Programmes MAY publish supplementary reader tools
(per-cohort enrolment dashboards, per-module WCAG
audit consoles, per-cohort inclusive-access provision
trackers, per-period funder-report drafting wizards)
alongside the canonical evidence package; the tools
are non-normative.

## §18 Public Catalogue Feed

Programmes publish a public catalogue feed listing the
in-force curriculum-module list, the WCAG 2.2
conformance summary, the operating jurisdiction's
funder binding, the safeguarding-lead contact, and
the aggregate enrolled and completed counts. The feed
enables peer-programme and civil-society discovery of
the programme's posture.

## §19 UNESCO Digital Literacy Framework Integration

Programmes that adopt the UNESCO Digital Literacy
Global Framework as a community-recognised baseline
publish a per-cycle alignment statement and consume
UNESCO publications as inputs to the framework
adoption. Integration carries the UNESCO Institute
for Statistics (UIS) coordinator reference where the
programme submits to the UIS digital-literacy
indicator collection.

## §20 Public Catalogue Aggregator Integration

Civil-society researchers and academic-research
consortia consume aggregate enrolment and completion
statistics for independent analysis. Integration
carries the consumer's identifier, the per-research-
purpose data-access agreement, and the programme's
publication of consumer-attribution in any derivative
research output. Per-learner records are NOT shared
through this channel.

## §21 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with the operating jurisdiction's
education ministry or funder, at least one civil-
society partner organisation (where applicable), at
least one accessibility auditor, the operating
jurisdiction's child-protection-agency referral
channel, the operating jurisdiction's online-safety-
authority binding (where applicable), at least one
external evaluator, and at least one long-term
archive, and has published at least one externally
citable evidence package.

Sunsetting an integration is announced via the well-
known discovery document at least 90 calendar days
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-digital-citizenship
- **Last Updated:** 2026-04-28
