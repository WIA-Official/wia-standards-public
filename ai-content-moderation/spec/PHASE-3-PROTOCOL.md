# WIA-ai-content-moderation PHASE 3 — PROTOCOL Specification

**Standard:** WIA-ai-content-moderation
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
AI-content-moderation operator: EU DSA compliance
discipline, EU AI Act 2024 alignment for moderation
systems classified under Annex III, NIST AI RMF +
AI 600-1 GenAI Profile alignment, classifier model
governance, human-reviewer wellness governance, statutory
escalation discipline (CSAM, terrorism, imminent
violence), trusted-flagger handling, transparency-report
discipline, and out-of-court dispute resolution.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 22989:2022 (AI concepts and terminology)
- ISO/IEC TR 24028:2020 (trustworthiness in AI)
- ISO/IEC 42001:2023 (AI management system)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 24029-1:2021 (robustness)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 9457 (Problem Details)
- EU Digital Services Act (Regulation (EU) 2022/2065)
- EU AI Act 2024 (Regulation (EU) 2024/1689)
- US 18 U.S.C. § 2258A (CSAM mandatory reporting)
- UK Online Safety Act 2023
- KR Information and Communications Network Act +
  Telecommunications Business Act intermediary
  provisions
- NIST AI Risk Management Framework + AI 600-1 GenAI
  Profile
- ETSI TS 104 224 (trusted AI for online services)
- C2PA Content Credentials specification
- Santa Clara Principles on Transparency and
  Accountability in Content Moderation (cited as
  community-recognised baseline reference)

---

## §1 EU DSA Compliance Discipline

Operators that distribute services in EU jurisdictions
honour the EU Digital Services Act (Regulation (EU)
2022/2065). Per-Article alignment:

- Article 14 transparency in terms and conditions →
  policy version (PHASE-1 §3) publication discipline;
- Article 15 transparency reporting → transparency
  report (PHASE-1 §9) at the cadence the regulation
  requires;
- Article 16 notice-and-action mechanism → content-item
  registration on user notice;
- Article 17 statement of reasons → reviewer-decision
  rationale (PHASE-1 §6 `rationaleRef`);
- Article 20 internal complaint-handling system →
  appeal (PHASE-1 §7);
- Article 21 out-of-court dispute settlement →
  appeal `outOfCourtBodyRef` cite;
- Article 22 trusted flaggers → operator's trusted-
  flagger registration discipline (PHASE-3 §6);
- Article 33 VLOP designation → programme record
  `vlopDesignation` flag;
- Article 34/35 VLOP risk assessment and mitigation →
  for designated programmes;
- Article 42 VLOP transparency report (additional
  content beyond Article 15) for designated programmes.

## §2 EU AI Act 2024 Alignment

Content-moderation systems may classify as high-risk under
Annex III (specifically point 8 for systems used by
online platforms classified as VLOPs, where the AI Act
intersects with DSA-side obligations). Operators record
the per-classifier AI Act risk classification and the
per-classification obligations:

- conformity assessment per Article 43;
- post-market monitoring per Article 72;
- incident-reporting per Article 73 to the operating
  jurisdiction's AI Office.

## §3 NIST AI RMF + AI 600-1 GenAI Profile Alignment

The operator's AI RMF alignment covers the four functions
(Govern, Map, Measure, Manage) applied to the moderation
classifiers. The GenAI Profile addresses risks specific
to generative-AI output moderation: hallucinated content,
synthetic-content provenance, watermark integrity (per
C2PA Content Credentials).

## §4 Classifier Model Governance

Classifier models (PHASE-1 §5 `classifierVersion`) follow
the operator's model-governance discipline:

- per-model model card (intended use, training-data
  summary, evaluation results, known limitations,
  per-protected-class fairness analysis);
- per-model robustness evaluation per ISO/IEC 24029-1;
- per-model adversarial-input evaluation against the
  operator's red-team scenarios;
- per-model jurisdiction-specific calibration (a hate-
  speech classifier calibrated for one language /
  cultural context may overfit or underfit when applied
  to another without recalibration).

Classifier upgrades trigger A/B-test deployment under the
operator's gradual-rollout policy with appeal-rate
monitoring as a regression indicator.

## §5 Human-Reviewer Wellness Governance

Content moderation exposes human reviewers to potentially
traumatic content. The operator's reviewer-wellness
discipline:

- per-reviewer queue-rotation cadence so that reviewers
  do not concentrate on traumatic categories;
- per-reviewer mandatory wellness check-ins at the
  operator's published cadence;
- per-reviewer access to clinical mental-health support;
- per-reviewer wellness-flag (PHASE-1 §6) tracking that
  triggers temporary rotation or extended leave;
- per-cohort reviewer turnover monitoring as an
  indicator of programme-level wellness pressure.

## §6 Trusted-Flagger Handling

Per DSA Article 22, trusted flaggers (civil-society
organisations awarded trusted-flagger status by EU
Digital Services Coordinators) submit notices that
the operator processes with priority. The operator's
trusted-flagger discipline:

- per-flagger registration with the awarding DSC;
- per-flagger queue-priority binding;
- per-flagger feedback loop on flag-decision outcomes
  so that flaggers can refine their submission quality.

## §7 CSAM Statutory Discipline

CSAM detection and reporting follow the operating
jurisdiction's statutory regime:

- US: 18 U.S.C. § 2258A mandates electronic
  communications service providers to report apparent
  CSAM to the NCMEC CyberTipline. The operator's
  pipeline routes detections to NCMEC without
  operator-discretionary delay;
- EU: Regulation (EU) 2021/1232 (interim derogation
  from ePrivacy for CSAM detection) and the in-process
  EU CSAM Regulation;
- KR: KCSC and Korean National Police Agency reporting
  channels per the operating jurisdiction's law;
- UK: NCA-CEOP reporting channel.

CSAM records receive the operator's strictest access
controls; only the statutory escalation team handles
the records, and the records are preserved per the
authority's required retention.

## §8 Other Statutory Escalations

- Imminent violence threats: routed per the operating
  jurisdiction's imminent-violence reporting
  expectation (typically local law enforcement);
- Credible self-harm threats: routed per the operating
  jurisdiction's mental-health-crisis reporting
  expectation;
- Terrorism content: routed per the EU Regulation
  (EU) 2021/784 on addressing the dissemination of
  terrorist content online for EU jurisdictions, or
  equivalent national rules;
- Court-order takedowns: processed per the order's
  legal authority and the operator's legal-review
  discipline.

## §9 Transparency Report Discipline

Transparency reports (PHASE-1 §9) follow:

- per-jurisdiction transparency-report obligations
  (DSA Article 15 EU; Santa Clara Principles for
  voluntary disclosures elsewhere);
- per-VLOP additional content per DSA Article 42;
- per-period publication cadence (typically biannual
  for DSA Article 15, voluntary cadence for non-EU
  jurisdictions);
- per-report publication in the operator's supported
  languages.

## §10 Out-of-Court Dispute Resolution

Per DSA Article 21, users may escalate appeals to
out-of-court dispute settlement bodies certified by
EU DSCs. The operator records the escalation in the
appeal record (PHASE-1 §7 `outOfCourtBodyRef`) and
participates in the body's procedure under the body's
certified rules.

## §11 Records Retention

Programme records — every policy version / content item /
classifier output / reviewer decision / appeal /
statutory escalation / transparency report / API audit
log — retain per the operating jurisdiction's records-
retention rules. CSAM records retain per the statutory
authority's required retention. DSA-required records
retain per Article 24 obligations.

## §12 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) so that
moderation event timestamps and audit logs are
consistent across the operator's runtime fleet.

## §13 Per-Jurisdiction Cultural-Context Discipline

Moderation classifiers calibrated for one jurisdiction
may misclassify content in another due to language,
cultural-context, and political-context differences.
The operator's per-jurisdiction discipline:

- per-language classifier calibration with native-
  speaker review;
- per-culture political-speech discipline (the line
  between protected speech and prohibited speech
  varies dramatically across jurisdictions);
- per-jurisdiction adaptation of borderline categories
  (e.g. cannabis-related content legal in some
  jurisdictions, prohibited in others; alcohol
  marketing similarly varies);
- per-jurisdiction reviewer cohort with cultural
  competence (a Korean-language reviewer for Korean-
  language content is not interchangeable with an
  English-language reviewer who happens to speak
  Korean as a second language).

## §14 Crisis-Period Operating Discipline

During crisis periods (election periods, mass-violence
events, public-health emergencies, natural disasters),
the operator's crisis-operating discipline:

- per-crisis surge-capacity activation (additional
  reviewer hours, expedited classifier-update cadence);
- per-crisis policy-clarification publication so that
  users understand how the operator interprets policy
  during the crisis;
- per-crisis escalation-channel availability (24-hour
  contact for regulators, trusted flaggers, and
  affected communities);
- per-crisis post-event review with public reporting
  on the operator's transparency cycle.

## §15 Generative-AI Output Moderation Discipline

For programmes that moderate generative-AI outputs
(PHASE-1 §2 `platformClass=generative-ai-output`),
additional discipline applies:

- per-output watermark / C2PA Content Credentials
  manifest verification at the moderation gate;
- per-output synthetic-content disclosure check (the
  output declares its synthetic provenance per the
  operator's disclosure policy);
- per-output factual-claim review for outputs in
  regulated domains (medical, legal, financial advice);
- per-output personality-impersonation check (the
  output does not impersonate a specific real person
  without consent and disclosure).

Generative-AI moderation feeds the operator's NIST AI
600-1 GenAI Profile alignment (PHASE-3 §3).

## §16 Quality Dossier and Conformance

The operator's quality dossier records the governing
frameworks, the policy-governance committee composition,
the classifier-governance discipline, the reviewer-
wellness programme, the trusted-flagger register, the
statutory-authority bindings, the transparency-report
schedule, and the operator's incident history. The
dossier is reviewed at least annually by the operator's
trust-and-safety quality manager.

A programme conformant with WIA-ai-content-moderation
publishes its policy-version register, its transparency
reports, the statutory-escalation summary (aggregate
counts), the appeal volume and reversal-rate, and its
classifier-governance attestation, and answers an annual
self-assessment that maps each clause of this PHASE to
the operator's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-ai-content-moderation
- **Last Updated:** 2026-04-28
