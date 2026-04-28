# WIA-ai-assistant PHASE 3 — PROTOCOL Specification

**Standard:** WIA-ai-assistant
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an AI
assistant operator: governing AI-framework alignment
(NIST AI RMF + AI 600-1, ISO/IEC 42001, EU AI Act 2024),
model-card discipline, system-prompt governance, tool-
authorisation governance, conversation transcript privacy
and retention, evaluation cadence (capability + robustness
+ safety + alignment + regulatory conformance), red-team
exercises, deprecation, and incident response.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 22989:2022 (AI concepts and terminology)
- ISO/IEC 23053:2022 (framework for AI systems using ML)
- ISO/IEC 24029-1:2021 (robustness of neural networks)
- ISO/IEC TR 24028:2020 (trustworthiness in AI)
- ISO/IEC 42001:2023 (AI management system)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- NIST AI Risk Management Framework (AI RMF 1.0)
- NIST AI 600-1 GenAI Profile
- ETSI ISG SAI Group Reports series
- EU AI Act 2024 (Regulation (EU) 2024/1689)
- KR AI Basic Act (effective from 2026 per the operating
  jurisdiction's enacted timeline; the operator records
  the in-force version)
- US Executive Order on Safe, Secure, and Trustworthy AI
  + agency implementation guidance
- HuggingFace Model Card framework (community-managed)

---

## §1 Governing AI-Framework Alignment

The operator selects one or more governing AI frameworks
per programme (PHASE-1 §2 `governingAiFrameworkRefs`).
Per-framework alignment:

- **ISO/IEC 42001:2023**: AI management system framework;
  the operator adopts the management-system clauses (5.
  Leadership, 6. Planning, 7. Support, 8. Operation, 9.
  Performance evaluation, 10. Improvement) and demonstrates
  conformance through certification by an accredited
  certification body.
- **NIST AI RMF 1.0 + AI 600-1 GenAI Profile**: voluntary
  US framework; the operator implements the four
  functions (Govern, Map, Measure, Manage) and the GenAI
  Profile's risk-mitigation actions per use-case category.
- **EU AI Act 2024**: legally binding for assistants
  deployed in EU jurisdictions; the operator binds the
  per-system risk classification (prohibited / high-risk
  / limited-risk / minimal-risk) and the per-classification
  obligations (conformity assessment, transparency
  obligations, post-market monitoring).
- **ETSI ISG SAI Reports**: technical reference for
  securing AI systems against adversarial attacks; the
  operator records SAI-aligned mitigations in the
  quality dossier.

## §2 Model-Card Discipline

Each registered model carries a Model Card that documents
intended use, training-data summary, evaluation results,
known limitations, and per-use-case risk assessment. The
operator's per-model-card review cadence:

- per-model-version review at registration;
- per-model-version review on material model-provider
  update (capability change, training-data refresh);
- per-jurisdiction adaptation when the operator binds
  the model to a new jurisdiction with different
  governance expectations.

Model cards aligned to HuggingFace Model Card v1 / v2
conventions support cross-operator portability.

## §3 System-Prompt Governance

System prompts (PHASE-1 §4) are content-addressed and
versioned through the operator's prompt-governance
committee. Per-prompt review covers:

- safety filtering (the prompt does not jailbreak,
  destabilise, or bias the assistant beyond the operator's
  declared envelope);
- legal review (the prompt does not instruct the
  assistant to provide unauthorised professional
  advice in regulated domains);
- accessibility review (where the assistant interacts
  with users via accessible interfaces);
- per-language review for multi-language deployments.

Prompt revisions trigger re-evaluation per §5; a prompt
that fails re-evaluation is not promoted to the
in-production registry.

## §4 Tool Authorisation Governance

Tools (PHASE-1 §5) carry per-tool authorisation scopes.
The operator's tool-governance committee reviews:

- per-tool least-privilege design (the tool exposes only
  the data and actions necessary for its intended use);
- per-tool input validation (JSON Schema 2020-12
  enforcement at the orchestration layer);
- per-tool output safety filtering (the tool's output
  is filtered before being passed back to the assistant
  for inclusion in the user-facing response);
- per-tool audit logging (every invocation is recorded
  per PHASE-1 §7).

Tools with `authorisationScope=user-context-mutating`
require user consent at first invocation; consent is
recorded in the operator's IDP, never in the WIA
DATA-FORMAT layer.

## §5 Evaluation Cadence

Per the NIST AI RMF Measure function and ISO/IEC 24029-1,
the operator runs evaluation runs at the following
cadences:

- capability benchmarks: at every model registration and
  every major model update;
- robustness evaluations (ISO/IEC 24029-1 aligned):
  quarterly during steady-state operation and on every
  model / prompt revision;
- safety red-team exercises: at programme launch, on
  every model upgrade, and at the operator's red-team
  cadence (typically quarterly);
- alignment human-preference evaluations: continuous
  thumbs-up / thumbs-down feedback aggregation in
  steady-state, with structured review at each model
  upgrade;
- regulatory conformance evaluations: per the operating
  jurisdiction's required cadence.

## §6 Conversation Transcript Privacy and Retention

Conversation transcripts contain PII (user input, user
context shared with the assistant) and operator-internal
detail. The operator's redaction policy:

- per-jurisdiction PII categories redacted per the
  jurisdiction's data-protection law (GDPR, K-PIPA,
  CCPA, equivalent);
- per-conversation `unredactedRetentionExpiry`
  (PHASE-1 §6) — typically 30-90 days for general-
  purpose assistants, longer for regulated-context
  assistants where audit obligations require
  retention;
- post-expiry transcript carries only the redacted
  form; the unredacted form purges per the operator's
  data-retention pipeline.

## §7 Red-Team Discipline

Red-team exercises probe the assistant against:

- jailbreak attempts (instruction-injection, prompt-
  injection through tool outputs, encoding attacks);
- harmful-content elicitation (per the operator's
  prohibited-content categories);
- PII-leak elicitation;
- tool-misuse elicitation (induce the assistant to
  invoke tools for unauthorised purposes);
- regulated-advice elicitation (medical, legal,
  financial);
- demographic-bias elicitation (per the operator's
  fairness review criteria).

Red-team findings flow into the operator's safety
incident pipeline and the operator's prompt /
authorisation governance.

## §8 Deprecation and Sunsetting

Model deprecation announcements (PHASE-1 §3
`deprecationDate`) trigger the operator's migration
workflow: existing conversations migrate to a successor
model, the deprecated model is removed from the
registry's active list, and the deprecated model
remains addressable for audit purposes. Sunset windows
honour the operator's published commitment to
consumers and downstream integrators.

## §9 Records Retention

Programme records — every model / system prompt / tool
schema / evaluation run / safety incident / API audit
log — retain per the operating jurisdiction's records-
retention rules. Conversation transcripts retain per §6
plus the longer of operator policy and jurisdiction
required retention.

## §10 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) so that
conversation event timestamps and audit logs are
consistent across the operator's runtime fleet.

## §11 Quality Dossier

The operator's quality dossier records the governing AI-
framework enrolments, the model-card register, the
prompt-governance committee composition, the tool-
authorisation matrix, the evaluation cadence, the
red-team partner of record, the per-jurisdiction
data-protection binding, and the operator's incident
history. The dossier is reviewed at least annually by
the operator's AI quality manager.

## §12 Cross-Jurisdictional Operation

Multi-jurisdiction operators honour each jurisdiction's
AI governance regime; per-conversation governing-
jurisdiction tagging supports downstream regulator-
specific reporting.

## §13 Memory Persistence Governance

Per-user memory persistence (PHASE-1 §10) is governed by:

- per-jurisdiction data-protection law (GDPR Articles
  6/7/13/14/17/22, K-PIPA, CCPA, equivalent rules);
- per-jurisdiction transparency obligations (the user
  knows what is remembered and can review / edit /
  delete);
- per-jurisdiction sensitive-category rules (race,
  health, sexual orientation, political opinion, biometric
  data, religious belief — each with stricter
  consent-and-notice obligations).

The operator's memory-governance committee reviews the
memory framework at least annually and at every material
change.

## §14 Inclusive Design Discipline

Assistants serve users across language, cultural, ability,
and accessibility dimensions. The operator's inclusive-
design discipline:

- per-jurisdiction language coverage with native-quality
  output (not machine-translated as a primary fallback);
- per-language cultural-context adaptation (date formats,
  number formats, address formats, formality registers);
- per-modality accessibility (screen-reader compatibility,
  keyboard-only navigation, voice-first alternative for
  non-text users);
- per-vulnerable-group safety review (minors, users with
  cognitive disabilities, users in mental-health crisis,
  users in domestic-violence situations) where the
  programme's user base includes the group.

## §15 Hallucination Mitigation Discipline

Generative assistants hallucinate (produce plausible but
incorrect content). The operator's hallucination-mitigation
discipline:

- per-domain confidence-thresholding (the assistant
  declines to answer when its internal confidence is
  below the operator's declared threshold for the
  domain);
- per-domain source-citation requirement (the assistant
  cites authoritative sources for factual claims in
  regulated domains);
- per-domain retrieval-augmented generation (RAG)
  configuration where the operator binds the assistant
  to an authoritative knowledge corpus;
- per-domain defer-to-human escalation (the assistant
  escalates to human review for high-stakes domains
  including medical, legal, financial advice).

Hallucination-impact incidents (user acted on incorrect
output and suffered harm) feed the safety incident pipeline
at severity proportional to the harm.

## §16 Conformance and Auditing

A programme conformant with WIA-ai-assistant publishes
its governing AI-framework enrolments, its model
register, its evaluation outcomes catalogue, its safety
incident summary at major and above, and its red-team
exercise summary, and answers an annual self-assessment
that maps each clause of this PHASE to the operator's
implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-ai-assistant
- **Last Updated:** 2026-04-28
