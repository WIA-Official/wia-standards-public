# WIA-ai-assistant PHASE 4 — INTEGRATION Specification

**Standard:** WIA-ai-assistant
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an AI assistant operator
integrates with the systems that surround it: foundation-
model providers; evaluation service vendors (Holistic
Evaluation of Language Models, evaluation harness
providers, MLPerf-aligned benchmark services); red-team
service providers; AI governance regulators; HuggingFace
Model Hub for model-card publication; MLflow Model Registry
for operator-internal model lifecycle; ONNX-aligned model
artefact tooling; the operator's IDP (for user-token
mediation); the operator's CRM (for clinical user
identity); and long-term archives.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO/IEC 42001:2023 (AI management system)
- ISO 8601 (date and time)
- W3C Verifiable Credentials Data Model 2.0 (optional)
- HuggingFace Model Card framework (community-managed)
- MLflow Model Registry conventions (community-managed)
- ONNX (Open Neural Network Exchange)

---

## §1 Foundation-Model Provider Integration

The operator's foundation-model provider (Anthropic,
OpenAI, Google, Mistral, Meta, Alibaba, Naver
HyperCLOVA, equivalent providers) operates the runtime
that the assistant consumes. Integration carries the
provider's identifier, the per-model-version contract,
the per-rate-limit and quota binding, and the provider's
deprecation-notification endpoint. Provider deprecations
trigger the operator's migration workflow per PHASE-3 §8.

## §2 Evaluation Service Vendor Integration

Evaluation services (HELM, EleutherAI lm-evaluation-
harness, OpenAI Evals, MLPerf Inference v4, vendor-
managed evaluation platforms, custom operator-internal
evaluators) emit signed evaluation reports per evaluation
run. Integration carries the service's identifier, the
per-benchmark suite reference, and the report intake
endpoint that the operator's evaluation pipeline consumes.

## §3 Red-Team Service Provider Integration

Red-team providers (operator-internal red team, third-party
red-team firms, crowdsourced red-team platforms)
exercise the assistant against the operator's threat-
model. Integration carries the provider's identifier,
the per-engagement scope, and the per-finding intake
endpoint that flows into the safety incident pipeline.

## §4 AI Governance Regulator Integration

AI governance regulators (EU AI Office for EU AI Act
2024 high-risk systems, US NIST AI RMF voluntary
framework counterparts, KR Personal Information
Protection Commission for AI-related PII, KR Korea
Communications Commission, equivalent national
authorities) consume the operator's governance evidence.
Integration carries the regulator's identifier, the
per-record-class submission template, and the regulator's
incident-notification intake.

## §5 HuggingFace Model Hub Integration

Operators that publish derivative models (fine-tuned,
adapter-augmented, distilled) to the HuggingFace Model
Hub integrate with HuggingFace's model-card publication
endpoint. Integration carries the HuggingFace
organisation reference, the per-model repository
binding, and the model-card revision pickup cadence.

## §6 MLflow Model Registry Integration

Operators that operate an internal MLflow Model Registry
(or equivalent operator-internal registry) integrate the
WIA `model` records with the MLflow registry's per-
version stage transitions (None / Staging / Production /
Archived). The integration is bidirectional: WIA model-
register events flow to MLflow stage transitions, and
MLflow stage transitions trigger WIA model-record
status updates.

## §7 ONNX Artefact Tooling Integration

Operators that bind model artefacts to ONNX format for
cross-runtime portability integrate ONNX validation
tooling (onnx checker, onnxruntime, onnx-mlir) into the
model-registration workflow. Integration carries the
tooling identifier and the per-model conformance check
report.

## §8 Identity Provider Integration

The operator's IDP federates user identity. The
DATA-FORMAT layer's `userTokenRef` (PHASE-1 §6) is an
opaque token the IDP issues; the IDP-side mapping to
clinical user identity (email, phone, account-number)
is held only in the IDP and the operator's CRM, not on
this API.

## §9 CRM Integration for Subject-Access Requests

The operator's CRM mediates subject-access requests
(GDPR Article 15, K-PIPA equivalent, CCPA equivalent).
The CRM translates the subject's request into authorised
WIA API calls under the operator's service-account
credentials, retrieves the subject's conversations and
tool-call history, and returns the redacted record set
to the subject through the CRM's secure channel.

## §10 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  models/                      — model registrations and
                                  cards
  system-prompts/              — prompt revisions in force
  tool-schemas/                — tool definitions and
                                  authorisation scopes
  conversations/               — conversation summaries
                                  (transcripts content-
                                  addressed and gated by
                                  the operator's
                                  disclosure policy)
  tool-calls/                  — tool-call audit history
  evaluation-runs/             — per-cadence evaluation
                                  outcomes
  safety-incidents/            — incident records and
                                  remediation references
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is
signed by the operator's HTTP-message-signature key
(RFC 9421) and counter-signed by the operator's AI
governance officer when the package supports a regulator
submission.

## §11 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:ai-assistant:evidence-mismatch`.

## §12 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-ai-assistant` that links to the API
root, the operator's governing AI-framework enrolments,
the published quality dossier, the model registry
summary (Production-stage models), and the per-
jurisdiction regulator binding.

## §13 Long-Term Archive Integration

Operators designate a long-term archive that holds
model registrations, evaluation runs, and safety-
incident records beyond the operator's primary retention
horizon. Quarterly deposits round-trip content-
addresses; on programme wind-down, remaining records
transfer to the archive with content-addresses
preserved.

## §14 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (ISO/IEC
42001 certification, EU AI Act conformity-assessment
outcome, NIST AI RMF self-attestation, evaluation
benchmark scores) to consumers of W3C Verifiable
Credentials MAY re-issue the attestations as Verifiable
Credentials under the Data Model 2.0 specification.
Re-issuance is optional; the canonical record remains
the JSON evidence-package manifest.

## §15 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds
with `Last-Event-ID` resume support; subscribers that
disconnect during long evaluation or incident windows
resume from the last seen event identifier without
losing visibility of priority-1 events.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible
with prior-minor clients. Major revisions go through a
deprecation window of at least one full ISO/IEC 42001
surveillance cycle so that regulator and consumer
integrations have time to migrate.

## §17 Cross-Standard Linkage

Operators that consume adjacent WIA standards (WIA-
generative-ai for foundation-model lifecycle, WIA-
ai-content-moderation for output-side moderation, WIA-
content-ai for content-pipeline integration, WIA-
intent-lang for intent-language declarative governance)
emit cross-standard linkage records.

## §18 Reader Tooling

Operators MAY publish supplementary reader tools (per-
programme conversation-volume dashboards, per-model
evaluation trend charts, safety-incident timeline
visualisers, prompt revision diff views) alongside the
canonical evidence package; the tools are non-normative.

## §19 Public Catalogue and Aggregator Feeds

Operators publish a public catalogue of registered
models (Production stage), governing AI-framework
enrolments, and per-quarter aggregate safety-incident
statistics through an Atom or JSON Feed. Aggregator
consumers subscribe to compare operator behaviour
across the AI-assistant industry.

## §20 Audit-Reviewer Workflow Integration

External audit reviewers (ISO/IEC 42001 surveillance
auditors, EU AI Act conformity-assessment bodies, NIST
AI RMF aligned external reviewers) consume audit-trail
exports through dedicated client certificates. The export
carries the API audit logs for the audit window, the
evaluation-run history, the safety-incident catalogue,
and the per-jurisdiction regulatory-submission history.
Audit access is scoped to the audit window the reviewer
was commissioned for.

## §21 Subject-Access Subject-Erasure Workflow Integration

The operator's CRM (PHASE-4 §9) routes data-subject
rights requests (access, rectification, erasure, opt-out
of automated decision-making) into the WIA platform.
Integration carries the CRM's identifier, the per-rights
intake reference, and the operator's response SLA per
jurisdiction (typically 30 days under GDPR, similar
windows under K-PIPA / CCPA / equivalent).

## §22 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with at least one foundation-model
provider, at least one evaluation service vendor, the
operator's IDP, the operator's CRM, the relevant AI
governance regulator (where the jurisdiction has one
the operator's class is subject to), and at least one
long-term archive, and has published at least one
externally citable evidence package.

Sunsetting an integration is announced via the well-
known discovery document at least 90 calendar days
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-ai-assistant
- **Last Updated:** 2026-04-28
