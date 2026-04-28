# WIA-ai-assistant PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-ai-assistant
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-ai-assistant. The standard covers persistent record
shapes for AI assistant deployments — model registration,
version pinning, system-prompt and tool-schema management,
conversation transcripts (with PII redaction), tool-call
records, evaluation runs, safety-incident reporting, and
deprecation. The format is consumed by AI assistant
operators, model providers, evaluation services,
red-teaming providers, and the regulators that supervise
AI deployments under the operating jurisdiction's AI
governance regime.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 22989:2022 (Information technology — Artificial
  intelligence — Concepts and terminology)
- ISO/IEC 23053:2022 (Framework for AI systems using
  machine learning)
- ISO/IEC 24029-1:2021 (Assessment of the robustness of
  neural networks — Overview)
- ISO/IEC TR 24028:2020 (Overview of trustworthiness in AI)
- ISO/IEC 42001:2023 (AI management system)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 5646 / BCP 47 (language tags)
- OpenAPI Specification 3.1 (canonical tool-schema
  envelope)
- JSON Schema 2020-12 (per-tool argument schema)
- NIST AI Risk Management Framework (AI RMF 1.0) and the
  NIST AI 600-1 GenAI Profile
- ETSI ISG SAI (Securing Artificial Intelligence) Group
  Reports series
- HuggingFace Model Card framework (community-managed
  reference for model documentation)
- MLflow Model Registry conventions (community-managed
  reference for model lifecycle records)
- ONNX (Open Neural Network Exchange) for model
  artefact portability where applicable

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts an
AI-assistant operator manages. Implementations covered
include:

- General-purpose chat assistants (operator-hosted
  consumer-facing assistants).
- Domain-specialised assistants (legal-research, medical-
  triage decision-support, customer-service).
- Multi-modal assistants (text + image + audio).
- Voice-first assistants (smart-speaker, in-vehicle).
- Assistants embedded in productivity software
  (writing, coding, data-analysis copilots).
- Multi-agent orchestration platforms (where multiple
  assistants coordinate per the operator's orchestration
  policy).

Foundation-model training pipelines and model-weight
artefact catalogues are governed by adjacent WIA
standards and are out of scope here; this PHASE addresses
the records of operating an AI assistant once a model
has been registered.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
programmeOperator    : string (institutional identifier of
                         the assistant operator)
programmeRegistered  : string (ISO 8601 / RFC 3339)
assistantClass       : enum ("general-chat-consumer" |
                         "general-chat-enterprise" |
                         "domain-legal-research" |
                         "domain-medical-decision-support" |
                         "domain-customer-service" |
                         "voice-first-smart-speaker" |
                         "voice-first-in-vehicle" |
                         "embedded-coding-copilot" |
                         "embedded-writing-copilot" |
                         "embedded-data-analysis-copilot" |
                         "multi-agent-orchestration")
governingAiFrameworkRefs : array of enum ("ISO-IEC-42001"
                         | "NIST-AI-RMF-1.0" |
                         "NIST-AI-600-1-GenAI-Profile" |
                         "EU-AI-Act-2024" |
                         "ETSI-ISG-SAI-Reports" |
                         "user-defined")
jurisdictionScope    : array of string (ISO 3166-1 / 3166-2)
programmeStatus      : enum ("design" | "operating" |
                         "limited-rollout" | "frozen" |
                         "deprecated" | "archived")
```

## §3 Model Record

```
model:
  modelId            : string (uuidv7)
  programmeId        : string (uuidv7)
  modelProviderRef   : string (institutional identifier of
                         the foundation-model provider)
  modelFamilyName    : string (e.g. "claude-sonnet" or
                         operator-internal model family
                         designation)
  modelVersion       : string (provider-published version)
  modelCardRef       : string (URI of the HuggingFace-style
                         Model Card describing intended use,
                         training data summary, evaluation
                         results, known limitations)
  parameterCountRange : enum ("under-1B" | "1B-10B" |
                         "10B-100B" | "100B-1T" | "over-1T"
                         | "undisclosed")
  modalityProfile    : array of enum ("text-input" |
                         "text-output" | "image-input" |
                         "image-output" | "audio-input" |
                         "audio-output" | "video-input")
  contextWindowTokens : integer
  registeredAt       : string (ISO 8601)
  deprecationDate    : string (ISO 8601; absent until
                         deprecation announced)
```

## §4 System Prompt Record

```
systemPrompt:
  promptId           : string (uuidv7)
  programmeId        : string (uuidv7)
  promptText         : string (UTF-8; system-instruction
                         text the assistant operates under)
  promptDigest       : string (SHA-256 of the prompt text;
                         supports change-tracking and
                         per-conversation pinning)
  promptLanguage     : string (BCP 47 tag for the primary
                         language of the prompt)
  approvedBy         : string (operator-internal approver
                         token; the operator's prompt-
                         governance committee approves
                         changes)
  effectiveFrom      : string (ISO 8601)
  effectiveUntil     : string (ISO 8601; absent until
                         superseded)
  supersededBy       : string (URI of the successor prompt;
                         absent for current)
```

## §5 Tool Schema Record

Tools are external functions the assistant may call
during conversation. Tool schemas follow OpenAPI 3.1 +
JSON Schema 2020-12 conventions.

```
toolSchema:
  toolId             : string (uuidv7)
  programmeId        : string (uuidv7)
  toolName           : string (operator-internal tool
                         identifier; e.g. "search_web",
                         "fetch_user_record")
  description        : string (UTF-8; what the tool does)
  inputSchemaRef     : string (URI of the JSON Schema
                         2020-12 input schema)
  outputSchemaRef    : string (URI of the JSON Schema
                         2020-12 output schema; absent for
                         tools whose output is unstructured)
  authorisationScope : enum ("public-read-only" |
                         "user-context-read-only" |
                         "user-context-mutating" |
                         "operator-internal-system-action" |
                         "external-api-call")
  rateLimitPerSession : integer
  registeredAt       : string (ISO 8601)
```

## §6 Conversation Transcript Record

Transcripts capture user-assistant exchanges. PII redaction
follows the operator's redaction policy (PHASE-3 §6); the
DATA-FORMAT layer carries only opaque user tokens.

```
conversation:
  conversationId     : string (uuidv7)
  programmeId        : string (uuidv7)
  modelRef           : string (model UUID — pinned model
                         version for the conversation)
  systemPromptRef    : string (system prompt UUID — pinned
                         prompt version)
  userTokenRef       : string (opaque user token; clinical
                         identity in operator IDP)
  startedAt          : string (ISO 8601)
  endedAt            : string (ISO 8601; absent for active
                         conversations)
  messageCount       : integer
  transcriptArtefactRef : string (content-addressed URI of
                         the redacted transcript)
  unredactedRetentionExpiry : string (ISO 8601; the time
                         after which the unredacted
                         transcript is purged per the
                         operator's retention policy)
```

## §7 Tool Call Record

```
toolCall:
  callId             : string (uuidv7)
  conversationRef    : string (conversation UUID)
  toolRef            : string (tool UUID)
  invokedAt          : string (ISO 8601)
  inputArgsRef       : string (URI of the input arguments
                         payload; redacted of user PII)
  outputArtefactRef  : string (URI of the output payload)
  outcome            : enum ("succeeded" | "failed" |
                         "rate-limited" | "rejected-by-
                         policy" | "timed-out")
  latencyMs          : integer
```

## §8 Evaluation Run Record

```
evaluationRun:
  runId              : string (uuidv7)
  programmeId        : string (uuidv7)
  modelRef           : string (model UUID under evaluation)
  systemPromptRef    : string (system prompt UUID)
  evaluationKind     : enum ("capability-benchmark" |
                         "robustness-iso-iec-24029-1" |
                         "safety-red-team" |
                         "alignment-human-preference" |
                         "regulatory-conformance" |
                         "user-defined")
  benchmarkSuiteRef  : string (URI of the benchmark suite
                         the run uses; e.g. operator-
                         registered MMLU run, operator
                         red-team scenario set)
  startedAt          : string (ISO 8601)
  endedAt            : string (ISO 8601)
  resultsArtefactRef : string (URI of the results report)
  passVerdict        : enum ("pass" | "fail" | "partial" |
                         "inconclusive")
```

## §9 Safety Incident Record

```
safetyIncident:
  incidentId         : string (uuidv7)
  programmeId        : string (uuidv7)
  detectedAt         : string (ISO 8601)
  classification     : enum ("harmful-content-generated" |
                         "pii-leak" | "tool-misuse" |
                         "jailbreak-success" |
                         "model-hallucination-impactful" |
                         "user-self-harm-disclosure" |
                         "fraud-attempt-against-user" |
                         "regulatory-non-compliance" |
                         "user-defined")
  severity           : enum ("informational" | "minor" |
                         "major" | "critical")
  conversationRef    : string (conversation UUID where
                         the incident manifested; absent
                         for incidents outside conversation
                         context)
  rootCauseRef       : string (URI of the investigation
                         report)
  remediationRef     : string (URI of the remediation
                         action plan)
  regulatorNotificationRef : string (URI of the regulator
                         notification artefact when one is
                         filed)
```

## §10 Memory and Context Persistence Record

Assistants that persist user-context memory across sessions
(per the operator's memory framework) carry per-user memory
records. Memory persistence is a privacy-sensitive feature
governed by the operator's per-jurisdiction data-protection
policy.

```
memoryRecord:
  memoryId           : string (uuidv7)
  programmeId        : string (uuidv7)
  userTokenRef       : string (opaque user token)
  capturedAt         : string (ISO 8601)
  memoryClass        : enum ("user-preference" |
                         "user-context-fact" |
                         "user-conversation-summary" |
                         "user-tool-history" |
                         "user-defined")
  memoryArtefactRef  : string (URI of the redacted memory
                         payload — operator's redaction
                         policy applied)
  visibilityScope    : enum ("user-private-no-other-tools"
                         | "shared-across-assistant-tools" |
                         "shared-across-operator-products")
  retentionExpiry    : string (ISO 8601; per the operator's
                         memory-retention policy)
  userOptOutHonoured : boolean (true when the user has
                         exercised their right to opt out
                         of memory persistence; subsequent
                         retrieval is suppressed)
```

## §11 User Feedback Record

```
userFeedback:
  feedbackId         : string (uuidv7)
  conversationRef    : string (conversation UUID)
  capturedAt         : string (ISO 8601)
  feedbackKind       : enum ("thumbs-up" | "thumbs-down" |
                         "report-harmful" | "report-pii-leak"
                         | "report-incorrect" |
                         "structured-rating" |
                         "free-text-comment")
  ratingValue        : number (where the feedback is a
                         structured rating; absent for
                         binary or report kinds)
  freeTextRef        : string (URI of the redacted free-text
                         comment; absent for non-text
                         feedback)
```

## §12 Source Citation Record

Assistants that operate under RAG configuration or that
cite authoritative sources for factual claims emit per-
response source-citation records.

```
sourceCitation:
  citationId         : string (uuidv7)
  conversationRef    : string (conversation UUID)
  responseTurnIndex  : integer (which assistant-turn the
                         citation belongs to)
  sourceArtefactRef  : string (URI of the cited source —
                         operator's RAG corpus identifier,
                         per-document accession reference)
  sourceTrustClass   : enum ("operator-authoritative" |
                         "operator-curated-public" |
                         "user-uploaded-document" |
                         "live-web-fetch" |
                         "model-internal-no-source")
  passageDigest      : string (SHA-256 of the cited passage
                         that grounded the assistant's
                         response; supports later
                         citation-integrity verification)
```

## §13 Conformance

Implementations claiming PHASE-1 conformance emit each of
the records defined above for every operating programme
and honour the system-prompt content-addressing rule per
§4.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-ai-assistant
- **Last Updated:** 2026-04-28
