# WIA-generative-ai PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-generative-ai
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer
for WIA-generative-ai. The standard covers persistent
record shapes for the lifecycle of a generative-AI
system — the foundation model and its training data
provenance; the system that wraps the model into a
deployed service; the fine-tuning, alignment, and
evaluation artefacts that adapt the model to its
deployment; the input-prompt, retrieval-augmentation,
and output-generation log; the content-provenance and
synthetic-content-marking record; the evaluation,
red-team, and post-deployment monitoring record; and
the incident, complaint, and corrective-action record.
Records are consumed by the system's deployer
(operator), the model provider (where distinct from
the deployer), the deployer's compliance and risk
function, the regulatory authority for the operating
jurisdiction (the EU AI Office for general-purpose AI
models with systemic risk, the Member-State market-
surveillance authority for high-risk AI systems, the
US sector regulators for sector-specific deployments,
and KR PIPC / NIA / FSC for KR-jurisdiction
deployments), the affected-rights holder under the EU
Copyright Directive opt-out and the equivalent
jurisdictional regimes, and the end-user through the
content-provenance and right-to-explanation channels.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 42001:2023 (AI management system)
- ISO/IEC 22989:2022 (AI concepts and terminology)
- ISO/IEC 23053:2022 (framework for AI systems using
  ML)
- ISO/IEC 24029-2:2023 (AI robustness via formal
  methods)
- ISO/IEC 23894:2023 (AI risk management)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- C2PA (Coalition for Content Provenance and
  Authenticity) Content Credentials specification
  v1.4 — the operating wire format for content
  provenance and synthetic-content marking
- W3C Verifiable Credentials Data Model 2.0
- HuggingFace Model Card (the de facto model-
  documentation format) and Datasheet for Datasets
  conventions
- MLflow (the de facto model-tracking and registry
  toolkit; cited as the format reference for the
  registry record shape)
- ONNX (the de facto open neural-network exchange
  format; cited as the model-artefact wire format)
- NIST AI Risk Management Framework 1.0 + NIST AI
  600-1 GenAI Profile (Generative AI Profile)
- US OMB M-24-10 (the US federal-agency guidance on
  AI use cases) for federal deployments
- US Executive Order 14110 (Safe, Secure, and
  Trustworthy Development and Use of Artificial
  Intelligence) and successor executive guidance
- EU AI Act (Regulation (EU) 2024/1689) Articles 3
  (definitions including "general-purpose AI
  model"), 6 (high-risk classification rules), 7
  (Annex III amendment), 8 to 17 (high-risk system
  obligations), 25 (provider obligations), 26
  (deployer obligations), 27 (fundamental-rights
  impact assessment), 50 (transparency obligations
  for AI systems intended to interact with natural
  persons / generate content), 51 to 55 (general-
  purpose AI model obligations), 56 (codes of
  practice for GPAI), Annex III (high-risk areas),
  Annex IX, Annex XI (technical documentation for
  GPAI)
- EU Code of Practice for General-Purpose AI Models
  (the AI Act Article 56 published code)
- EU GDPR (Regulation (EU) 2016/679) Articles 5, 6,
  9, 12 to 22, 22(3), 24, 25, 32, 35
- EU Copyright Directive 2019/790 Article 4 text-
  and-data-mining (TDM) opt-out
- KR AI Industry Promotion Act (인공지능 산업진흥법)
  and KR PIPA Article 28 automated decision-making
  discipline
- KR Communications Commission and Personal
  Information Protection Commission (PIPC) AI-
  related guidance
- OWASP Top 10 for Large Language Model Applications

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
a generative-AI operator (the deployer or, where
distinct, the model provider) maintains:

- The model registry record — model identity,
  version, weights digest, training data manifest,
  evaluation results.
- The system / deployment record — the wrapper that
  turns the model into a deployed service (system
  prompt, tool catalogue, retrieval index, safety
  filters).
- The fine-tuning, alignment, and adapter record.
- The input-output transcript record — prompts,
  retrieved documents, tool calls, model outputs.
- The content-provenance and synthetic-content-
  marking record.
- The evaluation, red-team, and ongoing monitoring
  record.
- The incident and complaint record.
- The fundamental-rights impact assessment record
  (where Article 27 applies).

Records align with ISO/IEC 42001:2023 AI management
system requirements; the EU AI Act technical
documentation requirements (Articles 11 and 53,
Annexes IV and XI) are encoded in PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       deployer or provider)
operatorRole         : enum ("model-provider" |
                       "system-deployer" |
                       "downstream-deployer" |
                       "evaluator" | "user-defined")
operatorJurisdiction : array of string (ISO 3166-1)
governingFrameworks  : array of enum ("EU-AI-ACT-2024
                       -1689" | "EU-AI-ACT-GPAI-ART-
                       51-55" | "EU-AI-ACT-HIGH-
                       RISK-ANNEX-III" | "EU-AI-ACT
                       -GPAI-COP" | "EU-COPYRIGHT-
                       2019-790-ART-4-TDM" |
                       "EU-GDPR-ART-22" | "US-EO-
                       14110" | "US-OMB-M-24-10" |
                       "NIST-AI-RMF" | "NIST-AI-600
                       -1-GENAI-PROFILE" | "ISO-IEC
                       -42001" | "ISO-IEC-23894-AI-
                       RISK" | "C2PA-CONTENT-
                       CREDENTIALS" | "KR-AI-
                       산업진흥법" | "KR-PIPA-ART-28
                       -AUTO-DECISION" | "OWASP-LLM
                       -TOP-10" | "user-defined")
modelKindAtSurface   : enum ("text-foundation" |
                       "image-foundation" |
                       "video-foundation" |
                       "audio-foundation" |
                       "multimodal-foundation" |
                       "code-foundation" |
                       "domain-fine-tuned" |
                       "rag-system" | "agent-system"
                       | "user-defined")
gpaiClassification   : enum ("not-gpai" | "gpai-
                       not-systemic" | "gpai-
                       systemic-risk" | "gpai-foss-
                       not-systemic")
                       (per EU AI Act Article 51 the
                       systemic-risk classification
                       applies if the cumulative
                       compute used for training
                       exceeds the threshold the AI
                       Act Annex XIII designates,
                       or if the AI Office designates
                       the model as systemic-risk)
programmeStatus      : enum ("design" | "evaluating"
                       | "operating" | "monitored" |
                       "wind-down" | "archived")
```

## §3 Model Registry Record

The model registry record persists the foundation
model's identity and provenance:

```
modelRecord:
  modelId            : string (uuidv7; the operator's
                       internal model identifier)
  modelName          : string (the model-card-published
                       name)
  providerRef        : string (the model provider's
                       legal identity; absent when the
                       operator is the provider)
  modelArchitecture  : string (the architecture family
                       — transformer-decoder,
                       transformer-encoder-decoder,
                       diffusion, mixture-of-experts,
                       state-space-model, etc.)
  parameterCount     : integer (declared parameter
                       count; for GPAI the EU AI Act
                       Annex XI requires disclosure)
  weightsDigest      : string (SHA-256 of the model
                       weights archive — preserved
                       even when weights are not
                       distributed)
  trainingDataManifestRef : string (URI of the
                       training data manifest — the
                       Datasheet-for-Datasets-aligned
                       record listing data sources,
                       collection methods, copyright-
                       opt-out filtering, content-
                       moderation filtering, and
                       personal-data exclusion
                       discipline)
  trainingComputeRef : object (the cumulative
                       training compute in FLOP, the
                       hardware estimator method, and
                       the period over which compute
                       was accumulated; required for
                       EU AI Act Annex XI disclosure
                       and for GPAI systemic-risk
                       threshold evaluation)
  modelCardRef       : string (URI of the published
                       model card — HuggingFace
                       format or equivalent)
  releaseLicense     : string (the licence under
                       which the model is released —
                       for FOSS GPAI the AI Act
                       Article 53(2) exemption from
                       certain GPAI obligations
                       applies)
  releasedAt         : string (ISO 8601)
```

## §4 System / Deployment Record

```
systemRecord:
  systemId           : string (uuidv7)
  programmeRef       : string
  modelRef           : string (PHASE-1 §3)
  systemPrompt       : string (the system-prompt /
                       instruction template that
                       conditions the model's
                       behaviour at deployment)
  toolCatalogue      : array of object (the tools the
                       system can invoke — function-
                       calling specifications,
                       authorised endpoints, tool-use
                       safety constraints)
  retrievalIndexRef  : string (URI of the retrieval
                       index — the corpus and the
                       embedding-model used to embed
                       it; absent for non-RAG
                       systems)
  safetyFiltersRef   : array of string (the input-
                       output safety filters
                       configured — the prompt-
                       injection filter, the unsafe-
                       content filter, the PII
                       filter, the secret-leakage
                       filter)
  intendedPurpose    : string (the intended purpose
                       declared per EU AI Act Article
                       3(12); high-risk systems'
                       intended purpose drives the
                       Article 6 classification)
  highRiskClassification : enum ("not-high-risk" |
                       "annex-iii-biometric" |
                       "annex-iii-critical-
                       infrastructure" | "annex-iii-
                       education" | "annex-iii-
                       employment" | "annex-iii-
                       essential-services" | "annex-
                       iii-law-enforcement" |
                       "annex-iii-migration" |
                       "annex-iii-justice" |
                       "annex-iii-democratic-
                       process" | "user-defined")
  deployedAt         : string (ISO 8601)
  retiredAt          : string (ISO 8601; absent until
                       retired)
```

## §5 Fine-Tune / Alignment Record

```
fineTuneRecord:
  fineTuneId         : string (uuidv7)
  baseModelRef       : string (the model the fine-
                       tune is derived from)
  fineTuneKind       : enum ("supervised-fine-tune"
                       | "rlhf-reward-model" |
                       "rlhf-policy" | "dpo" |
                       "lora-adapter" | "instruction-
                       tune" | "constitutional" |
                       "user-defined")
  trainingDataRef    : string (URI of the fine-tune
                       data manifest)
  evaluationReportRef : string (URI of the evaluation
                       report covering capability,
                       fairness, robustness, and
                       safety metrics)
  approvedAt         : string (ISO 8601)
  approvingFunctionRef : string (the operator's AI
                       management committee or model-
                       risk committee identifier)
```

## §6 Input-Output Transcript Record

```
transcriptRecord:
  transcriptId       : string (uuidv7)
  systemRef          : string
  userIdentityRef    : string (the end-user's
                       identity — encrypted at rest
                       where required by GDPR Article
                       5 / KR PIPA)
  startedAt          : string (ISO 8601)
  endedAt            : string (ISO 8601)
  promptList         : array of object (the user's
                       prompts and any retrieved
                       documents fetched by the RAG
                       layer)
  toolCalls          : array of object (the tool
                       calls the model issued and the
                       tool responses received)
  outputList         : array of object (the model
                       outputs returned to the user)
  safetyDecisions    : array of object (the safety-
                       filter decisions applied —
                       block, redact, warn, allow;
                       the rule that fired)
  contentCredentialsRef : array of string (the C2PA
                       Content Credentials
                       references attached to the
                       generated outputs — required
                       for AI-generated content under
                       EU AI Act Article 50(2))
```

## §7 Content-Provenance and Synthetic-Content
       Marking Record

```
contentCredentialsRecord:
  credentialId       : string (uuidv7)
  outputRef          : string (PHASE-1 §6 transcript
                       output reference)
  c2paManifestRef    : string (URI of the C2PA Content
                       Credentials manifest — the
                       canonical wire format for
                       provenance assertions)
  syntheticContentDeclaration : enum ("ai-generated"
                       | "ai-modified" | "ai-
                       composed" | "user-defined")
  watermarkDetail    : object (the watermark
                       method applied to the output
                       — invisible cryptographic
                       watermark, perceptual mark,
                       or steganographic label)
  signedBy           : string (the operator's signing
                       key reference; the signature
                       is verifiable per the C2PA
                       trust list)
```

## §8 Evaluation and Red-Team Record

```
evaluationRecord:
  evaluationId       : string (uuidv7)
  systemRef          : string
  evaluationKind     : enum ("capability-benchmark"
                       | "fairness-evaluation" |
                       "robustness-test" |
                       "adversarial-red-team" |
                       "fundamental-rights-
                       assessment" | "post-market-
                       monitoring" | "user-defined")
  evaluatorRef       : string (the internal team or
                       external organisation that
                       performed the evaluation)
  startedAt          : string (ISO 8601)
  completedAt        : string (ISO 8601)
  metricsRef         : string (URI of the metrics
                       report)
  findingsRef        : string (URI of the findings
                       narrative — including any
                       systemic-risk indicators per
                       AI Act Article 55(1)(a))
```

## §9 Incident and Complaint Record

```
incidentRecord:
  incidentId         : string (uuidv7)
  systemRef          : string
  reportedAt         : string (ISO 8601)
  reporterKind       : enum ("end-user" | "operator-
                       internal" | "downstream-
                       deployer" | "third-party-
                       researcher" | "regulator")
  incidentKind       : enum ("safety-failure" |
                       "fundamental-rights-violation"
                       | "privacy-leak" |
                       "fabricated-output-causing-
                       harm" | "prompt-injection-
                       exploit" | "copyright-
                       infringement" | "user-
                       defined")
  severityKind       : enum ("minor" | "moderate" |
                       "serious" | "critical")
  rootCauseRef       : string (URI of the root-cause
                       narrative; absent until
                       investigated)
  correctiveActions  : array of object (the
                       operator's remediation steps
                       and their completion times)
  regulatorReportRef : string (URI of the regulator
                       report under EU AI Act
                       Article 73 serious-incident
                       reporting; absent unless
                       reported)
```

## §10 Conformance

Implementations claiming PHASE-1 conformance maintain
the records defined above for every deployed
generative-AI system, preserve the model-registry and
training-data-manifest records on the EU AI Act
Article 18 ten-year retention discipline (for high-
risk systems) and on the GPAI-equivalent disclosure
discipline (Article 53(1)(a) summary of training
content), and emit Content Credentials per EU AI Act
Article 50(2) for every AI-generated output where the
discipline applies.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-generative-ai
- **Last Updated:** 2026-04-28
