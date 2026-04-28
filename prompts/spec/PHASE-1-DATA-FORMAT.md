# WIA-prompts PHASE 1 — Data Format Specification

**Standard:** WIA-prompts
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
WIA-prompts, the cross-vendor LLM prompt and
prompt-evaluation interoperability standard. The
records bind every prompt artefact, model card,
evaluation suite, evaluation result, safety policy,
and prompt-injection mitigation to a documented
identifier scheme, a model card schema, and a
provenance trail so that operators, auditors, and
researchers can reproduce LLM-mediated workflows
and audit their behaviour over time.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 23053:2022 (Framework for AI systems using ML)
- ISO/IEC 23894:2023 (AI risk management)
- ISO/IEC 42001:2023 (AI management system)
- NIST AI Risk Management Framework 1.0 (NIST AI 100-1)
- EU AI Act (Regulation (EU) 2024/1689)
- IETF RFC 4122 (UUID), RFC 8259 (JSON), RFC 8785 (JCS)
- IETF RFC 7515 (JWS), RFC 7517 (JWK), RFC 9421 (HTTP Message Signatures)
- W3C SHACL, W3C JSON-LD 1.1, W3C VC 2.0
- OASIS LegalRuleML 1.0 (informative for compliance prompts)
- BCP 47 / RFC 5646, ISO 639-3, Unicode 15.1
- HuggingFace Model Card schema (informative)
- OpenAI / Anthropic / Google function-call schemas (informative)
- OWASP LLM Top 10 (informative)
- MITRE ATLAS adversarial ML knowledge base (informative)

---

## §1 Scope

This PHASE applies to records that describe LLM
prompts and the evaluation, governance, and
mitigation infrastructure around them. The
records support published prompts, prompt
templates with parameter slots, conversation-
state references, structured output contracts,
multimodal attachments, evaluation suites and
results, safety policies, prompt-injection
mitigation declarations, and per-deployment risk
classifications.

In scope: prompt record, template record, parameter
record, output-contract record, conversation-state
record, model-card record, evaluation-suite record,
evaluation-result record, safety-policy record,
mitigation record, and the cross-references binding
each record to its publisher identity and signed
provenance.

Out of scope: model training pipelines (covered by
WIA-training-pipeline if and when published);
sovereign restrictions on the LLM itself
(governed by export-control and AI-safety
sovereign regimes).

## §2 Prompt record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `promptRef`          | UUID (RFC 4122) opaque                          |
| `name`               | localised prompt name (BCP 47 keys)             |
| `summary`            | one-paragraph description                       |
| `kind`               | `system`, `user`, `assistant`, `tool-call`,     |
|                      | `tool-result`, `developer`                       |
| `language`           | BCP 47 tag                                      |
| `body`               | UTF-8 text or JSON-LD multi-modal envelope      |
| `version`            | Semantic Versioning 2.0.0                       |
| `publisherRef`       | publisher identity URI                          |
| `signingKeyRef`      | JWKS URL                                        |

`promptRef` is the only invariant identifier
across edits; the publisher's slug is mutable.

## §3 Template record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `templateRef`        | URI                                             |
| `promptRef`          | this PHASE §2                                   |
| `parameters[]`       | parameter records (this PHASE §4)               |
| `renderEngine`       | `mustache`, `handlebars`, `jinja2`,             |
|                      | `f-string`, `wia-prompt-dsl-1.0`                 |
| `safeRendering`      | boolean — whether the renderer escapes user     |
|                      | input by default                                 |
| `outputContractRef`  | optional this PHASE §5                          |

Templates with `safeRendering: false` are flagged
high-risk and require an associated mitigation
record (this PHASE §10) before they can be
published.

## §4 Parameter record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `parameterRef`       | UUID                                            |
| `name`               | identifier within the template                  |
| `type`               | JSON-Schema type (`string`, `number`,           |
|                      | `boolean`, `array`, `object`)                   |
| `shape`              | optional SHACL shape constraining the value     |
| `sanitiser`          | optional sanitiser identifier (e.g.             |
|                      | `html-sanitise`, `sql-escape`,                   |
|                      | `prompt-injection-strip`)                        |
| `required`           | boolean                                         |
| `default`            | optional default value                          |

Parameters that accept user input MUST declare a
sanitiser unless the template's `safeRendering`
default applies.

## §5 Output-contract record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `contractRef`        | URI                                             |
| `kind`               | `text`, `json-schema`, `tool-call`, `audio`,    |
|                      | `image`, `multimodal`                           |
| `schema`             | JSON-Schema or function-call schema             |
| `validators[]`       | validator references (e.g. JSON-Schema 2020-12, |
|                      | OpenAPI schema, custom regex)                    |
| `repairPolicy`       | `none`, `retry-once`, `retry-with-correction`,  |
|                      | `fail-fast`                                      |

Output-contract records bind the model's expected
response shape so that downstream consumers can
validate the response without running the prompt.

## §6 Conversation-state record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `conversationRef`    | UUID                                            |
| `promptRefs[]`       | the ordered prompt sequence                     |
| `actor`              | `user`, `assistant`, `tool`, `system`           |
| `traceParent`        | W3C Trace Context for correlation               |
| `tokens.input`       | observed input token count                      |
| `tokens.output`      | observed output token count                     |
| `costBreakdown`      | optional per-prompt cost summary                 |

The conversation-state record is the audit
artefact for end-to-end LLM workflows.

## §7 Model-card record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `modelRef`           | URI                                             |
| `vendor`             | model vendor identity                           |
| `versionTag`         | the vendor's published version tag              |
| `parameterCount`     | optional, where disclosed                        |
| `trainingScope`      | declared training-data scope                    |
| `intendedUses[]`     | declared intended uses                          |
| `restrictedUses[]`   | declared restricted uses                        |
| `safetyEvaluation`   | URI to safety-evaluation report                  |
| `fairnessAudit`      | URI to fairness audit report                    |
| `riskClassification` | EU AI Act / NIST AI RMF classification          |

## §8 Evaluation-suite record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `suiteRef`           | URI                                             |
| `tasks[]`            | per-task descriptors (BIG-bench, HELM, MMLU,    |
|                      | TruthfulQA, sovereign-equivalent benchmarks)    |
| `metrics[]`          | per-task metrics (accuracy, F1, BLEU, BERTScore,|
|                      | exact-match, semantic similarity)                |
| `dataset[]`          | dataset references with provenance              |
| `samplingPolicy`     | how prompts and inputs are drawn                 |

## §9 Evaluation-result record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `resultRef`          | UUID                                            |
| `suiteRef`           | this PHASE §8                                   |
| `modelRef`           | this PHASE §7                                   |
| `runAt`              | ISO 8601                                        |
| `metrics`            | per-metric numeric values                       |
| `confidenceInterval` | optional CI bounds                              |
| `sampleSize`         | number of evaluation samples                    |
| `reproducer`         | URI to reproduction harness                      |

Results are signed by the evaluator over the
canonical JSON form (RFC 8785).

## §10 Safety-policy / mitigation record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `policyRef`          | URI                                             |
| `kind`               | `prompt-injection-mitigation`,                   |
|                      | `output-redaction`, `human-in-the-loop`,        |
|                      | `rate-limit`, `topic-restriction`,              |
|                      | `pii-redaction`                                  |
| `appliesTo`          | promptRef, templateRef, or modelRef             |
| `referenceFramework` | NIST AI RMF function (`govern`, `map`,          |
|                      | `measure`, `manage`)                             |
| `evidence`           | URI to evidence (test logs, audit report)        |

Safety policies are mandatory for prompts whose
template's `safeRendering: false` or whose
publication scope is high-risk under EU AI Act.

## §11 Cross-domain references (informative)

- WIA-language-bridge — multilingual prompt translations
- WIA-learning-analytics — prompt-mediated tutoring
- WIA-plugins — plugin host-prompt artefacts
- WIA-multiverse-interface — research prompts

## Annex A — Conformance disclosure

Implementations declare the prompt schema, the
canonicalisation form (RFC 8785), and the JWS key
set used to sign prompts and evaluation results.

## Annex B — Worked prompt record (informative)

```json
{
  "promptRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "name": "summarise-kpi-report",
  "kind": "system",
  "language": "en",
  "body": "Summarise the attached KPI report in 200 words...",
  "version": "1.4.0"
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with ISO/IEC 23053 / 42001 revisions
where applicable.

## Annex D — Conformance level

Conformance is "Core" (prompt + template +
parameter + output-contract + conversation-state
+ model-card) or "Full" (adds evaluation-suite,
evaluation-result, and safety-policy records).

## Annex E — Privacy

Personal data appearing in prompt parameters or
conversation-state records is processed under the
deployment's privacy regime (GDPR, K-PIPA, CCPA,
LGPD). Pseudonymisation is the default; PII fields
in `body` are redacted at logging time.

## Annex F — EU AI Act risk classification

Prompts that operate within EU AI Act high-risk
contexts (Annex III) carry a `riskClassification`
field on the model-card record. Operators publish
the conformity assessment per Article 43.

## Annex G — Multimodal prompt envelopes

Multimodal prompts carry a JSON-LD envelope:

```json
{
  "@context": "https://wiastandards.com/contexts/prompts.jsonld",
  "kind": "user",
  "parts": [
    {"kind": "text", "body": "Describe the image."},
    {"kind": "image", "url": "https://...", "mimeType": "image/png"}
  ]
}
```

Image, audio, and video parts reference URIs with
declared MIME types. The signature covers the
envelope verbatim including the part URIs.

## Annex H — Tool-call shape

Tool-call prompts and responses share a canonical
shape:

| Field            | Source / Binding                               |
|------------------|------------------------------------------------|
| `toolCallId`     | UUID per call                                  |
| `name`           | tool identifier                                |
| `arguments`      | JSON object validated against the tool schema  |
| `result`         | populated on the tool-result side              |

Tool schemas reference the WIA-plugins registry
when the tool is delivered as a plugin.

## Annex I — Logging and redaction

Conversation logs honour the deployment's
redaction policy. Default categories redacted at
log time:

- email addresses;
- phone numbers;
- credit card / IBAN strings;
- national identifiers (SSN, RRN, JNI);
- API keys and bearer tokens.

Redaction is performed before logs leave the
process boundary.

## Annex J — Function-call schema crosswalk

The output-contract schema for `tool-call` aligns
with the function-call shapes used by major model
vendors. Crosswalk between vendor-specific shapes
is published at `/v1/registry/function-call-
crosswalk` so that the same prompt can target
multiple vendors without rewriting the contract.

## Annex K — Cohort and persona records

Personas (e.g. "5th-grade physics tutor",
"medical-record summarisation") are defined as
named prompt sets bound by a cohort tag. The
cohort tag references SHACL constraints over
parameter values so that downstream evaluators
can sample from a specific persona consistently.

弘益人間 (Hongik Ingan) — Benefit All Humanity
