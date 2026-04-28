# WIA-prompts PHASE 2 — API Interface Specification

**Standard:** WIA-prompts
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-prompts
participants expose so that operators, model
vendors, evaluation labs, safety auditors, and
deployment platforms can publish prompts, run
evaluations, register safety policies, and
reconcile conversation-state records through a
single contract.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9457 (Problem Details)
- IETF RFC 7515 (JWS), RFC 7519 (JWT), RFC 9421 (HTTP Message Signatures)
- IETF RFC 9530 (Digest Fields), RFC 8259 (JSON), RFC 8785 (JCS)
- ISO/IEC 23053:2022, ISO/IEC 42001:2023
- W3C SHACL, W3C JSON-LD 1.1, W3C VC 2.0
- BCP 47 / RFC 5646

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces
between prompt publishers, prompt registries,
model vendors, evaluation labs, safety auditors,
and deployment platforms.

## §2 Operation groups

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/prompts`       | prompt registry                                  |
| `/v1/templates`     | template registry                                |
| `/v1/contracts`     | output-contract registry                         |
| `/v1/models`        | model-card registry                              |
| `/v1/evaluations`   | evaluation suites and results                    |
| `/v1/safety`        | safety policy and mitigation records             |
| `/v1/conversations` | conversation-state store                         |
| `/v1/registry`      | registry directory                               |

## §3 Authentication

Read endpoints accept anonymous or JWT bearer.
Write endpoints require a JWT bound to the
publisher identity. Safety-policy endpoints
additionally require a publisher's risk-officer
attestation when EU AI Act high-risk
classification applies.

## §4 Prompt operations

### 4.1 Publish prompt

```
POST /v1/prompts
```

Body: prompt record (PHASE-1 §2) signed by the
publisher.

### 4.2 Lookup

```
GET /v1/prompts/{promptRef}
```

Returns the canonical prompt record with
conditional GET via `ETag`.

### 4.3 Search

```
GET /v1/prompts?q=<text>&kind=<kind>&language=<bcp47>
```

Returns matching prompts; pagination mandatory
beyond 100 results.

## §5 Template operations

### 5.1 Publish template

```
POST /v1/templates
```

Body: template record (PHASE-1 §3) with parameter
references.

### 5.2 Render

```
POST /v1/templates/{templateRef}/render
```

Body: parameter values. Response: the rendered
prompt body or a Problem Details fragment listing
sanitiser violations.

## §6 Output-contract operations

### 6.1 Register

```
POST /v1/contracts
```

Body: output-contract record (PHASE-1 §5).

### 6.2 Validate

```
POST /v1/contracts/{contractRef}/validate
```

Body: a candidate response. Response: a verdict
plus any repair suggestions per the declared
repair policy.

## §7 Model operations

### 7.1 Register model card

```
POST /v1/models
```

Body: model-card record (PHASE-1 §7).

### 7.2 Lookup

```
GET /v1/models/{modelRef}
```

Returns the canonical model card.

## §8 Evaluation operations

### 8.1 Register suite

```
POST /v1/evaluations/suites
```

Body: evaluation-suite record (PHASE-1 §8).

### 8.2 Submit result

```
POST /v1/evaluations/results
```

Body: evaluation-result record (PHASE-1 §9) signed
by the evaluator.

### 8.3 Compare

```
POST /v1/evaluations/compare
```

Body: two `resultRef`. Response: a structured
comparison across metrics with significance
testing where the sample size permits.

## §9 Safety operations

### 9.1 Register policy

```
POST /v1/safety
```

Body: safety-policy record (PHASE-1 §10).

### 9.2 Apply policy check

```
POST /v1/safety/check
```

Body: prompt or template reference and the policy
to check. Response: the policy verdict with a
reasoning trace if the policy is violated.

## §10 Conversation operations

### 10.1 Append turn

```
POST /v1/conversations/{conversationRef}/turns
```

Body: conversation turn including prompt
reference, actor, and timestamp. Response: the
turn reference.

### 10.2 Lookup

```
GET /v1/conversations/{conversationRef}
```

Returns the conversation-state record.

### 10.3 Replay

```
POST /v1/conversations/{conversationRef}/replay
```

Body: target model reference. Replays the
conversation against the target model so that
auditors can compare behaviour.

## §11 Error semantics

Errors are `application/problem+json` (RFC 9457)
namespaced under
`https://wiastandards.com/errors/prompts/`.

## §12 Caching and rate limits

Prompt and template records carry strong `ETag`.
Conversation-state records are not cached. Rate-
limit headers follow the draft-ietf-httpapi-
ratelimit-headers convention.

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info: {title: WIA-prompts API, version: 1.0.0}
paths:
  /v1/prompts:
    post:
      summary: Publish a prompt
      requestBody:
        required: true
        content:
          application/json:
            schema: {$ref: 'PromptRecord.schema.json'}
      responses:
        '201': {description: Prompt published}
```

## Annex B — Idempotency

Mutating operations honour `Idempotency-Key` for
24h.

## Annex C — Webhook subscriptions

Subscribers receive events on `prompt.published`,
`template.published`, `evaluation.completed`,
`safety.policy-applied`. Delivery is signed with
HMAC-SHA-256.

## Annex D — Federation

Federation between sister registries follows the
discovery contract in PHASE-3.

## Annex E — Bulk export

`POST /v1/registry/export` returns a signed URL
to a `tar.zst` of records filtered by publisher
and date range.

## Annex F — Sandbox

`/v1/sandbox` mirrors production with synthetic
prompts and ephemeral state.

## Annex G — Quotas

Per-publisher quotas: 1,000 prompt versions / hour;
10,000 / day.

## Annex H — Audit feed

`GET /v1/registry/audit?since=<timestamp>`.

## Annex I — Public introspection

`GET /v1/registry/stats`.

## Annex J — Webhook payload shape

```json
{
  "event": "evaluation.completed",
  "resultRef": "f63f4f04-...",
  "suiteRef": "https://reg.example.org/suites/mmlu-1.0",
  "modelRef": "https://reg.example.org/models/example-7b",
  "runAt": "2026-04-28T11:32:00+09:00"
}
```

## Annex K — Prompt-injection scanner

```
POST /v1/safety/scan
```

Body: a prompt body with parameter values.
Response: a list of detected prompt-injection
patterns and recommended mitigations.

## Annex L — Cost estimator

```
POST /v1/conversations/cost-estimate
```

Body: an ordered prompt sequence and a target
model reference. Response: an estimated cost
range based on the model's published rate card.

## Annex M — Prompt diff

```
POST /v1/prompts/diff
```

Body: two prompt references. Response: a
structural diff at the body, parameter,
output-contract, and safety-policy levels.

## Annex N — Tool-call schema validation

```
POST /v1/contracts/{contractRef}/validate-tool-call
```

Body: a candidate tool-call response. Response:
schema verdict plus repaired payload when the
declared repair policy permits.

## Annex O — Reviewer queue

```
GET /v1/safety/queue?reviewerRef={ref}
```

Returns the safety reviewer's pending queue.
Reviewers acknowledge mitigation records before
the prompt enters production.

## Annex P — Bulk evaluation submission

```
POST /v1/evaluations/results/bulk
Content-Type: application/jsonl
```

Body: a JSON-Lines stream of evaluation results.
Response: a per-line verdict.

## Annex Q — Federated benchmarks

```
POST /v1/registry/benchmark
```

Body: SHACL filter over models and a target
evaluation suite. Response: anonymised benchmark
of the deployment's model results against the
registered population.

## Annex R — Conversation export

```
GET /v1/conversations/{conversationRef}/export
```

Returns a signed export of the conversation:
turn history, model versions, tool calls, and
applied safety policies.

## Annex S — Vendor catalogue

```
GET /v1/registry/vendors/{vendorRef}
```

Returns the vendor's catalogue with model
versions, declared safety evaluations, and EU AI
Act risk classifications.

## Annex T — Researcher access

```
POST /v1/registry/researcher-access
```

Body: SHACL filter and a project identifier.
Response: scoped JWT for de-identified
conversation extracts.

## Annex U — Conformity assessment endpoint

```
GET /v1/registry/eu-ai-act/{deploymentRef}
```

Returns the deployment's EU AI Act conformity
assessment with article references and
post-market monitoring status.

## Annex V — Vendor migration kit

```
POST /v1/registry/vendor-migration
```

Body: source vendor identity and a SHACL filter
over prompts and model cards. Response: a signed
URL referencing a `tar.zst` of records suitable
for import at the destination vendor's registry.

## Annex W — Audit-grade conversation export

Conversation exports include the conversation
turns, the resolved model versions, the applied
safety policies, and a per-turn redaction trace
so that auditors can reproduce the conversation
and verify policy application.

## Annex X — Bulk policy application

```
POST /v1/safety/bulk-apply
```

Body: SHACL filter over prompts and a policy
reference. Response: a per-prompt verdict for
operators rolling out new safety policies across
the catalogue.

## Annex Y — Quotas

Per-publisher quotas: 1,000 prompt versions /
hour; 10,000 / day. Per-conversation token caps
are configured per deployment.

## Annex Z — Sandbox

`/v1/sandbox` mirrors production with synthetic
prompts. Sandbox responses carry
`X-WIA-Sandbox: true` and reset every 24h.

## Annex AA — Audit feed retention

The audit feed is retained for at least 7 years
under the operator's data-management plan to
support EU AI Act post-market monitoring and
sovereign-equivalent retention requirements.

## Annex AB — Cohort definition

```
POST /v1/registry/cohorts
```

Body: a SHACL filter over conversation-state
records. Response: an opaque `cohortRef` that
researchers and auditors can reference in
extracts and benchmark queries.

## Annex AC — Bulk export

```
POST /v1/registry/export
```

Body: a SHACL filter over prompts, model cards,
and conversations. Response: a signed URL
referencing a `tar.zst` of records suitable for
auditor review.

## Annex AD — Webhook delivery shape

Webhook payloads are canonicalised per RFC 8785
before HMAC-SHA-256 signing. Signature appears
in `X-WIA-Signature: sha256=<hex>`. Deliveries
follow at-least-once semantics with backoff.

## Annex AE — Compliance attestation

```
GET /v1/registry/attestations
```

Returns the deployment's compliance attestations
(ISO/IEC 42001, NIST AI RMF, EU AI Act conformity
assessment) with effective and expiry dates.

## Annex AF — Idempotency for evaluation

Evaluation submissions accept an
`Idempotency-Key`. Re-submission with the same
key returns the original verdict. Used by
evaluation harnesses that retry transient
failures.

## Annex AG — Reviewer endorsement

```
POST /v1/safety/{policyRef}/endorse
```

Body: reviewer LEI / DID, verdict (`endorsed`,
`disputed`, `superseded`), reasoning. Endorsements
accumulate as a publication trail visible on the
policy record.

弘益人間 (Hongik Ingan) — Benefit All Humanity
