# WIA-ai-assistant PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-ai-assistant
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that an AI
assistant operator exposes for the records defined in
PHASE-1. Consumers include model providers, evaluation
service vendors, red-teaming providers, regulators
(under the operating jurisdiction's AI governance regime),
and the operator's own audit and analytics platforms.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 / 6902 (JSON Pointer / Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C Trace Context
- OpenAPI Specification 3.1
- JSON Schema 2020-12
- NIST AI Risk Management Framework + AI 600-1 GenAI
  Profile

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The
OpenAPI 3.1 document at `/v1/openapi.json` is canonical.

The chat-completion runtime (per-token streaming, sampling
parameters) is documented by the operator's model
provider's API; this WIA facade is the metadata, audit,
and governance layer over the runtime, not the runtime
itself.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-ai-assistant",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "models":             "/v1/models",
    "systemPrompts":      "/v1/system-prompts",
    "toolSchemas":        "/v1/tool-schemas",
    "conversations":      "/v1/conversations",
    "toolCalls":          "/v1/tool-calls",
    "evaluationRuns":     "/v1/evaluation-runs",
    "safetyIncidents":    "/v1/safety-incidents",
    "evidence":           "/v1/evidence",
    "openapi":            "/v1/openapi.json"
  }
}
```

## §3 Programme Lifecycle

```
POST   /v1/programmes              — register a programme
GET    /v1/programmes/{pid}        — retrieve programme
PATCH  /v1/programmes/{pid}/status — advance status
PATCH  /v1/programmes/{pid}/governing-frameworks
                                    — update governing AI
                                       framework enrolment
```

Programmes that operate in EU jurisdictions where the EU
AI Act 2024 applies require an `EU-AI-Act-2024` framework
enrolment with a per-risk-category attestation; mismatched
submissions return `409` with type
`urn:wia:ai-assistant:eu-ai-act-attestation-required`.

## §4 Model Registration

```
POST   /v1/programmes/{pid}/models       — register a model
PATCH  /v1/models/{mid}/deprecation      — announce
                                              deprecation
GET    /v1/models/{mid}                  — retrieve model
```

Model registrations require a `modelCardRef`; submissions
without the model card return `422` with type
`urn:wia:ai-assistant:model-card-required`.

## §5 System Prompts

```
POST   /v1/programmes/{pid}/system-prompts
                                          — register a prompt
PATCH  /v1/system-prompts/{spid}/superseded-by
                                          — record successor
GET    /v1/system-prompts/{spid}         — retrieve prompt
                                              metadata
GET    /v1/system-prompts/{spid}/content — retrieve prompt
                                              text (gated by
                                              the operator's
                                              prompt-disclosure
                                              policy)
```

Prompt registrations require an `approvedBy` reference;
operator-internal prompt-governance committee approval is
prerequisite.

## §6 Tool Schemas

```
POST   /v1/programmes/{pid}/tool-schemas — register a tool
GET    /v1/tool-schemas/{tid}            — retrieve tool
                                              metadata
GET    /v1/tool-schemas/{tid}/input-schema
                                          — fetch JSON
                                              Schema input
GET    /v1/tool-schemas/{tid}/output-schema
                                          — fetch JSON
                                              Schema output
```

Tools with `authorisationScope=user-context-mutating` or
`operator-internal-system-action` require operator
authorisation matrix binding before the tool can be
invoked from a conversation.

## §7 Conversation Lifecycle

```
POST   /v1/programmes/{pid}/conversations
                                          — open a
                                              conversation
                                              (pin model and
                                              system prompt)
PATCH  /v1/conversations/{cid}/end       — close conversation
                                              with message
                                              count
GET    /v1/conversations/{cid}           — retrieve metadata
GET    /v1/conversations/{cid}/transcript
                                          — fetch redacted
                                              transcript
                                              (per operator's
                                              redaction policy)
```

Conversation submissions whose `userTokenRef` is not
recognised by the operator's IDP return `403 Forbidden`
with type `urn:wia:ai-assistant:user-token-not-resolved`.

## §8 Tool Calls

```
POST   /v1/conversations/{cid}/tool-calls
                                          — record a tool
                                              invocation
GET    /v1/tool-calls/{tcid}             — retrieve call
                                              record
GET    /v1/conversations/{cid}/tool-calls
                                          — list tool calls
                                              for a
                                              conversation
```

Tool calls that exceed the per-session rate limit return
`429 Too Many Requests` with `Retry-After`; calls
rejected by the authorisation matrix return `403` with
type `urn:wia:ai-assistant:tool-not-authorised`.

## §9 Evaluation Runs

```
POST   /v1/programmes/{pid}/evaluation-runs
                                          — register an
                                              evaluation
                                              run
PATCH  /v1/evaluation-runs/{rid}/end     — close run with
                                              results and
                                              verdict
GET    /v1/evaluation-runs/{rid}         — retrieve run
GET    /v1/programmes/{pid}/evaluation-runs?
       kind={k}&from={t}                   — query runs
```

Robustness evaluation runs (`kind=robustness-iso-iec-
24029-1`) cite the per-test-vector benchmark suite; the
operator's pipeline regenerates results on each model or
prompt revision.

## §10 Safety Incidents

```
POST   /v1/programmes/{pid}/safety-incidents
                                          — register an
                                              incident
PATCH  /v1/safety-incidents/{sid}/severity
                                          — update severity
PATCH  /v1/safety-incidents/{sid}/root-cause
                                          — attach root-
                                              cause reference
PATCH  /v1/safety-incidents/{sid}/remediation
                                          — attach
                                              remediation
                                              plan
GET    /v1/safety-incidents/{sid}        — retrieve incident
```

Incidents at severity `critical` automatically trigger
regulator notification through the integration described
in PHASE-4 §6 where the jurisdiction requires.

## §11 Errors, Authentication, Caching, Audit

Errors: `application/problem+json` per RFC 9457 with the
types named in §3-§10 plus
`urn:wia:ai-assistant:evidence-mismatch`. Authentication:
mutually-authenticated TLS for evaluation, regulator, and
partner consumers; consumer-facing transcripts respect the
operator's IDP. Caching: stable resources (closed
conversations, completed evaluation runs, signed evidence
packages) cacheable with `Cache-Control: max-age=
31536000, immutable`. Audit logs carry `programmeId`,
`conversationId`, `traceId`, the issuing client
certificate's subject, and the operator's clock skew vs
the operating jurisdiction's NTP service.

## §12 Streaming Subscription

Consumers subscribe via Server-Sent Events at:

- `/v1/programmes/{pid}/events` — programme-wide events
  (model deprecation, prompt revision, evaluation
  outcomes, safety incident escalations).
- `/v1/conversations/{cid}/events` — conversation-scoped
  events (tool-call outcomes, end-of-conversation
  notifications).

Subscribers reconnect via the `Last-Event-ID` header.

## §13 Bulk and Pagination

```
POST   /v1/bulk/conversations            — batched
                                              conversation
                                              metadata import
POST   /v1/bulk/tool-calls               — batched tool-call
                                              ingest
GET    /v1/bulk/{operationId}            — operation status
```

Cursor-based pagination uses the `cursor` query parameter
and `Link` headers (RFC 8288); cursors persist for at
least 24 hours.

## §14 Worked Example: Conversation to Incident

1. The operator opens a conversation pinning model and
   system prompt.
2. The user issues prompts that trigger tool calls
   recorded against the conversation.
3. A safety-monitor pipeline detects PII leak in the
   assistant response and registers a safety incident
   referencing the conversation.
4. The operator's incident-response team attaches root-
   cause and remediation references.
5. Per the operator's policy, severity `major` triggers
   regulator notification through the PHASE-4 integration.

## §15 Memory and Feedback Endpoints

```
POST   /v1/users/{utid}/memory             — register a
                                                memory record
PATCH  /v1/memory/{mid}/opt-out             — record user
                                                opt-out
GET    /v1/users/{utid}/memory?
       class={c}&visibility={v}              — query memory
                                                for a user
POST   /v1/conversations/{cid}/feedback    — register user
                                                feedback
GET    /v1/feedback/{fid}                  — retrieve feedback
GET    /v1/conversations/{cid}/feedback    — list feedback
                                                for a
                                                conversation
```

Memory submissions for users that have opted out (PHASE-1
§10 `userOptOutHonoured=true`) return `409 Conflict` with
type `urn:wia:ai-assistant:memory-opt-out-active` and the
operator's pipeline suppresses the persistence.

## §16 Provenance and Aggregation

```
GET    /v1/provenance/{recordId}    — provenance entry for
                                       any PHASE-1 record
GET    /v1/aggregate/conversation-volume?period=...
GET    /v1/aggregate/safety-incident-rate?period=...
GET    /v1/aggregate/feedback-distribution?period=...
GET    /v1/aggregate/tool-usage-by-scope?period=...
```

## §17 Source Citation Endpoint

```
POST   /v1/conversations/{cid}/source-citations
                                          — register a per-
                                            response citation
GET    /v1/source-citations/{scid}        — retrieve citation
GET    /v1/conversations/{cid}/source-citations
                                          — list citations
                                            for a conversation
```

Citation submissions whose `sourceTrustClass` is
`model-internal-no-source` for a regulated-domain
conversation return `409` with type
`urn:wia:ai-assistant:source-citation-required` and the
operator's defer-to-human escalation triggers.

## §18 Conformance

A conformant server passes the test vectors published
under `tests/phase-vectors/phase-2-api-interface/`, emits
an OpenAPI 3.1 document, signs evidence packages per RFC
9421, and rejects PII in any DATA-FORMAT field this PHASE
marks as opaque.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-ai-assistant
- **Last Updated:** 2026-04-28
