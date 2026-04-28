# WIA-generative-ai PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-generative-ai
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that a
generative-AI operator (model provider, system
deployer, or downstream deployer) exposes for the
records defined in PHASE-1. The contract is consumed
by the deployer's application surface, the end-user
through the user-facing channel, the operator's
compliance and audit functions, and the regulatory
authority for the operating jurisdiction.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC 6901 /
  6902 (JSON Pointer / Patch), RFC 8288 (Web Linking),
  RFC 8259 (JSON), RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022, ISO/IEC 42001:2023
- W3C Trace Context, W3C Verifiable Credentials Data
  Model 2.0
- C2PA Content Credentials specification v1.4
- HuggingFace Model Card conventions
- ONNX (open neural-network exchange)
- EU AI Act Articles 50, 51 to 55, 71, 73
- EU GDPR Articles 12 to 22

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the
operator. Versioning uses `/v1/` path segments. The
OpenAPI 3.1 document at `/v1/openapi.json` is canonical.

The API exposes the inference surface (the user-
facing prompt-and-completion endpoint), the system
configuration surface (the deployer-facing system /
tool / index management endpoint), the registry
surface (the operator-facing model registry endpoint),
the content-provenance surface (the C2PA Content
Credentials issuance and verification endpoint), the
right-to-explanation and complaint surfaces (the user-
facing rights surfaces), and the regulator-examination
surface.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-generative-ai",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "models":               "/v1/models",
    "systems":              "/v1/systems",
    "fineTunes":            "/v1/fine-tunes",
    "inference":            "/v1/inference",
    "transcripts":          "/v1/transcripts",
    "contentCredentials":   "/v1/content-credentials",
    "evaluations":          "/v1/evaluations",
    "incidents":            "/v1/incidents",
    "userRights":           "/v1/user-rights",
    "examination":          "/v1/examination",
    "openapi":              "/v1/openapi.json"
  }
}
```

## §3 Inference Endpoints

```
POST   /v1/inference          (the user-facing prompt-
                                and-completion endpoint)
POST   /v1/inference/stream   (server-sent events for
                                token streaming)
POST   /v1/inference/batch    (asynchronous batch
                                inference)
```

The inference endpoint accepts the user's prompt, the
RAG-supplied retrieval results (where the system
configuration declares retrieval), and the tool-use
context. The response carries the model output, the
applied safety-filter decisions, the C2PA Content
Credentials reference for AI-generated content per
EU AI Act Article 50(2), and the latency and token-
usage metrics. The discipline at PHASE-3 §4 applies
before the output is returned.

## §4 System and Model Management Endpoints

```
GET    /v1/models
GET    /v1/models/{modelId}
POST   /v1/models                  (register a model)
GET    /v1/systems
GET    /v1/systems/{systemId}
POST   /v1/systems                 (register a deployed
                                    system)
PATCH  /v1/systems/{systemId}      (update system
                                    prompt, tools, or
                                    safety filters)
GET    /v1/fine-tunes
POST   /v1/fine-tunes              (record a new fine-
                                    tune)
```

The system-registration endpoint records the system
prompt, the tool catalogue, the retrieval index, the
safety filters, the intended-purpose declaration, and
the EU AI Act high-risk classification (where
applicable). System updates trigger the operator's
re-evaluation discipline (PHASE-3 §6).

## §5 Transcript and Audit Endpoints

```
GET    /v1/transcripts?user={userId}&from={iso}&to={iso}
GET    /v1/transcripts/{transcriptId}
DELETE /v1/transcripts/{transcriptId}     (user-
                                           initiated
                                           erasure
                                           per GDPR
                                           Article 17)
```

The transcript endpoint exposes the operator's
preserved input-output log; the user's right to access
their transcripts is exercised through the user-rights
surface.

## §6 Content-Credentials Endpoints

```
POST   /v1/content-credentials/issue
                              (issue a C2PA Content
                               Credentials manifest
                               for an AI-generated
                               output)
POST   /v1/content-credentials/verify
                              (verify a third-party-
                               supplied manifest)
GET    /v1/content-credentials/{credentialId}
```

Issuance follows the C2PA v1.4 specification — the
manifest carries the producer claim, the AI-generated
declaration, the signing key reference, and the
optional watermark detail. Verification returns the
manifest's chain-of-trust evaluation against the
C2PA trust list.

## §7 User-Rights Endpoints

```
GET    /v1/user-rights/me
POST   /v1/user-rights/me/access-request
POST   /v1/user-rights/me/erasure-request
POST   /v1/user-rights/me/article-22-3-review
POST   /v1/user-rights/me/article-50-disclosure
POST   /v1/user-rights/me/complaint
```

The Article 22(3) review endpoint records the user's
request for human review of an automated decision
(where the generative-AI system produces decisions
that fall under GDPR Article 22). The Article 50
disclosure endpoint records the user's request to
confirm that they are interacting with an AI system
or have been served AI-generated content; the
operator's response carries the disclosure narrative.

## §8 Evaluation and Incident Endpoints

```
GET    /v1/evaluations
GET    /v1/evaluations/{evaluationId}
POST   /v1/evaluations
POST   /v1/incidents              (operator-internal
                                   or external-
                                   researcher report)
GET    /v1/incidents/{incidentId}
PATCH  /v1/incidents/{incidentId} (record root cause
                                   and corrective
                                   actions)
```

The incident endpoint records reportable incidents
under EU AI Act Article 73 (for high-risk systems)
and the operator's internal incident-management
discipline; the corrective-action workflow is
exercised in PHASE-3 §9.

## §9 Examination Endpoints

```
GET    /v1/examination/programmes
GET    /v1/examination/models
GET    /v1/examination/systems
GET    /v1/examination/training-data-summary
GET    /v1/examination/incidents
GET    /v1/examination/audit-events
```

The examination scope is read-only and bound to the
regulator's identity (EU AI Office for GPAI,
Member-State market-surveillance authority for high-
risk systems, US sector regulator for sectoral
deployments, KR PIPC / NIA / FSC for KR
deployments).

## §10 Authentication, HTTP Status, and Caching

Bearer tokens conform to OAuth 2.1; per-surface
audiences distinguish user-facing, deployer-facing,
operator-facing, and examination scopes. Standard
HTTP status codes (200, 201, 202, 400, 401, 403, 404,
409, 422, 429, 503) apply with Problem Details
bodies. Generative-AI inference responses carry
`Cache-Control: no-store` to ensure that user-
specific or sensitive content does not enter shared
caches; the C2PA manifest references are cacheable
under their own integrity.

## §11 Webhook and Streaming Surface

The operator publishes lifecycle events through a
webhook channel registered by enterprise customers
and the operator's compliance function:

- `inference.completed`, `inference.refused`,
  `inference.streamed-completion`.
- `system.deployed`, `system.updated`,
  `system.retired`.
- `incident.reported`, `incident.resolved`.
- `evaluation.completed`.

Webhook signatures use HTTP Message Signatures (RFC
9421). Trace-context (`traceparent`) is propagated
across the operator's pipeline so that the inference
chain (prompt → retrieval → tool calls → model
output) can be reconstructed end to end.

## §12 Rate Limiting and Abuse-Mitigation Surface

Per-client and per-tenant rate limits are declared on
the OpenAPI document. The operator's rate-limiter
applies token-bucket throttling per scope; abuse-
pattern detection (high-volume jailbreak probing,
abusive-content generation attempts) feeds the
operator's incident-record surface. `429 Too Many
Requests` carries the `RateLimit` headers (RFC 9110
draft) declaring the remaining budget, the reset
window, and the per-window quota.

## §13 Conformance

Implementations claiming PHASE-2 conformance publish
the OpenAPI document, expose the inference surface
with the EU AI Act Article 50(2) Content Credentials
discipline, expose the user-rights surface for the
GDPR Article 15 / 17 / 22 and EU AI Act Article 50
disclosure rights, expose the regulator examination
surface, propagate trace-context across the inference
pipeline, and apply the rate-limiting and abuse-
mitigation discipline at the API boundary.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-generative-ai
- **Last Updated:** 2026-04-28
