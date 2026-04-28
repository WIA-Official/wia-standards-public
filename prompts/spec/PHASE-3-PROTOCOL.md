# WIA-prompts PHASE 3 — Protocol Specification

**Standard:** WIA-prompts
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocols used by
WIA-prompts participants for discovery, signed
prompt publication, conversation streaming,
evaluation result transport, safety-policy
distribution, and federation.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 6797 (HSTS)
- IETF RFC 8615 (Well-Known URIs), RFC 7517 (JWK), RFC 7515 (JWS)
- IETF RFC 9421 (HTTP Message Signatures), RFC 9530 (Digest Fields)
- IETF RFC 8259 (JSON), RFC 8785 (JCS)
- W3C Trace Context, W3C Server-Sent Events
- WHATWG Fetch (CORS, redirect handling)

---

## §1 Scope

This PHASE defines the on-the-wire behaviour
between prompt publishers and registries; between
deployment platforms and model vendors; and
between federated registries.

## §2 Discovery

A WIA-prompts registry serves a discovery document
at:

```
GET /.well-known/wia/prompts
```

Response (`application/json`):

```json
{
  "registry": "https://prompts.wiastandards.com",
  "openapi": "https://prompts.wiastandards.com/openapi.json",
  "operationGroups": ["/v1/prompts", "/v1/templates",
                      "/v1/contracts", "/v1/models",
                      "/v1/evaluations", "/v1/safety",
                      "/v1/conversations", "/v1/registry"],
  "supportedRenderEngines": ["mustache", "handlebars",
                             "jinja2", "wia-prompt-dsl-1.0"],
  "keySet": "https://prompts.wiastandards.com/.well-known/jwks.json"
}
```

The discovery document is signed (RFC 9421).

## §3 Transport

HTTPS with TLS 1.3 and HSTS preload. Conversation
streaming uses Server-Sent Events or WebSocket
over TLS depending on the platform's preference.

## §4 Content negotiation

| Accept                                | Use                                      |
|---------------------------------------|------------------------------------------|
| `application/json`                    | record bodies                            |
| `application/ld+json`                 | JSON-LD records (model cards, policies)  |
| `text/event-stream`                   | streaming conversation responses         |
| `application/problem+json`            | error                                    |

## §5 Signed publication

Prompt, template, model-card, evaluation-result,
and safety-policy records are signed with detached
JWS (RFC 7515) over the canonical JSON form (RFC
8785). The `kid` references the publisher's key
in the registry's JWKS.

## §6 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| `promptRef`        | UUID (RFC 4122)                                 |
| `templateRef`      | URI                                             |
| `contractRef`      | URI                                             |
| `modelRef`         | URI                                             |
| `suiteRef`         | URI                                             |
| `resultRef`        | UUID                                            |
| `policyRef`        | URI                                             |
| `conversationRef`  | UUID                                            |

## §7 Streaming responses

Conversation responses MAY be streamed via Server-
Sent Events. Each event carries `id`, `event`,
`data`, and optional `retry`. Stream resumption
follows SSE `Last-Event-ID`. Streaming responses
include the model's signature only at the final
event (`event: complete`); intermediate events
carry the running response without signature.

## §8 Caching and immutability

Prompt and template records at a specific version
are immutable; updates emit new versions. Model
cards are mutable through versioning. Evaluation
results and safety policies are immutable once
published.

## §9 Federation

Federated registries form a directed graph in the
discovery document. Cross-registry queries follow
the graph and carry an `X-WIA-Federation-Path`
header.

## §10 Replay protection

Signed publications carry `iat`/`exp` JWS claims
(max 24h). Conversation submissions carry per-
conversation nonces; duplicate turn submissions
are rejected as replay.

## §11 Error semantics

Errors are `application/problem+json` (RFC 9457).
Protocol-level codes:

| Code | Meaning                                              |
|------|------------------------------------------------------|
| 200  | success                                              |
| 304  | conditional GET unchanged                            |
| 400  | malformed JSON / SHACL violation                     |
| 401  | missing or invalid token                             |
| 403  | publisher not authorised for the prompt scope        |
| 410  | tombstone (withdrawn prompt, retired model)          |
| 422  | schema violation                                     |
| 429  | rate limit                                           |
| 503  | model vendor unavailable                             |

## §12 Observability

Servers SHOULD emit OpenTelemetry traces with
`wia.prompts.operation`, `wia.prompts.promptRef`,
`wia.prompts.modelRef`, and
`wia.prompts.conversationRef` attributes.

Trace IDs propagate via W3C Trace Context.

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** discovery served,
  prompts signed.
- **Tier 2 — Verified:** evaluation interop tested
  against a reference suite; model-card schema
  audited.
- **Tier 3 — Anchored:** continuous evidence stream
  per PHASE-4 Annex G; EU AI Act Article 43
  conformity assessment if high-risk.

## Annex B — SSE event format

Conversation streaming events carry a `data`
field containing a JSON object with `delta`
(append-only token stream), `tokenCount`, and an
optional `metadata` block.

## Annex C — Discovery document signature

The signature over `/.well-known/wia/prompts`
covers `@authority`, `@path`, `content-digest`
(RFC 9530), and `content-type`.

## Annex D — Cross-Origin Resource Sharing

Read endpoints serve `Access-Control-Allow-Origin:
*` with `ETag`, `Link`, and Trace Context headers
exposed.

## Annex E — Trust anchor rotation

Publisher signing keys rotate per the publisher's
policy. Recommended cadence is 12 months for
high-volume publishers.

## Annex F — Federation hop cap

Federated lookups carry an
`X-WIA-Federation-Hops` header; queries with
`Hops > 3` are dropped.

## Annex G — TLS profile baseline

TLS 1.3 with PFS-only cipher suites; NIST SP
800-52 Rev. 2 baseline.

## Annex H — Streaming back-pressure

Server-Sent Event streams honour client back-
pressure: when a client falls behind, the server
buffers up to 64 KB then closes the stream with
`event: error` and `data: {"reason":
"client-stalled"}`.

## Annex I — Sandbox endpoints

`/v1/sandbox` mirrors production with synthetic
prompts and ephemeral state.

## Annex J — Replay-resistant conversation submit

Conversation turn submissions carry an
`Idempotency-Key` and a per-conversation sequence
number. Duplicates are deduplicated server-side
within the 24h idempotency window.

## Annex K — Hardware-backed publisher key

Publisher signing keys MAY be hardware-bound (HSM,
FIDO2, smart card). Hardware binding is mandatory
for prompts that fall under EU AI Act high-risk
classification. The registry records the
attestation in the publisher record.

## Annex L — JSON canonicalisation

JSON-bearing records are canonicalised per RFC
8785 prior to JWS signature so that signatures
verify across implementations.

## Annex M — Discovery cache TTL

The discovery document carries
`Cache-Control: public, max-age=300,
stale-while-revalidate=60` so that consumers
cache it briefly without missing key rotations.

## Annex N — SSE keepalive

SSE streams emit a keepalive comment every 15
seconds to maintain the connection across NAT and
proxy timeout windows. Disconnects without a
clean close trigger a `conversation.disconnect`
audit event.

## Annex O — Webhook delivery

Webhook deliveries follow at-least-once semantics
with exponential backoff (2, 4, 8, 16, 32, 64,
128, 256, 512 seconds) capped at 9 attempts.
Failed deliveries enter a dead-letter queue.

## Annex P — Tool-call streaming

Tool calls emitted by the model during a
conversation are streamed inline with the SSE
event sequence. Each tool-call event carries a
`tool_call_id` so that the client can correlate
the call with the subsequent tool-result.

## Annex Q — Cross-Origin Resource Sharing

Read endpoints serve `Access-Control-Allow-Origin:
*` with `ETag`, `Link`, and Trace Context
headers exposed. Browser clients fetching
streaming responses use `withCredentials: true`
when authenticated.

## Annex R — Connection coalescing

HTTP/2 clients MAY coalesce connections across
sub-domains served by the same certificate. The
registry publishes the coalescing policy in the
discovery document.

## Annex S — Compression preferences

Read endpoints declare
`Accept-Encoding: br, zstd, gzip`. Streaming
responses use chunked transfer encoding without
compression so that real-time delivery is not
delayed by buffering.

## Annex T — Sandbox endpoints

`/v1/sandbox` mirrors production with synthetic
prompts and ephemeral state.

## Annex U — Replay protection on signed prompt
##           publish

Prompt publishes carry `iat`/`exp` JWS claims
(max 24h). The registry rejects replays whose
`iat` is older than the max window or whose
`jti` was already seen.

## Annex V — Discovery JWKS rotation

Discovery JWKS rotation cadence is 24 months
with a 6-month overlap. Old keys remain in the
JWKS during overlap. Clients re-resolve JWKS on
every cache TTL boundary.

## Annex W — Server-Sent Event reconnection

Clients reconnect using `Last-Event-ID`. The
server replays from the last event the client
acknowledged, up to a 60s window after which
replay is unavailable.

## Annex X — Trust matrix surface

The discovery document carries a peer trust
matrix listing each peer's accepted operation
groups and JWKS URL. Trust matrix changes are
surfaced as audit events.

## Annex Y — Conformance attestation lookup

```
GET /v1/registry/attestations
```

Returns the deployment's active compliance
attestations (ISO/IEC 42001, NIST AI RMF, EU AI
Act conformity) with effective and expiry dates.

## Annex Z — Streaming response chunked encoding

Streaming responses use chunked transfer encoding
with no compression so that delivery is real-
time. Final-event signatures cover the full
response body assembled by the client; the
client recomputes the canonical assembly before
verification.

## Annex AA — Trace context propagation

W3C Trace Context headers (`traceparent`,
`tracestate`) propagate across the full
conversation lifecycle: client → registry →
model vendor → tool host. Cross-org boundaries
preserve the traceparent so that auditors can
reconstruct end-to-end timelines.

## Annex AB — TLS PSK resumption

TLS 1.3 session resumption via PSK is permitted.
Resumed sessions inherit authentication state
but cannot extend privileges beyond the original
session scope.

弘益人間 (Hongik Ingan) — Benefit All Humanity
