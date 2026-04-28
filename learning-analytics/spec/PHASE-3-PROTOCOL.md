# WIA-learning-analytics PHASE 3 — Protocol Specification

**Standard:** WIA-learning-analytics
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocols used by
WIA-learning-analytics participants for discovery,
xAPI / Caliper transport, profile distribution,
intervention delivery, dashboard access policy,
and signed evidence transport.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 6797 (HSTS)
- IETF RFC 8615 (Well-Known URIs), RFC 7517 (JWK), RFC 7515 (JWS)
- IETF RFC 9421 (HTTP Message Signatures), RFC 9530 (Digest Fields)
- IEEE 1484.20.1 xAPI 1.0.3 (LRS contract)
- IMS Caliper 1.2 (Sensor API, Profile)
- ISO/IEC 20748-3 Reference Architecture
- W3C Web Crypto API, W3C SHACL

---

## §1 Scope

This PHASE defines the on-the-wire behaviour between
sensors and LRSs; between LRSs and downstream
analytics consumers; between profile servers and
LRSs; and between intervention orchestrators and
their human reviewers.

## §2 Discovery

A WIA-learning-analytics registry serves a discovery
document at:

```
GET /.well-known/wia/learning-analytics
```

Response (`application/json`):

```json
{
  "registry": "https://la.wiastandards.com",
  "lrs": "https://la.wiastandards.com/v1/statements",
  "openapi": "https://la.wiastandards.com/openapi.json",
  "operationGroups": ["/v1/statements", "/v1/profiles",
                      "/v1/activities", "/v1/actors",
                      "/v1/models", "/v1/interventions",
                      "/v1/dashboards", "/v1/registry"],
  "supportedProfiles": ["cmi5", "ADL", "Open-Cmi5"],
  "keySet": "https://la.wiastandards.com/.well-known/jwks.json"
}
```

The discovery document is signed (RFC 9421).

## §3 Transport

HTTPS with TLS 1.3 (RFC 8446) and HSTS preload.
Caliper push endpoints use HTTP/2 streams to amortise
the cost of frequent envelope deliveries.

## §4 Content negotiation

| Accept                                | Use                                      |
|---------------------------------------|------------------------------------------|
| `application/json`                    | xAPI / record bodies                     |
| `application/x-imscaliper+json`       | Caliper envelopes                        |
| `application/cmi5+json`               | cmi5 ASN/launch payload                  |
| `application/sequence-diagram+json`   | profile pattern visualisation            |
| `application/problem+json`            | error                                    |

## §5 xAPI transport

xAPI statements are transmitted per IEEE 1484.20.1
§7. The standard's wire format applies; this PHASE
imposes the additional constraints of §2 PHASE-1
(forbidden mailbox identifiers, profile-required
verbs, statement size limits).

The xAPI version header is `X-Experience-API-Version:
1.0.3`. Statements emitted with later xAPI versions
are accepted only if the registry advertises support
in its discovery document.

## §6 Caliper transport

Caliper Sensor envelopes are POSTed to the LRS
endpoint. Each envelope carries:

- `sensor` (sensor identity URI);
- `sendTime` (ISO 8601);
- `dataVersion` (Caliper Profile URL);
- `data[]` (array of event objects).

The LRS persists envelopes verbatim and indexes the
inner events for query.

## §7 Profile distribution

Profiles are distributed as signed JSON-LD documents
served from the profile registry. Profile signatures
use detached JWS (RFC 7515) over the canonical form
(RFC 8785). Consumers verify the signature before
applying templates and patterns.

## §8 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| `actorRef`         | UUID (RFC 4122)                                 |
| `statementRef`     | UUID; client-provided                           |
| `profileRef`       | URI                                             |
| `modelRef`         | UUID                                            |
| `interventionRef`  | UUID                                            |
| `dashboardRef`     | URI                                             |
| `activityRef`      | URI                                             |

## §9 Caching and immutability

Statements are append-only. Voiding emits a new
statement; the original remains queryable. Profiles
are immutable per published version; new versions are
new resources. Activity records are mutable; they
carry strong `ETag`.

## §10 Federation

Federated LRSs propagate statements across peers when
the destination LRS declares support for the profile
asserted on the statement. Trust between peers is
anchored in the JWKS exposed at
`/.well-known/jwks.json`.

## §11 Replay protection

Statement submissions carry `iat`/`exp` JWS claims
when signed. The registry rejects statements whose
`stored` time would be earlier than its `timestamp`
by more than 24 hours unless the publisher declares
a backfill window with a documented justification.

## §12 Error semantics

Errors are `application/problem+json` (RFC 9457).
Protocol-level codes:

| Code | Meaning                                              |
|------|------------------------------------------------------|
| 200  | success                                              |
| 204  | statement stored                                     |
| 400  | malformed JSON / xAPI / Caliper                      |
| 401  | missing or invalid token                             |
| 403  | profile collision; actor consent withdrawn           |
| 409  | statement ID collision with divergent body           |
| 410  | actor consent withdrawn (read tombstone)             |
| 413  | payload too large                                    |
| 422  | profile template / pattern violation                 |
| 426  | TLS upgrade required                                 |

## §13 Observability

Servers SHOULD emit OpenTelemetry traces with
`wia.la.operation`, `wia.la.profile`,
`wia.la.actor` (pseudonymised), and
`wia.la.statementCount` attributes.

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** discovery served,
  statements signed.
- **Tier 2 — Verified:** profile and pattern
  conformance audited; Caliper interop tested.
- **Tier 3 — Anchored:** continuous evidence stream
  per PHASE-4 Annex G.

## Annex B — xAPI Resources subset

The LRS implements the xAPI Resource subset per
IEEE 1484.20.1 §7.2 (Statements API) and §7.3 (State
API). The Activities API and Agents API are
optional; deployments that publish profile-specific
activity dictionaries surface them at §6 of PHASE-2.

## Annex C — Caliper push frequency

Sensors batch events at minute granularity by
default; high-frequency sensors (e.g. live coding
keystroke capture) MAY batch at 10s granularity, in
which case the LRS rate-limit applies. The default
quota is 10,000 statements per minute per
deployment.

## Annex D — Discovery document signature

The signature over `/.well-known/wia/learning-
analytics` covers `@authority`, `@path`,
`content-digest` (RFC 9530), and `content-type`.

## Annex E — Cross-Origin Resource Sharing

The LRS exposes statement query endpoints with
`Access-Control-Allow-Origin: *` and
`Access-Control-Expose-Headers: ETag,
X-Experience-API-Version, Link`.

## Annex F — Trust anchor rotation

Signing keys rotate on a 24-month rolling schedule
with a 6-month overlap window during which old keys
remain in the JWKS.

## Annex G — Connection management

Long-lived sensors use HTTP/2 streams. A sensor MAY
emit a continuous Caliper envelope stream over a
single connection; the LRS terminates the connection
on heartbeat loss after 30 seconds.

## Annex H — Replay-resistant intervention delivery

Intervention deliveries are signed (RFC 9421) with
`iat`/`exp` claims (max 1h). The reviewer endpoint
verifies the signature and the freshness window
before exposing the intervention payload.

## Annex I — Recording retention

The default retention window is 7 years for sovereign
education ministries, 5 years for accredited
providers, and 13 months for self-study deployments.
Retention windows are declared at registration and
verified at audit.

## Annex J — Server-Sent Events for live dashboards

Dashboards refreshing in real time use Server-Sent
Events to subscribe to relevant Caliper events. The
SSE stream is filtered to the dashboard's declared
data scope; cross-scope leakage is rejected at the
sensor boundary.

## Annex K — xAPI batch encoding

Batched xAPI statements use `application/json` array
encoding. The batch size is bounded by §11 of PHASE-1
and the LRS deployment's per-request size limit. The
LRS commits the batch atomically: either every
statement is stored or none are, with a Problem
Details fragment listing the offending statement on
failure.

## Annex L — Caliper envelope ordering

Caliper envelopes carry monotonically increasing
sendTime per sensor. The LRS reorders inner events
by `eventTime` for query consistency but preserves
the envelope's `sendTime` ordering for audit.

## Annex M — JWS attachment signature

The xAPI statement signature attachment carries a
detached JWS over the canonical statement (RFC 8785).
The signature MUST cover the `actor`, `verb`,
`object`, `result`, `context`, `timestamp` fields;
attachments other than the signature are not part of
the signed content.

## Annex N — Sensor authentication

Sensors authenticate at LRS submission via OAuth 2.0
client-credentials flows. The client identity is
bound to a sensor identity document referenced in
the discovery JWKS. Sensor identity rotation follows
the same 24-month cadence as registry signing keys.

## Annex N1 — Backfill window

A publisher may submit statements with timestamps
older than 24 hours by declaring a backfill window
in the discovery document. Backfill submissions
carry an `X-WIA-Backfill: <window-id>` header and
are subjected to additional audit logging so that
ministry reviewers can detect retroactive
manipulation.

## Annex N2 — Live dashboard SSE keepalive

SSE streams emit a keepalive comment every 15 seconds
to maintain the connection across NAT and proxy
timeout windows. Disconnects without a clean close
trigger an audit-feed entry so that dashboard owners
can detect prolonged outages affecting data delivery.

## Annex O — Federation propagation cap

A federated propagation hop carries an
`X-WIA-Federation-Hops` header incremented at each
peer. Statements received with `Hops > 3` are
dropped to prevent loop-storm scenarios.

## Annex P — Statement compression

xAPI statement batches MAY be compressed using
`Content-Encoding: zstd` or `gzip`. The LRS
declares supported encodings in
`Accept-Encoding: zstd, gzip` returned during
preflight. Compression ratios for typical mixed
xAPI / Caliper traffic average 4:1; the LRS reserves
the right to reject batches whose compressed size
exceeds 10 MB.

## Annex Q — Profile schema validation

When profiles are published, the registry runs SHACL
validation against the canonical xAPI Profile schema
plus any registry-specific overlays. Validation
failures surface as a Problem Details fragment with
the failing SHACL rule and the offending profile
fragment highlighted.

## Annex R — JWS attachment publication path

Statement signature attachments are mirrored at:

```
GET /v1/statements/{statementRef}/signature
```

Mirroring serves clients that cache statement bodies
separately from attachments. The mirror response is
cached identically to the primary attachment. Mirror
hits are recorded in the audit feed under
`signature.mirror-hit` so that publishers can detect
unusual signature-fetch patterns indicative of
verification probing.

弘益人間 (Hongik Ingan) — Benefit All Humanity
