# WIA-plugins PHASE 3 — Protocol Specification

**Standard:** WIA-plugins
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocols used by
WIA-plugins participants for discovery, plugin
search and lookup, archive retrieval, signature
verification, capability negotiation, and SBOM /
vulnerability query.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 6797 (HSTS)
- IETF RFC 8615 (Well-Known URIs), RFC 7517 (JWK), RFC 7515 (JWS)
- IETF RFC 9421 (HTTP Message Signatures), RFC 9530 (Digest Fields)
- IETF RFC 7234 (HTTP cache), RFC 8259 (JSON), RFC 8785 (JCS)
- W3C Subresource Integrity, W3C Content Security Policy 3
- WHATWG Fetch (CORS, redirect), URL Living Standard

---

## §1 Scope

This PHASE defines the on-the-wire behaviour
between hosts and plugin registries; between
publishers and registries; and between federated
registries.

## §2 Discovery

A WIA-plugins registry serves a discovery document
at:

```
GET /.well-known/wia/plugins
```

Response (`application/json`):

```json
{
  "registry": "https://plugins.wiastandards.com",
  "openapi": "https://plugins.wiastandards.com/openapi.json",
  "operationGroups": ["/v1/plugins", "/v1/manifests",
                      "/v1/releases", "/v1/sboms",
                      "/v1/capabilities", "/v1/hosts",
                      "/v1/locales", "/v1/registry"],
  "supportedHosts": ["host:vscode", "host:wordpress",
                     "host:browser", "host:obs-studio"],
  "keySet": "https://plugins.wiastandards.com/.well-known/jwks.json"
}
```

The discovery document is signed (RFC 9421) using
the registry's signing key from the declared JWKS.

## §3 Transport

HTTPS with TLS 1.3 and HSTS preload mandatory.
Archive retrieval supports HTTP/2 and HTTP/3
streams to amortise download cost.

## §4 Content negotiation

| Accept                                | Use                                      |
|---------------------------------------|------------------------------------------|
| `application/json`                    | record bodies                            |
| `application/zip`                     | release archives (`vsix`, `crx`, `xpi`)  |
| `application/x-tar+gzip`              | tar.gz release archives                  |
| `application/wasm`                    | WebAssembly Component releases           |
| `application/cyclonedx+json`          | CycloneDX 1.5 SBOM                       |
| `application/spdx+json`               | SPDX 2.3 SBOM                            |
| `application/problem+json`            | error                                    |

## §5 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| `pluginRef`        | UUID (RFC 4122)                                 |
| `manifestRef`      | URI                                             |
| `releaseRef`       | URI                                             |
| `archiveDigest`    | `sha-512` per RFC 9530                          |
| `sbomRef`          | URI                                             |
| `capabilityRef`    | UUID                                            |
| `hostRef`          | URI                                             |

## §6 Signature verification

Hosts verify in the order:

1. Discovery document signature (registry trust
   anchor).
2. Manifest signature (publisher key under the
   signed key set).
3. Archive digest match (hash check after fetch).
4. SBOM signature (publisher key, same set).

A failure at any step aborts installation.

## §7 Caching and integrity

Release archives and SBOMs are immutable; they
carry `Cache-Control: public, max-age=31536000,
immutable`. Manifest records carry strong `ETag`
and short max-age. Subresource Integrity (`integrity`
HTML attribute) is the in-browser path; the
`Digest` header (RFC 9530) is the API-level path.

## §8 Federation

Federated registries form a directed graph in the
discovery document. Cross-registry queries follow
the graph and carry an `X-WIA-Federation-Path`
header.

## §9 Replay and yanking

Yanking a release is signalled by a tombstone-like
state visible at the release-record endpoint. The
archive remains fetchable for at least 30 days
after yank to support audit reproductions.
Replay attacks on signed publishes are prevented
by the manifest's `iat`/`exp` JWS claims (max 24h).

## §10 Error semantics

Errors are `application/problem+json` (RFC 9457).
Protocol-level codes:

| Code | Meaning                                              |
|------|------------------------------------------------------|
| 200  | success                                              |
| 304  | conditional GET unchanged                            |
| 401  | missing or invalid token                             |
| 403  | host policy refused capability                       |
| 404  | unknown plugin / release                             |
| 409  | publication conflict (digest divergence)             |
| 410  | yanked release                                       |
| 422  | manifest validation failure                          |
| 426  | TLS upgrade required                                 |
| 503  | federation peer unavailable                          |

## §11 Observability

Servers SHOULD emit OpenTelemetry traces with
`wia.plugins.operation`, `wia.plugins.host`,
`wia.plugins.publisher`, and
`wia.plugins.releaseRef` attributes.

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** discovery served,
  releases signed.
- **Tier 2 — Verified:** federation interop tested,
  signing key rotation exercised.
- **Tier 3 — Anchored:** continuous evidence stream
  per PHASE-4 Annex G.

## Annex B — Discovery document signature

The signature over `/.well-known/wia/plugins`
covers `@authority`, `@path`, `content-digest`
(RFC 9530), and `content-type`.

## Annex C — Cross-Origin Resource Sharing

Read endpoints serve `Access-Control-Allow-Origin:
*` with `ETag`, `Digest`, and `Link` exposed.
Browser hosts fetching releases over CORS MUST set
`crossorigin="anonymous"` so SRI can be enforced.

## Annex D — Trust anchor rotation

Signing key sets rotate on a 24-month rolling
schedule with a 6-month overlap.

## Annex E — Connection management

HTTP/2 connection coalescing applies to the
registry sub-domains. HTTP/3 0-RTT is permitted for
read endpoints; write endpoints reject 0-RTT to
avoid replay.

## Annex F — Federation hop cap

Federated lookups carry an
`X-WIA-Federation-Hops` header; queries with `Hops
> 3` are dropped.

## Annex G — Yank propagation

Yanks propagate across federation peers within 60s
of the originating yank event. Peers that fail to
honour the yank within 24h are flagged in the
audit feed.

## Annex H — TLS profile baseline

TLS 1.3 with PFS-only cipher suites; NIST SP
800-52 Rev. 2 baseline. TLS 1.2 not accepted for
write endpoints.

## Annex I — Background update protocol

Hosts poll for updates on a documented cadence
(default 24h). The poll uses `If-None-Match` with
the publisher's manifest `ETag` so that unchanged
manifests return 304 without transferring the
body. Update notifications surface in the host's
UI per host policy.

## Annex J — Streamed archive download

Archive downloads use chunked HTTP/2 streams. The
host computes the SHA-512 digest as bytes arrive
and aborts the download on first divergence from
the `Digest` header.

## Annex K — Federated yank propagation

Yank events propagate through the federation graph
within 60 seconds. Peers that fail to honour the
yank within 24h are flagged in the audit feed and
surfaced as `federation.yank-lag` events.

## Annex L — Background-job semantics

Long-running jobs (large archive uploads, SBOM
generation, vulnerability join) accept 202
Accepted with a polling URL and a webhook
subscription option. Polling responses include a
progress percentage and an estimated completion
time.

## Annex M — Replay-resistant install token

Install tokens carry `iat`/`exp` claims (max 1h)
and an `aud` matching the host's identity. Tokens
captured by an intermediary cannot be replayed
against a different host.

## Annex N — Sandbox endpoints

`/v1/sandbox` mirrors production with synthetic
plugins and ephemeral state. Sandbox responses
carry `X-WIA-Sandbox: true`.

## Annex O — JSON canonicalisation

JSON-bearing manifest and SBOM records are
canonicalised per RFC 8785 prior to signature.

## Annex P — Connection-level rate limits

Per-host download rate limits default to 1 GB / hr
per IP for unauthenticated users; authenticated
users are bounded by the deployment's quota.

## Annex Q — Hardware-backed publisher key

Publisher signing keys MAY be hardware-bound (HSM,
FIDO2 with attestation, smart card). When
hardware-bound, the registry records the
attestation in the publisher record. Hardware-
bound keys are mandatory for plugins running with
elevated capabilities (`subprocess`, `filesystem:
read-write`, `eval: permit`).

## Annex R — Replica labels

Registries that operate read replicas surface the
serving replica in the `X-WIA-Replica` response
header. The label is informative; clients MUST NOT
route based on it.

## Annex S — Cross-Origin install token

Install tokens served to browser hosts include the
`SameSite=Lax` cookie attribute and are bound to
the host's origin. Cross-origin install attempts
are rejected at the Sec-Fetch-Site boundary.

## Annex T — Discovery cache TTL

The discovery document carries
`Cache-Control: public, max-age=300,
stale-while-revalidate=60`.

## Annex U — Webhook delivery retry

Webhook deliveries follow at-least-once semantics
with exponential backoff (2, 4, 8, 16, 32, 64,
128, 256, 512 seconds) capped at 9 attempts.
Failed deliveries enter a dead-letter queue
inspectable in the audit feed.

## Annex V — Mirror registry interop

A federation peer that mirrors the primary
registry signs a peer assertion published at
`/.well-known/wia/peer/{id}`. Mirrors that
maintain a peer assertion participate in yank
propagation and federation queries; mirrors
without a peer assertion are read-only consumers.

## Annex W — WebSub push delivery

Atom feeds publish a `<link rel="hub">` pointing
to the registry's WebSub hub. Subscribers register
at the hub with a callback URL and a verification
token. The hub pushes new entries to subscribers
within 60 seconds of publication.

## Annex X — Compression preferences

Read endpoints declare
`Accept-Encoding: br, zstd, gzip`. Brotli is
preferred for HTTP/2 clients; gzip is the
universal fallback.

## Annex Y — Connection coalescing

HTTP/2 clients MAY coalesce connections across
sub-domains served by the same certificate when
the server presents a matching origin. The
registry publishes a connection-coalescing policy
in the discovery document so that clients can
reason about it.

## Annex Z — Pagination convention

List endpoints carry a stable opaque cursor in the
`next` field of the response body and in the
`Link` header (`rel=next`). Cursors are valid for
at least 24h after issuance.

## Annex AA — TLS session resumption

TLS 1.3 session resumption is permitted via PSK.
Resumed sessions inherit the original session's
authentication state but cannot extend privileges
beyond the original session.

弘益人間 (Hongik Ingan) — Benefit All Humanity
