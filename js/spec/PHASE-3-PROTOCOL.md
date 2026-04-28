# WIA-js PHASE 3 — Protocol Specification

**Standard:** WIA-js
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocol that
participants in WIA-js use for package discovery,
module fetching, signature verification, capability
negotiation, and conformance evidence transport. The
protocol is layered over HTTP semantics so that
existing CDN, cache, and proxy infrastructure carries
the workload without modification.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 9001 (QUIC), RFC 6797 (HSTS)
- IETF RFC 8615 (Well-Known URIs), RFC 7517 (JWK), RFC 7515 (JWS)
- IETF RFC 9457 (Problem Details), RFC 9530 (Digest Fields), RFC 7234 (HTTP cache)
- IETF RFC 9530 (Digest Fields), RFC 9421 (HTTP Message Signatures)
- WHATWG Fetch (CORS, redirect handling), URL Living Standard
- W3C Subresource Integrity, W3C Content Security Policy 3
- ECMAScript Module specifier resolution (ECMA-262 §16)

---

## §1 Scope

This PHASE defines the on-the-wire behaviour between a
JS client (build tool, runtime loader, browser fetcher)
and a WIA-js registry; between two federated registries;
and between a publishing principal and a registry. It
also defines the discovery contract that lets clients
locate the API surfaces enumerated in PHASE-2 without
prior configuration.

## §2 Discovery

A WIA-js registry serves a discovery document at:

```
GET /.well-known/wia/js
```

Response (`application/json`, RFC 8259):

```json
{
  "registry": "https://registry.wiastandards.com",
  "openapi": "https://registry.wiastandards.com/openapi/v1.json",
  "operationGroups": ["/v1/packages", "/v1/modules", "/v1/sboms",
                      "/v1/manifests", "/v1/runtimes",
                      "/v1/conformance", "/v1/registry"],
  "keySet": "https://registry.wiastandards.com/.well-known/jwks.json",
  "ecmaWindow": ["es2022", "es2023", "es2024", "es2025"]
}
```

The discovery document is signed (RFC 9421 HTTP Message
Signatures) using the registry's signing key from the
declared JWKS.

## §3 Transport

Connections use HTTPS exclusively. TLS configuration
follows RFC 8446 (TLS 1.3) with HSTS (RFC 6797) preload
and a 6-month max-age. Plain HTTP redirects to HTTPS
are permitted only as the boot bootstrap from a freshly
typed URL.

HTTP/2 (RFC 9113) and HTTP/3 (RFC 9114, RFC 9001) are
both supported; clients MUST honour `Alt-Svc` to upgrade.

## §4 Content negotiation

| `Accept`                          | Use                                       |
|-----------------------------------|-------------------------------------------|
| `application/json`                | canonical record bodies                   |
| `application/gzip`                | tarball archives                           |
| `application/zip`                 | zip archives                               |
| `application/wasm`                | bound WebAssembly modules                  |
| `application/javascript`          | ESM / IIFE module bodies                   |
| `application/source-map`          | source-map response bodies                 |
| `application/problem+json`        | error responses                            |
| `application/cyclonedx+json`      | CycloneDX 1.5 SBOM                         |
| `application/spdx+json`           | SPDX 2.3 SBOM                              |

A request without `Accept` defaults to
`application/json` for record endpoints and to the
declared archive content type for archive endpoints.

## §5 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| Package name       | npm convention `@scope/name` or `name`          |
| Version            | Semantic Versioning 2.0.0                       |
| Module specifier   | bare specifier resolved via §6                  |
| `packageRef`       | UUID (RFC 4122)                                  |
| `moduleRef`        | URI                                              |
| Tarball digest     | `sha-512` per RFC 9530                          |
| Source-map URL     | absolute URL or in-archive relative path        |

## §6 Specifier resolution

Resolution proceeds against the runtime profile's
declared algorithm:

1. `node-cjs` — Node CommonJS resolution (`mainFields`).
2. `node-esm` — Node ESM resolution (`exports` with
   conditions per the request's condition list).
3. `import-maps` — W3C Import Maps (browser-native
   bare-specifier remap).
4. `jsr` — JSR resolution (`@scope/name@version`).
5. `deno-nm` / `bun-nm` — runtime-specific resolution.

When two specifiers resolve to artifacts with different
digests but identical names, the registry MUST surface
the divergence; clients MUST refuse to install both.

## §7 Signing and verification

### 7.1 Manifest signing

Manifests are signed with JWS (RFC 7515). The signature
appears as a sidecar (`<artifact>.sig`) carrying a
detached JWS. The `kid` header references the publisher
key in the JWKS.

### 7.2 Tarball signing

Tarballs are pinned by their `sha-512` digest in the
manifest. The digest is the hash over the gzip-decoded
tar bytes per Node `npm pack` semantics.

### 7.3 SBOM signing

SBOMs are signed with detached JWS over the canonical
JSON form (RFC 8785). The signature MUST cover the SBOM
in its entirety; partial signatures are rejected.

### 7.4 Verification order

A client verifies in the order:

1. Discovery document signature (registry trust anchor).
2. Manifest signature (publisher key under the signed
   key set).
3. Tarball digest match (hash check after fetch).
4. SBOM signature (publisher key, same set).

A failure at any step aborts installation.

## §8 Capability negotiation

Clients announce their capability profile in the
`X-WIA-Capability` request header (an opaque
`capabilityRef` from PHASE-1 §6) when fetching package
metadata. The registry uses this to filter the
compatibility response in §8.2 of PHASE-2.

## §9 Caching and integrity

Tarball and SBOM resources are immutable once
published. `Cache-Control: public, max-age=31536000,
immutable` is mandatory. Manifest record responses are
mutable (the manifest may be re-published with a
correction); they carry
`Cache-Control: public, max-age=300,
stale-while-revalidate=60` and a strong `ETag`.

Subresource Integrity (`integrity` HTML attribute) is
the in-browser path. The `Digest` header (RFC 9530) is
the API-level path. Both MUST be served with the same
SHA-512 digest for any given artifact.

## §10 Federation

Federated registries form a directed graph in the
discovery document. A query that cannot be satisfied
locally MAY be forwarded along the federation edge if
both peers declare a shared trust anchor in their
JWKS. Federation responses carry an
`X-WIA-Federation-Path` header listing the registries
that handled the request, terminating at the
authoritative registry.

## §11 Replay and unpublishing

Unpublishing a version is signalled by a tombstone
record visible at the version metadata endpoint with
`unpublishedAt` populated. The tarball is removed from
the cache hierarchy on a 7-day window. Consumers that
have already pinned the digest MAY continue to fetch
from a CI cache; the tombstone signals that no new
fetches will be served.

Replay attacks on signed manifests are prevented by the
manifest's `iat`/`exp` JWS claims (max 24h) plus the
mandatory tombstone reconciliation.

## §12 Error semantics

Errors follow the catalogue in PHASE-2 §11. The
following protocol-level errors are reserved:

| Code  | Meaning                                              |
|-------|------------------------------------------------------|
| 200   | success                                              |
| 304   | conditional GET, body unchanged                      |
| 401   | missing or invalid authentication                    |
| 403   | privately mapped resource                            |
| 404   | unknown package, version, or module                  |
| 409   | publication conflict (digest divergence)             |
| 410   | tombstone — version unpublished                      |
| 422   | manifest validation failure                          |
| 429   | rate limit exceeded                                  |
| 503   | federation peer unavailable                          |

## §13 Observability

Servers SHOULD emit OpenTelemetry traces with the
`http.request.method`, `wia.js.operation`,
`wia.js.package`, and `wia.js.version` attributes.
Trace IDs propagate across federation boundaries via
`traceparent` (W3C Trace Context).

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** the protocol surface is
  served and the discovery document is signed.
- **Tier 2 — Verified:** federation interop tested
  against a reference peer; signing key rotation
  exercised.
- **Tier 3 — Anchored:** continuous evidence stream
  captured per PHASE-4 Annex G.

## Annex B — Replay-resistant publish flow

```text
publisher --[OIDC]--> CI runner
CI runner --[ephemeral key]--> registry: POST /v1/packages
registry --[verify OIDC + key]--> publish accepted
registry --[tombstone bus]--> federated peers
```

## Annex C — Cross-Origin Resource Sharing

The registry serves all read endpoints with
`Access-Control-Allow-Origin: *` and
`Access-Control-Expose-Headers: ETag, Digest, Link,
X-WIA-Federation-Path`. Write endpoints require the
preflight `OPTIONS` exchange and return
`Access-Control-Allow-Methods: POST, PUT, DELETE`.

Browser clients fetching tarballs over CORS MUST set
`crossorigin="anonymous"` on the `<script>` element so
that Subresource Integrity can be enforced.

## Annex D — Connection management

Long-lived connections are encouraged. A registry
SHOULD honour HTTP/2 connection coalescing across
sub-domains served by the same certificate, and HTTP/3
0-RTT for repeat clients within the 24-hour ticket
lifetime. Servers MUST reject 0-RTT requests for write
endpoints to avoid replay.

## Annex E — Discovery document signature

The signature over `/.well-known/wia/js` covers the
JSON body verbatim (no canonicalisation). The signature
is conveyed via the `Signature` and `Signature-Input`
HTTP fields per RFC 9421. The signed components include
`@authority`, `@path`, `content-digest` (RFC 9530), and
`content-type`. Verifiers reject signatures whose
covered components do not include all four.

## Annex F — Compatibility with existing registries

A WIA-js registry MAY also expose the npm Registry API
or the JSR HTTP API as a compatibility surface. The
mapping is informative; the WIA-js endpoints in PHASE-2
are the authoritative interface. A registry that
exposes both surfaces MUST surface a notice in the
discovery document so that clients know which surface
to prefer for signed operations.

## Annex G — Tarball integrity verification timing

Clients MUST verify the `Digest` header (RFC 9530) before
decompressing the archive contents into the install
target. Implementations that decompress in streaming
mode MUST keep the partially extracted state in a
quarantine directory and atomically rename to the
install target only after the digest verification
succeeds. This prevents partial-state attacks where a
truncated download is observable as a complete install.

## Annex H — Server name indication

TLS clients MUST present SNI matching the registry
hostname. Registries operating behind a multi-tenant
TLS terminator MUST refuse connections whose SNI is
absent or mismatched, returning TLS alert
`unrecognized_name`. This protects publishers from
cross-tenant key confusion when a single termination
point fronts multiple registry tenants.

## Annex I — Trust anchor rotation

The signing key set is rotated on a 24-month rolling
schedule. Old keys remain in the JWKS for a 6-month
overlap period after rotation. Clients SHOULD pin the
JWKS URL but NOT individual keys; pinning keys
increases the operational risk of a stranded fleet
when a rotation occurs.

弘益人間 (Hongik Ingan) — Benefit All Humanity
