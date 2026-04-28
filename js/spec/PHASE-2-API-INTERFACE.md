# WIA-js PHASE 2 — API Interface Specification

**Standard:** WIA-js
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-js
participants expose so that registries, conformance
auditors, build pipelines, and consumer applications
can resolve packages, fetch modules, retrieve SBOMs,
verify signatures, and report conformance evidence
without bespoke integration per provider.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 7515 (JWS), RFC 7516 (JWE), RFC 7519 (JWT)
- IETF RFC 7234, RFC 5861 (HTTP cache)
- IETF RFC 9530 (Digest Fields)
- WHATWG Fetch Living Standard
- W3C WebIDL, W3C Subresource Integrity
- npm Registry API (public surface)
- JSR HTTP API (public surface)
- ECMA-262 Module Loader pseudo-algorithms (informative)

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces between
package publishers, package registries, package
consumers (build tools, runtimes), conformance auditors,
and SBOM stores. It does not specify the in-process
JavaScript Module Loader API; that is governed by
ECMA-262 §16 and the host runtime.

## §2 Operation groups

The API is partitioned into seven operation groups,
each with a stable URL prefix.

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/packages`      | package query and tarball/zip retrieval          |
| `/v1/modules`       | module-graph resolution, loader simulation       |
| `/v1/manifests`     | package.json / deno.json / jsr.json validation   |
| `/v1/sboms`         | SBOM upload, retrieval, and search               |
| `/v1/runtimes`      | runtime profile registry                         |
| `/v1/conformance`   | conformance evidence intake and lookup           |
| `/v1/registry`      | registry directory + identity                    |

Each group is documented as a separate OpenAPI 3.1
document published under `/.well-known/openapi/<group>`.

## §3 Authentication and authorisation

Read endpoints are publicly accessible by default.
Write endpoints require either:

- a JWT bearer (RFC 7519) issued by the registry, with
  `aud` matching the registry's identity URL and `scope`
  enumerating the permitted operation groups; or
- an ephemeral signing identity bound to a workload-
  identity OIDC token (publish-from-CI flow).

Tokens MUST NOT be embedded in URL query strings.

## §4 Package operations

### 4.1 List package versions

```
GET /v1/packages/{name}/versions
```

Returns versions matching the optional Browserslist /
semver-range query parameters. Pagination is mandatory
once the version count exceeds 100; the cursor is
opaque and stable across pages.

### 4.2 Fetch package metadata

```
GET /v1/packages/{name}/{version}
```

Returns the canonical package record (PHASE-1 §2). The
record carries an `ETag` derived from the SHA-512 of the
canonical JSON form (RFC 8785). Conditional GET MUST be
honoured.

### 4.3 Fetch tarball

```
GET /v1/packages/{name}/{version}/archive
```

Returns the registered archive in `application/gzip` or
`application/zip` per the `Accept` header. The response
carries `Digest: sha-512=<base64>` (RFC 9530); the
client MUST verify the digest before installation.

### 4.4 Publish package

```
POST /v1/packages/{name}/{version}
```

Accepts `multipart/form-data` with the manifest, the
archive, and an attached SBOM. The registry MUST reject
publication when:

- the manifest version already exists with a different
  archive digest;
- the SBOM's component graph cannot be parsed;
- the manifest declares an ECMA-262 edition outside the
  PHASE-1 Annex B four-year window.

## §5 Module operations

### 5.1 Resolve module graph

```
POST /v1/modules/resolve
```

Body: an entry-specifier list and a runtime profile
reference. Response: the resolved module graph (PHASE-1
§3) with cycles flagged. Resolution is deterministic
when the input manifest set is fixed.

### 5.2 Loader simulation

```
POST /v1/modules/simulate-load
```

Body: an entry specifier, condition list, and import-map
URI. Response: the ordered evaluation list as a Module
Loader would produce, including TLA boundary positions
and dynamic-import insertion points.

### 5.3 Source-map lookup

```
GET /v1/modules/{moduleRef}/sourcemap
```

Returns the source-map (Source Map v3) for the module.
Private maps return `403` with a `WWW-Authenticate`
challenge.

## §6 Manifest operations

### 6.1 Validate manifest

```
POST /v1/manifests/validate
```

Body: a candidate `package.json`, `deno.json`, or
`jsr.json`. Response: a Problem Details (RFC 9457)
report listing JSON-Schema violations and any cross-
field constraint failures (e.g. `exports` referencing a
path that is not in the archive).

### 6.2 Diff manifests

```
POST /v1/manifests/diff
```

Body: two manifest documents. Response: a structured
diff classifying changes into `breaking`, `additive`,
`editorial` per Semantic Versioning 2.0.0. The classifier
is open-source and reproducible from the URL in the
`X-WIA-Classifier-Source` response header.

## §7 SBOM operations

### 7.1 Upload SBOM

```
POST /v1/sboms
```

Body: CycloneDX 1.5 JSON or SPDX 2.3 JSON with a JWS
signature in the `signature` field. The registry MUST
verify the signature against the publisher's registered
key set (`/v1/registry/keys/{publisher}`) before
accepting.

### 7.2 Lookup by package

```
GET /v1/sboms?package={name}&version={version}
```

Returns the canonical SBOM record (PHASE-1 §7).

### 7.3 Vulnerability join

```
GET /v1/sboms/{sbomRef}/vulnerabilities
```

Returns the OSV.dev advisories that match any component
in the SBOM, with the date of the advisory cross-walk.
The response is informative; the registry does not gate
download on advisory presence.

## §8 Runtime profile operations

### 8.1 Register profile

```
POST /v1/runtimes
```

Body: a runtime profile record (PHASE-1 §4). The
registry assigns a `profileRef` and returns it.
Profiles already registered with identical content are
de-duplicated to the existing `profileRef`.

### 8.2 Resolve compatibility

```
POST /v1/runtimes/compat
```

Body: a package name and runtime profile reference.
Response: a compatibility verdict (`compatible`,
`partial`, `incompatible`) with a list of unmet
requirements (e.g. ECMA-402 tier mismatch, missing Web
API, incompatible WASI version).

## §9 Conformance operations

### 9.1 Submit evidence

```
POST /v1/conformance/{packageRef}/evidence
```

Body: an evidence package per PHASE-4 §5 (SBOM + test-
vector matrix + signed manifest). Response: a Problem
Details document listing every requirement that the
evidence does not satisfy.

### 9.2 Lookup conformance

```
GET /v1/conformance/{packageRef}
```

Returns the most recent conformance attestation, the
auditor identity, the audit window, and the evidence
URL.

## §10 Registry directory

```
GET /v1/registry
```

Returns the registry's identity (URI), supported
operation groups, OpenAPI URL, and signing key set.

## §11 Error semantics

All error responses use `application/problem+json` per
RFC 9457 with `type` URLs in the
`https://wiastandards.com/errors/js/` namespace. The
`detail` field is human-readable English; the `instance`
field is a UUID identifying the request for support
correlation.

## §12 Caching, conditional requests, and rate limits

`ETag` and `Last-Modified` are mandatory on package and
SBOM read endpoints. `Cache-Control` defaults to
`public, max-age=300, stale-while-revalidate=60`.
Rate-limit headers (`X-RateLimit-Limit`,
`X-RateLimit-Remaining`, `X-RateLimit-Reset`) follow the
draft-ietf-httpapi-ratelimit-headers convention.

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info:
  title: WIA-js Registry API
  version: 1.0.0
servers:
  - url: https://registry.wiastandards.com
paths:
  /v1/packages/{name}/{version}:
    get:
      summary: Fetch package metadata
      parameters:
        - {name: name, in: path, required: true, schema: {type: string}}
        - {name: version, in: path, required: true, schema: {type: string}}
      responses:
        '200':
          description: Canonical package record
          content:
            application/json:
              schema: {$ref: 'PackageRecord.schema.json'}
```

## Annex B — Idempotency

Mutating operations accept an `Idempotency-Key` request
header. The registry persists the result for 24h so
that retried requests do not produce duplicate entries.

## Annex C — Subresource Integrity

Tarball responses MAY carry an `Integrity` header
exposing the SRI digest for direct in-browser
verification per W3C Subresource Integrity.

## Annex D — Federation

Federated registries expose a common `/v1/registry`
document carrying the federation graph (peer URLs +
trust anchor key sets). Cross-registry queries follow
the federation graph and SHOULD include
`X-WIA-Federation-Trace` for observability.

## Annex E — Pagination

List endpoints carry a stable opaque cursor in the
`next` field of the response body and in the `Link`
header (`rel=next`). Cursors are valid for at least 24h
after issuance; an expired cursor returns 410 with a
Problem Details `pagination/cursor-expired` type. The
default page size is 100; the maximum is 1000. Clients
MUST NOT increment offsets manually.

## Annex F — Webhook subscriptions

Publishers may subscribe to registry events via:

```
POST /v1/registry/webhooks
```

Body: a webhook record with `url`, `secret`, and event
filter (`package.published`, `package.unpublished`,
`sbom.vulnerability`, `conformance.tier-changed`).

Webhook deliveries are signed with HMAC-SHA-256 over
the canonical body using the subscription secret. The
signature appears in the `X-WIA-Signature` header as
`sha256=<hex>`; consumers MUST verify before acting.

Retry policy: at-least-once with exponential backoff
(2, 4, 8, 16, 32, 64, 128, 256, 512 seconds), giving up
after 9 attempts. Failed deliveries are recorded in
`/v1/registry/webhooks/{id}/deliveries`.

## Annex G — Public introspection

```
GET /v1/registry/stats
```

Returns aggregate counters (package count, SBOM count,
vulnerabilities surfaced, federation peers). Endpoints
under `/stats` are public, cached for 5 minutes, and
SHOULD NOT be used for billing or compliance reporting
because the counters are eventually consistent.

## Annex H — Bulk export

Auditors performing large-scale reviews may request a
bulk export:

```
POST /v1/registry/export
```

Body: a filter over packages (publisher, ECMA edition,
runtime profile, conformance tier). Response: a
short-lived signed URL pointing to a `tar.zst`
containing one folder per matching package
(`metadata.json`, `sbom.json`, `evidence.json`,
`testmatrix.json`). The URL expires within 1 hour;
re-issue requires a new request with the same
idempotency key.

Bulk exports do not include archive bytes; auditors
fetch archives separately via §4.3 to keep the export
manageable.

## Annex I — Quotas

Per-publisher publish quotas default to 100 versions /
hour and 1000 versions / day. The registry surfaces the
remaining quota in `X-Quota-Remaining` on every write
response. Publishers exceeding quota receive a
`429 Too Many Requests` with a `Retry-After` value.

## Annex J — Read replica labels

Registries that operate read replicas surface the
serving replica in the `X-WIA-Replica` response header.
The label is informative and intended for end-to-end
debugging; clients MUST NOT route based on it.

A replica that lags behind primary by more than 60s
returns 503 with a Problem Details `replica/stale`
type so that conformance evidence cannot be served
from a stale view.

## Annex K — Sandbox endpoints

Test deployments expose `/v1/sandbox` mirroring the
`/v1` surface but with synthetic packages and ephemeral
state. Sandbox responses carry `X-WIA-Sandbox: true`.
Sandbox state is cleared on a 24h rolling window; do
not store credentials, secrets, or production data in
sandbox endpoints.

弘益人間 (Hongik Ingan) — Benefit All Humanity
