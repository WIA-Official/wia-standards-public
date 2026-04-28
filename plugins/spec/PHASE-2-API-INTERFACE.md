# WIA-plugins PHASE 2 — API Interface Specification

**Standard:** WIA-plugins
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-plugins
participants expose so that publishers, plugin
registries, hosts, marketplaces, and audit
authorities can publish, search, install, update,
and audit plugins through a single contract.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9457 (Problem Details)
- IETF RFC 7515 (JWS), RFC 7519 (JWT), RFC 9530 (Digest Fields)
- IETF RFC 9421 (HTTP Message Signatures), RFC 7234 (HTTP cache)
- CycloneDX 1.5 / SPDX 2.3
- Semantic Versioning 2.0.0
- Web Bundle / WebExtensions Manifest (informative)
- WebAssembly Component Model

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces
between plugin publishers, plugin registries,
hosts, marketplaces, and audit authorities. It
does not specify the in-host plugin lifecycle API;
that is governed by the host's own documentation.

## §2 Operation groups

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/plugins`       | plugin record registry                           |
| `/v1/manifests`     | manifest validation and lookup                   |
| `/v1/releases`      | release record store                             |
| `/v1/sboms`         | SBOM upload and lookup                           |
| `/v1/capabilities`  | capability declaration registry                  |
| `/v1/hosts`         | host catalogue                                   |
| `/v1/locales`       | locale-bundle store                              |
| `/v1/registry`      | registry directory                               |

## §3 Authentication

Read endpoints are publicly accessible. Write
endpoints require a JWT bound to the publisher
identity. Publish-from-CI flows accept ephemeral
signing identities bound to a workload-identity
OIDC token.

## §4 Plugin operations

### 4.1 Register plugin

```
POST /v1/plugins
```

Body: plugin record (PHASE-1 §2). Response:
`pluginRef`.

### 4.2 Search

```
GET /v1/plugins?q=<text>&host=<hostRef>&category=<cat>
```

Returns matching plugins; pagination mandatory
beyond 100 results.

### 4.3 Lookup

```
GET /v1/plugins/{pluginRef}
```

Returns the canonical plugin record with
conditional GET via `ETag`.

## §5 Manifest operations

### 5.1 Validate

```
POST /v1/manifests/validate
```

Body: a candidate manifest. Response: a Problem
Details (RFC 9457) report listing JSON-Schema
violations and any cross-field constraint failures.

### 5.2 Diff

```
POST /v1/manifests/diff
```

Body: two manifest documents. Response: a
structured diff classifying changes into
`breaking`, `additive`, `editorial` per Semantic
Versioning 2.0.0.

## §6 Release operations

### 6.1 Publish release

```
POST /v1/releases
```

Body: release record (PHASE-1 §6) plus the
attached archive and SBOM in a multipart form. The
registry MUST verify:

- the archive digest matches the declared digest;
- the SBOM is parseable;
- the manifest version follows semver bumps from
  prior releases.

### 6.2 Lookup

```
GET /v1/releases/{releaseRef}
```

Returns the canonical release record.

### 6.3 Yank

```
PUT /v1/releases/{releaseRef}/yank
```

Body: justification. Yanked releases remain
queryable but are excluded from default install
flows; new installs must explicitly accept yanked
releases.

## §7 SBOM operations

### 7.1 Upload SBOM

```
POST /v1/sboms
```

Body: CycloneDX 1.5 or SPDX 2.3 with attached JWS
signature.

### 7.2 Vulnerability join

```
GET /v1/sboms/{sbomRef}/vulnerabilities
```

Returns OSV.dev advisories that match any
component in the SBOM.

## §8 Capability operations

### 8.1 Register

```
POST /v1/capabilities
```

Body: capability declaration record (PHASE-1 §4).

### 8.2 Lookup

```
GET /v1/capabilities/{capabilityRef}
```

Returns the declaration. Hosts MAY refuse
installation when a declaration exceeds the host's
permission model.

## §9 Host operations

### 9.1 Register host

```
POST /v1/hosts
```

Body: host descriptor with API surface URL and
runtime-profile reference.

### 9.2 Compatibility check

```
POST /v1/hosts/{hostRef}/compat
```

Body: a manifest reference. Response: a
compatibility verdict (`compatible`, `partial`,
`incompatible`) with unmet requirements.

## §10 Locale operations

### 10.1 Upload bundle

```
POST /v1/locales
```

Body: locale-bundle record (PHASE-1 §8) with
messages JSON.

### 10.2 Lookup

```
GET /v1/plugins/{pluginRef}/locales/{lang}
```

Returns the bundle for the language; falls back to
BCP 47 lookup.

## §11 Error semantics

Errors are `application/problem+json` (RFC 9457)
namespaced under
`https://wiastandards.com/errors/plugins/`.

## §12 Caching and rate limits

Release archives carry `Cache-Control: public,
max-age=31536000, immutable`. Manifest records
carry strong `ETag` with short max-age. Rate limit
headers follow the draft-ietf-httpapi-ratelimit-
headers convention.

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info: {title: WIA-plugins API, version: 1.0.0}
paths:
  /v1/plugins:
    post:
      summary: Register a plugin
      requestBody:
        required: true
        content:
          application/json:
            schema: {$ref: 'PluginRecord.schema.json'}
      responses:
        '201': {description: Plugin registered}
```

## Annex B — Idempotency

Mutating operations honour `Idempotency-Key` for
24h.

## Annex C — Webhook subscriptions

Subscribers receive events on `plugin.registered`,
`release.published`, `release.yanked`,
`vulnerability.surfaced`. Delivery is signed with
HMAC-SHA-256.

## Annex D — Federation

Federation between sister registries follows the
discovery contract in PHASE-3. Cross-registry
queries follow `X-WIA-Federation-Path`.

## Annex E — Bulk export

`POST /v1/registry/export` returns a signed URL to
a `tar.zst` of the deployment's records filtered
by host and date range.

## Annex F — Sandbox

`/v1/sandbox` mirrors production with synthetic
plugins.

## Annex G — Quotas

Per-publisher quotas: 100 releases / hour;
1000 / day.

## Annex H — Audit feed

`GET /v1/registry/audit?since=<timestamp>`.

## Annex I — Public introspection

`GET /v1/registry/stats` returns aggregate
counters.

## Annex J — Webhook payload shape

```json
{
  "event": "release.published",
  "releaseRef": "https://reg.example.org/releases/foo/1.4.0",
  "publishedAt": "2026-04-28T11:32:00+09:00"
}
```

## Annex K — Bulk SBOM export

```
POST /v1/sboms/export
```

Body: filter by publisher and date range. Response:
a signed URL to a `tar.zst` of SBOMs.

## Annex L — Vendor catalogue

```
GET /v1/registry/publishers/{publisherRef}
```

Returns the publisher's catalogue with the
attestation chain (domain ownership, identity
verification).

## Annex M — Plugin install endpoints

```
POST /v1/plugins/{pluginRef}/install-token
```

Body: requesting host identity and version range.
Response: a short-lived signed install token plus
the canonical archive URL. The install token
contains the SHA-512 of the archive so that the
host can verify integrity before extraction.

## Annex N — Plugin uninstall reporting

```
POST /v1/plugins/{pluginRef}/uninstall-report
```

Body: anonymised reason categories (e.g.
`functional-replacement`, `performance`,
`compatibility`, `unwanted`). Reports feed into
the registry's anonymised usage analytics.

## Annex O — Telemetry consent

```
POST /v1/plugins/{pluginRef}/telemetry-consent
```

Body: user consent state (`granted`, `denied`,
`withdrawn`). The host stores the consent locally
and reports the aggregate consent ratio to the
registry without identifying users.

## Annex P — Locale negotiation

```
GET /v1/plugins/{pluginRef}/locales
```

Returns the locale catalogue available for the
plugin. The host uses this list when surfacing the
plugin's UI strings to the user.

## Annex Q — Compatibility report endpoint

```
GET /v1/registry/compatibility?host={hostRef}&hostVersion={ver}
```

Returns a per-plugin compatibility verdict for the
host version. Used by hosts to power upgrade-check
flows.

## Annex R — Reverse dependency graph

```
GET /v1/plugins/{pluginRef}/reverse-deps
```

Returns plugins that depend on the given plugin.
Used by maintainers planning a major bump to
estimate the downstream impact.

## Annex S — Plugin marketplace mirroring

```
POST /v1/registry/mirror
```

Body: target marketplace identity and a SHACL
filter selecting plugins to mirror. Response: a
mirror plan with cadence and signature
preservation.

## Annex T — SBOM bulk import

```
POST /v1/sboms/bulk
Content-Type: application/jsonl
```

Body: a JSON-Lines stream of CycloneDX or SPDX
documents. Response: a per-line verdict.

## Annex U — Capability survey

```
GET /v1/registry/capabilities/survey
```

Returns aggregate counters of which capabilities
are declared most frequently across the plugin
catalogue. Used by host maintainers planning
permission-model changes.

## Annex V — Plugin moderation

```
POST /v1/plugins/{pluginRef}/moderation
```

Body: moderation action (`flag`, `quarantine`,
`unflag`) and justification. Moderation actions
are signed by the registry operator and visible to
publishers via the audit feed.

## Annex W — Webhook payload signature

Webhook payloads are canonicalised per RFC 8785
before HMAC-SHA-256 signing. The signature is
delivered in `X-WIA-Signature: sha256=<hex>`.

## Annex X — Audit-grade publisher report

```
POST /v1/registry/publisher-report
```

Body: publisher reference and date range. Response:
a tar.zst archive of the publisher's catalogue,
release records, SBOM signatures, and audit feed
entries.

## Annex Y — Plugin lifecycle webhook

Subscribers receive `lifecycle.installed`,
`lifecycle.activated`, `lifecycle.deactivated`,
`lifecycle.updated`, `lifecycle.uninstalled`
events from hosts that opt into reporting.
Reporting is opt-in per host and respects user
telemetry preferences.

## Annex Z — Capability negotiation API

```
POST /v1/capabilities/negotiate
```

Body: a host's permission profile and a plugin's
capability declaration. Response: the negotiated
intersection plus a list of capabilities that the
host refused.

## Annex AA — Plugin catalogue Atom feed

```
GET /v1/plugins/feed
```

Returns a paginated Atom 1.0 feed (RFC 5005) of
plugin events: registrations, updates, yanks. The
feed exposes a `<link rel="hub">` for WebSub
subscribers.

## Annex AB — Per-host install statistics

```
GET /v1/registry/stats?host={hostRef}
```

Returns aggregate counters per host: total
plugins, daily active installs, average update
lag. Statistics are eventually consistent.

## Annex AC — Cross-host equivalence

```
GET /v1/plugins/{pluginRef}/equivalents
```

Returns plugins that the publisher has declared as
equivalent across hosts (e.g. the VS Code and
JetBrains versions of the same plugin). Used by
hosts to prompt users about cross-host options.

## Annex AD — Reviewer queue endpoint

```
GET /v1/registry/moderation/queue
```

Returns the moderation queue for the registry
operator. Used by moderators reviewing flagged
plugins. Access requires the
`moderation:read` JWT scope.

## Annex AE — Quotas

Per-publisher publish quotas: 100 releases / hour;
1000 / day; 10 yanks / hour. The registry surfaces
remaining quota in `X-Quota-Remaining` on every
write response.

弘益人間 (Hongik Ingan) — Benefit All Humanity
