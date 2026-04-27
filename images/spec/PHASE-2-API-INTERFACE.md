# WIA-images PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-images
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
images programme exposes for the records defined in PHASE-1.
Consumers include editorial CMS systems, image-CDN edge services,
accessibility tooling, rights-clearance operators, provenance
verification services, and citation tools that resolve published
images to their underlying records.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 / 9111 / 9457 / 6901 / 6902 / 8259 / 8288 / 9421
- IETF RFC 7807 (legacy Problem Details kept for compatibility
  with auditor tooling that has not yet migrated to RFC 9457)
- IETF RFC 5646 / BCP 47 (language tags)
- IETF RFC 5789 (PATCH method)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C WCAG 2.2 (text-alternative conventions)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments and Semantic
Versioning 2.0.0. The OpenAPI 3.1 document at `/v1/openapi.json` is
canonical. Bitstream payloads are content-addressed; the API
exposes references to those artefacts rather than streaming binary
payloads through the JSON layer.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-images",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "assets":         "/v1/assets",
    "bitstreams":     "/v1/bitstreams",
    "captures":       "/v1/captures",
    "renditions":     "/v1/renditions",
    "accessibility":  "/v1/accessibility",
    "provenance":     "/v1/provenance",
    "rights":         "/v1/rights",
    "delivery":       "/v1/delivery",
    "evidence":       "/v1/evidence",
    "openapi":        "/v1/openapi.json"
  }
}
```

## §3 Assets

```
POST   /v1/assets                      — register an asset
GET    /v1/assets/{aid}                — retrieve asset record
PATCH  /v1/assets/{aid}/status         — advance asset status
PATCH  /v1/assets/{aid}/withdraw       — record a withdrawal notice
```

Withdrawal notices remain addressable at the asset URL alongside
the canonical record so that downstream caches can detect the
withdrawal during normal refresh.

## §4 Bitstreams and Captures

```
POST   /v1/assets/{aid}/bitstreams     — register a bitstream record
GET    /v1/bitstreams/{bid}            — retrieve bitstream record
GET    /v1/bitstreams/{bid}/artefact   — fetch content-addressed
                                          binary
POST   /v1/bitstreams/{bid}/captures   — register capture metadata
GET    /v1/captures/{cid}              — retrieve capture record
PATCH  /v1/captures/{cid}/geo-redact   — apply GPS redaction
```

GPS redaction is a one-way operation; once applied, subsequent
attempts to reveal coordinates return `404 Not Found` from the API
even with elevated client certificates. The `geoRedacted` audit
flag remains observable.

## §5 Renditions

```
POST   /v1/bitstreams/{bid}/renditions — register a rendition
GET    /v1/renditions/{rid}            — retrieve rendition record
GET    /v1/renditions/{rid}/artefact   — fetch the rendition binary
POST   /v1/bulk/renditions             — batched rendition register
```

Renditions whose `derivationRecipe` does not resolve in the API's
recipe register return `422` with type
`urn:wia:images:recipe-unresolved`. Recipe pinning ensures
reproducibility — re-running the recipe against the same source
bitstream produces a byte-identical rendition.

## §6 Accessibility

```
POST   /v1/assets/{aid}/accessibility       — register text
                                              alternative
GET    /v1/accessibility/{acid}             — retrieve record
PATCH  /v1/accessibility/{acid}/review      — append review outcome
GET    /v1/assets/{aid}/accessibility?locale={l}
                                            — query by locale
```

Auto-generated alt-text MUST be reviewed before the asset
transitions to `ready-for-delivery`; submissions to publish-status
endpoints whose accessibility records are still in `auto-generated`
or `needs-review` state return `409 Conflict` with type
`urn:wia:images:accessibility-review-required`.

## §7 Provenance Attestations

```
POST   /v1/assets/{aid}/provenance      — register a manifest
GET    /v1/provenance/{pid}             — retrieve provenance record
POST   /v1/provenance/{pid}/verify      — re-verify signature chain
GET    /v1/provenance/{pid}/manifest    — fetch the C2PA manifest
```

Verification re-runs the C2PA signature chain against the manifest;
verification failures are recorded as a verification event but do
not strip the manifest from the asset, so that downstream consumers
can decide whether to trust an unverified manifest.

## §8 Rights Clearance

```
POST   /v1/assets/{aid}/rights          — register a clearance
GET    /v1/rights/{rcid}                — retrieve clearance
PATCH  /v1/rights/{rcid}/expiry         — extend or shorten expiry
GET    /v1/assets/{aid}/rights?at={t}   — clearance in force at a
                                          given time
```

Rendition delivery requests for an asset whose rights clearance has
expired or whose `geographyScope` excludes the requesting client
return `403 Forbidden` with type
`urn:wia:images:licence-geography-violation` or
`urn:wia:images:licence-expired`.

## §9 Delivery State

```
POST   /v1/renditions/{rid}/delivery    — record delivery to an
                                          edge
PATCH  /v1/delivery/{did}/invalidate    — append invalidation event
GET    /v1/delivery/{did}               — retrieve delivery record
```

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:images:asset-withdrawn`
- `urn:wia:images:recipe-unresolved`
- `urn:wia:images:accessibility-review-required`
- `urn:wia:images:licence-geography-violation`
- `urn:wia:images:licence-expired`
- `urn:wia:images:provenance-verify-failure`
- `urn:wia:images:evidence-mismatch`

## §11 Authentication

Mutually-authenticated TLS for editorial CMS, CDN edge, rights-
clearance operator, and provenance-verification connections. Public
read-only endpoints (the OpenAPI document, public-domain catalogue
listings) are reachable without a client certificate.

## §12 Caching, ETag, and Conditional Requests

Stable resources (signed renditions, signed evidence packages) are
cacheable with `Cache-Control: max-age=31536000, immutable`.
Mutable resources (draft assets, in-review accessibility entries)
are cacheable for 60 seconds. ETags on every PATCH endpoint with
`If-Match` conditional requests.

## §13 Streaming

Editorial pipelines subscribe to asset events via Server-Sent Events
at `/v1/assets/{aid}/events`. Topics include status transitions,
accessibility review outcomes, rights-clearance changes,
provenance-verification events, and rendition completions.

## §14 Worked Example: From Capture to Citation

1. Photographer's CMS POSTs a new asset and bitstream.
2. EXIF capture metadata is registered; GPS is redacted per the
   operator's privacy policy.
3. Renditions are registered through bulk upload.
4. Accessibility tooling generates auto alt-text; an editor reviews
   and registers the human-reviewed record.
5. Rights-clearance operator registers the licence terms; the
   asset transitions to `ready-for-delivery`.
6. Provenance manifest is registered; verification succeeds.
7. CDN edges record delivery state; citation tool requests an
   evidence package and pins the manifest digest.

## §15 IPTC Metadata Endpoints

```
POST   /v1/bitstreams/{bid}/iptc       — register IPTC metadata
GET    /v1/iptc/{iid}                  — retrieve IPTC record
PATCH  /v1/iptc/{iid}                  — update mutable fields
                                          (creator typo, headline)
```

IPTC submissions whose `creator` or `copyrightNotice` are empty for
editorial-context assets return `422` with type
`urn:wia:images:iptc-required-field-empty`.

## §16 Bulk Operations and Pagination

Bulk endpoints accept arrays for high-volume rendition register,
accessibility submissions, and rights-clearance ingest. Cursor-
based pagination uses the `cursor` query parameter and `Link`
headers (RFC 8288) with cursors persisted for at least 24 hours.

## §17 Privacy-Preserving Aggregation

Aggregate consumers (research collaboratives, publisher comparison
analyses) fetch population-level statistics through endpoints
that emit counts, means, and dispersions:

```
GET    /v1/aggregate/format-mix?period=...
GET    /v1/aggregate/accessibility-coverage?period=...
GET    /v1/aggregate/training-disposition?period=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:images:cohort-too-small`.

## §18 Audit and Observability

Every endpoint emits structured logs with `assetId`, `traceId`,
the issuing client certificate's subject, and the publisher /
CDN edge clock skew vs the reference NTP source.

## §19 Person-Pictured and Print-Production Endpoints

```
POST   /v1/assets/{aid}/person-pictured        — register a
                                                  person-pictured
                                                  record
GET    /v1/person-pictured/{ppid}              — retrieve record
PATCH  /v1/person-pictured/{ppid}/release      — append release
                                                  reference
POST   /v1/renditions/{rid}/print-production   — register print
                                                  production record
GET    /v1/print-production/{prid}             — retrieve record
```

Person-pictured submissions for `releaseKind=no-release-no-
publication` halt publication of the asset; the API returns
`409 Conflict` with type
`urn:wia:images:release-required` when a publish-status PATCH is
attempted on such an asset.

## §20 Streaming Subscription Topics

Consumers subscribe via Server-Sent Events at
`/v1/assets/{aid}/events` for asset-scoped topics, or at
`/v1/operators/{oid}/events` for operator-wide topics. Operator-
wide topics include rights-clearance expiry warnings, take-down
queue depth, and AI-training opt-out registry refresh events.

## §21 Provenance Verification Endpoint

```
POST   /v1/provenance/{pid}/sweep    — request a manifest re-
                                       verification sweep against
                                       the current C2PA trust list
GET    /v1/provenance/{pid}/sweep-history — retrieve sweep history
```

Sweep events run on the operator's declared cadence (typically
weekly); manifests that fall out of trust receive a stale-
manifest event in the streaming subscription. Sweep results are
recorded as an append-only history per manifest.

## §22 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, signs evidence packages per RFC 9421, and enforces the
accessibility-review gate at publish time.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-images
- **Last Updated:** 2026-04-27
