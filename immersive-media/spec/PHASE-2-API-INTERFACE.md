# WIA-immersive-media PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-immersive-media
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
immersive-media programme exposes for the records defined in
PHASE-1. Consumers include HMD platforms, WebXR client runtimes,
mixed-reality anchor services, accessibility tooling, rights-
clearance operators, and publication channels that surface
immersive content to end users.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 / 9111 / 9457 / 6901 / 6902 / 8259 / 8288 / 9421
- IETF RFC 5789 (PATCH method)
- IETF RFC 5646 / BCP 47 (language tags)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- W3C WebXR Device API
- W3C WCAG 2.2
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments. The OpenAPI 3.1
document at `/v1/openapi.json` is canonical. Scene and volumetric
payloads are content-addressed; the API exposes references rather
than streaming binary payloads through the JSON layer.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-immersive-media",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "assets":         "/v1/assets",
    "scenes":         "/v1/scenes",
    "volumetric":     "/v1/volumetric",
    "spatialAudio":   "/v1/spatial-audio",
    "accessibility":  "/v1/accessibility",
    "comfort":        "/v1/comfort",
    "mrAnchors":      "/v1/mr-anchors",
    "provenance":     "/v1/provenance",
    "rights":         "/v1/rights",
    "evidence":       "/v1/evidence",
    "openapi":        "/v1/openapi.json"
  }
}
```

## §3 Assets

```
POST   /v1/assets                       — register an asset
GET    /v1/assets/{aid}                 — retrieve asset record
PATCH  /v1/assets/{aid}/status          — advance asset status
PATCH  /v1/assets/{aid}/withdraw        — record a withdrawal
                                          notice
```

## §4 Scenes and Volumetric

```
POST   /v1/assets/{aid}/scenes          — register a scene record
GET    /v1/scenes/{sid}                 — retrieve scene record
GET    /v1/scenes/{sid}/artefact        — fetch the scene file
POST   /v1/assets/{aid}/volumetric      — register volumetric record
GET    /v1/volumetric/{vid}             — retrieve volumetric record
GET    /v1/volumetric/{vid}/artefact    — fetch the compressed
                                          sequence
```

Submissions whose scene file fails parsing under the cited
encoding return `422` with type
`urn:wia:immersive-media:scene-parse-failure`. Volumetric
submissions whose `pointCount` or `frameRate` exceed the platform
target's certified envelope are accepted with a `requires-platform-
gating` flag in the asset record.

## §5 Spatial Audio

```
POST   /v1/assets/{aid}/spatial-audio   — register spatial-audio
                                          record
GET    /v1/spatial-audio/{aid}          — retrieve record
GET    /v1/spatial-audio/{aid}/artefact — fetch encoded audio
```

## §6 Accessibility and Comfort

```
POST   /v1/assets/{aid}/accessibility   — register accessibility
                                          record
GET    /v1/accessibility/{acid}         — retrieve record
PATCH  /v1/accessibility/{acid}/review  — append review outcome
POST   /v1/assets/{aid}/comfort         — register comfort profile
GET    /v1/comfort/{cpid}               — retrieve comfort profile
PATCH  /v1/comfort/{cpid}/age-rating    — adjust age rating
```

Assets transitioning to `ready-for-delivery` MUST have an
accessibility record in `human-reviewed` or `human-authored` state
and a comfort profile that has been reviewed by an operator-
qualified comfort reviewer; submissions that do not satisfy both
return `409 Conflict` with type
`urn:wia:immersive-media:gating-incomplete`.

## §7 Mixed-Reality Anchors

```
POST   /v1/assets/{aid}/mr-anchors      — register an anchor
GET    /v1/mr-anchors/{maid}            — retrieve anchor
PATCH  /v1/mr-anchors/{maid}/refresh    — request anchor transform
                                          refresh
```

## §8 Provenance and Rights

```
POST   /v1/assets/{aid}/provenance      — register manifest
GET    /v1/provenance/{pid}             — retrieve provenance
                                          record
POST   /v1/assets/{aid}/rights          — register clearance
GET    /v1/rights/{rcid}                — retrieve clearance
```

## §9 Evidence Package

```
POST   /v1/assets/{aid}/evidence        — request package
                                          generation
GET    /v1/evidence/{packageId}         — retrieve a package
GET    /v1/evidence/{packageId}/manifest — manifest only
```

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:immersive-media:scene-parse-failure`
- `urn:wia:immersive-media:gating-incomplete`
- `urn:wia:immersive-media:age-gate-violation`
- `urn:wia:immersive-media:photosensitive-warning-required`
- `urn:wia:immersive-media:rights-scope-violation`
- `urn:wia:immersive-media:evidence-mismatch`

## §11 Authentication

Mutually-authenticated TLS for HMD platform, WebXR runtime, and
rights-clearance operator connections. Public read-only endpoints
(scene-discovery feeds, the OpenAPI document) are reachable
without a client certificate.

## §12 Caching and Streaming

Stable resources (signed scene files, signed evidence packages)
are cacheable with `Cache-Control: max-age=31536000, immutable`.
Mutable resources are cacheable for 60 seconds. Subscribers consume
asset events via Server-Sent Events at
`/v1/assets/{aid}/events`; topics include status transitions,
comfort-profile updates, accessibility-review outcomes, and MR-
anchor refresh notifications.

## §13 Worked Example: From Capture to Citation

1. Volumetric producer POSTs an asset and registers the scene + the
   volumetric sequence.
2. Spatial-audio mix is registered with HRTF personalisation hint.
3. Accessibility tooling generates a long description; an editor
   reviews and registers the human-reviewed record.
4. Comfort profile is registered; the platform gating endpoint
   classifies the asset against age and photosensitive thresholds.
5. Provenance manifest signs the edit chain; rights-clearance
   operator confirms licence terms.
6. Evidence package is generated; manifest digest is pinned in the
   release notes.

## §14 Streaming Variant and Character Endpoints

```
POST   /v1/assets/{aid}/streaming-variants    — register a tier
                                                  variant
GET    /v1/streaming-variants/{svid}          — retrieve variant
GET    /v1/streaming-variants/{svid}/artefact — fetch the tier-
                                                  specific scene
POST   /v1/assets/{aid}/character-variants    — register a character
                                                  variant
GET    /v1/character-variants/{cvid}          — retrieve character
                                                  variant
```

Submissions that lack at least one streaming variant for the
operator's declared minimum supported tier return `409 Conflict`
with type `urn:wia:immersive-media:streaming-variants-incomplete`.

## §15 Bulk and Pagination

Bulk endpoints accept arrays of streaming-variant submissions for
producers that publish across many client tiers simultaneously.
Cursor-based pagination uses the `cursor` query parameter and
`Link` headers (RFC 8288) with cursors persisted at least 24 hours.

## §16 Privacy-Preserving Aggregation

Aggregate consumers (research collaboratives, platform-comparison
analyses) fetch population-level statistics through endpoints
that emit counts, means, and dispersions:

```
GET    /v1/aggregate/comfort-distribution?period=...
GET    /v1/aggregate/asset-class-mix?period=...
GET    /v1/aggregate/photosensitive-incidence?period=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:immersive-media:cohort-too-small`.

## §17 Audit and Observability

Every endpoint emits structured logs with `assetId`, `traceId`,
the issuing client certificate's subject, and the producer / HMD
platform clock skew vs the reference NTP source.

## §18 Telemetry and Provenance-History Endpoints

```
POST   /v1/assets/{aid}/telemetry          — append a telemetry
                                              record
GET    /v1/telemetry/{tid}                 — retrieve telemetry
                                              record
GET    /v1/assets/{aid}/telemetry?from={t} — telemetry window
POST   /v1/provenance/{pid}/sweep          — request manifest re-
                                              verification
GET    /v1/provenance/{pid}/history        — provenance verification
                                              history
```

Telemetry submissions whose `intervalDurationS` falls below the
operator's minimum aggregation window return `422` with type
`urn:wia:immersive-media:telemetry-window-too-narrow`. The minimum
window protects user privacy by preventing inference of individual
session boundaries from high-frequency telemetry.

## §19 MR-Anchor Refresh Streaming

MR-anchor records (PHASE-1 §8) are bound to physical locations and
their resolved transforms drift over time as platform-side
localisation services update. Subscribers consume anchor refresh
events via Server-Sent Events at
`/v1/mr-anchors/{maid}/events`; subscribers that hold the anchor's
current transform receive notifications when the platform service
publishes an updated transform.

Subscribers that disconnect may resume from the last seen event
identifier via the `Last-Event-ID` header (W3C EventSource
semantics).

## §20 Withdrawal and Supersession Endpoints

```
PATCH  /v1/assets/{aid}/withdraw         — withdraw the asset
                                            with a public notice
                                            reference
POST   /v1/assets/{aid}/successors       — register a successor
                                            asset reference
GET    /v1/assets/{aid}/withdrawal-history
                                          — retrieve the withdrawal
                                            and supersession chain
```

Withdrawn or superseded assets remain addressable at their
content-addresses; subsequent retrievals receive the public notice
alongside the canonical record so that downstream caches and
citation tools see the withdrawal context.

## §21 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, signs evidence packages per RFC 9421, and enforces the
gating rule that prevents publication of assets without reviewed
accessibility and comfort records.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-immersive-media
- **Last Updated:** 2026-04-27
