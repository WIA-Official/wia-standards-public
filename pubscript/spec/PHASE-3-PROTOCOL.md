# WIA-pubscript PHASE 3 — Protocol Specification

**Standard:** WIA-pubscript
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocols used
by WIA-pubscript participants for discovery,
multi-modality artefact retrieval, signed
publication, narration synchronisation,
accessibility evidence transport, and federation.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 6797 (HSTS)
- IETF RFC 8615 (Well-Known URIs), RFC 7517 (JWK), RFC 7515 (JWS)
- IETF RFC 9421 (HTTP Message Signatures), RFC 9530 (Digest Fields)
- IETF RFC 8259 (JSON), RFC 8785 (JCS)
- W3C SMIL 3.0, EPUB Media Overlays 3.3
- W3C TTML2, WebVTT 1.0, SSML 1.1, PLS 1.0
- W3C Subresource Integrity, W3C Content Security Policy 3
- WHATWG Fetch (CORS, redirect)

---

## §1 Scope

This PHASE defines the on-the-wire behaviour
between publishers, distribution platforms,
reading-app vendors, and accessibility auditors.

## §2 Discovery

A WIA-pubscript registry serves a discovery
document at:

```
GET /.well-known/wia/pubscript
```

Response (`application/json`):

```json
{
  "registry": "https://pubscript.wiastandards.com",
  "openapi": "https://pubscript.wiastandards.com/openapi.json",
  "operationGroups": ["/v1/publications",
                      "/v1/representations",
                      "/v1/manifests",
                      "/v1/narrations",
                      "/v1/accessibility",
                      "/v1/contributors",
                      "/v1/registry"],
  "supportedFormats": {
    "visual": ["epub-3.3", "pdf-ua-1", "html5", "web-publication"],
    "auditory": ["audiobook-w3c", "mp3", "aac", "flac", "opus"],
    "tactile": ["brf", "ueb", "ks-x-1026", "tactile-graphic-png"],
    "spatial": ["gltf-2.0", "usdz", "openxr-scene"],
    "gestural": ["sign-video-mp4", "signwriting", "bvh"]
  },
  "keySet": "https://pubscript.wiastandards.com/.well-known/jwks.json"
}
```

The discovery document is signed (RFC 9421).

## §3 Transport

HTTPS with TLS 1.3 and HSTS preload. Large
artefacts (audiobook, sign-language video) use
HTTP/2 or HTTP/3 streaming.

## §4 Content negotiation

| Accept                                | Use                                      |
|---------------------------------------|------------------------------------------|
| `application/json`                    | record bodies                            |
| `application/epub+zip`                | EPUB 3.3 archives                        |
| `application/pdf`                     | PDF/UA-1                                 |
| `audio/mpeg`, `audio/aac`,            | audio artefacts                          |
| `audio/flac`, `audio/opus`            |                                          |
| `application/x-brf`                   | Braille Ready Format                     |
| `model/gltf+json`,                    | spatial scene graphs                     |
| `model/vnd.usdz+zip`                  |                                          |
| `video/mp4`                           | sign-language video                      |
| `application/smil+xml`                | SMIL 3.0 / Media Overlays                |
| `application/ssml+xml`                | SSML 1.1                                 |
| `text/vtt`                            | WebVTT cues                              |
| `application/problem+json`            | error                                    |

## §5 Signed publication

Publication, representation, manifest,
accessibility, and narration records are signed
with detached JWS (RFC 7515) over the canonical
JSON form (RFC 8785). The `kid` references the
publisher's key in the registry's JWKS.

## §6 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| `publicationRef`   | UUID (RFC 4122) opaque                          |
| ISBN-13            | ISO 2108                                        |
| DOI                | per DOI Foundation                              |
| `representationRef`| URI                                             |
| `manifestRef`      | URI                                             |
| `narrationRef`     | UUID                                            |
| `accessibilityRef` | URI                                             |
| `contributorRef`   | UUID                                            |
| `digestRef`        | `sha-512` per RFC 9530                          |

## §7 Caching and immutability

Representation artefacts are immutable; they
carry `Cache-Control: public, max-age=31536000,
immutable`. Publication and manifest records are
mutable through versioning; they carry strong
`ETag`.

## §8 Federation

Federated registries form a directed graph in the
discovery document. Cross-registry queries follow
the graph and carry an `X-WIA-Federation-Path`
header.

## §9 Replay protection

Signed publications carry `iat`/`exp` JWS claims
(max 24h). Accessibility evidence submissions are
signed by the auditor with the auditor's
ISO/IEC 17065-bound key.

## §10 Error semantics

Errors are `application/problem+json` (RFC 9457).
Protocol-level codes:

| Code | Meaning                                              |
|------|------------------------------------------------------|
| 200  | success                                              |
| 304  | conditional GET unchanged                            |
| 400  | malformed JSON / EPUB / PDF / Braille                |
| 401  | missing or invalid token                             |
| 403  | publisher not authorised for the modality            |
| 410  | tombstone (withdrawn publication)                    |
| 422  | accessibility-declaration violation                  |
| 426  | TLS upgrade required                                 |
| 503  | federation peer unavailable                          |

## §11 Observability

Servers SHOULD emit OpenTelemetry traces with
`wia.pubscript.operation`,
`wia.pubscript.publicationRef`,
`wia.pubscript.modality`, and
`wia.pubscript.publisher` attributes.

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** discovery served,
  publications signed.
- **Tier 2 — Verified:** EPUB Accessibility 1.1
  conformance audited; equivalence claim
  verified by external auditor.
- **Tier 3 — Anchored:** continuous evidence
  stream per PHASE-4 Annex G; sovereign
  legal-deposit registration.

## Annex B — Discovery document signature

The signature over `/.well-known/wia/pubscript`
covers `@authority`, `@path`, `content-digest`
(RFC 9530), and `content-type`.

## Annex C — Cross-Origin Resource Sharing

Read endpoints serve `Access-Control-Allow-Origin:
*` with `ETag`, `Digest`, and `Link` exposed.
Reading-app browser clients fetching artefacts
over CORS MUST set `crossorigin="anonymous"` so
that Subresource Integrity can be enforced.

## Annex D — Trust anchor rotation

Publisher signing keys rotate per the publisher's
policy. Recommended cadence is 24 months for
high-volume publishers.

## Annex E — Federation hop cap

Federated lookups carry an
`X-WIA-Federation-Hops` header; queries with
`Hops > 3` are dropped.

## Annex F — TLS profile baseline

TLS 1.3 with PFS-only cipher suites; NIST SP
800-52 Rev. 2 baseline.

## Annex G — Streaming audiobook delivery

Audiobook chapters are delivered as chunked HTTP/2
or HTTP/3 streams. The reader app verifies the
chapter digest after stream completion. SMIL Media
Overlays accompany the stream so that text-audio
synchronisation remains aligned through chapter
boundaries.

## Annex H — Sign-language video delivery

Sign-language video is delivered as HLS or DASH
adaptive streaming. The reader app honours the
publisher's `signLanguage` ISO 639-3 code so that
the correct sign language is selected for the
viewer's locale.

## Annex I — Replay-resistant accessibility
##           evidence

Accessibility evidence submissions carry
`iat`/`exp` claims (max 24h) and the auditor's
signature so that a captured submission cannot
be replayed against a different publication.

## Annex J — Sandbox endpoints

`/v1/sandbox` mirrors production with synthetic
publications and ephemeral state.

## Annex K — Adaptive bitrate streaming

Audiobook and sign-language video deliveries
support HLS (RFC 8216) and DASH (ISO/IEC 23009)
adaptive bitrate streaming. The reader app
selects the rendition matching its bandwidth and
device class.

## Annex L — DRM neutrality

The standard does not mandate or forbid DRM. DRM
layers are applied by distribution platforms
without altering the publisher's signatures; the
auditor verifies signatures on the unwrapped
artefact.

## Annex M — Connection coalescing

HTTP/2 clients MAY coalesce connections across
sub-domains served by the same certificate. The
registry publishes the coalescing policy in the
discovery document.

## Annex N — Compression preferences

Read endpoints declare
`Accept-Encoding: br, zstd, gzip`. Audiobook,
video, and Braille payloads are pre-compressed
in their canonical formats and are NOT recompressed
at the transport layer.

## Annex O — Webhook delivery

Webhook deliveries follow at-least-once semantics
with exponential backoff capped at 9 attempts.
Failed deliveries enter a dead-letter queue.

## Annex P — JSON canonicalisation

JSON-bearing publication, manifest, and
accessibility records are canonicalised per
RFC 8785 prior to JWS signature.

## Annex Q — Pagination convention

List endpoints carry a stable opaque cursor in
the `next` field of the response body and in
the `Link` header (`rel=next`). Cursors are
valid for at least 24h after issuance.

## Annex R — Hardware-backed publisher key

Publisher signing keys MAY be hardware-bound
(HSM, FIDO2 with attestation, smart card). Major
publishers use hardware-bound keys for high-
volume releases.

## Annex S — Discovery cache TTL

The discovery document carries
`Cache-Control: public, max-age=300,
stale-while-revalidate=60`.

## Annex T — Trust matrix

The discovery document carries a peer trust
matrix listing each peer's accepted operation
groups and JWKS URL. Trust changes surface as
audit events.

## Annex U — TLS PSK resumption

TLS 1.3 session resumption via PSK is permitted
for read endpoints. Write endpoints require full
handshake to prevent privilege replay.

## Annex V — Connection rate limits

Per-deployment download quotas default to 10 GB
/ hour for unauthenticated users; authenticated
users are bounded by the publisher's quota.
Quotas surface in `X-Quota-Remaining`.

## Annex W — Background-job semantics

Long-running jobs (bulk legal-deposit, manifest
audit, equivalence diff across large catalogues)
return 202 Accepted with a polling URL and a
webhook subscription option.

## Annex X — Subresource Integrity for reader
##           apps

Browser-based reader apps loading scripts and
stylesheets reference Subresource Integrity
hashes in the EPUB Package Document. Reader
apps verify SRI before executing scripts.

## Annex Y — Federation peer assertion

Federation peers exchange peer assertions signed
under their JWKS. Trust is bilateral and
revocable; revoked peers are quarantined for 30
days then purged from the discovery graph.

## Annex Z — Replay-resistant deposit

Sovereign legal-deposit submissions carry
`iat`/`exp` claims (max 24h). The deposit
authority verifies the claim before mirroring
into the deposit collection.

## Annex AA — Per-modality digest verification

Reader apps verify the modality artefact digest
before opening it. Unverified artefacts are
quarantined; the user is informed via a status
indicator that signature verification failed.

弘益人間 (Hongik Ingan) — Benefit All Humanity
