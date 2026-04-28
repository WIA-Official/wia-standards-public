# WIA-pubscript PHASE 2 — API Interface Specification

**Standard:** WIA-pubscript
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surfaces that WIA-
pubscript participants expose so that publishers,
reading-app vendors, accessibility auditors,
sovereign-equivalent legal-deposit registries, and
distribution platforms can publish multi-sensory
publications, retrieve per-modality
representations, declare accessibility, and
reconcile cross-format manifests through a single
contract.

References (CITATION-POLICY ALLOW only):
- OpenAPI Specification 3.1
- IETF RFC 9110 (HTTP Semantics), RFC 9457 (Problem Details)
- IETF RFC 7515 (JWS), RFC 7519 (JWT), RFC 9421 (HTTP Message Signatures)
- IETF RFC 9530 (Digest Fields), RFC 8259 (JSON), RFC 8785 (JCS)
- W3C EPUB 3.3, EPUB Accessibility 1.1
- W3C Web Publications, W3C Audiobooks Manifest
- W3C SMIL 3.0, EPUB Media Overlays 3.3
- W3C TTML2, WebVTT 1.0, SSML 1.1, PLS 1.0
- ONIX for Books 3.1, Schema.org `Book`
- ISO 14289-1:2014 (PDF/UA-1)

---

## §1 Scope

This PHASE specifies the HTTP-based interfaces
between publishers, reading-app vendors,
accessibility auditors, distribution platforms,
and audit authorities.

## §2 Operation groups

| Prefix              | Group                                           |
|---------------------|-------------------------------------------------|
| `/v1/publications`  | publication registry                             |
| `/v1/representations`| per-modality representation store               |
| `/v1/manifests`     | cross-format manifest registry                   |
| `/v1/narrations`    | narration / media-overlay store                  |
| `/v1/accessibility` | accessibility declaration registry              |
| `/v1/contributors`  | contributor registry                             |
| `/v1/registry`      | registry directory                               |

## §3 Authentication

Read endpoints accept anonymous or JWT bearer.
Write endpoints require a JWT bound to the
publisher identity. Sovereign legal-deposit
registries accept signatures from registered
publishers only.

## §4 Publication operations

### 4.1 Publish publication

```
POST /v1/publications
```

Body: publication record (PHASE-1 §2) signed by
the publisher.

### 4.2 Lookup

```
GET /v1/publications/{publicationRef}
```

Returns the canonical publication record with
conditional GET via `ETag`.

### 4.3 Search

```
GET /v1/publications?isbn={isbn}&language={bcp47}&genre={ONIX}
```

Returns matching publications; pagination beyond
100.

## §5 Representation operations

### 5.1 Upload representation

```
POST /v1/representations
```

Body: representation record (PHASE-1 §3) plus the
attached artefact (EPUB, PDF/UA, audiobook,
Braille file, glTF / USDZ scene, sign-language
video).

### 5.2 Fetch artefact

```
GET /v1/representations/{representationRef}/artefact
```

Returns the artefact bytes. The response carries
`Digest: sha-512=<base64>` (RFC 9530).

### 5.3 Per-modality lookup

```
GET /v1/publications/{publicationRef}/representations?modality={mod}
```

Returns the representation set for the modality.

## §6 Manifest operations

### 6.1 Publish manifest

```
POST /v1/manifests
```

Body: cross-format manifest (PHASE-1 §6).

### 6.2 Lookup

```
GET /v1/manifests/{manifestRef}
```

Returns the manifest with its
`equivalenceClaim` field.

## §7 Narration operations

### 7.1 Upload narration

```
POST /v1/narrations
```

Body: narration record (PHASE-1 §4) plus the
SMIL Media Overlay or sign-language video.

### 7.2 Pronunciation lookup

```
GET /v1/narrations/{narrationRef}/pronunciation
```

Returns the W3C PLS 1.0 lexicon used for
narration.

## §8 Accessibility operations

### 8.1 Submit declaration

```
POST /v1/accessibility
```

Body: accessibility declaration (PHASE-1 §5) with
attached evidence (Ace by DAISY report, WCAG
checker output).

### 8.2 Lookup

```
GET /v1/accessibility/{accessibilityRef}
```

Returns the declaration plus evidence URLs.

## §9 Contributor operations

### 9.1 Register contributor

```
POST /v1/contributors
```

Body: contributor record (PHASE-1 §7).

### 9.2 Lookup

```
GET /v1/contributors/{contributorRef}
```

Returns the canonical contributor record.

## §10 Error semantics

Errors are `application/problem+json` (RFC 9457)
namespaced under
`https://wiastandards.com/errors/pubscript/`.

## §11 Caching and rate limits

Representation artefacts are immutable; they
carry `Cache-Control: public, max-age=31536000,
immutable`. Publication and manifest records
carry strong `ETag`. Rate limits follow the
draft-ietf-httpapi-ratelimit-headers convention.

## Annex A — OpenAPI 3.1 fragment

```yaml
openapi: 3.1.0
info: {title: WIA-pubscript API, version: 1.0.0}
paths:
  /v1/publications:
    post:
      summary: Publish a publication
      requestBody:
        required: true
        content:
          application/json:
            schema: {$ref: 'PublicationRecord.schema.json'}
      responses:
        '201': {description: Publication published}
```

## Annex B — Idempotency

Mutating operations honour `Idempotency-Key` for
24h.

## Annex C — Webhook subscriptions

Subscribers receive events on
`publication.published`,
`representation.uploaded`,
`manifest.published`,
`accessibility.declared`. Delivery is signed
with HMAC-SHA-256.

## Annex D — Federation

Federation between sister registries follows the
discovery contract in PHASE-3.

## Annex E — Bulk export

`POST /v1/registry/export` returns a signed URL
to a `tar.zst` of records filtered by publisher
and date range.

## Annex F — Sandbox

`/v1/sandbox` mirrors production with synthetic
publications.

## Annex G — Quotas

Per-publisher quotas: 100 publications / day;
1,000 representation uploads / day.

## Annex H — Audit feed

`GET /v1/registry/audit?since=<timestamp>`.

## Annex I — Public introspection

`GET /v1/registry/stats`.

## Annex J — Webhook payload shape

```json
{
  "event": "representation.uploaded",
  "representationRef": "https://reg.example.org/repr/...",
  "publicationRef": "https://reg.example.org/pubs/...",
  "modality": "tactile"
}
```

## Annex K — Equivalence claim audit

```
POST /v1/manifests/{manifestRef}/audit
```

Body: a SHACL filter selecting representations to
compare. Response: a structural diff verdict
across modalities supporting the
`equivalenceClaim`. Auditors use the verdict to
confirm verified-equivalent claims.

## Annex L — Sovereign legal-deposit

```
POST /v1/registry/legal-deposit
```

Body: publication reference and the destination
legal-deposit authority (KORLI, BNF, BL, LoC,
NDL). The registry mirrors the publication to the
authority's deposit endpoint per the bilateral
deposit agreement.

## Annex M — Cross-modality search

```
POST /v1/representations/search
```

Body: a content fingerprint (text n-gram,
audio fingerprint, Braille pattern signature,
sign-language gesture sequence). Response:
representations whose content fingerprint
matches across modalities.

## Annex N — Pronunciation lexicon publication

```
POST /v1/registry/pronunciation
```

Body: a W3C PLS 1.0 lexicon for a specific
publisher / domain. Lexicons are versioned with
Semantic Versioning 2.0.0 and surface to
narration records that reference them.

## Annex O — Reader-app capability negotiation

```
GET /v1/registry/reader-capabilities?app={appRef}
```

Returns the reader app's declared capabilities
(supported modalities, codec set, accessibility
features). Used by publishers planning per-app
test matrices.

## Annex P — Bulk legal-deposit

```
POST /v1/registry/legal-deposit/bulk
```

Body: a SHACL filter selecting publications and
the destination legal-deposit authority.
Response: a deposit job reference for
asynchronous mirroring.

## Annex Q — Vendor catalogue

```
GET /v1/registry/publishers/{publisherRef}
```

Returns the publisher's catalogue with the
attestation chain (ISBN issuance authority, ONIX
metadata source, signing key set).

## Annex R — Audit-grade evidence export

```
POST /v1/accessibility/{accessibilityRef}/audit-export
```

Body: a date range and a SHACL filter over
representations. Response: a signed `tar.zst`
archive containing the accessibility checker
output, the WCAG conformance report, and the
publisher's evidence narrative.

## Annex S — Pronunciation lexicon round-trip

```
POST /v1/registry/pronunciation/round-trip
```

Body: a publication reference and a target
voice family. Response: the resolved
pronunciation tokens for the publication's
content, with any unresolved tokens listed for
the publisher to remedy.

## Annex T — Sign-language coverage

```
GET /v1/publications/{publicationRef}/sign-coverage
```

Returns the sign-language coverage matrix
across the publication's chapters: per-chapter
sign-language video presence, signing variant
(ISO 639-3 code), and quality grade.

## Annex U — Manifest SHACL validation

```
POST /v1/manifests/validate
```

Body: a candidate manifest. Response: a Problem
Details report listing SHACL violations against
the canonical manifest shape.

## Annex V — Reading-app catalogue

```
GET /v1/registry/reader-apps
```

Returns the catalogue of registered reading apps
with their declared capabilities, supported
modalities, and conformance attestation
references.

## Annex W — Equivalence diff endpoint

```
POST /v1/manifests/{manifestRef}/equivalence-diff
```

Body: two representation references within the
manifest. Response: a structural diff showing
content overlap and gaps so that publishers can
remediate equivalence drift.

## Annex X — Marrakesh export

```
POST /v1/publications/{publicationRef}/marrakesh
```

Body: requesting authorised entity identity and
target accessible-format. Response: a signed
distribution token enabling cross-border
production of the accessible copy under the
Marrakesh Treaty exception.

## Annex Y — Reading-app feedback

```
POST /v1/reader-apps/{appRef}/feedback
```

Body: structured feedback on rendering,
accessibility, or performance issues observed
by the reading app. Feedback informs the
publisher's next release.

## Annex Z — Cohort-based access

```
GET /v1/representations/{representationRef}?cohort={cohortRef}
```

Returns the representation when the requester is
authorised under the cohort SHACL filter (e.g.,
educational institutions, library subscribers,
sovereign-equivalent licensee groups).

## Annex AA — Per-modality preview

```
GET /v1/representations/{representationRef}/preview
```

Returns a short preview clip of the modality:
first chapter for visual / auditory / Braille,
30-second clip for sign-language video, exterior
view for spatial. Previews are public and
unwatermarked so that consumers can choose
modalities without commitment.

## Annex AB — Translator workflow

```
POST /v1/contributors/translator-workflow
```

Body: source language version reference and
target BCP 47 tag. Initiates a translator
workflow: translator assignment, draft tracking,
quality review, and final publication of the
target language version.

## Annex AC — Equivalence harness

```
POST /v1/manifests/{manifestRef}/equivalence-harness
```

Body: harness invocation parameters. Response: a
job reference; the harness compares modalities
content and emits an equivalence report
referenced from the manifest.

## Annex AD — Bulk publication import

```
POST /v1/publications/bulk
Content-Type: application/jsonl
```

Body: a JSON-Lines stream of publication
records. Used by publishers migrating large
backlists from a previous registry.

## Annex AE — Webhook payload signature

Webhook payloads are canonicalised per RFC 8785
before HMAC-SHA-256 signing. Signature appears
in `X-WIA-Signature: sha256=<hex>`. Deliveries
follow at-least-once semantics.

## Annex AF — Audit feed retention

The audit feed is retained for at least 7 years
to support legal-deposit traceability and
sovereign-equivalent retention requirements.

弘益人間 (Hongik Ingan) — Benefit All Humanity
