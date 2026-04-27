# WIA-images PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-images
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for WIA-images.
The standard covers exchange of digital still-image assets and their
operational metadata across publishing pipelines, content-management
systems, image-CDN services, accessibility tooling, rights-clearance
operators, and downstream renderers (web, mobile, print). The format
captures source asset identity, bitstream metadata, encoding
variants, derivative renditions, accessibility descriptions,
provenance attestations, rights-clearance records, and the
delivery / cache state of each rendition.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (testing and calibration laboratories — used for
  colour-management calibration where the laboratory model applies)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- ISO/IEC 15444 (JPEG 2000 family)
- ISO/IEC 23000 (MPEG-A application formats; cited only for HEIF
  envelope reference)
- ISO/IEC 21000-21 (MPEG-21 — Cross-Media Exchange; cited normatively
  for cross-platform rights metadata definitions)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- W3C WCAG 2.2 (cited normatively for accessibility text-alternative
  conventions)
- W3C PNG (Portable Network Graphics) Specification
- W3C SVG 2 (Scalable Vector Graphics)
- C2PA (Content Authenticity / provenance manifest framework — cited
  normatively as a community-managed envelope for provenance
  attestations)

---

## §1 Scope

This PHASE document defines persistent shapes for image asset
records as they flow through ingestion, transformation, delivery,
and archival. Implementations covered include:

- Editorial and publishing pipelines that ingest source photographs
  and illustrations.
- Content-management systems that hold per-asset metadata and
  rights-clearance state.
- Image-CDN services that serve responsive renditions to web and
  mobile clients.
- Accessibility tooling that generates and reviews text alternatives.
- Rights-clearance operators that track licence terms and expiry.
- Provenance services that emit C2PA-compatible attestations.

Generative-image model internals (training data, model weights),
real-time camera-pipeline calibration, and broadcast-video
distribution are out of scope.

## §2 Asset Identifier

```
assetId           : string (uuidv7)
assetCreatedAt    : string (ISO 8601 / RFC 3339)
operatorRef       : string (institutional identifier of the
                       publishing operator)
sourceKind        : enum  ("photograph" | "illustration" |
                       "screenshot" | "diagram" | "infographic" |
                       "synthetic-rendered" | "user-uploaded" |
                       "scan-of-physical")
intendedUse       : enum  ("editorial" | "advertising" |
                       "internal-only" | "public-domain-source" |
                       "ai-training" | "user-generated-content")
assetStatus       : enum  ("draft" | "ready-for-delivery" |
                       "embargoed" | "withdrawn" | "deprecated")
```

A withdrawn asset (rights-clearance lapse, take-down notice,
operator-initiated removal) emits a withdrawal notice that the API
serves alongside the canonical record.

## §3 Bitstream Record

The bitstream record describes the source-of-truth digital file.
The bitstream itself is content-addressed; the record carries the
metadata needed to interpret it.

```
bitstream:
  bitstreamId     : string (uuidv7)
  assetId         : string (uuidv7)
  format          : enum ("jpeg" | "png" | "webp" | "avif" |
                       "heif" | "tiff" | "jpeg-2000" | "svg" |
                       "raw-cr2" | "raw-nef" | "raw-arw" |
                       "raw-dng" | "user-defined")
  contentDigest   : string (SHA-256 of the bitstream)
  artefactRef     : string (content-addressed URI)
  pixelWidth      : integer
  pixelHeight     : integer
  bitDepth        : integer
  colourSpace     : enum ("srgb" | "display-p3" | "rec-2020" |
                       "adobe-rgb" | "prophoto-rgb" | "cmyk" |
                       "grayscale" | "lab")
  iccProfileRef   : string (content-addressed URI of the embedded
                       ICC profile, when not srgb)
  exifRef         : string (content-addressed URI of the EXIF
                       sidecar; absent when EXIF is empty or
                       stripped)
```

## §4 Capture and Camera Metadata

For photograph-kind assets, the capture record carries the EXIF-
sourced camera metadata that downstream consumers (forensics, image
search, rights verification) reference.

```
capture:
  captureId       : string (uuidv7)
  bitstreamId     : string (uuidv7)
  capturedAt      : string (ISO 8601 / RFC 3339; timezone-pinned)
  cameraMake      : string
  cameraModel     : string
  lensModel       : string
  exposureSeconds : number
  apertureFNumber : number
  isoSensitivity  : integer
  focalLengthMm   : number
  flashFired      : boolean
  geoLatitude     : number (when EXIF GPSLatitude is present and
                       not redacted)
  geoLongitude    : number
  geoRedacted     : boolean (operator may strip GPS for privacy)
```

Programmes that publish externally cited photographs MUST record
whether GPS was redacted; redaction is permanent at the bitstream
level once a derivative is published, but the capture record's
`geoRedacted` flag preserves the audit fact.

## §5 Variant and Rendition Record

The original bitstream is rendered into multiple variants for
delivery: width-stepped responsive renditions, format-converted
copies for browser support fan-out, focal-cropped art-direction
variants, and accessibility-friendly renditions.

```
rendition:
  renditionId     : string (uuidv7)
  bitstreamId     : string (uuidv7, source bitstream)
  variantPurpose  : enum ("responsive-step" | "format-convert" |
                       "art-direction-crop" | "thumbnail" |
                       "preview" | "accessibility-friendly" |
                       "preview-blurhash" | "preview-svg-placeholder"
                       | "user-defined")
  format          : enum (matches §3.format)
  pixelWidth      : integer
  pixelHeight     : integer
  contentDigest   : string (SHA-256)
  artefactRef     : string (content-addressed URI)
  derivationRecipe : string (recipe identifier; renditions produced
                       by a deterministic pipeline cite the recipe
                       so re-rendering is reproducible)
```

## §6 Accessibility Description Record

Accessibility tooling emits text alternatives, long descriptions,
and decorative-flag metadata for each asset. The record carries the
description per locale and the review state.

```
accessibility:
  accessibilityId : string (uuidv7)
  assetId         : string (uuidv7)
  locale          : string (BCP 47)
  altText         : string (short text alternative; required for
                       non-decorative assets)
  longDescriptionRef : string (content-addressed URI of an
                       extended description, when the image
                       requires more than altText)
  isDecorative    : boolean (when true, altText is empty per
                       WCAG 2.2 1.1.1 decorative image guidance)
  reviewState     : enum ("auto-generated" | "human-reviewed" |
                       "human-authored" | "needs-review")
  reviewedAt      : string (ISO 8601; absent until reviewed)
```

Auto-generated text alternatives MUST be reviewed before the
asset is published in editorial or advertising contexts; the
review state gates the publication pipeline.

## §7 Provenance Attestation Record

Provenance attestations (C2PA-style content credentials) describe
the asset's chain of edits and the signers responsible for each
step.

```
provenance:
  provenanceId    : string (uuidv7)
  assetId         : string (uuidv7)
  manifestRef     : string (content-addressed URI of the C2PA
                       manifest)
  signerRef       : string (institutional signer identifier)
  steps           : array of ProvenanceStep
  isVerified      : boolean (signature verification result at
                       last verification)
  lastVerifiedAt  : string (ISO 8601)

ProvenanceStep:
  stepKind        : enum ("captured" | "imported" | "edited" |
                       "synthesised" | "format-converted" |
                       "rights-cleared" | "published" |
                       "user-defined")
  occurredAt      : string (ISO 8601)
  actorRef        : string (institutional or tool identifier)
```

## §8 Rights and Clearance Record

```
rightsClearance:
  clearanceId     : string (uuidv7)
  assetId         : string (uuidv7)
  rightsHolderRef : string (institutional rights-holder identifier)
  licenceCode     : enum ("rf" | "rm" | "editorial-use-only" |
                       "creative-commons-by" |
                       "creative-commons-by-sa" |
                       "creative-commons-by-nd" |
                       "creative-commons-by-nc" |
                       "public-domain" | "operator-internal" |
                       "ai-training-permitted" |
                       "ai-training-prohibited")
  licenceTermsRef : string (content-addressed URI of the licence
                       terms artefact)
  licenceExpiresAt: string (ISO 8601 date; absent for perpetual)
  geographyScope  : array of string (ISO 3166-1 codes; absent for
                       worldwide)
  embargoUntil    : string (ISO 8601; absent for unembargoed)
```

A rendition served to a request whose context falls outside the
licence's `geographyScope` returns `403 Forbidden` from the API
with type `urn:wia:images:licence-geography-violation`.

## §9 Delivery State Record

```
deliveryState:
  deliveryId      : string (uuidv7)
  renditionId     : string (uuidv7)
  cdnEdgeId       : string (content-delivery edge identifier)
  cachedAt        : string (ISO 8601)
  invalidationsApplied : array of InvalidationEntry
```

## §10 IPTC Photo Metadata Record

Editorial-context assets carry IPTC PhotoMetadata fields (creator,
credit line, copyright notice, headline, caption, keywords) which
are referenced as a content-addressed sidecar to the bitstream so
that downstream consumers can rebuild the IPTC fan-out from a
single source of truth.

```
iptcMetadata:
  iptcId          : string (uuidv7)
  bitstreamId     : string (uuidv7)
  iptcArtefactRef : string (content-addressed URI of the IPTC JSON
                       sidecar)
  creator         : string (author display string)
  credit          : string (publisher-required credit line)
  copyrightNotice : string
  headline        : string
  description     : string (free text caption; localised
                       descriptions follow PHASE-1 §6 accessibility
                       records when relevant)
  keywords        : array of string
  subjectCode     : array of string (IPTC Subject Codes when
                       applicable)
```

IPTC fields are mandatory for editorial-context assets and SHOULD
be present for advertising-context assets; assets that lack
IPTC creator and copyright fields cannot transition past
`ready-for-delivery` for those use contexts.

## §11 Person-Pictured Subject Record

Editorial photographs that depict identifiable people carry a
person-pictured record so that downstream consumers can honour
model releases, image-rights restrictions, and right-to-erasure
requests.

```
personPictured:
  recordId        : string (uuidv7)
  assetId         : string (uuidv7)
  subjectToken    : string (opaque token mapped in operator CRM
                       to the depicted person's contact record)
  releaseKind     : enum ("signed-model-release" |
                       "editorial-public-figure" |
                       "implicit-public-event" |
                       "no-release-no-publication")
  releaseRef      : string (content-addressed URI of the signed
                       release artefact when applicable)
  ageMinor        : boolean (additional minor-rights workflows
                       apply when true)
  jurisdictionScope : array of string (ISO 3166 codes)
```

The clinical / legal identity of the depicted person is held in
the operator's CRM, not in the API. Right-to-erasure requests
flow through the CRM's subject-access workflow.

## §12 Print-Production Variant (Optional)

Print-bound assets carry colour-managed CMYK renditions and
print-process-specific metadata so that downstream press operations
do not need to derive colour conversions ad-hoc.

```
printProduction:
  recordId        : string (uuidv7)
  renditionId     : string (uuidv7)
  inkProfile      : string (CMYK profile identifier)
  paperStock      : string (paper specification reference)
  totalAreaCoveragePct : number (TAC budget)
  separationStrategy   : enum ("ucr" | "gcr-light" |
                       "gcr-medium" | "gcr-heavy" |
                       "user-defined")
```

## §13 Animated and Sequence Image Variant

Animated formats (animated WebP, animated AVIF, animated PNG, GIF)
and image-sequence assets (multi-frame OME-TIFF, photo bursts)
carry a sequence record that describes per-frame metadata so that
downstream consumers can reason about timing, looping, and frame-
level accessibility.

```
sequenceVariant:
  recordId        : string (uuidv7)
  bitstreamId     : string (uuidv7)
  frameCount      : integer
  totalDurationMs : integer (omitted for non-temporal sequences)
  loopBehaviour   : enum ("none" | "indefinite" |
                       "fixed-count" | "ping-pong" |
                       "user-defined")
  perFrameDurationMs : array of integer
  reduceMotionAlternativeRef : string (URI of a still-image
                       fallback honoured by clients that respect
                       prefers-reduced-motion)
```

Animated assets MUST publish a `reduceMotionAlternativeRef` so that
clients honouring user-agent reduced-motion preferences can fall
back gracefully.

## §14 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every published asset and honour the
content-addressing rules in §3-§9.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-images
- **Last Updated:** 2026-04-27
