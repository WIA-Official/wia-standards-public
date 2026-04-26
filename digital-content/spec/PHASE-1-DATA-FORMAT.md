# WIA-EDU-019 Digital Content Standard v1.0

## Phase 1: Content Format & Structure

**Status:** ✅ Complete
**Version:** 1.0.0
**Date:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the foundational content format and structure requirements for WIA-compliant digital content. Phase 1 establishes format specifications, metadata schemas, and accessibility requirements for all types of digital educational content.

## 2. Scope

Phase 1 covers:
- Multimedia format specifications (video, audio, image, 3D)
- Document and interactive content formats
- Comprehensive metadata schemas
- Accessibility markup and features
- Content structure and organization

## 3. Video Content Formats

### 3.1 Required Formats

| Container | Codec | Resolution | Bitrate | Status |
|-----------|-------|------------|---------|--------|
| MP4 | H.264 (AVC) | 1920x1080 | 5 Mbps | ✅ Required |
| WebM | VP9 | 1920x1080 | 5 Mbps | ✅ Required |
| MP4 | H.265 (HEVC) | 3840x2160 | 15 Mbps | ⚠️ Optional |

### 3.2 Video Specifications

**Minimum Requirements:**
- Resolution: 1280x720 (720p HD)
- Frame Rate: 24-30 fps
- Bitrate: 2.5 Mbps minimum
- Color Space: BT.709
- Aspect Ratio: 16:9 standard

**Recommended Standards:**
- Resolution: 1920x1080 (1080p Full HD)
- Frame Rate: 30 fps
- Bitrate: 5-8 Mbps
- Color Space: BT.709 (BT.2020 for HDR)
- HDR: HDR10 or HLG (for HDR content)

## 4. Audio Content Formats

### 4.1 Required Formats

| Format | Codec | Sample Rate | Bitrate | Status |
|--------|-------|-------------|---------|--------|
| MP3 | MPEG-1 Layer 3 | 44.1 kHz | 128-320 kbps | ✅ Required |
| AAC | Advanced Audio Coding | 48 kHz | 128-256 kbps | ✅ Required |
| OGG | Opus | 48 kHz | 128 kbps | ✅ Required |

### 4.2 Audio Specifications

- **Sample Rate:** 44.1 kHz minimum (48 kHz recommended)
- **Bit Depth:** 16-bit minimum (24-bit for high-quality)
- **Channels:** Stereo minimum (5.1/7.1 surround optional)
- **Dynamic Range:** Normalized to -23 LUFS for broadcast

## 5. Image Content Formats

### 5.1 Required Formats

| Format | Type | Use Case | Status |
|--------|------|----------|--------|
| JPEG | Lossy | Photos, general images | ✅ Required |
| PNG | Lossless | Graphics, transparency | ✅ Required |
| WebP | Both | Web optimization | ✅ Required |
| SVG | Vector | Scalable graphics | ✅ Required |

### 5.2 Image Specifications

- **Color Space:** sRGB for web, Adobe RGB for print
- **Resolution:** Minimum 72 DPI for web, 300 DPI for print
- **Compression:** Quality 85-95 for JPEG
- **Metadata:** EXIF, IPTC, XMP embedded
- **Accessibility:** Alternative text (alt) required for all images

## 6. Interactive Content Formats

### 6.1 HTML5 Requirements

All interactive content MUST use:
- HTML5 semantic markup
- CSS3 for styling
- JavaScript ES6+ (transpiled for compatibility)
- Responsive design (mobile-first approach)

### 6.2 WebGL for 3D Content

- WebGL 2.0 required
- glTF 2.0 for 3D models
- PBR (Physically Based Rendering) materials
- Performance: 30 FPS minimum on mobile, 60 FPS on desktop

## 7. Metadata Requirements

### 7.1 Dublin Core (Required)

```xml
<metadata xmlns:dc="http://purl.org/dc/elements/1.1/">
  <dc:title>Content Title</dc:title>
  <dc:creator>Creator Name</dc:creator>
  <dc:subject>Subject/Topic</dc:subject>
  <dc:description>Content Description</dc:description>
  <dc:publisher>Publisher Name</dc:publisher>
  <dc:date>2025-01-15</dc:date>
  <dc:type>Content Type</dc:type>
  <dc:format>File Format</dc:format>
  <dc:identifier>Unique ID</dc:identifier>
  <dc:language>en</dc:language>
  <dc:rights>License Info</dc:rights>
</metadata>
```

### 7.2 WIA-Specific Metadata

```json
{
  "wia": {
    "standard": "WIA-EDU-019",
    "version": "1.0.0",
    "certificationDate": "2025-01-15",
    "wcagLevel": "AA"
  }
}
```

## 8. Accessibility Requirements

### 8.1 WCAG 2.1 Level AA Compliance

All content MUST meet WCAG 2.1 Level AA standards:

**Video Content:**
- Closed captions (synchronized, accurate)
- Audio descriptions (for visual-only information)
- Text transcripts

**Interactive Content:**
- Keyboard navigation support
- Screen reader compatibility
- ARIA labels and roles
- Focus indicators

**Color and Contrast:**
- Contrast ratio 4.5:1 minimum for normal text
- Contrast ratio 3:1 minimum for large text
- No reliance on color alone

## 9. Content Structure

### 9.1 Package Organization

```
content-package/
├── manifest.json
├── metadata.json
├── content/
│   ├── index.html
│   ├── video.mp4
│   ├── audio.mp3
│   └── images/
├── accessibility/
│   ├── captions_en.vtt
│   └── transcript.txt
└── license.txt
```

## 10. Validation and Compliance

### 10.1 Validation Tools

```bash
# Validate content format
wia-validate --type video --format mp4 content.mp4
wia-validate --type audio --format aac audio.aac
wia-validate --accessibility --wcag AA index.html
```

### 10.2 Compliance Checklist

- ✅ All required formats present
- ✅ Metadata complete and valid
- ✅ WCAG 2.1 AA compliance
- ✅ Accessibility features included
- ✅ File integrity verified

## 11. Best Practices

1. **Multi-Format Support:** Provide multiple formats for maximum compatibility
2. **Optimization:** Balance quality and file size
3. **Accessibility First:** Design with accessibility from the start
4. **Metadata Completeness:** Include comprehensive metadata
5. **Testing:** Validate across multiple devices and platforms

## 12. References

- W3C HTML5: https://www.w3.org/TR/html5/
- WCAG 2.1: https://www.w3.org/TR/WCAG21/
- Schema.org: https://schema.org/
- Dublin Core: http://dublincore.org/

---

**WIA-EDU-019 Phase 1 Complete**
弘益人間 · Benefit All Humanity

© 2025 WIA - World Certification Industry Association
MIT License


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
