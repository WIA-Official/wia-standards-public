# WIA-EDU-008 Digital Textbook Standard v1.0

## Phase 1: Data Format & Structure

**Status:** ✅ Complete
**Version:** 1.0.0
**Date:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the foundational data format and structure requirements for WIA-compliant digital textbooks. Phase 1 establishes EPUB3 as the mandatory format with comprehensive metadata, accessibility features, and interactive capabilities.

## 2. Scope

Phase 1 covers:
- File format requirements (EPUB3)
- Metadata schemas
- Accessibility markup
- Content structure
- Multimedia integration
- Annotation support

## 3. EPUB3 Requirements

### 3.1 Base Format

All digital textbooks MUST conform to EPUB 3.3 specification (or later) as defined by the W3C EPUB 3 Community Group.

**Mandatory Features:**
- Valid EPUB3 structure with `mimetype`, `META-INF/container.xml`, and OPF package document
- UTF-8 encoding for all text content
- HTML5 semantic markup for content documents
- CSS3 for styling
- Navigation document (nav.xhtml) using HTML5 `<nav>` element

### 3.2 File Structure

```
textbook.epub (ZIP archive with mimetype first, uncompressed)
├── mimetype                    # MUST be first file, uncompressed
├── META-INF/
│   ├── container.xml          # Points to OPF file location
│   ├── encryption.xml         # Optional: DRM info
│   └── signatures.xml         # Optional: digital signatures
├── OEBPS/  (or custom directory)
│   ├── content.opf            # Package document
│   ├── toc.ncx                # EPUB2 backward compatibility
│   ├── nav.xhtml              # EPUB3 navigation document
│   ├── chapters/
│   │   ├── chapter-01.xhtml
│   │   ├── chapter-02.xhtml
│   │   └── ...
│   ├── images/
│   │   ├── cover.jpg
│   │   └── ...
│   ├── audio/
│   ├── video/
│   ├── css/
│   │   └── styles.css
│   └── js/
│       └── interactive.js
```

## 4. Metadata Requirements

### 4.1 Dublin Core Metadata (Required)

The OPF package document MUST include the following Dublin Core metadata:

```xml
<metadata xmlns:dc="http://purl.org/dc/elements/1.1/">
  <dc:identifier id="bookid">urn:isbn:978-3-16-148410-0</dc:identifier>
  <dc:title>Introduction to Biology</dc:title>
  <dc:creator id="creator1">Smith, Jane</dc:creator>
  <dc:creator id="creator2">Doe, John</dc:creator>
  <dc:language>en-US</dc:language>
  <dc:date>2025-01-15</dc:date>
  <dc:publisher>Academic Press</dc:publisher>
  <dc:subject>Biology</dc:subject>
  <dc:subject>Life Sciences</dc:subject>
  <dc:description>A comprehensive introduction to biological sciences...</dc:description>
  <dc:rights>Copyright 2025 Academic Press. All rights reserved.</dc:rights>
</metadata>
```

### 4.2 WIA-Specific Metadata (Required)

```xml
<meta property="wia:standard">WIA-EDU-008</meta>
<meta property="wia:version">1.0</meta>
<meta property="wia:certificationDate">2025-01-15</meta>
<meta property="wia:educationLevel">Grade 9-10</meta>
<meta property="wia:contentVersion">1.0.0</meta>
<meta property="wia:wcagLevel">AA</meta>
```

### 4.3 Accessibility Metadata (Required)

Following schema.org and ONIX accessibility metadata:

```xml
<meta property="schema:accessMode">textual</meta>
<meta property="schema:accessMode">visual</meta>
<meta property="schema:accessMode">auditory</meta>
<meta property="schema:accessibilityFeature">alternativeText</meta>
<meta property="schema:accessibilityFeature">captions</meta>
<meta property="schema:accessibilityFeature">structuralNavigation</meta>
<meta property="schema:accessibilityFeature">tableOfContents</meta>
<meta property="schema:accessibilityHazard">none</meta>
<meta property="schema:accessibilityControl">fullKeyboardControl</meta>
<meta property="schema:accessibilitySummary">This publication conforms to WCAG 2.1 Level AA</meta>
```

## 5. Accessibility Requirements

### 5.1 WCAG 2.1 Level AA Compliance

All content MUST meet WCAG 2.1 Level AA criteria.

### 5.2 Images

**All images MUST have:**
- Descriptive `alt` attributes
- `role="img"` for semantic clarity
- Extended descriptions via `aria-describedby` for complex images

```html
<figure>
  <img src="images/cell-diagram.svg"
       alt="Eukaryotic cell structure showing nucleus, mitochondria, and other organelles"
       role="img"
       aria-describedby="cell-desc" />
  <figcaption id="cell-desc">
    Detailed cross-section of a eukaryotic cell. The large central nucleus
    contains chromatin and nucleolus. Surrounding organelles include...
  </figcaption>
</figure>
```

### 5.3 Mathematical Content

Mathematics MUST be encoded in MathML with alternative text:

```html
<math xmlns="http://www.w3.org/1998/Math/MathML"
      alttext="E equals m c squared">
  <mi>E</mi>
  <mo>=</mo>
  <mi>m</mi>
  <msup>
    <mi>c</mi>
    <mn>2</mn>
  </msup>
</math>
```

### 5.4 Tables

Tables MUST have:
- `<caption>` elements
- Proper `<th>` headers with `scope` attributes
- ARIA labels where appropriate

```html
<table>
  <caption>Periodic Table Elements - First 10</caption>
  <thead>
    <tr>
      <th scope="col">Atomic Number</th>
      <th scope="col">Symbol</th>
      <th scope="col">Name</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>1</td>
      <td>H</td>
      <td>Hydrogen</td>
    </tr>
    <!-- ... -->
  </tbody>
</table>
```

### 5.5 Multimedia

**Video Requirements:**
- WebVTT captions for all spoken content
- Audio descriptions track for visual content
- Transcript available as alternative

```html
<video controls aria-label="Photosynthesis process">
  <source src="video/photosynthesis.mp4" type="video/mp4">
  <track kind="captions" src="captions/photosynthesis-en.vtt"
         srclang="en" label="English">
  <track kind="descriptions" src="descriptions/photosynthesis-en.vtt"
         srclang="en" label="Audio Description">
</video>

<details>
  <summary>Video Transcript</summary>
  <p>Photosynthesis is the process by which plants...</p>
</details>
```

**Audio Requirements:**
- Transcripts for all audio content
- Synchronization with visual content where applicable

## 6. Interactive Elements

### 6.1 Quizzes and Assessments

Interactive elements MUST be:
- Keyboard accessible
- Screen reader compatible
- Provide immediate feedback

```html
<section class="quiz" role="region" aria-label="Chapter 1 Self-Assessment">
  <h2>Quiz: Chapter 1</h2>

  <div class="question" role="group" aria-labelledby="q1-text">
    <p id="q1-text">What is the powerhouse of the cell?</p>
    <div role="radiogroup" aria-labelledby="q1-text">
      <label>
        <input type="radio" name="q1" value="nucleus"> Nucleus
      </label>
      <label>
        <input type="radio" name="q1" value="mitochondria"> Mitochondria
      </label>
      <label>
        <input type="radio" name="q1" value="ribosome"> Ribosome
      </label>
    </div>
  </div>

  <button type="button" onclick="checkQuiz()" aria-label="Submit quiz answers">
    Submit Answers
  </button>

  <div id="quiz-results" role="status" aria-live="polite" aria-atomic="true">
    <!-- Results appear here -->
  </div>
</section>
```

## 7. Annotation Support

### 7.1 Annotation Data Format

Textbooks MUST support the W3C Web Annotation Data Model:

```json
{
  "@context": "http://www.w3.org/ns/anno.jsonld",
  "type": "Annotation",
  "id": "urn:uuid:12345678-1234-1234-1234-123456789abc",
  "created": "2025-12-25T10:30:00Z",
  "creator": "user@example.com",
  "motivation": "highlighting",
  "target": {
    "source": "urn:isbn:978-3-16-148410-0",
    "selector": {
      "type": "FragmentSelector",
      "conformsTo": "http://www.w3.org/TR/media-frags/",
      "value": "xpointer(/html/body/section[2]/p[3])"
    }
  },
  "body": {
    "type": "TextualBody",
    "value": "Important for exam",
    "format": "text/plain",
    "language": "en"
  },
  "stylesheet": {
    "type": "CssStylesheet",
    "value": ".highlighted { background-color: #FFFF00; }"
  }
}
```

## 8. Content Versioning

### 8.1 Version Metadata

```xml
<meta property="wia:contentVersion">1.2.0</meta>
<meta property="wia:previousVersion">1.1.0</meta>
<meta property="wia:versionDate">2025-12-25</meta>
<meta property="wia:changelog">
  Updated Chapter 5 with new research findings.
  Corrected errors in Chapter 7 equations.
  Added interactive simulations to Chapter 9.
</meta>
```

## 9. Validation and Testing

### 9.1 EPUB Validation

All textbooks MUST pass EPUBCheck validation:

```bash
java -jar epubcheck.jar textbook.epub
```

### 9.2 Accessibility Testing

Accessibility MUST be validated with ACE (Accessibility Checker for EPUB):

```bash
ace textbook.epub
```

## 10. Conformance Levels

**Level 1 (Required):** EPUB3 valid, basic accessibility (WCAG A)
**Level 2 (Required):** Enhanced accessibility (WCAG AA)
**Level 3 (Recommended):** Full accessibility (WCAG AAA)

---

## Appendix A: Sample OPF Package Document

See reference implementation at: https://github.com/WIA-Official/digital-textbook/examples

## Appendix B: Validation Checklist

- [ ] EPUB3 structure valid
- [ ] EPUBCheck passes with no errors
- [ ] All required metadata present
- [ ] All images have alt text
- [ ] Math encoded in MathML
- [ ] Videos have captions
- [ ] Tables have proper headers
- [ ] Interactive elements keyboard accessible
- [ ] ACE accessibility check passes
- [ ] Navigation document present
- [ ] Content version metadata included

---

**Philosophy:** 弘益人間 · Benefit All Humanity

© 2025 WIA - World Certification Industry Association
License: CC BY 4.0


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
