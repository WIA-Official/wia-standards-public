# WIA-EDU-023: Implementation Guide
## Cultural Heritage Digitization

**Version:** 1.0.0
**Last Updated:** 2025-01-26

---

## 1. Getting Started

### 1.1 Prerequisites

**Hardware:**
- Camera (20MP+) or smartphone with good camera
- Computer with GPU for processing (recommended)
- Storage: 1TB+ for projects
- Optional: 3D scanner, LiDAR device

**Software:**
- 3D capture: Agisoft Metashape, RealityCapture, or Meshroom (free)
- 3D editing: Blender (free), MeshLab (free)
- Metadata: Spreadsheet or dedicated system

---

## 2. Quick Start Workflow

### Step 1: Planning
1. Assess artifact characteristics
2. Choose capture method (photogrammetry, LiDAR, etc.)
3. Prepare environment (lighting, background)
4. Set up color/scale references

### Step 2: Capture
1. Take overlapping photos from all angles
2. Include scale bars and color targets
3. Verify coverage completeness
4. Back up raw images immediately

### Step 3: Processing
1. Import images to software
2. Align photos (sparse point cloud)
3. Build dense point cloud
4. Generate mesh
5. Create texture atlas
6. Export to standard formats

### Step 4: Quality Check
1. Verify accuracy with measurements
2. Check for holes or artifacts
3. Validate color accuracy
4. Test model in viewer

### Step 5: Metadata
1. Complete Dublin Core fields
2. Add technical capture details
3. Document processing steps
4. Assign persistent identifier

### Step 6: Preservation
1. Store master files in archival format
2. Generate checksums
3. Create multiple backups
4. Document storage locations

### Step 7: Dissemination
1. Create optimized web versions
2. Generate thumbnails
3. Upload to repository
4. Share with appropriate licenses

---

## 3. Best Practices

### Photogrammetry Tips
- Use manual camera settings (fixed ISO, aperture, white balance)
- Shoot RAW for maximum quality
- Overlap images 60-80%
- Avoid shiny, transparent, or very dark objects (or use scanning spray)
- Circle object in horizontal bands
- Take additional detail shots

### Metadata Essentials
- Document everything immediately
- Use controlled vocabularies
- Link to authority files (Getty, Wikidata)
- Include uncertainty estimates for reconstructions
- Store metadata with files (sidecar approach)

### Accessibility
- Provide multiple formats (3D, 2D images, text)
- Include detailed descriptions
- Test with screen readers
- Offer multiple languages
- Ensure keyboard navigation

---

## 4. Common Issues

### Problem: Incomplete mesh coverage
**Solution:** Retake photos from missing angles

### Problem: Texture seams visible
**Solution:** Improve UV unwrapping, use blending in texture baking

### Problem: File too large for web
**Solution:** Decimate mesh, compress textures, use Draco compression

### Problem: Colors don't match original
**Solution:** Use color calibration targets, shoot in RAW, correct in processing

---

## 5. Tools & Resources

### Free/Open Source
- **Meshroom:** Photogrammetry (Windows/Linux)
- **Blender:** 3D modeling and optimization
- **CloudCompare:** Point cloud processing
- **QGIS:** GIS and georeferencing

### Institutional
- **Agisoft Metashape:** Professional photogrammetry
- **RealityCapture:** Fast photogrammetry
- **Faro Scene:** LiDAR processing

### Validation
- **ColorChecker Passport:** Color calibration
- **Scale bars:** Dimensionally accurate references
- **Control measurements:** Calipers, laser distance meters

---

## 6. Training Resources

- WIA Online Courses: https://training.wiastandards.com
- Cultural Heritage Imaging: http://culturalheritageimaging.org
- Smithsonian Digitization Program: https://3d.si.edu
- Community Forums: https://forum.wiastandards.com

---

## 7. Certification Path

1. Complete training modules
2. Submit portfolio (3 projects minimum)
3. Pass technical assessment
4. Peer review
5. Receive WIA certification

---

## 8. Support

**Technical Support:** support@wiastandards.com
**Community Forum:** https://forum.wiastandards.com
**GitHub Issues:** https://github.com/WIA-Official/wia-standards/issues

---

© 2025 WIA - MIT License


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
