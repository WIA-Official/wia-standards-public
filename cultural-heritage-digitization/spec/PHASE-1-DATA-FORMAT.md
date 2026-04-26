# WIA-EDU-023: Cultural Heritage Digitization Standard
## Overview & Foundations

> **Philosophy:** 弘益人間 (Benefit All Humanity)

**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-01-26

---

## 1. Introduction

The WIA Cultural Heritage Digitization Standard (WIA-EDU-023) provides comprehensive guidelines for creating, managing, preserving, and disseminating digital representations of cultural heritage artifacts, monuments, and sites.

### 1.1 Purpose

This standard aims to:

- **Preserve** cultural heritage through high-quality digital capture
- **Democratize** access to cultural treasures worldwide
- **Standardize** digitization practices across institutions
- **Enable** interoperability and data sharing
- **Ensure** long-term accessibility and preservation
- **Respect** cultural sensitivities and indigenous rights

### 1.2 Scope

This standard covers:

- 3D scanning and photogrammetry workflows
- Digital preservation and archival formats
- Metadata standards and documentation
- Virtual heritage experiences (VR/AR/Web)
- Historical reconstruction methodologies
- Accessibility and universal design
- Rights management and cultural protocols

### 1.3 Target Audience

- **Museums and cultural institutions**
- **Archaeological teams and researchers**
- **Digital preservation specialists**
- **Educators and content creators**
- **Technology vendors and developers**
- **Government cultural agencies**

---

## 2. Principles

### 2.1 Core Principles

**Universal Access**
Cultural heritage belongs to all humanity. Digital preservation must maximize accessibility while respecting cultural protocols.

**Scientific Rigor**
All digitization must meet high standards of accuracy, documentation, and reproducibility.

**Open Standards**
Prefer non-proprietary, openly documented formats and protocols to ensure long-term accessibility.

**Sustainability**
Plan for long-term preservation, not just initial capture.

**Cultural Sensitivity**
Respect indigenous rights, sacred objects, and cultural protocols. Engage source communities in digitization decisions.

**Transparency**
Clearly distinguish evidence from interpretation. Document all methodologies and assumptions.

### 2.2 Ethical Framework

- **Do No Harm:** Digital activity must not endanger physical artifacts
- **Community Consent:** Obtain permission from source communities
- **Benefit Sharing:** Ensure communities benefit from digitization
- **Repatriation Support:** Digital access can complement physical repatriation
- **Privacy Respect:** Consider privacy implications of cultural documentation

---

## 3. Standard Architecture

### 3.1 Four-Phase Model

**Phase 1: Capture**
- 3D scanning, photogrammetry, imaging
- Quality assurance and validation
- Raw data preservation

**Phase 2: Processing**
- Mesh generation, texture mapping
- Restoration and reconstruction
- Metadata creation

**Phase 3: Preservation**
- Long-term archival storage
- Format migration planning
- Redundancy and backup

**Phase 4: Dissemination**
- Web delivery, VR/AR experiences
- Educational resources
- Research access

### 3.2 Technology Stack

**Capture Technologies:**
- Photogrammetry (structure from motion)
- LiDAR (terrestrial, aerial, handheld)
- Structured light scanning
- CT/X-ray imaging

**File Formats:**
- Point clouds: E57, LAS, PTS
- Meshes: glTF 2.0, OBJ, PLY, FBX
- Archival: X3D, COLLADA
- Images: TIFF (master), JPEG 2000, PNG

**Metadata Standards:**
- Dublin Core 1.1
- CIDOC Conceptual Reference Model (ISO 21127)
- METS (Metadata Encoding & Transmission Standard)
- PREMIS (preservation metadata)

---

## 4. Quality Standards

### 4.1 Capture Requirements

**Small Artifacts (<50cm)**
- Minimum resolution: 100 points/mm²
- Geometric accuracy: ±0.1mm
- Texture resolution: 4K minimum
- Color accuracy: ΔE < 2.0

**Medium Objects (50cm-5m)**
- Minimum resolution: 10 points/cm²
- Geometric accuracy: ±1mm
- Texture resolution: 8K minimum
- Complete surface coverage: >95%

**Large Structures (>5m)**
- Minimum resolution: 1cm accuracy
- Survey control points documented
- Registration accuracy: <2cm RMSE
- Photographic documentation: 24MP+

**Archaeological Sites**
- Minimum resolution: 5cm ground sampling distance
- Georeferencing: RTK GPS or total station
- Contextual photography
- Excavation documentation integration

### 4.2 Processing Standards

- **Mesh quality:** Manifold, watertight where appropriate
- **Texture baking:** No visible seams or distortion
- **Decimation:** Preserve visual fidelity, document reduction ratio
- **File optimization:** Balance quality and file size for use case

---

## 5. Compliance

### 5.1 Conformance Levels

**Level 1 - Basic Compliance**
- Minimum quality standards met
- Dublin Core metadata complete
- Files in approved formats
- Basic preservation copies created

**Level 2 - Full Compliance** (Recommended)
- Best practices followed throughout
- CIDOC-CRM semantic metadata
- Multiple resolution levels
- Comprehensive documentation
- WCAG AA accessibility

**Level 3 - Excellence**
- Exceeds all requirements
- Linked Open Data implementation
- Advanced reconstruction with uncertainty mapping
- WCAG AAA accessibility
- Community engagement documented

### 5.2 Certification

Organizations may seek WIA certification demonstrating compliance with this standard. Certification process includes:

1. Portfolio review of digitized assets
2. Metadata quality assessment
3. Preservation infrastructure audit
4. Accessibility testing
5. Peer review

---

## 6. Governance

### 6.1 Standard Maintenance

This standard is maintained by the WIA Educational Standards Committee with input from:

- Museum professionals
- Archaeological experts
- Digital preservation specialists
- Technology vendors
- Indigenous representatives
- Accessibility advocates

### 6.2 Versioning

- **Major versions** (X.0.0): Significant changes, may break compatibility
- **Minor versions** (1.X.0): New features, backward compatible
- **Patches** (1.0.X): Bug fixes, clarifications

Updates published annually with community input period.

---

## 7. Related Standards

- **ISO 21127:** CIDOC Conceptual Reference Model
- **ISO 14721:** OAIS Reference Model
- **ISO 19115:** Geographic Metadata
- **ISO 16363:** Digital Repository Audit
- **WCAG 2.1:** Web accessibility
- **IIIF:** International Image Interoperability Framework
- **W3C Web Annotation:** Annotation data model

---

## 8. References

1. UNESCO Convention on Cultural Heritage (1972, 2003)
2. London Charter for Computer-based Visualization (2009)
3. Seville Principles for Virtual Archaeology (2011)
4. ICOM Code of Ethics for Museums
5. UN Declaration on the Rights of Indigenous Peoples

---

## 9. Contact

**WIA Standards Committee**
Email: standards@wiastandards.com
Web: https://wiastandards.com/cultural-heritage-digitization
GitHub: https://github.com/WIA-Official/wia-standards

---

© 2025 WIA - World Certification Industry Association
**License:** MIT
**Philosophy:** 弘益人間 · Benefit All Humanity


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
