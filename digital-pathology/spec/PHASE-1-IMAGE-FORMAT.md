# WIA-MED-008 Phase 1: Image Format Specification

## Version 1.0.0

**Status:** ✅ Complete  
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines the standard image format for digital pathology whole slide images (WSI) in the WIA-MED-008 standard.

### 1.1 Goals

- **Interoperability:** Enable seamless data exchange between systems
- **Efficiency:** Optimize storage and transmission
- **Quality:** Maintain diagnostic-grade image quality
- **Extensibility:** Support future enhancements

---

## 2. File Format

### 2.1 Container Format

**Primary Format:** WIA-DPS (WIA Digital Pathology Standard)
- Base: BigTIFF (TIFF 6.0 with 64-bit offsets)
- Extension: `.wdps` or `.tif`

**Supported Legacy Formats (Read-only):**
- DICOM Supplement 145
- OME-TIFF
- SVS (Aperio)
- NDPI (Hamamatsu)
- MRXS (3DHistech)

### 2.2 File Structure

```
WIA-DPS File Structure:
├── Header
│   ├── Magic Number: "WDPS" (4 bytes)
│   ├── Version: 1.0 (2 bytes)
│   └── Flags (2 bytes)
├── Metadata Section (JSON, gzip compressed)
├── Image Pyramid
│   ├── Level 0 (40x, 0.25 µm/pixel)
│   ├── Level 1 (20x, 0.5 µm/pixel)
│   ├── Level 2 (10x, 1.0 µm/pixel)
│   ├── Level 3 (5x, 2.0 µm/pixel)
│   ├── Level 4 (2.5x, 4.0 µm/pixel)
│   └── Level 5 (1.25x, 8.0 µm/pixel)
├── Annotations (Optional, GeoJSON)
└── AI Results (Optional)
```

---

## 3. Image Specifications

### 3.1 Resolution Requirements

| Magnification | µm/pixel | Use Case | Status |
|--------------|----------|----------|---------|
| 40x | 0.25 | Primary diagnosis | **Required** |
| 20x | 0.5 | General review | **Required** |
| 10x | 1.0 | Overview | Recommended |
| 5x | 2.0 | Navigation | Recommended |
| 2.5x | 4.0 | Thumbnail | Optional |
| 1.25x | 8.0 | Slide overview | Optional |

### 3.2 Color Space

**Standard:** sRGB IEC61966-2.1
- Color depth: 24-bit RGB (8 bits per channel)
- ICC profile: Embedded
- Calibration: Regular color calibration required

### 3.3 Tile Structure

- **Tile Size:** 256×256 pixels (standard)
- **Alternative:** 512×512 pixels (allowed)
- **Format:** JPEG, JPEG2000, or WebP
- **Quality:** 90 (JPEG scale 0-100)

---

## 4. Compression

### 4.1 Recommended Compression

| Stain Type | Compression | Quality | Rationale |
|-----------|-------------|---------|-----------|
| H&E | JPEG | 90 | High compression, diagnostic quality |
| IHC | JPEG2000 (lossless) | - | Preserve color accuracy |
| Fluorescence | Deflate/LZW | - | Quantitative analysis |
| Multispectral | JPEG2000 (lossless) | - | Spectral unmixing |

### 4.2 Compression Ratios

- **H&E:** 15:1 typical (5GB → 330MB)
- **IHC:** 3:1 typical (5GB → 1.7GB)
- **Fluorescence:** 2:1 typical

---

## 5. Metadata Schema

### 5.1 Required Metadata (JSON)

```json
{
  "wia_version": "1.0.0",
  "specimen": {
    "accession_number": "required",
    "specimen_id": "required",
    "organ": "required",
    "specimen_type": "required"
  },
  "scan": {
    "scanner_model": "required",
    "scan_date": "ISO 8601 format",
    "magnification": 40,
    "resolution_um_per_pixel": 0.25
  },
  "image": {
    "width_pixels": "integer",
    "height_pixels": "integer",
    "pyramid_levels": "integer",
    "tile_size": 256,
    "compression": "JPEG|JPEG2000|WebP",
    "color_space": "sRGB"
  },
  "quality": {
    "focus_quality_score": "0.0-1.0",
    "passed_qc": "boolean"
  }
}
```

### 5.2 Optional Metadata

- Processing information (fixation, embedding, staining)
- Clinical information (diagnosis, patient demographics)
- AI analysis results
- Annotations

---

## 6. Quality Metrics

### 6.1 Image Quality Standards

| Metric | Requirement | Measurement |
|--------|-------------|-------------|
| Focus Quality | > 90% | Brenner gradient |
| Color Accuracy | ΔE < 5 | Delta E (H&E) |
| Resolution | MTF50 > 200 lp/mm | MTF measurement |
| SNR | > 40 dB | Signal-to-noise ratio |

### 6.2 Quality Control

- **Daily:** Standard slide scan and QC check
- **Weekly:** Calibration verification
- **Monthly:** Full system calibration

---

## 7. Implementation Guidelines

### 7.1 Reading WIA-DPS Files

```python
import wia_dps

# Open WIA-DPS file
slide = wia_dps.open_slide('specimen.wdps')

# Get metadata
metadata = slide.get_metadata()
print(f"Scanner: {metadata['scan']['scanner_model']}")
print(f"Magnification: {metadata['scan']['magnification']}x")

# Read tile
level = 0  # Highest resolution
x, y = 10000, 15000
tile = slide.read_region((x, y), level, (256, 256))

# Get thumbnail
thumbnail = slide.get_thumbnail((1024, 1024))
```

### 7.2 Writing WIA-DPS Files

```python
import wia_dps

# Create new slide
writer = wia_dps.WDPSWriter('output.wdps')

# Set metadata
writer.set_metadata({
    'specimen': {'accession_number': 'S25-00123'},
    'scan': {'magnification': 40, 'resolution_um_per_pixel': 0.25}
})

# Add image data
writer.write_level(0, image_array)  # Full resolution
writer.write_level(1, downsampled_array)  # 20x

# Finalize
writer.close()
```

---

## 8. Validation

### 8.1 Compliance Testing

Tools: `wia-dps-validate`

```bash
# Validate file
$ wia-dps-validate specimen.wdps

Results:
  ✅ File structure: PASS
  ✅ Metadata schema: PASS
  ✅ Image quality: PASS (FQ: 94.5%)
  ✅ Compression: PASS (JPEG q=90)
  
  Overall: COMPLIANT
```

### 8.2 Certification

- **WIA-MED-008 Certified:** Products passing validation
- **Certification Period:** 3 years
- **Renewal:** Required before expiration

---

## 9. References

- TIFF 6.0 Specification
- DICOM Supplement 145
- OME-TIFF Specification
- sRGB Color Space Standard
- JPEG/JPEG2000 Standards

---

## 10. Appendix

### 10.1 Color Calibration Targets

- Macbeth ColorChecker
- IT8 Chart
- Standard H&E reference slides

### 10.2 Test Datasets

Available at: https://wia.live/datasets/med-008

---

**© 2025 WIA - World Certification Industry Association**  
**License:** MIT License  
**Contact:** standards@wia.live


## Annex E — Implementation Notes for PHASE-1-IMAGE-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-IMAGE-FORMAT.

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
evidence for PHASE-1-IMAGE-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-image-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-IMAGE-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-IMAGE-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-IMAGE-FORMAT.
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
for PHASE-1-IMAGE-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-IMAGE-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-IMAGE-FORMAT validation when the
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
