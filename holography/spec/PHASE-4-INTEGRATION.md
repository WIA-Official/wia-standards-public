# WIA-QUA-010 PHASE 4 — Integration Specification

**Standard:** WIA-QUA-010
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

### 10.3 Holographic Tomography

#### 10.3.1 Optical Diffraction Tomography (ODT)

Record holograms at multiple angles:

1. Rotate sample or illumination angle
2. Reconstruct complex amplitude for each angle
3. Compute 3D refractive index distribution

**Reconstruction:**
```
n(x,y,z) = n₀ + Δn(x,y,z)
```

Use filtered back-projection or iterative algorithms.

#### 10.3.2 3D Refractive Index Imaging

**Resolution:** 200-500 nm lateral, 500-1000 nm axial
**Field of view:** 50-200 μm
**Refractive index accuracy:** Δn ≈ 0.001

**Applications:**
- 3D cell structure
- Organelle imaging
- Tissue analysis
- Disease diagnosis

### 10.4 Dental Holography

#### 10.4.1 Tooth Surface Profiling

Holographic interferometry for:
- Cavity detection
- Enamel erosion
- Crown fitting
- Orthodontic planning

**Sensitivity:** 0.1-1 μm surface change

#### 10.4.2 Bite Analysis

3D holographic recording of:
- Occlusion patterns
- Jaw movement
- Temporomandibular joint (TMJ) function

### 10.5 Surgical Planning

#### 10.5.1 Holographic Visualization

Display patient data as 3D hologram:
- CT/MRI data conversion
- Organ structures
- Blood vessels
- Tumors

**Benefits:**
- Better spatial understanding
- Improved planning
- Training and education
- Patient communication

#### 10.5.2 Intraoperative Guidance

Real-time holographic overlay:
- Augmented reality guidance
- Navigation systems
- Hands-free visualization
- Reduced surgical time

---

## 11. Implementation Guidelines

### 11.1 Required Components

Any WIA-QUA-010 compliant system must include:

1. **Hologram Recorder**: Interference pattern capture
2. **Reconstruction Engine**: Wavefront regeneration
3. **CGH Generator**: Computer hologram synthesis
4. **Quality Analyzer**: Performance metrics
5. **Format Converter**: Interoperability support

### 11.2 API Interface

#### 11.2.1 Record Hologram

```typescript
interface RecordingParams {
  wavelength: number;        // meters
  objectBeam: WaveData;
  referenceBeam: WaveData;
  medium: RecordingMedium;
  exposureTime?: number;     // seconds
  temperature?: number;      // Celsius
}

interface HologramData {
  id: string;
  interferencePattern: number[][];
  spatialFrequency: number;  // lines/mm
  efficiency: number;        // percent
  medium: RecordingMedium;
  timestamp: Date;
}
```

#### 11.2.2 Reconstruct Hologram

```typescript
interface ReconstructionParams {
  hologram: HologramData;
  reconstructionBeam: WaveData;
  wavelength: number;
  distance?: number;         // reconstruction distance
}

interface ReconstructedWave {
  amplitude: number[][];
  phase: number[][];
  intensity: number[][];
  quality: QualityMetrics;
}
```

#### 11.2.3 Generate CGH

```typescript
interface CGHParams {
  scene: Scene3D;
  wavelength: number;
  resolution: { width: number; height: number };
  method: 'point-source' | 'polygon' | 'layer' | 'gerchberg-saxton';
  iterations?: number;
}

interface ComputedHologram {
  pattern: number[][];
  phase: number[][];
  computationTime: number;   // milliseconds
  method: string;
}
```

### 11.3 Data Formats

#### 11.3.1 Hologram File Format

```json
{
  "standard": "WIA-QUA-010",
  "version": "1.0",
  "hologram": {
    "type": "transmission" | "reflection" | "volume" | "cgh",
    "wavelength": 532e-9,
    "dimensions": {
      "width": 4096,
      "height": 4096,
      "thickness": 10e-6
    },
    "data": {
      "amplitude": "base64_encoded_data",
      "phase": "base64_encoded_data"
    },
    "metadata": {
      "created": "2025-12-26T00:00:00Z",
      "medium": "photopolymer",
      "recording_geometry": {
        "reference_angle": 30,
        "wavelength": 532e-9
      }
    }
  }
}
```

#### 11.3.2 Reconstruction Parameters

```json
{
  "reconstruction": {
    "wavelength": 532e-9,
    "distance": 0.5,
    "method": "fresnel" | "angular-spectrum" | "rayleigh-sommerfeld",
    "reference_beam": {
      "angle": 30,
      "amplitude": 1.0,
      "phase": 0
    }
  }
}
```

### 11.4 Quality Metrics

#### 11.4.1 Diffraction Efficiency

```
η = (P_diffracted / P_incident) × 100%
```

**Target:** >30% for transmission, >50% for reflection

#### 11.4.2 Signal-to-Noise Ratio (SNR)

```
SNR = 10 × log₁₀(P_signal / P_noise)
```

**Target:** >20 dB for good quality

#### 11.4.3 Spatial Resolution

```
Resolution = 1 / f_spatial_max
```

**Target:** <1 μm for high-quality holograms

#### 11.4.4 Reconstruction Fidelity

Mean Squared Error between original and reconstructed:

```
MSE = (1/N) × Σ(I_original - I_reconstructed)²
```

**Target:** MSE < 0.01 (normalized)

### 11.5 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| H001 | Insufficient coherence | Use laser source |
| H002 | Underexposure | Increase exposure time |
| H003 | Overexposure | Decrease exposure or power |
| H004 | Poor fringe visibility | Check beam ratio |
| H005 | Medium degradation | Replace recording medium |
| H006 | Phase unwrap failure | Use phase-shifting method |
| H007 | Computation timeout | Reduce resolution or points |

---

## 12. References

### 12.1 Foundational Papers

1. Gabor, D. (1948). "A New Microscopic Principle" - Nobel Prize work
2. Leith, E. & Upatnieks, J. (1962). "Reconstructed Wavefronts and Communication Theory"
3. Denisyuk, Y. (1962). "On the Reproduction of the Optical Properties of an Object by the Wave Field of its Scattered Radiation"
4. Benton, S. (1969). "Hologram Reconstructions with Extended Incoherent Sources" - Rainbow holograms

### 12.2 Key Textbooks

1. Hariharan, P. "Optical Holography" (Cambridge University Press)
2. Goodman, J.W. "Introduction to Fourier Optics" (Roberts & Company)
3. Schnars, U. & Jüptner, W. "Digital Holography" (Springer)
4. Poon, T. & Liu, J. "Introduction to Modern Digital Holography" (Cambridge)

### 12.3 Standards Organizations

- ISO/TC 172/SC 9 - Laser and electro-optical systems
- SPIE - International Society for Optics and Photonics
- OSA - Optical Society of America
- IEEE Holography Standards Committee

### 12.4 WIA Standards

- WIA-INTENT: Intent-based holographic creation
- WIA-OMNI-API: Universal holography API
- WIA-QUANTUM: Quantum holographic computing
- WIA-VISUAL: Visual data representation

---

## Appendix A: Calculation Examples

### A.1 Interference Fringe Spacing

```
Given:
- Wavelength: λ = 532 nm (green laser)
- Angle between beams: θ = 60°

Calculation:
d = λ / (2 × sin(θ/2))
d = 532e-9 / (2 × sin(30°))
d = 532e-9 / 1.0
d = 532 nm

Spatial frequency:
f = 1/d = 1/(532e-9 m) = 1,880,000 lines/m
f ≈ 1,880 lines/mm
```

### A.2 Diffraction Efficiency

```
Given:
- Hologram thickness: d = 15 μm
- Refractive index modulation: Δn = 0.03
- Wavelength: λ = 532 nm
- Bragg angle: θ = 30°

Calculation (volume hologram):
η = sin²(π × Δn × d / (λ × cos(θ)))
η = sin²(π × 0.03 × 15e-6 / (532e-9 × cos(30°)))
η = sin²(3.07)
η ≈ 0.03 = 3%

Note: For maximum efficiency, argument should be π/2
Optimal Δn = (λ × cos(θ)) / (2d)
Optimal Δn = (532e-9 × 0.866) / (2 × 15e-6)
Optimal Δn ≈ 0.015
```

### A.3 Storage Capacity

```
Given:
- Medium volume: 1 cm³
- Wavelength: λ = 532 nm
- Dynamic range: M = 5

Theoretical capacity:
C = (V / λ³) × M²
C = (10⁻⁶ m³ / (532e-9)³) × 25
C ≈ 1.66 × 10¹⁴ bits
C ≈ 20.75 TB

Practical (10% of theoretical):
C_practical ≈ 2 TB
```

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA-QUA-010 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


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

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
