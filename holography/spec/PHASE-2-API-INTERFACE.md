# WIA-QUA-010 PHASE 2 — API Interface Specification

**Standard:** WIA-QUA-010
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

- `Λ` = Fringe spacing
- `θ` = Internal viewing angle

#### 4.2.3 Rainbow Holograms

Modified reflection hologram:
- Transfer process with horizontal slit
- Different colors at different viewing angles
- Common in security applications
- Viewable in white light

### 4.3 Volume Holograms

#### 4.3.1 Thickness Criterion

Volume hologram when:
```
Q = (2π × λ × d) / (n × Λ²) > 10
```

Where:
- `Q` = Klein-Cook parameter
- `d` = Hologram thickness
- `Λ` = Grating period

#### 4.3.2 Coupled Wave Theory

Diffraction efficiency for volume gratings:

```
η = sin²(π × Δn × d / (λ × cos(θ)))
```

Maximum efficiency: η = 100% when argument = π/2

#### 4.3.3 Multiplexing

Volume holograms enable multiple recordings:

- **Angular Multiplexing**: Different angles
- **Wavelength Multiplexing**: Different colors
- **Phase-Code Multiplexing**: Different phase patterns
- **Shift Multiplexing**: Different positions

Storage capacity:
```
N_max ≈ (V / λ³) × (Δn)²
```

Where `V` = volume of medium

---

## 5. Computer-Generated Holography

### 5.1 Principles

CGH creates holograms without physical objects through digital computation.

#### 5.1.1 Wavefront Calculation

For point source at (x₀, y₀, z₀):

```
O(x,y) = (A / r) × exp(ikr)
```

Where:
```
r = √[(x-x₀)² + (y-y₀)² + z₀²]
k = 2π/λ
```

For complex objects, superposition of point sources:
```
O(x,y) = Σᵢ (Aᵢ / rᵢ) × exp(ikrᵢ)
```

#### 5.1.2 Interference Pattern

```
I(x,y) = |R + O(x,y)|²
I(x,y) = |R|² + |O|² + R*O + RO*
```

Binary quantization for fabrication:
```
H(x,y) = 1 if I(x,y) > threshold
H(x,y) = 0 otherwise
```

### 5.2 Computation Methods

#### 5.2.1 Point-Source Method

Direct calculation from 3D point cloud:

**Complexity:** O(N × M)
- N = number of object points
- M = number of hologram pixels

**Memory:** Moderate
**Quality:** High fidelity
**Speed:** Slow for large objects

#### 5.2.2 Polygon-Based Method

Calculate from 3D polygonal mesh:

1. Tessellate surfaces into polygons
2. Calculate wavefront from each polygon
3. Sum contributions at hologram plane

**Complexity:** O(P × M)
- P = number of polygons

#### 5.2.3 Layer-Based Method

Decompose 3D scene into 2D layers:

```
O(x,y,z) = Σₙ Oₙ(x,y) × δ(z - zₙ)
```

Propagate each layer to hologram plane using Fresnel diffraction.

**Advantages:**
- Faster computation (2D FFT per layer)
- Parallel processing friendly
- Good for flat or layered objects

#### 5.2.4 Gerchberg-Saxton Algorithm

Iterative phase retrieval:

1. Start with random phase at hologram plane
2. Propagate to object plane (FFT)
3. Replace amplitude with target, keep phase
4. Propagate back to hologram plane (IFFT)
5. Replace amplitude with uniform, keep phase
6. Repeat until convergence

**Convergence:** Typically 10-100 iterations
**Quality:** Good phase-only holograms

### 5.3 Fabrication Methods

#### 5.3.1 E-beam Lithography

- **Resolution:** <10 nm
- **Material:** Photoresist
- **Speed:** Very slow
- **Cost:** Very high
- **Use:** Research, master patterns

#### 5.3.2 Laser Writing

- **Resolution:** 0.5-2 μm
- **Material:** Photopolymer, photoresist
- **Speed:** Moderate
- **Cost:** Moderate
- **Use:** Prototyping, HOE production

#### 5.3.3 Photolithography

- **Resolution:** 100 nm - 1 μm
- **Material:** Photoresist
- **Speed:** Fast (parallel)
- **Cost:** Low (mass production)
- **Use:** Mass production from master

---

## 6. Digital Holography

### 6.1 Recording Principles

Digital holography uses electronic cameras instead of photographic media.

#### 6.1.1 Digital Recording

```
I(m,n) = |R(m,n) + O(m,n)|²
```

Where (m,n) are pixel indices.

**Requirements:**
- Pixel size < λ/(2NA) for Nyquist sampling
- Typical: 3-10 μm pixels
- Resolution: 1-10 megapixels

#### 6.1.2 Off-Axis Configuration

Reference beam at angle to separate orders:

```
R(x,y) = A_R × exp(i × k_x × x)
```

Spatial frequency carrier: k_x = k × sin(θ)

#### 6.1.3 Phase-Shifting Digital Holography

Record multiple holograms with phase shifts:

```
I₀ = |R + O|²
I₁ = |R × exp(iπ/2) + O|²
I₂ = |R × exp(iπ) + O|²
I₃ = |R × exp(i3π/2) + O|²
```

Extract complex amplitude:
```
O = [(I₃ - I₁) + i(I₀ - I₂)] / (4R)
```

### 6.2 Numerical Reconstruction

#### 6.2.1 Fresnel Transform Method

Propagate recorded wavefront to distance z:

```
U(x,y,z) = F⁻¹{F[U(x,y,0)] × exp(i × k × z × √[1 - (λf_x)² - (λf_y)²])}
```

Where F denotes Fourier transform.

#### 6.2.2 Angular Spectrum Method

More accurate for short distances:

```
U(x,y,z) = F⁻¹{F[U(x,y,0)] × H(f_x, f_y, z)}
```

Transfer function:
```
H(f_x, f_y, z) = exp(i × 2π × z × √[(n/λ)² - f_x² - f_y²])
```

#### 6.2.3 Focus Detection

Auto-focus by maximizing sharpness metric:

```
S(z) = ∬ |∇U(x,y,z)|² dx dy
```

Optimal focus at z where S(z) is maximum.

### 6.3 Applications

#### 6.3.1 Digital Holographic Microscopy (DHM)

- **Resolution:** Sub-micrometer
- **Field of view:** 100 μm - 1 mm
- **Contrast:** Quantitative phase imaging
- **Applications:** Cell imaging, particle analysis

#### 6.3.2 Holographic Interferometry

- **Sensitivity:** λ/100 displacement
- **Applications:** Vibration analysis, stress measurement
- **Method:** Subtract phase maps from different states

```
Δφ(x,y) = φ₂(x,y) - φ₁(x,y)
```

#### 6.3.3 Digital Particle Holography

Track 3D positions of particles in volume:

1. Record hologram of particle field
2. Reconstruct at multiple depths
3. Detect particle positions (focus metrics)
4. Track particles over time

**Applications:** Fluid dynamics, aerosols, plankton

---

## 7. Holographic Displays

### 7.1 Display Principles

True 3D display without glasses using wavefront reconstruction.

#### 7.1.1 Spatial Light Modulators (SLM)

**LCD-based SLM:**
- Resolution: 1920×1080 to 3840×2160
- Pixel pitch: 3-8 μm
- Refresh rate: 60-240 Hz
- Modulation: Phase or amplitude

**LCOS (Liquid Crystal on Silicon):**
- Higher resolution: 4096×4096
- Smaller pixels: 1-3 μm
- Higher fill factor: >90%
- Better contrast

**DMD (Digital Micromirror Device):**
- Binary amplitude modulation
- Very high speed: >20 kHz
- Good for time-multiplexing
- Limited diffraction efficiency

#### 7.1.2 Field of View

```
FOV = 2 × arctan(D / 2f)
```

Where:
- D = Display aperture
- f = Focal length to viewer

#### 7.1.3 Viewing Angle

Maximum angle determined by pixel pitch:

```
θ_max = arcsin(λ / 2p)
```

Where p = pixel pitch.

For p = 8 μm, λ = 532 nm:
```
θ_max = arcsin(532e-9 / 16e-6) ≈ 1.9°
```

**Solution:** Multiple SLMs or enlarged pupils

### 7.2 Color Holographic Displays

#### 7.2.1 Time-Sequential Color

Display R, G, B holograms sequentially:
- Requires 3× refresh rate
- Single SLM
- Simpler optics
- Potential color breakup


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
