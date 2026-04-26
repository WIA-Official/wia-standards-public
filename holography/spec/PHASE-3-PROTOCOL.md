# WIA-QUA-010 PHASE 3 — Protocol Specification

**Standard:** WIA-QUA-010
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

#### 7.2.2 Spatial Color Multiplexing

Separate SLMs or color filters:
- Simultaneous color
- More complex optics
- No flicker
- Reduced resolution per color

#### 7.2.3 Wavelength Multiplexing

Single SLM with multiple lasers:
- Requires wavelength-selective elements
- Compact system
- Alignment critical
- Chromatic aberrations

### 7.3 Computational Requirements

#### 7.3.1 CGH Calculation Speed

For real-time display (30 fps):
```
Time_per_frame < 33 ms
```

For 4K hologram (4096×4096 pixels) with 100k object points:
```
Operations ≈ 4096² × 100k ≈ 1.7 × 10¹²
```

**Required:** >50 TFLOPS for real-time

**Solutions:**
- GPU acceleration
- Lookup tables
- Novel algorithms (Fresnel, layer-based)
- Dedicated hardware (FPGA, ASIC)

#### 7.3.2 Data Throughput

For 4K SLM at 60 Hz with 8-bit phase:
```
Data rate = 4096 × 4096 × 8 bits × 60 Hz
         = 8 Gbps
```

**Interface:** HDMI 2.1, DisplayPort 2.0, or custom

---

## 8. Data Storage

### 8.1 Holographic Data Storage (HDS)

#### 8.1.1 Storage Principle

Record data pages as holograms in volume medium.

**Data page:** 2D array of bits (e.g., 1024×1024)
**Multiplexing:** Store many pages in same volume
**Parallelism:** Read/write entire page at once

#### 8.1.2 Storage Capacity

Theoretical maximum:

```
C_max = (V / λ³) × M²
```

Where:
- V = Material volume
- λ = Wavelength
- M = Dynamic range (Δn)

For 1 cm³ volume, λ = 532 nm, M = 5:
```
C_max ≈ 1.6 × 10¹⁵ bits ≈ 200 TB
```

**Practical capacity:** 1-10% of theoretical (2-20 TB)

#### 8.1.3 Multiplexing Methods

**Angular Multiplexing:**
```
N_angular ≈ Δθ_total / Δθ_selectivity
N_angular ≈ (90°) / (0.01°) ≈ 9000 holograms
```

**Wavelength Multiplexing:**
```
N_wavelength ≈ Δλ_total / Δλ_selectivity
```

**Spatial Multiplexing:**
- Shift medium between recordings
- Limited by beam size and medium size

**Phase-Code Multiplexing:**
- Different random phase patterns
- Orthogonal codes

### 8.2 Data Encoding

#### 8.2.1 Modulation Codes

**Binary:** Direct 0/1 encoding
**Gray code:** Reduce errors
**Run-length limited (RLL):** Prevent long runs
**Error correction:** Reed-Solomon, LDPC

#### 8.2.2 Data Page Format

```
[Sync pattern | User data | ECC | Alignment marks]
```

Typical page:
- 1024×1024 pixels
- 800×800 data bits (640 kbits)
- 200×200 ECC bits
- Sync and alignment markers

#### 8.2.3 Error Correction

Raw BER (Bit Error Rate): 10⁻³ to 10⁻⁴
After ECC: <10⁻¹² (target)

**Reed-Solomon (RS):**
```
RS(255, 239) → 16 bytes correction per 255
```

### 8.3 Read/Write Performance

#### 8.3.1 Write Speed

```
Write_speed = Page_size × Multiplexing × Refresh_rate
```

Example:
- Page: 640 kbits
- 1000 pages/location
- 10 ms per page

```
Speed = 640 kbits × 1000 / (10 ms × 1000)
      = 64 Mbps
```

#### 8.3.2 Read Speed

Faster than write (no photochemistry):

```
Read_speed = Page_size × Frame_rate
```

Example:
- Page: 640 kbits
- Camera: 1000 fps

```
Speed = 640 kbits × 1000 fps = 640 Mbps
```

#### 8.3.3 Access Time

Random access to any page:
- Angular repositioning: 1-10 ms
- SLM update: 10-100 ms
- Total: 10-100 ms

**Faster than HDD, slower than SSD**

---

## 9. Security Holograms

### 9.1 Authentication Features

#### 9.1.1 Visual Features

**Rainbow Effect:**
- Different colors at different angles
- Difficult to reproduce
- Visible in white light

**3D Imagery:**
- Depth perception
- Parallax effect
- Multiple viewing positions

**Microtext:**
- Sub-millimeter text in hologram
- Requires magnification
- Hard to counterfeit

**Hidden Images:**
- Visible only at specific angles
- Multiple image planes
- Switchable images

#### 9.1.2 Machine-Readable Features

**Optical Variable Devices (OVD):**
- Specific spectral signatures
- Measurable diffraction patterns
- Unique optical response

**Encrypted Data:**
- Holographically encoded information
- Readable only with correct key/angle
- Digital signature integration

### 9.2 Production Security

#### 9.2.1 Origination

Master hologram creation:
- Requires specialized equipment
- Controlled environment
- Secure facility
- Limited access

#### 9.2.2 Replication

**Embossing:**
1. Create metal shim from master
2. Hot emboss into plastic film
3. Metallize surface
4. Laminate or apply adhesive

**Volume:** >1 million copies/hour
**Cost:** <$0.01 per hologram

#### 9.2.3 Tamper Evidence

- Hologram destruction on removal
- Void patterns appear
- Non-transferable designs
- Fragile substrates

### 9.3 Applications

#### 9.3.1 Currency & Documents

- Banknotes
- Passports
- ID cards
- Certificates
- Licenses

#### 9.3.2 Product Protection

- Pharmaceutical packaging
- Electronics
- Luxury goods
- Software
- Tickets

#### 9.3.3 Brand Security

- Product labels
- Packaging
- Promotional materials
- Warranty seals

---

## 10. Medical Applications

### 10.1 Holographic Microscopy

#### 10.1.1 Digital Holographic Microscopy (DHM)

**Advantages over conventional microscopy:**
- Quantitative phase imaging
- 3D information from single shot
- Extended depth of field
- Numerical refocusing

**Phase sensitivity:**
```
Δφ_min ≈ 2π × (OPD / λ)
```

For OPD (Optical Path Difference) = 1 nm, λ = 532 nm:
```
Δφ ≈ 0.012 radians (0.7°)
```

#### 10.1.2 Cell Imaging

**Label-free imaging:**
- No fluorescent dyes needed
- Reduced phototoxicity
- Long-term observation
- Natural cell behavior

**Measurable parameters:**
- Cell volume and dry mass
- Refractive index
- Membrane fluctuations
- Morphology changes

```
Dry mass = (λ / 2πα) × ∬ Δφ(x,y) dx dy
```

Where α = specific refractive increment (≈0.18 mL/g for proteins)

#### 10.1.3 Live Cell Dynamics

Track cellular processes:
- Cell division
- Migration
- Deformation
- Response to drugs

**Temporal resolution:** 1-1000 fps
**Spatial resolution:** 200 nm - 1 μm

### 10.2 Holographic Endoscopy

#### 10.2.1 Fiber Bundle Holography

Transmit holographic information through fiber:
- Coherent fiber bundle
- Each fiber as sampling point
- Reconstruct hologram numerically
- Miniature endoscope possible

**Diameter:** 1-3 mm
**Working distance:** 5-50 mm
**Resolution:** 5-20 μm

#### 10.2.2 Applications

- Gastrointestinal imaging
- Bronchoscopy
- Minimally invasive surgery
- In vivo diagnostics


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
