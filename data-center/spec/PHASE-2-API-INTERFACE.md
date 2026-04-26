# WIA-COMM-013 PHASE 2 — API Interface Specification

**Standard:** WIA-COMM-013
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 4. Cooling Systems

### 4.1 PUE (Power Usage Effectiveness)

**Definition:**
```
PUE = Total Facility Power / IT Equipment Power

Example:
Total Facility Power: 1200 kW
IT Equipment Power: 1000 kW
PUE = 1200 / 1000 = 1.2
```

**Industry Benchmarks:**
- **Legacy Data Centers**: PUE 1.8-2.0
- **Typical Enterprise**: PUE 1.5-1.7
- **Modern Efficient**: PUE 1.2-1.4
- **Hyperscale Best Practice**: PUE 1.1-1.15
- **Theoretical Minimum**: PUE 1.0

**Google Fleet Average (2024)**: PUE 1.10
**Facebook/Meta Average**: PUE 1.09

### 4.2 Cooling Technologies

#### 4.2.1 CRAC (Computer Room Air Conditioner)

```
Specifications:
- Capacity: 10-30 tons (35-105 kW)
- Technology: Direct expansion (DX)
- Refrigerant: R410A, R134a, or eco-friendly
- COP (Coefficient of Performance): 2.5-3.5
- Layout: Perimeter or in-row placement
- Control: Thermostat or BMS integration

Advantages:
+ Self-contained system
+ Easier installation
+ Lower upfront cost

Disadvantages:
- Lower efficiency than CRAH
- Refrigerant leaks
- Shorter lifespan
```

#### 4.2.2 CRAH (Computer Room Air Handler)

```
Specifications:
- Capacity: 20-100 tons (70-350 kW)
- Technology: Chilled water
- Water temp: 7-12°C (45-54°F)
- COP: 3.5-5.0
- Fan control: Variable speed (VFD)

Advantages:
+ Higher efficiency
+ Centralized chiller plant
+ Better scalability

Disadvantages:
- Requires chilled water infrastructure
- Higher upfront cost
- More complex piping
```

#### 4.2.3 In-Row Cooling

```
Placement: Between server racks
Capacity: 20-40 kW per unit
Ideal for: High-density racks (10-30 kW)

Benefits:
+ Localized cooling
+ Shorter airflow path
+ Better hot spot management
+ Supports higher rack densities
```

#### 4.2.4 Liquid Cooling

**Direct-to-Chip (Cold Plate):**
```
Working Principle:
- Liquid flows through cold plates on CPUs/GPUs
- Heat transferred to liquid
- Liquid cooled in heat exchanger
- Recirculated

Specifications:
- Coolant: Water, glycol, or dielectric fluid
- Temperature: 25-40°C
- Rack density: 50-100 kW
- PUE improvement: 1.05-1.15

Use Cases:
- HPC (High-Performance Computing)
- AI/ML training
- GPU clusters
```

**Immersion Cooling:**
```
Types:
1. Single-phase: Servers submerged, fluid doesn't boil
2. Two-phase: Fluid boils, vapor condenses

Specifications:
- Dielectric fluid (non-conductive)
- Rack density: 100-250 kW
- PUE: <1.05
- No fans required

Advantages:
+ Highest density
+ Ultra-low PUE
+ Reduced noise
+ Better reliability (no dust)

Challenges:
- High upfront cost
- Maintenance complexity
- Limited vendor support
```

### 4.3 Airflow Management

**Hot Aisle / Cold Aisle Configuration:**
```
Layout:
┌─────────┐  ┌─────────┐  ┌─────────┐
│  Rack   │  │  Rack   │  │  Rack   │
│ ◄─────  │  │ ◄─────  │  │ ◄─────  │  Cold Aisle
└─────────┘  └─────────┘  └─────────┘
     ▲            ▲            ▲
     │            │            │
     │   Hot Aisle (Contained) │
     │            │            │
     ▼            ▼            ▼
┌─────────┐  ┌─────────┐  ┌─────────┐
│  ─────► │  │  ─────► │  │  ─────► │
│  Rack   │  │  Rack   │  │  Rack   │
└─────────┘  └─────────┘  └─────────┘

Containment Options:
1. Cold Aisle Containment (CAC): Enclose cold aisle
2. Hot Aisle Containment (HAC): Enclose hot aisle (more efficient)
3. Chimney/Duct: Exhaust heat directly to return
```

**Best Practices:**
- **Rack fronts** face cold aisle
- **Blanking panels** fill empty rack spaces
- **Cable management** avoid blocking airflow
- **Floor tiles** perforate in cold aisle only (60% open area)
- **Ceiling height** 10-12 feet minimum
- **Raised floor** 24-36 inches for optimal airflow

### 4.4 Free Cooling (Economizers)

**Air-Side Economizer:**
```
Use outside air when ambient < 15°C (59°F)
PUE reduction: 0.1-0.3
Annual savings: 30-60% cooling energy
```

**Water-Side Economizer:**
```
Use cooling tower water directly
Wet-bulb temp < 10°C (50°F)
PUE reduction: 0.1-0.2
More reliable than air-side
```

---

## 5. Network Architecture

### 5.1 Spine-Leaf Topology

```
Modern Standard Architecture:

        [Core/Spine Layer]
      ┌──────┬──────┬──────┐
      │Spine1│Spine2│Spine3│ (40/100G uplinks)
      └──┬───┴───┬──┴───┬──┘
         │       │      │
    ┌────┴───────┴──────┴────┐
    │                         │
[Leaf/ToR Layer]         [Leaf/ToR Layer]
┌─────┬─────┬─────┐      ┌─────┬─────┐
│Leaf1│Leaf2│Leaf3│      │Leaf4│Leaf5│ (10/25G downlinks)
└──┬──┴──┬──┴──┬──┘      └──┬──┴──┬──┘
   │     │     │            │     │
 Racks Racks Racks        Racks Racks

Characteristics:
✓ Non-blocking bandwidth
✓ Predictable latency
✓ East-west traffic optimization
✓ Horizontal scalability
✓ No spanning tree required
```

### 5.2 Network Equipment

**Top-of-Rack (ToR) Switches:**
- **Ports**: 48-64 × 10/25GbE downlinks
- **Uplinks**: 4-8 × 40/100GbE to spine
- **Redundancy**: Dual switches per rack (Tier III+)
- **Protocols**: BGP, VXLAN, EVPN

**Spine Switches:**
- **Ports**: 32-64 × 40/100/400GbE
- **Capacity**: 12.8-25.6 Tbps
- **Redundancy**: N+1 or 2N configuration
- **Buffers**: Deep buffers for incast

**Core Routers:**
- **Capacity**: 100+ Tbps
- **Uplinks**: Multiple 100/400GbE WAN
- **Redundancy**: Geographic diversity
- **Features**: DDoS protection, traffic shaping

### 5.3 Cabling Standards

| Type | Use Case | Distance | Bandwidth |
|------|----------|----------|-----------|
| Cat6A | Server to ToR | 100m | 10GbE |
| Cat8 | High-density ToR | 30m | 25/40GbE |
| Single-Mode Fiber | Rack to spine | 10km+ | 100GbE+ |
| Multi-Mode Fiber (OM4) | Rack to spine | 550m | 100GbE |
| DAC (Direct Attach Copper) | In-rack | 7m | 100GbE |

### 5.4 Software-Defined Networking (SDN)

**Components:**
- **Controller**: Centralized network management (OpenFlow, NETCONF)
- **Orchestration**: Automated provisioning
- **Telemetry**: Real-time monitoring
- **Automation**: Intent-based networking

**Benefits:**
- Faster deployment
- Network virtualization
- Improved troubleshooting
- Better resource utilization

---

## 6. Rack Design and Layout

### 6.1 Standard Rack Specifications

```
Standard 19-inch Rack:
- Height: 42U-48U
- Width: 19 inches (482.6 mm)
- Depth: 36-48 inches (914-1219 mm)
- Weight capacity: 1000-3000 lbs (450-1360 kg)
- Power: 2-4 PDUs per rack
```

### 6.2 Rack Density Classifications

| Density Class | Power per Rack | Typical Workload |
|---------------|----------------|------------------|
| Low | 2-5 kW | Storage, networking |
| Medium | 5-10 kW | General servers |
| High | 10-20 kW | Virtualization, database |
| Very High | 20-30 kW | GPU, HPC |
| Ultra-High | 30-100 kW | AI/ML, liquid-cooled |

### 6.3 Floor Layout Planning

**Capacity Calculation:**
```
Total Raised Floor Area: 10,000 sq ft
Racks: 40 (arranged in 5 rows × 8 racks)
Rack footprint: 2 ft × 4 ft = 8 sq ft
Aisle width: 4 ft (cold), 5 ft (hot)
Support space: 30% (power, cooling, walkways)

Usable area = 10,000 × 0.7 = 7,000 sq ft
Rack area = 40 × 8 = 320 sq ft
Utilization = 320 / 7,000 = 4.6%
```

**Best Practices:**
- **Cold aisle width**: 3-4 feet
- **Hot aisle width**: 4-5 feet
- **Equipment spacing**: 6-inch clearance
- **Emergency exits**: <200 feet from any point
- **Loading dock**: Direct access to data hall

---


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
