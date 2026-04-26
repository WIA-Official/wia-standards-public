# WIA-QUA-016 PHASE 3 — Protocol Specification

**Standard:** WIA-QUA-016
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 8. Network Topology

### 8.1 FTL Network Architectures

#### 8.1.1 Point-to-Point

**Direct Links**:

Each pair of nodes has dedicated FTL channel:
- Simplest architecture
- Highest performance per link
- Scales poorly: N(N-1)/2 links for N nodes
- Best for small, critical networks

**Use Case**: Emergency communication between key locations.

#### 8.1.2 Hub-and-Spoke

**Central Relay**:

```
Nodes → Hub → Nodes
```

**Properties**:
- N-1 links for N nodes (including hub)
- Single point of failure (hub)
- Simplified routing
- Lower total energy than full mesh

**Use Case**: Planetary system communication (star = hub, planets = spokes).

#### 8.1.3 Mesh Network

**Interconnected Nodes**:

Each node connects to multiple (but not all) others:
- Redundant paths
- Fault tolerant
- Moderate complexity
- Scalable

**Degree**: k = average connections per node (typically k = 3-6)

#### 8.1.4 Hierarchical

**Multi-Tier**:

```
Tier 1: Interstellar FTL backbone
Tier 2: Inter-system FTL
Tier 3: Planetary FTL
Tier 4: Local conventional
```

**Advantages**:
- Efficient for different distance scales
- Manageable complexity
- Optimized resource allocation

### 8.2 Routing Protocols

#### 8.2.1 Dijkstra's Algorithm (Shortest Path)

**Cost Metric**:

For FTL network, cost might be:
- Energy requirement
- Setup time
- Reliability (inverse)
- Signal degradation

**Algorithm**:
1. Initialize: distance[source] = 0, distance[all others] = ∞
2. Unvisited = all nodes
3. Current = node with minimum distance in unvisited
4. For each neighbor: update distance if shorter path found
5. Mark current as visited
6. Repeat until destination visited

**Result**: Optimal path from source to destination.

#### 8.2.2 Load Balancing

**Dynamic Routing**:

Distribute traffic to avoid congestion:
- Monitor link utilization
- Route new requests to underutilized paths
- Periodic rebalancing

**Metrics**:
```
Load_i = current_traffic_i / capacity_i
```

Prefer paths with min(Load).

#### 8.2.3 Causality-Aware Routing

**Temporal Constraints**:

For FTL network:
- Track event timestamps
- Ensure causal consistency
- Block routes creating CTCs

**Constraint**:
```
For path A → B → C:
t_send(A) < t_receive(B) < t_send(B) < t_receive(C)
(in preferred reference frame)
```

If violated, reject route.

### 8.3 Network Synchronization

#### 8.3.1 Time Coordination

**Challenge**: FTL breaks conventional time synchronization.

**Solutions**:

1. **Preferred Frame Time**:
   - Universal absolute time in preferred frame
   - All nodes synchronize to this
   - Violates relativity but prevents paradoxes

2. **Causal Ordering**:
   - Use Lamport timestamps
   - Partial ordering of events
   - Event A → Event B if A causally affects B

3. **Network Time Protocol (FTL-NTP)**:
   - Designate master clock
   - Periodic FTL sync signals
   - Maintain global time with ps precision

#### 8.3.2 Synchronization Protocol

```
1. Master broadcasts time beacon t_master
2. Node receives at local time t_local
3. Calculate offset: Δt = t_master - t_local
4. Adjust local clock
5. Repeat periodically (e.g., every 1 hour)
```

**Precision**: Limited by FTL signal timing jitter.

### 8.4 Scalability Analysis

#### 8.4.1 Network Growth

**Nodes vs. Links**:

| Topology | Links | Scalability |
|----------|-------|-------------|
| Full Mesh | N(N-1)/2 | Poor: O(N²) |
| Ring | N | Good: O(N) |
| Star | N-1 | Good: O(N) |
| Mesh (k=const) | kN/2 | Excellent: O(N) |
| Hierarchical | O(N log N) | Very Good |

#### 8.4.2 Routing Table Size

**Full Routing Table**:
- Size: O(N) entries per node
- Update cost: O(E) where E = edges

**Hierarchical Addressing**:
- Size: O(log N) per node
- Scales to galactic networks

---

## 9. Energy Requirements

### 9.1 Tachyonic Communication Energy

### 9.1.1 Field Generation

**Tachyon Emission**:

Energy to create tachyon with momentum p:
```
E_tachyon = |m|c² / √(v²/c² - 1)
```

For v >> c:
```
E_tachyon ≈ |m|c³ / v
```

**Low energy at high velocity** (opposite of normal matter).

**Total Power**:
```
P = n × E_tachyon × f
```

Where:
- n = tachyons per pulse
- f = repetition rate (Hz)

#### 9.1.2 Modulation Energy

**Signal Power**:
```
P_signal = P_carrier + P_modulation
```

**Example**:
- |m| = 1 GeV/c² (tachyon mass)
- v = 10c
- f = 1 GHz
- n = 10⁶ per pulse

```
P ≈ 10⁶ × (10⁹ eV) / 10 × 10⁹ Hz ≈ 10 MW
```

Manageable for large installation.

**Challenge**: Creating tachyons in the first place (unknown mechanism).

### 9.2 Alcubierre Metric Energy

#### 9.2.1 Negative Energy Requirement

**Original Calculation** (Alcubierre, 1994):

For bubble radius R = 100 m, warp factor v_s = 10c:
```
E_total ≈ -10⁴⁵ J ≈ -10⁸ × M_☉ c²
```

(100 million solar masses in exotic matter!)

#### 9.2.2 Optimized Metrics

**Van Den Broeck (1999)**:

Modify bubble shape to reduce energy:
- Internal space: 200 m³ (usable volume)
- External footprint: 10⁻³² m (microscopic)

**Energy Reduction**:
```
E_optimized ≈ -10²⁹ J ≈ -10⁻⁶ M_☉ c²
```

(Few solar masses - still enormous!)

**Communication Application**:

For small signal bubble (R = 1 m):
```
E_signal ≈ -10²⁶ J ≈ -1000 kg c²
```

(Still requires tons of exotic matter per signal)

#### 9.2.3 Exotic Matter Production

**Casimir Effect**:

Negative energy density:
```
ρ = -π²ħc / (720 d⁴)
```

Where d = plate separation.

**Example**:
- d = 1 nm
```
ρ ≈ -10⁻³ J/m³
```

**27 orders of magnitude too small** for Alcubierre drive.

**Other Candidates**:
- Squeezed vacuum states: ρ < 0 locally, but small
- Quantum inequalities: Limit negative energy accumulation
- No known mechanism for required scale

### 9.3 Subspace Communication Energy

#### 9.3.1 Dimensional Transition

**Energy to Access Bulk**:

For extra dimensions at scale R:
```
E_access ≈ ℏc / R
```

**Planck Scale** (R ≈ 10⁻³⁵ m):
```
E_access ≈ 10¹⁹ GeV = 1.6 × 10⁹ J per particle
```

(Billion joules to send single subatomic particle into bulk!)

**TeV Scale** (R ≈ 10⁻¹⁸ m):
```
E_access ≈ 1 TeV = 1.6 × 10⁻⁷ J per particle
```

(More manageable but still requires particle accelerator)

#### 9.3.2 Signal Coupling

**Kaluza-Klein Coupling**:
```
g_KK ≈ (E / M_Planck)^(d/2)
```

For d = 6 extra dimensions:
```
g_KK ≈ (10¹² eV / 10¹⁹ eV)³ ≈ 10⁻²¹
```

**Extremely weak coupling** requires high signal power.

**Required Power**:

To transmit 1 Mbps at 1 AU distance:
```
P ≈ (4π AU²) × (ℏω / g_KK²) × (1 Mbps)
```

Estimate:
```
P ≈ 10²⁰ W = 100 billion gigawatts
```

(Total solar output ≈ 10²⁶ W)

### 9.4 Energy Efficiency Comparison

| Method | Energy per Bit | Energy per Second (1 Gbps) | Feasibility |
|--------|----------------|---------------------------|-------------|
| Conventional Radio | 10⁻¹⁸ J | 10⁻⁹ W | ✓ Practical |
| Laser Communication | 10⁻¹⁶ J | 10⁻⁷ W | ✓ Current Tech |
| Quantum Teleportation | 10⁻¹⁵ J | 10⁻⁶ W | ⚠ Research |
| Tachyonic (theoretical) | 10⁻¹⁰ J (?) | 10⁻¹ W (?) | ✗ Unknown |
| Alcubierre | 10²⁶ J | 10³⁵ W | ✗ Impossible |
| Subspace | 10¹⁵ J | 10²⁴ W | ✗ Impossible |

---

## 10. Error Correction and Signal Integrity

### 10.1 FTL Channel Noise Model

#### 10.1.1 Noise Sources

**Quantum Fluctuations**:
- Vacuum fluctuations
- Zero-point energy
- Heisenberg uncertainty

**Metric Distortions**:
- Gravitational waves
- Spacetime curvature variations
- Tidal forces

**Bulk Interactions** (subspace):
- Scattering from bulk matter
- Higher-dimensional turbulence
- Brane vibrations

**Tachyonic Dispersion**:
- Different velocities for different energies
- Pulse spreading
- Intersymbol interference

#### 10.1.2 Bit Error Rate (BER)

**Model**:
```
BER = ½ erfc(√[SNR/2])
```

Where:
```
SNR = P_signal / (N₀ × B)
```

- P_signal = received power
- N₀ = noise spectral density
- B = bandwidth

**Example**:
- SNR = 10 dB (factor of 10)
```
BER ≈ 4 × 10⁻³ (0.4% errors)
```

Requires error correction.

### 10.2 Classical Error Correction

#### 10.2.1 Block Codes

**Hamming (7,4) Code**:
- 4 data bits + 3 parity bits
- Corrects 1-bit errors
- Detects 2-bit errors
- Code rate: 4/7 ≈ 57%

**Reed-Solomon Codes**:
- Widely used in communications
- Symbol-based (not bit-based)
- Corrects t errors with 2t parity symbols
- Excellent for burst errors

**Example**: RS(255, 223):
- 223 data bytes
- 32 parity bytes
- Corrects up to 16 symbol errors
- Code rate: 87%

#### 10.2.2 Convolutional Codes

**Encoder**:

Memory + XOR gates create redundancy:
```
Output 1 = D₀ ⊕ D₁ ⊕ D₂
Output 2 = D₀ ⊕ D₂
```

Where D₀, D₁, D₂ are current and past input bits.

**Viterbi Decoder**:
- Maximum likelihood decoding
- Complexity: O(2^k) where k = constraint length
- Excellent performance

**Code rate**: Typically 1/2, 1/3, 2/3

#### 10.2.3 Turbo Codes

**Iterative Decoding**:

Two parallel encoders + interleaver:
- Code rate: ~1/3
- Near Shannon limit performance
- Used in 4G/5G, deep space communication

**Performance**:
- At BER = 10⁻⁵: SNR ≈ 0.7 dB (0.7 dB from Shannon limit)

### 10.3 Quantum Error Correction

#### 10.3.1 Stabilizer Codes

**Quantum Errors**:
- Bit flip: X error
- Phase flip: Z error
- Both: Y error (= iXZ)

**Syndrome Measurement**:

Measure stabilizers without collapsing protected state:
```


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
