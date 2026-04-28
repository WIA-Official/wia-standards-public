# WIA-QUA-003 — Phase 2: API

> Quantum-internet API surface: entanglement distribution, repeater orchestration, and quantum-internet architecture endpoints integrators code against.

## 3. Entanglement Distribution

### 3.1 Direct Distribution

#### 3.1.1 Photon Pair Sources

**Spontaneous Parametric Down-Conversion (SPDC)**:
- Nonlinear crystal splits pump photon into signal and idler
- Creates entangled photon pairs
- Wavelengths: 810nm pump → 1550nm pairs (telecom)

**Quality Metrics**:
- Brightness: pairs per second per mW pump power
- Heralding efficiency: detection probability of one photon given the other
- Spectral purity: lack of frequency correlations

#### 3.1.2 Fiber Transmission

**Loss**:
```
P = P₀ × 10^(-αL/10)
```

Where:
- `α` = attenuation coefficient (0.2 dB/km at 1550nm)
- `L` = distance in km
- `P₀` = initial power

**Maximum Distance**: ~100-150 km for direct transmission (limited by photon loss)

### 3.2 Entanglement Swapping

Extends entanglement range by creating entanglement between distant nodes that never directly interacted.

#### 3.2.1 Protocol

1. **Initial State**:
   - Nodes A-B share entangled pair: |Φ⁺⟩ᴬᴮ
   - Nodes B-C share entangled pair: |Φ⁺⟩ᴮᶜ
   - Combined state: |Φ⁺⟩ᴬᴮ ⊗ |Φ⁺⟩ᴮᶜ

2. **Bell State Measurement (BSM)**:
   - Node B performs BSM on its two qubits
   - Measurement projects A-C into entangled state
   - BSM result determines which Bell state A-C share

3. **Classical Communication**:
   - B broadcasts BSM result to A and C
   - A or C apply correction based on result

4. **Final State**:
   - Nodes A and C now share entanglement
   - Fidelity reduced but still useful

**Success Probability**: 0.25 for standard BSM (4 Bell states, 2 distinguishable)

### 3.3 Entanglement Purification

Improves fidelity of noisy entangled states.

#### 3.3.1 BBPSSW Protocol

Starting with two copies of noisy state:
```
ρ = F|Φ⁺⟩⟨Φ⁺| + (1-F)/3 (|Φ⁻⟩⟨Φ⁻| + |Ψ⁺⟩⟨Ψ⁺| + |Ψ⁻⟩⟨Ψ⁻|)
```

**Protocol**:
1. Apply bilateral CNOT gates
2. Measure target qubits in computational basis
3. If results match, keep source pair (higher fidelity)
4. If results differ, discard both pairs

**Fidelity Improvement**:
```
F' = [F² + ((1-F)/3)²] / [F² + 2F(1-F)/3 + 5((1-F)/3)²]
```

**Success Probability**:
```
P = F² + 2F(1-F)/3 + 5((1-F)/3)²
```

---


## 4. Quantum Repeaters

### 4.1 Architecture

Quantum repeaters overcome photon loss to enable long-distance quantum communication.

#### 4.1.1 Components

1. **Entanglement Sources**: Generate entangled pairs
2. **Quantum Memories**: Store qubits during protocol
3. **Bell State Measurement**: Perform entanglement swapping
4. **Classical Communication**: Coordinate operations

#### 4.1.2 Operation

**Setup**:
- Divide distance into N segments of length L₀
- Place repeater at each segment boundary
- L₀ chosen based on direct entanglement fidelity

**Protocol**:
1. Generate entanglement over each segment
2. Store in quantum memories
3. Perform nested entanglement swapping
4. Optional: purify before swapping
5. Establish end-to-end entanglement

### 4.2 Performance Analysis

#### 4.2.1 Secret Key Rate

**Without Repeaters**:
```
R₀ ∝ η_det × 10^(-αL/10)
```

Exponential decay with distance L.

**With Repeaters**:
```
R ∝ (η_det × 10^(-αL₀/10))^N
```

Where N = L/L₀ (number of segments).

**Improvement**: Polynomial decay instead of exponential.

#### 4.2.2 Memory Requirements

**Storage Time**:
```
T_mem ≥ N × T_gen + (N-1) × T_BSM
```

Where:
- T_gen = entanglement generation time
- T_BSM = Bell measurement time
- N = number of segments

**Coherence Time**: Must exceed storage time requirement.

### 4.3 Quantum Memory Technologies

| Technology | Storage Time | Efficiency | Wavelength |
|------------|--------------|------------|------------|
| Rare-earth ions | > 1 second | 50% | 1550 nm |
| Quantum dots | ~1 ms | 20% | 900 nm |
| Atomic ensembles | 10-100 ms | 30-70% | 795 nm |
| NV centers | > 1 second | 10% | 637 nm |
| Trapped ions | > 10 minutes | 80% | Variable |

---


## 6. Quantum Internet Architecture

### 6.1 Network Layers

#### 6.1.1 Physical Layer

**Quantum Channels**:
- Optical fiber (1310nm, 1550nm)
- Free-space optical links
- Satellite quantum links

**Link Budget**:
```
P_received = P_transmitted - α×L - L_coupling - L_detection
```

#### 6.1.2 Link Layer

**Functions**:
- Point-to-point entanglement generation
- Entanglement distillation
- Link-level error correction

**Metrics**:
- Link fidelity
- Entanglement generation rate
- Link availability

#### 6.1.3 Network Layer

**Functions**:
- Quantum routing
- Path selection
- Resource allocation

**Routing Metrics**:
- Fidelity-distance product
- Availability
- Latency

#### 6.1.4 Transport Layer

**Functions**:
- End-to-end entanglement distribution
- Teleportation-based communication
- Error correction and recovery

#### 6.1.5 Application Layer

**Services**:
- QKD
- Distributed quantum computing
- Quantum sensing
- Quantum clock synchronization

### 6.2 Network Topology

#### 6.2.1 Star Topology

**Structure**: Central hub connected to multiple end nodes.

**Advantages**:
- Simple management
- Easy to add nodes
- Centralized control

**Disadvantages**:
- Single point of failure
- Hub capacity limit

#### 6.2.2 Mesh Topology

**Structure**: Multiple interconnections between nodes.

**Advantages**:
- High reliability (multiple paths)
- Load balancing
- Scalability

**Disadvantages**:
- Complex routing
- Higher cost

#### 6.2.3 Hybrid Topology

**Structure**: Combination of star and mesh.

**Use Case**:
- Metro networks: Star within cities
- Long-distance: Mesh between cities
- Satellite: Global connectivity

---



## A.1 Endpoint reference

```http
POST /qnet/v1/entanglement/request   # request an entangled pair
GET  /qnet/v1/path/{src}/{dst}       # query best path between nodes
POST /qnet/v1/repeater/swap          # request entanglement swap at a repeater
POST /qnet/v1/architecture/announce  # announce node capabilities to the network
```

Every endpoint follows the discovery convention at `/.well-known/wia-quantum-network`.

## A.2 Entanglement-distribution semantics

Entanglement-distribution is the canonical quantum-network primitive. The endpoint accepts source and destination node identifiers and returns either an entanglement record (success) or a problem document explaining why distribution failed (insufficient repeater capacity, channel quality below threshold, or memory expiry).

## A.3 Quantum-internet architecture

The standard recognises three architecture tiers:
- **Trusted-node networks** — repeaters fully trusted; simplest topology
- **Curve repeater networks** — partial trust; entanglement-swap chain
- **Quantum-internet networks** — fully untrusted intermediate nodes; end-to-end entanglement

The host's discovery document declares which tier it operates at.

## A.4 Conformance coverage

Conformant hosts MUST pass: discovery round-trip, entanglement request, path query, repeater swap, problem-detail emission, and rate-limit headers.


## A.5 Path-query semantics

```http
GET /qnet/v1/path/{src}/{dst}
Accept: application/json

→ 200 OK
{
  "wia_qnet_version": "1.0.0",
  "path_id": "path_01HX...",
  "src": "urn:wia:qnet:node-A",
  "dst": "urn:wia:qnet:node-Z",
  "hops": [
    { "node": "urn:wia:qnet:node-A" },
    { "link": "link_AB", "estimated_fidelity": 0.95 },
    { "node": "urn:wia:qnet:node-B (repeater)" },
    { "link": "link_BZ", "estimated_fidelity": 0.92 },
    { "node": "urn:wia:qnet:node-Z" }
  ],
  "estimated_end_to_end_fidelity": 0.87,
  "estimated_latency_ms": 142
}
```

## A.6 Bulk export

Every quantum-network host exposes a bulk export endpoint for audit
and reproducibility. The export contains every entanglement record,
swap operation, and teleportation event in a time window with a
Merkle root commitment over the bundle.

## A.7 Architecture-tier negotiation

When two networks meet at a federation boundary, the architecture
tier of each is declared via the discovery document. A workload
that requires a specific tier (e.g., fully untrusted-node networks
for high-security workloads) refuses paths that traverse a lower-
tier network.


## B.1 Conformance test suite

A black-box conformance test suite is published at
`https://github.com/WIA-Official/wia-quantum-network-conformance` and walks
through every public endpoint plus the cross-Phase integration
scenarios. Hosts publishing `bridge_profile=Full` SHOULD additionally
pass the suite's bridge-extension tests.

The suite checks: discovery document round-trip, every Phase 2 endpoint
with mock data, problem-detail emission for malformed inputs, rate-limit
header presence and exhaustion behaviour, replay-defence bounds (300-second
skew, 600-second nonce cache), and audit-log envelope shape.

## B.2 Reference container

The `wia/quantum-network-host:1.0.0` container image implements every Phase 2
endpoint with mock data; integrators exercise their bridge against it
before going to production. The container ships with a small library of
mock scenarios so the conformance suite has fixtures to run against.

## B.3 Companion CLI

The `cli/quantum-network.sh` script ships sample envelope generators (validate,
info, plus phase-specific subcommands) so an implementer can produce
conformant payloads without hand-rolling JSON. The CLI has no dependency
beyond `jq` and POSIX shell, so it runs in any CI environment without
additional tooling installation.

## B.4 Operational considerations

Quantum-network infrastructure has three operational considerations
that integrators consistently underestimate. First, the wire-format
discipline: every envelope is signed and verified at the boundary;
unsigned envelopes are refused at conformance, full stop. Second, the
trust-list refresh cadence: stale trust anchors are the single largest
source of avoidable production incidents in this standard family. Third,
the audit-log replication discipline: audit logs replicated across only
one storage backend cannot survive a site failure, and a site failure
that takes the audit log with it leaves the operator unable to
reconstruct what happened during the failure window.

## B.5 Backwards-compatibility promise

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields, optional query parameters,
new envelope types, new endpoints, or new protocol exchanges; hosts
MUST NOT remove or repurpose existing ones. Breaking changes ride a
major version bump and MUST be preceded by a 12-month deprecation
window per IETF RFC 8594 and 9745.

## B.6 Governance

The standard is maintained by the WIA Standards Committee. Change
proposals follow the WIA RFC process: anyone may submit a proposal;
the Committee reviews quarterly; accepted proposals enter an open
comment period before merging into a minor-version release. Breaking
changes require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.

## C.1 Glossary

The following terms appear repeatedly throughout this Phase and
the wider quantum-network standard: entanglement swap; quantum repeater; quantum memory; quantum teleportation; trusted-node network; curve repeater network; quantum-internet; link-state announcement; fidelity-weighted routing; URN scheme; trust list.

Implementers unfamiliar with the domain should treat these terms as
load-bearing — every endpoint, every protocol exchange, and every
integration document below assumes the reader understands what each
term means in context. Expanded definitions appear in the standard's
companion glossary at `https://wiastandards.com/quantum-network/glossary/`.

## C.2 Cross-standard composition

This Phase composes with adjacent WIA family standards as follows:

- **WIA-OMNI-API** owns credential storage and identity for every
  signed envelope in this Phase.
- **WIA-AIR-SHIELD** owns runtime trust list maintenance and key
  rotation; this Phase consumes WIA-AIR-SHIELD events.
- **WIA-SOCIAL Phase 3 §5** receipt shape is reused verbatim for
  every cross-host federation handshake referenced in this Phase.
- **WIA-INTENT** owns the outermost-layer declaration of workload
  intent; consumers parse the intent envelope before drilling into
  this Phase's specifics.

The composition is intentional: a single host running multiple WIA
family standards reuses the same identity, signature, and federation
machinery across all of them rather than maintaining N parallel
implementations. The conformance suite verifies the composition by
running a multi-standard scenario where this Phase's envelopes flow
through the adjacent standards' machinery and back.

## C.3 Implementation runbook

A first implementation of this Phase typically follows the runbook:

1. Stand up the reference container ('wia/quantum-network-host:1.0.0') in a
   development environment.
2. Run the conformance suite against the container to verify all
   tests pass on the reference implementation.
3. Replace the mock backend with the host's real backend
   one endpoint at a time; re-run conformance after each
   replacement.
4. Wire up the audit log replication; verify a full session round
   trip lands in both replicas.
5. Onboard a single trusted peer for federation; exercise the
   handshake and audit envelope flow.
6. Expand to multiple peers; rotate trust anchors per the
   30-day cadence.
7. Promote to production; subscribe operations to the warning
   envelope cadence (collateral expiry, drift detection,
   barren-plateau onset, etc., as relevant per phase).

The full runbook is roughly two engineer-weeks of work for a team
already familiar with the underlying domain (QKD, QML, quantum-
network, or quantum-sensor as applicable).
