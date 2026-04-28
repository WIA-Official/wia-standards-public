# WIA-BIO-012 — Phase 2: API Interface

> Synthetic-biology canonical Phase 2: design API (DNA assembly + circuit + computational tools).

# WIA-BIO-012: Synthetic Biology Specification v1.0

> **Standard ID:** WIA-BIO-012
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Genetic Part Standards](#2-genetic-part-standards)
3. [DNA Assembly Methods](#3-dna-assembly-methods)
4. [Genetic Circuit Design](#4-genetic-circuit-design)
5. [Metabolic Engineering](#5-metabolic-engineering)
6. [Cell-Free Systems](#6-cell-free-systems)
7. [Computational Design Tools](#7-computational-design-tools)
8. [Biosafety and Biosecurity](#8-biosafety-and-biosecurity)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---


## 3. DNA Assembly Methods

### 3.1 BioBrick Assembly

Standard restriction-ligation method:

```
1. Digest upstream part with EcoRI-SpeI
2. Digest downstream part with XbaI-PstI
3. Ligate compatible SpeI-XbaI junction
4. Result: EcoRI-[Part1]-[Part2]-PstI
```

Scar sequence formed:
```
SpeI-XbaI → TACTAGAG (8 bp scar)
```

### 3.2 Golden Gate Assembly

Type IIS enzyme-based scarless assembly:

```
1. Design parts with BsaI sites and 4-bp overhangs
2. Mix all parts with BsaI and ligase
3. Thermocycle: 37°C (5 min) / 16°C (10 min) × 30 cycles
4. Final: 50°C (5 min), 80°C (10 min)
```

Overhang design rules:
- Avoid palindromes
- GC content: 25-75%
- No self-complementary sequences
- Minimize secondary structure

### 3.3 Gibson Assembly

Isothermal assembly via homologous recombination:

```
1. Design 20-40 bp overlapping sequences
2. Mix DNA fragments with:
   - T5 exonuclease (3' → 5' resection)
   - Phusion polymerase (fill gaps)
   - Taq ligase (seal nicks)
3. Incubate at 50°C for 60 minutes
```

Overlap calculation:
```
Tm = 81.5 + 0.41(%GC) - 675/length - % mismatch
```

Optimal Tm: 48-55°C for 20-40 bp overlaps

### 3.4 CPEC (Circular Polymerase Extension Cloning)

Primer-based extension without ligase:

```
1. Design primers with 15-30 bp overlaps
2. PCR amplify fragments
3. Mix PCR products
4. Extension cycling without primers
5. Transform directly
```

---



## 4. Genetic Circuit Design

### 4.1 Basic Logic Gates

#### 4.1.1 NOT Gate (Repressor)
```
Output = Pmax / (1 + ([Repressor]/K)ⁿ)
```

Example: LacI repression
```
GFP = Pmax / (1 + ([LacI]/K)²)
```

#### 4.1.2 AND Gate
```
Output = Pmax × ([A]/Ka)ⁿ × ([B]/Kb)ᵐ /
         (1 + [A]/Ka + [B]/Kb + ([A]/Ka)ⁿ×([B]/Kb)ᵐ)
```

#### 4.1.3 OR Gate
```
Output = Pmax × (([A]/Ka)ⁿ + ([B]/Kb)ᵐ) /
         (1 + ([A]/Ka)ⁿ + ([B]/Kb)ᵐ)
```

### 4.2 Oscillators

Repressilator model:
```
dm₁/dt = -m₁ + α/(1 + p₃ⁿ) + α₀
dm₂/dt = -m₂ + α/(1 + p₁ⁿ) + α₀
dm₃/dt = -m₃ + α/(1 + p₂ⁿ) + α₀
dp₁/dt = -β(p₁ - m₁)
dp₂/dt = -β(p₂ - m₂)
dp₃/dt = -β(p₃ - m₃)
```

Where:
- `mᵢ` = mRNA concentration
- `pᵢ` = Protein concentration
- `α` = Transcription rate
- `β` = Translation rate
- `n` = Hill coefficient

### 4.3 Toggle Switch

Bistable circuit with two stable states:

```
du/dt = α₁/(1 + v²) - u
dv/dt = α₂/(1 + u²) - v
```

Where:
- `u, v` = Protein concentrations
- `α₁, α₂` = Production rates

Stability conditions:
```
α₁ × α₂ > 1 (bistability exists)
```

### 4.4 Feed-Forward Loops

Coherent Type-1 FFL:
```
X → Y → Z
X ----→ Z
```

Response time:
```
τ = 1/k × ln(K/(K-1))
```

Where:
- `k` = Degradation rate
- `K` = Threshold ratio

---



## 7. Computational Design Tools

### 7.1 RBS Calculator

Predicts translation initiation rate:

```
1. Model ribosome binding using thermodynamics
2. Calculate ΔG of mRNA-rRNA hybridization
3. Account for mRNA secondary structure
4. Predict spacing between RBS and start codon
5. Output: Translation Initiation Rate (TIR)
```

Algorithm:
```
ΔG_total = ΔG_mRNA:rRNA + ΔG_spacing + ΔG_standby + ΔG_start
TIR = K × exp(-ΔG_total / RT)
```

### 7.2 Codon Optimization

Maximize expression in target organism:

```
1. Calculate codon usage table for host
2. Replace rare codons with common ones
3. Maintain GC content (40-60%)
4. Avoid secondary structures
5. Remove restriction sites
6. Preserve regulatory elements
```

Fitness function:
```
F = w₁×CAI + w₂×GC + w₃×CFD
```

Where:
- `CAI` = Codon Adaptation Index
- `GC` = GC content score
- `CFD` = Codon Frequency Distribution
- `w` = Weights

### 7.3 Circuit Modeling

ODE-based simulation:

```python
def genetic_circuit(t, y, params):
    mRNA, Protein = y
    k_tx, k_tl, k_deg_m, k_deg_p = params

    dmRNA = k_tx - k_deg_m * mRNA
    dProtein = k_tl * mRNA - k_deg_p * Protein

    return [dmRNA, dProtein]
```

Stochastic simulation (Gillespie):
```
1. Calculate propensity of each reaction
2. Sample time to next reaction: τ = -ln(rand)/a_total
3. Select reaction proportional to propensity
4. Update molecule counts
5. Repeat until end time
```

---




---

## A.1 Endpoint reference

```http
POST /synbio/v1/parts                              # contribute genetic part
GET  /synbio/v1/parts/{id}                         # fetch genetic part
POST /synbio/v1/circuits                           # design genetic circuit
GET  /synbio/v1/circuits/{id}/simulation           # simulation results
POST /synbio/v1/assemblies                         # plan an assembly
GET  /synbio/v1/assemblies/{id}/oligos             # required oligos for assembly
POST /synbio/v1/cell-free                          # design cell-free reaction
```

Every endpoint follows the discovery convention at `/.well-known/wia-synthetic-biology`.

## A.2 Genetic-circuit design

`POST /circuits` accepts a directed-graph description of the circuit (nodes = parts, edges = regulatory or compositional relationships) and returns the validated circuit envelope with composability checks (no internal restriction-site collisions, position-class compatibility for MoClo, intercompatible chassis). The circuit is then ready for simulation, assembly planning, and lab execution.

## A.3 Simulation API

`GET /circuits/{id}/simulation` runs the kinetic simulation against the digital-twin descriptor (Phase 1 §A.5). Output: time-series of mRNA / protein concentrations per node, sensitivity analysis, parameter-uncertainty propagation. The simulator is configurable (deterministic ODE, stochastic Gillespie, hybrid τ-leap).

## A.4 Assembly-planning API

`POST /assemblies` plans an assembly from a circuit. The planner picks the assembly method (Gibson, Golden Gate / MoClo, BioBrick, LCR, SLIC, CPEC) based on the parts available and the chassis. Output: an oligo list, a PCR plan, an assembly-step sequence, and a verification plan (Sanger primers, restriction digests).

## A.5 Cell-free reaction design

`POST /cell-free` designs cell-free expression reactions. Inputs: target protein(s), expression chassis (PURE, E. coli S30, wheat germ, rabbit reticulocyte), reaction volume, expected yield. Output: optimised reaction recipe with energy-regeneration cocktail composition and expected time-course.

## A.6 Computational design tools

The reference adapters expose Cello (genetic-circuit synthesis), Constellation (sequence design), iBioSim (model generation), j5 (assembly automation), and RAVEN (assembly automation) through the same canonical envelopes. Adapters live outside the conformance scope but the envelopes they consume are conformance-tested.

## A.7 Webhook events

Webhook events: `part.created`, `circuit.designed`, `simulation.completed`, `assembly.planned`, `cell-free.designed`, `assembly.completed`, `verification.passed`, `verification.failed`. HMAC-SHA256 signing per the WIA family policy; receivers dedupe on `deliveryId`. Retry policy: 3 attempts at 1s/4s/16s; failures route to the dead-letter queue and operations is paged.

## A.8 Rate-limit envelope

1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h premium tier. Simulation runs count against a separate computational quota because they are CPU- or GPU-bounded; large simulations queue with priority based on the tenant's subscription class.

## A.9 Bulk operations

`POST /synbio/v1/parts/bulk` and `POST /synbio/v1/circuits/bulk` accept up to 100 entries per request. Per-entry success/failure status is returned in the response body. Idempotency is keyed by the SHA-256 of the sorted entry-identifier list; retries with the same key short-circuit to the cached response.

## A.10 Authentication and scopes

OAuth 2.0 + OpenID Connect authentication. Scopes: `parts:read`, `parts:write`, `circuits:read`, `circuits:write`, `assemblies:read`, `assemblies:write`, `cell-free:design`, `simulations:run`. Access tokens are 1 hour; refresh tokens are 30 days. API keys are an optional secondary path for instrument-to-system traffic and MUST be rotated every 90 days.

## A.11 Pagination, sort, filter

Cursor-based pagination via `?after=cursor&limit=N` capped at 100. Sort via `?sort=field&order=asc|desc` with multi-field comma-list support. Filtering syntax: `?field=value` for equality, `?field[op]=value` for `gte`/`lte`/`in`/`like`. Cursors are opaque — clients MUST NOT decode them.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/synthetic-biology/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-synthetic-biology-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/synthetic-biology-host:1.0.0` ships every synthetic-biology envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/synthetic-biology.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Synthetic-biology deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
