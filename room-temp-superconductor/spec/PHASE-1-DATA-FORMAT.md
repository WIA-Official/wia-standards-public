# WIA-QUA-019 — Phase 1: Data Format

> Room-temperature superconductor canonical envelopes: terminology, physical-state model, and material-system descriptors that fix the wire format for every protocol below.

## 1. Introduction

### 1.1 Purpose

The WIA-QUA-019 standard provides:
- Definition and criteria for room-temperature superconductivity
- Material classifications and properties database
- High-pressure synthesis protocols (diamond anvil cells)
- Ambient-pressure synthesis methods (LK-99 and candidates)
- Comprehensive characterization standards
- Validation criteria for superconductivity claims
- Application specifications and simulations
- Safety and handling protocols
- Integration with existing superconductor standards

### 1.2 Scope

This standard covers:
- **Materials**: Hydrogen-rich hydrides (H₃S, LaH₁₀, YH₉, C-S-H), LK-99 type compounds, cuprates, nickelates, and novel candidates
- **Temperature Range**: Tc ≥ 300K (27°C) as minimum room-temperature threshold
- **Pressure Regimes**: Both high-pressure (100-300 GPa) and ambient-pressure materials
- **Synthesis**: Diamond anvil cells, laser heating, ambient pressure methods
- **Characterization**: Resistance, magnetization, specific heat, spectroscopy
- **Applications**: Power grids, transportation, quantum computing, medicine
- **Validation**: Multi-method confirmation protocols

### 1.3 Related Standards

- **WIA-QUA-007**: Superconducting (low-temperature systems)
- **WIA-QUA-001**: Quantum Computing Foundation
- **WIA-ENERGY-001**: Energy Efficiency
- **WIA-MATERIALS-001**: Advanced Materials
- **WIA-TRANSPORT-001**: Transportation Systems
- **WIA-MEDICAL-001**: Medical Devices

---


## 2. Terminology

### 2.1 Core Terms

- **Room-Temperature Superconductivity**: Zero resistance and Meissner effect at Tc ≥ 300K
- **Critical Temperature (Tc)**: Temperature below which superconductivity occurs
- **High-Pressure Superconductivity**: Tc enhanced by pressures >10 GPa
- **Hydrogen-Rich Hydride**: Compound with >50 atomic % hydrogen
- **Diamond Anvil Cell (DAC)**: Device generating pressures >100 GPa
- **LK-99**: Copper-substituted lead apatite: Pb₁₀₋ₓCuₓ(PO₄)₆O
- **Meissner Effect**: Perfect diamagnetism (χ = -1) in superconducting state
- **Cooper Pair**: Bound electron pair enabling superconductivity
- **Critical Current Density (Jc)**: Maximum current before superconductivity lost
- **Zero-Field-Cooled (ZFC)**: Sample cooled without applied magnetic field
- **Field-Cooled (FC)**: Sample cooled with applied magnetic field

### 2.2 Acronyms

- **RTS**: Room-Temperature Superconductor
- **HTSC**: High-Temperature Superconductor (Tc > 77K)
- **DAC**: Diamond Anvil Cell
- **GPa**: GigaPascal (10⁹ Pa, ~10⁴ atmospheres)
- **SQUID**: Superconducting Quantum Interference Device
- **VSM**: Vibrating Sample Magnetometer
- **XRD**: X-Ray Diffraction
- **ARPES**: Angle-Resolved Photoemission Spectroscopy
- **ZFC**: Zero-Field-Cooled
- **FC**: Field-Cooled

---


## 3. Room-Temperature Superconductivity Physics

### 3.1 Defining Room-Temperature Superconductivity

#### 3.1.1 Temperature Criterion

**Minimum Threshold:**
```
Tc_min = 300K (26.85°C, 80.33°F)
```

**Preferred Targets:**
```
Tc_preferred = 350K (76.85°C, 170.33°F)
Tc_ideal = 400K (126.85°C, 260.33°F)
```

**Rationale:**
- 300K represents true "room temperature" in most environments
- Higher Tc provides operational margin for temperature fluctuations
- 400K enables high-temperature applications (engines, industry)

#### 3.1.2 Superconductivity Criteria

A material qualifies as room-temperature superconductor if:

1. **Zero Resistance**
   ```
   R(T < Tc) / R(T > Tc) < 10⁻⁶
   ```
   - Sharp transition (ΔTc < 5K preferred)
   - Reproducible across multiple samples
   - Independent of measurement frequency (DC to MHz)

2. **Meissner Effect**
   ```
   χ = M/H < -0.9
   ```
   - Magnetic field expulsion observed
   - Both ZFC and FC measurements show diamagnetism
   - Levitation demonstration at T ≥ 300K

3. **Critical Current**
   ```
   Jc(T = 300K) > 10⁴ A/m² minimum
   Jc(T = 300K) > 10⁶ A/m² for applications
   ```

4. **Reproducibility**
   - Confirmed by ≥3 independent laboratories
   - Multiple characterization methods agree
   - Published in peer-reviewed journals

### 3.2 Cooper Pairing Mechanisms

#### 3.2.1 Conventional Electron-Phonon Coupling

**BCS Theory Extension:**

Energy gap at T=0:
```
Δ₀ = 1.764 × kB × Tc
```

For Tc = 300K:
```
Δ₀ ≈ 45 meV
```

**Challenges:**
- Conventional BCS predicts Tc_max ~ 30-40K for realistic phonon frequencies
- Room-temperature Tc requires:
  - Very high phonon frequencies (ω_ph > 200 meV)
  - Strong electron-phonon coupling (λ > 2)
  - Light elements (high ω_ph) → Hydrogen-rich compounds

#### 3.2.2 Hydrogen-Rich Hydrides

**McMillan Formula:**
```
Tc = (ω_log / 1.2) × exp[-1.04(1 + λ) / (λ - μ*(1 + 0.62λ))]
```

Where:
- ω_log = logarithmic average phonon frequency
- λ = electron-phonon coupling constant
- μ* = Coulomb pseudopotential

**Hydrogen Advantages:**
- Lightest element → highest phonon frequencies
- ω_ph ∝ 1/√M (M = atomic mass)
- High pressure → increased overlap → larger λ

**Predicted Tc for Metallic Hydrogen:**
```
Tc_H > 400K at P > 500 GPa (theoretical)
```

#### 3.2.3 Alternative Pairing Mechanisms

**Non-Phononic Mechanisms:**

1. **Excitonic Pairing**
   - Electron-electron attraction mediated by excitons
   - Possible in doped insulators
   - LK-99 proposed mechanism

2. **Spin Fluctuations**
   - Magnetic interactions mediate pairing
   - d-wave symmetry (cuprates, iron-based)
   - Possible in nickelates

3. **Plasmon Mediation**
   - Electronic plasmons couple electrons
   - High-frequency modes possible
   - Graphene-based systems

4. **Topological Superconductivity**
   - Band topology enhances pairing
   - Protected edge states
   - Twisted bilayer graphene

### 3.3 Pressure-Temperature Phase Diagrams

#### 3.3.1 H₃S System

**Phase Diagram:**
```
Tc(P) = Tc0 + α₁P + α₂P² + α₃P³

For H₃S:
Tc0 = 150K (extrapolated to P=0)
Maximum: Tc = 203K at P = 155 GPa
Structure: Im-3m (cubic)
```

**Pressure Effects:**
- Increased lattice overlap → stronger coupling
- Structural transitions at critical pressures
- Tc peaks then decreases at very high P

#### 3.3.2 LaH₁₀ System

**Phase Diagram:**
```
Tc_max = 250K at P = 170 GPa

Structure: Fm-3m (fcc clathrate cage)
Stability: P > 150 GPa
Decomposition: P < 100 GPa → La + H₂
```

**Hydrogen Content:**
- 90.9 atomic % hydrogen
- H atoms form clathrate cage around La
- Metallic hydrogen bonding

#### 3.3.3 C-S-H System (Record Holder)

**Current Record:**
```
Tc = 288K (-15°C) at P = 267 GPa

Structure: Im-3m
Composition: C₁₅H₃₂S₂ (proposed)
Discovery: 2020
```

**Significance:**
- Only 12K below room temperature
- Demonstrates path to Tc > 300K
- Complex ternary hydride

---


## 4. Material Systems

### 4.1 Hydrogen-Rich Hydrides

#### 4.1.1 H₃S (Hydrogen Sulfide)

**Properties:**
```
Chemical Formula: H₃S
Critical Temperature: Tc = 203K at 155 GPa
Crystal Structure: Im-3m (cubic)
Hydrogen Content: 75 atomic %
Discovery Year: 2015
Status: Confirmed by multiple groups
```

**Synthesis:**
- Precursor: Elemental S + H₂ gas
- Pressure: 155-200 GPa
- Temperature: Laser heating to 1500-2500K
- Product: Metallic H₃S

**Characteristics:**
- First hydride with Tc > 200K
- Well-studied and reproducible
- Model system for high-Tc hydrides

#### 4.1.2 LaH₁₀ (Lanthanum Decahydride)

**Properties:**
```
Chemical Formula: LaH₁₀
Critical Temperature: Tc = 250K at 170 GPa
Crystal Structure: Fm-3m (fcc clathrate)
Hydrogen Content: 90.9 atomic %
Discovery Year: 2019
Status: Confirmed
```

**Synthesis:**
- Precursor: La metal + H₂ gas
- Pressure: 170-200 GPa
- Temperature: Laser heating to 2000K
- Annealing: Slow cooling required

**Structure:**
- H atoms form clathrate cage
- La at cage center
- Metallic hydrogen bonding
- 32 H atoms in unit cell

#### 4.1.3 YH₉ (Yttrium Hydride)

**Properties:**
```
Chemical Formula: YH₉
Critical Temperature: Tc = 243K at 201 GPa
Crystal Structure: P6₃/mmc (hexagonal)
Hydrogen Content: 90 atomic %
Discovery Year: 2021
Status: Confirmed
```

#### 4.1.4 C-S-H (Carbonaceous Sulfur Hydride)

**Properties:**
```
Proposed Formula: C₁₅H₃₂S₂
Critical Temperature: Tc = 288K at 267 GPa
Highest Confirmed Tc: Yes
Temperature: -15°C (just below room temp!)
Discovery Year: 2020
Status: Confirmed
```

**Significance:**
- Closest to room temperature achieved
- Complex ternary composition
- Path to Tc > 300K demonstrated

### 4.2 LK-99 Type Materials

#### 4.2.1 LK-99 (Copper-Substituted Lead Apatite)

**Properties:**
```
Chemical Formula: Pb₁₀₋ₓCuₓ(PO₄)₆O
Typical Doping: x = 0.1 (10% Cu substitution)
Claimed Tc: ~400K (127°C)
Operating Pressure: Ambient (1 atm)
Discovery Year: 2023
Status: HIGHLY CONTROVERSIAL - Under investigation
```

**Synthesis Protocol:**
1. **Starting Materials:**
   - Lead oxide (PbO): 10g
   - Lead sulfate (PbSO₄): 5g
   - Copper phosphide (Cu₃P): 0.5g

2. **Reaction:**
   - Mix thoroughly in crucible
   - Heat to 1000-1200°C
   - Hold for 10-24 hours
   - Cool slowly (1°C/min)

3. **Annealing:**
   - Reheat to 800°C
   - Hold for 48-96 hours
   - Cool slowly

**Proposed Mechanism:**
- Cu²⁺ substitution creates holes
- Quantum well at Cu-O layer
- Possible excitonic pairing
- Structural distortion key

**Controversy:**
- Contradictory experimental results
- Some groups observe diamagnetic signals
- Many groups find no superconductivity
- Possible sample-dependent effects

**Required Validation:**
- Zero resistance measurement (not just resistance drop)
- Meissner effect confirmation (full field expulsion)
- Critical current measurement
- Multiple independent confirmations

#### 4.2.2 Other Apatite-Based Candidates

**Exploration:**
- Other metal substitutions (Ag, Au, Ni)
- Different rare-earth apatites
- Optimized synthesis conditions
- Pressure-assisted formation

### 4.3 Ambient-Pressure High-Tc Materials

#### 4.3.1 Cuprate Superconductors

**YBCO (YBa₂Cu₃O₇):**
```
Tc = 93K (-180°C)
Pressure: Ambient
Structure: Orthorhombic perovskite
Mechanism: d-wave pairing
Status: Mature technology
```

**Limitations:**
- Tc well below room temperature
- Liquid nitrogen required (77K)
- Still too cold for most applications

#### 4.3.2 Nickelate Superconductors

**Nd₀.₈Sr₀.₂NiO₂:**
```
Tc = 15K (-258°C)
Discovery: 2019
Structure: Similar to cuprates
Mechanism: Under investigation
Potential: Tc optimization possible?
```

#### 4.3.3 Magic-Angle Twisted Bilayer Graphene

**Properties:**
```
Tc = 1-3K (current)
Twist Angle: 1.1° ("magic angle")
Mechanism: Topological, flat bands
Potential: Structure optimization
```

**Future Potential:**
- Multi-layer optimization
- Different 2D materials
- Electrostatic gating
- Strain engineering

### 4.4 Theoretical Predictions

#### 4.4.1 Metallic Hydrogen

**Predictions:**
```
Tc > 400K (theoretical)
Required Pressure: P > 500 GPa
Structure: Multiple phases predicted
Status: Not yet synthesized/confirmed
```

**Challenges:**
- Extreme pressures required
- Metastability unclear
- Detection difficult

#### 4.4.2 Other Predicted High-Tc Materials

1. **Ternary Hydrides:**
   - Li-Mg-H systems
   - Ca-Y-H systems
   - Target: Lower pressure, higher Tc

2. **Boron-Based Compounds:**
   - MgB₂H₁₂ and variants
   - Predicted Tc ~ 250-300K

3. **Organic Superconductors:**
   - Engineered organic molecules
   - Room-temp predictions exist

---



## A.1 Canonical envelope conventions

Every Phase 1 envelope follows the WIA family baseline: UTF-8 JSON
with RFC 8785 canonicalisation, Ed25519 signatures (IETF RFC 8032),
ULIDs as identifiers, and explicit measurement uncertainty per
BIPM JCGM 100. Critical-temperature claims carry both Type A
(statistical) and Type B (systematic) uncertainty so the chain of
evidence behind a room-temperature-superconductor claim is auditable.

## A.2 Material-system descriptor

```json
{
  "wia_rts_version": "1.0.0",
  "type": "material_descriptor",
  "material_id": "mat_01HX...",
  "system": "hydride" | "graphite" | "complex_oxide" | "other",
  "stoichiometry": "LaH10",
  "pressure_GPa_required": 170,
  "estimated_Tc_K": 250,
  "Tc_uncertainty_K": 8,
  "synthesised_at": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## A.3 Falsifiable measurement record

Every claim of room-temperature-superconductivity carries a
falsifiable measurement record: zero-resistance evidence at the
claimed Tc, Meissner-effect demonstration, specific-heat anomaly
at the transition, and isotope-effect measurement (where the
material system supports isotope substitution). The record is
signed by the laboratory and counter-signed by an independent
verifier when one is available.


## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/room-temp-superconductor/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-room-temp-superconductor-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/room-temp-superconductor-host:1.0.0` ships every Phase 2 endpoint with mock
data for integrators to test against. The companion CLI at
`cli/room-temp-superconductor.sh` ships sample envelope generators with no dependencies
beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, signature, and
audit machinery rather than maintaining N parallel implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up reference
container; run conformance suite against it; replace mock backend
with real backend one endpoint at a time; wire audit log
replication; onboard a single trusted peer for federation; expand
to multiple peers; promote to production with warning-envelope
subscription.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the standards four-Phase architecture: Phase
1 envelopes are the wire-format contract; Phase 2 surfaces them
through HTTPS; Phase 3 wraps them in protocol exchanges that cross
trust boundaries; Phase 4 integrates with the broader ecosystem.

弘益人間 — Benefit All Humanity.
