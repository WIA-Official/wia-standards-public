# WIA-QUA-007 — Phase 1: Data Format

> Superconducting canonical Phase 1: physics + materials + classification.

# WIA-QUA-007: Superconducting - Complete Specification v1.0

> **Standard:** WIA-QUA-007
> **Title:** Superconducting
> **Version:** 1.0.0
> **Status:** Active
> **Date:** 2025-12-26
> **Authors:** WIA Quantum Research Group
> **Category:** Future Technology / Quantum / Physics

---

## Abstract

This specification defines a comprehensive framework for superconducting systems - materials and devices that exhibit zero electrical resistance and perfect diamagnetism below a critical temperature. The standard encompasses theoretical foundations (BCS theory, Cooper pairs), material classifications (Type I, Type II, HTSC), device implementations (qubits, Josephson junctions, SQUIDs), and practical applications (quantum computing, MRI, maglev, power transmission).

**弘益人間 (Benefit All Humanity)** - This standard serves humanity by enabling revolutionary technologies through superconductivity: efficient energy systems, powerful quantum computers, advanced medical imaging, and ultra-sensitive scientific instruments.

---


## 1. Introduction

### 1.1 Purpose

The WIA-QUA-007 standard provides:
- Superconducting material specifications
- Cooper pair and BCS theory implementation
- Josephson junction design standards
- Superconducting qubit architectures
- SQUID magnetometer protocols
- Maglev system specifications
- Cryogenic system requirements
- Power transmission standards

### 1.2 Scope

This standard covers:
- Type I and Type II superconductors
- High-temperature superconductors (HTSC)
- Unconventional superconductors
- Superconducting qubits (transmon, flux, phase)
- Josephson junctions and devices
- SQUID magnetometers (dc and rf)
- Superconducting magnets (MRI, LHC)
- Maglev levitation systems
- Superconducting power cables
- Cryogenic cooling systems

### 1.3 Related Standards

- **WIA-QUA-001**: Quantum Computing Foundation
- **WIA-QUA-002**: Quantum Entanglement
- **WIA-QUA-005**: Quantum Error Correction
- **WIA-ENERGY-001**: Energy Efficiency
- **WIA-TRANSPORT-001**: Transportation Systems
- **WIA-MEDICAL-001**: Medical Devices

---



## 2. Terminology

### 2.1 Core Terms

- **Superconductivity**: Zero electrical resistance below critical temperature
- **Cooper Pair**: Bound state of two electrons forming boson
- **BCS Theory**: Bardeen-Cooper-Schrieffer quantum theory of superconductivity
- **Critical Temperature (Tc)**: Temperature below which superconductivity occurs
- **Critical Field (Bc)**: Magnetic field that destroys superconductivity
- **Critical Current (Jc)**: Current density that destroys superconductivity
- **Meissner Effect**: Expulsion of magnetic field from superconductor
- **Josephson Junction**: Tunnel barrier between two superconductors
- **SQUID**: Superconducting Quantum Interference Device
- **Flux Quantum**: Φ₀ = h/2e = 2.067833848 × 10⁻¹⁵ Wb

### 2.2 Acronyms

- **BCS**: Bardeen-Cooper-Schrieffer
- **HTSC**: High-Temperature Superconductor
- **LTS**: Low-Temperature Superconductor
- **YBCO**: YBa₂Cu₃O₇ (Yttrium Barium Copper Oxide)
- **BSCCO**: Bismuth Strontium Calcium Copper Oxide
- **MRI**: Magnetic Resonance Imaging
- **LHC**: Large Hadron Collider
- **SQUID**: Superconducting Quantum Interference Device
- **RSFQ**: Rapid Single Flux Quantum

---



## 3. Superconducting Physics

### 3.1 Cooper Pairs

#### 3.1.1 Formation Mechanism

Cooper pairs form through electron-phonon interaction:

1. **Lattice Distortion**: Electron attracts positive ions
2. **Phonon Emission**: Creates lattice vibration
3. **Attractive Potential**: Second electron attracted to distortion
4. **Pair Formation**: Two electrons bind with opposite spin and momentum

**Mathematical Description**:

```
Ψ(r₁, r₂) = Ψ₀ × exp(i k · R) × φ(r)
```

Where:
- `Ψ(r₁, r₂)` = Cooper pair wavefunction
- `R = (r₁ + r₂)/2` = Center of mass coordinate
- `r = r₁ - r₂` = Relative coordinate
- `k` = Total momentum (usually zero)
- `φ(r)` = Relative wavefunction

#### 3.1.2 Cooper Pair Properties

```typescript
interface CooperPair {
  // Constituent electrons
  electrons: {
    electron1: {
      spin: 'up';
      momentum: Vector3; // k
      energy: number; // E(k)
    };
    electron2: {
      spin: 'down';
      momentum: Vector3; // -k
      energy: number; // E(-k)
    };
  };

  // Pair characteristics
  totalSpin: 0; // Singlet state
  totalMomentum: Vector3; // Usually zero
  bindingEnergy: number; // 2Δ (meV)
  coherenceLength: number; // ξ (nanometers)
  pairDensity: number; // pairs/m³

  // Quantum properties
  wavefunction: ComplexFunction;
  phasedifference: number; // radians
  bosonic: true; // Cooper pairs are bosons
}
```

### 3.2 BCS Theory

#### 3.2.1 Energy Gap Equation

The BCS energy gap at temperature T:

```
Δ(T) = Δ₀ × tanh(1.74 × √(Tc/T - 1))
```

At zero temperature:

```
Δ₀ = 1.764 × kB × Tc
```

Where:
- `Δ(T)` = Energy gap at temperature T (meV)
- `Δ₀` = Energy gap at T = 0 K
- `kB` = Boltzmann constant (8.617 × 10⁻⁵ eV/K)
- `Tc` = Critical temperature (Kelvin)
- `T` = Operating temperature (Kelvin)

#### 3.2.2 Coherence Length

The BCS coherence length:

```
ξ₀ = ℏ × vF / (π × Δ₀)
```

Temperature-dependent coherence length:

```
ξ(T) = ξ₀ × √(Δ₀ / Δ(T))
```

Where:
- `ξ₀` = Coherence length at T = 0
- `ℏ` = Reduced Planck constant (1.055 × 10⁻³⁴ J·s)
- `vF` = Fermi velocity (m/s)
- `Δ₀` = Energy gap at T = 0

#### 3.2.3 London Penetration Depth

```
λL(T) = λL(0) / √(1 - (T/Tc)⁴)
```

At zero temperature:

```
λL(0) = √(m / (μ₀ × ns × e²))
```

Where:
- `λL` = London penetration depth (nm)
- `m` = Electron mass (9.109 × 10⁻³¹ kg)
- `μ₀` = Permeability (4π × 10⁻⁷ H/m)
- `ns` = Superelectron density (m⁻³)
- `e` = Elementary charge (1.602 × 10⁻¹⁹ C)

#### 3.2.4 Critical Field

For Type I superconductors:

```
Bc(T) = Bc(0) × [1 - (T/Tc)²]
```

For Type II superconductors:

```
Bc1(T) = Bc1(0) × [1 - (T/Tc)²]
Bc2(T) = Bc2(0) × [1 - (T/Tc)²]
```

Where:
- `Bc1` = Lower critical field (vortex entry)
- `Bc2` = Upper critical field (superconductivity destroyed)

### 3.3 Ginzburg-Landau Theory

#### 3.3.1 GL Parameter

```
κ = λL / ξ
```

Classification:
- `κ < 1/√2`: Type I superconductor
- `κ > 1/√2`: Type II superconductor

#### 3.3.2 GL Equations

```typescript
interface GinzburgLandauTheory {
  // GL parameter
  kappa: number; // κ = λL/ξ

  // Order parameter
  orderParameter: ComplexFunction; // ψ(r)

  // Free energy functional
  freeEnergy: {
    condensationEnergy: number;
    kineticEnergy: number;
    fieldEnergy: number;
    totalEnergy: number;
  };

  // Critical fields
  criticalFields: {
    bc1: number; // Tesla
    bc2: number; // Tesla
    bcThermodynamic: number; // Tesla
  };
}
```

---



## 4. Material Classifications

### 4.1 Type I Superconductors

#### 4.1.1 Characteristics

- Pure metallic elements
- Single critical field Bc
- Complete Meissner effect (perfect diamagnetism)
- Low critical temperatures (Tc < 10 K)
- Low critical fields (Bc < 0.1 T)
- GL parameter κ < 1/√2

#### 4.1.2 Common Type I Materials

```typescript
interface TypeIMaterial {
  name: string;
  element: string;
  tc: number; // Kelvin
  bc: number; // Tesla
  lambda: number; // London depth (nm)
  xi: number; // Coherence length (nm)
}

const TYPE_I_MATERIALS: TypeIMaterial[] = [
  {
    name: 'Mercury',
    element: 'Hg',
    tc: 4.15,
    bc: 0.041,
    lambda: 39,
    xi: 230
  },
  {
    name: 'Lead',
    element: 'Pb',
    tc: 7.19,
    bc: 0.080,
    lambda: 37,
    xi: 83
  },
  {
    name: 'Tin',
    element: 'Sn',
    tc: 3.72,
    bc: 0.031,
    lambda: 51,
    xi: 230
  },
  {
    name: 'Aluminum',
    element: 'Al',
    tc: 1.18,
    bc: 0.010,
    lambda: 16,
    xi: 1600
  }
];
```

### 4.2 Type II Superconductors

#### 4.2.1 Characteristics

- Alloys and compounds
- Two critical fields (Bc1, Bc2)
- Mixed (vortex) state between Bc1 and Bc2
- Higher critical fields (>10 T possible)
- GL parameter κ > 1/√2
- Practical for high-field applications

#### 4.2.2 Vortex State

In the mixed state (Bc1 < B < Bc2):

```typescript
interface VortexState {
  // Flux quantization
  fluxQuantum: 2.067833848e-15; // Φ₀ (Wb)

  // Vortex lattice
  vortexDensity: number; // vortices/m²
  vortexSpacing: number; // meters
  latticeStructure: 'hexagonal' | 'square';

  // Vortex core
  coreRadius: number; // ~ξ (coherence length)
  normalCore: boolean; // Core is normal (non-SC)

  // Pinning
  pinningCenters: PinningCenter[];
  criticalCurrent: number; // A/m²
}

interface PinningCenter {
  type: 'grain-boundary' | 'precipitate' | 'defect' | 'columnar';
  density: number; // centers/m³
  pinningForce: number; // N
}
```

#### 4.2.3 Common Type II Materials

```typescript
const TYPE_II_MATERIALS = [
  {
    name: 'NbTi',
    composition: 'Nb-47wt%Ti',
    tc: 9.2,
    bc1: 0.02,
    bc2: 15,
    jc: 3e9, // A/m² at 4.2K, 5T
    application: 'MRI magnets',
    cost: 'low'
  },
  {
    name: 'Nb3Sn',
    composition: 'Nb₃Sn',
    tc: 18.3,
    bc1: 0.03,
    bc2: 30,
    jc: 1e10, // A/m² at 4.2K, 12T
    application: 'High-field magnets',
    cost: 'medium'
  },
  {
    name: 'MgB2',
    composition: 'MgB₂',
    tc: 39,
    bc1: 0.1,
    bc2: 74,
    jc: 1e9, // A/m² at 4.2K, 3T
    application: 'Cables, cryocooled',
    cost: 'low'
  }
];
```

### 4.3 High-Temperature Superconductors (HTSC)

#### 4.3.1 Cuprate Superconductors

Crystal structure: CuO₂ planes (superconducting layers)

```typescript
interface CuprateHTSC {
  // Chemical formula
  formula: string;
  family: 'YBCO' | 'BSCCO' | 'TBCCO' | 'HgBCO';

  // Critical parameters
  tc: number; // >77K (liquid nitrogen!)
  bc2_ab: number; // In-plane critical field
  bc2_c: number; // c-axis critical field
  anisotropy: number; // bc2_ab / bc2_c

  // Crystal structure
  latticeParameters: {
    a: number; // Å
    b: number; // Å
    c: number; // Å
  };

  // Pairing symmetry
  pairingSymmetry: 'd-wave';
  nodePositions: number[]; // Angular positions of nodes

  // Applications
  applications: string[];
}

const CUPRATE_HTSC = [
  {
    formula: 'YBa₂Cu₃O₇₋ₓ',
    family: 'YBCO',
    tc: 92,
    bc2_ab: 120,
    bc2_c: 30,
    anisotropy: 4,
    applications: ['Maglev', 'Power cables', 'Current leads']
  },
  {
    formula: 'Bi₂Sr₂Ca₂Cu₃O₁₀',
    family: 'BSCCO',
    tc: 110,
    bc2_ab: 200,
    bc2_c: 20,
    anisotropy: 10,
    applications: ['Wires', 'Tapes', 'Magnets']
  },
  {
    formula: 'HgBa₂Ca₂Cu₃O₈',
    family: 'HgBCO',
    tc: 133, // Record for cuprates!
    bc2_ab: 150,
    bc2_c: 40,
    anisotropy: 3.75,
    applications: ['Research']
  }
];
```

#### 4.3.2 Iron-Based Superconductors

```typescript
const IRON_PNICTIDES = [
  {
    formula: 'LaFeAsO₁₋ₓFₓ',
    family: '1111',
    tc: 26,
    description: 'First iron-based SC (2008)'
  },
  {
    formula: 'Ba₁₋ₓKₓFe₂As₂',
    family: '122',
    tc: 38,
    description: 'Most studied family'
  },
  {
    formula: 'FeSe',
    family: '11',
    tc: 8, // Bulk; >100K in monolayer!
    description: 'Simplest structure'
  }
];
```

---




---

## A.1 Material-classification envelope

The Phase 1 envelope groups superconductors by family (LTS — low-temperature, conventional BCS; HTS — high-temperature cuprates; iron-based pnictides/chalcogenides; magnesium diboride; hydrides under pressure) with the canonical fields: chemical formula, critical temperature `T_c` in kelvin, upper critical field `H_c2` in tesla, critical current density `J_c` in A/cm^2 at the stated temperature and field, and the operating-condition envelope (cryogen, mechanical strain, anisotropy axis). Material data follows the reporting conventions of IEC 61788 series (superconductivity test methods) and the IEEE Council on Superconductivity's Standards Coordinating Committee outputs.

## A.2 Critical-parameter reporting

A characterisation report MUST list `T_c` (resistive transition midpoint and 90/10 onset/offset definitions), `H_c2(T)` extrapolated to T = 0, `J_c(T, B)` at the canonical operating point, and the n-value of the E–J curve. Reports MUST cite IEC 61788-1/-2/-3/-4 for measurement methodology and MUST include a magnetic-field-orientation diagram for anisotropic conductors (REBCO, Bi-2212, MgB2 wires).

## A.3 Cryogen and operating-condition descriptor

The cryogen descriptor enumerates the working fluid (liquid helium at 4.2 K, supercritical helium, liquid neon at 27 K, liquid hydrogen at 20 K, liquid nitrogen at 77 K, sub-cooled nitrogen at 65 K, cryocooler with first/second stage temperatures). Each entry carries the boil-off rate (W/L for liquid baths, W of refrigeration capacity for cryocoolers), the design temperature margin (T_margin = T_c – T_op), and the contingency for off-design events.

## A.4 Josephson-junction parameters

Every Josephson-junction envelope carries: critical current `I_c`, normal-state resistance `R_n`, IcRn product (related to the gap), Stewart-McCumber parameter `β_c`, plasma frequency, and the SFQ pulse area for digital applications. Junctions for qubit applications carry additional fields: anharmonicity, coherence time T1/T2/T2*, and the EJ/EC ratio.

## A.5 SQUID and superconducting-detector descriptors

SQUID descriptors carry sensor type (DC SQUID, RF SQUID, nano-SQUID), flux noise floor in `μΦ_0/√Hz`, magnetic-field noise in `fT/√Hz`, slew rate, dynamic range, and the cryogenic interface (4 K stage, 100 mK stage, dilution refrigerator integration). Single-photon detectors (SNSPD, TES) follow the same descriptor pattern with detection efficiency, dark-count rate, timing jitter, and recovery time.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/superconducting/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-superconducting-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/superconducting-host:1.0.0` ships every superconducting envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/superconducting.sh` ships sample envelope generators with no
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
ecosystem. Superconducting deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
