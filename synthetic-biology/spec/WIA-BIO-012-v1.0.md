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

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for synthetic biology, enabling the design, construction, and optimization of engineered biological systems for beneficial applications.

### 1.2 Scope

The standard covers:
- Standardized genetic parts (BioBricks, MoClo, etc.)
- DNA assembly protocols
- Genetic circuit modeling and design
- Metabolic pathway engineering
- Cell-free expression systems
- Biosafety and ethical guidelines

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize synthetic biology while ensuring responsible development that benefits humanity and protects the environment.

### 1.4 Terminology

- **BioBrick**: Standardized DNA part with defined prefix/suffix sequences
- **Genetic Circuit**: Network of regulatory genetic elements
- **Metabolic Flux**: Rate of flow through a metabolic pathway
- **Chassis Organism**: Host organism for engineered genetic systems
- **Part Registry**: Repository of characterized biological parts

---

## 2. Genetic Part Standards

### 2.1 BioBrick Standard (RFC10)

The BioBrick standard defines standardized DNA parts with compatible restriction sites:

```
Prefix:  EcoRI - XbaI
Part:    [Genetic Element]
Suffix:  SpeI - PstI
```

Assembly format:
```
5'-GAATTC-GCGGCCGC-T-TCTAGA-[Part]-TACTAG-AGCGGCCGC-CTGCAG-3'
     EcoRI   NotI      XbaI          SpeI    NotI    PstI
```

### 2.2 Part Categories

#### 2.2.1 Promoters
Control transcription initiation:

| Part | Type | Strength | Organism |
|------|------|----------|----------|
| BBa_J23100 | Constitutive | 1791 RPU | E. coli |
| BBa_J23119 | Constitutive | 1 RPU | E. coli |
| BBa_R0011 | Inducible (LacI) | Variable | E. coli |
| BBa_R0040 | Inducible (TetR) | Variable | E. coli |

Strength calculation:
```
P = Pmax × f(inducer)
```

For Hill function regulation:
```
P = Pmax × [I]ⁿ / (Kd + [I]ⁿ)
```

Where:
- `P` = Promoter activity (RPU)
- `Pmax` = Maximum activity
- `[I]` = Inducer concentration
- `Kd` = Dissociation constant
- `n` = Hill coefficient (cooperativity)

#### 2.2.2 Ribosome Binding Sites (RBS)
Control translation initiation:

| Part | Type | Strength | ΔG (kcal/mol) |
|------|------|----------|---------------|
| BBa_B0034 | Strong | High | -12.1 |
| BBa_B0032 | Medium | Medium | -9.8 |
| BBa_B0033 | Weak | Low | -6.5 |

Translation initiation rate:
```
TIR = K × exp(-ΔG / RT)
```

Where:
- `TIR` = Translation Initiation Rate
- `K` = Proportionality constant
- `ΔG` = Free energy of ribosome binding
- `R` = Gas constant (1.987 cal/mol·K)
- `T` = Temperature (K)

#### 2.2.3 Coding Sequences (CDS)
Protein-coding regions:

| Part | Protein | Function | Size (bp) |
|------|---------|----------|-----------|
| BBa_E0040 | GFP | Fluorescence | 720 |
| BBa_E1010 | RFP | Fluorescence | 678 |
| BBa_K082003 | Luciferase | Bioluminescence | 1653 |

Codon optimization:
```
CAI = exp(Σ ln(w_i) / L)
```

Where:
- `CAI` = Codon Adaptation Index
- `w_i` = Relative adaptiveness of codon i
- `L` = Number of codons

#### 2.2.4 Terminators
Stop transcription:

| Part | Type | Efficiency |
|------|------|------------|
| BBa_B0015 | Double | >95% |
| BBa_B0010 | Single | ~85% |

Termination efficiency:
```
η = (T_background - T_part) / T_background
```

Where:
- `η` = Termination efficiency
- `T_background` = Background transcription
- `T_part` = Transcription with terminator

### 2.3 MoClo (Modular Cloning) Standard

Hierarchical assembly using Type IIS restriction enzymes:

```
Level 0: Individual parts
Level 1: Transcription units (promoter-RBS-CDS-terminator)
Level 2: Multi-gene constructs
Level 3+: Complex assemblies
```

Golden Gate reaction:
```
BsaI/BpiI digestion + T4 ligase
37°C (digestion) / 16°C (ligation) cycles
```

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

## 5. Metabolic Engineering

### 5.1 Flux Balance Analysis (FBA)

Optimization problem:
```
Maximize: c^T × v
Subject to: S × v = 0
            v_min ≤ v ≤ v_max
```

Where:
- `v` = Flux vector
- `S` = Stoichiometric matrix
- `c` = Objective function coefficients

### 5.2 Michaelis-Menten Kinetics

Enzyme reaction rate:
```
v = (v_max × [S]) / (Km + [S])
```

For multiple substrates:
```
v = (v_max × [A] × [B]) / ((Ka + [A]) × (Kb + [B]))
```

### 5.3 Metabolic Control Analysis

Flux control coefficient:
```
C_E^J = (∂J/∂E) × (E/J)
```

Where:
- `C_E^J` = Control coefficient of enzyme E on flux J
- `∂J/∂E` = Change in flux per change in enzyme
- `E/J` = Normalization factor

Summation theorem:
```
Σ C_i^J = 1
```

### 5.4 Pathway Optimization

Dynamic programming approach:

```
1. Identify rate-limiting steps
2. Calculate flux control coefficients
3. Overexpress high-control enzymes
4. Delete competing pathways
5. Balance cofactor usage
6. Optimize gene expression levels
```

Theoretical maximum yield:
```
Y_max = (mol product / mol substrate) × stoichiometry
```

---

## 6. Cell-Free Systems

### 6.1 PURE System Components

Minimal protein synthesis system:

| Component | Concentration | Function |
|-----------|---------------|----------|
| T7 RNAP | 100 nM | Transcription |
| Ribosomes | 1.5 μM | Translation |
| Amino acids | 2 mM each | Protein building blocks |
| NTPs | 1.5 mM each | Energy, transcription |
| tRNAs | Variable | Translation |
| Factors | Variable | Initiation, elongation |

### 6.2 Crude Extract Systems

E. coli S30 extract:

```
Protein synthesis rate = 1-5 μg/mL/hr
Duration: 4-8 hours
Yield: 100-2000 μg/mL
```

Optimization parameters:
- Mg²⁺ concentration: 12-18 mM
- K⁺ concentration: 80-150 mM
- pH: 7.2-7.8
- Temperature: 30-37°C

### 6.3 Applications

1. **Rapid Prototyping**: Test genetic circuits in hours
2. **Toxic Proteins**: Express without killing cells
3. **Non-Natural Chemistry**: Use non-standard amino acids
4. **Metabolic Engineering**: Cell-free metabolic pathways
5. **Biosensors**: In vitro diagnostic systems

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

## 8. Biosafety and Biosecurity

### 8.1 Biosafety Levels

| Level | Organisms | Containment |
|-------|-----------|-------------|
| BSL-1 | Non-pathogenic | Basic lab practices |
| BSL-2 | Moderate risk | BSL-1 + biosafety cabinet |
| BSL-3 | Serious disease | BSL-2 + specialized equipment |
| BSL-4 | Lethal pathogens | Maximum containment |

### 8.2 Risk Assessment Matrix

```
Risk = Likelihood × Severity

Likelihood: 1 (rare) to 5 (common)
Severity: 1 (negligible) to 5 (catastrophic)

Risk Level:
1-5: Low (acceptable)
6-10: Medium (review required)
11-15: High (mitigation required)
16-25: Extreme (prohibit)
```

### 8.3 Biocontainment Strategies

#### 8.3.1 Kill Switches
```
Toxin-antitoxin system:
- Constitutive toxin expression
- Inducible antitoxin expression
- Removal of inducer → cell death
```

Examples:
- ccdB/ccdA (DNA gyrase inhibition)
- mazF/mazE (mRNA cleavage)
- hok/sok (membrane depolarization)

#### 8.3.2 Auxotrophy
```
Delete essential biosynthesis genes:
- ΔthyA (thymine)
- ΔdapA (diaminopimelic acid)
- Multiple deletions for redundancy
```

#### 8.3.3 Semantic Containment
```
Recode organism with non-standard genetic code:
- Replace UAG (amber) codon function
- Require synthetic amino acids
- Orthogonal translation system
```

### 8.4 Dual-Use Research Concerns

Screen for:
- Enhanced pathogenicity
- Increased transmissibility
- Antibiotic resistance transfer
- Toxin production
- Immune evasion

Mitigation:
- Institutional Biosafety Committee (IBC) review
- Ethics committee approval
- Secure data/materials handling
- Publication review

---

## 9. Implementation Guidelines

### 9.1 Required Components

Any WIA-BIO-012 compliant system must include:

1. **Part Registry**: Documented biological parts
2. **Assembly Protocol**: DNA construction method
3. **Circuit Model**: Computational predictions
4. **Safety Assessment**: Risk analysis
5. **Documentation**: Complete characterization data

### 9.2 API Interface

#### 9.2.1 Calculate Promoter Strength
```typescript
interface PromoterParams {
  pmax: number;              // Maximum strength (RPU)
  inducerConcentration: number; // M
  kd: number;                // Dissociation constant (M)
  hillCoefficient: number;   // Cooperativity
}

interface PromoterResponse {
  strength: number;          // Current strength (RPU)
  foldInduction: number;     // Induced/basal ratio
  saturation: number;        // % of maximum (0-100)
}
```

#### 9.2.2 Design Genetic Circuit
```typescript
interface CircuitDesign {
  parts: string[];           // BioBrick IDs
  host: string;              // Chassis organism
  purpose: string;           // Circuit function
  optimization?: 'speed' | 'yield' | 'robustness';
}

interface CircuitResult {
  expectedExpression: number;
  plasmidSize: number;
  warnings: string[];
  assemblyMethod: string;
}
```

#### 9.2.3 Optimize Pathway
```typescript
interface PathwayOptimization {
  target: string;            // Product
  substrate: string;         // Input
  host: string;              // Organism
  constraints: {
    maxGenes?: number;
    targetYield?: number;    // mol/mol
    cofactorBalance?: boolean;
  };
}

interface PathwayResult {
  genes: string[];
  theoreticalYield: number;
  fluxDistribution: Record<string, number>;
  bottlenecks: string[];
}
```

### 9.3 Data Formats

#### 9.3.1 BioBrick Part
```json
{
  "id": "BBa_J23100",
  "type": "promoter",
  "sequence": "TTGACAGCTAGCTCAGTCCTAGGTATTATGCTAGC",
  "length": 35,
  "strength": 1791,
  "organism": "E. coli",
  "characterization": {
    "method": "flow_cytometry",
    "units": "RPU",
    "conditions": "LB, 37C, exponential phase"
  }
}
```

#### 9.3.2 Genetic Circuit
```json
{
  "id": "CIR-001",
  "name": "Inducible GFP",
  "parts": [
    "BBa_R0011",
    "BBa_B0034",
    "BBa_E0040",
    "BBa_B0015"
  ],
  "host": "E. coli DH5α",
  "plasmid": "pSB1C3",
  "inducer": {
    "name": "IPTG",
    "concentration": 1e-3,
    "units": "M"
  }
}
```

### 9.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| B001 | Invalid part combination | Check compatibility |
| B002 | Assembly method mismatch | Update part design |
| B003 | Exceeded size limit | Reduce construct size |
| B004 | Biosafety violation | Revise design |
| B005 | Expression too high | Weaken promoter/RBS |
| B006 | Pathway imbalance | Adjust enzyme levels |

---

## 10. References

### 10.1 Foundational Papers

1. Elowitz, M.B. & Leibler, S. (2000). "A synthetic oscillatory network of transcriptional regulators"
2. Gardner, T.S. et al. (2000). "Construction of a genetic toggle switch in E. coli"
3. Endy, D. (2005). "Foundations for engineering biology"
4. Keasling, J.D. (2008). "Synthetic biology for synthetic chemistry"
5. Church, G.M. et al. (2014). "Realizing the potential of synthetic biology"

### 10.2 Assembly Standards

- RFC10: BioBrick Assembly
- RFC25: Freiburg Assembly
- RFC1000: MoClo Assembly
- RFC43: iBrick Assembly

### 10.3 Biological Constants

| Constant | Symbol | Value | Unit |
|----------|--------|-------|------|
| RNA polymerase speed | v_RNAP | 40-80 | nt/s |
| Ribosome speed | v_rib | 15-20 | aa/s |
| mRNA half-life | t₁/₂ | 3-8 | min |
| Protein half-life | t₁/₂ | 10-120 | min |
| Plasmid copy number | n | 15-300 | copies/cell |

### 10.4 WIA Standards

- WIA-INTENT: Intent-based biological queries
- WIA-OMNI-API: Universal API gateway
- WIA-DATA: Data management standards
- WIA-SOCIAL: Collaborative engineering

---

## Appendix A: Example Calculations

### A.1 Promoter Strength Calculation

```
Given:
- Pmax = 1000 RPU (BBa_J23100)
- [IPTG] = 1 mM = 1×10⁻³ M
- Kd = 100 nM = 1×10⁻⁷ M
- n = 2 (Hill coefficient)

Calculation:
P = 1000 × (1×10⁻³)² / (1×10⁻⁷ + (1×10⁻³)²)
P = 1000 × 1×10⁻⁶ / (1×10⁻⁷ + 1×10⁻⁶)
P = 1000 × 1×10⁻⁶ / 1.1×10⁻⁶
P ≈ 909 RPU

Result: ~91% of maximum strength
```

### A.2 Gene Expression Dynamics

```
Given:
- k_tx = 0.5 /s (transcription)
- k_tl = 0.1 /s (translation)
- k_deg_m = 0.231 /min (mRNA, t₁/₂ = 3 min)
- k_deg_p = 0.0069 /min (protein, t₁/₂ = 100 min)

Steady state:
[mRNA]_ss = k_tx / k_deg_m = 0.5 / 0.00385 = 130 molecules
[Protein]_ss = k_tl × [mRNA]_ss / k_deg_p
             = 0.1 × 130 / 0.0069
             ≈ 1,884 molecules

Time to 90% steady state:
t_90 ≈ 2.3 / k_deg_p = 2.3 / 0.0069 ≈ 333 minutes
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-012 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
