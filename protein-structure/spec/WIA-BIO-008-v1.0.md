# WIA-BIO-008: Protein Structure Specification v1.0

> **Standard ID:** WIA-BIO-008
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Bioinformatics Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Structure Determination Methods](#2-structure-determination-methods)
3. [Prediction Algorithms](#3-prediction-algorithms)
4. [Quality Assessment](#4-quality-assessment)
5. [Molecular Dynamics](#5-molecular-dynamics)
6. [Protein-Ligand Docking](#6-protein-ligand-docking)
7. [Structural Comparison](#7-structural-comparison)
8. [Implementation Guidelines](#8-implementation-guidelines)
9. [Data Formats](#9-data-formats)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines computational methods for protein structure prediction, analysis, validation, and molecular simulation, integrating modern AI-based approaches with traditional biophysical techniques.

### 1.2 Scope

The standard covers:
- Experimental and computational structure determination
- AI-powered prediction (AlphaFold, RoseTTAFold)
- Quality metrics and validation
- Molecular dynamics simulations
- Protein-ligand interactions
- Structural databases and formats

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard democratizes access to protein structure analysis, accelerating biomedical research and drug discovery for the benefit of all humanity.

### 1.4 Terminology

- **Primary Structure**: Amino acid sequence
- **Secondary Structure**: Local backbone conformations (α-helix, β-sheet)
- **Tertiary Structure**: 3D arrangement of all atoms
- **Quaternary Structure**: Multi-chain complex arrangement
- **Residue**: Individual amino acid in the sequence
- **Backbone**: N-Cα-C atoms forming the protein chain

---

## 2. Structure Determination Methods

### 2.1 X-ray Crystallography

#### 2.1.1 Principles

Protein crystals diffract X-rays to produce diffraction patterns:

```
I(hkl) = |F(hkl)|²
F(hkl) = ∑ⱼ fⱼ exp[2πi(hxⱼ + kyⱼ + lzⱼ)]
```

Where:
- `I(hkl)` = Intensity of reflection
- `F(hkl)` = Structure factor
- `fⱼ` = Atomic scattering factor
- `(h,k,l)` = Miller indices
- `(xⱼ,yⱼ,zⱼ)` = Atomic coordinates

#### 2.1.2 Resolution

Resolution determines atomic detail:

```
d = λ / (2 sin θ)
```

Where:
- `d` = Resolution (Å)
- `λ` = X-ray wavelength
- `θ` = Diffraction angle

**Resolution categories:**
- High: <2.0 Å (atomic detail)
- Medium: 2.0-3.0 Å (backbone trace)
- Low: >3.0 Å (overall fold)

### 2.2 NMR Spectroscopy

#### 2.2.1 Nuclear Overhauser Effect (NOE)

Distance constraints from NOE:

```
NOE ∝ r⁻⁶
```

Where `r` = Distance between nuclei (<5 Å)

#### 2.2.2 Chemical Shifts

Predict secondary structure from chemical shifts:

```
Δδ = δ_observed - δ_random_coil
```

- `Δδ > 0`: α-helix propensity
- `Δδ < 0`: β-sheet propensity

### 2.3 Cryo-Electron Microscopy

#### 2.3.1 Single Particle Analysis

Reconstruct 3D structure from 2D projections:

```
f(r) = ∫ F(k) exp(2πik·r) dk
```

Where:
- `f(r)` = 3D density map
- `F(k)` = Fourier transform of projections
- `k` = Reciprocal space vector

#### 2.3.2 Resolution

Fourier Shell Correlation (FSC):

```
FSC(k) = ∑ F₁(k)F₂*(k) / √(∑|F₁(k)|² ∑|F₂(k)|²)
```

Resolution at FSC = 0.143

---

## 3. Prediction Algorithms

### 3.1 AlphaFold

#### 3.1.1 Architecture

AlphaFold uses Evoformer + Structure module:

```
1. Multiple Sequence Alignment (MSA)
2. Pair representation
3. Evoformer blocks (48 layers)
4. Structure module (8 iterations)
5. Output: 3D coordinates + confidence
```

#### 3.1.2 Confidence Metrics

**pLDDT (predicted Local Distance Difference Test):**

```
pLDDT = 100 × (1/N) ∑ᵢ P(|error_i| < threshold)
```

Where:
- `N` = Number of residues
- `threshold` = 15 Å

**Confidence levels:**
- Very high: pLDDT > 90
- High: 70 < pLDDT ≤ 90
- Medium: 50 < pLDDT ≤ 70
- Low: pLDDT ≤ 50

#### 3.1.3 PAE (Predicted Aligned Error)

```
PAE(i,j) = Expected error in position of residue i
           when aligned on residue j
```

Low PAE (<5 Å) indicates confident relative positioning.

### 3.2 RoseTTAFold

#### 3.2.1 Three-Track Architecture

```
1D track: Sequence features
2D track: Pair-wise interactions
3D track: Coordinate frames

Integrated through attention mechanisms
```

#### 3.2.2 Scoring Function

Rosetta energy function:

```
E_total = w₁E_vdw + w₂E_elec + w₃E_solv + w₄E_hbond + w₅E_rama
```

Where:
- `E_vdw` = Van der Waals energy
- `E_elec` = Electrostatic energy
- `E_solv` = Solvation energy
- `E_hbond` = Hydrogen bond energy
- `E_rama` = Ramachandran energy
- `wᵢ` = Weights

### 3.3 Homology Modeling

#### 3.3.1 Template Identification

Sequence identity threshold:

```
Identity = (N_match / N_aligned) × 100%
```

**Reliability:**
- >50%: High accuracy
- 30-50%: Moderate accuracy
- <30%: Low accuracy (twilight zone)

#### 3.3.2 Model Building

```
1. Template selection (BLAST, HHsearch)
2. Alignment (ClustalW, MUSCLE)
3. Model construction (MODELLER, SWISS-MODEL)
4. Loop modeling
5. Side-chain refinement
6. Energy minimization
```

---

## 4. Quality Assessment

### 4.1 RMSD (Root Mean Square Deviation)

#### 4.1.1 Formula

```
RMSD = √[(1/N) ∑ᵢ₌₁ᴺ (rᵢ - r'ᵢ)²]
```

Where:
- `N` = Number of atoms
- `rᵢ` = Position in structure 1
- `r'ᵢ` = Position in structure 2 (after optimal superposition)

#### 4.1.2 Calculation

```
1. Align structures (Kabsch algorithm)
2. Calculate squared distances
3. Average and take square root
```

**Typical values:**
- RMSD < 1 Å: Near-identical
- RMSD 1-2 Å: Similar
- RMSD 2-4 Å: Different conformations
- RMSD > 4 Å: Different folds

### 4.2 TM-Score

#### 4.2.1 Formula

```
TM-score = max[1/L_target ∑ᵢ₌₁ᴸᵃˡⁱᵍⁿ 1/(1 + (dᵢ/d₀)²)]
```

Where:
- `L_target` = Target length
- `L_align` = Aligned residues
- `dᵢ` = Distance between residues
- `d₀ = 1.24 ∛(L_target - 15) - 1.8`

#### 4.2.2 Interpretation

```
TM-score > 0.5: Same fold
TM-score 0.3-0.5: Similar topology
TM-score < 0.3: Different fold
```

### 4.3 GDT-TS (Global Distance Test Total Score)

```
GDT-TS = (P₁ + P₂ + P₄ + P₈) / 4
```

Where `Pₙ` = Percentage of residues within n Å after superposition

### 4.4 Ramachandran Plot

Validate backbone dihedral angles:

```
φ (phi) = C_{i-1} - N_i - Cα_i - C_i
ψ (psi) = N_i - Cα_i - C_i - N_{i+1}
```

**Allowed regions:**
- Core (most favored): >98%
- Allowed: 1-2%
- Generously allowed: <0.5%
- Disallowed: 0%

### 4.5 MolProbity Score

Comprehensive quality metric:

```
MolProbity = log(1 + clashscore) + max(0, 2-Rwork) +
             max(0, percentile_rama - 95)
```

**Good models:** MolProbity < 2.0

---

## 5. Molecular Dynamics

### 5.1 Force Fields

#### 5.1.1 Energy Function

```
E_total = E_bonded + E_non-bonded

E_bonded = ∑ K_b(r - r₀)² + ∑ K_θ(θ - θ₀)² +
           ∑ (V_n/2)[1 + cos(nφ - γ)]

E_non-bonded = ∑ [A_ij/r_ij¹² - B_ij/r_ij⁶] +
               ∑ q_iq_j/(4πε₀r_ij)
```

Where:
- `K_b, K_θ` = Force constants
- `V_n` = Dihedral barrier
- `A_ij, B_ij` = Lennard-Jones parameters
- `q_i` = Partial charges

#### 5.1.2 Common Force Fields

- **AMBER**: Assisted Model Building with Energy Refinement
- **CHARMM**: Chemistry at HARvard Macromolecular Mechanics
- **GROMOS**: GROningen MOlecular Simulation
- **OPLS**: Optimized Potentials for Liquid Simulations

### 5.2 Integration Algorithms

#### 5.2.1 Verlet Algorithm

```
r(t + Δt) = 2r(t) - r(t - Δt) + a(t)Δt²
v(t) = [r(t + Δt) - r(t - Δt)] / (2Δt)
```

#### 5.2.2 Leap-Frog Algorithm

```
v(t + Δt/2) = v(t - Δt/2) + a(t)Δt
r(t + Δt) = r(t) + v(t + Δt/2)Δt
```

**Typical timestep:** 1-2 femtoseconds (fs)

### 5.3 Thermostats and Barostats

#### 5.3.1 Berendsen Thermostat

```
dT/dt = (T₀ - T) / τ_T
```

Where:
- `T₀` = Target temperature
- `T` = Current temperature
- `τ_T` = Coupling constant

#### 5.3.2 Nosé-Hoover Thermostat

```
dv_i/dt = F_i/m_i - ξv_i
dξ/dt = (T - T₀) / (Q τ_T²)
```

More rigorous canonical ensemble.

### 5.4 Analysis Metrics

#### 5.4.1 RMSF (Root Mean Square Fluctuation)

```
RMSF_i = √[⟨(r_i - ⟨r_i⟩)²⟩]
```

Measures residue flexibility.

#### 5.4.2 Radius of Gyration

```
R_g = √[(∑ᵢ m_i r_i²) / (∑ᵢ m_i)]
```

Measures protein compactness.

---

## 6. Protein-Ligand Docking

### 6.1 Scoring Functions

#### 6.1.1 Binding Affinity

Gibbs free energy:

```
ΔG_bind = ΔH - TΔS
```

Empirical scoring:

```
ΔG = ΔG_vdw + ΔG_elec + ΔG_hbond + ΔG_desolv + ΔG_tor
```

Where:
- `ΔG_vdw` = Van der Waals contribution
- `ΔG_elec` = Electrostatic contribution
- `ΔG_hbond` = Hydrogen bonding
- `ΔG_desolv` = Desolvation penalty
- `ΔG_tor` = Torsional entropy

#### 6.1.2 Dissociation Constant

```
K_d = exp(ΔG / RT)
```

Where:
- `R` = Gas constant (1.987 cal/mol·K)
- `T` = Temperature (K)

**Typical values:**
- Strong binding: K_d < 1 nM
- Moderate: 1 nM - 1 μM
- Weak: >1 μM

### 6.2 Docking Algorithms

#### 6.2.1 AutoDock Vina

Iterated local search global optimizer:

```
1. Generate random conformations
2. Local optimization (Broyden-Fletcher-Goldfarb-Shanno)
3. Mutation and selection
4. Iterate until convergence
```

#### 6.2.2 Glide (Schrödinger)

```
1. Conformational search
2. Rough scoring
3. Grid-based energy minimization
4. Refined scoring
```

### 6.3 Binding Site Prediction

#### 6.3.1 Geometric Methods

**Pocket detection:**
- FPocket: Alpha sphere method
- SURFNET: Spherical probe
- CASTp: Computational geometry

#### 6.3.2 Energy-Based Methods

```
E_probe = ∑ V_LJ(r) + ∑ V_elec(r)
```

Map favorable interaction sites.

---

## 7. Structural Comparison

### 7.1 Alignment Algorithms

#### 7.1.1 Kabsch Algorithm

Optimal rotation matrix:

```
1. Center both structures at origin
2. Compute covariance matrix: H = ∑ᵢ (rᵢ ⊗ r'ᵢ)
3. SVD: H = UΣVᵀ
4. Rotation: R = VUᵀ
5. Apply rotation to minimize RMSD
```

#### 7.1.2 TM-align

Dynamic programming alignment maximizing TM-score:

```
1. Initial alignment (secondary structure)
2. Iterative refinement
3. Calculate optimal superposition
4. Update alignment
5. Converge to maximum TM-score
```

### 7.2 Secondary Structure Assignment

#### 7.2.1 DSSP Algorithm

Define Secondary Structure of Proteins:

```
E_hbond = 0.084 × [1/r_ON + 1/r_CH - 1/r_OH - 1/r_CN] kcal/mol
```

If E < -0.5 kcal/mol: hydrogen bond exists

**Assignments:**
- H: α-helix
- E: β-strand
- T: Turn
- C: Coil

#### 7.2.2 STRIDE

Secondary structure from φ/ψ angles and hydrogen bonds.

---

## 8. Implementation Guidelines

### 8.1 Required Components

Any WIA-BIO-008 compliant system must include:

1. **Structure Predictor**: AlphaFold or equivalent
2. **Quality Validator**: RMSD, TM-score calculations
3. **Docking Engine**: Protein-ligand interaction
4. **MD Simulator**: Molecular dynamics (optional)
5. **Format Converter**: PDB, mmCIF, mmTF support

### 8.2 API Interface

#### 8.2.1 Structure Prediction

```typescript
interface PredictionRequest {
  sequence: string;
  method: 'alphafold' | 'rosettafold' | 'homology';
  templates?: string[];
  msa?: MultipleSequenceAlignment;
}

interface PredictionResponse {
  structure: ProteinStructure;
  confidence: ConfidenceMetrics;
  secondaryStructure: SecondaryStructureAnnotation;
  metadata: PredictionMetadata;
}
```

#### 8.2.2 Quality Assessment

```typescript
interface QualityRequest {
  structure: ProteinStructure;
  reference?: ProteinStructure;
}

interface QualityResponse {
  rmsd?: number;
  tmScore?: number;
  gdtTS?: number;
  ramachandran: RamachandranStats;
  molProbity: MolProbityScore;
  clashScore: number;
}
```

#### 8.2.3 Docking

```typescript
interface DockingRequest {
  protein: ProteinStructure;
  ligand: string | LigandStructure;
  bindingSite?: BindingSite;
  exhaustiveness?: number;
  numModes?: number;
}

interface DockingResponse {
  poses: DockingPose[];
  bindingAffinity: number;
  interactions: Interaction[];
  rmsd?: number;
}
```

### 8.3 Data Formats

#### 8.3.1 PDB Format

```
ATOM    1  N   MET A   1      -8.901  -4.127  -0.555  1.00 20.00           N
ATOM    2  CA  MET A   1      -8.608  -3.135  -1.582  1.00 20.00           C
ATOM    3  C   MET A   1      -7.117  -2.964  -1.897  1.00 20.00           C
```

Fields: Record, serial, atom, residue, chain, resSeq, x, y, z, occupancy, B-factor

#### 8.3.2 mmCIF Format

```
loop_
_atom_site.group_PDB
_atom_site.id
_atom_site.type_symbol
_atom_site.label_atom_id
_atom_site.label_comp_id
_atom_site.label_asym_id
_atom_site.Cartn_x
_atom_site.Cartn_y
_atom_site.Cartn_z
ATOM 1 N N MET A -8.901 -4.127 -0.555
```

#### 8.3.3 JSON Format (WIA-BIO-008)

```json
{
  "protein": {
    "id": "P12345",
    "sequence": "MKTAYIAK...",
    "chains": [
      {
        "id": "A",
        "residues": [
          {
            "index": 1,
            "name": "MET",
            "atoms": [
              {"name": "N", "position": [-8.901, -4.127, -0.555]},
              {"name": "CA", "position": [-8.608, -3.135, -1.582]}
            ]
          }
        ]
      }
    ]
  }
}
```

### 8.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| B001 | Invalid sequence | Check sequence format |
| B002 | Structure not found | Verify PDB ID or path |
| B003 | Alignment failed | Adjust parameters |
| B004 | Low confidence | Use experimental structure |
| B005 | Docking failed | Check binding site |
| B006 | Invalid format | Convert to PDB/mmCIF |

---

## 9. Data Formats

### 9.1 Sequence Formats

#### 9.1.1 FASTA

```
>sp|P12345|PROTEIN_HUMAN Protein name
MKTAYIAKQRQISFVKSHFSRQLEERLGLIEVQAPILSRVGDGTQDNLSGAEKAVQVKV
KALPDAQFEVVHSLAKWKRQTLGQHDFSAGEGLYTHMKALRPDEDRLSPLHSVYVDQWD
```

#### 9.1.2 Single-letter Codes

```
A: Alanine      M: Methionine
C: Cysteine     N: Asparagine
D: Aspartic     P: Proline
E: Glutamic     Q: Glutamine
F: Phenylalanine R: Arginine
G: Glycine      S: Serine
H: Histidine    T: Threonine
I: Isoleucine   V: Valine
K: Lysine       W: Tryptophan
L: Leucine      Y: Tyrosine
```

### 9.2 Structure Repositories

#### 9.2.1 Protein Data Bank (PDB)

- **URL**: https://www.rcsb.org/
- **Entries**: >200,000 structures
- **Format**: PDB, mmCIF, mmTF

#### 9.2.2 AlphaFold Database

- **URL**: https://alphafold.ebi.ac.uk/
- **Entries**: >200 million predictions
- **Coverage**: UniProt proteins

#### 9.2.3 SWISS-MODEL

- **URL**: https://swissmodel.expasy.org/
- **Type**: Homology modeling server
- **Quality**: Automated assessment

---

## 10. References

### 10.1 Key Publications

1. Jumper et al. (2021). "Highly accurate protein structure prediction with AlphaFold". Nature.
2. Baek et al. (2021). "Accurate prediction of protein structures and interactions using RoseTTAFold". Science.
3. Zhang & Skolnick (2004). "Scoring function for automated assessment of protein structure template quality". Proteins.
4. Trott & Olson (2010). "AutoDock Vina: improving the speed and accuracy of docking". J Comput Chem.
5. Abraham et al. (2015). "GROMACS: High performance molecular simulations". SoftwareX.

### 10.2 Software Tools

| Tool | Purpose | Reference |
|------|---------|-----------|
| AlphaFold | Structure prediction | DeepMind |
| RoseTTAFold | Structure prediction | Baker Lab |
| PyMOL | Visualization | Schrödinger |
| GROMACS | Molecular dynamics | Uppsala University |
| AutoDock Vina | Docking | Scripps Research |
| DSSP | Secondary structure | CMBI |

### 10.3 Databases

- **PDB**: Protein Data Bank
- **UniProt**: Protein sequences and annotations
- **Pfam**: Protein families
- **SCOP**: Structural Classification of Proteins
- **CATH**: Class, Architecture, Topology, Homology

---

## Appendix A: Example Calculations

### A.1 RMSD Calculation

```
Given two aligned structures:
Structure 1: [(0,0,0), (1,0,0), (1,1,0)]
Structure 2: [(0.1,0.1,0), (1.1,0.1,0), (1.1,1.1,0)]

Differences: [(0.1,0.1,0), (0.1,0.1,0), (0.1,0.1,0)]
Squared: [0.02, 0.02, 0.02]
Mean: 0.02
RMSD = √0.02 = 0.141 Å
```

### A.2 TM-Score Calculation

```
Given:
- L_target = 100 residues
- L_align = 90 residues
- Average distance = 2 Å
- d₀ = 1.24 × ∛(100-15) - 1.8 = 3.46 Å

TM-score = (1/100) × ∑[1/(1+(2/3.46)²)]
         = (1/100) × 90 × 0.786
         = 0.707 (Good fold similarity)
```

### A.3 Binding Affinity

```
Given:
- ΔH = -10 kcal/mol
- T = 298 K
- ΔS = -20 cal/mol·K = -0.02 kcal/mol·K

ΔG = -10 - (298 × -0.02)
   = -10 + 5.96
   = -4.04 kcal/mol

K_d = exp(-4.04 / (0.001987 × 298))
    = exp(-6.82)
    = 1.1 μM (moderate binding)
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-008 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
