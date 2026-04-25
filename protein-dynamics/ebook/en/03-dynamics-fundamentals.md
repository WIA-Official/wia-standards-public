# Chapter 2: Protein Dynamics Fundamentals

## Understanding Molecular Motion

**弘益人間 (Benefit All Humanity)**

---

## 2.1 The Physics of Protein Motion

### Proteins as Dynamic Systems

Proteins exist in constant motion at physiological temperatures. Even in a crystal, atoms vibrate with amplitudes of 0.5-2 Å. This motion is not random noise—it is functionally essential.

### The Energy Landscape View

The behavior of a protein can be understood through its free energy landscape—a multidimensional surface where each point represents a possible conformation:

```
Free Energy (G)
    ↑
    │     ╱╲
    │    ╱  ╲    ╱╲
    │   ╱    ╲  ╱  ╲
    │  ╱      ╲╱    ╲
    │ ╱                ╲
    └──────────────────────→
         Conformational Coordinate

    Minima = Stable states
    Barriers = Transition states
    Depth = Population (Boltzmann)
```

The Boltzmann distribution governs populations:

```
P(state_i) = exp(-G_i / kT) / Σ exp(-G_j / kT)
```

Where:
- G_i = Free energy of state i
- k = Boltzmann constant
- T = Temperature

### Timescales of Motion

Protein motions span an extraordinary range of timescales:

| Timescale | Motion Type | Examples | Methods |
|-----------|-------------|----------|---------|
| fs (10⁻¹⁵ s) | Bond vibrations | C-C stretch, N-H bend | IR spectroscopy |
| ps (10⁻¹² s) | Side chain rotation | Methyl rotation, ring flips | NMR T1, MD |
| ns (10⁻⁹ s) | Loop motions | Flexible loops, termini | NMR relaxation, MD |
| μs (10⁻⁶ s) | Domain motions | Hinge bending, twisting | FRET, μs-MD |
| ms (10⁻³ s) | Conformational exchange | Folding, allostery | NMR CPMG, HDX |
| s | Large rearrangements | Folding, assembly | Stopped-flow, kinetics |

---

## 2.2 Molecular Dynamics Simulation

### The Fundamental Approach

Molecular dynamics (MD) simulation numerically solves Newton's equations of motion for all atoms:

```
F_i = m_i × a_i = m_i × (d²r_i/dt²)
```

The force on each atom comes from the potential energy function (force field):

```
V = V_bonds + V_angles + V_dihedrals + V_nonbonded
```

### Force Field Components

**1. Bonded Terms**

Bond stretching (harmonic approximation):
```
V_bond = Σ (k_b/2)(r - r_0)²
```

Angle bending:
```
V_angle = Σ (k_θ/2)(θ - θ_0)²
```

Dihedral rotation:
```
V_dihedral = Σ k_φ[1 + cos(nφ - δ)]
```

**2. Non-bonded Terms**

Van der Waals (Lennard-Jones):
```
V_LJ = Σ 4ε[(σ/r)¹² - (σ/r)⁶]
```

Electrostatics (Coulomb):
```
V_elec = Σ q_i × q_j / (4πε₀r_ij)
```

### Modern Force Fields

| Force Field | Strengths | Applications |
|-------------|-----------|--------------|
| AMBER ff19SB | Protein backbone | General proteins |
| CHARMM36m | Membrane proteins | GPCRs, channels |
| OPLS-AA/M | Drug-like molecules | Drug binding |
| Martini 3 | Coarse-grained | Large systems |
| ANI-2x | Neural network | High accuracy |

### Simulation Protocol

A typical MD simulation follows this workflow:

```python
def run_md_simulation(structure, parameters):
    """Standard MD simulation protocol."""

    # 1. System preparation
    system = add_hydrogens(structure)
    system = solvate(system, water_model="TIP3P",
                     box_padding=12.0)  # Å
    system = add_ions(system, concentration=0.15)  # M NaCl

    # 2. Energy minimization
    minimize(system, tolerance=1000.0)  # kJ/mol/nm

    # 3. Equilibration
    # NVT: Temperature equilibration
    equilibrate_nvt(system, temperature=300,
                    time=100)  # ps
    # NPT: Pressure equilibration
    equilibrate_npt(system, temperature=300,
                    pressure=1.0, time=1000)  # ps

    # 4. Production MD
    trajectory = run_production(
        system,
        temperature=300,    # K
        pressure=1.0,       # bar
        time=1000,          # ns
        timestep=2.0,       # fs
        save_interval=10    # ps
    )

    return trajectory
```

---

## 2.3 Flexibility Metrics

### B-factors and RMSF

**B-factor (Temperature Factor)**
Crystallographic B-factors measure atomic displacement:

```
B = (8π²/3) × <u²>
```

Where <u²> is the mean-square displacement.

**Root Mean Square Fluctuation (RMSF)**
From MD trajectories:

```
RMSF_i = √(<(r_i - <r_i>)²>)
```

Both metrics identify flexible regions:

```json
{
  "flexibility_profile": {
    "residues": [1, 2, 3, ...],
    "rmsf_values": [0.8, 0.7, 0.6, 1.2, 2.5, ...],
    "flexible_regions": [
      {"start": 45, "end": 52, "type": "loop", "mean_rmsf": 2.3},
      {"start": 120, "end": 128, "type": "terminus", "mean_rmsf": 3.1}
    ]
  }
}
```

### Order Parameters

For NMR, the Lipari-Szabo order parameter S² measures motion:

```
S² = 0: Completely flexible (isotropic motion)
S² = 1: Completely rigid (no motion)
```

Typical values:
- Structured regions: S² > 0.85
- Loops: S² = 0.6-0.8
- Disordered: S² < 0.5

```python
def calculate_order_parameters(trajectory, atom_pairs):
    """Calculate S² from MD trajectory."""
    order_params = []
    for pair in atom_pairs:
        vectors = get_bond_vectors(trajectory, pair)
        S2 = calculate_S2(vectors)  # Lipari-Szabo
        order_params.append(S2)
    return order_params
```

---

## 2.4 Principal Component Analysis

### Dimensionality Reduction

Proteins have 3N degrees of freedom (N = number of atoms), but most motion occurs along a few collective coordinates. PCA identifies these essential dynamics.

### The Math

1. Build covariance matrix from trajectory:
```
C_ij = <(r_i - <r_i>)(r_j - <r_j>)>
```

2. Diagonalize:
```
C × v_k = λ_k × v_k
```

3. Eigenvalues (λ) give variance explained
4. Eigenvectors (v) define collective motions

### Implementation

```python
import numpy as np
from sklearn.decomposition import PCA

def perform_pca(trajectory, n_components=10):
    """Perform PCA on protein trajectory."""
    # Flatten coordinates: (n_frames, 3*n_atoms)
    coords = trajectory.xyz.reshape(len(trajectory), -1)

    # Center the data
    coords_centered = coords - coords.mean(axis=0)

    # PCA
    pca = PCA(n_components=n_components)
    projections = pca.fit_transform(coords_centered)

    return {
        'projections': projections,
        'components': pca.components_,
        'variance_explained': pca.explained_variance_ratio_,
        'cumulative_variance': np.cumsum(pca.explained_variance_ratio_)
    }
```

### Interpretation

```
Typical variance explained:
  PC1: 30-50%  - Largest conformational change
  PC2: 15-25%  - Second mode
  PC3: 8-15%   - Third mode
  ...
  PC1-3: 60-80% - Usually sufficient for major motions
```

---

## 2.5 Normal Mode Analysis

### The Harmonic Approximation

Near an energy minimum, the potential is approximately harmonic:

```
V ≈ V₀ + (1/2) × Σ H_ij × Δx_i × Δx_j
```

Where H is the Hessian matrix of second derivatives.

### Solving for Modes

1. Mass-weight the Hessian: H' = M⁻¹/² × H × M⁻¹/²
2. Diagonalize: H' × e_k = ω_k² × e_k
3. Frequencies ω_k and mode shapes e_k

### Low-Frequency Modes

The lowest-frequency non-trivial modes (modes 7-12 after removing translation/rotation) often capture functional motions:

| Mode | Frequency (cm⁻¹) | Motion Type |
|------|------------------|-------------|
| 7 | 5-20 | Global bending/twisting |
| 8 | 10-30 | Domain motion |
| 9-12 | 20-50 | Subdomain motions |

### Elastic Network Models

For large proteins, coarse-grained elastic networks are efficient:

**Gaussian Network Model (GNM)**
- Isotropic fluctuations only
- Fast computation
- B-factor prediction

**Anisotropic Network Model (ANM)**
- Directional information
- Mode shapes
- Collectivity analysis

```python
from prody import ANM, calcANM

def calculate_anm_modes(structure, n_modes=20, cutoff=15.0):
    """Calculate ANM normal modes."""
    # Build ANM
    anm = ANM('protein')
    anm.buildHessian(structure.select('calpha'), cutoff=cutoff)

    # Calculate modes
    anm.calcModes(n_modes=n_modes)

    return {
        'eigenvalues': anm.getEigvals(),
        'eigenvectors': anm.getEigvecs(),
        'mode_collectivity': [calcCollectivity(mode)
                              for mode in anm]
    }
```

---

## 2.6 Cross-Correlation Analysis

### Correlated Motions

Functionally important residue pairs often move in a correlated manner. Dynamic cross-correlation measures this:

```
C_ij = <Δr_i · Δr_j> / (√<Δr_i²> × √<Δr_j²>)
```

Where:
- C_ij = +1: Perfect positive correlation (same direction)
- C_ij = -1: Perfect anti-correlation (opposite direction)
- C_ij = 0: Uncorrelated

### Correlation Patterns

Typical patterns reveal structural relationships:

```
Correlation Matrix

                Residue Number →
                ┌────────────────────────┐
              1 │++   +-     ++         │ Helix 1
  Residue       │ ++   +-     ++        │
  Number      50│      +++    --    ++  │ Beta sheet
              ↓ │ +-    +++   --    ++  │
             100│           +++    +-   │ Domain 2
                │  ++   --    +++  +-   │
                └────────────────────────┘

  ++ = Strongly correlated
  -- = Anti-correlated
  (blank) = Uncorrelated
```

### Community Detection

From correlation matrices, we can identify dynamically coupled communities:

```python
import networkx as nx
from community import best_partition

def detect_dynamic_communities(correlation_matrix, threshold=0.5):
    """Identify communities of correlated residues."""
    # Build graph from strong correlations
    G = nx.Graph()
    n = len(correlation_matrix)

    for i in range(n):
        for j in range(i+1, n):
            if abs(correlation_matrix[i,j]) > threshold:
                G.add_edge(i, j, weight=abs(correlation_matrix[i,j]))

    # Detect communities
    partition = best_partition(G)

    return partition
```

---

## 2.7 Allosteric Communication

### Long-Range Coupling

Allostery—the regulation of activity at one site by binding at another—depends on dynamics. Communication pathways transmit conformational signals across the protein.

### Quantifying Allostery

**Mutual Information**
Measures information shared between residue motions:

```
MI(i,j) = Σ P(x_i, x_j) × log[P(x_i, x_j) / (P(x_i) × P(x_j))]
```

**Transfer Entropy**
Measures directional information flow:

```
TE(i→j) = Information transferred from i to j
```

### Pathway Analysis

Allosteric pathways connect active and regulatory sites:

```json
{
  "allosteric_network": {
    "active_site": [145, 147, 166, 168, 200],
    "allosteric_site": [55, 57, 58, 60],
    "communication_pathways": [
      {
        "start": 55,
        "end": 145,
        "pathway": [55, 80, 95, 110, 130, 145],
        "strength": 0.85,
        "mechanism": "backbone_coupled"
      }
    ]
  }
}
```

---

## 2.8 Experimental Validation

### NMR Methods

**Relaxation**
- T1: Spin-lattice relaxation (ps-ns motions)
- T2: Spin-spin relaxation (μs-ms motions)
- NOE: Nuclear Overhauser Effect (spatial proximity + dynamics)

**Chemical Exchange**
- CPMG relaxation dispersion: μs-ms exchange
- R1ρ experiments: 10 μs - 10 ms motions
- ZZ-exchange: Slow (ms-s) exchange

### Cryo-EM Ensemble Analysis

Modern cryo-EM can resolve conformational heterogeneity:

1. **3D Classification**: Separate discrete states
2. **3D Variability Analysis**: Continuous conformational landscapes
3. **Multi-body Refinement**: Decompose into rigid bodies with relative motion

### Integration with WIA Standard

```json
{
  "experimental_validation": {
    "nmr_data": {
      "S2_values": [0.92, 0.88, 0.45, ...],
      "exchange_rates": {"residue_75": 1500}
    },
    "cryoem_ensemble": {
      "num_classes": 4,
      "populations": [0.55, 0.25, 0.15, 0.05]
    },
    "agreement_metrics": {
      "rmsf_correlation": 0.87,
      "S2_rmsd": 0.08
    }
  }
}
```

---

## 2.9 Intrinsically Disordered Proteins

### The Other Half of the Proteome

30-50% of eukaryotic proteins contain significant disordered regions. These IDPs/IDRs:

- Lack stable tertiary structure
- Sample extended conformational ensembles
- Often fold upon binding (coupled folding-binding)
- Are enriched in signaling and regulatory proteins

### Characterizing Disorder

**Ensemble Description**
IDPs require ensemble representations:

```python
def generate_idr_ensemble(sequence, n_conformers=1000):
    """Generate conformational ensemble for IDR."""
    ensemble = []
    for _ in range(n_conformers):
        # Sample from Ramachandran distribution
        phi_psi = sample_ramachandran(sequence,
                                       coil_only=True)
        structure = build_structure(sequence, phi_psi)
        ensemble.append(structure)

    return ensemble
```

**Key Metrics**

| Metric | Ordered Protein | IDP |
|--------|-----------------|-----|
| Rg | Compact | Extended |
| RMSF | 0.5-2 Å | 5-20 Å |
| S² | > 0.8 | < 0.4 |
| Secondary structure | Stable | Transient |

### Phase Separation

IDPs drive liquid-liquid phase separation (LLPS):

```
Dilute Phase ↔ Condensed Phase (membraneless organelles)
```

Dynamics within condensates:
- Rapid exchange with surroundings
- Liquid-like internal mobility
- Can mature to gel/solid states (pathology)

---

## 2.10 Summary: The WIA Dynamics Framework

The WIA-PROTEIN-DYNAMICS standard integrates all these concepts:

```json
{
  "dynamics_metrics": {
    "timescales": {
      "ps_motions": {"amplitude": 0.5, "unit": "angstrom"},
      "ns_motions": {"amplitude": 2.0, "unit": "angstrom"},
      "us_ms_motions": {"rate": 1000, "unit": "per_second"}
    },
    "flexibility": {
      "b_factors": [...],
      "rmsf": [...],
      "order_parameters_S2": [...]
    },
    "pca": {
      "num_components": 10,
      "variance_explained": [0.35, 0.20, ...]
    },
    "correlations": {
      "matrix": [...],
      "communities": [...]
    },
    "disorder": {
      "regions": [...],
      "total_percent": 25
    }
  }
}
```

---

## Key Takeaways

1. Proteins are dynamic, sampling conformational ensembles
2. Motion spans femtoseconds to seconds
3. MD simulation numerically solves equations of motion
4. PCA and NMA identify collective coordinates
5. Cross-correlations reveal coupled motions
6. Allosteric pathways transmit conformational signals
7. IDPs require ensemble descriptions
8. The WIA standard unifies these metrics

---

**Next Chapter:** [Conformational Ensembles](./04-conformational-ensembles.md)

弘益人間 - Benefit All Humanity
