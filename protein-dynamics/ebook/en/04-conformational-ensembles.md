# Chapter 3: Conformational Ensembles

## Representing Protein States

**弘益人間 (Benefit All Humanity)**

---

## 3.1 The Ensemble Paradigm

### Beyond Single Structures

A single structure represents just one snapshot from a continuum of conformations. The ensemble view recognizes that proteins exist as collections of states, each with a defined population determined by thermodynamics:

```
Ensemble = {(Structure₁, Population₁), (Structure₂, Population₂), ...}

Population_i = exp(-ΔG_i/RT) / Σ exp(-ΔG_j/RT)
```

### Why Ensembles Matter

**Functional Implications**

| Single Structure View | Ensemble View |
|----------------------|---------------|
| Enzyme has one active site geometry | Samples multiple catalytically competent states |
| Drug binds to one conformation | Drug selects or induces specific conformations |
| Mutation changes one structure | Mutation shifts population distribution |
| Allostery is mysterious | Allostery shifts conformational equilibrium |

### Experimental Evidence for Ensembles

Multiple techniques reveal protein heterogeneity:

**NMR Spectroscopy**
- Multiple peaks for same residue = multiple states
- Peak broadening = conformational exchange
- PRE/DEER = distance distributions, not single values

**Cryo-EM**
- 3D classification yields multiple structures
- Continuous heterogeneity analysis reveals gradients
- Resolution varies across maps (flexibility)

**Single-Molecule FRET**
- Distance fluctuations in real-time
- Population histograms show state distributions
- Hidden Markov models extract kinetics

---

## 3.2 Generating Ensembles from MD

### Standard MD Approach

Molecular dynamics naturally samples conformational space:

```python
import mdtraj as md
import numpy as np

def generate_md_ensemble(topology, trajectory_file,
                         n_clusters=10, stride=10):
    """
    Generate conformational ensemble from MD trajectory.

    Parameters:
    -----------
    topology : str
        Path to topology file (PDB, GRO)
    trajectory_file : str
        Path to trajectory file (XTC, DCD)
    n_clusters : int
        Number of representative structures
    stride : int
        Read every Nth frame

    Returns:
    --------
    dict : Ensemble with structures and populations
    """
    # Load trajectory
    traj = md.load(trajectory_file, top=topology, stride=stride)

    # Align to reference (first frame)
    traj.superpose(traj[0])

    # Compute pairwise RMSD
    from scipy.cluster.hierarchy import linkage, fcluster
    rmsd_matrix = np.zeros((len(traj), len(traj)))
    for i in range(len(traj)):
        rmsd_matrix[i] = md.rmsd(traj, traj, frame=i)

    # Hierarchical clustering
    condensed = squareform(rmsd_matrix)
    linkage_matrix = linkage(condensed, method='ward')
    clusters = fcluster(linkage_matrix, n_clusters, criterion='maxclust')

    # Get medoids (most representative structure per cluster)
    ensemble = []
    for c in range(1, n_clusters + 1):
        mask = clusters == c
        cluster_frames = np.where(mask)[0]
        population = mask.sum() / len(clusters)

        # Find medoid
        sub_rmsd = rmsd_matrix[np.ix_(cluster_frames, cluster_frames)]
        medoid_idx = cluster_frames[np.argmin(sub_rmsd.sum(axis=1))]

        ensemble.append({
            'state_id': f'state_{c}',
            'frame_index': int(medoid_idx),
            'population': float(population),
            'cluster_size': int(mask.sum()),
            'structure': traj[medoid_idx]
        })

    # Sort by population
    ensemble.sort(key=lambda x: -x['population'])

    return ensemble
```

### Convergence Assessment

MD simulations must be long enough to adequately sample the ensemble:

```python
def assess_convergence(trajectory, property_function,
                       block_sizes=[10, 50, 100, 500]):
    """
    Assess convergence using block averaging.

    Parameters:
    -----------
    trajectory : mdtraj.Trajectory
    property_function : callable
        Function that computes property from trajectory
    block_sizes : list
        Block sizes for averaging

    Returns:
    --------
    dict : Convergence statistics
    """
    values = property_function(trajectory)
    n = len(values)

    results = []
    for block_size in block_sizes:
        n_blocks = n // block_size
        if n_blocks < 2:
            continue

        block_means = []
        for i in range(n_blocks):
            start = i * block_size
            end = start + block_size
            block_means.append(np.mean(values[start:end]))

        block_means = np.array(block_means)
        std_error = np.std(block_means) / np.sqrt(n_blocks)

        results.append({
            'block_size': block_size,
            'n_blocks': n_blocks,
            'mean': np.mean(block_means),
            'std_error': std_error
        })

    return {
        'converged': results[-1]['std_error'] < 0.1 * abs(results[-1]['mean']),
        'block_analysis': results
    }
```

---

## 3.3 Enhanced Sampling Methods

### The Sampling Problem

Standard MD is limited by kinetic barriers. Transitions between states may take microseconds to milliseconds—longer than typical simulations.

### Replica Exchange MD (REMD)

Run multiple replicas at different temperatures, periodically attempting swaps:

```
T₁=300K ←→ T₂=310K ←→ T₃=320K ←→ ... ←→ T₃₂=450K

Swap probability: P = min(1, exp(Δβ × ΔE))
```

High-temperature replicas cross barriers; exchanges propagate sampling to physiological temperature.

```python
def setup_remd(n_replicas=32, t_min=300, t_max=450):
    """Setup replica exchange temperatures."""
    # Geometric spacing for optimal exchange rates
    temperatures = []
    for i in range(n_replicas):
        t = t_min * (t_max / t_min) ** (i / (n_replicas - 1))
        temperatures.append(t)

    return {
        'n_replicas': n_replicas,
        'temperatures': temperatures,
        'target_exchange_rate': 0.2,
        'exchange_interval_ps': 2.0
    }
```

### Metadynamics

Deposit Gaussian bias potentials on visited conformations, forcing exploration of new regions:

```
V_bias(s,t) = Σ w × exp(-(s - s(t'))² / 2σ²)

s = collective variable (CV)
w = Gaussian height
σ = Gaussian width
```

The bias gradually fills energy wells, allowing escape:

```
Free Energy
    │
    │    ╱╲
    │   ╱  ╲    ╱╲
    │  ╱    ╲  ╱  ╲  ←── Barrier (too high for MD)
    │ ╱      ╲╱    ╲
    └──────────────────

After Metadynamics:
    │             Bias fills
    │  ██  ╱╲     valleys
    │ ████╱  ╲████╱╲
    │████████╲████████  ←── Barrier now crossable
    └──────────────────
```

### Accelerated MD (aMD)

Reduce barrier heights by adding boost potential when energy is below threshold:

```
E* = E + ΔV(E)

ΔV = (E_thresh - E)² / (α + E_thresh - E)
```

### WIA Protocol for Enhanced Sampling

```json
{
  "enhanced_sampling_protocol": {
    "method": "metadynamics",
    "collective_variables": [
      {
        "type": "rmsd",
        "reference": "active_site.pdb",
        "atoms": "backbone"
      },
      {
        "type": "distance",
        "atom1": "residue_50_CA",
        "atom2": "residue_200_CA"
      }
    ],
    "parameters": {
      "gaussian_height_kJ": 1.0,
      "gaussian_width": [0.1, 0.2],
      "deposit_interval_ps": 1.0,
      "bias_factor": 15
    },
    "simulation_time_ns": 500,
    "reweighting": "Tiwary-Parrinello"
  }
}
```

---

## 3.4 Machine Learning Ensemble Generation

### AlphaFlow

AlphaFlow combines AlphaFold with flow matching to generate conformational ensembles:

```
Sequence → AlphaFold Features → Flow Model → Ensemble
```

Key advantages:
- No simulation required
- Fast generation (seconds vs. hours)
- Trained on PDB structural diversity
- Captures experimental heterogeneity

```python
# Conceptual AlphaFlow usage
from alphaflow import AlphaFlowModel

def generate_alphaflow_ensemble(sequence, n_samples=100):
    """Generate ensemble using AlphaFlow."""
    model = AlphaFlowModel.load_pretrained()

    ensemble = []
    for i in range(n_samples):
        structure = model.sample(sequence)
        confidence = model.get_confidence(structure)
        ensemble.append({
            'structure': structure,
            'plddt': confidence
        })

    return ensemble
```

### Boltzmann Generators

Neural networks trained to sample from the Boltzmann distribution:

```
z ~ N(0,1) → Neural Network → x with P(x) ∝ exp(-E(x)/kT)
```

Training uses:
- Energy-based loss
- Normalizing flows
- Reweighting for correct distribution

Advantages:
- Sample rare states efficiently
- 100-1000× faster than MD for rare events
- Learn from limited data

### Distributional Graphormer

Predicts the full distribution over structures, not just a point estimate:

```
Input: Sequence
Output: P(structure | sequence) - full distribution
```

Can sample diverse structures and estimate populations.

---

## 3.5 Free Energy Landscapes

### The 2D Projection

High-dimensional conformational space is often projected onto 2D for visualization:

```
Free Energy (kcal/mol)
  8 │                     ╔══╗
  7 │         ╔══╗        ║  ║
  6 │  ╔══╗   ║  ║   ╔══╗ ║  ║
  5 │  ║  ║   ║  ║   ║  ║ ╚══╝
  4 │  ║  ╚═══╝  ║   ║  ║
  3 │  ║        ║   ║  ║
  2 │  ║   ╔════╝   ║  ║
  1 │  ╚═══╝        ╚══╝
  0 │     ★ Global    ★ Metastable
    └──────────────────────────────→
                CV1 (e.g., RMSD)
```

### Computing the Landscape

From simulation or ensemble data:

```python
import numpy as np

def compute_free_energy_landscape(cv1_values, cv2_values,
                                  n_bins=50, temperature=300):
    """
    Compute 2D free energy landscape from CV values.

    Parameters:
    -----------
    cv1_values, cv2_values : array-like
        Collective variable trajectories
    n_bins : int
        Number of bins per dimension
    temperature : float
        Temperature in Kelvin

    Returns:
    --------
    dict : Free energy landscape data
    """
    kB = 0.001987  # kcal/mol/K
    kT = kB * temperature

    # 2D histogram
    H, xedges, yedges = np.histogram2d(
        cv1_values, cv2_values, bins=n_bins, density=True
    )

    # Convert to free energy
    # G = -kT * ln(P)
    with np.errstate(divide='ignore'):
        G = -kT * np.log(H)
        G[np.isinf(G)] = np.nan

    # Set minimum to zero
    G = G - np.nanmin(G)

    # Find minima
    from scipy.ndimage import minimum_filter
    local_min = (H == minimum_filter(H, size=3))
    minima_coords = np.where(local_min & (H > 0))

    minima = []
    for i, j in zip(*minima_coords):
        minima.append({
            'cv1': (xedges[i] + xedges[i+1]) / 2,
            'cv2': (yedges[j] + yedges[j+1]) / 2,
            'free_energy': float(G[i, j]),
            'population': float(H[i, j])
        })

    return {
        'free_energy_surface': G.tolist(),
        'cv1_edges': xedges.tolist(),
        'cv2_edges': yedges.tolist(),
        'minima': minima,
        'temperature_K': temperature
    }
```

### Identifying Transition Pathways

Between minima, transition states define the barrier:

```python
def find_minimum_energy_path(G, start, end, method='string'):
    """
    Find minimum free energy path between two minima.

    Uses the string method or nudged elastic band.
    """
    if method == 'string':
        # Initialize straight path
        path = np.linspace(start, end, n_images)

        # Iteratively optimize
        for iteration in range(max_iter):
            # Compute gradients
            gradients = compute_gradients(G, path)

            # Move perpendicular to path
            tangents = compute_tangents(path)
            perp_grad = gradients - np.sum(gradients * tangents, axis=1, keepdims=True) * tangents

            # Update positions
            path = path - step_size * perp_grad

            # Re-parameterize (equal spacing)
            path = reparameterize(path)

        return path

    elif method == 'neb':
        # Nudged elastic band implementation
        ...
```

---

## 3.6 Population Estimation

### From Simulation

Population from simulation time spent in each state:

```python
def estimate_populations(trajectory, state_assignments):
    """Estimate populations from trajectory."""
    unique_states = np.unique(state_assignments)
    populations = {}

    for state in unique_states:
        count = np.sum(state_assignments == state)
        populations[state] = count / len(state_assignments)

    return populations
```

### Reweighting Biased Simulations

Enhanced sampling introduces bias that must be removed:

**WHAM (Weighted Histogram Analysis Method)**
Combines data from multiple biased simulations.

**Tiwary-Parrinello Reweighting**
For metadynamics:

```
w(t) = exp(V_bias(s(t), t) / kT)
P_unbiased(s) = Σ δ(s - s(t)) × w(t) / Σ w(t)
```

```python
def reweight_metadynamics(cv_trajectory, bias_trajectory,
                          temperature=300, n_bins=50):
    """
    Reweight metadynamics to recover unbiased distribution.
    """
    kB = 0.001987  # kcal/mol/K
    kT = kB * temperature

    # Compute weights
    weights = np.exp(bias_trajectory / kT)

    # Weighted histogram
    hist, edges = np.histogram(cv_trajectory, bins=n_bins,
                                weights=weights, density=True)

    # Free energy
    G = -kT * np.log(hist)
    G = G - np.min(G[np.isfinite(G)])

    return G, edges
```

### Markov State Models

For long timescale dynamics, build kinetic models:

```
P(state_j at t+τ | state_i at t) = T_ij

T = Transition probability matrix
```

Populations come from the stationary distribution:

```
π × T = π  (left eigenvector with eigenvalue 1)
```

---

## 3.7 WIA Ensemble Format

### Schema Definition

```json
{
  "$schema": "https://wia.live/schemas/conformational-ensemble/v1.0.0",
  "conformational_ensemble": {
    "protein_id": "P00533",
    "generation_method": "metadynamics",
    "num_states": 5,

    "states": [
      {
        "state_id": "ground",
        "name": "Active conformation",
        "population": 0.65,
        "relative_energy": {"value": 0.0, "unit": "kcal/mol"},
        "coordinates_pdb": "state_ground.pdb",
        "rmsd_from_reference": 0.0,
        "characteristics": {
          "active_site_accessible": true,
          "key_contacts": ["R841-E762", "K745-D855"]
        }
      },
      {
        "state_id": "excited_1",
        "name": "DFG-out inactive",
        "population": 0.25,
        "relative_energy": {"value": 0.8, "unit": "kcal/mol"},
        "coordinates_pdb": "state_excited1.pdb",
        "rmsd_from_reference": 3.2,
        "characteristics": {
          "active_site_accessible": false,
          "dfg_motif": "out"
        }
      }
    ],

    "free_energy_landscape": {
      "collective_variables": ["RMSD_to_active", "Activation_loop_distance"],
      "cv_units": ["angstrom", "angstrom"],
      "surface": "fel_surface.npy",
      "minima": [
        {"cv": [0.0, 12.5], "energy": 0.0, "state": "ground"},
        {"cv": [3.2, 18.0], "energy": 0.8, "state": "excited_1"}
      ],
      "barriers": [
        {"from": "ground", "to": "excited_1", "height": 4.2, "ts_cv": [1.5, 15.0]}
      ]
    },

    "kinetics": {
      "transition_rates": {
        "ground_to_excited_1": {"value": 1000, "unit": "per_second"},
        "excited_1_to_ground": {"value": 4000, "unit": "per_second"}
      },
      "exchange_timescale_us": 250
    }
  }
}
```

---

## 3.8 Visualization and Analysis

### Ensemble Visualization

```python
import py3Dmol

def visualize_ensemble(ensemble, view_width=800, view_height=600):
    """Visualize conformational ensemble in 3D."""
    viewer = py3Dmol.view(width=view_width, height=view_height)

    # Color by population (high pop = opaque, low = transparent)
    max_pop = max(s['population'] for s in ensemble)

    for i, state in enumerate(ensemble):
        alpha = state['population'] / max_pop
        viewer.addModel(state['pdb_string'], 'pdb')
        viewer.setStyle({'model': i}, {
            'cartoon': {
                'color': 'spectrum',
                'opacity': alpha
            }
        })

    viewer.zoomTo()
    return viewer
```

### Comparison Metrics

```python
def compare_ensembles(ensemble1, ensemble2):
    """
    Compare two conformational ensembles.

    Returns metrics for similarity.
    """
    # Population overlap (Jensen-Shannon divergence)
    from scipy.spatial.distance import jensenshannon
    p1 = [s['population'] for s in ensemble1]
    p2 = [s['population'] for s in ensemble2]
    js_div = jensenshannon(p1, p2)

    # Structural similarity (best RMSD matching)
    from scipy.optimize import linear_sum_assignment
    rmsd_matrix = compute_pairwise_rmsd(ensemble1, ensemble2)
    row_ind, col_ind = linear_sum_assignment(rmsd_matrix)
    mean_rmsd = rmsd_matrix[row_ind, col_ind].mean()

    return {
        'population_divergence': js_div,
        'mean_matched_rmsd': mean_rmsd,
        'num_states_1': len(ensemble1),
        'num_states_2': len(ensemble2)
    }
```

---

## Summary

- Proteins exist as conformational ensembles, not single structures
- MD simulation samples ensembles but may miss rare states
- Enhanced sampling (REMD, metadynamics) overcomes barriers
- ML methods (AlphaFlow, Boltzmann Generators) enable fast ensemble generation
- Free energy landscapes visualize conformational space
- Population estimation requires proper reweighting
- The WIA format standardizes ensemble representation

---

**Next Chapter:** [Phase 1: Data Format](./05-phase1-data-format.md)

弘益人間 - Benefit All Humanity
