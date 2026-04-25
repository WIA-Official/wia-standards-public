# Chapter 6: Phase 3 - Protocols

## Simulation and Analysis Workflows

**弘益人間 (Benefit All Humanity)**

---

## 6.1 Protocol Overview

### Standardized Workflows

The WIA-PROTEIN-DYNAMICS standard defines protocols for:
1. **Ensemble Generation**: Creating conformational ensembles
2. **Dynamics Analysis**: Extracting dynamics metrics
3. **Allosteric Mapping**: Identifying communication pathways
4. **Binding Simulation**: Drug binding dynamics
5. **Validation**: Comparing with experiments

### Protocol Requirements

Each protocol specifies:
- Input requirements and formats
- Processing steps and parameters
- Quality control checkpoints
- Output specifications
- Validation criteria

---

## 6.2 Ensemble Generation Protocol

### Protocol 1: Standard MD Ensemble

**Purpose:** Generate conformational ensemble from conventional MD simulation.

**Prerequisites:**
- Starting structure (PDB format)
- Force field parameters
- Solvation and ion parameters

**Protocol Steps:**

```yaml
ensemble_generation_md:
  version: "1.0"
  protocol_id: "WIA-PD-ENS-001"

  step_1_structure_preparation:
    actions:
      - add_hydrogens:
          ph: 7.0
          histidine_protonation: "auto"
      - check_structure:
          missing_atoms: "add"
          clashes: "minimize"
      - assign_protonation_states:
          method: "propka"
          ph: 7.0

  step_2_system_setup:
    solvation:
      water_model: "tip3p"
      box_shape: "dodecahedron"
      min_distance_to_wall_nm: 1.2
    ions:
      neutralize: true
      concentration_M: 0.15
      positive_ion: "Na"
      negative_ion: "Cl"

  step_3_energy_minimization:
    algorithm: "steepest_descent"
    max_steps: 50000
    tolerance_kJ_mol_nm: 1000
    position_restraints:
      heavy_atoms: 1000  # kJ/mol/nm^2

  step_4_equilibration:
    nvt:
      duration_ps: 100
      temperature_K: 300
      thermostat: "v-rescale"
      tau_t_ps: 0.1
      position_restraints:
        heavy_atoms: 1000
    npt:
      duration_ps: 1000
      temperature_K: 300
      pressure_bar: 1.0
      barostat: "parrinello-rahman"
      tau_p_ps: 2.0
      compressibility: 4.5e-5
      position_restraints:
        backbone: 100

  step_5_production:
    duration_ns: 1000
    temperature_K: 300
    pressure_bar: 1.0
    timestep_fs: 2
    constraints: "h-bonds"
    save_interval_ps: 10
    energy_save_interval_ps: 1
    checkpoint_interval_ns: 10

  step_6_analysis:
    trajectory_processing:
      remove_pbc: true
      center: "protein"
      fit: "backbone"
    clustering:
      method: "gromos"
      rmsd_cutoff_nm: 0.15
      atoms: "backbone"
    convergence:
      block_averaging: true
      block_sizes: [10, 50, 100, 500]

  quality_control:
    - check: "temperature_stability"
      tolerance_K: 2
    - check: "pressure_stability"
      tolerance_bar: 50
    - check: "energy_drift"
      max_drift_percent: 0.1
    - check: "rmsd_stability"
      after_ns: 100
```

### Protocol 2: Enhanced Sampling Ensemble

**Purpose:** Generate ensemble using metadynamics for better conformational sampling.

```yaml
ensemble_generation_metadynamics:
  version: "1.0"
  protocol_id: "WIA-PD-ENS-002"

  prerequisites:
    equilibrated_structure: true
    collective_variables_defined: true

  collective_variables:
    - cv_1:
        type: "rmsd"
        atoms: "backbone"
        reference: "starting_structure"
    - cv_2:
        type: "distance"
        group_1: "residues 50-60 and name CA"
        group_2: "residues 150-160 and name CA"

  metadynamics_parameters:
    bias_factor: 15
    gaussian_height_kJ: 1.0
    gaussian_sigma:
      cv_1: 0.1  # nm
      cv_2: 0.2  # nm
    deposit_interval_ps: 1.0
    grid:
      cv_1: [0.0, 1.0, 100]  # min, max, bins
      cv_2: [1.0, 4.0, 150]

  simulation:
    duration_ns: 500
    walkers: 4  # Multiple walker metadynamics
    walker_exchange_interval_ps: 100

  reweighting:
    method: "tiwary-parrinello"
    block_analysis: true
    error_estimation: "bootstrap"

  output:
    free_energy_surface: true
    representative_structures: true
    transition_pathways: true
```

### Protocol 3: AlphaFlow Ensemble

**Purpose:** Generate ensemble using ML-based approach.

```yaml
ensemble_generation_alphaflow:
  version: "1.0"
  protocol_id: "WIA-PD-ENS-003"

  input:
    sequence: "FASTA format"
    msa: "optional"
    template: "optional"

  model:
    name: "alphaflow"
    version: "1.0"
    checkpoint: "production"

  sampling:
    num_samples: 100
    temperature: 1.0
    diversity_factor: 0.8

  post_processing:
    energy_minimization: true
    clash_removal: true
    hydrogen_optimization: true

  clustering:
    method: "k-medoids"
    metric: "tm-score"
    optimal_k: "silhouette"

  validation:
    check_stereochemistry: true
    ramachandran_outliers_max_percent: 2.0
    clash_score_max: 10
```

---

## 6.3 Dynamics Analysis Protocol

### Flexibility Analysis

```yaml
flexibility_analysis:
  version: "1.0"
  protocol_id: "WIA-PD-FLEX-001"

  input:
    trajectory: "xtc/dcd format"
    topology: "pdb/gro format"
    reference: "first frame"

  rmsf_calculation:
    atoms: "name CA"
    fit_atoms: "backbone and not name H*"
    time_range_ns: [100, "end"]  # Skip equilibration

  bfactor_derivation:
    convert_rmsf: true
    formula: "B = (8 * pi^2 / 3) * RMSF^2"

  order_parameters:
    calculate_S2: true
    bond_vectors: "NH"
    correlation_function:
      max_lag_ps: 500
      fit_model: "lipari-szabo"

  flexible_region_identification:
    threshold_percentile: 90
    min_length: 3
    merge_gap: 2

  output:
    per_residue_metrics: true
    flexible_regions: true
    visualization: "pymol_script"
```

### Principal Component Analysis

```yaml
pca_analysis:
  version: "1.0"
  protocol_id: "WIA-PD-PCA-001"

  input:
    trajectory: "aligned trajectory"
    atoms: "name CA"

  pca_parameters:
    n_components: 20
    method: "svd"
    center: true

  analysis:
    scree_plot: true
    variance_threshold: 0.90
    project_trajectory: true
    extreme_projections: true

  mode_visualization:
    modes: [1, 2, 3]
    amplitude_angstrom: 3.0
    n_frames: 50
    output_format: "pdb_trajectory"

  free_energy_projection:
    dimensions: [1, 2]
    bins: 50
    temperature_K: 300
```

---

## 6.4 Allosteric Mapping Protocol

### Cross-Correlation Analysis

```yaml
allosteric_mapping:
  version: "1.0"
  protocol_id: "WIA-PD-ALLO-001"

  step_1_correlation_matrix:
    input:
      trajectory: "aligned trajectory"
      atoms: "name CA"
    method: "pearson"
    normalize: true
    output: "correlation_matrix.npy"

  step_2_community_detection:
    input: "correlation_matrix.npy"
    method: "girvan-newman"
    resolution: 1.0
    min_community_size: 5
    output: "communities.json"

  step_3_pathway_identification:
    source_sites:
      - name: "allosteric_site"
        residues: [55, 57, 58, 60]
    target_sites:
      - name: "active_site"
        residues: [145, 147, 166, 168]

    pathway_algorithm: "dijkstra"
    edge_weight: "1 - abs(correlation)"
    n_pathways: 5
    output: "pathways.json"

  step_4_mutual_information:
    calculate: true
    bins: 20
    normalize: true
    compare_with_correlation: true

  visualization:
    network_graph: true
    pathway_on_structure: true
    pymol_session: true
```

---

## 6.5 Drug Binding Protocol

### Binding Pathway Simulation

```yaml
binding_simulation:
  version: "1.0"
  protocol_id: "WIA-PD-BIND-001"

  system_preparation:
    protein:
      structure: "equilibrated PDB"
      binding_site_residues: "auto-detect or specify"
    ligand:
      input_format: "smiles/mol2/sdf"
      parameterization: "gaff2"
      partial_charges: "am1-bcc"
    complex:
      initial_ligand_position: "20 angstrom from binding site"
      solvate: true
      neutralize: true

  steered_md:
    enabled: true
    pull_direction: "towards binding site center"
    pull_rate_nm_ps: 0.001
    spring_constant_kJ_mol_nm2: 1000
    duration_ns: 20

  unbiased_md:
    after_binding: true
    duration_ns: 100
    check_stability: true
    stability_threshold_ns: 50

  analysis:
    binding_pathway:
      identify_intermediates: true
      clustering_rmsd_nm: 0.2
    interaction_analysis:
      hydrogen_bonds: true
      hydrophobic_contacts: true
      salt_bridges: true
    rmsd_tracking:
      reference: "bound crystal structure"
      atoms: "ligand heavy atoms"

  free_energy:
    method: "umbrella_sampling"
    collective_variable: "distance to binding site"
    windows: 20
    window_spacing_nm: 0.1
    per_window_ns: 10
    wham_analysis: true
```

### Binding Free Energy Calculation

```yaml
binding_free_energy:
  version: "1.0"
  protocol_id: "WIA-PD-BIND-002"

  method: "fep"  # or "ti", "mmpbsa"

  fep_protocol:
    n_lambda: 20
    lambda_schedule:
      vdw: "soft-core"
      electrostatics: "linear"
    equilibration_per_lambda_ns: 1
    production_per_lambda_ns: 5

    restraints:
      boresch:
        k_distance: 10.0
        k_angle: 10.0
        k_dihedral: 10.0
      analytical_correction: true

  analysis:
    overlap_matrix: true
    convergence_check: true
    forward_backward: true
    error_estimation: "bootstrap"

  output:
    delta_g_kcal_mol: true
    per_lambda_results: true
    uncertainty: true
```

---

## 6.6 Validation Protocol

### Experimental Validation

```yaml
validation_protocol:
  version: "1.0"
  protocol_id: "WIA-PD-VAL-001"

  nmr_validation:
    order_parameters:
      experimental_source: "BMRB entry"
      compare_S2: true
      correlation_threshold: 0.8
      rmsd_threshold: 0.1
    chemical_shifts:
      predictor: "sparta+"
      compare_CA: true
      compare_CB: true
      compare_N: true
      compare_HN: true

  xray_validation:
    b_factors:
      pdb_id: "reference structure"
      normalize: true
      correlation_threshold: 0.7
    electron_density:
      compare_occupancies: true

  cryoem_validation:
    heterogeneity:
      compare_class_populations: true
      structural_rmsd: true
    local_resolution:
      correlate_with_flexibility: true

  cross_validation:
    holdout_fraction: 0.2
    n_repeats: 5
    metrics:
      - "population_divergence"
      - "structural_similarity"
      - "dynamics_correlation"

  reporting:
    validation_score: true
    detailed_comparison: true
    figures: true
```

---

## 6.7 Protocol Execution

### Command-Line Interface

```bash
# Run ensemble generation
wia-pd protocol run WIA-PD-ENS-001 \
  --input structure.pdb \
  --config my_params.yaml \
  --output ensemble_results/

# Run dynamics analysis
wia-pd protocol run WIA-PD-FLEX-001 \
  --trajectory md.xtc \
  --topology system.gro \
  --output flexibility_analysis/

# Run validation
wia-pd protocol run WIA-PD-VAL-001 \
  --prediction prediction.json \
  --experimental nmr_data.json \
  --output validation_report/
```

### Python API

```python
from wia_pd import protocols

# Load and run protocol
protocol = protocols.load("WIA-PD-ENS-001")

result = protocol.run(
    input_structure="structure.pdb",
    parameters={
        "production_duration_ns": 1000,
        "temperature_K": 310
    },
    output_dir="ensemble_results"
)

# Check status
print(f"Status: {result.status}")
print(f"Ensemble size: {result.ensemble.num_states}")

# Access results
ensemble = result.get_ensemble()
for state in ensemble.states:
    print(f"  {state.name}: {state.population:.2%}")
```

---

## 6.8 Quality Control Checkpoints

### Simulation Quality Metrics

```python
def check_simulation_quality(trajectory, topology):
    """
    Comprehensive quality control for MD simulations.
    """
    checks = {
        'passed': [],
        'warnings': [],
        'failed': []
    }

    # 1. Temperature stability
    temp_mean, temp_std = analyze_temperature(trajectory)
    if temp_std > 5:
        checks['warnings'].append(f"Temperature fluctuation: {temp_std:.1f} K")
    else:
        checks['passed'].append("Temperature stable")

    # 2. Pressure stability
    press_mean, press_std = analyze_pressure(trajectory)
    if press_std > 100:
        checks['warnings'].append(f"Pressure fluctuation: {press_std:.0f} bar")
    else:
        checks['passed'].append("Pressure stable")

    # 3. Energy conservation
    energy_drift = calculate_energy_drift(trajectory)
    if abs(energy_drift) > 0.01:  # kJ/mol/ns
        checks['failed'].append(f"Energy drift: {energy_drift:.3f} kJ/mol/ns")
    else:
        checks['passed'].append("Energy conserved")

    # 4. RMSD stability
    rmsd = calculate_rmsd(trajectory, topology)
    if rmsd[-1000:].std() > 0.2:  # nm
        checks['warnings'].append("RMSD not converged")
    else:
        checks['passed'].append("RMSD stable")

    # 5. Box size consistency
    box_sizes = get_box_sizes(trajectory)
    if box_sizes.std() / box_sizes.mean() > 0.05:
        checks['warnings'].append("Box size fluctuation > 5%")

    return checks
```

---

## Summary

The WIA-PROTEIN-DYNAMICS protocols provide:
- Standardized workflows for all dynamics operations
- Detailed parameter specifications
- Quality control checkpoints
- Multiple execution interfaces (CLI, Python, API)
- Validation against experimental data

---

**Next Chapter:** [Phase 4: Integration](./08-phase4-integration.md)

弘益人間 - Benefit All Humanity
