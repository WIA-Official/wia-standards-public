/**
 * WIA-PROTEIN-DYNAMICS TypeScript Types
 *
 * Type definitions for protein dynamics standardization
 *
 * @version 1.0.0
 * @license MIT
 * @philosophy 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Protein identifier types
 */
export type ProteinIdType = 'uniprot' | 'pdb' | 'alphafold' | 'ensembl' | 'refseq';

/**
 * Experimental methods for structure determination
 */
export type ExperimentalMethod =
  | 'X-RAY'
  | 'NMR'
  | 'CRYO-EM'
  | 'NEUTRON'
  | 'ELECTRON_CRYSTALLOGRAPHY';

/**
 * Simulation engines supported
 */
export type SimulationEngine =
  | 'gromacs'
  | 'amber'
  | 'openmm'
  | 'namd'
  | 'desmond';

/**
 * Force fields for molecular dynamics
 */
export type ForceField =
  | 'amber14sb'
  | 'charmm36m'
  | 'oplsaa'
  | 'gromos'
  | 'ff19sb';

/**
 * Water models
 */
export type WaterModel = 'tip3p' | 'tip4p' | 'tip4pew' | 'spc' | 'opc';

// ============================================================================
// Protein Structure Types
// ============================================================================

/**
 * 3D coordinates
 */
export interface Coordinate {
  x: number;
  y: number;
  z: number;
}

/**
 * Atom representation
 */
export interface Atom {
  id: number;
  name: string;
  element: string;
  residue_name: string;
  residue_number: number;
  chain_id: string;
  coordinates: Coordinate;
  occupancy?: number;
  b_factor?: number;
}

/**
 * Residue representation
 */
export interface Residue {
  number: number;
  name: string;
  chain_id: string;
  atoms: Atom[];
  secondary_structure?: 'helix' | 'sheet' | 'coil' | 'turn';
}

/**
 * Chain representation
 */
export interface Chain {
  id: string;
  residues: Residue[];
  sequence: string;
}

/**
 * Complete protein structure
 */
export interface ProteinStructure {
  id: string;
  title?: string;
  chains: Chain[];
  resolution?: number;
  experimental_method?: ExperimentalMethod;
}

// ============================================================================
// Conformational Ensemble Types
// ============================================================================

/**
 * Single conformational state
 */
export interface ConformationalState {
  id: string;
  name: string;
  population: number;
  representative_structure: ProteinStructure;
  relative_energy_kJ_mol?: number;
  rmsd_to_reference_angstrom?: number;
  description?: string;
}

/**
 * Transition between conformational states
 */
export interface StateTransition {
  from_state: string;
  to_state: string;
  rate_per_second: number;
  barrier_kJ_mol?: number;
  pathway_description?: string;
}

/**
 * Conformational ensemble
 */
export interface ConformationalEnsemble {
  protein_id: string;
  method: 'md' | 'metadynamics' | 'remd' | 'alphaflow' | 'boltzmann_generator' | 'nmr' | 'cryoem';
  temperature_K: number;
  states: ConformationalState[];
  transitions?: StateTransition[];
  total_simulation_time_ns?: number;
  convergence_metrics?: ConvergenceMetrics;
}

/**
 * Convergence metrics for ensemble generation
 */
export interface ConvergenceMetrics {
  block_averaging_error?: number;
  autocorrelation_time_ns?: number;
  effective_samples?: number;
  population_std?: number[];
}

// ============================================================================
// Dynamics Metrics Types
// ============================================================================

/**
 * Per-residue flexibility metrics
 */
export interface FlexibilityMetrics {
  residue_numbers: number[];
  rmsf_angstrom: number[];
  b_factors: number[];
  order_parameters_S2?: number[];
}

/**
 * Principal Component Analysis results
 */
export interface PCAResults {
  n_components: number;
  eigenvalues: number[];
  variance_explained: number[];
  cumulative_variance: number[];
  eigenvectors?: number[][][];
}

/**
 * Free Energy Landscape
 */
export interface FreeEnergyLandscape {
  collective_variables: CollectiveVariable[];
  grid_dimensions: number[];
  grid_spacing: number[];
  free_energy_kJ_mol: number[];
  temperature_K: number;
  reweighting_method?: string;
}

/**
 * Collective variable definition
 */
export interface CollectiveVariable {
  type: 'rmsd' | 'distance' | 'angle' | 'dihedral' | 'coordination' | 'radius_of_gyration' | 'pca_projection';
  name: string;
  definition: string;
  min_value: number;
  max_value: number;
  unit: string;
}

// ============================================================================
// Drug Binding Types
// ============================================================================

/**
 * Drug/ligand information
 */
export interface Ligand {
  id: string;
  name: string;
  smiles?: string;
  molecular_weight_Da?: number;
  chembl_id?: string;
  pubchem_cid?: string;
}

/**
 * Binding site definition
 */
export interface BindingSite {
  id: string;
  name: string;
  residues: number[];
  center: Coordinate;
  volume_angstrom3?: number;
  druggability_score?: number;
  is_cryptic?: boolean;
}

/**
 * Binding thermodynamics
 */
export interface BindingThermodynamics {
  dG_kcal_mol: number;
  dG_error?: number;
  dH_kcal_mol?: number;
  TdS_kcal_mol?: number;
  Kd_nM?: number;
  IC50_nM?: number;
  method: 'fep' | 'ti' | 'mm-pbsa' | 'mm-gbsa' | 'experimental';
}

/**
 * Binding kinetics
 */
export interface BindingKinetics {
  kon_per_M_per_s: number;
  koff_per_s: number;
  residence_time_s: number;
  diffusion_limited?: boolean;
  binding_mechanism?: 'conformational_selection' | 'induced_fit' | 'mixed';
}

/**
 * Complete drug binding profile
 */
export interface DrugBindingProfile {
  protein_id: string;
  ligand: Ligand;
  binding_site: BindingSite;
  thermodynamics?: BindingThermodynamics;
  kinetics?: BindingKinetics;
  pose?: ProteinStructure;
  interaction_fingerprint?: string;
}

// ============================================================================
// Allosteric Network Types
// ============================================================================

/**
 * Communication pathway in protein
 */
export interface CommunicationPathway {
  id: string;
  source_residues: number[];
  target_residues: number[];
  pathway_residues: number[];
  pathway_strength: number;
  mechanism?: 'backbone' | 'sidechain' | 'water_mediated';
}

/**
 * Dynamic community (group of correlated residues)
 */
export interface DynamicCommunity {
  id: string;
  residues: number[];
  intra_community_correlation: number;
  functional_annotation?: string;
}

/**
 * Allosteric network analysis
 */
export interface AllostericNetwork {
  protein_id: string;
  correlation_matrix?: number[][];
  communities: DynamicCommunity[];
  pathways: CommunicationPathway[];
  allosteric_sites?: BindingSite[];
}

// ============================================================================
// Simulation Protocol Types
// ============================================================================

/**
 * Simulation parameters
 */
export interface SimulationParameters {
  engine: SimulationEngine;
  force_field: ForceField;
  water_model: WaterModel;
  temperature_K: number;
  pressure_bar?: number;
  timestep_fs: number;
  duration_ns: number;
  save_interval_ps: number;
}

/**
 * System preparation settings
 */
export interface SystemPreparation {
  add_hydrogens: boolean;
  ph: number;
  solvation: {
    box_type: 'cubic' | 'dodecahedron' | 'octahedron';
    min_distance_nm: number;
  };
  ions: {
    neutralize: boolean;
    concentration_M: number;
    positive_ion: string;
    negative_ion: string;
  };
}

/**
 * Complete simulation protocol
 */
export interface SimulationProtocol {
  id: string;
  name: string;
  version: string;
  preparation: SystemPreparation;
  minimization: {
    algorithm: 'steepest_descent' | 'conjugate_gradient' | 'lbfgs';
    max_steps: number;
    force_tolerance: number;
  };
  equilibration: {
    nvt_duration_ps: number;
    npt_duration_ps: number;
  };
  production: SimulationParameters;
  analysis: string[];
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Complete protein dynamics profile
 */
export interface ProteinDynamicsProfile {
  metadata: {
    wia_version: string;
    created_at: string;
    updated_at: string;
    protein_id: string;
    protein_name: string;
    organism?: string;
  };
  static_structure: ProteinStructure;
  conformational_ensemble?: ConformationalEnsemble;
  dynamics_metrics?: {
    flexibility: FlexibilityMetrics;
    pca?: PCAResults;
    free_energy_landscape?: FreeEnergyLandscape;
  };
  drug_binding?: DrugBindingProfile[];
  allosteric_network?: AllostericNetwork;
}

/**
 * API pagination
 */
export interface Pagination {
  page: number;
  limit: number;
  total: number;
  total_pages: number;
}

/**
 * API error response
 */
export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

/**
 * API success response wrapper
 */
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  pagination?: Pagination;
}

// ============================================================================
// Query Types
// ============================================================================

/**
 * Protein dynamics query parameters
 */
export interface ProteinDynamicsQuery {
  protein_id?: string;
  protein_name?: string;
  organism?: string;
  has_ensemble?: boolean;
  has_binding_data?: boolean;
  min_resolution?: number;
  page?: number;
  limit?: number;
}

/**
 * Ensemble generation request
 */
export interface EnsembleGenerationRequest {
  protein_id: string;
  method: 'md' | 'metadynamics' | 'alphaflow';
  parameters?: Partial<SimulationParameters>;
  priority?: 'low' | 'normal' | 'high';
}

/**
 * Binding prediction request
 */
export interface BindingPredictionRequest {
  protein_id: string;
  ligand_smiles: string;
  method?: 'docking' | 'md' | 'fep';
  include_kinetics?: boolean;
}

// ============================================================================
// Integration Types
// ============================================================================

/**
 * Database integration source
 */
export interface DatabaseSource {
  name: 'pdb' | 'alphafold' | 'uniprot' | 'chembl' | 'bindingdb';
  base_url: string;
  last_sync?: string;
}

/**
 * Identifier mapping result
 */
export interface IdentifierMapping {
  source_id: string;
  source_type: ProteinIdType;
  mappings: Record<ProteinIdType, string | string[]>;
}

/**
 * Data synchronization status
 */
export interface SyncStatus {
  protein_id: string;
  sources: {
    source: string;
    last_sync: string;
    status: 'up_to_date' | 'update_available' | 'error';
  }[];
}
