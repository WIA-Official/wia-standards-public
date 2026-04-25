/**
 * WIA-BIO-008: Protein Structure - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bioinformatics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Structure Types
// ============================================================================

/**
 * 3D coordinates in Angstroms
 */
export interface Coordinates3D {
  x: number;
  y: number;
  z: number;
}

/**
 * Individual atom in protein structure
 */
export interface Atom {
  /** Atom name (e.g., "CA", "N", "C", "O") */
  name: string;

  /** Element symbol (e.g., "C", "N", "O") */
  element: string;

  /** 3D position in Angstroms */
  position: Coordinates3D;

  /** B-factor (temperature factor) */
  bFactor?: number;

  /** Occupancy (0-1) */
  occupancy?: number;

  /** Charge */
  charge?: number;
}

/**
 * Amino acid residue
 */
export interface Residue {
  /** Residue index (1-based) */
  index: number;

  /** Three-letter code (e.g., "MET", "ALA") */
  name: string;

  /** Single-letter code (e.g., "M", "A") */
  code: string;

  /** Chain identifier */
  chainId: string;

  /** Atoms in this residue */
  atoms: Atom[];

  /** Backbone dihedral angles */
  dihedrals?: {
    phi?: number;   // C(i-1)-N-CA-C
    psi?: number;   // N-CA-C-N(i+1)
    omega?: number; // CA-C-N-CA(i+1)
  };

  /** Secondary structure */
  secondaryStructure?: 'H' | 'E' | 'T' | 'C'; // Helix, Strand, Turn, Coil
}

/**
 * Protein chain
 */
export interface Chain {
  /** Chain identifier (e.g., "A", "B") */
  id: string;

  /** Residues in this chain */
  residues: Residue[];

  /** Chain type */
  type?: 'protein' | 'dna' | 'rna' | 'ligand' | 'water';

  /** Organism source */
  organism?: string;
}

/**
 * Complete protein structure
 */
export interface ProteinStructure {
  /** Structure identifier */
  id: string;

  /** Protein name */
  name?: string;

  /** PDB ID if available */
  pdbId?: string;

  /** UniProt ID if available */
  uniprotId?: string;

  /** Chains in the structure */
  chains: Chain[];

  /** Full amino acid sequence */
  sequence: string;

  /** Header information */
  header?: {
    classification?: string;
    deposition?: Date;
    release?: Date;
    authors?: string[];
    resolution?: number;
    method?: 'X-ray' | 'NMR' | 'Cryo-EM' | 'Model';
  };

  /** Quality metrics */
  quality?: QualityMetrics;
}

// ============================================================================
// Secondary Structure
// ============================================================================

/**
 * Secondary structure annotation
 */
export interface SecondaryStructureAnnotation {
  /** Per-residue assignments */
  assignments: ('H' | 'E' | 'T' | 'C')[];

  /** Helix regions */
  helices: StructureRegion[];

  /** Strand regions */
  strands: StructureRegion[];

  /** Turn regions */
  turns: StructureRegion[];

  /** Statistics */
  stats: {
    helixRatio: number;  // Fraction of helix residues
    strandRatio: number; // Fraction of strand residues
    coilRatio: number;   // Fraction of coil residues
  };
}

/**
 * Structural region (helix, strand, etc.)
 */
export interface StructureRegion {
  /** Start residue index */
  start: number;

  /** End residue index */
  end: number;

  /** Chain identifier */
  chainId: string;

  /** Type of structure */
  type: 'helix' | 'strand' | 'turn' | 'coil';

  /** Length in residues */
  length: number;
}

// ============================================================================
// Structure Prediction
// ============================================================================

/**
 * Structure prediction request
 */
export interface PredictionRequest {
  /** Amino acid sequence */
  sequence: string;

  /** Prediction method */
  method: 'alphafold' | 'rosettafold' | 'homology' | 'threading';

  /** Template structures for homology modeling */
  templates?: string[];

  /** Multiple sequence alignment */
  msa?: MultipleSequenceAlignment;

  /** Number of models to generate */
  numModels?: number;

  /** Additional parameters */
  parameters?: Record<string, unknown>;
}

/**
 * Multiple sequence alignment
 */
export interface MultipleSequenceAlignment {
  /** Query sequence */
  query: string;

  /** Aligned sequences */
  sequences: AlignedSequence[];

  /** Alignment depth */
  depth: number;
}

/**
 * Aligned sequence
 */
export interface AlignedSequence {
  /** Sequence identifier */
  id: string;

  /** Aligned sequence with gaps */
  sequence: string;

  /** Sequence identity to query */
  identity: number;

  /** E-value */
  evalue?: number;

  /** Organism */
  organism?: string;
}

/**
 * Structure prediction result
 */
export interface PredictionResult {
  /** Predicted structure */
  structure: ProteinStructure;

  /** Confidence metrics */
  confidence: ConfidenceMetrics;

  /** Secondary structure prediction */
  secondaryStructure: SecondaryStructureAnnotation;

  /** Metadata */
  metadata: PredictionMetadata;

  /** Alternative models */
  alternatives?: ProteinStructure[];
}

/**
 * Confidence metrics
 */
export interface ConfidenceMetrics {
  /** Per-residue confidence (pLDDT for AlphaFold) */
  perResidue: number[];

  /** Mean confidence */
  mean: number;

  /** Minimum confidence */
  min: number;

  /** Maximum confidence */
  max: number;

  /** Predicted aligned error (PAE) matrix */
  pae?: number[][];

  /** Confidence level */
  level: 'very-high' | 'high' | 'medium' | 'low';
}

/**
 * Prediction metadata
 */
export interface PredictionMetadata {
  /** Method used */
  method: string;

  /** Version of prediction software */
  version?: string;

  /** Prediction date */
  date: Date;

  /** Computation time in seconds */
  computationTime?: number;

  /** Number of templates used */
  numTemplates?: number;

  /** MSA depth */
  msaDepth?: number;
}

// ============================================================================
// Quality Assessment
// ============================================================================

/**
 * Quality metrics for structure validation
 */
export interface QualityMetrics {
  /** Root mean square deviation */
  rmsd?: number;

  /** TM-score */
  tmScore?: number;

  /** GDT-TS score */
  gdtTS?: number;

  /** Ramachandran statistics */
  ramachandran?: RamachandranStats;

  /** MolProbity score */
  molProbity?: MolProbityScore;

  /** Clash score */
  clashScore?: number;

  /** Overall quality */
  overall: 'excellent' | 'good' | 'acceptable' | 'poor';
}

/**
 * Ramachandran plot statistics
 */
export interface RamachandranStats {
  /** Percentage in favored region */
  favored: number;

  /** Percentage in allowed region */
  allowed: number;

  /** Percentage in outlier region */
  outlier: number;

  /** Outlier residue indices */
  outlierResidues?: number[];
}

/**
 * MolProbity score components
 */
export interface MolProbityScore {
  /** Overall score */
  score: number;

  /** Clash score */
  clashScore: number;

  /** Rotamer outliers percentage */
  rotamerOutliers: number;

  /** Ramachandran outliers percentage */
  ramachandranOutliers: number;

  /** Percentile rank */
  percentile?: number;
}

/**
 * RMSD calculation request
 */
export interface RMSDRequest {
  /** First structure */
  structure1: ProteinStructure;

  /** Second structure */
  structure2: ProteinStructure;

  /** Atoms to compare */
  atoms?: 'CA' | 'backbone' | 'all';

  /** Align structures before calculation */
  align?: boolean;

  /** Chain mapping */
  chainMapping?: Record<string, string>;
}

/**
 * RMSD calculation result
 */
export interface RMSDResult {
  /** RMSD value in Angstroms */
  value: number;

  /** Number of atoms compared */
  numAtoms: number;

  /** Rotation matrix (if aligned) */
  rotationMatrix?: number[][];

  /** Translation vector (if aligned) */
  translationVector?: Coordinates3D;

  /** Per-residue RMSD */
  perResidue?: number[];
}

// ============================================================================
// Molecular Dynamics
// ============================================================================

/**
 * Molecular dynamics simulation parameters
 */
export interface MDSimulationParams {
  /** Input structure */
  structure: ProteinStructure;

  /** Simulation duration in nanoseconds */
  duration: number;

  /** Temperature in Kelvin */
  temperature?: number;

  /** Pressure in bar */
  pressure?: number;

  /** Force field */
  forceField?: 'AMBER' | 'CHARMM' | 'GROMOS' | 'OPLS';

  /** Solvent model */
  solvent?: 'explicit' | 'implicit' | 'none';

  /** Timestep in femtoseconds */
  timestep?: number;

  /** Output frequency */
  outputFrequency?: number;
}

/**
 * Molecular dynamics trajectory
 */
export interface MDTrajectory {
  /** Simulation ID */
  id: string;

  /** Input parameters */
  parameters: MDSimulationParams;

  /** Frames */
  frames: ProteinStructure[];

  /** Time points in nanoseconds */
  times: number[];

  /** Energy values */
  energies?: EnergyTimeSeries;

  /** RMSD from starting structure */
  rmsd?: number[];

  /** Radius of gyration */
  radiusOfGyration?: number[];

  /** RMSF per residue */
  rmsf?: number[];
}

/**
 * Energy components over time
 */
export interface EnergyTimeSeries {
  /** Total energy */
  total: number[];

  /** Potential energy */
  potential: number[];

  /** Kinetic energy */
  kinetic: number[];

  /** Temperature */
  temperature: number[];

  /** Pressure */
  pressure?: number[];
}

// ============================================================================
// Protein-Ligand Docking
// ============================================================================

/**
 * Ligand structure
 */
export interface Ligand {
  /** Ligand identifier */
  id: string;

  /** Ligand name */
  name?: string;

  /** SMILES string */
  smiles?: string;

  /** InChI string */
  inchi?: string;

  /** 3D coordinates */
  atoms?: Atom[];

  /** Molecular weight */
  molecularWeight?: number;

  /** Chemical formula */
  formula?: string;
}

/**
 * Binding site definition
 */
export interface BindingSite {
  /** Center of binding site */
  center: Coordinates3D;

  /** Radius in Angstroms */
  radius: number;

  /** Residues in binding site */
  residues?: number[];

  /** Site name/identifier */
  name?: string;
}

/**
 * Docking request
 */
export interface DockingRequest {
  /** Protein structure */
  protein: ProteinStructure;

  /** Ligand */
  ligand: string | Ligand; // SMILES or Ligand object

  /** Binding site */
  bindingSite?: BindingSite;

  /** Docking exhaustiveness (1-8+) */
  exhaustiveness?: number;

  /** Number of binding modes to generate */
  numModes?: number;

  /** Energy range for modes (kcal/mol) */
  energyRange?: number;

  /** Docking software */
  software?: 'autodock-vina' | 'glide' | 'gold' | 'dock';
}

/**
 * Docking result
 */
export interface DockingResult {
  /** Docking poses */
  poses: DockingPose[];

  /** Best binding affinity (kcal/mol) */
  bindingAffinity: number;

  /** Best pose index */
  bestPoseIndex: number;

  /** Protein-ligand interactions */
  interactions: Interaction[];

  /** Binding site residues */
  bindingSiteResidues: number[];

  /** Computation time */
  computationTime?: number;
}

/**
 * Docking pose (protein-ligand complex)
 */
export interface DockingPose {
  /** Pose rank */
  rank: number;

  /** Binding affinity (kcal/mol) */
  affinity: number;

  /** Ligand conformation */
  ligand: Ligand;

  /** RMSD from reference (if available) */
  rmsd?: number;

  /** Interactions in this pose */
  interactions: Interaction[];
}

/**
 * Protein-ligand interaction
 */
export interface Interaction {
  /** Interaction type */
  type: 'hydrogen-bond' | 'hydrophobic' | 'pi-stacking' | 'salt-bridge' | 'vdw' | 'electrostatic';

  /** Protein residue */
  residue: {
    index: number;
    name: string;
    chainId: string;
    atom?: string;
  };

  /** Ligand atom */
  ligandAtom?: string;

  /** Distance in Angstroms */
  distance: number;

  /** Interaction strength/energy */
  strength?: number;
}

// ============================================================================
// Structural Alignment
// ============================================================================

/**
 * Structural alignment request
 */
export interface AlignmentRequest {
  /** Reference structure */
  reference: ProteinStructure;

  /** Mobile structure to align */
  mobile: ProteinStructure;

  /** Alignment method */
  method?: 'kabsch' | 'tm-align' | 'ce' | 'fatcat';

  /** Atoms to use for alignment */
  atoms?: 'CA' | 'backbone' | 'all';
}

/**
 * Structural alignment result
 */
export interface AlignmentResult {
  /** Aligned mobile structure */
  alignedStructure: ProteinStructure;

  /** RMSD after alignment */
  rmsd: number;

  /** TM-score */
  tmScore: number;

  /** Number of aligned residues */
  numAligned: number;

  /** Sequence alignment */
  sequenceAlignment: {
    reference: string;
    mobile: string;
    identity: number;
  };

  /** Rotation matrix */
  rotationMatrix: number[][];

  /** Translation vector */
  translationVector: Coordinates3D;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for protein structure calculations
 */
export const PROTEIN_STRUCTURE_CONSTANTS = {
  /** Gas constant in kcal/(mol·K) */
  GAS_CONSTANT: 0.001987,

  /** Room temperature in Kelvin */
  ROOM_TEMPERATURE: 298.15,

  /** Physiological temperature in Kelvin */
  PHYSIOLOGICAL_TEMPERATURE: 310.15,

  /** Van der Waals radii (Angstroms) */
  VDW_RADII: {
    C: 1.70,
    N: 1.55,
    O: 1.52,
    S: 1.80,
    H: 1.20,
  },

  /** Typical bond lengths (Angstroms) */
  BOND_LENGTHS: {
    'C-C': 1.54,
    'C-N': 1.47,
    'C-O': 1.43,
    'C=O': 1.23,
    'N-H': 1.01,
    'O-H': 0.96,
  },

  /** Amino acid molecular weights (Da) */
  AMINO_ACID_WEIGHTS: {
    A: 89.09, C: 121.16, D: 133.10, E: 147.13,
    F: 165.19, G: 75.07, H: 155.16, I: 131.17,
    K: 146.19, L: 131.17, M: 149.21, N: 132.12,
    P: 115.13, Q: 146.15, R: 174.20, S: 105.09,
    T: 119.12, V: 117.15, W: 204.23, Y: 181.19,
  },

  /** Quality thresholds */
  QUALITY_THRESHOLDS: {
    RMSD_EXCELLENT: 1.0,
    RMSD_GOOD: 2.0,
    TM_SCORE_SAME_FOLD: 0.5,
    TM_SCORE_EXCELLENT: 0.8,
    RAMACHANDRAN_FAVORED_MIN: 95.0,
    MOLPROBITY_GOOD: 2.0,
    CLASH_SCORE_GOOD: 10.0,
  },

  /** Hydrogen bond criteria */
  HBOND_MAX_DISTANCE: 3.5, // Angstroms
  HBOND_MIN_ANGLE: 120,    // Degrees

  /** Secondary structure cutoffs */
  HELIX_PHI: -60,  // Degrees
  HELIX_PSI: -45,  // Degrees
  STRAND_PHI: -120, // Degrees
  STRAND_PSI: 120,  // Degrees
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Amino acid codes
 */
export type AminoAcid =
  | 'A' | 'C' | 'D' | 'E' | 'F' | 'G' | 'H' | 'I' | 'K' | 'L'
  | 'M' | 'N' | 'P' | 'Q' | 'R' | 'S' | 'T' | 'V' | 'W' | 'Y';

/**
 * Three-letter amino acid codes
 */
export type AminoAcidThreeLetter =
  | 'ALA' | 'CYS' | 'ASP' | 'GLU' | 'PHE' | 'GLY' | 'HIS' | 'ILE' | 'LYS' | 'LEU'
  | 'MET' | 'ASN' | 'PRO' | 'GLN' | 'ARG' | 'SER' | 'THR' | 'VAL' | 'TRP' | 'TYR';

/**
 * Secondary structure types
 */
export type SecondaryStructureType = 'H' | 'E' | 'T' | 'C';

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-008 error codes
 */
export enum ProteinErrorCode {
  INVALID_SEQUENCE = 'B001',
  STRUCTURE_NOT_FOUND = 'B002',
  ALIGNMENT_FAILED = 'B003',
  LOW_CONFIDENCE = 'B004',
  DOCKING_FAILED = 'B005',
  INVALID_FORMAT = 'B006',
  SIMULATION_FAILED = 'B007',
  QUALITY_FAILED = 'B008',
}

/**
 * Protein structure error
 */
export class ProteinStructureError extends Error {
  constructor(
    public code: ProteinErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ProteinStructureError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Coordinates3D,
  Atom,
  Residue,
  Chain,
  ProteinStructure,

  // Secondary structure
  SecondaryStructureAnnotation,
  StructureRegion,

  // Prediction
  PredictionRequest,
  PredictionResult,
  MultipleSequenceAlignment,
  AlignedSequence,
  ConfidenceMetrics,
  PredictionMetadata,

  // Quality
  QualityMetrics,
  RamachandranStats,
  MolProbityScore,
  RMSDRequest,
  RMSDResult,

  // Molecular dynamics
  MDSimulationParams,
  MDTrajectory,
  EnergyTimeSeries,

  // Docking
  Ligand,
  BindingSite,
  DockingRequest,
  DockingResult,
  DockingPose,
  Interaction,

  // Alignment
  AlignmentRequest,
  AlignmentResult,
};

export { PROTEIN_STRUCTURE_CONSTANTS, ProteinErrorCode, ProteinStructureError };
