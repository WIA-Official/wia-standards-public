/**
 * WIA-BIO-012: Synthetic Biology - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Biology Types
// ============================================================================

/**
 * BioBrick part types
 */
export type PartType =
  | 'promoter'
  | 'rbs'
  | 'cds'
  | 'terminator'
  | 'operator'
  | 'origin'
  | 'plasmid_backbone'
  | 'composite';

/**
 * Chassis organisms
 */
export type ChassisOrganism =
  | 'E. coli'
  | 'B. subtilis'
  | 'S. cerevisiae'
  | 'P. pastoris'
  | 'CHO'
  | 'HEK293'
  | 'custom';

/**
 * DNA assembly methods
 */
export type AssemblyMethod =
  | 'biobrick'
  | 'golden-gate'
  | 'gibson'
  | 'cpec'
  | 'moclo'
  | 'gateway'
  | 'custom';

/**
 * Biosafety levels
 */
export type BiosafetLevel = 'BSL-1' | 'BSL-2' | 'BSL-3' | 'BSL-4';

// ============================================================================
// BioBrick Parts
// ============================================================================

/**
 * Standard BioBrick part
 */
export interface BioBrickPart {
  /** Part identifier (e.g., BBa_J23100) */
  id: string;

  /** Part name */
  name?: string;

  /** Part type */
  type: PartType;

  /** DNA sequence */
  sequence: string;

  /** Sequence length in base pairs */
  length: number;

  /** Compatible chassis organisms */
  organisms: ChassisOrganism[];

  /** Part characterization data */
  characterization?: PartCharacterization;

  /** Author/contributor */
  author?: string;

  /** Part status */
  status: 'available' | 'planning' | 'deleted' | 'deprecated';

  /** Registry URL */
  registryUrl?: string;
}

/**
 * Part characterization data
 */
export interface PartCharacterization {
  /** Measurement method */
  method: string;

  /** Measurement units */
  units: string;

  /** Measured value */
  value?: number;

  /** Standard deviation */
  stddev?: number;

  /** Experimental conditions */
  conditions?: string;

  /** Reference publication */
  reference?: string;
}

/**
 * Promoter-specific data
 */
export interface PromoterPart extends BioBrickPart {
  type: 'promoter';

  /** Promoter strength in RPU (Relative Promoter Units) */
  strength: number;

  /** Is it inducible? */
  inducible: boolean;

  /** Inducer molecule (if inducible) */
  inducer?: string;

  /** Dissociation constant Kd (M) */
  kd?: number;

  /** Hill coefficient */
  hillCoefficient?: number;
}

/**
 * RBS-specific data
 */
export interface RBSPart extends BioBrickPart {
  type: 'rbs';

  /** Translation initiation rate */
  translationRate: number;

  /** Free energy of binding (kcal/mol) */
  deltaG?: number;

  /** Spacing to start codon (bp) */
  spacing?: number;
}

/**
 * Coding sequence data
 */
export interface CDSPart extends BioBrickPart {
  type: 'cds';

  /** Protein name */
  protein: string;

  /** Protein function */
  function: string;

  /** Amino acid sequence */
  aminoAcidSequence?: string;

  /** Molecular weight (Da) */
  molecularWeight?: number;

  /** Codon Adaptation Index */
  cai?: number;

  /** GC content (%) */
  gcContent?: number;
}

/**
 * Terminator data
 */
export interface TerminatorPart extends BioBrickPart {
  type: 'terminator';

  /** Termination efficiency (%) */
  efficiency: number;

  /** Terminator type */
  terminatorType: 'intrinsic' | 'rho-dependent' | 'bidirectional';
}

// ============================================================================
// Genetic Circuits
// ============================================================================

/**
 * Genetic circuit design
 */
export interface GeneticCircuit {
  /** Circuit identifier */
  id: string;

  /** Circuit name */
  name: string;

  /** BioBrick parts in order */
  parts: string[];

  /** Host organism */
  host: ChassisOrganism;

  /** Plasmid backbone */
  plasmid?: string;

  /** Circuit purpose/function */
  purpose: string;

  /** Circuit topology */
  topology?: CircuitTopology;

  /** Expected behavior */
  expectedBehavior?: CircuitBehavior;

  /** Assembly method */
  assemblyMethod: AssemblyMethod;

  /** Total construct size (bp) */
  size?: number;

  /** Biosafety level required */
  biosafety: BiosafetLevel;

  /** Creation date */
  created: Date;
}

/**
 * Circuit topology/architecture
 */
export interface CircuitTopology {
  /** Number of transcription units */
  transcriptionUnits: number;

  /** Number of regulatory connections */
  connections: number;

  /** Circuit type */
  type:
    | 'simple'
    | 'logic-gate'
    | 'oscillator'
    | 'toggle-switch'
    | 'feed-forward'
    | 'feedback'
    | 'custom';

  /** Regulatory edges */
  edges?: RegulatoryEdge[];
}

/**
 * Regulatory connection between parts
 */
export interface RegulatoryEdge {
  /** Source part ID */
  from: string;

  /** Target part ID */
  to: string;

  /** Regulation type */
  type: 'activation' | 'repression' | 'no-effect';

  /** Strength of regulation (0-1) */
  strength?: number;
}

/**
 * Expected circuit behavior
 */
export interface CircuitBehavior {
  /** Expression level (low/medium/high) */
  expressionLevel: 'low' | 'medium' | 'high';

  /** Response time (seconds) */
  responseTime?: number;

  /** Dynamic range (fold change) */
  dynamicRange?: number;

  /** Stability */
  stability: 'stable' | 'oscillating' | 'bistable' | 'chaotic';

  /** Metabolic burden (0-1) */
  metabolicBurden?: number;
}

// ============================================================================
// Promoter and Gene Expression
// ============================================================================

/**
 * Promoter strength calculation parameters
 */
export interface PromoterParameters {
  /** Maximum promoter strength (RPU) */
  pmax: number;

  /** Inducer concentration (M) */
  inducerConcentration?: number;

  /** Dissociation constant (M) */
  kd?: number;

  /** Hill coefficient (cooperativity) */
  hillCoefficient?: number;

  /** Basal expression level (RPU) */
  basalLevel?: number;
}

/**
 * Promoter strength result
 */
export interface PromoterStrength {
  /** Current strength (RPU) */
  strength: number;

  /** Fold induction (induced/basal) */
  foldInduction?: number;

  /** Saturation (% of maximum) */
  saturation: number;

  /** Is fully induced? */
  isFullyInduced: boolean;
}

/**
 * Gene expression parameters
 */
export interface GeneExpressionParameters {
  /** Transcription rate constant (per second) */
  transcriptionRate: number;

  /** Translation rate constant (per second) */
  translationRate: number;

  /** mRNA degradation rate (per second) */
  mRNADegradation: number;

  /** Protein degradation rate (per second) */
  proteinDegradation: number;

  /** Simulation duration (seconds) */
  duration: number;

  /** Time step for simulation (seconds) */
  timeStep?: number;
}

/**
 * Gene expression result
 */
export interface GeneExpressionResult {
  /** Final mRNA concentration (molecules/cell) */
  finalmRNAConcentration: number;

  /** Final protein concentration (molecules/cell) */
  finalProteinConcentration: number;

  /** Time to steady state (seconds) */
  steadyStateTime: number;

  /** mRNA half-life (seconds) */
  mRNAHalfLife: number;

  /** Protein half-life (seconds) */
  proteinHalfLife: number;

  /** Time series data */
  timeSeries?: TimeSeriesData[];
}

/**
 * Time series data point
 */
export interface TimeSeriesData {
  /** Time point (seconds) */
  time: number;

  /** mRNA level */
  mRNA: number;

  /** Protein level */
  protein: number;
}

// ============================================================================
// Metabolic Engineering
// ============================================================================

/**
 * Metabolic pathway
 */
export interface MetabolicPathway {
  /** Pathway identifier */
  id: string;

  /** Pathway name */
  name: string;

  /** Substrate (input) */
  substrate: string;

  /** Product (output) */
  product: string;

  /** Enzymes/reactions in pathway */
  reactions: EnzymeReaction[];

  /** Theoretical yield (mol/mol) */
  theoreticalYield: number;

  /** Host organism */
  host: ChassisOrganism;

  /** Pathway classification */
  classification: 'native' | 'heterologous' | 'synthetic';
}

/**
 * Enzyme-catalyzed reaction
 */
export interface EnzymeReaction {
  /** Reaction identifier */
  id: string;

  /** Enzyme name */
  enzyme: string;

  /** EC number */
  ecNumber?: string;

  /** Substrates */
  substrates: Metabolite[];

  /** Products */
  products: Metabolite[];

  /** Kinetic parameters */
  kinetics?: MichaelisMentenKinetics;

  /** Flux control coefficient */
  controlCoefficient?: number;
}

/**
 * Metabolite
 */
export interface Metabolite {
  /** Metabolite name */
  name: string;

  /** Stoichiometric coefficient */
  coefficient: number;

  /** Concentration (M) */
  concentration?: number;
}

/**
 * Michaelis-Menten kinetics
 */
export interface MichaelisMentenKinetics {
  /** Maximum velocity (μmol/min/mg) */
  vmax: number;

  /** Michaelis constant (M) */
  km: number;

  /** Catalytic efficiency (kcat/Km) */
  catalyticEfficiency?: number;

  /** Turnover number kcat (per second) */
  kcat?: number;
}

/**
 * Pathway optimization parameters
 */
export interface PathwayOptimizationParams {
  /** Target product */
  target: string;

  /** Substrate */
  substrate: string;

  /** Host organism */
  host: ChassisOrganism;

  /** Optimization objective */
  objective: 'yield' | 'titer' | 'productivity' | 'specificity';

  /** Constraints */
  constraints?: {
    /** Maximum number of genes */
    maxGenes?: number;

    /** Target yield (mol/mol) */
    targetYield?: number;

    /** Cofactor balance required */
    cofactorBalance?: boolean;

    /** Maximum metabolic burden */
    maxBurden?: number;
  };
}

/**
 * Pathway optimization result
 */
export interface PathwayOptimizationResult {
  /** Optimized pathway */
  pathway: MetabolicPathway;

  /** Genes to overexpress */
  genesToOverexpress: string[];

  /** Genes to delete */
  genesToDelete: string[];

  /** Predicted yield (mol/mol) */
  predictedYield: number;

  /** Flux distribution */
  fluxDistribution: Record<string, number>;

  /** Identified bottlenecks */
  bottlenecks: string[];

  /** Optimization score */
  score: number;
}

// ============================================================================
// DNA Assembly
// ============================================================================

/**
 * DNA assembly plan
 */
export interface AssemblyPlan {
  /** Assembly identifier */
  id: string;

  /** Method to use */
  method: AssemblyMethod;

  /** Parts to assemble */
  parts: BioBrickPart[];

  /** Assembly order */
  order: number[];

  /** Expected product size (bp) */
  expectedSize: number;

  /** Required reagents */
  reagents?: string[];

  /** Estimated time (hours) */
  estimatedTime?: number;

  /** Success probability (0-1) */
  successProbability?: number;
}

/**
 * Assembly result
 */
export interface AssemblyResult {
  /** Was assembly successful? */
  success: boolean;

  /** Final construct */
  construct?: GeneticCircuit;

  /** Assembled sequence */
  sequence?: string;

  /** Actual size (bp) */
  actualSize?: number;

  /** Errors encountered */
  errors: string[];

  /** Warnings */
  warnings: string[];

  /** Verification method */
  verificationMethod?: 'sequencing' | 'restriction' | 'pcr' | 'none';
}

// ============================================================================
// Cell-Free Systems
// ============================================================================

/**
 * Cell-free expression system
 */
export interface CellFreeSystem {
  /** System type */
  type: 'PURE' | 'S30' | 'S12' | 'custom';

  /** Source organism */
  source: ChassisOrganism;

  /** Components */
  components: CellFreeComponent[];

  /** Reaction conditions */
  conditions: ReactionConditions;

  /** Expected yield (μg/mL) */
  expectedYield?: number;

  /** Duration (hours) */
  duration: number;
}

/**
 * Cell-free system component
 */
export interface CellFreeComponent {
  /** Component name */
  name: string;

  /** Concentration */
  concentration: number;

  /** Units */
  units: string;

  /** Function */
  function: 'transcription' | 'translation' | 'energy' | 'cofactor' | 'other';
}

/**
 * Reaction conditions
 */
export interface ReactionConditions {
  /** Temperature (°C) */
  temperature: number;

  /** pH */
  pH: number;

  /** Mg2+ concentration (mM) */
  magnesium?: number;

  /** K+ concentration (mM) */
  potassium?: number;

  /** Buffer system */
  buffer?: string;
}

// ============================================================================
// Biosafety and Risk Assessment
// ============================================================================

/**
 * Biosafety assessment
 */
export interface BiosafetAssessment {
  /** Required biosafety level */
  level: BiosafetLevel;

  /** Risk score (1-25) */
  riskScore: number;

  /** Risk category */
  riskCategory: 'low' | 'medium' | 'high' | 'extreme';

  /** Identified hazards */
  hazards: BiohazardType[];

  /** Required containment measures */
  containment: string[];

  /** Kill switch required? */
  killSwitchRequired: boolean;

  /** Ethics review required? */
  ethicsReviewRequired: boolean;

  /** Dual-use concern? */
  dualUseConcern: boolean;

  /** Mitigation strategies */
  mitigation: string[];
}

/**
 * Types of biological hazards
 */
export type BiohazardType =
  | 'pathogenicity'
  | 'toxin-production'
  | 'allergen'
  | 'environmental-release'
  | 'horizontal-gene-transfer'
  | 'antibiotic-resistance'
  | 'immune-evasion'
  | 'dual-use';

/**
 * Risk assessment result
 */
export interface RiskAssessment {
  /** Likelihood score (1-5) */
  likelihood: number;

  /** Severity score (1-5) */
  severity: number;

  /** Overall risk (likelihood × severity) */
  overallRisk: number;

  /** Risk description */
  description: string;

  /** Recommended actions */
  recommendations: string[];
}

// ============================================================================
// Simulation and Modeling
// ============================================================================

/**
 * Circuit simulation parameters
 */
export interface SimulationParameters {
  /** Circuit to simulate */
  circuit: GeneticCircuit;

  /** Simulation duration (seconds) */
  duration: number;

  /** Time step (seconds) */
  timeStep: number;

  /** Initial conditions */
  initialConditions?: Record<string, number>;

  /** Environmental conditions */
  environment?: EnvironmentalConditions;

  /** Simulation method */
  method: 'deterministic' | 'stochastic' | 'hybrid';
}

/**
 * Environmental conditions
 */
export interface EnvironmentalConditions {
  /** Temperature (°C) */
  temperature: number;

  /** Growth phase */
  growthPhase: 'lag' | 'exponential' | 'stationary' | 'death';

  /** Inducer concentrations */
  inducers?: Record<string, number>;

  /** Nutrient availability */
  nutrients?: Record<string, number>;
}

/**
 * Simulation result
 */
export interface SimulationResult {
  /** Simulation identifier */
  id: string;

  /** Was simulation successful? */
  success: boolean;

  /** Time series data */
  timeSeries: TimeSeriesData[];

  /** Final state */
  finalState: Record<string, number>;

  /** Steady state reached? */
  steadyStateReached: boolean;

  /** Time to steady state (seconds) */
  timeToSteadyState?: number;

  /** Simulation warnings */
  warnings: string[];

  /** Computation time (ms) */
  computationTime: number;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Biological constants
 */
export const BIO_CONSTANTS = {
  /** RNA polymerase speed (nt/s) */
  RNAP_SPEED: 60, // nucleotides per second

  /** Ribosome speed (aa/s) */
  RIBOSOME_SPEED: 17, // amino acids per second

  /** Typical mRNA half-life (seconds) */
  MRNA_HALF_LIFE: 300, // 5 minutes

  /** Typical protein half-life (seconds) */
  PROTEIN_HALF_LIFE: 3600, // 1 hour

  /** Avogadro's number */
  AVOGADRO: 6.022e23,

  /** Gas constant (cal/mol·K) */
  GAS_CONSTANT: 1.987,

  /** Standard temperature (K) */
  STANDARD_TEMP: 310, // 37°C

  /** E. coli cell volume (L) */
  ECOLI_VOLUME: 1e-15,

  /** Molecules to nM conversion */
  MOLECULES_TO_NM: 1.66, // for E. coli

  /** Plasmid copy number range */
  PLASMID_COPY_LOW: 15,
  PLASMID_COPY_HIGH: 300,
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

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-012 error codes
 */
export enum BioErrorCode {
  INVALID_PART = 'B001',
  ASSEMBLY_FAILED = 'B002',
  SIZE_EXCEEDED = 'B003',
  BIOSAFETY_VIOLATION = 'B004',
  EXPRESSION_TOO_HIGH = 'B005',
  PATHWAY_IMBALANCE = 'B006',
  INCOMPATIBLE_PARTS = 'B007',
  INVALID_SEQUENCE = 'B008',
  SIMULATION_FAILED = 'B009',
  OPTIMIZATION_FAILED = 'B010',
}

/**
 * Synthetic biology error
 */
export class SyntheticBiologyError extends Error {
  constructor(
    public code: BioErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'SyntheticBiologyError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Parts
  BioBrickPart,
  PartCharacterization,
  PromoterPart,
  RBSPart,
  CDSPart,
  TerminatorPart,

  // Circuits
  GeneticCircuit,
  CircuitTopology,
  RegulatoryEdge,
  CircuitBehavior,

  // Expression
  PromoterParameters,
  PromoterStrength,
  GeneExpressionParameters,
  GeneExpressionResult,
  TimeSeriesData,

  // Metabolism
  MetabolicPathway,
  EnzymeReaction,
  Metabolite,
  MichaelisMentenKinetics,
  PathwayOptimizationParams,
  PathwayOptimizationResult,

  // Assembly
  AssemblyPlan,
  AssemblyResult,

  // Cell-free
  CellFreeSystem,
  CellFreeComponent,
  ReactionConditions,

  // Safety
  BiosafetAssessment,
  RiskAssessment,

  // Simulation
  SimulationParameters,
  EnvironmentalConditions,
  SimulationResult,
};

export { BIO_CONSTANTS, BioErrorCode, SyntheticBiologyError };
