/**
 * WIA-BIO-014: CRISPR Protocol - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core CRISPR Types
// ============================================================================

/**
 * CRISPR-Cas system types
 */
export type CasSystemType =
  | 'SpCas9'      // Streptococcus pyogenes
  | 'SaCas9'      // Staphylococcus aureus
  | 'Cas12a'      // Cpf1
  | 'Cas13'       // RNA targeting
  | 'BaseEditor'  // Base editing
  | 'PrimeEditor' // Prime editing
  | 'Custom';

/**
 * PAM (Protospacer Adjacent Motif) sequences
 */
export type PAMType =
  | 'NGG'      // SpCas9
  | 'NAG'      // SpCas9 (low activity)
  | 'NGA'      // SpCas9 (low activity)
  | 'NNGRRT'   // SaCas9
  | 'TTTV'     // Cas12a (V = A, C, or G)
  | 'None';    // Cas13 (RNA targeting)

/**
 * Editing outcome types
 */
export type EditType =
  | 'NHEJ'       // Non-Homologous End Joining
  | 'HDR'        // Homology-Directed Repair
  | 'base_edit'  // Base editing (C→T or A→G)
  | 'prime_edit' // Prime editing
  | 'deletion'   // Large deletion
  | 'insertion'; // Insertion

/**
 * Organism/species for gRNA design
 */
export type Organism =
  | 'human'
  | 'mouse'
  | 'rat'
  | 'zebrafish'
  | 'fly'
  | 'worm'
  | 'yeast'
  | 'arabidopsis'
  | 'custom';

// ============================================================================
// Guide RNA Design
// ============================================================================

/**
 * Guide RNA design request
 */
export interface GuideRNARequest {
  /** Target DNA sequence (20-23bp including PAM) */
  targetSequence: string;

  /** PAM type for Cas system */
  pamType: PAMType;

  /** Target organism */
  organism: Organism;

  /** Chromosome (e.g., 'chr7', '7', 'X') */
  chromosome?: string;

  /** Genomic position (0-based) */
  position?: number;

  /** Strand orientation */
  strand?: '+' | '-';

  /** Gene name or ID */
  gene?: string;

  /** Custom genome reference */
  genomeReference?: string;
}

/**
 * Guide RNA design response
 */
export interface GuideRNA {
  /** Guide RNA sequence (without PAM) */
  sequence: string;

  /** PAM sequence */
  pam: string;

  /** Full target sequence (gRNA + PAM) */
  fullTarget: string;

  /** On-target activity score (0-1) */
  onTargetScore: number;

  /** GC content percentage (0-100) */
  gcContent: number;

  /** Predicted editing efficiency (0-100%) */
  predictedEfficiency: number;

  /** Specificity score (0-100) */
  specificityScore: number;

  /** Design warnings */
  warnings: string[];

  /** Genomic coordinates */
  coordinates?: GenomicCoordinates;

  /** Predicted off-target sites */
  offTargets?: OffTargetSite[];
}

/**
 * Genomic coordinates
 */
export interface GenomicCoordinates {
  /** Chromosome */
  chromosome: string;

  /** Start position (0-based) */
  start: number;

  /** End position (0-based) */
  end: number;

  /** Strand */
  strand: '+' | '-';

  /** Genome assembly (e.g., 'hg38', 'mm10') */
  assembly: string;
}

// ============================================================================
// Off-Target Prediction
// ============================================================================

/**
 * Off-target prediction request
 */
export interface OffTargetRequest {
  /** Guide RNA sequence */
  guideRNA: string;

  /** Genome assembly */
  genome: string;

  /** Maximum number of mismatches */
  maxMismatches: number;

  /** Include DNA bulges */
  includeBulges?: boolean;

  /** Include RNA bulges */
  includeRNABulges?: boolean;

  /** Include PAM variants (NAG, NGA) */
  includePAMVariants?: boolean;

  /** Filter by chromatin accessibility */
  filterByChromatin?: boolean;
}

/**
 * Off-target site
 */
export interface OffTargetSite {
  /** Genomic coordinates */
  coordinates: GenomicCoordinates;

  /** Off-target sequence */
  sequence: string;

  /** Number of mismatches */
  mismatches: number;

  /** Positions of mismatches (0-based) */
  mismatchPositions: number[];

  /** Number of DNA bulges */
  dnaBulges?: number;

  /** Number of RNA bulges */
  rnaBulges?: number;

  /** CFD (Cutting Frequency Determination) score (0-1) */
  cfdScore: number;

  /** MIT specificity score (0-100) */
  mitScore: number;

  /** Genomic annotation */
  annotation: OffTargetAnnotation;

  /** Chromatin state */
  chromatinState?: 'open' | 'closed' | 'intermediate';

  /** Predicted cleavage activity (0-100%) */
  predictedActivity: number;
}

/**
 * Off-target annotation
 */
export interface OffTargetAnnotation {
  /** Genomic region type */
  region: 'exon' | 'intron' | 'promoter' | 'intergenic' | 'UTR' | 'enhancer';

  /** Gene name (if applicable) */
  gene?: string;

  /** Gene ID */
  geneId?: string;

  /** Transcript ID */
  transcriptId?: string;

  /** Functional consequence */
  consequence?: string;
}

// ============================================================================
// CRISPR Protocol
// ============================================================================

/**
 * Complete CRISPR protocol
 */
export interface CRISPRProtocol {
  /** Protocol ID */
  id: string;

  /** Protocol version */
  version: string;

  /** Creation date */
  created: Date;

  /** Last updated */
  updated: Date;

  /** Target information */
  target: TargetInfo;

  /** Guide RNA */
  guideRNA: GuideRNA;

  /** Cas system */
  casSystem: CasSystem;

  /** Delivery method */
  delivery: DeliveryMethod;

  /** Editing strategy */
  strategy: EditingStrategy;

  /** Validation methods */
  validation: ValidationMethod[];

  /** Safety checks */
  safety: SafetyChecks;

  /** Experimental parameters */
  parameters?: ExperimentalParameters;
}

/**
 * Target information
 */
export interface TargetInfo {
  /** Gene symbol */
  gene: string;

  /** Gene ID (Ensembl, NCBI, etc.) */
  geneId?: string;

  /** Genomic coordinates */
  coordinates: GenomicCoordinates;

  /** Target region type */
  regionType: 'exon' | 'intron' | 'promoter' | 'enhancer' | 'other';

  /** Functional purpose */
  purpose: 'knockout' | 'knockin' | 'correction' | 'tagging' | 'activation' | 'repression';

  /** Description */
  description?: string;
}

/**
 * Cas system configuration
 */
export interface CasSystem {
  /** System type */
  type: CasSystemType;

  /** Specific variant */
  variant: 'WT' | 'HF1' | 'HiFi' | 'eSpCas9' | 'xCas9' | 'Custom';

  /** Cas protein source */
  source: 'recombinant' | 'plasmid' | 'mRNA' | 'virus';

  /** Concentration (if applicable) */
  concentration?: string;

  /** Modifications */
  modifications?: string[];
}

/**
 * Delivery method
 */
export interface DeliveryMethod {
  /** Delivery type */
  method:
    | 'RNP_electroporation'
    | 'RNP_nucleofection'
    | 'RNP_microinjection'
    | 'plasmid_transfection'
    | 'lentivirus'
    | 'AAV'
    | 'adenovirus'
    | 'LNP'
    | 'nanoparticle'
    | 'other';

  /** Cell type */
  cellType: string;

  /** Detailed parameters */
  parameters: DeliveryParameters;

  /** Expected efficiency (0-100%) */
  expectedEfficiency?: number;
}

/**
 * Delivery parameters
 */
export interface DeliveryParameters {
  /** Electroporation voltage (V) */
  voltage?: number;

  /** Pulse duration (ms) */
  pulseDuration?: number;

  /** Number of pulses */
  numPulses?: number;

  /** Viral titer (vg/mL or TU/mL) */
  titer?: string;

  /** MOI (Multiplicity of Infection) */
  moi?: number;

  /** Incubation time */
  incubationTime?: string;

  /** Additional details */
  [key: string]: unknown;
}

/**
 * Editing strategy
 */
export interface EditingStrategy {
  /** Editing type */
  type: EditType;

  /** Repair template (for HDR) */
  template?: RepairTemplate;

  /** Prime editing pegRNA (for prime editing) */
  pegRNA?: PrimeEditingRNA;

  /** Expected outcome */
  expectedOutcome: string;

  /** Timeline */
  timeline: string;
}

/**
 * Repair template for HDR
 */
export interface RepairTemplate {
  /** Template type */
  type: 'ssODN' | 'dsDNA' | 'plasmid';

  /** Template sequence */
  sequence: string;

  /** Left homology arm length (bp) */
  leftHA: number;

  /** Right homology arm length (bp) */
  rightHA: number;

  /** Desired edit */
  edit: string;

  /** Strand preference */
  strand?: 'target' | 'non-target';
}

/**
 * Prime editing guide RNA
 */
export interface PrimeEditingRNA {
  /** Spacer sequence */
  spacer: string;

  /** PBS (Primer Binding Site) */
  pbs: string;

  /** RT (Reverse Transcriptase) template */
  rtTemplate: string;

  /** Nicking sgRNA (for PE3) */
  nickingSgRNA?: string;

  /** Extension type */
  extension?: 'standard' | 'epegRNA';
}

/**
 * Validation method
 */
export interface ValidationMethod {
  /** Method name */
  method: 'T7E1' | 'Sanger' | 'NGS' | 'ddPCR' | 'Western' | 'Flow' | 'Other';

  /** Description */
  description: string;

  /** Timing (days post-transfection) */
  timing: number;

  /** Parameters */
  parameters?: Record<string, unknown>;
}

/**
 * Safety checks
 */
export interface SafetyChecks {
  /** Off-target analysis completed */
  offTargetAnalysis: boolean;

  /** Ethics approval obtained */
  ethicsApproval: boolean;

  /** Biosafety level */
  biosafety: 'BSL1' | 'BSL2' | 'BSL3' | 'BSL4';

  /** Regulatory compliance */
  regulatory: string[];

  /** Risk assessment */
  riskLevel: 'low' | 'medium' | 'high';

  /** Mitigation strategies */
  mitigation: string[];
}

/**
 * Experimental parameters
 */
export interface ExperimentalParameters {
  /** Cell density */
  cellDensity?: string;

  /** Culture conditions */
  cultureConditions?: string;

  /** Treatment duration */
  treatmentDuration?: string;

  /** Recovery time */
  recoveryTime?: string;

  /** Selection method */
  selection?: string;

  /** Custom parameters */
  [key: string]: unknown;
}

// ============================================================================
// Editing Efficiency
// ============================================================================

/**
 * Editing efficiency calculation request
 */
export interface EfficiencyRequest {
  /** Number of edited reads */
  editedReads: number;

  /** Total number of reads */
  totalReads: number;

  /** Edit type */
  editType: EditType;

  /** Sequencing method */
  sequencingMethod?: 'Sanger' | 'NGS' | 'ddPCR';
}

/**
 * Editing efficiency response
 */
export interface EfficiencyResponse {
  /** Editing efficiency (0-100%) */
  efficiency: number;

  /** 95% confidence interval */
  confidenceInterval: [number, number];

  /** Quality assessment */
  quality: 'high' | 'medium' | 'low';

  /** Sample size adequacy */
  adequateSampleSize: boolean;

  /** Recommendations */
  recommendation: string;

  /** Statistical details */
  statistics?: {
    standardError: number;
    pValue?: number;
    significance?: string;
  };
}

/**
 * Detailed editing outcomes
 */
export interface EditingOutcomes {
  /** Total reads analyzed */
  totalReads: number;

  /** Wild-type reads */
  wildType: number;

  /** Percentage wild-type */
  wildTypePercent: number;

  /** Edited reads */
  edited: number;

  /** Percentage edited */
  editedPercent: number;

  /** Indel spectrum */
  indelSpectrum: IndelAllele[];

  /** HDR events (if applicable) */
  hdrEvents?: number;

  /** HDR percentage */
  hdrPercent?: number;

  /** Most common edit */
  topEdit: IndelAllele;
}

/**
 * Indel allele
 */
export interface IndelAllele {
  /** Allele sequence */
  sequence: string;

  /** Edit type */
  type: 'insertion' | 'deletion' | 'substitution' | 'complex';

  /** Size (bp) */
  size: number;

  /** Position */
  position: number;

  /** Read count */
  count: number;

  /** Percentage */
  percentage: number;

  /** In-frame? */
  inFrame?: boolean;
}

// ============================================================================
// Validation Results
// ============================================================================

/**
 * Protocol validation request
 */
export interface ProtocolValidation {
  /** Protocol to validate */
  protocol: CRISPRProtocol;

  /** Check ethical compliance */
  checkEthics?: boolean;

  /** Check regulatory compliance */
  checkRegulatory?: boolean;

  /** Perform off-target analysis */
  performOffTargetAnalysis?: boolean;
}

/**
 * Protocol validation result
 */
export interface ValidationResult {
  /** Is protocol valid? */
  isValid: boolean;

  /** Validation errors (blocking) */
  errors: string[];

  /** Validation warnings (non-blocking) */
  warnings: string[];

  /** Safety assessment */
  safety: {
    offTargetRisk: 'low' | 'medium' | 'high';
    overallRisk: 'low' | 'medium' | 'high';
    ethicsCompliant: boolean;
    regulatoryCompliant: boolean;
  };

  /** Quality scores */
  quality: {
    gRNAQuality: number;      // 0-100
    specificityScore: number;  // 0-100
    protocolCompleteness: number; // 0-100
  };

  /** Recommendations */
  recommendations: string[];

  /** Timestamp */
  validated: Date;
}

// ============================================================================
// Simulation and Analysis
// ============================================================================

/**
 * CRISPR editing simulation
 */
export interface EditingSimulation {
  /** Simulation ID */
  id: string;

  /** Input protocol */
  protocol: CRISPRProtocol;

  /** Predicted outcomes */
  outcomes: PredictedOutcomes;

  /** Off-target predictions */
  offTargets: OffTargetSite[];

  /** Success probability (0-1) */
  successProbability: number;

  /** Estimated timeline */
  timeline: {
    design: string;
    preparation: string;
    execution: string;
    validation: string;
    total: string;
  };

  /** Cost estimate (optional) */
  costEstimate?: {
    reagents: number;
    labor: number;
    equipment: number;
    total: number;
    currency: string;
  };
}

/**
 * Predicted editing outcomes
 */
export interface PredictedOutcomes {
  /** Predicted NHEJ efficiency (0-100%) */
  nhejEfficiency: number;

  /** Predicted HDR efficiency (0-100%) */
  hdrEfficiency?: number;

  /** Predicted base editing efficiency (0-100%) */
  baseEditEfficiency?: number;

  /** Predicted indel spectrum */
  predictedIndels: {
    insertions: number;
    deletions: number;
    avgSize: number;
  };

  /** On-target score (0-1) */
  onTargetScore: number;

  /** Specificity score (0-100) */
  specificityScore: number;
}

// ============================================================================
// Physical Constants and Parameters
// ============================================================================

/**
 * CRISPR-related constants
 */
export const CRISPR_CONSTANTS = {
  /** Optimal GC content range */
  OPTIMAL_GC_MIN: 40,
  OPTIMAL_GC_MAX: 60,

  /** Maximum poly-T length */
  MAX_POLY_T: 4,

  /** Minimum on-target score */
  MIN_ON_TARGET_SCORE: 0.4,

  /** Minimum specificity score */
  MIN_SPECIFICITY_SCORE: 50,

  /** Maximum recommended off-targets */
  MAX_OFF_TARGETS: 5,

  /** Base editor editing window */
  BE_WINDOW_START: 4,
  BE_WINDOW_END: 8,

  /** Prime editor PBS length */
  PE_PBS_LENGTH: 13,

  /** Typical editing efficiencies */
  TYPICAL_NHEJ: 60,
  TYPICAL_HDR: 10,
  TYPICAL_BASE_EDIT: 40,
  TYPICAL_PRIME_EDIT: 25,
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
 * WIA-BIO-014 error codes
 */
export enum CRISPRErrorCode {
  NO_PAM_FOUND = 'C001',
  HIGH_OFF_TARGET_RISK = 'C002',
  LOW_PREDICTED_EFFICIENCY = 'C003',
  INVALID_SEQUENCE = 'C004',
  DELIVERY_INCOMPATIBLE = 'C005',
  ETHICS_VIOLATION = 'C006',
  INVALID_PARAMETERS = 'C007',
  GENOME_NOT_FOUND = 'C008',
  OFF_TARGET_ANALYSIS_FAILED = 'C009',
  VALIDATION_FAILED = 'C010',
}

/**
 * CRISPR protocol error
 */
export class CRISPRProtocolError extends Error {
  constructor(
    public code: CRISPRErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'CRISPRProtocolError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  CasSystemType,
  PAMType,
  EditType,
  Organism,

  // Guide RNA
  GuideRNARequest,
  GuideRNA,
  GenomicCoordinates,

  // Off-targets
  OffTargetRequest,
  OffTargetSite,
  OffTargetAnnotation,

  // Protocol
  CRISPRProtocol,
  TargetInfo,
  CasSystem,
  DeliveryMethod,
  DeliveryParameters,
  EditingStrategy,
  RepairTemplate,
  PrimeEditingRNA,
  ValidationMethod,
  SafetyChecks,
  ExperimentalParameters,

  // Efficiency
  EfficiencyRequest,
  EfficiencyResponse,
  EditingOutcomes,
  IndelAllele,

  // Validation
  ProtocolValidation,
  ValidationResult,

  // Simulation
  EditingSimulation,
  PredictedOutcomes,
};

export { CRISPR_CONSTANTS, CRISPRErrorCode, CRISPRProtocolError };
