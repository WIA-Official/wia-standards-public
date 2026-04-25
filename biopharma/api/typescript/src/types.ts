/**
 * WIA-BIO-016: Biopharma - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biopharmaceutical Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Biopharmaceutical Types
// ============================================================================

/**
 * Biopharmaceutical drug classes
 */
export type DrugClass =
  | 'monoclonal-antibody'
  | 'antibody-fragment'
  | 'fusion-protein'
  | 'cytokine'
  | 'enzyme'
  | 'vaccine'
  | 'antibody-drug-conjugate'
  | 'peptide'
  | 'recombinant-protein';

/**
 * Antibody isotypes
 */
export type AntibodyIsotype = 'IgG1' | 'IgG2' | 'IgG3' | 'IgG4' | 'IgM' | 'IgA' | 'IgE';

/**
 * Antibody formats
 */
export type AntibodyFormat =
  | 'full-igg'
  | 'fab'
  | 'scfv'
  | 'diabody'
  | 'bite'
  | 'nanobody'
  | 'dart';

/**
 * Vaccine types
 */
export type VaccineType =
  | 'mrna'
  | 'viral-vector'
  | 'protein-subunit'
  | 'inactivated'
  | 'live-attenuated'
  | 'toxoid'
  | 'conjugate';

/**
 * Expression systems
 */
export type ExpressionSystem =
  | 'e-coli'
  | 'yeast'
  | 'cho'
  | 'hek293'
  | 'ns0'
  | 'sp2-0'
  | 'insect'
  | 'plant';

// ============================================================================
// Molecular Characterization
// ============================================================================

/**
 * Molecular weight information
 */
export interface MolecularWeight {
  /** Theoretical molecular weight in Daltons */
  theoretical: number;

  /** Measured molecular weight in Daltons */
  measured: number;

  /** Measurement uncertainty in Daltons */
  uncertainty?: number;

  /** Method used for measurement */
  method: 'mass-spec' | 'sec-mals' | 'sds-page' | 'calculated';

  /** Mass accuracy in ppm */
  accuracy?: number;
}

/**
 * Glycan structure
 */
export interface Glycan {
  /** Glycan type */
  type: 'n-glycan' | 'o-glycan';

  /** Glycan composition (e.g., "G0F", "G1F", "G2F") */
  composition: string;

  /** Relative abundance (0-1) */
  abundance: number;

  /** Number of sialic acids */
  sialicAcids?: number;

  /** Fucosylation status */
  fucosylated: boolean;

  /** High mannose content */
  highMannose: boolean;
}

/**
 * Post-translational modifications
 */
export interface PTM {
  /** Type of modification */
  type: 'glycosylation' | 'phosphorylation' | 'acetylation' | 'methylation' | 'oxidation';

  /** Position in sequence */
  position: number;

  /** Modified residue */
  residue: string;

  /** Modification details */
  details?: string;

  /** Occupancy (0-1) */
  occupancy?: number;
}

/**
 * Protein characterization data
 */
export interface ProteinCharacterization {
  /** Amino acid sequence */
  sequence: string;

  /** Molecular weight */
  molecularWeight: MolecularWeight;

  /** Isoelectric point */
  isoelectricPoint: number;

  /** Extinction coefficient (M⁻¹cm⁻¹) */
  extinctionCoefficient: number;

  /** Glycosylation sites */
  glycosylationSites?: number[];

  /** Glycan structures */
  glycans?: Glycan[];

  /** Post-translational modifications */
  ptms?: PTM[];

  /** Disulfide bonds */
  disulfideBonds?: Array<[number, number]>;
}

// ============================================================================
// Binding Affinity and Kinetics
// ============================================================================

/**
 * Binding affinity parameters
 */
export interface BindingAffinity {
  /** Antibody concentration in M */
  antibodyConc: number;

  /** Antigen concentration in M */
  antigenConc: number;

  /** Complex concentration in M */
  complexConc: number;

  /** Temperature in °C */
  temperature: number;

  /** pH */
  pH?: number;

  /** Buffer composition */
  buffer?: string;
}

/**
 * Binding affinity result
 */
export interface BindingAffinityResult {
  /** Dissociation constant (Kd) in M */
  kd: number;

  /** Free antibody concentration in M */
  freeAntibody: number;

  /** Free antigen concentration in M */
  freeAntigen: number;

  /** Binding strength classification */
  bindingStrength: 'ultra-high' | 'very-high' | 'high' | 'moderate' | 'low' | 'very-low';

  /** Fraction bound (0-1) */
  fractionBound: number;

  /** Suitable for therapeutic use */
  therapeuticSuitability: boolean;
}

/**
 * Kinetic parameters
 */
export interface KineticParameters {
  /** Association rate constant (M⁻¹s⁻¹) */
  kon: number;

  /** Dissociation rate constant (s⁻¹) */
  koff: number;

  /** Calculated Kd from kinetics (M) */
  kd: number;

  /** Half-life of complex (seconds) */
  complexHalfLife: number;

  /** Method used */
  method: 'spr' | 'bli' | 'ifa' | 'elisa' | 'kinexA';

  /** Temperature in °C */
  temperature: number;
}

// ============================================================================
// Pharmacokinetics and Pharmacodynamics
// ============================================================================

/**
 * Pharmacokinetic parameters
 */
export interface PKParameters {
  /** Dose in mg */
  dose: number;

  /** Bioavailability (0-1) */
  bioavailability: number;

  /** Clearance in L/h */
  clearance: number;

  /** Volume of distribution in L */
  volumeOfDistribution: number;

  /** Route of administration */
  route: 'iv' | 'sc' | 'im' | 'oral' | 'inhalation';

  /** Patient weight in kg */
  patientWeight?: number;
}

/**
 * Pharmacokinetic result
 */
export interface PKResult {
  /** Area under curve (mg·h/L) */
  auc: number;

  /** Maximum concentration (mg/L) */
  cmax?: number;

  /** Time to maximum concentration (hours) */
  tmax?: number;

  /** Half-life (hours) */
  halfLife: number;

  /** Initial concentration (mg/L) */
  c0: number;

  /** Elimination rate constant (h⁻¹) */
  ke: number;

  /** Recommended dosing interval (hours) */
  dosingInterval: number;
}

/**
 * Pharmacodynamic parameters
 */
export interface PDParameters {
  /** Drug concentration (mg/L) */
  concentration: number;

  /** Maximum effect */
  emax: number;

  /** EC50 (mg/L) */
  ec50: number;

  /** Hill coefficient */
  hillCoefficient?: number;

  /** Baseline effect */
  baseline?: number;
}

/**
 * Pharmacodynamic result
 */
export interface PDResult {
  /** Predicted effect */
  effect: number;

  /** Receptor occupancy (0-1) */
  receptorOccupancy: number;

  /** Effect percentage (0-100) */
  effectPercentage: number;

  /** Therapeutic range */
  therapeuticRange: {
    min: number;
    max: number;
  };

  /** Within therapeutic range */
  withinRange: boolean;
}

// ============================================================================
// Immunogenicity
// ============================================================================

/**
 * Anti-drug antibody (ADA) data
 */
export interface ADAData {
  /** ADA incidence rate (0-1) */
  incidenceRate: number;

  /** ADA titer (if positive) */
  titer?: number;

  /** Neutralizing antibody status */
  neutralizing: boolean;

  /** Time to ADA development (days) */
  timeToADA?: number;

  /** Clinical impact */
  clinicalImpact: 'none' | 'minimal' | 'moderate' | 'severe';
}

/**
 * Immunogenicity assessment parameters
 */
export interface ImmunogenicityParams {
  /** ADA incidence rate (0-1) */
  adaRate: number;

  /** Clinical impact severity (1-10) */
  severity: number;

  /** Treatment duration factor */
  durationFactor: number;

  /** Patient tolerance factor */
  toleranceFactor: number;

  /** Prior immunogenicity data available */
  priorData?: boolean;
}

/**
 * Immunogenicity risk assessment
 */
export interface ImmunogenicityRisk {
  /** Immunogenicity risk score (0-100) */
  riskScore: number;

  /** Risk level */
  riskLevel: 'low' | 'moderate' | 'high' | 'very-high';

  /** Predicted ADA incidence (0-1) */
  predictedADA: number;

  /** Recommended monitoring frequency */
  monitoringFrequency: 'standard' | 'frequent' | 'intensive';

  /** Mitigation strategies */
  mitigationStrategies: string[];

  /** Clinical recommendations */
  recommendations: string[];
}

// ============================================================================
// Stability and Quality
// ============================================================================

/**
 * Stability test conditions
 */
export interface StabilityConditions {
  /** Temperature in °C */
  temperature: number;

  /** Relative humidity (%) */
  humidity?: number;

  /** Duration in months */
  duration: number;

  /** Storage condition type */
  type: 'long-term' | 'accelerated' | 'stress' | 'real-time';

  /** Light exposure */
  lightExposed?: boolean;
}

/**
 * Stability test result
 */
export interface StabilityResult {
  /** Test conditions */
  conditions: StabilityConditions;

  /** Remaining potency (%) */
  potency: number;

  /** Monomer content (%) */
  monomerContent: number;

  /** Aggregate content (%) */
  aggregates: number;

  /** pH */
  pH: number;

  /** Appearance */
  appearance: 'clear' | 'slightly-opalescent' | 'opalescent' | 'turbid' | 'precipitate';

  /** Passes specification */
  passesSpec: boolean;

  /** Predicted shelf life (months) */
  predictedShelfLife?: number;
}

/**
 * Quality attributes
 */
export interface QualityAttributes {
  /** Purity by SEC (%) */
  purity: number;

  /** Monomer content (%) */
  monomer: number;

  /** Aggregates (%) */
  aggregates: number;

  /** Fragments (%) */
  fragments: number;

  /** Charge variants - Main peak (%) */
  mainPeak: number;

  /** Acidic variants (%) */
  acidicVariants: number;

  /** Basic variants (%) */
  basicVariants: number;

  /** Potency (% of reference) */
  potency: number;

  /** Endotoxin (EU/mg) */
  endotoxin: number;

  /** Host cell proteins (ppm) */
  hcp?: number;

  /** DNA (pg/dose) */
  residualDNA?: number;
}

// ============================================================================
// Biosimilar Development
// ============================================================================

/**
 * Biosimilarity comparison
 */
export interface BiosimilarComparison {
  /** Attribute being compared */
  attribute: string;

  /** Reference product value */
  referenceValue: number;

  /** Biosimilar value */
  biosimilarValue: number;

  /** Similarity percentage (0-100) */
  similarity: number;

  /** Tier classification */
  tier: 1 | 2 | 3;

  /** Within acceptance criteria */
  acceptable: boolean;
}

/**
 * Biosimilar assessment
 */
export interface BiosimilarAssessment {
  /** Overall similarity score (0-100) */
  overallSimilarity: number;

  /** Individual attribute comparisons */
  comparisons: BiosimilarComparison[];

  /** Analytical similarity achieved */
  analyticalSimilarity: boolean;

  /** PK similarity demonstrated */
  pkSimilarity?: boolean;

  /** Clinical similarity demonstrated */
  clinicalSimilarity?: boolean;

  /** Biosimilarity conclusion */
  conclusion: 'similar' | 'not-similar' | 'further-testing-required';

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Drug Candidate Validation
// ============================================================================

/**
 * Drug candidate parameters
 */
export interface DrugCandidate {
  /** Drug type */
  drugType: DrugClass;

  /** Target name */
  target: string;

  /** Target affinity (Kd in M) */
  targetAffinity: number;

  /** Molecular weight */
  molecularWeight: number;

  /** Stability score (0-1) */
  stability: number;

  /** Immunogenicity risk */
  immunogenicityRisk: 'low' | 'moderate' | 'high';

  /** Expression system */
  expressionSystem: ExpressionSystem;

  /** Manufacturing complexity */
  manufacturingComplexity: 'low' | 'medium' | 'high';

  /** Development stage */
  developmentStage: 'discovery' | 'preclinical' | 'phase1' | 'phase2' | 'phase3' | 'approved';
}

/**
 * Drug candidate validation result
 */
export interface DrugValidationResult {
  /** Is candidate valid */
  isValid: boolean;

  /** Validation errors */
  errors: string[];

  /** Validation warnings */
  warnings: string[];

  /** Development feasibility */
  feasibility: 'high' | 'medium' | 'low';

  /** Estimated development timeline (years) */
  estimatedTimeline: number;

  /** Estimated development cost (millions USD) */
  estimatedCost: number;

  /** Key development risks */
  risks: string[];

  /** Recommendations */
  recommendations: string[];

  /** Success probability (0-1) */
  successProbability: number;
}

// ============================================================================
// Manufacturing and Process
// ============================================================================

/**
 * Cell culture parameters
 */
export interface CellCultureParams {
  /** Cell line */
  cellLine: string;

  /** Expression system */
  expressionSystem: ExpressionSystem;

  /** Volumetric productivity (mg/L) */
  productivity: number;

  /** Culture duration (days) */
  cultureDuration: number;

  /** Viability (%) */
  viability: number;

  /** Cell density (cells/mL) */
  cellDensity: number;
}

/**
 * Purification process
 */
export interface PurificationProcess {
  /** Steps in purification */
  steps: Array<{
    name: string;
    type: 'affinity' | 'ion-exchange' | 'hydrophobic' | 'size-exclusion' | 'ultrafiltration';
    yield: number; // 0-1
    purity: number; // 0-1
  }>;

  /** Overall yield (0-1) */
  overallYield: number;

  /** Final purity (0-1) */
  finalPurity: number;

  /** Process duration (hours) */
  processDuration: number;
}

// ============================================================================
// Regulatory and Compliance
// ============================================================================

/**
 * Regulatory pathway
 */
export type RegulatoryPathway =
  | 'fda-bla'
  | 'ema-maa'
  | 'pmda-japan'
  | 'nmpa-china'
  | 'biosimilar-351k'
  | 'accelerated'
  | 'breakthrough';

/**
 * Clinical trial phase
 */
export type ClinicalPhase = 'preclinical' | 'phase1' | 'phase2' | 'phase3' | 'phase4';

/**
 * Regulatory submission
 */
export interface RegulatorySubmission {
  /** Regulatory pathway */
  pathway: RegulatoryPathway;

  /** Submission type */
  type: 'ind' | 'bla' | 'maa' | 'amendment' | 'annual-report';

  /** Target indication */
  indication: string;

  /** Clinical phase */
  phase?: ClinicalPhase;

  /** Estimated submission date */
  estimatedSubmission?: Date;

  /** Estimated approval date */
  estimatedApproval?: Date;

  /** Orphan drug designation */
  orphanDrug?: boolean;

  /** Fast track designation */
  fastTrack?: boolean;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Biopharmaceutical constants
 */
export const BIOPHARMA_CONSTANTS = {
  /** Avogadro's number */
  AVOGADRO: 6.022e23,

  /** Gas constant (cal/mol·K) */
  GAS_CONSTANT: 1.987,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.381e-23,

  /** Standard temperature (K) */
  STANDARD_TEMP: 298.15,

  /** Typical IgG molecular weight (Da) */
  IGG_MW: 150000,

  /** Typical IgG half-life (days) */
  IGG_HALF_LIFE: 21,

  /** FcRn binding Kd at pH 6.0 (M) */
  FCRN_KD: 200e-9,

  /** Typical plasma volume (L) */
  PLASMA_VOLUME: 3.5,

  /** Minimum therapeutic Kd (M) */
  MIN_THERAPEUTIC_KD: 10e-9,

  /** Maximum aggregate content (%) */
  MAX_AGGREGATES: 5,

  /** Minimum monomer content (%) */
  MIN_MONOMER: 95,

  /** Maximum endotoxin (EU/mg) */
  MAX_ENDOTOXIN: 1,

  /** Minimum stability (months at 2-8°C) */
  MIN_SHELF_LIFE: 24,
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
 * Concentration units
 */
export type ConcentrationUnit = 'M' | 'mM' | 'μM' | 'nM' | 'pM' | 'mg/mL' | 'μg/mL';

/**
 * Time units
 */
export type TimeUnit = 'seconds' | 'minutes' | 'hours' | 'days' | 'weeks' | 'months' | 'years';

/**
 * Temperature units
 */
export type TemperatureUnit = 'celsius' | 'fahrenheit' | 'kelvin';

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-016 error codes
 */
export enum BiopharmaErrorCode {
  INVALID_AFFINITY = 'BIO001',
  INSUFFICIENT_STABILITY = 'BIO002',
  HIGH_IMMUNOGENICITY = 'BIO003',
  POOR_PK_PROPERTIES = 'BIO004',
  QUALITY_FAILURE = 'BIO005',
  REGULATORY_NONCOMPLIANCE = 'BIO006',
  INVALID_PARAMETERS = 'BIO007',
  MANUFACTURING_FAILURE = 'BIO008',
  BIOSIMILARITY_FAILURE = 'BIO009',
  SAFETY_CONCERN = 'BIO010',
}

/**
 * Biopharma error
 */
export class BiopharmaError extends Error {
  constructor(
    public code: BiopharmaErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BiopharmaError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  DrugClass,
  AntibodyIsotype,
  AntibodyFormat,
  VaccineType,
  ExpressionSystem,

  // Characterization
  MolecularWeight,
  Glycan,
  PTM,
  ProteinCharacterization,

  // Binding
  BindingAffinity,
  BindingAffinityResult,
  KineticParameters,

  // PK/PD
  PKParameters,
  PKResult,
  PDParameters,
  PDResult,

  // Immunogenicity
  ADAData,
  ImmunogenicityParams,
  ImmunogenicityRisk,

  // Quality
  StabilityConditions,
  StabilityResult,
  QualityAttributes,

  // Biosimilar
  BiosimilarComparison,
  BiosimilarAssessment,

  // Validation
  DrugCandidate,
  DrugValidationResult,

  // Manufacturing
  CellCultureParams,
  PurificationProcess,

  // Regulatory
  RegulatoryPathway,
  ClinicalPhase,
  RegulatorySubmission,

  // Utility
  ConcentrationUnit,
  TimeUnit,
  TemperatureUnit,
};

export { BIOPHARMA_CONSTANTS, BiopharmaErrorCode, BiopharmaError };
