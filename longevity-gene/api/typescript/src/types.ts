/**
 * WIA-AUG-017: Longevity Gene Editing - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Longevity Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Target Gene Types
// ============================================================================

/**
 * Longevity-associated target genes
 */
export enum TargetGene {
  // Telomere Maintenance
  TERT = 'TERT',           // Telomerase Reverse Transcriptase
  TERC = 'TERC',           // Telomerase RNA Component
  RTEL1 = 'RTEL1',         // Regulator of Telomere Elongation Helicase 1

  // Sirtuins (NAD+ dependent deacetylases)
  SIRT1 = 'SIRT1',         // Sirtuin 1
  SIRT3 = 'SIRT3',         // Sirtuin 3 (mitochondrial)
  SIRT6 = 'SIRT6',         // Sirtuin 6 (DNA repair, inflammation)

  // Metabolic Regulation
  AMPK = 'AMPK',           // AMP-activated protein kinase
  MTOR = 'MTOR',           // Mechanistic target of rapamycin
  FOXO3 = 'FOXO3',         // Forkhead box O3

  // Growth Factors
  GDF11 = 'GDF11',         // Growth Differentiation Factor 11
  KLOTHO = 'KLOTHO',       // Alpha-Klotho

  // DNA Repair
  BRCA1 = 'BRCA1',         // Breast Cancer 1
  TP53 = 'TP53',           // Tumor Protein p53
  ATM = 'ATM',             // Ataxia Telangiectasia Mutated

  // Lipid Metabolism
  APOE = 'APOE',           // Apolipoprotein E
  CETP = 'CETP',           // Cholesteryl Ester Transfer Protein
  PCSK9 = 'PCSK9',         // Proprotein Convertase Subtilisin/Kexin Type 9

  // Inflammation
  IL6 = 'IL6',             // Interleukin 6
  TNF = 'TNF',             // Tumor Necrosis Factor
  NLRP3 = 'NLRP3',         // NOD-like receptor family pyrin domain containing 3

  // Mitochondrial
  TFAM = 'TFAM',           // Mitochondrial Transcription Factor A
  PGC1A = 'PGC1A',         // Peroxisome proliferator-activated receptor gamma coactivator 1-alpha
  NRF1 = 'NRF1',           // Nuclear Respiratory Factor 1
  NRF2 = 'NRF2',           // Nuclear factor erythroid 2-related factor 2
}

/**
 * Gene function categories
 */
export enum GeneFunctionCategory {
  TELOMERE_MAINTENANCE = 'telomere_maintenance',
  METABOLIC_REGULATION = 'metabolic_regulation',
  DNA_REPAIR = 'dna_repair',
  STRESS_RESPONSE = 'stress_response',
  INFLAMMATION = 'inflammation',
  MITOCHONDRIAL = 'mitochondrial',
  LIPID_METABOLISM = 'lipid_metabolism',
  GROWTH_FACTOR = 'growth_factor',
}

/**
 * Evidence strength for longevity association
 */
export type EvidenceLevel = 'Strong' | 'Moderate' | 'Emerging' | 'Theoretical';

/**
 * Target gene information
 */
export interface TargetGeneInfo {
  /** Gene symbol */
  gene: TargetGene;

  /** Full gene name */
  fullName: string;

  /** Primary function category */
  category: GeneFunctionCategory;

  /** Evidence strength */
  evidenceLevel: EvidenceLevel;

  /** Chromosome location */
  chromosome: string;

  /** Known longevity association (years of lifespan extension in model organisms) */
  longevityEffect: number;

  /** Associated diseases prevented */
  diseasesPrevented: string[];

  /** Editing complexity (1-10, 10 = most complex) */
  editingComplexity: number;

  /** Safety profile (1-10, 10 = safest) */
  safetyProfile: number;
}

// ============================================================================
// Editing Technology Types
// ============================================================================

/**
 * Gene editing technologies
 */
export enum EditingTechnology {
  CRISPR_CAS9 = 'crispr_cas9',
  BASE_EDITING = 'base_editing',
  PRIME_EDITING = 'prime_editing',
  EPIGENETIC_MODIFICATION = 'epigenetic_modification',
  RNA_INTERFERENCE = 'rna_interference',
  GENE_THERAPY = 'gene_therapy',
}

/**
 * Editing technology characteristics
 */
export interface EditingTechProfile {
  /** Technology type */
  technology: EditingTechnology;

  /** Precision level (0-1, 1 = perfect precision) */
  precision: number;

  /** Editing efficiency (0-1, 1 = 100% efficiency) */
  efficiency: number;

  /** Off-target risk (0-1, 0 = no risk) */
  offTargetRisk: number;

  /** Reversibility (0-1, 1 = fully reversible) */
  reversibility: number;

  /** Delivery complexity (1-10) */
  deliveryComplexity: number;

  /** Cost factor (1-10, 10 = most expensive) */
  costFactor: number;
}

// ============================================================================
// Delivery Mechanism Types
// ============================================================================

/**
 * Delivery methods for gene editing
 */
export enum DeliveryMethod {
  AAV_VECTOR = 'aav_vector',                   // Adeno-Associated Virus
  LENTIVIRUS = 'lentivirus',                   // Lentiviral Vector
  LIPID_NANOPARTICLE = 'lipid_nanoparticle',   // LNP
  ELECTROPORATION = 'electroporation',         // Electrical pulse
  DIRECT_INJECTION = 'direct_injection',       // Direct tissue injection
  EXOSOME = 'exosome',                         // Exosome-mediated
  MICRONEEDLE = 'microneedle',                 // Microneedle array
}

/**
 * Tissue targets for delivery
 */
export enum TissueTarget {
  LIVER = 'liver',
  MUSCLE = 'muscle',
  ADIPOSE = 'adipose',
  BRAIN = 'brain',
  HEART = 'heart',
  BONE_MARROW = 'bone_marrow',
  BLOOD = 'blood',
  SKIN = 'skin',
  SYSTEMIC = 'systemic',
}

/**
 * Delivery protocol
 */
export interface DeliveryProtocol {
  /** Delivery method */
  method: DeliveryMethod;

  /** Target tissues */
  tissues: TissueTarget[];

  /** Dose (in genome copies or concentration) */
  dose: number;

  /** Dose unit */
  doseUnit: string;

  /** Administration route */
  route: 'intravenous' | 'intramuscular' | 'subcutaneous' | 'local' | 'oral';

  /** Number of doses */
  numberOfDoses: number;

  /** Interval between doses (days) */
  doseInterval: number;
}

// ============================================================================
// Aging Biomarker Types
// ============================================================================

/**
 * Epigenetic clock types
 */
export enum EpigeneticClock {
  HORVATH = 'horvath',           // Multi-tissue age predictor
  HANNUM = 'hannum',             // Blood-based clock
  PHENOAGE = 'phenoage',         // Phenotypic age
  GRIMAGE = 'grimage',           // Mortality risk predictor
  DUNEDIN_PACE = 'dunedin_pace', // Pace of aging
}

/**
 * Epigenetic age measurement
 */
export interface EpigeneticAge {
  /** Clock type */
  clock: EpigeneticClock;

  /** Epigenetic age (years) */
  age: number;

  /** Chronological age for comparison */
  chronologicalAge: number;

  /** Age acceleration (positive = older, negative = younger) */
  ageAcceleration: number;

  /** Measurement confidence (0-1) */
  confidence: number;
}

/**
 * Telomere measurement
 */
export interface TelomereMeasurement {
  /** Mean telomere length (kilobases) */
  meanLength: number;

  /** Shortest 20th percentile length */
  shortestPercentile: number;

  /** Cell type measured */
  cellType: 'leukocyte' | 'lymphocyte' | 'tissue_specific';

  /** Measurement method */
  method: 'qPCR' | 'flow_FISH' | 'southern_blot';
}

/**
 * Senescent cell markers
 */
export interface SenescentCellMarkers {
  /** Percentage of p16+ cells */
  p16Positive: number;

  /** Percentage of p21+ cells */
  p21Positive: number;

  /** SA-β-gal activity (0-1) */
  betaGalActivity: number;

  /** SASP factor levels */
  saspFactors: {
    il6: number;
    il8: number;
    mmp3: number;
  };
}

/**
 * Inflammation markers
 */
export interface InflammationMarkers {
  /** C-Reactive Protein (mg/L) */
  crp: number;

  /** Interleukin-6 (pg/mL) */
  il6: number;

  /** Tumor Necrosis Factor alpha (pg/mL) */
  tnfAlpha: number;

  /** Fibrinogen (mg/dL) */
  fibrinogen: number;
}

/**
 * Metabolic biomarkers
 */
export interface MetabolicBiomarkers {
  /** Fasting glucose (mg/dL) */
  glucoseFasting: number;

  /** HbA1c (%) */
  hba1c: number;

  /** Total cholesterol (mg/dL) */
  cholesterolTotal: number;

  /** LDL cholesterol (mg/dL) */
  ldl: number;

  /** HDL cholesterol (mg/dL) */
  hdl: number;

  /** Triglycerides (mg/dL) */
  triglycerides: number;

  /** Insulin resistance (HOMA-IR) */
  homaIR: number;
}

/**
 * Comprehensive aging biomarker assessment
 */
export interface AgingBiomarkers {
  /** Epigenetic age measurements */
  epigeneticAges: EpigeneticAge[];

  /** Telomere measurements */
  telomeres: TelomereMeasurement;

  /** Senescent cell markers */
  senescentCells: SenescentCellMarkers;

  /** Inflammation markers */
  inflammation: InflammationMarkers;

  /** Metabolic biomarkers */
  metabolic: MetabolicBiomarkers;

  /** Functional capacity metrics */
  functional: {
    vo2Max: number;            // mL/kg/min
    gripStrength: number;      // kg
    walkingSpeed: number;      // m/s
  };
}

/**
 * Biological age assessment result
 */
export interface BiologicalAgeAssessment {
  /** Chronological age */
  chronologicalAge: number;

  /** Calculated biological age */
  biologicalAge: number;

  /** Age acceleration (years) */
  ageAcceleration: number;

  /** Contributing biomarkers */
  biomarkers: AgingBiomarkers;

  /** Assessment confidence (0-1) */
  confidence: number;

  /** Risk stratification */
  riskLevel: 'low' | 'moderate' | 'high' | 'very_high';
}

// ============================================================================
// Gene Editing Protocol Types
// ============================================================================

/**
 * Off-target prediction
 */
export interface OffTargetSite {
  /** Chromosome */
  chromosome: string;

  /** Position */
  position: number;

  /** Sequence */
  sequence: string;

  /** Mismatch count */
  mismatches: number;

  /** Predicted score (0-1, 1 = highest likelihood) */
  score: number;

  /** Gene affected (if any) */
  geneAffected?: string;

  /** Functional consequence */
  consequence: 'silent' | 'missense' | 'nonsense' | 'regulatory';
}

/**
 * Guide RNA design
 */
export interface GuideRNA {
  /** Guide RNA sequence (20bp) */
  sequence: string;

  /** PAM sequence */
  pamSequence: string;

  /** On-target score (0-100) */
  onTargetScore: number;

  /** Off-target predictions */
  offTargets: OffTargetSite[];

  /** GC content (%) */
  gcContent: number;

  /** Secondary structure stability */
  structureScore: number;
}

/**
 * Editing protocol design
 */
export interface EditingProtocol {
  /** Protocol identifier */
  id: string;

  /** Target genes */
  targetGenes: TargetGene[];

  /** Editing technology */
  technology: EditingTechnology;

  /** Guide RNAs (if applicable) */
  guideRNAs?: GuideRNA[];

  /** Delivery protocol */
  delivery: DeliveryProtocol;

  /** Pre-screening requirements */
  preScreening: string[];

  /** Safety monitoring plan */
  safetyMonitoring: string[];

  /** Expected outcomes */
  expectedOutcomes: {
    biologicalAgeReduction: number; // years
    healthspanExtension: number;    // years
    successProbability: number;     // 0-1
  };

  /** Estimated cost */
  estimatedCost: number;

  /** Treatment duration */
  durationWeeks: number;
}

// ============================================================================
// Safety and Monitoring Types
// ============================================================================

/**
 * Off-target evaluation result
 */
export interface OffTargetEvaluation {
  /** Evaluation ID */
  id: string;

  /** Evaluation date */
  date: Date;

  /** Method used */
  method: 'wgs' | 'targeted_sequencing' | 'computational' | 'guide_seq';

  /** Off-target sites detected */
  sitesDetected: OffTargetSite[];

  /** Total genome coverage (%) */
  genomeCoverage: number;

  /** Pass/fail status */
  passed: boolean;

  /** Maximum acceptable off-target rate (%) */
  threshold: number;

  /** Actual off-target rate (%) */
  actualRate: number;
}

/**
 * Efficacy monitoring data point
 */
export interface EfficacyDataPoint {
  /** Measurement timestamp */
  timestamp: Date;

  /** Days since treatment */
  daysSinceTreatment: number;

  /** Biological age assessment */
  biologicalAge: BiologicalAgeAssessment;

  /** Gene expression levels */
  geneExpression: Map<TargetGene, number>;

  /** Clinical observations */
  clinical: {
    adverseEvents: string[];
    functionalStatus: 'improved' | 'stable' | 'declined';
    qualityOfLife: number; // 0-100
  };
}

/**
 * Healthspan tracking
 */
export interface HealthspanTracking {
  /** Patient identifier */
  patientId: string;

  /** Treatment start date */
  treatmentStart: Date;

  /** Follow-up duration (years) */
  followUpYears: number;

  /** Efficacy measurements */
  measurements: EfficacyDataPoint[];

  /** Disease-free survival */
  diseaseFreeYears: number;

  /** Quality-adjusted life years (QALYs) */
  qalys: number;

  /** Overall success */
  success: boolean;
}

// ============================================================================
// Risk Assessment Types
// ============================================================================

/**
 * Risk factors
 */
export interface RiskFactors {
  /** Age (years) */
  age: number;

  /** Family history of longevity */
  familyLongevity: 'short' | 'average' | 'long';

  /** Pre-existing conditions */
  conditions: string[];

  /** Current medications */
  medications: string[];

  /** Lifestyle factors */
  lifestyle: {
    smoking: boolean;
    alcoholUse: 'none' | 'moderate' | 'heavy';
    exerciseLevel: 'sedentary' | 'moderate' | 'active';
    diet: 'poor' | 'average' | 'good';
  };

  /** Genetic risk alleles */
  geneticRisks: string[];
}

/**
 * Risk assessment result
 */
export interface RiskAssessment {
  /** Overall risk level */
  riskLevel: 'low' | 'moderate' | 'high' | 'very_high';

  /** Cancer risk increase (%) */
  cancerRisk: number;

  /** Immune response risk */
  immuneRisk: number;

  /** Off-target risk */
  offTargetRisk: number;

  /** Contraindications */
  contraindications: string[];

  /** Recommendations */
  recommendations: string[];

  /** Proceed with treatment */
  approved: boolean;
}

// ============================================================================
// Patient and Protocol Types
// ============================================================================

/**
 * Patient information
 */
export interface PatientInfo {
  /** Patient ID (anonymized) */
  id: string;

  /** Age */
  age: number;

  /** Gender */
  gender: 'male' | 'female' | 'other';

  /** Health status */
  healthStatus: 'excellent' | 'good' | 'fair' | 'poor';

  /** Family history */
  familyHistory: string[];

  /** Genetic profile */
  geneticProfile?: {
    apoeGenotype: string;
    otherRiskAlleles: string[];
  };
}

/**
 * Treatment goals
 */
export enum TreatmentGoal {
  HEALTHSPAN_EXTENSION = 'healthspan',
  DISEASE_PREVENTION = 'disease_prevention',
  METABOLIC_HEALTH = 'metabolic_health',
  COGNITIVE_ENHANCEMENT = 'cognitive_enhancement',
  PHYSICAL_PERFORMANCE = 'physical_performance',
  RECOVERY = 'recovery',
}

/**
 * Risk tolerance
 */
export type RiskTolerance = 'low' | 'moderate' | 'high';

/**
 * Gene selection input
 */
export interface GeneSelectionInput {
  /** Patient information */
  patient: PatientInfo;

  /** Treatment goals */
  goals: TreatmentGoal[];

  /** Risk tolerance */
  riskTolerance: RiskTolerance;

  /** Budget constraints */
  maxCost?: number;
}

/**
 * Gene selection result
 */
export interface GeneSelectionResult {
  /** Recommended target genes */
  recommendedGenes: TargetGene[];

  /** Gene priorities (1 = highest) */
  priorities: Map<TargetGene, number>;

  /** Rationale for each gene */
  rationale: Map<TargetGene, string>;

  /** Expected combined effect */
  expectedEffect: {
    biologicalAgeReduction: number;
    healthspanExtension: number;
    confidence: number;
  };

  /** Alternative options */
  alternatives: TargetGene[][];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Longevity-related constants
 */
export const LONGEVITY_CONSTANTS = {
  /** Biological age thresholds */
  AGE_THRESHOLDS: {
    YOUNG: 30,
    MIDDLE: 50,
    SENIOR: 70,
    ELDERLY: 85,
  },

  /** Biomarker reference ranges */
  BIOMARKER_RANGES: {
    TELOMERE_NORMAL_MIN: 5.5,  // kb
    TELOMERE_NORMAL_MAX: 10.0,
    SENESCENT_CELLS_MAX: 15,   // %
    CRP_NORMAL_MAX: 3.0,       // mg/L
    IL6_NORMAL_MAX: 5.0,       // pg/mL
  },

  /** Editing success thresholds */
  SUCCESS_THRESHOLDS: {
    MIN_BIOLOGICAL_AGE_REDUCTION: 2.0,  // years
    MIN_HEALTHSPAN_EXTENSION: 3.0,      // years
    MAX_OFF_TARGET_RATE: 0.1,           // %
    MIN_EFFICACY_CONFIDENCE: 0.80,      // 80%
  },

  /** Safety limits */
  SAFETY_LIMITS: {
    MAX_TELOMERASE_UPREGULATION: 3.0,   // fold change
    MAX_MTOR_INHIBITION: 0.5,           // 50% reduction
    MAX_SIMULTANEOUS_EDITS: 5,          // number of genes
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Error codes
 */
export enum LongevityErrorCode {
  INVALID_BIOMARKER = 'L001',
  GENE_SELECTION_FAILED = 'L002',
  PROTOCOL_DESIGN_FAILED = 'L003',
  OFF_TARGET_EXCEEDED = 'L004',
  SAFETY_VIOLATION = 'L005',
  EFFICACY_INSUFFICIENT = 'L006',
  CONTRAINDICATION_DETECTED = 'L007',
}

/**
 * Longevity gene editing error
 */
export class LongevityError extends Error {
  constructor(
    public code: LongevityErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'LongevityError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  TargetGeneInfo,
  EditingTechProfile,
  DeliveryProtocol,
  EpigeneticAge,
  TelomereMeasurement,
  SenescentCellMarkers,
  InflammationMarkers,
  MetabolicBiomarkers,
  AgingBiomarkers,
  BiologicalAgeAssessment,
  OffTargetSite,
  GuideRNA,
  EditingProtocol,
  OffTargetEvaluation,
  EfficacyDataPoint,
  HealthspanTracking,
  RiskFactors,
  RiskAssessment,
  PatientInfo,
  GeneSelectionInput,
  GeneSelectionResult,
};

export {
  TargetGene,
  GeneFunctionCategory,
  EditingTechnology,
  DeliveryMethod,
  TissueTarget,
  EpigeneticClock,
  TreatmentGoal,
  LONGEVITY_CONSTANTS,
  LongevityErrorCode,
  LongevityError,
};
