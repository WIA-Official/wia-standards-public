/**
 * WIA-BIO-009: Drug Discovery - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Drug Discovery Types
// ============================================================================

/**
 * Chemical compound representation
 */
export interface Compound {
  /** Unique compound identifier */
  id: string;

  /** Compound name */
  name?: string;

  /** SMILES notation */
  smiles: string;

  /** InChI identifier */
  inchi?: string;

  /** Molecular formula */
  formula?: string;

  /** Molecular weight in Daltons */
  molecularWeight?: number;

  /** 2D structure (MOL format) */
  structure2D?: string;

  /** 3D structure (PDB/SDF format) */
  structure3D?: string;

  /** Compound source */
  source?: 'natural' | 'synthetic' | 'semi-synthetic' | 'biologic';

  /** Chemical class */
  chemicalClass?: string;
}

/**
 * Drug target (protein, RNA, DNA, etc.)
 */
export interface DrugTarget {
  /** Target identifier */
  id: string;

  /** Target name */
  name: string;

  /** Target type */
  type: 'enzyme' | 'gpcr' | 'ion-channel' | 'nuclear-receptor' | 'ppi' | 'other';

  /** Gene symbol */
  gene?: string;

  /** UniProt ID */
  uniprotId?: string;

  /** Species */
  species: string;

  /** Disease association */
  diseaseAssociation?: {
    disease: string;
    evidence: 'genetic' | 'biochemical' | 'clinical' | 'literature';
    score: number; // 0-1
  }[];

  /** Druggability score (0-1) */
  druggability?: number;

  /** 3D structure availability */
  structure?: {
    pdbId?: string;
    resolution?: number; // Å
    method?: 'x-ray' | 'cryo-em' | 'nmr' | 'model';
  };
}

/**
 * Screening assay configuration
 */
export interface Assay {
  /** Assay identifier */
  id: string;

  /** Assay name */
  name: string;

  /** Assay type */
  type: 'binding' | 'enzymatic' | 'cellular' | 'phenotypic' | 'biochemical';

  /** Target information */
  target: DrugTarget;

  /** Assay format */
  format?: 'HTS' | 'fragment' | 'DEL' | 'virtual' | 'manual';

  /** Detection method */
  detection?: 'fluorescence' | 'luminescence' | 'absorbance' | 'radioactive' | 'mass-spec';

  /** Quality metrics */
  quality?: {
    zFactor?: number;
    signalToBackground?: number;
    cv?: number; // %
  };

  /** Protocol details */
  protocol?: string;
}

// ============================================================================
// Screening & Hit Discovery
// ============================================================================

/**
 * Compound screening request
 */
export interface ScreeningRequest {
  /** Target to screen against */
  target: DrugTarget;

  /** Compounds to screen */
  compounds: Compound[];

  /** Assay configuration */
  assay: Assay;

  /** Screening mode */
  mode?: 'single-point' | 'dose-response' | 'titration';

  /** Activity threshold */
  threshold?: {
    ic50?: number;
    ec50?: number;
    ki?: number;
    inhibition?: number; // %
  };

  /** Concentration range (M) */
  concentrationRange?: {
    min: number;
    max: number;
    points?: number;
  };
}

/**
 * Compound hit from screening
 */
export interface CompoundHit {
  /** Compound information */
  compound: Compound;

  /** IC50 value (M) */
  ic50?: number;

  /** EC50 value (M) */
  ec50?: number;

  /** Ki value (M) */
  ki?: number;

  /** % Inhibition at single concentration */
  inhibitionPercent?: number;

  /** Hill slope */
  hillSlope?: number;

  /** R-squared of fit */
  rSquared?: number;

  /** Number of replicates */
  replicates: number;

  /** Coefficient of variation (%) */
  cv?: number;

  /** Confirmation status */
  confirmed?: boolean;

  /** PAINS flag */
  isPAINS?: boolean;

  /** Counter-screen results */
  counterScreen?: {
    cytotoxicity?: number; // CC50
    aggregation?: boolean;
    interference?: boolean;
  };
}

/**
 * Screening result
 */
export interface ScreeningResult {
  /** Assay used */
  assay: Assay;

  /** Hit compounds */
  hits: CompoundHit[];

  /** Statistics */
  statistics: {
    totalCompounds: number;
    hitsCount: number;
    hitRate: number; // %
    zFactor?: number;
    signalToBackground?: number;
  };

  /** Screening date */
  date: Date;

  /** Quality assessment */
  quality: 'excellent' | 'good' | 'acceptable' | 'poor';
}

// ============================================================================
// Lead Optimization
// ============================================================================

/**
 * Lead optimization request
 */
export interface LeadOptimizationRequest {
  /** Starting lead compound */
  leadCompound: Compound;

  /** Optimization objectives */
  objectives: ('potency' | 'selectivity' | 'solubility' | 'permeability' | 'safety')[];

  /** Property constraints */
  constraints?: {
    mw?: { min?: number; max?: number };
    logP?: { min?: number; max?: number };
    tpsa?: { min?: number; max?: number };
    hbd?: { max?: number };
    hba?: { max?: number };
  };

  /** Target profile */
  targetProfile?: {
    ic50?: number;
    selectivity?: number;
    bioavailability?: number;
  };

  /** Medicinal chemistry strategy */
  strategy?: 'scaffold-hop' | 'bioisoster' | 'conformational-restriction' | 'fragment-growth';
}

/**
 * Optimized compound suggestion
 */
export interface OptimizedCompound {
  /** New compound structure */
  compound: Compound;

  /** Parent compound */
  parent: Compound;

  /** Modifications made */
  modifications: {
    type: 'scaffold-hop' | 'bioisosteric' | 'substituent' | 'conformational';
    description: string;
    position?: string;
  }[];

  /** Predicted improvements */
  predictions: {
    potency?: { current: number; predicted: number; confidence: number };
    selectivity?: { current: number; predicted: number; confidence: number };
    solubility?: { current: number; predicted: number; confidence: number };
    permeability?: { current: number; predicted: number; confidence: number };
  };

  /** SAR analysis */
  sar?: {
    deltaIC50: number; // log units
    similarityScore: number; // 0-1
    novelty: number; // 0-1
  };

  /** Lipinski compliance */
  lipinskiCompliant?: boolean;
}

// ============================================================================
// ADMET Prediction
// ============================================================================

/**
 * ADMET prediction request
 */
export interface ADMETRequest {
  /** Compound to analyze */
  compound?: Compound;

  /** SMILES string (alternative to compound) */
  smiles?: string;

  /** Models to run */
  models: ('solubility' | 'permeability' | 'metabolism' | 'hERG' | 'cyp450' | 'toxicity')[];

  /** Include experimental validation */
  includeExperimental?: boolean;
}

/**
 * Absorption properties
 */
export interface AbsorptionProfile {
  /** Aqueous solubility (LogS) */
  solubility: number;

  /** Solubility classification */
  solubilityClass: 'high' | 'good' | 'moderate' | 'low' | 'very-low';

  /** Caco-2 permeability (nm/s) */
  permeability: number;

  /** Permeability classification */
  permeabilityClass: 'high' | 'medium' | 'low';

  /** Oral bioavailability (%) */
  bioavailability: number;

  /** Human intestinal absorption (%) */
  hia?: number;
}

/**
 * Distribution properties
 */
export interface DistributionProfile {
  /** Volume of distribution (L/kg) */
  volumeOfDistribution: number;

  /** Plasma protein binding (%) */
  plasmaProteinBinding: number;

  /** Fraction unbound */
  fractionUnbound: number;

  /** Blood-brain barrier penetration */
  bbbPenetration: 'high' | 'medium' | 'low';

  /** CNS drug-likeness */
  cnsScore?: number; // -3 to +2
}

/**
 * Metabolism properties
 */
export interface MetabolismProfile {
  /** Metabolic stability (t½ in liver microsomes, min) */
  stability: number;

  /** Intrinsic clearance (mL/min/kg) */
  intrinsicClearance: number;

  /** Half-life (hours) */
  halfLife: number;

  /** CYP450 inhibition (IC50 values in μM) */
  cyp450Inhibition: {
    CYP1A2?: number;
    CYP2C9?: number;
    CYP2C19?: number;
    CYP2D6?: number;
    CYP3A4?: number;
  };

  /** CYP450 substrate */
  cyp450Substrate?: {
    CYP1A2?: boolean;
    CYP2C9?: boolean;
    CYP2C19?: boolean;
    CYP2D6?: boolean;
    CYP3A4?: boolean;
  };

  /** Major metabolites */
  metabolites?: {
    structure: string; // SMILES
    pathway: string;
    abundance: number; // %
  }[];
}

/**
 * Excretion properties
 */
export interface ExcretionProfile {
  /** Total clearance (mL/min/kg) */
  clearance: number;

  /** Renal clearance (%) */
  renalClearance: number;

  /** Hepatic clearance (%) */
  hepaticClearance: number;

  /** Excretion half-life (hours) */
  excretionHalfLife: number;
}

/**
 * Toxicity assessment
 */
export interface ToxicityProfile {
  /** hERG inhibition (IC50 in μM) */
  hERG_IC50: number;

  /** hERG risk assessment */
  hERG_Risk: 'low' | 'medium' | 'high';

  /** Ames mutagenicity */
  ames: 'positive' | 'negative' | 'inconclusive';

  /** Hepatotoxicity risk */
  hepatotoxicityRisk: 'low' | 'medium' | 'high';

  /** Cardiotoxicity risk */
  cardiotoxicityRisk?: 'low' | 'medium' | 'high';

  /** LD50 (mg/kg) */
  ld50?: number;

  /** Therapeutic index */
  therapeuticIndex?: number;

  /** PAINS alerts */
  painsAlerts?: string[];
}

/**
 * Complete ADMET profile
 */
export interface ADMETProfile {
  /** Compound analyzed */
  compound: Compound;

  /** Absorption properties */
  absorption: AbsorptionProfile;

  /** Distribution properties */
  distribution: DistributionProfile;

  /** Metabolism properties */
  metabolism: MetabolismProfile;

  /** Excretion properties */
  excretion?: ExcretionProfile;

  /** Toxicity assessment */
  toxicity: ToxicityProfile;

  /** Overall drug-likeness score (0-1) */
  drugLikenessScore: number;

  /** Lipinski rule compliance */
  lipinskiCompliance: {
    compliant: boolean;
    violations: string[];
    rules: {
      mw: boolean;
      logP: boolean;
      hbd: boolean;
      hba: boolean;
    };
  };

  /** Prediction confidence */
  confidence?: {
    overall: number; // 0-1
    adme: number;
    toxicity: number;
  };
}

// ============================================================================
// Pharmacokinetics & Pharmacodynamics
// ============================================================================

/**
 * Pharmacokinetic parameters
 */
export interface PharmacokineticsData {
  /** Area under curve (h·μg/mL) */
  auc: number;

  /** Maximum concentration (μg/mL) */
  cmax: number;

  /** Time to maximum concentration (h) */
  tmax: number;

  /** Clearance (L/h/kg) */
  clearance: number;

  /** Volume of distribution (L/kg) */
  volumeOfDistribution: number;

  /** Half-life (h) */
  halfLife: number;

  /** Bioavailability (%) */
  bioavailability?: number;

  /** Dose (mg/kg) */
  dose: number;

  /** Route of administration */
  route: 'oral' | 'iv' | 'ip' | 'sc' | 'im' | 'topical';

  /** Species */
  species: string;
}

/**
 * Pharmacodynamic response
 */
export interface PharmacodynamicsData {
  /** Effect magnitude */
  effect: number;

  /** EC50 (μg/mL) */
  ec50?: number;

  /** Maximum effect */
  emax?: number;

  /** Effect duration (h) */
  duration?: number;

  /** Biomarker response */
  biomarkers?: Record<string, number>;
}

// ============================================================================
// Clinical Trials
// ============================================================================

/**
 * Clinical trial phase
 */
export type ClinicalPhase = 'Phase-I' | 'Phase-II' | 'Phase-III' | 'Phase-IV';

/**
 * Clinical trial data
 */
export interface ClinicalTrialData {
  /** Trial identifier (NCT number) */
  nctId: string;

  /** Trial title */
  title: string;

  /** Phase */
  phase: ClinicalPhase;

  /** Study type */
  studyType: 'interventional' | 'observational' | 'expanded-access';

  /** Indication/disease */
  indication: string;

  /** Number of patients */
  enrollment: number;

  /** Primary endpoints */
  primaryEndpoints: string[];

  /** Secondary endpoints */
  secondaryEndpoints?: string[];

  /** Status */
  status: 'recruiting' | 'active' | 'completed' | 'terminated' | 'suspended';

  /** Results */
  results?: {
    efficacy?: {
      endpoint: string;
      result: number;
      pValue?: number;
      confidenceInterval?: [number, number];
    }[];
    safety?: {
      adverseEvents: {
        term: string;
        grade: 1 | 2 | 3 | 4 | 5;
        frequency: number; // %
      }[];
    };
  };
}

/**
 * Safety report
 */
export interface SafetyReport {
  /** Report identifier */
  id: string;

  /** Compound */
  compound: Compound;

  /** Study phase */
  phase: ClinicalPhase;

  /** Adverse events */
  adverseEvents: {
    term: string;
    grade: 1 | 2 | 3 | 4 | 5;
    serious: boolean;
    frequency: number; // %
    attribution: 'unrelated' | 'unlikely' | 'possible' | 'probable' | 'definite';
  }[];

  /** Dose-limiting toxicities */
  dlt?: {
    description: string;
    doseLevel: number;
    frequency: number; // %
  }[];

  /** Maximum tolerated dose (mg/kg or mg/m²) */
  mtd?: number;

  /** No observed adverse effect level */
  noael?: number;

  /** Overall safety assessment */
  assessment: 'safe' | 'acceptable' | 'concerning' | 'unacceptable';
}

// ============================================================================
// Regulatory Submission
// ============================================================================

/**
 * Regulatory submission type
 */
export type SubmissionType = 'IND' | 'NDA' | 'BLA' | 'ANDA' | 'MAA' | 'CTA';

/**
 * Regulatory agency
 */
export type RegulatoryAgency = 'FDA' | 'EMA' | 'PMDA' | 'NMPA' | 'TGA' | 'Health-Canada';

/**
 * Regulatory submission
 */
export interface RegulatorySubmission {
  /** Submission identifier */
  id: string;

  /** Submission type */
  type: SubmissionType;

  /** Regulatory agency */
  agency: RegulatoryAgency;

  /** Compound/drug */
  compound: Compound;

  /** Indication */
  indication: string;

  /** Status */
  status: 'draft' | 'submitted' | 'under-review' | 'approved' | 'rejected' | 'withdrawn';

  /** Submission date */
  submissionDate?: Date;

  /** Approval date */
  approvalDate?: Date;

  /** Application number */
  applicationNumber?: string;

  /** Documents */
  documents?: {
    module: 1 | 2 | 3 | 4 | 5;
    section: string;
    title: string;
    version: string;
  }[];

  /** Review timeline (days) */
  reviewTimeline?: {
    standard: number;
    priority: number;
  };
}

/**
 * Drug dossier (complete package)
 */
export interface DrugDossier {
  /** Compound */
  compound: Compound;

  /** Target */
  target: DrugTarget;

  /** Discovery data */
  discovery: {
    screening: ScreeningResult[];
    optimization: OptimizedCompound[];
  };

  /** ADMET profile */
  admet: ADMETProfile;

  /** Pharmacokinetics */
  pharmacokinetics: PharmacokineticsData[];

  /** Efficacy studies */
  efficacy?: {
    model: string;
    species: string;
    result: string;
    statisticalSignificance: boolean;
  }[];

  /** Toxicology */
  toxicology?: SafetyReport[];

  /** Clinical trials */
  clinicalTrials?: ClinicalTrialData[];

  /** Regulatory submissions */
  submissions?: RegulatorySubmission[];

  /** Manufacturing */
  manufacturing?: {
    method: string;
    purity: number; // %
    stability: string;
  };
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Drug discovery constants and thresholds
 */
export const DRUG_DISCOVERY_CONSTANTS = {
  /** Lipinski's Rule of Five */
  LIPINSKI: {
    MW_MAX: 500,
    LOGP_MAX: 5,
    HBD_MAX: 5,
    HBA_MAX: 10,
  },

  /** Extended drug-likeness */
  EXTENDED_RULES: {
    PSA_MIN: 40,
    PSA_MAX: 140,
    ROTATABLE_BONDS_MAX: 10,
    AROMATIC_RINGS_MAX: 3,
    SP3_FRACTION_MIN: 0.25,
  },

  /** Assay quality thresholds */
  ASSAY_QUALITY: {
    Z_FACTOR_MIN: 0.5,
    SIGNAL_TO_BACKGROUND_MIN: 5,
    CV_MAX: 20, // %
  },

  /** Hit criteria */
  HIT_CRITERIA: {
    IC50_MAX: 10e-6, // 10 μM
    HILL_SLOPE_MIN: 0.5,
    HILL_SLOPE_MAX: 2.0,
    R_SQUARED_MIN: 0.9,
    CV_MAX: 20, // %
  },

  /** Lead criteria */
  LEAD_CRITERIA: {
    IC50_MAX: 100e-9, // 100 nM
    SELECTIVITY_MIN: 100, // fold
    BIOAVAILABILITY_MIN: 30, // %
  },

  /** ADMET thresholds */
  ADMET_THRESHOLDS: {
    SOLUBILITY_MIN: -4, // LogS
    PERMEABILITY_MIN: 50, // nm/s
    HERG_SAFETY_MARGIN: 30, // fold
    CYP450_IC50_MIN: 10e-6, // 10 μM
    BBB_LOGPS_MAX: 5, // for CNS drugs
  },

  /** Toxicity limits */
  TOXICITY: {
    LD50_MIN: 500, // mg/kg
    THERAPEUTIC_INDEX_MIN: 3,
    NOAEL_SAFETY_MARGIN: 10, // fold over therapeutic dose
  },

  /** Clinical trial */
  CLINICAL: {
    PHASE_I_ENROLLMENT_MIN: 20,
    PHASE_II_ENROLLMENT_MIN: 100,
    PHASE_III_ENROLLMENT_MIN: 300,
    STATISTICAL_POWER: 0.8,
    SIGNIFICANCE_LEVEL: 0.05,
  },
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
 * Dose units
 */
export type DoseUnit = 'mg/kg' | 'mg/m²' | 'mg' | 'μg' | 'IU';

/**
 * Time units
 */
export type TimeUnit = 's' | 'min' | 'h' | 'day' | 'week' | 'month';

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-009 error codes
 */
export enum DrugDiscoveryErrorCode {
  INVALID_SMILES = 'B001',
  TARGET_NOT_FOUND = 'B002',
  ASSAY_QUALITY_LOW = 'B003',
  ADMET_PREDICTION_FAILED = 'B004',
  TOXICITY_ALERT = 'B005',
  REGULATORY_NON_COMPLIANCE = 'B006',
  INSUFFICIENT_DATA = 'B007',
  LIPINSKI_VIOLATION = 'B008',
  PAINS_DETECTED = 'B009',
  SELECTIVITY_LOW = 'B010',
}

/**
 * Drug discovery error
 */
export class DrugDiscoveryError extends Error {
  constructor(
    public code: DrugDiscoveryErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DrugDiscoveryError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Compound,
  DrugTarget,
  Assay,

  // Screening
  ScreeningRequest,
  CompoundHit,
  ScreeningResult,

  // Lead optimization
  LeadOptimizationRequest,
  OptimizedCompound,

  // ADMET
  ADMETRequest,
  ADMETProfile,
  AbsorptionProfile,
  DistributionProfile,
  MetabolismProfile,
  ExcretionProfile,
  ToxicityProfile,

  // PK/PD
  PharmacokineticsData,
  PharmacodynamicsData,

  // Clinical
  ClinicalPhase,
  ClinicalTrialData,
  SafetyReport,

  // Regulatory
  SubmissionType,
  RegulatoryAgency,
  RegulatorySubmission,
  DrugDossier,

  // Utility
  ConcentrationUnit,
  DoseUnit,
  TimeUnit,
};

export { DRUG_DISCOVERY_CONSTANTS, DrugDiscoveryErrorCode, DrugDiscoveryError };
