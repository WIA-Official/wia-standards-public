/**
 * WIA-BIO-020: Regenerative Medicine - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Regenerative Medicine Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Tissue types supported by regenerative medicine protocols
 */
export type TissueType =
  | 'cardiac'
  | 'neural'
  | 'bone'
  | 'cartilage'
  | 'skin'
  | 'muscle'
  | 'liver'
  | 'kidney'
  | 'vascular';

/**
 * Stem cell types
 */
export type StemCellType =
  | 'ESC'   // Embryonic Stem Cells
  | 'iPSC'  // Induced Pluripotent Stem Cells
  | 'MSC'   // Mesenchymal Stem Cells
  | 'HSC'   // Hematopoietic Stem Cells
  | 'NSC'   // Neural Stem Cells
  | 'CSC';  // Cardiac Stem Cells

/**
 * Growth factor types
 */
export type GrowthFactorType =
  | 'VEGF'  // Vascular Endothelial Growth Factor
  | 'FGF'   // Fibroblast Growth Factor
  | 'TGF-β' // Transforming Growth Factor Beta
  | 'BMP'   // Bone Morphogenetic Protein
  | 'PDGF'  // Platelet-Derived Growth Factor
  | 'IGF'   // Insulin-like Growth Factor
  | 'EGF'   // Epidermal Growth Factor
  | 'BDNF'  // Brain-Derived Neurotrophic Factor
  | 'NGF'   // Nerve Growth Factor
  | 'HGF';  // Hepatocyte Growth Factor

/**
 * Scaffold material types
 */
export type ScaffoldMaterial =
  | 'collagen'
  | 'gelatin'
  | 'chitosan'
  | 'hyaluronic-acid'
  | 'fibrin'
  | 'alginate'
  | 'PLA'    // Polylactic Acid
  | 'PLGA'   // Poly(lactic-co-glycolic acid)
  | 'PCL'    // Polycaprolactone
  | 'PEG'    // Polyethylene Glycol
  | 'hydroxyapatite'
  | 'beta-TCP'; // Beta-Tricalcium Phosphate

// ============================================================================
// Cell Characterization
// ============================================================================

/**
 * Cell marker expression data
 */
export interface CellMarkers {
  /** Marker name */
  name: string;

  /** Expression level (0-1) */
  expression: number;

  /** Is this marker expected to be positive? */
  expectedPositive: boolean;
}

/**
 * Cell culture characterization
 */
export interface CellCharacterization {
  /** Cell type */
  cellType: StemCellType | string;

  /** Passage number */
  passage: number;

  /** Cell markers and their expression levels */
  markers: CellMarkers[];

  /** Viability (0-1) */
  viability: number;

  /** Karyotype (e.g., "46,XY") */
  karyotype?: string;

  /** Mycoplasma test result */
  mycoplasma: 'negative' | 'positive' | 'pending';

  /** Population doubling time in hours */
  doublingTime?: number;

  /** Sterility test result */
  sterility?: 'pass' | 'fail' | 'pending';

  /** Endotoxin level (EU/ml) */
  endotoxin?: number;

  /** Date of characterization */
  testDate: Date;
}

/**
 * Cell differentiation status
 */
export interface DifferentiationStatus {
  /** Target lineage */
  targetLineage: string;

  /** Differentiation efficiency (0-1) */
  efficiency: number;

  /** Key differentiation markers */
  markers: CellMarkers[];

  /** Days in differentiation */
  daysInCulture: number;

  /** Functional assay results */
  functionalAssays?: Record<string, number>;
}

// ============================================================================
// Regeneration Calculations
// ============================================================================

/**
 * Regeneration rate calculation parameters
 */
export interface RegenerationRequest {
  /** Target tissue type */
  tissueType: TissueType;

  /** Cell density (cells/ml) */
  cellDensity: number;

  /** Time frame for regeneration (days) */
  timeFrame: number;

  /** Growth factors to be used */
  growthFactors: GrowthFactorType[];

  /** Initial tissue volume (cm³) */
  initialVolume?: number;

  /** Target tissue volume (cm³) */
  targetVolume?: number;

  /** Patient age (affects regeneration rate) */
  patientAge?: number;
}

/**
 * Regeneration rate calculation result
 */
export interface RegenerationResponse {
  /** Regeneration rate (cells/day) */
  rate: number;

  /** Expected recovery percentage (0-100) */
  recoveryPercentage: number;

  /** Estimated time to complete regeneration (days) */
  timeToComplete: number;

  /** Feasibility assessment */
  feasibility: 'high' | 'medium' | 'low' | 'infeasible';

  /** Required cell dose */
  requiredCellDose: number;

  /** Confidence level (0-1) */
  confidence: number;

  /** Recommendations */
  recommendations: string[];

  /** Expected tissue integration score (0-1) */
  integrationScore?: number;
}

// ============================================================================
// Cell Survival Assessment
// ============================================================================

/**
 * Cell survival assessment parameters
 */
export interface SurvivalRequest {
  /** Cell type being assessed */
  cellType: StemCellType | string;

  /** Number of viable cells */
  viableCells: number;

  /** Total number of cells */
  totalCells: number;

  /** Culture conditions description */
  cultureConditions: string;

  /** Temperature (°C) */
  temperature?: number;

  /** CO₂ level (%) */
  co2Level?: number;

  /** Culture duration (hours) */
  cultureDuration?: number;
}

/**
 * Cell survival assessment result
 */
export interface SurvivalResponse {
  /** Survival rate (0-100%) */
  survivalRate: number;

  /** Viability classification */
  viability: 'excellent' | 'good' | 'fair' | 'poor';

  /** Recommendations for improvement */
  recommendations: string[];

  /** Expected survival after cryopreservation */
  cryoSurvival?: number;

  /** Quality grade */
  qualityGrade: 'A' | 'B' | 'C' | 'D' | 'F';
}

// ============================================================================
// Scaffold Design
// ============================================================================

/**
 * Mechanical requirements for scaffold
 */
export interface MechanicalRequirements {
  /** Young's modulus (MPa or GPa) */
  youngModulus?: number;

  /** Tensile strength (MPa) */
  tensileStrength?: number;

  /** Compressive strength (MPa) */
  compressiveStrength?: number;

  /** Elastic modulus (MPa) */
  elasticModulus?: number;
}

/**
 * Scaffold design request parameters
 */
export interface ScaffoldRequest {
  /** Target tissue type */
  tissueType: TissueType;

  /** Scaffold material */
  material: ScaffoldMaterial;

  /** Desired porosity (0-1) */
  porosity: number;

  /** Scaffold volume (cm³) */
  size: number;

  /** Mechanical requirements */
  mechanicalRequirements?: MechanicalRequirements;

  /** Desired pore size (μm) */
  poreSize?: number;

  /** Degradation time (months) */
  degradationTime?: number;

  /** Cell seeding density (cells/cm³) */
  cellSeeding?: number;
}

/**
 * Scaffold dimensions
 */
export interface ScaffoldDimensions {
  /** Width (mm) */
  width: number;

  /** Height (mm) */
  height: number;

  /** Depth (mm) */
  depth: number;

  /** Volume (cm³) */
  volume: number;

  /** Surface area (cm²) */
  surfaceArea: number;
}

/**
 * Mechanical properties of scaffold
 */
export interface MechanicalProperties {
  /** Young's modulus (MPa) */
  youngModulus: number;

  /** Tensile strength (MPa) */
  tensileStrength: number;

  /** Compressive strength (MPa) */
  compressiveStrength: number;

  /** Matches native tissue? */
  matchesNative: boolean;
}

/**
 * Scaffold design response
 */
export interface ScaffoldResponse {
  /** Scaffold design specifications */
  design: {
    /** Material used */
    material: ScaffoldMaterial;

    /** Dimensions */
    dimensions: ScaffoldDimensions;

    /** Pore size (μm) */
    poreSize: number;

    /** Actual porosity (0-1) */
    porosity: number;

    /** Degradation time (months) */
    degradationTime: number;

    /** Recommended cell seeding density */
    cellSeeding: number;
  };

  /** Mechanical properties */
  mechanical: MechanicalProperties;

  /** Biocompatibility rating */
  biocompatibility: 'excellent' | 'good' | 'acceptable' | 'poor';

  /** Fabrication method recommended */
  fabricationMethod: '3D-printing' | 'electrospinning' | 'freeze-drying' | 'molding';

  /** Cost estimate (USD) */
  costEstimate?: number;

  /** Manufacturing time (days) */
  manufacturingTime?: number;
}

// ============================================================================
// Growth Factor Optimization
// ============================================================================

/**
 * Growth factor delivery parameters
 */
export interface GrowthFactorRequest {
  /** Growth factor type */
  type: GrowthFactorType;

  /** Target concentration (ng/ml) */
  targetConcentration: number;

  /** Delivery duration (days) */
  duration: number;

  /** Delivery method */
  deliveryMethod: 'bolus' | 'scaffold' | 'microsphere' | 'gene-delivery';

  /** Tissue volume (cm³) */
  tissueVolume?: number;

  /** Combination with other factors */
  combination?: GrowthFactorType[];
}

/**
 * Growth factor delivery response
 */
export interface GrowthFactorResponse {
  /** Recommended total dose (μg) */
  totalDose: number;

  /** Dosing schedule */
  schedule: {
    frequency: string;
    dose: number;
    unit: string;
  };

  /** Release kinetics */
  releaseKinetics: {
    half_life: number; // hours
    peakConcentration: number;
    sustainedRelease: boolean;
  };

  /** Expected efficacy (0-1) */
  efficacy: number;

  /** Synergistic effects */
  synergyBonus?: number;

  /** Cost estimate (USD) */
  costEstimate?: number;
}

// ============================================================================
// Clinical Applications
// ============================================================================

/**
 * Patient information for regenerative therapy
 */
export interface PatientInfo {
  /** Patient ID */
  patientId: string;

  /** Age */
  age: number;

  /** Gender */
  gender: 'male' | 'female' | 'other';

  /** Weight (kg) */
  weight: number;

  /** Medical condition */
  condition: string;

  /** Comorbidities */
  comorbidities?: string[];

  /** HLA type (for allogeneic therapy) */
  hlaType?: string;

  /** Immunosuppression status */
  immunosuppressed?: boolean;
}

/**
 * Treatment protocol
 */
export interface TreatmentProtocol {
  /** Protocol ID */
  protocolId: string;

  /** Tissue/organ target */
  target: TissueType;

  /** Cell source */
  cellSource: {
    type: StemCellType;
    autologous: boolean;
    dose: number; // total cells
  };

  /** Scaffold details */
  scaffold?: ScaffoldResponse;

  /** Growth factors */
  growthFactors?: GrowthFactorResponse[];

  /** Delivery method */
  deliveryMethod: 'injection' | 'surgical-implant' | 'catheter' | 'topical';

  /** Treatment schedule */
  schedule: {
    sessions: number;
    interval: number; // days between sessions
    followUp: number[]; // days for follow-up visits
  };

  /** Immunosuppression required */
  immunosuppression?: {
    required: boolean;
    drugs?: string[];
    duration?: number; // days
  };
}

/**
 * Treatment outcome
 */
export interface TreatmentOutcome {
  /** Patient ID */
  patientId: string;

  /** Treatment date */
  treatmentDate: Date;

  /** Baseline measurements */
  baseline: Record<string, number>;

  /** Post-treatment measurements */
  postTreatment: Record<string, number>;

  /** Improvement percentage */
  improvementPercentage: number;

  /** Adverse events */
  adverseEvents?: string[];

  /** Success classification */
  success: 'complete' | 'partial' | 'minimal' | 'none';

  /** Follow-up duration (days) */
  followUpDuration: number;
}

// ============================================================================
// Quality Control
// ============================================================================

/**
 * Quality control test result
 */
export interface QualityControlTest {
  /** Test name */
  testName: string;

  /** Test result */
  result: string | number | boolean;

  /** Acceptance criteria */
  acceptanceCriteria: string;

  /** Pass/Fail status */
  status: 'pass' | 'fail' | 'pending';

  /** Test date */
  testDate: Date;

  /** Technician ID */
  technicianId?: string;
}

/**
 * Batch release testing
 */
export interface BatchRelease {
  /** Batch ID */
  batchId: string;

  /** Manufacturing date */
  manufacturingDate: Date;

  /** Expiration date */
  expirationDate: Date;

  /** Quality control tests */
  qcTests: QualityControlTest[];

  /** Overall release status */
  releaseStatus: 'approved' | 'rejected' | 'pending';

  /** GMP compliant */
  gmpCompliant: boolean;

  /** Certificate of Analysis */
  coa?: string;
}

// ============================================================================
// Regulatory Compliance
// ============================================================================

/**
 * Regulatory status
 */
export interface RegulatoryStatus {
  /** FDA RMAT designation */
  fdaRMAT?: boolean;

  /** EMA ATMP approval */
  emaATMP?: boolean;

  /** Clinical trial phase */
  clinicalPhase?: 'preclinical' | 'phase-1' | 'phase-2' | 'phase-3' | 'approved';

  /** IND number */
  indNumber?: string;

  /** Trial registration */
  trialRegistration?: string; // ClinicalTrials.gov ID

  /** Ethics approval */
  ethicsApproval?: boolean;

  /** IRB approval date */
  irbApprovalDate?: Date;
}

// ============================================================================
// Safety Monitoring
// ============================================================================

/**
 * Adverse event
 */
export interface AdverseEvent {
  /** Event ID */
  eventId: string;

  /** Patient ID */
  patientId: string;

  /** Event description */
  description: string;

  /** Severity grade (1-5) */
  grade: 1 | 2 | 3 | 4 | 5;

  /** Serious adverse event */
  serious: boolean;

  /** Related to treatment */
  relatedToTreatment: 'definitely' | 'probably' | 'possibly' | 'unlikely' | 'unrelated';

  /** Onset date */
  onsetDate: Date;

  /** Resolution date */
  resolutionDate?: Date;

  /** Action taken */
  actionTaken?: string;
}

/**
 * Safety monitoring
 */
export interface SafetyMonitoring {
  /** Patient ID */
  patientId: string;

  /** Monitoring date */
  date: Date;

  /** Vital signs */
  vitalSigns: {
    temperature: number;   // °C
    heartRate: number;     // bpm
    bloodPressure: string; // e.g., "120/80"
    respiratoryRate: number; // breaths/min
  };

  /** Laboratory values */
  labValues?: Record<string, number>;

  /** Adverse events */
  adverseEvents?: AdverseEvent[];

  /** Imaging findings */
  imaging?: string;

  /** Overall status */
  status: 'stable' | 'improving' | 'worsening' | 'critical';
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Biological constants for regenerative medicine
 */
export const BIO_CONSTANTS = {
  /** Minimum cell viability for clinical use */
  MIN_VIABILITY: 0.7,

  /** Minimum cell purity */
  MIN_PURITY: 0.9,

  /** Maximum endotoxin level (EU/kg) */
  MAX_ENDOTOXIN: 5,

  /** GMP clean room class */
  GMP_CLASS: 'ISO-5',

  /** Typical MSC markers (positive) */
  MSC_POSITIVE_MARKERS: ['CD73', 'CD90', 'CD105'],

  /** Typical MSC markers (negative) */
  MSC_NEGATIVE_MARKERS: ['CD45', 'CD34', 'CD14', 'CD19', 'HLA-DR'],

  /** Pluripotency markers */
  PLURIPOTENCY_MARKERS: ['OCT4', 'NANOG', 'SSEA4', 'TRA-1-60'],

  /** Scaffold porosity range */
  SCAFFOLD_POROSITY: { min: 0.5, max: 0.95 },

  /** Growth factor concentration ranges (ng/ml) */
  GROWTH_FACTOR_RANGE: {
    VEGF: { min: 10, max: 50 },
    FGF: { min: 5, max: 20 },
    'TGF-β': { min: 1, max: 10 },
    BMP: { min: 50, max: 200 },
    PDGF: { min: 10, max: 100 },
    IGF: { min: 50, max: 200 },
    EGF: { min: 5, max: 50 },
    BDNF: { min: 10, max: 100 },
    NGF: { min: 50, max: 200 },
    HGF: { min: 10, max: 100 },
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

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-020 error codes
 */
export enum BioErrorCode {
  CELL_CONTAMINATION = 'B001',
  LOW_VIABILITY = 'B002',
  INSUFFICIENT_MARKERS = 'B003',
  SCAFFOLD_DEGRADATION = 'B004',
  GROWTH_FACTOR_INSTABILITY = 'B005',
  IMMUNE_REJECTION_RISK = 'B006',
  INVALID_PARAMETERS = 'B007',
  QC_FAILURE = 'B008',
  REGULATORY_NONCOMPLIANCE = 'B009',
  SAFETY_CONCERN = 'B010',
}

/**
 * Regenerative medicine error
 */
export class RegenerativeMedicineError extends Error {
  constructor(
    public code: BioErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'RegenerativeMedicineError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  TissueType,
  StemCellType,
  GrowthFactorType,
  ScaffoldMaterial,

  // Cell characterization
  CellMarkers,
  CellCharacterization,
  DifferentiationStatus,

  // Regeneration
  RegenerationRequest,
  RegenerationResponse,

  // Cell survival
  SurvivalRequest,
  SurvivalResponse,

  // Scaffold
  MechanicalRequirements,
  ScaffoldRequest,
  ScaffoldDimensions,
  MechanicalProperties,
  ScaffoldResponse,

  // Growth factors
  GrowthFactorRequest,
  GrowthFactorResponse,

  // Clinical
  PatientInfo,
  TreatmentProtocol,
  TreatmentOutcome,

  // Quality control
  QualityControlTest,
  BatchRelease,

  // Regulatory
  RegulatoryStatus,

  // Safety
  AdverseEvent,
  SafetyMonitoring,
};

export { BIO_CONSTANTS, BioErrorCode, RegenerativeMedicineError };
