/**
 * WIA-BIO-005: Cellular Therapy - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cell Types
// ============================================================================

/**
 * Cell product types
 */
export type CellType =
  | 'CAR-T'
  | 'CAR-NK'
  | 'TIL'
  | 'MSC'
  | 'iPSC'
  | 'HSC'
  | 'DC'
  | 'Treg'
  | 'Other';

/**
 * Cell source
 */
export type CellSource =
  | 'autologous'
  | 'allogeneic'
  | 'xenogeneic';

/**
 * Manufacturing stage
 */
export type ManufacturingStage =
  | 'collection'
  | 'activation'
  | 'transduction'
  | 'expansion'
  | 'harvest'
  | 'cryopreservation'
  | 'complete'
  | 'failed';

/**
 * Cell product information
 */
export interface CellProduct {
  /** Unique product identifier */
  productId: string;

  /** Type of cell product */
  cellType: CellType;

  /** Autologous or allogeneic */
  source: CellSource;

  /** Patient identifier */
  patientId?: string;

  /** Donor identifier (for allogeneic) */
  donorId?: string;

  /** Manufacturing batch number */
  batchId: string;

  /** Total cell count */
  totalCells: number;

  /** Viable cell count */
  viableCells: number;

  /** Viability percentage */
  viability: number;

  /** Target marker expression (e.g., CAR+, CD3+) */
  targetMarker?: {
    name: string;
    percentage: number;
  };

  /** Manufacturing dates */
  dates: {
    collection: Date;
    started: Date;
    completed?: Date;
  };

  /** Storage information */
  storage?: StorageInfo;
}

/**
 * Storage information
 */
export interface StorageInfo {
  /** Storage location identifier */
  location: string;

  /** Storage temperature in Celsius */
  temperature: number;

  /** Freeze date */
  freezeDate: Date;

  /** Expiration date */
  expirationDate?: Date;

  /** Number of freeze-thaw cycles */
  freezeThawCycles: number;
}

// ============================================================================
// Manufacturing Process
// ============================================================================

/**
 * Manufacturing process parameters
 */
export interface ManufacturingProcess {
  /** Batch identifier */
  batchId: string;

  /** Cell type being manufactured */
  cellType: CellType;

  /** Current stage */
  currentStage: ManufacturingStage;

  /** Start date */
  startDate: Date;

  /** Expected completion date */
  expectedCompletion: Date;

  /** Starting material quality */
  startingMaterial: {
    totalCells: number;
    viability: number;
    cd3Percent?: number;
    cd56Percent?: number;
  };

  /** Process parameters */
  parameters: {
    /** Culture medium */
    medium: string;

    /** Cytokines used */
    cytokines?: string[];

    /** MOI for transduction */
    moi?: number;

    /** Target expansion ratio */
    targetExpansion: number;

    /** Culture duration in days */
    cultureDays: number;
  };

  /** Current metrics */
  currentMetrics: {
    cellCount: number;
    viability: number;
    dayInCulture: number;
    expansionRatio?: number;
  };

  /** Process alerts */
  alerts: Alert[];

  /** Critical process parameters */
  cpp: CriticalProcessParameters;
}

/**
 * Critical Process Parameters
 */
export interface CriticalProcessParameters {
  /** Temperature in Celsius */
  temperature: number;

  /** pH */
  pH: number;

  /** Dissolved oxygen (% saturation) */
  dissolvedOxygen: number;

  /** Cell density (cells/mL) */
  cellDensity: number;

  /** Osmolality (mOsm/kg) */
  osmolality: number;

  /** All within acceptable ranges */
  withinRanges: boolean;
}

/**
 * Process alert
 */
export interface Alert {
  /** Alert ID */
  id: string;

  /** Alert severity */
  severity: 'info' | 'warning' | 'critical';

  /** Alert message */
  message: string;

  /** Timestamp */
  timestamp: Date;

  /** Resolved status */
  resolved: boolean;

  /** Resolution notes */
  resolution?: string;
}

// ============================================================================
// Quality Control
// ============================================================================

/**
 * Quality control metrics
 */
export interface QualityMetrics {
  /** Cell viability percentage */
  viability: number;

  /** Potency percentage */
  potency: number;

  /** Sterility test result */
  sterility: boolean;

  /** Endotoxin level (EU/mL) */
  endotoxin: number;

  /** Mycoplasma test result */
  mycoplasma: boolean;

  /** Identity markers */
  identity: IdentityMarkers;

  /** Cell count */
  cellCount: {
    total: number;
    viable: number;
    targetMarkerPositive?: number;
  };

  /** Appearance */
  appearance: {
    color: string;
    clarity: string;
    particulates: boolean;
  };
}

/**
 * Identity markers for flow cytometry
 */
export interface IdentityMarkers {
  /** Positive markers with percentages */
  positive: {
    [marker: string]: number;
  };

  /** Negative markers with percentages */
  negative: {
    [marker: string]: number;
  };
}

/**
 * Potency assay result
 */
export interface PotencyAssay {
  /** Assay type */
  type: 'cytotoxicity' | 'cytokine-secretion' | 'proliferation' | 'other';

  /** Method used */
  method: string;

  /** E:T ratios tested */
  etRatios?: number[];

  /** Specific lysis percentages */
  specificLysis?: number[];

  /** Cytokine levels (pg/mL) */
  cytokines?: {
    [cytokine: string]: number;
  };

  /** Overall potency score */
  potency: number;

  /** Pass/fail status */
  passed: boolean;

  /** Comments */
  comments?: string;
}

/**
 * Quality assessment result
 */
export interface QualityResult {
  /** Pass all release criteria */
  passRelease: boolean;

  /** Quality grade */
  grade: 'A' | 'B' | 'C' | 'F';

  /** Failed tests */
  failures: string[];

  /** Warnings */
  warnings: string[];

  /** Recommendation */
  recommendation: 'release' | 'conditional-release' | 'reject' | 'retest';

  /** Release criteria checklist */
  criteria: ReleaseCriteria;

  /** Comments */
  comments?: string;
}

/**
 * Release criteria checklist
 */
export interface ReleaseCriteria {
  viability: { required: number; actual: number; pass: boolean };
  identity: { required: string; actual: string; pass: boolean };
  potency: { required: number; actual: number; pass: boolean };
  sterility: { required: boolean; actual: boolean; pass: boolean };
  endotoxin: { required: number; actual: number; pass: boolean };
  mycoplasma: { required: boolean; actual: boolean; pass: boolean };
}

// ============================================================================
// Dosing and Administration
// ============================================================================

/**
 * Dose calculation parameters
 */
export interface DoseParameters {
  /** Cell type */
  cellType: CellType;

  /** Total cell count */
  totalCells: number;

  /** Viability percentage */
  viability: number;

  /** Patient weight in kg */
  patientWeight: number;

  /** Target marker percentage (e.g., CAR+) */
  targetMarkerPercent?: number;

  /** Dosing strategy */
  strategy: 'weight-based' | 'flat-dose' | 'split-dose';
}

/**
 * Dose calculation result
 */
export interface DoseResult {
  /** Dose in cells/kg */
  dose: number;

  /** Total viable cells */
  totalViableCells: number;

  /** Target marker positive cells */
  targetMarkerCells?: number;

  /** Within recommended range */
  withinRange: boolean;

  /** Recommended dose range */
  recommendedDose: {
    minimum: number;
    maximum: number;
    unit: 'cells/kg' | 'total cells';
  };

  /** Dose assessment */
  assessment: 'optimal' | 'acceptable' | 'suboptimal' | 'out-of-range';

  /** Warnings or recommendations */
  notes: string[];
}

/**
 * Patient dose information
 */
export interface PatientDose {
  /** Patient identifier */
  patientId: string;

  /** Product identifier */
  productId: string;

  /** Cell type */
  cellType: CellType;

  /** Calculated dose */
  dose: DoseResult;

  /** Administration route */
  route: AdministrationRoute;

  /** Scheduled infusion date */
  scheduledDate: Date;

  /** Actual infusion date */
  actualDate?: Date;

  /** Pre-medications */
  preMedications?: string[];

  /** Lymphodepletion regimen */
  lymphodepletion?: LymphodepletionRegimen;
}

/**
 * Administration route
 */
export type AdministrationRoute =
  | 'intravenous'
  | 'intra-arterial'
  | 'intrathecal'
  | 'intraperitoneal'
  | 'local-injection';

/**
 * Lymphodepletion chemotherapy regimen
 */
export interface LymphodepletionRegimen {
  /** Regimen name */
  name: string;

  /** Drugs used */
  drugs: {
    name: string;
    dose: string;
    days: number[];
  }[];

  /** Start date */
  startDate: Date;

  /** Completion date */
  completionDate: Date;

  /** Days before CAR-T infusion */
  daysBeforeInfusion: number;
}

// ============================================================================
// Safety Monitoring
// ============================================================================

/**
 * Cytokine Release Syndrome (CRS) assessment
 */
export interface CRSAssessment {
  /** Patient identifier */
  patientId: string;

  /** Assessment date/time */
  timestamp: Date;

  /** Days post-infusion */
  daysPostInfusion: number;

  /** CRS grade (0-4) */
  grade: 0 | 1 | 2 | 3 | 4;

  /** Fever (Celsius) */
  temperature: number;

  /** Hypotension present */
  hypotension: boolean;

  /** Vasopressor requirement */
  vasopressors: 'none' | 'single' | 'multiple';

  /** Hypoxia present */
  hypoxia: boolean;

  /** Oxygen requirement */
  oxygenRequirement: 'none' | 'low-flow' | 'high-flow' | 'ventilator';

  /** Biomarkers */
  biomarkers?: {
    crp?: number;      // mg/dL
    ferritin?: number; // ng/mL
    il6?: number;      // pg/mL
    ifnGamma?: number; // pg/mL
  };

  /** Treatment given */
  treatment?: CRSTreatment;

  /** Notes */
  notes?: string;
}

/**
 * CRS treatment
 */
export interface CRSTreatment {
  /** Tocilizumab doses */
  tocilizumab?: {
    dose: number; // mg/kg
    doses: number;
    lastDose: Date;
  };

  /** Corticosteroids */
  corticosteroids?: {
    drug: string;
    dose: string;
    started: Date;
  };

  /** ICU admission */
  icu: boolean;

  /** Response to treatment */
  response: 'improved' | 'stable' | 'worsened';
}

/**
 * ICANS (Immune Effector Cell-Associated Neurotoxicity) assessment
 */
export interface ICANSAssessment {
  /** Patient identifier */
  patientId: string;

  /** Assessment date/time */
  timestamp: Date;

  /** Days post-infusion */
  daysPostInfusion: number;

  /** ICANS grade (0-4) */
  grade: 0 | 1 | 2 | 3 | 4;

  /** ICE score (0-10) */
  iceScore: number;

  /** ICE components */
  iceComponents: {
    orientation: number; // 0-4
    naming: number;      // 0-3
    following: number;   // 0-2
    writing: number;     // 0-1
  };

  /** Level of consciousness */
  consciousness: 'awake' | 'awakens-to-voice' | 'awakens-to-touch' | 'unresponsive';

  /** Seizures */
  seizures: boolean;

  /** Motor findings */
  motorFindings?: string;

  /** Cerebral edema */
  cerebralEdema: boolean;

  /** Treatment */
  treatment?: ICANSTreatment;

  /** Notes */
  notes?: string;
}

/**
 * ICANS treatment
 */
export interface ICANSTreatment {
  /** Dexamethasone */
  dexamethasone?: {
    dose: number; // mg
    frequency: string;
    started: Date;
  };

  /** Anti-seizure medication */
  antiSeizure?: {
    drug: string;
    dose: string;
    started: Date;
  };

  /** ICU level care */
  icu: boolean;

  /** Intubation */
  intubated: boolean;

  /** Response to treatment */
  response: 'improved' | 'stable' | 'worsened';
}

/**
 * Safety event
 */
export interface SafetyEvent {
  /** Event ID */
  eventId: string;

  /** Patient identifier */
  patientId: string;

  /** Event type */
  type: 'CRS' | 'ICANS' | 'infection' | 'cytopenia' | 'other';

  /** Severity */
  severity: 1 | 2 | 3 | 4 | 5;

  /** Event description */
  description: string;

  /** Onset date */
  onsetDate: Date;

  /** Resolution date */
  resolutionDate?: Date;

  /** Treatment given */
  treatment?: string;

  /** Outcome */
  outcome?: 'resolved' | 'resolving' | 'ongoing' | 'fatal';

  /** Serious adverse event */
  serious: boolean;

  /** Related to cell therapy */
  related: 'definite' | 'probable' | 'possible' | 'unlikely' | 'unrelated';
}

// ============================================================================
// Cryopreservation
// ============================================================================

/**
 * Cryopreservation protocol
 */
export interface CryopreservationProtocol {
  /** Protocol identifier */
  protocolId: string;

  /** Cryoprotectant formulation */
  cryoprotectant: {
    dmso: number;      // %
    hsa?: number;      // %
    dextran?: number;  // %
    other?: string;
  };

  /** Freezing method */
  freezingMethod: 'controlled-rate' | 'passive' | 'slow-freeze';

  /** Cooling rate */
  coolingRate?: number; // °C/min

  /** Target temperature */
  targetTemperature: number; // °C

  /** Storage temperature */
  storageTemperature: number; // °C

  /** Container type */
  containerType: 'cryovial' | 'cryobag' | 'straw';

  /** Fill volume */
  fillVolume: number; // mL

  /** Cell concentration */
  cellConcentration: number; // cells/mL
}

/**
 * Thawing protocol
 */
export interface ThawingProtocol {
  /** Thawing method */
  method: 'water-bath' | 'dry-thaw' | 'bedside';

  /** Thawing temperature */
  temperature: number; // °C

  /** Thawing duration */
  duration: number; // minutes

  /** Wash steps */
  washSteps: number;

  /** Dilution media */
  dilutionMedia?: string;

  /** Post-thaw rest time */
  restTime?: number; // minutes

  /** Quality check required */
  qualityCheck: boolean;
}

/**
 * Post-thaw quality metrics
 */
export interface PostThawQuality {
  /** Viability */
  viability: number;

  /** Recovery percentage */
  recovery: number;

  /** Potency retained */
  potencyRetained: number;

  /** Meets acceptance criteria */
  acceptable: boolean;

  /** Time to infusion */
  timeToInfusion?: number; // hours
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Cellular therapy constants and thresholds
 */
export const CELLULAR_THERAPY_CONSTANTS = {
  /** Minimum viability for clinical use */
  MIN_VIABILITY: 85,

  /** Minimum potency for release */
  MIN_POTENCY: 70,

  /** Maximum endotoxin (EU/kg) */
  MAX_ENDOTOXIN: 5,

  /** Minimum post-thaw recovery */
  MIN_RECOVERY: 70,

  /** CAR-T dose ranges (cells/kg) */
  CAR_T_DOSE: {
    MIN: 1e6,
    MAX: 5e6,
  },

  /** MSC dose ranges (cells/kg) */
  MSC_DOSE: {
    MIN: 1e6,
    MAX: 2e6,
  },

  /** Critical process parameters */
  CPP: {
    TEMPERATURE: { MIN: 36.5, MAX: 37.5 }, // °C
    PH: { MIN: 7.2, MAX: 7.4 },
    DISSOLVED_OXYGEN: { MIN: 20, MAX: 40 }, // %
    CELL_DENSITY: { MIN: 0.5e6, MAX: 2.0e6 }, // cells/mL
    OSMOLALITY: { MIN: 260, MAX: 320 }, // mOsm/kg
  },

  /** Storage temperatures */
  STORAGE: {
    CRYOGENIC: -150, // °C or colder
    ULTRA_LOW: -80,  // °C
    REFRIGERATED: 4, // °C
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
 * Batch processing result
 */
export interface BatchResult {
  /** Batch identifier */
  batchId: string;

  /** Success status */
  success: boolean;

  /** Final product */
  product?: CellProduct;

  /** Quality assessment */
  quality?: QualityResult;

  /** Manufacturing summary */
  manufacturingSummary: {
    startDate: Date;
    completionDate: Date;
    totalDays: number;
    finalExpansion: number;
    deviations: string[];
  };

  /** Errors encountered */
  errors?: string[];
}

/**
 * Certificate of Analysis
 */
export interface CertificateOfAnalysis {
  /** Certificate number */
  certificateNumber: string;

  /** Product information */
  product: CellProduct;

  /** Quality test results */
  testResults: QualityMetrics;

  /** Potency assay */
  potencyAssay: PotencyAssay;

  /** Release decision */
  releaseDecision: {
    approved: boolean;
    approvedBy: string;
    approvalDate: Date;
    expirationDate: Date;
  };

  /** Manufacturing summary */
  manufacturing: {
    facility: string;
    batchRecord: string;
    deviations: string[];
  };

  /** Storage and shipping */
  storage: {
    location: string;
    temperature: number;
    shippingConditions?: string;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-005 error codes
 */
export enum CellularTherapyErrorCode {
  VIABILITY_LOW = 'C001',
  POTENCY_FAILURE = 'C002',
  STERILITY_FAILURE = 'C003',
  DOSE_OUT_OF_RANGE = 'C004',
  IDENTITY_MISMATCH = 'C005',
  MANUFACTURING_DEVIATION = 'C006',
  QUALITY_FAILURE = 'C007',
  STORAGE_VIOLATION = 'C008',
  EXPIRED_PRODUCT = 'C009',
  INVALID_PARAMETERS = 'C010',
}

/**
 * Cellular therapy error
 */
export class CellularTherapyError extends Error {
  constructor(
    public code: CellularTherapyErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'CellularTherapyError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  CellProduct,
  StorageInfo,

  // Manufacturing
  ManufacturingProcess,
  CriticalProcessParameters,
  Alert,

  // Quality
  QualityMetrics,
  IdentityMarkers,
  PotencyAssay,
  QualityResult,
  ReleaseCriteria,

  // Dosing
  DoseParameters,
  DoseResult,
  PatientDose,
  LymphodepletionRegimen,

  // Safety
  CRSAssessment,
  CRSTreatment,
  ICANSAssessment,
  ICANSTreatment,
  SafetyEvent,

  // Cryo
  CryopreservationProtocol,
  ThawingProtocol,
  PostThawQuality,

  // Results
  BatchResult,
  CertificateOfAnalysis,
};

export { CELLULAR_THERAPY_CONSTANTS, CellularTherapyErrorCode, CellularTherapyError };
