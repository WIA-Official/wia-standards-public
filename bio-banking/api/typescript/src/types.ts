/**
 * WIA-BIO-019: Bio-Banking - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Sample types supported by the bio-banking standard
 */
export enum SampleType {
  WHOLE_BLOOD = 'whole_blood',
  PLASMA = 'plasma',
  SERUM = 'serum',
  BUFFY_COAT = 'buffy_coat',
  DNA = 'dna',
  RNA = 'rna',
  TISSUE_FRESH_FROZEN = 'tissue_fresh_frozen',
  TISSUE_FFPE = 'tissue_ffpe',
  PBMC = 'pbmc',
  CELL_LINE = 'cell_line',
  URINE = 'urine',
  SALIVA = 'saliva',
  STOOL = 'stool',
  OTHER = 'other',
}

/**
 * Storage conditions for biospecimens
 */
export enum StorageCondition {
  LIQUID_NITROGEN = 'liquid_nitrogen', // -196°C
  ULTRA_LOW_FREEZER = 'ultra_low_freezer', // -80°C
  STANDARD_FREEZER = 'standard_freezer', // -20°C
  REFRIGERATED = 'refrigerated', // 4°C
  AMBIENT = 'ambient', // 20-25°C
}

/**
 * Sample status throughout lifecycle
 */
export enum SampleStatus {
  COLLECTED = 'collected',
  IN_TRANSIT = 'in_transit',
  RECEIVED = 'received',
  PROCESSING = 'processing',
  STORED = 'stored',
  RETRIEVED = 'retrieved',
  IN_USE = 'in_use',
  RETURNED = 'returned',
  DEPLETED = 'depleted',
  DESTROYED = 'destroyed',
}

/**
 * Biosafety risk groups
 */
export enum RiskGroup {
  RG1 = 'rg1', // Low risk
  RG2 = 'rg2', // Moderate risk
  RG3 = 'rg3', // High risk
  RG4 = 'rg4', // Extreme risk
}

// ============================================================================
// Sample and Donor Information
// ============================================================================

/**
 * Donor demographic information (de-identified)
 */
export interface DonorInfo {
  /** Unique donor identifier (de-identified) */
  donorId: string;

  /** Age at collection */
  age?: number;

  /** Biological sex */
  sex?: 'male' | 'female' | 'other' | 'unknown';

  /** Ethnicity */
  ethnicity?: string;

  /** Collection site */
  collectionSite?: string;

  /** Medical history (relevant) */
  medicalHistory?: string[];

  /** Current medications */
  medications?: string[];

  /** Fasting status */
  fasting?: boolean;

  /** Consent ID reference */
  consentId?: string;
}

/**
 * Sample metadata
 */
export interface Sample {
  /** Unique sample identifier */
  id: string;

  /** Sample type */
  type: SampleType;

  /** Donor information */
  donor: DonorInfo;

  /** Collection date and time */
  collectionDate: Date;

  /** Volume or mass */
  volume: number;

  /** Unit (mL, mg, etc.) */
  volumeUnit: string;

  /** Storage condition */
  storageCondition: StorageCondition;

  /** Current storage location */
  location: StorageLocation;

  /** Sample status */
  status: SampleStatus;

  /** Quality metrics */
  quality?: QualityMetrics;

  /** Aliquots derived from this sample */
  aliquots?: Aliquot[];

  /** Parent sample ID (if this is an aliquot) */
  parentSampleId?: string;

  /** Biosafety risk group */
  riskGroup?: RiskGroup;

  /** Special handling notes */
  notes?: string;

  /** Created timestamp */
  createdAt: Date;

  /** Last updated timestamp */
  updatedAt: Date;
}

/**
 * Aliquot information
 */
export interface Aliquot {
  /** Aliquot ID */
  id: string;

  /** Parent sample ID */
  parentSampleId: string;

  /** Volume */
  volume: number;

  /** Volume unit */
  volumeUnit: string;

  /** Storage location */
  location: StorageLocation;

  /** Creation date */
  createdDate: Date;

  /** Current status */
  status: SampleStatus;
}

/**
 * Storage location within biobank
 */
export interface StorageLocation {
  /** Freezer or tank ID */
  freezer: string;

  /** Rack number */
  rack?: string;

  /** Box number */
  box?: string;

  /** Position within box */
  position?: string;

  /** Full location string */
  fullLocation?: string;
}

// ============================================================================
// Quality Metrics
// ============================================================================

/**
 * Quality metrics for samples
 */
export interface QualityMetrics {
  /** Sample Integrity Score (0-1) */
  integrityScore?: number;

  /** Storage Quality Index */
  storageQualityIndex?: number;

  /** DNA-specific metrics */
  dna?: DNAQuality;

  /** RNA-specific metrics */
  rna?: RNAQuality;

  /** Protein-specific metrics */
  protein?: ProteinQuality;

  /** Cell-specific metrics */
  cell?: CellQuality;

  /** Blood-specific metrics */
  blood?: BloodQuality;

  /** Temperature excursions logged */
  temperatureExcursions?: TemperatureExcursion[];

  /** Last quality check date */
  lastChecked?: Date;
}

/**
 * DNA quality metrics
 */
export interface DNAQuality {
  /** Concentration (ng/μL) */
  concentration: number;

  /** A260/A280 purity ratio */
  a260_280: number;

  /** A260/A230 ratio (organic contamination) */
  a260_230: number;

  /** DNA Integrity Number (if available) */
  din?: number;

  /** Fragment size (bp) */
  fragmentSize?: number;
}

/**
 * RNA quality metrics
 */
export interface RNAQuality {
  /** Concentration (ng/μL) */
  concentration: number;

  /** RNA Integrity Number (1-10) */
  rin: number;

  /** A260/A280 ratio */
  a260_280: number;

  /** 28S/18S ratio */
  ratio_28s_18s?: number;
}

/**
 * Protein quality metrics
 */
export interface ProteinQuality {
  /** Total protein concentration (mg/mL or g/L) */
  concentration: number;

  /** Concentration method */
  method: 'BCA' | 'Bradford' | 'Lowry' | 'A280' | 'other';

  /** Degradation status */
  degradation: 'none' | 'minimal' | 'moderate' | 'severe';

  /** Functional activity (if applicable, 0-1) */
  activity?: number;
}

/**
 * Cell quality metrics
 */
export interface CellQuality {
  /** Cell count (cells/mL) */
  cellCount: number;

  /** Viability percentage (0-100) */
  viability: number;

  /** Viability method */
  viabilityMethod: 'trypan_blue' | 'flow_cytometry' | 'mtt' | 'atp' | 'other';

  /** Post-thaw recovery (if applicable, 0-1) */
  postThawRecovery?: number;

  /** Contamination status */
  contamination?: {
    mycoplasma: boolean;
    bacteria: boolean;
    fungi: boolean;
  };
}

/**
 * Blood quality metrics
 */
export interface BloodQuality {
  /** Hemolysis index */
  hemolysisIndex?: number;

  /** Icterus index */
  icterusIndex?: number;

  /** Lipemia index */
  lipemiaIndex?: number;

  /** Total protein (g/L) */
  totalProtein?: number;

  /** Clotting time (for serum) */
  clottingTime?: number;
}

/**
 * Temperature excursion event
 */
export interface TemperatureExcursion {
  /** Event ID */
  id: string;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Target temperature */
  targetTemp: number;

  /** Actual temperature range */
  actualTempRange: {
    min: number;
    max: number;
  };

  /** Duration in minutes */
  duration: number;

  /** Severity */
  severity: 'minor' | 'moderate' | 'severe' | 'critical';

  /** Corrective action taken */
  correctiveAction?: string;

  /** Impact assessment */
  impactAssessment?: string;
}

// ============================================================================
// Sample Integrity Calculations
// ============================================================================

/**
 * Parameters for Sample Integrity Score calculation
 */
export interface IntegrityScoreParams {
  /** Initial viability or quality (0-1) */
  initialViability: number;

  /** Current viability or quality (0-1) */
  currentViability: number;

  /** Temperature deviation from optimal (°C) */
  tempDeviation: number;

  /** Maximum acceptable temperature deviation (°C) */
  maxTempDeviation: number;

  /** Storage time (days) */
  storageTime: number;

  /** Maximum recommended storage time (days) */
  maxStorageTime: number;
}

/**
 * Parameters for Storage Quality Index calculation
 */
export interface StorageQualityParams {
  /** Sample type (affects degradation rate) */
  sampleType: SampleType;

  /** Storage time (years) */
  storageTime: number;

  /** Storage temperature (Kelvin) */
  storageTemp: number;

  /** Degradation rate constant (sample-specific) */
  degradationRate?: number;

  /** Activation energy (kJ/mol) */
  activationEnergy?: number;
}

/**
 * Result of integrity calculation
 */
export interface IntegrityResult {
  /** Sample Integrity Score (0-1) */
  score: number;

  /** Interpretation */
  interpretation: 'excellent' | 'good' | 'acceptable' | 'marginal' | 'poor';

  /** Recommendation */
  recommendation: string;

  /** Details of calculation */
  details: {
    viabilityFactor: number;
    temperatureFactor: number;
    timeFactor: number;
  };
}

// ============================================================================
// Chain of Custody
// ============================================================================

/**
 * Chain of custody event
 */
export interface CustodyEvent {
  /** Event ID */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** Event type */
  action: CustodyAction;

  /** Operator ID */
  operator: string;

  /** Location */
  location: string;

  /** Sample ID */
  sampleId: string;

  /** Sample condition at event */
  condition?: {
    temperature?: number;
    volume?: number;
    appearance?: string;
  };

  /** Previous event hash (SHA-256) */
  previousHash?: string;

  /** Current event hash (SHA-256) */
  hash: string;

  /** Digital signature (optional) */
  signature?: string;

  /** Notes */
  notes?: string;
}

/**
 * Custody action types
 */
export enum CustodyAction {
  COLLECTION = 'collection',
  TRANSPORT = 'transport',
  RECEIPT = 'receipt',
  PROCESSING = 'processing',
  ALIQUOTING = 'aliquoting',
  STORAGE = 'storage',
  RETRIEVAL = 'retrieval',
  RETURN = 'return',
  TRANSFER = 'transfer',
  DESTRUCTION = 'destruction',
  QUALITY_CHECK = 'quality_check',
}

/**
 * Complete chain of custody for a sample
 */
export interface ChainOfCustody {
  /** Sample ID */
  sampleId: string;

  /** Custody events */
  events: CustodyEvent[];

  /** Is chain intact? */
  intact: boolean;

  /** Verification timestamp */
  verifiedAt?: Date;

  /** Custody holder */
  currentCustodian?: string;
}

// ============================================================================
// Consent Management
// ============================================================================

/**
 * Consent types
 */
export enum ConsentType {
  BROAD = 'broad',
  SPECIFIC = 'specific',
  TIERED = 'tiered',
}

/**
 * Consent information
 */
export interface Consent {
  /** Consent ID */
  consentId: string;

  /** Donor ID */
  donorId: string;

  /** Consent type */
  type: ConsentType;

  /** Consent date */
  consentDate: Date;

  /** Expiry date (if applicable) */
  expiryDate?: Date;

  /** Permitted purposes */
  purposes: string[];

  /** Allow commercial use? */
  commercialUse: boolean;

  /** Allow sharing with third parties? */
  thirdPartySharing: boolean;

  /** Allow international transfer? */
  internationalTransfer: boolean;

  /** Return of results to donor? */
  returnResults: boolean;

  /** Consent form version */
  formVersion: string;

  /** Is consent still valid? */
  isValid: boolean;

  /** Withdrawal date (if withdrawn) */
  withdrawalDate?: Date;

  /** Language of consent */
  language: string;

  /** Electronic signature */
  signature?: string;
}

// ============================================================================
// Sample Requests
// ============================================================================

/**
 * Sample request
 */
export interface SampleRequest {
  /** Request ID */
  requestId: string;

  /** Requester ID */
  requesterId: string;

  /** Requester organization */
  organization: string;

  /** Project ID */
  projectId: string;

  /** Project title */
  projectTitle: string;

  /** Requested samples */
  samples: RequestedSample[];

  /** Purpose of request */
  purpose: string;

  /** Ethics approval number */
  ethicsApproval: string;

  /** Material Transfer Agreement */
  mta?: string;

  /** Request date */
  requestDate: Date;

  /** Desired delivery date */
  desiredDeliveryDate?: Date;

  /** Request status */
  status: RequestStatus;

  /** Approval/rejection notes */
  notes?: string;

  /** Approved by */
  approvedBy?: string;

  /** Approval date */
  approvalDate?: Date;
}

/**
 * Individual sample within a request
 */
export interface RequestedSample {
  /** Sample ID */
  sampleId: string;

  /** Volume requested */
  volumeRequested: number;

  /** Volume unit */
  volumeUnit: string;

  /** Expected return date (if returnable) */
  returnDate?: Date;

  /** Is this a permanent withdrawal? */
  permanent: boolean;
}

/**
 * Request status
 */
export enum RequestStatus {
  PENDING = 'pending',
  UNDER_REVIEW = 'under_review',
  APPROVED = 'approved',
  REJECTED = 'rejected',
  FULFILLED = 'fulfilled',
  CANCELLED = 'cancelled',
}

// ============================================================================
// Inventory Management
// ============================================================================

/**
 * Inventory query parameters
 */
export interface InventoryQuery {
  /** Sample type filter */
  sampleType?: SampleType;

  /** Storage condition filter */
  storageCondition?: StorageCondition;

  /** Minimum volume available */
  minVolume?: number;

  /** Only available (not depleted) samples */
  availableOnly?: boolean;

  /** Collection date range */
  collectionDateRange?: {
    from: Date;
    to: Date;
  };

  /** Minimum quality score */
  minQualityScore?: number;

  /** Donor age range */
  donorAgeRange?: {
    min: number;
    max: number;
  };

  /** Donor sex */
  donorSex?: 'male' | 'female' | 'other';

  /** Limit results */
  limit?: number;

  /** Offset for pagination */
  offset?: number;
}

/**
 * Inventory summary
 */
export interface InventorySummary {
  /** Total number of samples */
  totalSamples: number;

  /** Available samples */
  availableSamples: number;

  /** Depleted samples */
  depletedSamples: number;

  /** Breakdown by sample type */
  byType: Record<SampleType, number>;

  /** Breakdown by storage condition */
  byStorage: Record<StorageCondition, number>;

  /** Total volume available */
  totalVolume: {
    value: number;
    unit: string;
  };

  /** Last updated */
  lastUpdated: Date;
}

// ============================================================================
// Storage Equipment
// ============================================================================

/**
 * Storage equipment (freezer, tank, etc.)
 */
export interface StorageEquipment {
  /** Equipment ID */
  id: string;

  /** Equipment type */
  type: 'liquid_nitrogen_tank' | 'ultra_low_freezer' | 'standard_freezer' | 'refrigerator';

  /** Location */
  location: string;

  /** Target temperature */
  targetTemp: number;

  /** Current temperature */
  currentTemp: number;

  /** Capacity (number of boxes or samples) */
  capacity: number;

  /** Current occupancy */
  occupancy: number;

  /** Status */
  status: 'operational' | 'maintenance' | 'alarm' | 'offline';

  /** Last maintenance date */
  lastMaintenance?: Date;

  /** Next maintenance due */
  nextMaintenance?: Date;

  /** Temperature log */
  temperatureLog?: TemperatureReading[];

  /** Alarms */
  alarms?: EquipmentAlarm[];
}

/**
 * Temperature reading
 */
export interface TemperatureReading {
  /** Timestamp */
  timestamp: Date;

  /** Temperature */
  temperature: number;

  /** Within acceptable range? */
  inRange: boolean;
}

/**
 * Equipment alarm
 */
export interface EquipmentAlarm {
  /** Alarm ID */
  id: string;

  /** Alarm type */
  type: 'high_temp' | 'low_temp' | 'power_failure' | 'door_open' | 'low_ln2' | 'other';

  /** Timestamp */
  timestamp: Date;

  /** Severity */
  severity: 'info' | 'warning' | 'critical';

  /** Description */
  description: string;

  /** Acknowledged? */
  acknowledged: boolean;

  /** Acknowledged by */
  acknowledgedBy?: string;

  /** Resolution */
  resolution?: string;

  /** Resolved timestamp */
  resolvedAt?: Date;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants and degradation parameters
 */
export const BIO_CONSTANTS = {
  /** Gas constant (J/mol·K) */
  GAS_CONSTANT: 8.314,

  /** Degradation parameters by sample type */
  DEGRADATION_PARAMS: {
    [SampleType.DNA]: {
      k: 0.001, // 1/year
      Ea: 80, // kJ/mol
      halfLife_80C: 100, // years
    },
    [SampleType.RNA]: {
      k: 0.05,
      Ea: 60,
      halfLife_80C: 14,
    },
    [SampleType.PLASMA]: {
      k: 0.02,
      Ea: 70,
      halfLife_80C: 35,
    },
    [SampleType.PBMC]: {
      k: 0.005,
      Ea: 90,
      halfLife_80C: 140,
    },
  },

  /** Storage temperature constants (Kelvin) */
  STORAGE_TEMPS: {
    LIQUID_NITROGEN: 77, // -196°C
    ULTRA_LOW: 193, // -80°C
    STANDARD_FREEZER: 253, // -20°C
    REFRIGERATED: 277, // 4°C
    AMBIENT: 293, // 20°C
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-019 error codes
 */
export enum BioErrorCode {
  INVALID_SAMPLE_ID = 'B001',
  SAMPLE_NOT_FOUND = 'B002',
  INSUFFICIENT_VOLUME = 'B003',
  QUALITY_BELOW_THRESHOLD = 'B004',
  CONSENT_EXPIRED = 'B005',
  CONSENT_NOT_COMPATIBLE = 'B006',
  STORAGE_FULL = 'B007',
  TEMPERATURE_EXCURSION = 'B008',
  INVALID_LOCATION = 'B009',
  CHAIN_OF_CUSTODY_BROKEN = 'B010',
  EQUIPMENT_FAILURE = 'B011',
  UNAUTHORIZED_ACCESS = 'B012',
}

/**
 * Bio-banking error
 */
export class BioBankingError extends Error {
  constructor(
    public code: BioErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BioBankingError';
  }
}

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
 * Pagination metadata
 */
export interface PaginationMeta {
  /** Total count */
  total: number;

  /** Limit per page */
  limit: number;

  /** Offset */
  offset: number;

  /** Has more pages? */
  hasMore: boolean;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Data items */
  items: T[];

  /** Pagination metadata */
  meta: PaginationMeta;
}

// ============================================================================
// Export All
// ============================================================================

export type {
  DonorInfo,
  Sample,
  Aliquot,
  StorageLocation,
  QualityMetrics,
  DNAQuality,
  RNAQuality,
  ProteinQuality,
  CellQuality,
  BloodQuality,
  TemperatureExcursion,
  IntegrityScoreParams,
  StorageQualityParams,
  IntegrityResult,
  CustodyEvent,
  ChainOfCustody,
  Consent,
  SampleRequest,
  RequestedSample,
  InventoryQuery,
  InventorySummary,
  StorageEquipment,
  TemperatureReading,
  EquipmentAlarm,
  PaginationMeta,
  PaginatedResponse,
};

export {
  SampleType,
  StorageCondition,
  SampleStatus,
  RiskGroup,
  CustodyAction,
  ConsentType,
  RequestStatus,
  BIO_CONSTANTS,
  BioErrorCode,
  BioBankingError,
};
