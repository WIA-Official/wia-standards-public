/**
 * WIA-BIO-017: Bio-Safety - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biosafety Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Biosafety Types
// ============================================================================

/**
 * Biosafety levels (BSL-1 through BSL-4)
 */
export type BiosaftyLevel = 'BSL-1' | 'BSL-2' | 'BSL-3' | 'BSL-4';

/**
 * Risk groups for biological agents
 */
export type RiskGroup = 1 | 2 | 3 | 4;

/**
 * Severity levels for incidents
 */
export type SeverityLevel = 'minor' | 'moderate' | 'serious' | 'critical';

/**
 * Risk level categories
 */
export type RiskLevel = 'low' | 'medium' | 'high' | 'extreme';

/**
 * Exposure routes
 */
export type ExposureRoute = 'inhalation' | 'ingestion' | 'percutaneous' | 'mucous-membrane' | 'other';

// ============================================================================
// Risk Assessment Types
// ============================================================================

/**
 * Parameters for biosafety risk assessment
 */
export interface RiskAssessment {
  /** Hazard level (1-4, corresponding to Risk Group) */
  hazardLevel: RiskGroup;

  /** Probability of exposure (0-1) */
  exposureProbability: number;

  /** Extent of potential impact (1-10) */
  impactExtent: number;

  /** Containment effectiveness (0.1-1) */
  containmentEffectiveness: number;

  /** Optional: Agent name */
  agentName?: string;

  /** Optional: Procedure description */
  procedureDescription?: string;
}

/**
 * Risk score calculation result
 */
export interface RiskScore {
  /** Calculated risk score (0-100) */
  riskScore: number;

  /** Risk level category */
  riskLevel: RiskLevel;

  /** Recommended biosafety level */
  bslRecommended: BiosaftyLevel;

  /** Safety recommendations */
  recommendations: string[];

  /** Individual risk components */
  components: {
    hazard: number;
    probability: number;
    impact: number;
    containment: number;
  };
}

/**
 * Exposure assessment parameters
 */
export interface ExposureAssessment {
  /** Dose concentration (CFU/ml or viral particles/ml) */
  doseConcentration: number;

  /** Frequency of exposure events */
  frequency: number;

  /** Time duration of exposure (minutes) */
  duration: number;

  /** Safety factor (typically 0.01-0.1) */
  safetyFactor?: number;

  /** Exposure route */
  route?: ExposureRoute;
}

/**
 * Exposure assessment result
 */
export interface ExposureResult {
  /** Calculated exposure assessment value */
  exposureValue: number;

  /** Estimated infectious dose */
  infectiousDose: number;

  /** Risk of infection (0-1) */
  infectionRisk: number;

  /** Recommended actions */
  recommendations: string[];

  /** Medical evaluation required? */
  medicalEvaluationRequired: boolean;
}

// ============================================================================
// BSL Validation Types
// ============================================================================

/**
 * BSL compliance validation parameters
 */
export interface BSLValidation {
  /** Required biosafety level for the pathogen */
  pathogenClass: BiosaftyLevel;

  /** Actual biosafety level of the facility */
  facilityLevel: BiosaftyLevel;

  /** Type of procedure being performed */
  procedureType: string;

  /** Available PPE */
  ppeAvailable: string[];

  /** Engineering controls in place */
  engineeringControls?: string[];

  /** Administrative controls */
  administrativeControls?: string[];

  /** Aerosol-generating procedure? */
  aerosolGenerating?: boolean;
}

/**
 * BSL compliance validation result
 */
export interface ComplianceResult {
  /** Is the setup compliant? */
  isCompliant: boolean;

  /** Compliance violations */
  violations: string[];

  /** Warnings (non-blocking) */
  warnings: string[];

  /** Recommendations for improvement */
  recommendations: string[];

  /** Overall compliance score (0-100) */
  complianceScore: number;

  /** Required upgrades */
  requiredUpgrades?: string[];
}

// ============================================================================
// PPE Types
// ============================================================================

/**
 * PPE requirement parameters
 */
export interface PPERequest {
  /** Biosafety level */
  bslLevel: RiskGroup;

  /** Type of procedure */
  procedureType: string;

  /** Is it an aerosol-generating procedure? */
  aerosolGenerating: boolean;

  /** Chemical exposure expected? */
  chemicalExposure?: boolean;

  /** Sharps involved? */
  sharpsInvolved?: boolean;

  /** Duration of procedure (hours) */
  duration?: number;
}

/**
 * Glove specifications
 */
export interface GloveRequirement {
  /** Glove material */
  material: 'latex' | 'nitrile' | 'neoprene' | 'butyl' | 'silver-shield';

  /** Number of glove layers */
  layers: 1 | 2;

  /** Extended cuff required? */
  extendedCuff: boolean;

  /** Chemical resistance rating (0-10) */
  chemicalResistance?: number;

  /** Puncture resistance rating (0-10) */
  punctureResistance?: number;
}

/**
 * Complete PPE requirements
 */
export interface PPERequirements {
  /** Glove specifications */
  gloves: GloveRequirement;

  /** Respiratory protection */
  respiratoryProtection: 'none' | 'surgical-mask' | 'N95' | 'PAPR' | 'supplied-air';

  /** Eye protection */
  eyeProtection: 'none' | 'safety-glasses' | 'goggles' | 'face-shield' | 'full-face-respirator';

  /** Body protection */
  bodyProtection: 'lab-coat' | 'gown' | 'solid-front-gown' | 'coverall' | 'positive-pressure-suit';

  /** Footwear requirements */
  footwear: 'closed-toe-shoes' | 'dedicated-shoes' | 'dedicated-boots';

  /** Additional PPE */
  additional: string[];

  /** PPE donning sequence */
  donningSequence?: string[];

  /** PPE doffing sequence */
  doffingSequence?: string[];
}

// ============================================================================
// Biological Agent Types
// ============================================================================

/**
 * Biological agent information
 */
export interface BiologicalAgent {
  /** Agent name */
  name: string;

  /** Risk group classification */
  riskGroup: RiskGroup;

  /** Recommended biosafety level */
  bslLevel: BiosaftyLevel;

  /** Type of organism */
  type: 'bacteria' | 'virus' | 'fungus' | 'parasite' | 'prion' | 'toxin' | 'other';

  /** Transmission routes */
  transmissionRoutes: ExposureRoute[];

  /** Infectious dose (ID50) */
  infectiousDose?: number;

  /** Incubation period (days) */
  incubationPeriod?: {
    min: number;
    max: number;
  };

  /** Available prophylaxis? */
  prophylaxisAvailable: boolean;

  /** Available treatment? */
  treatmentAvailable: boolean;

  /** Special handling requirements */
  specialRequirements?: string[];

  /** Select agent status */
  selectAgent?: boolean;
}

// ============================================================================
// Incident Types
// ============================================================================

/**
 * Biosafety incident report
 */
export interface IncidentReport {
  /** Unique incident ID */
  incidentId: string;

  /** Date and time of incident */
  date: Date | string;

  /** Location of incident */
  location: string;

  /** Severity level */
  severity: 1 | 2 | 3 | 4;

  /** Incident type */
  type: 'spill' | 'exposure' | 'equipment-failure' | 'breach' | 'procedural-violation' | 'other';

  /** Personnel involved */
  personnel: string[];

  /** Biological agent involved */
  agent: BiologicalAgent;

  /** Agent concentration (if applicable) */
  concentration?: number;

  /** Detailed description */
  description: string;

  /** Immediate actions taken */
  immediateActions: string[];

  /** Medical evaluation */
  medicalEvaluation?: MedicalEvaluation;

  /** Root cause */
  rootCause?: string;

  /** Corrective actions */
  correctiveActions: string[];

  /** Preventive actions */
  preventiveActions?: string[];

  /** Follow-up required? */
  followUpRequired: boolean;

  /** Status */
  status: 'open' | 'under-investigation' | 'resolved' | 'closed';

  /** Reported by */
  reportedBy: string;

  /** Reviewed by */
  reviewedBy?: string;
}

/**
 * Medical evaluation for exposure incident
 */
export interface MedicalEvaluation {
  /** Exposure route */
  exposureRoute: ExposureRoute;

  /** Estimated dose */
  estimatedDose?: number;

  /** Symptoms present? */
  symptomsPresent: boolean;

  /** Symptom description */
  symptoms?: string[];

  /** Immunization status */
  immunizationStatus?: 'current' | 'outdated' | 'none';

  /** Post-exposure prophylaxis administered? */
  prophylaxisAdministered: boolean;

  /** Prophylaxis details */
  prophylaxisDetails?: string;

  /** Follow-up schedule */
  followUpSchedule?: Date[];

  /** Medical restrictions */
  medicalRestrictions?: string[];

  /** Return to work status */
  returnToWork?: 'immediate' | 'restricted' | 'not-cleared';
}

// ============================================================================
// Decontamination Types
// ============================================================================

/**
 * Decontamination method
 */
export type DecontaminationMethod =
  | 'autoclave'
  | 'chemical-disinfection'
  | 'dry-heat'
  | 'uv-irradiation'
  | 'incineration'
  | 'other';

/**
 * Decontamination procedure
 */
export interface DecontaminationProcedure {
  /** Method used */
  method: DecontaminationMethod;

  /** Agent being decontaminated */
  agent: string;

  /** Material/surface type */
  materialType: string;

  /** Temperature (if applicable) */
  temperature?: number;

  /** Pressure (if applicable) */
  pressure?: number;

  /** Contact time (minutes) */
  contactTime: number;

  /** Disinfectant concentration (if applicable) */
  concentration?: number;

  /** Validation method */
  validation?: 'biological-indicator' | 'chemical-indicator' | 'visual' | 'none';

  /** Expected log reduction */
  expectedLogReduction: number;

  /** Procedure effective? */
  effective: boolean;
}

/**
 * Disinfectant information
 */
export interface Disinfectant {
  /** Disinfectant name */
  name: string;

  /** Active ingredient */
  activeIngredient: string;

  /** Working concentration */
  concentration: string;

  /** Contact time (minutes) */
  contactTime: number;

  /** Spectrum of activity */
  spectrum: ('bacteria' | 'virus' | 'fungi' | 'spores' | 'mycobacteria')[];

  /** Effective against specific agents */
  effectiveAgainst: string[];

  /** Limitations */
  limitations?: string[];

  /** Compatible materials */
  compatibleMaterials: string[];

  /** Incompatible materials */
  incompatibleMaterials: string[];

  /** Safety precautions */
  safetyPrecautions: string[];
}

// ============================================================================
// Facility Types
// ============================================================================

/**
 * Biosafety facility specifications
 */
export interface BiosaftyFacility {
  /** Facility ID */
  facilityId: string;

  /** Facility name */
  name: string;

  /** Biosafety level */
  bslLevel: BiosaftyLevel;

  /** Location */
  location: {
    building: string;
    room: string;
    address?: string;
  };

  /** Engineering controls */
  engineeringControls: {
    bscType?: 'class-I' | 'class-II-A1' | 'class-II-A2' | 'class-II-B1' | 'class-II-B2' | 'class-III';
    hepaFiltration: boolean;
    directionalAirflow: boolean;
    negativePressure: boolean;
    anteroom: boolean;
    autoclave: 'in-room' | 'in-facility' | 'external';
    eyewashStation: boolean;
    safetyShower: boolean;
  };

  /** Access controls */
  accessControls: {
    restrictedAccess: boolean;
    selfClosingDoors: boolean;
    lockableDoors: boolean;
    accessLog: boolean;
    biometricAccess?: boolean;
  };

  /** Certification status */
  certification: {
    certified: boolean;
    certificationDate?: Date;
    expirationDate?: Date;
    certifyingAuthority?: string;
    nextInspectionDate?: Date;
  };

  /** Authorized agents */
  authorizedAgents: BiologicalAgent[];

  /** Personnel authorized */
  authorizedPersonnel: string[];
}

// ============================================================================
// Training Types
// ============================================================================

/**
 * Biosafety training record
 */
export interface TrainingRecord {
  /** Personnel ID */
  personnelId: string;

  /** Personnel name */
  name: string;

  /** Training type */
  trainingType: 'initial' | 'refresher' | 'agent-specific' | 'emergency' | 'equipment';

  /** Training date */
  trainingDate: Date;

  /** Expiration date */
  expirationDate?: Date;

  /** Topics covered */
  topicsCovered: string[];

  /** Competency assessment */
  competencyAssessment?: {
    passed: boolean;
    score?: number;
    assessor: string;
  };

  /** Certification issued? */
  certificationIssued: boolean;

  /** Certificate number */
  certificateNumber?: string;

  /** Next training due */
  nextTrainingDue?: Date;
}

// ============================================================================
// Containment Types
// ============================================================================

/**
 * Containment effectiveness assessment
 */
export interface ContainmentEffectiveness {
  /** BSC effectiveness (0.8-0.999) */
  bscEffectiveness: number;

  /** PPE effectiveness (0.5-0.99) */
  ppeEffectiveness: number;

  /** Procedural adherence (0.6-0.95) */
  proceduralAdherence: number;

  /** Overall containment effectiveness */
  overallEffectiveness: number;

  /** Number of containment breaches */
  breaches: number;

  /** Total operations performed */
  totalOperations: number;

  /** Containment efficiency percentage */
  containmentEfficiency: number;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Biosafety constants and thresholds
 */
export const BIOSAFETY_CONSTANTS = {
  /** Minimum autoclave temperature (°C) */
  AUTOCLAVE_TEMP: 121,

  /** Minimum autoclave time (minutes) */
  AUTOCLAVE_TIME: 15,

  /** Minimum autoclave pressure (psi) */
  AUTOCLAVE_PRESSURE: 15,

  /** Minimum BSC face velocity (fpm) */
  BSC_FACE_VELOCITY: 75,

  /** UV-C germicidal wavelength (nm) */
  UV_WAVELENGTH: 254,

  /** Minimum log reduction for sterilization */
  MIN_LOG_REDUCTION: 6,

  /** Spill contact time for disinfectant (minutes) */
  SPILL_CONTACT_TIME: 20,

  /** Bleach working concentration (ppm) */
  BLEACH_CONCENTRATION: 5000,

  /** Risk score thresholds */
  RISK_THRESHOLDS: {
    LOW: 10,
    MEDIUM: 30,
    HIGH: 60,
  },

  /** Infectious dose multipliers by route */
  ROUTE_MULTIPLIERS: {
    inhalation: 1.0,
    ingestion: 0.3,
    percutaneous: 0.05,
    'mucous-membrane': 0.3,
    other: 0.1,
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
 * Biosafety validation result
 */
export interface ValidationResult {
  /** Is valid? */
  isValid: boolean;

  /** Errors (blocking) */
  errors: string[];

  /** Warnings (non-blocking) */
  warnings: string[];

  /** Info messages */
  info: string[];
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-017 error codes
 */
export enum BiosaftyErrorCode {
  INSUFFICIENT_CONTAINMENT = 'B001',
  PPE_NON_COMPLIANCE = 'B002',
  UNAUTHORIZED_ACCESS = 'B003',
  DECONTAMINATION_FAILURE = 'B004',
  EXPOSURE_INCIDENT = 'B005',
  EQUIPMENT_MALFUNCTION = 'B006',
  INVALID_PARAMETERS = 'B007',
  BSL_MISMATCH = 'B008',
  TRAINING_EXPIRED = 'B009',
  CERTIFICATION_REQUIRED = 'B010',
}

/**
 * Biosafety error
 */
export class BiosaftyError extends Error {
  constructor(
    public code: BiosaftyErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BiosaftyError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  BiosaftyLevel,
  RiskGroup,
  SeverityLevel,
  RiskLevel,
  ExposureRoute,

  // Risk assessment
  RiskAssessment,
  RiskScore,
  ExposureAssessment,
  ExposureResult,

  // BSL validation
  BSLValidation,
  ComplianceResult,

  // PPE
  PPERequest,
  GloveRequirement,
  PPERequirements,

  // Biological agents
  BiologicalAgent,

  // Incidents
  IncidentReport,
  MedicalEvaluation,

  // Decontamination
  DecontaminationMethod,
  DecontaminationProcedure,
  Disinfectant,

  // Facilities
  BiosaftyFacility,

  // Training
  TrainingRecord,

  // Containment
  ContainmentEffectiveness,

  // Validation
  ValidationResult,
};

export { BIOSAFETY_CONSTANTS, BiosaftyErrorCode, BiosaftyError };
