/**
 * WIA-AUG-002: Cybernetic Implant - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Cybernetics Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Implant Classification Types
// ============================================================================

/**
 * Implant type classification
 */
export type ImplantType = 'PASSIVE' | 'ACTIVE' | 'SMART' | 'NEURAL_INTERFACE';

/**
 * Biocompatibility class per ISO 10993
 */
export type BiocompatibilityClass = 'CLASS_I' | 'CLASS_II' | 'CLASS_III';

/**
 * Power source types
 */
export type PowerSource = 'BATTERY' | 'WIRELESS' | 'BIO_HARVEST' | 'HYBRID';

/**
 * Integration level (1-5)
 */
export type IntegrationLevel = 1 | 2 | 3 | 4 | 5;

/**
 * Tissue type for implant contact
 */
export type TissueType = 'skin' | 'muscle' | 'bone' | 'blood' | 'neural';

/**
 * Contact duration classification
 */
export type ContactDuration = 'limited' | 'prolonged' | 'permanent';

/**
 * Implant classification input
 */
export interface ImplantClassificationInput {
  /** Does the implant have a power source */
  hasPowerSource: boolean;

  /** Does the implant have processing capability */
  hasProcessing: boolean;

  /** Does the implant connect to neural tissue */
  hasNeuralConnection: boolean;

  /** Does the implant exhibit adaptive behavior */
  adaptiveBehavior: boolean;

  /** Does the implant have AI capability */
  aiCapability: boolean;

  /** Anatomical location */
  location: string;

  /** Integration level (1-5) */
  integrationLevel: IntegrationLevel;
}

/**
 * Implant classification result
 */
export interface ImplantClassification {
  /** Determined implant type */
  type: ImplantType;

  /** Integration level */
  integrationLevel: IntegrationLevel;

  /** Biocompatibility class */
  bioClass: BiocompatibilityClass;

  /** Risk level */
  riskLevel: 'low' | 'moderate' | 'high' | 'critical';

  /** Regulatory requirements */
  requirements: string[];

  /** Recommended monitoring frequency */
  monitoringFrequency: 'daily' | 'weekly' | 'biweekly' | 'monthly' | 'continuous';
}

// ============================================================================
// Biocompatibility Types
// ============================================================================

/**
 * Material composition entry
 */
export interface MaterialEntry {
  /** Material name */
  name: string;

  /** Material type */
  type: 'metal' | 'polymer' | 'ceramic' | 'composite' | 'biological';

  /** Percentage composition */
  percentage: number;

  /** Biocompatibility status */
  approved: boolean;

  /** ISO 10993 test status */
  tested: boolean;
}

/**
 * Biocompatibility test type
 */
export type BiocompatibilityTest =
  | 'cytotoxicity'
  | 'sensitization'
  | 'irritation'
  | 'systemic_toxicity'
  | 'genotoxicity'
  | 'implantation'
  | 'hemocompatibility'
  | 'carcinogenicity'
  | 'reproductive_toxicity';

/**
 * Test result status
 */
export type TestStatus = 'PASS' | 'FAIL' | 'PENDING' | 'NOT_REQUIRED';

/**
 * Biocompatibility test result
 */
export interface BioTestResult {
  /** Test type */
  test: BiocompatibilityTest;

  /** ISO standard reference */
  standard: string;

  /** Test result */
  result: TestStatus;

  /** Test date */
  date?: Date;

  /** Testing laboratory */
  lab?: string;

  /** Report number */
  reportNumber?: string;

  /** Notes */
  notes?: string;
}

/**
 * Biocompatibility assessment parameters
 */
export interface BioAssessmentParams {
  /** Implant identifier */
  implantId: string;

  /** Material composition */
  materialComposition: MaterialEntry[];

  /** Contact duration */
  contactDuration: ContactDuration;

  /** Tissue type */
  tissueType: TissueType;

  /** Existing test results */
  testResults?: BioTestResult[];
}

/**
 * Biocompatibility assessment result
 */
export interface BiocompatibilityResult {
  /** Implant ID */
  implantId: string;

  /** Assigned biocompatibility class */
  class: BiocompatibilityClass;

  /** Required tests for this class */
  requiredTests: BiocompatibilityTest[];

  /** Completed test results */
  testResults: BioTestResult[];

  /** Is compliant */
  compliant: boolean;

  /** Pending tests */
  pendingTests: BiocompatibilityTest[];

  /** Failed tests */
  failedTests: BiocompatibilityTest[];

  /** Overall assessment */
  assessment: 'approved' | 'conditional' | 'rejected';

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Power Management Types
// ============================================================================

/**
 * Charging method
 */
export type ChargingMethod = 'inductive' | 'rf' | 'wired' | 'none';

/**
 * Power configuration
 */
export interface PowerConfig {
  /** Primary power source */
  primarySource: PowerSource;

  /** Backup power sources */
  backupSources: PowerSource[];

  /** Charging schedule */
  chargingSchedule: {
    frequency: 'daily' | 'weekly' | 'monthly' | 'continuous';
    duration: number; // minutes
    method: ChargingMethod;
  };

  /** Low power threshold (%) */
  lowPowerThreshold: number;

  /** Critical power threshold (%) */
  criticalPowerThreshold: number;

  /** Safe mode configuration */
  safeModeConfig: {
    trigger: number; // percentage
    reducedFunctions: string[];
    essentialFunctions: string[];
  };
}

/**
 * Power metrics
 */
export interface PowerMetrics {
  /** Current battery level (0-100%) */
  batteryLevel: number;

  /** Charge cycles count */
  chargeCycles: number;

  /** Current power consumption (mW) */
  powerConsumption: number;

  /** Estimated runtime (hours) */
  estimatedRuntime: number;

  /** Charging status */
  chargingStatus: 'charging' | 'discharging' | 'full' | 'fault';

  /** Active power source */
  activeSource: PowerSource;

  /** Source availability */
  sourceAvailability: {
    battery: boolean;
    wireless: boolean;
    bioHarvest: boolean;
  };
}

/**
 * Power alert
 */
export interface PowerAlert {
  /** Alert ID */
  id: string;

  /** Alert type */
  type: 'low_power' | 'critical_power' | 'charging_required' | 'source_failure' | 'abnormal_consumption';

  /** Severity */
  severity: 'info' | 'warning' | 'critical';

  /** Message */
  message: string;

  /** Timestamp */
  timestamp: Date;

  /** Acknowledged */
  acknowledged: boolean;
}

// ============================================================================
// Communication Protocol Types
// ============================================================================

/**
 * Communication protocol
 */
export type CommProtocol = 'BLE' | 'MICS' | 'NFC' | 'MBAN' | 'NEURAL';

/**
 * Communication security level
 */
export type SecurityLevel = 'none' | 'basic' | 'standard' | 'high' | 'medical_grade';

/**
 * Connection configuration
 */
export interface ConnectionConfig {
  /** Protocol to use */
  protocol: CommProtocol;

  /** Security level */
  security: SecurityLevel;

  /** Encryption enabled */
  encryption: boolean;

  /** Authentication required */
  authentication: boolean;

  /** Data rate (kbps) */
  dataRate?: number;

  /** Range (meters) */
  range?: number;
}

/**
 * Connection status
 */
export interface Connection {
  /** Connection ID */
  id: string;

  /** Protocol in use */
  protocol: CommProtocol;

  /** Connection status */
  status: 'connected' | 'connecting' | 'disconnected' | 'error';

  /** Signal strength (0-100%) */
  signalStrength: number;

  /** Data rate (kbps) */
  dataRate: number;

  /** Encryption status */
  encrypted: boolean;

  /** Latency (ms) */
  latency: number;

  /** Last activity */
  lastActivity: Date;
}

/**
 * Implant data packet
 */
export interface ImplantData {
  /** Data type */
  type: 'telemetry' | 'command' | 'config' | 'diagnostic' | 'neural';

  /** Payload */
  payload: Record<string, unknown>;

  /** Timestamp */
  timestamp: Date;

  /** Priority */
  priority: 'low' | 'normal' | 'high' | 'critical';

  /** Encryption required */
  encrypted: boolean;
}

/**
 * Transmission result
 */
export interface TransmissionResult {
  /** Success status */
  success: boolean;

  /** Message ID */
  messageId: string;

  /** Transmission time (ms) */
  transmissionTime: number;

  /** Acknowledgment received */
  acknowledged: boolean;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Rejection Monitoring Types
// ============================================================================

/**
 * Biomarker measurement
 */
export interface Biomarker {
  /** Biomarker name */
  name: string;

  /** Measured value */
  value: number;

  /** Unit of measurement */
  unit: string;

  /** Normal range */
  normalRange: { min: number; max: number };

  /** Current status */
  status: 'normal' | 'warning' | 'critical';

  /** Measurement timestamp */
  timestamp: Date;
}

/**
 * Clinical signs
 */
export interface ClinicalSigns {
  /** Pain level (0-10) */
  pain: number;

  /** Swelling present */
  swelling: boolean;

  /** Redness present */
  redness: boolean;

  /** Warmth present */
  warmth: boolean;

  /** Discharge present */
  discharge: boolean;

  /** Other observations */
  observations?: string;
}

/**
 * Device metrics for rejection monitoring
 */
export interface DeviceMetrics {
  /** Functionality (0-100%) */
  functionality: number;

  /** Power consumption (mW) */
  powerConsumption: number;

  /** Communication quality (0-100%) */
  communicationQuality: number;

  /** Signal quality (SNR dB) */
  signalQuality?: number;

  /** Impedance (kΩ) */
  impedance?: number;
}

/**
 * Rejection monitoring data
 */
export interface RejectionMonitoringData {
  /** Implant ID */
  implantId: string;

  /** Timestamp */
  timestamp: Date;

  /** Biomarkers */
  biomarkers: {
    crp: number;          // mg/L
    il6: number;          // pg/mL
    temperature: number;  // Celsius
    antibodies: number;   // AU/mL
    tcells?: number;      // % deviation
  };

  /** Clinical signs */
  clinicalSigns: ClinicalSigns;

  /** Device metrics */
  deviceMetrics: DeviceMetrics;

  /** Additional biomarkers */
  additionalBiomarkers?: Biomarker[];
}

/**
 * Rejection status
 */
export interface RejectionStatus {
  /** Overall risk level */
  riskLevel: 'green' | 'yellow' | 'orange' | 'red';

  /** Risk score (0-100) */
  riskScore: number;

  /** Alert parameters */
  alerts: string[];

  /** Clinical assessment needed */
  clinicalAssessmentNeeded: boolean;

  /** Urgency */
  urgency: 'routine' | 'soon' | 'urgent' | 'emergency';

  /** Recommendations */
  recommendations: string[];

  /** Next monitoring date */
  nextMonitoring: Date;
}

// ============================================================================
// Firmware Update Types
// ============================================================================

/**
 * Update type classification
 */
export type UpdateType = 'critical_security' | 'safety_patch' | 'feature_update' | 'optimization';

/**
 * Update information
 */
export interface UpdateInfo {
  /** Update ID */
  id: string;

  /** Version */
  version: string;

  /** Update type */
  type: UpdateType;

  /** Urgency */
  urgency: 'immediate' | 'scheduled' | 'elective' | 'recommended';

  /** Mandatory flag */
  mandatory: boolean;

  /** Release date */
  releaseDate: Date;

  /** Description */
  description: string;

  /** Size (bytes) */
  size: number;

  /** Release notes */
  releaseNotes: string;

  /** Compatibility */
  compatibility: {
    minVersion: string;
    maxVersion: string;
    hardwareRevision: string[];
  };
}

/**
 * Update result
 */
export interface UpdateResult {
  /** Success status */
  success: boolean;

  /** Previous version */
  previousVersion: string;

  /** New version */
  newVersion: string;

  /** Update duration (seconds) */
  duration: number;

  /** Verification passed */
  verified: boolean;

  /** Error message if failed */
  error?: string;

  /** Rollback available */
  rollbackAvailable: boolean;
}

/**
 * Rollback result
 */
export interface RollbackResult {
  /** Success status */
  success: boolean;

  /** Current version */
  currentVersion: string;

  /** Rolled back to version */
  rolledBackToVersion: string;

  /** Rollback duration (seconds) */
  duration: number;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// End-of-Life Types
// ============================================================================

/**
 * Explantation trigger category
 */
export type ExplantCategory = 'device_failure' | 'medical_necessity' | 'upgrade' | 'end_of_service';

/**
 * Explantation parameters
 */
export interface ExplantParams {
  /** Implant ID */
  implantId: string;

  /** Reason category */
  category: ExplantCategory;

  /** Detailed reason */
  reason: string;

  /** Urgency */
  urgency: 'emergency' | 'urgent' | 'elective' | 'scheduled';

  /** Scheduled date */
  scheduledDate?: Date;

  /** Replacement planned */
  replacementPlanned: boolean;

  /** Patient consent */
  patientConsent: boolean;
}

/**
 * Explantation schedule
 */
export interface ExplantSchedule {
  /** Schedule ID */
  id: string;

  /** Implant ID */
  implantId: string;

  /** Scheduled date */
  scheduledDate: Date;

  /** Category */
  category: ExplantCategory;

  /** Status */
  status: 'scheduled' | 'confirmed' | 'in_progress' | 'completed' | 'cancelled';

  /** Surgeon assigned */
  surgeon?: string;

  /** Facility */
  facility?: string;

  /** Pre-op requirements */
  preOpRequirements: string[];

  /** Estimated duration (minutes) */
  estimatedDuration: number;
}

/**
 * Deactivation result
 */
export interface DeactivationResult {
  /** Success status */
  success: boolean;

  /** Implant ID */
  implantId: string;

  /** Deactivation timestamp */
  timestamp: Date;

  /** Data extracted */
  dataExtracted: boolean;

  /** Safe mode activated */
  safeModeActivated: boolean;

  /** Power status */
  powerStatus: 'off' | 'minimal' | 'safe_mode';

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Device Information Types
// ============================================================================

/**
 * Device information
 */
export interface DeviceInfo {
  /** Device ID */
  id: string;

  /** Implant type */
  type: ImplantType;

  /** Manufacturer */
  manufacturer: string;

  /** Model */
  model: string;

  /** Serial number */
  serialNumber: string;

  /** Firmware version */
  firmwareVersion: string;

  /** Hardware revision */
  hardwareRevision: string;

  /** Manufacturing date */
  manufactureDate: Date;

  /** Implantation date */
  implantDate?: Date;

  /** Power source */
  powerSource: PowerSource;

  /** Integration level */
  integrationLevel: IntegrationLevel;

  /** Anatomical location */
  location: string;
}

/**
 * Patient information (anonymized)
 */
export interface PatientInfo {
  /** Patient ID (anonymized) */
  id: string;

  /** Age at implantation */
  age: number;

  /** Immune status */
  immuneStatus: 'normal' | 'compromised' | 'enhanced';

  /** Existing implants */
  existingImplants: string[];

  /** Known allergies */
  allergies: string[];

  /** Medical conditions */
  conditions: string[];
}

/**
 * Location information
 */
export interface LocationInfo {
  /** Anatomical site */
  anatomicalSite: string;

  /** Integration level */
  integrationLevel: IntegrationLevel;

  /** Tissue type */
  tissueType: TissueType;

  /** Side (if applicable) */
  side?: 'left' | 'right' | 'bilateral';

  /** Depth (mm) */
  depth?: number;
}

// ============================================================================
// Assessment Types
// ============================================================================

/**
 * Implant assessment input
 */
export interface ImplantAssessmentInput {
  /** Device information */
  device: {
    type: ImplantType;
    manufacturer: string;
    model: string;
    powerSource: PowerSource;
  };

  /** Patient information */
  patient: {
    age: number;
    immuneStatus: 'normal' | 'compromised' | 'enhanced';
    existingImplants: string[];
  };

  /** Location information */
  location: {
    anatomicalSite: string;
    integrationLevel: IntegrationLevel;
    tissueType: TissueType;
  };
}

/**
 * Overall risk assessment
 */
export interface OverallRiskAssessment {
  /** Risk score (0-100) */
  score: number;

  /** Risk level */
  level: 'low' | 'moderate' | 'high' | 'critical';

  /** Risk factors */
  factors: {
    surgical: number;
    biological: number;
    technical: number;
    longTerm: number;
  };

  /** Recommendations */
  recommendations: string[];
}

/**
 * Comprehensive implant assessment
 */
export interface ImplantAssessment {
  /** Classification */
  classification: ImplantClassification;

  /** Biocompatibility */
  biocompatibility: {
    class: BiocompatibilityClass;
    requiredTests: BiocompatibilityTest[];
    assessment: string;
  };

  /** Power management plan */
  powerPlan: {
    strategy: 'battery' | 'wireless' | 'hybrid';
    estimatedLifetime: number; // years
    chargingFrequency: string;
  };

  /** Monitoring plan */
  monitoringPlan: {
    frequency: 'continuous' | 'daily' | 'weekly' | 'biweekly' | 'monthly';
    duration: number; // months
    biomarkers: string[];
  };

  /** Overall risk */
  overallRisk: OverallRiskAssessment;

  /** Approval status */
  approvalStatus: 'approved' | 'conditional' | 'rejected';

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Vital Metrics Types
// ============================================================================

/**
 * Vital metrics
 */
export interface VitalMetrics {
  /** Timestamp */
  timestamp: Date;

  /** Implant ID */
  implantId: string;

  /** Functionality (0-100%) */
  functionality: number;

  /** Power level (0-100%) */
  powerLevel: number;

  /** Signal quality (0-100%) */
  signalQuality: number;

  /** Operating temperature (Celsius) */
  temperature: number;

  /** Communication status */
  communicationStatus: 'active' | 'intermittent' | 'lost';

  /** Alerts */
  alerts: PowerAlert[];

  /** Status summary */
  status: 'normal' | 'warning' | 'critical';
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Implant-related constants
 */
export const IMPLANT_CONSTANTS = {
  /** Integration level descriptions */
  INTEGRATION_LEVELS: {
    1: 'Subcutaneous',
    2: 'Muscular',
    3: 'Osseous',
    4: 'Neural',
    5: 'Cognitive',
  },

  /** Biomarker thresholds */
  BIOMARKER_THRESHOLDS: {
    CRP: { normal: 3, warning: 10, critical: Infinity },
    IL6: { normal: 5, warning: 20, critical: Infinity },
    TEMPERATURE: { normal: 37, warning: 38, critical: 39 },
    ANTIBODIES: { normal: 100, warning: 500, critical: Infinity },
  },

  /** Power thresholds */
  POWER_THRESHOLDS: {
    LOW: 20,
    CRITICAL: 5,
  },

  /** Update urgency timeframes (hours) */
  UPDATE_URGENCY: {
    immediate: 24,
    scheduled: 168, // 1 week
    elective: 720,  // 30 days
    recommended: 2160, // 90 days
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Implant error codes
 */
export enum ImplantErrorCode {
  CLASSIFICATION_FAILED = 'I001',
  BIOCOMPATIBILITY_FAILED = 'I002',
  POWER_SYSTEM_ERROR = 'I003',
  COMMUNICATION_ERROR = 'I004',
  REJECTION_DETECTED = 'I005',
  UPDATE_FAILED = 'I006',
  DEVICE_MALFUNCTION = 'I007',
  SAFETY_THRESHOLD_EXCEEDED = 'I008',
}

/**
 * Implant error class
 */
export class ImplantError extends Error {
  constructor(
    public code: ImplantErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ImplantError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  ImplantType,
  BiocompatibilityClass,
  PowerSource,
  IntegrationLevel,
  TissueType,
  ContactDuration,
  ImplantClassificationInput,
  ImplantClassification,
  MaterialEntry,
  BiocompatibilityTest,
  TestStatus,
  BioTestResult,
  BioAssessmentParams,
  BiocompatibilityResult,
  ChargingMethod,
  PowerConfig,
  PowerMetrics,
  PowerAlert,
  CommProtocol,
  SecurityLevel,
  ConnectionConfig,
  Connection,
  ImplantData,
  TransmissionResult,
  Biomarker,
  ClinicalSigns,
  DeviceMetrics,
  RejectionMonitoringData,
  RejectionStatus,
  UpdateType,
  UpdateInfo,
  UpdateResult,
  RollbackResult,
  ExplantCategory,
  ExplantParams,
  ExplantSchedule,
  DeactivationResult,
  DeviceInfo,
  PatientInfo,
  LocationInfo,
  ImplantAssessmentInput,
  OverallRiskAssessment,
  ImplantAssessment,
  VitalMetrics,
};

export { IMPLANT_CONSTANTS, ImplantErrorCode, ImplantError };
