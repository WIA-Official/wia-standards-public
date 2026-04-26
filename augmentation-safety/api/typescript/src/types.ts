/**
 * WIA-AUG-013: Augmentation Safety - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Safety Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Safety Classification Types
// ============================================================================

/**
 * Safety classification levels for augmentation devices
 */
export type SafetyLevel = 'Level1' | 'Level2' | 'Level3' | 'Level4' | 'Level5';

/**
 * Classification criteria input for device assessment
 */
export interface ClassificationInput {
  /** Depth of invasiveness (1-10) */
  invasiveness: number;

  /** Proximity to vital organs (1-10) */
  vitalProximity: number;

  /** Reversibility potential (1-10, 10 = irreversible) */
  reversibility: number;

  /** Severity of failure consequences (1-10) */
  failureConsequence: number;

  /** Level of biological interaction (1-10) */
  biologicalInteraction: number;
}

/**
 * Classification result with detailed breakdown
 */
export interface ClassificationResult {
  /** Assigned safety level */
  level: SafetyLevel;

  /** Raw classification score (0-100) */
  score: number;

  /** Score breakdown by criteria */
  breakdown: {
    invasiveness: number;
    vitalProximity: number;
    reversibility: number;
    failureConsequence: number;
    biologicalInteraction: number;
  };

  /** Weighted contribution of each criterion */
  weights: {
    invasiveness: number;
    vitalProximity: number;
    reversibility: number;
    failureConsequence: number;
    biologicalInteraction: number;
  };

  /** Regulatory requirements for this level */
  requirements: string[];
}

// ============================================================================
// Risk Assessment Types
// ============================================================================

/**
 * Risk Priority Number (RPN) components
 */
export interface RPNComponents {
  /** Severity of failure impact (1-10) */
  severity: number;

  /** Probability of occurrence (1-10) */
  occurrence: number;

  /** Difficulty of detection (1-10) */
  detection: number;
}

/**
 * Risk level classification
 */
export type RiskLevel = 'Low' | 'Moderate' | 'High' | 'Critical' | 'Unacceptable';

/**
 * Failure mode record for FMEA
 */
export interface FailureMode {
  /** Unique failure mode identifier */
  id: string;

  /** Component or subsystem affected */
  component: string;

  /** Type of failure */
  failureType: string;

  /** Root cause of failure */
  cause: string;

  /** Effect on user/system */
  effect: string;

  /** RPN components */
  rpn: RPNComponents;

  /** Calculated RPN value */
  rpnValue: number;

  /** Proposed mitigation strategy */
  mitigation: string;

  /** Responsible party */
  owner: string;

  /** Target completion date */
  deadline?: Date;

  /** Status of mitigation */
  status: 'Open' | 'InProgress' | 'Closed' | 'Deferred';
}

/**
 * Risk assessment result
 */
export interface RiskAssessment {
  /** Assessment identifier */
  id: string;

  /** Device being assessed */
  deviceId: string;

  /** Assessment date */
  date: Date;

  /** All identified failure modes */
  failureModes: FailureMode[];

  /** Overall risk level */
  overallRisk: RiskLevel;

  /** Total RPN across all failure modes */
  totalRPN: number;

  /** Maximum individual RPN */
  maxRPN: number;

  /** Number of unacceptable risks */
  unacceptableCount: number;

  /** Assessment passed/failed */
  passed: boolean;

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Biocompatibility Types
// ============================================================================

/**
 * ISO 10993 test categories
 */
export type BiocompatibilityTest =
  | 'Cytotoxicity'
  | 'Sensitization'
  | 'Irritation'
  | 'SystemicToxicity'
  | 'Genotoxicity'
  | 'Implantation'
  | 'Carcinogenicity'
  | 'ReproductiveToxicity';

/**
 * Test result status
 */
export type TestStatus = 'Pass' | 'Fail' | 'Pending' | 'NotRequired';

/**
 * Biocompatibility test result
 */
export interface BioTestResult {
  /** Test type */
  test: BiocompatibilityTest;

  /** Test result status */
  status: TestStatus;

  /** ISO standard reference */
  standard: string;

  /** Test date */
  date?: Date;

  /** Testing laboratory */
  lab?: string;

  /** Certificate/report number */
  reportNumber?: string;

  /** Notes or observations */
  notes?: string;
}

/**
 * Material classification for biocompatibility
 */
export interface MaterialClassification {
  /** Material category */
  category: 'Inert' | 'Bioactive' | 'Biodegradable';

  /** Contact duration */
  contactDuration: 'Limited' | 'Prolonged' | 'Permanent';

  /** Contact type */
  contactType: 'Surface' | 'External' | 'Implant';

  /** Type of tissue contact */
  tissueContact: 'Skin' | 'Mucosal' | 'Bone' | 'Blood' | 'Neural';
}

/**
 * Biocompatibility assessment result
 */
export interface BiocompatibilityResult {
  /** Device identifier */
  deviceId: string;

  /** Material classification */
  material: MaterialClassification;

  /** Required tests based on classification */
  requiredTests: BiocompatibilityTest[];

  /** Test results */
  testResults: BioTestResult[];

  /** All required tests passed */
  compliant: boolean;

  /** Pending tests */
  pendingTests: BiocompatibilityTest[];

  /** Failed tests */
  failedTests: BiocompatibilityTest[];
}

// ============================================================================
// Safety Testing Types
// ============================================================================

/**
 * Testing phase
 */
export type TestPhase = 'Bench' | 'InVitro' | 'InVivo' | 'PhaseI' | 'PhaseII' | 'PhaseIII' | 'PostMarket';

/**
 * Clinical trial data
 */
export interface ClinicalTrialData {
  /** Trial identifier */
  trialId: string;

  /** Trial phase */
  phase: TestPhase;

  /** Number of subjects */
  subjects: number;

  /** Trial duration in months */
  duration: number;

  /** Primary endpoints */
  primaryEndpoints: string[];

  /** Secondary endpoints */
  secondaryEndpoints: string[];

  /** Adverse events */
  adverseEvents: {
    serious: number;
    moderate: number;
    minor: number;
  };

  /** Success rate */
  successRate: number;

  /** Statistical confidence */
  confidence: number;

  /** Trial status */
  status: 'Planned' | 'Recruiting' | 'Active' | 'Completed' | 'Terminated';
}

// ============================================================================
// Emergency Procedures Types
// ============================================================================

/**
 * Emergency category classification
 */
export type EmergencyCategory = 'A' | 'B' | 'C' | 'D';

/**
 * Safe mode configuration
 */
export interface SafeMode {
  /** How safe mode is triggered */
  trigger: 'manual' | 'automatic' | 'remote';

  /** Actions taken in safe mode */
  actions: string[];

  /** Power state during safe mode */
  powerState: 'off' | 'minimal' | 'backup';

  /** Communication state */
  communication: 'active' | 'beacon' | 'silent';

  /** Can safe mode be reversed */
  reversible: boolean;

  /** Timeout in seconds before escalation */
  timeout: number;
}

/**
 * Emergency event record
 */
export interface EmergencyEvent {
  /** Event identifier */
  id: string;

  /** Device identifier */
  deviceId: string;

  /** Event timestamp */
  timestamp: Date;

  /** Emergency category */
  category: EmergencyCategory;

  /** Event description */
  description: string;

  /** Actions taken */
  actions: string[];

  /** Resolution status */
  resolved: boolean;

  /** Resolution timestamp */
  resolvedAt?: Date;

  /** Root cause (if determined) */
  rootCause?: string;
}

// ============================================================================
// Long-term Monitoring Types
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
  normal: { min: number; max: number };

  /** Current status */
  status: 'normal' | 'warning' | 'critical';
}

/**
 * Monitoring data point
 */
export interface MonitoringData {
  /** Device identifier */
  deviceId: string;

  /** Measurement timestamp */
  timestamp: Date;

  /** Device metrics */
  metrics: {
    /** Battery/power level (0-100%) */
    powerLevel: number;

    /** Signal quality (0-100%) */
    signalQuality: number;

    /** Operating temperature (Celsius) */
    operatingTemperature: number;

    /** Impedance per channel (Ohms) */
    impedance: number[];

    /** Overall functionality (0-100%) */
    functionality: number;

    /** Biological markers */
    biomarkers: Biomarker[];
  };

  /** Active alerts */
  alerts: Alert[];

  /** User feedback */
  userFeedback?: {
    satisfaction: number;
    issues: string[];
    comments: string;
  };
}

/**
 * Alert record
 */
export interface Alert {
  /** Alert identifier */
  id: string;

  /** Alert type */
  type: 'power' | 'signal' | 'temperature' | 'impedance' | 'functionality' | 'biomarker';

  /** Severity level */
  severity: 'info' | 'warning' | 'critical';

  /** Alert message */
  message: string;

  /** Alert timestamp */
  timestamp: Date;

  /** Whether alert has been acknowledged */
  acknowledged: boolean;
}

// ============================================================================
// Device Information Types
// ============================================================================

/**
 * Device information
 */
export interface DeviceInfo {
  /** Device identifier */
  id: string;

  /** Device type */
  type: string;

  /** Manufacturer */
  manufacturer: string;

  /** Model name */
  model: string;

  /** Serial number */
  serialNumber: string;

  /** Firmware version */
  firmwareVersion: string;

  /** Manufacturing date */
  manufactureDate: Date;

  /** Implantation date (if applicable) */
  implantDate?: Date;

  /** Assigned safety level */
  safetyLevel: SafetyLevel;
}

/**
 * Patient information (anonymized)
 */
export interface PatientInfo {
  /** Patient identifier (anonymized) */
  id: string;

  /** Age at implantation */
  age: number;

  /** General health status */
  healthStatus: 'excellent' | 'good' | 'fair' | 'poor';

  /** Existing medical conditions */
  existingConditions: string[];

  /** Known allergies */
  allergies: string[];
}

// ============================================================================
// Safety Report Types
// ============================================================================

/**
 * Comprehensive safety report
 */
export interface SafetyReport {
  /** Report identifier */
  id: string;

  /** Report generation date */
  generatedAt: Date;

  /** Device information */
  device: DeviceInfo;

  /** Classification result */
  classification: ClassificationResult;

  /** Risk assessment */
  riskAssessment: RiskAssessment;

  /** Biocompatibility results */
  biocompatibility: BiocompatibilityResult;

  /** Clinical trial data */
  clinicalTrials: ClinicalTrialData[];

  /** Monitoring summary */
  monitoringSummary: {
    duration: number; // days
    dataPoints: number;
    averageMetrics: MonitoringData['metrics'];
    alertCount: number;
    criticalAlerts: number;
  };

  /** Overall compliance status */
  compliant: boolean;

  /** Certification status */
  certificationStatus: 'pending' | 'approved' | 'rejected' | 'expired';

  /** Certification valid until */
  validUntil?: Date;

  /** Recommendations and findings */
  recommendations: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Safety-related constants
 */
export const SAFETY_CONSTANTS = {
  /** Classification score thresholds */
  CLASSIFICATION_THRESHOLDS: {
    LEVEL1_MAX: 15,
    LEVEL2_MAX: 30,
    LEVEL3_MAX: 50,
    LEVEL4_MAX: 75,
    LEVEL5_MAX: 100,
  },

  /** RPN thresholds */
  RPN_THRESHOLDS: {
    LOW_MAX: 50,
    MODERATE_MAX: 100,
    HIGH_MAX: 200,
    CRITICAL_MAX: 500,
    UNACCEPTABLE_MAX: 1000,
  },

  /** Classification weights */
  CLASSIFICATION_WEIGHTS: {
    invasiveness: 0.25,
    vitalProximity: 0.20,
    reversibility: 0.15,
    failureConsequence: 0.20,
    biologicalInteraction: 0.20,
  },

  /** Minimum required stability */
  MIN_STABILITY: 0.95,

  /** Alert thresholds */
  ALERT_THRESHOLDS: {
    POWER_WARNING: 20,
    POWER_CRITICAL: 5,
    SIGNAL_WARNING: 70,
    SIGNAL_CRITICAL: 50,
    TEMP_WARNING_HIGH: 38,
    TEMP_WARNING_LOW: 35,
    TEMP_CRITICAL_HIGH: 40,
    TEMP_CRITICAL_LOW: 33,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Safety error codes
 */
export enum SafetyErrorCode {
  CLASSIFICATION_INVALID_INPUT = 'S001',
  RISK_UNACCEPTABLE = 'S002',
  BIOCOMPATIBILITY_FAILED = 'S003',
  CLINICAL_TRIAL_FAILED = 'S004',
  MONITORING_DATA_MISSING = 'S005',
  EMERGENCY_TRIGGERED = 'S006',
  CERTIFICATION_EXPIRED = 'S007',
  DEVICE_MALFUNCTION = 'S008',
}

/**
 * Safety error class
 */
export class SafetyError extends Error {
  constructor(
    public code: SafetyErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'SafetyError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  SafetyLevel,
  ClassificationInput,
  ClassificationResult,
  RPNComponents,
  RiskLevel,
  FailureMode,
  RiskAssessment,
  BiocompatibilityTest,
  TestStatus,
  BioTestResult,
  MaterialClassification,
  BiocompatibilityResult,
  TestPhase,
  ClinicalTrialData,
  EmergencyCategory,
  SafeMode,
  EmergencyEvent,
  Biomarker,
  MonitoringData,
  Alert,
  DeviceInfo,
  PatientInfo,
  SafetyReport,
};

export { SAFETY_CONSTANTS, SafetyErrorCode, SafetyError };
