/**
 * WIA-AUG-010: Artificial Organ - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Organ Type Classification
// ============================================================================

/**
 * Primary artificial organ types
 */
export type OrganType =
  | 'HEART'
  | 'KIDNEY'
  | 'LIVER'
  | 'LUNG'
  | 'PANCREAS'
  | 'BLADDER'
  | 'INTESTINE'
  | 'SKIN';

/**
 * Technology approaches for artificial organs
 */
export type TechnologyType =
  | 'MECHANICAL'
  | 'BIOARTIFICIAL'
  | 'BIOPRINTED'
  | 'XENOTRANSPLANT'
  | 'HYBRID';

/**
 * Power system types
 */
export type PowerSystemType =
  | 'BATTERY'
  | 'TET'
  | 'BIOFUEL'
  | 'HYBRID'
  | 'EXTERNAL';

/**
 * Operational state of artificial organ
 */
export type OperationalState =
  | 'STANDBY'
  | 'ACTIVE'
  | 'DEGRADED'
  | 'ERROR'
  | 'MAINTENANCE'
  | 'FAILSAFE';

/**
 * Performance levels
 */
export type PerformanceLevel =
  | 'SUBOPTIMAL'
  | 'ADEQUATE'
  | 'OPTIMAL'
  | 'SUPERIOR';

/**
 * Rejection risk levels
 */
export type RejectionRiskLevel =
  | 'LOW'
  | 'MODERATE'
  | 'HIGH'
  | 'CRITICAL';

/**
 * Failsafe modes
 */
export type FailsafeMode =
  | 'BACKUP'
  | 'EMERGENCY'
  | 'MINIMAL'
  | 'SHUTDOWN';

// ============================================================================
// Organ Classification Types
// ============================================================================

/**
 * Input for organ classification
 */
export interface OrganInput {
  /** Primary organ function type */
  organType: OrganType;

  /** Technology approach */
  technologyType: TechnologyType;

  /** Target function level (% of natural organ) */
  targetFunction: number;

  /** Power system type */
  powerSystem: PowerSystemType;

  /** Biocompatibility rating (0-1) */
  biocompatibilityRating: number;

  /** Invasiveness level (1-10) */
  invasiveness?: number;

  /** Reversibility flag */
  reversible?: boolean;

  /** Expected lifespan (years) */
  expectedLifespan?: number;
}

/**
 * Organ classification result
 */
export interface OrganClassification {
  /** Organ type */
  type: OrganType;

  /** Technology category */
  technology: TechnologyType;

  /** Performance level */
  performanceLevel: PerformanceLevel;

  /** Safety classification */
  safetyClass: string;

  /** Power system */
  powerSystem: PowerSystemType;

  /** Biocompatibility score */
  biocompatibility: number;

  /** Reversibility status */
  reversible: boolean;

  /** Recommended monitoring frequency */
  monitoringFrequency: string;
}

// ============================================================================
// Function Monitoring Types
// ============================================================================

/**
 * Output metrics
 */
export interface OutputMetrics {
  /** Current output value */
  value: number;

  /** Unit of measurement */
  unit: string;

  /** Target value */
  target: number;

  /** Percentage of target achieved */
  percentOfTarget: number;

  /** Trend (positive = improving) */
  trend?: number;
}

/**
 * Efficiency metrics
 */
export interface EfficiencyMetrics {
  /** Overall efficiency (0-1) */
  value: number;

  /** Energy input */
  energyInput: number;

  /** Useful output */
  usefulOutput: number;

  /** Energy unit */
  energyUnit: string;
}

/**
 * Biomarker data
 */
export interface BiomarkerData {
  /** Blood chemistry values */
  bloodChemistry?: Record<string, number>;

  /** Metabolic indicators */
  metabolicIndicators?: Record<string, number>;

  /** Organ-specific markers */
  organSpecific?: Record<string, number>;

  /** Timestamp of measurement */
  timestamp: Date;
}

/**
 * Physiological metrics
 */
export interface PhysiologicalMetrics {
  /** Temperature (°C) */
  temperature?: number;

  /** Pressure (mmHg or cmH₂O) */
  pressure?: number;

  /** Flow rate (ml/min) */
  flow?: number;

  /** Resistance */
  resistance?: number;

  /** Additional metrics */
  other?: Record<string, number>;
}

/**
 * Comprehensive function metrics
 */
export interface FunctionMetrics {
  /** Organ identifier */
  organId: string;

  /** Timestamp */
  timestamp: Date;

  /** Output metrics */
  output: OutputMetrics;

  /** Efficiency metrics */
  efficiency: EfficiencyMetrics;

  /** Biomarker data */
  biomarkers?: BiomarkerData;

  /** Physiological metrics */
  physiology?: PhysiologicalMetrics;

  /** Battery level (0-100) */
  batteryLevel?: number;

  /** Operational state */
  operationalState: OperationalState;
}

/**
 * Monitoring result
 */
export interface MonitoringResult {
  /** Organ identifier */
  organId: string;

  /** Current status */
  status: OperationalState;

  /** Performance assessment */
  performance: PerformanceLevel;

  /** Function metrics */
  metrics: FunctionMetrics;

  /** Active alerts */
  alerts: Alert[];

  /** Recommendations */
  recommendations: string[];

  /** Next monitoring time */
  nextMonitoring: Date;
}

// ============================================================================
// Biocompatibility Types
// ============================================================================

/**
 * Tissue integration data
 */
export interface TissueIntegration {
  /** Capsule thickness (mm) */
  capsuleThickness: number;

  /** Vascularization (vessels/mm²) */
  vascularization: number;

  /** Cell infiltration (cells/mm²) */
  cellInfiltration: number;

  /** Fibrosis score (0-4) */
  fibrosis: number;

  /** Adhesion score (0-4) */
  adhesions: number;

  /** Integration quality (0-1) */
  integrationQuality?: number;
}

/**
 * Immune response data
 */
export interface ImmuneResponse {
  /** CRP level (mg/L) */
  CRP: number;

  /** ESR (mm/h) */
  ESR: number;

  /** WBC count (cells/μL) */
  WBC: number;

  /** Lymphocyte percentage */
  lymphocytes?: number;

  /** Cytokine levels */
  cytokines?: {
    IL6?: number;
    TNFalpha?: number;
    IL1beta?: number;
    IFNgamma?: number;
  };

  /** Complement levels */
  complement?: {
    C3?: number;
    C4?: number;
    CH50?: number;
  };
}

/**
 * Material safety assessment
 */
export interface MaterialSafety {
  /** Cytotoxicity score (0-1, 1=safe) */
  cytotoxicity: number;

  /** Sensitization score (0-1, 1=safe) */
  sensitization: number;

  /** Irritation score (0-1, 1=safe) */
  irritation: number;

  /** Systemic toxicity score (0-1, 1=safe) */
  systemicToxicity: number;

  /** Genotoxicity score (0-1, 1=safe) */
  genotoxicity?: number;

  /** Overall safety score */
  overallScore?: number;
}

/**
 * Functional stability assessment
 */
export interface FunctionalStability {
  /** Performance consistency (0-1) */
  consistency: number;

  /** Output variability (coefficient of variation) */
  variability: number;

  /** Degradation rate (% per year) */
  degradationRate: number;

  /** Stability score (0-1) */
  stabilityScore?: number;
}

/**
 * Biocompatibility assessment data
 */
export interface BiocompatibilityData {
  /** Tissue integration */
  tissueIntegration: TissueIntegration;

  /** Immune response */
  immuneResponse: ImmuneResponse;

  /** Material safety */
  materialSafety: MaterialSafety;

  /** Functional stability */
  functionalStability: FunctionalStability;

  /** Assessment date */
  assessmentDate: Date;
}

/**
 * Biocompatibility score result
 */
export interface BiocompatibilityScore {
  /** Overall biocompatibility score (0-1) */
  overallScore: number;

  /** Tissue integration score (0-1) */
  tissueIntegrationScore: number;

  /** Immune response score (0-1, 1=minimal response) */
  immuneResponseScore: number;

  /** Material safety score (0-1) */
  materialSafetyScore: number;

  /** Functional stability score (0-1) */
  functionalStabilityScore: number;

  /** Classification */
  classification: 'EXCELLENT' | 'GOOD' | 'ACCEPTABLE' | 'POOR';

  /** Warnings */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Rejection Detection Types
// ============================================================================

/**
 * Immune markers for rejection detection
 */
export interface ImmuneMarkers {
  /** C-reactive protein (mg/L) */
  CRP: number;

  /** Erythrocyte sedimentation rate (mm/h) */
  ESR: number;

  /** White blood cell count (cells/μL) */
  WBC: number;

  /** Lymphocyte percentage */
  lymphocytes?: number;

  /** Cytokine levels (pg/ml) */
  cytokines?: {
    IL6: number;
    TNFalpha: number;
    IL1beta?: number;
    IFNgamma?: number;
  };

  /** Complement levels */
  complement?: {
    C3: number;
    C4: number;
    CH50?: number;
  };
}

/**
 * Antibody levels
 */
export interface AntibodyLevels {
  /** Donor-specific antibodies */
  DSA?: number;

  /** Panel reactive antibodies (%) */
  PRA?: number;

  /** IgG level */
  IgG?: number;

  /** IgM level */
  IgM?: number;

  /** Overall antibody score (0-1) */
  overallScore: number;
}

/**
 * Performance trend data
 */
export interface PerformanceTrend {
  /** Current output */
  currentOutput: number;

  /** Baseline output */
  baselineOutput: number;

  /** 7-day trend (% change) */
  trend7days: number;

  /** 30-day trend (% change) */
  trend30days: number;

  /** Variability (coefficient of variation) */
  variability: number;

  /** Efficiency trend */
  efficiencyTrend: number;
}

/**
 * Rejection detection input
 */
export interface RejectionData {
  /** Organ identifier */
  organId: string;

  /** Immune markers */
  immuneMarkers: ImmuneMarkers;

  /** Antibody levels */
  antibodyLevels: AntibodyLevels;

  /** Performance trend */
  performanceTrend: PerformanceTrend;

  /** Inflammation indicators */
  inflammationIndicators?: Record<string, number>;

  /** Assessment date */
  assessmentDate: Date;
}

/**
 * Rejection assessment result
 */
export interface RejectionAssessment {
  /** Organ identifier */
  organId: string;

  /** Rejection risk score (0-1) */
  riskScore: number;

  /** Risk level */
  level: RejectionRiskLevel;

  /** Immune marker contribution */
  immuneMarkerScore: number;

  /** Performance degradation contribution */
  performanceDegradationScore: number;

  /** Inflammation indicator contribution */
  inflammationScore: number;

  /** Antibody level contribution */
  antibodyScore: number;

  /** Recommended action */
  recommendation: string;

  /** Alert priority */
  priority: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

  /** Detailed findings */
  findings: string[];

  /** Follow-up interval */
  followUpInterval: string;
}

// ============================================================================
// Performance Optimization Types
// ============================================================================

/**
 * Performance targets
 */
export interface PerformanceTargets {
  /** Target output */
  targetOutput: number;

  /** Target efficiency (0-1) */
  targetEfficiency: number;

  /** Maximum power consumption */
  maxPowerConsumption?: number;

  /** Response time (milliseconds) */
  responseTime?: number;

  /** Physiological demand */
  physiologicalDemand?: number;
}

/**
 * Control parameters
 */
export interface ControlParameters {
  /** Current output */
  currentOutput: number;

  /** Energy input */
  energyInput: number;

  /** Control mode */
  controlMode: 'FIXED' | 'ADAPTIVE' | 'PREDICTIVE';

  /** Safety constraints */
  constraints: {
    maxOutput: number;
    minOutput: number;
    maxPower: number;
  };
}

/**
 * Optimization result
 */
export interface OptimizationResult {
  /** Optimized output setting */
  optimizedOutput: number;

  /** Expected efficiency */
  expectedEfficiency: number;

  /** Power consumption */
  powerConsumption: number;

  /** Control adjustments */
  adjustments: Record<string, number>;

  /** Performance improvement (%) */
  performanceImprovement: number;

  /** Estimated duration */
  estimatedDuration?: string;
}

// ============================================================================
// Maintenance and Service Types
// ============================================================================

/**
 * Service schedule
 */
export interface ServiceSchedule {
  /** Next service date */
  nextServiceDate: Date;

  /** Service type */
  serviceType: 'PREVENTIVE' | 'PREDICTIVE' | 'CORRECTIVE' | 'EMERGENCY';

  /** Urgency level */
  urgency: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

  /** Reason for service */
  reason: string;

  /** Estimated downtime (hours) */
  estimatedDowntime: number;

  /** Required components */
  requiredComponents?: string[];

  /** Preparation instructions */
  preparationInstructions?: string[];
}

/**
 * Maintenance prediction
 */
export interface MaintenancePrediction {
  /** Operating hours */
  operatingHours: number;

  /** Cycle count */
  cycleCount: number;

  /** Estimated hours to service */
  hoursToService: number;

  /** Estimated service date */
  estimatedServiceDate: Date;

  /** Confidence level (0-1) */
  confidenceLevel: number;

  /** Wear indicators */
  wearIndicators: Record<string, number>;

  /** Performance degradation rate */
  degradationRate: number;

  /** Recommended actions */
  recommendedActions: string[];
}

/**
 * Component wear assessment
 */
export interface WearAssessment {
  /** Component name */
  component: string;

  /** Wear level (0-1, 1=fully worn) */
  wearLevel: number;

  /** Expected remaining life (hours) */
  remainingLife: number;

  /** Replacement recommended */
  replacementRecommended: boolean;

  /** Replacement urgency */
  urgency: 'LOW' | 'MEDIUM' | 'HIGH' | 'IMMEDIATE';
}

// ============================================================================
// Failsafe and Emergency Types
// ============================================================================

/**
 * Failsafe trigger types
 */
export type FailsafeTrigger =
  | 'SYSTEM_FAILURE'
  | 'POWER_LOSS'
  | 'PERFORMANCE_DEGRADATION'
  | 'REJECTION'
  | 'MANUAL_ACTIVATION';

/**
 * Emergency situation types
 */
export type EmergencySituation =
  | 'COMPLETE_FAILURE'
  | 'PARTIAL_FAILURE'
  | 'POWER_CRITICAL'
  | 'REJECTION_CRITICAL'
  | 'INFECTION';

/**
 * Failsafe status
 */
export interface FailsafeStatus {
  /** Failsafe mode active */
  active: boolean;

  /** Current mode */
  mode: FailsafeMode;

  /** Trigger that activated failsafe */
  trigger: FailsafeTrigger;

  /** Backup system status */
  backupSystemStatus: 'ACTIVE' | 'STANDBY' | 'UNAVAILABLE';

  /** Estimated backup duration (minutes) */
  backupDuration: number;

  /** Actions taken */
  actionsTaken: string[];

  /** Next steps */
  nextSteps: string[];

  /** Emergency contacts notified */
  emergencyContactsNotified: boolean;
}

/**
 * Emergency response
 */
export interface EmergencyResponse {
  /** Recommended action */
  action: 'IMMEDIATE_MEDICAL_INTERVENTION' | 'URGENT_MEDICAL_CONTACT' | 'MONITOR_CLOSELY' | 'ROUTINE_FOLLOW_UP';

  /** Notifications to send */
  notifications: Array<'PATIENT' | 'EMERGENCY_CONTACT' | 'MEDICAL_TEAM' | 'EMERGENCY_SERVICES'>;

  /** Device operating mode */
  deviceMode: 'EMERGENCY_MODE' | 'BACKUP_MODE' | 'NORMAL_MODE';

  /** Instructions for patient */
  instructions: string;

  /** Estimated time until intervention needed */
  timeToIntervention?: number;

  /** Alternative support options */
  alternativeSupport?: string[];

  /** Emergency protocol reference */
  protocolReference?: string;
}

// ============================================================================
// Status and Monitoring Types
// ============================================================================

/**
 * Alert types
 */
export interface Alert {
  /** Alert identifier */
  id: string;

  /** Alert type */
  type:
    | 'POWER'
    | 'PERFORMANCE'
    | 'REJECTION'
    | 'BIOCOMPATIBILITY'
    | 'MAINTENANCE'
    | 'SAFETY'
    | 'OTHER';

  /** Severity */
  severity: 'INFO' | 'WARNING' | 'CRITICAL';

  /** Alert message */
  message: string;

  /** Timestamp */
  timestamp: Date;

  /** Acknowledged */
  acknowledged: boolean;

  /** Recommended action */
  recommendedAction?: string;
}

/**
 * Organ status
 */
export interface OrganStatus {
  /** Organ identifier */
  organId: string;

  /** Timestamp */
  timestamp: Date;

  /** Operational state */
  operationalState: OperationalState;

  /** Organ type */
  organType: OrganType;

  /** Technology type */
  technologyType: TechnologyType;

  /** Function metrics */
  functionMetrics: FunctionMetrics;

  /** Biocompatibility score */
  biocompatibilityScore?: number;

  /** Rejection risk */
  rejectionRisk?: RejectionRiskLevel;

  /** Active alerts */
  alerts: Alert[];

  /** Battery level (0-100) */
  batteryLevel?: number;

  /** Power source */
  powerSource?: PowerSystemType;

  /** Next scheduled service */
  nextService?: Date;

  /** Operating hours */
  operatingHours?: number;

  /** Cycle count */
  cycleCount?: number;
}

/**
 * Historical data query
 */
export interface TimeRange {
  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Data resolution */
  resolution?: 'MINUTE' | 'HOUR' | 'DAY' | 'WEEK';
}

/**
 * Historical data
 */
export interface HistoricalData {
  /** Organ identifier */
  organId: string;

  /** Time range */
  timeRange: TimeRange;

  /** Data points */
  dataPoints: Array<{
    timestamp: Date;
    metrics: FunctionMetrics;
  }>;

  /** Summary statistics */
  summary: {
    averageOutput: number;
    averageEfficiency: number;
    minOutput: number;
    maxOutput: number;
    alertCount: number;
    uptimePercentage: number;
  };
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Biocompatibility thresholds
 */
export const BIOCOMPATIBILITY_THRESHOLDS = {
  EXCELLENT: 0.90,
  GOOD: 0.75,
  ACCEPTABLE: 0.60,
  POOR: 0.0,
} as const;

/**
 * Rejection risk thresholds
 */
export const REJECTION_RISK_THRESHOLDS = {
  LOW: 0.40,
  MODERATE: 0.70,
  HIGH: 0.85,
  CRITICAL: 1.0,
} as const;

/**
 * Performance thresholds
 */
export const PERFORMANCE_THRESHOLDS = {
  SUBOPTIMAL: 0.60,
  ADEQUATE: 0.75,
  OPTIMAL: 0.90,
  SUPERIOR: 1.0,
} as const;

/**
 * Minimum performance requirements
 */
export const MINIMUM_REQUIREMENTS = {
  HEART_OUTPUT: 4.0,           // L/min
  KIDNEY_GFR: 60,             // ml/min/1.73m²
  LIVER_DETOX: 0.70,          // 70% of natural
  LUNG_GAS_EXCHANGE: 250,     // ml/min O₂
  PANCREAS_INSULIN: 40,       // units/day
  EFFICIENCY: 0.70,           // 70%
  BIOCOMPATIBILITY: 0.60,     // 60%
  RELIABILITY: 0.95,          // 95%
} as const;

/**
 * Alert priority levels
 */
export const ALERT_PRIORITIES = {
  INFO: 0,
  WARNING: 1,
  CRITICAL: 2,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Artificial organ error codes
 */
export enum OrganErrorCode {
  CLASSIFICATION_INVALID_INPUT = 'ORG001',
  MONITORING_FAILED = 'ORG002',
  BIOCOMPATIBILITY_ASSESSMENT_FAILED = 'ORG003',
  REJECTION_DETECTION_FAILED = 'ORG004',
  OPTIMIZATION_FAILED = 'ORG005',
  SERVICE_SCHEDULING_FAILED = 'ORG006',
  FAILSAFE_ACTIVATION_FAILED = 'ORG007',
  INVALID_ORGAN_TYPE = 'ORG008',
  INVALID_TECHNOLOGY_TYPE = 'ORG009',
  POWER_SYSTEM_ERROR = 'ORG010',
}

/**
 * Artificial organ error class
 */
export class OrganError extends Error {
  constructor(
    public code: OrganErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'OrganError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  OrganType,
  TechnologyType,
  PowerSystemType,
  OperationalState,
  PerformanceLevel,
  RejectionRiskLevel,
  FailsafeMode,
  FailsafeTrigger,
  EmergencySituation,
  OrganInput,
  OrganClassification,
  OutputMetrics,
  EfficiencyMetrics,
  BiomarkerData,
  PhysiologicalMetrics,
  FunctionMetrics,
  MonitoringResult,
  TissueIntegration,
  ImmuneResponse,
  MaterialSafety,
  FunctionalStability,
  BiocompatibilityData,
  BiocompatibilityScore,
  ImmuneMarkers,
  AntibodyLevels,
  PerformanceTrend,
  RejectionData,
  RejectionAssessment,
  PerformanceTargets,
  ControlParameters,
  OptimizationResult,
  ServiceSchedule,
  MaintenancePrediction,
  WearAssessment,
  FailsafeStatus,
  EmergencyResponse,
  Alert,
  OrganStatus,
  TimeRange,
  HistoricalData,
};

export {
  BIOCOMPATIBILITY_THRESHOLDS,
  REJECTION_RISK_THRESHOLDS,
  PERFORMANCE_THRESHOLDS,
  MINIMUM_REQUIREMENTS,
  ALERT_PRIORITIES,
  OrganErrorCode,
  OrganError,
};
