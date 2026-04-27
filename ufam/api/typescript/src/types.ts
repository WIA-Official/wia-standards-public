/**
 * WIA-AUTO-027: Universal Fluid for All Mobility - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Fluids Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Fluid Types
// ============================================================================

/**
 * Fluid base type classification
 */
export type FluidBaseType = 'mineral' | 'synthetic' | 'bio-based' | 'hybrid';

/**
 * Fluid grade classification based on pour point
 */
export type FluidGrade = 'A' | 'B' | 'C';

/**
 * Vehicle type classification
 */
export type VehicleType =
  | 'electric-vehicle'
  | 'hybrid-vehicle'
  | 'autonomous-vehicle'
  | 'heavy-machinery'
  | 'marine-vessel'
  | 'aircraft'
  | 'industrial';

/**
 * Operating conditions classification
 */
export type OperatingConditions = 'light' | 'normal' | 'severe' | 'extreme';

/**
 * Fluid function types
 */
export type FluidFunction = 'cooling' | 'lubrication' | 'hydraulic' | 'universal';

// ============================================================================
// Fluid Specifications
// ============================================================================

/**
 * Physical properties of a fluid
 */
export interface FluidPhysicalProperties {
  /** Kinematic viscosity at 40°C (cSt) */
  viscosity40C: number;

  /** Kinematic viscosity at 100°C (cSt) */
  viscosity100C: number;

  /** Viscosity index (dimensionless) */
  viscosityIndex: number;

  /** Pour point (°C) */
  pourPoint: number;

  /** Flash point (°C) */
  flashPoint: number;

  /** Density at 15°C (kg/m³) */
  density15C: number;

  /** Thermal conductivity at 25°C (W/m·K) */
  thermalConductivity?: number;

  /** Specific heat capacity at 25°C (kJ/kg·K) */
  specificHeat?: number;

  /** Bulk modulus at 40°C (GPa) */
  bulkModulus?: number;
}

/**
 * Chemical properties of a fluid
 */
export interface FluidChemicalProperties {
  /** Total Acid Number (mg KOH/g) */
  tan: number;

  /** Total Base Number (mg KOH/g) */
  tbn?: number;

  /** Oxidation level (abs/cm via FTIR) */
  oxidation?: number;

  /** Nitration level (abs/cm via FTIR) */
  nitration?: number;

  /** Sulfation level (abs/cm via FTIR) */
  sulfation?: number;

  /** Water content (%) */
  waterContent: number;

  /** pH value */
  pH?: number;
}

/**
 * Environmental properties of a fluid
 */
export interface FluidEnvironmentalProperties {
  /** Biodegradability percentage in 28 days (%) */
  biodegradability: number;

  /** Bio-based carbon content (%) */
  bioBasedContent: number;

  /** Aquatic toxicity LC50 (mg/L) */
  toxicityLC50: number;

  /** Renewable Carbon Index (%) */
  renewableCarbonIndex: number;

  /** Carbon footprint (kg CO₂e/kg fluid) */
  carbonFootprint?: number;

  /** Circular Economy Score (0-100) */
  circularEconomyScore?: number;
}

/**
 * Complete fluid specification
 */
export interface FluidSpecification {
  /** Unique fluid identifier */
  fluidId: string;

  /** Fluid name/brand */
  name: string;

  /** Manufacturer */
  manufacturer?: string;

  /** Base type */
  type: FluidBaseType;

  /** Grade classification */
  grade: FluidGrade;

  /** Primary function */
  function: FluidFunction;

  /** Physical properties */
  physical: FluidPhysicalProperties;

  /** Chemical properties */
  chemical?: FluidChemicalProperties;

  /** Environmental properties */
  environmental: FluidEnvironmentalProperties;

  /** Certification date */
  certificationDate?: Date;

  /** Expiration date */
  expirationDate?: Date;
}

// ============================================================================
// Fluid Measurements and Monitoring
// ============================================================================

/**
 * Real-time fluid measurements from sensors
 */
export interface FluidMeasurements {
  /** Current viscosity (cSt) */
  viscosity: number;

  /** Measurement temperature (°C) */
  temperature: number;

  /** Total Acid Number (mg KOH/g) */
  tan: number;

  /** Water content (%) */
  waterContent: number;

  /** Particle count (particles/mL) for particles ≥25μm */
  particleCount: number;

  /** ISO 4406 cleanliness code (e.g., "16/14/11") */
  isoCode?: string;

  /** Oxidation level (abs/cm) */
  oxidation?: number;

  /** Nitration level (abs/cm) */
  nitration?: number;

  /** Total Base Number (mg KOH/g) */
  tbn?: number;

  /** Dielectric constant */
  dielectricConstant?: number;

  /** Color (ASTM D1500) */
  color?: number;
}

/**
 * Sensor data with metadata
 */
export interface SensorData {
  /** Timestamp of measurement */
  timestamp: Date;

  /** Sensor identifier */
  sensorId: string;

  /** Vehicle identifier */
  vehicleId: string;

  /** Location of vehicle */
  location?: {
    latitude: number;
    longitude: number;
  };

  /** Measurements */
  measurements: FluidMeasurements;

  /** Overall health score (0-100) */
  healthScore?: number;

  /** Active alerts */
  alerts?: Alert[];

  /** Data quality indicator */
  quality?: 'excellent' | 'good' | 'fair' | 'poor';
}

/**
 * Alert/Warning information
 */
export interface Alert {
  /** Alert type */
  type: 'info' | 'warning' | 'critical';

  /** Alert code */
  code: string;

  /** Human-readable message */
  message: string;

  /** Parameter that triggered alert */
  parameter: string;

  /** Current value */
  value: number;

  /** Threshold value */
  threshold: number;

  /** Recommended action */
  action?: string;
}

// ============================================================================
// Fluid Condition Analysis
// ============================================================================

/**
 * Fluid condition status
 */
export type FluidConditionStatus = 'excellent' | 'good' | 'acceptable' | 'marginal' | 'critical';

/**
 * Recommendation action types
 */
export type RecommendationAction =
  | 'continue_monitoring'
  | 'increase_monitoring'
  | 'plan_replacement'
  | 'immediate_replacement'
  | 'chemical_replenishment'
  | 'filtration_enhancement'
  | 'fluid_reclamation';

/**
 * Individual parameter condition
 */
export interface ParameterCondition {
  /** Parameter name */
  parameter: string;

  /** Current value */
  value: number;

  /** Unit */
  unit: string;

  /** Status */
  status: 'normal' | 'caution' | 'warning' | 'critical';

  /** Score (0-100) */
  score: number;

  /** Action limit */
  actionLimit?: number;

  /** Condemning limit */
  condemnLimit?: number;
}

/**
 * Complete fluid condition analysis result
 */
export interface FluidConditionResult {
  /** Overall condition status */
  condition: FluidConditionStatus;

  /** Overall health score (0-100) */
  healthScore: number;

  /** Individual parameter conditions */
  parameters: ParameterCondition[];

  /** Remaining useful life (hours) */
  remainingLifeHours: number;

  /** Confidence in RUL estimate (0-1) */
  confidence: number;

  /** Recommended action */
  recommendation: RecommendationAction;

  /** Priority level */
  priority: 'low' | 'medium' | 'high' | 'urgent';

  /** Next test interval (hours) */
  nextTestHours: number;

  /** Detailed analysis */
  details: {
    viscosityStatus: string;
    oxidationStatus: string;
    contaminationStatus: string;
    degradationStatus: string;
  };

  /** Warnings and recommendations */
  warnings: string[];
  recommendations: string[];
}

// ============================================================================
// Replacement Scheduling
// ============================================================================

/**
 * Operating profile for a vehicle/system
 */
export interface OperatingProfile {
  /** Vehicle/system type */
  vehicleType: VehicleType;

  /** Operating conditions */
  conditions: OperatingConditions;

  /** Average operating temperature (°C) */
  averageTemperature?: number;

  /** Daily operating hours */
  dailyHours?: number;

  /** Load factor (0-1) */
  loadFactor?: number;

  /** Duty cycle description */
  dutyCycle?: string;
}

/**
 * Schedule parameters for replacement calculation
 */
export interface ScheduleParameters {
  /** Fluid type */
  fluidType: FluidBaseType;

  /** Vehicle type */
  vehicleType: VehicleType;

  /** Operating conditions */
  operatingConditions: OperatingConditions;

  /** Current operating hours */
  currentHours: number;

  /** Current fluid condition */
  currentCondition?: FluidConditionResult;

  /** Operating profile */
  profile?: OperatingProfile;
}

/**
 * Replacement schedule result
 */
export interface ReplacementSchedule {
  /** Recommended action */
  recommendedAction: RecommendationAction;

  /** Timeline in days (if applicable) */
  timelineDays?: number;

  /** Timeline in operating hours */
  timelineHours: number;

  /** Next replacement date estimate */
  nextReplacementDate?: Date;

  /** Confidence level (0-1) */
  confidence: number;

  /** Reason for recommendation */
  reason: string;

  /** Cost estimate */
  costEstimate?: {
    fluidCost: number;
    laborCost: number;
    totalCost: number;
    currency: string;
  };

  /** Alternative options */
  alternatives?: Array<{
    action: RecommendationAction;
    cost: number;
    lifeExtensionHours: number;
    description: string;
  }>;
}

// ============================================================================
// Compatibility
// ============================================================================

/**
 * Compatibility check parameters
 */
export interface CompatibilityCheckParams {
  /** First fluid identifier or type */
  fluidA: string;

  /** Second fluid identifier or type */
  fluidB: string;

  /** Mixture ratio (0-1, proportion of fluidB) */
  mixtureRatio: number;

  /** Target application */
  application?: FluidFunction;

  /** Operating temperature range */
  temperatureRange?: {
    min: number;
    max: number;
  };
}

/**
 * Compatibility result
 */
export interface CompatibilityResult {
  /** Overall compatibility */
  compatible: boolean;

  /** Compatibility score (0-100) */
  compatibilityScore: number;

  /** Expected mixture properties */
  expectedProperties?: {
    viscosity40C: number;
    viscosityIndex: number;
    pourPoint: number;
  };

  /** Warnings */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];

  /** Risks */
  risks?: string[];

  /** Testing recommended */
  testingRecommended: boolean;
}

// ============================================================================
// Fluid Management
// ============================================================================

/**
 * Fluid lifecycle tracking
 */
export interface FluidLifecycle {
  /** Fluid batch identifier */
  batchId: string;

  /** Installation date */
  installationDate: Date;

  /** Initial properties */
  initialProperties: FluidPhysicalProperties & FluidChemicalProperties;

  /** Current operating hours */
  operatingHours: number;

  /** Measurement history */
  measurementHistory: Array<{
    date: Date;
    hours: number;
    measurements: FluidMeasurements;
    healthScore: number;
  }>;

  /** Maintenance events */
  maintenanceEvents?: Array<{
    date: Date;
    type: 'replacement' | 'replenishment' | 'filtration' | 'reclamation';
    description: string;
    cost?: number;
  }>;

  /** Current condition */
  currentCondition?: FluidConditionResult;

  /** Projected replacement date */
  projectedReplacementDate?: Date;
}

/**
 * Fleet-wide fluid management
 */
export interface FleetFluidManagement {
  /** Fleet identifier */
  fleetId: string;

  /** Fleet name */
  fleetName: string;

  /** Total vehicles */
  totalVehicles: number;

  /** Vehicles by fluid health */
  healthDistribution: {
    excellent: number;
    good: number;
    acceptable: number;
    marginal: number;
    critical: number;
  };

  /** Upcoming replacements */
  upcomingReplacements: Array<{
    vehicleId: string;
    daysUntilReplacement: number;
    priority: 'low' | 'medium' | 'high' | 'urgent';
  }>;

  /** Total fluid costs (monthly) */
  monthlyCosts: {
    fluidPurchases: number;
    labor: number;
    disposal: number;
    total: number;
    currency: string;
  };

  /** Environmental metrics */
  environmentalMetrics?: {
    totalFluidUsed: number; // liters
    bioBasedPercentage: number;
    recycledPercentage: number;
    carbonFootprint: number; // kg CO₂e
  };
}

// ============================================================================
// Predictive Analytics
// ============================================================================

/**
 * Remaining Useful Life (RUL) estimation
 */
export interface RemainingLifeEstimate {
  /** Estimated remaining hours */
  remainingHours: number;

  /** Confidence interval */
  confidenceInterval: {
    lower: number; // hours
    upper: number; // hours
    confidence: number; // 0-1 (e.g., 0.95 for 95%)
  };

  /** Prediction method */
  method: 'linear' | 'exponential' | 'machine-learning' | 'hybrid';

  /** Key degradation factors */
  degradationFactors: Array<{
    factor: string;
    impact: 'low' | 'medium' | 'high';
    description: string;
  }>;

  /** Recommended actions to extend life */
  lifeExtensionActions?: string[];
}

/**
 * Anomaly detection result
 */
export interface AnomalyDetection {
  /** Anomaly detected */
  anomalyDetected: boolean;

  /** Anomaly score (0-1, higher = more anomalous) */
  anomalyScore: number;

  /** Anomalous parameters */
  anomalousParameters: string[];

  /** Anomaly type */
  anomalyType?: 'sudden_change' | 'drift' | 'outlier' | 'pattern_break';

  /** Severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Possible causes */
  possibleCauses: string[];

  /** Recommended investigations */
  recommendedInvestigations: string[];
}

// ============================================================================
// Testing and Certification
// ============================================================================

/**
 * Test result for a specific property
 */
export interface TestResult {
  /** Test name */
  testName: string;

  /** Test method (e.g., "ASTM D445") */
  testMethod: string;

  /** Measured value */
  value: number;

  /** Unit */
  unit: string;

  /** Specification limit */
  specLimit?: {
    min?: number;
    max?: number;
  };

  /** Pass/Fail status */
  status: 'pass' | 'fail' | 'borderline';

  /** Test date */
  testDate: Date;

  /** Laboratory */
  laboratory?: string;

  /** Certificate number */
  certificateNumber?: string;
}

/**
 * Complete certification report
 */
export interface CertificationReport {
  /** Report identifier */
  reportId: string;

  /** Fluid specification */
  fluidSpec: FluidSpecification;

  /** All test results */
  testResults: TestResult[];

  /** Overall certification status */
  certificationStatus: 'certified' | 'conditional' | 'failed';

  /** Certification date */
  certificationDate: Date;

  /** Valid until */
  validUntil: Date;

  /** Certifying body */
  certifyingBody: string;

  /** Notes and comments */
  notes?: string[];
}

// ============================================================================
// Physical Constants and Calculations
// ============================================================================

/**
 * Physical constants for fluid calculations
 */
export const FLUID_CONSTANTS = {
  /** Standard temperature for viscosity measurement (°C) */
  STANDARD_TEMP_40C: 40,
  STANDARD_TEMP_100C: 100,

  /** Condemning limits */
  MAX_TAN: 4.0, // mg KOH/g
  MAX_WATER: 0.2, // %
  MAX_VISCOSITY_CHANGE: 0.15, // 15%

  /** Action limits */
  ACTION_TAN: 2.5, // mg KOH/g
  ACTION_WATER: 0.1, // %
  ACTION_VISCOSITY_CHANGE: 0.10, // 10%

  /** Biodegradability */
  MIN_BIODEGRADABLE: 60, // % in 28 days
  HIGHLY_BIODEGRADABLE: 90, // % in 28 days

  /** Toxicity */
  MIN_LC50: 100, // mg/L (minimum acceptable)
  SAFE_LC50: 1000, // mg/L (practically non-toxic)

  /** Health score thresholds */
  HEALTH_EXCELLENT: 80,
  HEALTH_GOOD: 60,
  HEALTH_ACCEPTABLE: 40,
  HEALTH_MARGINAL: 20,
} as const;

/**
 * Scoring weights for health calculation
 */
export const HEALTH_WEIGHTS = {
  viscosity: 0.25,
  tan: 0.30,
  water: 0.20,
  particles: 0.15,
  oxidation: 0.10,
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
 * Pagination parameters
 */
export interface PaginationParams {
  /** Page number (1-indexed) */
  page: number;

  /** Items per page */
  pageSize: number;

  /** Sort field */
  sortBy?: string;

  /** Sort direction */
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated result
 */
export interface PaginatedResult<T> {
  /** Items in current page */
  items: T[];

  /** Total number of items */
  total: number;

  /** Current page */
  page: number;

  /** Page size */
  pageSize: number;

  /** Total pages */
  totalPages: number;

  /** Has next page */
  hasNext: boolean;

  /** Has previous page */
  hasPrevious: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-027 error codes
 */
export enum FluidErrorCode {
  INVALID_MEASUREMENT = 'F001',
  SENSOR_FAILURE = 'F002',
  OUT_OF_RANGE = 'F003',
  INCOMPATIBLE_FLUIDS = 'F004',
  CERTIFICATION_EXPIRED = 'F005',
  UNKNOWN_FLUID = 'F006',
  ANALYSIS_FAILED = 'F007',
  DATA_QUALITY_POOR = 'F008',
  CRITICAL_CONDITION = 'F009',
  IMMEDIATE_ACTION_REQUIRED = 'F010',
}

/**
 * Fluid management error
 */
export class FluidManagementError extends Error {
  constructor(
    public code: FluidErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'FluidManagementError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  FluidBaseType,
  FluidGrade,
  VehicleType,
  OperatingConditions,
  FluidFunction,

  // Specifications
  FluidPhysicalProperties,
  FluidChemicalProperties,
  FluidEnvironmentalProperties,
  FluidSpecification,

  // Measurements
  FluidMeasurements,
  SensorData,
  Alert,

  // Condition Analysis
  FluidConditionStatus,
  RecommendationAction,
  ParameterCondition,
  FluidConditionResult,

  // Scheduling
  OperatingProfile,
  ScheduleParameters,
  ReplacementSchedule,

  // Compatibility
  CompatibilityCheckParams,
  CompatibilityResult,

  // Management
  FluidLifecycle,
  FleetFluidManagement,

  // Predictive
  RemainingLifeEstimate,
  AnomalyDetection,

  // Testing
  TestResult,
  CertificationReport,

  // Utility
  PaginationParams,
  PaginatedResult,
};

export { FLUID_CONSTANTS, HEALTH_WEIGHTS, FluidErrorCode, FluidManagementError };
