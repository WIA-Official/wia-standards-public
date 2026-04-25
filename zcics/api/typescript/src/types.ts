/**
 * WIA-AUTO-026: Zero-Chemical Intelligent Cleaning System - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core System Types
// ============================================================================

/**
 * Vehicle information
 */
export interface Vehicle {
  /** Vehicle identifier */
  id?: string;

  /** Vehicle type */
  type: 'sedan' | 'suv' | 'truck' | 'van' | 'coupe' | 'hatchback' | 'wagon' | 'bus' | 'motorcycle';

  /** Vehicle make */
  make?: string;

  /** Vehicle model */
  model?: string;

  /** Year of manufacture */
  year?: number;

  /** Vehicle color */
  color?: string;

  /** Total surface area in square meters */
  surfaceArea: number;
}

/**
 * Dirt and contamination assessment
 */
export interface DirtAssessment {
  /** Overall dirt level */
  level: 'light' | 'medium' | 'heavy' | 'extreme';

  /** Types of contamination present */
  contaminationTypes: ContaminationType[];

  /** Dirt density in mg/cm² */
  dirtDensity?: number;

  /** Special concerns or problem areas */
  specialConcerns?: string[];

  /** Image analysis data */
  imageData?: ImageAnalysis;
}

/**
 * Types of contamination
 */
export type ContaminationType =
  | 'dust'
  | 'pollen'
  | 'road_grime'
  | 'mud'
  | 'salt'
  | 'oil'
  | 'grease'
  | 'tar'
  | 'bird_droppings'
  | 'tree_sap'
  | 'water_spots'
  | 'mold'
  | 'bacteria'
  | 'other';

/**
 * AI-based image analysis results
 */
export interface ImageAnalysis {
  /** Reflectance percentage (80-95% = clean) */
  reflectance: number;

  /** Particle count per cm² */
  particleCount: number;

  /** Identified contamination zones */
  zones: ContaminationZone[];

  /** AI confidence score */
  confidence: number;
}

/**
 * Contamination zone in image
 */
export interface ContaminationZone {
  /** Zone location (relative coordinates 0-1) */
  location: {
    x: number;
    y: number;
    width: number;
    height: number;
  };

  /** Contamination type in this zone */
  type: ContaminationType;

  /** Severity in this zone */
  severity: 'light' | 'medium' | 'heavy';
}

// ============================================================================
// Electrolyzed Water Types
// ============================================================================

/**
 * Electrolyzed water parameters
 */
export interface ElectrolyzedWater {
  /** Water type */
  type: 'acidic' | 'alkaline';

  /** pH level (acidic: 2-4, alkaline: 10-13) */
  pH: number;

  /** Oxidation-Reduction Potential in mV */
  orp: number;

  /** Chlorine concentration in ppm (acidic only) */
  chlorinePPM?: number;

  /** Sodium hydroxide concentration % (alkaline only) */
  naohPercent?: number;

  /** Water temperature in Celsius */
  temperature: number;

  /** Production timestamp */
  producedAt: Date;

  /** Water volume in liters */
  volume: number;

  /** Quality status */
  quality: 'excellent' | 'good' | 'acceptable' | 'expired';
}

/**
 * Electrolyzed water generation parameters
 */
export interface EWGenerationParams {
  /** Desired water type */
  type: 'acidic' | 'alkaline';

  /** Target volume in liters */
  volume: number;

  /** Voltage setting (5-24V) */
  voltage?: number;

  /** Current setting in amperes (2-20A) */
  current?: number;

  /** Salt concentration percentage */
  saltConcentration?: number;

  /** Flow rate in L/min */
  flowRate?: number;
}

/**
 * Cleaning efficiency calculation parameters
 */
export interface CleaningEfficiencyParams {
  /** Chlorine concentration in ppm */
  chlorineConcentration: number;

  /** Water volume in liters */
  waterVolume: number;

  /** pH level */
  pH: number;

  /** Contact time in seconds */
  contactTime: number;

  /** Dirt density in mg/cm² (optional) */
  dirtDensity?: number;

  /** Surface area in cm² (optional) */
  surfaceArea?: number;
}

/**
 * Cleaning efficiency result
 */
export interface CleaningEfficiencyResult {
  /** Cleaning efficiency percentage */
  efficiency: number;

  /** Expected cleanliness score (0-100) */
  expectedCleanliness: number;

  /** Effectiveness rating */
  effectiveness: 'excellent' | 'good' | 'moderate' | 'poor';

  /** Recommendations for improvement */
  recommendations: string[];
}

// ============================================================================
// UV-C Sterilization Types
// ============================================================================

/**
 * UV-C sterilization parameters
 */
export interface UVCParameters {
  /** UV intensity in mW/cm² */
  intensity: number;

  /** Exposure time in seconds */
  exposureTime: number;

  /** Surface area in cm² */
  surfaceArea: number;

  /** Wavelength in nm (typically 254) */
  wavelength?: number;

  /** Target pathogen reduction percentage */
  targetReduction: number;

  /** Specific pathogen type */
  pathogen?: PathogenType;
}

/**
 * Pathogen types for UV-C sterilization
 */
export type PathogenType =
  | 'e_coli'
  | 'salmonella'
  | 'staphylococcus'
  | 'sars_cov_2'
  | 'mold_spores'
  | 'general_bacteria'
  | 'general_virus';

/**
 * UV-C validation result
 */
export interface UVCValidationResult {
  /** Is configuration valid? */
  isValid: boolean;

  /** Calculated UV dose in mJ/cm² */
  calculatedDose: number;

  /** Required dose for target reduction */
  requiredDose: number;

  /** Safety margin (calculated/required) */
  safetyMargin: number;

  /** Validation status */
  status: 'adequate' | 'marginal' | 'insufficient';

  /** Warnings or errors */
  warnings: string[];

  /** Recommendation */
  recommendation: string;
}

/**
 * UV lamp status
 */
export interface UVLampStatus {
  /** Lamp identifier */
  lampId: string;

  /** Current intensity in mW/cm² */
  currentIntensity: number;

  /** Rated intensity when new */
  ratedIntensity: number;

  /** Lamp temperature in Celsius */
  temperature: number;

  /** Operating current in mA */
  current: number;

  /** Total operating hours */
  hoursUsed: number;

  /** Rated lamp life in hours */
  ratedLife: number;

  /** Lamp health percentage */
  health: number;

  /** Status */
  status: 'operational' | 'degraded' | 'replace_soon' | 'failed';
}

// ============================================================================
// Plasma Cleaning Types
// ============================================================================

/**
 * Plasma cleaning parameters
 */
export interface PlasmaParameters {
  /** Plasma generation method */
  method: 'dbd' | 'corona' | 'appj' | 'rf' | 'microwave' | 'icp';

  /** Operating power in watts */
  power: number;

  /** Frequency in Hz */
  frequency: number;

  /** Process gas type */
  gasType: 'air' | 'oxygen' | 'argon' | 'nitrogen' | 'argon_oxygen';

  /** Gas flow rate in L/min */
  gasFlowRate: number;

  /** Treatment duration in seconds */
  duration: number;

  /** Treatment area in cm² */
  treatmentArea: number;

  /** Operating pressure (Pa or Torr) */
  pressure?: number;
}

/**
 * Plasma cleaning rate calculation
 */
export interface PlasmaCleaningRate {
  /** Cleaning rate in mg/min */
  rate: number;

  /** Energy efficiency factor (0-1) */
  energyEfficiency: number;

  /** Expected contamination removal */
  expectedRemoval: number;

  /** Treatment effectiveness */
  effectiveness: 'excellent' | 'good' | 'moderate' | 'poor';
}

// ============================================================================
// Ozone Treatment Types
// ============================================================================

/**
 * Ozone treatment parameters
 */
export interface OzoneParameters {
  /** Treatment medium */
  medium: 'air' | 'water';

  /** Ozone concentration in ppm */
  concentration: number;

  /** Treatment duration in minutes */
  duration: number;

  /** Treatment volume in liters or m³ */
  volume: number;

  /** Temperature in Celsius */
  temperature: number;

  /** Target application */
  application: 'odor_removal' | 'disinfection' | 'oxidation' | 'sanitization';
}

/**
 * Ozone generator configuration
 */
export interface OzoneGeneratorConfig {
  /** Production method */
  productionMethod: 'corona_discharge' | 'uv_photolysis' | 'electrolysis';

  /** Output rate in g/hour */
  outputRate: number;

  /** Maximum concentration in mg/L */
  maxConcentration: number;

  /** Feed gas type */
  feedGas: 'air' | 'oxygen';

  /** Cooling method */
  cooling: 'air' | 'water';

  /** Power consumption in watts */
  powerConsumption: number;
}

/**
 * Ozone safety monitoring
 */
export interface OzoneSafety {
  /** Current air concentration in ppm */
  airConcentration: number;

  /** Current water concentration in ppm */
  waterConcentration: number;

  /** Safety threshold in ppm */
  safetyThreshold: number;

  /** Is concentration safe? */
  isSafe: boolean;

  /** Time to safe levels in minutes */
  purgeTimeRemaining?: number;

  /** Alerts */
  alerts: OzoneAlert[];
}

/**
 * Ozone safety alert
 */
export interface OzoneAlert {
  /** Alert severity */
  severity: 'info' | 'warning' | 'danger' | 'critical';

  /** Alert message */
  message: string;

  /** Recommended action */
  action: string;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Cleaning Session Types
// ============================================================================

/**
 * Cleaning session
 */
export interface CleaningSession {
  /** Unique session identifier */
  sessionId: string;

  /** Session timestamp */
  timestamp: Date;

  /** Vehicle information */
  vehicle: Vehicle;

  /** Dirt assessment */
  assessment: DirtAssessment;

  /** Cleaning plan */
  plan: CleaningPlan;

  /** Execution details */
  execution?: CleaningExecution;

  /** Results */
  results?: CleaningResults;

  /** Environmental impact */
  environmentalImpact?: EnvironmentalImpact;

  /** Session status */
  status: 'pending' | 'in_progress' | 'completed' | 'failed' | 'cancelled';
}

/**
 * Cleaning plan
 */
export interface CleaningPlan {
  /** Plan identifier */
  planId?: string;

  /** Cleaning methods to use */
  methods: CleaningMethod[];

  /** Cleaning stages */
  stages: CleaningStage[];

  /** Estimated total duration in seconds */
  estimatedDuration: number;

  /** Estimated water usage in liters */
  estimatedWater: number;

  /** Estimated energy usage in kWh */
  estimatedEnergy: number;

  /** Expected cleanliness score */
  expectedCleanliness: number;

  /** Optimization criteria */
  optimizedFor: OptimizationCriteria[];
}

/**
 * Cleaning methods
 */
export type CleaningMethod =
  | 'electrolyzed_water'
  | 'uv_sterilization'
  | 'plasma_cleaning'
  | 'ozone_treatment'
  | 'combined';

/**
 * Optimization criteria
 */
export type OptimizationCriteria =
  | 'effectiveness'
  | 'speed'
  | 'water_efficiency'
  | 'energy_efficiency'
  | 'cost';

/**
 * Cleaning stage
 */
export interface CleaningStage {
  /** Stage name */
  name: string;

  /** Stage method */
  method: CleaningMethod;

  /** Stage duration in seconds */
  duration: number;

  /** Method-specific parameters */
  parameters: Record<string, any>;

  /** Stage order */
  order: number;
}

/**
 * Cleaning execution details
 */
export interface CleaningExecution {
  /** Start time */
  startTime: Date;

  /** End time */
  endTime?: Date;

  /** Actual duration in seconds */
  actualDuration?: number;

  /** Executed stages */
  stages: ExecutedStage[];

  /** Real-time progress */
  progress?: CleaningProgress;
}

/**
 * Executed stage
 */
export interface ExecutedStage extends CleaningStage {
  /** Stage start time */
  startTime: Date;

  /** Stage end time */
  endTime?: Date;

  /** Actual parameters used */
  actualParameters: Record<string, any>;

  /** Stage status */
  status: 'pending' | 'in_progress' | 'completed' | 'failed' | 'skipped';

  /** Sensor readings during stage */
  sensorReadings?: SensorTelemetry[];
}

/**
 * Real-time cleaning progress
 */
export interface CleaningProgress {
  /** Progress percentage (0-100) */
  progressPercent: number;

  /** Current stage */
  currentStage: string;

  /** Elapsed time in seconds */
  elapsedTime: number;

  /** Remaining time in seconds */
  remainingTime: number;

  /** Real-time metrics */
  metrics: {
    waterUsed: number;
    energyUsed: number;
    currentCleanliness: number;
  };
}

/**
 * Cleaning results
 */
export interface CleaningResults {
  /** Overall cleanliness score (0-100) */
  cleanlinessScore: number;

  /** Sterilization rate percentage */
  sterilizationRate: number;

  /** Total water used in liters */
  waterUsed: number;

  /** Water recovered in liters */
  waterRecovered: number;

  /** Total energy used in kWh */
  energyUsed: number;

  /** Quality check status */
  qualityCheck: 'passed' | 'failed' | 'needs_review';

  /** Quality check details */
  qualityDetails?: string[];

  /** Customer rating (optional) */
  customerRating?: number;

  /** Before/after images */
  images?: {
    before?: string;
    after?: string;
  };
}

/**
 * Environmental impact metrics
 */
export interface EnvironmentalImpact {
  /** Water saved vs traditional method in liters */
  waterSavedVsTraditional: number;

  /** Carbon footprint in kg CO₂e */
  carbonFootprint: number;

  /** Carbon saved vs traditional in kg CO₂e */
  carbonSavedVsTraditional: number;

  /** Zero chemicals used */
  zeroChemicals: boolean;

  /** Water recovery rate percentage */
  waterRecoveryRate: number;

  /** Energy efficiency score (0-100) */
  energyEfficiencyScore: number;
}

// ============================================================================
// Sensor and Telemetry Types
// ============================================================================

/**
 * Sensor telemetry data
 */
export interface SensorTelemetry {
  /** Device identifier */
  deviceId: string;

  /** Timestamp */
  timestamp: Date;

  /** Electrolyzed water sensors */
  electrolyzedWater?: {
    acidic?: ElectrolyzedWaterSensor;
    alkaline?: ElectrolyzedWaterSensor;
  };

  /** UV system sensors */
  uvSystem?: {
    lamps: UVLampStatus[];
  };

  /** Plasma system sensors */
  plasma?: PlasmaSensor;

  /** Ozone sensors */
  ozone?: OzoneSensor;

  /** Environmental sensors */
  environmental?: EnvironmentalSensor;

  /** System status */
  systemStatus?: SystemStatus;
}

/**
 * Electrolyzed water sensor data
 */
export interface ElectrolyzedWaterSensor {
  /** pH level */
  pH: number;

  /** ORP in mV */
  orp: number;

  /** Chlorine in ppm (acidic) */
  chlorinePPM?: number;

  /** Temperature in Celsius */
  temperature: number;

  /** Flow rate in L/min */
  flowRate: number;

  /** Sensor health */
  sensorHealth: 'good' | 'degraded' | 'failed';
}

/**
 * Plasma sensor data
 */
export interface PlasmaSensor {
  /** Plasma status */
  status: 'off' | 'standby' | 'active' | 'fault';

  /** Power in watts */
  power: number;

  /** Frequency in Hz */
  frequency: number;

  /** Gas flow rate in L/min */
  gasFlow: number;

  /** Temperature in Celsius */
  temperature?: number;
}

/**
 * Ozone sensor data
 */
export interface OzoneSensor {
  /** Air concentration in ppm */
  concentrationAir: number;

  /** Water concentration in ppm */
  concentrationWater: number;

  /** Generator status */
  generatorStatus: 'off' | 'standby' | 'producing' | 'fault';

  /** Production rate in g/hour */
  productionRate: number;
}

/**
 * Environmental sensor data
 */
export interface EnvironmentalSensor {
  /** Ambient temperature in Celsius */
  ambientTemperature: number;

  /** Humidity percentage */
  humidity: number;

  /** Air quality index */
  airQualityIndex: number;

  /** Atmospheric pressure in hPa */
  pressure?: number;
}

/**
 * System status
 */
export interface SystemStatus {
  /** Operational mode */
  operationalMode: 'idle' | 'cleaning' | 'maintenance' | 'error';

  /** Active session ID */
  activeSession?: string;

  /** System alarms */
  alarms: SystemAlarm[];

  /** System warnings */
  warnings: string[];

  /** Overall health score (0-100) */
  healthScore: number;
}

/**
 * System alarm
 */
export interface SystemAlarm {
  /** Alarm code */
  code: string;

  /** Alarm severity */
  severity: 'info' | 'warning' | 'error' | 'critical';

  /** Alarm message */
  message: string;

  /** Timestamp */
  timestamp: Date;

  /** Corrective action */
  correctiveAction?: string;

  /** Is acknowledged? */
  acknowledged: boolean;
}

// ============================================================================
// Equipment Configuration Types
// ============================================================================

/**
 * Equipment configuration
 */
export interface EquipmentConfig {
  /** Facility identifier */
  facilityId: string;

  /** EW generator configuration */
  ewGenerator?: EWGeneratorConfig;

  /** UV system configuration */
  uvSystem?: UVSystemConfig;

  /** Plasma generator configuration */
  plasmaGenerator?: PlasmaGeneratorConfig;

  /** Ozone generator configuration */
  ozoneGenerator?: OzoneGeneratorConfig;

  /** Water system configuration */
  waterSystem?: WaterSystemConfig;
}

/**
 * EW generator configuration
 */
export interface EWGeneratorConfig {
  /** Model name */
  model: string;

  /** Manufacturer */
  manufacturer: string;

  /** Serial number */
  serialNumber: string;

  /** Production capacity in L/min */
  capacity: number;

  /** Electrode material */
  electrodeMaterial: string;

  /** Membrane type */
  membraneType: string;

  /** Installation date */
  installationDate: Date;

  /** Last maintenance date */
  lastMaintenance?: Date;

  /** Operating parameters */
  parameters: {
    voltage: number;
    currentMax: number;
    saltConcentration: number;
    productionRate: number;
  };
}

/**
 * UV system configuration
 */
export interface UVSystemConfig {
  /** Model name */
  model: string;

  /** Lamp type */
  lampType: 'low_pressure_mercury' | 'uv_led' | 'amalgam';

  /** Number of lamps */
  lampCount: number;

  /** Power per lamp in watts */
  powerPerLamp: number;

  /** Wavelength in nm */
  wavelength: number;

  /** Effective range in cm */
  effectiveRange: number;

  /** Lamp life in hours */
  lampLife: number;
}

/**
 * Plasma generator configuration
 */
export interface PlasmaGeneratorConfig {
  /** Model name */
  model: string;

  /** Plasma type */
  type: 'dielectric_barrier_discharge' | 'corona' | 'appj' | 'rf' | 'microwave';

  /** Power rating in watts */
  powerRating: number;

  /** Frequency in Hz */
  frequency: number;

  /** Gas type */
  gasType: string;

  /** Treatment area in m² */
  treatmentArea: number;
}

/**
 * Water system configuration
 */
export interface WaterSystemConfig {
  /** Total storage capacity in liters */
  storageCapacity: number;

  /** Filtration system type */
  filtrationType: 'sediment' | 'carbon' | 'reverse_osmosis' | 'multi_stage';

  /** Water recovery rate percentage */
  recoveryRate: number;

  /** Recirculation capability */
  recirculation: boolean;

  /** Maximum flow rate in L/min */
  maxFlowRate: number;
}

// ============================================================================
// API Types
// ============================================================================

/**
 * API request to start cleaning session
 */
export interface StartCleaningRequest {
  /** Vehicle information */
  vehicle: Vehicle;

  /** Dirt assessment (optional, will be auto-detected if not provided) */
  assessment?: DirtAssessment;

  /** Enable eco mode */
  ecoMode?: boolean;

  /** Custom parameters */
  customParameters?: {
    skipOzone?: boolean;
    extraSterilization?: boolean;
    waterLimit?: number;
    timeLimit?: number;
  };
}

/**
 * API response for start cleaning
 */
export interface StartCleaningResponse {
  /** Session ID */
  sessionId: string;

  /** Session status */
  status: 'started' | 'queued' | 'error';

  /** Estimated completion time */
  estimatedCompletion: Date;

  /** Cleaning plan */
  plan: CleaningPlan;

  /** Error message if status is error */
  error?: string;
}

/**
 * API request to get session status
 */
export interface GetSessionStatusRequest {
  /** Session ID */
  sessionId: string;
}

/**
 * API response for session status
 */
export interface GetSessionStatusResponse {
  /** Session ID */
  sessionId: string;

  /** Session status */
  status: 'pending' | 'in_progress' | 'completed' | 'failed' | 'cancelled';

  /** Progress details */
  progress?: CleaningProgress;

  /** Results (if completed) */
  results?: CleaningResults;

  /** Error message (if failed) */
  error?: string;
}

/**
 * Analytics request
 */
export interface AnalyticsRequest {
  /** Start date */
  startDate: Date;

  /** End date */
  endDate: Date;

  /** Facility ID */
  facilityId?: string;

  /** Metrics to include */
  metrics?: AnalyticsMetric[];
}

/**
 * Analytics metrics
 */
export type AnalyticsMetric =
  | 'sessions'
  | 'water_usage'
  | 'energy_usage'
  | 'cleanliness'
  | 'efficiency'
  | 'environmental_impact';

/**
 * Analytics response
 */
export interface AnalyticsResponse {
  /** Period information */
  period: {
    start: Date;
    end: Date;
    days: number;
  };

  /** Summary statistics */
  summary: {
    totalSessions: number;
    averageDuration: number;
    averageCleanliness: number;
    totalWaterUsed: number;
    totalWaterSaved: number;
    totalEnergyUsed: number;
    totalCarbonAvoided: number;
  };

  /** Trends */
  trends?: {
    efficiencyTrend: 'improving' | 'stable' | 'declining';
    resourceOptimization: 'excellent' | 'good' | 'needs_improvement';
    qualityConsistency: 'excellent' | 'good' | 'variable';
  };

  /** Detailed data points */
  dataPoints?: AnalyticsDataPoint[];
}

/**
 * Analytics data point
 */
export interface AnalyticsDataPoint {
  /** Date */
  date: Date;

  /** Sessions on this date */
  sessions: number;

  /** Average cleanliness */
  avgCleanliness: number;

  /** Water used */
  waterUsed: number;

  /** Energy used */
  energyUsed: number;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and chemical constants for zero-chemical cleaning
 */
export const CLEANING_CONSTANTS = {
  /** Standard chlorine concentration range for acidic EW (ppm) */
  CHLORINE_RANGE: { min: 10, max: 80 },

  /** Standard pH ranges */
  PH_RANGE: {
    acidic: { min: 2.0, max: 4.0 },
    neutral: { min: 6.5, max: 7.5 },
    alkaline: { min: 10.0, max: 13.0 },
  },

  /** Standard ORP ranges (mV) */
  ORP_RANGE: {
    acidic: { min: 1000, max: 1200 },
    alkaline: { min: -900, max: -800 },
  },

  /** UV-C wavelength (nm) */
  UVC_WAVELENGTH: 254,

  /** Required UV doses for pathogen reduction (mJ/cm²) */
  UV_DOSES: {
    e_coli_99: 4,
    e_coli_999: 6,
    salmonella_99: 8,
    salmonella_999: 12,
    sars_cov_2_999: 16.9,
    general_99: 6,
    general_999: 10,
  },

  /** Ozone safety limits (ppm) */
  OZONE_LIMITS: {
    osha_8hr: 0.1,
    niosh_8hr: 0.1,
    short_term_15min: 0.3,
  },

  /** Water usage comparison (liters per vehicle) */
  WATER_USAGE: {
    traditional: 200,
    zero_chemical: 30,
    recovered: 22,
    net_usage: 8,
  },

  /** Energy usage (kWh per vehicle) */
  ENERGY_USAGE: {
    traditional: 3.5,
    zero_chemical: 0.6,
  },

  /** Typical cleaning durations (seconds) */
  DURATION: {
    light: 480,
    medium: 600,
    heavy: 900,
    extreme: 1200,
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
 * Validation result
 */
export interface ValidationResult {
  /** Is valid? */
  isValid: boolean;

  /** Validation errors */
  errors: string[];

  /** Validation warnings */
  warnings: string[];
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-026 error codes
 */
export enum CleaningErrorCode {
  INSUFFICIENT_WATER = 'C001',
  INSUFFICIENT_ENERGY = 'C002',
  EQUIPMENT_FAILURE = 'C003',
  SENSOR_MALFUNCTION = 'C004',
  PARAMETER_OUT_OF_RANGE = 'C005',
  SAFETY_THRESHOLD_EXCEEDED = 'C006',
  SESSION_NOT_FOUND = 'C007',
  INVALID_CONFIGURATION = 'C008',
  OZONE_LEVEL_UNSAFE = 'C009',
  UV_LAMP_FAILURE = 'C010',
  QUALITY_CHECK_FAILED = 'C011',
}

/**
 * Zero-chemical cleaning system error
 */
export class CleaningSystemError extends Error {
  constructor(
    public code: CleaningErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'CleaningSystemError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  Vehicle,
  DirtAssessment,
  ContaminationType,
  ImageAnalysis,
  ContaminationZone,

  // Electrolyzed water
  ElectrolyzedWater,
  EWGenerationParams,
  CleaningEfficiencyParams,
  CleaningEfficiencyResult,

  // UV-C
  UVCParameters,
  PathogenType,
  UVCValidationResult,
  UVLampStatus,

  // Plasma
  PlasmaParameters,
  PlasmaCleaningRate,

  // Ozone
  OzoneParameters,
  OzoneGeneratorConfig,
  OzoneSafety,
  OzoneAlert,

  // Cleaning session
  CleaningSession,
  CleaningPlan,
  CleaningMethod,
  OptimizationCriteria,
  CleaningStage,
  CleaningExecution,
  ExecutedStage,
  CleaningProgress,
  CleaningResults,
  EnvironmentalImpact,

  // Sensors
  SensorTelemetry,
  ElectrolyzedWaterSensor,
  PlasmaSensor,
  OzoneSensor,
  EnvironmentalSensor,
  SystemStatus,
  SystemAlarm,

  // Equipment
  EquipmentConfig,
  EWGeneratorConfig,
  UVSystemConfig,
  PlasmaGeneratorConfig,
  WaterSystemConfig,

  // API
  StartCleaningRequest,
  StartCleaningResponse,
  GetSessionStatusRequest,
  GetSessionStatusResponse,
  AnalyticsRequest,
  AnalyticsMetric,
  AnalyticsResponse,
  AnalyticsDataPoint,

  // Utility
  ValidationResult,
};

export { CLEANING_CONSTANTS, CleaningErrorCode, CleaningSystemError };
