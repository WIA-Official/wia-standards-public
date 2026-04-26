/**
 * WIA-IND-026: Predictive Maintenance
 * TypeScript Type Definitions
 *
 * @version 1.0.0
 * @standard WIA-IND-026
 * @category IND (Industry)
 */

// ============================================================================
// Core Configuration
// ============================================================================

export interface PredictiveMaintenanceConfig {
  /** API key for authentication */
  apiKey: string;

  /** Organization ID */
  organizationId: string;

  /** API endpoint URL */
  endpoint?: string;

  /** Request timeout in milliseconds */
  timeout?: number;

  /** Retry configuration */
  retry?: {
    maxRetries: number;
    retryDelay: number;
    exponentialBackoff: boolean;
  };

  /** Enable debug logging */
  debug?: boolean;
}

// ============================================================================
// Asset Management
// ============================================================================

export enum AssetType {
  ROTATING_MACHINERY = 'ROTATING_MACHINERY',
  PUMP = 'PUMP',
  MOTOR = 'MOTOR',
  GEARBOX = 'GEARBOX',
  COMPRESSOR = 'COMPRESSOR',
  FAN = 'FAN',
  TURBINE = 'TURBINE',
  CONVEYOR = 'CONVEYOR',
  HVAC = 'HVAC',
  TRANSFORMER = 'TRANSFORMER',
  SWITCHGEAR = 'SWITCHGEAR',
  BEARING = 'BEARING',
  VALVE = 'VALVE',
  HEAT_EXCHANGER = 'HEAT_EXCHANGER',
  CUSTOM = 'CUSTOM'
}

export interface AssetSpecification {
  manufacturer?: string;
  model?: string;
  serialNumber?: string;
  installationDate?: string;
  powerRating?: string;
  operatingSpeed?: string;
  capacity?: string;
  [key: string]: any;
}

export interface Asset {
  /** Unique asset identifier */
  assetId: string;

  /** Asset type */
  assetType: AssetType;

  /** Physical location */
  location: string;

  /** Asset name/description */
  name?: string;

  /** Technical specifications */
  specifications?: AssetSpecification;

  /** Parent asset ID (for hierarchies) */
  parentAssetId?: string;

  /** Asset status */
  status?: 'ACTIVE' | 'INACTIVE' | 'MAINTENANCE' | 'DECOMMISSIONED';

  /** Criticality level */
  criticality?: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

  /** Custom metadata */
  metadata?: Record<string, any>;

  /** Creation timestamp */
  createdAt?: Date;

  /** Last update timestamp */
  updatedAt?: Date;
}

export interface AssetRegisterRequest {
  assetId: string;
  assetType: AssetType;
  location: string;
  name?: string;
  specifications?: AssetSpecification;
  parentAssetId?: string;
  criticality?: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  metadata?: Record<string, any>;
}

// ============================================================================
// Sensor Management
// ============================================================================

export enum SensorType {
  VIBRATION = 'VIBRATION',
  TEMPERATURE = 'TEMPERATURE',
  THERMAL_CAMERA = 'THERMAL_CAMERA',
  ACOUSTIC = 'ACOUSTIC',
  ULTRASONIC = 'ULTRASONIC',
  OIL_PARTICLE_COUNTER = 'OIL_PARTICLE_COUNTER',
  OIL_VISCOSITY = 'OIL_VISCOSITY',
  OIL_WATER_CONTENT = 'OIL_WATER_CONTENT',
  CURRENT = 'CURRENT',
  VOLTAGE = 'VOLTAGE',
  PRESSURE = 'PRESSURE',
  FLOW = 'FLOW',
  DISPLACEMENT = 'DISPLACEMENT',
  SPEED = 'SPEED',
  TORQUE = 'TORQUE',
  CUSTOM = 'CUSTOM'
}

export interface Sensor {
  /** Unique sensor identifier */
  sensorId: string;

  /** Sensor type */
  type: SensorType;

  /** Asset this sensor is attached to */
  assetId: string;

  /** Physical location on asset */
  location: string;

  /** Sampling rate in Hz */
  samplingRate: number;

  /** Measurement unit */
  unit?: string;

  /** Sensor manufacturer and model */
  model?: string;

  /** Calibration date */
  calibrationDate?: Date;

  /** Next calibration due date */
  nextCalibrationDate?: Date;

  /** Sensor status */
  status?: 'ACTIVE' | 'INACTIVE' | 'FAULTY' | 'CALIBRATION';

  /** Custom metadata */
  metadata?: Record<string, any>;
}

export interface SensorAttachRequest {
  assetId: string;
  sensors: Array<{
    sensorId: string;
    type: SensorType;
    location: string;
    samplingRate: number;
    unit?: string;
    model?: string;
    metadata?: Record<string, any>;
  }>;
}

// ============================================================================
// Data Collection
// ============================================================================

export enum DataQuality {
  GOOD = 'GOOD',
  UNCERTAIN = 'UNCERTAIN',
  BAD = 'BAD'
}

export interface DataPoint {
  /** Unix timestamp in milliseconds */
  timestamp: number;

  /** Sensor ID */
  sensorId: string;

  /** Asset ID */
  assetId: string;

  /** Measured value(s) */
  value: number | number[];

  /** Measurement unit */
  unit: string;

  /** Data quality indicator */
  quality: DataQuality;

  /** Additional metadata */
  metadata?: {
    operatingCondition?: string;
    environmentalTemp?: number;
    load?: number;
    speed?: number;
    [key: string]: any;
  };
}

export interface DataIngestRequest {
  sensorId: string;
  timestamp: number;
  values: number | number[];
  unit: string;
  quality?: DataQuality;
  metadata?: Record<string, any>;
}

export interface DataStreamOptions {
  assetId: string;
  sensorIds?: string[];
  onData: (data: DataPoint) => void | Promise<void>;
  onError?: (error: Error) => void;
  onClose?: () => void;
}

// ============================================================================
// Vibration Analysis
// ============================================================================

export enum AnalysisType {
  // Time domain
  RMS = 'RMS',
  PEAK = 'PEAK',
  PEAK_TO_PEAK = 'PEAK_TO_PEAK',
  CREST_FACTOR = 'CREST_FACTOR',
  KURTOSIS = 'KURTOSIS',
  SKEWNESS = 'SKEWNESS',

  // Frequency domain
  FFT = 'FFT',
  POWER_SPECTRUM = 'POWER_SPECTRUM',
  ENVELOPE = 'ENVELOPE',
  ORDER_ANALYSIS = 'ORDER_ANALYSIS',

  // Advanced
  ORBIT_ANALYSIS = 'ORBIT_ANALYSIS',
  WATERFALL = 'WATERFALL',
  SPECTROGRAM = 'SPECTROGRAM'
}

export interface VibrationAnalysisRequest {
  assetId: string;
  sensorId: string;
  analysisTypes: AnalysisType[];
  startTime?: number;
  endTime?: number;
  bearingInfo?: BearingInformation;
  gearInfo?: GearInformation;
}

export interface BearingInformation {
  numberOfBalls: number;
  ballDiameter: number;
  pitchDiameter: number;
  contactAngle: number;
  shaftSpeed: number; // RPM
}

export interface GearInformation {
  numberOfTeeth: number;
  shaftSpeed: number; // RPM
}

export interface VibrationAnalysisResult {
  assetId: string;
  sensorId: string;
  timestamp: number;

  /** Time-domain metrics */
  timeDomain?: {
    rms?: number;
    peak?: number;
    peakToPeak?: number;
    crestFactor?: number;
    kurtosis?: number;
    skewness?: number;
  };

  /** Frequency-domain data */
  frequencyDomain?: {
    frequencies: number[];
    amplitudes: number[];
    dominantFrequencies: Array<{
      frequency: number;
      amplitude: number;
      harmonic?: number; // 1X, 2X, 3X, etc.
    }>;
  };

  /** Bearing condition analysis */
  bearingCondition?: {
    overall: 'GOOD' | 'ACCEPTABLE' | 'UNSATISFACTORY' | 'UNACCEPTABLE';
    defectFrequencies: {
      BPFO?: { frequency: number; amplitude: number; detected: boolean };
      BPFI?: { frequency: number; amplitude: number; detected: boolean };
      BSF?: { frequency: number; amplitude: number; detected: boolean };
      FTF?: { frequency: number; amplitude: number; detected: boolean };
    };
    estimatedDefectSeverity?: number; // 0-100
  };

  /** Gear condition analysis */
  gearCondition?: {
    overall: 'GOOD' | 'ACCEPTABLE' | 'UNSATISFACTORY' | 'UNACCEPTABLE';
    GMF?: { frequency: number; amplitude: number };
    sidebands?: Array<{ frequency: number; amplitude: number }>;
  };

  /** Severity assessment per ISO 20816 */
  severity?: {
    zone: 'A' | 'B' | 'C' | 'D';
    description: 'Good' | 'Acceptable' | 'Unsatisfactory' | 'Unacceptable';
    rmsVelocity: number; // mm/s
  };

  /** Remaining useful life estimate */
  remainingUsefulLife?: {
    days: number;
    confidence: number; // 0-1
  };

  /** Recommendations */
  recommendations?: string[];
}

// ============================================================================
// Thermal Analysis
// ============================================================================

export interface ThermalAnalysisRequest {
  assetId: string;
  imageData: Buffer | string; // Raw image or base64
  referenceTemperature?: number;
  emissivity?: number;
  algorithm?: 'HOTSPOT_DETECTION' | 'ANOMALY_DETECTION' | 'PATTERN_RECOGNITION';
  historicalBaseline?: boolean;
}

export interface Hotspot {
  location: { x: number; y: number };
  temperature: number;
  delta: number; // ΔT from reference
  severity: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  area: number; // pixels
}

export interface ThermalAnalysisResult {
  assetId: string;
  timestamp: number;

  /** Detected hotspots */
  hotspots: Hotspot[];

  /** Temperature statistics */
  statistics: {
    min: number;
    max: number;
    mean: number;
    stdDev: number;
  };

  /** Overall assessment */
  assessment: 'NORMAL' | 'CAUTION' | 'WARNING' | 'CRITICAL';

  /** Anomaly score (0-100) */
  anomalyScore?: number;

  /** Thermal pattern classification */
  pattern?: string;

  /** Recommendations */
  recommendations?: string[];

  /** Annotated image (base64) */
  annotatedImage?: string;
}

// ============================================================================
// Oil Analysis
// ============================================================================

export interface OilAnalysisRequest {
  assetId: string;
  sampleId: string;
  sampleDate?: Date;
  tests: {
    viscosity?: number; // cSt
    viscosityTemp?: number; // °C
    waterContent?: number; // ppm
    TAN?: number; // mg KOH/g
    TBN?: number; // mg KOH/g
    particleCount?: {
      '4μm'?: number;
      '6μm'?: number;
      '14μm'?: number;
      '21μm'?: number;
    };
    metals?: {
      iron?: number; // ppm
      copper?: number;
      aluminum?: number;
      chromium?: number;
      tin?: number;
      lead?: number;
      [key: string]: number | undefined;
    };
    [key: string]: any;
  };
}

export interface OilAnalysisResult {
  assetId: string;
  sampleId: string;
  timestamp: number;

  /** Overall oil condition */
  condition: 'EXCELLENT' | 'GOOD' | 'FAIR' | 'POOR' | 'CRITICAL';

  /** ISO cleanliness code */
  isoCode?: string; // e.g., "18/16/13"

  /** Individual test results with thresholds */
  testResults: {
    viscosity?: {
      value: number;
      status: 'NORMAL' | 'CAUTION' | 'WARNING' | 'CRITICAL';
      changeFromNew?: number; // percentage
    };
    waterContent?: {
      value: number;
      status: 'NORMAL' | 'CAUTION' | 'WARNING' | 'CRITICAL';
    };
    contamination?: {
      level: 'LOW' | 'MEDIUM' | 'HIGH' | 'SEVERE';
      particleCount?: Record<string, number>;
    };
    wear?: {
      overall: 'NORMAL' | 'ELEVATED' | 'HIGH' | 'SEVERE';
      metals: Record<string, {
        value: number;
        status: 'NORMAL' | 'CAUTION' | 'WARNING' | 'CRITICAL';
      }>;
    };
  };

  /** Trending data */
  trend?: 'STABLE' | 'IMPROVING' | 'DEGRADING' | 'RAPID_DEGRADATION';

  /** Recommendations */
  recommendations: string[];

  /** Next sample due date */
  nextSampleDate?: Date;
}

// ============================================================================
// Acoustic Analysis
// ============================================================================

export interface AcousticAnalysisRequest {
  assetId: string;
  audioData: Buffer;
  samplingRate: number;
  analysisTypes: Array<'LEAK_DETECTION' | 'CAVITATION' | 'MECHANICAL_LOOSENESS' | 'BEARING_DEFECTS' | 'ELECTRICAL_DISCHARGE'>;
}

export interface AcousticAnomaly {
  type: string;
  frequency: number; // Hz
  amplitude: number; // dB
  severity: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  location?: string;
  confidence: number; // 0-1
}

export interface AcousticAnalysisResult {
  assetId: string;
  timestamp: number;

  /** Detected anomalies */
  anomalies: AcousticAnomaly[];

  /** Overall sound level */
  overallLevel: number; // dB

  /** Frequency spectrum */
  spectrum?: {
    frequencies: number[];
    amplitudes: number[];
  };

  /** Leak rate estimate (if applicable) */
  leakRate?: {
    value: number;
    unit: string; // e.g., "CFM"
  };

  /** Assessment */
  assessment: 'NORMAL' | 'MONITOR' | 'INVESTIGATE' | 'IMMEDIATE_ACTION';

  /** Recommendations */
  recommendations?: string[];
}

// ============================================================================
// Predictive Models
// ============================================================================

export enum FailureMode {
  BEARING_INNER_RACE = 'BEARING_INNER_RACE',
  BEARING_OUTER_RACE = 'BEARING_OUTER_RACE',
  BEARING_BALL = 'BEARING_BALL',
  BEARING_CAGE = 'BEARING_CAGE',
  BEARING_LUBRICATION = 'BEARING_LUBRICATION',
  GEAR_TOOTH_WEAR = 'GEAR_TOOTH_WEAR',
  GEAR_TOOTH_BREAKAGE = 'GEAR_TOOTH_BREAKAGE',
  MISALIGNMENT = 'MISALIGNMENT',
  IMBALANCE = 'IMBALANCE',
  LOOSENESS = 'LOOSENESS',
  RESONANCE = 'RESONANCE',
  SHAFT_CRACK = 'SHAFT_CRACK',
  ROTOR_BAR_BREAKAGE = 'ROTOR_BAR_BREAKAGE',
  WINDING_FAILURE = 'WINDING_FAILURE',
  INSULATION_DEGRADATION = 'INSULATION_DEGRADATION',
  SEAL_FAILURE = 'SEAL_FAILURE',
  CAVITATION = 'CAVITATION',
  OVERHEATING = 'OVERHEATING',
  CONTAMINATION = 'CONTAMINATION',
  CORROSION = 'CORROSION',
  EROSION = 'EROSION',
  CUSTOM = 'CUSTOM'
}

export interface PredictionRequest {
  assetId: string;
  timeHorizon?: number; // days
  threshold?: number; // probability threshold (0-100)
  includeRecommendations?: boolean;
}

export interface FailurePrediction {
  failureMode: FailureMode | string;
  probability: number; // 0-100
  severity: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  daysToFailure: number;
  confidence: number; // 0-1
  contributingFactors: Array<{
    factor: string;
    importance: number; // 0-100
  }>;
  recommendedMaintenanceDate: Date;
  recommendations: string[];
}

export interface PredictionResult {
  assetId: string;
  timestamp: number;
  predictions: FailurePrediction[];
  overallRisk: number; // 0-100
  remainingUsefulLife?: {
    days: number;
    confidence: number;
    lowerBound: number;
    upperBound: number;
  };
}

// ============================================================================
// Maintenance Management
// ============================================================================

export enum WorkOrderPriority {
  LOW = 'LOW',
  MEDIUM = 'MEDIUM',
  HIGH = 'HIGH',
  CRITICAL = 'CRITICAL'
}

export enum WorkOrderStatus {
  OPEN = 'OPEN',
  SCHEDULED = 'SCHEDULED',
  IN_PROGRESS = 'IN_PROGRESS',
  COMPLETED = 'COMPLETED',
  CANCELLED = 'CANCELLED'
}

export interface RequiredPart {
  partNumber: string;
  description: string;
  quantity: number;
  availability: boolean;
  leadTime?: number; // days
  cost?: number;
}

export interface WorkOrder {
  workOrderId: string;
  assetId: string;
  priority: WorkOrderPriority;
  predictedFailureMode?: FailureMode | string;
  failureProbability?: number;
  remainingUsefulLife?: {
    days: number;
    confidence: number;
  };
  recommendedActions: string[];
  requiredParts?: RequiredPart[];
  estimatedDuration?: number; // hours
  estimatedCost?: number;
  requiredSkills?: string[];
  scheduledDate?: Date;
  completedDate?: Date;
  createdBy: 'SYSTEM' | 'USER';
  createdAt: Date;
  updatedAt?: Date;
  status: WorkOrderStatus;
  notes?: string;
  metadata?: Record<string, any>;
}

export interface CreateWorkOrderRequest {
  assetId: string;
  priority: WorkOrderPriority;
  predictedFailure?: string;
  recommendedActions: string[];
  requiredParts?: RequiredPart[];
  scheduledDate?: Date;
  notes?: string;
  metadata?: Record<string, any>;
}

export interface MaintenanceWindow {
  start: Date;
  end: Date;
  description?: string;
}

export interface OptimizeScheduleRequest {
  assets: string[];
  constraints?: {
    maintenanceWindows?: MaintenanceWindow[];
    availableCrews?: number;
    spareParts?: Record<string, number>;
    maxDowntime?: number; // hours
  };
  objectives?: Array<'MINIMIZE_DOWNTIME' | 'MINIMIZE_COST' | 'MAXIMIZE_RELIABILITY'>;
}

export interface MaintenanceBundle {
  bundleId: string;
  assets: string[];
  workOrders: WorkOrder[];
  window: MaintenanceWindow;
  totalDowntime: number; // hours
  totalCost: number;
  savings: number;
  assignedCrew?: string[];
}

export interface OptimizeScheduleResult {
  bundles: MaintenanceBundle[];
  unbundledWorkOrders: WorkOrder[];
  totalDowntime: number;
  totalCost: number;
  totalSavings: number;
  optimization: {
    algorithm: string;
    executionTime: number; // ms
    objectives: Record<string, number>;
  };
}

// ============================================================================
// Inventory Management
// ============================================================================

export interface SparePart {
  partNumber: string;
  description: string;
  quantity: number;
  location: string;
  cost: number;
  leadTime: number; // days
  criticality: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  minimumStock: number;
  reorderPoint: number;
  lastRestocked?: Date;
}

export interface InventoryCheckResult {
  parts: SparePart[];
  lowStockItems: SparePart[];
  outOfStockItems: SparePart[];
}

// ============================================================================
// Reporting and Analytics
// ============================================================================

export interface ReportRequest {
  assetIds?: string[];
  startDate: Date;
  endDate: Date;
  reportType: 'SUMMARY' | 'DETAILED' | 'TRENDS' | 'COMPLIANCE';
  includeCharts?: boolean;
  format?: 'JSON' | 'PDF' | 'EXCEL';
}

export interface AssetHealthReport {
  assetId: string;
  overallHealth: number; // 0-100
  healthTrend: 'IMPROVING' | 'STABLE' | 'DEGRADING';
  metrics: {
    vibration?: { value: number; status: string; trend: string };
    temperature?: { value: number; status: string; trend: string };
    oilCondition?: { value: string; status: string; trend: string };
  };
  recentPredictions: FailurePrediction[];
  maintenanceHistory: WorkOrder[];
}

export interface PerformanceMetrics {
  totalAssets: number;
  activeAssets: number;
  assetsAtRisk: number;
  predictionAccuracy: number; // percentage
  falsePositiveRate: number; // percentage
  caughtFailures: number;
  missedFailures: number;
  averageRULError: number; // days
  maintenanceCostSavings: number;
  downtimeReduction: number; // percentage
}

// ============================================================================
// Events and Notifications
// ============================================================================

export enum EventType {
  PREDICTION_ALARM = 'PREDICTION_ALARM',
  THRESHOLD_EXCEEDED = 'THRESHOLD_EXCEEDED',
  WORK_ORDER_CREATED = 'WORK_ORDER_CREATED',
  WORK_ORDER_COMPLETED = 'WORK_ORDER_COMPLETED',
  SENSOR_FAULT = 'SENSOR_FAULT',
  DATA_QUALITY_ISSUE = 'DATA_QUALITY_ISSUE',
  ASSET_STATUS_CHANGE = 'ASSET_STATUS_CHANGE',
  MAINTENANCE_DUE = 'MAINTENANCE_DUE'
}

export interface Event {
  eventId: string;
  eventType: EventType;
  timestamp: Date;
  assetId?: string;
  sensorId?: string;
  severity: 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';
  message: string;
  data?: Record<string, any>;
}

export interface NotificationChannel {
  type: 'EMAIL' | 'SMS' | 'WEBHOOK' | 'PUSH';
  destination: string;
  events: EventType[];
  enabled: boolean;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface ApiResponse<T = any> {
  status: 'success' | 'error';
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  meta?: {
    timestamp: number;
    requestId: string;
    rateLimit?: {
      limit: number;
      remaining: number;
      reset: number;
    };
  };
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  hasMore: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

export class PredictiveMaintenanceError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'PredictiveMaintenanceError';
    Object.setPrototypeOf(this, PredictiveMaintenanceError.prototype);
  }
}

export class ValidationError extends PredictiveMaintenanceError {
  constructor(message: string, details?: any) {
    super(message, 'VALIDATION_ERROR', 400, details);
    this.name = 'ValidationError';
  }
}

export class AuthenticationError extends PredictiveMaintenanceError {
  constructor(message: string = 'Authentication failed') {
    super(message, 'AUTHENTICATION_ERROR', 401);
    this.name = 'AuthenticationError';
  }
}

export class NotFoundError extends PredictiveMaintenanceError {
  constructor(resource: string, id: string) {
    super(`${resource} not found: ${id}`, 'NOT_FOUND', 404);
    this.name = 'NotFoundError';
  }
}

export class RateLimitError extends PredictiveMaintenanceError {
  constructor(
    public limit: number,
    public reset: number
  ) {
    super('Rate limit exceeded', 'RATE_LIMIT_EXCEEDED', 429, { limit, reset });
    this.name = 'RateLimitError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

export type DeepPartial<T> = {
  [P in keyof T]?: T[P] extends object ? DeepPartial<T[P]> : T[P];
};

export type TimeSeriesData<T> = Array<{
  timestamp: number;
  value: T;
}>;

export type Callback<T = void> = (error: Error | null, result?: T) => void;

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
