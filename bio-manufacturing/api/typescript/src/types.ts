/**
 * WIA-BIO-015: Bio-Manufacturing - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bio-Manufacturing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Bio-Manufacturing Types
// ============================================================================

/**
 * Expression system types
 */
export type ExpressionSystem =
  | 'CHO'
  | 'E.coli'
  | 'Yeast'
  | 'Pichia'
  | 'Insect'
  | 'HEK-293'
  | 'PER.C6'
  | 'custom';

/**
 * Product types
 */
export type ProductType =
  | 'antibody'
  | 'protein'
  | 'enzyme'
  | 'vaccine'
  | 'viral-vector'
  | 'cell-therapy'
  | 'gene-therapy'
  | 'biofuel'
  | 'custom';

/**
 * Bioreactor operation modes
 */
export type OperationMode = 'batch' | 'fed-batch' | 'perfusion' | 'continuous';

/**
 * Cell line information
 */
export interface CellLine {
  /** Cell line identifier */
  id: string;

  /** Cell line name */
  name: string;

  /** Expression system */
  system: ExpressionSystem;

  /** Product being expressed */
  product: string;

  /** Product type */
  productType: ProductType;

  /** Typical growth rate (h⁻¹) */
  growthRate: number;

  /** Typical doubling time (hours) */
  doublingTime: number;

  /** Typical product titer (g/L) */
  typicalTiter: number;

  /** Passage stability (0-1) */
  stability: number;

  /** Generation number */
  generation?: number;
}

// ============================================================================
// Fermentation Parameters
// ============================================================================

/**
 * Culture conditions
 */
export interface CultureConditions {
  /** Temperature in °C */
  temperature: number;

  /** pH value */
  pH: number;

  /** Dissolved oxygen (% of air saturation) */
  dissolvedOxygen: number;

  /** Agitation speed (rpm) */
  agitation?: number;

  /** Aeration rate (vvm - volume per volume per minute) */
  aeration?: number;

  /** Pressure (bar) */
  pressure?: number;

  /** CO₂ concentration (%) */
  co2?: number;
}

/**
 * Cell culture metrics
 */
export interface CellCultureMetrics {
  /** Viable cell density (cells/mL) */
  cellDensity: number;

  /** Total cell density (cells/mL) */
  totalCellDensity?: number;

  /** Viability (%) */
  viability: number;

  /** Specific growth rate (h⁻¹) */
  growthRate?: number;

  /** Product concentration (g/L) */
  productConcentration?: number;

  /** Glucose concentration (g/L) */
  glucose?: number;

  /** Lactate concentration (mM) */
  lactate?: number;

  /** Glutamine concentration (mM) */
  glutamine?: number;

  /** Ammonia concentration (mM) */
  ammonia?: number;
}

/**
 * Fermentation parameters for calculations
 */
export interface FermentationParameters {
  /** Initial cell density (cells/mL) */
  initialCellDensity: number;

  /** Culture time (hours) */
  cultureTime: number;

  /** Working volume (L) */
  volume: number;

  /** Initial substrate concentration (g/L) */
  initialSubstrate?: number;

  /** Feed rate (L/h) for fed-batch */
  feedRate?: number;

  /** Feed substrate concentration (g/L) */
  feedConcentration?: number;
}

// ============================================================================
// Bioreactor Configuration
// ============================================================================

/**
 * Bioreactor specifications
 */
export interface BioreactorConfig {
  /** Reactor identifier */
  id: string;

  /** Reactor name */
  name: string;

  /** Reactor type */
  type: 'stirred-tank' | 'wave' | 'perfusion' | 'airlift' | 'custom';

  /** Total volume (L) */
  totalVolume: number;

  /** Working volume (L) */
  workingVolume: number;

  /** Operation mode */
  mode: OperationMode;

  /** Height to diameter ratio */
  heightDiameterRatio?: number;

  /** Impeller type */
  impellerType?: 'rushton' | 'pitched-blade' | 'marine' | 'custom';

  /** Number of impellers */
  numberOfImpellers?: number;

  /** Sparger type */
  spargerType?: 'ring' | 'sintered' | 'drilled-pipe' | 'custom';
}

/**
 * Real-time bioreactor status
 */
export interface BioreactorStatus {
  /** Reactor identifier */
  reactorId: string;

  /** Timestamp */
  timestamp: Date;

  /** Current culture conditions */
  conditions: CultureConditions;

  /** Current cell metrics */
  metrics: CellCultureMetrics;

  /** Reactor phase */
  phase: 'inoculation' | 'lag' | 'exponential' | 'stationary' | 'death' | 'harvest';

  /** Alarms */
  alarms: BioreactorAlarm[];

  /** Operational status */
  status: 'running' | 'paused' | 'error' | 'complete';
}

/**
 * Bioreactor alarm
 */
export interface BioreactorAlarm {
  /** Alarm type */
  type: 'pH' | 'temperature' | 'DO' | 'pressure' | 'viability' | 'contamination' | 'other';

  /** Alarm severity */
  severity: 'info' | 'warning' | 'critical';

  /** Alarm message */
  message: string;

  /** Timestamp */
  timestamp: Date;

  /** Acknowledged */
  acknowledged: boolean;
}

// ============================================================================
// Yield and Productivity
// ============================================================================

/**
 * Yield calculation parameters
 */
export interface YieldParameters {
  /** Final product concentration (g/L) */
  productConcentration: number;

  /** Final volume (L) */
  finalVolume: number;

  /** Initial substrate concentration (g/L) */
  substrateConcentration: number;

  /** Initial volume (L) */
  initialVolume: number;

  /** Feed volume (L) - for fed-batch */
  feedVolume?: number;

  /** Feed substrate concentration (g/L) */
  feedSubstrate?: number;
}

/**
 * Yield calculation result
 */
export interface YieldResult {
  /** Product yield (dimensionless) */
  yield: number;

  /** Total product mass (g) */
  totalProduct: number;

  /** Total substrate mass (g) */
  totalSubstrate: number;

  /** Conversion efficiency (%) */
  efficiency: number;

  /** Performance assessment */
  performance: 'excellent' | 'good' | 'fair' | 'poor';

  /** Yield formatted */
  yieldFormatted: string;
}

/**
 * Productivity calculation parameters
 */
export interface ProductivityParameters {
  /** Final product concentration (g/L) */
  finalConcentration: number;

  /** Initial product concentration (g/L) */
  initialConcentration: number;

  /** Culture time (hours) */
  cultureTime: number;

  /** Working volume (L) */
  volume?: number;
}

/**
 * Productivity calculation result
 */
export interface ProductivityResult {
  /** Volumetric productivity (g/L/h) */
  value: number;

  /** Daily productivity (g/L/day) */
  perDay: number;

  /** Total product (g) if volume provided */
  totalProduct?: number;

  /** Performance assessment */
  performance: 'excellent' | 'good' | 'fair' | 'poor';

  /** Productivity formatted */
  formatted: string;
}

/**
 * Specific productivity
 */
export interface SpecificProductivity {
  /** Cell-specific productivity (pg/cell/day) */
  qp: number;

  /** Average viable cell density (cells/mL) */
  averageVCD: number;

  /** Integral of viable cells (IVC) */
  ivc: number;

  /** Product-to-cell yield (g/cell/h) */
  yieldPerCell: number;
}

// ============================================================================
// Growth Kinetics
// ============================================================================

/**
 * Growth kinetics parameters
 */
export interface GrowthKinetics {
  /** Specific growth rate (h⁻¹) */
  specificGrowthRate: number;

  /** Doubling time (hours) */
  doublingTime: number;

  /** Maximum growth rate (h⁻¹) */
  maxGrowthRate?: number;

  /** Lag time (hours) */
  lagTime?: number;

  /** Saturation constant Ks (g/L) */
  saturationConstant?: number;
}

/**
 * Substrate consumption
 */
export interface SubstrateConsumption {
  /** Specific consumption rate (g/cell/h or mmol/cell/h) */
  specificRate: number;

  /** Yield coefficient (cells/g substrate) */
  yieldCoefficient: number;

  /** Maintenance coefficient (g/cell/h) */
  maintenanceCoefficient?: number;

  /** Substrate type */
  substrate: 'glucose' | 'glutamine' | 'custom';
}

// ============================================================================
// Downstream Processing
// ============================================================================

/**
 * Chromatography step
 */
export interface ChromatographyStep {
  /** Step number */
  stepNumber: number;

  /** Step name */
  name: string;

  /** Chromatography type */
  type: 'protein-a' | 'cation-exchange' | 'anion-exchange' | 'hydrophobic' | 'size-exclusion' | 'multimodal';

  /** Resin volume (L) */
  resinVolume: number;

  /** Binding capacity (g/L resin) */
  bindingCapacity: number;

  /** Load volume (L) */
  loadVolume: number;

  /** Expected recovery (%) */
  recovery: number;

  /** Expected purity (%) */
  purity: number;

  /** Residence time (minutes) */
  residenceTime?: number;

  /** Buffer compositions */
  buffers?: {
    equilibration?: string;
    load?: string;
    wash?: string;
    elution?: string;
  };
}

/**
 * Purification protocol
 */
export interface PurificationProtocol {
  /** Protocol identifier */
  id: string;

  /** Protocol name */
  name: string;

  /** Product type */
  productType: ProductType;

  /** Chromatography steps */
  steps: ChromatographyStep[];

  /** Viral inactivation included */
  viralInactivation: boolean;

  /** Overall recovery (%) */
  overallRecovery: number;

  /** Final purity (%) */
  finalPurity: number;

  /** Estimated duration (hours) */
  duration: number;

  /** Estimated cost ($ per gram) */
  costPerGram?: number;
}

/**
 * Purification design parameters
 */
export interface PurificationDesign {
  /** Product type */
  productType: ProductType;

  /** Scale (L of harvest) */
  scale: number;

  /** Starting concentration (g/L) */
  startingConcentration: number;

  /** Target purity (%) */
  targetPurity: number;

  /** Target recovery (%) */
  targetRecovery: number;

  /** Budget constraint ($ per gram) */
  budgetConstraint?: number;
}

// ============================================================================
// Quality Control
// ============================================================================

/**
 * Critical Quality Attribute (CQA)
 */
export interface CriticalQualityAttribute {
  /** Attribute name */
  name: string;

  /** Measured value */
  value: number;

  /** Unit */
  unit: string;

  /** Lower specification limit */
  lsl?: number;

  /** Upper specification limit */
  usl?: number;

  /** Target value */
  target?: number;

  /** In specification */
  inSpec: boolean;

  /** Criticality */
  criticality: 'high' | 'medium' | 'low';
}

/**
 * Quality control result
 */
export interface QualityControlResult {
  /** Batch identifier */
  batchId: string;

  /** Product name */
  product: string;

  /** Test date */
  testDate: Date;

  /** Critical quality attributes */
  cqas: CriticalQualityAttribute[];

  /** Overall status */
  status: 'pass' | 'fail' | 'investigate';

  /** Certificate of Analysis */
  coa?: string;

  /** Deviations */
  deviations: string[];
}

/**
 * Process capability metrics
 */
export interface ProcessCapability {
  /** Attribute name */
  attribute: string;

  /** Process mean */
  mean: number;

  /** Process standard deviation */
  stdDev: number;

  /** Cp (process capability) */
  cp: number;

  /** Cpk (process capability index) */
  cpk: number;

  /** Capability assessment */
  assessment: 'capable' | 'marginally-capable' | 'not-capable';

  /** Recommendation */
  recommendation?: string;
}

// ============================================================================
// Process Analytical Technology (PAT)
// ============================================================================

/**
 * PAT sensor reading
 */
export interface SensorReading {
  /** Sensor identifier */
  sensorId: string;

  /** Parameter being measured */
  parameter: string;

  /** Measured value */
  value: number;

  /** Unit */
  unit: string;

  /** Timestamp */
  timestamp: Date;

  /** Quality indicator */
  quality: 'good' | 'uncertain' | 'bad';

  /** Calibration status */
  calibrated: boolean;
}

/**
 * Process trend
 */
export interface ParameterTrend {
  /** Parameter name */
  parameter: string;

  /** Current value */
  current: number;

  /** Trend direction */
  trend: 'increasing' | 'stable' | 'decreasing';

  /** Rate of change */
  rateOfChange: number;

  /** Predicted value in 1 hour */
  predicted?: number;

  /** Alert status */
  alert: 'none' | 'warning' | 'critical';
}

// ============================================================================
// Optimization
// ============================================================================

/**
 * Design of Experiments (DOE) configuration
 */
export interface DOEConfig {
  /** DOE type */
  type: 'full-factorial' | 'fractional-factorial' | 'central-composite' | 'box-behnken';

  /** Factors to optimize */
  factors: DOEFactor[];

  /** Response to measure */
  responses: string[];

  /** Number of center points */
  centerPoints?: number;

  /** Number of replicates */
  replicates?: number;
}

/**
 * DOE factor
 */
export interface DOEFactor {
  /** Factor name */
  name: string;

  /** Low level */
  lowLevel: number;

  /** High level */
  highLevel: number;

  /** Center point */
  centerPoint?: number;

  /** Unit */
  unit: string;

  /** Factor type */
  type: 'continuous' | 'categorical';
}

/**
 * Optimization result
 */
export interface OptimizationResult {
  /** Optimization identifier */
  id: string;

  /** Optimized parameters */
  parameters: Record<string, number>;

  /** Predicted response */
  predictedResponse: number;

  /** Confidence interval */
  confidenceInterval: [number, number];

  /** Improvement over baseline (%) */
  improvement: number;

  /** Recommendation */
  recommendation: string;
}

// ============================================================================
// Monitoring and Validation
// ============================================================================

/**
 * Monitoring result
 */
export interface MonitoringResult {
  /** Reactor identifier */
  reactorId: string;

  /** Timestamp */
  timestamp: Date;

  /** Overall status */
  status: 'optimal' | 'acceptable' | 'warning' | 'critical';

  /** Warnings */
  warnings: string[];

  /** Critical alarms */
  criticalAlarms: string[];

  /** Recommendations */
  recommendations: string[];

  /** Parameter trends */
  trends: ParameterTrend[];

  /** Predicted harvest time */
  predictedHarvest?: Date;
}

/**
 * Validation parameters
 */
export interface ValidationParameters {
  /** Batch identifier */
  batchId: string;

  /** Target titer (g/L) */
  targetTiter: number;

  /** Target viability (%) */
  targetViability: number;

  /** Maximum lactate (mM) */
  maxLactate: number;

  /** Maximum ammonia (mM) */
  maxAmmonia: number;

  /** GMP requirements */
  gmpRequired: boolean;
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Is valid */
  isValid: boolean;

  /** Errors (blocking) */
  errors: string[];

  /** Warnings (non-blocking) */
  warnings: string[];

  /** Compliance status */
  compliance: {
    gmp: boolean;
    cqas: boolean;
    sterility: boolean;
    documentation: boolean;
  };

  /** Quality score (0-100) */
  qualityScore: number;

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Bio-manufacturing constants
 */
export const BIO_CONSTANTS = {
  /** Avogadro's number (mol⁻¹) */
  AVOGADRO: 6.022e23,

  /** Gas constant (J/mol·K) */
  GAS_CONSTANT: 8.314,

  /** Standard temperature (K) */
  STANDARD_TEMP: 298,

  /** Standard pressure (Pa) */
  STANDARD_PRESSURE: 101325,

  /** Water density (kg/L) */
  WATER_DENSITY: 1.0,

  /** Typical cell density ranges (cells/mL) */
  CELL_DENSITY: {
    CHO_MIN: 1e5,
    CHO_MAX: 1e7,
    ECOLI_MIN: 1e8,
    ECOLI_MAX: 1e10,
    YEAST_MIN: 1e7,
    YEAST_MAX: 1e9,
  },

  /** Typical growth rates (h⁻¹) */
  GROWTH_RATE: {
    CHO: 0.03,
    ECOLI: 0.7,
    YEAST: 0.25,
  },

  /** Minimum acceptable viability (%) */
  MIN_VIABILITY: 70,

  /** Typical product titers (g/L) */
  TYPICAL_TITER: {
    ANTIBODY: 5.0,
    PROTEIN: 2.0,
    ENZYME: 3.0,
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
 * Time series data point
 */
export interface TimeSeriesPoint {
  /** Timestamp */
  time: Date;

  /** Value */
  value: number;

  /** Parameter name */
  parameter: string;
}

/**
 * Batch summary
 */
export interface BatchSummary {
  /** Batch identifier */
  batchId: string;

  /** Product */
  product: string;

  /** Cell line */
  cellLine: string;

  /** Start date */
  startDate: Date;

  /** Harvest date */
  harvestDate?: Date;

  /** Final titer (g/L) */
  finalTiter?: number;

  /** Total product (g) */
  totalProduct?: number;

  /** Productivity (g/L/day) */
  productivity?: number;

  /** Status */
  status: 'running' | 'harvested' | 'purifying' | 'complete' | 'failed';

  /** Quality status */
  qualityStatus?: 'pass' | 'fail' | 'pending';
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-015 error codes
 */
export enum BioErrorCode {
  LOW_VIABILITY = 'B001',
  HIGH_LACTATE = 'B002',
  LOW_TITER = 'B003',
  pH_EXCURSION = 'B004',
  CONTAMINATION = 'B005',
  EQUIPMENT_FAILURE = 'B006',
  INVALID_PARAMETERS = 'B007',
  GMP_VIOLATION = 'B008',
  OUT_OF_SPEC = 'B009',
  INSUFFICIENT_DATA = 'B010',
}

/**
 * Bio-manufacturing error
 */
export class BioManufacturingError extends Error {
  constructor(
    public code: BioErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BioManufacturingError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  CellLine,
  CultureConditions,
  CellCultureMetrics,
  FermentationParameters,

  // Bioreactor
  BioreactorConfig,
  BioreactorStatus,
  BioreactorAlarm,

  // Yield and productivity
  YieldParameters,
  YieldResult,
  ProductivityParameters,
  ProductivityResult,
  SpecificProductivity,

  // Growth
  GrowthKinetics,
  SubstrateConsumption,

  // Downstream
  ChromatographyStep,
  PurificationProtocol,
  PurificationDesign,

  // Quality
  CriticalQualityAttribute,
  QualityControlResult,
  ProcessCapability,

  // PAT
  SensorReading,
  ParameterTrend,

  // Optimization
  DOEConfig,
  DOEFactor,
  OptimizationResult,

  // Monitoring
  MonitoringResult,
  ValidationParameters,
  ValidationResult,

  // Utility
  TimeSeriesPoint,
  BatchSummary,
};

export { BIO_CONSTANTS, BioErrorCode, BioManufacturingError };
