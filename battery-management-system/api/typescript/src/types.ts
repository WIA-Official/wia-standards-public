/**
 * WIA-AUTO-006: Battery Management System - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Battery Chemistry and Configuration
// ============================================================================

/**
 * Supported battery cell chemistries
 */
export type BatteryCellChemistry =
  | 'lithium-ion-nmc'    // Nickel Manganese Cobalt
  | 'lithium-ion-lfp'    // Lithium Iron Phosphate
  | 'lithium-ion-nca'    // Nickel Cobalt Aluminum
  | 'lithium-polymer'    // Li-Po
  | 'lithium-titanate'   // LTO
  | 'nimh'               // Nickel Metal Hydride
  | 'lead-acid'
  | 'sodium-ion'
  | 'solid-state';

/**
 * Battery cell configuration
 */
export interface BatteryCell {
  /** Unique cell identifier */
  id: string;

  /** Cell chemistry type */
  chemistry: BatteryCellChemistry;

  /** Nominal voltage (V) */
  nominalVoltage: number;

  /** Maximum voltage (V) */
  maxVoltage: number;

  /** Minimum voltage (V) */
  minVoltage: number;

  /** Rated capacity (Ah) */
  ratedCapacity: number;

  /** Current capacity (Ah) - may degrade over time */
  currentCapacity?: number;

  /** Internal resistance (Ohms) */
  internalResistance: number;

  /** Maximum continuous discharge current (A) */
  maxDischargeCurrent: number;

  /** Maximum continuous charge current (A) */
  maxChargeCurrent: number;

  /** Operating temperature range */
  temperatureRange: {
    minCharge: number;    // °C
    maxCharge: number;    // °C
    minDischarge: number; // °C
    maxDischarge: number; // °C
  };

  /** Manufacturing date */
  manufactureDate?: Date;

  /** Manufacturer name */
  manufacturer?: string;
}

/**
 * Battery pack configuration
 */
export interface BatteryPackConfig {
  /** Pack identifier */
  packId: string;

  /** Total number of cells */
  cellCount: number;

  /** Number of cells in series */
  seriesGroups: number;

  /** Number of cells in parallel */
  parallelGroups: number;

  /** Cell chemistry (all cells must match) */
  cellChemistry: BatteryCellChemistry;

  /** Nominal cell voltage (V) */
  nominalVoltage: number;

  /** Total pack capacity (Ah) */
  capacity: number;

  /** Maximum charge current (A) */
  maxChargeCurrent: number;

  /** Maximum discharge current (A) */
  maxDischargeCurrent: number;

  /** Maximum operating temperature (°C) */
  maxTemperature: number;

  /** Minimum operating temperature (°C) */
  minTemperature: number;
}

// ============================================================================
// State of Charge (SoC)
// ============================================================================

/**
 * SoC estimation method
 */
export type SoCEstimationMethod =
  | 'coulomb-counting'  // Ampere-hour integration
  | 'ocv'               // Open Circuit Voltage
  | 'ekf'               // Extended Kalman Filter
  | 'neural-network'    // AI-based estimation
  | 'fusion';           // Multi-method fusion

/**
 * SoC calculation request
 */
export interface SoCRequest {
  /** Estimation method */
  method: SoCEstimationMethod;

  /** Initial SoC (%) - required for coulomb counting */
  initialSoC?: number;

  /** Current (A) - negative for discharge */
  current: number;

  /** Voltage (V) - required for OCV method */
  voltage?: number;

  /** Temperature (°C) */
  temperature?: number;

  /** Duration of measurement (seconds) */
  duration: number;

  /** Battery capacity (Ah) */
  capacity: number;

  /** Coulombic efficiency (0-1) */
  efficiency?: number;
}

/**
 * SoC calculation response
 */
export interface SoCResponse {
  /** State of Charge (0-100%) */
  percentage: number;

  /** Remaining capacity (Ah) */
  remainingCapacity: number;

  /** Confidence level (0-1) */
  confidence: number;

  /** Method used */
  method: SoCEstimationMethod;

  /** Estimated error (%) */
  estimatedError?: number;

  /** Timestamp of calculation */
  timestamp: Date;
}

/**
 * SoC estimation state (for Kalman Filter)
 */
export interface SoCEstimationState {
  /** Current SoC estimate (%) */
  soc: number;

  /** RC network voltage (V) */
  vRC?: number;

  /** State covariance matrix */
  covariance?: number[][];

  /** Last update timestamp */
  lastUpdate: Date;
}

// ============================================================================
// State of Health (SoH)
// ============================================================================

/**
 * SoH estimation method
 */
export type SoHEstimationMethod =
  | 'capacity'      // Capacity-based
  | 'resistance'    // Internal resistance-based
  | 'impedance'     // EIS-based
  | 'model'         // Degradation model
  | 'combined';     // Multi-method

/**
 * SoH calculation request
 */
export interface SoHRequest {
  /** Estimation method */
  method: SoHEstimationMethod;

  /** Current maximum capacity (Ah) */
  currentCapacity?: number;

  /** Rated capacity when new (Ah) */
  ratedCapacity: number;

  /** Current internal resistance (Ohms) */
  currentResistance?: number;

  /** Initial internal resistance (Ohms) */
  initialResistance?: number;

  /** Number of charge/discharge cycles */
  cycles?: number;

  /** Age in days */
  age?: number;

  /** Average operating temperature (°C) */
  temperature?: number;

  /** Average depth of discharge (%) */
  averageDoD?: number;
}

/**
 * SoH calculation response
 */
export interface SoHResponse {
  /** State of Health (0-100%) */
  percentage: number;

  /** Estimated remaining useful life (cycles) */
  remainingLife: number;

  /** Degradation rate (%/cycle) */
  degradationRate: number;

  /** Capacity fade (%) */
  capacityFade: number;

  /** Resistance increase (%) */
  resistanceIncrease: number;

  /** Confidence level (0-1) */
  confidence: number;

  /** Method used */
  method: SoHEstimationMethod;

  /** Timestamp of calculation */
  timestamp: Date;
}

/**
 * Battery degradation model parameters
 */
export interface DegradationModel {
  /** Cycle-based capacity fade rate */
  cycleFadeRate: number;

  /** Calendar aging rate */
  calendarAgingRate: number;

  /** Temperature acceleration factor */
  temperatureFactor: number;

  /** Activation energy (J/mol) */
  activationEnergy: number;

  /** Cycling exponent */
  cycleExponent: number;
}

// ============================================================================
// Cell Balancing
// ============================================================================

/**
 * Cell balancing method
 */
export type BalancingMethod =
  | 'passive'    // Resistive discharge
  | 'active';    // Energy transfer (capacitive/inductive)

/**
 * Balancing request
 */
export interface BalancingRequest {
  /** Balancing method */
  method: BalancingMethod;

  /** Array of cell voltages (V) */
  cellVoltages: number[];

  /** Target maximum voltage difference (V) */
  targetDelta: number;

  /** Maximum balancing current (A) */
  maxCurrent?: number;

  /** Maximum balancing time (seconds) */
  maxTime?: number;

  /** Balancing resistor value (Ohms) - for passive */
  balancingResistor?: number;
}

/**
 * Balancing response
 */
export interface BalancingResponse {
  /** Indices of cells requiring balancing */
  cellsToBalance: number[];

  /** Estimated balancing time (seconds) */
  estimatedTime: number;

  /** Energy to be dissipated (J) */
  energyDissipated: number;

  /** Energy to be transferred (J) - for active balancing */
  energyTransferred?: number;

  /** Balancing efficiency (0-1) */
  efficiency?: number;

  /** Balancing method used */
  method: BalancingMethod;

  /** Current status */
  status: 'pending' | 'in-progress' | 'complete' | 'failed';

  /** Timestamp */
  timestamp: Date;
}

/**
 * Cell balancing status
 */
export interface CellBalancingStatus {
  /** Cell index */
  cellIndex: number;

  /** Current voltage (V) */
  voltage: number;

  /** Target voltage (V) */
  targetVoltage: number;

  /** Balancing active */
  isBalancing: boolean;

  /** Balancing current (A) */
  balancingCurrent?: number;

  /** Time remaining (seconds) */
  timeRemaining?: number;
}

// ============================================================================
// Thermal Management
// ============================================================================

/**
 * Cooling method
 */
export type CoolingMethod =
  | 'air'              // Air cooling (natural or forced)
  | 'liquid'           // Liquid cooling
  | 'pcm'              // Phase Change Material
  | 'thermoelectric'   // Peltier cooling
  | 'refrigerant';     // Active refrigeration

/**
 * Thermal management request
 */
export interface ThermalRequest {
  /** Temperature measurements (°C) from multiple sensors */
  temperatures: number[];

  /** Ambient temperature (°C) */
  ambientTemp: number;

  /** Pack current (A) */
  packCurrent: number;

  /** Pack voltage (V) */
  packVoltage?: number;

  /** Cooling method */
  coolingMethod: CoolingMethod;

  /** Heat transfer coefficient (W/m²·K) */
  heatTransferCoeff?: number;

  /** Pack surface area (m²) */
  surfaceArea?: number;
}

/**
 * Thermal management response
 */
export interface ThermalResponse {
  /** Maximum temperature (°C) */
  maxTemperature: number;

  /** Average temperature (°C) */
  avgTemperature: number;

  /** Minimum temperature (°C) */
  minTemperature: number;

  /** Hotspot location (sensor index) */
  hotspotLocation: number;

  /** Heat generation rate (W) */
  heatGeneration: number;

  /** Cooling required */
  coolingRequired: boolean;

  /** Required cooling power (W) */
  coolingPower: number;

  /** Time to temperature limit (seconds) */
  timeToLimit: number;

  /** Thermal status */
  status: 'ok' | 'warning' | 'critical' | 'emergency';

  /** Recommended action */
  recommendedAction?: string;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Thermal model parameters
 */
export interface ThermalModel {
  /** Thermal capacitance (J/K) */
  thermalCapacitance: number;

  /** Heat transfer coefficient (W/m²·K) */
  heatTransferCoeff: number;

  /** Surface area (m²) */
  surfaceArea: number;

  /** Internal thermal resistance (K/W) */
  thermalResistance: number;
}

// ============================================================================
// Safety and Protection
// ============================================================================

/**
 * Fault type
 */
export type FaultType =
  | 'overvoltage'
  | 'undervoltage'
  | 'overcurrent-charge'
  | 'overcurrent-discharge'
  | 'short-circuit'
  | 'overtemperature'
  | 'undertemperature'
  | 'cell-imbalance'
  | 'isolation-fault'
  | 'communication-error'
  | 'sensor-fault'
  | 'contactor-fault'
  | 'fuse-blown';

/**
 * Alert severity level
 */
export type AlertSeverity = 'info' | 'warning' | 'critical' | 'emergency';

/**
 * Fault/Alert message
 */
export interface Alert {
  /** Unique alert ID */
  alertId: string;

  /** Severity level */
  severity: AlertSeverity;

  /** Fault type */
  type: FaultType;

  /** Description */
  description: string;

  /** Affected cell ID (if applicable) */
  cellId?: string;

  /** Measured value */
  measuredValue?: number;

  /** Threshold that was exceeded */
  threshold?: number;

  /** Action taken by BMS */
  actionTaken?: string;

  /** Is fault latched (requires manual reset) */
  latched: boolean;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Protection thresholds
 */
export interface ProtectionThresholds {
  /** Overvoltage warning (V) */
  overvoltageWarning: number;

  /** Overvoltage fault (V) */
  overvoltageFault: number;

  /** Undervoltage warning (V) */
  undervoltageWarning: number;

  /** Undervoltage fault (V) */
  undervoltageFault: number;

  /** Overcurrent charge warning (A) */
  overcurrentChargeWarning: number;

  /** Overcurrent charge fault (A) */
  overcurrentChargeFault: number;

  /** Overcurrent discharge warning (A) */
  overcurrentDischargeWarning: number;

  /** Overcurrent discharge fault (A) */
  overcurrentDischargeFault: number;

  /** Short circuit current (A) */
  shortCircuitCurrent: number;

  /** Overtemperature warning (°C) */
  overtemperatureWarning: number;

  /** Overtemperature fault (°C) */
  overtemperatureFault: number;

  /** Undertemperature warning (°C) */
  undertemperatureWarning: number;

  /** Undertemperature fault (°C) */
  undertemperatureFault: number;

  /** Cell imbalance threshold (V) */
  cellImbalanceThreshold: number;

  /** Minimum isolation resistance (Ohms) */
  minIsolationResistance: number;
}

/**
 * Protection status
 */
export interface ProtectionStatus {
  /** Is system safe to operate */
  safe: boolean;

  /** Active faults */
  faults: Alert[];

  /** Active warnings */
  warnings: Alert[];

  /** Required actions */
  actionsRequired: string[];

  /** Maximum allowed power (W) */
  powerLimit: number;

  /** Maximum allowed charge current (A) */
  chargeCurrentLimit: number;

  /** Maximum allowed discharge current (A) */
  dischargeCurrentLimit: number;

  /** Contactors state */
  contactorsState: {
    main: boolean;
    precharge: boolean;
    charge: boolean;
  };
}

// ============================================================================
// Battery Pack Status
// ============================================================================

/**
 * Individual cell status
 */
export interface CellStatus {
  /** Cell identifier */
  cellId: string;

  /** Cell index in pack */
  cellIndex: number;

  /** Voltage (V) */
  voltage: number;

  /** Temperature (°C) */
  temperature?: number;

  /** State of Charge (%) */
  soc?: number;

  /** Internal resistance (Ohms) */
  resistance?: number;

  /** Is balancing active */
  balancingActive: boolean;

  /** Fault flags */
  faultFlags: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Complete battery pack status
 */
export interface BatteryPackStatus {
  /** Pack identifier */
  packId: string;

  /** Total pack voltage (V) */
  packVoltage: number;

  /** Pack current (A) - negative for discharge */
  packCurrent: number;

  /** Pack power (W) - negative for discharge */
  packPower: number;

  /** State of Charge (%) */
  soc: number;

  /** State of Health (%) */
  soh: number;

  /** Number of cells */
  cellCount: number;

  /** Minimum cell voltage (V) */
  minCellVoltage: number;

  /** Maximum cell voltage (V) */
  maxCellVoltage: number;

  /** Average cell voltage (V) */
  avgCellVoltage: number;

  /** Cell voltage delta (V) */
  cellVoltageDelta: number;

  /** Average temperature (°C) */
  avgTemperature: number;

  /** Maximum temperature (°C) */
  maxTemperature: number;

  /** Minimum temperature (°C) */
  minTemperature: number;

  /** Balancing status */
  balancingStatus: 'idle' | 'active' | 'complete' | 'disabled';

  /** Active faults */
  faults: Alert[];

  /** Active warnings */
  warnings: Alert[];

  /** Available power (W) */
  powerAvailable: number;

  /** Energy remaining (kWh) */
  energyRemaining: number;

  /** Estimated time to empty (seconds) - at current discharge rate */
  timeToEmpty?: number;

  /** Estimated time to full (seconds) - at current charge rate */
  timeToFull?: number;

  /** Total charge/discharge cycles */
  cycles: number;

  /** Charge throughput (Ah) */
  chargeThroughput?: number;

  /** Discharge throughput (Ah) */
  dischargeThroughput?: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Historical data point
 */
export interface HistoricalDataPoint {
  /** Timestamp */
  timestamp: Date;

  /** SoC (%) */
  soc: number;

  /** Voltage (V) */
  voltage: number;

  /** Current (A) */
  current: number;

  /** Power (W) */
  power: number;

  /** Temperature (°C) */
  temperature: number;

  /** Energy (kWh) - cumulative */
  energy?: number;
}

// ============================================================================
// BMS Operations
// ============================================================================

/**
 * BMS operating mode
 */
export type BMSMode =
  | 'idle'
  | 'charging'
  | 'discharging'
  | 'balancing'
  | 'diagnostics'
  | 'fault'
  | 'shutdown';

/**
 * Charge profile
 */
export interface ChargeProfile {
  /** Profile name */
  name: string;

  /** Constant Current phase current (A) */
  ccCurrent: number;

  /** Constant Voltage phase voltage (V) */
  cvVoltage: number;

  /** Termination current (A) */
  terminationCurrent: number;

  /** Maximum charge time (seconds) */
  maxChargeTime: number;

  /** Temperature limits */
  temperatureLimits: {
    min: number; // °C
    max: number; // °C
  };
}

/**
 * Discharge profile
 */
export interface DischargeProfile {
  /** Profile name */
  name: string;

  /** Maximum discharge current (A) */
  maxCurrent: number;

  /** Cutoff voltage (V) */
  cutoffVoltage: number;

  /** Temperature limits */
  temperatureLimits: {
    min: number; // °C
    max: number; // °C
  };
}

/**
 * BMS calibration data
 */
export interface CalibrationData {
  /** Voltage sensor offsets (V) */
  voltageOffsets: number[];

  /** Current sensor offset (A) */
  currentOffset: number;

  /** Current sensor gain */
  currentGain: number;

  /** Temperature sensor offsets (°C) */
  temperatureOffsets: number[];

  /** Last calibration date */
  calibrationDate: Date;

  /** Calibration valid until */
  validUntil?: Date;
}

// ============================================================================
// Advanced Features
// ============================================================================

/**
 * State of Function (SoF) - available power
 */
export interface StateOfFunction {
  /** Maximum charge power (W) */
  maxChargePower: number;

  /** Maximum discharge power (W) */
  maxDischargePower: number;

  /** Maximum continuous charge power (W) */
  maxContinuousChargePower: number;

  /** Maximum continuous discharge power (W) */
  maxContinuousDischargePower: number;

  /** Peak power duration (seconds) */
  peakPowerDuration: number;

  /** Limiting factor */
  limitingFactor: 'voltage' | 'current' | 'temperature' | 'soc' | 'soh';

  /** Derating factors */
  deratingFactors: {
    thermal: number;     // 0-1
    voltage: number;     // 0-1
    soc: number;         // 0-1
    soh: number;         // 0-1
  };
}

/**
 * Battery impedance data (from EIS)
 */
export interface ImpedanceData {
  /** Frequency (Hz) */
  frequency: number;

  /** Real part (Ohms) */
  real: number;

  /** Imaginary part (Ohms) */
  imaginary: number;

  /** Magnitude (Ohms) */
  magnitude: number;

  /** Phase angle (degrees) */
  phase: number;
}

/**
 * Predictive maintenance data
 */
export interface PredictiveMaintenanceData {
  /** Estimated time to next service (hours) */
  timeToService: number;

  /** Estimated time to replacement (hours) */
  timeToReplacement: number;

  /** Failure probability (0-1) */
  failureProbability: number;

  /** Recommended actions */
  recommendations: string[];

  /** Health score (0-100) */
  healthScore: number;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Battery chemistry parameters
 */
export const CHEMISTRY_PARAMETERS: Record<BatteryCellChemistry, {
  nominalVoltage: number;
  maxVoltage: number;
  minVoltage: number;
  energyDensity: number; // Wh/kg
}> = {
  'lithium-ion-nmc': {
    nominalVoltage: 3.7,
    maxVoltage: 4.2,
    minVoltage: 2.5,
    energyDensity: 200,
  },
  'lithium-ion-lfp': {
    nominalVoltage: 3.2,
    maxVoltage: 3.65,
    minVoltage: 2.0,
    energyDensity: 160,
  },
  'lithium-ion-nca': {
    nominalVoltage: 3.6,
    maxVoltage: 4.2,
    minVoltage: 2.7,
    energyDensity: 250,
  },
  'lithium-polymer': {
    nominalVoltage: 3.7,
    maxVoltage: 4.2,
    minVoltage: 3.0,
    energyDensity: 180,
  },
  'lithium-titanate': {
    nominalVoltage: 2.4,
    maxVoltage: 2.8,
    minVoltage: 1.5,
    energyDensity: 90,
  },
  'nimh': {
    nominalVoltage: 1.2,
    maxVoltage: 1.45,
    minVoltage: 0.9,
    energyDensity: 80,
  },
  'lead-acid': {
    nominalVoltage: 2.0,
    maxVoltage: 2.4,
    minVoltage: 1.75,
    energyDensity: 40,
  },
  'sodium-ion': {
    nominalVoltage: 3.1,
    maxVoltage: 3.8,
    minVoltage: 2.0,
    energyDensity: 150,
  },
  'solid-state': {
    nominalVoltage: 3.8,
    maxVoltage: 4.5,
    minVoltage: 2.5,
    energyDensity: 400,
  },
} as const;

/**
 * BMS constants
 */
export const BMS_CONSTANTS = {
  /** Minimum SoC for operation (%) */
  MIN_SOC: 10,

  /** Maximum SoC for operation (%) */
  MAX_SOC: 90,

  /** SoC for storage (%) */
  STORAGE_SOC: 50,

  /** Default coulombic efficiency */
  DEFAULT_EFFICIENCY: 0.98,

  /** Maximum cell imbalance (V) */
  MAX_CELL_IMBALANCE: 0.1,

  /** Balancing threshold (V) */
  BALANCING_THRESHOLD: 0.01,

  /** Maximum cell temperature rise rate (°C/s) */
  MAX_TEMP_RATE: 5,

  /** Minimum data logging interval (ms) */
  MIN_LOG_INTERVAL: 100,

  /** CAN bus update rate (Hz) */
  CAN_UPDATE_RATE: 10,

  /** Fault response time (ms) */
  FAULT_RESPONSE_TIME: 100,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * BMS error codes
 */
export enum BMSErrorCode {
  OVERVOLTAGE = 'BMS001',
  UNDERVOLTAGE = 'BMS002',
  OVERCURRENT_CHARGE = 'BMS003',
  OVERCURRENT_DISCHARGE = 'BMS004',
  SHORT_CIRCUIT = 'BMS005',
  OVERTEMPERATURE = 'BMS006',
  UNDERTEMPERATURE = 'BMS007',
  CELL_IMBALANCE = 'BMS008',
  ISOLATION_FAULT = 'BMS009',
  COMMUNICATION_ERROR = 'BMS010',
  SENSOR_FAULT = 'BMS011',
  CALIBRATION_ERROR = 'BMS012',
  BALANCING_FAILURE = 'BMS013',
  SOC_ESTIMATION_ERROR = 'BMS014',
  SOH_ESTIMATION_ERROR = 'BMS015',
}

/**
 * BMS error class
 */
export class BMSError extends Error {
  constructor(
    public code: BMSErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BMSError';
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

// ============================================================================
// Export All
// ============================================================================

export type {
  // Battery configuration
  BatteryCell,
  BatteryPackConfig,

  // SoC
  SoCRequest,
  SoCResponse,
  SoCEstimationState,

  // SoH
  SoHRequest,
  SoHResponse,
  DegradationModel,

  // Balancing
  BalancingRequest,
  BalancingResponse,
  CellBalancingStatus,

  // Thermal
  ThermalRequest,
  ThermalResponse,
  ThermalModel,

  // Safety
  Alert,
  ProtectionThresholds,
  ProtectionStatus,

  // Status
  CellStatus,
  BatteryPackStatus,
  HistoricalDataPoint,

  // Operations
  ChargeProfile,
  DischargeProfile,
  CalibrationData,

  // Advanced
  StateOfFunction,
  ImpedanceData,
  PredictiveMaintenanceData,
};

export {
  CHEMISTRY_PARAMETERS,
  BMS_CONSTANTS,
  BMSErrorCode,
  BMSError,
};
