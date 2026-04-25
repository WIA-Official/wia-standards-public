/**
 * WIA-AUTO-007: Hydrogen Vehicle - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Vector for position, velocity, etc.
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Temperature with unit specification
 */
export interface Temperature {
  value: number;
  unit: 'C' | 'K' | 'F';
}

/**
 * Pressure with unit specification
 */
export interface Pressure {
  value: number;
  unit: 'bar' | 'Pa' | 'psi' | 'MPa';
}

// ============================================================================
// Fuel Cell Types
// ============================================================================

/**
 * Fuel cell technology types
 */
export type FuelCellType = 'PEMFC' | 'SOFC' | 'AFC' | 'PAFC' | 'MCFC';

/**
 * Fuel cell operating status
 */
export type FuelCellStatus =
  | 'off'
  | 'startup'
  | 'idle'
  | 'active'
  | 'peak'
  | 'shutdown'
  | 'fault';

/**
 * Fuel cell stack configuration
 */
export interface FuelCellStack {
  /** Stack identifier */
  id: string;

  /** Fuel cell technology type */
  type: FuelCellType;

  /** Number of cells in series */
  numberOfCells: number;

  /** Active cell area in cm² */
  cellArea: number;

  /** Rated power output in kW */
  ratedPower: number;

  /** Operating voltage in V */
  voltage: number;

  /** Operating current in A */
  current: number;

  /** Stack temperature in °C */
  temperature: number;

  /** Current power output in kW */
  powerOutput: number;

  /** Current efficiency (0-1) */
  efficiency: number;

  /** Stack status */
  status: FuelCellStatus;

  /** Operating hours */
  operatingHours: number;

  /** Power density in kW/L */
  powerDensity?: number;
}

/**
 * Fuel cell performance parameters
 */
export interface FuelCellParams {
  /** Power output in kW */
  powerOutput: number;

  /** Hydrogen flow rate in kg/h */
  hydrogenFlowRate: number;

  /** Stack voltage in V */
  stackVoltage: number;

  /** Stack current in A */
  stackCurrent: number;

  /** Operating temperature in °C */
  temperature?: number;

  /** Operating pressure in bar */
  pressure?: number;
}

/**
 * Fuel cell efficiency calculation result
 */
export interface EfficiencyResult {
  /** Electrical efficiency (0-1) */
  efficiency: number;

  /** Power density in kW/L */
  powerDensity: number;

  /** Current density in A/cm² */
  currentDensity: number;

  /** Voltage efficiency (0-1) */
  voltageEfficiency: number;

  /** Heat generation in kW */
  heatGeneration?: number;

  /** Feasibility assessment */
  feasibility: 'optimal' | 'acceptable' | 'suboptimal' | 'critical';
}

/**
 * Fuel cell performance map point
 */
export interface PerformancePoint {
  /** Current density in A/cm² */
  currentDensity: number;

  /** Cell voltage in V */
  voltage: number;

  /** Power density in kW/cm² */
  powerDensity: number;

  /** Efficiency at this operating point */
  efficiency: number;
}

/**
 * Complete fuel cell performance map
 */
export interface PerformanceMap {
  /** Map identifier */
  mapId: string;

  /** Rated power in kW */
  ratedPower: number;

  /** Operating points */
  operatingPoints: PerformancePoint[];

  /** Optimal operating range */
  optimalRange?: {
    currentDensityMin: number;
    currentDensityMax: number;
  };
}

// ============================================================================
// Hydrogen Storage Types
// ============================================================================

/**
 * Hydrogen storage tank types
 */
export type TankType = 'Type I' | 'Type II' | 'Type III' | 'Type IV';

/**
 * Hydrogen storage standards
 */
export type StorageStandard = 'H35' | 'H70';

/**
 * Tank status
 */
export type TankStatus = 'normal' | 'low' | 'critical' | 'refueling' | 'fault';

/**
 * Hydrogen storage system
 */
export interface HydrogenStorage {
  /** Tank system identifier */
  id: string;

  /** Tank type */
  tankType: TankType;

  /** Storage standard */
  standard: StorageStandard;

  /** Number of tanks */
  numberOfTanks: number;

  /** Total volume in liters */
  totalVolume: number;

  /** Current pressure in bar */
  pressure: number;

  /** Current temperature in °C */
  temperature: number;

  /** Hydrogen mass remaining in kg */
  massRemaining: number;

  /** Total capacity in kg */
  capacity: number;

  /** State of charge (0-1) */
  soc: number;

  /** Tank status */
  status: TankStatus;

  /** Working pressure in bar */
  workingPressure: number;

  /** Maximum pressure in bar */
  maxPressure: number;
}

/**
 * Hydrogen storage capacity calculation
 */
export interface StorageCapacity {
  /** Hydrogen mass in kg */
  mass: number;

  /** Gravimetric density in % */
  gravimetricDensity: number;

  /** Volumetric density in kg/m³ */
  volumetricDensity: number;

  /** Energy stored in MJ */
  energyStored: number;

  /** Energy stored in kWh */
  energyStoredKwh: number;

  /** Tank system weight in kg */
  tankWeight?: number;
}

/**
 * Tank pressure validation
 */
export interface TankValidation {
  /** Current pressure in bar */
  pressure: number;

  /** Current temperature in °C */
  temperature: number;

  /** Tank type */
  tankType: TankType;

  /** Storage standard */
  standard: StorageStandard;

  /** Check for refueling? */
  isRefueling?: boolean;
}

/**
 * Tank validation result
 */
export interface TankValidationResult {
  /** Is the tank state valid? */
  isValid: boolean;

  /** Warning messages */
  warnings: string[];

  /** Error messages */
  errors: string[];

  /** Safety margin in % */
  safetyMargin: number;

  /** Maximum allowed pressure in bar */
  maxAllowedPressure: number;

  /** Temperature rise limit check */
  temperatureOk: boolean;
}

// ============================================================================
// Power Electronics Types
// ============================================================================

/**
 * DC/DC converter status
 */
export type ConverterStatus = 'off' | 'active' | 'bypass' | 'fault';

/**
 * DC/DC converter configuration
 */
export interface DCConverter {
  /** Converter identifier */
  id: string;

  /** Input voltage in V */
  inputVoltage: number;

  /** Output voltage in V */
  outputVoltage: number;

  /** Current in A */
  current: number;

  /** Power output in kW */
  powerOutput: number;

  /** Efficiency (0-1) */
  efficiency: number;

  /** Duty cycle (0-1) */
  dutyCycle: number;

  /** Status */
  status: ConverterStatus;

  /** Operating temperature in °C */
  temperature: number;
}

/**
 * Inverter (motor controller) status
 */
export type InverterStatus = 'off' | 'active' | 'regen' | 'fault';

/**
 * Three-phase inverter configuration
 */
export interface Inverter {
  /** Inverter identifier */
  id: string;

  /** DC bus voltage in V */
  dcBusVoltage: number;

  /** AC line voltage in V */
  acVoltage: number;

  /** Line current in A */
  current: number;

  /** Power output in kW */
  powerOutput: number;

  /** Efficiency (0-1) */
  efficiency: number;

  /** Power factor */
  powerFactor: number;

  /** Frequency in Hz */
  frequency: number;

  /** Status */
  status: InverterStatus;

  /** Operating temperature in °C */
  temperature: number;
}

/**
 * Power electronics system
 */
export interface PowerElectronics {
  /** DC/DC converter */
  converter: DCConverter;

  /** Inverter */
  inverter: Inverter;

  /** DC bus voltage in V */
  dcBusVoltage: number;

  /** Total system efficiency (0-1) */
  systemEfficiency: number;
}

// ============================================================================
// Electric Motor Types
// ============================================================================

/**
 * Motor types
 */
export type MotorType = 'PMSM' | 'IM' | 'SRM' | 'SynRM';

/**
 * Motor status
 */
export type MotorStatus =
  | 'off'
  | 'forward'
  | 'reverse'
  | 'regen'
  | 'fault';

/**
 * Electric motor configuration
 */
export interface ElectricMotor {
  /** Motor identifier */
  id: string;

  /** Motor type */
  type: MotorType;

  /** Rated power in kW */
  ratedPower: number;

  /** Maximum torque in N·m */
  maxTorque: number;

  /** Current power output in kW */
  powerOutput: number;

  /** Current torque in N·m */
  torque: number;

  /** Rotational speed in RPM */
  rpm: number;

  /** Motor efficiency (0-1) */
  efficiency: number;

  /** Motor temperature in °C */
  temperature: number;

  /** Status */
  status: MotorStatus;

  /** Power density in kW/kg */
  powerDensity?: number;
}

// ============================================================================
// Battery Buffer Types
// ============================================================================

/**
 * Battery operating mode
 */
export type BatteryMode =
  | 'idle'
  | 'assist'
  | 'charge'
  | 'discharge'
  | 'regen';

/**
 * Battery buffer system (for peak power and regen)
 */
export interface BatteryBuffer {
  /** Battery identifier */
  id: string;

  /** Capacity in kWh */
  capacity: number;

  /** State of charge (0-1) */
  soc: number;

  /** Voltage in V */
  voltage: number;

  /** Current in A (+ discharge, - charge) */
  current: number;

  /** Power in kW (+ discharge, - charge) */
  power: number;

  /** Temperature in °C */
  temperature: number;

  /** Operating mode */
  mode: BatteryMode;

  /** State of health (0-1) */
  soh?: number;
}

// ============================================================================
// Refueling Types
// ============================================================================

/**
 * Refueling protocol
 */
export type RefuelingProtocol = 'SAE J2601' | 'ISO 19880-1' | 'Custom';

/**
 * Nozzle type
 */
export type NozzleType = 'TIR-21' | 'TIR-22';

/**
 * Refueling parameters
 */
export interface RefuelingParams {
  /** Target pressure in bar */
  targetPressure: number;

  /** Ambient temperature in °C */
  ambientTemp: number;

  /** Current tank pressure in bar */
  currentPressure: number;

  /** Current tank temperature in °C */
  currentTemp: number;

  /** Tank volume in liters */
  tankVolume: number;

  /** Tank type */
  tankType?: TankType;

  /** Storage standard */
  standard?: StorageStandard;
}

/**
 * Refueling plan
 */
export interface RefuelingPlan {
  /** Pre-cooling temperature in °C */
  preCoolTemp: number;

  /** Hydrogen flow rate in kg/min */
  flowRate: number;

  /** Estimated refueling time in minutes */
  estimatedTime: number;

  /** Final tank temperature in °C */
  finalTemp: number;

  /** Pressure ramp rate in bar/s */
  pressureRampRate: number;

  /** Total hydrogen mass to dispense in kg */
  hydrogenMass: number;

  /** Safety checks */
  safetyChecks: SafetyCheck[];

  /** Is refueling safe? */
  isSafe: boolean;
}

/**
 * Refueling session data
 */
export interface RefuelingSession {
  /** Session identifier */
  sessionId: string;

  /** Start timestamp */
  timestampStart: Date;

  /** End timestamp */
  timestampEnd?: Date;

  /** Vehicle identifier */
  vehicleId: string;

  /** Dispenser identifier */
  dispenserId: string;

  /** Refueling protocol used */
  protocol: RefuelingProtocol;

  /** Initial tank state */
  initialState: {
    pressure: number;
    temperature: number;
    mass: number;
  };

  /** Final tank state */
  finalState?: {
    pressure: number;
    temperature: number;
    mass: number;
  };

  /** Refueling parameters */
  refuelingParams?: {
    preCoolTemp: number;
    peakFlowRate: number;
    averageFlowRate: number;
    totalMass: number;
    duration: number;
    pressureRampRate: number;
  };

  /** Station information */
  stationInfo?: {
    location: string;
    supplyPressure: number;
    ambientTemp: number;
  };
}

// ============================================================================
// Vehicle Types
// ============================================================================

/**
 * Vehicle operating mode
 */
export type VehicleMode =
  | 'off'
  | 'standby'
  | 'eco'
  | 'normal'
  | 'sport'
  | 'power';

/**
 * Complete hydrogen vehicle configuration
 */
export interface HydrogenVehicle {
  /** Vehicle identifier */
  vehicleId: string;

  /** Vehicle make and model */
  model: string;

  /** Fuel cell stack */
  fuelCell: FuelCellStack;

  /** Hydrogen storage system */
  hydrogenStorage: HydrogenStorage;

  /** Power electronics */
  powerElectronics: PowerElectronics;

  /** Electric motor */
  motor: ElectricMotor;

  /** Battery buffer */
  batteryBuffer?: BatteryBuffer;

  /** Current vehicle speed in km/h */
  speed: number;

  /** Odometer reading in km */
  odometer: number;

  /** Remaining range in km */
  rangeRemaining: number;

  /** Energy consumption in MJ/km */
  energyConsumption: number;

  /** Operating mode */
  mode: VehicleMode;

  /** Last update timestamp */
  timestamp: Date;
}

/**
 * Vehicle status summary
 */
export interface VehicleStatus {
  /** Vehicle ID */
  vehicleId: string;

  /** Timestamp */
  timestamp: Date;

  /** Is vehicle ready to drive? */
  isReady: boolean;

  /** Fuel cell status */
  fuelCell: {
    power: number;
    efficiency: number;
    status: FuelCellStatus;
  };

  /** Hydrogen status */
  hydrogen: {
    massRemaining: number;
    soc: number;
    pressure: number;
  };

  /** Range and consumption */
  range: {
    remaining: number;
    consumption: number;
  };

  /** Warnings */
  warnings: string[];

  /** Errors */
  errors: string[];
}

// ============================================================================
// Performance Calculations
// ============================================================================

/**
 * Range calculation parameters
 */
export interface RangeParams {
  /** Hydrogen capacity in kg */
  hydrogenCapacity: number;

  /** Fuel cell efficiency (0-1) */
  fuelCellEfficiency: number;

  /** Overall system efficiency (0-1) */
  systemEfficiency: number;

  /** Energy consumption in MJ/km */
  energyConsumption: number;

  /** Include battery buffer assist? */
  includeBatteryAssist?: boolean;

  /** Battery assist efficiency (0-1) */
  batteryEfficiency?: number;
}

/**
 * Range calculation result
 */
export interface RangeResult {
  /** Estimated range in km */
  range: number;

  /** Total energy available in MJ */
  energyAvailable: number;

  /** Usable energy in MJ */
  energyUsable: number;

  /** Reserve range buffer in km */
  rangeBuffer: number;

  /** Energy breakdown */
  breakdown: {
    hydrogenEnergy: number;
    fuelCellEnergy: number;
    usableEnergy: number;
    batteryEnergy?: number;
  };
}

/**
 * Efficiency calculation parameters
 */
export interface EfficiencyParams {
  /** Fuel cell efficiency (0-1) */
  fuelCellEfficiency: number;

  /** DC/DC converter efficiency (0-1) */
  converterEfficiency: number;

  /** Inverter efficiency (0-1) */
  inverterEfficiency: number;

  /** Motor efficiency (0-1) */
  motorEfficiency: number;

  /** Transmission efficiency (0-1) */
  transmissionEfficiency?: number;

  /** Auxiliary systems efficiency (0-1) */
  auxiliaryEfficiency?: number;
}

/**
 * System efficiency result
 */
export interface SystemEfficiency {
  /** Overall powertrain efficiency (0-1) */
  powertrainEfficiency: number;

  /** Tank-to-wheel efficiency (0-1) */
  tankToWheelEfficiency: number;

  /** Efficiency breakdown by component */
  breakdown: {
    fuelCell: number;
    converter: number;
    inverter: number;
    motor: number;
    transmission?: number;
    auxiliary?: number;
  };

  /** Total losses in % */
  totalLosses: number;
}

// ============================================================================
// Safety Types
// ============================================================================

/**
 * Safety check result
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Check status */
  status: 'pass' | 'fail' | 'warning' | 'info';

  /** Description */
  description: string;

  /** Measured value */
  value?: number;

  /** Expected/threshold value */
  threshold?: number;

  /** Corrective action if failed */
  correctiveAction?: string;

  /** Severity level */
  severity?: 'low' | 'medium' | 'high' | 'critical';
}

/**
 * Hydrogen leak detection
 */
export interface LeakDetection {
  /** Sensor identifier */
  sensorId: string;

  /** Hydrogen concentration in % */
  concentration: number;

  /** Location */
  location: string;

  /** Is leak detected? */
  leakDetected: boolean;

  /** Alarm threshold in % (default 0.04% = 1% LEL) */
  alarmThreshold: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Emergency shutdown trigger
 */
export type EmergencyTrigger =
  | 'collision'
  | 'rollover'
  | 'fire'
  | 'leak'
  | 'manual'
  | 'system-fault';

/**
 * Emergency shutdown event
 */
export interface EmergencyShutdown {
  /** Event identifier */
  eventId: string;

  /** Trigger reason */
  trigger: EmergencyTrigger;

  /** Timestamp */
  timestamp: Date;

  /** Hydrogen valves closed? */
  valvesClosed: boolean;

  /** Time to shutdown in milliseconds */
  shutdownTime: number;

  /** Vehicle state at shutdown */
  vehicleState: {
    speed: number;
    hydrogenPressure: number;
    fuelCellPower: number;
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and automotive constants
 */
export const HYDROGEN_CONSTANTS = {
  /** Lower Heating Value in MJ/kg */
  LHV_H2: 120,

  /** Higher Heating Value in MJ/kg */
  HHV_H2: 142,

  /** H2 density at STP in kg/m³ */
  DENSITY_STP: 0.0899,

  /** H2 specific heat in kJ/kg·K */
  SPECIFIC_HEAT: 14.3,

  /** Faraday constant in C/mol */
  FARADAY_CONSTANT: 96485,

  /** Universal gas constant in J/mol·K */
  GAS_CONSTANT: 8.314,

  /** H2 molecular weight in g/mol */
  MOLECULAR_WEIGHT: 2.016,

  /** Standard cell potential in V */
  STANDARD_CELL_POTENTIAL: 1.23,

  /** Thermoneutral voltage in V */
  THERMONEUTRAL_VOLTAGE: 1.48,

  /** Lower Explosive Limit in % */
  LEL_H2: 4.0,

  /** H35 working pressure in bar */
  H35_PRESSURE: 350,

  /** H70 working pressure in bar */
  H70_PRESSURE: 700,

  /** Target PEMFC efficiency */
  TARGET_PEMFC_EFFICIENCY: 0.60,

  /** Target system efficiency */
  TARGET_SYSTEM_EFFICIENCY: 0.50,

  /** Minimum safety margin for tanks */
  MIN_SAFETY_MARGIN: 0.10,
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

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-007 error codes
 */
export enum HydrogenVehicleErrorCode {
  INVALID_PARAMETERS = 'H001',
  FUEL_CELL_FAULT = 'H002',
  HYDROGEN_LEAK = 'H003',
  TANK_OVERPRESSURE = 'H004',
  TEMPERATURE_CRITICAL = 'H005',
  MOTOR_FAULT = 'H006',
  BATTERY_FAULT = 'H007',
  REFUELING_ERROR = 'H008',
  SYSTEM_FAULT = 'H009',
  SAFETY_VIOLATION = 'H010',
}

/**
 * Hydrogen vehicle error
 */
export class HydrogenVehicleError extends Error {
  constructor(
    public code: HydrogenVehicleErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'HydrogenVehicleError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  Vector3,
  Temperature,
  Pressure,

  // Fuel Cell
  FuelCellType,
  FuelCellStatus,
  FuelCellStack,
  FuelCellParams,
  EfficiencyResult,
  PerformancePoint,
  PerformanceMap,

  // Storage
  TankType,
  StorageStandard,
  TankStatus,
  HydrogenStorage,
  StorageCapacity,
  TankValidation,
  TankValidationResult,

  // Power Electronics
  ConverterStatus,
  DCConverter,
  InverterStatus,
  Inverter,
  PowerElectronics,

  // Motor
  MotorType,
  MotorStatus,
  ElectricMotor,

  // Battery
  BatteryMode,
  BatteryBuffer,

  // Refueling
  RefuelingProtocol,
  NozzleType,
  RefuelingParams,
  RefuelingPlan,
  RefuelingSession,

  // Vehicle
  VehicleMode,
  HydrogenVehicle,
  VehicleStatus,

  // Performance
  RangeParams,
  RangeResult,
  EfficiencyParams,
  SystemEfficiency,

  // Safety
  SafetyCheck,
  LeakDetection,
  EmergencyTrigger,
  EmergencyShutdown,
};

export { HYDROGEN_CONSTANTS, HydrogenVehicleErrorCode, HydrogenVehicleError };
