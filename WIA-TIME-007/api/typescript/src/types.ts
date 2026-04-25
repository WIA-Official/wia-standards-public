/**
 * WIA-TIME-007: Time Energy Source - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Energy Types
// ============================================================================

/**
 * Energy source types for time travel
 */
export type EnergySourceType =
  | 'fusion'
  | 'antimatter'
  | 'exotic-matter'
  | 'zero-point'
  | 'casimir'
  | 'vacuum-polarization'
  | 'black-hole'
  | 'temporal-flux';

/**
 * Time travel methods requiring different energy profiles
 */
export type TimeTravelMethod = 'field' | 'wormhole' | 'ctc' | 'alcubierre' | 'natural';

/**
 * Energy source configuration
 */
export interface EnergySource {
  /** Source type */
  type: EnergySourceType;

  /** Output power in watts */
  power: number;

  /** Total energy capacity in joules */
  capacity: number;

  /** Current energy level in joules */
  currentEnergy: number;

  /** Efficiency factor (0-1) */
  efficiency: number;

  /** Source status */
  status: 'offline' | 'standby' | 'active' | 'charging' | 'depleted' | 'fault';

  /** Last update timestamp */
  updated: Date;
}

// ============================================================================
// Power Requirements
// ============================================================================

/**
 * Parameters for calculating power requirements
 */
export interface PowerRequirementParams {
  /** Mass to be displaced in kg */
  mass: number;

  /** Temporal displacement in seconds (negative for past) */
  displacement: number;

  /** Travel method */
  method: TimeTravelMethod;

  /** Safety margin (0-1, typically 0.2 for 20%) */
  safetyMargin?: number;

  /** Include exotic matter generation cost */
  includeExoticMatter?: boolean;

  /** Include field generation cost */
  includeFieldEnergy?: boolean;
}

/**
 * Power requirement calculation result
 */
export interface PowerRequirements {
  /** Total energy required in joules */
  totalEnergy: number;

  /** Total power in watts */
  totalPower: number;

  /** Duration of energy application in seconds */
  duration: number;

  /** Exotic matter required in kg */
  exoticMatterRequired: number;

  /** Feasibility assessment */
  feasibility: 'possible' | 'difficult' | 'theoretical' | 'impossible';

  /** Energy breakdown */
  breakdown: {
    /** Rest mass energy (mc²) */
    restMass: number;

    /** Temporal displacement energy */
    temporal: number;

    /** Exotic matter generation energy */
    exotic: number;

    /** Field generation energy */
    field: number;

    /** Safety margin energy */
    safety: number;
  };

  /** Human-readable energy format */
  energyFormatted: string;

  /** Comparisons to known energy sources */
  comparisons: EnergyComparison[];
}

/**
 * Energy comparison to known sources
 */
export interface EnergyComparison {
  /** Source name */
  source: string;

  /** Energy ratio (required / source) */
  ratio: number;

  /** Description */
  description: string;
}

// ============================================================================
// Exotic Matter
// ============================================================================

/**
 * Exotic matter configuration
 */
export interface ExoticMatterConfig {
  /** Target mass in kg */
  mass: number;

  /** Energy density in kg/m³ (must be negative) */
  density: number;

  /** Stabilization field strength in N/kg */
  stabilizationField: number;

  /** Containment type */
  containmentType: 'magnetic' | 'gravitational' | 'quantum' | 'hybrid';

  /** Production method */
  productionMethod?: 'casimir-cavity' | 'vacuum-manipulation' | 'quantum-squeezing';
}

/**
 * Exotic matter generation result
 */
export interface ExoticMatterResult {
  /** Unique identifier */
  id: string;

  /** Generated mass in kg */
  mass: number;

  /** Volume occupied in m³ */
  volume: number;

  /** Actual energy density in kg/m³ */
  density: number;

  /** Stability factor (0-1) */
  stability: number;

  /** Expected lifetime in seconds */
  lifetimeExpected: number;

  /** Energy cost to generate in joules */
  energyCost: number;

  /** Production rate in kg/s */
  productionRate: number;

  /** Current status */
  status: 'generating' | 'stable' | 'decaying' | 'failed';

  /** Creation timestamp */
  created: Date;

  /** Containment field strength */
  containmentField: {
    strength: number;
    type: string;
    stability: number;
  };
}

// ============================================================================
// Zero-Point Energy
// ============================================================================

/**
 * Zero-point energy extraction parameters
 */
export interface ZeroPointEnergyParams {
  /** Extraction volume in m³ */
  volume: number;

  /** Resonance frequency in Hz */
  frequency: number;

  /** Extraction efficiency (0-1) */
  efficiency: number;

  /** Extraction duration in seconds */
  duration: number;

  /** Cavity Q-factor */
  qFactor?: number;
}

/**
 * Zero-point energy extraction result
 */
export interface ZeroPointEnergyResult {
  /** Total energy extracted in joules */
  total: number;

  /** Average power in watts */
  power: number;

  /** Extraction rate in watts */
  rate: number;

  /** Energy density in J/m³ */
  energyDensity: number;

  /** Quantum efficiency achieved */
  quantumEfficiency: number;

  /** Vacuum quality factor */
  vacuumQuality: number;

  /** Extraction status */
  status: 'extracting' | 'complete' | 'failed';

  /** Warnings */
  warnings: string[];
}

// ============================================================================
// Casimir Effect
// ============================================================================

/**
 * Casimir effect configuration
 */
export interface CasimirConfig {
  /** Plate area in m² */
  plateArea: number;

  /** Plate separation in meters */
  separation: number;

  /** Plate material */
  material: 'gold' | 'silver' | 'aluminum' | 'graphene' | 'superconductor';

  /** Operating temperature in Kelvin */
  temperature: number;

  /** Vacuum pressure in Torr */
  vacuumPressure: number;
}

/**
 * Casimir energy result
 */
export interface CasimirResult {
  /** Casimir energy in joules */
  energy: number;

  /** Casimir force in newtons */
  force: number;

  /** Negative energy density in J/m³ */
  energyDensity: number;

  /** Work extractable in joules */
  workExtractable: number;

  /** Practical power output in watts */
  power: number;

  /** Configuration status */
  status: 'operational' | 'calibrating' | 'misaligned' | 'fault';
}

// ============================================================================
// Temporal Flux Capacitor
// ============================================================================

/**
 * Flux capacitor configuration
 */
export interface FluxCapacitorConfig {
  /** Capacity in joules */
  capacity: number;

  /** Charge rate in watts */
  chargeRate: number;

  /** Temporal voltage in temporal volts */
  voltage?: number;

  /** Efficiency (0-1) */
  efficiency: number;

  /** Capacitor class */
  class?: 'I' | 'II' | 'III';

  /** Dielectric material */
  dielectric?: 'barium-titanate' | 'strontium-titanate' | 'metamaterial';
}

/**
 * Flux capacitor state
 */
export interface FluxCapacitor {
  /** Unique identifier */
  id: string;

  /** Configuration */
  config: FluxCapacitorConfig;

  /** Current charge in joules */
  currentCharge: number;

  /** Charge percentage (0-100) */
  chargePercent: number;

  /** Current voltage in temporal volts */
  voltage: number;

  /** Operating temperature in Kelvin */
  temperature: number;

  /** Capacitor status */
  status: 'offline' | 'charging' | 'ready' | 'discharging' | 'fault';

  /** Time to full charge in seconds */
  timeToFull: number;

  /** Time to complete discharge in seconds */
  timeToEmpty: number;

  /** Charge cycles completed */
  cycles: number;

  /** Health indicator (0-1) */
  health: number;

  /** Created timestamp */
  created: Date;

  /** Last updated */
  updated: Date;
}

/**
 * Flux capacitor bank (multiple capacitors)
 */
export interface FluxCapacitorBank {
  /** Bank identifier */
  id: string;

  /** Capacitors in bank */
  capacitors: FluxCapacitor[];

  /** Total capacity in joules */
  totalCapacity: number;

  /** Total current charge in joules */
  totalCharge: number;

  /** Average charge percent */
  averageCharge: number;

  /** Bank configuration */
  configuration: 'series' | 'parallel' | 'hybrid';

  /** Bank status */
  status: 'operational' | 'degraded' | 'fault';
}

// ============================================================================
// Energy Storage
// ============================================================================

/**
 * Energy storage system
 */
export interface EnergyStorage {
  /** Storage identifier */
  id: string;

  /** Storage type */
  type: 'flux-capacitor' | 'antimatter-pod' | 'superconducting-coil' | 'gravitational';

  /** Maximum capacity in joules */
  capacity: number;

  /** Current stored energy in joules */
  stored: number;

  /** Maximum charge rate in watts */
  maxChargeRate: number;

  /** Maximum discharge rate in watts */
  maxDischargeRate: number;

  /** Storage efficiency (0-1) */
  efficiency: number;

  /** Self-discharge rate in joules/second */
  selfDischargeRate: number;

  /** Storage status */
  status: 'empty' | 'charging' | 'full' | 'discharging' | 'standby' | 'fault';

  /** Safety status */
  safetyStatus: 'safe' | 'warning' | 'critical';
}

// ============================================================================
// Energy Management
// ============================================================================

/**
 * Energy manifest - complete energy inventory
 */
export interface EnergyManifest {
  /** Manifest timestamp */
  timestamp: Date;

  /** Total available energy in joules */
  totalAvailable: number;

  /** Total capacity in joules */
  totalCapacity: number;

  /** Energy sources */
  sources: EnergySource[];

  /** Storage systems */
  storage: EnergyStorage[];

  /** Current consumption in watts */
  currentConsumption: number;

  /** Projected runtime in seconds */
  projectedRuntime: number;

  /** Safety status */
  safetyStatus: 'nominal' | 'warning' | 'critical' | 'emergency';

  /** Warnings */
  warnings: string[];

  /** Errors */
  errors: string[];
}

/**
 * Energy allocation plan
 */
export interface EnergyAllocation {
  /** Allocation identifier */
  id: string;

  /** Target operation */
  operation: string;

  /** Required energy in joules */
  required: number;

  /** Allocated energy in joules */
  allocated: number;

  /** Source assignments */
  assignments: {
    sourceId: string;
    sourceType: EnergySourceType;
    amount: number;
  }[];

  /** Priority (1-10, 10 = highest) */
  priority: number;

  /** Allocation status */
  status: 'pending' | 'approved' | 'active' | 'completed' | 'cancelled';

  /** Start time */
  startTime?: Date;

  /** Completion time */
  completionTime?: Date;
}

// ============================================================================
// Safety and Monitoring
// ============================================================================

/**
 * Energy safety check
 */
export interface EnergySafetyCheck {
  /** Check name */
  name: string;

  /** Check type */
  type: 'energy-level' | 'containment' | 'radiation' | 'temperature' | 'structural';

  /** Check status */
  status: 'pass' | 'warning' | 'fail' | 'critical';

  /** Measured value */
  measuredValue: number;

  /** Expected value */
  expectedValue: number;

  /** Threshold value */
  threshold: number;

  /** Threshold type */
  thresholdType: 'minimum' | 'maximum' | 'range';

  /** Description */
  description: string;

  /** Corrective action if failed */
  correctiveAction?: string;

  /** Last checked */
  timestamp: Date;
}

/**
 * Energy monitoring data
 */
export interface EnergyMonitoring {
  /** Monitoring session ID */
  sessionId: string;

  /** Start time */
  startTime: Date;

  /** Current readings */
  current: {
    totalEnergy: number;
    power: number;
    temperature: number;
    radiation: number;
    pressure: number;
    stability: number;
  };

  /** Historical data points */
  history: {
    timestamp: Date;
    energy: number;
    power: number;
    temperature: number;
  }[];

  /** Safety checks */
  safetyChecks: EnergySafetyCheck[];

  /** Anomalies detected */
  anomalies: {
    type: string;
    severity: 'low' | 'medium' | 'high' | 'critical';
    description: string;
    timestamp: Date;
  }[];

  /** Overall status */
  status: 'normal' | 'monitoring' | 'warning' | 'alert' | 'emergency';
}

// ============================================================================
// Validation and Analysis
// ============================================================================

/**
 * Energy system validation parameters
 */
export interface EnergyValidationParams {
  /** Total energy available in joules */
  totalEnergy: number;

  /** Required energy in joules */
  requiredEnergy: number;

  /** Energy sources to validate */
  sources: EnergySource[];

  /** Storage systems to validate */
  storage: EnergyStorage[];

  /** Perform safety checks */
  safetyCheck?: boolean;

  /** Minimum stability required (0-1) */
  minStability?: number;
}

/**
 * Energy system validation result
 */
export interface EnergyValidationResult {
  /** Is system valid */
  isValid: boolean;

  /** Validation errors */
  errors: string[];

  /** Validation warnings */
  warnings: string[];

  /** Available energy in joules */
  availableEnergy: number;

  /** Energy deficit/surplus */
  energyDelta: number;

  /** Safety checks performed */
  safetyChecks: EnergySafetyCheck[];

  /** Recommendations */
  recommendations: string[];

  /** Risk assessment */
  risk: 'low' | 'medium' | 'high' | 'extreme';

  /** Validation timestamp */
  timestamp: Date;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for energy calculations
 */
export const ENERGY_CONSTANTS = {
  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Planck constant in J·s */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Reduced Planck constant in J·s */
  REDUCED_PLANCK: 1.054571817e-34,

  /** Gravitational constant in m³/kg·s² */
  GRAVITATIONAL_CONSTANT: 6.67430e-11,

  /** Vacuum permittivity in F/m */
  VACUUM_PERMITTIVITY: 8.8541878128e-12,

  /** Vacuum permeability in H/m */
  VACUUM_PERMEABILITY: 1.25663706212e-6,

  /** Boltzmann constant in J/K */
  BOLTZMANN_CONSTANT: 1.380649e-23,

  /** Flux capacitance in F_temporal */
  FLUX_CAPACITANCE: 1.21e9,

  /** Standard exotic matter density in kg/m³ */
  EXOTIC_MATTER_DENSITY: -1e15,

  /** Casimir force constant in N·m² */
  CASIMIR_CONSTANT: 1.3e-27,

  /** Maximum safe energy in joules */
  MAX_SAFE_ENERGY: 1e25,

  /** Maximum power density in W/m³ */
  MAX_POWER_DENSITY: 1e30,
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations
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
 * WIA-TIME-007 error codes
 */
export enum EnergyErrorCode {
  INSUFFICIENT_ENERGY = 'E001',
  EXOTIC_MATTER_UNSTABLE = 'E002',
  CAPACITOR_OVERLOAD = 'E003',
  SAFETY_THRESHOLD_EXCEEDED = 'E004',
  ZPE_EXTRACTION_FAILED = 'E005',
  ANTIMATTER_CONTAINMENT_BREACH = 'E006',
  INVALID_PARAMETERS = 'E007',
  ENERGY_SOURCE_FAILURE = 'E008',
  STORAGE_FAULT = 'E009',
  POWER_LIMIT_EXCEEDED = 'E010',
}

/**
 * Energy system error
 */
export class EnergySystemError extends Error {
  constructor(
    public code: EnergyErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'EnergySystemError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Energy sources
  EnergySource,

  // Power requirements
  PowerRequirementParams,
  PowerRequirements,
  EnergyComparison,

  // Exotic matter
  ExoticMatterConfig,
  ExoticMatterResult,

  // Zero-point energy
  ZeroPointEnergyParams,
  ZeroPointEnergyResult,

  // Casimir effect
  CasimirConfig,
  CasimirResult,

  // Flux capacitor
  FluxCapacitorConfig,
  FluxCapacitor,
  FluxCapacitorBank,

  // Storage
  EnergyStorage,

  // Management
  EnergyManifest,
  EnergyAllocation,

  // Safety
  EnergySafetyCheck,
  EnergyMonitoring,

  // Validation
  EnergyValidationParams,
  EnergyValidationResult,
};

export { ENERGY_CONSTANTS, EnergyErrorCode, EnergySystemError };
