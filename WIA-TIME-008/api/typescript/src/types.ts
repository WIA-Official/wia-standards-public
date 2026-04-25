/**
 * WIA-TIME-008: Temporal Power Generation - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Temporal Energy Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional vector for spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Reactor types
 */
export type ReactorType =
  | 'single-stage'
  | 'multi-stage'
  | 'cascade'
  | 'quantum-temporal'
  | 'ctc-loop'
  | 'entropy-funnel';

/**
 * Safety levels
 */
export type SafetyLevel = 'standard' | 'high' | 'maximum' | 'extreme';

/**
 * Reactor status
 */
export type ReactorStatus =
  | 'offline'
  | 'initializing'
  | 'starting'
  | 'running'
  | 'optimizing'
  | 'degraded'
  | 'shutting-down'
  | 'emergency-shutdown'
  | 'failed';

/**
 * Power class rating
 */
export type PowerClass = 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8;

// ============================================================================
// Temporal Reactor
// ============================================================================

/**
 * Temporal reactor configuration
 */
export interface TemporalReactorConfig {
  /** Reactor type */
  reactorType: ReactorType;

  /** Number of stages (for multi-stage reactors) */
  stages?: number;

  /** Target power output in watts */
  outputPower: number;

  /** Target efficiency (0-1) */
  efficiency: number;

  /** Safety level */
  safetyLevel: SafetyLevel;

  /** Containment type */
  containmentType?: 'magnetic' | 'gravitational' | 'magnetic-gravitational';

  /** Cooling system */
  coolingSystem?: 'passive' | 'active-air' | 'active-liquid' | 'cryogenic';

  /** Startup time in seconds */
  startupTime?: number;

  /** Shutdown time in seconds */
  shutdownTime?: number;
}

/**
 * Temporal reactor instance
 */
export interface TemporalReactor {
  /** Unique reactor identifier */
  id: string;

  /** Reactor configuration */
  config: TemporalReactorConfig;

  /** Current status */
  status: ReactorStatus;

  /** Current power output in watts */
  powerOutput: number;

  /** Current efficiency (0-1) */
  currentEfficiency: number;

  /** Field stability (0-1) */
  stability: number;

  /** Core temperature in Kelvin */
  temperature: number;

  /** Chronon capture rate */
  chrononCaptureRate: number;

  /** Creation timestamp */
  created: Date;

  /** Last update timestamp */
  updated: Date;

  /** Operational hours */
  operationalHours: number;
}

/**
 * Reactor metrics
 */
export interface ReactorMetrics {
  /** Power output in watts */
  powerOutput: number;

  /** Voltage in volts */
  voltage: number;

  /** Current in amperes */
  current: number;

  /** Efficiency (0-1) */
  efficiency: number;

  /** Stability (0-1) */
  stability: number;

  /** Core temperature in Kelvin */
  temperature: number;

  /** Chronon flux */
  chrononFlux: number;

  /** Radiation level */
  radiationLevel: number;

  /** Containment integrity (0-1) */
  containmentIntegrity: number;

  /** Energy output total (joules) */
  totalEnergyOutput: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Power output calculation parameters
 */
export interface PowerOutputParams {
  /** Reactor type */
  reactorType: ReactorType;

  /** Number of stages */
  stages?: number;

  /** Chronon capture rate (chronons/second) */
  chrononCaptureRate: number;

  /** Conversion efficiency (0-1) */
  efficiency: number;

  /** Field stability (0-1) */
  stability?: number;
}

/**
 * Power output calculation result
 */
export interface PowerOutputResult {
  /** Power output in watts */
  power: number;

  /** Energy per hour in joules */
  energyPerHour: number;

  /** Energy per day in joules */
  energyPerDay: number;

  /** Power class rating */
  powerClass: PowerClass;

  /** Feasibility assessment */
  feasibility: 'practical' | 'achievable' | 'challenging' | 'theoretical';

  /** Formatted power output */
  powerFormatted: string;

  /** Equivalent comparisons */
  equivalents: {
    description: string;
    value: number;
  }[];
}

// ============================================================================
// Chrono-Dynamo
// ============================================================================

/**
 * Chrono-dynamo configuration
 */
export interface ChronoDynamoConfig {
  /** Rotor diameter in meters */
  rotorDiameter: number;

  /** Rotation rate in RPM */
  rotationRate: number;

  /** Number of temporal poles */
  poles: number;

  /** Field strength in Tesla (temporal) */
  fieldStrength: number;

  /** Rotor mass in kg */
  rotorMass: number;

  /** Coil windings */
  coilWindings: number;
}

/**
 * Chrono-dynamo instance
 */
export interface ChronoDynamo {
  /** Dynamo identifier */
  id: string;

  /** Configuration */
  config: ChronoDynamoConfig;

  /** Current rotation rate in RPM */
  currentRotationRate: number;

  /** Power output in watts */
  powerOutput: number;

  /** Efficiency (0-1) */
  efficiency: number;

  /** Torque in N·m */
  torque: number;

  /** Status */
  status: 'stopped' | 'accelerating' | 'running' | 'decelerating';

  /** Operational hours */
  operationalHours: number;
}

// ============================================================================
// Time Crystal Power Cell
// ============================================================================

/**
 * Time crystal types
 */
export type CrystalType = 'discrete' | 'continuous' | 'quantum-temporal';

/**
 * Time crystal cell configuration
 */
export interface TimeCrystalCellConfig {
  /** Crystal type */
  crystalType: CrystalType;

  /** Resonance frequency in Hz */
  resonanceFrequency: number;

  /** Energy capacity in joules */
  capacity: number;

  /** Charge rate in watts */
  chargeRate: number;

  /** Discharge rate in watts */
  dischargeRate: number;

  /** Operating temperature in Kelvin */
  operatingTemperature?: number;

  /** Quality factor */
  qualityFactor?: number;
}

/**
 * Time crystal cell instance
 */
export interface TimeCrystalCell {
  /** Cell identifier */
  id: string;

  /** Configuration */
  config: TimeCrystalCellConfig;

  /** Current charge level (0-1) */
  chargeLevel: number;

  /** Current temperature in Kelvin */
  temperature: number;

  /** Quantum coherence (0-1) */
  coherence: number;

  /** Status */
  status: 'idle' | 'charging' | 'discharging' | 'maintaining';

  /** Charge cycles completed */
  chargeCycles: number;

  /** Creation timestamp */
  created: Date;
}

/**
 * Charge operation parameters
 */
export interface ChargeParams {
  /** Power source */
  source: string;

  /** Target charge level (0-1) */
  targetCharge: number;

  /** Maximum time in seconds */
  maxTime: number;

  /** Charge rate override in watts */
  chargeRateOverride?: number;
}

/**
 * Discharge operation parameters
 */
export interface DischargeParams {
  /** Amount of energy to discharge in joules */
  amount: number;

  /** Discharge rate in watts */
  rate: number;

  /** Maximum time in seconds */
  maxTime?: number;
}

/**
 * Discharge operation result
 */
export interface DischargeResult {
  /** Amount discharged in joules */
  amount: number;

  /** Duration in seconds */
  duration: number;

  /** Average power in watts */
  averagePower: number;

  /** Final charge level (0-1) */
  finalChargeLevel: number;

  /** Success status */
  success: boolean;
}

// ============================================================================
// Entropy Harvesting
// ============================================================================

/**
 * Entropy harvesting methods
 */
export type HarvestingMethod =
  | 'temporal-gradient'
  | 'timeline-divergence'
  | 'quantum-decoherence'
  | 'causal-uncertainty';

/**
 * Entropy harvester configuration
 */
export interface EntropyHarvesterConfig {
  /** Harvesting method */
  harvestingMethod: HarvestingMethod;

  /** Harvesting efficiency (0-1) */
  efficiency: number;

  /** Maximum harvesting rate in watts */
  maxRate: number;

  /** Gradient source */
  gradientSource?: string;

  /** Minimum gradient required */
  minGradient?: number;
}

/**
 * Entropy harvester instance
 */
export interface EntropyHarvester {
  /** Harvester identifier */
  id: string;

  /** Configuration */
  config: EntropyHarvesterConfig;

  /** Current harvesting rate in watts */
  currentRate: number;

  /** Total harvested energy in joules */
  totalHarvested: number;

  /** Status */
  status: 'idle' | 'harvesting' | 'optimizing';

  /** Operational hours */
  operationalHours: number;
}

/**
 * Gradient configuration
 */
export interface GradientConfig {
  /** Timeline A identifier */
  timelineA: string;

  /** Timeline B identifier */
  timelineB: string;

  /** Divergence point */
  divergencePoint: Date;

  /** Gradient magnitude */
  gradientMagnitude?: number;
}

/**
 * Harvest operation parameters
 */
export interface HarvestParams {
  /** Harvesting duration in seconds */
  duration: number;

  /** Temporal gradient in seconds */
  temporalGradient?: number;

  /** Auto-optimize during harvest */
  autoOptimize?: boolean;
}

/**
 * Harvest operation result
 */
export interface HarvestResult {
  /** Total energy harvested in joules */
  total: number;

  /** Average power in watts */
  averagePower: number;

  /** Peak power in watts */
  peakPower: number;

  /** Efficiency achieved (0-1) */
  efficiency: number;

  /** Duration in seconds */
  duration: number;

  /** Success status */
  success: boolean;
}

// ============================================================================
// CTC Loop Generator
// ============================================================================

/**
 * CTC loop configuration
 */
export interface CTCLoopConfig {
  /** Wormhole throat radius in meters */
  throatRadius: number;

  /** Temporal offset in seconds (negative for backward) */
  temporalOffset: number;

  /** Exotic matter mass in kg */
  exoticMatter: number;

  /** Loop efficiency (0-1) */
  loopEfficiency: number;

  /** Seed power in watts */
  seedPower: number;

  /** Paradox detection enabled */
  paradoxDetection?: boolean;
}

/**
 * CTC loop generator instance
 */
export interface CTCLoopGenerator {
  /** Generator identifier */
  id: string;

  /** Configuration */
  config: CTCLoopConfig;

  /** Current power output in watts */
  powerOutput: number;

  /** Loop stability (0-1) */
  stability: number;

  /** Paradox probability (0-1) */
  paradoxProbability: number;

  /** Status */
  status: 'inactive' | 'establishing' | 'active' | 'degrading' | 'terminated';

  /** Loop iterations */
  iterations: number;
}

// ============================================================================
// Optimization
// ============================================================================

/**
 * Optimization parameters
 */
export interface OptimizationParams {
  /** Target efficiency (0-1) */
  targetEfficiency?: number;

  /** Target power output in watts */
  targetPower?: number;

  /** Maximum power in watts */
  maxPower?: number;

  /** Optimization duration in seconds */
  duration?: number;

  /** Auto-adjust parameters */
  autoAdjust?: boolean;
}

/**
 * Optimization result
 */
export interface OptimizationResult {
  /** Optimization success */
  success: boolean;

  /** Efficiency before */
  efficiencyBefore: number;

  /** Efficiency after */
  efficiencyAfter: number;

  /** Power before */
  powerBefore: number;

  /** Power after */
  powerAfter: number;

  /** Adjustments made */
  adjustments: {
    parameter: string;
    before: number;
    after: number;
  }[];

  /** Duration in seconds */
  duration: number;
}

// ============================================================================
// Safety
// ============================================================================

/**
 * Safety check
 */
export interface SafetyCheck {
  /** Check name */
  name: string;

  /** Status */
  status: 'pass' | 'warning' | 'fail';

  /** Measured value */
  value: number;

  /** Expected value */
  expected: number;

  /** Threshold */
  threshold: number;

  /** Description */
  description: string;

  /** Corrective action */
  correctiveAction?: string;
}

/**
 * Safety status
 */
export interface SafetyStatus {
  /** Overall safety status */
  overall: 'safe' | 'warning' | 'critical';

  /** Individual checks */
  checks: SafetyCheck[];

  /** Errors */
  errors: string[];

  /** Warnings */
  warnings: string[];

  /** Recommendations */
  recommendations: string[];

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for temporal power calculations
 */
export const POWER_CONSTANTS = {
  /** Planck constant in J·s */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Reduced Planck constant in J·s */
  HBAR: 1.054571817e-34,

  /** Boltzmann constant in J/K */
  BOLTZMANN_CONSTANT: 1.380649e-23,

  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Gravitational constant in m³/kg·s² */
  GRAVITATIONAL_CONSTANT: 6.67430e-11,

  /** Planck energy in joules */
  PLANCK_ENERGY: 1.9561e9,

  /** Planck time in seconds */
  PLANCK_TIME: 5.391247e-44,

  /** Energy per chronon (approximate) */
  ENERGY_PER_CHRONON: 1.9561e9,

  /** Maximum safe power per kg */
  MAX_SPECIFIC_POWER: 1e6, // 1 MW/kg

  /** Minimum reactor efficiency */
  MIN_EFFICIENCY: 0.3,

  /** Maximum reactor efficiency */
  MAX_EFFICIENCY: 0.99,

  /** Standard operating temperature in K */
  STANDARD_TEMPERATURE: 293.15,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-008 error codes
 */
export enum PowerErrorCode {
  POWER_OVERFLOW = 'P001',
  LOW_EFFICIENCY = 'P002',
  FIELD_UNSTABLE = 'P003',
  HIGH_TEMPERATURE = 'P004',
  CONTAINMENT_BREACH = 'P005',
  CHRONON_DEPLETION = 'P006',
  PARADOX_DETECTED = 'P007',
  INVALID_PARAMETERS = 'P008',
  REACTOR_OFFLINE = 'P009',
  SAFETY_VIOLATION = 'P010',
}

/**
 * Temporal power error
 */
export class TemporalPowerError extends Error {
  constructor(
    public code: PowerErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TemporalPowerError';
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
  // Core
  ReactorType,
  SafetyLevel,
  ReactorStatus,
  PowerClass,

  // Reactor
  TemporalReactorConfig,
  TemporalReactor,
  ReactorMetrics,
  PowerOutputParams,
  PowerOutputResult,

  // Chrono-Dynamo
  ChronoDynamoConfig,
  ChronoDynamo,

  // Time Crystal
  CrystalType,
  TimeCrystalCellConfig,
  TimeCrystalCell,
  ChargeParams,
  DischargeParams,
  DischargeResult,

  // Entropy
  HarvestingMethod,
  EntropyHarvesterConfig,
  EntropyHarvester,
  GradientConfig,
  HarvestParams,
  HarvestResult,

  // CTC
  CTCLoopConfig,
  CTCLoopGenerator,

  // Optimization
  OptimizationParams,
  OptimizationResult,

  // Safety
  SafetyCheck,
  SafetyStatus,
};

export { POWER_CONSTANTS, PowerErrorCode, TemporalPowerError };
