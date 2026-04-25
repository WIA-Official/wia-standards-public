/**
 * WIA-AUTO-028: Solid-State Battery - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Energy Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Battery Types
// ============================================================================

/**
 * Solid electrolyte material types
 */
export type ElectrolyteType =
  | 'oxide-llzo'          // Garnet-type Li₇La₃Zr₂O₁₂
  | 'oxide-nasicon'       // NASICON-type
  | 'sulfide-argyrodite'  // Li₆PS₅Cl
  | 'sulfide-lgps'        // Li₁₀GeP₂S₁₂
  | 'polymer-peo'         // Polyethylene oxide
  | 'composite'           // Hybrid composite
  | 'custom';

/**
 * Electrode material types
 */
export interface ElectrodeMaterial {
  /** Material identifier */
  type: 'lithium-metal' | 'silicon' | 'graphite' | 'NMC811' | 'NMC622' | 'LFP' | 'lithium-rich' | 'custom';

  /** Specific capacity (mAh/g) */
  capacity: number;

  /** Nominal voltage vs. Li/Li+ (V) */
  voltage: number;

  /** Density (g/cm³) */
  density: number;

  /** Loading (mg/cm²) for electrodes */
  loading?: number;
}

/**
 * Battery chemistry configuration
 */
export interface BatteryChemistry {
  /** Anode material */
  anode: ElectrodeMaterial;

  /** Cathode material */
  cathode: ElectrodeMaterial;

  /** Solid electrolyte type */
  electrolyte: {
    type: ElectrolyteType;
    thickness: number;      // μm
    conductivity: number;   // S/cm
    density: number;        // g/cm³
  };

  /** Cell architecture */
  architecture?: 'bipolar-stacked' | 'wound' | 'stacked-pouch';
}

/**
 * Battery physical dimensions
 */
export interface BatteryDimensions {
  /** Length (mm) */
  length: number;

  /** Width (mm) */
  width: number;

  /** Height/thickness (mm) */
  height: number;

  /** Total mass (kg) */
  mass: number;

  /** Total volume (L) */
  volume?: number;

  /** Form factor */
  formFactor?: 'pouch' | 'prismatic' | 'cylindrical';
}

// ============================================================================
// Energy Density
// ============================================================================

/**
 * Energy density calculation parameters
 */
export interface EnergyDensityParams {
  /** Cell capacity (Ah) */
  capacity: number;

  /** Nominal voltage (V) */
  voltage: number;

  /** Total mass (kg) */
  mass: number;

  /** Total volume (L) */
  volume?: number;

  /** Coulombic efficiency (0-1) */
  efficiency?: number;
}

/**
 * Energy density calculation result
 */
export interface EnergyDensityResult {
  /** Gravimetric energy density (Wh/kg) */
  gravimetric: number;

  /** Volumetric energy density (Wh/L) */
  volumetric?: number;

  /** Total energy (Wh) */
  totalEnergy: number;

  /** Performance grade */
  grade: 'Premium' | 'Standard' | 'Basic';

  /** Certification level (1-3) */
  level: 1 | 2 | 3;

  /** Comparison to targets */
  comparison: {
    gravimetricTarget: number;
    volumetricTarget?: number;
    gravimetricAchievement: number;  // percentage
    volumetricAchievement?: number;  // percentage
  };
}

/**
 * Mass breakdown of battery components
 */
export interface MassBreakdown {
  /** Anode mass (kg) */
  anode: number;

  /** Cathode mass (kg) */
  cathode: number;

  /** Electrolyte mass (kg) */
  electrolyte: number;

  /** Current collectors mass (kg) */
  currentCollectors: number;

  /** Packaging mass (kg) */
  packaging: number;

  /** Other inactive materials (kg) */
  other?: number;

  /** Total mass (kg) */
  total: number;

  /** Active material ratio (%) */
  activeMaterialRatio: number;
}

// ============================================================================
// Charging Performance
// ============================================================================

/**
 * Fast charging capability levels
 */
export type ChargingLevel = 'ultra-fast' | 'fast' | 'standard' | 'slow';

/**
 * Charging time estimation parameters
 */
export interface ChargingTimeParams {
  /** Battery capacity (Ah) */
  capacity: number;

  /** Current state of charge (%) */
  currentSOC: number;

  /** Target state of charge (%) */
  targetSOC: number;

  /** Charging power (kW) */
  chargingPower: number;

  /** Battery temperature (°C) */
  temperature: number;

  /** Cooling system type */
  coolingType?: 'passive' | 'air' | 'liquid';

  /** Efficiency factor (0-1) */
  efficiency?: number;
}

/**
 * Charging time estimation result
 */
export interface ChargingTimeResult {
  /** Charging time (minutes) */
  minutes: number;

  /** C-rate (charge rate relative to capacity) */
  cRate: number;

  /** Charging level classification */
  level: ChargingLevel;

  /** Maximum temperature during charging (°C) */
  maxTemperature: number;

  /** Energy efficiency (%) */
  energyEfficiency: number;

  /** Current profile over time */
  currentProfile?: {
    time: number[];      // seconds
    current: number[];   // A
    voltage: number[];   // V
  };

  /** Warnings */
  warnings: string[];

  /** Cooling required */
  coolingRequired: boolean;
}

/**
 * Charging protocol definition
 */
export interface ChargingProtocol {
  /** Protocol name */
  name: string;

  /** Phases of charging */
  phases: ChargingPhase[];

  /** Maximum voltage (V) */
  maxVoltage: number;

  /** Minimum voltage (V) */
  minVoltage: number;

  /** Temperature limits */
  temperatureLimits: {
    min: number;  // °C
    max: number;  // °C
  };
}

/**
 * Individual charging phase
 */
export interface ChargingPhase {
  /** Phase identifier */
  id: string;

  /** SOC range */
  socRange: {
    start: number;  // %
    end: number;    // %
  };

  /** Charging mode */
  mode: 'CC' | 'CV' | 'CP';  // Constant Current, Constant Voltage, Constant Power

  /** Current or power setpoint */
  setpoint: {
    value: number;
    unit: 'A' | 'C-rate' | 'kW';
  };

  /** Cutoff condition */
  cutoff: {
    parameter: 'voltage' | 'current' | 'time' | 'soc';
    value: number;
  };
}

// ============================================================================
// Thermal Management
// ============================================================================

/**
 * Thermal simulation parameters
 */
export interface ThermalSimulationParams {
  /** Ambient temperature (°C) */
  ambientTemperature: number;

  /** Power input/output (W) */
  power: number;

  /** Cooling system type */
  coolingType: 'passive' | 'air' | 'liquid' | 'phase-change';

  /** Simulation duration (seconds) */
  duration: number;

  /** Initial battery temperature (°C) */
  initialTemperature?: number;

  /** Battery thermal properties */
  thermalProperties?: {
    mass: number;              // kg
    specificHeat: number;      // J/(kg·K)
    thermalConductivity: {
      inPlane: number;         // W/(m·K)
      throughPlane: number;    // W/(m·K)
    };
    surfaceArea: number;       // m²
  };
}

/**
 * Thermal simulation result
 */
export interface ThermalSimulationResult {
  /** Final battery temperature (°C) */
  finalTemperature: number;

  /** Maximum temperature reached (°C) */
  maxTemperature: number;

  /** Minimum temperature (°C) */
  minTemperature: number;

  /** Temperature profile over time */
  temperatureProfile: {
    time: number[];        // seconds
    temperature: number[]; // °C
  };

  /** Heat generated (J) */
  heatGenerated: number;

  /** Heat dissipated (J) */
  heatDissipated: number;

  /** Cooling power required (W) */
  coolingPowerRequired: number;

  /** Thermal runaway risk */
  thermalRunawayRisk: 'none' | 'low' | 'medium' | 'high';

  /** Warnings */
  warnings: string[];
}

/**
 * Temperature range specification
 */
export interface TemperatureRange {
  /** Operating range */
  operating: {
    min: number;  // °C
    max: number;  // °C
  };

  /** Optimal performance range */
  optimal: {
    min: number;  // °C
    max: number;  // °C
  };

  /** Storage range */
  storage: {
    min: number;  // °C
    max: number;  // °C
  };

  /** Critical limits */
  critical: {
    shutdownTemp: number;      // °C
    thermalRunawayTemp: number; // °C
  };
}

// ============================================================================
// Performance & Validation
// ============================================================================

/**
 * Battery performance specifications
 */
export interface BatteryPerformance {
  /** Energy density */
  energyDensity: {
    gravimetric: number;  // Wh/kg
    volumetric: number;   // Wh/L
  };

  /** Power density */
  powerDensity: {
    peak: number;         // W/kg
    continuous: number;   // W/kg
  };

  /** Cycle life */
  cycleLife: {
    cycles: number;
    capacityRetention: number;  // %
    dod: number;                // % depth of discharge
  };

  /** Fast charging */
  fastCharge: {
    level: ChargingLevel;
    time_10_80: number;    // minutes
    maxCRate: number;
    maxPower: number;      // kW
  };

  /** Temperature performance */
  temperature: TemperatureRange;

  /** Efficiency */
  efficiency: {
    coulombic: number;      // 0-1
    energyRoundTrip: number; // 0-1
  };

  /** Internal resistance */
  resistance: {
    dcr: number;           // mΩ
    acImpedance?: number;  // mΩ
  };
}

/**
 * Battery validation parameters
 */
export interface BatteryValidationParams {
  /** Capacity (Ah) */
  capacity: number;

  /** Voltage (V) */
  voltage: number;

  /** Energy density (Wh/kg) */
  energyDensity: number;

  /** Cycle life (cycles) */
  cycleLife: number;

  /** Fast charging time 10-80% (minutes) */
  fastChargingTime?: number;

  /** Temperature range */
  temperature?: {
    min: number;  // °C
    max: number;  // °C
  };

  /** Power density (W/kg) */
  powerDensity?: number;

  /** Safety certifications */
  certifications?: string[];
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Overall validation status */
  isValid: boolean;

  /** Certification level (1, 2, or 3) */
  level: 1 | 2 | 3;

  /** Performance grade */
  grade: 'Premium' | 'Standard' | 'Basic' | 'Below Standard';

  /** Individual check results */
  checks: {
    energyDensity: CheckResult;
    cycleLife: CheckResult;
    fastCharging: CheckResult;
    temperature: CheckResult;
    powerDensity: CheckResult;
    safety: CheckResult;
  };

  /** Warnings (non-blocking issues) */
  warnings: string[];

  /** Errors (blocking issues) */
  errors: string[];

  /** Recommendations for improvement */
  recommendations: string[];

  /** Comparison to standard requirements */
  comparisonToStandard: {
    parameter: string;
    required: number;
    actual: number;
    unit: string;
    status: 'pass' | 'fail' | 'exceeds';
  }[];
}

/**
 * Individual check result
 */
export interface CheckResult {
  /** Check status */
  status: 'pass' | 'fail' | 'warning' | 'not-tested';

  /** Measured value */
  measured?: number;

  /** Required value */
  required?: number;

  /** Unit */
  unit?: string;

  /** Details */
  details?: string;
}

// ============================================================================
// State of Health & Degradation
// ============================================================================

/**
 * Battery state of health parameters
 */
export interface StateOfHealth {
  /** Current capacity vs. initial (%) */
  capacityRetention: number;

  /** Resistance increase vs. initial (%) */
  resistanceIncrease: number;

  /** Cycle count */
  cycleCount: number;

  /** Equivalent full cycles */
  equivalentFullCycles: number;

  /** Calendar age (days) */
  calendarAge: number;

  /** Overall SOH (0-100%) */
  overall: number;

  /** Remaining useful life estimation */
  remainingLife?: {
    cycles: number;
    years: number;
  };

  /** Degradation mechanisms */
  degradation?: {
    capacityFade: number;     // % of total degradation
    powerFade: number;        // % of total degradation
    interfaceResistance: number;
    lithiumLoss: number;
  };
}

/**
 * Cycle life prediction parameters
 */
export interface CycleLifePredictionParams {
  /** Current cycle count */
  currentCycles: number;

  /** Current capacity retention (%) */
  currentRetention: number;

  /** Average DOD (%) */
  averageDOD: number;

  /** Average C-rate */
  averageCRate: number;

  /** Average temperature (°C) */
  averageTemperature: number;

  /** End-of-life criterion (% retention) */
  eolCriterion?: number;
}

/**
 * Cycle life prediction result
 */
export interface CycleLifePredictionResult {
  /** Predicted total cycle life */
  predictedCycles: number;

  /** Remaining cycles */
  remainingCycles: number;

  /** Confidence interval */
  confidence: {
    lower: number;  // cycles
    upper: number;  // cycles
  };

  /** Degradation rate (% per 100 cycles) */
  degradationRate: number;

  /** Expected EOL date */
  estimatedEOL?: Date;

  /** Factors affecting life */
  factors: {
    temperature: 'positive' | 'neutral' | 'negative';
    dod: 'positive' | 'neutral' | 'negative';
    cRate: 'positive' | 'neutral' | 'negative';
  };
}

// ============================================================================
// Safety & Testing
// ============================================================================

/**
 * Safety test results
 */
export interface SafetyTestResults {
  /** Test standard (e.g., "UL2580", "UN38.3") */
  standard: string;

  /** Test date */
  testDate: Date | string;

  /** Laboratory */
  laboratory: string;

  /** Individual test results */
  tests: {
    overcharge: TestResult;
    overDischarge: TestResult;
    shortCircuit: TestResult;
    nailPenetration: TestResult;
    crush: TestResult;
    impact: TestResult;
    thermalCycling: TestResult;
    vibration: TestResult;
    altitude: TestResult;
  };

  /** Overall result */
  overallResult: 'pass' | 'fail' | 'conditional';

  /** Certification issued */
  certificationIssued: boolean;

  /** Certificate number */
  certificateNumber?: string;
}

/**
 * Individual test result
 */
export interface TestResult {
  /** Test conducted */
  conducted: boolean;

  /** Result */
  result?: 'pass' | 'fail';

  /** Observations */
  observations?: string[];

  /** Measurements */
  measurements?: {
    parameter: string;
    value: number;
    unit: string;
    limit: number;
    status: 'within-limit' | 'exceeds-limit';
  }[];

  /** Date tested */
  testDate?: Date | string;
}

// ============================================================================
// Manufacturing & Quality
// ============================================================================

/**
 * Manufacturing quality metrics
 */
export interface QualityMetrics {
  /** Material purity (%) */
  purity: number;

  /** Thickness uniformity (% variation) */
  thicknessUniformity: number;

  /** Defect rate (per million) */
  defectRate: number;

  /** Interface contact (% of active area) */
  interfaceContact: number;

  /** Electrolyte density (% of theoretical) */
  electrolyteDensity: number;

  /** Ionic conductivity (S/cm) */
  ionicConductivity: number;

  /** Production yield (%) */
  yield: number;
}

/**
 * Production environment requirements
 */
export interface ProductionEnvironment {
  /** Humidity (% RH) */
  humidity: number;

  /** Dew point (°C) */
  dewPoint: number;

  /** Temperature (°C) */
  temperature: number;

  /** Cleanliness class */
  cleanliness: string;  // e.g., "ISO Class 5"

  /** Oxygen level (ppm) */
  oxygenLevel?: number;

  /** H2S detection (ppm) for sulfide electrolytes */
  h2sLevel?: number;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and electrochemical constants
 */
export const BATTERY_CONSTANTS = {
  /** Faraday constant (C/mol) */
  FARADAY: 96485.3329,

  /** Universal gas constant (J/(mol·K)) */
  GAS_CONSTANT: 8.314462618,

  /** Absolute zero (°C) */
  ABSOLUTE_ZERO: -273.15,

  /** Lithium properties */
  LITHIUM: {
    atomicMass: 6.94,         // g/mol
    density: 0.534,           // g/cm³
    capacity: 3860,           // mAh/g
    voltage: 0.0,             // V vs. Li/Li+
  },

  /** Energy density targets */
  ENERGY_DENSITY_TARGETS: {
    BASIC: {
      gravimetric: 300,       // Wh/kg
      volumetric: 700,        // Wh/L
    },
    STANDARD: {
      gravimetric: 400,       // Wh/kg
      volumetric: 1000,       // Wh/L
    },
    PREMIUM: {
      gravimetric: 450,       // Wh/kg
      volumetric: 1100,       // Wh/L
    },
  },

  /** Cycle life targets */
  CYCLE_LIFE_TARGETS: {
    BASIC: 1000,
    STANDARD: 2000,
    PREMIUM: 3000,
  },

  /** Temperature limits */
  TEMPERATURE: {
    OPTIMAL_MIN: 15,          // °C
    OPTIMAL_MAX: 35,          // °C
    OPERATING_MIN: -30,       // °C
    OPERATING_MAX: 60,        // °C
    STORAGE_MIN: -40,         // °C
    STORAGE_MAX: 80,          // °C
    SHUTDOWN: 70,             // °C
    THERMAL_RUNAWAY: 150,     // °C (solid-state)
  },

  /** Charging rate limits */
  CHARGING: {
    ULTRA_FAST_CRATE: 6,      // C-rate
    FAST_CRATE: 4,            // C-rate
    STANDARD_CRATE: 2,        // C-rate
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
 * Battery specification document
 */
export interface BatterySpecification {
  /** Standard compliance */
  standard: 'WIA-AUTO-028';
  version: string;

  /** Battery identification */
  battery: {
    id: string;
    manufacturer: string;
    model: string;
    type: 'solid-state';
    chemistry: BatteryChemistry;
  };

  /** Specifications */
  specifications: {
    nominalVoltage: number;
    capacity: number;
    energyDensity: {
      gravimetric: number;
      volumetric: number;
    };
    powerDensity: {
      peak: number;
      continuous: number;
    };
    dimensions: BatteryDimensions;
    cycleLife: number;
  };

  /** Performance */
  performance: BatteryPerformance;

  /** Safety */
  safety: {
    certifications: string[];
    features: string[];
  };

  /** Certification level */
  certificationLevel: 'Level-1-Basic' | 'Level-2-Standard' | 'Level-3-Premium';
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-028 error codes
 */
export enum BatteryErrorCode {
  INVALID_PARAMETERS = 'B001',
  INSUFFICIENT_CAPACITY = 'B002',
  TEMPERATURE_OUT_OF_RANGE = 'B003',
  VOLTAGE_OUT_OF_RANGE = 'B004',
  OVERCURRENT = 'B005',
  THERMAL_RUNAWAY = 'B006',
  VALIDATION_FAILED = 'B007',
  SAFETY_TEST_FAILED = 'B008',
  DEGRADATION_EXCEEDED = 'B009',
  MANUFACTURING_DEFECT = 'B010',
}

/**
 * Solid-state battery error
 */
export class SolidStateBatteryError extends Error {
  constructor(
    public code: BatteryErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'SolidStateBatteryError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  ElectrodeMaterial,
  BatteryChemistry,
  BatteryDimensions,

  // Energy density
  EnergyDensityParams,
  EnergyDensityResult,
  MassBreakdown,

  // Charging
  ChargingTimeParams,
  ChargingTimeResult,
  ChargingProtocol,
  ChargingPhase,

  // Thermal
  ThermalSimulationParams,
  ThermalSimulationResult,
  TemperatureRange,

  // Performance
  BatteryPerformance,
  BatteryValidationParams,
  ValidationResult,
  CheckResult,

  // Health
  StateOfHealth,
  CycleLifePredictionParams,
  CycleLifePredictionResult,

  // Safety
  SafetyTestResults,
  TestResult,

  // Manufacturing
  QualityMetrics,
  ProductionEnvironment,

  // Specification
  BatterySpecification,
};

export { BATTERY_CONSTANTS, BatteryErrorCode, SolidStateBatteryError };
