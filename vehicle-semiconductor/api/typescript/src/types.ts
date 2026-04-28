/**
 * WIA-AUTO-009: Vehicle Semiconductor - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Semiconductor Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Semiconductor Types and Classifications
// ============================================================================

/**
 * Automotive semiconductor component types
 */
export type SemiconductorType =
  | 'MCU'           // Microcontroller Unit
  | 'SoC'           // System on Chip
  | 'PMIC'          // Power Management IC
  | 'MOSFET'        // Power Transistor
  | 'IGBT'          // Insulated Gate Bipolar Transistor
  | 'Sensor-IC'     // Sensor Interface IC
  | 'CAN'           // CAN Transceiver
  | 'LIN'           // LIN Transceiver
  | 'Gate-Driver'   // MOSFET/IGBT Driver
  | 'OpAmp'         // Operational Amplifier
  | 'ADC'           // Analog-to-Digital Converter
  | 'DAC'           // Digital-to-Analog Converter
  | 'Memory'        // Flash, EEPROM, FRAM
  | 'Other';

/**
 * AEC-Q temperature grades
 */
export type TemperatureGrade = 0 | 1 | 2 | 3;

/**
 * Temperature grade specifications
 */
export interface TemperatureGradeSpec {
  grade: TemperatureGrade;
  min: number;      // Celsius
  max: number;      // Celsius
  typicalApplication: string;
}

/**
 * ASIL safety levels per ISO 26262
 */
export type ASILLevel = 'QM' | 'ASIL-A' | 'ASIL-B' | 'ASIL-C' | 'ASIL-D';

/**
 * AEC qualification standards
 */
export type AECQualification = 'AEC-Q100' | 'AEC-Q101' | 'AEC-Q200' | 'None';

/**
 * Package types
 */
export type PackageType =
  | 'LQFP'    // Low-profile Quad Flat Package
  | 'TQFP'    // Thin Quad Flat Package
  | 'BGA'     // Ball Grid Array
  | 'QFN'     // Quad Flat No-lead
  | 'DIP'     // Dual In-line Package
  | 'SOIC'    // Small Outline IC
  | 'TO-220'  // Transistor Outline
  | 'TO-247'  // Transistor Outline
  | 'DPAK'    // Discrete Package
  | 'Custom';

// ============================================================================
// Component Specifications
// ============================================================================

/**
 * Temperature range specification
 */
export interface TemperatureRange {
  /** Minimum temperature in Celsius */
  min: number;
  /** Maximum temperature in Celsius */
  max: number;
  /** Junction temperature maximum */
  tjMax?: number;
  /** Unit (always celsius for automotive) */
  unit: 'celsius';
}

/**
 * Voltage specification
 */
export interface VoltageSpec {
  /** Nominal voltage in V */
  nominal: number;
  /** Minimum voltage in V */
  min: number;
  /** Maximum voltage in V */
  max: number;
  /** Absolute maximum rating */
  absoluteMax?: number;
  /** Unit */
  unit: 'V';
}

/**
 * Current specification
 */
export interface CurrentSpec {
  /** Typical current in A */
  typical: number;
  /** Maximum current in A */
  max: number;
  /** Peak/surge current */
  peak?: number;
  /** Unit */
  unit: 'A';
}

/**
 * MCU/SoC core configuration
 */
export interface CoreConfiguration {
  /** Processor architecture */
  architecture: string;  // e.g., 'ARM Cortex-M7', 'ARM Cortex-A53'
  /** Number of cores */
  cores: number;
  /** Clock frequency in MHz */
  frequency: number;
  /** Lockstep cores for safety */
  lockstep?: boolean;
  /** FPU available */
  fpu?: boolean;
  /** DSP extensions */
  dsp?: boolean;
}

/**
 * Memory configuration
 */
export interface MemoryConfiguration {
  /** Flash memory in KB */
  flash?: number;
  /** RAM in KB */
  ram?: number;
  /** EEPROM/FRAM in KB */
  eeprom?: number;
  /** ROM in KB */
  rom?: number;
  /** Cache sizes */
  cache?: {
    l1?: number;
    l2?: number;
    l3?: number;
  };
  /** ECC protection */
  ecc?: boolean;
}

/**
 * Communication interfaces
 */
export interface CommunicationInterfaces {
  /** CAN channels */
  can?: number;
  /** CAN-FD support */
  canFD?: boolean;
  /** LIN channels */
  lin?: number;
  /** FlexRay channels */
  flexRay?: number;
  /** Ethernet ports */
  ethernet?: string[];  // e.g., ['100Base-T1', '1000Base-T1']
  /** SPI channels */
  spi?: number;
  /** I2C channels */
  i2c?: number;
  /** UART channels */
  uart?: number;
  /** USB ports */
  usb?: string[];  // e.g., ['USB 2.0', 'USB 3.1']
  /** PCIe lanes */
  pcie?: number;
}

/**
 * Safety features
 */
export interface SafetyFeatures {
  /** Lockstep CPU cores */
  lockstepCores?: boolean;
  /** Memory ECC */
  memoryECC?: boolean;
  /** CRC units */
  crcUnits?: number;
  /** Watchdog timers */
  watchdogTimers?: string[];  // e.g., ['independent', 'window']
  /** Voltage monitoring */
  voltageMonitoring?: boolean;
  /** Temperature monitoring */
  temperatureMonitoring?: boolean;
  /** Clock monitoring */
  clockMonitoring?: boolean;
  /** Safe state management */
  safeState?: boolean;
  /** BIST (Built-In Self Test) */
  bist?: boolean;
}

/**
 * Security features
 */
export interface SecurityFeatures {
  /** Secure boot */
  secureBoot?: boolean;
  /** Hardware encryption */
  encryption?: string[];  // e.g., ['AES-256', 'RSA-2048']
  /** Hash algorithms */
  hash?: string[];  // e.g., ['SHA-256', 'SHA-512']
  /** True Random Number Generator */
  trng?: boolean;
  /** Hardware Security Module */
  hsm?: boolean;
  /** TrustZone */
  trustZone?: boolean;
  /** Tamper detection */
  tamperDetection?: boolean;
}

/**
 * Complete component specification
 */
export interface ComponentSpecification {
  /** Part number */
  partNumber: string;
  /** Semiconductor type */
  type: SemiconductorType;
  /** Manufacturer */
  manufacturer: string;
  /** Description */
  description?: string;
  /** Datasheet URL */
  datasheetUrl?: string;

  /** Core configuration (for MCU/SoC) */
  core?: CoreConfiguration;
  /** Memory configuration */
  memory?: MemoryConfiguration;
  /** Package type */
  package: PackageType;
  /** Pin count */
  pinCount?: number;

  /** Temperature specifications */
  temperatureGrade: TemperatureGrade;
  temperatureRange: TemperatureRange;

  /** Voltage specifications */
  voltage: VoltageSpec;
  /** Current specifications */
  current?: CurrentSpec;

  /** ASIL safety level */
  asilLevel: ASILLevel;
  /** AEC qualification */
  aecQualification: AECQualification;

  /** Communication interfaces */
  interfaces?: CommunicationInterfaces;
  /** Safety features */
  safetyFeatures?: SafetyFeatures;
  /** Security features */
  securityFeatures?: SecurityFeatures;

  /** Reliability metrics */
  reliability: ReliabilityMetrics;

  /** Additional specifications */
  additionalSpecs?: Record<string, unknown>;
}

// ============================================================================
// Reliability and Testing
// ============================================================================

/**
 * Reliability metrics
 */
export interface ReliabilityMetrics {
  /** FIT rate (Failures In Time per billion hours) at reference temperature */
  fitRate: number;
  /** Reference temperature for FIT rate (Celsius) */
  referenceTemp: number;
  /** Mean Time Between Failures (hours) */
  mtbf: number;
  /** Expected operational lifetime (years) */
  expectedLifetime: number;
  /** Confidence level (percentage) */
  confidence?: number;
  /** Activation energy (eV) for Arrhenius calculation */
  activationEnergy?: number;
}

/**
 * Test status
 */
export type TestStatus = 'PASS' | 'FAIL' | 'PENDING' | 'N/A';

/**
 * AEC-Q100 test results
 */
export interface AECQ100TestResults {
  /** Preconditioning */
  preconditioning?: TestStatus;
  /** Temperature Cycling */
  temperatureCycling: TestStatus;
  /** High Temperature Operating Life */
  htol: TestStatus;
  /** High Temperature Storage Life */
  htsl: TestStatus;
  /** Temperature Humidity Bias */
  thb: TestStatus;
  /** Power Temperature Cycling */
  ptc?: TestStatus;
  /** Highly Accelerated Stress Test */
  hast?: TestStatus;
  /** Autoclave/uHAST */
  autoclave?: TestStatus;
  /** Electrostatic Discharge */
  esd: TestStatus;
  /** Latchup */
  latchup: TestStatus;
  /** Wire bond shear */
  wireBondShear?: TestStatus;
  /** Moisture Sensitivity Level */
  eMSL: TestStatus;
  /** Electrical Disturbance */
  electricalDisturbance: TestStatus;
}

/**
 * ESD test results
 */
export interface ESDTestResults {
  /** Human Body Model (V) */
  hbm: number;
  /** Charged Device Model (V) */
  cdm: number;
  /** Machine Model (V) */
  mm?: number;
  /** Status */
  status: TestStatus;
}

/**
 * Environmental test results
 */
export interface EnvironmentalTestResults {
  /** Vibration test */
  vibration?: {
    status: TestStatus;
    duration: number;      // hours
    acceleration: number;  // g
    frequency?: string;    // e.g., '10-2000 Hz'
  };
  /** Mechanical shock */
  mechanicalShock?: {
    status: TestStatus;
    acceleration: number;  // g
    duration: number;      // ms
  };
  /** Salt spray */
  saltSpray?: {
    status: TestStatus;
    duration: number;  // hours
  };
}

/**
 * Complete test results
 */
export interface TestResults {
  /** Component ID */
  componentId: string;
  /** Test date */
  testDate: Date | string;
  /** Lot number */
  lotNumber: string;
  /** Wafer number */
  waferNumber?: string;
  /** Sample size */
  sampleSize?: number;

  /** AEC-Q100 tests */
  aecQ100?: AECQ100TestResults;
  /** ESD tests */
  esd?: ESDTestResults;
  /** Environmental tests */
  environmental?: EnvironmentalTestResults;

  /** Overall status */
  overallStatus: TestStatus;
  /** Number of failures */
  failures: number;
  /** Notes */
  notes?: string;
}

/**
 * Certification information
 */
export interface Certification {
  /** AEC-Q100 certified */
  aecQ100: boolean;
  /** AEC-Q101 certified */
  aecQ101?: boolean;
  /** AEC-Q200 certified */
  aecQ200?: boolean;
  /** ISO 26262 ASIL level */
  iso26262: ASILLevel;
  /** Certification date */
  certificationDate: Date | string;
  /** Expiry date */
  expiryDate: Date | string;
  /** Certificate number */
  certificateNumber?: string;
  /** Certification body */
  certificationBody?: string;
}

// ============================================================================
// Functional Safety (ISO 26262)
// ============================================================================

/**
 * Safety Integrity Level requirements
 */
export interface ASILRequirements {
  /** Required ASIL level */
  level: ASILLevel;
  /** Target PMHF (Probabilistic Metric for Hardware Failures) in FIT */
  targetPMHF: number;
  /** Target Latent Fault Metric (percentage) */
  targetLFM: number;
  /** Required Diagnostic Coverage (percentage) */
  requiredDC: number;
  /** Single Point Fault Metric target */
  spfm?: number;
}

/**
 * Safety mechanism
 */
export interface SafetyMechanism {
  /** Mechanism ID */
  id: string;
  /** Mechanism name */
  name: string;
  /** Description */
  description: string;
  /** Type */
  type: 'detection' | 'prevention' | 'mitigation' | 'redundancy';
  /** Diagnostic coverage contribution (percentage) */
  dcContribution: number;
  /** Enabled */
  enabled: boolean;
}

/**
 * Safety metrics
 */
export interface SafetyMetrics {
  /** ASIL level achieved */
  asilAchieved: ASILLevel;
  /** Diagnostic Coverage (percentage) */
  diagnosticCoverage: number;
  /** PMHF (FIT) */
  pmhf: number;
  /** Latent Fault Metric (percentage) */
  lfm: number;
  /** Single Point Fault Metric (percentage) */
  spfm?: number;
  /** Safety mechanisms */
  mechanisms: SafetyMechanism[];
}

/**
 * Safety assessment parameters
 */
export interface SafetyAssessment {
  /** Required ASIL level */
  asilRequired: ASILLevel;
  /** Application description */
  application: string;
  /** Component specifications */
  componentSpecs: ComponentSpecification;
  /** Operating conditions */
  operatingConditions?: {
    temperature: number;
    voltage: number;
    dutyCycle?: number;
  };
}

/**
 * Safety assessment result
 */
export interface SafetyResult {
  /** Is compliant with required ASIL */
  isCompliant: boolean;
  /** ASIL achieved */
  asilAchieved: ASILLevel;
  /** Safety metrics */
  metrics: SafetyMetrics;
  /** Warnings */
  warnings: string[];
  /** Recommendations */
  recommendations: string[];
  /** Gaps to address */
  gaps?: string[];
}

// ============================================================================
// Component Classification
// ============================================================================

/**
 * Component classification parameters
 */
export interface ComponentClassification {
  /** Part number */
  partNumber: string;
  /** Semiconductor type */
  type: SemiconductorType;
  /** Temperature grade */
  temperatureGrade: TemperatureGrade;
  /** Voltage class */
  voltageClass: string;  // e.g., '12V', '48V', '400V', '800V'
  /** Required ASIL level */
  asilLevel: ASILLevel;
  /** Application */
  application: string;
}

/**
 * Classification result
 */
export interface ClassificationResult {
  /** Is automotive grade */
  isAutomotiveGrade: boolean;
  /** AEC qualification standard */
  aecQualification: AECQualification;
  /** Recommended applications */
  recommendedApplications: string[];
  /** Safety rating */
  safetyRating: ASILLevel;
  /** Reliability metrics */
  reliabilityMetrics: ReliabilityMetrics;
  /** Temperature suitability */
  temperatureSuitability: {
    grade: TemperatureGrade;
    locations: string[];
  };
  /** Cost category */
  costCategory?: 'low' | 'medium' | 'high' | 'premium';
}

// ============================================================================
// Reliability Calculations
// ============================================================================

/**
 * Reliability calculation parameters
 */
export interface ReliabilityParams {
  /** FIT rate at reference temperature */
  fitRate: number;
  /** Operating temperature (Celsius) */
  operatingTemp: number;
  /** Reference temperature (Celsius), typically 55°C */
  referenceTemp: number;
  /** Activation energy (eV), typically 0.7 */
  activationEnergy: number;
  /** Operating hours per year */
  operatingHoursPerYear?: number;
}

/**
 * Calculate reliability metrics result
 */
export interface CalculatedReliability {
  /** Adjusted FIT rate at operating temperature */
  fitRate: number;
  /** Mean Time Between Failures (hours) */
  mtbf: number;
  /** Failure rate per year */
  failureRatePerYear: number;
  /** Expected lifetime (years) */
  expectedLifetime: number;
  /** Confidence level (percentage) */
  confidence: number;
  /** Acceleration factor */
  accelerationFactor: number;
}

// ============================================================================
// Validation and Quality
// ============================================================================

/**
 * AEC-Q100 validation parameters
 */
export interface AECQ100Validation {
  /** Component ID */
  componentId: string;
  /** Temperature grade */
  temperatureGrade: TemperatureGrade;
  /** Test results */
  testResults: AECQ100TestResults;
  /** Additional environmental tests */
  environmentalTests?: EnvironmentalTestResults;
  /** Sample size */
  sampleSize?: number;
}

/**
 * Validation result
 */
export interface ValidationResult {
  /** Is compliant with AEC-Q100 */
  isCompliant: boolean;
  /** Certification level achieved */
  certificationLevel: string;
  /** Failed tests */
  failedTests: string[];
  /** Warnings */
  warnings: string[];
  /** Recommendations */
  recommendations: string[];
  /** Certification */
  certification?: Certification;
}

// ============================================================================
// Power Management
// ============================================================================

/**
 * Power converter specifications
 */
export interface PowerConverterSpec {
  /** Converter type */
  type: 'buck' | 'boost' | 'buck-boost' | 'LDO' | 'charge-pump';
  /** Input voltage range */
  inputVoltage: VoltageSpec;
  /** Output voltage range */
  outputVoltage: VoltageSpec;
  /** Output current */
  outputCurrent: CurrentSpec;
  /** Efficiency (percentage) */
  efficiency: number;
  /** Switching frequency (kHz) */
  switchingFrequency?: number;
  /** Quiescent current (µA) */
  quiescentCurrent?: number;
  /** Dropout voltage (mV) */
  dropoutVoltage?: number;
}

/**
 * Battery management system specifications
 */
export interface BMSSpecification {
  /** Number of cells supported */
  cellCount: number;
  /** Cell voltage range */
  cellVoltage: VoltageSpec;
  /** Current measurement range */
  currentRange: CurrentSpec;
  /** Voltage measurement accuracy (mV) */
  voltageAccuracy: number;
  /** Current measurement accuracy (percentage) */
  currentAccuracy: number;
  /** Temperature channels */
  temperatureChannels: number;
  /** Cell balancing */
  cellBalancing: 'passive' | 'active' | 'none';
  /** Communication interface */
  communication: string[];  // e.g., ['CAN', 'SPI', 'I2C']
}

// ============================================================================
// Sensor Specifications
// ============================================================================

/**
 * Sensor IC specifications
 */
export interface SensorICSpec {
  /** Sensor type */
  sensorType: 'current' | 'temperature' | 'pressure' | 'position' | 'hall' | 'imu' | 'other';
  /** Measurement range */
  measurementRange: {
    min: number;
    max: number;
    unit: string;
  };
  /** Accuracy */
  accuracy: {
    value: number;
    unit: string;
  };
  /** Resolution */
  resolution?: {
    value: number;
    unit: string;
  };
  /** Output type */
  outputType: 'analog' | 'digital' | 'pwm';
  /** Response time (ms) */
  responseTime?: number;
  /** Bandwidth (kHz) */
  bandwidth?: number;
}

// ============================================================================
// Physical Constants and Formulas
// ============================================================================

/**
 * Physical constants for automotive semiconductors
 */
export const AUTOMOTIVE_CONSTANTS = {
  /** Boltzmann constant (eV/K) */
  BOLTZMANN: 8.617333262e-5,

  /** Typical activation energy (eV) */
  ACTIVATION_ENERGY: 0.7,

  /** Reference temperature for FIT calculations (Celsius) */
  REFERENCE_TEMP: 55,

  /** Standard automotive voltages */
  VOLTAGES: {
    LV_12V: { nominal: 12, min: 9, max: 16 },
    LV_24V: { nominal: 24, min: 18, max: 32 },
    LV_48V: { nominal: 48, min: 36, max: 60 },
    HV_400V: { nominal: 400, min: 250, max: 450 },
    HV_800V: { nominal: 800, min: 500, max: 900 },
  },

  /** Temperature grades */
  TEMP_GRADES: {
    0: { min: -40, max: 150, application: 'Engine compartment' },
    1: { min: -40, max: 125, application: 'Under hood' },
    2: { min: -40, max: 105, application: 'Passenger cabin' },
    3: { min: -40, max: 85, application: 'Mild environment' },
  } as Record<TemperatureGrade, { min: number; max: number; application: string }>,

  /** ASIL target metrics */
  ASIL_TARGETS: {
    'ASIL-D': { pmhf: 10, lfm: 1, dc: 99 },
    'ASIL-C': { pmhf: 100, lfm: 10, dc: 97 },
    'ASIL-B': { pmhf: 100, lfm: 10, dc: 90 },
    'ASIL-A': { pmhf: 1000, lfm: 10, dc: 60 },
    'QM': { pmhf: 10000, lfm: 100, dc: 0 },
  } as Record<ASILLevel, { pmhf: number; lfm: number; dc: number }>,

  /** Expected operational lifetime (years) */
  EXPECTED_LIFETIME: 20,

  /** Operating hours per year (assumes continuous operation) */
  HOURS_PER_YEAR: 8760,
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
 * WIA-AUTO-009 error codes
 */
export enum SemiconductorErrorCode {
  INVALID_TEMPERATURE_GRADE = 'A009-001',
  INSUFFICIENT_ASIL = 'A009-002',
  TEST_FAILURE = 'A009-003',
  CERTIFICATION_EXPIRED = 'A009-004',
  RELIABILITY_BELOW_TARGET = 'A009-005',
  INCOMPATIBLE_VOLTAGE = 'A009-006',
  SAFETY_MECHANISM_MISSING = 'A009-007',
  DIAGNOSTIC_COVERAGE_LOW = 'A009-008',
  INVALID_COMPONENT_TYPE = 'A009-009',
  QUALIFICATION_INCOMPLETE = 'A009-010',
}

/**
 * Vehicle semiconductor error
 */
export class VehicleSemiconductorError extends Error {
  constructor(
    public code: SemiconductorErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'VehicleSemiconductorError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Component specs
  ComponentSpecification,
  CoreConfiguration,
  MemoryConfiguration,
  CommunicationInterfaces,
  SafetyFeatures,
  SecurityFeatures,
  TemperatureRange,
  VoltageSpec,
  CurrentSpec,

  // Classification
  ComponentClassification,
  ClassificationResult,

  // Reliability
  ReliabilityMetrics,
  ReliabilityParams,
  CalculatedReliability,

  // Testing
  TestResults,
  AECQ100TestResults,
  ESDTestResults,
  EnvironmentalTestResults,
  Certification,

  // Validation
  AECQ100Validation,
  ValidationResult,

  // Safety
  ASILRequirements,
  SafetyMechanism,
  SafetyMetrics,
  SafetyAssessment,
  SafetyResult,

  // Power management
  PowerConverterSpec,
  BMSSpecification,

  // Sensors
  SensorICSpec,
};

export {
  AUTOMOTIVE_CONSTANTS,
  SemiconductorErrorCode,
  VehicleSemiconductorError,
};
