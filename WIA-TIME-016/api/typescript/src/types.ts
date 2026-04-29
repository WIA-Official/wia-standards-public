/**
 * WIA-TIME-016: Temporal Material - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Temporal Materials Science Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Material certification levels
 */
export type CertificationLevel =
  | 'WIA-TEMP-CERT-1' // Basic (< 1 Tesla)
  | 'WIA-TEMP-CERT-2' // Intermediate (1-5 Tesla)
  | 'WIA-TEMP-CERT-3' // Advanced (5-10 Tesla)
  | 'WIA-TEMP-CERT-4'; // Exotic (> 10 Tesla)

/**
 * Material categories
 */
export type MaterialCategory =
  | 'exotic-matter'
  | 'temporal-alloy'
  | 'chrono-shield'
  | 'quantum-composite';

/**
 * Material status
 */
export type MaterialStatus =
  | 'certified'
  | 'pending-certification'
  | 'expired'
  | 'revoked'
  | 'not-certified';

/**
 * Test result status
 */
export type TestStatus =
  | 'passed'
  | 'failed'
  | 'in-progress'
  | 'pending'
  | 'not-applicable';

// ============================================================================
// Exotic Matter
// ============================================================================

/**
 * Exotic matter production methods
 */
export type ExoticMatterProductionMethod =
  | 'casimir-effect'
  | 'squeezed-light'
  | 'dynamic-casimir'
  | 'quantum-vacuum-engineering';

/**
 * Electromagnetic confinement types
 */
export type ConfinementType =
  | 'electromagnetic'
  | 'penning-trap'
  | 'magnetic-bottle'
  | 'optical-trap';

/**
 * Exotic matter configuration
 */
export interface ExoticMatterConfig {
  /** Energy density (negative value in J/m³) */
  energyDensity: number;

  /** Quantity in kilograms */
  quantity: number;

  /** Production method */
  productionMethod: ExoticMatterProductionMethod;

  /** Confinement type */
  confinementType: ConfinementType;

  /** Target stability duration in seconds */
  stabilityDuration: number;

  /** Operating temperature in Kelvin */
  temperature: number;

  /** Magnetic field strength in Tesla */
  magneticField: number;

  /** Vacuum level in Pascals */
  vacuumLevel: number;
}

/**
 * Exotic matter state
 */
export interface ExoticMatterState {
  /** Current status */
  status: 'stable' | 'degrading' | 'unstable' | 'failed';

  /** Current energy density (J/m³) */
  currentEnergyDensity: number;

  /** Current quantity (kg) */
  currentQuantity: number;

  /** Decay rate (% per hour) */
  decayRate: number;

  /** Position stability (nm) */
  positionStability: number;

  /** Confinement field strength (Tesla) */
  confinementField: number;

  /** Time since production (seconds) */
  age: number;

  /** Warnings */
  warnings: string[];

  /** Last measurement timestamp */
  lastMeasurement: Date;
}

/**
 * Exotic matter stability analysis
 */
export interface StabilityAnalysis {
  /** Overall stability percentage (0-100) */
  percentage: number;

  /** Estimated remaining lifetime (seconds) */
  remainingLifetime: number;

  /** Decay projection curve */
  decayProjection: {
    time: number; // seconds from now
    quantity: number; // kg
  }[];

  /** Stability trend */
  trend: 'improving' | 'stable' | 'degrading' | 'critical';

  /** Recommended action */
  recommendation: string;
}

// ============================================================================
// Temporal Alloys
// ============================================================================

/**
 * Temporal alloy types
 */
export type TemporalAlloyType =
  | 'CTA-7' // Chronium-Titanium Alloy, standard
  | 'CTA-9' // Chronium-Titanium Alloy, high performance
  | 'TS-316' // Temporal Steel, enhanced stainless
  | 'TS-410' // Temporal Steel, ferritic
  | 'NC-1'; // Neutronium Composite

/**
 * Alloy composition (element percentages)
 */
export interface AlloyComposition {
  /** Element symbol to percentage (0-1) */
  [element: string]: number;
}

/**
 * Temporal alloy configuration
 */
export interface TemporalAlloyConfig {
  /** Alloy type */
  type: TemporalAlloyType;

  /** Chemical composition */
  composition: AlloyComposition;

  /** Temporal Stability Index (0-1) */
  temporalStabilityIndex: number;

  /** Thickness in millimeters */
  thickness: number;

  /** Density in g/cm³ */
  density: number;

  /** Melting point in Celsius */
  meltingPoint: number;

  /** Tensile strength in MPa */
  tensileStrength: number;

  /** Maximum temporal field in Tesla */
  maxTemporalField: number;

  /** Quantum coherence time in seconds */
  quantumCoherenceTime?: number;

  /** Operating temperature range in Kelvin */
  temperatureRange: {
    min: number;
    max: number;
  };
}

/**
 * Temporal alloy properties
 */
export interface AlloyProperties {
  /** Current Temporal Stability Index */
  currentTSI: number;

  /** Baseline TSI */
  baselineTSI: number;

  /** Degradation percentage */
  degradation: number;

  /** Operating hours */
  operatingHours: number;

  /** Estimated remaining life (hours) */
  remainingLife: number;

  /** Current mechanical properties */
  mechanical: {
    tensileStrength: number; // MPa
    hardness: number; // HV
    elasticity: number; // GPa
  };

  /** Current electrical properties */
  electrical: {
    resistivity: number; // μΩ·cm
    conductivity: number; // MS/m
  };

  /** Quantum properties */
  quantum?: {
    coherenceTime: number; // seconds
    decoherenceRate: number; // s⁻¹
  };
}

/**
 * Temporal alloy test results
 */
export interface AlloyTestResults {
  /** Test timestamp */
  timestamp: Date;

  /** Test type */
  testType: string;

  /** Test status */
  status: TestStatus;

  /** TSI measurement */
  tsi: number;

  /** Field exposure parameters */
  fieldExposure: {
    strength: number; // Tesla
    duration: number; // hours
    temperature: number; // K
  };

  /** Mechanical test results */
  mechanical?: {
    tensileStrength: number;
    hardness: number;
    impact: number;
  };

  /** Microstructural analysis */
  microstructure?: {
    grainSize: number; // μm
    porosity: number; // %
    defects: string[];
  };

  /** Pass/fail criteria */
  passed: boolean;

  /** Notes and observations */
  notes: string;
}

// ============================================================================
// Chrono-Shielding Materials
// ============================================================================

/**
 * Shielding layer materials
 */
export type ShieldingLayerMaterial =
  | 'lead-tungsten' // Pb-W alloy
  | 'exotic-polymer' // EMIP
  | 'superconducting-ceramic' // YBCO
  | 'temporal-reflector'; // TWR

/**
 * Shielding coverage configuration
 */
export type CoverageType =
  | '360-spherical'
  | 'cylindrical'
  | 'planar'
  | 'custom';

/**
 * Shielding layer definition
 */
export interface ShieldingLayer {
  /** Layer material */
  material: ShieldingLayerMaterial;

  /** Thickness in millimeters */
  thickness: number;

  /** Density in g/cm³ */
  density?: number;

  /** Operating temperature in Kelvin */
  operatingTemperature?: number;

  /** Special requirements */
  requirements?: string[];
}

/**
 * Chrono-shield configuration
 */
export interface ChronoShieldConfig {
  /** Shield layers (ordered from outer to inner) */
  layers: ShieldingLayer[];

  /** Coverage type */
  coverage: CoverageType;

  /** Design field strength in Tesla */
  fieldStrength: number;

  /** Dimensions */
  dimensions?: {
    radius?: number; // meters (for spherical)
    length?: number; // meters (for cylindrical)
    diameter?: number; // meters (for cylindrical)
    area?: number; // m² (for planar)
  };

  /** Total weight in kg */
  totalWeight?: number;

  /** Service life in years */
  serviceLife?: number;
}

/**
 * Shield effectiveness metrics
 */
export interface ShieldEffectiveness {
  /** Temporal radiation blocking percentage */
  blocking: number;

  /** Chrono-particle attenuation percentage */
  attenuation: number;

  /** Field strength reduction percentage */
  fieldReduction: number;

  /** Residual field inside shield (Tesla) */
  residualField: number;

  /** Effective dose reduction factor */
  doseReductionFactor: number;

  /** Shield integrity status */
  integrity: 'excellent' | 'good' | 'fair' | 'poor' | 'failed';

  /** Identified weak points */
  weakPoints: string[];
}

/**
 * Shield maintenance record
 */
export interface ShieldMaintenanceRecord {
  /** Maintenance date */
  date: Date;

  /** Maintenance type */
  type: 'inspection' | 'repair' | 'replacement' | 'upgrade';

  /** Layers affected */
  layersAffected: number[];

  /** Work performed */
  workPerformed: string;

  /** Parts replaced */
  partsReplaced?: string[];

  /** Test results after maintenance */
  postMaintenanceTest?: ShieldEffectiveness;

  /** Technician ID */
  technicianId: string;

  /** Next maintenance due date */
  nextMaintenanceDue: Date;
}

// ============================================================================
// Quantum-Stable Composites
// ============================================================================

/**
 * Quantum composite types
 */
export type QuantumCompositeType =
  | 'CWC-88' // Crystalline Tungsten-Carbide
  | 'DNV-Q1' // Diamond-Nitrogen-Vacancy
  | 'TIC-Bi2Se3' // Topological Insulator Composite
  | 'custom';

/**
 * Quantum composite configuration
 */
export interface QuantumCompositeConfig {
  /** Composite type */
  type: QuantumCompositeType;

  /** Material composition */
  composition: {
    primaryPhase: string;
    secondaryPhases?: string[];
    dopants?: { [element: string]: number }; // ppm
  };

  /** Quantum coherence time in seconds */
  coherenceTime: number;

  /** Operating temperature range in Kelvin */
  temperatureRange: {
    min: number;
    max: number;
  };

  /** Temporal stability index */
  temporalStabilityIndex: number;

  /** Application */
  application: 'quantum-sensing' | 'quantum-computing' | 'precision-timing' | 'navigation' | 'other';
}

/**
 * Quantum state properties
 */
export interface QuantumStateProperties {
  /** T1 relaxation time (seconds) */
  t1: number;

  /** T2 coherence time (seconds) */
  t2: number;

  /** T2* dephasing time (seconds) */
  t2Star: number;

  /** Fidelity (0-1) */
  fidelity: number;

  /** Decoherence rate (s⁻¹) */
  decoherenceRate: number;

  /** Noise spectral density */
  noiseDensity: number;

  /** Current operating temperature (K) */
  temperature: number;
}

/**
 * Quantum composite test results
 */
export interface QuantumCompositeTestResults {
  /** Test timestamp */
  timestamp: Date;

  /** Quantum state measurements */
  quantumState: QuantumStateProperties;

  /** Material characterization */
  material: {
    crystallinity: number; // %
    defectDensity: number; // cm⁻³
    impurityLevel: number; // ppm
  };

  /** Temporal field response */
  fieldResponse: {
    fieldStrength: number; // Tesla
    coherenceReduction: number; // %
    recoveryTime: number; // seconds
  };

  /** Status */
  status: TestStatus;

  /** Notes */
  notes: string;
}

// ============================================================================
// Material Degradation
// ============================================================================

/**
 * Degradation mechanisms
 */
export type DegradationMechanism =
  | 'temporal-fatigue'
  | 'quantum-decoherence'
  | 'atomic-displacement'
  | 'radiation-damage'
  | 'thermal-cycling'
  | 'corrosion';

/**
 * Degradation model types
 */
export type DegradationModelType =
  | 'linear'
  | 'exponential'
  | 'threshold-based'
  | 'custom';

/**
 * Degradation parameters
 */
export interface DegradationParameters {
  /** Model type */
  modelType: DegradationModelType;

  /** Degradation mechanism */
  mechanism: DegradationMechanism;

  /** Model coefficients */
  coefficients: {
    alpha?: number; // linear coefficient
    beta?: number; // exponential coefficient
    gamma?: number; // threshold coefficient
    threshold?: number; // threshold value
  };

  /** Environmental factors */
  environment: {
    fieldStrength: number; // Tesla
    temperature: number; // K
    exposureTime: number; // hours
    cycleCount?: number; // for fatigue
  };
}

/**
 * Degradation analysis results
 */
export interface DegradationAnalysis {
  /** Current degradation percentage */
  currentDegradation: number;

  /** Degradation rate (% per 1000 hours) */
  degradationRate: number;

  /** Predicted end of life (hours from now) */
  predictedEOL: number;

  /** Confidence interval */
  confidence: {
    lower: number; // hours
    upper: number; // hours
    level: number; // % (e.g., 95)
  };

  /** Degradation curve */
  curve: {
    time: number; // hours from start
    degradation: number; // %
  }[];

  /** Dominant mechanism */
  dominantMechanism: DegradationMechanism;

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Material Certification
// ============================================================================

/**
 * Certification test types
 */
export type CertificationTestType =
  | 'temporal-stress-test'
  | 'quantum-stability-test'
  | 'radiation-resistance-test'
  | 'thermal-cycling-test'
  | 'chrono-fatigue-test'
  | 'mechanical-properties'
  | 'chemical-analysis'
  | 'microstructure-analysis';

/**
 * Certification test configuration
 */
export interface CertificationTestConfig {
  /** Test type */
  type: CertificationTestType;

  /** Duration in hours */
  duration: number;

  /** Test parameters */
  parameters: {
    [key: string]: number | string | boolean;
  };

  /** Acceptance criteria */
  acceptanceCriteria: {
    [metric: string]: {
      min?: number;
      max?: number;
      target?: number;
    };
  };
}

/**
 * Certification test result
 */
export interface CertificationTestResult {
  /** Test configuration */
  config: CertificationTestConfig;

  /** Test start time */
  startTime: Date;

  /** Test end time */
  endTime: Date;

  /** Test status */
  status: TestStatus;

  /** Measured values */
  measurements: {
    [metric: string]: number;
  };

  /** Pass/fail for each criterion */
  criteriaResults: {
    [criterion: string]: boolean;
  };

  /** Overall pass/fail */
  passed: boolean;

  /** Test report */
  report: string;

  /** Attachments */
  attachments?: string[];
}

/**
 * Material certification
 */
export interface MaterialCertification {
  /** Certification ID */
  certificationId: string;

  /** Material information */
  material: {
    name: string;
    category: MaterialCategory;
    manufacturer: string;
    batchNumber: string;
  };

  /** Certification level */
  level: CertificationLevel;

  /** Test suite */
  tests: CertificationTestResult[];

  /** Overall result */
  result: {
    passed: boolean;
    score: number; // 0-100
    grade: 'A' | 'B' | 'C' | 'F';
  };

  /** Certification status */
  status: MaterialStatus;

  /** Issue date */
  issueDate: Date;

  /** Expiration date */
  expirationDate: Date;

  /** Certifying authority */
  authority: string;

  /** Certificate number */
  certificateNumber: string;
}

// ============================================================================
// Manufacturing and Quality Control
// ============================================================================

/**
 * Manufacturing process types
 */
export type ManufacturingProcess =
  | 'vacuum-arc-melting'
  | 'quantum-annealing'
  | 'cvd'
  | 'mbe'
  | 'powder-metallurgy'
  | 'additive-manufacturing';

/**
 * Manufacturing specification
 */
export interface ManufacturingSpec {
  /** Process type */
  process: ManufacturingProcess;

  /** Process parameters */
  parameters: {
    [parameter: string]: {
      value: number;
      unit: string;
      tolerance: number;
    };
  };

  /** Quality control checkpoints */
  qualityCheckpoints: {
    stage: string;
    checks: string[];
    samplingRate: number; // 0-1 (fraction to inspect)
  }[];

  /** Environmental requirements */
  environment: {
    cleanroomClass?: string;
    temperature: { value: number; tolerance: number };
    humidity: { value: number; tolerance: number };
    vibration?: string;
  };

  /** Special requirements */
  specialRequirements?: string[];
}

/**
 * Quality control record
 */
export interface QualityControlRecord {
  /** Record ID */
  recordId: string;

  /** Material batch */
  batchNumber: string;

  /** Manufacturing date */
  manufacturingDate: Date;

  /** Process used */
  process: ManufacturingProcess;

  /** Inspector ID */
  inspectorId: string;

  /** Inspection results */
  inspections: {
    checkpoint: string;
    timestamp: Date;
    results: { [check: string]: boolean };
    measurements?: { [parameter: string]: number };
    passed: boolean;
  }[];

  /** Non-conformances */
  nonConformances: {
    description: string;
    severity: 'minor' | 'major' | 'critical';
    correctionAction: string;
    verified: boolean;
  }[];

  /** Final disposition */
  disposition: 'accept' | 'reject' | 'rework' | 'use-as-is';

  /** Traceability data */
  traceability: {
    rawMaterials: { [material: string]: string }; // material -> cert number
    equipment: string[];
    operators: string[];
  };
}

// ============================================================================
// Safety and Handling
// ============================================================================

/**
 * Hazard classifications
 */
export type HazardClass =
  | 'extreme-hazard' // Exotic matter
  | 'high-hazard' // Radioactive, highly toxic
  | 'moderate-hazard' // Temporal alloys, chrono-shields
  | 'low-hazard'; // Most quantum composites

/**
 * Personal protective equipment requirements
 */
export interface PPERequirements {
  /** Head protection */
  head?: 'hard-hat' | 'full-face-shield' | 'temporal-helmet';

  /** Eye protection */
  eyes?: 'safety-glasses' | 'goggles' | 'face-shield' | 'temporal-filter';

  /** Respiratory protection */
  respiratory?: 'dust-mask' | 'N95' | 'P100' | 'SCBA' | 'supplied-air';

  /** Hand protection */
  hands?: 'nitrile-gloves' | 'neoprene-gloves' | 'lead-gloves' | 'double-layer';

  /** Body protection */
  body?: 'lab-coat' | 'coveralls' | 'lead-apron' | 'temporal-suit';

  /** Foot protection */
  feet?: 'safety-shoes' | 'steel-toed-boots';

  /** Additional equipment */
  additional?: string[];
}

/**
 * Storage requirements
 */
export interface StorageRequirements {
  /** Storage temperature range (K) */
  temperature: {
    min: number;
    max: number;
  };

  /** Relative humidity range (%) */
  humidity?: {
    min: number;
    max: number;
  };

  /** Temporal shielding required */
  temporalShielding: boolean;

  /** Magnetic shielding required */
  magneticShielding?: boolean;

  /** Maximum storage duration (days) */
  maxStorageDuration?: number;

  /** Container specifications */
  container: {
    material: string;
    sealType: string;
    pressureRating?: number; // Pa
    vacuumRequired?: boolean;
  };

  /** Special handling notes */
  specialNotes?: string[];
}

/**
 * Emergency response procedure
 */
export interface EmergencyProcedure {
  /** Emergency type */
  type: 'containment-failure' | 'exposure' | 'fire' | 'spill' | 'other';

  /** Immediate actions (< 1 second) */
  immediateActions: string[];

  /** Short-term actions (< 30 seconds) */
  shortTermActions: string[];

  /** Response team actions (< 5 minutes) */
  responseActions: string[];

  /** Evacuation distance (meters) */
  evacuationDistance?: number;

  /** Emergency contacts */
  contacts: {
    role: string;
    phone: string;
    available: string; // "24/7", "business hours", etc.
  }[];

  /** Post-incident procedures */
  postIncident: string[];
}

/**
 * Material safety data
 */
export interface MaterialSafetyData {
  /** Material identification */
  material: {
    name: string;
    category: MaterialCategory;
    casNumber?: string;
  };

  /** Hazard classification */
  hazardClass: HazardClass;

  /** Specific hazards */
  hazards: {
    type: string;
    severity: 'low' | 'moderate' | 'high' | 'extreme';
    description: string;
  }[];

  /** PPE requirements */
  ppe: PPERequirements;

  /** Storage requirements */
  storage: StorageRequirements;

  /** Handling precautions */
  handling: string[];

  /** Emergency procedures */
  emergency: EmergencyProcedure[];

  /** First aid measures */
  firstAid: {
    exposure: string;
    measures: string[];
  }[];

  /** Disposal procedures */
  disposal: {
    method: string;
    requirements: string[];
    specialInstructions?: string[];
  };
}

// ============================================================================
// Material Database and Selection
// ============================================================================

/**
 * Material requirements specification
 */
export interface MaterialRequirements {
  /** Application name */
  application: string;

  /** Required temporal field strength (Tesla) */
  temporalFieldStrength: number;

  /** Operating duration (hours) */
  operatingDuration: number;

  /** Temperature range (K) */
  temperatureRange: {
    min: number;
    max: number;
  };

  /** Mechanical requirements */
  mechanical?: {
    tensileStrength?: number; // MPa
    hardness?: number; // HV
    elasticity?: number; // GPa
  };

  /** Quantum requirements */
  quantum?: {
    coherenceTime?: number; // seconds
    fidelity?: number; // 0-1
  };

  /** Shielding requirements */
  shielding?: {
    radiationBlocking?: number; // %
    fieldReduction?: number; // %
  };

  /** Budget constraints */
  budget?: {
    maxCostPerKg?: number;
    maxCostPerUnit?: number;
  };

  /** Certification requirements */
  certificationRequired?: CertificationLevel;

  /** Special requirements */
  special?: string[];
}

/**
 * Material selection result
 */
export interface MaterialSelectionResult {
  /** Recommended material */
  material: {
    name: string;
    category: MaterialCategory;
    type: string;
  };

  /** Suitability score (0-100) */
  suitability: number;

  /** Meets requirements */
  meetsRequirements: boolean;

  /** Strengths */
  strengths: string[];

  /** Weaknesses */
  weaknesses: string[];

  /** Cost estimate */
  costEstimate?: {
    perKg: number;
    perUnit: number;
    total: number;
  };

  /** Availability */
  availability: 'in-stock' | 'lead-time-short' | 'lead-time-long' | 'custom-order';

  /** Alternative materials */
  alternatives: {
    name: string;
    suitability: number;
    notes: string;
  }[];
}

/**
 * Material database entry
 */
export interface MaterialDatabaseEntry {
  /** Material identifier */
  id: string;

  /** Material name */
  name: string;

  /** Category */
  category: MaterialCategory;

  /** Type/grade */
  type: string;

  /** Configuration */
  config:
    | ExoticMatterConfig
    | TemporalAlloyConfig
    | ChronoShieldConfig
    | QuantumCompositeConfig;

  /** Current properties */
  properties:
    | ExoticMatterState
    | AlloyProperties
    | ShieldEffectiveness
    | QuantumStateProperties;

  /** Certification */
  certification?: MaterialCertification;

  /** Safety data */
  safetyData: MaterialSafetyData;

  /** Manufacturing spec */
  manufacturingSpec?: ManufacturingSpec;

  /** Quality records */
  qualityRecords: QualityControlRecord[];

  /** Test results */
  testResults:
    | AlloyTestResults[]
    | QuantumCompositeTestResults[]
    | CertificationTestResult[];

  /** Metadata */
  metadata: {
    created: Date;
    updated: Date;
    createdBy: string;
    updatedBy: string;
    notes?: string;
  };
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  /** Success flag */
  success: boolean;

  /** Response data */
  data?: T;

  /** Error information */
  error?: {
    code: string;
    message: string;
    details?: any;
  };

  /** Metadata */
  metadata?: {
    timestamp: Date;
    requestId: string;
    version: string;
  };
}

/**
 * Pagination information
 */
export interface PaginationInfo {
  /** Current page */
  page: number;

  /** Page size */
  pageSize: number;

  /** Total items */
  totalItems: number;

  /** Total pages */
  totalPages: number;

  /** Has next page */
  hasNext: boolean;

  /** Has previous page */
  hasPrevious: boolean;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Items */
  items: T[];

  /** Pagination info */
  pagination: PaginationInfo;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Measurement with uncertainty
 */
export interface MeasurementWithUncertainty {
  /** Measured value */
  value: number;

  /** Uncertainty (±) */
  uncertainty: number;

  /** Unit */
  unit: string;

  /** Confidence level (e.g., 0.95 for 95%) */
  confidence: number;
}

/**
 * Time series data point
 */
export interface TimeSeriesDataPoint<T = number> {
  /** Timestamp */
  timestamp: Date;

  /** Value */
  value: T;

  /** Quality indicator */
  quality?: 'good' | 'suspect' | 'bad';
}

/**
 * Traceability record
 */
export interface TraceabilityRecord {
  /** Item identifier */
  itemId: string;

  /** Parent item(s) */
  parents: string[];

  /** Process step */
  processStep: string;

  /** Timestamp */
  timestamp: Date;

  /** Operator */
  operator: string;

  /** Additional data */
  data?: { [key: string]: any };
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Re-export all types for convenience
  CertificationLevel,
  MaterialCategory,
  MaterialStatus,
  TestStatus,
  ExoticMatterProductionMethod,
  ConfinementType,
  TemporalAlloyType,
  ShieldingLayerMaterial,
  CoverageType,
  QuantumCompositeType,
  DegradationMechanism,
  DegradationModelType,
  CertificationTestType,
  ManufacturingProcess,
  HazardClass,
};
