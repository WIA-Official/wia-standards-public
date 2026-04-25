/**
 * WIA-DEF-009: Stealth Technology - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional vector for dimensions
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Platform dimensions
 */
export interface Dimensions {
  /** Length in meters */
  length: number;

  /** Width in meters */
  width: number;

  /** Height in meters */
  height: number;
}

/**
 * Platform types
 */
export type PlatformType = 'aircraft' | 'naval' | 'ground-vehicle' | 'uav' | 'missile' | 'satellite' | 'structure';

/**
 * Radar bands
 */
export type RadarBand = 'L-band' | 'S-band' | 'C-band' | 'X-band' | 'Ku-band' | 'Ka-band';

/**
 * Infrared bands
 */
export type IRBand = 'SWIR' | 'MWIR' | 'LWIR' | 'VLWIR';

/**
 * Polarization types
 */
export type Polarization = 'HH' | 'VV' | 'HV' | 'VH';

/**
 * Classification levels
 */
export type StealthClassification = 'very-low' | 'low' | 'moderate' | 'high' | 'very-high';

export type AcousticClassification = 'very-quiet' | 'quiet' | 'moderate' | 'loud' | 'very-loud';

// ============================================================================
// Radar Cross-Section (RCS)
// ============================================================================

/**
 * Target shapes for RCS calculation
 */
export type TargetShape = 'sphere' | 'flat-plate' | 'cylinder' | 'cone' | 'faceted' | 'complex';

/**
 * RCS calculation request
 */
export interface RCSRequest {
  /** Radar frequency in Hz */
  frequency: number;

  /** Target shape type */
  targetShape: TargetShape;

  /** Platform dimensions */
  dimensions: Dimensions;

  /** Total surface area in m² */
  surfaceArea: number;

  /** Is RAM coating applied? */
  ramCoating: boolean;

  /** Radar incidence angle in degrees (0 = head-on) */
  incidenceAngle: number;

  /** Radar polarization */
  polarization?: Polarization;

  /** Specific radar band */
  radarBand?: RadarBand;
}

/**
 * RCS calculation result
 */
export interface RCSResponse {
  /** RCS value in square meters */
  value: number;

  /** RCS in dBsm (decibel square meters) */
  dBsm: number;

  /** Wavelength used in calculation */
  wavelength: number;

  /** Classification of RCS level */
  classification: StealthClassification;

  /** Reduction factor compared to baseline */
  reductionFactor: number;

  /** Reduction in dB */
  reductionDB: number;

  /** RCS dependency on viewing aspect */
  aspectDependency: {
    /** Frontal aspect (0°) */
    frontal: number;
    /** Side aspect (90°) */
    side: number;
    /** Rear aspect (180°) */
    rear: number;
  };

  /** Frequency dependence region */
  region: 'rayleigh' | 'mie' | 'optical';

  /** Contributing factors */
  contributions?: {
    geometric: number;
    material: number;
    shaping: number;
  };
}

// ============================================================================
// Infrared Signature
// ============================================================================

/**
 * Cooling system types
 */
export type CoolingSystem = 'none' | 'passive' | 'active';

/**
 * IR signature calculation request
 */
export interface IRSignatureRequest {
  /** Surface temperature in Kelvin */
  surfaceTemp: number;

  /** Surface emissivity (0-1) */
  emissivity: number;

  /** Total surface area in m² */
  surfaceArea: number;

  /** Type of cooling system */
  coolingSystem?: CoolingSystem;

  /** Exhaust temperature in Kelvin (if applicable) */
  exhaustTemp?: number;

  /** Exhaust cross-section area in m² */
  exhaustArea?: number;

  /** Ambient temperature in Kelvin */
  ambientTemp?: number;

  /** Low-emissivity coating applied? */
  lowECoating?: boolean;
}

/**
 * IR signature calculation result
 */
export interface IRSignatureResponse {
  /** Total radiated power in watts */
  totalPower: number;

  /** Power in MWIR band (3-5 μm) */
  mwirPower: number;

  /** Power in LWIR band (8-12 μm) */
  lwirPower: number;

  /** Power in SWIR band (1-3 μm) */
  swirPower?: number;

  /** Detection range estimates */
  detectionRange: {
    /** MWIR detection range in meters */
    mwir: number;
    /** LWIR detection range in meters */
    lwir: number;
  };

  /** Classification of IR signature level */
  classification: StealthClassification;

  /** Temperature contrast with ambient */
  temperatureContrast: number;

  /** Emission by surface component */
  components?: {
    airframe: number;
    exhaust: number;
    engine: number;
  };
}

// ============================================================================
// Acoustic Signature
// ============================================================================

/**
 * Acoustic environment types
 */
export type AcousticEnvironment = 'air' | 'water' | 'ground';

/**
 * Acoustic signature calculation request
 */
export interface AcousticSignatureRequest {
  /** Acoustic source power in watts */
  sourcePower: number;

  /** Dominant frequency in Hz */
  frequency: number;

  /** Measurement distance in meters */
  distance: number;

  /** Applied dampening in dB */
  dampening?: number;

  /** Acoustic environment */
  environment: AcousticEnvironment;

  /** Background noise level in dB */
  backgroundNoise?: number;
}

/**
 * Acoustic signature calculation result
 */
export interface AcousticSignatureResponse {
  /** Sound power level in dB */
  soundPowerLevel: number;

  /** Sound pressure level in dB at specified distance */
  soundPressureLevel: number;

  /** Estimated detection range in meters */
  detectionRange: number;

  /** Classification of acoustic signature */
  classification: AcousticClassification;

  /** Signal-to-noise ratio */
  signalToNoise: number;

  /** Frequency analysis */
  frequencyAnalysis?: {
    lowFrequency: number;  // < 500 Hz
    midFrequency: number;  // 500-2000 Hz
    highFrequency: number; // > 2000 Hz
  };
}

// ============================================================================
// Visual Camouflage
// ============================================================================

/**
 * Visual camouflage types
 */
export type CamouflageType = 'solid' | 'disruptive' | 'adaptive' | 'digital' | 'multispectral';

/**
 * Visual signature request
 */
export interface VisualSignatureRequest {
  /** Target size (area) in m² */
  targetSize: number;

  /** Target luminance */
  luminance: number;

  /** Contrast with background (0-1) */
  contrast: number;

  /** Observer visibility threshold */
  visibilityThreshold?: number;

  /** Camouflage type applied */
  camouflageType?: CamouflageType;

  /** Atmospheric conditions */
  atmosphere?: {
    visibility: number;  // km
    humidity: number;    // percentage
    haze: boolean;
  };
}

/**
 * Visual signature response
 */
export interface VisualSignatureResponse {
  /** Detection range in meters */
  detectionRange: number;

  /** Recognition range in meters */
  recognitionRange: number;

  /** Identification range in meters */
  identificationRange: number;

  /** Effectiveness of camouflage (0-1) */
  camouflageEffectiveness: number;

  /** Color matching quality */
  colorMatching?: {
    deltaE: number;  // CIE Delta E
    match: 'excellent' | 'good' | 'fair' | 'poor';
  };
}

// ============================================================================
// RAM (Radar Absorbing Materials)
// ============================================================================

/**
 * RAM material types
 */
export type RAMMaterial = 'ferrite' | 'carbon-composite' | 'metamaterial' | 'nanostructured' | 'jaumann' | 'salisbury';

/**
 * RAM configuration
 */
export interface RAMConfiguration {
  /** Material type */
  material: RAMMaterial;

  /** Thickness in millimeters */
  thickness: number;

  /** Number of layers (for multi-layer absorbers) */
  layers?: number;

  /** Operating frequency range */
  frequencyRange: {
    min: number;  // Hz
    max: number;  // Hz
  };

  /** Expected absorption in dB */
  absorptionDB: number;

  /** Weight penalty in kg/m² */
  weightPenalty?: number;
}

/**
 * RAM performance metrics
 */
export interface RAMPerformance {
  /** Absorption efficiency percentage */
  absorptionEfficiency: number;

  /** Reflection coefficient magnitude */
  reflectionCoefficient: number;

  /** Bandwidth (frequency range) in GHz */
  bandwidth: number;

  /** Angular stability (performance vs angle) */
  angularStability: number;

  /** Environmental durability rating (0-1) */
  durability: number;
}

// ============================================================================
// Stealth Platform
// ============================================================================

/**
 * Stealth features configuration
 */
export interface StealthFeatures {
  /** RCS reduction features */
  rcsReduction: {
    /** Geometric shaping applied */
    shaping: boolean;
    /** RAM coating applied */
    ramCoating: boolean;
    /** Edge treatment type */
    edgeTreatment: 'straight' | 'serrated' | 'aligned';
    /** Internal weapons bay */
    internalWeapons: boolean;
  };

  /** IR suppression features */
  irSuppression: {
    /** Exhaust cooling system */
    exhaustCooling: boolean;
    /** Low-emissivity coating */
    lowEmissivityCoating: boolean;
    /** Serpentine exhaust ducts */
    serpentineDucts: boolean;
  };

  /** Acoustic dampening features */
  acousticDampening: {
    /** Engine insulation */
    engineInsulation: boolean;
    /** Airframe acoustic treatment */
    airframeTreatment: boolean;
    /** Active noise cancellation */
    activeNoiseCancellation?: boolean;
  };

  /** Visual camouflage features */
  visualCamouflage: {
    /** Matte finish */
    matteFinish: boolean;
    /** Disruptive pattern */
    disruptivePattern: boolean;
    /** Adaptive displays */
    adaptiveDisplays?: boolean;
  };
}

/**
 * Stealth platform specification
 */
export interface StealthPlatform {
  /** Platform identifier */
  id: string;

  /** Platform name */
  name?: string;

  /** Platform type */
  type: PlatformType;

  /** Platform dimensions */
  dimensions: Dimensions;

  /** Total surface area */
  surfaceArea: number;

  /** Operating mass in kg */
  mass: number;

  /** Stealth features implemented */
  stealthFeatures: StealthFeatures;

  /** Design requirements */
  requirements?: {
    maxRCS: number;          // m²
    maxIRSignature: number;  // watts
    maxAcoustic: number;     // dB
    minVisualRange: number;  // meters
  };
}

/**
 * Comprehensive stealth performance
 */
export interface StealthPerformance {
  /** Platform being evaluated */
  platform: StealthPlatform;

  /** RCS performance */
  rcs: RCSResponse;

  /** IR signature performance */
  irSignature: IRSignatureResponse;

  /** Acoustic signature performance */
  acousticSignature: AcousticSignatureResponse;

  /** Visual signature performance */
  visualSignature: VisualSignatureResponse;

  /** Overall stealth rating (0-10) */
  overallRating: number;

  /** Multi-spectrum effectiveness */
  multiSpectrumScore: {
    radar: number;     // 0-1
    infrared: number;  // 0-1
    acoustic: number;  // 0-1
    visual: number;    // 0-1
    combined: number;  // 0-1
  };

  /** Survivability enhancement factor */
  survivabilityFactor: number;

  /** Recommendations for improvement */
  recommendations: string[];
}

// ============================================================================
// Threat Environment
// ============================================================================

/**
 * Threat radar system
 */
export interface ThreatRadar {
  /** Radar designation */
  name: string;

  /** Operating frequency band */
  band: RadarBand;

  /** Specific frequency in Hz */
  frequency: number;

  /** Detection range for 1m² target */
  detectionRange: number;

  /** Tracking capability */
  tracking: boolean;
}

/**
 * Threat IR system
 */
export interface ThreatIR {
  /** Sensor designation */
  name: string;

  /** Operating band */
  band: IRBand;

  /** Sensitivity in watts */
  sensitivity: number;

  /** Detection range for reference signature */
  detectionRange: number;
}

/**
 * Threat environment specification
 */
export interface ThreatEnvironment {
  /** Threat radars present */
  radarBands: RadarBand[];

  /** Specific radar systems */
  radars?: ThreatRadar[];

  /** IR detector types */
  irDetectors: IRBand[];

  /** Specific IR systems */
  irSystems?: ThreatIR[];

  /** Acoustic sensors present */
  acousticSensors: boolean;

  /** Visual/optical sensors */
  visualSensors: boolean;

  /** Threat severity (0-1) */
  severity: number;
}

// ============================================================================
// Design Optimization
// ============================================================================

/**
 * Design optimization parameters
 */
export interface OptimizationParams {
  /** Platform to optimize */
  platform: StealthPlatform;

  /** Threat environment to optimize against */
  threatEnvironment: ThreatEnvironment;

  /** Optimization priorities (weights sum to 1) */
  priorities: {
    rcs: number;
    infrared: number;
    acoustic: number;
    visual: number;
  };

  /** Constraints */
  constraints?: {
    maxWeight: number;      // kg
    maxCost: number;        // relative units
    maxComplexity: number;  // 0-1
  };
}

/**
 * Optimization result
 */
export interface OptimizationResult {
  /** Original performance */
  baseline: StealthPerformance;

  /** Optimized performance */
  optimized: StealthPerformance;

  /** Improvements achieved */
  improvements: {
    rcsReduction: number;   // dB
    irReduction: number;    // percentage
    acousticReduction: number; // dB
    visualImprovement: number; // percentage
  };

  /** Recommended modifications */
  modifications: string[];

  /** Cost-benefit analysis */
  costBenefit?: {
    performanceGain: number;
    weightPenalty: number;
    costIncrease: number;
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for stealth calculations
 */
export const STEALTH_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Stefan-Boltzmann constant (W/m²K⁴) */
  STEFAN_BOLTZMANN: 5.67e-8,

  /** Acoustic reference power (watts) */
  ACOUSTIC_REF_POWER: 1e-12,

  /** Permittivity of vacuum (F/m) */
  VACUUM_PERMITTIVITY: 8.854e-12,

  /** Permeability of vacuum (H/m) */
  VACUUM_PERMEABILITY: 1.257e-6,

  /** Standard atmospheric pressure (Pa) */
  ATMOSPHERIC_PRESSURE: 101325,

  /** Radar band frequencies (Hz) */
  RADAR_BANDS: {
    'L-band': { min: 1e9, max: 2e9 },
    'S-band': { min: 2e9, max: 4e9 },
    'C-band': { min: 4e9, max: 8e9 },
    'X-band': { min: 8e9, max: 12e9 },
    'Ku-band': { min: 12e9, max: 18e9 },
    'Ka-band': { min: 27e9, max: 40e9 },
  },

  /** IR band wavelengths (micrometers) */
  IR_BANDS: {
    'SWIR': { min: 1, max: 3 },
    'MWIR': { min: 3, max: 5 },
    'LWIR': { min: 8, max: 12 },
    'VLWIR': { min: 12, max: 20 },
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

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-009 error codes
 */
export enum StealthErrorCode {
  INVALID_FREQUENCY = 'S001',
  TEMPERATURE_OUT_OF_BOUNDS = 'S002',
  IMPOSSIBLE_GEOMETRY = 'S003',
  MATERIAL_INCOMPATIBILITY = 'S004',
  CALCULATION_OVERFLOW = 'S005',
  INSUFFICIENT_DATA = 'S006',
  INVALID_PARAMETERS = 'S007',
}

/**
 * Stealth technology error
 */
export class StealthTechError extends Error {
  constructor(
    public code: StealthErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'StealthTechError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Dimensions,
  PlatformType,
  RadarBand,
  IRBand,
  Polarization,
  StealthClassification,
  AcousticClassification,

  // RCS
  TargetShape,
  RCSRequest,
  RCSResponse,

  // IR
  CoolingSystem,
  IRSignatureRequest,
  IRSignatureResponse,

  // Acoustic
  AcousticEnvironment,
  AcousticSignatureRequest,
  AcousticSignatureResponse,

  // Visual
  CamouflageType,
  VisualSignatureRequest,
  VisualSignatureResponse,

  // RAM
  RAMMaterial,
  RAMConfiguration,
  RAMPerformance,

  // Platform
  StealthFeatures,
  StealthPlatform,
  StealthPerformance,

  // Threat
  ThreatRadar,
  ThreatIR,
  ThreatEnvironment,

  // Optimization
  OptimizationParams,
  OptimizationResult,
};

export { STEALTH_CONSTANTS, StealthErrorCode, StealthTechError };
