/**
 * WIA-QUA-010: Holography - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum & Future Technology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Physics Types
// ============================================================================

/**
 * Two-dimensional vector for spatial coordinates
 */
export interface Vector2 {
  x: number;
  y: number;
}

/**
 * Three-dimensional vector for spatial coordinates
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Complex number representation
 */
export interface Complex {
  real: number;
  imaginary: number;
}

/**
 * Complex wave amplitude at a point
 */
export interface ComplexAmplitude {
  amplitude: number;
  phase: number; // radians
}

/**
 * Wave data representing electromagnetic field
 */
export interface WaveData {
  /** Amplitude distribution */
  amplitude: number[][];

  /** Phase distribution in radians */
  phase: number[][];

  /** Wavelength in meters */
  wavelength: number;

  /** Wave direction (optional) */
  direction?: Vector3;

  /** Polarization state (optional) */
  polarization?: 'linear' | 'circular' | 'elliptical';
}

// ============================================================================
// Recording Media
// ============================================================================

/**
 * Types of holographic recording media
 */
export type RecordingMedium =
  | 'silver-halide'
  | 'photopolymer'
  | 'photorefractive'
  | 'dichromated-gelatin'
  | 'photoresist'
  | 'digital-sensor';

/**
 * Recording medium properties
 */
export interface MediumProperties {
  /** Medium type */
  type: RecordingMedium;

  /** Sensitivity in J/cm² */
  sensitivity: number;

  /** Maximum resolution in lines/mm */
  resolution: number;

  /** Thickness in meters */
  thickness: number;

  /** Refractive index */
  refractiveIndex: number;

  /** Refractive index modulation (Δn) */
  indexModulation?: number;

  /** Shrinkage factor (0-1) */
  shrinkage: number;

  /** Processing required */
  processing: 'none' | 'chemical' | 'thermal' | 'optical';
}

// ============================================================================
// Hologram Types
// ============================================================================

/**
 * Types of holograms
 */
export type HologramType =
  | 'transmission'
  | 'reflection'
  | 'volume'
  | 'surface-relief'
  | 'rainbow'
  | 'computer-generated'
  | 'digital';

/**
 * Hologram recording geometry
 */
export interface RecordingGeometry {
  /** Reference beam angle in degrees */
  referenceAngle: number;

  /** Object beam angle in degrees */
  objectAngle: number;

  /** Distance from object to recording plane in meters */
  objectDistance: number;

  /** Beam ratio (reference:object) */
  beamRatio: number;

  /** Configuration type */
  configuration: 'in-line' | 'off-axis' | 'denisyuk';
}

/**
 * Complete hologram data
 */
export interface HologramData {
  /** Unique hologram identifier */
  id: string;

  /** Hologram type */
  type: HologramType;

  /** Recording wavelength in meters */
  wavelength: number;

  /** Interference pattern (intensity distribution) */
  interferencePattern: number[][];

  /** Dimensions */
  dimensions: {
    width: number;   // pixels or meters
    height: number;  // pixels or meters
    thickness?: number; // meters (for volume holograms)
  };

  /** Recording geometry */
  geometry: RecordingGeometry;

  /** Recording medium */
  medium: MediumProperties;

  /** Spatial frequency in lines/mm */
  spatialFrequency: number;

  /** Diffraction efficiency (0-1) */
  efficiency: number;

  /** Quality metrics */
  quality?: QualityMetrics;

  /** Creation timestamp */
  timestamp: Date;

  /** Metadata */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Recording and Reconstruction
// ============================================================================

/**
 * Parameters for hologram recording
 */
export interface RecordingParams {
  /** Recording wavelength in meters */
  wavelength: number;

  /** Object beam wave data */
  objectBeam: WaveData;

  /** Reference beam wave data */
  referenceBeam: WaveData;

  /** Recording medium */
  medium: RecordingMedium;

  /** Medium properties (optional, uses defaults if not provided) */
  mediumProperties?: MediumProperties;

  /** Exposure time in seconds */
  exposureTime?: number;

  /** Exposure energy in J/cm² */
  exposureEnergy?: number;

  /** Temperature in Celsius */
  temperature?: number;

  /** Recording geometry */
  geometry?: RecordingGeometry;
}

/**
 * Hologram recording result
 */
export interface RecordingResult {
  /** Recorded hologram data */
  hologram: HologramData;

  /** Recording success status */
  success: boolean;

  /** Warnings during recording */
  warnings: string[];

  /** Actual exposure received */
  actualExposure?: number;

  /** Processing steps applied */
  processing?: string[];
}

/**
 * Parameters for hologram reconstruction
 */
export interface ReconstructionParams {
  /** Hologram to reconstruct */
  hologram: HologramData;

  /** Reconstruction beam wave data */
  reconstructionBeam: WaveData;

  /** Reconstruction wavelength in meters */
  wavelength: number;

  /** Reconstruction distance in meters */
  distance?: number;

  /** Reconstruction method */
  method?: 'fresnel' | 'fraunhofer' | 'angular-spectrum' | 'rayleigh-sommerfeld';

  /** Output plane location */
  outputPlane?: number;
}

/**
 * Reconstructed wave field
 */
export interface ReconstructedWave {
  /** Amplitude distribution */
  amplitude: number[][];

  /** Phase distribution in radians */
  phase: number[][];

  /** Intensity distribution */
  intensity: number[][];

  /** Reconstruction quality */
  quality: QualityMetrics;

  /** Reconstruction method used */
  method: string;

  /** Computation time in milliseconds */
  computationTime: number;
}

// ============================================================================
// Computer-Generated Holography (CGH)
// ============================================================================

/**
 * 3D point in space
 */
export interface Point3D extends Vector3 {
  /** Point amplitude/brightness */
  amplitude?: number;

  /** Point color (wavelength in meters) */
  wavelength?: number;
}

/**
 * 3D scene representation
 */
export interface Scene3D {
  /** Scene identifier */
  id: string;

  /** Point cloud representation */
  points?: Point3D[];

  /** Polygon mesh (vertices and faces) */
  mesh?: {
    vertices: Vector3[];
    faces: number[][];
    normals?: Vector3[];
  };

  /** Layered representation */
  layers?: {
    depth: number;
    image: number[][];
  }[];

  /** Scene bounding box */
  bounds?: {
    min: Vector3;
    max: Vector3;
  };
}

/**
 * CGH computation methods
 */
export type CGHMethod =
  | 'point-source'
  | 'polygon-based'
  | 'layer-based'
  | 'fresnel'
  | 'fourier'
  | 'gerchberg-saxton'
  | 'iterative-fourier-transform';

/**
 * Parameters for CGH generation
 */
export interface CGHParams {
  /** 3D scene to encode */
  scene: Scene3D;

  /** Target wavelength in meters */
  wavelength: number;

  /** Hologram resolution */
  resolution: {
    width: number;
    height: number;
  };

  /** Computation method */
  method: CGHMethod;

  /** Number of iterations (for iterative methods) */
  iterations?: number;

  /** Pixel pitch in meters */
  pixelPitch?: number;

  /** Hologram-to-scene distance in meters */
  distance?: number;

  /** Phase-only or complex amplitude */
  phaseOnly?: boolean;
}

/**
 * Computed hologram result
 */
export interface ComputedHologram {
  /** Hologram identifier */
  id: string;

  /** Computed pattern (amplitude or intensity) */
  pattern: number[][];

  /** Phase distribution */
  phase: number[][];

  /** Computation time in milliseconds */
  computationTime: number;

  /** Method used */
  method: CGHMethod;

  /** Convergence (for iterative methods) */
  convergence?: {
    iterations: number;
    finalError: number;
    converged: boolean;
  };

  /** Quality metrics */
  quality?: QualityMetrics;
}

// ============================================================================
// Digital Holography
// ============================================================================

/**
 * Digital holography configuration
 */
export type DigitalHolographyMode =
  | 'off-axis'
  | 'inline'
  | 'phase-shifting'
  | 'dual-wavelength';

/**
 * Digital recording parameters
 */
export interface DigitalRecordingParams {
  /** Camera sensor properties */
  sensor: {
    width: number;      // pixels
    height: number;     // pixels
    pixelSize: number;  // meters
    bitDepth: number;   // bits
  };

  /** Recording mode */
  mode: DigitalHolographyMode;

  /** Number of phase shifts (for phase-shifting mode) */
  phaseShifts?: number;

  /** Reference beam angle for off-axis */
  referenceAngle?: number;

  /** Recording wavelength(s) in meters */
  wavelengths: number[];
}

/**
 * Digital hologram
 */
export interface DigitalHologram {
  /** Recorded intensity pattern(s) */
  intensities: number[][][]; // [frame][y][x]

  /** Recording parameters */
  recording: DigitalRecordingParams;

  /** Extracted complex amplitude (after processing) */
  complexAmplitude?: {
    amplitude: number[][];
    phase: number[][];
  };

  /** Timestamp */
  timestamp: Date;
}

/**
 * Numerical reconstruction parameters
 */
export interface NumericalReconstructionParams {
  /** Digital hologram to reconstruct */
  hologram: DigitalHologram;

  /** Reconstruction distances in meters */
  distances: number[];

  /** Reconstruction method */
  method: 'fresnel' | 'angular-spectrum' | 'convolution';

  /** Auto-focus enabled */
  autoFocus?: boolean;

  /** Phase unwrapping */
  phaseUnwrap?: boolean;
}

// ============================================================================
// Holographic Display
// ============================================================================

/**
 * Spatial Light Modulator (SLM) type
 */
export type SLMType = 'LCD' | 'LCOS' | 'DMD' | 'MEMS';

/**
 * SLM properties
 */
export interface SLMProperties {
  /** SLM type */
  type: SLMType;

  /** Resolution */
  resolution: {
    width: number;
    height: number;
  };

  /** Pixel pitch in meters */
  pixelPitch: number;

  /** Fill factor (0-1) */
  fillFactor: number;

  /** Refresh rate in Hz */
  refreshRate: number;

  /** Modulation type */
  modulation: 'phase' | 'amplitude' | 'complex';

  /** Bit depth */
  bitDepth: number;

  /** Maximum viewing angle in degrees */
  viewingAngle?: number;
}

/**
 * Holographic display configuration
 */
export interface HolographicDisplayConfig {
  /** SLM properties */
  slm: SLMProperties;

  /** Light source wavelengths in meters */
  wavelengths: number[];

  /** Color mode */
  colorMode: 'monochrome' | 'time-sequential' | 'spatial-multiplexing';

  /** Field of view in degrees */
  fieldOfView: number;

  /** Viewing distance in meters */
  viewingDistance: number;

  /** Eye box size in meters */
  eyeBoxSize?: number;
}

/**
 * Display frame data
 */
export interface DisplayFrame {
  /** Frame identifier */
  id: string;

  /** Hologram pattern(s) for SLM */
  patterns: number[][][]; // [wavelength/color][y][x]

  /** Target wavelengths */
  wavelengths: number[];

  /** Frame timestamp */
  timestamp: number;

  /** Rendering time in milliseconds */
  renderTime?: number;
}

// ============================================================================
// Holographic Data Storage
// ============================================================================

/**
 * Multiplexing method for data storage
 */
export type MultiplexingMethod =
  | 'angular'
  | 'wavelength'
  | 'phase-code'
  | 'shift'
  | 'peristrophic';

/**
 * Data page format
 */
export interface DataPage {
  /** Page identifier */
  id: string;

  /** User data bits */
  data: number[][];

  /** Error correction code bits */
  ecc: number[][];

  /** Sync and alignment markers */
  markers: number[][];

  /** Page dimensions */
  dimensions: {
    width: number;
    height: number;
  };

  /** Encoding scheme */
  encoding: 'binary' | 'gray-code' | 'rll';
}

/**
 * Holographic storage volume
 */
export interface StorageVolume {
  /** Volume identifier */
  id: string;

  /** Physical dimensions in meters */
  dimensions: Vector3;

  /** Recording medium */
  medium: MediumProperties;

  /** Multiplexing method(s) used */
  multiplexing: MultiplexingMethod[];

  /** Number of stored pages */
  pageCount: number;

  /** Total capacity in bits */
  capacity: number;

  /** Used capacity in bits */
  used: number;

  /** Page locations/addresses */
  pageMap: Map<string, StorageAddress>;
}

/**
 * Storage address for multiplexed hologram
 */
export interface StorageAddress {
  /** Spatial position */
  position: Vector3;

  /** Angular orientation */
  angle?: number;

  /** Wavelength */
  wavelength?: number;

  /** Phase code */
  phaseCode?: number[];
}

// ============================================================================
// Quality Metrics
// ============================================================================

/**
 * Quality metrics for holograms
 */
export interface QualityMetrics {
  /** Diffraction efficiency (0-1) */
  diffractionEfficiency?: number;

  /** Signal-to-noise ratio in dB */
  snr?: number;

  /** Spatial resolution in lines/mm */
  spatialResolution?: number;

  /** Uniformity (0-1) */
  uniformity?: number;

  /** Contrast */
  contrast?: number;

  /** Reconstruction fidelity (MSE or SSIM) */
  fidelity?: number;

  /** Dynamic range */
  dynamicRange?: number;

  /** Overall quality score (0-100) */
  overallScore?: number;
}

/**
 * Interference pattern analysis
 */
export interface InterferenceAnalysis {
  /** Fringe visibility (0-1) */
  visibility: number;

  /** Average fringe spacing in meters */
  fringeSpacing: number;

  /** Spatial frequency in lines/mm */
  spatialFrequency: number;

  /** Contrast */
  contrast: number;

  /** Phase stability */
  phaseStability?: number;
}

// ============================================================================
// Applications
// ============================================================================

/**
 * Security hologram features
 */
export interface SecurityFeatures {
  /** Visual authentication features */
  visual: {
    rainbowEffect: boolean;
    depth3D: boolean;
    microtext: boolean;
    hiddenImages: boolean;
    kinegram: boolean;
  };

  /** Machine-readable features */
  machineReadable?: {
    opticalSignature: number[];
    encryptedData: string;
    digitalWatermark: boolean;
  };

  /** Tamper evidence */
  tamperEvidence: {
    destructiveRemoval: boolean;
    voidPattern: boolean;
    serialNumber?: string;
  };
}

/**
 * Medical holography application
 */
export interface MedicalHologram {
  /** Application type */
  applicationType:
    | 'microscopy'
    | 'endoscopy'
    | 'tomography'
    | 'dental'
    | 'surgical-planning';

  /** Sample information */
  sample?: {
    type: string;
    preparation: string;
    staining?: string;
  };

  /** Imaging parameters */
  imaging: {
    magnification: number;
    numericalAperture?: number;
    depthOfField?: number;
    resolution: number;
  };

  /** Quantitative measurements */
  measurements?: {
    phase: number[][];
    refractiveIndex?: number[][];
    thickness?: number[][];
    volume?: number;
    dryMass?: number;
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for holography
 */
export const HOLOGRAPHY_CONSTANTS = {
  /** Speed of light in m/s */
  SPEED_OF_LIGHT: 299792458,

  /** Visible spectrum wavelengths in meters */
  WAVELENGTH: {
    VIOLET: 380e-9,
    BLUE: 450e-9,
    CYAN: 500e-9,
    GREEN: 532e-9,
    YELLOW: 580e-9,
    ORANGE: 610e-9,
    RED: 650e-9,
  },

  /** Common laser wavelengths in meters */
  LASER: {
    UV_405: 405e-9,
    BLUE_473: 473e-9,
    GREEN_532: 532e-9,
    RED_633: 633e-9,
    RED_660: 660e-9,
  },

  /** Typical spatial frequencies in lines/mm */
  SPATIAL_FREQUENCY: {
    LOW: 500,
    MEDIUM: 2000,
    HIGH: 5000,
    MAX: 10000,
  },

  /** Refractive indices */
  REFRACTIVE_INDEX: {
    AIR: 1.0,
    GLASS: 1.5,
    PHOTOPOLYMER: 1.5,
    SILVER_HALIDE: 1.6,
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
 * Holography simulation result
 */
export interface SimulationResult {
  /** Simulation identifier */
  id: string;

  /** Input parameters */
  parameters: RecordingParams | CGHParams | ReconstructionParams;

  /** Simulation output */
  output: HologramData | ComputedHologram | ReconstructedWave;

  /** Simulation duration in milliseconds */
  duration: number;

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;

  /** Warnings */
  warnings?: string[];
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-010 error codes
 */
export enum HolographyErrorCode {
  INSUFFICIENT_COHERENCE = 'H001',
  UNDEREXPOSURE = 'H002',
  OVEREXPOSURE = 'H003',
  POOR_FRINGE_VISIBILITY = 'H004',
  MEDIUM_DEGRADATION = 'H005',
  PHASE_UNWRAP_FAILURE = 'H006',
  COMPUTATION_TIMEOUT = 'H007',
  INVALID_WAVELENGTH = 'H008',
  RESOLUTION_MISMATCH = 'H009',
  INSUFFICIENT_MEMORY = 'H010',
}

/**
 * Holography error
 */
export class HolographyError extends Error {
  constructor(
    public code: HolographyErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'HolographyError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Vector2,
  Vector3,
  Complex,
  ComplexAmplitude,
  WaveData,

  // Media
  RecordingMedium,
  MediumProperties,

  // Holograms
  HologramType,
  RecordingGeometry,
  HologramData,

  // Recording/Reconstruction
  RecordingParams,
  RecordingResult,
  ReconstructionParams,
  ReconstructedWave,

  // CGH
  Point3D,
  Scene3D,
  CGHMethod,
  CGHParams,
  ComputedHologram,

  // Digital Holography
  DigitalHolographyMode,
  DigitalRecordingParams,
  DigitalHologram,
  NumericalReconstructionParams,

  // Display
  SLMType,
  SLMProperties,
  HolographicDisplayConfig,
  DisplayFrame,

  // Storage
  MultiplexingMethod,
  DataPage,
  StorageVolume,
  StorageAddress,

  // Quality
  QualityMetrics,
  InterferenceAnalysis,

  // Applications
  SecurityFeatures,
  MedicalHologram,

  // Simulation
  SimulationResult,
};

export { HOLOGRAPHY_CONSTANTS, HolographyErrorCode, HolographyError };

**弘익人間 (홍익인간) · Benefit All Humanity**
