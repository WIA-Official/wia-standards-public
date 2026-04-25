/**
 * WIA-QUA-020: Scientific Instrument - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Scientific Instrumentation Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional vector
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Measurement quality indicators
 */
export interface QualityMetrics {
  snr: number;
  validity: boolean;
  flags: string[];
  confidenceLevel?: number;
}

/**
 * Uncertainty budget
 */
export interface Uncertainty {
  statistical: number;
  systematic: number;
  combined: number;
  expandedUncertainty: number;
  coverageFactor: number;
}

/**
 * Sample information
 */
export interface Sample {
  id: string;
  name: string;
  description?: string;
  preparation?: string;
  properties?: Record<string, any>;
}

// ============================================================================
// Instrument Configuration
// ============================================================================

/**
 * Instrument types supported by this standard
 */
export type InstrumentType =
  | 'particle-accelerator'
  | 'mass-spectrometer'
  | 'electron-microscope'
  | 'xray-crystallography'
  | 'nmr-spectrometer'
  | 'gravitational-wave-detector'
  | 'telescope'
  | 'spectrophotometer'
  | 'chromatography'
  | 'calorimeter';

/**
 * Base instrument configuration
 */
export interface InstrumentConfig {
  instrumentType: InstrumentType;
  model: string;
  serialNumber: string;
  manufacturer: string;
  location?: string;
  calibrationDate: Date | string;
  firmwareVersion?: string;
  softwareVersion?: string;
}

/**
 * Instrument initialization parameters
 */
export interface InstrumentInit {
  config: InstrumentConfig;
  warmupTime?: number;
  selfTest?: boolean;
  autoCalibration?: boolean;
}

// ============================================================================
// Particle Accelerator Types
// ============================================================================

/**
 * Accelerator types
 */
export type AcceleratorType =
  | 'cyclotron'
  | 'synchrotron'
  | 'linear'
  | 'collider';

/**
 * Particle types
 */
export type ParticleType =
  | 'proton'
  | 'electron'
  | 'positron'
  | 'ion'
  | 'antiproton';

/**
 * Particle accelerator configuration
 */
export interface AcceleratorConfig extends InstrumentConfig {
  instrumentType: 'particle-accelerator';
  type: AcceleratorType;
  particleType: ParticleType;
  energy: number; // GeV
  beamCurrent: number; // mA
  circumference?: number; // meters
  rfFrequency: number; // MHz
  magneticField: number; // Tesla
  vacuumPressure: number; // Torr
  luminosity?: number; // cm⁻²s⁻¹
}

/**
 * Beam parameters
 */
export interface BeamParameters {
  energy: number; // GeV
  current: number; // mA
  emittance: {
    horizontal: number; // mm·mrad
    vertical: number; // mm·mrad
  };
  betaFunction: {
    horizontal: number; // m
    vertical: number; // m
  };
}

/**
 * Collision event
 */
export interface CollisionEvent {
  eventId: string;
  timestamp: Date | string;
  energy: number; // TeV
  luminosity: number; // cm⁻²s⁻¹
  particles: {
    detected: number;
    identified: Array<{
      type: string;
      energy: number;
      momentum: Vector3;
    }>;
  };
}

// ============================================================================
// Mass Spectrometer Types
// ============================================================================

/**
 * Mass spectrometer types
 */
export type MassSpectrometerType =
  | 'Orbitrap'
  | 'TOF'
  | 'quadrupole'
  | 'ion-trap'
  | 'FTICR';

/**
 * Ionization methods
 */
export type IonizationMethod =
  | 'ESI'
  | 'MALDI'
  | 'APCI'
  | 'EI'
  | 'CI'
  | 'DESI';

/**
 * Mass spectrometer configuration
 */
export interface MassSpectrometerConfig extends InstrumentConfig {
  instrumentType: 'mass-spectrometer';
  type: MassSpectrometerType;
  resolution: number; // m/Δm at FWHM
  massAccuracy: number; // ppm
  massRange: [number, number]; // [min, max] m/z
  ionization: IonizationMethod;
  polarity: 'positive' | 'negative' | 'both';
  scanRate: number; // Hz
  dynamicRange: number; // orders of magnitude
}

/**
 * Mass spectrum peak
 */
export interface MassPeak {
  mz: number;
  intensity: number;
  charge?: number;
  resolution?: number;
  signalToNoise?: number;
}

/**
 * Mass spectrum
 */
export interface MassSpectrum {
  spectrumId: string;
  timestamp: Date | string;
  msLevel: number;
  peaks: MassPeak[];
  basePeak: MassPeak;
  totalIonCurrent: number;
  scanRange: [number, number];
  retentionTime?: number;
  precursorMz?: number;
}

// ============================================================================
// Electron Microscope Types
// ============================================================================

/**
 * Electron microscope types
 */
export type ElectronMicroscopeType =
  | 'TEM'
  | 'SEM'
  | 'STEM'
  | 'cryo-EM';

/**
 * Imaging modes
 */
export type ImagingMode =
  | 'bright-field'
  | 'dark-field'
  | 'HAADF'
  | 'phase-contrast';

/**
 * Electron microscope configuration
 */
export interface ElectronMicroscopeConfig extends InstrumentConfig {
  instrumentType: 'electron-microscope';
  type: ElectronMicroscopeType;
  accelerationVoltage: number; // kV
  resolution: number; // nm
  magnification: [number, number]; // [min, max]
  detectors: Array<'SE' | 'BSE' | 'EDX' | 'EELS' | 'HAADF'>;
  vacuumLevel: number; // Torr
  cryogenicStage?: boolean;
  temperatureRange?: [number, number]; // Kelvin
}

/**
 * Microscope image metadata
 */
export interface MicroscopeImage {
  imageId: string;
  timestamp: Date | string;
  mode: ImagingMode;
  magnification: number;
  resolution: number; // nm
  accelerationVoltage: number; // kV
  pixelSize: number; // nm
  dimensions: {
    width: number;
    height: number;
  };
  defocus?: number; // nm
  exposureTime?: number; // ms
  data: string; // base64 encoded or file path
}

// ============================================================================
// X-ray Crystallography Types
// ============================================================================

/**
 * X-ray source types
 */
export type XRaySource =
  | 'Cu-Ka'
  | 'Mo-Ka'
  | 'synchrotron'
  | 'rotating-anode';

/**
 * X-ray crystallography configuration
 */
export interface XRayCrystallographyConfig extends InstrumentConfig {
  instrumentType: 'xray-crystallography';
  source: XRaySource;
  wavelength: number; // Angstroms
  detectorType: 'CCD' | 'pixel-array' | 'image-plate';
  resolution: number; // Angstroms
  temperatureControl: boolean;
  goniometer: {
    axes: number;
    accuracy: number; // degrees
  };
}

/**
 * Crystal structure
 */
export interface CrystalStructure {
  structureId: string;
  spaceGroup: string;
  unitCell: {
    a: number;
    b: number;
    c: number;
    alpha: number;
    beta: number;
    gamma: number;
  };
  resolution: number; // Angstroms
  rFactor: number;
  atoms: Array<{
    element: string;
    x: number;
    y: number;
    z: number;
    occupancy: number;
    bFactor: number;
  }>;
}

// ============================================================================
// NMR Spectrometer Types
// ============================================================================

/**
 * NMR nucleus types
 */
export type NMRNucleus = '1H' | '13C' | '15N' | '31P' | '19F' | '29Si';

/**
 * NMR pulse sequences
 */
export type PulseSequence =
  | '1D'
  | 'COSY'
  | 'TOCSY'
  | 'NOESY'
  | 'HSQC'
  | 'HMBC';

/**
 * NMR spectrometer configuration
 */
export interface NMRSpectrometerConfig extends InstrumentConfig {
  instrumentType: 'nmr-spectrometer';
  fieldStrength: number; // MHz (for 1H)
  magneticField: number; // Tesla
  probeType: string;
  temperatureRange: [number, number]; // Kelvin
  homogeneity: number; // ppb/cm³
  stability: number; // ppb/hour
}

/**
 * NMR spectrum
 */
export interface NMRSpectrum {
  spectrumId: string;
  timestamp: Date | string;
  nucleus: NMRNucleus;
  frequency: number; // MHz
  pulseSequence: PulseSequence;
  chemicalShifts: number[]; // ppm
  intensities: number[];
  peaks: Array<{
    shift: number; // ppm
    intensity: number;
    width: number; // Hz
    multiplicity?: string;
  }>;
}

// ============================================================================
// Gravitational Wave Detector Types
// ============================================================================

/**
 * Gravitational wave detector configuration
 */
export interface GravitationalWaveDetectorConfig extends InstrumentConfig {
  instrumentType: 'gravitational-wave-detector';
  armLength: number; // meters
  laserWavelength: number; // nm
  laserPower: number; // Watts
  sensitivity: number; // strain
  frequencyRange: [number, number]; // Hz
}

/**
 * Gravitational wave event
 */
export interface GravitationalWaveEvent {
  eventId: string;
  detectionTime: Date | string;
  strain: number;
  frequency: number; // Hz
  duration: number; // seconds
  sourceType: 'BBH' | 'BNS' | 'BHNS' | 'supernova';
  distance: number; // Mpc
  masses?: {
    m1: number; // solar masses
    m2: number;
  };
  confidence: number; // 0-1
}

// ============================================================================
// Telescope Types
// ============================================================================

/**
 * Telescope types
 */
export type TelescopeType =
  | 'optical'
  | 'radio'
  | 'infrared'
  | 'x-ray'
  | 'gamma-ray';

/**
 * Telescope configuration
 */
export interface TelescopeConfig extends InstrumentConfig {
  instrumentType: 'telescope';
  type: TelescopeType;
  aperture: number; // meters
  wavelengthRange: [number, number]; // meters
  focalLength: number; // meters
  fieldOfView: number; // arcminutes
  angularResolution: number; // arcseconds
  location: {
    latitude: number;
    longitude: number;
    altitude: number;
  };
}

/**
 * Astronomical observation
 */
export interface AstronomicalObservation {
  observationId: string;
  timestamp: Date | string;
  target: string;
  coordinates: {
    ra: number; // right ascension (degrees)
    dec: number; // declination (degrees)
  };
  exposureTime: number; // seconds
  wavelength: number; // nm
  magnitude?: number;
  flux?: number; // Jy
  data: string; // FITS file path or data
}

// ============================================================================
// Spectrophotometer Types
// ============================================================================

/**
 * Spectrophotometer types
 */
export type SpectrophotometerType =
  | 'UV-Vis'
  | 'IR'
  | 'Raman'
  | 'fluorescence'
  | 'atomic-absorption';

/**
 * Spectrophotometer configuration
 */
export interface SpectrophotometerConfig extends InstrumentConfig {
  instrumentType: 'spectrophotometer';
  type: SpectrophotometerType;
  wavelengthRange: [number, number]; // nm
  bandwidth: number; // nm
  photometricRange: [number, number]; // absorbance
  accuracy: number; // absorbance units
}

/**
 * Absorption spectrum
 */
export interface AbsorptionSpectrum {
  spectrumId: string;
  timestamp: Date | string;
  wavelengths: number[]; // nm
  absorbances: number[];
  transmittances?: number[];
  peaks: Array<{
    wavelength: number;
    absorbance: number;
    width: number;
  }>;
  sample: Sample;
}

// ============================================================================
// Chromatography Types
// ============================================================================

/**
 * Chromatography types
 */
export type ChromatographyType =
  | 'HPLC'
  | 'GC'
  | 'GC-MS'
  | 'LC-MS'
  | 'ion-chromatography';

/**
 * Chromatography configuration
 */
export interface ChromatographyConfig extends InstrumentConfig {
  instrumentType: 'chromatography';
  type: ChromatographyType;
  columnDimensions: {
    length: number; // mm
    diameter: number; // mm
    particleSize: number; // µm
  };
  maxPressure: number; // bar
  flowRate: [number, number]; // mL/min
  temperatureRange: [number, number]; // °C
  detectorType: 'UV' | 'FID' | 'MS' | 'ELSD' | 'RID';
}

/**
 * Chromatogram
 */
export interface Chromatogram {
  chromatogramId: string;
  timestamp: Date | string;
  retentionTimes: number[]; // minutes
  intensities: number[];
  peaks: Array<{
    retentionTime: number;
    area: number;
    height: number;
    width: number;
    compound?: string;
  }>;
  sample: Sample;
}

// ============================================================================
// Calorimeter Types
// ============================================================================

/**
 * Calorimeter types
 */
export type CalorimeterType =
  | 'DSC'
  | 'ITC'
  | 'bomb'
  | 'TGA'
  | 'DTA';

/**
 * Calorimeter configuration
 */
export interface CalorimeterConfig extends InstrumentConfig {
  instrumentType: 'calorimeter';
  type: CalorimeterType;
  temperatureRange: [number, number]; // °C
  heatingRate: [number, number]; // °C/min
  sensitivity: number; // µW
  sampleSize: [number, number]; // mg
}

/**
 * Calorimetry data
 */
export interface CalorimetryData {
  dataId: string;
  timestamp: Date | string;
  temperatures: number[]; // °C
  heatFlows: number[]; // W/g
  transitions: Array<{
    temperature: number;
    enthalpy: number; // J/g
    type: 'melting' | 'crystallization' | 'glass-transition' | 'decomposition';
  }>;
  sample: Sample;
}

// ============================================================================
// Measurement Request/Response
// ============================================================================

/**
 * Generic measurement request
 */
export interface MeasurementRequest {
  instrumentId: string;
  sample?: Sample;
  parameters?: Record<string, any>;
  metadata?: Record<string, any>;
}

/**
 * Generic measurement result
 */
export interface MeasurementResult {
  measurementId: string;
  instrumentId: string;
  instrumentType: InstrumentType;
  timestamp: Date | string;
  sample?: Sample;
  data: any;
  quality: QualityMetrics;
  uncertainty?: Uncertainty;
  metadata?: Record<string, any>;
}

// ============================================================================
// Calibration Types
// ============================================================================

/**
 * Calibration standard
 */
export interface CalibrationStandard {
  standardId: string;
  name: string;
  type: string;
  certifiedValues: Record<string, {
    value: number;
    uncertainty: number;
    unit: string;
  }>;
  certificateNumber: string;
  expirationDate: Date | string;
  traceability: string;
}

/**
 * Calibration procedure
 */
export interface CalibrationProcedure {
  procedureId: string;
  instrumentType: InstrumentType;
  method: string;
  standard: CalibrationStandard;
  steps: Array<{
    stepNumber: number;
    description: string;
    expectedValue?: number;
    tolerance?: number;
  }>;
}

/**
 * Calibration result
 */
export interface CalibrationResult {
  calibrationId: string;
  instrumentId: string;
  timestamp: Date | string;
  procedure: string;
  standard: string;
  results: Array<{
    parameter: string;
    measuredValue: number;
    referenceValue: number;
    deviation: number;
    pass: boolean;
  }>;
  uncertainty: Uncertainty;
  nextCalibrationDue: Date | string;
  certifiedBy?: string;
}

// ============================================================================
// Data Acquisition
// ============================================================================

/**
 * ADC configuration
 */
export interface ADCConfig {
  resolution: number; // bits
  samplingRate: number; // Hz
  inputRange: [number, number]; // V
  channels: number;
  bufferSize: number;
}

/**
 * Time series data
 */
export interface TimeSeriesData {
  dataId: string;
  instrumentId: string;
  startTime: Date | string;
  sampleRate: number; // Hz
  unit: string;
  data: number[];
  timestamps: string[];
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants used in scientific instruments
 */
export const PHYSICAL_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Planck constant (J·s) */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Reduced Planck constant (J·s) */
  REDUCED_PLANCK: 1.054571817e-34,

  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,

  /** Electron mass (kg) */
  ELECTRON_MASS: 9.1093837015e-31,

  /** Proton mass (kg) */
  PROTON_MASS: 1.67262192369e-27,

  /** Atomic mass unit (kg) */
  ATOMIC_MASS_UNIT: 1.66053906660e-27,

  /** Avogadro constant (mol⁻¹) */
  AVOGADRO: 6.02214076e23,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-020 error codes
 */
export enum InstrumentErrorCode {
  INSTRUMENT_NOT_INITIALIZED = 'SI001',
  CALIBRATION_EXPIRED = 'SI002',
  SAMPLE_PREPARATION_ERROR = 'SI003',
  MEASUREMENT_TIMEOUT = 'SI004',
  DATA_ACQUISITION_FAILED = 'SI005',
  HARDWARE_MALFUNCTION = 'SI006',
  INVALID_CONFIGURATION = 'SI007',
  OUT_OF_RANGE = 'SI008',
  QUALITY_CHECK_FAILED = 'SI009',
}

/**
 * Scientific instrument error
 */
export class InstrumentError extends Error {
  constructor(
    public code: InstrumentErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'InstrumentError';
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
// Export All Types
// ============================================================================

export type {
  // Core
  Vector3,
  QualityMetrics,
  Uncertainty,
  Sample,

  // Instrument config
  InstrumentType,
  InstrumentConfig,
  InstrumentInit,

  // Particle accelerators
  AcceleratorType,
  ParticleType,
  AcceleratorConfig,
  BeamParameters,
  CollisionEvent,

  // Mass spectrometers
  MassSpectrometerType,
  IonizationMethod,
  MassSpectrometerConfig,
  MassPeak,
  MassSpectrum,

  // Electron microscopes
  ElectronMicroscopeType,
  ImagingMode,
  ElectronMicroscopeConfig,
  MicroscopeImage,

  // X-ray crystallography
  XRaySource,
  XRayCrystallographyConfig,
  CrystalStructure,

  // NMR spectrometers
  NMRNucleus,
  PulseSequence,
  NMRSpectrometerConfig,
  NMRSpectrum,

  // Gravitational wave detectors
  GravitationalWaveDetectorConfig,
  GravitationalWaveEvent,

  // Telescopes
  TelescopeType,
  TelescopeConfig,
  AstronomicalObservation,

  // Spectrophotometers
  SpectrophotometerType,
  SpectrophotometerConfig,
  AbsorptionSpectrum,

  // Chromatography
  ChromatographyType,
  ChromatographyConfig,
  Chromatogram,

  // Calorimeters
  CalorimeterType,
  CalorimeterConfig,
  CalorimetryData,

  // Measurement
  MeasurementRequest,
  MeasurementResult,

  // Calibration
  CalibrationStandard,
  CalibrationProcedure,
  CalibrationResult,

  // Data acquisition
  ADCConfig,
  TimeSeriesData,
};

export {
  PHYSICAL_CONSTANTS,
  InstrumentErrorCode,
  InstrumentError,
};

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
