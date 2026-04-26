/**
 * WIA-QUA-009: Photonics - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Quantum Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Physics Types
// ============================================================================

/**
 * Wavelength units
 */
export type WavelengthUnit = 'nm' | 'um' | 'mm' | 'm' | 'Å';

/**
 * Energy units
 */
export type EnergyUnit = 'J' | 'eV' | 'meV' | 'keV' | 'MeV';

/**
 * Power units
 */
export type PowerUnit = 'W' | 'mW' | 'uW' | 'nW' | 'kW' | 'MW';

/**
 * Photon properties
 */
export interface Photon {
  /** Wavelength in meters */
  wavelength: number;

  /** Frequency in Hz */
  frequency: number;

  /** Energy in joules */
  energy: number;

  /** Energy in electronvolts */
  energyEV: number;

  /** Momentum in kg·m/s */
  momentum: number;

  /** Polarization state */
  polarization?: PolarizationState;

  /** Spin angular momentum (±ℏ) */
  spin?: number;
}

/**
 * Polarization states
 */
export type PolarizationType = 'linear' | 'circular' | 'elliptical' | 'unpolarized';

/**
 * Polarization state
 */
export interface PolarizationState {
  /** Type of polarization */
  type: PolarizationType;

  /** Angle in radians (for linear) */
  angle?: number;

  /** Handedness (for circular/elliptical) */
  handedness?: 'left' | 'right';

  /** Ellipticity (for elliptical) */
  ellipticity?: number;

  /** Degree of polarization (0-1) */
  degreeOfPolarization: number;
}

// ============================================================================
// Optical Materials
// ============================================================================

/**
 * Material optical properties
 */
export interface OpticalMaterial {
  /** Material name */
  name: string;

  /** Refractive index (wavelength-dependent) */
  refractiveIndex: (wavelength: number) => number;

  /** Extinction coefficient (absorption) */
  extinctionCoefficient?: (wavelength: number) => number;

  /** Dispersion coefficient */
  dispersion?: number;

  /** Nonlinear refractive index (n₂) */
  nonlinearIndex?: number;

  /** Bandgap energy in eV */
  bandgap?: number;

  /** Transparency range [min, max] in meters */
  transparencyRange?: [number, number];
}

/**
 * Sellmeier equation coefficients for refractive index
 */
export interface SellmeierCoefficients {
  B1: number;
  B2: number;
  B3: number;
  C1: number;
  C2: number;
  C3: number;
}

// ============================================================================
// Optical Components
// ============================================================================

/**
 * Optical fiber properties
 */
export interface OpticalFiber {
  /** Fiber identifier */
  id: string;

  /** Fiber type */
  type: 'single-mode' | 'multi-mode' | 'photonic-crystal' | 'hollow-core';

  /** Core radius in meters */
  coreRadius: number;

  /** Cladding radius in meters */
  claddingRadius: number;

  /** Core refractive index */
  coreIndex: number;

  /** Cladding refractive index */
  claddingIndex: number;

  /** Operating wavelength in meters */
  wavelength: number;

  /** Numerical aperture */
  numericalAperture: number;

  /** Mode field diameter in meters */
  modeFieldDiameter?: number;

  /** Attenuation in dB/km */
  attenuation: number;

  /** Dispersion in ps/(nm·km) */
  dispersion: number;

  /** Effective area in m² */
  effectiveArea?: number;

  /** Nonlinear coefficient in 1/(W·m) */
  nonlinearCoefficient?: number;
}

/**
 * Waveguide properties
 */
export interface Waveguide {
  /** Waveguide identifier */
  id: string;

  /** Waveguide type */
  type: 'planar' | 'ridge' | 'strip' | 'rib' | 'slot';

  /** Width in meters */
  width: number;

  /** Height in meters */
  height: number;

  /** Core material */
  coreMaterial: OpticalMaterial;

  /** Cladding material */
  claddingMaterial: OpticalMaterial;

  /** Effective refractive index */
  effectiveIndex: number;

  /** Group velocity in m/s */
  groupVelocity: number;

  /** Propagation loss in dB/cm */
  propagationLoss: number;

  /** Supported modes */
  modes: WaveguideMode[];
}

/**
 * Waveguide mode
 */
export interface WaveguideMode {
  /** Mode identifier (e.g., "TE0", "TM1") */
  id: string;

  /** Effective index */
  neff: number;

  /** Mode area in m² */
  modeArea: number;

  /** Confinement factor (0-1) */
  confinementFactor: number;
}

// ============================================================================
// Laser Systems
// ============================================================================

/**
 * Laser types
 */
export type LaserType =
  | 'diode'
  | 'solid-state'
  | 'gas'
  | 'fiber'
  | 'dye'
  | 'excimer'
  | 'free-electron';

/**
 * Laser operation modes
 */
export type LaserMode = 'CW' | 'pulsed' | 'Q-switched' | 'mode-locked';

/**
 * Laser system configuration
 */
export interface LaserSystem {
  /** Laser identifier */
  id: string;

  /** Laser type */
  type: LaserType;

  /** Gain medium */
  gainMedium: string;

  /** Operating wavelength(s) in meters */
  wavelength: number | number[];

  /** Output power in watts */
  power: number;

  /** Operation mode */
  mode: LaserMode;

  /** Beam quality (M²) */
  beamQuality?: number;

  /** Linewidth in Hz */
  linewidth?: number;

  /** Pulse duration in seconds (for pulsed) */
  pulseDuration?: number;

  /** Repetition rate in Hz (for pulsed) */
  repetitionRate?: number;

  /** Beam divergence in radians */
  divergence?: number;

  /** Wall-plug efficiency (0-1) */
  efficiency: number;

  /** Polarization */
  polarization?: PolarizationState;
}

/**
 * Laser cavity configuration
 */
export interface LaserCavity {
  /** Cavity length in meters */
  length: number;

  /** Mirror reflectivities [R1, R2] */
  reflectivities: [number, number];

  /** Round-trip loss */
  roundTripLoss: number;

  /** Cavity modes */
  modes: CavityMode[];
}

/**
 * Cavity mode
 */
export interface CavityMode {
  /** Mode number */
  n: number;

  /** Frequency in Hz */
  frequency: number;

  /** Q-factor */
  qFactor: number;
}

// ============================================================================
// Light Sources
// ============================================================================

/**
 * LED properties
 */
export interface LED {
  /** LED identifier */
  id: string;

  /** LED type */
  type: 'standard' | 'high-brightness' | 'OLED' | 'microLED' | 'quantum-dot';

  /** Peak wavelength in meters */
  peakWavelength: number;

  /** Spectral width (FWHM) in meters */
  spectralWidth: number;

  /** Forward voltage in volts */
  forwardVoltage: number;

  /** Forward current in amperes */
  forwardCurrent: number;

  /** Luminous flux in lumens */
  luminousFlux: number;

  /** Luminous efficacy in lm/W */
  efficacy: number;

  /** Color rendering index (CRI) */
  colorRenderingIndex?: number;

  /** Color temperature in Kelvin */
  colorTemperature?: number;

  /** Viewing angle in degrees */
  viewingAngle: number;

  /** Modulation bandwidth in Hz */
  bandwidth?: number;
}

// ============================================================================
// Photodetectors
// ============================================================================

/**
 * Photodetector types
 */
export type PhotodetectorType =
  | 'photodiode'
  | 'PIN'
  | 'APD'
  | 'photomultiplier'
  | 'CCD'
  | 'CMOS'
  | 'SPAD';

/**
 * Photodetector properties
 */
export interface Photodetector {
  /** Detector identifier */
  id: string;

  /** Detector type */
  type: PhotodetectorType;

  /** Active area in m² */
  activeArea: number;

  /** Responsivity in A/W */
  responsivity: number;

  /** Quantum efficiency (0-1) */
  quantumEfficiency: number;

  /** Dark current in amperes */
  darkCurrent: number;

  /** Noise equivalent power in W/√Hz */
  noisePower: number;

  /** Bandwidth in Hz */
  bandwidth: number;

  /** Spectral response range [min, max] in meters */
  spectralRange: [number, number];

  /** Gain (for APD/PMT) */
  gain?: number;

  /** Rise time in seconds */
  riseTime: number;
}

// ============================================================================
// Silicon Photonics
// ============================================================================

/**
 * Photonic integrated circuit
 */
export interface PhotonicIC {
  /** Circuit identifier */
  id: string;

  /** Technology node (e.g., "220nm SOI") */
  technology: string;

  /** Chip dimensions [width, height] in meters */
  dimensions: [number, number];

  /** Waveguides */
  waveguides: Waveguide[];

  /** Components */
  components: PhotonicComponent[];

  /** Operating wavelength range [min, max] */
  wavelengthRange: [number, number];

  /** Total insertion loss in dB */
  insertionLoss: number;

  /** Fabrication process */
  process: 'SOI' | 'SiN' | 'InP' | 'GaAs' | 'LiNbO3';
}

/**
 * Photonic component types
 */
export interface PhotonicComponent {
  /** Component identifier */
  id: string;

  /** Component type */
  type:
    | 'modulator'
    | 'coupler'
    | 'filter'
    | 'switch'
    | 'detector'
    | 'laser'
    | 'resonator'
    | 'grating';

  /** Position [x, y] on chip in meters */
  position: [number, number];

  /** Footprint [width, height] in meters */
  footprint: [number, number];

  /** Insertion loss in dB */
  insertionLoss: number;

  /** Bandwidth in Hz */
  bandwidth?: number;

  /** Extinction ratio in dB (for modulators) */
  extinctionRatio?: number;
}

// ============================================================================
// Quantum Photonics
// ============================================================================

/**
 * Single photon source
 */
export interface SinglePhotonSource {
  /** Source identifier */
  id: string;

  /** Source type */
  type: 'SPDC' | 'quantum-dot' | 'NV-center' | 'SFWM' | 'atom';

  /** Emission wavelength in meters */
  wavelength: number;

  /** Photon generation rate in Hz */
  generationRate: number;

  /** Single photon purity (g²(0)) */
  purity: number;

  /** Indistinguishability */
  indistinguishability: number;

  /** Collection efficiency (0-1) */
  collectionEfficiency: number;

  /** Spectral width in Hz */
  spectralWidth: number;

  /** Polarization */
  polarization: PolarizationState;
}

/**
 * Quantum state of light
 */
export interface QuantumLightState {
  /** Number of photons */
  photonNumber: number;

  /** Fock state coefficients */
  fockCoefficients?: number[];

  /** Coherent state amplitude */
  coherentAmplitude?: number;

  /** Squeezing parameter */
  squeezing?: number;

  /** Entanglement measure */
  entanglement?: number;

  /** Purity */
  purity: number;
}

// ============================================================================
// Nonlinear Optics
// ============================================================================

/**
 * Nonlinear optical processes
 */
export type NonlinearProcess =
  | 'SHG' // Second harmonic generation
  | 'THG' // Third harmonic generation
  | 'SFG' // Sum frequency generation
  | 'DFG' // Difference frequency generation
  | 'OPA' // Optical parametric amplification
  | 'FWM' // Four-wave mixing
  | 'SPM' // Self-phase modulation
  | 'XPM' // Cross-phase modulation
  | 'SRS' // Stimulated Raman scattering
  | 'SBS'; // Stimulated Brillouin scattering

/**
 * Nonlinear interaction
 */
export interface NonlinearInteraction {
  /** Process type */
  process: NonlinearProcess;

  /** Input wavelengths in meters */
  inputWavelengths: number[];

  /** Output wavelengths in meters */
  outputWavelengths: number[];

  /** Nonlinear medium */
  medium: OpticalMaterial;

  /** Interaction length in meters */
  length: number;

  /** Phase matching condition */
  phaseMatching: 'critical' | 'non-critical' | 'QPM' | 'none';

  /** Conversion efficiency (0-1) */
  efficiency: number;

  /** Input power in watts */
  inputPower: number;

  /** Output power in watts */
  outputPower: number;
}

// ============================================================================
// Optical Computing
// ============================================================================

/**
 * Optical logic gate
 */
export interface OpticalLogicGate {
  /** Gate type */
  type: 'AND' | 'OR' | 'NOT' | 'XOR' | 'NAND' | 'NOR';

  /** Number of inputs */
  inputs: number;

  /** Operating wavelength in meters */
  wavelength: number;

  /** Switching time in seconds */
  switchingTime: number;

  /** Energy per operation in joules */
  energyPerOperation: number;

  /** Contrast ratio (on/off) */
  contrastRatio: number;
}

/**
 * Optical interconnect
 */
export interface OpticalInterconnect {
  /** Interconnect identifier */
  id: string;

  /** Data rate in bits/s */
  dataRate: number;

  /** Link length in meters */
  length: number;

  /** Number of channels */
  channels: number;

  /** Modulation format */
  modulation: 'OOK' | 'PAM4' | 'QAM' | 'OFDM';

  /** Bit error rate */
  ber: number;

  /** Power consumption in watts */
  powerConsumption: number;
}

// ============================================================================
// LiDAR Systems
// ============================================================================

/**
 * LiDAR system configuration
 */
export interface LiDARSystem {
  /** System identifier */
  id: string;

  /** LiDAR type */
  type: 'scanning' | 'flash' | 'FMCW' | 'OPA';

  /** Operating wavelength in meters */
  wavelength: number;

  /** Laser pulse energy in joules */
  pulseEnergy: number;

  /** Pulse repetition rate in Hz */
  pulseRate: number;

  /** Maximum range in meters */
  maxRange: number;

  /** Range resolution in meters */
  rangeResolution: number;

  /** Angular resolution [horizontal, vertical] in radians */
  angularResolution: [number, number];

  /** Field of view [horizontal, vertical] in radians */
  fieldOfView: [number, number];

  /** Points per second */
  pointRate: number;

  /** Receiver sensitivity in watts */
  sensitivity: number;
}

/**
 * LiDAR point cloud data
 */
export interface LiDARPoint {
  /** 3D position [x, y, z] in meters */
  position: [number, number, number];

  /** Intensity (0-1) */
  intensity: number;

  /** Return number (for multiple returns) */
  returnNumber: number;

  /** Time of flight in seconds */
  timeOfFlight: number;
}

// ============================================================================
// Photonic Crystals
// ============================================================================

/**
 * Photonic crystal structure
 */
export interface PhotonicCrystal {
  /** Crystal identifier */
  id: string;

  /** Dimensionality */
  dimension: '1D' | '2D' | '3D';

  /** Lattice type */
  lattice: 'square' | 'triangular' | 'honeycomb' | 'fcc' | 'diamond';

  /** Lattice constant in meters */
  latticeConstant: number;

  /** Refractive index contrast */
  indexContrast: number;

  /** Band gap center wavelength in meters */
  bandGapCenter: number;

  /** Band gap width in meters */
  bandGapWidth: number;

  /** Defect modes */
  defectModes?: PhotonicDefect[];
}

/**
 * Photonic crystal defect
 */
export interface PhotonicDefect {
  /** Defect type */
  type: 'point' | 'line' | 'plane';

  /** Position in lattice units */
  position: [number, number, number];

  /** Resonant wavelength in meters */
  resonantWavelength: number;

  /** Quality factor */
  qFactor: number;

  /** Mode volume in (λ/n)³ */
  modeVolume: number;
}

// ============================================================================
// Photonics Calculations
// ============================================================================

/**
 * Photon energy calculation parameters
 */
export interface PhotonEnergyParams {
  /** Wavelength in meters OR frequency in Hz */
  wavelength?: number;
  frequency?: number;

  /** Number of photons (optional) */
  count?: number;

  /** Desired output unit */
  unit?: EnergyUnit;
}

/**
 * Photon energy calculation result
 */
export interface PhotonEnergyResult {
  /** Wavelength in meters */
  wavelength: number;

  /** Frequency in Hz */
  frequency: number;

  /** Energy per photon in joules */
  energy: number;

  /** Energy in requested unit */
  value: number;

  /** Unit */
  unit: EnergyUnit;

  /** Total power in watts (if count specified) */
  power?: number;

  /** Color (for visible wavelengths) */
  color?: string;
}

/**
 * Fiber design parameters
 */
export interface FiberDesignParams {
  /** Core radius in meters */
  coreRadius: number;

  /** Cladding radius in meters */
  claddingRadius: number;

  /** Core refractive index */
  coreIndex: number;

  /** Cladding refractive index */
  claddingIndex: number;

  /** Operating wavelength in meters */
  wavelength: number;

  /** Fiber type preference */
  preferredType?: 'single-mode' | 'multi-mode';
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants used in photonics calculations
 */
export const PHOTONICS_CONSTANTS = {
  /** Speed of light in vacuum (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Planck's constant (J·s) */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Reduced Planck's constant (J·s) */
  REDUCED_PLANCK: 1.054571817e-34,

  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,

  /** Vacuum permittivity (F/m) */
  VACUUM_PERMITTIVITY: 8.8541878128e-12,

  /** Vacuum permeability (H/m) */
  VACUUM_PERMEABILITY: 1.25663706212e-6,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,

  /** Impedance of free space (Ω) */
  FREE_SPACE_IMPEDANCE: 376.730313668,

  /** Electronvolt to joule conversion */
  EV_TO_JOULE: 1.602176634e-19,

  /** Visible wavelength range [min, max] in meters */
  VISIBLE_RANGE: [380e-9, 750e-9] as [number, number],

  /** Common telecom wavelengths (nm converted to m) */
  TELECOM_WAVELENGTHS: {
    O_BAND: 1310e-9,
    C_BAND: 1550e-9,
    L_BAND: 1625e-9,
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
 * Wavelength conversion result
 */
export interface WavelengthConversion {
  /** Original value */
  input: number;

  /** Input unit */
  inputUnit: WavelengthUnit;

  /** Converted value in meters */
  meters: number;

  /** Converted value in nanometers */
  nanometers: number;

  /** Frequency in Hz */
  frequency: number;

  /** Energy in eV */
  energyEV: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-QUA-009 error codes
 */
export enum PhotonicsErrorCode {
  INVALID_WAVELENGTH = 'P001',
  INVALID_MATERIAL = 'P002',
  INVALID_PARAMETERS = 'P003',
  CALCULATION_ERROR = 'P004',
  OUT_OF_RANGE = 'P005',
  UNSUPPORTED_OPERATION = 'P006',
  PHYSICAL_IMPOSSIBILITY = 'P007',
}

/**
 * Photonics error
 */
export class PhotonicsError extends Error {
  constructor(
    public code: PhotonicsErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'PhotonicsError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Photon,
  PolarizationState,
  OpticalMaterial,

  // Components
  OpticalFiber,
  Waveguide,
  WaveguideMode,

  // Lasers and sources
  LaserSystem,
  LaserCavity,
  LED,

  // Detectors
  Photodetector,

  // Silicon photonics
  PhotonicIC,
  PhotonicComponent,

  // Quantum
  SinglePhotonSource,
  QuantumLightState,

  // Nonlinear
  NonlinearInteraction,

  // Computing
  OpticalLogicGate,
  OpticalInterconnect,

  // LiDAR
  LiDARSystem,
  LiDARPoint,

  // Photonic crystals
  PhotonicCrystal,
  PhotonicDefect,

  // Calculations
  PhotonEnergyParams,
  PhotonEnergyResult,
  FiberDesignParams,
  WavelengthConversion,
};

export { PHOTONICS_CONSTANTS, PhotonicsErrorCode, PhotonicsError };

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
