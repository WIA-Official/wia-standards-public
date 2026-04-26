/**
 * WIA-COMM-007: Optical Communication - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Wavelength units
 */
export type WavelengthUnit = 'nm' | 'um' | 'mm' | 'm';

/**
 * Power units
 */
export type PowerUnit = 'dBm' | 'mW' | 'W';

/**
 * Frequency units
 */
export type FrequencyUnit = 'Hz' | 'MHz' | 'GHz' | 'THz';

/**
 * Data rate units
 */
export type DataRateUnit = 'bps' | 'kbps' | 'Mbps' | 'Gbps' | 'Tbps';

// ============================================================================
// Fiber Optics
// ============================================================================

/**
 * Fiber types
 */
export type FiberType =
  | 'SMF' // Single-Mode Fiber
  | 'MMF' // Multi-Mode Fiber
  | 'DSF' // Dispersion-Shifted Fiber
  | 'NZDSF' // Non-Zero Dispersion-Shifted Fiber
  | 'DCF' // Dispersion Compensating Fiber
  | 'PCF'; // Photonic Crystal Fiber

/**
 * Multi-mode fiber types
 */
export type MMFType = 'OM1' | 'OM2' | 'OM3' | 'OM4' | 'OM5';

/**
 * Optical fiber properties
 */
export interface OpticalFiber {
  /** Fiber identifier */
  id: string;

  /** Fiber type */
  type: FiberType;

  /** Core diameter in meters */
  coreDiameter: number;

  /** Cladding diameter in meters */
  claddingDiameter: number;

  /** Core refractive index */
  coreIndex: number;

  /** Cladding refractive index */
  claddingIndex: number;

  /** Operating wavelength in meters */
  wavelength: number;

  /** Attenuation in dB/km */
  attenuation: number;

  /** Dispersion in ps/(nm·km) */
  dispersion: number;

  /** Numerical aperture */
  numericalAperture: number;

  /** Mode field diameter in meters (for SMF) */
  modeFieldDiameter?: number;

  /** Effective area in m² */
  effectiveArea?: number;

  /** Nonlinear coefficient in 1/(W·m) */
  nonlinearCoefficient?: number;

  /** PMD coefficient in ps/√km */
  pmdCoefficient?: number;
}

// ============================================================================
// Wavelength Division Multiplexing
// ============================================================================

/**
 * WDM types
 */
export type WDMType = 'CWDM' | 'DWDM' | 'LWDM';

/**
 * ITU-T DWDM grid spacing
 */
export type DWDMSpacing = '12.5GHz' | '25GHz' | '50GHz' | '100GHz';

/**
 * Wavelength bands
 */
export type WavelengthBand = 'O' | 'E' | 'S' | 'C' | 'L' | 'U';

/**
 * WDM channel
 */
export interface WDMChannel {
  /** Channel number */
  channelNumber: number;

  /** Center frequency in Hz */
  frequency: number;

  /** Center wavelength in meters */
  wavelength: number;

  /** Optical power in dBm */
  power: number;

  /** OSNR in dB (0.1 nm bandwidth) */
  osnr?: number;

  /** Data rate in bps */
  dataRate: number;

  /** Modulation format */
  modulation: ModulationFormat;
}

/**
 * DWDM system configuration
 */
export interface DWDMSystem {
  /** System identifier */
  id: string;

  /** WDM type */
  type: WDMType;

  /** Channel spacing */
  spacing: DWDMSpacing;

  /** Number of channels */
  channels: number;

  /** Start frequency in Hz */
  startFrequency: number;

  /** Channel list */
  channelList: WDMChannel[];

  /** Total capacity in bps */
  totalCapacity: number;

  /** Wavelength band */
  band: WavelengthBand;

  /** Fiber type */
  fiberType: FiberType;
}

// ============================================================================
// Optical Amplifiers
// ============================================================================

/**
 * Amplifier types
 */
export type AmplifierType = 'EDFA' | 'Raman' | 'SOA' | 'Hybrid';

/**
 * Optical amplifier
 */
export interface OpticalAmplifier {
  /** Amplifier identifier */
  id: string;

  /** Amplifier type */
  type: AmplifierType;

  /** Position along fiber (meters) */
  position: number;

  /** Small-signal gain in dB */
  gain: number;

  /** Noise figure in dB */
  noiseFigure: number;

  /** Saturation output power in dBm */
  saturationPower: number;

  /** Operating wavelength range [min, max] in meters */
  wavelengthRange: [number, number];

  /** Input power in dBm */
  inputPower?: number;

  /** Output power in dBm */
  outputPower?: number;

  /** ASE power in dBm */
  asePower?: number;
}

// ============================================================================
// Modulation Formats
// ============================================================================

/**
 * Modulation formats
 */
export type ModulationFormat =
  | 'OOK' // On-Off Keying
  | 'DPSK' // Differential Phase-Shift Keying
  | 'DQPSK' // Differential QPSK
  | 'DP-QPSK' // Dual-Polarization QPSK
  | 'DP-8QAM' // Dual-Polarization 8QAM
  | 'DP-16QAM' // Dual-Polarization 16QAM
  | 'DP-64QAM' // Dual-Polarization 64QAM
  | 'PAM4' // 4-level Pulse Amplitude Modulation
  | 'PAM8'; // 8-level Pulse Amplitude Modulation

/**
 * Modulation scheme properties
 */
export interface ModulationScheme {
  /** Format name */
  format: ModulationFormat;

  /** Bits per symbol */
  bitsPerSymbol: number;

  /** Spectral efficiency in bit/s/Hz */
  spectralEfficiency: number;

  /** Required OSNR in dB (for BER = 10⁻³) */
  requiredOSNR: number;

  /** Chromatic dispersion tolerance in ps/nm */
  dispersionTolerance: number;

  /** Polarization multiplexing */
  dualPolarization: boolean;
}

// ============================================================================
// Coherent Optical Transmission
// ============================================================================

/**
 * Coherent receiver configuration
 */
export interface CoherentReceiver {
  /** Receiver identifier */
  id: string;

  /** Modulation format */
  modulation: ModulationFormat;

  /** Symbol rate in baud */
  symbolRate: number;

  /** Local oscillator power in dBm */
  loPower: number;

  /** ADC resolution in bits */
  adcResolution: number;

  /** ADC sampling rate in samples/s */
  samplingRate: number;

  /** DSP enabled */
  dspEnabled: boolean;

  /** FEC type */
  fecType: FECType;

  /** FEC overhead percentage */
  fecOverhead: number;
}

/**
 * Digital signal processing parameters
 */
export interface DSPParameters {
  /** Chromatic dispersion compensation enabled */
  cdCompensation: boolean;

  /** Polarization demultiplexing */
  polarizationDemux: boolean;

  /** Carrier phase recovery */
  carrierRecovery: boolean;

  /** Timing recovery */
  timingRecovery: boolean;

  /** Adaptive equalization */
  adaptiveEQ: boolean;

  /** Number of taps */
  taps?: number;
}

// ============================================================================
// Optical Transceivers
// ============================================================================

/**
 * Transceiver form factors
 */
export type TransceiverFormFactor =
  | 'SFP'
  | 'SFP+'
  | 'SFP28'
  | 'SFP56'
  | 'QSFP'
  | 'QSFP+'
  | 'QSFP28'
  | 'QSFP56'
  | 'QSFP-DD'
  | 'QSFP-DD800'
  | 'CFP'
  | 'CFP2'
  | 'CFP4'
  | 'CFP8';

/**
 * Optical transceiver
 */
export interface OpticalTransceiver {
  /** Transceiver identifier */
  id: string;

  /** Form factor */
  formFactor: TransceiverFormFactor;

  /** Data rate in bps */
  dataRate: number;

  /** Operating wavelength(s) in meters */
  wavelength: number | number[];

  /** TX power in dBm */
  txPower: number;

  /** RX sensitivity in dBm */
  rxSensitivity: number;

  /** Maximum reach in meters */
  maxReach: number;

  /** Fiber type */
  fiberType: FiberType;

  /** Number of lanes */
  lanes: number;

  /** Modulation format */
  modulation: ModulationFormat;

  /** Power consumption in watts */
  powerConsumption: number;

  /** Operating temperature range [min, max] in Celsius */
  temperatureRange: [number, number];
}

// ============================================================================
// Link Budget
// ============================================================================

/**
 * Link budget calculation parameters
 */
export interface LinkBudgetParams {
  /** TX power in dBm */
  txPower: number;

  /** RX sensitivity in dBm */
  rxSensitivity?: number;

  /** Fiber length in meters */
  fiberLength: number;

  /** Fiber type */
  fiberType: FiberType;

  /** Wavelength in meters */
  wavelength: number;

  /** Fiber attenuation in dB/km */
  fiberLoss?: number;

  /** Number of connectors */
  connectors?: number;

  /** Connector loss in dB */
  connectorLoss?: number;

  /** Number of splices */
  splices?: number;

  /** Splice loss in dB */
  spliceLoss?: number;

  /** Optical amplifiers */
  amplifiers?: OpticalAmplifier[];

  /** Required margin in dB */
  requiredMargin?: number;
}

/**
 * Link budget calculation result
 */
export interface LinkBudgetResult {
  /** Transmitted power in dBm */
  txPower: number;

  /** Received power in dBm */
  rxPower: number;

  /** Total fiber loss in dB */
  fiberLoss: number;

  /** Total connector loss in dB */
  connectorLoss: number;

  /** Total splice loss in dB */
  spliceLoss: number;

  /** Total amplifier gain in dB */
  amplifierGain: number;

  /** Total path loss in dB */
  totalLoss: number;

  /** Link margin in dB */
  margin: number;

  /** OSNR in dB (if amplifiers present) */
  osnr?: number;

  /** Link status */
  status: 'pass' | 'fail' | 'marginal';

  /** Distance in km */
  distanceKm: number;
}

// ============================================================================
// Dispersion
// ============================================================================

/**
 * Dispersion parameters
 */
export interface DispersionParams {
  /** Fiber length in meters */
  fiberLength: number;

  /** Fiber type */
  fiberType: FiberType;

  /** Wavelength in meters */
  wavelength: number;

  /** Dispersion coefficient in ps/(nm·km) */
  dispersionCoeff?: number;

  /** Signal spectral width in meters */
  spectralWidth: number;

  /** Data rate in bps */
  dataRate: number;
}

/**
 * Dispersion calculation result
 */
export interface DispersionResult {
  /** Chromatic dispersion in ps/nm */
  chromaticDispersion: number;

  /** Pulse broadening in ps */
  pulseBroadening: number;

  /** Dispersion penalty in dB */
  penalty: number;

  /** Compensation required */
  compensationRequired: boolean;

  /** Recommended DCF length in meters */
  dcfLength?: number;

  /** PMD in ps */
  pmd?: number;
}

// ============================================================================
// Forward Error Correction
// ============================================================================

/**
 * FEC types
 */
export type FECType =
  | 'none'
  | 'RS(255,239)'
  | 'RS(528,514)'
  | 'SD-FEC'
  | 'HD-FEC'
  | 'LDPC'
  | 'OFEC';

/**
 * FEC properties
 */
export interface FECScheme {
  /** FEC type */
  type: FECType;

  /** Overhead percentage */
  overhead: number;

  /** Net coding gain in dB */
  netCodingGain: number;

  /** Pre-FEC BER threshold */
  preFecBer: number;

  /** Post-FEC BER target */
  postFecBer: number;

  /** Latency in μs */
  latency: number;
}

// ============================================================================
// Optical Performance Monitoring
// ============================================================================

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
  /** Optical power in dBm */
  opticalPower: number;

  /** OSNR in dB (0.1 nm bandwidth) */
  osnr: number;

  /** BER (bit error rate) */
  ber: number;

  /** Q-factor in dB */
  qFactor: number;

  /** Chromatic dispersion in ps/nm */
  chromaticDispersion: number;

  /** PMD in ps */
  pmd: number;

  /** Wavelength in meters */
  wavelength: number;

  /** Wavelength accuracy in meters */
  wavelengthAccuracy: number;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// ROADM and Optical Switching
// ============================================================================

/**
 * ROADM degree
 */
export interface ROADMDegree {
  /** Degree number */
  degreeNumber: number;

  /** Direction/port name */
  direction: string;

  /** Number of wavelengths */
  wavelengths: number;

  /** Add/drop channels */
  addDropChannels: number[];

  /** Insertion loss in dB */
  insertionLoss: number;
}

/**
 * ROADM node
 */
export interface ROADMNode {
  /** Node identifier */
  id: string;

  /** Node name */
  name: string;

  /** Number of degrees */
  degrees: number;

  /** Degree configurations */
  degreeConfig: ROADMDegree[];

  /** Wavelength selective switch (WSS) */
  wss: boolean;

  /** Colorless */
  colorless: boolean;

  /** Directionless */
  directionless: boolean;

  /** Contentionless */
  contentionless: boolean;

  /** Maximum add/drop capacity */
  maxAddDrop: number;
}

// ============================================================================
// Free-Space Optical Communication
// ============================================================================

/**
 * FSO link parameters
 */
export interface FSOLink {
  /** Link identifier */
  id: string;

  /** Distance in meters */
  distance: number;

  /** TX power in dBm */
  txPower: number;

  /** TX aperture diameter in meters */
  txAperture: number;

  /** RX aperture diameter in meters */
  rxAperture: number;

  /** Wavelength in meters */
  wavelength: number;

  /** Beam divergence in radians */
  beamDivergence: number;

  /** Atmospheric visibility in km */
  visibility: number;

  /** Atmospheric attenuation in dB/km */
  atmosphericLoss: number;

  /** Geometric loss in dB */
  geometricLoss: number;

  /** Link margin in dB */
  linkMargin: number;
}

// ============================================================================
// Submarine Cable Systems
// ============================================================================

/**
 * Submarine cable system
 */
export interface SubmarineCable {
  /** Cable identifier */
  id: string;

  /** Cable name */
  name: string;

  /** Total length in meters */
  length: number;

  /** Number of fiber pairs */
  fiberPairs: number;

  /** Capacity per fiber pair in bps */
  capacityPerPair: number;

  /** Total capacity in bps */
  totalCapacity: number;

  /** Repeater spacing in meters */
  repeaterSpacing: number;

  /** Number of repeaters */
  repeaters: number;

  /** Operating wavelength bands */
  bands: WavelengthBand[];

  /** Power feed voltage in volts */
  powerFeedVoltage: number;

  /** Design lifetime in years */
  lifetime: number;

  /** Year commissioned */
  yearCommissioned: number;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for optical communication
 */
export const OPTICAL_COMM_CONSTANTS = {
  /** Speed of light in vacuum (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Planck's constant (J·s) */
  PLANCK_CONSTANT: 6.62607015e-34,

  /** Elementary charge (C) */
  ELEMENTARY_CHARGE: 1.602176634e-19,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,

  /** Reference frequency for DWDM grid (Hz) */
  ITU_REFERENCE_FREQUENCY: 193.1e12,

  /** Reference wavelength for DWDM grid (m) */
  ITU_REFERENCE_WAVELENGTH: 1552.52e-9,

  /** Standard fiber attenuation @ 1550 nm (dB/km) */
  STANDARD_FIBER_LOSS: 0.2,

  /** Standard fiber dispersion @ 1550 nm (ps/(nm·km)) */
  STANDARD_FIBER_DISPERSION: 17,

  /** Typical connector loss (dB) */
  CONNECTOR_LOSS: 0.5,

  /** Typical splice loss (dB) */
  SPLICE_LOSS: 0.1,

  /** Wavelength bands (meters) */
  WAVELENGTH_BANDS: {
    O_BAND: [1260e-9, 1360e-9],
    E_BAND: [1360e-9, 1460e-9],
    S_BAND: [1460e-9, 1530e-9],
    C_BAND: [1530e-9, 1565e-9],
    L_BAND: [1565e-9, 1625e-9],
    U_BAND: [1625e-9, 1675e-9],
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
 * Power conversion result
 */
export interface PowerConversion {
  /** Original value */
  input: number;

  /** Input unit */
  inputUnit: PowerUnit;

  /** Power in dBm */
  dBm: number;

  /** Power in mW */
  mW: number;

  /** Power in W */
  W: number;
}

/**
 * Wavelength conversion result
 */
export interface WavelengthConversion {
  /** Original value */
  input: number;

  /** Input unit */
  inputUnit: WavelengthUnit;

  /** Wavelength in meters */
  meters: number;

  /** Wavelength in nanometers */
  nanometers: number;

  /** Frequency in Hz */
  frequency: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-COMM-007 error codes
 */
export enum OpticalCommErrorCode {
  INVALID_WAVELENGTH = 'OC001',
  INVALID_POWER = 'OC002',
  INVALID_PARAMETERS = 'OC003',
  CALCULATION_ERROR = 'OC004',
  OUT_OF_RANGE = 'OC005',
  INSUFFICIENT_MARGIN = 'OC006',
  EXCESSIVE_DISPERSION = 'OC007',
  EXCESSIVE_LOSS = 'OC008',
  UNSUPPORTED_OPERATION = 'OC009',
}

/**
 * Optical communication error
 */
export class OpticalCommError extends Error {
  constructor(
    public code: OpticalCommErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'OpticalCommError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Fiber optics
  OpticalFiber,

  // WDM
  WDMChannel,
  DWDMSystem,

  // Amplifiers
  OpticalAmplifier,

  // Modulation
  ModulationScheme,

  // Coherent
  CoherentReceiver,
  DSPParameters,

  // Transceivers
  OpticalTransceiver,

  // Link budget
  LinkBudgetParams,
  LinkBudgetResult,

  // Dispersion
  DispersionParams,
  DispersionResult,

  // FEC
  FECScheme,

  // Monitoring
  PerformanceMetrics,

  // ROADM
  ROADMDegree,
  ROADMNode,

  // FSO
  FSOLink,

  // Submarine
  SubmarineCable,

  // Conversions
  PowerConversion,
  WavelengthConversion,
};

export { OPTICAL_COMM_CONSTANTS, OpticalCommErrorCode, OpticalCommError };

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
