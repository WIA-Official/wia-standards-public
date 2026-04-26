/**
 * WIA-SEMI-006 Photonic Chip TypeScript SDK
 * Type Definitions for Silicon Photonics Components
 *
 * @author World Certification Industry Association (WIA)
 * @license MIT
 * @version 1.0.0
 */

// ============================================================================
// Fundamental Types
// ============================================================================

/** Wavelength in nanometers (nm) */
export type Wavelength = number;

/** Optical power in milliwatts (mW) or dBm */
export type OpticalPower = number;

/** Refractive index (dimensionless) */
export type RefractiveIndex = number;

/** Temperature in Celsius */
export type Temperature = number;

/** Data rate in Gbps */
export type DataRate = number;

// ============================================================================
// Waveguide Types
// ============================================================================

export interface WaveguideGeometry {
  /** Waveguide width in nanometers */
  width: number;
  /** Waveguide height in nanometers */
  height: number;
  /** Core material (e.g., "Si", "SiN", "Si3N4") */
  material: string;
  /** Cladding material (e.g., "SiO2", "Air") */
  cladding: string;
}

export interface WaveguideProperties {
  /** Effective refractive index */
  effectiveIndex: RefractiveIndex;
  /** Group index */
  groupIndex: number;
  /** Propagation loss in dB/cm */
  propagationLoss: number;
  /** Mode field diameter in micrometers */
  modeFieldDiameter: number;
  /** Confinement factor (0-1) */
  confinementFactor: number;
}

export interface WaveguideMode {
  /** Mode type: "TE" or "TM" */
  type: "TE" | "TM";
  /** Mode order (0 for fundamental) */
  order: number;
  /** Effective index at operating wavelength */
  effectiveIndex: RefractiveIndex;
  /** Mode profile data (normalized intensity) */
  profile?: number[][];
}

// ============================================================================
// Modulator Types
// ============================================================================

export interface ModulatorSpecs {
  /** Modulator type */
  type: "MZM" | "RingModulator" | "EAM";
  /** 3dB electro-optic bandwidth in GHz */
  bandwidth: number;
  /** Vπ voltage in volts */
  vPi: number;
  /** Vπ×Lπ figure of merit in V·cm */
  vPiLength: number;
  /** Insertion loss in dB */
  insertionLoss: number;
  /** Extinction ratio in dB */
  extinctionRatio: number;
  /** Drive voltage (Vpp) */
  driveVoltage: number;
  /** Energy per bit in fJ/bit */
  energyPerBit?: number;
}

export interface ModulationFormat {
  /** Format type */
  type: "NRZ" | "PAM4" | "PAM8" | "QPSK" | "16-QAM" | "64-QAM";
  /** Bits per symbol */
  bitsPerSymbol: number;
  /** Symbol rate in GBaud */
  symbolRate: number;
  /** Required OSNR in dB */
  requiredOSNR: number;
}

export interface ModulatorPerformance {
  /** Operating wavelength in nm */
  wavelength: Wavelength;
  /** Optical bandwidth in GHz */
  opticalBandwidth: number;
  /** Chirp parameter (alpha) */
  chirpParameter: number;
  /** Temperature coefficient (nm/°C) */
  temperatureCoefficient: number;
}

// ============================================================================
// Photodetector Types
// ============================================================================

export interface PhotodetectorSpecs {
  /** Detector type */
  type: "PIN" | "APD" | "UTC-PD";
  /** Material system */
  material: "Ge" | "Ge/Si" | "InGaAs";
  /** Responsivity in A/W */
  responsivity: number;
  /** Dark current in nA */
  darkCurrent: number;
  /** 3dB bandwidth in GHz */
  bandwidth: number;
  /** Saturation current in mA */
  saturationCurrent: number;
  /** Junction capacitance in fF */
  capacitance?: number;
}

export interface APDParameters extends PhotodetectorSpecs {
  type: "APD";
  /** Avalanche gain (M) */
  gain: number;
  /** Excess noise factor (F) */
  excessNoiseFactor: number;
  /** Operating voltage in V */
  operatingVoltage: number;
}

// ============================================================================
// WDM and Multiplexer Types
// ============================================================================

export interface WDMChannelGrid {
  /** ITU-T grid type */
  gridType: "DWDM" | "CWDM" | "LWDM";
  /** Channel spacing in GHz */
  channelSpacing: number;
  /** Center frequency in THz */
  centerFrequency: number;
  /** Number of channels */
  numChannels: number;
  /** Frequency accuracy in GHz */
  frequencyAccuracy: number;
}

export interface MultiplexerSpecs {
  /** MUX/DEMUX type */
  type: "AWG" | "Echelle" | "RingResonator" | "ThinFilm";
  /** Number of channels */
  channels: number;
  /** Insertion loss in dB */
  insertionLoss: number;
  /** Adjacent channel crosstalk in dB */
  crosstalk: number;
  /** Passband width (1dB) in nm */
  passbandWidth: number;
  /** Polarization dependent loss in dB */
  pdl: number;
  /** Temperature sensitivity in nm/°C */
  temperatureSensitivity: number;
}

export interface WDMChannel {
  /** Channel number */
  channelNumber: number;
  /** Center wavelength in nm */
  wavelength: Wavelength;
  /** Frequency in THz */
  frequency: number;
  /** Data rate per channel in Gbps */
  dataRate: DataRate;
  /** Modulation format */
  modulation: ModulationFormat;
}

// ============================================================================
// Optical Link Types
// ============================================================================

export interface OpticalLink {
  /** Link type */
  type: "SR" | "DR" | "FR" | "LR" | "ER";
  /** Total link length in meters */
  length: number;
  /** Fiber type */
  fiberType: "MMF" | "SMF";
  /** Number of lanes/wavelengths */
  lanes: number;
  /** Data rate per lane in Gbps */
  dataRatePerLane: DataRate;
  /** Total aggregate bandwidth in Gbps */
  aggregateBandwidth: DataRate;
}

export interface LinkBudget {
  /** Transmitter output power in dBm */
  txPower: OpticalPower;
  /** Fiber loss in dB */
  fiberLoss: number;
  /** Connector loss in dB */
  connectorLoss: number;
  /** Other insertion losses in dB */
  otherLosses: number;
  /** Total link loss in dB */
  totalLoss: number;
  /** Receiver sensitivity in dBm */
  rxSensitivity: OpticalPower;
  /** Link margin in dB */
  linkMargin: number;
}

// ============================================================================
// Coupling and Fiber Attachment
// ============================================================================

export interface FiberCouplingSpecs {
  /** Coupling method */
  method: "EdgeCoupling" | "GratingCoupler";
  /** Coupling efficiency (loss) in dB */
  couplingLoss: number;
  /** Optical bandwidth in nm */
  bandwidth: number;
  /** Alignment tolerance in micrometers */
  alignmentTolerance: {
    lateral: number;
    vertical: number;
  };
  /** Polarization dependent loss in dB */
  pdl?: number;
}

export interface GratingCoupler extends FiberCouplingSpecs {
  method: "GratingCoupler";
  /** Optimal fiber angle in degrees */
  fiberAngle: number;
  /** Grating period in nm */
  gratingPeriod: number;
  /** Back reflection in dB */
  backReflection: number;
}

// ============================================================================
// Thermal Management
// ============================================================================

export interface ThermalProperties {
  /** Operating temperature range in °C */
  temperatureRange: {
    min: Temperature;
    max: Temperature;
  };
  /** Power dissipation in mW */
  powerDissipation: number;
  /** Thermal resistance (junction-to-case) in °C/W */
  thermalResistance: number;
  /** Temperature coefficient for wavelength shift in nm/°C */
  wavelengthTempCoeff: number;
}

export interface ThermalControl {
  /** Control type */
  type: "Passive" | "TEC" | "Heater";
  /** Target temperature in °C */
  targetTemperature: Temperature;
  /** Temperature stability in °C */
  temperatureStability: number;
  /** Control power in mW */
  controlPower?: number;
}

// ============================================================================
// Co-Packaged Optics (CPO)
// ============================================================================

export interface CPOConfiguration {
  /** Integration approach */
  integrationType: "OnPackage" | "NearPackage" | "InPackage";
  /** ASIC to photonics distance in mm */
  asicDistance: number;
  /** Number of optical I/O channels */
  opticalChannels: number;
  /** Total bandwidth in Tbps */
  totalBandwidth: number;
  /** Power budget in watts */
  powerBudget: {
    photonics: number;
    serdes: number;
    total: number;
  };
  /** Latency in nanoseconds */
  latency: number;
}

// ============================================================================
// Testing and Measurement
// ============================================================================

export interface OpticalMeasurement {
  /** Measurement type */
  type: "Power" | "Spectrum" | "BER" | "EyeDiagram" | "Dispersion";
  /** Wavelength(s) measured */
  wavelength: Wavelength | Wavelength[];
  /** Measured value */
  value: number | number[];
  /** Unit of measurement */
  unit: string;
  /** Measurement timestamp */
  timestamp: Date;
  /** Measurement uncertainty */
  uncertainty?: number;
}

export interface BERTestResult {
  /** Bit error rate */
  ber: number;
  /** Test duration in seconds */
  duration: number;
  /** Number of bits tested */
  bitsTested: number;
  /** Number of errors detected */
  errors: number;
  /** Optical power level in dBm */
  opticalPower: OpticalPower;
  /** FEC enabled */
  fecEnabled: boolean;
}

export interface EyeDiagramMetrics {
  /** Eye height in mV or percentage */
  eyeHeight: number;
  /** Eye width in ps or percentage */
  eyeWidth: number;
  /** Extinction ratio in dB */
  extinctionRatio: number;
  /** RMS jitter in ps */
  jitterRMS: number;
  /** Peak-to-peak jitter in ps */
  jitterPP: number;
  /** Q-factor */
  qFactor: number;
}

// ============================================================================
// Device Specifications and Compliance
// ============================================================================

export interface WIACompliance {
  /** Standard version */
  standardVersion: "WIA-SEMI-006-v1.0";
  /** Compliance status */
  compliant: boolean;
  /** Certification date */
  certificationDate?: Date;
  /** Certificate number */
  certificateNumber?: string;
  /** Test reports */
  testReports?: string[];
}

export interface PhotonicChipSpecs {
  /** Chip identifier */
  chipId: string;
  /** Manufacturer */
  manufacturer: string;
  /** Technology node */
  technologyNode: string;
  /** Operating wavelength range */
  wavelengthRange: {
    min: Wavelength;
    max: Wavelength;
  };
  /** Components integrated */
  components: {
    modulators?: ModulatorSpecs[];
    photodetectors?: PhotodetectorSpecs[];
    multiplexers?: MultiplexerSpecs[];
    switches?: number;
    lasers?: number;
  };
  /** WIA compliance */
  wiaCompliance: WIACompliance;
}

// ============================================================================
// Simulation and Calculation Types
// ============================================================================

export interface SimulationParameters {
  /** Wavelength range for simulation */
  wavelengthRange: {
    start: Wavelength;
    end: Wavelength;
    step: number;
  };
  /** Temperature for simulation */
  temperature: Temperature;
  /** Input power in mW */
  inputPower?: OpticalPower;
}

export interface SimulationResult {
  /** Simulation type */
  type: string;
  /** Input parameters */
  parameters: SimulationParameters;
  /** Results data */
  data: {
    wavelength: Wavelength[];
    transmission?: number[];
    phase?: number[];
    loss?: number[];
  };
  /** Computation time in ms */
  computationTime?: number;
}

// ============================================================================
// Export all types
// ============================================================================

export type PhotonicDevice =
  | ModulatorSpecs
  | PhotodetectorSpecs
  | MultiplexerSpecs;

export interface PhotonicCircuit {
  /** Circuit name */
  name: string;
  /** Devices in the circuit */
  devices: PhotonicDevice[];
  /** Interconnections */
  connections: Array<{
    from: string;
    to: string;
    waveguide: WaveguideGeometry;
  }>;
  /** Overall performance metrics */
  performance: {
    totalLoss: number;
    totalBandwidth: DataRate;
    powerConsumption: number;
  };
}
