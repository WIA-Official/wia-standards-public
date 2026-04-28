/**
 * WIA-COMM-009: Wireless Power Transfer - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Three-dimensional position vector
 */
export interface Position3D {
  x: number;  // meters
  y: number;  // meters
  z: number;  // meters (distance/height)
}

/**
 * Geographic location
 */
export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
}

/**
 * Coil parameters for inductive/resonant systems
 */
export interface CoilParameters {
  diameter: number;       // meters
  turns: number;          // number of windings
  wireDiameter: number;   // meters
  inductance: number;     // Henry
  resistance: number;     // Ohms
  quality: number;        // Q-factor
  ferrite?: {
    material: 'Mn-Zn' | 'Ni-Zn' | 'none';
    permeability: number; // µr (relative permeability)
  };
}

/**
 * Antenna parameters for far-field systems
 */
export interface AntennaParameters {
  type: 'patch' | 'horn' | 'phased-array' | 'parabolic' | 'dipole';
  gain: number;           // dBi
  beamwidth: number;      // degrees
  polarization: 'linear' | 'circular-right' | 'circular-left';
  aperture?: number;      // m² (effective area)
  efficiency: number;     // 0-1
}

// ============================================================================
// WPT Technology Types
// ============================================================================

/**
 * Wireless power transfer technologies
 */
export type WPTTechnology =
  | 'inductive'
  | 'capacitive'
  | 'resonant'
  | 'microwave'
  | 'laser'
  | 'ultrasound';

/**
 * Inductive WPT standards
 */
export type InductiveStandard =
  | 'Qi'              // Wireless Power Consortium
  | 'AirFuel'         // AirFuel Alliance
  | 'SAE-J2954'       // EV wireless charging
  | 'proprietary';

/**
 * Frequency bands
 */
export type FrequencyBand =
  | 'LF'    // 30-300 kHz
  | 'HF'    // 3-30 MHz
  | 'VHF'   // 30-300 MHz
  | 'UHF'   // 300 MHz - 3 GHz
  | 'SHF'   // 3-30 GHz (microwaves)
  | 'EHF'   // 30-300 GHz (millimeter wave)
  | 'IR'    // Infrared
  | 'Visible' // Visible light
  | 'UV';   // Ultraviolet

// ============================================================================
// Inductive Power Transfer (IPT)
// ============================================================================

/**
 * Inductive WPT configuration
 */
export interface InductiveWPTConfig {
  // Standard compliance
  standard: InductiveStandard;
  version?: string;

  // Electrical parameters
  frequency: number;        // Hz
  maxPower: number;         // Watts
  voltage: number;          // Volts DC
  current?: number;         // Amperes

  // Coil design
  transmitterCoil: CoilParameters;
  receiverCoil?: CoilParameters;

  // Control & safety
  foreignObjectDetection: boolean;
  livingObjectProtection: boolean;
  thermalProtection: boolean;
  maxTemperature: number;   // Celsius
  sarCompliance: boolean;
  maxSAR: number;          // W/kg

  // Communication protocol
  commProtocol?: 'ASK' | 'FSK' | 'Bluetooth' | 'NFC';
  commBitRate?: number;     // bps
}

/**
 * Inductive power transfer session
 */
export interface InductivePowerTransfer {
  sessionId: string;
  timestamp: Date | string;
  deviceId: string;

  // Power metrics
  powerTransmitted: number;  // Watts
  powerDelivered: number;    // Watts
  efficiency: number;        // 0-1

  // Coupling
  coupling: number;          // k (0-1)
  distance: number;          // meters
  alignment: Position3D;
  alignmentQuality: 'excellent' | 'good' | 'fair' | 'poor';

  // Environmental
  temperature: number;       // Celsius
  foreignObjects: boolean;
  livingObjectDetected: boolean;

  // Status
  status: 'initializing' | 'charging' | 'complete' | 'error' | 'paused';
  chargeLevel?: number;      // 0-100%
  estimatedTimeRemaining?: number; // seconds
}

// ============================================================================
// Capacitive Power Transfer (CPT)
// ============================================================================

/**
 * Capacitive WPT configuration
 */
export interface CapacitiveWPTConfig {
  // Electrical parameters
  frequency: number;        // Hz
  maxPower: number;         // Watts
  voltage: number;          // Volts AC

  // Plate design
  plateDimensions: {
    length: number;         // meters
    width: number;          // meters
    thickness: number;      // meters
  };
  dielectric: {
    material: string;       // e.g., 'FR-4', 'ceramic', 'air'
    permittivity: number;   // εr
    thickness: number;      // meters
  };

  // Safety
  maxElectricField: number; // V/m
  touchSafe: boolean;
}

/**
 * Capacitive power transfer session
 */
export interface CapacitivePowerTransfer {
  sessionId: string;
  timestamp: Date | string;

  powerTransmitted: number;
  powerDelivered: number;
  efficiency: number;

  capacitance: number;      // Farads
  electricField: number;    // V/m
  distance: number;         // meters (plate separation)

  status: 'active' | 'inactive' | 'error';
}

// ============================================================================
// Resonant Wireless Power
// ============================================================================

/**
 * Resonant WPT configuration
 */
export interface ResonantWPTConfig {
  // Resonant frequency
  frequency: number;        // Hz (typically 6.78 MHz)
  bandwidth: number;        // Hz

  // Coil parameters
  transmitterCoil: CoilParameters;
  receiverCoil: CoilParameters;

  // Power
  maxPower: number;         // Watts
  voltageRating: number;    // Volts

  // Resonant network
  compensationTopology: 'SS' | 'SP' | 'PS' | 'PP' | 'LCC' | 'LCL';
  capacitance: {
    primary: number;        // Farads
    secondary: number;      // Farads
  };

  // Control
  frequencyTracking: boolean;
  impedanceMatching: boolean;
  adaptiveTuning: boolean;
}

/**
 * Resonant power transfer session
 */
export interface ResonantPowerTransfer {
  sessionId: string;
  timestamp: Date | string;

  // Power
  powerTransmitted: number;
  powerDelivered: number;
  efficiency: number;
  powerFactor: number;

  // Coupling
  coupling: number;         // k
  mutualInductance: number; // Henry
  distance: number;         // meters
  alignment: Position3D;

  // Resonance
  resonantFrequency: number; // Hz
  frequencyOffset: number;   // Hz (from optimal)
  qFactor: {
    transmitter: number;
    receiver: number;
  };

  // Environmental
  temperature: number;
  heatDissipation: number;  // Watts

  status: 'active' | 'tuning' | 'error';
}

// ============================================================================
// Microwave Power Transfer
// ============================================================================

/**
 * Microwave WPT configuration
 */
export interface MicrowaveWPTConfig {
  // RF parameters
  frequency: number;        // Hz (e.g., 2.45 GHz, 5.8 GHz)
  transmitPower: number;    // Watts
  bandwidth: number;        // Hz

  // Transmitter antenna
  transmitAntenna: AntennaParameters;

  // Receiver rectenna
  receiverAntenna: AntennaParameters;
  rectennaEfficiency: number; // 0-1
  rectifierType: 'Schottky' | 'GaN-HEMT' | 'SiC';

  // Beam control
  beamSteering: boolean;
  trackingMode: 'pilot-tone' | 'retrodirective' | 'gps' | 'manual';
  phasedArrayElements?: number;

  // Safety
  maxPowerDensity: number;  // W/m²
  safetyZone: number;       // meters
  humanDetection: boolean;
}

/**
 * Microwave beam transmission
 */
export interface MicrowaveBeamTransmission {
  transmissionId: string;
  timestamp: Date | string;

  // Power
  transmitPower: number;    // Watts
  receivedPower: number;    // Watts
  efficiency: number;       // 0-1

  // Beam parameters
  frequency: number;        // Hz
  beamDirection: {
    azimuth: number;        // degrees
    elevation: number;      // degrees
  };
  beamwidth: number;        // degrees
  powerDensity: {
    peak: number;           // W/m²
    average: number;        // W/m²
  };

  // Link budget
  distance: number;         // meters
  pathLoss: number;         // dB
  atmosphericLoss: number;  // dB
  totalLoss: number;        // dB

  // Receiver
  rectennaVoltage: number;  // Volts DC
  rectennaCurrent: number;  // Amperes

  status: 'transmitting' | 'searching' | 'locked' | 'error';
}

// ============================================================================
// Laser Power Beaming
// ============================================================================

/**
 * Laser power beaming configuration
 */
export interface LaserPowerBeamingConfig {
  // Laser parameters
  wavelength: number;       // meters (e.g., 1064e-9 for Nd:YAG)
  laserPower: number;       // Watts
  laserType: 'solid-state' | 'fiber' | 'diode' | 'gas';
  beamQuality: number;      // M² factor

  // Beam characteristics
  beamDiameter: number;     // meters
  divergence: number;       // radians
  spotSize?: number;        // meters (at target)

  // Photovoltaic receiver
  pvCellType: 'GaAs' | 'InGaAs' | 'Si' | 'multi-junction';
  pvEfficiency: number;     // 0-1
  pvArea: number;           // m²
  pvVoltage: number;        // Volts

  // Beam control
  beamSteering: boolean;
  adaptiveOptics: boolean;
  tracking: boolean;

  // Safety
  eyeSafeMode: boolean;
  beamInterruption: boolean;
  maxIrradiance: number;    // W/m²
}

/**
 * Laser power transmission
 */
export interface LaserPowerTransmission {
  transmissionId: string;
  timestamp: Date | string;

  // Power
  laserPower: number;       // Watts
  deliveredPower: number;   // Watts (electrical from PV)
  efficiency: {
    optical: number;        // laser-to-spot
    conversion: number;     // spot-to-electrical
    total: number;          // end-to-end
  };

  // Beam
  wavelength: number;
  beamDiameter: number;
  spotSize: number;
  irradiance: number;       // W/m²

  // Link
  distance: number;         // meters
  atmosphericTransmission: number; // 0-1
  alignment: {
    lateral: number;        // meters (offset)
    angular: number;        // radians
  };

  // Receiver
  pvVoltage: number;
  pvCurrent: number;
  pvTemperature: number;    // Celsius

  status: 'active' | 'acquiring' | 'interrupted' | 'error';
}

// ============================================================================
// EV Dynamic Charging
// ============================================================================

/**
 * EV dynamic charging (in-road) configuration
 */
export interface EVDynamicChargingConfig {
  // Road infrastructure
  segmentLength: number;    // meters
  laneWidth: number;        // meters
  coilSpacing: number;      // meters

  // Power system
  frequency: number;        // Hz (typically 85 kHz)
  maxPower: number;         // Watts (e.g., 20-200 kW)
  segments: number;         // Number of energized segments

  // Coil design
  roadCoil: CoilParameters;
  vehicleCoil: CoilParameters;

  // Control
  dynamicSwitching: boolean;
  speedAdaptation: boolean;
  multiVehicle: boolean;

  // Safety
  airGap: number;           // meters (road-to-vehicle)
  foreignObjectDetection: boolean;
}

/**
 * EV dynamic charging session
 */
export interface EVDynamicChargingSession {
  sessionId: string;
  vehicleId: string;
  timestamp: Date | string;

  // Vehicle state
  speed: number;            // m/s
  position: GeoLocation;
  batteryLevel: number;     // % (0-100)

  // Power transfer
  powerReceived: number;    // Watts
  energyTransferred: number; // Joules
  efficiency: number;       // 0-1

  // Coil engagement
  activeSegments: number[];
  coupling: number;
  alignment: {
    lateral: number;        // meters (left/right offset)
    longitudinal: number;   // meters (front/back)
  };

  // Billing
  durationSeconds: number;
  costPerKwh?: number;
  totalCost?: number;

  status: 'charging' | 'idle' | 'complete';
}

// ============================================================================
// Space Solar Power
// ============================================================================

/**
 * Space-based solar power satellite configuration
 */
export interface SpaceSolarPowerConfig {
  // Satellite
  orbitAltitude: number;    // meters (e.g., GEO at 35,786 km)
  solarPanelArea: number;   // m²
  solarPanelEfficiency: number; // 0-1
  generatedPower: number;   // Watts

  // Transmission
  transmissionTechnology: 'microwave' | 'laser';
  frequency?: number;       // Hz (for microwave)
  wavelength?: number;      // meters (for laser)
  transmitPower: number;    // Watts
  transmitAperture: number; // meters (antenna/mirror diameter)

  // Ground station
  receiverLocation: GeoLocation;
  receiverAperture: number; // meters
  receiverEfficiency: number; // 0-1

  // Link budget
  slantRange: number;       // meters
  atmosphericLoss: number;  // dB
  ionosphericLoss?: number; // dB (for microwave)
}

/**
 * Space solar power transmission
 */
export interface SpaceSolarPowerTransmission {
  transmissionId: string;
  timestamp: Date | string;

  // Power
  solarPowerCollected: number; // Watts
  transmittedPower: number;
  receivedPower: number;
  gridPower: number;        // Watts delivered to grid
  efficiency: {
    solarToRF: number;
    linkEfficiency: number;
    rfToDC: number;
    total: number;
  };

  // Satellite state
  sunAngle: number;         // degrees
  batteryCharge?: number;   // % (for eclipse)

  // Link
  distance: number;
  atmosphericConditions: 'clear' | 'cloudy' | 'rain' | 'storm';
  linkMargin: number;       // dB

  status: 'transmitting' | 'eclipse' | 'maintenance' | 'error';
}

// ============================================================================
// Safety & Regulations
// ============================================================================

/**
 * SAR (Specific Absorption Rate) measurement
 */
export interface SARMeasurement {
  value: number;            // W/kg
  averagingMass: number;    // kg (1g or 10g)
  position: string;         // e.g., 'head', 'body', 'limb'
  frequency: number;        // Hz
  limit: number;            // W/kg (regulatory limit)
  compliant: boolean;
}

/**
 * EMI/EMC compliance
 */
export interface EMICompliance {
  standard: 'FCC-Part-15' | 'EN-55011' | 'CISPR-11' | 'ARIB-STD-T66';
  emissions: {
    conducted: {
      measured: number;     // dBµV
      limit: number;        // dBµV
      compliant: boolean;
    };
    radiated: {
      measured: number;     // dBµV/m
      limit: number;        // dBµV/m
      compliant: boolean;
    };
  };
  immunity: {
    ems: boolean;           // Electromagnetic susceptibility
    esd: boolean;           // Electrostatic discharge
  };
}

/**
 * Foreign object detection (FOD) result
 */
export interface ForeignObjectDetection {
  detected: boolean;
  method: 'Q-factor' | 'thermal' | 'capacitive' | 'radar' | 'visual';
  location?: Position3D;
  objectType?: 'metallic' | 'living' | 'unknown';
  confidence: number;       // 0-1
  action: 'continue' | 'reduce-power' | 'stop';
}

// ============================================================================
// Efficiency Calculation
// ============================================================================

/**
 * Efficiency analysis result
 */
export interface EfficiencyAnalysis {
  technology: WPTTechnology;
  distance: number;         // meters
  frequency: number;        // Hz
  power: number;            // Watts

  // Component efficiencies
  inverterEfficiency: number;
  coilEfficiency?: number;
  couplingEfficiency?: number;
  rectifierEfficiency?: number;
  totalEfficiency: number;

  // Losses
  losses: {
    conduction: number;     // Watts
    radiation: number;      // Watts
    dielectric: number;     // Watts
    mismatch: number;       // Watts
    total: number;          // Watts
  };

  // Thermal
  heatDissipation: number;  // Watts
  coolingRequired: boolean;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for wireless power calculations
 */
export const WPT_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Vacuum permeability (H/m) */
  MU_0: 4 * Math.PI * 1e-7,

  /** Vacuum permittivity (F/m) */
  EPSILON_0: 8.854187817e-12,

  /** Impedance of free space (Ohms) */
  Z_0: 376.730313668,

  /** Planck constant (J·s) */
  PLANCK: 6.62607015e-34,

  /** Boltzmann constant (J/K) */
  BOLTZMANN: 1.380649e-23,
} as const;

/**
 * ISM frequency bands (license-free)
 */
export const ISM_BANDS = {
  LF_135_KHZ: { center: 135e3, bandwidth: 2e3 },
  HF_6_78_MHZ: { center: 6.78e6, bandwidth: 15e3 },
  HF_13_56_MHZ: { center: 13.56e6, bandwidth: 14e3 },
  HF_27_12_MHZ: { center: 27.12e6, bandwidth: 163e3 },
  UHF_433_MHZ: { center: 433.05e6, bandwidth: 1.74e6 },
  UHF_915_MHZ: { center: 915e6, bandwidth: 26e6 },
  SHF_2_45_GHZ: { center: 2.45e9, bandwidth: 100e6 },
  SHF_5_8_GHZ: { center: 5.8e9, bandwidth: 125e6 },
  SHF_24_GHZ: { center: 24.125e9, bandwidth: 250e6 },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-COMM-009 error codes
 */
export enum WirelessPowerErrorCode {
  MISALIGNMENT = 'WPT001',
  LOW_COUPLING = 'WPT002',
  FOREIGN_OBJECT_DETECTED = 'WPT003',
  LIVING_OBJECT_DETECTED = 'WPT004',
  OVERHEAT = 'WPT005',
  OVERCURRENT = 'WPT006',
  OVERVOLTAGE = 'WPT007',
  COMMUNICATION_FAILURE = 'WPT008',
  SAR_VIOLATION = 'WPT009',
  EMI_VIOLATION = 'WPT010',
  BEAM_INTERRUPTION = 'WPT011',
  EFFICIENCY_TOO_LOW = 'WPT012',
}

/**
 * Wireless power transfer error
 */
export class WirelessPowerError extends Error {
  constructor(
    public code: WirelessPowerErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'WirelessPowerError';
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
  Position3D,
  GeoLocation,
  CoilParameters,
  AntennaParameters,
  WPTTechnology,
  InductiveStandard,
  FrequencyBand,
  InductiveWPTConfig,
  InductivePowerTransfer,
  CapacitiveWPTConfig,
  CapacitivePowerTransfer,
  ResonantWPTConfig,
  ResonantPowerTransfer,
  MicrowaveWPTConfig,
  MicrowaveBeamTransmission,
  LaserPowerBeamingConfig,
  LaserPowerTransmission,
  EVDynamicChargingConfig,
  EVDynamicChargingSession,
  SpaceSolarPowerConfig,
  SpaceSolarPowerTransmission,
  SARMeasurement,
  EMICompliance,
  ForeignObjectDetection,
  EfficiencyAnalysis,
};

export {
  WPT_CONSTANTS,
  ISM_BANDS,
  WirelessPowerErrorCode,
  WirelessPowerError,
};

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
