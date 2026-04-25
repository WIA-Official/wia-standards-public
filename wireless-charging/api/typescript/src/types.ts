/**
 * WIA-COMM-008: Wireless Charging - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Physics Types
// ============================================================================

/**
 * Power units
 */
export type PowerUnit = 'W' | 'mW' | 'kW' | 'MW';

/**
 * Frequency units
 */
export type FrequencyUnit = 'Hz' | 'kHz' | 'MHz';

/**
 * Distance units
 */
export type DistanceUnit = 'mm' | 'cm' | 'm';

/**
 * Temperature units
 */
export type TemperatureUnit = 'C' | 'F' | 'K';

// ============================================================================
// Wireless Charging Standards
// ============================================================================

/**
 * Wireless charging standard types
 */
export type ChargingStandard = 'Qi-BPP' | 'Qi-EPP' | 'Qi-MP' | 'AirFuel' | 'SAE-J2954' | 'ISO-19363';

/**
 * Qi power profiles
 */
export type QiPowerProfile = 'BPP' | 'EPP' | 'MP';

/**
 * Charging technology types
 */
export type ChargingTechnology = 'inductive' | 'resonant' | 'rf-microwave';

/**
 * Power level categories
 */
export type PowerLevel = 'low' | 'medium' | 'high' | 'ultra-high';

// ============================================================================
// Coil Types
// ============================================================================

/**
 * Coil configuration
 */
export interface CoilConfig {
  /** Outer diameter in mm */
  outerDiameter: number;

  /** Inner diameter in mm */
  innerDiameter: number;

  /** Number of turns */
  turns: number;

  /** Wire gauge (AWG) */
  wireGauge: number;

  /** Wire type */
  wireType: 'solid' | 'litz';

  /** Litz wire strand count (if applicable) */
  litzStrands?: number;

  /** Core material */
  coreMaterial: 'air' | 'ferrite' | 'nanocrystalline';

  /** Ferrite thickness in mm (if applicable) */
  ferriteThickness?: number;

  /** Inductance in μH */
  inductance: number;

  /** DC resistance in Ω */
  resistance: number;

  /** Quality factor Q */
  qualityFactor: number;
}

/**
 * Transmitter coil (power delivery)
 */
export interface TransmitterCoil extends CoilConfig {
  /** Role identifier */
  role: 'transmitter';

  /** Operating frequency in Hz */
  frequency: number;

  /** Maximum current in A */
  maxCurrent: number;

  /** Maximum power in W */
  maxPower: number;

  /** Temperature sensor */
  temperatureSensor?: TemperatureSensor;
}

/**
 * Receiver coil (power reception)
 */
export interface ReceiverCoil extends CoilConfig {
  /** Role identifier */
  role: 'receiver';

  /** Operating frequency in Hz */
  frequency: number;

  /** Maximum voltage in V */
  maxVoltage: number;

  /** Rectification type */
  rectificationType: 'diode' | 'synchronous';
}

// ============================================================================
// Power Transfer
// ============================================================================

/**
 * Power transfer configuration
 */
export interface PowerTransferConfig {
  /** Transmitter coil */
  transmitter: TransmitterCoil;

  /** Receiver coil */
  receiver: ReceiverCoil;

  /** Separation distance in mm */
  distance: number;

  /** Lateral misalignment in mm */
  lateralOffset?: { x: number; y: number };

  /** Angular misalignment in degrees */
  angularOffset?: number;

  /** Operating frequency in Hz */
  frequency: number;

  /** Input power in W */
  inputPower: number;
}

/**
 * Power transfer result
 */
export interface PowerTransferResult {
  /** Coupling coefficient (0-1) */
  couplingCoefficient: number;

  /** Mutual inductance in H */
  mutualInductance: number;

  /** Transferred power in W */
  transferredPower: number;

  /** Efficiency (0-1) */
  efficiency: number;

  /** Output voltage in V */
  outputVoltage: number;

  /** Output current in A */
  outputCurrent: number;

  /** Coil losses in W */
  coilLosses: number;

  /** Rectifier losses in W */
  rectifierLosses: number;

  /** Total losses in W */
  totalLosses: number;
}

// ============================================================================
// Foreign Object Detection (FOD)
// ============================================================================

/**
 * Foreign object detection method
 */
export type FODMethod = 'q-factor' | 'power-loss' | 'frequency-shift' | 'multi-method';

/**
 * FOD configuration
 */
export interface FODConfig {
  /** Detection method */
  method: FODMethod;

  /** Baseline Q factor */
  baselineQ: number;

  /** Q factor change threshold (percentage) */
  qThreshold: number;

  /** Power loss threshold in W */
  powerLossThreshold: number;

  /** Frequency shift threshold in Hz */
  frequencyShiftThreshold: number;

  /** Sampling rate in Hz */
  samplingRate: number;

  /** Detection sensitivity */
  sensitivity: 'low' | 'medium' | 'high';
}

/**
 * FOD result
 */
export interface FODResult {
  /** Foreign object detected */
  detected: boolean;

  /** Detection confidence (0-1) */
  confidence: number;

  /** Measured Q factor */
  measuredQ: number;

  /** Q factor change percentage */
  qChange: number;

  /** Power loss in W */
  powerLoss: number;

  /** Frequency shift in Hz */
  frequencyShift: number;

  /** Recommended action */
  action: 'continue' | 'reduce-power' | 'stop-charging' | 'alert-user';
}

// ============================================================================
// Alignment and Positioning
// ============================================================================

/**
 * Alignment feedback type
 */
export type AlignmentFeedbackType = 'visual' | 'auditory' | 'haptic' | 'multi-modal';

/**
 * Alignment status
 */
export interface AlignmentStatus {
  /** Alignment score (0-100) */
  score: number;

  /** Alignment quality */
  quality: 'poor' | 'fair' | 'good' | 'excellent';

  /** Lateral offset in mm */
  lateralOffset: { x: number; y: number };

  /** Angular offset in degrees */
  angularOffset: number;

  /** Recommended adjustment */
  recommendation: {
    direction?: 'up' | 'down' | 'left' | 'right';
    distance?: number; // in mm
  };

  /** Feedback for user */
  feedback: AlignmentFeedback;
}

/**
 * Alignment feedback
 */
export interface AlignmentFeedback {
  /** Feedback type */
  type: AlignmentFeedbackType;

  /** Visual feedback */
  visual?: {
    ledColor: 'red' | 'yellow' | 'green';
    displayMessage?: string;
  };

  /** Auditory feedback */
  auditory?: {
    beepFrequency: number; // Hz
    tonePitch: number; // Hz
  };

  /** Haptic feedback */
  haptic?: {
    vibrationIntensity: number; // 0-100
    pulseRate: number; // Hz
  };
}

// ============================================================================
// Thermal Management
// ============================================================================

/**
 * Temperature sensor
 */
export interface TemperatureSensor {
  /** Sensor type */
  type: 'NTC' | 'thermocouple' | 'infrared';

  /** Current temperature in °C */
  currentTemperature: number;

  /** Warning threshold in °C */
  warningThreshold: number;

  /** Cutoff threshold in °C */
  cutoffThreshold: number;

  /** Sampling rate in Hz */
  samplingRate: number;
}

/**
 * Thermal management configuration
 */
export interface ThermalConfig {
  /** Temperature sensors */
  sensors: TemperatureSensor[];

  /** Cooling method */
  coolingMethod: 'passive' | 'active-fan' | 'liquid';

  /** Dynamic power scaling enabled */
  dynamicPowerScaling: boolean;

  /** Duty cycle control enabled */
  dutyCycleControl: boolean;

  /** Maximum temperature in °C */
  maxTemperature: number;
}

/**
 * Thermal status
 */
export interface ThermalStatus {
  /** Current maximum temperature in °C */
  maxTemperature: number;

  /** Temperature distribution */
  temperatures: {
    transmitterCoil?: number;
    receiverCoil?: number;
    rectifier?: number;
    battery?: number;
  };

  /** Thermal state */
  state: 'normal' | 'warning' | 'critical';

  /** Recommended action */
  action: 'continue' | 'reduce-power' | 'pause-charging' | 'stop-charging';

  /** Power scaling factor (0-1) */
  powerScalingFactor: number;
}

// ============================================================================
// EMF Safety
// ============================================================================

/**
 * EMF safety standard
 */
export type EMFStandard = 'ICNIRP-2010' | 'IEEE-C95.1' | 'SAE-J2954';

/**
 * EMF exposure limits
 */
export interface EMFLimits {
  /** Standard name */
  standard: EMFStandard;

  /** Magnetic field limit in μT (RMS) */
  magneticFieldLimit: number;

  /** Electric field limit in V/m (RMS) */
  electricFieldLimit: number;

  /** SAR limit in W/kg (if applicable) */
  sarLimit?: number;

  /** Frequency range */
  frequencyRange: { min: number; max: number };
}

/**
 * EMF measurement
 */
export interface EMFMeasurement {
  /** Measurement point */
  location: { x: number; y: number; z: number };

  /** Magnetic flux density in μT */
  magneticField: number;

  /** Electric field in V/m */
  electricField: number;

  /** Distance from coil in mm */
  distance: number;

  /** Compliance status */
  compliant: boolean;

  /** Applicable standard */
  standard: EMFStandard;

  /** Safety margin (percentage below limit) */
  safetyMargin: number;
}

// ============================================================================
// Charging Session
// ============================================================================

/**
 * Charging phase
 */
export type ChargingPhase =
  | 'idle'
  | 'selection'
  | 'ping'
  | 'identification'
  | 'configuration'
  | 'power-transfer'
  | 'renegotiation'
  | 'complete'
  | 'error';

/**
 * Charging session
 */
export interface ChargingSession {
  /** Session ID */
  id: string;

  /** Start timestamp */
  startTime: Date;

  /** End timestamp */
  endTime?: Date;

  /** Current phase */
  currentPhase: ChargingPhase;

  /** Charging standard */
  standard: ChargingStandard;

  /** Device information */
  device: {
    manufacturer?: string;
    model?: string;
    batteryCapacity?: number; // mAh
    currentBatteryLevel?: number; // percentage
  };

  /** Power statistics */
  powerStats: {
    averagePower: number; // W
    peakPower: number; // W
    totalEnergy: number; // Wh
    averageEfficiency: number; // percentage
  };

  /** Alignment statistics */
  alignmentStats: {
    averageScore: number;
    minScore: number;
    maxScore: number;
  };

  /** Thermal statistics */
  thermalStats: {
    maxTemperature: number; // °C
    averageTemperature: number; // °C
  };

  /** FOD events */
  fodEvents: {
    timestamp: Date;
    action: string;
  }[];

  /** Session status */
  status: 'active' | 'paused' | 'completed' | 'error';

  /** Error message (if any) */
  errorMessage?: string;
}

// ============================================================================
// Multi-Device Charging
// ============================================================================

/**
 * Multi-device charging architecture
 */
export type MultiDeviceArchitecture = 'multi-coil-array' | 'single-large-coil' | 'guided-positioning';

/**
 * Device on charging pad
 */
export interface ChargingDevice {
  /** Device ID */
  id: string;

  /** Position on pad */
  position: { x: number; y: number };

  /** Requested power in W */
  requestedPower: number;

  /** Allocated power in W */
  allocatedPower: number;

  /** Current power in W */
  currentPower: number;

  /** Assigned coil ID (for multi-coil systems) */
  assignedCoilId?: string;

  /** Charging priority */
  priority: 'low' | 'normal' | 'high';

  /** Charging session */
  session: ChargingSession;
}

/**
 * Multi-device charger configuration
 */
export interface MultiDeviceChargerConfig {
  /** Architecture type */
  architecture: MultiDeviceArchitecture;

  /** Number of coils */
  coilCount: number;

  /** Coil configurations */
  coils: TransmitterCoil[];

  /** Maximum total power in W */
  maxTotalPower: number;

  /** Power allocation strategy */
  allocationStrategy: 'equal' | 'priority-based' | 'first-come-first-served';

  /** Time-division multiplexing enabled */
  tdmEnabled: boolean;

  /** Multiplexing frequency in Hz (if TDM enabled) */
  tdmFrequency?: number;
}

/**
 * Multi-device charging status
 */
export interface MultiDeviceStatus {
  /** Active devices */
  devices: ChargingDevice[];

  /** Total power consumption in W */
  totalPower: number;

  /** Power utilization (percentage) */
  powerUtilization: number;

  /** Available capacity in W */
  availableCapacity: number;

  /** Can accept new device */
  canAcceptNewDevice: boolean;
}

// ============================================================================
// EV Wireless Charging
// ============================================================================

/**
 * SAE J2954 power class
 */
export type SAEPowerClass = 'WPT1' | 'WPT2' | 'WPT3' | 'WPT4';

/**
 * Ground assembly (charging pad in pavement)
 */
export interface GroundAssembly {
  /** Assembly ID */
  id: string;

  /** Power class */
  powerClass: SAEPowerClass;

  /** Rated power in kW */
  ratedPower: number;

  /** Coil configuration */
  coil: TransmitterCoil;

  /** Installation height from ground in mm */
  installationHeight: number;

  /** Operating frequency in Hz */
  frequency: number;

  /** Maximum ground clearance in mm */
  maxGroundClearance: number;

  /** Alignment tolerance in mm */
  alignmentTolerance: { x: number; y: number };
}

/**
 * Vehicle assembly (charging pad on vehicle)
 */
export interface VehicleAssembly {
  /** Assembly ID */
  id: string;

  /** Power class */
  powerClass: SAEPowerClass;

  /** Rated power in kW */
  ratedPower: number;

  /** Coil configuration */
  coil: ReceiverCoil;

  /** Ground clearance in mm */
  groundClearance: number;

  /** Vehicle make/model */
  vehicle: {
    make: string;
    model: string;
    year: number;
    batteryCapacity: number; // kWh
  };
}

/**
 * EV charging session
 */
export interface EVChargingSession {
  /** Session ID */
  id: string;

  /** Ground assembly */
  groundAssembly: GroundAssembly;

  /** Vehicle assembly */
  vehicleAssembly: VehicleAssembly;

  /** Start timestamp */
  startTime: Date;

  /** End timestamp */
  endTime?: Date;

  /** Charging phase */
  phase: ChargingPhase;

  /** Power statistics */
  powerStats: {
    currentPower: number; // kW
    averagePower: number; // kW
    totalEnergy: number; // kWh
    efficiency: number; // percentage
  };

  /** Alignment status */
  alignment: AlignmentStatus;

  /** Battery status */
  battery: {
    initialSOC: number; // percentage
    currentSOC: number; // percentage
    targetSOC: number; // percentage
  };

  /** Session status */
  status: 'active' | 'paused' | 'completed' | 'error';
}

// ============================================================================
// Efficiency Optimization
// ============================================================================

/**
 * Impedance matching configuration
 */
export interface ImpedanceMatchingConfig {
  /** Coil impedance in Ω */
  coilImpedance: number;

  /** Load impedance in Ω */
  loadImpedance: number;

  /** Matching network topology */
  topology: 'L-match' | 'Pi-match' | 'T-match';

  /** Component values */
  components: {
    capacitors?: number[]; // F
    inductors?: number[]; // H
  };
}

/**
 * Resonance tuning configuration
 */
export interface ResonanceTuningConfig {
  /** Target frequency in Hz */
  targetFrequency: number;

  /** Coil inductance in H */
  coilInductance: number;

  /** Tuning capacitance in F */
  tuningCapacitance: number;

  /** Measured resonant frequency in Hz */
  measuredFrequency: number;

  /** Frequency error in Hz */
  frequencyError: number;

  /** Tuning status */
  status: 'tuned' | 'needs-adjustment' | 'out-of-range';
}

/**
 * Efficiency optimization result
 */
export interface EfficiencyOptimizationResult {
  /** Original efficiency */
  originalEfficiency: number;

  /** Optimized efficiency */
  optimizedEfficiency: number;

  /** Efficiency gain */
  efficiencyGain: number;

  /** Optimization methods applied */
  methodsApplied: (
    | 'impedance-matching'
    | 'resonance-tuning'
    | 'coil-alignment'
    | 'high-q-coils'
    | 'active-rectification'
  )[];

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Communication Protocols
// ============================================================================

/**
 * Qi communication packet
 */
export interface QiPacket {
  /** Packet type */
  type: 'signal-strength' | 'control-error' | 'power-request' | 'end-power' | 'proprietary';

  /** Header byte */
  header: number;

  /** Message bytes */
  message: number[];

  /** Checksum byte */
  checksum: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * AirFuel BLE control message
 */
export interface AirFuelBLEMessage {
  /** Message type */
  type: 'discovery' | 'pairing' | 'power-negotiation' | 'status' | 'authentication';

  /** Device ID */
  deviceId: string;

  /** Message payload */
  payload: Record<string, any>;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Certification and Testing
// ============================================================================

/**
 * Certification status
 */
export interface CertificationStatus {
  /** Standard name */
  standard: ChargingStandard;

  /** Certification authority */
  authority: 'WPC' | 'AirFuel' | 'SAE' | 'ISO' | 'UL' | 'CE' | 'FCC';

  /** Certification ID */
  certificationId?: string;

  /** Issue date */
  issueDate?: Date;

  /** Expiry date */
  expiryDate?: Date;

  /** Status */
  status: 'pending' | 'certified' | 'expired' | 'revoked';

  /** Test results */
  testResults?: TestResults;
}

/**
 * Test results
 */
export interface TestResults {
  /** Power transfer test */
  powerTransfer: {
    passed: boolean;
    powerLevels: number[]; // W
    efficiencies: number[]; // percentage
  };

  /** Communication test */
  communication: {
    passed: boolean;
    packetErrorRate: number; // percentage
  };

  /** FOD test */
  fod: {
    passed: boolean;
    detectionRate: number; // percentage
    falsePositiveRate: number; // percentage
  };

  /** EMI/EMC test */
  emiEmc: {
    passed: boolean;
    complianceStandard: string;
  };

  /** Safety test */
  safety: {
    passed: boolean;
    maxTemperature: number; // °C
    emfCompliance: boolean;
  };

  /** Overall result */
  overall: {
    passed: boolean;
    score: number; // 0-100
  };
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * Wireless charging SDK configuration
 */
export interface WirelessChargingConfig {
  /** Charging standard */
  standard: ChargingStandard;

  /** Transmitter configuration */
  transmitter?: TransmitterCoil;

  /** Receiver configuration */
  receiver?: ReceiverCoil;

  /** FOD configuration */
  fod: FODConfig;

  /** Thermal configuration */
  thermal: ThermalConfig;

  /** EMF safety standard */
  emfStandard: EMFStandard;

  /** Debug mode enabled */
  debug?: boolean;

  /** Logging enabled */
  logging?: boolean;
}

/**
 * SDK Events
 */
export type WirelessChargingEvent =
  | { type: 'session-started'; session: ChargingSession }
  | { type: 'session-ended'; session: ChargingSession }
  | { type: 'power-changed'; power: number }
  | { type: 'efficiency-changed'; efficiency: number }
  | { type: 'fod-detected'; result: FODResult }
  | { type: 'thermal-warning'; status: ThermalStatus }
  | { type: 'alignment-changed'; status: AlignmentStatus }
  | { type: 'error'; error: Error };

/**
 * Event handler type
 */
export type EventHandler<T extends WirelessChargingEvent = WirelessChargingEvent> = (event: T) => void;

// ============================================================================
// Exports
// ============================================================================

export * from './types';
