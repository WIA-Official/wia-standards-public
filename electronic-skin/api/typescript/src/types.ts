/**
 * WIA-SEMI-016 Electronic Skin SDK - Type Definitions
 *
 * @packageDocumentation
 * @module @wia-official/electronic-skin
 */

/**
 * Device classification by application domain
 */
export enum DeviceClass {
  /** Medical devices (prosthetics, patient monitoring) */
  MEDICAL = 'A',
  /** Consumer electronics (wearables, smartphones) */
  CONSUMER = 'B',
  /** Industrial/Robotics (collaborative robots, automation) */
  INDUSTRIAL = 'C',
  /** Research and development */
  RESEARCH = 'D'
}

/**
 * Device classification by contact duration
 */
export enum ContactType {
  /** Limited contact (<24 hours) */
  LIMITED = 'I',
  /** Prolonged contact (24 hours to 30 days) */
  PROLONGED = 'II',
  /** Permanent contact (>30 days) */
  PERMANENT = 'III'
}

/**
 * Sensing modality types
 */
export enum SensingModality {
  /** Pressure/force sensing */
  PRESSURE = 'SM-P',
  /** Temperature sensing */
  TEMPERATURE = 'SM-T',
  /** Strain/deformation sensing */
  STRAIN = 'SM-S',
  /** Multi-modal sensing (2+ modalities) */
  MULTIMODAL = 'SM-M'
}

/**
 * Wireless communication protocols
 */
export enum WirelessProtocol {
  BLE = 'BLE',
  ZIGBEE = 'Zigbee',
  NFC = 'NFC',
  WIFI = 'Wi-Fi'
}

/**
 * Sensor types
 */
export enum SensorType {
  CAPACITIVE = 'capacitive',
  PIEZORESISTIVE = 'piezoresistive',
  PIEZOELECTRIC = 'piezoelectric',
  TRIBOELECTRIC = 'triboelectric',
  OPTICAL = 'optical'
}

/**
 * Mechanical properties specification
 */
export interface MechanicalProperties {
  /** Stretchability as percentage (minimum 30%) */
  stretchability: number;
  /** Thickness in micrometers (10-500 μm) */
  thickness: number;
  /** Young's modulus in MPa (0.5-2 MPa target) */
  youngsModulus: number;
  /** Durability in cycles (minimum 10,000 at 30% strain) */
  durability: number;
}

/**
 * Pressure sensing specifications
 */
export interface PressureSensing {
  /** Minimum detectable pressure in kPa */
  minPressure: number;
  /** Maximum reliable pressure in kPa */
  maxPressure: number;
  /** Sensitivity in kPa⁻¹ (minimum 0.1) */
  sensitivity: number;
  /** Spatial resolution in mm */
  spatialResolution: number;
  /** Response time in milliseconds */
  responseTime: number;
  /** Hysteresis as percentage of full scale (max 10%) */
  hysteresis: number;
}

/**
 * Temperature sensing specifications
 */
export interface TemperatureSensing {
  /** Operating temperature range minimum in °C */
  minTemperature: number;
  /** Operating temperature range maximum in °C */
  maxTemperature: number;
  /** Temperature accuracy in °C */
  accuracy: number;
  /** Response time in seconds */
  responseTime: number;
}

/**
 * Electrical characteristics
 */
export interface ElectricalCharacteristics {
  /** Sheet resistance in Ω/sq (target <100) */
  sheetResistance?: number;
  /** Signal-to-noise ratio in dB (minimum 40) */
  snr: number;
  /** Power consumption in mW */
  powerConsumption: number;
  /** Operating voltage in V */
  operatingVoltage: number;
}

/**
 * Biocompatibility test results
 */
export interface BiocompatibilityResults {
  /** Cytotoxicity grade (0-1 required) */
  cytotoxicity: number;
  /** Cell viability percentage (≥70% required) */
  cellViability: number;
  /** Sensitization test passed */
  sensitizationPassed: boolean;
  /** Irritation index (<2.0 required) */
  irritationIndex: number;
  /** ISO 10993 compliance */
  iso10993Compliant: boolean;
}

/**
 * Wireless communication specifications
 */
export interface WirelessSpecification {
  /** Communication protocol */
  protocol: WirelessProtocol;
  /** Data rate in kbps */
  dataRate: number;
  /** Communication latency in ms */
  latency: number;
  /** Communication range in meters */
  range: number;
  /** Encryption enabled */
  encrypted: boolean;
}

/**
 * Sensor reading data point
 */
export interface SensorReading {
  /** Timestamp in milliseconds since epoch */
  timestamp: number;
  /** Sensor ID */
  sensorId: string;
  /** Raw sensor value */
  rawValue: number;
  /** Calibrated value in physical units */
  calibratedValue: number;
  /** Unit of measurement */
  unit: string;
  /** Data quality indicator (0-1, 1 = best) */
  quality?: number;
}

/**
 * Multi-sensor array reading
 */
export interface ArrayReading {
  /** Timestamp in milliseconds since epoch */
  timestamp: number;
  /** Array of sensor readings */
  sensors: SensorReading[];
  /** Computed features (e.g., center of pressure) */
  features?: Record<string, number>;
}

/**
 * Device configuration
 */
export interface DeviceConfiguration {
  /** Device unique identifier */
  deviceId: string;
  /** Device classification */
  deviceClass: DeviceClass;
  /** Contact type */
  contactType: ContactType;
  /** Sensing modalities */
  sensingModalities: SensingModality[];
  /** Sensor type */
  sensorType: SensorType;
  /** Number of sensors in array */
  sensorCount: number;
  /** Sampling rate in Hz */
  samplingRate: number;
  /** Wireless specification (if applicable) */
  wireless?: WirelessSpecification;
}

/**
 * Device specifications (complete)
 */
export interface DeviceSpecification {
  /** Configuration */
  configuration: DeviceConfiguration;
  /** Mechanical properties */
  mechanical: MechanicalProperties;
  /** Pressure sensing specs */
  pressureSensing?: PressureSensing;
  /** Temperature sensing specs */
  temperatureSensing?: TemperatureSensing;
  /** Electrical characteristics */
  electrical: ElectricalCharacteristics;
  /** Biocompatibility results (for medical devices) */
  biocompatibility?: BiocompatibilityResults;
}

/**
 * Calibration coefficients
 */
export interface CalibrationData {
  /** Sensor ID */
  sensorId: string;
  /** Calibration type (linear, polynomial, lookup) */
  type: 'linear' | 'polynomial' | 'lookup';
  /** Calibration coefficients */
  coefficients: number[];
  /** Calibration timestamp */
  timestamp: number;
  /** Calibration valid until (timestamp) */
  validUntil?: number;
}

/**
 * Device status
 */
export interface DeviceStatus {
  /** Device connected */
  connected: boolean;
  /** Battery level (0-100%) */
  batteryLevel?: number;
  /** Signal strength (0-100%) */
  signalStrength?: number;
  /** Error messages */
  errors: string[];
  /** Warnings */
  warnings: string[];
  /** Last update timestamp */
  lastUpdate: number;
}

/**
 * Event types
 */
export enum EventType {
  /** Device connected */
  CONNECTED = 'connected',
  /** Device disconnected */
  DISCONNECTED = 'disconnected',
  /** Data received */
  DATA = 'data',
  /** Error occurred */
  ERROR = 'error',
  /** Warning issued */
  WARNING = 'warning',
  /** Battery low */
  BATTERY_LOW = 'battery_low',
  /** Calibration needed */
  CALIBRATION_NEEDED = 'calibration_needed'
}

/**
 * Event handler callback type
 */
export type EventHandler<T = any> = (data: T) => void;

/**
 * Compliance check result
 */
export interface ComplianceResult {
  /** Overall compliance status */
  compliant: boolean;
  /** Standard version checked against */
  standardVersion: string;
  /** Individual requirement results */
  requirements: {
    name: string;
    passed: boolean;
    actualValue: number | string | boolean;
    requiredValue: number | string | boolean;
    message?: string;
  }[];
  /** Timestamp of compliance check */
  timestamp: number;
}

/**
 * Data processing options
 */
export interface ProcessingOptions {
  /** Apply baseline correction */
  baselineCorrection?: boolean;
  /** Filter type (none, lowpass, highpass, bandpass) */
  filterType?: 'none' | 'lowpass' | 'highpass' | 'bandpass';
  /** Filter cutoff frequency in Hz */
  filterCutoff?: number;
  /** Apply calibration */
  applyCalibration?: boolean;
  /** Compute features */
  computeFeatures?: boolean;
}

/**
 * Computed features from sensor array
 */
export interface ComputedFeatures {
  /** Total force/pressure across all sensors */
  totalForce: number;
  /** Center of pressure X coordinate */
  centerOfPressureX: number;
  /** Center of pressure Y coordinate */
  centerOfPressureY: number;
  /** Contact area (number of active sensors) */
  contactArea: number;
  /** Maximum sensor value */
  maxValue: number;
  /** Mean sensor value */
  meanValue: number;
  /** Standard deviation */
  standardDeviation: number;
}
