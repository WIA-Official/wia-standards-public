/**
 * WIA-IND-002: Smart Textile Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-ind-002
 */

/**
 * Smart textile category
 */
export enum SmartTextileCategory {
  /** Conductive fabrics with embedded wiring */
  Conductive = 'conductive',
  /** Textiles with integrated sensors */
  Sensing = 'sensing',
  /** Power harvesting textiles */
  EnergyHarvesting = 'energy_harvesting',
  /** Display-capable fabrics */
  Display = 'display',
  /** Thermally adaptive textiles */
  ThermalAdaptive = 'thermal_adaptive',
  /** Shape-changing textiles */
  ShapeMemory = 'shape_memory'
}

/**
 * Conductive thread type
 */
export enum ConductiveThreadType {
  /** Silver-coated thread */
  SilverCoated = 'silver_coated',
  /** Stainless steel fiber */
  StainlessSteel = 'stainless_steel',
  /** Carbon nanotube yarn */
  CarbonNanotube = 'carbon_nanotube',
  /** Copper wire */
  Copper = 'copper',
  /** Conductive polymer */
  ConductivePolymer = 'conductive_polymer'
}

/**
 * Sensor type embedded in textile
 */
export enum TextileSensorType {
  /** Temperature sensor */
  Temperature = 'temperature',
  /** Pressure/strain sensor */
  Pressure = 'pressure',
  /** Humidity sensor */
  Humidity = 'humidity',
  /** Biometric sensor */
  Biometric = 'biometric',
  /** Motion/accelerometer */
  Motion = 'motion',
  /** Light/photo sensor */
  Light = 'light',
  /** Chemical sensor */
  Chemical = 'chemical'
}

/**
 * Power harvesting method
 */
export enum PowerHarvestingMethod {
  /** Solar cell integration */
  Solar = 'solar',
  /** Piezoelectric energy */
  Piezoelectric = 'piezoelectric',
  /** Thermoelectric conversion */
  Thermoelectric = 'thermoelectric',
  /** Triboelectric generation */
  Triboelectric = 'triboelectric',
  /** RF energy harvesting */
  RFHarvesting = 'rf_harvesting'
}

/**
 * Textile conductivity specifications
 */
export interface ConductivitySpec {
  /** Resistance per unit length (ohms/meter) */
  resistancePerMeter: number;
  /** Maximum current capacity (mA) */
  maxCurrent: number;
  /** Voltage rating (V) */
  voltageRating: number;
  /** Temperature coefficient */
  temperatureCoefficient: number;
}

/**
 * Washability specifications
 */
export interface WashabilitySpec {
  /** Maximum wash cycles */
  maxWashCycles: number;
  /** Maximum wash temperature (°C) */
  maxTemperature: number;
  /** Dry cleaning compatible */
  dryCleanable: boolean;
  /** Machine wash compatible */
  machineWashable: boolean;
  /** Conductivity retention after washing (%) */
  conductivityRetention: number;
}

/**
 * Stretch and flexibility specs
 */
export interface FlexibilitySpec {
  /** Maximum stretch percentage */
  maxStretch: number;
  /** Elastic recovery rate (%) */
  recoveryRate: number;
  /** Minimum bend radius (mm) */
  minBendRadius: number;
  /** Fatigue resistance (bend cycles) */
  fatigueResistance: number;
}

/**
 * Smart textile sensor configuration
 */
export interface SensorConfig {
  /** Sensor type */
  type: TextileSensorType;
  /** Sensor location on textile */
  location: {
    x: number;
    y: number;
    zone?: string;
  };
  /** Sampling rate (Hz) */
  samplingRate: number;
  /** Sensitivity setting */
  sensitivity: 'low' | 'medium' | 'high';
  /** Calibration data */
  calibration?: {
    offset: number;
    scale: number;
    lastCalibrated: string;
  };
}

/**
 * Sensor reading data
 */
export interface SensorReading {
  /** Sensor ID */
  sensorId: string;
  /** Sensor type */
  type: TextileSensorType;
  /** Raw value */
  rawValue: number;
  /** Processed value */
  processedValue: number;
  /** Unit of measurement */
  unit: string;
  /** Reading timestamp */
  timestamp: number;
  /** Data quality indicator (0-100) */
  quality: number;
}

/**
 * Power management configuration
 */
export interface PowerConfig {
  /** Power harvesting method */
  harvestingMethod?: PowerHarvestingMethod;
  /** Battery capacity (mAh) */
  batteryCapacity?: number;
  /** Current battery level (%) */
  batteryLevel?: number;
  /** Power consumption (mW) */
  powerConsumption: number;
  /** Sleep mode enabled */
  sleepModeEnabled: boolean;
  /** Sleep timeout (seconds) */
  sleepTimeout: number;
}

/**
 * Communication protocol configuration
 */
export interface CommunicationConfig {
  /** Protocol type */
  protocol: 'bluetooth' | 'ble' | 'nfc' | 'wifi' | 'thread' | 'zigbee';
  /** Device address */
  address: string;
  /** Connection status */
  connected: boolean;
  /** Signal strength (dBm) */
  signalStrength?: number;
  /** Data rate (kbps) */
  dataRate?: number;
}

/**
 * Textile zone definition
 */
export interface TextileZone {
  /** Zone identifier */
  id: string;
  /** Zone name */
  name: string;
  /** Zone boundaries */
  boundaries: {
    topLeft: { x: number; y: number };
    bottomRight: { x: number; y: number };
  };
  /** Sensors in this zone */
  sensors: string[];
  /** Zone functionality */
  functionality: 'sensing' | 'actuating' | 'display' | 'power';
}

/**
 * Complete smart textile specification
 */
export interface SmartTextileSpec {
  /** WIA standard identifier */
  standard: 'WIA-IND-002';
  /** Version */
  version: string;
  /** Textile ID */
  textileId: string;
  /** Category */
  category: SmartTextileCategory;
  /** Conductive properties */
  conductivity?: ConductivitySpec;
  /** Washability properties */
  washability: WashabilitySpec;
  /** Flexibility properties */
  flexibility: FlexibilitySpec;
  /** Embedded sensors */
  sensors: SensorConfig[];
  /** Power configuration */
  power: PowerConfig;
  /** Communication settings */
  communication: CommunicationConfig;
  /** Defined zones */
  zones: TextileZone[];
}

/**
 * Actuation command for smart textile
 */
export interface ActuationCommand {
  /** Target zone or component */
  target: string;
  /** Actuation type */
  type: 'heating' | 'cooling' | 'vibration' | 'led' | 'shape_change';
  /** Intensity (0-100) */
  intensity: number;
  /** Duration in milliseconds */
  duration: number;
  /** Pattern for activation */
  pattern?: 'constant' | 'pulse' | 'fade' | 'wave';
}

/**
 * Certification level for smart textiles
 */
export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

/**
 * Compliance test result
 */
export interface ComplianceReport {
  /** Standard identifier */
  standard: 'WIA-IND-002';
  /** Test date */
  testDate: string;
  /** Textile tested */
  textileId: string;
  /** Certification level achieved */
  certificationLevel: CertificationLevel;
  /** Individual tests */
  tests: {
    name: string;
    passed: boolean;
    value?: number;
    required?: number;
    unit?: string;
  }[];
  /** Overall compliance */
  compliant: boolean;
}

/**
 * Event types for smart textile SDK
 */
export type SmartTextileEventType =
  | 'sensor-data'
  | 'power-update'
  | 'connection-change'
  | 'actuation-complete'
  | 'error'
  | 'calibration-needed';

/**
 * Event callback type
 */
export type EventCallback<T = unknown> = (data: T) => void;
