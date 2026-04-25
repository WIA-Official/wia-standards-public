/**
 * WIA-SEMI-003 Power Semiconductor TypeScript Type Definitions
 *
 * @packageDocumentation
 */

/**
 * Standard identifier for WIA Power Semiconductor Standard
 */
export const WIA_SEMI_003 = 'WIA-SEMI-003';

/**
 * Device type enumeration
 */
export enum DeviceType {
  IGBT = 'IGBT',
  MOSFET = 'MOSFET',
  SIC_MOSFET = 'SiC_MOSFET',
  SIC_DIODE = 'SiC_Diode',
  GAN_HEMT = 'GaN_HEMT',
  SUPERJUNCTION_MOSFET = 'SuperJunction_MOSFET',
  POWER_MODULE = 'Power_Module'
}

/**
 * Package type enumeration
 */
export enum PackageType {
  TO_247 = 'TO-247',
  TO_247_4 = 'TO-247-4',
  TO_220 = 'TO-220',
  TO_263 = 'TO-263',
  MODULE = 'Module',
  D2PAK = 'D2PAK',
  DPAK = 'DPAK',
  SOT_227 = 'SOT-227'
}

/**
 * Certification level
 */
export enum CertificationLevel {
  LEVEL_1_DATA = 1,
  LEVEL_2_API = 2,
  LEVEL_3_PROTOCOL = 3,
  LEVEL_4_ECOSYSTEM = 4
}

/**
 * Test conditions for measurements
 */
export interface TestConditions {
  temperature: number;
  temperatureUnit: string;
  gateVoltage?: number;
  drainCurrent?: number;
  testStandard?: string;
  additionalConditions?: Record<string, any>;
}

/**
 * Measurement value with unit and test conditions
 */
export interface MeasurementValue {
  value: number;
  unit: string;
  min?: number;
  max?: number;
  typical?: number;
  testConditions?: TestConditions;
}

/**
 * Manufacturer information
 */
export interface Manufacturer {
  name: string;
  registryId: string;
  contact: string;
  website?: string;
  supportEmail?: string;
}

/**
 * Device metadata
 */
export interface DeviceMetadata {
  standardId: string;
  version: string;
  deviceId: string;
  manufacturer: Manufacturer;
  deviceType: DeviceType;
  packageType: PackageType;
  certificationLevel: CertificationLevel;
  certificationDate: string;
  datasheet: string;
  revisionHistory?: Array<{
    version: string;
    date: string;
    changes: string;
  }>;
}

/**
 * Voltage ratings
 */
export interface VoltageRatings {
  vds_max?: MeasurementValue;  // For MOSFETs
  vces_max?: MeasurementValue; // For IGBTs
  vgs_max?: MeasurementValue;
  vge_max?: MeasurementValue;
  breakdownVoltage?: MeasurementValue;
}

/**
 * Current ratings
 */
export interface CurrentRatings {
  continuousDrain: MeasurementValue;
  pulsedDrain: MeasurementValue;
  avalancheCurrent?: MeasurementValue;
  diodeCurrent?: MeasurementValue;
}

/**
 * On-state characteristics
 */
export interface OnStateCharacteristics {
  rds_on?: MeasurementValue;  // For MOSFETs
  vce_sat?: MeasurementValue; // For IGBTs
  temperatureCoefficient: number;
  currentDependency?: Array<{
    current: number;
    resistance: number;
  }>;
}

/**
 * Electrical characteristics
 */
export interface ElectricalCharacteristics {
  voltageRatings: VoltageRatings;
  currentRatings: CurrentRatings;
  onStateCharacteristics: OnStateCharacteristics;
  capacitances?: {
    ciss: MeasurementValue;
    coss: MeasurementValue;
    crss: MeasurementValue;
  };
  gateCharge?: {
    qg: MeasurementValue;
    qgs: MeasurementValue;
    qgd: MeasurementValue;
  };
}

/**
 * Foster thermal model parameters
 */
export interface FosterModelParameter {
  R: number;  // Thermal resistance (°C/W)
  tau: number;  // Time constant (seconds)
}

/**
 * Thermal impedance data
 */
export interface ThermalImpedance {
  junctionToCase: {
    steadyState: MeasurementValue;
    transient?: {
      model: 'Foster' | 'Cauer';
      parameters: FosterModelParameter[];
    };
  };
  junctionToAmbient?: MeasurementValue;
  caseToHeatsink?: MeasurementValue;
}

/**
 * Thermal characteristics
 */
export interface ThermalCharacteristics {
  maxJunctionTemperature: MeasurementValue;
  storageTemperatureRange: {
    min: number;
    max: number;
    unit: string;
  };
  thermalImpedance: ThermalImpedance;
}

/**
 * Switching characteristics
 */
export interface SwitchingCharacteristics {
  turnOnTime: MeasurementValue;
  turnOffTime: MeasurementValue;
  riseTime?: MeasurementValue;
  fallTime?: MeasurementValue;
  switchingEnergy: {
    turnOn: MeasurementValue;
    turnOff: MeasurementValue;
    total?: MeasurementValue;
  };
}

/**
 * Complete power semiconductor device specification
 */
export interface PowerSemiconductorDevice {
  metadata: DeviceMetadata;
  electrical: ElectricalCharacteristics;
  thermal: ThermalCharacteristics;
  switching: SwitchingCharacteristics;
  mechanical?: {
    dimensions: {
      length: number;
      width: number;
      height: number;
      unit: string;
    };
    weight?: MeasurementValue;
    pinout: Array<{
      pin: number;
      name: string;
      function: string;
    }>;
  };
}

/**
 * Real-time telemetry data
 */
export interface TelemetryData {
  timestamp: string;
  deviceId: string;
  measurements: {
    junctionTemperature?: number;
    caseTemperature?: number;
    drainCurrent?: number;
    drainVoltage?: number;
    gateVoltage?: number;
    powerDissipation?: number;
  };
  status: {
    operational: boolean;
    warningFlags?: string[];
    errorFlags?: string[];
  };
}

/**
 * Device control command
 */
export interface ControlCommand {
  deviceId: string;
  command: string;
  parameters?: Record<string, any>;
  timestamp: string;
}

/**
 * API response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: string;
}

/**
 * Configuration options for API client
 */
export interface ClientConfig {
  baseUrl: string;
  apiKey?: string;
  timeout?: number;
  retryAttempts?: number;
}
