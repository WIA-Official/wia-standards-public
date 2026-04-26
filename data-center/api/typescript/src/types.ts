/**
 * WIA-COMM-013: Data Center - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Data Center Infrastructure Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Data center tier levels
 */
export type TierLevel = 'I' | 'II' | 'III' | 'IV';

/**
 * Redundancy configurations
 */
export type RedundancyConfig = 'N' | 'N+1' | '2N' | '2N+1';

/**
 * Cooling system types
 */
export type CoolingType = 'CRAC' | 'CRAH' | 'in-row' | 'liquid' | 'immersion';

/**
 * Geographic coordinates
 */
export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
  city?: string;
  country?: string;
}

// ============================================================================
// Tier Classification
// ============================================================================

/**
 * Tier requirements specification
 */
export interface TierRequirements {
  /** Tier level */
  tier: TierLevel;

  /** Number of power distribution paths */
  powerPaths: number;

  /** Redundancy configuration */
  redundancy: RedundancyConfig;

  /** Supports concurrent maintenance */
  concurrentMaintenance: boolean;

  /** Fault tolerant capability */
  faultTolerant: boolean;

  /** Expected availability (%) */
  availability: number;

  /** Expected annual downtime (hours) */
  annualDowntime: number;
}

/**
 * Tier compliance validation result
 */
export interface TierCompliance {
  /** Is facility compliant? */
  isCompliant: boolean;

  /** Tier level evaluated */
  tier: TierLevel;

  /** Validation checks performed */
  checks: {
    name: string;
    passed: boolean;
    required: string;
    actual: string;
  }[];

  /** Overall score (0-100) */
  score: number;

  /** Recommendations for compliance */
  recommendations: string[];
}

// ============================================================================
// Power Infrastructure
// ============================================================================

/**
 * Power capacity parameters
 */
export interface PowerCapacityParams {
  /** Number of racks */
  rackCount: number;

  /** Average power per rack (kW) */
  avgPowerPerRack: number;

  /** Redundancy configuration */
  redundancy: RedundancyConfig;

  /** UPS efficiency (0-1) */
  upsEfficiency: number;

  /** Expected growth rate (0-1) */
  growthRate?: number;

  /** Planning horizon (years) */
  planningYears?: number;
}

/**
 * Power capacity calculation result
 */
export interface PowerCapacityResult {
  /** Total IT load (kW) */
  itLoad: number;

  /** Total capacity with redundancy (kW) */
  totalCapacity: number;

  /** UPS capacity required (kVA) */
  upsCapacity: number;

  /** Generator capacity required (kW) */
  generatorCapacity: number;

  /** Number of UPS modules (N+1) */
  upsModules: number;

  /** Number of generators (N+1) */
  generators: number;

  /** Projected capacity in planning horizon (kW) */
  projectedCapacity?: number;
}

/**
 * UPS configuration
 */
export interface UPSConfig {
  /** UPS identifier */
  id: string;

  /** Capacity (kVA) */
  capacity: number;

  /** Efficiency (%) */
  efficiency: number;

  /** Battery runtime (minutes) */
  batteryRuntime: number;

  /** Topology */
  topology: 'online' | 'line-interactive' | 'standby';

  /** Output voltage (V) */
  outputVoltage: number;

  /** Input voltage (V) */
  inputVoltage: number;

  /** Status */
  status: 'online' | 'on-battery' | 'bypass' | 'fault';
}

/**
 * Generator configuration
 */
export interface GeneratorConfig {
  /** Generator identifier */
  id: string;

  /** Capacity (kW) */
  capacity: number;

  /** Fuel type */
  fuelType: 'diesel' | 'natural-gas' | 'propane' | 'dual-fuel';

  /** Fuel consumption (gal/hr at full load) */
  fuelConsumption: number;

  /** Fuel tank capacity (gallons) */
  fuelTankCapacity: number;

  /** Runtime at full load (hours) */
  runtime: number;

  /** Startup time (seconds) */
  startupTime: number;

  /** Status */
  status: 'standby' | 'running' | 'fault' | 'maintenance';
}

/**
 * PDU (Power Distribution Unit) configuration
 */
export interface PDUConfig {
  /** PDU identifier */
  id: string;

  /** Rack location */
  rackId: string;

  /** Input voltage (V) */
  inputVoltage: number;

  /** Output voltage (V) */
  outputVoltage: number;

  /** Number of outlets */
  outlets: number;

  /** Total capacity (kW) */
  capacity: number;

  /** Current load (kW) */
  currentLoad: number;

  /** Utilization (%) */
  utilization: number;

  /** Metering capability */
  metered: boolean;

  /** Switched outlets */
  switched: boolean;
}

// ============================================================================
// Cooling and Thermal Management
// ============================================================================

/**
 * PUE (Power Usage Effectiveness) calculation parameters
 */
export interface PUEParams {
  /** IT equipment load (kW) */
  itLoad: number;

  /** Cooling load (kW) */
  coolingLoad: number;

  /** Lighting load (kW) */
  lightingLoad: number;

  /** UPS losses (kW) */
  upsLosses: number;

  /** Other facility loads (kW) */
  otherLoads?: number;
}

/**
 * PUE calculation result
 */
export interface PUEResult {
  /** Total facility power (kW) */
  totalFacilityPower: number;

  /** IT equipment power (kW) */
  itEquipmentPower: number;

  /** PUE value */
  pue: number;

  /** DCiE (Data Center infrastructure Efficiency) % */
  dcie: number;

  /** Efficiency rating */
  rating: 'excellent' | 'good' | 'fair' | 'poor';

  /** Breakdown by component */
  breakdown: {
    it: number;
    cooling: number;
    lighting: number;
    ups: number;
    other: number;
  };
}

/**
 * Cooling capacity parameters
 */
export interface CoolingCapacityParams {
  /** IT load (kW) */
  itLoad: number;

  /** Target PUE */
  targetPUE: number;

  /** Ambient temperature (°C) */
  ambientTemp: number;

  /** Cooling type */
  coolingType: CoolingType;

  /** Redundancy configuration */
  redundancy: RedundancyConfig;
}

/**
 * Cooling capacity result
 */
export interface CoolingCapacityResult {
  /** Cooling load (kW) */
  coolingLoad: number;

  /** Cooling capacity in tons */
  coolingTons: number;

  /** Required airflow (CFM) */
  airflow: number;

  /** Number of cooling units (N) */
  coolingUnits: number;

  /** Total units with redundancy */
  totalUnits: number;

  /** Estimated COP (Coefficient of Performance) */
  cop: number;
}

/**
 * CRAC/CRAH unit configuration
 */
export interface CoolingUnitConfig {
  /** Unit identifier */
  id: string;

  /** Type */
  type: CoolingType;

  /** Capacity (tons or kW) */
  capacity: number;

  /** Supply temperature (°C) */
  supplyTemp: number;

  /** Return temperature (°C) */
  returnTemp: number;

  /** Airflow (CFM) */
  airflow: number;

  /** COP (Coefficient of Performance) */
  cop: number;

  /** Status */
  status: 'running' | 'standby' | 'fault' | 'maintenance';

  /** Variable speed fan */
  variableSpeed: boolean;
}

/**
 * Thermal monitoring data
 */
export interface ThermalData {
  /** Timestamp */
  timestamp: Date;

  /** Cold aisle temperature (°C) */
  coldAisleTemp: number;

  /** Hot aisle temperature (°C) */
  hotAisleTemp: number;

  /** Temperature differential (°C) */
  tempDifferential: number;

  /** Humidity (%) */
  humidity: number;

  /** Rack location */
  rackId: string;

  /** Sensor height (U position) */
  sensorHeight: number;

  /** Alarm status */
  alarm: boolean;
}

// ============================================================================
// Rack Design and Layout
// ============================================================================

/**
 * Rack specification
 */
export interface RackSpec {
  /** Rack identifier */
  id: string;

  /** Height in U (rack units) */
  height: number;

  /** Width (inches) */
  width: number;

  /** Depth (inches) */
  depth: number;

  /** Weight capacity (lbs) */
  weightCapacity: number;

  /** Power capacity (kW) */
  powerCapacity: number;

  /** Current power draw (kW) */
  currentPower: number;

  /** Power density classification */
  densityClass: 'low' | 'medium' | 'high' | 'very-high' | 'ultra-high';
}

/**
 * Rack layout design parameters
 */
export interface RackLayoutParams {
  /** Total number of racks */
  rackCount: number;

  /** Average power per rack (kW) */
  avgPowerPerRack: number;

  /** Floor area (sq ft) */
  floorArea: number;

  /** Enable hot/cold aisle containment */
  aisleContainment?: boolean;

  /** Cold aisle width (ft) */
  coldAisleWidth?: number;

  /** Hot aisle width (ft) */
  hotAisleWidth?: number;
}

/**
 * Rack layout design result
 */
export interface RackLayoutResult {
  /** Number of rows */
  rows: number;

  /** Racks per row */
  racksPerRow: number;

  /** Total IT load (kW) */
  totalITLoad: number;

  /** Total rack footprint (sq ft) */
  rackFootprint: number;

  /** Floor utilization (%) */
  floorUtilization: number;

  /** Power density (kW/1000 sq ft) */
  powerDensity: number;

  /** Recommended cooling approach */
  coolingRecommendation: string;

  /** Layout diagram (ASCII) */
  layoutDiagram?: string;
}

// ============================================================================
// Network Infrastructure
// ============================================================================

/**
 * Network topology types
 */
export type NetworkTopology = 'spine-leaf' | 'core-aggregation-access' | 'mesh' | 'ring';

/**
 * Network switch configuration
 */
export interface NetworkSwitch {
  /** Switch identifier */
  id: string;

  /** Switch role */
  role: 'spine' | 'leaf' | 'tor' | 'core' | 'access';

  /** Port count */
  ports: number;

  /** Port speed (Gbps) */
  portSpeed: number;

  /** Uplink ports */
  uplinks: number;

  /** Uplink speed (Gbps) */
  uplinkSpeed: number;

  /** Total switching capacity (Tbps) */
  capacity: number;

  /** Rack location */
  rackId?: string;

  /** Redundant pair */
  redundantPair?: string;
}

// ============================================================================
// Physical Security
// ============================================================================

/**
 * Security access levels
 */
export type AccessLevel = 'public' | 'office' | 'viewing' | 'cage' | 'rack';

/**
 * Authentication methods
 */
export type AuthMethod = 'keycard' | 'pin' | 'biometric' | 'escort';

/**
 * Access control configuration
 */
export interface AccessControl {
  /** Zone identifier */
  zoneId: string;

  /** Zone name */
  zoneName: string;

  /** Access level */
  accessLevel: AccessLevel;

  /** Required authentication methods */
  requiredAuth: AuthMethod[];

  /** Multi-factor authentication */
  mfa: boolean;

  /** Mantrap/airlock */
  mantrap: boolean;

  /** CCTV coverage */
  cctv: boolean;

  /** Access log retention (days) */
  logRetention: number;
}

/**
 * Surveillance camera configuration
 */
export interface SurveillanceCamera {
  /** Camera identifier */
  id: string;

  /** Location */
  location: string;

  /** Camera type */
  type: 'fixed' | 'ptz' | '360';

  /** Resolution */
  resolution: '1080p' | '4K' | '8K';

  /** Recording */
  recording: boolean;

  /** Retention period (days) */
  retentionDays: number;

  /** Night vision */
  nightVision: boolean;

  /** Motion detection */
  motionDetection: boolean;
}

// ============================================================================
// Fire Suppression
// ============================================================================

/**
 * Fire suppression system types
 */
export type SuppressionType = 'FM-200' | 'Novec-1230' | 'Inergen' | 'pre-action' | 'dry-pipe';

/**
 * Fire suppression configuration
 */
export interface FireSuppression {
  /** System identifier */
  id: string;

  /** System type */
  type: SuppressionType;

  /** Protected zone */
  zone: string;

  /** Discharge time (seconds) */
  dischargeTime: number;

  /** Agent concentration (%) */
  concentration: number;

  /** Safe for electronics */
  electronicsSafe: boolean;

  /** Safe for occupied spaces */
  occupiedSafe: boolean;

  /** EPO (Emergency Power Off) */
  epoIntegration: boolean;

  /** Last test date */
  lastTest?: Date;
}

/**
 * Early warning fire detection
 */
export interface FireDetection {
  /** Detector identifier */
  id: string;

  /** Detection type */
  type: 'VESDA' | 'smoke' | 'heat' | 'flame';

  /** Sensitivity level */
  sensitivity: 'low' | 'medium' | 'high' | 'very-high';

  /** Alert levels */
  alertLevels: ('alert' | 'action' | 'fire1' | 'fire2')[];

  /** Current status */
  status: 'normal' | 'alert' | 'alarm' | 'fault';

  /** Last maintenance */
  lastMaintenance?: Date;
}

// ============================================================================
// DCIM and Monitoring
// ============================================================================

/**
 * DCIM platform features
 */
export interface DCIMFeatures {
  /** Asset management */
  assetManagement: boolean;

  /** Capacity planning */
  capacityPlanning: boolean;

  /** Environmental monitoring */
  environmentalMonitoring: boolean;

  /** Power monitoring */
  powerMonitoring: boolean;

  /** 3D visualization */
  visualization3D: boolean;

  /** Change management */
  changeManagement: boolean;

  /** Workflow automation */
  workflowAutomation: boolean;

  /** Reporting and analytics */
  reporting: boolean;
}

/**
 * Environmental sensor data
 */
export interface EnvironmentalSensor {
  /** Sensor identifier */
  id: string;

  /** Sensor type */
  type: 'temperature' | 'humidity' | 'airflow' | 'water-leak';

  /** Location */
  location: string;

  /** Current reading */
  value: number;

  /** Unit of measurement */
  unit: string;

  /** Threshold minimum */
  thresholdMin?: number;

  /** Threshold maximum */
  thresholdMax?: number;

  /** Alarm status */
  alarm: boolean;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Asset tracking information
 */
export interface DataCenterAsset {
  /** Asset identifier */
  id: string;

  /** Asset type */
  type: 'server' | 'switch' | 'storage' | 'ups' | 'pdu' | 'cooling' | 'other';

  /** Manufacturer */
  manufacturer: string;

  /** Model */
  model: string;

  /** Serial number */
  serialNumber: string;

  /** Rack location */
  rackId: string;

  /** U position (start) */
  uPosition: number;

  /** Height in U */
  height: number;

  /** Power consumption (kW) */
  powerDraw: number;

  /** Installation date */
  installDate: Date;

  /** Warranty expiration */
  warrantyExpiry?: Date;

  /** Status */
  status: 'active' | 'standby' | 'maintenance' | 'decommissioned';
}

// ============================================================================
// Edge Data Centers
// ============================================================================

/**
 * Edge data center types
 */
export type EdgeType = 'micro' | 'modular' | 'mini';

/**
 * Edge data center configuration
 */
export interface EdgeDataCenter {
  /** Facility identifier */
  id: string;

  /** Edge type */
  edgeType: EdgeType;

  /** IT capacity (kW) */
  itCapacity: number;

  /** Location */
  location: GeoLocation;

  /** Form factor */
  formFactor: 'rack' | 'container' | 'building';

  /** Remote management */
  remoteManagement: boolean;

  /** Lights-out operation */
  lightsOut: boolean;

  /** Operating temperature range (°C) */
  tempRange: {
    min: number;
    max: number;
  };

  /** Cellular backup */
  cellularBackup: boolean;

  /** Target latency (ms) */
  targetLatency: number;
}

// ============================================================================
// Green Data Centers
// ============================================================================

/**
 * Renewable energy sources
 */
export type RenewableSource = 'solar' | 'wind' | 'hydro' | 'fuel-cell' | 'geothermal';

/**
 * Renewable energy configuration
 */
export interface RenewableEnergy {
  /** Source type */
  source: RenewableSource;

  /** Capacity (kW) */
  capacity: number;

  /** Annual generation (kWh) */
  annualGeneration: number;

  /** Percentage of facility load */
  loadPercentage: number;

  /** Carbon offset (tons CO2/year) */
  carbonOffset: number;

  /** Grid-tied */
  gridTied: boolean;

  /** Battery storage */
  batteryStorage?: {
    capacity: number; // kWh
    power: number; // kW
  };
}

/**
 * Water usage effectiveness
 */
export interface WaterUsage {
  /** WUE (Water Usage Effectiveness) L/kWh */
  wue: number;

  /** Annual water consumption (gallons) */
  annualConsumption: number;

  /** Water source */
  source: 'municipal' | 'well' | 'rainwater' | 'recycled';

  /** Cooling technology */
  coolingTech: 'evaporative' | 'air-cooled' | 'closed-loop';

  /** Water recycling */
  recycling: boolean;
}

// ============================================================================
// Disaster Recovery
// ============================================================================

/**
 * Recovery objectives
 */
export interface RecoveryObjectives {
  /** RTO - Recovery Time Objective (hours) */
  rto: number;

  /** RPO - Recovery Point Objective (minutes) */
  rpo: number;

  /** Geographic redundancy */
  geoRedundant: boolean;

  /** Distance between sites (miles) */
  siteDistance?: number;

  /** Replication type */
  replication: 'synchronous' | 'asynchronous' | 'snapshot';

  /** Failover automation */
  autoFailover: boolean;
}

/**
 * Disaster recovery site
 */
export interface DRSite {
  /** Site identifier */
  id: string;

  /** Site type */
  type: 'hot' | 'warm' | 'cold';

  /** Location */
  location: GeoLocation;

  /** Capacity (kW) */
  capacity: number;

  /** Network connectivity */
  connectivity: {
    bandwidth: number; // Gbps
    latency: number; // ms
    redundant: boolean;
  };

  /** Last failover test */
  lastTest?: Date;

  /** Test frequency (months) */
  testFrequency: number;
}

// ============================================================================
// Cost Estimation
// ============================================================================

/**
 * CapEx (Capital Expenditure) breakdown
 */
export interface CapExBreakdown {
  /** Electrical infrastructure */
  electrical: number;

  /** Mechanical infrastructure (cooling) */
  mechanical: number;

  /** Construction/building */
  construction: number;

  /** Network infrastructure */
  network: number;

  /** Security and fire suppression */
  security: number;

  /** Total CapEx */
  total: number;

  /** Cost per kW */
  costPerKW: number;
}

/**
 * OpEx (Operating Expenditure) breakdown
 */
export interface OpExBreakdown {
  /** Annual electricity cost */
  electricity: number;

  /** Staffing costs */
  staffing: number;

  /** Maintenance and repairs */
  maintenance: number;

  /** Network/connectivity costs */
  connectivity?: number;

  /** Total annual OpEx */
  total: number;

  /** Cost per kW per year */
  costPerKWYear: number;
}

/**
 * TCO (Total Cost of Ownership)
 */
export interface TCOAnalysis {
  /** CapEx */
  capex: CapExBreakdown;

  /** Annual OpEx */
  opex: OpExBreakdown;

  /** Analysis period (years) */
  years: number;

  /** Total cost over period */
  totalCost: number;

  /** Cost per kW over period */
  costPerKW: number;
}

// ============================================================================
// Physical Constants and Standards
// ============================================================================

/**
 * Data center physical constants
 */
export const DATACENTER_CONSTANTS = {
  /** Standard rack height (U) */
  STANDARD_RACK_HEIGHT: 42,

  /** Standard rack width (inches) */
  STANDARD_RACK_WIDTH: 19,

  /** Cooling conversion: 1 ton = 3.517 kW */
  TON_TO_KW: 3.517,

  /** CFM per kW (typical airflow requirement) */
  CFM_PER_KW: 180,

  /** Watts per U for low density */
  LOW_DENSITY_WATTS_PER_U: 100,

  /** Watts per U for high density */
  HIGH_DENSITY_WATTS_PER_U: 500,
} as const;

/**
 * Tier availability standards
 */
export const TIER_STANDARDS: Record<TierLevel, TierRequirements> = {
  I: {
    tier: 'I',
    powerPaths: 1,
    redundancy: 'N',
    concurrentMaintenance: false,
    faultTolerant: false,
    availability: 99.671,
    annualDowntime: 28.8,
  },
  II: {
    tier: 'II',
    powerPaths: 1,
    redundancy: 'N+1',
    concurrentMaintenance: false,
    faultTolerant: false,
    availability: 99.741,
    annualDowntime: 22.0,
  },
  III: {
    tier: 'III',
    powerPaths: 2,
    redundancy: 'N+1',
    concurrentMaintenance: true,
    faultTolerant: false,
    availability: 99.982,
    annualDowntime: 1.6,
  },
  IV: {
    tier: 'IV',
    powerPaths: 2,
    redundancy: '2N',
    concurrentMaintenance: true,
    faultTolerant: true,
    availability: 99.995,
    annualDowntime: 0.4,
  },
};

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
// Error Types
// ============================================================================

/**
 * WIA-COMM-013 error codes
 */
export enum DCErrorCode {
  INVALID_TIER = 'DC001',
  INSUFFICIENT_POWER = 'DC002',
  INSUFFICIENT_COOLING = 'DC003',
  INVALID_REDUNDANCY = 'DC004',
  COMPLIANCE_FAILURE = 'DC005',
  CAPACITY_EXCEEDED = 'DC006',
  INVALID_LAYOUT = 'DC007',
  MONITORING_FAILURE = 'DC008',
  INVALID_PARAMETERS = 'DC009',
  CALCULATION_ERROR = 'DC010',
}

/**
 * Data center infrastructure error
 */
export class DataCenterError extends Error {
  constructor(
    public code: DCErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'DataCenterError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  TierLevel,
  RedundancyConfig,
  CoolingType,
  GeoLocation,
  TierRequirements,
  TierCompliance,
  PowerCapacityParams,
  PowerCapacityResult,
  UPSConfig,
  GeneratorConfig,
  PDUConfig,
  PUEParams,
  PUEResult,
  CoolingCapacityParams,
  CoolingCapacityResult,
  CoolingUnitConfig,
  ThermalData,
  RackSpec,
  RackLayoutParams,
  RackLayoutResult,
  NetworkTopology,
  NetworkSwitch,
  AccessLevel,
  AuthMethod,
  AccessControl,
  SurveillanceCamera,
  SuppressionType,
  FireSuppression,
  FireDetection,
  DCIMFeatures,
  EnvironmentalSensor,
  DataCenterAsset,
  EdgeType,
  EdgeDataCenter,
  RenewableSource,
  RenewableEnergy,
  WaterUsage,
  RecoveryObjectives,
  DRSite,
  CapExBreakdown,
  OpExBreakdown,
  TCOAnalysis,
};

export { DATACENTER_CONSTANTS, TIER_STANDARDS, DCErrorCode, DataCenterError };
