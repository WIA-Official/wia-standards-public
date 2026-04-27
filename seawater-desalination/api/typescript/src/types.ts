/**
 * WIA-ENV-002 Seawater Desalination SDK Type Definitions
 *
 * Comprehensive type definitions for desalination plant management,
 * water quality monitoring, and reverse osmosis system control.
 *
 * @packageDocumentation
 * @module @wia/seawater-desalination
 * @version 1.0.0
 */

// ============================================================================
// Core Identifier Types
// ============================================================================

/** Unique identifier for desalination plants */
export type PlantId = string;

/** Unique identifier for treatment trains */
export type TrainId = string;

/** Unique identifier for membrane modules */
export type MembraneId = string;

/** Unique identifier for water quality samples */
export type SampleId = string;

/** ISO 8601 formatted timestamp string */
export type ISOTimestamp = string;

// ============================================================================
// Desalination Process Types
// ============================================================================

/** Supported desalination technologies */
export type DesalinationTechnology =
  | 'reverse_osmosis'
  | 'multi_stage_flash'
  | 'multi_effect_distillation'
  | 'electrodialysis'
  | 'forward_osmosis'
  | 'membrane_distillation'
  | 'hybrid';

/** Plant operational status */
export type PlantStatus =
  | 'operational'
  | 'standby'
  | 'maintenance'
  | 'cleaning'
  | 'shutdown'
  | 'emergency';

/** Membrane condition status */
export type MembraneStatus =
  | 'optimal'
  | 'good'
  | 'degraded'
  | 'fouled'
  | 'damaged'
  | 'replacement_needed';

// ============================================================================
// Plant Configuration Types
// ============================================================================

/**
 * Desalination plant configuration
 */
export interface DesalinationPlant {
  /** Unique plant identifier */
  id: PlantId;

  /** Plant name */
  name: string;

  /** Plant location */
  location: PlantLocation;

  /** Desalination technology used */
  technology: DesalinationTechnology;

  /** Plant capacity specifications */
  capacity: PlantCapacity;

  /** Treatment train configurations */
  trains: TreatmentTrain[];

  /** Current operational status */
  status: PlantStatus;

  /** Plant metadata */
  metadata: PlantMetadata;

  /** Energy configuration */
  energy: EnergyConfig;
}

/**
 * Plant geographic location
 */
export interface PlantLocation {
  /** Latitude in decimal degrees */
  latitude: number;

  /** Longitude in decimal degrees */
  longitude: number;

  /** Country code (ISO 3166-1 alpha-2) */
  countryCode: string;

  /** City or region name */
  city: string;

  /** Coastal or inland location */
  locationType: 'coastal' | 'inland';

  /** Water source type */
  waterSource: WaterSourceType;
}

/** Types of water sources for desalination */
export type WaterSourceType =
  | 'seawater'
  | 'brackish_groundwater'
  | 'brackish_surface'
  | 'industrial_wastewater'
  | 'municipal_wastewater';

/**
 * Plant production capacity specifications
 */
export interface PlantCapacity {
  /** Design capacity in cubic meters per day */
  designCapacityM3Day: number;

  /** Current operating capacity in cubic meters per day */
  operatingCapacityM3Day: number;

  /** Maximum capacity in cubic meters per day */
  maxCapacityM3Day: number;

  /** Recovery rate percentage */
  recoveryRatePercent: number;

  /** Number of treatment trains */
  numberOfTrains: number;
}

/**
 * Treatment train configuration
 */
export interface TreatmentTrain {
  /** Train identifier */
  id: TrainId;

  /** Train name */
  name: string;

  /** Train capacity in m3/day */
  capacityM3Day: number;

  /** Number of stages */
  stages: number;

  /** Membrane configuration */
  membranes: MembraneConfig;

  /** Current train status */
  status: PlantStatus;

  /** Pretreatment systems */
  pretreatment: PretreatmentSystem[];

  /** Post-treatment systems */
  postTreatment: PostTreatmentSystem[];
}

/**
 * Membrane module configuration
 */
export interface MembraneConfig {
  /** Membrane type */
  type: MembraneType;

  /** Membrane manufacturer */
  manufacturer: string;

  /** Membrane model */
  model: string;

  /** Number of elements per vessel */
  elementsPerVessel: number;

  /** Number of vessels */
  numberOfVessels: number;

  /** Active membrane area in square meters */
  activeAreaM2: number;

  /** Membrane age in days */
  ageInDays: number;

  /** Current membrane status */
  status: MembraneStatus;

  /** Salt rejection rate percentage */
  saltRejectionPercent: number;
}

/** Membrane types for reverse osmosis */
export type MembraneType =
  | 'spiral_wound'
  | 'hollow_fiber'
  | 'tubular'
  | 'plate_and_frame';

/**
 * Pretreatment system configuration
 */
export interface PretreatmentSystem {
  /** System type */
  type: PretreatmentType;

  /** System capacity */
  capacityM3Hour: number;

  /** Current status */
  status: 'active' | 'standby' | 'maintenance';

  /** Chemical dosing if applicable */
  chemicalDosing?: ChemicalDosing;
}

/** Types of pretreatment processes */
export type PretreatmentType =
  | 'intake_screen'
  | 'coagulation'
  | 'flocculation'
  | 'sedimentation'
  | 'dissolved_air_flotation'
  | 'multimedia_filter'
  | 'ultrafiltration'
  | 'microfiltration'
  | 'cartridge_filter'
  | 'antiscalant_dosing'
  | 'acid_dosing'
  | 'chlorination'
  | 'dechlorination';

/**
 * Post-treatment system configuration
 */
export interface PostTreatmentSystem {
  /** System type */
  type: PostTreatmentType;

  /** System capacity */
  capacityM3Hour: number;

  /** Current status */
  status: 'active' | 'standby' | 'maintenance';
}

/** Types of post-treatment processes */
export type PostTreatmentType =
  | 'remineralization'
  | 'ph_adjustment'
  | 'disinfection'
  | 'fluoridation'
  | 'blending'
  | 'storage'
  | 'pumping';

/**
 * Chemical dosing configuration
 */
export interface ChemicalDosing {
  /** Chemical name */
  chemical: string;

  /** Dosing rate in mg/L */
  dosageRateMgL: number;

  /** Chemical tank level percentage */
  tankLevelPercent: number;

  /** Pump status */
  pumpStatus: 'running' | 'stopped' | 'fault';
}

/**
 * Plant metadata information
 */
export interface PlantMetadata {
  /** Plant commissioning date */
  commissioningDate: ISOTimestamp;

  /** Operating organization */
  operator: string;

  /** Plant owner */
  owner: string;

  /** Design consultant */
  designConsultant?: string;

  /** EPC contractor */
  epcContractor?: string;

  /** Last major upgrade date */
  lastUpgrade?: ISOTimestamp;

  /** Regulatory permits */
  permits: OperatingPermit[];
}

/**
 * Operating permit information
 */
export interface OperatingPermit {
  /** Permit type */
  type: 'discharge' | 'intake' | 'environmental' | 'operating';

  /** Permit number */
  permitNumber: string;

  /** Issuing authority */
  authority: string;

  /** Issue date */
  issueDate: ISOTimestamp;

  /** Expiry date */
  expiryDate: ISOTimestamp;
}

/**
 * Energy configuration for the plant
 */
export interface EnergyConfig {
  /** Primary power source */
  primarySource: EnergySource;

  /** Backup power source */
  backupSource?: EnergySource;

  /** Energy recovery devices */
  energyRecovery: EnergyRecoveryDevice[];

  /** Specific energy consumption target (kWh/m3) */
  targetSecKwhM3: number;

  /** Renewable energy integration */
  renewablePercent: number;
}

/** Types of energy sources */
export type EnergySource =
  | 'grid'
  | 'solar'
  | 'wind'
  | 'natural_gas'
  | 'diesel'
  | 'nuclear'
  | 'hybrid';

/**
 * Energy recovery device configuration
 */
export interface EnergyRecoveryDevice {
  /** Device type */
  type: 'pressure_exchanger' | 'turbocharger' | 'pelton_wheel' | 'erd_pump';

  /** Manufacturer */
  manufacturer: string;

  /** Model */
  model: string;

  /** Energy recovery efficiency percentage */
  efficiencyPercent: number;

  /** Capacity in m3/hour */
  capacityM3Hour: number;
}

// ============================================================================
// Water Quality Types
// ============================================================================

/**
 * Water quality measurement
 */
export interface WaterQuality {
  /** Sample identifier */
  id: SampleId;

  /** Sample location */
  location: SampleLocation;

  /** Sample timestamp */
  timestamp: ISOTimestamp;

  /** Physical parameters */
  physical: PhysicalParameters;

  /** Chemical parameters */
  chemical: ChemicalParameters;

  /** Microbiological parameters */
  microbiological?: MicrobiologicalParameters;

  /** Quality compliance status */
  compliance: ComplianceStatus;
}

/** Sample location types */
export type SampleLocation =
  | 'intake'
  | 'pretreatment_inlet'
  | 'pretreatment_outlet'
  | 'ro_feed'
  | 'ro_permeate'
  | 'ro_concentrate'
  | 'post_treatment'
  | 'product_water'
  | 'distribution';

/**
 * Physical water quality parameters
 */
export interface PhysicalParameters {
  /** Temperature in Celsius */
  temperatureC: number;

  /** Turbidity in NTU */
  turbidityNTU: number;

  /** Total dissolved solids in mg/L */
  tdsMgL: number;

  /** Electrical conductivity in uS/cm */
  conductivityUSCm: number;

  /** pH value */
  pH: number;

  /** Silt density index */
  sdi15?: number;
}

/**
 * Chemical water quality parameters
 */
export interface ChemicalParameters {
  /** Chloride concentration in mg/L */
  chlorideMgL: number;

  /** Sodium concentration in mg/L */
  sodiumMgL: number;

  /** Calcium concentration in mg/L */
  calciumMgL: number;

  /** Magnesium concentration in mg/L */
  magnesiumMgL: number;

  /** Sulfate concentration in mg/L */
  sulfateMgL: number;

  /** Bicarbonate concentration in mg/L */
  bicarbonateMgL: number;

  /** Boron concentration in mg/L */
  boronMgL: number;

  /** Silica concentration in mg/L */
  silicaMgL: number;

  /** Free chlorine in mg/L */
  freeChlorineMgL?: number;

  /** Total organic carbon in mg/L */
  tocMgL?: number;
}

/**
 * Microbiological water quality parameters
 */
export interface MicrobiologicalParameters {
  /** Total coliform count per 100mL */
  totalColiformPer100mL: number;

  /** E. coli count per 100mL */
  eColiPer100mL: number;

  /** Heterotrophic plate count per mL */
  hpcPerML: number;

  /** Legionella presence */
  legionellaDetected: boolean;
}

/**
 * Compliance status with regulations
 */
export interface ComplianceStatus {
  /** Overall compliance */
  isCompliant: boolean;

  /** Applicable standard */
  standard: 'WHO' | 'EPA' | 'EU' | 'local';

  /** Non-compliant parameters */
  violations: ParameterViolation[];
}

/**
 * Parameter violation record
 */
export interface ParameterViolation {
  /** Parameter name */
  parameter: string;

  /** Measured value */
  measuredValue: number;

  /** Limit value */
  limitValue: number;

  /** Unit of measurement */
  unit: string;

  /** Violation severity */
  severity: 'minor' | 'major' | 'critical';
}

// ============================================================================
// Performance Metrics Types
// ============================================================================

/**
 * Plant performance metrics
 */
export interface PlantPerformance {
  /** Plant identifier */
  plantId: PlantId;

  /** Reporting period */
  period: ReportingPeriod;

  /** Production metrics */
  production: ProductionMetrics;

  /** Energy metrics */
  energy: EnergyMetrics;

  /** Quality metrics */
  quality: QualityMetrics;

  /** Efficiency metrics */
  efficiency: EfficiencyMetrics;
}

/**
 * Reporting period definition
 */
export interface ReportingPeriod {
  /** Start timestamp */
  start: ISOTimestamp;

  /** End timestamp */
  end: ISOTimestamp;

  /** Period type */
  type: 'hourly' | 'daily' | 'weekly' | 'monthly' | 'yearly';
}

/**
 * Production metrics
 */
export interface ProductionMetrics {
  /** Total water produced in m3 */
  totalProductionM3: number;

  /** Average daily production in m3/day */
  avgDailyProductionM3: number;

  /** Peak daily production in m3/day */
  peakDailyProductionM3: number;

  /** Plant availability percentage */
  availabilityPercent: number;

  /** Operating hours */
  operatingHours: number;
}

/**
 * Energy consumption metrics
 */
export interface EnergyMetrics {
  /** Total energy consumed in kWh */
  totalEnergyKwh: number;

  /** Specific energy consumption in kWh/m3 */
  specificEnergyKwhM3: number;

  /** High pressure pump energy in kWh */
  hpPumpEnergyKwh: number;

  /** Pretreatment energy in kWh */
  pretreatmentEnergyKwh: number;

  /** Energy recovered in kWh */
  energyRecoveredKwh: number;

  /** Peak demand in kW */
  peakDemandKw: number;
}

/**
 * Water quality metrics
 */
export interface QualityMetrics {
  /** Average product TDS in mg/L */
  avgProductTdsMgL: number;

  /** Average product conductivity in uS/cm */
  avgConductivityUSCm: number;

  /** Compliance rate percentage */
  complianceRatePercent: number;

  /** Number of quality exceedances */
  exceedanceCount: number;
}

/**
 * Plant efficiency metrics
 */
export interface EfficiencyMetrics {
  /** Overall recovery rate percentage */
  recoveryRatePercent: number;

  /** Salt rejection rate percentage */
  saltRejectionPercent: number;

  /** Normalized permeate flow */
  normalizedPermeateFlow: number;

  /** Normalized salt passage */
  normalizedSaltPassage: number;

  /** Membrane differential pressure in bar */
  membraneDeltaPressureBar: number;
}

// ============================================================================
// Alarm and Alert Types
// ============================================================================

/**
 * Plant alarm configuration
 */
export interface AlarmConfig {
  /** Alarm identifier */
  id: string;

  /** Alarm name */
  name: string;

  /** Parameter to monitor */
  parameter: string;

  /** Alarm thresholds */
  thresholds: AlarmThresholds;

  /** Alarm priority */
  priority: AlarmPriority;

  /** Notification settings */
  notifications: NotificationSettings;

  /** Enabled status */
  enabled: boolean;
}

/**
 * Alarm threshold values
 */
export interface AlarmThresholds {
  /** Low-low threshold */
  lowLow?: number;

  /** Low threshold */
  low?: number;

  /** High threshold */
  high?: number;

  /** High-high threshold */
  highHigh?: number;

  /** Deadband value */
  deadband: number;

  /** Delay before alarming in seconds */
  delaySec: number;
}

/** Alarm priority levels */
export type AlarmPriority = 'critical' | 'high' | 'medium' | 'low' | 'info';

/**
 * Notification settings for alarms
 */
export interface NotificationSettings {
  /** Email recipients */
  email: string[];

  /** SMS recipients */
  sms: string[];

  /** Webhook URLs */
  webhooks: string[];

  /** Escalation delay in minutes */
  escalationDelayMin: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Base error class for desalination SDK
 */
export class WIADesalinationError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'WIADesalinationError';
  }
}

/**
 * Error thrown when plant is not found
 */
export class PlantNotFoundError extends WIADesalinationError {
  constructor(plantId: PlantId) {
    super(
      `Desalination plant not found: ${plantId}`,
      'PLANT_NOT_FOUND',
      { plantId }
    );
    this.name = 'PlantNotFoundError';
  }
}

/**
 * Error thrown when water quality limits are exceeded
 */
export class QualityLimitExceededError extends WIADesalinationError {
  constructor(parameter: string, value: number, limit: number) {
    super(
      `Water quality limit exceeded: ${parameter} = ${value} (limit: ${limit})`,
      'QUALITY_LIMIT_EXCEEDED',
      { parameter, value, limit }
    );
    this.name = 'QualityLimitExceededError';
  }
}

/**
 * Error thrown when membrane is critically fouled
 */
export class MembraneFoulingError extends WIADesalinationError {
  constructor(membraneId: MembraneId, foulingIndex: number) {
    super(
      `Critical membrane fouling detected: ${membraneId}`,
      'MEMBRANE_FOULING',
      { membraneId, foulingIndex }
    );
    this.name = 'MembraneFoulingError';
  }
}
