/**
 * WIA Direct Air Capture Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIADirectAirCapture {
  standard: 'WIA-DIRECT-AIR-CAPTURE';
  version: string;
  facility: FacilityInfo;
  technology: TechnologyConfig;
  operations: OperationsConfig;
  carbon: CarbonAccounting;
  energy: EnergyProfile;
  monitoring: MonitoringConfig;
  compliance?: ComplianceInfo;
  extensions?: Record<string, unknown>;
}

export interface FacilityInfo {
  id: string;
  name: string;
  status: FacilityStatus;
  location: GeoLocation;
  operator: OrganizationInfo;
  capacity: CaptureCapacity;
  constructionDate?: string;
  operationalDate?: string;
  decommissionDate?: string;
}

export type FacilityStatus = 'planning' | 'construction' | 'commissioning' | 'operational' | 'maintenance' | 'decommissioned';

export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude: number;
  country: string;
  region?: string;
  address?: string;
  timezone: string;
}

export interface OrganizationInfo {
  id: string;
  name: string;
  type: 'private' | 'public' | 'partnership' | 'research';
  contact: { email: string; phone?: string };
  certifications?: string[];
}

export interface CaptureCapacity {
  designCapacity: number;
  currentCapacity: number;
  unit: 'tCO2/year' | 'ktCO2/year' | 'MtCO2/year';
  utilizationRate?: number;
}

// ============================================================================
// Technology Types
// ============================================================================

export interface TechnologyConfig {
  type: DACTechnology;
  vendor?: string;
  model?: string;
  specifications: TechnologySpecs;
  sorbent?: SorbentInfo;
  equipment: Equipment[];
}

export type DACTechnology =
  | 'solid-sorbent'
  | 'liquid-solvent'
  | 'electrochemical'
  | 'membrane'
  | 'cryogenic'
  | 'hybrid';

export interface TechnologySpecs {
  captureEfficiency: number;
  energyRequirement: EnergyRequirement;
  waterConsumption?: number;
  operatingTemperature: TemperatureRange;
  operatingPressure: PressureRange;
  cycleTime: number;
  lifetime: number;
}

export interface EnergyRequirement {
  thermal: number;
  electrical: number;
  unit: 'kWh/tCO2' | 'GJ/tCO2';
}

export interface TemperatureRange {
  min: number;
  max: number;
  optimal: number;
  unit: 'celsius' | 'kelvin';
}

export interface PressureRange {
  min: number;
  max: number;
  unit: 'bar' | 'psi' | 'kPa';
}

export interface SorbentInfo {
  type: 'amine-based' | 'mof' | 'zeolite' | 'ionic-liquid' | 'alkali-solution' | 'custom';
  name?: string;
  regenerationMethod: 'temperature-swing' | 'pressure-swing' | 'moisture-swing' | 'electrochemical';
  lifetime: number;
  recyclability: number;
}

export interface Equipment {
  id: string;
  type: EquipmentType;
  manufacturer?: string;
  model?: string;
  count: number;
  status: 'operational' | 'maintenance' | 'offline' | 'failed';
}

export type EquipmentType =
  | 'air-contactor'
  | 'regeneration-unit'
  | 'compressor'
  | 'heat-exchanger'
  | 'pump'
  | 'blower'
  | 'filter'
  | 'storage-tank'
  | 'control-system';

// ============================================================================
// Operations Types
// ============================================================================

export interface OperationsConfig {
  mode: OperationMode;
  schedule: OperatingSchedule;
  maintenance: MaintenanceSchedule;
  automation: AutomationLevel;
  safety: SafetyConfig;
}

export type OperationMode = 'continuous' | 'batch' | 'semi-batch' | 'load-following';

export interface OperatingSchedule {
  hoursPerDay: number;
  daysPerYear: number;
  plannedDowntime: number;
  peakOperatingHours?: number[];
}

export interface MaintenanceSchedule {
  preventive: MaintenanceEvent[];
  predictive: boolean;
  averageDowntimePercent: number;
}

export interface MaintenanceEvent {
  type: string;
  frequency: string;
  duration: number;
  lastCompleted?: string;
  nextScheduled?: string;
}

export type AutomationLevel = 'manual' | 'semi-automated' | 'fully-automated' | 'autonomous';

export interface SafetyConfig {
  emergencyShutdown: boolean;
  leakDetection: boolean;
  fireSupression: boolean;
  safetyZone: number;
  permits: SafetyPermit[];
}

export interface SafetyPermit {
  type: string;
  issuer: string;
  number: string;
  validUntil: string;
}

// ============================================================================
// Carbon Accounting Types
// ============================================================================

export interface CarbonAccounting {
  methodology: AccountingMethodology;
  captured: CarbonMetrics;
  stored: StorageInfo;
  utilized?: UtilizationInfo;
  netRemoval: NetRemovalCalc;
  verification: VerificationInfo;
}

export interface AccountingMethodology {
  standard: 'iso-14064' | 'ghg-protocol' | 'verra' | 'gold-standard' | 'custom';
  version?: string;
  boundaries: string[];
  reportingPeriod: string;
}

export interface CarbonMetrics {
  totalCaptured: number;
  periodCaptured: number;
  cumulativeCaptured: number;
  unit: 'tCO2' | 'ktCO2';
  measurementUncertainty: number;
}

export interface StorageInfo {
  method: StorageMethod;
  location?: GeoLocation;
  capacity: number;
  currentStored: number;
  permanence: PermanenceInfo;
}

export type StorageMethod =
  | 'geological-saline'
  | 'geological-depleted'
  | 'enhanced-oil-recovery'
  | 'basalt-mineralization'
  | 'ocean-storage'
  | 'mineral-carbonation';

export interface PermanenceInfo {
  expectedDuration: number;
  unit: 'years';
  monitoringRequired: boolean;
  leakageRisk: 'low' | 'medium' | 'high';
}

export interface UtilizationInfo {
  method: UtilizationMethod;
  product?: string;
  buyer?: string;
  permanence: 'permanent' | 'temporary' | 'short-lived';
  percentage: number;
}

export type UtilizationMethod =
  | 'building-materials'
  | 'synthetic-fuels'
  | 'chemicals'
  | 'enhanced-agriculture'
  | 'food-beverage'
  | 'other';

export interface NetRemovalCalc {
  grossCapture: number;
  processEmissions: number;
  supplyChainEmissions: number;
  netRemoval: number;
  removalEfficiency: number;
}

export interface VerificationInfo {
  verifier: string;
  standard: string;
  lastVerified: string;
  nextVerification: string;
  certificateId?: string;
}

// ============================================================================
// Energy Types
// ============================================================================

export interface EnergyProfile {
  sources: EnergySource[];
  consumption: EnergyConsumption;
  carbonIntensity: number;
  renewablePercent: number;
  gridConnection?: GridConnection;
}

export interface EnergySource {
  type: 'solar' | 'wind' | 'geothermal' | 'nuclear' | 'grid' | 'waste-heat' | 'natural-gas';
  percentage: number;
  capacity?: number;
  carbonIntensity: number;
}

export interface EnergyConsumption {
  thermal: number;
  electrical: number;
  total: number;
  unit: 'MWh/year' | 'GJ/year';
  peakDemand?: number;
}

export interface GridConnection {
  connected: boolean;
  voltage?: number;
  frequency?: number;
  gridOperator?: string;
}

// ============================================================================
// Monitoring Types
// ============================================================================

export interface MonitoringConfig {
  realtime: boolean;
  sensors: SensorConfig[];
  dataFrequency: number;
  storage: DataStorage;
  alerts: AlertConfig[];
}

export interface SensorConfig {
  id: string;
  type: SensorType;
  location: string;
  accuracy: number;
  calibrationDate?: string;
}

export type SensorType = 'co2-concentration' | 'temperature' | 'pressure' | 'flow-rate' | 'humidity' | 'energy-meter' | 'leak-detector';

export interface DataStorage {
  local: boolean;
  cloud: boolean;
  retention: number;
  encryption: boolean;
}

export interface AlertConfig {
  metric: string;
  threshold: number;
  operator: 'gt' | 'lt' | 'eq';
  severity: 'info' | 'warning' | 'critical';
  channels: string[];
}

export interface ComplianceInfo {
  regulations: string[];
  permits: SafetyPermit[];
  audits: AuditRecord[];
}

export interface AuditRecord {
  date: string;
  auditor: string;
  type: string;
  result: 'passed' | 'failed' | 'conditional';
  findings?: string[];
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface FacilityResponse {
  id: string;
  name: string;
  status: FacilityStatus;
  location: string;
  capacity: CaptureCapacity;
  netRemoval: number;
  links: { self: string };
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
