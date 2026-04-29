/**
 * WIA Distributed Energy Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIADistributedEnergy {
  standard: 'WIA-DISTRIBUTED-ENERGY';
  version: string;
  network: NetworkInfo;
  resources: EnergyResource[];
  storage: StorageSystem[];
  loads: LoadProfile[];
  trading: EnergyTrading;
  grid: GridInteraction;
  monitoring: MonitoringConfig;
  extensions?: Record<string, unknown>;
}

export interface NetworkInfo {
  id: string;
  name: string;
  type: NetworkType;
  status: NetworkStatus;
  operator: OperatorInfo;
  location: GeoLocation;
  capacity: NetworkCapacity;
  createdAt: string;
}

export type NetworkType = 'microgrid' | 'virtual-power-plant' | 'community-energy' | 'industrial-park' | 'residential-cluster';
export type NetworkStatus = 'planning' | 'construction' | 'operational' | 'islanded' | 'maintenance' | 'offline';

export interface OperatorInfo {
  id: string;
  name: string;
  licenseNumber?: string;
  contact: { email: string; phone?: string };
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  country: string;
  region?: string;
  timezone: string;
}

export interface NetworkCapacity {
  generation: number;
  storage: number;
  peakLoad: number;
  unit: 'kW' | 'MW';
}

// ============================================================================
// Resource Types
// ============================================================================

export interface EnergyResource {
  id: string;
  type: ResourceType;
  name: string;
  owner: string;
  location: GeoLocation;
  capacity: ResourceCapacity;
  status: ResourceStatus;
  specifications: ResourceSpecs;
  inverter?: InverterInfo;
  metering: MeteringConfig;
}

export type ResourceType =
  | 'solar-pv'
  | 'wind-turbine'
  | 'micro-hydro'
  | 'fuel-cell'
  | 'chp'
  | 'diesel-generator'
  | 'biomass'
  | 'geothermal';

export type ResourceStatus = 'online' | 'offline' | 'curtailed' | 'fault' | 'maintenance';

export interface ResourceCapacity {
  rated: number;
  available: number;
  currentOutput: number;
  unit: 'kW' | 'MW';
}

export interface ResourceSpecs {
  efficiency: number;
  capacityFactor: number;
  rampRate?: number;
  minOutput?: number;
  maxOutput?: number;
  carbonIntensity?: number;
}

export interface InverterInfo {
  type: 'string' | 'central' | 'micro' | 'hybrid';
  capacity: number;
  efficiency: number;
  gridForming: boolean;
}

export interface MeteringConfig {
  meterId: string;
  type: 'smart' | 'advanced' | 'interval' | 'net';
  resolution: number;
  bidirectional: boolean;
  protocols: string[];
}

// ============================================================================
// Storage Types
// ============================================================================

export interface StorageSystem {
  id: string;
  type: StorageType;
  name: string;
  capacity: StorageCapacity;
  status: StorageStatus;
  specifications: StorageSpecs;
  controller: ControllerConfig;
}

export type StorageType = 'lithium-ion' | 'flow-battery' | 'lead-acid' | 'sodium-ion' | 'flywheel' | 'compressed-air' | 'thermal' | 'hydrogen';
export type StorageStatus = 'charging' | 'discharging' | 'idle' | 'fault' | 'maintenance';

export interface StorageCapacity {
  energy: number;
  power: number;
  stateOfCharge: number;
  stateOfHealth: number;
  energyUnit: 'kWh' | 'MWh';
  powerUnit: 'kW' | 'MW';
}

export interface StorageSpecs {
  roundTripEfficiency: number;
  depthOfDischarge: number;
  cycleLife: number;
  cRate: { charge: number; discharge: number };
  selfDischarge: number;
  warranty: number;
}

export interface ControllerConfig {
  type: 'local' | 'remote' | 'aggregated';
  algorithm: 'rule-based' | 'optimization' | 'machine-learning' | 'market-driven';
  parameters: Record<string, unknown>;
}

// ============================================================================
// Load Types
// ============================================================================

export interface LoadProfile {
  id: string;
  type: LoadType;
  name: string;
  category: LoadCategory;
  demand: LoadDemand;
  flexibility: LoadFlexibility;
  priority: LoadPriority;
}

export type LoadType = 'residential' | 'commercial' | 'industrial' | 'agricultural' | 'ev-charging' | 'hvac' | 'lighting' | 'critical';
export type LoadCategory = 'baseload' | 'controllable' | 'shiftable' | 'curtailable' | 'critical';
export type LoadPriority = 'essential' | 'high' | 'medium' | 'low' | 'deferrable';

export interface LoadDemand {
  current: number;
  peak: number;
  average: number;
  unit: 'kW' | 'MW';
  profile?: number[];
}

export interface LoadFlexibility {
  controllable: boolean;
  shiftWindow?: { start: string; end: string };
  curtailmentPotential?: number;
  demandResponse: boolean;
  incentiveSensitivity?: number;
}

// ============================================================================
// Trading Types
// ============================================================================

export interface EnergyTrading {
  enabled: boolean;
  platform: TradingPlatform;
  participants: Participant[];
  contracts: EnergyContract[];
  settlements: SettlementConfig;
}

export interface TradingPlatform {
  type: 'p2p' | 'centralized' | 'hybrid' | 'blockchain';
  protocol?: string;
  operator?: string;
  fees?: TradingFees;
}

export interface TradingFees {
  transaction: number;
  platform: number;
  unit: 'percent' | 'fixed';
}

export interface Participant {
  id: string;
  type: 'prosumer' | 'consumer' | 'producer' | 'aggregator';
  name: string;
  walletAddress?: string;
  creditRating?: number;
}

export interface EnergyContract {
  id: string;
  type: ContractType;
  seller: string;
  buyer: string;
  quantity: number;
  price: number;
  unit: 'kWh' | 'MWh';
  currency: string;
  startTime: string;
  endTime: string;
  status: ContractStatus;
}

export type ContractType = 'spot' | 'forward' | 'ppa' | 'auction' | 'bilateral';
export type ContractStatus = 'pending' | 'active' | 'completed' | 'cancelled' | 'disputed';

export interface SettlementConfig {
  frequency: 'realtime' | 'hourly' | 'daily' | 'monthly';
  currency: string;
  paymentMethod: 'fiat' | 'crypto' | 'credit';
  reconciliation: boolean;
}

// ============================================================================
// Grid Types
// ============================================================================

export interface GridInteraction {
  connectionPoint: ConnectionPoint;
  interconnection: InterconnectionAgreement;
  services: GridService[];
  islanding: IslandingCapability;
}

export interface ConnectionPoint {
  id: string;
  voltage: number;
  frequency: number;
  phases: 1 | 3;
  maxImport: number;
  maxExport: number;
}

export interface InterconnectionAgreement {
  utility: string;
  agreementId: string;
  tariff: string;
  netMetering: boolean;
  feedInTariff?: number;
  exportLimit?: number;
}

export interface GridService {
  type: GridServiceType;
  enabled: boolean;
  capacity?: number;
  compensation?: number;
}

export type GridServiceType =
  | 'frequency-regulation'
  | 'voltage-support'
  | 'spinning-reserve'
  | 'peak-shaving'
  | 'black-start'
  | 'demand-response';

export interface IslandingCapability {
  capable: boolean;
  automatic: boolean;
  transitionTime?: number;
  duration?: number;
  criticalLoadsSupported: string[];
}

// ============================================================================
// Monitoring Types
// ============================================================================

export interface MonitoringConfig {
  scada: SCADAConfig;
  analytics: AnalyticsConfig;
  forecasting: ForecastConfig;
  alerts: AlertRule[];
}

export interface SCADAConfig {
  enabled: boolean;
  protocol: 'modbus' | 'dnp3' | 'iec61850' | 'mqtt' | 'opcua';
  pollInterval: number;
  redundancy: boolean;
}

export interface AnalyticsConfig {
  enabled: boolean;
  metrics: string[];
  dashboard: boolean;
  retention: number;
}

export interface ForecastConfig {
  generation: boolean;
  load: boolean;
  price: boolean;
  horizon: number;
  models: string[];
}

export interface AlertRule {
  id: string;
  metric: string;
  condition: string;
  threshold: number;
  severity: 'info' | 'warning' | 'critical';
  actions: string[];
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface NetworkResponse {
  id: string;
  name: string;
  type: NetworkType;
  status: NetworkStatus;
  resourceCount: number;
  totalCapacity: number;
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
