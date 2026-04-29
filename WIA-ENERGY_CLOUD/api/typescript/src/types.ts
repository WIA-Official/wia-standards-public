/**
 * WIA Energy Cloud Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIAEnergyCloud {
  standard: 'WIA-ENERGY-CLOUD';
  version: string;
  platform: EnergyCloudPlatform;
  assets: EnergyAsset[];
  virtualPowerPlants: VirtualPowerPlant[];
  markets: EnergyMarket[];
  optimization: OptimizationConfig;
  forecasting: ForecastingConfig;
  billing: BillingConfig;
  extensions?: Record<string, unknown>;
}

export interface EnergyCloudPlatform {
  id: string;
  name: string;
  type: PlatformType;
  status: PlatformStatus;
  operator: OperatorInfo;
  coverage: CoverageArea;
  capabilities: PlatformCapability[];
  createdAt: string;
}

export type PlatformType = 'utility-scale' | 'community' | 'industrial' | 'residential' | 'hybrid';
export type PlatformStatus = 'initializing' | 'operational' | 'maintenance' | 'degraded' | 'offline';

export interface OperatorInfo {
  id: string;
  name: string;
  type: 'utility' | 'aggregator' | 'esco' | 'cooperative' | 'platform';
  license?: string;
  contact: { email: string; phone?: string };
}

export interface CoverageArea {
  regions: string[];
  gridZones: string[];
  totalCapacity: number;
  assetCount: number;
  unit: 'MW' | 'GW';
}

export interface PlatformCapability {
  type: CapabilityType;
  enabled: boolean;
  specifications?: Record<string, unknown>;
}

export type CapabilityType =
  | 'demand-response'
  | 'peer-to-peer'
  | 'virtual-power-plant'
  | 'grid-services'
  | 'ev-integration'
  | 'storage-optimization'
  | 'renewable-integration';

// ============================================================================
// Asset Types
// ============================================================================

export interface EnergyAsset {
  id: string;
  type: AssetType;
  name: string;
  owner: AssetOwner;
  location: GeoLocation;
  specifications: AssetSpecifications;
  status: AssetStatus;
  connectivity: ConnectivityInfo;
  metering: MeteringConfig;
}

export type AssetType = 'solar' | 'wind' | 'storage' | 'ev-charger' | 'heat-pump' | 'flexible-load' | 'generator' | 'building';
export type AssetStatus = 'online' | 'offline' | 'dispatching' | 'charging' | 'discharging' | 'standby' | 'fault';

export interface AssetOwner {
  id: string;
  name: string;
  type: 'residential' | 'commercial' | 'industrial' | 'utility';
  agreement: string;
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  country: string;
  region?: string;
  gridNode?: string;
}

export interface AssetSpecifications {
  capacity: number;
  capacityUnit: 'kW' | 'MW';
  energy?: number;
  energyUnit?: 'kWh' | 'MWh';
  efficiency: number;
  rampRate?: number;
  responseTime?: number;
}

export interface ConnectivityInfo {
  protocol: 'modbus' | 'mqtt' | 'ocpp' | 'ieee2030.5' | 'openadr' | 'api';
  endpoint: string;
  secure: boolean;
  latency: number;
  lastSeen: string;
}

export interface MeteringConfig {
  meterId: string;
  type: 'smart' | 'interval' | 'real-time';
  resolution: number;
  bidirectional: boolean;
}

// ============================================================================
// Virtual Power Plant Types
// ============================================================================

export interface VirtualPowerPlant {
  id: string;
  name: string;
  type: VPPType;
  status: VPPStatus;
  assets: VPPAsset[];
  capacity: VPPCapacity;
  dispatch: DispatchConfig;
  contracts: VPPContract[];
  performance: VPPPerformance;
}

export type VPPType = 'aggregation' | 'technical' | 'commercial' | 'flexibility';
export type VPPStatus = 'active' | 'dispatching' | 'standby' | 'offline';

export interface VPPAsset {
  assetId: string;
  contribution: number;
  availability: number;
  priority: number;
}

export interface VPPCapacity {
  total: number;
  available: number;
  committed: number;
  unit: 'MW';
}

export interface DispatchConfig {
  strategy: 'economic' | 'merit-order' | 'round-robin' | 'priority' | 'ai-optimized';
  constraints: DispatchConstraint[];
  schedule: DispatchSchedule;
}

export interface DispatchConstraint {
  type: 'ramp-rate' | 'min-duration' | 'max-cycles' | 'energy-limit';
  value: number;
  unit: string;
}

export interface DispatchSchedule {
  type: 'realtime' | 'day-ahead' | 'intraday';
  resolution: number;
  horizon: number;
}

export interface VPPContract {
  id: string;
  type: ContractType;
  counterparty: string;
  capacity: number;
  price: number;
  currency: string;
  validFrom: string;
  validTo: string;
  status: 'active' | 'pending' | 'expired';
}

export type ContractType = 'capacity' | 'energy' | 'frequency-regulation' | 'reserve' | 'demand-response';

export interface VPPPerformance {
  availability: number;
  responseAccuracy: number;
  dispatchSuccess: number;
  revenueGenerated: number;
}

// ============================================================================
// Market Types
// ============================================================================

export interface EnergyMarket {
  id: string;
  name: string;
  type: MarketType;
  operator: string;
  products: MarketProduct[];
  trading: TradingConfig;
  settlement: SettlementConfig;
}

export type MarketType = 'wholesale' | 'retail' | 'ancillary' | 'capacity' | 'flexibility' | 'p2p';

export interface MarketProduct {
  id: string;
  name: string;
  type: ProductType;
  unit: string;
  minQuantity: number;
  tradingPeriod: string;
  priceFormation: 'merit-order' | 'pay-as-bid' | 'pay-as-clear' | 'bilateral';
}

export type ProductType = 'energy' | 'capacity' | 'fcr' | 'afrr' | 'mfrr' | 'rr' | 'demand-response';

export interface TradingConfig {
  gateClosures: { market: string; closure: number }[];
  minimumBid: number;
  priceLimit: { min: number; max: number };
  positions: TradePosition[];
}

export interface TradePosition {
  id: string;
  product: string;
  direction: 'buy' | 'sell';
  quantity: number;
  price: number;
  status: 'pending' | 'matched' | 'executed' | 'cancelled';
  timestamp: string;
}

export interface SettlementConfig {
  frequency: 'realtime' | 'hourly' | 'daily' | 'monthly';
  currency: string;
  imbalancePricing: boolean;
  nettingEnabled: boolean;
}

// ============================================================================
// Optimization Types
// ============================================================================

export interface OptimizationConfig {
  enabled: boolean;
  algorithm: OptimizationAlgorithm;
  objectives: OptimizationObjective[];
  constraints: OptimizationConstraint[];
  horizon: { value: number; unit: 'hours' | 'days' };
  resolution: number;
}

export type OptimizationAlgorithm = 'milp' | 'genetic' | 'reinforcement-learning' | 'heuristic' | 'model-predictive';

export interface OptimizationObjective {
  type: 'cost-minimization' | 'revenue-maximization' | 'self-consumption' | 'peak-shaving' | 'carbon-reduction';
  weight: number;
  target?: number;
}

export interface OptimizationConstraint {
  type: string;
  parameter: string;
  operator: 'eq' | 'lt' | 'gt' | 'lte' | 'gte';
  value: number;
}

// ============================================================================
// Forecasting Types
// ============================================================================

export interface ForecastingConfig {
  enabled: boolean;
  models: ForecastModel[];
  dataSources: DataSource[];
  accuracy: ForecastAccuracy;
}

export interface ForecastModel {
  id: string;
  type: ForecastType;
  algorithm: string;
  horizon: number;
  resolution: number;
  features: string[];
}

export type ForecastType = 'generation' | 'consumption' | 'price' | 'flexibility' | 'weather';

export interface DataSource {
  type: 'historical' | 'weather' | 'market' | 'calendar' | 'real-time';
  provider: string;
  updateFrequency: number;
}

export interface ForecastAccuracy {
  mae: number;
  rmse: number;
  mape: number;
  lastEvaluated: string;
}

// ============================================================================
// Billing Types
// ============================================================================

export interface BillingConfig {
  model: BillingModel;
  tariffs: Tariff[];
  incentives: Incentive[];
  settlements: Settlement[];
}

export type BillingModel = 'net-metering' | 'time-of-use' | 'real-time-pricing' | 'feed-in-tariff' | 'peer-to-peer';

export interface Tariff {
  id: string;
  name: string;
  type: 'import' | 'export' | 'demand' | 'connection';
  rates: TariffRate[];
  validFrom: string;
  validTo?: string;
}

export interface TariffRate {
  period: string;
  rate: number;
  unit: string;
  currency: string;
}

export interface Incentive {
  type: 'subsidy' | 'tax-credit' | 'rebate' | 'premium';
  amount: number;
  unit: string;
  eligibility: string[];
  expires?: string;
}

export interface Settlement {
  id: string;
  period: string;
  assetId: string;
  energyImport: number;
  energyExport: number;
  gridServices: number;
  totalAmount: number;
  currency: string;
  status: 'pending' | 'calculated' | 'invoiced' | 'paid';
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface PlatformResponse {
  id: string;
  name: string;
  type: PlatformType;
  status: PlatformStatus;
  assetCount: number;
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
