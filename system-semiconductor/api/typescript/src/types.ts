/**
 * WIA-SEMI-001 TypeScript Type Definitions
 *
 * Version: 1.0.0
 * Standard: WIA-SEMI-001 System Semiconductor
 *
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 - Benefit All Humanity
 */

// ============================================================================
// Base Types
// ============================================================================

export interface Quantity {
  value: number;
  unit: string;
}

export interface FrequencyQuantity extends Quantity {
  unit: 'Hz' | 'kHz' | 'MHz' | 'GHz';
}

export interface PowerQuantity extends Quantity {
  unit: 'mW' | 'W';
}

export interface TemperatureQuantity extends Quantity {
  unit: '°C' | '°F' | 'K';
}

export interface TimeQuantity extends Quantity {
  unit: 'ns' | 'us' | 'ms' | 's';
}

// ============================================================================
// Chip Specification Types
// ============================================================================

export interface WIAChipSpecification {
  $schema: string;
  wiaVersion: string;
  metadata: ChipMetadata;
  architecture: ChipArchitecture;
  power: PowerSpecification;
  thermal: ThermalSpecification;
  interfaces?: InterfaceSpecification[];
  performance?: PerformanceMetrics;
}

export interface ChipMetadata {
  id: string;
  manufacturer: Manufacturer;
  productName: string;
  productFamily: string;
  marketSegment: MarketSegment[];
  releaseDate: string; // ISO 8601
  productionStatus: ProductionStatus;
  documentVersion: string;
  documentDate: string; // ISO 8601
}

export interface Manufacturer {
  name: string;
  division?: string;
  contact: string;
  website?: string;
}

export type MarketSegment =
  | 'mobile'
  | 'tablet'
  | 'laptop'
  | 'desktop'
  | 'server'
  | 'automotive'
  | 'iot'
  | 'embedded';

export type ProductionStatus =
  | 'announced'
  | 'sampling'
  | 'active'
  | 'eol-announced'
  | 'eol';

// ============================================================================
// Architecture Types
// ============================================================================

export interface ChipArchitecture {
  type: 'heterogeneous-multicore' | 'homogeneous-multicore' | 'single-core';
  instructionSet: InstructionSet;
  manufacturing: ManufacturingInfo;
  cpu?: CPUArchitecture;
  gpu?: GPUArchitecture;
  npu?: NPUArchitecture;
}

export interface InstructionSet {
  primary: string;
  extensions?: string[];
}

export interface ManufacturingInfo {
  processNode: string;
  technology: string;
  foundry?: string;
  dieSize: Quantity;
  transistorCount: Quantity;
}

export interface CPUArchitecture {
  clusters: CPUCluster[];
  sharedCache?: CacheHierarchy;
}

export interface CPUCluster {
  name: string;
  coreCount: number;
  microarchitecture: string;
  frequency: FrequencyRange;
  cache: CacheConfiguration;
  features?: string[];
}

export interface FrequencyRange {
  min: number;
  max: number;
  unit: 'MHz' | 'GHz';
}

export interface CacheConfiguration {
  l1i?: CacheSpec;
  l1d?: CacheSpec;
  l2?: CacheSpec;
}

export interface CacheHierarchy {
  l3?: CacheSpec;
  slc?: CacheSpec;
}

export interface CacheSpec {
  size: number;
  unit: 'KB' | 'MB';
}

export interface GPUArchitecture {
  architecture: string;
  computeUnits: number;
  frequency: FrequencyRange;
  features?: string[];
  apis?: string[];
}

export interface NPUArchitecture {
  architecture: string;
  performance: Quantity;
  precision: string[];
  frameworks?: string[];
}

// ============================================================================
// Power Types
// ============================================================================

export interface PowerSpecification {
  tdp: TDPSpecification;
  voltageRails: VoltageRail[];
  powerStates: PowerState[];
  powerManagement?: PowerManagementFeatures;
}

export interface TDPSpecification {
  typical: PowerQuantity;
  maximum: PowerQuantity;
}

export interface VoltageRail {
  name: string;
  nominal: number;
  min: number;
  max: number;
  unit: 'V';
  tolerance: number;
}

export interface PowerState {
  name: string;
  cpuFrequency?: number;
  gpuFrequency?: number;
  power: PowerQuantity;
}

export interface PowerManagementFeatures {
  features: string[];
  granularity?: string;
  transitionTime?: Record<string, TimeQuantity>;
}

// ============================================================================
// Thermal Types
// ============================================================================

export interface ThermalSpecification {
  junctionTemperature: TemperatureRange;
  thermalResistance: ThermalResistance;
  hotspots?: Hotspot[];
  coolingRequirements?: CoolingRequirements;
  thermalThrottling?: ThermalThrottling;
}

export interface TemperatureRange {
  typical: number;
  maximum: number;
  unit: '°C';
}

export interface ThermalResistance {
  junctionToCase?: Quantity;
  junctionToAmbient?: Quantity;
}

export interface Hotspot {
  location: string;
  peakTemperature: number;
  area: Quantity;
}

export interface CoolingRequirements {
  passive?: PassiveCooling;
  active?: ActiveCooling;
}

export interface PassiveCooling {
  minHeatsinkSize: Quantity;
  finHeight?: Quantity;
}

export interface ActiveCooling {
  fanRequired: boolean;
  optionalForSustained?: boolean;
}

export interface ThermalThrottling {
  enabled: boolean;
  thresholds: ThrottlingThreshold[];
}

export interface ThrottlingThreshold {
  temperature: number;
  action: string;
  unit: '°C';
}

// ============================================================================
// Interface Types
// ============================================================================

export interface InterfaceSpecification {
  type: string;
  standard: string;
  speed?: Quantity;
  configuration?: string;
}

// ============================================================================
// Performance Types
// ============================================================================

export interface PerformanceMetrics {
  cpu?: CPUPerformance;
  gpu?: GPUPerformance;
  npu?: NPUPerformance;
}

export interface CPUPerformance {
  singleCore?: BenchmarkResult;
  multiCore?: BenchmarkResult;
  integer?: BenchmarkResult;
  floatingPoint?: BenchmarkResult;
}

export interface GPUPerformance {
  graphics?: GraphicsBenchmark;
  compute?: BenchmarkResult;
  rayTracing?: BenchmarkResult;
  powerEfficiency?: Quantity;
}

export interface NPUPerformance {
  aiInference?: BenchmarkResult;
  frameworks?: Record<string, BenchmarkResult>;
}

export interface BenchmarkResult {
  benchmark: string;
  score?: number;
  value?: number;
  unit?: string;
  testConditions?: string;
}

export interface GraphicsBenchmark extends BenchmarkResult {
  onscreen?: Quantity;
  offscreen?: Quantity;
  conditions?: {
    resolution?: string;
    api?: string;
    quality?: string;
  };
}

// ============================================================================
// API Types
// ============================================================================

export interface ChipInfo extends WIAChipSpecification {}

export interface ChipCapabilities {
  cpu?: {
    cores: number;
    features: string[];
  };
  gpu?: {
    present: boolean;
    features: string[];
  };
  npu?: {
    present: boolean;
    performance: Quantity;
  };
}

export enum PowerMode {
  Efficiency = 'efficiency',
  Balanced = 'balanced',
  Performance = 'performance'
}

export interface PowerModeConfig {
  mode: PowerMode;
  sustainedDuration?: number; // milliseconds
  autoRevert?: boolean;
}

export interface PowerModeResponse {
  status: 'success' | 'error';
  currentMode: PowerMode;
  effectiveAt: string; // ISO 8601
  estimatedPower: PowerQuantity;
}

export interface FrequencyConfig {
  cpu?: FrequencyRange;
  gpu?: FrequencyRange;
}

export interface ThermalLimitConfig {
  threshold: number;
  unit: '°C';
  action?: string;
}

export interface PowerLimitConfig {
  limit: number;
  unit: 'W';
  enforcement?: 'soft' | 'hard';
}

export interface PowerMonitorData {
  timestamp: string; // ISO 8601
  total: PowerQuantity;
  breakdown?: {
    cpu?: PowerQuantity;
    gpu?: PowerQuantity;
    other?: PowerQuantity;
  };
}

export interface TemperatureMonitorData {
  timestamp: string; // ISO 8601
  sensors: TemperatureSensor[];
}

export interface TemperatureSensor {
  location: string;
  temperature: TemperatureQuantity;
  status: 'normal' | 'warning' | 'critical';
}

export interface FrequencyMonitorData {
  timestamp: string; // ISO 8601
  cpu?: {
    cluster0?: number[];
    cluster1?: number[];
    unit: 'MHz' | 'GHz';
  };
  gpu?: FrequencyQuantity;
}

export interface UtilizationMonitorData {
  timestamp: string; // ISO 8601
  cpu?: {
    overall: number;
    perCore?: number[];
  };
  gpu?: number;
  npu?: number;
  memory?: number;
  unit: '%';
}

export interface TelemetryData {
  timestamp: string; // ISO 8601
  power?: PowerQuantity;
  temperature?: TemperatureQuantity;
  frequency?: {
    cpu?: number;
    gpu?: number;
    unit: 'MHz' | 'GHz';
  };
  utilization?: {
    cpu?: number;
    gpu?: number;
    unit: '%';
  };
}

// ============================================================================
// Error Types
// ============================================================================

export interface WIAError {
  code: string;
  message: string;
  details?: Record<string, any>;
  timestamp: string; // ISO 8601
  requestId?: string;
}

export type ErrorCode =
  | 'INVALID_PARAMETER'
  | 'UNAUTHORIZED'
  | 'FORBIDDEN'
  | 'NOT_FOUND'
  | 'RATE_LIMIT_EXCEEDED'
  | 'THERMAL_THROTTLED'
  | 'POWER_LIMIT_EXCEEDED'
  | 'INTERNAL_ERROR';
