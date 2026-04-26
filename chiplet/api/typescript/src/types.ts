/**
 * WIA Chiplet Standard - Type Definitions
 *
 * @module @wia/chiplet/types
 * @version 1.0.0
 * @license MIT
 *
 * 弘益人間 · Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Chiplet type classification
 */
export enum ChipletType {
  PROCESSOR = 'processor',
  MEMORY = 'memory',
  IO = 'io',
  ACCELERATOR = 'accelerator',
  GRAPHICS = 'graphics',
  SECURITY = 'security',
  ANALOG = 'analog',
  CUSTOM = 'custom',
}

/**
 * Interface protocol types
 */
export enum InterfaceProtocol {
  RAW = 'raw',
  STREAMING = 'streaming',
  PCIE = 'pcie',
  CXL = 'cxl',
}

/**
 * Package integration types
 */
export enum PackageType {
  INTERPOSER_2_5D = '2.5D',
  STACKING_3D = '3D',
  ORGANIC = 'organic',
  HYBRID = 'hybrid',
}

/**
 * Power management modes
 */
export enum PowerMode {
  PERFORMANCE = 'performance',
  BALANCED = 'balanced',
  POWER_SAVER = 'power_saver',
  CUSTOM = 'custom',
}

// ============================================================================
// Chiplet Definitions
// ============================================================================

/**
 * Physical dimensions
 */
export interface Dimensions {
  width: number;
  height: number;
  unit: 'mm' | 'um';
}

/**
 * Position coordinates
 */
export interface Position {
  x: number;
  y: number;
  z?: number;
  rotation?: number;
}

/**
 * Technology specifications
 */
export interface TechnologySpec {
  processNode: string;
  dieSize: Dimensions;
  foundry?: string;
  maskSet?: string;
}

/**
 * Voltage range specification
 */
export interface VoltageRange {
  domain: string;
  min: number;
  max: number;
  typical: number;
  unit: 'V' | 'mV';
}

/**
 * Power specifications
 */
export interface PowerSpec {
  tdp: number;
  idle: number;
  max?: number;
  voltageRanges: VoltageRange[];
  powerGating?: boolean;
  dvfsSupported?: boolean;
}

/**
 * Thermal specifications
 */
export interface ThermalSpec {
  tjMax: number;
  thermalResistance: number;
  coolingRequired?: string;
  thermalSensors?: number;
}

/**
 * Chiplet interface specification
 */
export interface ChipletInterface {
  type: 'UCIe' | 'proprietary';
  version: string;
  lanes: number;
  protocol: InterfaceProtocol;
  bandwidth: number;
  latency?: number;
  energyPerBit?: number;
}

/**
 * Functional capabilities
 */
export interface Capabilities {
  compute?: {
    cores?: number;
    frequency?: number;
    isa?: string;
    features?: string[];
  };
  memory?: {
    capacity?: string;
    bandwidth?: number;
    type?: string;
  };
  io?: {
    interfaces?: string[];
    lanes?: number;
  };
  custom?: Record<string, any>;
}

/**
 * Main chiplet definition
 */
export interface Chiplet {
  id: string;
  type: ChipletType;
  version: string;
  vendor: string;
  name?: string;
  description?: string;
  technology: TechnologySpec;
  interfaces: ChipletInterface[];
  power: PowerSpec;
  thermal: ThermalSpec;
  capabilities?: Capabilities;
  metadata?: Record<string, any>;
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Chiplet runtime configuration
 */
export interface ChipletConfig {
  powerMode: PowerMode;
  frequencyMHz?: number;
  voltageV?: number;
  enabledFeatures?: string[];
  customSettings?: Record<string, any>;
}

/**
 * Validation result
 */
export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
  warnings?: ValidationWarning[];
}

/**
 * Validation error
 */
export interface ValidationError {
  field: string;
  message: string;
  code: string;
}

/**
 * Validation warning
 */
export interface ValidationWarning {
  field: string;
  message: string;
  code: string;
}

// ============================================================================
// Integration Types
// ============================================================================

/**
 * Chiplet connection definition
 */
export interface ChipletConnection {
  source: string;
  destination: string;
  interface: string;
  lanes: number;
  protocol: InterfaceProtocol;
  bandwidth?: number;
}

/**
 * Package specification
 */
export interface PackageSpec {
  type: PackageType;
  dimensions: Dimensions;
  material?: string;
  layers?: number;
  bumpPitch?: number;
  interposerMaterial?: 'silicon' | 'glass' | 'organic';
}

/**
 * Integration plan
 */
export interface IntegrationPlan {
  name: string;
  version: string;
  chiplets: ChipletPlacement[];
  connections: ChipletConnection[];
  package: PackageSpec;
  validated?: boolean;
}

/**
 * Chiplet placement in package
 */
export interface ChipletPlacement {
  chipletId: string;
  position: Position;
  rotation?: number;
  layer?: number;
}

/**
 * Integration constraints
 */
export interface IntegrationConstraints {
  maxPower?: number;
  maxThermal?: number;
  maxSize?: Dimensions;
  requiredInterfaces?: string[];
  timing?: TimingConstraints;
}

/**
 * Timing constraints
 */
export interface TimingConstraints {
  maxLatency?: number;
  minBandwidth?: number;
  clockDomains?: ClockDomain[];
}

/**
 * Clock domain
 */
export interface ClockDomain {
  name: string;
  frequency: number;
  chiplets: string[];
}

// ============================================================================
// Performance Types
// ============================================================================

/**
 * Performance metrics
 */
export interface PerformanceMetrics {
  timestamp: number;
  chipletId: string;
  compute?: ComputeMetrics;
  memory?: MemoryMetrics;
  power?: PowerMetrics;
  thermal?: ThermalMetrics;
  interface?: InterfaceMetrics;
}

/**
 * Compute performance metrics
 */
export interface ComputeMetrics {
  utilization: number;
  frequency: number;
  instructions?: number;
  cycles?: number;
  ipc?: number;
  throughput?: number;
}

/**
 * Memory performance metrics
 */
export interface MemoryMetrics {
  bandwidth: number;
  latency: number;
  utilization: number;
  readBandwidth?: number;
  writeBandwidth?: number;
  hitRate?: number;
}

/**
 * Power metrics
 */
export interface PowerMetrics {
  current: number;
  average: number;
  peak: number;
  idle: number;
  voltage: number;
  domains?: PowerDomainMetrics[];
}

/**
 * Power domain metrics
 */
export interface PowerDomainMetrics {
  domain: string;
  power: number;
  voltage: number;
  current: number;
}

/**
 * Thermal metrics
 */
export interface ThermalMetrics {
  junction: number;
  ambient: number;
  hotspot?: number;
  sensors?: ThermalSensor[];
  throttling?: boolean;
}

/**
 * Thermal sensor reading
 */
export interface ThermalSensor {
  id: string;
  location: Position;
  temperature: number;
}

/**
 * Interface performance metrics
 */
export interface InterfaceMetrics {
  bandwidth: number;
  utilization: number;
  latency: number;
  errors?: number;
  retries?: number;
  packets?: PacketMetrics;
}

/**
 * Packet-level metrics
 */
export interface PacketMetrics {
  transmitted: number;
  received: number;
  dropped: number;
  errorRate: number;
}

/**
 * System-level performance
 */
export interface SystemPerformance {
  totalCompute: number;
  totalMemoryBandwidth: number;
  totalPower: number;
  averageLatency: number;
  efficiency: number;
  chiplets: Map<string, PerformanceMetrics>;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Event severity levels
 */
export enum EventSeverity {
  INFO = 'info',
  WARNING = 'warning',
  ERROR = 'error',
  CRITICAL = 'critical',
}

/**
 * Chiplet event
 */
export interface ChipletEvent {
  timestamp: number;
  chipletId: string;
  type: string;
  severity: EventSeverity;
  message: string;
  data?: any;
}

/**
 * Event handler callback
 */
export type EventHandler = (event: ChipletEvent) => void;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Error codes
 */
export enum ChipletErrorCode {
  NOT_FOUND = 'CHIPLET_NOT_FOUND',
  INVALID_CONFIG = 'INVALID_CONFIGURATION',
  COMMUNICATION_ERROR = 'COMMUNICATION_ERROR',
  POWER_ERROR = 'POWER_ERROR',
  THERMAL_ERROR = 'THERMAL_ERROR',
  INTERFACE_ERROR = 'INTERFACE_ERROR',
  INTEGRATION_ERROR = 'INTEGRATION_ERROR',
  VALIDATION_ERROR = 'VALIDATION_ERROR',
}

/**
 * Chiplet error
 */
export class ChipletError extends Error {
  constructor(
    public code: ChipletErrorCode,
    message: string,
    public chipletId?: string,
    public details?: any
  ) {
    super(message);
    this.name = 'ChipletError';
    Object.setPrototypeOf(this, ChipletError.prototype);
  }
}

// ============================================================================
// Manager Interfaces
// ============================================================================

/**
 * Chiplet manager interface
 */
export interface IChipletManager {
  discoverChiplets(): Promise<Chiplet[]>;
  getChiplet(id: string): Promise<Chiplet | null>;
  getChipletsByType(type: ChipletType): Promise<Chiplet[]>;
  addChiplet(chiplet: Chiplet): Promise<void>;
  removeChiplet(id: string): Promise<void>;
}

/**
 * Configuration manager interface
 */
export interface IConfigurationManager {
  configure(id: string, config: ChipletConfig): Promise<void>;
  getConfiguration(id: string): Promise<ChipletConfig>;
  validateConfiguration(config: ChipletConfig): Promise<ValidationResult>;
  resetConfiguration(id: string): Promise<void>;
}

/**
 * Performance monitor interface
 */
export interface IPerformanceMonitor {
  getMetrics(id: string): Promise<PerformanceMetrics>;
  getSystemPerformance(): Promise<SystemPerformance>;
  subscribe(id: string, callback: EventHandler): void;
  unsubscribe(id: string, callback: EventHandler): void;
  startMonitoring(interval?: number): void;
  stopMonitoring(): void;
}

/**
 * Integration manager interface
 */
export interface IIntegrationManager {
  createPlan(name: string): IntegrationPlan;
  addChiplet(plan: IntegrationPlan, chiplet: Chiplet, position: Position): void;
  addConnection(plan: IntegrationPlan, connection: ChipletConnection): void;
  validatePlan(plan: IntegrationPlan): ValidationResult;
  optimizePlan(plan: IntegrationPlan, constraints?: IntegrationConstraints): IntegrationPlan;
  exportPlan(plan: IntegrationPlan): string;
  importPlan(data: string): IntegrationPlan;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Optional fields helper
 */
export type PartialChiplet = Partial<Chiplet>;

/**
 * Required fields helper
 */
export type RequiredChiplet = Required<Chiplet>;

/**
 * Read-only chiplet
 */
export type ReadonlyChiplet = Readonly<Chiplet>;

/**
 * Chiplet filter function
 */
export type ChipletFilter = (chiplet: Chiplet) => boolean;

/**
 * Chiplet comparator function
 */
export type ChipletComparator = (a: Chiplet, b: Chiplet) => number;

// ============================================================================
// Constants
// ============================================================================

/**
 * Default configuration values
 */
export const DEFAULT_CONFIG: ChipletConfig = {
  powerMode: PowerMode.BALANCED,
  frequencyMHz: 2000,
  voltageV: 0.75,
  enabledFeatures: [],
  customSettings: {},
};

/**
 * UCIe specification constants
 */
export const UCIE_CONSTANTS = {
  MIN_LANES: 16,
  MAX_LANES: 64,
  MIN_VOLTAGE: 0.4,
  MAX_VOLTAGE: 1.0,
  STANDARD_BUMP_PITCH: 55,
  ADVANCED_BUMP_PITCH: 25,
} as const;

/**
 * Thermal constants
 */
export const THERMAL_CONSTANTS = {
  MAX_JUNCTION_TEMP: 100,
  THROTTLE_TEMP: 95,
  SHUTDOWN_TEMP: 105,
  AMBIENT_TEMP: 25,
} as const;

/**
 * Performance thresholds
 */
export const PERFORMANCE_THRESHOLDS = {
  HIGH_UTILIZATION: 80,
  LOW_UTILIZATION: 20,
  CRITICAL_POWER: 0.9,
  CRITICAL_THERMAL: 0.95,
} as const;
