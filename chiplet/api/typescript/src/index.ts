/**
 * WIA Chiplet Standard - TypeScript SDK
 *
 * @module @wia/chiplet
 * @version 1.0.0
 * @license MIT
 *
 * 弘益人間 · Benefit All Humanity
 *
 * This SDK provides a comprehensive implementation of the WIA-SEMI-004
 * Chiplet Standard, enabling developers to design, integrate, and manage
 * modular semiconductor chiplet architectures.
 */

import { EventEmitter } from 'eventemitter3';

// Re-export all types
export * from './types';

import {
  Chiplet,
  ChipletType,
  ChipletConfig,
  ChipletError,
  ChipletErrorCode,
  ChipletInterface,
  ChipletConnection,
  IntegrationPlan,
  IntegrationConstraints,
  PackageSpec,
  Position,
  PerformanceMetrics,
  SystemPerformance,
  ValidationResult,
  ValidationError,
  ChipletEvent,
  EventHandler,
  EventSeverity,
  PowerMode,
  InterfaceProtocol,
  PackageType,
  IChipletManager,
  IConfigurationManager,
  IPerformanceMonitor,
  IIntegrationManager,
  ChipletPlacement,
  DEFAULT_CONFIG,
  UCIE_CONSTANTS,
  THERMAL_CONSTANTS,
} from './types';

// ============================================================================
// Chiplet Manager Implementation
// ============================================================================

/**
 * Main chiplet management class
 */
export class ChipletManager extends EventEmitter implements IChipletManager {
  private chiplets: Map<string, Chiplet> = new Map();

  /**
   * Discover all chiplets in the system
   */
  async discoverChiplets(): Promise<Chiplet[]> {
    this.emit('discovery:start');

    try {
      // In a real implementation, this would interface with hardware
      const discovered = Array.from(this.chiplets.values());

      this.emit('discovery:complete', { count: discovered.length });
      return discovered;
    } catch (error) {
      this.emit('discovery:error', error);
      throw new ChipletError(
        ChipletErrorCode.COMMUNICATION_ERROR,
        'Failed to discover chiplets',
        undefined,
        error
      );
    }
  }

  /**
   * Get a specific chiplet by ID
   */
  async getChiplet(id: string): Promise<Chiplet | null> {
    const chiplet = this.chiplets.get(id);
    if (!chiplet) {
      return null;
    }
    return { ...chiplet }; // Return a copy
  }

  /**
   * Get chiplets by type
   */
  async getChipletsByType(type: ChipletType): Promise<Chiplet[]> {
    const filtered = Array.from(this.chiplets.values()).filter((c) => c.type === type);
    return filtered.map((c) => ({ ...c }));
  }

  /**
   * Add a chiplet to the system
   */
  async addChiplet(chiplet: Chiplet): Promise<void> {
    if (this.chiplets.has(chiplet.id)) {
      throw new ChipletError(
        ChipletErrorCode.INVALID_CONFIG,
        `Chiplet with ID ${chiplet.id} already exists`
      );
    }

    this.chiplets.set(chiplet.id, { ...chiplet });
    this.emit('chiplet:added', chiplet);
  }

  /**
   * Remove a chiplet from the system
   */
  async removeChiplet(id: string): Promise<void> {
    const chiplet = this.chiplets.get(id);
    if (!chiplet) {
      throw new ChipletError(ChipletErrorCode.NOT_FOUND, `Chiplet ${id} not found`);
    }

    this.chiplets.delete(id);
    this.emit('chiplet:removed', { id });
  }

  /**
   * Get all chiplet IDs
   */
  getChipletIds(): string[] {
    return Array.from(this.chiplets.keys());
  }

  /**
   * Get chiplet count
   */
  getChipletCount(): number {
    return this.chiplets.size;
  }
}

// ============================================================================
// Configuration Manager Implementation
// ============================================================================

/**
 * Chiplet configuration management
 */
export class ConfigurationManager extends EventEmitter implements IConfigurationManager {
  private configurations: Map<string, ChipletConfig> = new Map();
  private chipletManager: ChipletManager;

  constructor(chipletManager: ChipletManager) {
    super();
    this.chipletManager = chipletManager;
  }

  /**
   * Configure a chiplet
   */
  async configure(id: string, config: ChipletConfig): Promise<void> {
    const chiplet = await this.chipletManager.getChiplet(id);
    if (!chiplet) {
      throw new ChipletError(ChipletErrorCode.NOT_FOUND, `Chiplet ${id} not found`);
    }

    // Validate configuration
    const validation = await this.validateConfiguration(config);
    if (!validation.valid) {
      throw new ChipletError(
        ChipletErrorCode.INVALID_CONFIG,
        'Invalid configuration',
        id,
        validation.errors
      );
    }

    this.configurations.set(id, { ...config });
    this.emit('configuration:updated', { id, config });
  }

  /**
   * Get current configuration
   */
  async getConfiguration(id: string): Promise<ChipletConfig> {
    const config = this.configurations.get(id);
    if (!config) {
      // Return default configuration
      return { ...DEFAULT_CONFIG };
    }
    return { ...config };
  }

  /**
   * Validate configuration
   */
  async validateConfiguration(config: ChipletConfig): Promise<ValidationResult> {
    const errors: ValidationError[] = [];

    // Validate power mode
    if (!Object.values(PowerMode).includes(config.powerMode)) {
      errors.push({
        field: 'powerMode',
        message: 'Invalid power mode',
        code: 'INVALID_POWER_MODE',
      });
    }

    // Validate frequency
    if (config.frequencyMHz && (config.frequencyMHz < 100 || config.frequencyMHz > 10000)) {
      errors.push({
        field: 'frequencyMHz',
        message: 'Frequency must be between 100 and 10000 MHz',
        code: 'INVALID_FREQUENCY',
      });
    }

    // Validate voltage
    if (config.voltageV) {
      if (config.voltageV < UCIE_CONSTANTS.MIN_VOLTAGE || config.voltageV > UCIE_CONSTANTS.MAX_VOLTAGE) {
        errors.push({
          field: 'voltageV',
          message: `Voltage must be between ${UCIE_CONSTANTS.MIN_VOLTAGE}V and ${UCIE_CONSTANTS.MAX_VOLTAGE}V`,
          code: 'INVALID_VOLTAGE',
        });
      }
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
    };
  }

  /**
   * Reset configuration to defaults
   */
  async resetConfiguration(id: string): Promise<void> {
    this.configurations.set(id, { ...DEFAULT_CONFIG });
    this.emit('configuration:reset', { id });
  }
}

// ============================================================================
// Performance Monitor Implementation
// ============================================================================

/**
 * Performance monitoring and metrics collection
 */
export class PerformanceMonitor extends EventEmitter implements IPerformanceMonitor {
  private chipletManager: ChipletManager;
  private metrics: Map<string, PerformanceMetrics> = new Map();
  private monitoringInterval?: NodeJS.Timeout;
  private subscribers: Map<string, Set<EventHandler>> = new Map();

  constructor(chipletManager: ChipletManager) {
    super();
    this.chipletManager = chipletManager;
  }

  /**
   * Get performance metrics for a chiplet
   */
  async getMetrics(id: string): Promise<PerformanceMetrics> {
    const chiplet = await this.chipletManager.getChiplet(id);
    if (!chiplet) {
      throw new ChipletError(ChipletErrorCode.NOT_FOUND, `Chiplet ${id} not found`);
    }

    // In a real implementation, this would read from hardware
    const metrics = this.metrics.get(id) || this.generateMockMetrics(id);
    this.metrics.set(id, metrics);

    return { ...metrics };
  }

  /**
   * Get system-wide performance
   */
  async getSystemPerformance(): Promise<SystemPerformance> {
    const chiplets = await this.chipletManager.discoverChiplets();
    const metricsMap = new Map<string, PerformanceMetrics>();

    let totalCompute = 0;
    let totalMemoryBandwidth = 0;
    let totalPower = 0;
    let totalLatency = 0;
    let count = 0;

    for (const chiplet of chiplets) {
      const metrics = await this.getMetrics(chiplet.id);
      metricsMap.set(chiplet.id, metrics);

      if (metrics.compute) {
        totalCompute += metrics.compute.throughput || 0;
      }

      if (metrics.memory) {
        totalMemoryBandwidth += metrics.memory.bandwidth;
        totalLatency += metrics.memory.latency;
        count++;
      }

      if (metrics.power) {
        totalPower += metrics.power.current;
      }
    }

    const averageLatency = count > 0 ? totalLatency / count : 0;
    const efficiency = totalPower > 0 ? totalCompute / totalPower : 0;

    return {
      totalCompute,
      totalMemoryBandwidth,
      totalPower,
      averageLatency,
      efficiency,
      chiplets: metricsMap,
    };
  }

  /**
   * Subscribe to performance events
   */
  subscribe(id: string, callback: EventHandler): void {
    if (!this.subscribers.has(id)) {
      this.subscribers.set(id, new Set());
    }
    this.subscribers.get(id)!.add(callback);
  }

  /**
   * Unsubscribe from performance events
   */
  unsubscribe(id: string, callback: EventHandler): void {
    const subs = this.subscribers.get(id);
    if (subs) {
      subs.delete(callback);
    }
  }

  /**
   * Start monitoring
   */
  startMonitoring(interval: number = 1000): void {
    if (this.monitoringInterval) {
      this.stopMonitoring();
    }

    this.monitoringInterval = setInterval(async () => {
      const chiplets = await this.chipletManager.discoverChiplets();

      for (const chiplet of chiplets) {
        const metrics = await this.getMetrics(chiplet.id);

        // Check for thermal issues
        if (metrics.thermal && metrics.thermal.junction > THERMAL_CONSTANTS.THROTTLE_TEMP) {
          this.notifySubscribers(chiplet.id, {
            timestamp: Date.now(),
            chipletId: chiplet.id,
            type: 'thermal:warning',
            severity: EventSeverity.WARNING,
            message: `Junction temperature ${metrics.thermal.junction}°C exceeds throttle threshold`,
            data: metrics.thermal,
          });
        }

        this.emit('metrics:updated', { chipletId: chiplet.id, metrics });
      }
    }, interval);
  }

  /**
   * Stop monitoring
   */
  stopMonitoring(): void {
    if (this.monitoringInterval) {
      clearInterval(this.monitoringInterval);
      this.monitoringInterval = undefined;
    }
  }

  /**
   * Notify subscribers of an event
   */
  private notifySubscribers(id: string, event: ChipletEvent): void {
    const subs = this.subscribers.get(id);
    if (subs) {
      subs.forEach((callback) => callback(event));
    }
  }

  /**
   * Generate mock metrics for testing
   */
  private generateMockMetrics(id: string): PerformanceMetrics {
    return {
      timestamp: Date.now(),
      chipletId: id,
      compute: {
        utilization: Math.random() * 100,
        frequency: 3000 + Math.random() * 500,
        throughput: 100 + Math.random() * 50,
      },
      memory: {
        bandwidth: 800 + Math.random() * 400,
        latency: 50 + Math.random() * 50,
        utilization: Math.random() * 100,
      },
      power: {
        current: 30 + Math.random() * 20,
        average: 35,
        peak: 50,
        idle: 5,
        voltage: 0.75,
      },
      thermal: {
        junction: 60 + Math.random() * 30,
        ambient: 25,
      },
      interface: {
        bandwidth: 256 + Math.random() * 128,
        utilization: Math.random() * 100,
        latency: 4 + Math.random() * 2,
      },
    };
  }
}

// ============================================================================
// Integration Manager Implementation
// ============================================================================

/**
 * Chiplet integration planning and optimization
 */
export class IntegrationManager extends EventEmitter implements IIntegrationManager {
  private chipletManager: ChipletManager;

  constructor(chipletManager: ChipletManager) {
    super();
    this.chipletManager = chipletManager;
  }

  /**
   * Create a new integration plan
   */
  createPlan(name: string): IntegrationPlan {
    return {
      name,
      version: '1.0.0',
      chiplets: [],
      connections: [],
      package: {
        type: PackageType.INTERPOSER_2_5D,
        dimensions: { width: 50, height: 50, unit: 'mm' },
      },
      validated: false,
    };
  }

  /**
   * Add a chiplet to the integration plan
   */
  addChiplet(plan: IntegrationPlan, chiplet: Chiplet, position: Position): void {
    const placement: ChipletPlacement = {
      chipletId: chiplet.id,
      position,
    };

    plan.chiplets.push(placement);
    plan.validated = false;
    this.emit('plan:chiplet-added', { plan, chiplet, position });
  }

  /**
   * Add a connection between chiplets
   */
  addConnection(plan: IntegrationPlan, connection: ChipletConnection): void {
    // Validate that both chiplets exist in the plan
    const hasSource = plan.chiplets.some((p) => p.chipletId === connection.source);
    const hasDest = plan.chiplets.some((p) => p.chipletId === connection.destination);

    if (!hasSource || !hasDest) {
      throw new ChipletError(
        ChipletErrorCode.INTEGRATION_ERROR,
        'Both source and destination chiplets must be in the plan'
      );
    }

    plan.connections.push(connection);
    plan.validated = false;
    this.emit('plan:connection-added', { plan, connection });
  }

  /**
   * Validate an integration plan
   */
  validatePlan(plan: IntegrationPlan): ValidationResult {
    const errors: ValidationError[] = [];

    // Check for duplicate chiplets
    const chipletIds = plan.chiplets.map((p) => p.chipletId);
    const duplicates = chipletIds.filter((id, index) => chipletIds.indexOf(id) !== index);
    if (duplicates.length > 0) {
      errors.push({
        field: 'chiplets',
        message: `Duplicate chiplets found: ${duplicates.join(', ')}`,
        code: 'DUPLICATE_CHIPLETS',
      });
    }

    // Validate connections reference valid chiplets
    for (const conn of plan.connections) {
      if (!chipletIds.includes(conn.source)) {
        errors.push({
          field: 'connections',
          message: `Connection source ${conn.source} not found in plan`,
          code: 'INVALID_CONNECTION_SOURCE',
        });
      }
      if (!chipletIds.includes(conn.destination)) {
        errors.push({
          field: 'connections',
          message: `Connection destination ${conn.destination} not found in plan`,
          code: 'INVALID_CONNECTION_DESTINATION',
        });
      }
    }

    // Validate lane counts
    for (const conn of plan.connections) {
      if (conn.lanes < UCIE_CONSTANTS.MIN_LANES || conn.lanes > UCIE_CONSTANTS.MAX_LANES) {
        errors.push({
          field: 'connections',
          message: `Lane count ${conn.lanes} out of valid range (${UCIE_CONSTANTS.MIN_LANES}-${UCIE_CONSTANTS.MAX_LANES})`,
          code: 'INVALID_LANE_COUNT',
        });
      }
    }

    const valid = errors.length === 0;
    plan.validated = valid;

    return {
      valid,
      errors: errors.length > 0 ? errors : undefined,
    };
  }

  /**
   * Optimize integration plan
   */
  optimizePlan(
    plan: IntegrationPlan,
    constraints?: IntegrationConstraints
  ): IntegrationPlan {
    const optimized = { ...plan };

    // Simple optimization: arrange chiplets in a grid
    const gridSize = Math.ceil(Math.sqrt(plan.chiplets.length));
    const spacing = 15; // mm

    plan.chiplets.forEach((placement, index) => {
      const row = Math.floor(index / gridSize);
      const col = index % gridSize;

      placement.position = {
        x: col * spacing,
        y: row * spacing,
      };
    });

    // Update package size based on layout
    const maxX = Math.max(...plan.chiplets.map((p) => p.position.x)) + spacing;
    const maxY = Math.max(...plan.chiplets.map((p) => p.position.y)) + spacing;

    optimized.package.dimensions = {
      width: maxX,
      height: maxY,
      unit: 'mm',
    };

    this.emit('plan:optimized', { plan: optimized });
    return optimized;
  }

  /**
   * Export integration plan to JSON
   */
  exportPlan(plan: IntegrationPlan): string {
    return JSON.stringify(plan, null, 2);
  }

  /**
   * Import integration plan from JSON
   */
  importPlan(data: string): IntegrationPlan {
    try {
      const plan = JSON.parse(data) as IntegrationPlan;
      this.emit('plan:imported', { plan });
      return plan;
    } catch (error) {
      throw new ChipletError(
        ChipletErrorCode.VALIDATION_ERROR,
        'Failed to parse integration plan',
        undefined,
        error
      );
    }
  }
}

// ============================================================================
// Unified Chiplet SDK
// ============================================================================

/**
 * Main SDK class providing unified access to all chiplet functionality
 */
export class ChipletSDK {
  public readonly manager: ChipletManager;
  public readonly config: ConfigurationManager;
  public readonly performance: PerformanceMonitor;
  public readonly integration: IntegrationManager;

  constructor() {
    this.manager = new ChipletManager();
    this.config = new ConfigurationManager(this.manager);
    this.performance = new PerformanceMonitor(this.manager);
    this.integration = new IntegrationManager(this.manager);
  }

  /**
   * Initialize the SDK
   */
  async initialize(): Promise<void> {
    const chiplets = await this.manager.discoverChiplets();
    console.log(`WIA Chiplet SDK initialized with ${chiplets.length} chiplets`);
    console.log('弘益人間 · Benefit All Humanity');
  }

  /**
   * Shutdown the SDK
   */
  async shutdown(): Promise<void> {
    this.performance.stopMonitoring();
    console.log('WIA Chiplet SDK shutdown complete');
  }
}

// ============================================================================
// Convenience Factory Functions
// ============================================================================

/**
 * Create a new chiplet SDK instance
 */
export function createChipletSDK(): ChipletSDK {
  return new ChipletSDK();
}

/**
 * Create a sample chiplet for testing
 */
export function createSampleChiplet(type: ChipletType, id?: string): Chiplet {
  const chipletId = id || `${type}-${Date.now()}`;

  return {
    id: chipletId,
    type,
    version: '1.0.0',
    vendor: 'WIA Sample',
    technology: {
      processNode: '5nm',
      dieSize: { width: 10, height: 10, unit: 'mm' },
    },
    interfaces: [
      {
        type: 'UCIe',
        version: '1.1',
        lanes: 32,
        protocol: InterfaceProtocol.CXL,
        bandwidth: 512,
      },
    ],
    power: {
      tdp: 50,
      idle: 5,
      voltageRanges: [
        {
          domain: 'core',
          min: 0.6,
          max: 1.0,
          typical: 0.75,
          unit: 'V',
        },
      ],
    },
    thermal: {
      tjMax: 100,
      thermalResistance: 0.3,
    },
  };
}

// ============================================================================
// Default Export
// ============================================================================

export default ChipletSDK;
