/**
 * WIA AAC Main Class
 * Primary API entry point for AAC sensor interaction
 */

import { WiaAacEventEmitter } from './EventEmitter';
import { SignalValidator, getSignalValidator } from './SignalValidator';
import { ISensorAdapter } from '../adapters/BaseAdapter';
import {
  WiaAacSignal,
  DeviceInfo,
  SensorType,
  EventType,
  EventHandler,
  EventHandlerOptions,
  EventDataMap,
  WiaAacError,
  ErrorCode,
  SensorConfig,
  WiaAacOptions,
  DEFAULT_OPTIONS,
  ConnectionState,
  SensorOptions
} from '../types';

// Logger interface
interface Logger {
  debug: (msg: string, ...args: unknown[]) => void;
  info: (msg: string, ...args: unknown[]) => void;
  warn: (msg: string, ...args: unknown[]) => void;
  error: (msg: string, ...args: unknown[]) => void;
}

// Adapter registry
const adapterRegistry: Map<SensorType, new () => ISensorAdapter> = new Map();

export class WiaAac {
  private options: Required<WiaAacOptions>;
  private emitter: WiaAacEventEmitter;
  private validator: SignalValidator;
  private adapter: ISensorAdapter | null = null;
  private config: SensorConfig | null = null;
  private state: ConnectionState = ConnectionState.DISCONNECTED;
  private signalBuffer: WiaAacSignal[] = [];
  private reconnectAttempts = 0;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private logger: Logger;

  constructor(options: WiaAacOptions = {}) {
    this.options = { ...DEFAULT_OPTIONS, ...options };
    this.emitter = new WiaAacEventEmitter();
    this.validator = getSignalValidator();
    this.logger = this.createLogger();
  }

  // ============================================
  // Static Methods
  // ============================================

  /**
   * Register a custom adapter
   */
  static registerAdapter(type: SensorType, adapterClass: new () => ISensorAdapter): void {
    adapterRegistry.set(type, adapterClass);
  }

  /**
   * Get registered adapter class
   */
  static getAdapterClass(type: SensorType): (new () => ISensorAdapter) | undefined {
    return adapterRegistry.get(type);
  }

  // ============================================
  // Connection Management
  // ============================================

  /**
   * Connect to a sensor
   */
  async connect(config: SensorConfig): Promise<void> {
    if (this.state === ConnectionState.CONNECTED) {
      throw this.createError(ErrorCode.DEVICE_BUSY, 'Already connected to a device');
    }

    this.config = config;
    this.state = ConnectionState.CONNECTING;
    this.logger.info(`Connecting to ${config.type} sensor...`);

    try {
      // Get adapter (use pre-set adapter if available)
      if (!this.adapter) {
        const AdapterClass = adapterRegistry.get(config.type);
        if (!AdapterClass) {
          // Use mock adapter if no real adapter registered
          this.logger.warn(`No adapter registered for ${config.type}, using mock`);
          this.adapter = this.createMockAdapter(config.type);
        } else {
          this.adapter = new AdapterClass();
        }
      }

      // Connect
      await this.adapter.connect(config);

      // Setup signal handler
      this.adapter.onSignal((signal) => this.handleSignal(signal));

      // Apply options
      if (config.options) {
        this.adapter.configure(config.options);
      }

      this.state = ConnectionState.CONNECTED;
      this.reconnectAttempts = 0;
      this.logger.info(`Connected to ${config.device?.manufacturer || config.type}`);

      // Emit connected event
      const deviceInfo = this.adapter.deviceInfo || {
        manufacturer: config.device?.manufacturer || 'Unknown',
        model: config.device?.model || config.type
      };
      this.emitter.emit('connected', deviceInfo);

    } catch (error) {
      this.state = ConnectionState.ERROR;
      const wiaError = this.createError(
        ErrorCode.CONNECTION_FAILED,
        `Failed to connect: ${error instanceof Error ? error.message : 'Unknown error'}`
      );
      this.emitter.emit('error', wiaError);
      throw wiaError;
    }
  }

  /**
   * Disconnect from the sensor
   */
  async disconnect(): Promise<void> {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }

    if (this.adapter) {
      try {
        await this.adapter.disconnect();
      } catch (error) {
        this.logger.error('Error during disconnect:', error);
      }
      this.adapter = null;
    }

    this.state = ConnectionState.DISCONNECTED;
    this.config = null;
    this.signalBuffer = [];

    this.emitter.emit('disconnected', { reason: 'user' });
    this.logger.info('Disconnected');
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.state === ConnectionState.CONNECTED;
  }

  /**
   * Get connection state
   */
  getConnectionState(): ConnectionState {
    return this.state;
  }

  // ============================================
  // Device Discovery
  // ============================================

  /**
   * List available devices
   */
  async listDevices(): Promise<DeviceInfo[]> {
    // In a real implementation, this would scan for devices
    // For now, return empty list
    this.logger.debug('Listing devices...');
    return [];
  }

  /**
   * Get connected device info
   */
  getDeviceInfo(): DeviceInfo | null {
    return this.adapter?.deviceInfo || null;
  }

  // ============================================
  // Event Handling
  // ============================================

  /**
   * Subscribe to an event
   */
  on<T extends EventType>(
    event: T,
    handler: EventHandler<T>,
    options?: EventHandlerOptions<T>
  ): void {
    this.emitter.on(event, handler, options);
  }

  /**
   * Unsubscribe from an event
   */
  off<T extends EventType>(event: T, handler: EventHandler<T>): void {
    this.emitter.off(event, handler);
  }

  /**
   * Subscribe to an event once
   */
  once<T extends EventType>(event: T, handler: EventHandler<T>): void {
    this.emitter.once(event, handler);
  }

  // ============================================
  // Signal Management
  // ============================================

  /**
   * Get the last received signal
   */
  getLastSignal(): WiaAacSignal | null {
    return this.signalBuffer.length > 0
      ? this.signalBuffer[this.signalBuffer.length - 1]
      : null;
  }

  /**
   * Get signal buffer
   */
  getSignalBuffer(): WiaAacSignal[] {
    return [...this.signalBuffer];
  }

  // ============================================
  // Configuration
  // ============================================

  /**
   * Update sensor options
   */
  configure(options: Partial<SensorOptions>): void {
    if (this.adapter) {
      this.adapter.configure(options as SensorOptions);
    }
  }

  /**
   * Get current configuration
   */
  getConfig(): SensorConfig | null {
    return this.config;
  }

  // ============================================
  // Private Methods
  // ============================================

  private handleSignal(signal: WiaAacSignal): void {
    // Validate if enabled
    if (this.options.validateSignals) {
      if (!this.validator.isValidSignal(signal)) {
        this.logger.warn('Invalid signal received, skipping');
        return;
      }
    }

    // Add to buffer
    this.signalBuffer.push(signal);
    if (this.signalBuffer.length > this.options.signalBufferSize) {
      this.signalBuffer.shift();
    }

    // Emit signal event
    this.emitter.emit('signal', signal);

    // Check for selection events
    this.checkSelectionEvent(signal);
  }

  private checkSelectionEvent(signal: WiaAacSignal): void {
    // Eye tracker fixation -> selection
    if (signal.type === 'eye_tracker') {
      const fixation = signal.data.fixation;
      if (fixation?.active && fixation.duration_ms && fixation.duration_ms >= (this.config?.options?.dwellTime || 1000)) {
        this.emitter.emit('selection', {
          timestamp: signal.timestamp.unix_ms,
          targetId: fixation.target_id || 'unknown',
          targetType: 'area',
          selectionMethod: 'dwell',
          position: { x: signal.data.gaze.x, y: signal.data.gaze.y },
          confidence: signal.meta?.confidence || 0.8
        });
      }
    }

    // Switch press -> selection
    if (signal.type === 'switch' && signal.data.state === 'pressed') {
      this.emitter.emit('selection', {
        timestamp: signal.timestamp.unix_ms,
        targetId: `switch_${signal.data.switch_id}`,
        targetType: 'button',
        selectionMethod: 'switch',
        confidence: 1.0
      });
    }
  }

  private createError(code: ErrorCode, message: string): WiaAacError {
    return {
      code,
      message,
      timestamp: Date.now(),
      recoverable: code < 300
    };
  }

  private createLogger(): Logger {
    const level = this.options.logLevel;
    const levels = ['debug', 'info', 'warn', 'error', 'none'];
    const levelIndex = levels.indexOf(level);

    const noop = () => {};
    const log = (lvl: string) => (msg: string, ...args: unknown[]) => {
      console.log(`[WiaAac:${lvl}] ${msg}`, ...args);
    };

    return {
      debug: levelIndex <= 0 ? log('debug') : noop,
      info: levelIndex <= 1 ? log('info') : noop,
      warn: levelIndex <= 2 ? log('warn') : noop,
      error: levelIndex <= 3 ? log('error') : noop
    };
  }

  private createMockAdapter(type: SensorType): ISensorAdapter {
    // Import dynamically to avoid circular dependency
    const { MockAdapter } = require('../adapters/MockAdapter');
    return new MockAdapter({ type, simulateSignals: false });
  }

  /**
   * Use a specific adapter instance (for testing)
   */
  useAdapter(adapter: ISensorAdapter): void {
    this.adapter = adapter;
  }
}
