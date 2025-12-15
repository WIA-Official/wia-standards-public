/**
 * WIA BCI Main Class
 * @module wia-bci/core/WiaBci
 */

import { BciEventEmitter } from './EventEmitter';
import { BciError, ErrorCodes } from './BciError';
import type {
  WiaBciOptions,
  DeviceConfig,
  DeviceInfo,
  ChannelInfo,
  BciState,
  EventType,
  EventHandler,
  SignalEvent,
  DEFAULT_OPTIONS,
} from '../types';
import type { IBciAdapter } from '../adapters/BaseAdapter';
import { SimulatorAdapter } from '../adapters/SimulatorAdapter';

/**
 * Logger interface
 */
interface Logger {
  debug: (message: string, ...args: unknown[]) => void;
  info: (message: string, ...args: unknown[]) => void;
  warn: (message: string, ...args: unknown[]) => void;
  error: (message: string, ...args: unknown[]) => void;
}

/**
 * Create logger based on log level
 */
function createLogger(level: WiaBciOptions['logLevel']): Logger {
  const levels = ['debug', 'info', 'warn', 'error'];
  const minLevel = levels.indexOf(level ?? 'info');

  const log = (logLevel: string, message: string, ...args: unknown[]) => {
    if (levels.indexOf(logLevel) >= minLevel) {
      const prefix = `[WIA-BCI] [${logLevel.toUpperCase()}]`;
      console[logLevel as 'log'](prefix, message, ...args);
    }
  };

  return {
    debug: (message, ...args) => log('debug', message, ...args),
    info: (message, ...args) => log('info', message, ...args),
    warn: (message, ...args) => log('warn', message, ...args),
    error: (message, ...args) => log('error', message, ...args),
  };
}

/**
 * WiaBci - Main BCI Interface Class
 *
 * @example
 * ```typescript
 * const bci = new WiaBci();
 *
 * await bci.connect({ type: 'eeg_headset' });
 *
 * bci.on('signal', (event) => {
 *   console.log('Signal:', event.data);
 * });
 *
 * await bci.startStream();
 * ```
 */
export class WiaBci {
  private emitter: BciEventEmitter;
  private logger: Logger;
  private options: Required<WiaBciOptions>;
  private adapter: IBciAdapter | null = null;
  private _connected = false;
  private _streaming = false;
  private samplesReceived = 0;
  private reconnectAttempts = 0;

  constructor(options?: WiaBciOptions) {
    this.options = {
      autoReconnect: options?.autoReconnect ?? true,
      reconnectInterval: options?.reconnectInterval ?? 3000,
      maxReconnectAttempts: options?.maxReconnectAttempts ?? 5,
      bufferSize: options?.bufferSize ?? 1000,
      logLevel: options?.logLevel ?? 'info',
    };

    this.emitter = new BciEventEmitter();
    this.logger = createLogger(this.options.logLevel);

    this.logger.debug('WiaBci initialized', this.options);
  }

  /**
   * List available BCI devices
   */
  async listDevices(): Promise<DeviceInfo[]> {
    this.logger.debug('Listing devices...');

    // In a real implementation, this would scan for devices
    // For now, return simulator as available
    const devices: DeviceInfo[] = [
      {
        id: 'simulator-001',
        name: 'WIA BCI Simulator',
        type: 'simulator',
        manufacturer: 'WIA',
        model: 'Simulator',
        capabilities: {
          channels: 8,
          maxSamplingRate: 250,
          supportedSamplingRates: [125, 250],
          resolution: 24,
          hasAccelerometer: false,
          hasGyroscope: false,
          hasImpedanceCheck: false,
          hasBatteryIndicator: false,
          supportedProtocols: [],
        },
        status: 'available',
      },
    ];

    return devices;
  }

  /**
   * Connect to a BCI device
   */
  async connect(config: DeviceConfig): Promise<void> {
    if (this._connected) {
      throw BciError.alreadyConnected();
    }

    this.logger.info('Connecting to device...', config);

    try {
      // Select appropriate adapter based on device type
      this.adapter = this.createAdapter(config);

      // Connect via adapter
      await this.adapter.connect(config);

      // Set up data handler
      this.adapter.onData((event) => {
        this.samplesReceived++;
        this.emitter.emit('signal', event);
        this.emitter.emit('data', event);
      });

      // Set up error handler
      this.adapter.onError((error) => {
        this.logger.error('Adapter error:', error);
        this.emitter.emit('error', error);
      });

      this._connected = true;
      this.reconnectAttempts = 0;

      this.logger.info('Connected successfully');
      this.emitter.emit('connected');
    } catch (error) {
      this.logger.error('Connection failed:', error);

      if (error instanceof BciError) {
        throw error;
      }

      throw BciError.connectionFailed(
        error instanceof Error ? error.message : 'Unknown error',
        error
      );
    }
  }

  /**
   * Disconnect from the current device
   */
  async disconnect(): Promise<void> {
    if (!this._connected || !this.adapter) {
      throw BciError.notConnected();
    }

    this.logger.info('Disconnecting...');

    if (this._streaming) {
      await this.stopStream();
    }

    await this.adapter.disconnect();
    this.adapter = null;
    this._connected = false;

    this.logger.info('Disconnected');
    this.emitter.emit('disconnected');
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this._connected;
  }

  /**
   * Start data streaming
   */
  async startStream(): Promise<void> {
    if (!this._connected || !this.adapter) {
      throw BciError.notConnected();
    }

    if (this._streaming) {
      this.logger.warn('Already streaming');
      return;
    }

    this.logger.info('Starting stream...');

    await this.adapter.startStream();
    this._streaming = true;

    this.logger.info('Stream started');
    this.emitter.emit('stream_started');
  }

  /**
   * Stop data streaming
   */
  async stopStream(): Promise<void> {
    if (!this._connected || !this.adapter) {
      throw BciError.notConnected();
    }

    if (!this._streaming) {
      this.logger.warn('Not streaming');
      return;
    }

    this.logger.info('Stopping stream...');

    await this.adapter.stopStream();
    this._streaming = false;

    this.logger.info('Stream stopped');
    this.emitter.emit('stream_stopped');
  }

  /**
   * Check if streaming
   */
  isStreaming(): boolean {
    return this._streaming;
  }

  /**
   * Get current device info
   */
  getDeviceInfo(): DeviceInfo | null {
    return this.adapter?.getDeviceInfo() ?? null;
  }

  /**
   * Get channel information
   */
  getChannels(): ChannelInfo[] {
    return this.adapter?.getChannels() ?? [];
  }

  /**
   * Get current BCI state
   */
  getState(): BciState {
    return {
      connected: this._connected,
      streaming: this._streaming,
      device: this.getDeviceInfo(),
      channels: this.getChannels(),
      samplingRate: this.adapter?.getDeviceInfo()?.capabilities.maxSamplingRate ?? 0,
      samplesReceived: this.samplesReceived,
      lastError: null,
    };
  }

  /**
   * Subscribe to an event
   */
  on<T extends EventType>(event: T, handler: EventHandler<T>): this {
    this.emitter.on(event, handler);
    return this;
  }

  /**
   * Unsubscribe from an event
   */
  off<T extends EventType>(event: T, handler: EventHandler<T>): this {
    this.emitter.off(event, handler);
    return this;
  }

  /**
   * Subscribe to an event once
   */
  once<T extends EventType>(event: T, handler: EventHandler<T>): this {
    this.emitter.once(event, handler);
    return this;
  }

  /**
   * Emit an event (for external use, e.g., markers)
   */
  emit<T extends EventType>(event: T, data?: Parameters<EventHandler<T>>[0]): this {
    this.emitter.emit(event, data as never);
    return this;
  }

  /**
   * Clean up resources
   */
  dispose(): void {
    this.logger.debug('Disposing WiaBci...');

    if (this._connected && this.adapter) {
      this.adapter.dispose();
      this.adapter = null;
    }

    this.emitter.removeAllListeners();
    this._connected = false;
    this._streaming = false;

    this.logger.debug('Disposed');
  }

  /**
   * Create appropriate adapter for device type
   */
  private createAdapter(config: DeviceConfig): IBciAdapter {
    switch (config.type) {
      case 'simulator':
        return new SimulatorAdapter();

      case 'eeg_headset':
      case 'eeg_cap':
        // In production, would check manufacturer and return appropriate adapter
        // For now, use simulator
        this.logger.warn(`Using simulator for device type: ${config.type}`);
        return new SimulatorAdapter();

      default:
        throw BciError.invalidConfig(`Unsupported device type: ${config.type}`);
    }
  }
}
