/**
 * WIA AAC Base Adapter
 * Abstract base class for all sensor adapters
 */

import {
  WiaAacSignal,
  DeviceInfo,
  SensorType,
  SensorConfig,
  SensorOptions,
  SensorOptionDescriptor
} from '../types';

export type SignalHandler = (signal: WiaAacSignal) => void;

export interface ISensorAdapter {
  readonly type: SensorType;
  readonly deviceInfo: DeviceInfo | null;
  readonly isConnected: boolean;

  connect(config: SensorConfig): Promise<void>;
  disconnect(): Promise<void>;

  getLastSignal(): WiaAacSignal | null;
  onSignal(handler: SignalHandler): void;
  offSignal(handler: SignalHandler): void;

  configure(options: SensorOptions): void;
  getSupportedOptions(): SensorOptionDescriptor[];
}

export abstract class BaseAdapter implements ISensorAdapter {
  abstract readonly type: SensorType;

  protected _deviceInfo: DeviceInfo | null = null;
  protected _isConnected = false;
  protected _lastSignal: WiaAacSignal | null = null;
  protected signalHandlers: Set<SignalHandler> = new Set();
  protected config: SensorConfig | null = null;
  protected options: SensorOptions = {};

  get deviceInfo(): DeviceInfo | null {
    return this._deviceInfo;
  }

  get isConnected(): boolean {
    return this._isConnected;
  }

  async connect(config: SensorConfig): Promise<void> {
    this.config = config;
    this._deviceInfo = {
      manufacturer: config.device?.manufacturer || 'Unknown',
      model: config.device?.model || this.type,
      firmware: '1.0.0',
      serial: config.device?.serial
    };
    this._isConnected = true;
  }

  async disconnect(): Promise<void> {
    this._isConnected = false;
    this._deviceInfo = null;
    this.config = null;
    this._lastSignal = null;
  }

  getLastSignal(): WiaAacSignal | null {
    return this._lastSignal;
  }

  onSignal(handler: SignalHandler): void {
    this.signalHandlers.add(handler);
  }

  offSignal(handler: SignalHandler): void {
    this.signalHandlers.delete(handler);
  }

  configure(options: SensorOptions): void {
    this.options = { ...this.options, ...options };
  }

  abstract getSupportedOptions(): SensorOptionDescriptor[];

  protected emitSignal(signal: WiaAacSignal): void {
    this._lastSignal = signal;
    for (const handler of this.signalHandlers) {
      try {
        handler(signal);
      } catch (error) {
        console.error('Error in signal handler:', error);
      }
    }
  }

  protected createSignal<T extends SensorType>(
    type: T,
    data: WiaAacSignal['data'],
    confidence = 0.9
  ): WiaAacSignal {
    return {
      version: '1.0.0',
      type,
      timestamp: {
        unix_ms: Date.now(),
        iso8601: new Date().toISOString()
      },
      sequence: Date.now(),
      device: this._deviceInfo || { manufacturer: 'Unknown', model: type },
      data,
      meta: {
        confidence,
        validity: true
      }
    } as WiaAacSignal;
  }
}
