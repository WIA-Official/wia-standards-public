/**
 * WIA BCI Base Adapter Interface
 * @module wia-bci/adapters/BaseAdapter
 */

import type {
  DeviceType,
  DeviceConfig,
  DeviceInfo,
  ChannelInfo,
  SignalEvent,
  ErrorEvent,
} from '../types';

/**
 * BCI Adapter Interface
 * All device-specific adapters must implement this interface
 */
export interface IBciAdapter {
  /** Device type this adapter handles */
  readonly type: DeviceType;
  /** Adapter name */
  readonly name: string;

  /**
   * Connect to the BCI device
   * @param config Device configuration
   */
  connect(config: DeviceConfig): Promise<void>;

  /**
   * Disconnect from the BCI device
   */
  disconnect(): Promise<void>;

  /**
   * Check if connected
   */
  isConnected(): boolean;

  /**
   * Start data streaming
   */
  startStream(): Promise<void>;

  /**
   * Stop data streaming
   */
  stopStream(): Promise<void>;

  /**
   * Check if streaming
   */
  isStreaming(): boolean;

  /**
   * Get device information
   */
  getDeviceInfo(): DeviceInfo | null;

  /**
   * Get channel information
   */
  getChannels(): ChannelInfo[];

  /**
   * Register data handler
   * @param handler Function to call when data is received
   */
  onData(handler: (data: SignalEvent) => void): void;

  /**
   * Register error handler
   * @param handler Function to call when error occurs
   */
  onError(handler: (error: ErrorEvent) => void): void;

  /**
   * Clean up resources
   */
  dispose(): void;
}

/**
 * Abstract Base Adapter
 * Provides common functionality for all adapters
 */
export abstract class BaseAdapter implements IBciAdapter {
  abstract readonly type: DeviceType;
  abstract readonly name: string;

  protected _connected = false;
  protected _streaming = false;
  protected deviceInfo: DeviceInfo | null = null;
  protected channels: ChannelInfo[] = [];
  protected dataHandler?: (data: SignalEvent) => void;
  protected errorHandler?: (error: ErrorEvent) => void;

  abstract connect(config: DeviceConfig): Promise<void>;
  abstract disconnect(): Promise<void>;
  abstract startStream(): Promise<void>;
  abstract stopStream(): Promise<void>;

  isConnected(): boolean {
    return this._connected;
  }

  isStreaming(): boolean {
    return this._streaming;
  }

  getDeviceInfo(): DeviceInfo | null {
    return this.deviceInfo;
  }

  getChannels(): ChannelInfo[] {
    return this.channels;
  }

  onData(handler: (data: SignalEvent) => void): void {
    this.dataHandler = handler;
  }

  onError(handler: (error: ErrorEvent) => void): void {
    this.errorHandler = handler;
  }

  dispose(): void {
    if (this._streaming) {
      this.stopStream().catch(() => {});
    }
    if (this._connected) {
      this.disconnect().catch(() => {});
    }
    this.dataHandler = undefined;
    this.errorHandler = undefined;
  }

  /**
   * Emit data event to handler
   */
  protected emitData(data: SignalEvent): void {
    this.dataHandler?.(data);
  }

  /**
   * Emit error event to handler
   */
  protected emitError(error: ErrorEvent): void {
    this.errorHandler?.(error);
  }
}
