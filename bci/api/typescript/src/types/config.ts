/**
 * WIA BCI Configuration Types
 * @module wia-bci/types/config
 */

/**
 * BCI Device Types
 */
export type DeviceType =
  | 'eeg_headset'
  | 'eeg_cap'
  | 'implant_cortical'
  | 'implant_endovascular'
  | 'fnirs'
  | 'hybrid'
  | 'simulator';

/**
 * Connection Protocol Types
 */
export type ConnectionProtocol =
  | 'usb'
  | 'bluetooth'
  | 'bluetooth_le'
  | 'wifi'
  | 'serial'
  | 'lsl';

/**
 * Log Levels
 */
export type LogLevel = 'debug' | 'info' | 'warn' | 'error';

/**
 * Filter Types
 */
export type FilterType = 'highpass' | 'lowpass' | 'bandpass' | 'notch';

/**
 * Filter Configuration
 */
export interface FilterConfig {
  type: FilterType;
  frequency: number | [number, number];
  order?: number;
}

/**
 * WiaBci Constructor Options
 */
export interface WiaBciOptions {
  /** Enable automatic reconnection on disconnect */
  autoReconnect?: boolean;
  /** Reconnection interval in milliseconds */
  reconnectInterval?: number;
  /** Maximum reconnection attempts */
  maxReconnectAttempts?: number;
  /** Internal buffer size for samples */
  bufferSize?: number;
  /** Logging level */
  logLevel?: LogLevel;
}

/**
 * Device-specific configuration
 */
export interface DeviceIdentifier {
  /** Device manufacturer */
  manufacturer?: string;
  /** Device model */
  model?: string;
  /** Device serial number */
  serialNumber?: string;
}

/**
 * Connection configuration
 */
export interface ConnectionConfig {
  /** Communication protocol */
  protocol?: ConnectionProtocol;
  /** Connection address (COM port, IP, etc.) */
  address?: string;
  /** Port number for network connections */
  port?: number;
}

/**
 * Acquisition configuration
 */
export interface AcquisitionConfig {
  /** Sampling rate in Hz */
  samplingRate?: number;
  /** Channels to acquire */
  channels?: string[];
  /** Signal filters to apply */
  filters?: FilterConfig[];
}

/**
 * Complete Device Configuration
 */
export interface DeviceConfig {
  /** Type of BCI device */
  type: DeviceType;
  /** Device identifier */
  device?: DeviceIdentifier;
  /** Connection settings */
  connection?: ConnectionConfig;
  /** Acquisition settings */
  acquisition?: AcquisitionConfig;
}

/**
 * Default configuration values
 */
export const DEFAULT_OPTIONS: Required<WiaBciOptions> = {
  autoReconnect: true,
  reconnectInterval: 3000,
  maxReconnectAttempts: 5,
  bufferSize: 1000,
  logLevel: 'info',
};
