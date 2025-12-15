/**
 * WIA AAC Configuration Types
 */

import { SensorType } from './signal';

// Connection State
export enum ConnectionState {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  RECONNECTING = 'reconnecting',
  ERROR = 'error'
}

// WiaAac Options
export interface WiaAacOptions {
  // Auto reconnect
  autoReconnect?: boolean;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;

  // Signal processing
  signalBufferSize?: number;
  validateSignals?: boolean;

  // Logging
  logLevel?: 'debug' | 'info' | 'warn' | 'error' | 'none';
}

// Default Options
export const DEFAULT_OPTIONS: Required<WiaAacOptions> = {
  autoReconnect: true,
  reconnectInterval: 3000,
  maxReconnectAttempts: 5,
  signalBufferSize: 100,
  validateSignals: true,
  logLevel: 'info'
};

// Connection Config
export interface ConnectionConfig {
  protocol?: 'usb' | 'bluetooth' | 'wifi' | 'serial';
  port?: string;
  baudRate?: number;
}

// Base Sensor Options
export interface BaseSensorOptions {
  sampleRate?: number;
  sensitivity?: number;
  dwellTime?: number;
  smoothing?: boolean;
  smoothingFactor?: number;
}

// Eye Tracker Options
export interface EyeTrackerOptions extends BaseSensorOptions {
  trackBothEyes?: boolean;
  trackPupil?: boolean;
  trackBlink?: boolean;
  fixationThreshold?: number;
  gazeFilter?: 'none' | 'average' | 'kalman';
}

// Switch Options
export interface SwitchOptions extends BaseSensorOptions {
  debounceTime?: number;
  holdThreshold?: number;
  multiPressWindow?: number;
}

// Muscle Sensor Options
export interface MuscleSensorOptions extends BaseSensorOptions {
  activationThreshold?: number;
  gestureRecognition?: boolean;
  channels?: number[];
}

// Brain Interface Options
export interface BrainInterfaceOptions extends BaseSensorOptions {
  channels?: string[];
  bandPassFilter?: { low: number; high: number };
  artifactRejection?: boolean;
  classificationModel?: string;
}

// Breath Options
export interface BreathOptions extends BaseSensorOptions {
  sipThreshold?: number;
  puffThreshold?: number;
  hardThreshold?: number;
}

// Head Movement Options
export interface HeadMovementOptions extends BaseSensorOptions {
  trackRotation?: boolean;
  gestureRecognition?: boolean;
  dwellRadius?: number;
}

// Sensor Options Union
export type SensorOptions =
  | EyeTrackerOptions
  | SwitchOptions
  | MuscleSensorOptions
  | BrainInterfaceOptions
  | BreathOptions
  | HeadMovementOptions
  | BaseSensorOptions;

// Device Filter
export interface DeviceFilter {
  manufacturer?: string;
  model?: string;
  serial?: string;
}

// Sensor Config
export interface SensorConfig {
  type: SensorType;
  device?: DeviceFilter;
  connection?: ConnectionConfig;
  options?: SensorOptions;
}

// Sensor Option Descriptor (for adapter capabilities)
export interface SensorOptionDescriptor {
  name: string;
  type: 'number' | 'boolean' | 'string' | 'array';
  description: string;
  default?: unknown;
  min?: number;
  max?: number;
  options?: string[];
}
