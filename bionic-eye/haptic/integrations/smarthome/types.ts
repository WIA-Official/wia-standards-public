/**
 * WIA Haptic Standard - Smart Home Integration Types
 *
 * Types for Matter/Thread smart home integration with haptic feedback.
 */

import { HapticPattern, BodyLocation } from '../../api/typescript/src/types';

/**
 * Smart device types
 */
export type SmartDeviceType =
  | 'light'
  | 'switch'
  | 'dimmer'
  | 'thermostat'
  | 'door_lock'
  | 'door_sensor'
  | 'window_sensor'
  | 'motion_sensor'
  | 'temperature_sensor'
  | 'humidity_sensor'
  | 'speaker'
  | 'fan'
  | 'blind'
  | 'garage_door'
  | 'outlet'
  | 'unknown';

/**
 * Smart device state
 */
export interface SmartDeviceState {
  on?: boolean;
  brightness?: number;      // 0-100
  colorTemperature?: number; // Kelvin
  color?: { h: number; s: number; v: number };
  temperature?: number;      // Celsius
  targetTemperature?: number;
  humidity?: number;         // 0-100
  locked?: boolean;
  open?: boolean;
  position?: number;         // 0-100 for blinds
  volume?: number;           // 0-100
  speed?: number;            // Fan speed 0-100
}

/**
 * Smart device representation
 */
export interface SmartDevice {
  id: string;
  name: string;
  type: SmartDeviceType;
  room?: string;
  state: SmartDeviceState;
  direction?: number;        // Direction from user (degrees)
  distance?: number;         // Distance from user (meters)
  reachable: boolean;
}

/**
 * Smart device event
 */
export interface SmartDeviceEvent {
  deviceId: string;
  type: SmartDeviceEventType;
  previousState?: SmartDeviceState;
  newState: SmartDeviceState;
  timestamp: number;
}

export type SmartDeviceEventType =
  | 'state_changed'
  | 'connected'
  | 'disconnected'
  | 'triggered'           // Motion sensor, button press
  | 'alert';              // Smoke, leak detection

/**
 * Smart home haptic configuration
 */
export interface SmartHomeHapticConfig {
  // Enable haptic feedback for device control
  feedbackOnControl: boolean;

  // Enable haptic feedback for state changes
  feedbackOnStateChange: boolean;

  // Direction-based device selection
  spatialSelection: {
    enabled: boolean;
    angleThreshold: number;   // Degrees for matching
    confirmationDelay: number; // ms before confirming selection
  };

  // Haptic intensity levels
  intensity: {
    control: number;          // 0-1
    notification: number;
    alert: number;
  };

  // Temperature encoding range (Celsius)
  temperatureRange: {
    min: number;
    max: number;
  };
}

/**
 * Default smart home haptic configuration
 */
export const DEFAULT_SMARTHOME_CONFIG: SmartHomeHapticConfig = {
  feedbackOnControl: true,
  feedbackOnStateChange: true,
  spatialSelection: {
    enabled: true,
    angleThreshold: 30,
    confirmationDelay: 500,
  },
  intensity: {
    control: 0.6,
    notification: 0.4,
    alert: 1.0,
  },
  temperatureRange: {
    min: 10,
    max: 35,
  },
};

/**
 * Smart home haptic integration interface
 */
export interface SmartHomeHapticIntegration {
  config: SmartHomeHapticConfig;

  // Device discovery and management
  discoverDevices(): Promise<SmartDevice[]>;
  getDevice(id: string): SmartDevice | undefined;
  getDevicesByRoom(room: string): SmartDevice[];
  getDevicesByType(type: SmartDeviceType): SmartDevice[];

  // Spatial device selection
  pointToDevice(direction: number): SmartDevice | null;
  selectDevice(device: SmartDevice): void;
  confirmSelection(): void;
  cancelSelection(): void;

  // Device control with haptic feedback
  controlDevice(deviceId: string, action: DeviceAction): Promise<void>;

  // Event handling
  onDeviceEvent(event: SmartDeviceEvent): void;
}

/**
 * Device control actions
 */
export type DeviceAction =
  | { type: 'toggle' }
  | { type: 'on' }
  | { type: 'off' }
  | { type: 'setBrightness'; value: number }
  | { type: 'setTemperature'; value: number }
  | { type: 'setPosition'; value: number }
  | { type: 'lock' }
  | { type: 'unlock' }
  | { type: 'open' }
  | { type: 'close' };
