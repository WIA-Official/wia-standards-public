/**
 * WIA BCI Device Types
 * @module wia-bci/types/device
 */

import type { ConnectionProtocol, DeviceType } from './config';

/**
 * Device Status
 */
export type DeviceStatus =
  | 'available'
  | 'connected'
  | 'streaming'
  | 'error'
  | 'disconnected';

/**
 * Device Capabilities
 */
export interface DeviceCapabilities {
  /** Number of recording channels */
  channels: number;
  /** Maximum sampling rate in Hz */
  maxSamplingRate: number;
  /** Supported sampling rates */
  supportedSamplingRates: number[];
  /** ADC resolution in bits */
  resolution: number;

  /** Has built-in accelerometer */
  hasAccelerometer: boolean;
  /** Has built-in gyroscope */
  hasGyroscope: boolean;
  /** Supports impedance checking */
  hasImpedanceCheck: boolean;
  /** Has battery level indicator */
  hasBatteryIndicator: boolean;

  /** Supported connection protocols */
  supportedProtocols: ConnectionProtocol[];
}

/**
 * Device Information
 */
export interface DeviceInfo {
  /** Unique device identifier */
  id: string;
  /** Device display name */
  name: string;
  /** Device type */
  type: DeviceType;
  /** Manufacturer name */
  manufacturer: string;
  /** Model name */
  model: string;
  /** Serial number */
  serialNumber?: string;
  /** Firmware version */
  firmwareVersion?: string;

  /** Device capabilities */
  capabilities: DeviceCapabilities;
  /** Current device status */
  status: DeviceStatus;
}

/**
 * Channel Information
 */
export interface ChannelInfo {
  /** Channel index (0-based) */
  index: number;
  /** Channel label (e.g., "Fp1", "C3") */
  label: string;
  /** Channel type */
  type: 'eeg' | 'emg' | 'eog' | 'ecg' | 'trigger' | 'accel' | 'aux';
  /** Physical unit */
  unit: string;
  /** Sampling rate (if different from global) */
  samplingRate?: number;
  /** Whether channel is enabled */
  enabled: boolean;
}

/**
 * BCI State
 */
export interface BciState {
  /** Whether connected to a device */
  connected: boolean;
  /** Whether streaming data */
  streaming: boolean;
  /** Current device info */
  device: DeviceInfo | null;
  /** Active channels */
  channels: ChannelInfo[];
  /** Current sampling rate */
  samplingRate: number;
  /** Total samples received */
  samplesReceived: number;
  /** Last error (if any) */
  lastError: string | null;
}

/**
 * Default channel configurations for standard placements
 */
export const STANDARD_10_20_CHANNELS: readonly string[] = [
  'Fp1', 'Fp2',
  'F7', 'F3', 'Fz', 'F4', 'F8',
  'T3', 'C3', 'Cz', 'C4', 'T4',
  'T5', 'P3', 'Pz', 'P4', 'T6',
  'O1', 'O2',
] as const;

export const STANDARD_10_10_CHANNELS: readonly string[] = [
  'Fp1', 'Fpz', 'Fp2',
  'AF7', 'AF3', 'AFz', 'AF4', 'AF8',
  'F7', 'F5', 'F3', 'F1', 'Fz', 'F2', 'F4', 'F6', 'F8',
  'FT7', 'FC5', 'FC3', 'FC1', 'FCz', 'FC2', 'FC4', 'FC6', 'FT8',
  'T7', 'C5', 'C3', 'C1', 'Cz', 'C2', 'C4', 'C6', 'T8',
  'TP7', 'CP5', 'CP3', 'CP1', 'CPz', 'CP2', 'CP4', 'CP6', 'TP8',
  'P7', 'P5', 'P3', 'P1', 'Pz', 'P2', 'P4', 'P6', 'P8',
  'PO7', 'PO3', 'POz', 'PO4', 'PO8',
  'O1', 'Oz', 'O2',
] as const;
