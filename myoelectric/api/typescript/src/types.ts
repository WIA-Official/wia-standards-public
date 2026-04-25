/**
 * WIA-MED-024: Myoelectric Control Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Type Aliases
// ============================================================================

export type Timestamp = string;
export type DeviceID = string;
export type SessionID = string;
export type PatientID = string;

// ============================================================================
// Enums
// ============================================================================

export enum SignalType {
  EMG = 'emg',
  EEG = 'eeg',
  EOG = 'eog',
  HYBRID = 'hybrid',
}

export enum DeviceStatus {
  CONNECTED = 'connected',
  DISCONNECTED = 'disconnected',
  CALIBRATING = 'calibrating',
  ACTIVE = 'active',
  ERROR = 'error',
  LOW_BATTERY = 'low_battery',
}

export enum GestureType {
  GRIP_OPEN = 'grip_open',
  GRIP_CLOSE = 'grip_close',
  WRIST_FLEX = 'wrist_flex',
  WRIST_EXTEND = 'wrist_extend',
  PINCH = 'pinch',
  POINT = 'point',
  REST = 'rest',
}

export enum ProstheticType {
  HAND = 'hand',
  ARM_BELOW_ELBOW = 'arm_below_elbow',
  ARM_ABOVE_ELBOW = 'arm_above_elbow',
  PARTIAL_HAND = 'partial_hand',
  LEG_BELOW_KNEE = 'leg_below_knee',
  LEG_ABOVE_KNEE = 'leg_above_knee',
}

export enum CalibrationStatus {
  NOT_CALIBRATED = 'not_calibrated',
  IN_PROGRESS = 'in_progress',
  COMPLETED = 'completed',
  FAILED = 'failed',
  EXPIRED = 'expired',
}

// ============================================================================
// Signal Types
// ============================================================================

export interface EMGSignal {
  signalId: string;
  deviceId: DeviceID;
  timestamp: Timestamp;
  channelData: ChannelData[];
  sampleRate: number;
  resolution: number;
  gain: number;
}

export interface ChannelData {
  channelId: number;
  values: number[];
  muscleGroup: string;
  quality: number;
}

export interface ProcessedSignal {
  signalId: string;
  rawSignalId: string;
  timestamp: Timestamp;
  features: SignalFeatures;
  classifiedGesture?: GestureType;
  confidence: number;
}

export interface SignalFeatures {
  meanAbsoluteValue: number;
  rmsValue: number;
  waveformLength: number;
  zeroCrossings: number;
  slopeSignChanges: number;
  frequency: FrequencyFeatures;
}

export interface FrequencyFeatures {
  meanFrequency: number;
  medianFrequency: number;
  peakFrequency: number;
  powerSpectrum: number[];
}

// ============================================================================
// Device Types
// ============================================================================

export interface MyoelectricDevice {
  deviceId: DeviceID;
  patientId: PatientID;
  prostheticType: ProstheticType;
  status: DeviceStatus;
  manufacturer: string;
  model: string;
  serialNumber: string;
  firmwareVersion: string;
  electrodeCount: number;
  batteryLevel: number;
  calibration: CalibrationInfo;
  lastSync: Timestamp;
}

export interface CalibrationInfo {
  status: CalibrationStatus;
  calibratedAt?: Timestamp;
  expiresAt?: Timestamp;
  gestures: GestureCalibration[];
  accuracy: number;
}

export interface GestureCalibration {
  gesture: GestureType;
  samples: number;
  threshold: number;
  accuracy: number;
  trained: boolean;
}

// ============================================================================
// Session Types
// ============================================================================

export interface ControlSession {
  sessionId: SessionID;
  deviceId: DeviceID;
  patientId: PatientID;
  startTime: Timestamp;
  endTime?: Timestamp;
  gestures: GestureEvent[];
  metrics: SessionMetrics;
}

export interface GestureEvent {
  eventId: string;
  gesture: GestureType;
  timestamp: Timestamp;
  confidence: number;
  duration: number;
  successful: boolean;
}

export interface SessionMetrics {
  totalGestures: number;
  successfulGestures: number;
  averageConfidence: number;
  averageLatency: number;
  fatigueFactor: number;
}

// ============================================================================
// Classifier Types
// ============================================================================

export interface ClassifierModel {
  modelId: string;
  name: string;
  version: string;
  patientId: PatientID;
  gestures: GestureType[];
  accuracy: number;
  trainedAt: Timestamp;
  samplesUsed: number;
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T = unknown> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Timestamp;
}

export interface APIError {
  code: string;
  message: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalCount: number;
  };
}
