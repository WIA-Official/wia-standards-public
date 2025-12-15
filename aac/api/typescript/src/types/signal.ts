/**
 * WIA AAC Signal Types
 * Based on Phase 1 Signal Format Standard
 */

// Sensor Types
export type SensorType =
  | 'eye_tracker'
  | 'switch'
  | 'muscle_sensor'
  | 'brain_interface'
  | 'breath'
  | 'head_movement'
  | 'custom';

// Timestamp
export interface Timestamp {
  unix_ms: number;
  iso8601?: string;
}

// Device Info
export interface DeviceInfo {
  manufacturer: string;
  model: string;
  firmware?: string;
  serial?: string;
}

// Meta
export interface SignalMeta {
  confidence?: number;
  validity?: boolean;
  raw?: Record<string, unknown>;
}

// Base Signal
export interface WiaAacSignalBase {
  $schema?: string;
  version: string;
  type: SensorType;
  timestamp: Timestamp;
  sequence?: number;
  device: DeviceInfo;
  meta?: SignalMeta;
}

// Eye Tracker Data
export interface EyeTrackerData {
  gaze: {
    x: number;
    y: number;
    z?: number | null;
  };
  fixation?: {
    active: boolean;
    duration_ms?: number;
    target_id?: string;
  };
  pupil?: {
    left_diameter_mm?: number;
    right_diameter_mm?: number;
  };
  blink?: {
    detected: boolean;
    duration_ms?: number;
  };
  eye_validity?: {
    left: boolean;
    right: boolean;
  };
}

// Switch Data
export interface SwitchData {
  switch_id: number;
  channel?: string;
  state: 'pressed' | 'released' | 'held';
  duration_ms?: number;
  pressure?: number | null;
  repeat_count?: number;
}

// Muscle Sensor Data
export interface MuscleSensorData {
  channel_id: number;
  muscle_group?: string;
  activation: number;
  raw_uv?: number;
  envelope_uv?: number;
  threshold_exceeded?: boolean;
  gesture?: string;
}

// Brain Interface Data
export interface BrainInterfaceData {
  channel_count: number;
  sample_rate_hz?: number;
  channels: Array<{
    id: string;
    value_uv: number;
  }>;
  bands?: {
    delta?: number;
    theta?: number;
    alpha?: number;
    beta?: number;
    gamma?: number;
  };
  classification?: {
    intent: string;
    confidence: number;
  };
  artifacts?: {
    eye_blink?: boolean;
    muscle?: boolean;
    movement?: boolean;
  };
}

// Breath Data
export interface BreathData {
  action: 'sip' | 'hard_sip' | 'puff' | 'hard_puff' | 'neutral';
  pressure_kpa?: number;
  pressure_normalized?: number;
  duration_ms?: number;
  intensity?: 'soft' | 'medium' | 'hard';
  baseline_kpa?: number;
}

// Head Movement Data
export interface HeadMovementData {
  position: {
    x: number;
    y: number;
  };
  rotation?: {
    pitch?: number;
    yaw?: number;
    roll?: number;
  };
  velocity?: {
    x?: number;
    y?: number;
  };
  gesture?: 'dwell' | 'nod' | 'shake' | 'tilt_left' | 'tilt_right' | 'none';
  dwell_time_ms?: number;
  face_detected?: boolean;
}

// Custom Data
export interface CustomData {
  custom_type: string;
  custom_data: Record<string, unknown>;
}

// Sensor Data Union
export type SensorData =
  | EyeTrackerData
  | SwitchData
  | MuscleSensorData
  | BrainInterfaceData
  | BreathData
  | HeadMovementData
  | CustomData;

// Full Signal Types
export interface EyeTrackerSignal extends WiaAacSignalBase {
  type: 'eye_tracker';
  data: EyeTrackerData;
}

export interface SwitchSignal extends WiaAacSignalBase {
  type: 'switch';
  data: SwitchData;
}

export interface MuscleSensorSignal extends WiaAacSignalBase {
  type: 'muscle_sensor';
  data: MuscleSensorData;
}

export interface BrainInterfaceSignal extends WiaAacSignalBase {
  type: 'brain_interface';
  data: BrainInterfaceData;
}

export interface BreathSignal extends WiaAacSignalBase {
  type: 'breath';
  data: BreathData;
}

export interface HeadMovementSignal extends WiaAacSignalBase {
  type: 'head_movement';
  data: HeadMovementData;
}

export interface CustomSignal extends WiaAacSignalBase {
  type: 'custom';
  data: CustomData;
}

// Signal Union Type
export type WiaAacSignal =
  | EyeTrackerSignal
  | SwitchSignal
  | MuscleSensorSignal
  | BrainInterfaceSignal
  | BreathSignal
  | HeadMovementSignal
  | CustomSignal;
