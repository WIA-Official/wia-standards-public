/**
 * WIA Haptic Standard - Core Type Definitions
 * @version 1.0.0
 */

// ============================================================================
// Body Locations
// ============================================================================

export type BodyRegion =
  | 'head'
  | 'neck'
  | 'shoulder'
  | 'arm'
  | 'wrist'
  | 'hand'
  | 'finger'
  | 'torso'
  | 'waist'
  | 'leg'
  | 'foot';

export type Laterality = 'left' | 'right' | 'center' | 'bilateral';

export type BodyLocation =
  // Head
  | 'forehead_left' | 'forehead_center' | 'forehead_right'
  | 'temple_left' | 'temple_right'
  // Neck
  | 'neck_left' | 'neck_center' | 'neck_right' | 'neck_back'
  // Shoulders
  | 'shoulder_left' | 'shoulder_right'
  // Arms
  | 'upper_arm_left' | 'upper_arm_right'
  | 'elbow_left' | 'elbow_right'
  | 'forearm_left' | 'forearm_right'
  // Wrists
  | 'wrist_left_dorsal' | 'wrist_left_volar'
  | 'wrist_right_dorsal' | 'wrist_right_volar'
  // Hands
  | 'palm_left' | 'palm_right'
  | 'back_hand_left' | 'back_hand_right'
  // Fingers
  | 'thumb_left' | 'index_left' | 'middle_left' | 'ring_left' | 'pinky_left'
  | 'thumb_right' | 'index_right' | 'middle_right' | 'ring_right' | 'pinky_right'
  // Torso Front
  | 'chest_left' | 'chest_center' | 'chest_right'
  | 'abdomen_left' | 'abdomen_center' | 'abdomen_right'
  // Torso Back
  | 'back_upper_left' | 'back_upper_center' | 'back_upper_right'
  | 'back_lower_left' | 'back_lower_center' | 'back_lower_right'
  // Waist
  | 'waist_left' | 'waist_front' | 'waist_right' | 'waist_back'
  // Legs
  | 'thigh_left' | 'thigh_right'
  | 'knee_left' | 'knee_right'
  | 'calf_left' | 'calf_right'
  | 'ankle_left' | 'ankle_right'
  // Feet
  | 'foot_left' | 'foot_right';

// ============================================================================
// Waveforms and Envelopes
// ============================================================================

export type WaveformType = 'sine' | 'square' | 'triangle' | 'sawtooth' | 'noise';
export type NoiseType = 'white' | 'pink' | 'brown';
export type EnvelopePreset = 'sharp' | 'punch' | 'smooth' | 'pulse' | 'swell' | 'fade';
export type FrequencyBand = 'very_low' | 'low' | 'mid' | 'high' | 'very_high';
export type IntensityLevel = 'subtle' | 'light' | 'medium' | 'strong' | 'maximum';

export interface WaveformConfig {
  type: WaveformType;
  dutyCycle?: number;      // 0.0-1.0, for square waves
  smoothing?: number;      // 0.0-1.0
  noiseType?: NoiseType;   // for noise waveform
}

export interface Envelope {
  attack: number;   // 0-500ms
  decay: number;    // 0-500ms
  sustain: number;  // 0.0-1.0
  release: number;  // 0-500ms
}

export interface IntensityModulation {
  type: 'none' | 'pulse' | 'wave' | 'random';
  rate?: number;   // Hz
  depth?: number;  // 0.0-1.0
}

// ============================================================================
// Haptic Categories
// ============================================================================

export type HapticCategory =
  | 'navigation'
  | 'notification'
  | 'confirmation'
  | 'spatial'
  | 'temporal'
  | 'social'
  | 'system'
  | 'content';

// ============================================================================
// Actuator Types
// ============================================================================

export type ActuatorType =
  | 'erm'         // Eccentric Rotating Mass
  | 'lra'         // Linear Resonant Actuator
  | 'piezo'       // Piezoelectric
  | 'voice_coil'; // Voice Coil

// ============================================================================
// Device Types
// ============================================================================

export type DeviceType =
  | 'smartwatch'
  | 'wristband'
  | 'bracelet'
  | 'haptic_glove'
  | 'ring'
  | 'handheld'
  | 'armband'
  | 'headband'
  | 'smart_glasses'
  | 'headset'
  | 'vest'
  | 'belt'
  | 'shirt'
  | 'full_suit'
  | 'insole'
  | 'anklet'
  | 'smartphone'
  | 'tablet'
  | 'cane'
  | 'gamepad';

// ============================================================================
// Connection States
// ============================================================================

export type ConnectionState =
  | 'disconnected'
  | 'connecting'
  | 'connected'
  | 'disconnecting'
  | 'error';

// ============================================================================
// Events
// ============================================================================

export interface HapticDeviceEvent {
  type: 'connected' | 'disconnected' | 'error' | 'battery' | 'pattern_complete';
  timestamp: number;
  data?: unknown;
}

export type HapticEventListener = (event: HapticDeviceEvent) => void;
