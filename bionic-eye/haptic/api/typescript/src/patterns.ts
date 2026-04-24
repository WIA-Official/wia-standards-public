/**
 * WIA Haptic Standard - Pattern Definitions
 * @version 1.0.0
 */

import {
  WaveformType,
  WaveformConfig,
  Envelope,
  EnvelopePreset,
  FrequencyBand,
  IntensityLevel,
  IntensityModulation,
  HapticCategory,
  BodyLocation,
} from './types';

// ============================================================================
// Haptic Primitive
// ============================================================================

export interface HapticPrimitive {
  id: string;
  name: string;
  waveform: WaveformConfig | WaveformType;
  envelope: Envelope | EnvelopePreset;
  frequency: number;           // 1-300 Hz
  intensity: number;           // 0.0-1.0
  duration: number;            // milliseconds
  metadata?: {
    category?: HapticCategory;
    description?: string;
    learnability?: 'easy' | 'moderate' | 'complex';
    tags?: string[];
  };
}

// ============================================================================
// Haptic Sequence
// ============================================================================

export interface SequenceStep {
  primitiveId?: string;
  primitive?: HapticPrimitive;
  delayBefore?: number;        // ms
  repeatCount?: number;
  repeatDelay?: number;        // ms
  intensityScale?: number;     // 0.0-2.0
}

export interface HapticSequence {
  id: string;
  name: string;
  steps: SequenceStep[];
  loop?: boolean;
  loopCount?: number;
  metadata?: {
    category?: HapticCategory;
    description?: string;
    totalDuration?: number;
    tags?: string[];
  };
}

// ============================================================================
// Spatial Pattern
// ============================================================================

export interface SpatialActuation {
  location?: BodyLocation;
  locations?: BodyLocation[];
  pattern: string | HapticPrimitive | HapticSequence;
  startTime?: number;          // ms
  intensityScale?: number;     // 0.0-2.0
}

export interface SpatialPattern {
  id: string;
  name: string;
  actuations: SpatialActuation[];
  metadata?: {
    category?: HapticCategory;
    description?: string;
    direction?: 'forward' | 'backward' | 'left' | 'right' | 'up' | 'down';
  };
}

// ============================================================================
// Unified Pattern Type
// ============================================================================

export type HapticPattern = HapticPrimitive | HapticSequence | SpatialPattern;

// ============================================================================
// Envelope Presets
// ============================================================================

export const ENVELOPE_PRESETS: Record<EnvelopePreset, Envelope> = {
  sharp:  { attack: 5,   decay: 20,  sustain: 0.0, release: 30  },
  punch:  { attack: 10,  decay: 50,  sustain: 0.3, release: 50  },
  smooth: { attack: 100, decay: 100, sustain: 0.7, release: 150 },
  pulse:  { attack: 5,   decay: 0,   sustain: 1.0, release: 5   },
  swell:  { attack: 200, decay: 50,  sustain: 0.8, release: 200 },
  fade:   { attack: 50,  decay: 200, sustain: 0.5, release: 300 },
};

// ============================================================================
// Frequency Band Ranges
// ============================================================================

export const FREQUENCY_BANDS: Record<FrequencyBand, [number, number]> = {
  very_low:  [1, 30],
  low:       [30, 80],
  mid:       [80, 150],
  high:      [150, 250],
  very_high: [250, 300],
};

// ============================================================================
// Intensity Presets
// ============================================================================

export const INTENSITY_LEVELS: Record<IntensityLevel, number> = {
  subtle:  0.2,
  light:   0.4,
  medium:  0.6,
  strong:  0.8,
  maximum: 1.0,
};

// ============================================================================
// Standard Primitives Library
// ============================================================================

export const STANDARD_PRIMITIVES: HapticPrimitive[] = [
  {
    id: 'tick',
    name: 'Tick',
    waveform: { type: 'square', dutyCycle: 0.5 },
    envelope: 'sharp',
    frequency: 150,
    intensity: 0.6,
    duration: 20,
    metadata: {
      category: 'confirmation',
      description: 'Quick acknowledgment tap',
      learnability: 'easy',
    },
  },
  {
    id: 'tick_double',
    name: 'Double Tick',
    waveform: { type: 'square', dutyCycle: 0.5 },
    envelope: 'sharp',
    frequency: 150,
    intensity: 0.6,
    duration: 20,
    metadata: {
      category: 'confirmation',
      description: 'Success confirmation',
      learnability: 'easy',
    },
  },
  {
    id: 'buzz',
    name: 'Buzz',
    waveform: { type: 'square', dutyCycle: 0.3 },
    envelope: 'punch',
    frequency: 180,
    intensity: 0.7,
    duration: 100,
    metadata: {
      category: 'notification',
      description: 'Attention-getting buzz',
      learnability: 'easy',
    },
  },
  {
    id: 'warning_pulse',
    name: 'Warning Pulse',
    waveform: { type: 'sine' },
    envelope: { attack: 50, decay: 100, sustain: 0.5, release: 100 },
    frequency: 200,
    intensity: 0.8,
    duration: 300,
    metadata: {
      category: 'notification',
      description: 'Alert warning pattern',
      learnability: 'easy',
    },
  },
  {
    id: 'soft_wave',
    name: 'Soft Wave',
    waveform: { type: 'sine' },
    envelope: 'smooth',
    frequency: 60,
    intensity: 0.4,
    duration: 400,
    metadata: {
      category: 'navigation',
      description: 'Gentle directional cue',
      learnability: 'easy',
    },
  },
  {
    id: 'obstacle_texture',
    name: 'Obstacle Texture',
    waveform: { type: 'noise', noiseType: 'pink' },
    envelope: { attack: 20, decay: 50, sustain: 0.7, release: 50 },
    frequency: 100,
    intensity: 0.7,
    duration: 200,
    metadata: {
      category: 'navigation',
      description: 'Indicates nearby obstacle',
      learnability: 'moderate',
    },
  },
  {
    id: 'heartbeat',
    name: 'Heartbeat',
    waveform: { type: 'sine' },
    envelope: 'punch',
    frequency: 80,
    intensity: 0.6,
    duration: 150,
    metadata: {
      category: 'social',
      description: 'Presence indicator',
      learnability: 'easy',
    },
  },
  {
    id: 'ramp_up',
    name: 'Ramp Up',
    waveform: { type: 'triangle' },
    envelope: 'swell',
    frequency: 120,
    intensity: 0.7,
    duration: 500,
    metadata: {
      category: 'temporal',
      description: 'Increasing intensity',
      learnability: 'moderate',
    },
  },
  {
    id: 'ramp_down',
    name: 'Ramp Down',
    waveform: { type: 'triangle' },
    envelope: 'fade',
    frequency: 120,
    intensity: 0.7,
    duration: 500,
    metadata: {
      category: 'temporal',
      description: 'Decreasing intensity',
      learnability: 'moderate',
    },
  },
  {
    id: 'click',
    name: 'Click',
    waveform: { type: 'square' },
    envelope: { attack: 2, decay: 5, sustain: 0, release: 3 },
    frequency: 200,
    intensity: 0.5,
    duration: 10,
    metadata: {
      category: 'content',
      description: 'UI selection click',
      learnability: 'easy',
    },
  },
];

// ============================================================================
// Standard Sequences Library
// ============================================================================

export const STANDARD_SEQUENCES: HapticSequence[] = [
  {
    id: 'success',
    name: 'Success',
    steps: [
      { primitiveId: 'tick', delayBefore: 0 },
      { primitiveId: 'tick', delayBefore: 100 },
    ],
    metadata: {
      category: 'confirmation',
      description: 'Action succeeded',
      totalDuration: 120,
    },
  },
  {
    id: 'failure',
    name: 'Failure',
    steps: [
      { primitiveId: 'buzz', delayBefore: 0 },
      { primitiveId: 'buzz', delayBefore: 150, intensityScale: 0.7 },
    ],
    metadata: {
      category: 'confirmation',
      description: 'Action failed',
      totalDuration: 350,
    },
  },
  {
    id: 'urgent_alert',
    name: 'Urgent Alert',
    steps: [
      { primitiveId: 'warning_pulse', delayBefore: 0, repeatCount: 3, repeatDelay: 150 },
    ],
    metadata: {
      category: 'notification',
      description: 'Urgent attention required',
      totalDuration: 1200,
    },
  },
  {
    id: 'navigation_tick',
    name: 'Navigation Tick',
    steps: [
      { primitiveId: 'soft_wave', delayBefore: 0 },
    ],
    metadata: {
      category: 'navigation',
      description: 'Directional guidance pulse',
      totalDuration: 400,
    },
  },
  {
    id: 'countdown_3',
    name: 'Countdown 3-2-1',
    steps: [
      { primitiveId: 'tick', delayBefore: 0, intensityScale: 0.6 },
      { primitiveId: 'tick', delayBefore: 1000, intensityScale: 0.8 },
      { primitiveId: 'tick', delayBefore: 1000, intensityScale: 1.0 },
    ],
    metadata: {
      category: 'temporal',
      description: 'Three-beat countdown',
      totalDuration: 2020,
    },
  },
  {
    id: 'heartbeat_presence',
    name: 'Heartbeat Presence',
    steps: [
      { primitiveId: 'heartbeat', delayBefore: 0 },
      { primitiveId: 'heartbeat', delayBefore: 200, intensityScale: 0.7 },
    ],
    loop: true,
    loopCount: 3,
    metadata: {
      category: 'social',
      description: 'Someone nearby',
      totalDuration: 1050,
    },
  },
];

// ============================================================================
// Pattern Utilities
// ============================================================================

/**
 * Resolve envelope preset to full envelope object
 */
export function resolveEnvelope(envelope: Envelope | EnvelopePreset): Envelope {
  if (typeof envelope === 'string') {
    return ENVELOPE_PRESETS[envelope];
  }
  return envelope;
}

/**
 * Resolve waveform to full config object
 */
export function resolveWaveform(waveform: WaveformConfig | WaveformType): WaveformConfig {
  if (typeof waveform === 'string') {
    return { type: waveform };
  }
  return waveform;
}

/**
 * Calculate total duration of a sequence
 */
export function calculateSequenceDuration(sequence: HapticSequence): number {
  let duration = 0;
  for (const step of sequence.steps) {
    duration += step.delayBefore ?? 0;
    const repeatCount = step.repeatCount ?? 1;
    const repeatDelay = step.repeatDelay ?? 0;
    // Simplified: assumes primitive duration is known
    duration += repeatCount * repeatDelay;
  }
  return duration;
}

/**
 * Find primitive by ID
 */
export function findPrimitive(id: string): HapticPrimitive | undefined {
  return STANDARD_PRIMITIVES.find(p => p.id === id);
}

/**
 * Find sequence by ID
 */
export function findSequence(id: string): HapticSequence | undefined {
  return STANDARD_SEQUENCES.find(s => s.id === id);
}

/**
 * Get frequency from band
 */
export function getFrequencyFromBand(band: FrequencyBand): number {
  const [min, max] = FREQUENCY_BANDS[band];
  return (min + max) / 2;
}

/**
 * Get intensity from level
 */
export function getIntensityFromLevel(level: IntensityLevel): number {
  return INTENSITY_LEVELS[level];
}
