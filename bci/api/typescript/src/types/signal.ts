/**
 * WIA BCI Signal Types (Phase 1 Compatible)
 * @module wia-bci/types/signal
 */

/**
 * Frequency Band Definition
 */
export type FrequencyBand =
  | 'delta'    // 0.5-4 Hz
  | 'theta'    // 4-8 Hz
  | 'alpha'    // 8-13 Hz
  | 'beta'     // 13-30 Hz
  | 'gamma'    // 30-100 Hz
  | [number, number];  // Custom band

/**
 * Band Power Values
 */
export interface BandPowers {
  /** Delta band power (0.5-4 Hz) */
  delta: number;
  /** Theta band power (4-8 Hz) */
  theta: number;
  /** Alpha band power (8-13 Hz) */
  alpha: number;
  /** Beta band power (13-30 Hz) */
  beta: number;
  /** Gamma band power (30-100 Hz) */
  gamma: number;
}

/**
 * Power Spectrum
 */
export interface PowerSpectrum {
  /** Frequency bins */
  frequencies: Float32Array;
  /** Power values */
  powers: Float32Array;
  /** Frequency resolution */
  resolution: number;
}

/**
 * Complex Number for FFT
 */
export interface ComplexNumber {
  real: number;
  imag: number;
}

/**
 * Complex Array for FFT results
 */
export type ComplexArray = ComplexNumber[];

/**
 * Hjorth Parameters
 */
export interface HjorthParams {
  /** Activity - signal variance */
  activity: number;
  /** Mobility - mean frequency */
  mobility: number;
  /** Complexity - bandwidth */
  complexity: number;
}

/**
 * Feature Vector - generic feature container
 */
export type FeatureVector = Record<string, number>;

/**
 * Artifact Types
 */
export type ArtifactType =
  | 'eye_blink'
  | 'eye_movement'
  | 'muscle'
  | 'movement'
  | 'electrode_pop'
  | 'line_noise'
  | 'saturation'
  | 'flat_line';

/**
 * Artifact Detection Result
 */
export interface ArtifactSegment {
  /** Start sample index */
  startSample: number;
  /** End sample index */
  endSample: number;
  /** Artifact type */
  type: ArtifactType;
  /** Affected channels (empty = all) */
  channels: number[];
  /** Confidence score */
  confidence: number;
}

/**
 * Recording Information (Phase 1 compatible)
 */
export interface RecordingInfo {
  /** Recording ID */
  recordingId: string;
  /** Start time (ISO 8601) */
  startTime: string;
  /** End time (ISO 8601) */
  endTime?: string;
  /** Duration in seconds */
  durationSeconds?: number;
  /** Timezone */
  timezone?: string;
}

/**
 * Session Information
 */
export interface SessionInfo {
  /** Session ID */
  sessionId: string;
  /** Session type */
  sessionType: 'research' | 'clinical' | 'training' | 'evaluation' | 'other';
  /** BCI paradigm */
  paradigm?: string;
  /** Description */
  description?: string;
}

/**
 * WIA BCI Recording (Phase 1 format)
 */
export interface WiaBciRecording {
  /** WIA specification version */
  wiaVersion: string;
  /** Format version */
  formatVersion: string;
  /** Recording ID */
  recordingId: string;
  /** Recording info */
  recordingInfo: RecordingInfo;
  /** Session info */
  session?: SessionInfo;
  /** Data files */
  dataFiles: Record<string, { file: string; metadata?: string; format?: string }>;
  /** References to other files */
  references?: Record<string, string>;
  /** Notes */
  notes?: string;
}

/**
 * Frequency Band Ranges (Hz)
 */
export const FREQUENCY_BANDS: Record<Exclude<FrequencyBand, [number, number]>, [number, number]> = {
  delta: [0.5, 4],
  theta: [4, 8],
  alpha: [8, 13],
  beta: [13, 30],
  gamma: [30, 100],
};
