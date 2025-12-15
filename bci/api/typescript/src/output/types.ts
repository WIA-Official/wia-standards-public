/**
 * WIA BCI Output Types
 *
 * Phase 4: Ecosystem Integration
 */

import { BandPowers } from '../types/signal';

// Output types
export type OutputType =
  | 'tts'
  | 'sign_language'
  | 'braille'
  | 'neurofeedback'
  | 'cursor'
  | 'custom';

// Output content
export interface OutputContent {
  type: 'text' | 'command' | 'classification' | 'signal';
  text?: string;
  command?: CommandContent;
  classification?: ClassificationContent;
  signal?: SignalContent;
  metadata?: OutputMetadata;
}

export interface CommandContent {
  action: string;
  params?: Record<string, unknown>;
}

export interface ClassificationContent {
  classId: number;
  className: string;
  confidence: number;
}

export interface SignalContent {
  bandPowers: BandPowers;
  channels: ChannelData[];
}

export interface ChannelData {
  channel: string;
  value: number;
  quality?: number;
}

export interface OutputMetadata {
  timestamp: number;
  source?: string;
  priority?: 'low' | 'normal' | 'high';
}

// Output options
export interface OutputOptions {
  language?: string;
  autoStart?: boolean;
  [key: string]: unknown;
}

// TTS types
export interface Voice {
  id: string;
  name: string;
  language: string;
  gender?: 'male' | 'female' | 'neutral';
  localService?: boolean;
  default?: boolean;
}

export interface TTSOptions {
  voice?: string;
  rate?: number;
  pitch?: number;
  volume?: number;
  language?: string;
}

// Sign language types
export interface ISPCode {
  code: string;
  meaning?: string;
  duration?: number;
  metadata?: {
    handshape?: string;
    location?: string;
    movement?: string;
    orientation?: string;
    nonManual?: string;
  };
}

export interface Avatar {
  id: string;
  name: string;
  style: 'realistic' | 'cartoon' | 'simple';
  preview?: string;
}

// Braille types
export interface BrailleOutput {
  original: string;
  ipa: string;
  braille: string;
  unicode: string[];
  cells: number;
  grade: 1 | 2;
}

export interface BrailleDisplay {
  id: string;
  name: string;
  manufacturer?: string;
  cells: number;
  rows: number;
  connected: boolean;
  battery?: number;
}

// Neurofeedback types
export type VisualizationMode =
  | 'band_powers'
  | 'topography'
  | 'time_series'
  | 'spectrogram'
  | 'classification'
  | 'cursor'
  | 'combined';

export interface CursorPosition {
  x: number;
  y: number;
  click?: boolean;
}

export interface NeurofeedbackTheme {
  background: string;
  foreground: string;
  positive: string;
  negative: string;
  neutral: string;
}

// Events
export type OutputEvent = 'start' | 'end' | 'error' | 'ready' | 'busy';

export interface OutputEventData {
  type: OutputEvent;
  adapter: OutputType;
  timestamp: number;
  content?: OutputContent;
  error?: Error;
}

export type OutputEventHandler = (event: OutputEventData) => void;

// Manager types
export interface OutputPreferences {
  primaryOutput: OutputType;
  enabledOutputs: OutputType[];
  language: string;
  tts?: TTSOptions;
}

export interface ManagerOptions {
  autoInitialize?: boolean;
  defaultOutputs?: OutputType[];
  preferences?: OutputPreferences;
}

// Error codes
export enum OutputErrorCode {
  // Adapter errors (1xxx)
  ADAPTER_NOT_FOUND = 1001,
  ADAPTER_NOT_READY = 1002,
  ADAPTER_BUSY = 1003,
  ADAPTER_INIT_FAILED = 1004,

  // TTS errors (2xxx)
  TTS_NOT_SUPPORTED = 2001,
  TTS_VOICE_NOT_FOUND = 2002,
  TTS_SYNTHESIS_FAILED = 2003,

  // Sign language errors (3xxx)
  SIGN_CONVERSION_FAILED = 3001,
  SIGN_AVATAR_NOT_FOUND = 3002,
  SIGN_ANIMATION_FAILED = 3003,

  // Braille errors (4xxx)
  BRAILLE_DISPLAY_NOT_FOUND = 4001,
  BRAILLE_CONVERSION_FAILED = 4002,
  BRAILLE_SEND_FAILED = 4003,

  // Neurofeedback errors (5xxx)
  NEURO_CANVAS_NOT_SET = 5001,
  NEURO_RENDER_FAILED = 5002,
}

export class OutputError extends Error {
  constructor(
    public readonly code: OutputErrorCode,
    message: string,
    public readonly adapter?: OutputType,
    public readonly cause?: Error
  ) {
    super(message);
    this.name = 'OutputError';
  }
}
