/**
 * WIA AAC Output Module Types
 * Phase 4: WIA Ecosystem Integration
 */

/**
 * Output adapter types
 */
export type OutputType = 'tts' | 'sign_language' | 'braille' | 'custom';

/**
 * Output adapter state
 */
export type OutputState = 'idle' | 'outputting' | 'paused' | 'error';

/**
 * Output event types
 */
export type OutputEventType = 'start' | 'end' | 'pause' | 'resume' | 'progress' | 'error';

/**
 * Output options
 */
export interface OutputOptions {
  /** Language code (ko, en, ja, ...) */
  language?: string;

  /** Voice ID (for TTS) */
  voice?: string;

  /** Output speed (0.5 ~ 2.0) */
  speed?: number;

  /** Volume (0.0 ~ 1.0) */
  volume?: number;

  /** Additional options */
  [key: string]: unknown;
}

/**
 * Output event
 */
export interface OutputEvent {
  /** Event type */
  type: OutputEventType;

  /** Adapter type */
  adapter: OutputType;

  /** Timestamp */
  timestamp: number;

  /** Additional data */
  data?: unknown;
}

/**
 * Output event handler
 */
export type OutputEventHandler = (event: OutputEvent) => void;

/**
 * Voice information (for TTS)
 */
export interface Voice {
  /** Voice ID */
  id: string;

  /** Voice name */
  name: string;

  /** Language code */
  language: string;

  /** Gender */
  gender?: 'male' | 'female' | 'neutral';

  /** Local voice */
  local?: boolean;
}

/**
 * ISP code (for sign language)
 */
export interface ISPCode {
  /** ISP code (e.g., "HS01-LC07-MV10-OR02-NM15") */
  code: string;

  /** Meaning (optional) */
  meaning?: string;

  /** Duration in milliseconds */
  duration?: number;

  /** Components */
  components?: {
    handshape: string;    // HS##
    location: string;     // LC##
    movement: string;     // MV##
    orientation: string;  // OR##
    nonManual: string;    // NM##
  };
}

/**
 * Braille output
 */
export interface BrailleOutput {
  /** Original text */
  text: string;

  /** IPA representation */
  ipa: string;

  /** Braille unicode string */
  braille: string;

  /** Braille unicode array */
  unicode: string[];

  /** Braille dot patterns (8-dot) */
  dots: number[];
}

/**
 * Braille display information
 */
export interface BrailleDisplay {
  /** Display ID */
  id: string;

  /** Display name */
  name: string;

  /** Number of cells */
  cells: number;

  /** Connection status */
  connected: boolean;
}

/**
 * Output error codes
 */
export enum OutputErrorCode {
  // Initialization errors (1xxx)
  INIT_FAILED = 1001,
  NOT_AVAILABLE = 1002,
  ALREADY_INITIALIZED = 1003,

  // Output errors (2xxx)
  OUTPUT_FAILED = 2001,
  OUTPUT_CANCELLED = 2002,
  OUTPUT_TIMEOUT = 2003,

  // Adapter errors (3xxx)
  ADAPTER_NOT_FOUND = 3001,
  ADAPTER_NOT_READY = 3002,
  ADAPTER_BUSY = 3003,

  // Conversion errors (4xxx)
  CONVERSION_FAILED = 4001,
  INVALID_INPUT = 4002,
  MAPPING_NOT_FOUND = 4003,

  // Device errors (5xxx)
  DEVICE_NOT_CONNECTED = 5001,
  DEVICE_ERROR = 5002
}

/**
 * Output error
 */
export class OutputError extends Error {
  constructor(
    public code: OutputErrorCode,
    message: string,
    public recoverable: boolean = true,
    public details?: unknown
  ) {
    super(message);
    this.name = 'OutputError';
  }
}
