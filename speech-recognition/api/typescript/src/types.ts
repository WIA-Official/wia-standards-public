/**
 * WIA-AI-022 Speech Recognition TypeScript SDK
 * Type Definitions
 *
 * @packageDocumentation
 */

/**
 * Supported sample rates (Hz)
 */
export type SampleRate = 8000 | 16000 | 44100 | 48000;

/**
 * Supported bit depths
 */
export type BitDepth = 16 | 24 | 32;

/**
 * Audio format types
 */
export type AudioFormat = 'wav' | 'flac' | 'ogg' | 'mp3';

/**
 * Supported language codes (ISO 639-1 with region)
 */
export type LanguageCode =
  | 'en-US' | 'en-GB'
  | 'es-ES' | 'fr-FR' | 'de-DE'
  | 'zh-CN' | 'ja-JP' | 'ko-KR'
  | 'ar-SA' | 'hi-IN' | 'pt-BR'
  | 'ru-RU' | 'it-IT' | 'nl-NL';

/**
 * Audio configuration
 */
export interface AudioConfig {
  /** Sample rate in Hz */
  sampleRate: SampleRate;

  /** Bit depth */
  bitDepth: BitDepth;

  /** Number of channels (1=mono, 2=stereo) */
  channels: 1 | 2;

  /** Audio format */
  format: AudioFormat;
}

/**
 * Voice Activity Detection configuration
 */
export interface VADConfig {
  /** VAD algorithm */
  algorithm: 'energy' | 'webrtc' | 'ml';

  /** Sensitivity (0.0=lenient, 1.0=strict) */
  sensitivity: number;

  /** Frame length in milliseconds */
  frameLengthMs: 10 | 20 | 30;
}

/**
 * VAD result for a single frame
 */
export interface VADResult {
  /** Whether frame contains speech */
  isSpeech: boolean;

  /** Confidence score (0.0-1.0) */
  confidence: number;

  /** Start time in seconds */
  startTime: number;

  /** End time in seconds */
  endTime: number;
}

/**
 * Transcription options
 */
export interface TranscriptionOptions {
  /** Target language (auto-detect if not specified) */
  language?: LanguageCode;

  /** Enable speaker diarization */
  diarization?: boolean;

  /** Number of speakers (or 'auto') */
  numSpeakers?: number | 'auto';

  /** Enable automatic punctuation */
  punctuation?: boolean;

  /** Enable automatic capitalization */
  capitalization?: boolean;

  /** Number of alternative hypotheses */
  alternatives?: number;

  /** Custom vocabulary boost */
  vocabulary?: string[];

  /** Profanity filtering */
  profanityFilter?: boolean;
}

/**
 * Word-level information
 */
export interface WordInfo {
  /** The word text */
  word: string;

  /** Start time in seconds */
  startTime: number;

  /** End time in seconds */
  endTime: number;

  /** Confidence score (0.0-1.0) */
  confidence: number;

  /** Speaker ID (if diarization enabled) */
  speakerId?: string;
}

/**
 * Speaker segment in diarization
 */
export interface SpeakerSegment {
  /** Speaker identifier */
  speakerId: string;

  /** Segment start time */
  startTime: number;

  /** Segment end time */
  endTime: number;

  /** Transcribed text for segment */
  text: string;

  /** Segment confidence */
  confidence: number;
}

/**
 * Speaker information
 */
export interface Speaker {
  /** Speaker identifier */
  id: string;

  /** Speaker name (if identified) */
  name?: string;

  /** Speaker embedding vector */
  embedding?: number[];

  /** Total speaking time */
  duration?: number;
}

/**
 * Transcription result
 */
export interface TranscriptionResult {
  /** Transcribed text */
  text: string;

  /** Overall confidence score */
  confidence: number;

  /** Word-level details */
  words?: WordInfo[];

  /** Audio duration in seconds */
  duration: number;

  /** Detected language */
  language?: LanguageCode;

  /** Alternative hypotheses */
  alternatives?: Array<{
    text: string;
    confidence: number;
  }>;

  /** Speaker diarization results */
  diarization?: {
    speakers: Speaker[];
    segments: SpeakerSegment[];
  };

  /** Metadata */
  metadata?: {
    modelVersion: string;
    timestamp: string;
    processingTime: number;
  };
}

/**
 * Streaming options
 */
export interface StreamOptions extends TranscriptionOptions {
  /** Interim results */
  interimResults?: boolean;

  /** Maximum silence duration before finalizing (ms) */
  maxSilenceMs?: number;

  /** Minimum utterance duration (ms) */
  minUtteranceMs?: number;
}

/**
 * Streaming result (partial or final)
 */
export interface StreamResult {
  /** Transcribed text */
  text: string;

  /** Whether this is a final result */
  isFinal: boolean;

  /** Confidence score */
  confidence: number;

  /** Word-level details (for final results) */
  words?: WordInfo[];

  /** Result stability (0.0-1.0, for interim results) */
  stability?: number;
}

/**
 * Transcription stream interface
 */
export interface TranscriptionStream {
  /** Write audio data to stream */
  write(audioChunk: ArrayBuffer): Promise<void>;

  /** Signal end of audio stream */
  end(): Promise<TranscriptionResult>;

  /** Close the stream */
  close(): void;

  /** Event: interim result available */
  on(event: 'data', callback: (result: StreamResult) => void): void;

  /** Event: final result available */
  on(event: 'final', callback: (result: TranscriptionResult) => void): void;

  /** Event: error occurred */
  on(event: 'error', callback: (error: Error) => void): void;

  /** Event: stream closed */
  on(event: 'close', callback: () => void): void;
}

/**
 * Named entity types
 */
export type EntityType =
  | 'PERSON'
  | 'ORGANIZATION'
  | 'LOCATION'
  | 'DATE'
  | 'TIME'
  | 'MONEY'
  | 'PHONE'
  | 'EMAIL'
  | 'URL';

/**
 * Named entity
 */
export interface Entity {
  /** Entity type */
  type: EntityType;

  /** Entity text */
  text: string;

  /** Start position in text */
  start: number;

  /** End position in text */
  end: number;

  /** Confidence score */
  confidence: number;

  /** Normalized value */
  normalized?: any;
}

/**
 * Intent classification result
 */
export interface Intent {
  /** Intent name */
  name: string;

  /** Intent domain */
  domain: string;

  /** Confidence score */
  confidence: number;

  /** Intent slots */
  slots: Record<string, any>;
}

/**
 * Sentiment analysis result
 */
export interface Sentiment {
  /** Sentiment label */
  label: 'positive' | 'negative' | 'neutral' | 'mixed';

  /** Sentiment score (-1.0 to +1.0) */
  score: number;

  /** Confidence score */
  confidence: number;
}

/**
 * ASR engine configuration
 */
export interface ASRConfig {
  /** API endpoint URL */
  endpoint?: string;

  /** API key for authentication */
  apiKey?: string;

  /** Default language */
  language?: LanguageCode;

  /** Audio configuration */
  audio?: Partial<AudioConfig>;

  /** VAD configuration */
  vad?: Partial<VADConfig>;

  /** Enable logging */
  logging?: boolean;

  /** Custom headers */
  headers?: Record<string, string>;
}

/**
 * Main ASR Engine interface
 */
export interface ASREngine {
  /**
   * Transcribe audio file
   */
  transcribeFile(
    filePath: string,
    options?: TranscriptionOptions
  ): Promise<TranscriptionResult>;

  /**
   * Transcribe audio buffer
   */
  transcribe(
    audioBuffer: ArrayBuffer,
    options?: TranscriptionOptions
  ): Promise<TranscriptionResult>;

  /**
   * Create streaming transcription session
   */
  createStream(options?: StreamOptions): TranscriptionStream;

  /**
   * Detect language in audio
   */
  detectLanguage(audioBuffer: ArrayBuffer): Promise<{
    language: LanguageCode;
    confidence: number;
  }>;

  /**
   * Extract named entities from text
   */
  extractEntities(text: string): Promise<Entity[]>;

  /**
   * Recognize intent from text
   */
  recognizeIntent(text: string): Promise<Intent>;

  /**
   * Analyze sentiment
   */
  analyzeSentiment(text: string): Promise<Sentiment>;
}

/**
 * Error codes
 */
export enum ASRErrorCode {
  /** Invalid audio format */
  INVALID_AUDIO_FORMAT = 'INVALID_AUDIO_FORMAT',

  /** Unsupported language */
  UNSUPPORTED_LANGUAGE = 'UNSUPPORTED_LANGUAGE',

  /** Audio too short */
  AUDIO_TOO_SHORT = 'AUDIO_TOO_SHORT',

  /** Audio too long */
  AUDIO_TOO_LONG = 'AUDIO_TOO_LONG',

  /** Authentication failed */
  AUTH_FAILED = 'AUTH_FAILED',

  /** Rate limit exceeded */
  RATE_LIMIT_EXCEEDED = 'RATE_LIMIT_EXCEEDED',

  /** Network error */
  NETWORK_ERROR = 'NETWORK_ERROR',

  /** Internal server error */
  INTERNAL_ERROR = 'INTERNAL_ERROR',
}

/**
 * ASR Error class
 */
export class ASRError extends Error {
  constructor(
    public code: ASRErrorCode,
    message: string,
    public details?: any
  ) {
    super(message);
    this.name = 'ASRError';
  }
}
