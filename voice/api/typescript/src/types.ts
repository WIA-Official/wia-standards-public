/**
 * WIA Voice Standard - TypeScript Type Definitions
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 * @description Voice recognition and synthesis for accessibility and voice assistants
 */

// ============================================================================
// Core Configuration
// ============================================================================

export interface WIAVoiceConfig {
  /** API key for authentication */
  apiKey?: string;

  /** API endpoint URL */
  endpoint?: string;

  /** API version */
  version?: string;

  /** Default language/locale */
  defaultLanguage?: string;

  /** Enable debug logging */
  debug?: boolean;

  /** Timeout for operations in milliseconds */
  timeout?: number;
}

// ============================================================================
// Audio Format Types
// ============================================================================

export type AudioFormat =
  | 'wav'
  | 'mp3'
  | 'ogg'
  | 'flac'
  | 'aac'
  | 'opus'
  | 'pcm';

export type AudioEncoding =
  | 'LINEAR16'
  | 'FLAC'
  | 'MULAW'
  | 'AMR'
  | 'AMR_WB'
  | 'OGG_OPUS'
  | 'MP3';

export interface AudioConfig {
  /** Audio format */
  format: AudioFormat;

  /** Audio encoding */
  encoding?: AudioEncoding;

  /** Sample rate in Hz (8000, 16000, 44100, 48000) */
  sampleRate: number;

  /** Number of audio channels (1 for mono, 2 for stereo) */
  channels: number;

  /** Bit depth (8, 16, 24, 32) */
  bitDepth?: number;

  /** Maximum audio duration in seconds */
  maxDuration?: number;
}

// ============================================================================
// Language and Locale Types
// ============================================================================

export interface LanguageConfig {
  /** Language code (ISO 639-1) */
  code: string;

  /** Language name */
  name: string;

  /** Locale/region (e.g., 'en-US', 'ko-KR') */
  locale: string;

  /** Supported features */
  features: LanguageFeatures;
}

export interface LanguageFeatures {
  /** Speech-to-text support */
  stt: boolean;

  /** Text-to-speech support */
  tts: boolean;

  /** Intent recognition support */
  intentRecognition: boolean;

  /** Wake word detection support */
  wakeWord: boolean;

  /** Punctuation support */
  punctuation: boolean;

  /** Number formatting */
  numberFormatting: boolean;
}

// ============================================================================
// Voice Profile Types
// ============================================================================

export interface VoiceProfile {
  /** Unique voice ID */
  voiceId: string;

  /** Voice name */
  name: string;

  /** Language code */
  language: string;

  /** Gender of voice */
  gender: VoiceGender;

  /** Age category */
  ageCategory?: VoiceAge;

  /** Voice characteristics */
  characteristics: VoiceCharacteristics;

  /** Neural/AI-generated voice */
  isNeural: boolean;

  /** Sample audio URL */
  sampleUrl?: string;
}

export type VoiceGender = 'male' | 'female' | 'neutral';

export type VoiceAge = 'child' | 'young-adult' | 'adult' | 'senior';

export interface VoiceCharacteristics {
  /** Voice pitch (0.5 to 2.0, 1.0 is normal) */
  pitch?: number;

  /** Speaking rate (0.25 to 4.0, 1.0 is normal) */
  rate?: number;

  /** Volume (0.0 to 1.0) */
  volume?: number;

  /** Emotion/style */
  style?: VoiceStyle;

  /** Voice quality descriptors */
  qualities?: string[];
}

export type VoiceStyle =
  | 'neutral'
  | 'cheerful'
  | 'empathetic'
  | 'calm'
  | 'excited'
  | 'sad'
  | 'angry'
  | 'whispering'
  | 'newscast'
  | 'customer-service';

// ============================================================================
// Speech Recognition Types (STT)
// ============================================================================

export interface SpeechRecognitionConfig {
  /** Audio configuration */
  audio: AudioConfig;

  /** Language configuration */
  language: string;

  /** Enable interim results */
  interimResults?: boolean;

  /** Maximum number of alternatives */
  maxAlternatives?: number;

  /** Enable profanity filter */
  profanityFilter?: boolean;

  /** Enable automatic punctuation */
  enableAutoPunctuation?: boolean;

  /** Speech contexts for better recognition */
  speechContexts?: SpeechContext[];

  /** Model selection */
  model?: RecognitionModel;

  /** Enable word-level timestamps */
  enableWordTimeOffsets?: boolean;
}

export interface SpeechContext {
  /** List of phrases to boost recognition */
  phrases: string[];

  /** Boost weight (-20 to 20) */
  boost?: number;
}

export type RecognitionModel =
  | 'default'
  | 'command'
  | 'phone-call'
  | 'video'
  | 'medical'
  | 'latest-long'
  | 'latest-short';

export interface RecognitionResult {
  /** Recognized transcript */
  transcript: string;

  /** Confidence score (0.0 to 1.0) */
  confidence: number;

  /** Is this a final result */
  isFinal: boolean;

  /** Alternative transcripts */
  alternatives?: RecognitionAlternative[];

  /** Word-level details */
  words?: WordInfo[];

  /** Language detected */
  languageCode?: string;
}

export interface RecognitionAlternative {
  /** Alternative transcript */
  transcript: string;

  /** Confidence score */
  confidence: number;
}

export interface WordInfo {
  /** The word */
  word: string;

  /** Start time offset in milliseconds */
  startTime: number;

  /** End time offset in milliseconds */
  endTime: number;

  /** Confidence score */
  confidence?: number;

  /** Speaker tag (for multi-speaker) */
  speakerTag?: number;
}

// ============================================================================
// Speech Synthesis Types (TTS)
// ============================================================================

export interface SpeechSynthesisConfig {
  /** Text to synthesize */
  text: string;

  /** Voice profile to use */
  voice: VoiceProfile;

  /** Audio output configuration */
  audioConfig: AudioConfig;

  /** SSML (Speech Synthesis Markup Language) enabled */
  ssml?: boolean;

  /** Speaking rate adjustment */
  speakingRate?: number;

  /** Pitch adjustment */
  pitch?: number;

  /** Volume level */
  volumeGainDb?: number;

  /** Effects profile */
  effectsProfile?: EffectsProfile[];
}

export type EffectsProfile =
  | 'headphone-class-device'
  | 'handset-class-device'
  | 'small-bluetooth-speaker'
  | 'medium-bluetooth-speaker'
  | 'large-home-entertainment'
  | 'large-automotive'
  | 'telephony';

export interface SynthesisResult {
  /** Generated audio data */
  audioContent: Buffer | ArrayBuffer;

  /** Audio format */
  format: AudioFormat;

  /** Duration in milliseconds */
  duration: number;

  /** Phoneme timings (if requested) */
  phonemes?: PhonemeInfo[];

  /** Word timings (if requested) */
  words?: WordTiming[];
}

export interface PhonemeInfo {
  /** Phoneme symbol */
  phoneme: string;

  /** Start time in milliseconds */
  startTime: number;

  /** End time in milliseconds */
  endTime: number;
}

export interface WordTiming {
  /** Word text */
  word: string;

  /** Start time in milliseconds */
  startTime: number;

  /** End time in milliseconds */
  endTime: number;
}

// ============================================================================
// Transcription Types
// ============================================================================

export interface TranscriptionConfig {
  /** Audio source */
  audioSource: AudioSource;

  /** Language (auto-detect if not specified) */
  language?: string;

  /** Enable speaker diarization */
  enableSpeakerDiarization?: boolean;

  /** Number of speakers (if known) */
  speakerCount?: number;

  /** Enable automatic language detection */
  enableAutomaticLanguageDetection?: boolean;

  /** Alternative languages to consider */
  alternativeLanguages?: string[];
}

export type AudioSource =
  | { type: 'file'; path: string }
  | { type: 'buffer'; data: Buffer | ArrayBuffer }
  | { type: 'stream'; stream: ReadableStream }
  | { type: 'url'; url: string };

export interface TranscriptionResult {
  /** Full transcript */
  fullTranscript: string;

  /** Segments with timestamps */
  segments: TranscriptSegment[];

  /** Detected language */
  language: string;

  /** Speaker information (if diarization enabled) */
  speakers?: SpeakerInfo[];

  /** Overall confidence */
  confidence: number;

  /** Processing duration */
  processingTime: number;
}

export interface TranscriptSegment {
  /** Segment text */
  text: string;

  /** Start time in milliseconds */
  startTime: number;

  /** End time in milliseconds */
  endTime: number;

  /** Speaker ID (if diarization enabled) */
  speaker?: number;

  /** Confidence score */
  confidence: number;

  /** Words in this segment */
  words: WordInfo[];
}

export interface SpeakerInfo {
  /** Speaker ID */
  speakerId: number;

  /** Speaker label/name */
  label?: string;

  /** Voice characteristics */
  voiceProfile?: VoiceCharacteristics;

  /** Total speaking time in milliseconds */
  totalSpeakingTime: number;
}

// ============================================================================
// Intent Recognition Types
// ============================================================================

export interface IntentRecognitionConfig {
  /** Text or transcript to analyze */
  text: string;

  /** Language code */
  language?: string;

  /** Domain/category hints */
  domain?: IntentDomain;

  /** Context from previous interactions */
  context?: ConversationContext;
}

export type IntentDomain =
  | 'general'
  | 'smart-home'
  | 'calendar'
  | 'music'
  | 'weather'
  | 'navigation'
  | 'shopping'
  | 'communication'
  | 'information';

export interface ConversationContext {
  /** Previous intents */
  previousIntents?: Intent[];

  /** Session variables */
  variables?: Record<string, any>;

  /** User preferences */
  preferences?: Record<string, any>;
}

export interface Intent {
  /** Intent name/type */
  name: string;

  /** Confidence score (0.0 to 1.0) */
  confidence: number;

  /** Extracted entities/parameters */
  entities: Entity[];

  /** Intent domain */
  domain?: IntentDomain;

  /** Suggested action */
  action?: string;

  /** Response text */
  responseText?: string;
}

export interface Entity {
  /** Entity type (e.g., 'date', 'location', 'person') */
  type: string;

  /** Entity value */
  value: any;

  /** Raw text matched */
  rawValue: string;

  /** Confidence score */
  confidence: number;

  /** Start position in text */
  startIndex?: number;

  /** End position in text */
  endIndex?: number;
}

// ============================================================================
// Wake Word Detection Types
// ============================================================================

export interface WakeWordConfig {
  /** Wake words to detect */
  wakeWords: string[];

  /** Sensitivity (0.0 to 1.0, higher = more sensitive) */
  sensitivity?: number;

  /** Audio configuration */
  audio: AudioConfig;

  /** Trigger action on detection */
  onDetected?: (wakeWord: string) => void;
}

export interface WakeWordDetection {
  /** Detected wake word */
  wakeWord: string;

  /** Confidence score */
  confidence: number;

  /** Detection timestamp */
  timestamp: Date;

  /** Audio segment with wake word */
  audioSegment?: Buffer;
}

// ============================================================================
// Speaker Identification Types
// ============================================================================

export interface SpeakerIdentificationConfig {
  /** Audio to analyze */
  audio: AudioSource;

  /** Known speaker profiles */
  speakerProfiles?: SpeakerProfile[];

  /** Create new profile if unknown */
  enrollUnknown?: boolean;
}

export interface SpeakerProfile {
  /** Unique speaker ID */
  speakerId: string;

  /** Speaker name */
  name?: string;

  /** Voice biometric data */
  voiceprint: string;

  /** Enrollment date */
  enrolledAt: Date;

  /** Metadata */
  metadata?: Record<string, any>;
}

export interface SpeakerIdentificationResult {
  /** Identified speaker */
  speaker?: SpeakerProfile;

  /** Confidence score */
  confidence: number;

  /** Is this a known speaker */
  isKnown: boolean;

  /** Alternative speaker matches */
  alternatives?: Array<{ speaker: SpeakerProfile; confidence: number }>;
}

// ============================================================================
// Event Types
// ============================================================================

export type VoiceEvent =
  | RecognitionEvent
  | SynthesisEvent
  | ErrorEvent
  | StatusEvent;

export interface RecognitionEvent {
  type: 'recognition';
  event: 'start' | 'interim' | 'final' | 'end';
  data: RecognitionResult;
  timestamp: Date;
}

export interface SynthesisEvent {
  type: 'synthesis';
  event: 'start' | 'progress' | 'end';
  data: Partial<SynthesisResult>;
  timestamp: Date;
}

export interface ErrorEvent {
  type: 'error';
  error: VoiceError;
  timestamp: Date;
}

export interface StatusEvent {
  type: 'status';
  status: 'connecting' | 'connected' | 'disconnected';
  timestamp: Date;
}

// ============================================================================
// Error Types
// ============================================================================

export interface VoiceError {
  /** Error code */
  code: VoiceErrorCode;

  /** Error message */
  message: string;

  /** Additional details */
  details?: Record<string, any>;
}

export type VoiceErrorCode =
  | 'AUDIO_ERROR'
  | 'NETWORK_ERROR'
  | 'AUTHENTICATION_ERROR'
  | 'INVALID_CONFIG'
  | 'UNSUPPORTED_LANGUAGE'
  | 'UNSUPPORTED_FORMAT'
  | 'QUOTA_EXCEEDED'
  | 'TIMEOUT'
  | 'UNKNOWN_ERROR';

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T> {
  /** Success status */
  success: boolean;

  /** Response data */
  data?: T;

  /** Error information */
  error?: VoiceError;

  /** Response metadata */
  metadata?: ResponseMetadata;
}

export interface ResponseMetadata {
  /** Request ID for tracking */
  requestId: string;

  /** Timestamp */
  timestamp: Date;

  /** Processing duration in milliseconds */
  processingTime?: number;

  /** Rate limit information */
  rateLimit?: RateLimitInfo;
}

export interface RateLimitInfo {
  /** Requests limit per period */
  limit: number;

  /** Remaining requests */
  remaining: number;

  /** Reset time */
  resetAt: Date;
}

/**
 * 弘益人間 (홍익인간)
 * Benefit All Humanity
 *
 * Voice accessibility for all
 */
