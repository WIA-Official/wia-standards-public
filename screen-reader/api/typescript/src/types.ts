/**
 * WIA Screen Reader Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @license MIT
 */

/**
 * Supported language codes (ISO 639-1)
 */
export type LanguageCode =
  | 'en' | 'ko' | 'ja' | 'zh' | 'es' | 'fr' | 'de' | 'it' | 'pt' | 'ru'
  | 'ar' | 'hi' | 'th' | 'vi' | 'id' | 'ms' | 'tl' | 'nl' | 'pl' | 'tr'
  | string;

/**
 * Braille grade types
 */
export type BrailleGrade = 1 | 2;

/**
 * ARIA element types
 */
export type ElementType =
  | 'heading' | 'paragraph' | 'list' | 'listitem' | 'link'
  | 'button' | 'textbox' | 'checkbox' | 'radio' | 'slider'
  | 'table' | 'row' | 'cell' | 'image' | 'figure' | 'form'
  | 'navigation' | 'main' | 'article' | 'section' | 'aside'
  | 'header' | 'footer' | 'dialog' | 'alert' | 'menu' | 'menuitem';

/**
 * ARIA landmark roles
 */
export type LandmarkRole =
  | 'banner' | 'main' | 'navigation' | 'contentinfo'
  | 'search' | 'form' | 'complementary' | 'region';

/**
 * ARIA live region types
 */
export type LiveRegion = 'off' | 'polite' | 'assertive';

/**
 * Pronunciation output
 */
export interface Pronunciation {
  /** International Phonetic Alphabet */
  ipa: string;
  /** WIA International Hangul Pronunciation */
  wihp: string;
  /** Romanized form */
  romanized?: string;
  /** Syllable breakdown */
  syllables?: string[];
}

/**
 * Braille cell representation
 */
export interface BrailleCell {
  /** Source character */
  char: string;
  /** Active dots (1-8) */
  dots: number[];
  /** Unicode braille character */
  unicode: string;
}

/**
 * Braille output
 */
export interface BrailleOutput {
  /** Grade 1 (uncontracted) braille */
  grade1: string;
  /** Grade 2 (contracted) braille */
  grade2: string;
  /** WIA Korean-based braille format */
  wia: string;
  /** Dot patterns for each cell */
  dots?: BrailleCell[];
  /** Total number of cells */
  cells: number;
}

/**
 * TTS configuration
 */
export interface TTSConfig {
  /** Speech rate (0.1-10, default 1.0) */
  rate?: number;
  /** Voice pitch (0.1-2.0, default 1.0) */
  pitch?: number;
  /** Volume (0-1, default 1.0) */
  volume?: number;
  /** Voice identifier */
  voice?: string;
  /** SSML markup */
  ssml?: string;
}

/**
 * Element state
 */
export interface ElementState {
  expanded?: boolean;
  selected?: boolean;
  checked?: boolean | 'mixed';
  pressed?: boolean | 'mixed';
  disabled?: boolean;
  readonly?: boolean;
  required?: boolean;
  invalid?: boolean;
}

/**
 * Position in a set
 */
export interface Position {
  /** Current position (1-indexed) */
  index: number;
  /** Total items */
  total: number;
}

/**
 * Semantic context
 */
export interface Context {
  /** HTML/ARIA element type */
  elementType?: ElementType;
  /** Heading level (1-6) */
  level?: number;
  /** ARIA landmark */
  landmark?: LandmarkRole;
  /** Live region type */
  live?: LiveRegion;
  /** Position in set */
  position?: Position;
  /** Element state */
  state?: ElementState;
}

/**
 * Processing metadata
 */
export interface Metadata {
  /** Processing timestamp */
  processedAt: Date;
  /** Processing time in milliseconds */
  processingTimeMs: number;
  /** Engine version */
  engineVersion: string;
  /** Confidence score (0-1) */
  confidence?: number;
}

/**
 * Screen reader result
 */
export interface ScreenReaderResult {
  /** Original text */
  text: string;
  /** Source language */
  language: LanguageCode;
  /** Pronunciation data */
  pronunciation: Pronunciation;
  /** Braille output */
  braille: BrailleOutput;
  /** TTS configuration */
  tts?: TTSConfig;
  /** Semantic context */
  context?: Context;
  /** Processing metadata */
  metadata: Metadata;
}

/**
 * WIA Screen Reader configuration
 */
export interface WIAScreenReaderConfig {
  /** Default language */
  defaultLanguage?: LanguageCode;
  /** Default braille grade */
  defaultBrailleGrade?: BrailleGrade;
  /** TTS settings */
  tts?: TTSConfig;
  /** Auto-detect language */
  autoDetectLanguage?: boolean;
  /** API endpoint (for remote processing) */
  apiEndpoint?: string;
  /** API key */
  apiKey?: string;
}

/**
 * Element processing options
 */
export interface ProcessElementOptions {
  /** Include child elements */
  includeChildren?: boolean;
  /** Maximum depth */
  maxDepth?: number;
  /** Include hidden elements */
  includeHidden?: boolean;
}

/**
 * Speak options
 */
export interface SpeakOptions extends TTSConfig {
  /** Interrupt current speech */
  interrupt?: boolean;
  /** Use WIHP pronunciation */
  useWIHP?: boolean;
}

/**
 * WIA Screen Reader data format (JSON schema compatible)
 */
export interface WIAScreenReaderData {
  wia_screen_reader: {
    version: string;
    text: string;
    language?: LanguageCode;
    pronunciation?: Partial<Pronunciation>;
    braille?: Partial<BrailleOutput>;
    tts?: TTSConfig;
    context?: Context;
    metadata?: Partial<Metadata>;
  };
}
