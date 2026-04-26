/**
 * WIA-UNI-012: Language Bridge Standard
 * TypeScript Type Definitions
 *
 * @package @wia/language-bridge
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export type Dialect = 'north' | 'south' | 'pyongyang' | 'seoul';
export type Register = 'formal' | 'informal' | 'neutral' | 'technical' | 'casual';
export type TranslationMode = 'realtime' | 'document' | 'educational' | 'sensitive';
export type Confidence = number; // 0-100

// ============================================================================
// Dictionary Types
// ============================================================================

export interface DictionaryEntry {
  headword: string;
  id: string;
  variants: {
    south: DialectVariant[];
    north: DialectVariant[];
  };
  etymology: Etymology;
  definitions: Definition[];
  culturalNotes: string[];
  examples: Example[];
  relatedTerms: string[];
  metadata?: {
    frequency?: 'very_high' | 'high' | 'medium' | 'low' | 'rare';
    domain?: string;
    firstRecorded?: number; // year
    lastUpdated?: Date;
  };
}

export interface DialectVariant {
  form: string;
  pronunciation: string;
  frequency: 'very_high' | 'high' | 'medium' | 'low' | 'rare';
  register: Register;
  firstUsage?: number; // year
  note?: string;
}

export interface Etymology {
  origin: string;
  path: string;
  notes: string;
  cognates?: {
    language: string;
    term: string;
  }[];
}

export interface Definition {
  text: string;
  domain?: string;
  examples?: string[];
}

export interface Example {
  region: 'north' | 'south';
  text: string;
  translation: string;
  context?: string;
}

// ============================================================================
// Translation Types
// ============================================================================

export interface TranslationRequest {
  text: string;
  sourceDialect: Dialect;
  targetDialect: Dialect;
  mode?: TranslationMode;
  context?: string;
  preserveNuance?: boolean;
  enableCulturalNotes?: boolean;
}

export interface TranslationResult {
  original: string;
  translated: string;
  confidence: Confidence;
  alternatives?: string[];
  culturalNotes?: string[];
  warnings?: string[];
  detectedRegister?: Register;
  suggestedRegister?: Register;
  processingTime: number; // milliseconds
}

export interface BatchTranslationRequest {
  texts: string[];
  sourceDialect: Dialect;
  targetDialect: Dialect;
  mode?: TranslationMode;
  maintainConsistency?: boolean; // use same terminology across batch
}

export interface BatchTranslationResult {
  results: TranslationResult[];
  totalProcessingTime: number;
  terminologyConsistency?: Record<string, string>; // mapping of consistent translations
}

// ============================================================================
// Dialect Mapping Types
// ============================================================================

export interface DialectDifference {
  category: string;
  northTerm: string;
  southTerm: string;
  explanation: string;
  culturalContext?: string;
  examples?: {
    north: string;
    south: string;
  };
}

export interface DialectMappingRequest {
  category?: string;
  includeHistorical?: boolean;
  limit?: number;
  offset?: number;
}

export interface DialectMappingResult {
  differences: DialectDifference[];
  total: number;
  categories: string[];
}

// ============================================================================
// Learning Types
// ============================================================================

export interface LearningLesson {
  id: string;
  title: string;
  titleKo: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced' | 'expert';
  topic: string;
  duration: number; // minutes
  objectives: string[];
  content: LessonContent;
  assessment?: Assessment;
}

export interface LessonContent {
  introduction: string;
  sections: {
    title: string;
    content: string;
    exercises?: Exercise[];
  }[];
  summary: string;
  vocabulary: DictionaryEntry[];
}

export interface Exercise {
  type: 'multiple-choice' | 'fill-blank' | 'translation' | 'listening' | 'speaking';
  question: string;
  options?: string[];
  correctAnswer: string | string[];
  explanation: string;
  audioUrl?: string;
}

export interface Assessment {
  questions: Exercise[];
  passingScore: number; // percentage
  certificateEligible: boolean;
}

export interface LearningProgress {
  userId: string;
  lessonsCompleted: string[];
  currentLesson?: string;
  scores: Record<string, number>; // lessonId -> score
  totalStudyTime: number; // minutes
  certificatesEarned: string[];
}

// ============================================================================
// Voice Translation Types
// ============================================================================

export interface VoiceTranslationRequest {
  audioData: Buffer | Blob;
  sourceDialect: Dialect;
  targetDialect: Dialect;
  outputFormat?: 'text' | 'audio' | 'both';
}

export interface VoiceTranslationResult {
  transcription: string;
  translation: string;
  confidence: Confidence;
  audioUrl?: string; // if outputFormat includes 'audio'
  detectedAccent?: string;
  pronunciationNotes?: string[];
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface LanguageBridgeConfig {
  apiKey?: string;
  apiEndpoint?: string;
  defaultSourceDialect?: Dialect;
  defaultTargetDialect?: Dialect;
  enableCaching?: boolean;
  cacheDuration?: number; // seconds
  timeout?: number; // milliseconds
  offlineMode?: boolean;
  logLevel?: 'debug' | 'info' | 'warn' | 'error';
}

export interface APIResponse<T = any> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    requestId: string;
    timestamp: string;
    version: string;
  };
}

// ============================================================================
// Search Types
// ============================================================================

export interface SearchRequest {
  query: string;
  searchType?: 'exact' | 'fuzzy' | 'phonetic';
  dialect?: Dialect;
  category?: string;
  limit?: number;
  offset?: number;
}

export interface SearchResult {
  entries: DictionaryEntry[];
  total: number;
  suggestions?: string[];
}

// ============================================================================
// Contribution Types
// ============================================================================

export interface ContributionRequest {
  type: 'new-entry' | 'correction' | 'example' | 'cultural-note';
  entryId?: string; // for corrections/additions
  data: Partial<DictionaryEntry> | Example | string;
  contributorId: string;
  notes?: string;
}

export interface ContributionStatus {
  id: string;
  status: 'pending' | 'under-review' | 'approved' | 'rejected';
  contributorId: string;
  submittedAt: Date;
  reviewedAt?: Date;
  reviewerNotes?: string;
}
