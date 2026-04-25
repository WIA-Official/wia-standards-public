/**
 * WIA Sign Language Standard
 * TypeScript Type Definitions
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export type SignLanguageType =
  | 'ASL'      // American Sign Language
  | 'BSL'      // British Sign Language
  | 'JSL'      // Japanese Sign Language
  | 'KSL'      // Korean Sign Language
  | 'CSL'      // Chinese Sign Language
  | 'LSF'      // French Sign Language (Langue des Signes Française)
  | 'DGS'      // German Sign Language (Deutsche Gebärdensprache)
  | 'Auslan'   // Australian Sign Language
  | 'ISL'      // Irish Sign Language
  | 'LSE'      // Spanish Sign Language (Lengua de Señas Española)
  | 'LIS'      // Italian Sign Language (Lingua dei Segni Italiana)
  | 'NZSL'     // New Zealand Sign Language
  | 'SSL'      // Swedish Sign Language
  | 'ISN'      // International Sign
  | 'Custom';

export type RecognitionMode =
  | 'realtime'
  | 'offline'
  | 'batch'
  | 'streaming';

export type HandDominance =
  | 'right'
  | 'left'
  | 'ambidextrous';

export type SignComplexity =
  | 'simple'      // Single hand, basic movement
  | 'moderate'    // One or two hands, moderate movement
  | 'complex'     // Two hands, intricate movements
  | 'advanced';   // Multiple components, facial expressions

// ============================================================================
// Configuration
// ============================================================================

export interface SignLanguageConfig {
  /** API key for authentication */
  apiKey: string;

  /** Primary sign language */
  primaryLanguage: SignLanguageType;

  /** Secondary languages for translation */
  secondaryLanguages?: SignLanguageType[];

  /** API endpoint */
  endpoint?: string;

  /** Recognition mode */
  mode?: RecognitionMode;

  /** Enable facial expression detection */
  enableFacialExpression?: boolean;

  /** Enable body language detection */
  enableBodyLanguage?: boolean;

  /** Minimum confidence threshold (0-1) */
  confidenceThreshold?: number;

  /** Frame rate for video processing */
  frameRate?: number;

  /** Custom metadata */
  metadata?: Record<string, any>;
}

// ============================================================================
// Hand Pose and Movement Types
// ============================================================================

export interface HandPose {
  /** Hand identifier (left/right) */
  hand: 'left' | 'right';

  /** 21 landmark points for hand skeleton */
  landmarks: Landmark3D[];

  /** Hand orientation in 3D space */
  orientation: Orientation3D;

  /** Hand shape classification */
  shape: HandShape;

  /** Confidence score (0-1) */
  confidence: number;

  /** Timestamp */
  timestamp: number;
}

export interface Landmark3D {
  x: number;
  y: number;
  z: number;
  visibility?: number;
}

export interface Orientation3D {
  /** Rotation around x-axis (pitch) */
  pitch: number;

  /** Rotation around y-axis (yaw) */
  yaw: number;

  /** Rotation around z-axis (roll) */
  roll: number;
}

export type HandShape =
  | 'fist'
  | 'flat'
  | 'point'
  | 'pinch'
  | 'spread'
  | 'curved'
  | 'bent'
  | 'custom';

export interface Movement {
  /** Type of movement */
  type: MovementType;

  /** Direction vector */
  direction?: Vector3D;

  /** Speed (units per second) */
  speed?: number;

  /** Path trajectory */
  trajectory?: Landmark3D[];

  /** Duration in milliseconds */
  duration: number;
}

export type MovementType =
  | 'static'
  | 'linear'
  | 'circular'
  | 'arc'
  | 'zigzag'
  | 'wave'
  | 'tap'
  | 'shake'
  | 'twist'
  | 'custom';

export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

// ============================================================================
// Facial Expression Types
// ============================================================================

export interface FacialExpression {
  /** Detected expressions */
  expressions: Expression[];

  /** Eye gaze direction */
  gazeDirection?: GazeDirection;

  /** Eyebrow position */
  eyebrows?: EyebrowPosition;

  /** Mouth shape */
  mouthShape?: MouthShape;

  /** Head pose */
  headPose?: Orientation3D;

  /** Confidence score */
  confidence: number;

  /** Timestamp */
  timestamp: number;
}

export type Expression =
  | 'neutral'
  | 'questioning'
  | 'emphasis'
  | 'negation'
  | 'affirmation'
  | 'surprise'
  | 'concern';

export interface GazeDirection {
  horizontal: number; // -1 (left) to 1 (right)
  vertical: number;   // -1 (down) to 1 (up)
}

export type EyebrowPosition =
  | 'raised'
  | 'neutral'
  | 'lowered'
  | 'furrowed';

export type MouthShape =
  | 'closed'
  | 'open'
  | 'rounded'
  | 'stretched'
  | 'pursed';

// ============================================================================
// Sign Vocabulary and Dictionary
// ============================================================================

export interface SignEntry {
  /** Unique identifier */
  id: string;

  /** Word or phrase in spoken language */
  word: string;

  /** Sign language type */
  language: SignLanguageType;

  /** Sign complexity level */
  complexity: SignComplexity;

  /** Hand poses sequence */
  handPoses: HandPose[];

  /** Movements involved */
  movements: Movement[];

  /** Required facial expressions */
  facialExpressions?: FacialExpression[];

  /** Video demonstration URL */
  videoUrl?: string;

  /** Image reference URL */
  imageUrl?: string;

  /** Category/topic */
  category?: string;

  /** Usage examples */
  examples?: string[];

  /** Regional variations */
  regionalVariations?: RegionalVariation[];

  /** Related signs */
  relatedSigns?: string[];

  /** Metadata */
  metadata?: Record<string, any>;
}

export interface RegionalVariation {
  region: string;
  description: string;
  videoUrl?: string;
}

export interface SignDictionary {
  /** Dictionary name */
  name: string;

  /** Sign language */
  language: SignLanguageType;

  /** Total entries */
  totalEntries: number;

  /** Dictionary entries */
  entries: SignEntry[];

  /** Categories */
  categories: string[];

  /** Version */
  version: string;

  /** Last updated */
  lastUpdated: Date;
}

// ============================================================================
// Recognition and Translation
// ============================================================================

export interface RecognitionRequest {
  /** Input source */
  source: InputSource;

  /** Target language for translation */
  targetLanguage?: string;

  /** Source sign language */
  sourceSignLanguage?: SignLanguageType;

  /** Recognition options */
  options?: RecognitionOptions;
}

export interface InputSource {
  /** Input type */
  type: 'video' | 'image' | 'stream' | 'webcam';

  /** Data payload (URL, base64, or buffer) */
  data: string | ArrayBuffer;

  /** Duration for video (milliseconds) */
  duration?: number;

  /** Frame rate */
  frameRate?: number;
}

export interface RecognitionOptions {
  /** Enable continuous recognition */
  continuous?: boolean;

  /** Include alternative interpretations */
  includeAlternatives?: boolean;

  /** Maximum alternatives to return */
  maxAlternatives?: number;

  /** Real-time callback */
  onPartialResult?: (result: PartialRecognitionResult) => void;
}

export interface RecognitionResult {
  /** Recognized text */
  text: string;

  /** Confidence score (0-1) */
  confidence: number;

  /** Detected signs */
  signs: DetectedSign[];

  /** Alternative interpretations */
  alternatives?: AlternativeResult[];

  /** Processing time (ms) */
  processingTime: number;

  /** Timestamp */
  timestamp: Date;
}

export interface DetectedSign {
  /** Recognized word/phrase */
  word: string;

  /** Confidence score */
  confidence: number;

  /** Time range in video */
  timeRange?: TimeRange;

  /** Frame indices */
  frameIndices?: number[];

  /** Hand poses detected */
  handPoses: HandPose[];

  /** Movements detected */
  movements?: Movement[];

  /** Facial expressions */
  facialExpressions?: FacialExpression[];
}

export interface TimeRange {
  start: number;
  end: number;
}

export interface AlternativeResult {
  text: string;
  confidence: number;
}

export interface PartialRecognitionResult {
  /** Partially recognized text */
  text: string;

  /** Confidence */
  confidence: number;

  /** Is final result */
  isFinal: boolean;
}

// ============================================================================
// Translation (Text to Sign)
// ============================================================================

export interface TranslationRequest {
  /** Text to translate */
  text: string;

  /** Source language (ISO 639-1) */
  sourceLanguage: string;

  /** Target sign language */
  targetSignLanguage: SignLanguageType;

  /** Output format */
  outputFormat: OutputFormat;

  /** Options */
  options?: TranslationOptions;
}

export type OutputFormat =
  | 'video'
  | 'animation'
  | 'sequence'
  | 'description';

export interface TranslationOptions {
  /** Avatar gender for video */
  avatarGender?: 'male' | 'female' | 'neutral';

  /** Avatar skin tone */
  avatarSkinTone?: string;

  /** Signing speed */
  speed?: 'slow' | 'normal' | 'fast';

  /** Include facial expressions */
  includeFacialExpressions?: boolean;

  /** Video quality */
  videoQuality?: 'low' | 'medium' | 'high' | '4k';
}

export interface TranslationResult {
  /** Original text */
  originalText: string;

  /** Sign sequence */
  signSequence: SignEntry[];

  /** Generated output */
  output: TranslationOutput;

  /** Metadata */
  metadata: TranslationMetadata;
}

export interface TranslationOutput {
  /** Output format */
  format: OutputFormat;

  /** Video URL (if format is video) */
  videoUrl?: string;

  /** Animation data (if format is animation) */
  animationData?: AnimationFrame[];

  /** Sign sequence description */
  description?: string;
}

export interface AnimationFrame {
  /** Frame number */
  frame: number;

  /** Hand poses for this frame */
  handPoses: HandPose[];

  /** Facial expression */
  facialExpression?: FacialExpression;

  /** Timestamp */
  timestamp: number;
}

export interface TranslationMetadata {
  /** Processing time */
  processingTime: number;

  /** Number of signs */
  signCount: number;

  /** Estimated duration */
  estimatedDuration: number;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Learning and Practice
// ============================================================================

export interface LearningModule {
  /** Module ID */
  id: string;

  /** Module title */
  title: string;

  /** Description */
  description: string;

  /** Difficulty level */
  level: 'beginner' | 'intermediate' | 'advanced';

  /** Lessons in module */
  lessons: Lesson[];

  /** Progress tracking */
  progress?: ModuleProgress;
}

export interface Lesson {
  /** Lesson ID */
  id: string;

  /** Lesson title */
  title: string;

  /** Signs to learn */
  signs: SignEntry[];

  /** Practice exercises */
  exercises: Exercise[];

  /** Estimated duration (minutes) */
  duration: number;
}

export interface Exercise {
  /** Exercise type */
  type: 'recognition' | 'production' | 'translation' | 'quiz';

  /** Instructions */
  instructions: string;

  /** Exercise data */
  data: any;

  /** Correct answer */
  correctAnswer?: any;
}

export interface ModuleProgress {
  /** Completion percentage */
  completion: number;

  /** Lessons completed */
  lessonsCompleted: number;

  /** Total lessons */
  totalLessons: number;

  /** Practice score */
  score: number;

  /** Last accessed */
  lastAccessed: Date;
}

export interface PracticeSession {
  /** Session ID */
  sessionId: string;

  /** Signs practiced */
  signsPracticed: string[];

  /** Performance metrics */
  performance: PerformanceMetrics;

  /** Duration (milliseconds) */
  duration: number;

  /** Timestamp */
  timestamp: Date;
}

export interface PerformanceMetrics {
  /** Accuracy percentage */
  accuracy: number;

  /** Signs correct */
  correct: number;

  /** Signs incorrect */
  incorrect: number;

  /** Average confidence */
  averageConfidence: number;

  /** Areas for improvement */
  improvements: string[];
}

// ============================================================================
// Accessibility Profiles
// ============================================================================

export interface AccessibilityProfile {
  /** User ID */
  userId: string;

  /** User's hand dominance */
  handDominance: HandDominance;

  /** Physical limitations */
  limitations?: PhysicalLimitation[];

  /** Preferred signing style */
  signingStyle?: 'formal' | 'casual' | 'regional';

  /** Visual preferences */
  visualPreferences?: VisualPreferences;

  /** Learning preferences */
  learningPreferences?: LearningPreferences;
}

export interface PhysicalLimitation {
  type: string;
  description: string;
  adaptations?: string[];
}

export interface VisualPreferences {
  /** Contrast level */
  contrast: 'low' | 'medium' | 'high';

  /** Font size multiplier */
  fontSizeMultiplier: number;

  /** Enable captions */
  captions: boolean;

  /** Video playback speed */
  playbackSpeed: number;
}

export interface LearningPreferences {
  /** Preferred lesson duration */
  lessonDuration: number;

  /** Reminders enabled */
  reminders: boolean;

  /** Practice frequency */
  practiceFrequency: 'daily' | 'weekly' | 'custom';
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Date;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
}

// ============================================================================
// Real-time Interpretation
// ============================================================================

export interface InterpretationSession {
  /** Session ID */
  sessionId: string;

  /** Source sign language */
  sourceLanguage: SignLanguageType;

  /** Target text language */
  targetLanguage: string;

  /** Session status */
  status: 'active' | 'paused' | 'ended';

  /** Start time */
  startTime: Date;

  /** End time */
  endTime?: Date;

  /** Transcript */
  transcript: TranscriptEntry[];
}

export interface TranscriptEntry {
  /** Entry ID */
  id: string;

  /** Recognized text */
  text: string;

  /** Confidence */
  confidence: number;

  /** Timestamp */
  timestamp: Date;

  /** Speaker/signer ID */
  signerId?: string;
}

// ============================================================================
// Event Types
// ============================================================================

export type SignLanguageEvent =
  | 'recognition-started'
  | 'recognition-progress'
  | 'recognition-completed'
  | 'recognition-error'
  | 'translation-started'
  | 'translation-completed'
  | 'translation-error'
  | 'session-started'
  | 'session-ended';

export interface EventCallback {
  (event: SignLanguageEvent, data: any): void;
}

/**
 * 弘益人間 (홍익인간)
 * Benefit All Humanity
 *
 * Making communication accessible to all
 */
