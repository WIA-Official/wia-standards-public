/**
 * WIA-PET-010 Pet Translator TypeScript Type Definitions
 * Version 2.0.0
 *
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ============================================================================
// Enums
// ============================================================================

export enum Species {
  Dog = 'dog',
  Cat = 'cat',
  Bird = 'bird',
  Rabbit = 'rabbit',
  GuineaPig = 'guinea_pig',
  Hamster = 'hamster',
  Mouse = 'mouse',
  Rat = 'rat',
}

export enum EventType {
  Vocalization = 'vocalization',
  BodyLanguage = 'body_language',
  FacialExpression = 'facial_expression',
  Combined = 'combined',
}

export enum Emotion {
  Happy = 'happy',
  Excited = 'excited',
  Anxious = 'anxious',
  Fearful = 'fearful',
  Angry = 'angry',
  Sad = 'sad',
  Content = 'content',
  Curious = 'curious',
  Playful = 'playful',
  Alert = 'alert',
  Pain = 'pain',
  Hungry = 'hungry',
  Bored = 'bored',
  Affectionate = 'affectionate',
}

export enum Urgency {
  Low = 'low',
  Medium = 'medium',
  High = 'high',
  Critical = 'critical',
}

export enum AudioFormat {
  WAV = 'wav',
  MP3 = 'mp3',
  FLAC = 'flac',
  OGG = 'ogg',
  RAW = 'raw',
}

export enum VideoFormat {
  MP4 = 'mp4',
  H264 = 'h264',
  H265 = 'h265',
  WEBM = 'webm',
}

export enum Language {
  English = 'en',
  Korean = 'ko',
  Spanish = 'es',
  French = 'fr',
  German = 'de',
  Japanese = 'ja',
  Chinese = 'zh',
  Italian = 'it',
  Portuguese = 'pt',
  Russian = 'ru',
  Arabic = 'ar',
  Hindi = 'hi',
  Dutch = 'nl',
}

export enum TimeOfDay {
  Morning = 'morning',
  Afternoon = 'afternoon',
  Evening = 'evening',
  Night = 'night',
}

// ============================================================================
// Pet Profile
// ============================================================================

export interface PetProfile {
  petId: string;
  name?: string;
  species: Species;
  breed?: string;
  age?: number; // months
  sex?: 'male' | 'female' | 'unknown';
  neutered?: boolean;
  weight?: number; // kg
  healthConditions?: string[];
  personalityTraits?: string[];
  notes?: string;
}

// ============================================================================
// Audio Data
// ============================================================================

export interface AudioData {
  sampleRate: number; // Hz
  channels: number;
  bitDepth: number;
  duration: number; // ms
  format: AudioFormat;
  encoding: 'base64' | 'url' | 'buffer';
  data: string | Buffer;
  metadata?: {
    deviceId?: string;
    location?: string;
    backgroundNoise?: number; // dB
  };
}

// ============================================================================
// Video Data
// ============================================================================

export interface VideoData {
  width: number;
  height: number;
  fps: number;
  duration: number; // ms
  format: VideoFormat;
  encoding: 'base64' | 'url' | 'buffer';
  frames?: string | Buffer;
  metadata?: {
    deviceId?: string;
    lightingCondition?: 'natural' | 'artificial' | 'dim' | 'dark';
    cameraAngle?: 'front' | 'side' | 'top' | 'rear';
  };
}

// ============================================================================
// Sensor Data
// ============================================================================

export interface SensorData {
  heartRate?: number; // bpm
  temperature?: number; // celsius
  activityLevel?: number; // 0.0 to 1.0
  location?: {
    lat: number;
    lon: number;
  };
}

// ============================================================================
// Context
// ============================================================================

export interface TemporalContext {
  timeOfDay: TimeOfDay;
  dayOfWeek?: string;
  isHoliday?: boolean;
  timeSinceLastMeal?: number; // seconds
  timeSinceLastWalk?: number; // seconds
  timeSinceLastPlay?: number; // seconds;
}

export interface EnvironmentalContext {
  location: string;
  temperature?: number; // celsius
  humidity?: number; // percentage
  weather?: string;
  noiseLevel?: number; // dB
  presentIndividuals?: string[];
}

export interface Activity {
  activity: string;
  timestamp: string; // ISO 8601
  duration?: number; // seconds
}

export interface BehavioralBaseline {
  typicalVocalizationsPerHour: number;
  averageArousalLevel: number; // 0.0 to 1.0
  commonEmotions: Emotion[];
}

export interface HistoricalContext {
  recentActivities: Activity[];
  behavioralBaseline?: BehavioralBaseline;
}

export interface Context {
  temporal: TemporalContext;
  environmental: EnvironmentalContext;
  historical?: HistoricalContext;
}

// ============================================================================
// Translation Event (Input)
// ============================================================================

export interface TranslationEvent {
  eventId?: string;
  timestamp?: string; // ISO 8601
  petProfile: PetProfile;
  modalities: {
    audio?: AudioData;
    video?: VideoData;
    sensors?: SensorData;
  };
  context?: Context;
}

// ============================================================================
// Emotion Result
// ============================================================================

export interface EmotionDimensional {
  valence: number; // -1.0 (negative) to 1.0 (positive)
  arousal: number; // 0.0 (calm) to 1.0 (excited)
  dominance: number; // 0.0 (submissive) to 1.0 (dominant)
}

export interface EmotionTimeline {
  emotion: Emotion;
  timestamp: number; // ms relative to event
}

export interface EmotionResult {
  primary: Emotion;
  secondary?: Emotion[];
  confidence: number; // 0.0 to 1.0
  dimensional: EmotionDimensional;
  timeline?: EmotionTimeline[];
}

// ============================================================================
// Intent Result
// ============================================================================

export interface IntentResult {
  category: string;
  specific: string;
  confidence: number; // 0.0 to 1.0
  urgency: Urgency;
  requiresResponse: boolean;
}

// ============================================================================
// Behavior Detection
// ============================================================================

export interface BehaviorResult {
  detected: string[];
  posture?: string;
  gaze?: string;
  movement?: string;
}

// ============================================================================
// Confidence Breakdown
// ============================================================================

export interface ConfidenceBreakdown {
  overall: number; // 0.0 to 1.0
  modality: {
    audio?: number;
    video?: number;
    context?: number;
  };
}

// ============================================================================
// Translation Alternative
// ============================================================================

export interface TranslationAlternative {
  translation: string;
  confidence: number;
}

// ============================================================================
// Insights
// ============================================================================

export interface Insights {
  comparedToBaseline?: string;
  predictedNextBehavior?: string;
  recommendedResponse?: string;
  healthAlert?: string;
}

// ============================================================================
// Translation Result (Output)
// ============================================================================

export interface TranslationResult {
  translationId: string;
  eventId: string;
  timestamp: string; // ISO 8601
  processingTime: number; // ms
  translations: Record<Language | string, string>;
  emotion: EmotionResult;
  intent: IntentResult;
  behaviors?: BehaviorResult;
  confidence: ConfidenceBreakdown;
  alternatives?: TranslationAlternative[];
  insights?: Insights;
}

// ============================================================================
// API Configuration
// ============================================================================

export interface PetTranslatorConfig {
  apiKey: string;
  baseUrl?: string;
  websocketUrl?: string;
  timeout?: number; // ms
  retries?: number;
  languages?: Language[];
  localProcessing?: boolean;
}

// ============================================================================
// Subscription Options
// ============================================================================

export interface SubscriptionOptions {
  petId: string;
  species: Species;
  breed?: string;
  streamConfig?: {
    audio: boolean;
    video: boolean;
    sensors: boolean;
  };
  languages?: Language[];
}

// ============================================================================
// Emotion Timeline Query
// ============================================================================

export interface EmotionTimelineQuery {
  petId: string;
  startDate: Date | string;
  endDate: Date | string;
  granularity?: 'minute' | 'hour' | 'day';
}

export interface EmotionTimelineDataPoint {
  timestamp: string; // ISO 8601
  emotion: Emotion;
  arousal: number;
  valence: number;
}

export interface EmotionTimelineSummary {
  averageValence: number;
  averageArousal: number;
  stressEpisodes: number;
  peakHappiness?: string; // ISO 8601
  mostCommonEmotion: Emotion;
}

export interface EmotionTimelineResponse {
  petId: string;
  timeRange: {
    start: string; // ISO 8601
    end: string; // ISO 8601
  };
  timeline: EmotionTimelineDataPoint[];
  summary: EmotionTimelineSummary;
}

// ============================================================================
// History Query
// ============================================================================

export interface HistoryQuery {
  petId: string;
  startDate?: Date | string;
  endDate?: Date | string;
  limit?: number;
  offset?: number;
  emotions?: Emotion[];
  minConfidence?: number;
}

export interface HistoryResponse {
  total: number;
  translations: TranslationResult[];
  hasMore: boolean;
}

// ============================================================================
// Webhook Configuration
// ============================================================================

export interface WebhookConfig {
  url: string;
  events: ('translation' | 'emotion_change' | 'health_alert')[];
  secret?: string;
  enabled: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

// ============================================================================
// Events
// ============================================================================

export interface TranslationEventData extends TranslationResult {}

export interface EmotionChangeEventData {
  petId: string;
  previousEmotion: Emotion;
  currentEmotion: Emotion;
  timestamp: string; // ISO 8601
}

export interface ConnectionEventData {
  status: 'connected' | 'disconnected' | 'error';
  message?: string;
}

export interface ErrorEventData extends APIError {}
