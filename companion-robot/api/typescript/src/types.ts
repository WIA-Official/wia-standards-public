/**
 * WIA-ROB-013 Companion Robot TypeScript Types
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Personality Types
// ============================================================================

export interface PersonalityTraits {
  openness: number;              // 0-100
  conscientiousness: number;     // 0-100
  extraversion: number;          // 0-100
  agreeableness: number;         // 0-100
  neuroticism: number;           // 0-100
  playfulness: number;           // 0-100
  formality: number;             // 0-100
  proactivity: number;           // 0-100
}

export enum CompanionPurpose {
  EMOTIONAL_SUPPORT = "emotional_support",
  EDUCATION = "education",
  HEALTH_WELLNESS = "health_wellness",
  ENTERTAINMENT = "entertainment",
  DAILY_ASSISTANCE = "daily_assistance"
}

export enum AgeRange {
  CHILD = "child",
  TEEN = "teen",
  ADULT = "adult",
  SENIOR = "senior"
}

export interface CompanionProfile {
  id: string;
  version: string;
  createdAt: string;
  lastUpdated: string;

  name: string;
  avatar?: string;
  description: string;

  personality: PersonalityTraits;
  primaryPurpose: CompanionPurpose;
  targetAgeRange: AgeRange;
  communicationStyle: string;

  languagePrimary: string;
  languagesSupported: string[];

  capabilities: string[];
  limitations: string[];
}

// ============================================================================
// Emotion Types
// ============================================================================

export enum PrimaryEmotion {
  JOY = "joy",
  SADNESS = "sadness",
  ANGER = "anger",
  FEAR = "fear",
  SURPRISE = "surprise",
  DISGUST = "disgust",
  CONTEMPT = "contempt"
}

export interface EmotionalDimensions {
  valence: number;    // -1.0 to +1.0
  arousal: number;    // 0.0 to 1.0
  dominance: number;  // -1.0 to +1.0
}

export interface EmotionProbability {
  emotion: string;
  probability: number;  // 0.0 to 1.0
}

export interface EmotionSources {
  text?: {
    emotion: PrimaryEmotion;
    confidence: number;
  };
  voice?: {
    emotion: PrimaryEmotion;
    confidence: number;
  };
  facial?: {
    emotion: PrimaryEmotion;
    confidence: number;
  };
}

export interface EmotionState {
  timestamp: string;
  primaryEmotion: PrimaryEmotion;
  secondaryEmotions: EmotionProbability[];
  dimensions: EmotionalDimensions;
  overallConfidence: number;
  sources: EmotionSources;
}

// ============================================================================
// Message Types
// ============================================================================

export interface MessageContent {
  text: string;
  language: string;
  emotionalTone?: EmotionalDimensions;
}

export interface MessageContext {
  conversationId: string;
  sessionId: string;
  threadId?: string;

  userEmotion?: EmotionState;
  userActivity?: string;
  userLocation?: string;

  timeOfDay: "morning" | "afternoon" | "evening" | "night";
  dayOfWeek: string;
  specialOccasion?: string;
}

export interface MessageMetadata {
  processingTime?: number;
  confidence?: number;
  alternatives?: string[];
}

export interface InteractionMessage {
  id: string;
  timestamp: string;
  sender: "user" | "companion";
  content: MessageContent;
  context: MessageContext;
  metadata: MessageMetadata;
}

// ============================================================================
// Session Types
// ============================================================================

export interface SessionPreferences {
  language?: string;
  notificationsEnabled?: boolean;
  [key: string]: any;
}

export interface Session {
  sessionId: string;
  companionId: string;
  userId: string;
  createdAt: string;
  expiresAt: string;
  status: "active" | "paused" | "ended";
  messageCount: number;
  duration: number;
}

// ============================================================================
// Memory Types
// ============================================================================

export enum MemoryType {
  EVENT = "event",
  FACT = "fact",
  PREFERENCE = "preference",
  GOAL = "goal",
  CONCERN = "concern"
}

export interface Memory {
  id: string;
  timestamp: string;
  type: MemoryType;
  content: string;
  importance: number;  // 0.0 to 1.0
  lastAccessed: string;
  accessCount: number;
}

export interface UserProfile {
  personalInfo: {
    name?: string;
    interests: string[];
    goals: string[];
    challenges: string[];
  };

  communicationPreferences: {
    preferredLanguage: string;
    formalityLevel: number;
    verbosity: number;
    emojiUsage: number;
  };

  emotionalProfile: {
    baselineAffect: EmotionalDimensions;
    expressionStyle: "reserved" | "moderate" | "expressive";
    commonTriggers: {
      positive: string[];
      negative: string[];
    };
    copingStrategies: string[];
  };
}

// ============================================================================
// API Response Types
// ============================================================================

export interface CompanionResponse {
  text: string;
  emotionalTone: EmotionalDimensions;
  suggestions?: string[];
}

export interface MessageResponse {
  messageId: string;
  companionResponse: CompanionResponse;
  processingTime: number;
}

export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
    timestamp: string;
  };
}

// ============================================================================
// Crisis Types
// ============================================================================

export enum CrisisType {
  SUICIDAL_IDEATION = "suicidal_ideation",
  SELF_HARM = "self_harm",
  HARM_TO_OTHERS = "harm_to_others",
  SEVERE_DISTRESS = "severe_distress",
  ABUSE_DISCLOSURE = "abuse_disclosure",
  SUBSTANCE_CRISIS = "substance_crisis"
}

export interface CrisisResource {
  type: "hotline" | "emergency" | "text_line" | "website";
  name: string;
  contact: string;
  availability: string;
  languages: string[];
}

export interface CrisisDetection {
  timestamp: string;
  severity: "low" | "medium" | "high" | "critical";
  type: CrisisType;
  indicators: string[];
  confidence: number;
  recommendedActions: string[];
  resourcesProvided: CrisisResource[];
  followUpRequired: boolean;
  followUpTiming?: string;
}
