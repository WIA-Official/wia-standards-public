/**
 * WIA Cognitive AAC - Prediction Types
 * 예측/학습 시스템 타입 정의
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

// ============================================================================
// Usage Pattern Types
// ============================================================================

export interface SymbolFrequency {
  symbolId: string;
  count: number;
  lastUsed: number;
  avgResponseTime: number;
}

export interface Phrase {
  id: string;
  symbols: string[];
  frequency: number;
  lastUsed: number;
  context?: string;
}

export interface SymbolChain {
  symbols: string[];
  frequency: number;
  avgInterval: number;
  probability: number;
}

export interface ConversationPattern {
  id: string;
  initiator: 'user' | 'partner';
  exchangeSequence: string[];
  frequency: number;
  typicalContext?: string;
}

export interface UsagePattern {
  temporal: {
    hourlyFrequency: Map<number, SymbolFrequency[]>;
    dayOfWeek: Map<number, SymbolFrequency[]>;
    contextual: Map<string, SymbolFrequency[]>;
  };
  sequences: {
    commonPhrases: Phrase[];
    symbolChains: SymbolChain[];
    conversationPatterns: ConversationPattern[];
  };
  contextual: {
    locationBased: Map<string, SymbolFrequency[]>;
    personBased: Map<string, SymbolFrequency[]>;
    activityBased: Map<string, SymbolFrequency[]>;
  };
}

// ============================================================================
// Context Types
// ============================================================================

export interface Context {
  time: Date;
  location?: string;
  conversationPartner?: string;
  recentActivity?: string;
  mood?: string;
  environmentalCues?: string[];
  timeOfDay?: 'morning' | 'afternoon' | 'evening' | 'night';
  dayType?: 'weekday' | 'weekend';
}

export interface ContextSignal {
  type: 'time' | 'location' | 'person' | 'activity' | 'environment';
  value: string;
  confidence: number;
  timestamp: number;
}

// ============================================================================
// Prediction Types
// ============================================================================

export type PredictionSource = 'frequency' | 'sequence' | 'context' | 'time' | 'routine' | 'reminiscence';

export interface PredictedSymbol {
  symbolId: string;
  probability: number;
  source: PredictionSource;
  explanation?: string;
  rank: number;
}

export interface PredictionResult {
  predictions: PredictedSymbol[];
  context: Context;
  timestamp: number;
  modelVersion: string;
}

export interface RecommendedBoard {
  boardId: string;
  symbols: PredictedSymbol[];
  reason: string;
  confidence: number;
}

// ============================================================================
// Autism-specific Prediction Types
// ============================================================================

export interface RoutineStep {
  id: string;
  activity: string;
  typicalTime?: { hour: number; minute: number };
  duration?: number;
  requiredSymbols: string[];
  transitionCues?: string[];
}

export interface Activity {
  id: string;
  name: string;
  category: string;
  associatedSymbols: string[];
}

export interface TransitionAid {
  currentActivity: Activity;
  nextActivity: Activity;
  warningSymbols: string[];
  countdownEnabled: boolean;
  visualScheduleSymbols: string[];
}

export interface RoutineDeviation {
  expectedStep: RoutineStep;
  actualBehavior: string;
  severity: 'minor' | 'moderate' | 'major';
  timestamp: number;
}

export interface RoutineBasedPrediction {
  dailyRoutine: RoutineStep[];
  currentStep: RoutineStep | null;
  nextStep: RoutineStep | null;
  deviationHistory: RoutineDeviation[];
}

// ============================================================================
// Dementia-specific Prediction Types
// ============================================================================

export interface Photo {
  id: string;
  url: string;
  description: string;
  people?: string[];
  location?: string;
  era?: string;
  significance?: string;
}

export interface Song {
  id: string;
  title: string;
  artist?: string;
  era?: string;
  significance?: string;
  associatedMemories?: string[];
}

export interface TemporalInfo {
  currentTime: Date;
  dayOfWeek: string;
  timeOfDay: string;
  season?: string;
  upcomingEvents?: string[];
  displayFormat: 'simple' | 'detailed';
}

export interface LocationInfo {
  currentLocation: string;
  familiarName?: string;
  relatedPeople?: string[];
  orientationCues: string[];
}

export interface Person {
  id: string;
  name: string;
  relationship: string;
  photoUrl?: string;
  recognitionHints?: string[];
  lastInteraction?: Date;
}

export interface RepetitiveQuestionResponse {
  questionPattern: string;
  gentleResponse: string;
  reassuranceMessage: string;
  redirectSymbols?: string[];
}

export interface ReminiscenceBasedPrediction {
  longTermMemoryTriggers: {
    familiarPhotos: Photo[];
    significantDates: { date: Date; description: string }[];
    musicMemories: Song[];
  };
  currentOrientationSupport: {
    temporal: TemporalInfo;
    location: LocationInfo | null;
    recognizedPeople: Person[];
  };
}

// ============================================================================
// Privacy Types
// ============================================================================

export type AnonymizationLevel = 'full' | 'partial' | 'none';
export type StorageLocation = 'local_only' | 'encrypted_cloud' | 'federated';

export interface PrivacySettings {
  localLearning: {
    enabled: boolean;
    modelStorage: StorageLocation;
    noCloudUpload: boolean;
  };
  federatedLearning: {
    enabled: boolean;
    consentGiven: boolean;
    anonymization: AnonymizationLevel;
    aggregationOnly: boolean;
  };
  retention: {
    maxDays: number;
    autoDelete: boolean;
    exportFormat: 'encrypted_backup' | 'plain_json';
  };
  dataMinimization: {
    excludePersonalInfo: boolean;
    excludeLocationData: boolean;
    excludeConversationContent: boolean;
  };
}

export interface DataRetentionPolicy {
  maxAgeDays: number;
  maxRecords: number;
  autoCleanup: boolean;
  cleanupInterval: number;
}

export interface ConsentRecord {
  timestamp: number;
  type: 'data_collection' | 'federated_learning' | 'analytics';
  granted: boolean;
  grantor: 'user' | 'caregiver' | 'guardian';
  expiresAt?: number;
}

// ============================================================================
// Learning Types
// ============================================================================

export interface LearningEvent {
  type: 'symbol_selection' | 'phrase_completion' | 'navigation' | 'context_change';
  timestamp: number;
  data: Record<string, unknown>;
  context: Context;
}

export interface ModelUpdate {
  modelId: string;
  version: string;
  timestamp: number;
  changes: {
    frequencyUpdates: number;
    sequenceUpdates: number;
    contextUpdates: number;
  };
}

export interface PatternLearnerConfig {
  minFrequencyThreshold: number;
  sequenceMaxLength: number;
  decayFactor: number;
  contextWeight: number;
  recencyWeight: number;
}

// ============================================================================
// Engine Configuration Types
// ============================================================================

export interface PredictionEngineConfig {
  maxPredictions: number;
  minConfidence: number;
  enableContextPrediction: boolean;
  enableSequencePrediction: boolean;
  enableTimePrediction: boolean;
  profileType: 'general' | 'autism' | 'dementia';
  privacySettings: PrivacySettings;
  learnerConfig: PatternLearnerConfig;
}

export interface PredictionEngineState {
  isInitialized: boolean;
  isLearning: boolean;
  lastUpdate: number;
  totalPredictions: number;
  accuracy: number;
}

// ============================================================================
// Event Handlers
// ============================================================================

export type PredictionEventHandler = (result: PredictionResult) => void;
export type LearningEventHandler = (event: LearningEvent) => void;
export type ContextChangeHandler = (context: Context) => void;

export interface PredictionEngineEvents {
  onPrediction?: PredictionEventHandler;
  onLearning?: LearningEventHandler;
  onContextChange?: ContextChangeHandler;
  onError?: (error: Error) => void;
}
