/**
 * WIA-EDU-025: Entertainment Robot Standard
 * TypeScript Type Definitions v1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 * © 2025 WIA - World Certification Industry Association
 * MIT License
 */

// ============================================================================
// Robot Profile Types
// ============================================================================

export type RobotType =
  | 'storytelling-robot'
  | 'performance-robot'
  | 'emotional-companion'
  | 'edutainment-host'
  | 'therapeutic-robot'
  | 'creative-partner';

export type AgeGroup = '3-6' | '7-12' | '13-18' | '18+';

export type GenreType =
  | 'fantasy'
  | 'adventure'
  | 'educational'
  | 'mystery'
  | 'science-fiction'
  | 'historical'
  | 'comedy';

export type PerformanceType = 'theater' | 'dance' | 'music' | 'puppet-show' | 'magic-show';

export interface RobotProfile {
  '@context': string;
  '@type': 'EntertainmentRobotProfile';
  robotId: string;
  robotType: RobotType;
  manufacturer: {
    name: string;
    did: string;
    certification: string;
  };
  capabilities: {
    storytelling: boolean;
    performance: boolean;
    emotionalIntelligence: boolean;
    therapeutic: boolean;
    multiRobotCoordination: boolean;
  };
  expressiveness: {
    voiceRange: string[];
    emotionalDisplay: string[];
    movementCapabilities: string[];
  };
  safetyCompliance: {
    certifications: string[];
    childSafe: boolean;
    dataPrivacy: string[];
    supervisionRequired: boolean;
  };
  version: string;
  timestamp: string;
}

// ============================================================================
// Interactive Story Types
// ============================================================================

export type EmotionalTone =
  | 'mysterious'
  | 'exciting'
  | 'calm'
  | 'suspenseful'
  | 'joyful'
  | 'thoughtful'
  | 'dramatic';

export interface InteractiveStory {
  '@context': string;
  '@type': 'InteractiveStory';
  storyId: string;
  title: string;
  genre: GenreType;
  targetAgeRange: AgeGroup;
  language: string;
  duration: number;
  narrator: {
    robotId?: string;
    voice: string;
    expressiveness: 'low' | 'medium' | 'high';
  };
  chapters: Chapter[];
  characters: Character[];
  learningObjectives: LearningObjective[];
  timestamp: string;
}

export interface Chapter {
  chapterId: string;
  title: string;
  duration: number;
  narrative: {
    beats: NarrativeBeat[];
  };
  choicePoints: ChoicePoint[];
  interactionPoints: number;
}

export interface NarrativeBeat {
  beatId: string;
  type: 'exposition' | 'rising-action' | 'climax' | 'falling-action' | 'resolution';
  text: string;
  emotionalTone: EmotionalTone;
  characterId: string;
  duration?: number;
}

export interface ChoicePoint {
  choiceId: string;
  prompt: string;
  options: ChoiceOption[];
  timeLimit?: number;
  defaultOption?: string;
}

export interface ChoiceOption {
  optionId: string;
  text: string;
  consequence: string;
  learningObjective?: string;
  emotionalImpact?: string;
}

export interface Character {
  characterId: string;
  name: string;
  traits: string[];
  emotionalRange: string[];
  voiceProfile: string;
  relationships?: Record<string, string>;
}

export interface LearningObjective {
  objectiveId: string;
  standard?: string;
  description: string;
  assessmentPoints: string[];
  bloomLevel?: string;
}

// ============================================================================
// Performance Types
// ============================================================================

export interface PerformanceProgram {
  '@context': string;
  '@type': 'PerformanceProgram';
  performanceId: string;
  title: string;
  type: PerformanceType;
  duration: number;
  educationalFocus?: string;
  acts: Act[];
  requiresMultipleRobots?: boolean;
  audienceInteraction: boolean;
  timestamp: string;
}

export interface Act {
  actNumber: number;
  title: string;
  duration: number;
  scenes: Scene[];
}

export interface Scene {
  sceneId: string;
  duration: number;
  actions: PerformanceAction[];
  interactionPoints?: InteractionPoint[];
}

export interface PerformanceAction {
  timestamp: number;
  type: 'speech' | 'movement' | 'music' | 'lighting' | 'special-effect';
  robotId?: string;
  data: any;
  emotion?: string;
}

export interface InteractionPoint {
  timestamp: number;
  type: 'audience-poll' | 'volunteer-request' | 'q-and-a' | 'call-response';
  question?: string;
  data?: any;
}

// ============================================================================
// Emotional Intelligence Types
// ============================================================================

export type PrimaryEmotion =
  | 'happy'
  | 'sad'
  | 'angry'
  | 'fearful'
  | 'surprised'
  | 'disgusted';

export type SecondaryEmotion =
  | 'excited'
  | 'calm'
  | 'frustrated'
  | 'confused'
  | 'proud'
  | 'shy'
  | 'anxious'
  | 'content'
  | 'bored';

export type EmotionSource = 'facial-expression' | 'voice-tone' | 'body-language' | 'conversation';

export interface EmotionalState {
  '@type': 'EmotionalState';
  timestamp: string;
  detectedEmotions: DetectedEmotion[];
  engagementLevel: number;
  attentionScore: number;
  overallSentiment: 'positive' | 'neutral' | 'negative';
}

export interface DetectedEmotion {
  emotion: PrimaryEmotion | SecondaryEmotion;
  confidence: number;
  source: EmotionSource;
}

export interface EmotionalResponse {
  '@type': 'EmotionalResponse';
  responseId: string;
  detectedEmotion: string;
  responseStrategy: 'encourage' | 'comfort' | 'challenge' | 'calm' | 'celebrate';
  robotResponse: {
    expression: string;
    tone: string;
    message: string;
    suggestedActivity?: string;
  };
  timestamp: string;
}

// ============================================================================
// Therapeutic Session Types
// ============================================================================

export type TherapeuticFocus =
  | 'autism-social-skills'
  | 'anxiety-management'
  | 'adhd-support'
  | 'emotional-regulation'
  | 'speech-therapy';

export type SessionType = 'structured-activity' | 'free-play' | 'guided-meditation' | 'conversation-practice';

export interface TherapeuticSession {
  '@context': string;
  '@type': 'TherapeuticSession';
  sessionId: string;
  therapeuticFocus: TherapeuticFocus;
  sessionType: SessionType;
  duration: number;
  protocol: {
    protocolId: string;
    evidenceBase: string;
    supervisionRequired: boolean;
  };
  phases: TherapyPhase[];
  progressTracking: {
    metrics: string[];
    dataRetention: string;
    reporting: string;
  };
  safetyProtocols: {
    guardianMonitoring: boolean;
    professionalSupervision: boolean;
    emergencyStop: boolean;
    privacyCompliance: string[];
  };
  timestamp: string;
}

export interface TherapyPhase {
  phase: 'warmup' | 'main-activity' | 'cooldown';
  duration: number;
  activities: TherapyActivity[];
}

export interface TherapyActivity {
  activityId: string;
  type: string;
  objective: string;
  duration?: number;
  adaptations?: string[];
}

// ============================================================================
// Edutainment Game Types
// ============================================================================

export type GameSubject = 'mathematics' | 'language-arts' | 'science' | 'social-skills' | 'creative-arts';

export type DifficultyLevel = 'easy' | 'medium' | 'hard' | 'adaptive';

export type GameFormat = 'interactive-quiz' | 'puzzle-challenge' | 'story-adventure' | 'creative-workshop';

export interface EdutainmentGame {
  '@context': string;
  '@type': 'EdutainmentGame';
  gameId: string;
  subject: GameSubject;
  difficulty: DifficultyLevel;
  format: GameFormat;
  gameFlow: {
    introduction: string;
    tutorial: string;
    gameplay: string;
    conclusion: string;
  };
  gamification: {
    points: boolean;
    levels: boolean;
    badges: boolean;
    leaderboard: boolean;
    rewards: string[];
  };
  learningMechanics: {
    scaffolding: boolean;
    immediateFeedback: boolean;
    adaptiveDifficulty: boolean;
    multimodalInput: string[];
  };
  timestamp: string;
}

// ============================================================================
// Session & Progress Types
// ============================================================================

export interface Session {
  '@type': 'EntertainmentSession';
  sessionId: string;
  userId: string;
  robotId: string;
  sessionType: 'story' | 'performance' | 'therapeutic' | 'game';
  startTime: string;
  endTime?: string;
  duration?: number;
  emotionalJourney: EmotionalState[];
  interactions: number;
  learningOutcomes: LearningOutcome[];
}

export interface LearningOutcome {
  objectiveId: string;
  skill: string;
  masteryLevel: 'novice' | 'beginner' | 'intermediate' | 'advanced' | 'expert';
  evidencePoints: string[];
  timestamp: string;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T> {
  data?: T;
  error?: APIError;
  pagination?: PaginationInfo;
}

export interface APIError {
  code: string;
  message: string;
  details?: string;
  timestamp: string;
  requestId?: string;
}

export interface PaginationInfo {
  page: number;
  limit: number;
  totalPages: number;
  totalItems: number;
  hasNext: boolean;
  hasPrevious: boolean;
  nextPage?: string;
  previousPage?: string;
}

// ============================================================================
// Verifiable Credential Types
// ============================================================================

export interface EntertainmentCredential {
  '@context': string[];
  type: string[];
  issuer: {
    id: string;
    name: string;
    certifiedBy: string;
  };
  issuanceDate: string;
  expirationDate?: string;
  credentialSubject: {
    id: string;
    achievement: {
      type: string;
      skill: string;
      level: string;
      description: string;
      dateEarned: string;
      context: 'storytelling' | 'performance' | 'therapeutic' | 'creative';
    };
    evidence: CredentialEvidence[];
  };
  proof: {
    type: string;
    created: string;
    proofPurpose: string;
    verificationMethod: string;
    proofValue: string;
  };
}

export interface CredentialEvidence {
  type: string;
  sessionId: string;
  timestamp: string;
  description: string;
}

// ============================================================================
// SDK Configuration Types
// ============================================================================

export interface WIAEntertainmentRobotConfig {
  apiKey: string;
  robotId?: string;
  baseURL?: string;
  environment?: 'production' | 'sandbox';
  timeout?: number;
  retryAttempts?: number;
  privacyMode?: 'strict' | 'standard';
}

export interface WebSocketConfig {
  url: string;
  reconnect?: boolean;
  maxReconnectAttempts?: number;
  reconnectInterval?: number;
}

// ============================================================================
// Privacy & Consent Types
// ============================================================================

export interface ConsentRequest {
  guardianEmail: string;
  childAge: number;
  dataTypes: ('emotion' | 'interaction' | 'learning-progress' | 'therapeutic')[];
  retentionPeriod: string;
  purposes: string[];
}

export interface PrivacySettings {
  emotionProcessing: 'local' | 'cloud';
  dataRetention: number;
  parentalAccess: boolean;
  thirdPartySharing: boolean;
  anonymization: boolean;
}

// ============================================================================
// Utility Types
// ============================================================================

export type Omit<T, K extends keyof T> = Pick<T, Exclude<keyof T, K>>;
export type Partial<T> = { [P in keyof T]?: T[P] };
export type Required<T> = { [P in keyof T]-?: T[P] };
