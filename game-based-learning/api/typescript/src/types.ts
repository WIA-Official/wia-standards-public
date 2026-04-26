/**
 * WIA-EDU-014 Game-Based Learning Standard - TypeScript Types
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @standard WIA-EDU-014
 */

// ============================================================================
// Phase 1: Data Format Types
// ============================================================================

export interface GameMetadata {
  id: string;
  standard: 'WIA-EDU-014';
  version: string;
  title: string;
  type: 'educational-game';
  category: 'puzzle' | 'simulation' | 'adventure' | 'strategy' | 'quiz';
  subject: 'mathematics' | 'science' | 'language' | 'history' | 'coding';
  topics: string[];
  gradeLevel: {
    min: number;
    max: number;
  };
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  targetAge: {
    min: number;
    max: number;
  };
  language: string[];
  platform: ('web' | 'ios' | 'android' | 'desktop')[];
  accessibility: AccessibilityFeatures;
  learningObjectives: LearningObjective[];
  metadata: {
    created: string;
    updated: string;
    author: string;
    publisher: string;
    license: string;
  };
}

export interface AccessibilityFeatures {
  wcagLevel: 'A' | 'AA' | 'AAA';
  features: ('screen-reader' | 'high-contrast' | 'subtitles' | 'keyboard-nav')[];
}

export interface LearningObjective {
  id: string;
  description: string;
  standard?: string;
}

export interface PlayerProfile {
  playerId: string;
  username: string;
  demographics: {
    age: number;
    grade: number;
    location: {
      country: string;
      region: string;
    };
  };
  preferences: PlayerPreferences;
  accessibility: PlayerAccessibility;
  learningProfile: LearningProfile;
  privacy: PrivacySettings;
}

export interface PlayerPreferences {
  difficulty: 'adaptive' | 'easy' | 'medium' | 'hard';
  soundEnabled: boolean;
  musicVolume: number;
  notificationsEnabled: boolean;
  language: string;
}

export interface PlayerAccessibility {
  screenReader: boolean;
  highContrast: boolean;
  extendedTime: boolean;
  alternativeInput: boolean;
  fontSize?: 'medium' | 'large' | 'xlarge';
}

export interface LearningProfile {
  style: 'visual' | 'auditory' | 'kinesthetic' | 'reading';
  pace: 'slow' | 'moderate' | 'fast';
  strengths: string[];
  growthAreas: string[];
}

export interface PrivacySettings {
  dataCollection: 'consented' | 'parent-consented' | 'declined';
  consentDate: string;
  shareWithTeachers: boolean;
  shareWithPeers: boolean;
}

export interface ProgressData {
  sessionId: string;
  playerId: string;
  gameId: string;
  startTime: string;
  endTime: string;
  duration: string;
  currentLevel: number;
  totalLevels: number;
  completionPercentage: number;
  performance: PerformanceData;
  skillProgress: SkillProgress[];
}

export interface PerformanceData {
  score: number;
  accuracy: number;
  problemsSolved: number;
  hintsUsed: number;
  attemptsAverage: number;
}

export interface SkillProgress {
  skillId: string;
  mastery: number;
  status: 'mastered' | 'developing' | 'beginning';
  lastPracticed: string;
}

export interface Achievement {
  achievementId: string;
  type: 'completion' | 'mastery' | 'performance' | 'persistence' | 'collaboration' | 'innovation';
  name: string;
  description: string;
  iconUrl: string;
  criteria: AchievementCriteria;
  points: number;
  rarity: 'common' | 'uncommon' | 'rare' | 'legendary';
}

export interface AchievementCriteria {
  skillsMastered?: string[];
  minAccuracy?: number;
  minProblems?: number;
  minPlayTime?: string;
}

export interface AchievementAward {
  awardId: string;
  achievementId: string;
  playerId: string;
  earnedDate: string;
  evidence: Record<string, any>;
  verifiable: boolean;
  credentialUrl?: string;
}

// ============================================================================
// Phase 2: API Interface Types
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
  requestId: string;
}

export interface PaginationInfo {
  total: number;
  limit: number;
  offset: number;
  next?: string;
}

export interface AuthToken {
  access_token: string;
  token_type: 'Bearer';
  expires_in: number;
  refresh_token?: string;
  scope: string;
}

// ============================================================================
// Phase 3: Protocol Types
// ============================================================================

export interface WebSocketMessage {
  type: string;
  timestamp: string;
  [key: string]: any;
}

export interface SessionCreate {
  type: 'session.create';
  gameId: string;
  mode: 'cooperative' | 'competitive' | 'team';
  maxPlayers: number;
  settings: SessionSettings;
}

export interface SessionSettings {
  difficulty: string;
  timeLimit: number;
  chatEnabled: boolean;
  voiceEnabled?: boolean;
}

export interface StateUpdate {
  type: 'state.update';
  sessionId: string;
  timestamp: string;
  sequence: number;
  state: GameState;
}

export interface GameState {
  currentChallenge: string;
  teamScore: number;
  timeRemaining: number;
  playerStates: Record<string, PlayerState>;
}

export interface PlayerState {
  position: string;
  score: number;
  [key: string]: any;
}

// ============================================================================
// Phase 4: Integration Types
// ============================================================================

export interface VerifiableCredential {
  '@context': string[];
  type: string[];
  issuer: {
    id: string;
    name: string;
  };
  issuanceDate: string;
  credentialSubject: {
    id: string;
    achievement: {
      type: string;
      name: string;
      description: string;
      criteria: Record<string, any>;
    };
  };
  proof: {
    type: string;
    created: string;
    verificationMethod: string;
    proofPurpose: string;
    proofValue: string;
  };
}

export interface LTILaunchRequest {
  iss: string;
  aud: string;
  sub: string;
  [key: string]: any;
}

export interface CurriculumStandard {
  framework: string;
  code: string;
  description: string;
}
