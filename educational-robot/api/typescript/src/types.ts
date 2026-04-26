/**
 * WIA-EDU-007: Educational Robot Standard
 * TypeScript Type Definitions v2.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 * © 2025 WIA - World Certification Industry Association
 * MIT License
 */

// ============================================================================
// Robot Profile Types
// ============================================================================

export type RobotType =
  | 'teaching-assistant'
  | 'stem-education'
  | 'social-companion'
  | 'programming-tutor'
  | 'language-learning';

export type AgeGroup = '3-6' | '7-12' | '13-18' | '18+';

export type SubjectArea =
  | 'mathematics'
  | 'science'
  | 'programming'
  | 'language'
  | 'robotics'
  | 'social-skills';

export type InteractionMode = 'voice' | 'touch' | 'gesture' | 'visual' | 'multimodal';

export interface RobotProfile {
  '@context': string;
  '@type': 'EducationalRobotProfile';
  robotId: string;
  robotType: RobotType;
  manufacturer: {
    name: string;
    did: string;
    certification: string;
  };
  specifications: {
    ageGroup: AgeGroup;
    subjectAreas: SubjectArea[];
    interactionModes: InteractionMode[];
    languages: string[];
  };
  capabilities: {
    teaching: string[];
    interaction: string[];
    assessment: string[];
    adaptation: string[];
  };
  safetyCompliance: {
    certifications: string[];
    childSafe: boolean;
    dataPrivacy: string;
    supervisionRequired: boolean;
  };
  version: string;
  timestamp: string;
}

// ============================================================================
// Learning Session Types
// ============================================================================

export type SessionType = 'lecture' | 'practice' | 'quiz' | 'project' | 'social' | 'remedial';

export type DifficultyLevel = 'novice' | 'beginner' | 'intermediate' | 'advanced' | 'expert';

export interface LearningSession {
  '@context': string;
  '@type': 'LearningSession';
  sessionId: string;
  robotId: string;
  studentId: string;
  sessionType: SessionType;
  startTime: string;
  endTime?: string;
  duration?: number;
  curriculum: {
    subjectArea: SubjectArea;
    lessonId: string;
    learningObjectives: string[];
    difficulty: DifficultyLevel;
  };
  interaction: {
    engagementScore: number;
    emotionalState: string[];
    questionsAsked: number;
    responsiveness: number;
  };
  assessment: {
    preTestScore: number;
    postTestScore: number;
    improvementRate: number;
    masteryLevel: DifficultyLevel;
  };
  adaptations: {
    difficultyAdjustments: object[];
    paceModifications: object[];
    modalityShifts: object[];
  };
  timestamp: string;
}

// ============================================================================
// Student Progress Types
// ============================================================================

export type AchievementType =
  | 'course-completion'
  | 'skill-mastery'
  | 'project-completion'
  | 'certification';

export type AchievementLevel = 'bronze' | 'silver' | 'gold' | 'platinum';

export interface StudentProgress {
  '@context': string;
  '@type': 'StudentProgressRecord';
  studentId: string;
  robotId: string;
  subjectArea: SubjectArea;
  progressPeriod: {
    startDate: string;
    endDate: string;
  };
  metrics: {
    sessionsCompleted: number;
    totalLearningTime: number;
    averageEngagement: number;
    skillsMastered: string[];
    challengeAreas: string[];
  };
  achievements: Achievement[];
  adaptivePath: {
    currentLevel: DifficultyLevel;
    recommendedNext: string[];
    learningStyle: string;
    pacePreference: string;
  };
  timestamp: string;
}

export interface Achievement {
  achievementId: string;
  type: AchievementType;
  dateEarned: string;
  level: AchievementLevel;
}

// ============================================================================
// Real-Time Communication Types
// ============================================================================

export type MessageType =
  | 'instruction'
  | 'question'
  | 'feedback'
  | 'response'
  | 'emotion'
  | 'gesture'
  | 'event'
  | 'heartbeat'
  | 'error';

export interface WebSocketMessage {
  '@context': string;
  '@type': 'RobotMessage' | 'StudentMessage' | 'SystemMessage';
  messageId: string;
  timestamp: string;
  sessionId: string;
  robotId?: string;
  messageType: MessageType;
  payload: any;
  metadata?: any;
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

export interface VerifiableCredential {
  '@context': string[];
  type: string[];
  issuer: {
    id: string;
    name: string;
    certifiedBy: string;
    certificationId: string;
  };
  issuanceDate: string;
  expirationDate?: string;
  credentialSubject: {
    id: string;
    achievement: {
      achievementType: AchievementType;
      subject: SubjectArea;
      skill: string;
      level: AchievementLevel;
      description: string;
      dateEarned: string;
      assessmentScore: number;
      bloomLevel: string;
      metadata: any;
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
  assessmentId: string;
  score: number;
  timestamp: string;
}

// ============================================================================
// SDK Configuration Types
// ============================================================================

export interface WIAEduRobotConfig {
  apiKey: string;
  robotId?: string;
  baseURL?: string;
  timeout?: number;
  retryAttempts?: number;
}

export interface WebSocketConfig {
  url: string;
  reconnect?: boolean;
  maxReconnectAttempts?: number;
  reconnectInterval?: number;
}

// ============================================================================
// Utility Types
// ============================================================================

export type Omit<T, K extends keyof T> = Pick<T, Exclude<keyof T, K>>;
export type Partial<T> = { [P in keyof T]?: T[P] };
export type Required<T> = { [P in keyof T]-?: T[P] };
