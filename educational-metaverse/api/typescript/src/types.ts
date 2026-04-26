/**
 * WIA-EDU-015: Educational Metaverse TypeScript Type Definitions
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Transform {
  position: Vector3;
  rotation: Quaternion;
  scale: Vector3;
}

// ============================================================================
// Campus Types
// ============================================================================

export interface CampusConfig {
  name: string;
  template?: CampusTemplate;
  maxConcurrentUsers?: number;
  worldSize?: Vector3;
  physicsEnabled?: boolean;
  spatialAudio?: boolean;
  vrSupport?: boolean;
  arSupport?: boolean;
  environments?: EnvironmentConfig[];
  customization?: CampusCustomization;
}

export type CampusTemplate =
  | 'modern-university'
  | 'classical-college'
  | 'high-school'
  | 'elementary-school'
  | 'corporate-campus'
  | 'custom';

export interface CampusCustomization {
  skybox?: string;
  weather?: boolean;
  seasons?: boolean;
  landmarks?: string[];
}

export interface EnvironmentConfig {
  type: EnvironmentType;
  id?: string;
  name: string;
  capacity: number;
  position: Vector3;
  features: string[];
}

export type EnvironmentType =
  | 'classroom'
  | 'library'
  | 'lab'
  | 'auditorium'
  | 'social-space'
  | 'outdoor';

export interface Campus {
  campusId: string;
  name: string;
  url: string;
  status: CampusStatus;
  createdAt: Date;
  settings: CampusConfig;
}

export type CampusStatus =
  | 'provisioning'
  | 'ready'
  | 'maintenance'
  | 'error';

// ============================================================================
// Avatar Types
// ============================================================================

export interface AvatarConfig {
  customization?: AvatarCustomization;
  expressions?: ExpressionConfig;
  movement?: MovementConfig;
}

export interface AvatarCustomization {
  bodyTypes?: BodyType[];
  accessories?: boolean;
  clothing?: boolean;
  animations?: string[];
}

export type BodyType = 'realistic' | 'stylized' | 'fantasy';

export interface ExpressionConfig {
  faceTracking?: boolean;
  emotionRecognition?: boolean;
  lipSync?: boolean;
}

export interface MovementConfig {
  walking?: boolean;
  running?: boolean;
  flying?: boolean;
  teleportation?: boolean;
}

export interface Avatar {
  avatarId: string;
  userId: string;
  customization: {
    bodyType: string;
    skinTone: string;
    hairStyle: string;
    hairColor: string;
    clothing: ClothingSet;
    accessories: string[];
  };
  animations: AnimationSet;
  capabilities: AvatarCapabilities;
}

export interface ClothingSet {
  top: string;
  bottom: string;
  shoes: string;
  accessories?: string[];
}

export interface AnimationSet {
  idle: string;
  walk: string;
  gestures: string[];
}

export interface AvatarCapabilities {
  faceTracking: boolean;
  lipSync: boolean;
  handTracking: boolean;
  fullBodyTracking: boolean;
}

// ============================================================================
// Field Trip Types
// ============================================================================

export interface FieldTripConfig {
  type: 'field-trip';
  title: string;
  destination: string;
  duration: number;
  features: FieldTripFeatures;
  participants: ParticipantConfig;
}

export interface FieldTripFeatures {
  guidedTour?: boolean;
  aiNarrator?: NarratorConfig;
  interactiveElements?: boolean;
  quizzes?: boolean;
  photoMode?: boolean;
}

export interface NarratorConfig {
  character: string;
  voice: string;
  language: string;
}

export interface ParticipantConfig {
  min: number;
  max: number;
  roleAssignment?: boolean;
}

export interface FieldTrip {
  tripId: string;
  title: string;
  destination: string;
  duration: number;
  status: 'scheduled' | 'active' | 'completed';
  participants: string[];
  createdAt: Date;
}

// ============================================================================
// Lab Types
// ============================================================================

export interface LabConfig {
  subject: string;
  type: LabType;
  equipment?: EquipmentItem[];
  experiments?: ExperimentConfig[];
  safety?: SafetyConfig;
}

export type LabType =
  | 'chemistry'
  | 'physics'
  | 'biology'
  | 'engineering'
  | 'computer-science';

export interface EquipmentItem {
  itemId: string;
  model: string;
  interactive: boolean;
  physics?: boolean;
}

export interface ExperimentConfig {
  id: string;
  name: string;
  safetyLevel: 'low' | 'medium' | 'high';
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
}

export interface SafetyConfig {
  virtualProtectiveGear?: boolean;
  instructorSupervision?: boolean;
  emergencyReset?: boolean;
}

// ============================================================================
// Social & Collaboration Types
// ============================================================================

export interface SocialFeatures {
  voiceChat: VoiceChatConfig;
  gestures: GestureConfig;
  collaboration: CollaborationConfig;
  moderation: ModerationConfig;
}

export interface VoiceChatConfig {
  enabled: boolean;
  spatialAudio: boolean;
  proximityBased: boolean;
  whisperMode?: boolean;
}

export interface GestureConfig {
  handRaise?: boolean;
  applause?: boolean;
  customEmotes?: boolean;
}

export interface CollaborationConfig {
  sharedWorkspaces?: boolean;
  groupProjects?: boolean;
  peerReview?: boolean;
}

export interface ModerationConfig {
  aiModeration?: boolean;
  reportingSystem?: boolean;
  safeZones?: boolean;
  ageAppropriate?: boolean;
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface AnalyticsConfig {
  period: string;
  metrics: AnalyticsMetric[];
}

export type AnalyticsMetric =
  | 'active-users'
  | 'engagement-time'
  | 'popular-locations'
  | 'social-interactions'
  | 'learning-progress'
  | 'completion-rates';

export interface AnalyticsData {
  campusId: string;
  period: string;
  metrics: {
    activeUsers: number;
    averageEngagement: number;
    popularLocations: LocationMetric[];
    socialInteractions: number;
    completionRates: number;
  };
}

export interface LocationMetric {
  name: string;
  visits: number;
}

// ============================================================================
// Quest & Gamification Types
// ============================================================================

export interface QuestConfig {
  title: string;
  type: 'educational-game';
  objectives: QuestObjective[];
  rewards: RewardSet;
  duration?: number;
}

export interface QuestObjective {
  task: string;
  points: number;
}

export interface RewardSet {
  badges?: string[];
  avatarItems?: string[];
  leaderboard?: boolean;
}

export interface Quest {
  questId: string;
  title: string;
  type: string;
  status: 'active' | 'completed' | 'expired';
  progress: number;
  objectives: QuestObjective[];
  rewards: RewardSet;
}

// ============================================================================
// Learning Data Types
// ============================================================================

export interface LearningDataConfig {
  format: 'xAPI' | 'SCORM' | 'JSON';
  period: string;
  includeAnalytics?: boolean;
}

export interface LearningData {
  userId: string;
  activities: LearningActivity[];
  achievements: Achievement[];
  progress: ProgressData;
}

export interface LearningActivity {
  activityId: string;
  type: string;
  timestamp: Date;
  duration: number;
  completed: boolean;
  score?: number;
}

export interface Achievement {
  achievementId: string;
  title: string;
  description: string;
  unlockedAt: Date;
  icon: string;
}

export interface ProgressData {
  level: number;
  experience: number;
  completionRate: number;
  strengths: string[];
  areasForImprovement: string[];
}

// ============================================================================
// Event Types
// ============================================================================

export interface MetaverseEvent {
  eventType: string;
  timestamp: Date;
  data: any;
}

export type UserEvent =
  | 'user:joined'
  | 'user:left'
  | 'user:moved'
  | 'interaction:started'
  | 'interaction:completed'
  | 'achievement:unlocked';

// ============================================================================
// Configuration Types
// ============================================================================

export interface MetaverseConfig {
  apiKey: string;
  region?: string;
  renderQuality?: 'low' | 'medium' | 'high' | 'ultra';
  debug?: boolean;
}

export interface SessionConfig {
  campusId: string;
  userId: string;
  avatarId?: string;
  spawnPoint?: Vector3;
}
