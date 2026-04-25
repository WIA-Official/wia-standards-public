/**
 * WIA-CHILD-001: Online Safety Standard - TypeScript Type Definitions
 * 弘益人間 - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / WIA
 */

// ============================================================================
// Core Types
// ============================================================================

export type ThreatLevel = 'safe' | 'low' | 'medium' | 'high' | 'critical';
export type ContentType = 'text' | 'image' | 'video' | 'url' | 'audio' | 'document';
export type ActionType = 'allow' | 'warn' | 'block' | 'report' | 'escalate';
export type AgeRange = 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17;
export type SensitivityLevel = 'low' | 'medium' | 'high' | 'maximum';
export type AlertChannel = 'email' | 'sms' | 'push' | 'call' | 'dashboard';

// ============================================================================
// Configuration
// ============================================================================

export interface ChildSafetyConfig {
  apiKey: string;
  environment?: 'production' | 'staging' | 'development';
  childProfile: ChildProfile;
  realTimeMonitoring?: boolean;
  parentalAlerts?: boolean;
  privacyMode?: 'strict' | 'balanced' | 'minimal';
  customRules?: SafetyRule[];
  platformIntegrations?: PlatformIntegration[];
}

export interface ChildProfile {
  id?: string;
  age: AgeRange;
  sensitivityLevel: SensitivityLevel;
  allowedCategories: ContentCategory[];
  blockedCategories?: ContentCategory[];
  approvedContacts?: string[];
  screenTimeLimit?: number; // minutes per day
  bedtimeRestriction?: TimeRestriction;
  customSettings?: Record<string, any>;
}

export interface TimeRestriction {
  start: string; // HH:MM format
  end: string; // HH:MM format
  timezone?: string;
  enabled: boolean;
}

// ============================================================================
// Content Analysis
// ============================================================================

export interface ContentAnalysisRequest {
  content: string | Buffer | URL;
  contentType: ContentType;
  context?: AnalysisContext;
  childAge?: AgeRange;
  timestamp?: number;
}

export interface AnalysisContext {
  platform?: string;
  source?: string;
  conversationHistory?: Message[];
  userProfile?: Partial<ChildProfile>;
  metadata?: Record<string, any>;
}

export interface ContentAnalysisResult {
  id: string;
  threatLevel: ThreatLevel;
  confidence: number; // 0-100
  action: ActionType;
  flaggedItems: FlaggedItem[];
  categories: DetectedCategory[];
  ageAppropriateness: {
    suitable: boolean;
    recommendedAge: AgeRange;
    reasoning: string;
  };
  processingTime: number; // milliseconds
  timestamp: number;
}

export interface FlaggedItem {
  type: 'keyword' | 'pattern' | 'visual' | 'behavioral';
  content: string;
  severity: ThreatLevel;
  description: string;
  location?: ContentLocation;
}

export interface ContentLocation {
  start?: number;
  end?: number;
  coordinates?: { x: number; y: number; width: number; height: number };
  timestamp?: number; // for video
}

export interface DetectedCategory {
  name: ContentCategory;
  confidence: number;
  subcategories?: string[];
}

export type ContentCategory =
  | 'education'
  | 'entertainment-kids'
  | 'entertainment-general'
  | 'social'
  | 'gaming'
  | 'news'
  | 'sports'
  | 'adult'
  | 'violence'
  | 'hate-speech'
  | 'self-harm'
  | 'gambling'
  | 'drugs'
  | 'weapons'
  | 'unknown';

// ============================================================================
// Threat Detection
// ============================================================================

export interface ThreatDetectionEngine {
  analyzeBehavior(interactions: Interaction[]): Promise<BehavioralAnalysis>;
  detectGrooming(conversation: Message[]): Promise<GroomingAnalysis>;
  identifyPredator(userProfile: UserProfile): Promise<PredatorRiskScore>;
  scanForCyberbullying(messages: Message[]): Promise<BullyingAnalysis>;
}

export interface Interaction {
  id: string;
  type: 'message' | 'friend-request' | 'comment' | 'share' | 'reaction';
  from: string;
  to: string;
  content?: string;
  timestamp: number;
  platform: string;
}

export interface Message {
  id: string;
  sender: string;
  recipient: string;
  content: string;
  timestamp: number;
  metadata?: Record<string, any>;
}

export interface BehavioralAnalysis {
  riskLevel: ThreatLevel;
  patterns: DetectedPattern[];
  recommendations: string[];
  confidence: number;
}

export interface DetectedPattern {
  type: 'grooming' | 'bullying' | 'predatory' | 'manipulation' | 'pressure';
  indicators: string[];
  severity: ThreatLevel;
  description: string;
  timeline?: TimelineEvent[];
}

export interface TimelineEvent {
  timestamp: number;
  event: string;
  significance: 'low' | 'medium' | 'high';
}

export interface GroomingAnalysis {
  detected: boolean;
  confidence: number;
  stages: GroomingStage[];
  redFlags: string[];
  urgency: 'low' | 'medium' | 'high' | 'critical';
  recommendedAction: ActionType;
}

export type GroomingStage =
  | 'targeting'
  | 'trust-building'
  | 'isolation'
  | 'desensitization'
  | 'exploitation';

export interface PredatorRiskScore {
  score: number; // 0-100
  riskLevel: ThreatLevel;
  indicators: RiskIndicator[];
  historicalData?: HistoricalRecord[];
  recommendation: 'allow' | 'monitor' | 'restrict' | 'block' | 'report';
}

export interface RiskIndicator {
  category: string;
  description: string;
  weight: number;
  evidence: string[];
}

export interface HistoricalRecord {
  date: number;
  incident: string;
  severity: ThreatLevel;
  platform: string;
  verified: boolean;
}

export interface BullyingAnalysis {
  detected: boolean;
  type: 'verbal' | 'social' | 'physical-threats' | 'cyberstalking';
  severity: ThreatLevel;
  participants: {
    bullies: string[];
    victim: string;
    bystanders: string[];
  };
  evidence: Message[];
  duration?: { start: number; end?: number };
}

// ============================================================================
// Monitoring & Alerts
// ============================================================================

export interface MonitoringSession {
  id: string;
  childProfile: string;
  startTime: number;
  endTime?: number;
  platform: string;
  activities: Activity[];
  alerts: Alert[];
  summary?: SessionSummary;
}

export interface Activity {
  id: string;
  timestamp: number;
  type: 'page-visit' | 'search' | 'message' | 'upload' | 'download' | 'interaction';
  description: string;
  threat: ThreatLevel;
  blocked: boolean;
  metadata?: Record<string, any>;
}

export interface Alert {
  id: string;
  level: 'info' | 'warning' | 'critical' | 'emergency';
  type: AlertType;
  message: string;
  timestamp: number;
  childProfile: string;
  context: AlertContext;
  status: 'pending' | 'acknowledged' | 'resolved' | 'escalated';
  actions: AlertAction[];
}

export type AlertType =
  | 'inappropriate-content'
  | 'stranger-contact'
  | 'grooming-detected'
  | 'cyberbullying'
  | 'privacy-risk'
  | 'screen-time-exceeded'
  | 'suspicious-activity'
  | 'policy-violation';

export interface AlertContext {
  platform?: string;
  location?: string;
  involvedParties?: string[];
  evidence?: Evidence[];
  relatedAlerts?: string[];
}

export interface Evidence {
  type: 'screenshot' | 'message' | 'video' | 'metadata' | 'log';
  data: string | Buffer;
  timestamp: number;
  description?: string;
}

export interface AlertAction {
  type: 'block' | 'notify-parent' | 'notify-authority' | 'log' | 'restrict';
  timestamp: number;
  success: boolean;
  details?: string;
}

export interface SessionSummary {
  duration: number; // minutes
  totalActivities: number;
  blockedContent: number;
  alerts: number;
  safetyScore: number; // 0-100
  recommendations: string[];
}

// ============================================================================
// Parental Controls
// ============================================================================

export interface ParentalControlSettings {
  screenTime: ScreenTimeSettings;
  contentFiltering: ContentFilteringSettings;
  socialRestrictions: SocialRestrictions;
  locationTracking: LocationSettings;
  notifications: NotificationSettings;
  emergencyContacts: EmergencyContact[];
}

export interface ScreenTimeSettings {
  enabled: boolean;
  dailyLimit: number; // minutes
  weeklyLimit?: number;
  bedtime: TimeRestriction;
  breakReminders?: boolean;
  gracePeriod?: number; // minutes
}

export interface ContentFilteringSettings {
  mode: SensitivityLevel;
  allowedCategories: ContentCategory[];
  blockedKeywords: string[];
  whitelistedUrls: string[];
  blacklistedUrls: string[];
  safeSearch: boolean;
  ageOverride?: AgeRange;
}

export interface SocialRestrictions {
  allowMessaging: boolean;
  approvedContactsOnly: boolean;
  blockStrangers: boolean;
  requireApproval: boolean;
  visibilitySettings: 'public' | 'friends' | 'private';
  locationSharing: boolean;
  photoSharing: boolean;
}

export interface LocationSettings {
  enabled: boolean;
  trackingFrequency: number; // minutes
  geofencing?: Geofence[];
  notifications: boolean;
}

export interface Geofence {
  id: string;
  name: string;
  center: { latitude: number; longitude: number };
  radius: number; // meters
  alertOnExit: boolean;
  alertOnEnter: boolean;
}

export interface NotificationSettings {
  channels: AlertChannel[];
  criticalOnly: boolean;
  quietHours?: TimeRestriction;
  emailAddress?: string;
  phoneNumber?: string;
  pushTokens?: string[];
}

export interface EmergencyContact {
  name: string;
  relationship: string;
  phone: string;
  email?: string;
  priority: number;
}

// ============================================================================
// Platform Integration
// ============================================================================

export interface PlatformIntegration {
  platform: 'web' | 'mobile' | 'gaming' | 'social' | 'streaming';
  name: string;
  apiVersion: string;
  enabled: boolean;
  capabilities: IntegrationCapability[];
  configuration?: Record<string, any>;
}

export interface IntegrationCapability {
  name: string;
  description: string;
  supported: boolean;
  configuration?: Record<string, any>;
}

export interface UserProfile {
  id: string;
  username?: string;
  displayName?: string;
  age?: number;
  accountCreated?: number;
  verificationStatus?: 'unverified' | 'verified' | 'suspicious' | 'flagged';
  trustScore?: number; // 0-100
  history?: HistoricalRecord[];
}

// ============================================================================
// Safety Rules
// ============================================================================

export interface SafetyRule {
  id: string;
  name: string;
  description: string;
  enabled: boolean;
  priority: number;
  conditions: RuleCondition[];
  actions: RuleAction[];
  schedule?: TimeRestriction;
}

export interface RuleCondition {
  type: 'content' | 'time' | 'location' | 'user' | 'platform' | 'custom';
  operator: 'equals' | 'contains' | 'matches' | 'greater-than' | 'less-than';
  value: any;
  caseSensitive?: boolean;
}

export interface RuleAction {
  type: 'block' | 'allow' | 'alert' | 'log' | 'redirect' | 'restrict';
  parameters?: Record<string, any>;
  message?: string;
}

// ============================================================================
// SDK Events
// ============================================================================

export interface SafetyMonitorEvents {
  content: (analysis: ContentAnalysisResult) => void;
  threat: (threat: ThreatDetectionResult) => void;
  alert: (alert: Alert) => void;
  sessionStart: (session: MonitoringSession) => void;
  sessionEnd: (session: MonitoringSession) => void;
  error: (error: SafetyError) => void;
}

export interface ThreatDetectionResult {
  type: AlertType;
  severity: ThreatLevel;
  details: BehavioralAnalysis | GroomingAnalysis | BullyingAnalysis;
  timestamp: number;
}

export interface SafetyError {
  code: string;
  message: string;
  details?: any;
  timestamp: number;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T = any> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: number;
  requestId: string;
}

export interface PaginatedResponse<T> extends APIResponse<T[]> {
  pagination: {
    page: number;
    pageSize: number;
    total: number;
    hasMore: boolean;
  };
}

// ============================================================================
// Reporting & Analytics
// ============================================================================

export interface SafetyReport {
  id: string;
  period: { start: number; end: number };
  childProfile: string;
  summary: ReportSummary;
  activities: Activity[];
  alerts: Alert[];
  threats: ThreatDetectionResult[];
  recommendations: string[];
  generatedAt: number;
}

export interface ReportSummary {
  totalScreenTime: number; // minutes
  contentBlocked: number;
  alertsTriggered: number;
  threatsDetected: number;
  safetyScore: number; // 0-100
  comparisonToPrevious?: {
    screenTime: number; // percentage change
    blockedContent: number;
    alerts: number;
  };
}

// ============================================================================
// 弘益人間 - Benefit All Humanity
// ============================================================================

/**
 * WIA-CHILD-001: Online Safety Standard
 * Protecting children in the digital age through intelligent, compassionate technology.
 *
 * © 2025 SmileStory Inc. / WIA
 */
