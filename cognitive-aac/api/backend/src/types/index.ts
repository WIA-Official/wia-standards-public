/**
 * WIA Cognitive AAC - Backend Types
 * 케어기버/전문가 통합 타입 정의
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

// ============================================================================
// Core Types
// ============================================================================

export type UserRole = 'user' | 'caregiver' | 'professional' | 'family' | 'admin';
export type ClientType = 'autism' | 'dementia' | 'general';
export type AlertSeverity = 'info' | 'warning' | 'critical';
export type MoodLevel = 'very_low' | 'low' | 'neutral' | 'high' | 'very_high';

export interface User {
  id: string;
  email: string;
  name: string;
  role: UserRole;
  createdAt: Date;
  updatedAt: Date;
  settings: UserSettings;
}

export interface UserSettings {
  language: string;
  timezone: string;
  notifications: NotificationPreferences;
}

export interface NotificationPreferences {
  email: boolean;
  push: boolean;
  sms: boolean;
  quietHours?: TimeRange;
}

export interface TimeRange {
  start: string; // HH:mm
  end: string;   // HH:mm
}

// ============================================================================
// Client (AAC User) Types
// ============================================================================

export interface Client {
  id: string;
  name: string;
  type: ClientType;
  dateOfBirth?: Date;
  profileId: string; // 링크 to CognitiveProfile
  caregivers: CaregiverLink[];
  professionals: ProfessionalLink[];
  family: FamilyLink[];
  createdAt: Date;
  updatedAt: Date;
}

export interface CaregiverLink {
  userId: string;
  relationship: string;
  isPrimary: boolean;
  permissions: CaregiverPermissions;
  addedAt: Date;
}

export interface ProfessionalLink {
  userId: string;
  role: 'slp' | 'ot' | 'psychologist' | 'physician' | 'other';
  organization?: string;
  permissions: ProfessionalPermissions;
  addedAt: Date;
}

export interface FamilyLink {
  userId: string;
  relationship: string;
  permissions: FamilyPermissions;
  addedAt: Date;
}

// ============================================================================
// Permission Types
// ============================================================================

export interface CaregiverPermissions {
  viewActivity: boolean;
  viewReports: boolean;
  adjustUI: boolean;
  sendMessages: boolean;
  manageAlerts: boolean;
  lockDevice: boolean;
}

export interface ProfessionalPermissions {
  viewFullHistory: boolean;
  conductAssessment: boolean;
  modifyTreatmentPlan: boolean;
  manageVocabulary: boolean;
  exportData: boolean;
  generateReports: boolean;
}

export interface FamilyPermissions {
  viewActivity: boolean;
  sharePhotos: boolean;
  sendMessages: boolean;
  viewCalendar: boolean;
  addMemories: boolean;
}

// ============================================================================
// Activity & Monitoring Types
// ============================================================================

export interface Activity {
  id: string;
  clientId: string;
  type: 'communication' | 'navigation' | 'selection' | 'session_start' | 'session_end';
  timestamp: Date;
  data: ActivityData;
}

export interface ActivityData {
  symbolId?: string;
  symbolLabel?: string;
  pageId?: string;
  duration?: number;
  responseTime?: number;
  successful?: boolean;
  context?: ActivityContext;
}

export interface ActivityContext {
  location?: string;
  conversationPartner?: string;
  mood?: MoodLevel;
  timeOfDay?: string;
}

export interface Message {
  id: string;
  clientId: string;
  content: string;
  symbols: string[];
  timestamp: Date;
  context?: ActivityContext;
}

export interface Mood {
  level: MoodLevel;
  confidence: number;
  indicators: string[];
  timestamp: Date;
}

export interface Alert {
  id: string;
  clientId: string;
  severity: AlertSeverity;
  type: AlertType;
  message: string;
  timestamp: Date;
  acknowledged: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: Date;
}

export type AlertType =
  | 'inactivity'
  | 'distress'
  | 'low_engagement'
  | 'high_error_rate'
  | 'routine_deviation'
  | 'emergency'
  | 'system';

export interface AlertThreshold {
  type: AlertType;
  enabled: boolean;
  threshold: number;
  cooldownMinutes: number;
}

// ============================================================================
// Report Types
// ============================================================================

export interface DailySummary {
  date: Date;
  clientId: string;
  totalCommunications: number;
  uniqueSymbolsUsed: number;
  avgResponseTime: number;
  moodTrend: MoodTrend;
  peakUsageTime: string;
  topSymbols: SymbolUsage[];
  highlights: string[];
  concerns: string[];
}

export interface MoodTrend {
  average: MoodLevel;
  changes: MoodChange[];
  stability: number; // 0-1
}

export interface MoodChange {
  from: MoodLevel;
  to: MoodLevel;
  timestamp: Date;
  trigger?: string;
}

export interface SymbolUsage {
  symbolId: string;
  label: string;
  count: number;
  percentage: number;
}

export interface WeeklyProgress {
  weekStart: Date;
  weekEnd: Date;
  clientId: string;
  dailySummaries: DailySummary[];
  overallStats: {
    totalCommunications: number;
    avgDailyCommunications: number;
    vocabularyGrowth: number;
    responseTimeImprovement: number;
  };
  goalProgress: GoalProgress[];
  recommendations: string[];
}

export interface CommunicationEntry {
  id: string;
  timestamp: Date;
  message: string;
  symbols: string[];
  context?: ActivityContext;
  responseTime?: number;
}

// ============================================================================
// Goal & Treatment Types
// ============================================================================

export interface Goal {
  id: string;
  clientId: string;
  title: string;
  description: string;
  category: GoalCategory;
  targetDate: Date;
  status: GoalStatus;
  progress: number; // 0-100
  milestones: Milestone[];
  createdBy: string;
  createdAt: Date;
  updatedAt: Date;
}

export type GoalCategory =
  | 'vocabulary_expansion'
  | 'communication_rate'
  | 'sentence_complexity'
  | 'social_communication'
  | 'self_advocacy'
  | 'independence'
  | 'other';

export type GoalStatus = 'not_started' | 'in_progress' | 'achieved' | 'discontinued';

export interface Milestone {
  id: string;
  title: string;
  targetDate: Date;
  achieved: boolean;
  achievedDate?: Date;
}

export interface GoalProgress {
  goalId: string;
  goalTitle: string;
  currentProgress: number;
  previousProgress: number;
  trend: 'improving' | 'stable' | 'declining';
  lastUpdate: Date;
}

// ============================================================================
// Assessment Types
// ============================================================================

export type AssessmentType =
  | 'communication_matrix'
  | 'aac_evaluation'
  | 'vocabulary_assessment'
  | 'comprehension_test'
  | 'custom';

export interface AssessmentResult {
  id: string;
  clientId: string;
  type: AssessmentType;
  conductedBy: string;
  conductedAt: Date;
  scores: AssessmentScore[];
  notes: string;
  recommendations: string[];
}

export interface AssessmentScore {
  domain: string;
  score: number;
  maxScore: number;
  percentile?: number;
  interpretation?: string;
}

export interface ProgressComparison {
  clientId: string;
  startDate: Date;
  endDate: Date;
  metrics: ProgressMetric[];
  summary: string;
}

export interface ProgressMetric {
  name: string;
  startValue: number;
  endValue: number;
  change: number;
  changePercent: number;
}

// ============================================================================
// Vocabulary & Symbol Types
// ============================================================================

export interface Symbol {
  id: string;
  label: string;
  imageUrl?: string;
  audioUrl?: string;
  category: string;
  isCore: boolean;
  customFor?: string; // clientId if custom
  createdAt: Date;
}

export interface BoardLayout {
  id: string;
  name: string;
  clientId: string;
  pages: BoardPage[];
  createdBy: string;
  createdAt: Date;
  updatedAt: Date;
}

export interface BoardPage {
  id: string;
  title: string;
  symbols: BoardSymbol[];
  gridConfig: GridConfig;
}

export interface BoardSymbol {
  symbolId: string;
  position: { row: number; col: number };
  size: { rowSpan: number; colSpan: number };
}

export interface GridConfig {
  rows: number;
  columns: number;
  gap: number;
}

// ============================================================================
// Family Engagement Types
// ============================================================================

export interface Photo {
  id: string;
  url: string;
  caption?: string;
  uploadedBy: string;
  uploadedAt: Date;
  people?: string[];
  location?: string;
  date?: Date;
}

export interface Event {
  id: string;
  title: string;
  description?: string;
  startTime: Date;
  endTime?: Date;
  location?: string;
  participants: string[];
  reminders: Reminder[];
  createdBy: string;
}

export interface Reminder {
  id: string;
  beforeMinutes: number;
  sent: boolean;
  sentAt?: Date;
}

export interface LifeStoryEntry {
  id: string;
  clientId: string;
  title: string;
  content: string;
  era?: string;
  photos?: string[];
  people?: string[];
  createdBy: string;
  createdAt: Date;
}

export interface ReminiscenceSession {
  id: string;
  clientId: string;
  topic: string;
  entries: LifeStoryEntry[];
  startedAt: Date;
  endedAt?: Date;
  notes?: string;
}

// ============================================================================
// Compliance Types
// ============================================================================

export interface AuditEntry {
  id: string;
  timestamp: Date;
  userId: string;
  userRole: UserRole;
  action: AuditAction;
  resourceType: string;
  resourceId: string;
  details?: string;
  ipAddress?: string;
}

export type AuditAction =
  | 'view'
  | 'create'
  | 'update'
  | 'delete'
  | 'export'
  | 'login'
  | 'logout'
  | 'consent_change';

export interface ConsentRecord {
  id: string;
  userId: string;
  clientId?: string;
  type: ConsentType;
  granted: boolean;
  grantedBy: string;
  grantedAt: Date;
  expiresAt?: Date;
  purpose: string;
  scope: string[];
}

export type ConsentType =
  | 'data_collection'
  | 'data_sharing'
  | 'professional_access'
  | 'family_access'
  | 'research'
  | 'marketing';

export interface AccessPolicy {
  id: string;
  name: string;
  roles: UserRole[];
  resources: string[];
  actions: string[];
  conditions?: AccessCondition[];
}

export interface AccessCondition {
  type: 'time_based' | 'consent_required' | 'relationship_required';
  params: Record<string, unknown>;
}

export interface DataExportRequest {
  id: string;
  requestedBy: string;
  clientId: string;
  status: 'pending' | 'processing' | 'completed' | 'failed';
  format: 'json' | 'csv' | 'pdf';
  requestedAt: Date;
  completedAt?: Date;
  downloadUrl?: string;
  expiresAt?: Date;
}

// ============================================================================
// Clinical Report Types
// ============================================================================

export interface ClinicalReport {
  id: string;
  clientId: string;
  type: 'progress' | 'assessment' | 'discharge' | 'quarterly';
  generatedBy: string;
  generatedAt: Date;
  period: { start: Date; end: Date };
  sections: ReportSection[];
  recommendations: string[];
  signature?: {
    name: string;
    credentials: string;
    date: Date;
  };
}

export interface ReportSection {
  title: string;
  content: string;
  data?: Record<string, unknown>;
  charts?: ChartData[];
}

export interface ChartData {
  type: 'line' | 'bar' | 'pie';
  title: string;
  data: unknown;
}

// ============================================================================
// Real-time Types
// ============================================================================

export interface LiveSession {
  clientId: string;
  startedAt: Date;
  lastActivity: Date;
  currentPage?: string;
  activeSymbols: string[];
  mood?: Mood;
  alerts: Alert[];
}

export interface WebSocketMessage {
  type: WSMessageType;
  clientId: string;
  timestamp: Date;
  payload: unknown;
}

export type WSMessageType =
  | 'activity'
  | 'mood_change'
  | 'alert'
  | 'config_change'
  | 'message'
  | 'session_start'
  | 'session_end';
