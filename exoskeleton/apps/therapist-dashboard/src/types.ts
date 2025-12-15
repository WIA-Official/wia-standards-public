/**
 * WIA Exoskeleton Therapist Dashboard Types
 *
 * Type definitions for the therapist dashboard application
 */

// ============================================================================
// Patient Types
// ============================================================================

export interface Patient {
  patientId: string;
  profile: PatientProfile;
  currentProgram: ProgramAssignment | null;
  sessions: SessionSummary[];
  assessments: AssessmentRecord[];
  alerts: PatientAlert[];
  status: PatientStatus;
}

export interface PatientProfile {
  firstName: string;
  lastName: string;
  dateOfBirth: Date;
  gender: 'male' | 'female' | 'other';
  medicalRecordNumber: string;
  diagnosis: Diagnosis;
  injuryDate?: Date;
  physician: string;
  emergencyContact: EmergencyContact;
  notes: string;
}

export interface Diagnosis {
  condition: Condition;
  severity: SeverityLevel;
  level?: string; // For SCI (e.g., "T10")
  side?: 'left' | 'right' | 'bilateral'; // For stroke
  additionalDiagnoses: string[];
}

export enum Condition {
  SCI = 'SCI',
  STROKE = 'STROKE',
  TBI = 'TBI',
  MS = 'MS',
  PD = 'PD',
  CP = 'CP',
  GBS = 'GBS',
  POST_OP = 'POST_OP',
  OTHER = 'OTHER',
}

export enum SeverityLevel {
  MILD = 'mild',
  MODERATE = 'moderate',
  SEVERE = 'severe',
  COMPLETE = 'complete',
}

export interface EmergencyContact {
  name: string;
  relationship: string;
  phone: string;
  email?: string;
}

export enum PatientStatus {
  ACTIVE = 'active',
  ON_HOLD = 'on_hold',
  COMPLETED = 'completed',
  DISCHARGED = 'discharged',
}

// ============================================================================
// Program Types
// ============================================================================

export interface ProgramAssignment {
  assignmentId: string;
  programId: string;
  programName: string;
  startDate: Date;
  endDate: Date;
  currentPhase: number;
  currentSession: number;
  completedSessions: number;
  totalSessions: number;
  progressPercent: number;
  modifications: ProgramModification[];
}

export interface ProgramModification {
  modificationId: string;
  date: Date;
  type: ModificationType;
  description: string;
  modifiedBy: string;
  previousValue: string;
  newValue: string;
}

export enum ModificationType {
  ASSISTANCE_LEVEL = 'assistance_level',
  DURATION = 'duration',
  SPEED = 'speed',
  ACTIVITY_SKIP = 'activity_skip',
  PHASE_CHANGE = 'phase_change',
  CUSTOM = 'custom',
}

// ============================================================================
// Session Types
// ============================================================================

export interface SessionSummary {
  sessionId: string;
  date: Date;
  duration: number; // minutes
  therapistId: string;
  therapistName: string;
  status: SessionStatus;
  activities: ActivitySummary[];
  metrics: SessionMetrics;
  patientReported: PatientReportedOutcomes;
  notes: string;
  alerts: SessionAlert[];
}

export enum SessionStatus {
  SCHEDULED = 'scheduled',
  IN_PROGRESS = 'in_progress',
  COMPLETED = 'completed',
  CANCELLED = 'cancelled',
  NO_SHOW = 'no_show',
}

export interface ActivitySummary {
  activityId: string;
  type: ActivityType;
  duration: number;
  completed: boolean;
  assistanceLevel: number;
  targetMet: boolean;
  notes?: string;
}

export enum ActivityType {
  STANDING = 'standing',
  SIT_TO_STAND = 'sit_to_stand',
  WEIGHT_SHIFTING = 'weight_shifting',
  TREADMILL_WALKING = 'treadmill_walking',
  OVERGROUND_WALKING = 'overground_walking',
  STAIR_CLIMBING = 'stair_climbing',
  BALANCE_TRAINING = 'balance_training',
  STRENGTH_TRAINING = 'strength_training',
}

export interface SessionMetrics {
  walkingDistance: number;
  walkingTime: number;
  walkingSpeed: number;
  stepCount: number;
  standingTime: number;
  assistanceLevel: number;
  gaitSymmetry: number;
  heartRateAvg: number;
  heartRateMax: number;
  fatigueLevel: number;
}

export interface PatientReportedOutcomes {
  painLevel: number;
  fatigueLevel: number;
  confidenceLevel: number;
  satisfactionScore: number;
  comments?: string;
}

export interface SessionAlert {
  alertId: string;
  type: AlertType;
  severity: AlertSeverity;
  message: string;
  timestamp: Date;
  acknowledged: boolean;
}

export enum AlertType {
  VITAL_SIGN = 'vital_sign',
  SAFETY = 'safety',
  PERFORMANCE = 'performance',
  EQUIPMENT = 'equipment',
  PATIENT_REQUEST = 'patient_request',
}

export enum AlertSeverity {
  INFO = 'info',
  WARNING = 'warning',
  CRITICAL = 'critical',
}

// ============================================================================
// Assessment Types
// ============================================================================

export interface AssessmentRecord {
  assessmentId: string;
  date: Date;
  type: AssessmentType;
  assessorId: string;
  assessorName: string;
  results: AssessmentResults;
  notes: string;
}

export enum AssessmentType {
  TEN_METER_WALK = 'ten_meter_walk',
  SIX_MINUTE_WALK = 'six_minute_walk',
  TUG = 'tug',
  BERG_BALANCE = 'berg_balance',
  FIM = 'fim',
  BASELINE = 'baseline',
  PROGRESS = 'progress',
  DISCHARGE = 'discharge',
}

export interface AssessmentResults {
  [key: string]: number | string | boolean;
}

// ============================================================================
// Progress Types
// ============================================================================

export interface ProgressData {
  patientId: string;
  dateRange: DateRange;
  summary: ProgressSummary;
  metrics: MetricProgress[];
  milestones: Milestone[];
  trends: TrendData[];
}

export interface DateRange {
  start: Date;
  end: Date;
}

export interface ProgressSummary {
  overallProgress: number;
  weeklyChange: number;
  trend: 'improving' | 'stable' | 'declining';
  projectedCompletion: Date | null;
  keyAchievements: string[];
  areasForFocus: string[];
}

export interface MetricProgress {
  metricId: string;
  name: string;
  nameKorean: string;
  baseline: number;
  current: number;
  target: number;
  unit: string;
  percentChange: number;
  trend: 'up' | 'down' | 'stable';
  history: DataPoint[];
}

export interface DataPoint {
  timestamp: Date;
  value: number;
}

export interface Milestone {
  milestoneId: string;
  name: string;
  nameKorean: string;
  targetDate: Date;
  achieved: boolean;
  achievedDate?: Date;
  metric: string;
  targetValue: number;
  currentValue: number;
}

export interface TrendData {
  metricId: string;
  slope: number;
  rSquared: number;
  projectedValue: number;
  projectedDate: Date;
}

// ============================================================================
// Alert Types
// ============================================================================

export interface PatientAlert {
  alertId: string;
  patientId: string;
  type: PatientAlertType;
  severity: AlertSeverity;
  message: string;
  messageKorean: string;
  createdAt: Date;
  acknowledged: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: Date;
  actions: AlertAction[];
}

export enum PatientAlertType {
  MISSED_SESSION = 'missed_session',
  DECLINING_PROGRESS = 'declining_progress',
  PLATEAU_DETECTED = 'plateau_detected',
  GOAL_ACHIEVED = 'goal_achieved',
  ASSESSMENT_DUE = 'assessment_due',
  PROGRAM_ENDING = 'program_ending',
  CONTRAINDICATION = 'contraindication',
  CUSTOM = 'custom',
}

export interface AlertAction {
  actionId: string;
  label: string;
  labelKorean: string;
  type: 'navigate' | 'dismiss' | 'schedule' | 'custom';
  target?: string;
}

// ============================================================================
// Report Types
// ============================================================================

export interface ProgressReport {
  reportId: string;
  patientId: string;
  generatedAt: Date;
  generatedBy: string;
  period: DateRange;
  executiveSummary: ExecutiveSummary;
  sessionSummary: SessionReportSummary;
  metrics: MetricReportSection;
  assessments: AssessmentReportSection;
  goals: GoalReportSection;
  recommendations: string[];
  nextSteps: string[];
}

export interface ExecutiveSummary {
  status: 'on_track' | 'ahead' | 'behind' | 'complete';
  overallProgress: number;
  keyHighlights: string[];
  concerns: string[];
}

export interface SessionReportSummary {
  sessionsCompleted: number;
  sessionsScheduled: number;
  attendanceRate: number;
  totalTrainingTime: number;
  averageSessionDuration: number;
}

export interface MetricReportSection {
  categories: MetricCategoryReport[];
  highlights: MetricHighlight[];
}

export interface MetricCategoryReport {
  category: string;
  categoryKorean: string;
  metrics: MetricProgress[];
  overallChange: number;
}

export interface MetricHighlight {
  type: 'achievement' | 'concern' | 'milestone';
  message: string;
  messageKorean: string;
  metric?: string;
  value?: number;
}

export interface AssessmentReportSection {
  assessments: AssessmentRecord[];
  changes: AssessmentChange[];
}

export interface AssessmentChange {
  assessmentType: AssessmentType;
  previousScore: number;
  currentScore: number;
  change: number;
  interpretation: string;
}

export interface GoalReportSection {
  goals: GoalProgress[];
  overallAchievement: number;
}

export interface GoalProgress {
  goalId: string;
  description: string;
  descriptionKorean: string;
  baseline: number;
  target: number;
  current: number;
  percentAchieved: number;
  status: 'not_started' | 'in_progress' | 'achieved' | 'exceeded';
}

// ============================================================================
// Live Session Types
// ============================================================================

export interface LiveSessionView {
  sessionId: string;
  patientId: string;
  patientName: string;
  startTime: Date;
  currentActivity: ActivityStatus;
  vitalSigns: VitalSigns;
  metrics: RealTimeMetrics;
  alerts: SessionAlert[];
  status: LiveSessionStatus;
}

export interface ActivityStatus {
  activityId: string;
  type: ActivityType;
  startTime: Date;
  elapsedTime: number;
  remainingTime: number;
  assistanceLevel: number;
  targetProgress: number;
}

export interface VitalSigns {
  heartRate: number;
  bloodPressure?: { systolic: number; diastolic: number };
  oxygenSaturation?: number;
  respiratoryRate?: number;
  timestamp: Date;
}

export interface RealTimeMetrics {
  walkingSpeed: number;
  stepCount: number;
  distance: number;
  gaitSymmetry: number;
  fatigue: number;
  assistanceUsed: number;
}

export enum LiveSessionStatus {
  ACTIVE = 'active',
  PAUSED = 'paused',
  RESTING = 'resting',
  ENDING = 'ending',
}

// ============================================================================
// Dashboard State Types
// ============================================================================

export interface DashboardState {
  currentUser: TherapistUser;
  patients: Patient[];
  selectedPatient: Patient | null;
  activeSessions: LiveSessionView[];
  alerts: PatientAlert[];
  schedule: ScheduledSession[];
  filters: DashboardFilters;
  view: DashboardView;
}

export interface TherapistUser {
  userId: string;
  name: string;
  nameKorean: string;
  role: TherapistRole;
  department: string;
  licenseNumber: string;
}

export enum TherapistRole {
  THERAPIST = 'therapist',
  SENIOR_THERAPIST = 'senior_therapist',
  SUPERVISOR = 'supervisor',
  ADMIN = 'admin',
}

export interface ScheduledSession {
  scheduleId: string;
  patientId: string;
  patientName: string;
  therapistId: string;
  scheduledTime: Date;
  duration: number;
  sessionType: string;
  notes?: string;
  status: 'scheduled' | 'confirmed' | 'in_progress' | 'completed' | 'cancelled';
}

export interface DashboardFilters {
  patientStatus: PatientStatus[];
  conditions: Condition[];
  dateRange: DateRange;
  searchQuery: string;
}

export enum DashboardView {
  PATIENT_LIST = 'patient_list',
  PATIENT_DETAIL = 'patient_detail',
  SESSION_DETAIL = 'session_detail',
  LIVE_MONITORING = 'live_monitoring',
  REPORTS = 'reports',
  SCHEDULE = 'schedule',
  SETTINGS = 'settings',
}
