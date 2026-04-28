/**
 * WIA-EDU-006: Virtual Classroom Standard - TypeScript Type Definitions
 *
 * © 2025 WIA - World Certification Industry Association
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ============================================================================
// Configuration Types
// ============================================================================

export interface ClassroomConfig {
  apiKey: string;
  region?: 'us-west-1' | 'us-east-1' | 'eu-west-1' | 'ap-southeast-1';
  environment?: 'production' | 'staging' | 'development';
  baseUrl?: string;
  timeout?: number;
}

// ============================================================================
// Session Types
// ============================================================================

export interface CreateSessionConfig {
  title: string;
  instructor: Instructor;
  schedule: Schedule;
  settings: SessionSettings;
  features?: SessionFeatures;
}

export interface Instructor {
  userId?: string;
  name: string;
  email: string;
  avatar?: string;
}

export interface Schedule {
  startTime: string; // ISO 8601
  duration: number; // minutes
  timezone?: string;
  recurrence?: Recurrence;
}

export interface Recurrence {
  type: 'daily' | 'weekly' | 'monthly';
  interval: number;
  endDate?: string;
}

export interface SessionSettings {
  maxStudents: number;
  waitingRoom?: boolean;
  requirePassword?: boolean;
  password?: string;
  enableRecording?: boolean;
  muteOnEntry?: boolean;
  videoOnEntry?: boolean;
  breakoutRooms?: BreakoutRoomSettings;
  attendance?: AttendanceSettings;
}

export interface BreakoutRoomSettings {
  enabled: boolean;
  autoAssign?: boolean;
  roomSize?: number;
}

export interface AttendanceSettings {
  faceRecognition?: boolean;
  participationTracking?: boolean;
  attentionMonitoring?: boolean;
}

export interface SessionFeatures {
  screenSharing?: boolean;
  polls?: boolean;
  quizzes?: boolean;
  handRaise?: boolean;
  chat?: boolean;
  whiteboard?: boolean;
}

export interface Session {
  sessionId: string;
  title: string;
  instructor: Instructor;
  schedule: Schedule;
  settings: SessionSettings;
  joinUrl: string;
  instructorUrl: string;
  status: SessionStatus;
  createdAt: string;
  updatedAt?: string;
}

export type SessionStatus =
  | 'scheduled'
  | 'waiting'
  | 'active'
  | 'ended'
  | 'cancelled';

// ============================================================================
// Participant Types
// ============================================================================

export interface Participant {
  userId: string;
  name: string;
  email: string;
  role: ParticipantRole;
  joinTime?: string;
  leaveTime?: string;
  status: ParticipantStatus;
  videoEnabled: boolean;
  audioEnabled: boolean;
  handRaised: boolean;
}

export type ParticipantRole = 'instructor' | 'student' | 'ta' | 'observer';
export type ParticipantStatus = 'waiting' | 'active' | 'disconnected';

// ============================================================================
// Whiteboard Types
// ============================================================================

export interface WhiteboardConfig {
  sessionId: string;
  settings: WhiteboardSettings;
}

export interface WhiteboardSettings {
  tools?: WhiteboardTool[];
  enableLatex?: boolean;
  enableFileUpload?: boolean;
  maxPages?: number;
  backgroundColor?: string;
}

export type WhiteboardTool =
  | 'pen'
  | 'highlighter'
  | 'shapes'
  | 'text'
  | 'eraser'
  | 'select';

export interface Whiteboard {
  whiteboardId: string;
  sessionId: string;
  pages: WhiteboardPage[];
  currentPage: number;
  settings: WhiteboardSettings;
}

export interface WhiteboardPage {
  pageId: string;
  width: number;
  height: number;
  background: string;
  objects: WhiteboardObject[];
}

export interface WhiteboardObject {
  id: string;
  type: 'path' | 'text' | 'shape' | 'image';
  timestamp: number;
  userId: string;
  [key: string]: any;
}

export interface WhiteboardAnnotation {
  type: 'text' | 'arrow' | 'highlight';
  content?: string;
  position: { x: number; y: number };
  color?: string;
  fontSize?: number;
}

// ============================================================================
// Breakout Room Types
// ============================================================================

export interface CreateBreakoutRoomsConfig {
  sessionId: string;
  count: number;
  duration?: number;
  autoAssign?: boolean;
  allowSelfSelect?: boolean;
}

export interface BreakoutRoom {
  roomId: string;
  name: string;
  participants: string[];
  startTime?: string;
  duration?: number;
  status: 'pending' | 'active' | 'ended';
}

export interface BroadcastMessage {
  sessionId: string;
  message: string;
  type?: 'info' | 'alert' | 'warning';
}

// ============================================================================
// Attendance Types
// ============================================================================

export interface Attendance {
  sessionId: string;
  totalEnrolled: number;
  present: number;
  late: number;
  absent: number;
  participants: AttendanceRecord[];
}

export interface AttendanceRecord {
  userId: string;
  name: string;
  email: string;
  joinTime?: string;
  leaveTime?: string;
  status: 'present' | 'late' | 'absent';
  duration?: number;
  attentionScore?: number;
  participationScore?: number;
}

export interface ExportAttendanceConfig {
  sessionId: string;
  format: 'csv' | 'xlsx' | 'json';
  includeEngagement?: boolean;
}

export interface AttendanceReport {
  sessionId: string;
  exportedAt: string;
  format: string;
  downloadUrl: string;
}

// ============================================================================
// Quiz & Poll Types
// ============================================================================

export interface CreateQuizConfig {
  sessionId: string;
  title: string;
  duration?: number;
  questions: QuizQuestion[];
  settings?: QuizSettings;
}

export interface QuizQuestion {
  type: 'multiple-choice' | 'true-false' | 'short-answer' | 'essay';
  question: string;
  options?: string[];
  correctAnswer?: number | boolean | string;
  points: number;
}

export interface QuizSettings {
  randomizeQuestions?: boolean;
  showCorrectAnswers?: boolean;
  allowReview?: boolean;
  timeLimit?: number;
}

export interface Quiz {
  quizId: string;
  sessionId: string;
  title: string;
  questions: QuizQuestion[];
  totalPoints: number;
  status: 'draft' | 'active' | 'ended';
}

export interface QuizResults {
  quizId: string;
  totalSubmissions: number;
  averageScore: number;
  passRate: number;
  submissions: QuizSubmission[];
}

export interface QuizSubmission {
  submissionId: string;
  student: string;
  score: number;
  totalPoints: number;
  answers: any[];
  submittedAt: string;
}

export interface CreatePollConfig {
  sessionId: string;
  question: string;
  options: string[];
  settings?: PollSettings;
}

export interface PollSettings {
  allowMultiple?: boolean;
  showResults?: 'immediately' | 'after-voting' | 'never';
  anonymous?: boolean;
}

export interface Poll {
  pollId: string;
  question: string;
  options: string[];
  results: Record<string, number>;
  status: 'active' | 'ended';
}

// ============================================================================
// Recording Types
// ============================================================================

export interface StartRecordingConfig {
  sessionId: string;
  settings?: RecordingSettings;
}

export interface RecordingSettings {
  quality?: '720p' | '1080p' | '4k';
  layout?: 'speaker-view' | 'gallery-view' | 'grid-view';
  enableTranscription?: boolean;
  transcriptionLanguage?: string;
  separateAudioTracks?: boolean;
}

export interface Recording {
  recordingId: string;
  sessionId: string;
  url: string;
  duration: number;
  size: number;
  format: string;
  quality: string;
  createdAt: string;
}

export interface Transcription {
  recordingId: string;
  language: string;
  segments: TranscriptionSegment[];
}

export interface TranscriptionSegment {
  timestamp: string;
  speaker: string;
  text: string;
  confidence: number;
}

export interface GenerateSubtitlesConfig {
  recordingId: string;
  format: 'srt' | 'vtt' | 'ass';
  language: string;
}

export interface Subtitles {
  recordingId: string;
  format: string;
  url: string;
}

// ============================================================================
// Engagement Types
// ============================================================================

export interface EngagementMetrics {
  sessionId: string;
  averageAttention: number;
  participationRate: number;
  chatMessages: number;
  questionsAsked: number;
  handRaises: number;
  pollResponses: number;
  quizParticipation: number;
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface SessionAnalytics {
  sessionId: string;
  duration: number;
  peakAttendance: number;
  averageAttendance: number;
  engagement: EngagementMetrics;
  recordingUrl?: string;
  transcriptUrl?: string;
}

// ============================================================================
// Event Types
// ============================================================================

export interface SessionEvent {
  type: string;
  timestamp: string;
  data: any;
}

export type EventType =
  | 'session:started'
  | 'session:ended'
  | 'student:joined'
  | 'student:left'
  | 'attendance:updated'
  | 'quiz:submitted'
  | 'poll:created'
  | 'breakout:created'
  | 'breakout:activity'
  | 'recording:started'
  | 'recording:stopped';

// ============================================================================
// Error Types
// ============================================================================

export interface APIError {
  code: number;
  message: string;
  details?: string;
  timestamp: string;
}

export class VirtualClassroomError extends Error {
  code: number;
  details?: string;

  constructor(message: string, code: number, details?: string) {
    super(message);
    this.name = 'VirtualClassroomError';
    this.code = code;
    this.details = details;
  }
}
