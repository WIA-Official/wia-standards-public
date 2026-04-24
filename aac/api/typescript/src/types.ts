/**
 * WIA AAC (Augmentative and Alternative Communication) Standard
 * TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * Unique identifier
 */
export type UUID = string;

/**
 * Language code (ISO 639-1)
 */
export type LanguageCode = string;

// ============================================================================
// Sensor Types
// ============================================================================

/**
 * Supported sensor types for AAC input
 */
export enum SensorType {
  EYE_TRACKER = 'eye_tracker',
  SWITCH = 'switch',
  MUSCLE_SENSOR = 'muscle_sensor',
  BRAIN_INTERFACE = 'brain_interface',
  BREATH = 'breath',
  HEAD_MOVEMENT = 'head_movement',
  VOICE = 'voice',
  TOUCH = 'touch',
  GESTURE = 'gesture',
  CUSTOM = 'custom',
}

/**
 * Sensor status
 */
export enum SensorStatus {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  CALIBRATING = 'calibrating',
  ACTIVE = 'active',
  ERROR = 'error',
  PAUSED = 'paused',
}

/**
 * Sensor configuration
 */
export interface SensorConfig {
  sensorId: string;
  type: SensorType;
  name: string;
  vendor?: string;
  model?: string;
  firmwareVersion?: string;
  samplingRate: number;
  sensitivity: number;
  calibrationRequired: boolean;
  calibrationData?: CalibrationData;
  connectionType: ConnectionType;
  connectionParams?: Record<string, unknown>;
}

/**
 * Connection type
 */
export enum ConnectionType {
  USB = 'usb',
  BLUETOOTH = 'bluetooth',
  WIFI = 'wifi',
  SERIAL = 'serial',
  WEBSOCKET = 'websocket',
  CUSTOM = 'custom',
}

/**
 * Calibration data
 */
export interface CalibrationData {
  calibrationId: string;
  userId: string;
  sensorId: string;
  calibratedAt: Timestamp;
  expiresAt?: Timestamp;
  points: CalibrationPoint[];
  accuracy: number;
  status: CalibrationStatus;
}

/**
 * Calibration point
 */
export interface CalibrationPoint {
  x: number;
  y: number;
  targetX: number;
  targetY: number;
  timestamp: Timestamp;
  valid: boolean;
}

/**
 * Calibration status
 */
export enum CalibrationStatus {
  PENDING = 'pending',
  IN_PROGRESS = 'in_progress',
  COMPLETED = 'completed',
  FAILED = 'failed',
  EXPIRED = 'expired',
}

// ============================================================================
// Signal Types
// ============================================================================

/**
 * Raw signal data
 */
export interface SignalData {
  signalId: string;
  sensorId: string;
  timestamp: Timestamp;
  sampleIndex: number;
  channels: ChannelData[];
  quality: SignalQuality;
  metadata?: Record<string, unknown>;
}

/**
 * Channel data
 */
export interface ChannelData {
  channelId: string;
  name: string;
  values: number[];
  unit: string;
}

/**
 * Signal quality
 */
export interface SignalQuality {
  overall: number;
  channelQualities: Record<string, number>;
  noiseLevel: number;
  signalToNoise: number;
  artifacts: ArtifactInfo[];
}

/**
 * Artifact information
 */
export interface ArtifactInfo {
  type: ArtifactType;
  severity: number;
  startIndex: number;
  endIndex: number;
  confidence: number;
}

/**
 * Artifact types
 */
export enum ArtifactType {
  BLINK = 'blink',
  MOVEMENT = 'movement',
  NOISE = 'noise',
  SATURATION = 'saturation',
  DISCONNECTION = 'disconnection',
  UNKNOWN = 'unknown',
}

// ============================================================================
// Symbol and Communication Types
// ============================================================================

/**
 * Symbol for AAC communication
 */
export interface Symbol {
  symbolId: string;
  type: SymbolType;
  label: string;
  description?: string;
  imageUrl?: string;
  audioUrl?: string;
  category: string;
  subcategory?: string;
  tags: string[];
  synonyms: string[];
  translations: SymbolTranslation[];
  usageFrequency: number;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

/**
 * Symbol type
 */
export enum SymbolType {
  PICTOGRAM = 'pictogram',
  ICON = 'icon',
  TEXT = 'text',
  PHOTO = 'photo',
  CUSTOM = 'custom',
}

/**
 * Symbol translation
 */
export interface SymbolTranslation {
  languageCode: LanguageCode;
  label: string;
  description?: string;
  audioUrl?: string;
}

/**
 * Symbol board
 */
export interface SymbolBoard {
  boardId: string;
  name: string;
  description?: string;
  userId: string;
  layout: BoardLayout;
  symbols: BoardSymbol[];
  theme: BoardTheme;
  isDefault: boolean;
  isShared: boolean;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

/**
 * Board layout
 */
export interface BoardLayout {
  rows: number;
  columns: number;
  cellWidth: number;
  cellHeight: number;
  gap: number;
  padding: number;
}

/**
 * Board symbol placement
 */
export interface BoardSymbol {
  symbolId: string;
  row: number;
  column: number;
  rowSpan?: number;
  colSpan?: number;
  customStyle?: SymbolStyle;
}

/**
 * Symbol style
 */
export interface SymbolStyle {
  backgroundColor?: string;
  borderColor?: string;
  borderWidth?: number;
  textColor?: string;
  fontSize?: number;
  fontWeight?: string;
}

/**
 * Board theme
 */
export interface BoardTheme {
  name: string;
  backgroundColor: string;
  cellBackgroundColor: string;
  cellBorderColor: string;
  textColor: string;
  highlightColor: string;
  fontSize: number;
}

// ============================================================================
// User Profile Types
// ============================================================================

/**
 * User profile
 */
export interface UserProfile {
  userId: string;
  name: string;
  dateOfBirth?: string;
  primaryLanguage: LanguageCode;
  secondaryLanguages: LanguageCode[];
  communicationLevel: CommunicationLevel;
  disabilities: DisabilityInfo[];
  preferences: UserPreferences;
  sensors: SensorConfig[];
  boards: string[];
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

/**
 * Communication level
 */
export enum CommunicationLevel {
  BEGINNER = 'beginner',
  INTERMEDIATE = 'intermediate',
  ADVANCED = 'advanced',
  FLUENT = 'fluent',
}

/**
 * Disability information
 */
export interface DisabilityInfo {
  type: DisabilityType;
  severity: DisabilitySeverity;
  description?: string;
  accommodations: string[];
}

/**
 * Disability type
 */
export enum DisabilityType {
  MOTOR = 'motor',
  VISUAL = 'visual',
  HEARING = 'hearing',
  COGNITIVE = 'cognitive',
  SPEECH = 'speech',
  MULTIPLE = 'multiple',
}

/**
 * Disability severity
 */
export enum DisabilitySeverity {
  MILD = 'mild',
  MODERATE = 'moderate',
  SEVERE = 'severe',
  PROFOUND = 'profound',
}

/**
 * User preferences
 */
export interface UserPreferences {
  scanSpeed: number;
  dwellTime: number;
  highlightStyle: HighlightStyle;
  audioFeedback: boolean;
  visualFeedback: boolean;
  hapticFeedback: boolean;
  autoSave: boolean;
  predictionEnabled: boolean;
  voiceOutputEnabled: boolean;
  voiceSettings: VoiceSettings;
}

/**
 * Highlight style
 */
export enum HighlightStyle {
  BORDER = 'border',
  BACKGROUND = 'background',
  GLOW = 'glow',
  SCALE = 'scale',
}

/**
 * Voice settings
 */
export interface VoiceSettings {
  voiceId: string;
  rate: number;
  pitch: number;
  volume: number;
  language: LanguageCode;
}

// ============================================================================
// Session and Analytics Types
// ============================================================================

/**
 * Communication session
 */
export interface CommunicationSession {
  sessionId: string;
  userId: string;
  startTime: Timestamp;
  endTime?: Timestamp;
  duration?: number;
  boardsUsed: string[];
  symbolsUsed: SymbolUsage[];
  messages: Message[];
  analytics: SessionAnalytics;
}

/**
 * Symbol usage
 */
export interface SymbolUsage {
  symbolId: string;
  count: number;
  averageDwellTime: number;
  firstUsedAt: Timestamp;
  lastUsedAt: Timestamp;
}

/**
 * Message
 */
export interface Message {
  messageId: string;
  content: string;
  symbols: string[];
  timestamp: Timestamp;
  outputType: OutputType;
  confidence: number;
}

/**
 * Output type
 */
export enum OutputType {
  SPEECH = 'speech',
  TEXT = 'text',
  BOTH = 'both',
}

/**
 * Session analytics
 */
export interface SessionAnalytics {
  totalSelections: number;
  correctSelections: number;
  errorRate: number;
  averageSelectionTime: number;
  wordsPerMinute: number;
  symbolsPerMinute: number;
  mostUsedSymbols: string[];
  fatigueScore: number;
}

// ============================================================================
// API Types
// ============================================================================

/**
 * API response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  timestamp: Timestamp;
  requestId: string;
}

/**
 * API error
 */
export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  pageSize: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Event types
 */
export enum EventType {
  SIGNAL = 'signal',
  SELECTION = 'selection',
  CALIBRATION = 'calibration',
  CONNECTION = 'connection',
  ERROR = 'error',
  MESSAGE = 'message',
  BOARD_CHANGE = 'board_change',
}

/**
 * Base event
 */
export interface BaseEvent {
  type: EventType;
  timestamp: Timestamp;
  sessionId?: string;
}

/**
 * Signal event
 */
export interface SignalEvent extends BaseEvent {
  type: EventType.SIGNAL;
  sensorId: string;
  data: SignalData;
}

/**
 * Selection event
 */
export interface SelectionEvent extends BaseEvent {
  type: EventType.SELECTION;
  symbolId: string;
  confidence: number;
  selectionMethod: SelectionMethod;
}

/**
 * Selection method
 */
export enum SelectionMethod {
  DWELL = 'dwell',
  CLICK = 'click',
  SWITCH = 'switch',
  VOICE = 'voice',
  GESTURE = 'gesture',
  BRAIN = 'brain',
}

/**
 * Event handler type
 */
export type EventHandler<T extends BaseEvent> = (event: T) => void | Promise<void>;
