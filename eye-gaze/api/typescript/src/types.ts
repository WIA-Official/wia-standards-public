/**
 * WIA Eye Gaze Tracking Standard
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

export type Timestamp = string;
export type UUID = string;

// ============================================================================
// Device Types
// ============================================================================

export enum EyeTrackerType {
  SCREEN_BASED = 'screen_based',
  HEAD_MOUNTED = 'head_mounted',
  GLASSES = 'glasses',
  WEBCAM = 'webcam',
  MOBILE = 'mobile',
  VR_INTEGRATED = 'vr_integrated',
}

export interface EyeTrackerDevice {
  deviceId: string;
  type: EyeTrackerType;
  manufacturer: string;
  model: string;
  serialNumber: string;
  firmwareVersion: string;
  status: DeviceStatus;
  capabilities: DeviceCapabilities;
  calibration: CalibrationStatus;
}

export enum DeviceStatus {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  TRACKING = 'tracking',
  CALIBRATING = 'calibrating',
  ERROR = 'error',
  PAUSED = 'paused',
}

export interface DeviceCapabilities {
  samplingRate: number;
  maxSamplingRate: number;
  accuracy: number;
  precision: number;
  trackingRange: TrackingRange;
  binocular: boolean;
  pupilTracking: boolean;
  blinkDetection: boolean;
  headTracking: boolean;
  depthTracking: boolean;
  supportedFeatures: Feature[];
}

export interface TrackingRange {
  horizontalDegrees: number;
  verticalDegrees: number;
  minDistance: number;
  maxDistance: number;
  trackableArea: TrackableArea;
}

export interface TrackableArea {
  width: number;
  height: number;
  unit: 'pixels' | 'cm' | 'mm';
}

export enum Feature {
  FIXATION_DETECTION = 'fixation_detection',
  SACCADE_DETECTION = 'saccade_detection',
  SMOOTH_PURSUIT = 'smooth_pursuit',
  VERGENCE = 'vergence',
  PUPIL_DILATION = 'pupil_dilation',
  BLINK_DETECTION = 'blink_detection',
  HEAD_POSITION = 'head_position',
  AREA_OF_INTEREST = 'area_of_interest',
  HEATMAP = 'heatmap',
}

// ============================================================================
// Calibration Types
// ============================================================================

export interface CalibrationStatus {
  isCalibrated: boolean;
  lastCalibration: Timestamp;
  quality: CalibrationQuality;
  pointsUsed: number;
  userId?: string;
}

export enum CalibrationQuality {
  EXCELLENT = 'excellent',
  GOOD = 'good',
  FAIR = 'fair',
  POOR = 'poor',
  FAILED = 'failed',
}

export interface CalibrationConfig {
  pointCount: CalibrationPointCount;
  targetSize: number;
  targetColor: string;
  backgroundColor: string;
  animationType: AnimationType;
  dwellTime: number;
  randomOrder: boolean;
  showFeedback: boolean;
}

export enum CalibrationPointCount {
  ONE = 1,
  TWO = 2,
  FIVE = 5,
  NINE = 9,
  THIRTEEN = 13,
  CUSTOM = 0,
}

export enum AnimationType {
  NONE = 'none',
  SHRINK = 'shrink',
  PULSE = 'pulse',
  FADE = 'fade',
}

export interface CalibrationResult {
  calibrationId: string;
  timestamp: Timestamp;
  quality: CalibrationQuality;
  points: CalibrationPoint[];
  averageError: number;
  maxError: number;
  validPoints: number;
  duration: number;
}

export interface CalibrationPoint {
  index: number;
  target: Point2D;
  measured: Point2D;
  error: number;
  valid: boolean;
  leftEye?: EyeCalibrationData;
  rightEye?: EyeCalibrationData;
}

export interface EyeCalibrationData {
  position: Point2D;
  error: number;
  valid: boolean;
}

export interface Point2D {
  x: number;
  y: number;
}

export interface Point3D {
  x: number;
  y: number;
  z: number;
}

// ============================================================================
// Gaze Data Types
// ============================================================================

export interface GazeData {
  timestamp: Timestamp;
  deviceTimestamp: number;
  gazePoint: GazePoint;
  leftEye: EyeData;
  rightEye: EyeData;
  combinedEye: EyeData;
  headPosition?: HeadPosition;
  quality: GazeQuality;
}

export interface GazePoint {
  x: number;
  y: number;
  z?: number;
  validity: DataValidity;
  confidence: number;
}

export interface EyeData {
  gazePoint: GazePoint;
  gazeOrigin: Point3D;
  gazeDirection: Vector3D;
  pupil: PupilData;
  eyelid: EyelidData;
  validity: DataValidity;
}

export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

export interface PupilData {
  diameter: number;
  position: Point2D;
  confidence: number;
  validity: DataValidity;
}

export interface EyelidData {
  opening: number;
  isBlinking: boolean;
  blinkDuration?: number;
}

export interface HeadPosition {
  position: Point3D;
  rotation: Rotation3D;
  velocity?: Vector3D;
  angularVelocity?: Vector3D;
}

export interface Rotation3D {
  pitch: number;
  yaw: number;
  roll: number;
}

export enum DataValidity {
  VALID = 'valid',
  PROBABLY_VALID = 'probably_valid',
  UNCERTAIN = 'uncertain',
  INVALID = 'invalid',
}

export interface GazeQuality {
  overall: number;
  leftEye: number;
  rightEye: number;
  tracking: boolean;
  issues: QualityIssue[];
}

export enum QualityIssue {
  LOW_CONFIDENCE = 'low_confidence',
  PARTIAL_OCCLUSION = 'partial_occlusion',
  HEAD_MOVEMENT = 'head_movement',
  POOR_LIGHTING = 'poor_lighting',
  OUT_OF_RANGE = 'out_of_range',
  GLASSES_REFLECTION = 'glasses_reflection',
}

// ============================================================================
// Event Types
// ============================================================================

export interface GazeEvent {
  eventId: string;
  timestamp: Timestamp;
  type: GazeEventType;
  data: EventData;
  duration?: number;
}

export enum GazeEventType {
  FIXATION = 'fixation',
  SACCADE = 'saccade',
  SMOOTH_PURSUIT = 'smooth_pursuit',
  BLINK = 'blink',
  DWELL = 'dwell',
  AOI_ENTER = 'aoi_enter',
  AOI_EXIT = 'aoi_exit',
  AOI_DWELL = 'aoi_dwell',
}

export type EventData =
  | FixationEvent
  | SaccadeEvent
  | SmoothPursuitEvent
  | BlinkEvent
  | DwellEvent
  | AOIEvent;

export interface FixationEvent {
  type: 'fixation';
  position: Point2D;
  duration: number;
  dispersion: number;
  startTimestamp: Timestamp;
  endTimestamp: Timestamp;
}

export interface SaccadeEvent {
  type: 'saccade';
  startPosition: Point2D;
  endPosition: Point2D;
  amplitude: number;
  peakVelocity: number;
  duration: number;
  direction: number;
}

export interface SmoothPursuitEvent {
  type: 'smooth_pursuit';
  path: Point2D[];
  velocity: number;
  duration: number;
  accuracy: number;
}

export interface BlinkEvent {
  type: 'blink';
  duration: number;
  eye: 'left' | 'right' | 'both';
  complete: boolean;
}

export interface DwellEvent {
  type: 'dwell';
  position: Point2D;
  targetId?: string;
  duration: number;
  threshold: number;
  activated: boolean;
}

export interface AOIEvent {
  type: 'aoi';
  aoiId: string;
  action: 'enter' | 'exit' | 'dwell';
  timestamp: Timestamp;
  duration?: number;
  gazePoint: Point2D;
}

// ============================================================================
// Area of Interest Types
// ============================================================================

export interface AreaOfInterest {
  aoiId: string;
  name: string;
  description?: string;
  shape: AOIShape;
  bounds: AOIBounds;
  tags: string[];
  metadata?: Record<string, unknown>;
}

export enum AOIShape {
  RECTANGLE = 'rectangle',
  ELLIPSE = 'ellipse',
  POLYGON = 'polygon',
  CIRCLE = 'circle',
}

export type AOIBounds = RectangleBounds | EllipseBounds | CircleBounds | PolygonBounds;

export interface RectangleBounds {
  shape: 'rectangle';
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface EllipseBounds {
  shape: 'ellipse';
  centerX: number;
  centerY: number;
  radiusX: number;
  radiusY: number;
}

export interface CircleBounds {
  shape: 'circle';
  centerX: number;
  centerY: number;
  radius: number;
}

export interface PolygonBounds {
  shape: 'polygon';
  points: Point2D[];
}

export interface AOIMetrics {
  aoiId: string;
  totalFixations: number;
  totalDwellTime: number;
  averageFixationDuration: number;
  firstFixationTime?: number;
  visitCount: number;
  revisitCount: number;
  entryCount: number;
  exitCount: number;
}

// ============================================================================
// Session Types
// ============================================================================

export interface EyeTrackingSession {
  sessionId: string;
  userId?: string;
  deviceId: string;
  startTime: Timestamp;
  endTime?: Timestamp;
  status: SessionStatus;
  config: SessionConfig;
  calibration: CalibrationResult;
  metrics: SessionMetrics;
}

export enum SessionStatus {
  INITIALIZING = 'initializing',
  CALIBRATING = 'calibrating',
  RECORDING = 'recording',
  PAUSED = 'paused',
  COMPLETED = 'completed',
  ERROR = 'error',
}

export interface SessionConfig {
  samplingRate: number;
  recordRawData: boolean;
  detectEvents: boolean;
  eventThresholds: EventThresholds;
  areasOfInterest: AreaOfInterest[];
  recordingFormat: RecordingFormat;
}

export interface EventThresholds {
  fixationMinDuration: number;
  fixationMaxDispersion: number;
  saccadeMinVelocity: number;
  dwellTime: number;
  blinkMaxDuration: number;
}

export enum RecordingFormat {
  CSV = 'csv',
  JSON = 'json',
  TSV = 'tsv',
  BINARY = 'binary',
}

export interface SessionMetrics {
  totalSamples: number;
  validSamples: number;
  dataLossPercentage: number;
  averageSamplingRate: number;
  totalFixations: number;
  totalSaccades: number;
  totalBlinks: number;
  averageFixationDuration: number;
  averageSaccadeAmplitude: number;
  scanPath: ScanPathMetrics;
}

export interface ScanPathMetrics {
  totalLength: number;
  convexHullArea: number;
  spatialDensity: number;
  directionChanges: number;
}

// ============================================================================
// Input Control Types
// ============================================================================

export interface GazeInputConfig {
  dwellTime: number;
  dwellActivation: boolean;
  smoothingLevel: SmoothingLevel;
  magneticTargets: boolean;
  cursorSpeed: number;
  clickZones: ClickZone[];
}

export enum SmoothingLevel {
  NONE = 'none',
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high',
}

export interface ClickZone {
  zoneId: string;
  bounds: AOIBounds;
  action: ClickAction;
  dwellTimeOverride?: number;
}

export enum ClickAction {
  LEFT_CLICK = 'left_click',
  RIGHT_CLICK = 'right_click',
  DOUBLE_CLICK = 'double_click',
  DRAG_START = 'drag_start',
  DRAG_END = 'drag_end',
  SCROLL_UP = 'scroll_up',
  SCROLL_DOWN = 'scroll_down',
  CUSTOM = 'custom',
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface HeatmapData {
  width: number;
  height: number;
  resolution: number;
  data: number[][];
  maxValue: number;
  totalPoints: number;
}

export interface GazePlotData {
  fixations: FixationPlot[];
  saccades: SaccadePlot[];
  scanPath: Point2D[];
}

export interface FixationPlot {
  id: string;
  x: number;
  y: number;
  duration: number;
  order: number;
}

export interface SaccadePlot {
  id: string;
  startX: number;
  startY: number;
  endX: number;
  endY: number;
  order: number;
}

// ============================================================================
// API Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  timestamp: Timestamp;
  requestId: string;
}

export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

export interface PaginationParams {
  page: number;
  pageSize: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}

export type EventHandler<T> = (event: T) => void | Promise<void>;
