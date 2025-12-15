/**
 * WIA Eye Gaze Standard - Type Definitions
 *
 * 弘益人間 - 널리 인간을 이롭게
 */

// ============================================
// Basic Types
// ============================================

export interface Vector2D {
  x: number;
  y: number;
}

export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

export interface BoundingBox {
  x: number;
  y: number;
  width: number;
  height: number;
}

// ============================================
// Gaze Data Types
// ============================================

export interface EyeData {
  gaze: Vector2D;
  valid: boolean;
  pupilDiameter?: number;
  pupilCenter?: Vector2D;
  gazeOrigin?: Vector3D;
  gazeDirection?: Vector3D;
  eyeOpenness?: number;
  eyeOpennessMm?: number;
}

export interface GazePoint {
  timestamp: number;
  x: number;
  y: number;
  confidence: number;
  valid: boolean;
  leftEye?: EyeData;
  rightEye?: EyeData;
  fixation?: boolean;
  saccade?: boolean;
  fixationId?: string;
  deviceTimestamp?: number;
  metadata?: Record<string, unknown>;
}

export type InvalidReason =
  | 'blink'
  | 'tracking_lost'
  | 'out_of_range'
  | 'calibration_required'
  | 'unknown';

export interface GazeBatch {
  version: string;
  deviceId: string;
  sessionId: string;
  startTimestamp: number;
  endTimestamp: number;
  samplingRate: number;
  samples: GazePoint[];
}

// ============================================
// Event Types
// ============================================

export enum GazeEventType {
  // Eye Movement Events
  FIXATION_START = 'fixation_start',
  FIXATION_UPDATE = 'fixation_update',
  FIXATION_END = 'fixation_end',
  SACCADE_START = 'saccade_start',
  SACCADE_END = 'saccade_end',
  SMOOTH_PURSUIT_START = 'smooth_pursuit_start',
  SMOOTH_PURSUIT_END = 'smooth_pursuit_end',

  // Eye State Events
  BLINK_START = 'blink_start',
  BLINK_END = 'blink_end',
  BLINK = 'blink',
  WINK_LEFT = 'wink_left',
  WINK_RIGHT = 'wink_right',
  DOUBLE_BLINK = 'double_blink',

  // Interaction Events
  DWELL_START = 'dwell_start',
  DWELL_PROGRESS = 'dwell_progress',
  DWELL_COMPLETE = 'dwell_complete',
  DWELL_CANCEL = 'dwell_cancel',
  GAZE_ENTER = 'gaze_enter',
  GAZE_LEAVE = 'gaze_leave',

  // System Events
  CALIBRATION_START = 'calibration_start',
  CALIBRATION_POINT = 'calibration_point',
  CALIBRATION_END = 'calibration_end',
  TRACKING_LOST = 'tracking_lost',
  TRACKING_RECOVERED = 'tracking_recovered',
  DEVICE_CONNECTED = 'device_connected',
  DEVICE_DISCONNECTED = 'device_disconnected',
}

export type TargetSemanticType =
  | 'button'
  | 'link'
  | 'text'
  | 'input'
  | 'image'
  | 'video'
  | 'menu'
  | 'menuitem'
  | 'listitem'
  | 'scrollbar'
  | 'keyboard_key'
  | 'aac_symbol'
  | 'custom';

export interface AccessibilityInfo {
  role?: string;
  name?: string;
  description?: string;
  state?: string[];
}

export interface GazeTarget {
  elementId: string;
  boundingBox: BoundingBox;
  semanticType: TargetSemanticType;
  label?: string;
  accessibility?: AccessibilityInfo;
  attributes?: Record<string, unknown>;
}

export interface GazeEvent {
  type: GazeEventType;
  timestamp: number;
  eventId: string;
  duration?: number;
  position?: Vector2D;
  target?: GazeTarget;
  gazeData?: GazePoint;
  previousEventId?: string;
  metadata?: Record<string, unknown>;
}

// ============================================
// Device Capability Types
// ============================================

export type DeviceType =
  | 'screen_based'
  | 'wearable'
  | 'remote'
  | 'integrated'
  | 'webcam_based'
  | 'unknown';

export interface EyeTrackerInfo {
  deviceId: string;
  vendor: string;
  model: string;
  firmwareVersion: string;
  protocolVersion: string;
  serialNumber?: string;
  deviceType?: DeviceType;
  vendorUrl?: string;
  productUrl?: string;
}

export interface SamplingRateSpec {
  supported: number[];
  default: number;
  current?: number;
}

export interface AccuracySpec {
  typical: number;
  best?: number;
}

export interface PrecisionSpec {
  typical: number;
  rms?: number;
}

export interface LatencySpec {
  average: number;
  maximum?: number;
}

export interface RangeSpec {
  min: number;
  max: number;
  optimal?: number;
}

export interface TrackingAreaSpec {
  horizontal: number;
  vertical: number;
}

export interface TrackingCapabilities {
  binocular: boolean;
  headTracking: boolean;
  gaze3D: boolean;
  samplingRate: SamplingRateSpec;
  accuracy: AccuracySpec;
  precision: PrecisionSpec;
  latency: LatencySpec;
  operatingDistance: RangeSpec;
  trackingArea: TrackingAreaSpec;
}

export interface DataCapabilities {
  gazePoint: boolean;
  eyeData: boolean;
  pupilDiameter: boolean;
  pupilPosition: boolean;
  eyeOpenness: boolean;
  eyeImages: boolean;
  gazeOrigin3D: boolean;
  gazeDirection3D: boolean;
  builtInFixationDetection: boolean;
  builtInSaccadeDetection: boolean;
  builtInBlinkDetection: boolean;
  deviceTimestamp: boolean;
  systemTimestamp: boolean;
  externalSync: boolean;
}

export type CalibrationType =
  | 'standard'
  | 'quick'
  | 'infant'
  | 'gaming'
  | 'accessibility'
  | 'automatic';

export interface CalibrationCapabilities {
  required: boolean;
  types: CalibrationType[];
  pointOptions: number[];
  defaultPoints: number;
  autoCalibration: boolean;
  profileManagement: boolean;
  qualityAssessment: boolean;
  adaptiveCalibration: boolean;
}

export interface AccessibilityCapabilities {
  dwellSelection: boolean;
  blinkInput: boolean;
  winkInput: boolean;
  switchEmulation: boolean;
  adaptiveDwellTime: boolean;
  adaptiveTargetSize: boolean;
  errorSmoothing: boolean;
  tremorCompensation: boolean;
  aacOptimizedMode: boolean;
  longSessionOptimization: boolean;
  fatigueDetection: boolean;
  breakReminder: boolean;
}

export type ConnectionType =
  | 'usb'
  | 'usb_c'
  | 'bluetooth'
  | 'wifi'
  | 'ethernet'
  | 'hdmi';

export type ApiProtocol =
  | 'native_sdk'
  | 'wia_standard'
  | 'tcp_ip'
  | 'websocket'
  | 'rest_api'
  | 'webrtc';

export interface ConnectivityCapabilities {
  connectionTypes: ConnectionType[];
  apiProtocols: ApiProtocol[];
  multiClient: boolean;
  remoteConnection: boolean;
  mobileConnection: boolean;
}

export type FeatureFlag =
  | 'GAZE_POINT'
  | 'BINOCULAR'
  | 'HEAD_TRACKING'
  | 'GAZE_3D'
  | 'PUPIL_DIAMETER'
  | 'PUPIL_POSITION'
  | 'EYE_OPENNESS'
  | 'EYE_IMAGES'
  | 'FIXATION_DETECTION'
  | 'SACCADE_DETECTION'
  | 'BLINK_DETECTION'
  | 'CALIBRATION'
  | 'AUTO_CALIBRATION'
  | 'CALIBRATION_PROFILES'
  | 'DWELL_SELECTION'
  | 'BLINK_INPUT'
  | 'AAC_MODE'
  | 'TREMOR_COMPENSATION'
  | 'DEVICE_TIMESTAMP'
  | 'EXTERNAL_SYNC'
  | 'MULTI_MONITOR'
  | 'SCREEN_RECORDING'
  | 'HEATMAP_GENERATION';

export interface EyeTrackerCapabilities {
  device: EyeTrackerInfo;
  tracking: TrackingCapabilities;
  data: DataCapabilities;
  calibration: CalibrationCapabilities;
  accessibility?: AccessibilityCapabilities;
  connectivity: ConnectivityCapabilities;
  supportedFeatures: FeatureFlag[];
}

// ============================================
// Calibration Types
// ============================================

export interface CalibrationPoint {
  x: number;
  y: number;
  index: number;
}

export interface PointCalibrationResult {
  pointIndex: number;
  position: Vector2D;
  accuracy: number;
  precision: number;
  valid: boolean;
}

export interface CalibrationResult {
  success: boolean;
  averageAccuracy?: number;
  averagePrecision?: number;
  pointResults?: PointCalibrationResult[];
  timestamp: number;
}

export interface CalibrationQuality {
  overall: 'excellent' | 'good' | 'fair' | 'poor';
  accuracy: number;
  precision: number;
  coverage: number;
}

// ============================================
// Tracker Status Types
// ============================================

export type TrackerState =
  | 'disconnected'
  | 'connecting'
  | 'connected'
  | 'calibrating'
  | 'tracking'
  | 'error';

export interface TrackerStatus {
  state: TrackerState;
  connected: boolean;
  tracking: boolean;
  calibrated: boolean;
  error?: string;
  batteryLevel?: number;
  userPresent?: boolean;
}

// ============================================
// Subscription Types
// ============================================

export interface Subscription {
  id: string;
  unsubscribe: () => void;
}

export type GazeCallback = (data: GazePoint) => void;
export type EventCallback<T extends GazeEvent = GazeEvent> = (event: T) => void;

// ============================================
// Configuration Types
// ============================================

export interface TrackerConfig {
  samplingRate?: number;
  smoothing?: boolean;
  smoothingFactor?: number;
  autoReconnect?: boolean;
  reconnectInterval?: number;
}

export interface FixationConfig {
  minDuration: number;
  maxDispersion: number;
  mergeDistance: number;
}

export interface DwellConfig {
  threshold: number;
  progressInterval: number;
  visualFeedback: boolean;
  cooldownPeriod: number;
}

export interface BlinkConfig {
  intentionalThreshold: number;
  doubleBlinkInterval: number;
  blinkAsInput: boolean;
}

// ============================================
// App Communication Types
// ============================================

export enum GazeAppMessageType {
  // Discovery
  APP_ANNOUNCE = 'app_announce',
  APP_QUERY = 'app_query',
  APP_RESPONSE = 'app_response',

  // Control
  CONTROL_REQUEST = 'control_request',
  CONTROL_GRANT = 'control_grant',
  CONTROL_DENY = 'control_deny',
  CONTROL_RELEASE = 'control_release',

  // Coordination
  PAUSE_TRACKING = 'pause_tracking',
  RESUME_TRACKING = 'resume_tracking',
  TARGET_SYNC = 'target_sync',
}

export interface GazeAppMessage {
  type: GazeAppMessageType;
  appId: string;
  timestamp: number;
  payload?: unknown;
}

export interface AppCapabilities {
  name: string;
  version: string;
  supportsControl: boolean;
  supportsDwell: boolean;
  supportsBlink: boolean;
  priority?: number;
}

export type TransportType = 'websocket' | 'ipc' | 'broadcast_channel';

export interface TransportConfig {
  type: TransportType;
  endpoint?: string;
}

// ============================================
// Dwell Types
// ============================================

export enum DwellFeedbackType {
  CIRCULAR_FILL = 'circular_fill',
  LINEAR_BAR = 'linear_bar',
  SHRINKING_RING = 'shrinking_ring',
  AUDIO_ONLY = 'audio_only',
  CUSTOM = 'custom',
}

export interface DwellState {
  active: boolean;
  target?: GazeTarget;
  startTime?: number;
  progress: number;
}
