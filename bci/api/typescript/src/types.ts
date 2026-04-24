/**
 * WIA BCI (Brain-Computer Interface) Standard
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

export enum DeviceType {
  EEG_HEADSET = 'eeg_headset',
  EEG_CAP = 'eeg_cap',
  FNIRS = 'fnirs',
  ECOG = 'ecog',
  IMPLANT = 'implant',
  HYBRID = 'hybrid',
}

export enum DeviceStatus {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  STREAMING = 'streaming',
  ERROR = 'error',
  CALIBRATING = 'calibrating',
}

export interface DeviceInfo {
  deviceId: string;
  type: DeviceType;
  manufacturer: string;
  model: string;
  serialNumber: string;
  firmwareVersion: string;
  hardwareVersion: string;
  batteryLevel?: number;
  status: DeviceStatus;
  capabilities: DeviceCapabilities;
  channels: ChannelInfo[];
}

export interface DeviceCapabilities {
  maxChannels: number;
  maxSamplingRate: number;
  supportedSamplingRates: number[];
  resolution: number;
  hasImpedanceCheck: boolean;
  hasAccelerometer: boolean;
  hasGyroscope: boolean;
  supportedProtocols: ConnectionProtocol[];
  wirelessRange?: number;
}

export interface ChannelInfo {
  channelId: string;
  name: string;
  position: ElectrodePosition;
  type: ChannelType;
  reference?: string;
  impedance?: number;
  enabled: boolean;
}

export enum ChannelType {
  EEG = 'eeg',
  EOG = 'eog',
  EMG = 'emg',
  ECG = 'ecg',
  ACCELEROMETER = 'accelerometer',
  GYROSCOPE = 'gyroscope',
  TRIGGER = 'trigger',
  REFERENCE = 'reference',
  GROUND = 'ground',
}

export interface ElectrodePosition {
  name: string;
  x: number;
  y: number;
  z?: number;
  system: ElectrodeSystem;
}

export enum ElectrodeSystem {
  STANDARD_10_20 = '10-20',
  STANDARD_10_10 = '10-10',
  STANDARD_10_5 = '10-5',
  CUSTOM = 'custom',
}

// ============================================================================
// Connection Types
// ============================================================================

export enum ConnectionProtocol {
  BLUETOOTH = 'bluetooth',
  BLUETOOTH_LE = 'bluetooth_le',
  USB = 'usb',
  WIFI = 'wifi',
  LSL = 'lsl',
  TCP = 'tcp',
  SERIAL = 'serial',
}

export interface ConnectionConfig {
  protocol: ConnectionProtocol;
  address?: string;
  port?: number;
  timeout?: number;
  autoReconnect?: boolean;
  maxReconnectAttempts?: number;
  reconnectInterval?: number;
}

export interface AcquisitionConfig {
  samplingRate: number;
  channels: string[];
  filters: FilterConfig[];
  referenceType: ReferenceType;
  notchFilter?: number;
  bufferSize?: number;
}

export enum ReferenceType {
  COMMON_AVERAGE = 'car',
  LINKED_EARS = 'linked_ears',
  SINGLE = 'single',
  BIPOLAR = 'bipolar',
  LAPLACIAN = 'laplacian',
}

export interface FilterConfig {
  type: FilterType;
  lowCutoff?: number;
  highCutoff?: number;
  order?: number;
  notchFrequency?: number;
}

export enum FilterType {
  LOWPASS = 'lowpass',
  HIGHPASS = 'highpass',
  BANDPASS = 'bandpass',
  BANDSTOP = 'bandstop',
  NOTCH = 'notch',
}

// ============================================================================
// Signal Types
// ============================================================================

export interface SignalPacket {
  packetId: string;
  deviceId: string;
  timestamp: Timestamp;
  sampleIndex: number;
  samples: number[][];
  channelNames: string[];
  samplingRate: number;
  quality: SignalQuality;
}

export interface SignalQuality {
  overall: number;
  channelQualities: Record<string, number>;
  impedances?: Record<string, number>;
  saturation: boolean;
  clipping: boolean;
}

export interface FrequencyBand {
  name: string;
  min: number;
  max: number;
}

export const FREQUENCY_BANDS: Record<string, FrequencyBand> = {
  DELTA: { name: 'Delta', min: 0.5, max: 4 },
  THETA: { name: 'Theta', min: 4, max: 8 },
  ALPHA: { name: 'Alpha', min: 8, max: 13 },
  BETA: { name: 'Beta', min: 13, max: 30 },
  GAMMA: { name: 'Gamma', min: 30, max: 100 },
};

export interface BandPowers {
  delta: number;
  theta: number;
  alpha: number;
  beta: number;
  gamma: number;
  total: number;
}

export interface PowerSpectrum {
  frequencies: number[];
  powers: number[];
  bandPowers: BandPowers;
}

export interface FeatureVector {
  timestamp: Timestamp;
  channelId: string;
  bandPowers: BandPowers;
  asymmetry?: Record<string, number>;
  coherence?: Record<string, number>;
  entropy?: number;
  hjorthParameters?: HjorthParameters;
}

export interface HjorthParameters {
  activity: number;
  mobility: number;
  complexity: number;
}

// ============================================================================
// Event Types
// ============================================================================

export enum EventType {
  SIGNAL = 'signal',
  MARKER = 'marker',
  CLASSIFICATION = 'classification',
  QUALITY = 'quality',
  ERROR = 'error',
  CONNECTION = 'connection',
  CALIBRATION = 'calibration',
}

export interface BaseEvent {
  type: EventType;
  timestamp: Timestamp;
  sessionId?: string;
}

export interface SignalEvent extends BaseEvent {
  type: EventType.SIGNAL;
  packet: SignalPacket;
}

export interface MarkerEvent extends BaseEvent {
  type: EventType.MARKER;
  markerId: string;
  label: string;
  value?: number;
  duration?: number;
}

export interface ClassificationEvent extends BaseEvent {
  type: EventType.CLASSIFICATION;
  classifierId: string;
  prediction: string;
  confidence: number;
  probabilities: Record<string, number>;
  features?: FeatureVector;
}

export interface QualityEvent extends BaseEvent {
  type: EventType.QUALITY;
  quality: SignalQuality;
  recommendations?: string[];
}

export interface ErrorEvent extends BaseEvent {
  type: EventType.ERROR;
  code: string;
  message: string;
  severity: ErrorSeverity;
  recoverable: boolean;
}

export enum ErrorSeverity {
  INFO = 'info',
  WARNING = 'warning',
  ERROR = 'error',
  CRITICAL = 'critical',
}

// ============================================================================
// Paradigm Types
// ============================================================================

export enum ParadigmType {
  P300 = 'p300',
  SSVEP = 'ssvep',
  MOTOR_IMAGERY = 'motor_imagery',
  ERROR_POTENTIAL = 'errp',
  HYBRID = 'hybrid',
  PASSIVE = 'passive',
}

export interface ParadigmConfig {
  type: ParadigmType;
  parameters: Record<string, unknown>;
  stimulusConfig?: StimulusConfig;
  classifierConfig?: ClassifierConfig;
}

export interface StimulusConfig {
  stimulusType: StimulusType;
  targets: Target[];
  duration: number;
  interval: number;
  randomization: boolean;
}

export enum StimulusType {
  VISUAL = 'visual',
  AUDITORY = 'auditory',
  TACTILE = 'tactile',
  MULTIMODAL = 'multimodal',
}

export interface Target {
  targetId: string;
  label: string;
  position?: { x: number; y: number };
  frequency?: number;
  color?: string;
  size?: number;
}

export interface ClassifierConfig {
  modelType: ModelType;
  features: string[];
  windowSize: number;
  stepSize: number;
  threshold: number;
  modelPath?: string;
}

export enum ModelType {
  LDA = 'lda',
  SVM = 'svm',
  CNN = 'cnn',
  RNN = 'rnn',
  TRANSFORMER = 'transformer',
  ENSEMBLE = 'ensemble',
}

// ============================================================================
// Session Types
// ============================================================================

export interface BciSession {
  sessionId: string;
  userId: string;
  deviceId: string;
  paradigm: ParadigmType;
  startTime: Timestamp;
  endTime?: Timestamp;
  status: SessionStatus;
  config: SessionConfig;
  metrics: SessionMetrics;
  recordings: RecordingInfo[];
}

export enum SessionStatus {
  INITIALIZING = 'initializing',
  CALIBRATING = 'calibrating',
  RUNNING = 'running',
  PAUSED = 'paused',
  COMPLETED = 'completed',
  ERROR = 'error',
}

export interface SessionConfig {
  device: ConnectionConfig;
  acquisition: AcquisitionConfig;
  paradigm: ParadigmConfig;
  recording: RecordingConfig;
}

export interface RecordingConfig {
  enabled: boolean;
  format: RecordingFormat;
  path?: string;
  maxSize?: number;
  compression?: boolean;
}

export enum RecordingFormat {
  EDF = 'edf',
  BDF = 'bdf',
  GDF = 'gdf',
  XDF = 'xdf',
  CSV = 'csv',
  BINARY = 'binary',
}

export interface RecordingInfo {
  recordingId: string;
  sessionId: string;
  format: RecordingFormat;
  path: string;
  size: number;
  duration: number;
  startTime: Timestamp;
  endTime: Timestamp;
  channels: string[];
  samplingRate: number;
  markers: MarkerEvent[];
}

export interface SessionMetrics {
  totalSamples: number;
  totalMarkers: number;
  classifications: number;
  accuracy: number;
  informationTransferRate: number;
  averageLatency: number;
  signalQuality: number;
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

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WiaBciConfig {
  device: DeviceType;
  connection: ConnectionConfig;
  acquisition?: AcquisitionConfig;
  paradigm?: ParadigmConfig;
  logging?: LogConfig;
}

export interface LogConfig {
  level: LogLevel;
  console: boolean;
  file?: string;
}

export enum LogLevel {
  DEBUG = 'debug',
  INFO = 'info',
  WARN = 'warn',
  ERROR = 'error',
}

export type EventHandler<T extends BaseEvent> = (event: T) => void | Promise<void>;
