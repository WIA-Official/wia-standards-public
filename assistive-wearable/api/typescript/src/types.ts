/**
 * WIA Assistive Wearable Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-assistive-wearable
 */

/**
 * Device category
 */
export enum DeviceCategory {
  HearingAid = 'hearing_aid',
  CochlearImplant = 'cochlear_implant',
  VisionAid = 'vision_aid',
  SmartGlasses = 'smart_glasses',
  Exoskeleton = 'exoskeleton',
  Prosthetic = 'prosthetic',
  OrthopedicBrace = 'orthopedic_brace',
  MobilityAid = 'mobility_aid',
  CommunicationDevice = 'communication_device',
  CognitiveAid = 'cognitive_aid',
  SensorySubstitution = 'sensory_substitution'
}

/**
 * Disability type
 */
export enum DisabilityType {
  HearingLoss = 'hearing_loss',
  VisionImpairment = 'vision_impairment',
  MotorDisability = 'motor_disability',
  CognitiveImpairment = 'cognitive_impairment',
  SpeechDisability = 'speech_disability',
  MultipleDisabilities = 'multiple'
}

/**
 * Assistance mode
 */
export enum AssistanceMode {
  Continuous = 'continuous',
  OnDemand = 'on_demand',
  Automatic = 'automatic',
  Scheduled = 'scheduled',
  Emergency = 'emergency'
}

/**
 * Connection type
 */
export enum ConnectionType {
  BluetoothLE = 'bluetooth_le',
  BluetoothClassic = 'bluetooth_classic',
  WiFi = 'wifi',
  NFC = 'nfc',
  USB = 'usb',
  Proprietary = 'proprietary'
}

/**
 * Battery status
 */
export interface BatteryStatus {
  /** Battery level (0-100) */
  level: number;
  /** Is charging */
  charging: boolean;
  /** Estimated time remaining in minutes */
  estimatedMinutes?: number;
  /** Battery health (0-100) */
  health: number;
  /** Cycles count */
  cycles?: number;
}

/**
 * Device info
 */
export interface DeviceInfo {
  /** Device ID */
  id: string;
  /** Device name */
  name: string;
  /** Device category */
  category: DeviceCategory;
  /** Manufacturer */
  manufacturer: string;
  /** Model */
  model: string;
  /** Serial number */
  serialNumber: string;
  /** Firmware version */
  firmwareVersion: string;
  /** Connection type */
  connectionType: ConnectionType;
  /** Supported disabilities */
  supportedDisabilities: DisabilityType[];
}

/**
 * User profile
 */
export interface UserProfile {
  /** User ID */
  id: string;
  /** User name */
  name: string;
  /** Primary disability */
  primaryDisability: DisabilityType;
  /** Secondary disabilities */
  secondaryDisabilities?: DisabilityType[];
  /** Preferred settings */
  preferences: UserPreferences;
  /** Emergency contacts */
  emergencyContacts: EmergencyContact[];
  /** Medical info */
  medicalInfo?: MedicalInfo;
}

/**
 * User preferences
 */
export interface UserPreferences {
  /** Language */
  language: string;
  /** Voice speed */
  voiceSpeed?: number;
  /** Voice pitch */
  voicePitch?: number;
  /** Font size */
  fontSize?: 'small' | 'medium' | 'large' | 'xlarge';
  /** High contrast mode */
  highContrast?: boolean;
  /** Haptic feedback */
  hapticFeedback?: boolean;
  /** Audio feedback */
  audioFeedback?: boolean;
  /** Custom settings */
  custom?: Record<string, unknown>;
}

/**
 * Emergency contact
 */
export interface EmergencyContact {
  /** Contact name */
  name: string;
  /** Relationship */
  relationship: string;
  /** Phone number */
  phone: string;
  /** Email */
  email?: string;
  /** Is primary */
  isPrimary: boolean;
}

/**
 * Medical info
 */
export interface MedicalInfo {
  /** Conditions */
  conditions: string[];
  /** Allergies */
  allergies?: string[];
  /** Medications */
  medications?: string[];
  /** Blood type */
  bloodType?: string;
  /** Doctor contact */
  doctorContact?: string;
  /** Notes */
  notes?: string;
}

/**
 * Hearing aid settings
 */
export interface HearingAidSettings {
  /** Master volume (0-100) */
  volume: number;
  /** Program/mode */
  program: string;
  /** Noise reduction level */
  noiseReduction: 'off' | 'low' | 'medium' | 'high';
  /** Directional microphone */
  directionalMic: boolean;
  /** Frequency bands */
  frequencyBands: FrequencyBand[];
  /** Telecoil enabled */
  telecoil: boolean;
  /** Bluetooth streaming volume */
  streamingVolume?: number;
}

/**
 * Frequency band
 */
export interface FrequencyBand {
  /** Center frequency in Hz */
  frequency: number;
  /** Gain in dB */
  gain: number;
  /** Compression ratio */
  compression: number;
}

/**
 * Vision aid settings
 */
export interface VisionAidSettings {
  /** Magnification level */
  magnification: number;
  /** Contrast adjustment */
  contrast: number;
  /** Color mode */
  colorMode: 'normal' | 'inverted' | 'grayscale' | 'high_contrast';
  /** Text-to-speech enabled */
  textToSpeech: boolean;
  /** Object recognition enabled */
  objectRecognition: boolean;
  /** Face recognition enabled */
  faceRecognition: boolean;
  /** Navigation assistance */
  navigationAssist: boolean;
}

/**
 * Exoskeleton settings
 */
export interface ExoskeletonSettings {
  /** Power assist level (0-100) */
  assistLevel: number;
  /** Mode */
  mode: 'walking' | 'standing' | 'sitting' | 'stairs' | 'custom';
  /** Gait speed */
  gaitSpeed: 'slow' | 'normal' | 'fast';
  /** Step height */
  stepHeight: number;
  /** Balance sensitivity */
  balanceSensitivity: number;
  /** Fall prevention level */
  fallPrevention: 'low' | 'medium' | 'high';
}

/**
 * Sensor data
 */
export interface SensorData {
  /** Sensor ID */
  sensorId: string;
  /** Sensor type */
  type: SensorType;
  /** Value */
  value: number | number[];
  /** Unit */
  unit: string;
  /** Timestamp */
  timestamp: Date;
  /** Quality (0-1) */
  quality: number;
}

/**
 * Sensor type
 */
export enum SensorType {
  Accelerometer = 'accelerometer',
  Gyroscope = 'gyroscope',
  HeartRate = 'heart_rate',
  SpO2 = 'spo2',
  Temperature = 'temperature',
  Pressure = 'pressure',
  EMG = 'emg',
  EEG = 'eeg',
  Proximity = 'proximity',
  Microphone = 'microphone',
  Camera = 'camera',
  GPS = 'gps'
}

/**
 * Health metric
 */
export interface HealthMetric {
  /** Metric type */
  type: string;
  /** Value */
  value: number;
  /** Unit */
  unit: string;
  /** Status */
  status: 'normal' | 'warning' | 'critical';
  /** Timestamp */
  timestamp: Date;
}

/**
 * Alert
 */
export interface Alert {
  /** Alert ID */
  id: string;
  /** Alert type */
  type: 'fall' | 'health' | 'device' | 'environment' | 'emergency';
  /** Severity */
  severity: 'info' | 'warning' | 'critical' | 'emergency';
  /** Message */
  message: string;
  /** Data */
  data?: Record<string, unknown>;
  /** Timestamp */
  timestamp: Date;
  /** Acknowledged */
  acknowledged: boolean;
}

/**
 * Accessibility feature
 */
export interface AccessibilityFeature {
  /** Feature ID */
  id: string;
  /** Feature name */
  name: string;
  /** Description */
  description: string;
  /** Supported disabilities */
  supportedDisabilities: DisabilityType[];
  /** Is enabled */
  enabled: boolean;
  /** Settings */
  settings?: Record<string, unknown>;
}

/**
 * Speech synthesis request
 */
export interface SpeechRequest {
  /** Text to speak */
  text: string;
  /** Language */
  language: string;
  /** Voice */
  voice?: string;
  /** Speed (0.5-2.0) */
  speed?: number;
  /** Pitch (0.5-2.0) */
  pitch?: number;
  /** Priority */
  priority: 'low' | 'normal' | 'high';
}

/**
 * Speech recognition result
 */
export interface SpeechRecognitionResult {
  /** Transcript */
  transcript: string;
  /** Confidence */
  confidence: number;
  /** Alternatives */
  alternatives?: string[];
  /** Is final */
  isFinal: boolean;
}

/**
 * Haptic feedback pattern
 */
export interface HapticPattern {
  /** Pattern ID */
  id: string;
  /** Pattern name */
  name: string;
  /** Meaning */
  meaning: string;
  /** Vibration sequence */
  sequence: VibrationStep[];
}

/**
 * Vibration step
 */
export interface VibrationStep {
  /** Duration in ms */
  duration: number;
  /** Intensity (0-100) */
  intensity: number;
  /** Pause after in ms */
  pauseAfter?: number;
}

/**
 * Gesture
 */
export interface Gesture {
  /** Gesture ID */
  id: string;
  /** Gesture name */
  name: string;
  /** Gesture type */
  type: 'tap' | 'double_tap' | 'swipe' | 'hold' | 'shake' | 'custom';
  /** Mapped action */
  action: string;
  /** Parameters */
  parameters?: Record<string, unknown>;
}

/**
 * Navigation instruction
 */
export interface NavigationInstruction {
  /** Instruction text */
  text: string;
  /** Distance in meters */
  distance: number;
  /** Direction */
  direction: 'forward' | 'left' | 'right' | 'back' | 'up' | 'down';
  /** Landmark */
  landmark?: string;
  /** Hazard warning */
  hazard?: string;
}

/**
 * Device status
 */
export interface DeviceStatus {
  /** Device ID */
  deviceId: string;
  /** Is connected */
  connected: boolean;
  /** Battery status */
  battery: BatteryStatus;
  /** Current mode */
  mode: AssistanceMode;
  /** Active features */
  activeFeatures: string[];
  /** Errors */
  errors?: string[];
  /** Last sync */
  lastSync: Date;
}

/**
 * SDK configuration
 */
export interface AssistiveWearableConfig {
  /** Device info */
  device: DeviceInfo;
  /** User profile */
  userProfile?: UserProfile;
  /** Auto-connect */
  autoConnect: boolean;
  /** Sync interval in seconds */
  syncInterval: number;
  /** Enable emergency detection */
  emergencyDetection: boolean;
  /** API endpoint */
  apiEndpoint?: string;
}

/**
 * Certification level
 */
export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

/**
 * Compliance report
 */
export interface ComplianceReport {
  /** Standard */
  standard: 'WIA-ASSISTIVE-WEARABLE';
  /** Test date */
  testDate: string;
  /** Configuration */
  config: AssistiveWearableConfig;
  /** Target level */
  targetLevel: CertificationLevel;
  /** Test results */
  tests: TestResult[];
  /** Overall pass */
  passed: boolean;
  /** Achieved level */
  achievedLevel?: CertificationLevel;
}

/**
 * Test result
 */
export interface TestResult {
  /** Test name */
  testName: string;
  /** Passed */
  passed: boolean;
  /** Notes */
  notes?: string;
}
