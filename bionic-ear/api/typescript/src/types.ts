/**
 * WIA-AUG-009: Bionic Ear - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Auditory Bionics Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Device Classification Types
// ============================================================================

/**
 * Bionic ear device types
 */
export type DeviceType =
  | 'COCHLEAR_IMPLANT'
  | 'ABI'
  | 'MIDDLE_EAR'
  | 'HYBRID'
  | 'BONE_CONDUCTION'
  | 'EXTERNAL_PROCESSOR';

/**
 * Device category based on complexity
 */
export type DeviceCategory = 'Basic' | 'Standard' | 'Advanced' | 'Premium';

/**
 * Hearing loss type
 */
export type HearingLossType = 'sensorineural' | 'conductive' | 'mixed' | 'neural';

/**
 * Hearing loss degree
 */
export type HearingLossDegree = 'moderate' | 'severe' | 'profound';

/**
 * Stimulation type
 */
export type StimulationType = 'electrical' | 'mechanical' | 'vibrational' | 'hybrid';

/**
 * Device classification input
 */
export interface DeviceClassificationInput {
  /** Type of device */
  type: DeviceType;

  /** Number of electrodes (if applicable) */
  electrodes?: number;

  /** Sound processing strategy */
  processingStrategy: ProcessingStrategy;

  /** Advanced features */
  features: string[];
}

/**
 * Device classification result
 */
export interface DeviceClassification {
  /** Device type */
  type: DeviceType;

  /** Hearing loss type */
  hearingLossType: HearingLossType;

  /** Hearing loss degree */
  hearingLossDegree: HearingLossDegree;

  /** Number of electrode contacts */
  electrodeCount?: number;

  /** Stimulation type */
  stimulationType: StimulationType;

  /** Complexity category */
  category: DeviceCategory;

  /** Complexity score */
  complexityScore: number;

  /** Recommended processing strategies */
  recommendedStrategies: ProcessingStrategy[];
}

// ============================================================================
// Processing Strategy Types
// ============================================================================

/**
 * Sound processing strategies
 */
export type ProcessingStrategy =
  | 'SPEAK'
  | 'CIS'
  | 'ACE'
  | 'FSP'
  | 'HDCIS'
  | 'MP3000';

/**
 * SPEAK strategy configuration
 */
export interface SpeakStrategy {
  /** Number of filter banks */
  filterBanks: number; // 20-22

  /** Number of selected channels per frame */
  selectedChannels: number; // 4-10

  /** Frame rate (Hz) */
  frameRate: number; // 180-300

  /** Pulse width (microseconds) */
  pulseWidth: number; // 50-400

  /** Amplitude mapping */
  amplitudeMapping: 'logarithmic' | 'linear';
}

/**
 * CIS strategy configuration
 */
export interface CISStrategy {
  /** Number of channels */
  channels: number; // 4-22

  /** Stimulation rate (Hz per channel) */
  stimulationRate: number; // 800-2400

  /** Pulse width (microseconds) */
  pulseWidth: number; // 10-50

  /** Compression function */
  compressionFunction: 'logarithmic' | 'linear' | 'polynomial';
}

/**
 * ACE strategy configuration
 */
export interface ACEStrategy {
  /** Total number of channels */
  totalChannels: number; // 8-22

  /** Active channels per frame */
  activeChannels: number; // 8-12

  /** Stimulation rate (Hz per channel) */
  stimulationRate: number; // 250-2400

  /** Frame rate (Hz) */
  frameRate: number; // 500-1800

  /** Pulse width (microseconds) */
  pulseWidth: number; // 20-50

  /** Dynamic range (dB) */
  dynamicRange: number; // 40-60
}

/**
 * FSP strategy configuration
 */
export interface FSPStrategy {
  /** Total number of channels */
  channels: number; // 12-22

  /** Fine structure channels (low freq) */
  fineStructureChannels: number; // 1-3

  /** Envelope channels (mid/high freq) */
  envelopeChannels: number;

  /** Fine structure frequency range */
  fineStructureRange: { low: number; high: number }; // 125-1000 Hz

  /** Envelope frequency range */
  envelopeRange: { low: number; high: number }; // 1000-8000 Hz

  /** Stimulation rate (Hz) */
  stimulationRate: number; // 500-5000
}

/**
 * Processing strategy config union type
 */
export type ProcessingConfig =
  | SpeakStrategy
  | CISStrategy
  | ACEStrategy
  | FSPStrategy;

// ============================================================================
// Electrode Array Types
// ============================================================================

/**
 * Electrode array configurations
 */
export type ElectrodeConfig =
  | 'PERIMODIOLAR'
  | 'LATERAL_WALL'
  | 'HYBRID'
  | 'STRAIGHT';

/**
 * Electrode material
 */
export type ElectrodeMaterial = 'Platinum-Iridium' | 'Platinum' | 'Gold';

/**
 * Electrode array specification
 */
export interface ElectrodeArray {
  /** Array type */
  type: ElectrodeConfig;

  /** Number of contacts */
  contactCount: number;

  /** Spacing between contacts (mm) */
  spacing: number;

  /** Insertion depth (mm) */
  insertionDepth: number;

  /** Electrode material */
  material: ElectrodeMaterial;

  /** Impedance range (kΩ) */
  impedanceRange: { min: number; max: number };

  /** Contact area (mm²) */
  contactArea: number;
}

/**
 * Current steering configuration
 */
export interface CurrentSteering {
  /** Current steering enabled */
  enabled: boolean;

  /** Total virtual channels */
  virtualChannels: number;

  /** Physical electrodes */
  physicalElectrodes: number;

  /** Steering ratio (0-1) */
  steeringRatio: number;

  /** Effective spectral resolution */
  spectralResolution: number;
}

// ============================================================================
// Frequency Mapping Types
// ============================================================================

/**
 * Frequency band allocation
 */
export interface FrequencyBand {
  /** Low frequency (Hz) */
  low: number;

  /** Mid frequency (Hz) */
  mid: number;

  /** High frequency (Hz) */
  high: number;
}

/**
 * Frequency map for electrode
 */
export interface FrequencyMap {
  /** Electrode number */
  electrodeNumber: number;

  /** Center frequency (Hz) */
  centerFrequency: number;

  /** Frequency range */
  frequencyRange: { low: number; high: number };

  /** Characteristic frequency (cochlear position) */
  characteristicFrequency: number;

  /** Gain adjustment (dB) */
  gainAdjustment: number;
}

/**
 * Frequency customization
 */
export interface FrequencyCustomization {
  /** Patient ID */
  patientId: string;

  /** Device ID */
  deviceId: string;

  /** Base frequency map */
  baseMap: FrequencyMap[];

  /** Custom adjustments */
  adjustments: FrequencyAdjustment[];

  /** Validation date */
  validatedDate: Date;
}

/**
 * Frequency adjustment
 */
export interface FrequencyAdjustment {
  /** Electrode number */
  electrode: number;

  /** Frequency shift (Hz) */
  frequencyShift: number;

  /** Gain change (dB) */
  gainChange: number;

  /** Reason for adjustment */
  reason: 'pitch_matching' | 'comfort' | 'speech_clarity' | 'music_quality';
}

// ============================================================================
// Speech Recognition Types
// ============================================================================

/**
 * Noise reduction mode
 */
export type NoiseReductionMode = 'off' | 'low' | 'medium' | 'high' | 'adaptive';

/**
 * Directional microphone mode
 */
export type DirectionalMode =
  | 'omnidirectional'
  | 'narrow'
  | 'medium'
  | 'wide'
  | 'adaptive';

/**
 * Noise reduction configuration
 */
export interface NoiseReduction {
  /** Noise reduction mode */
  mode: NoiseReductionMode;

  /** SNR threshold (dB) */
  snrThreshold: number;

  /** Reduction strength (0-1) */
  reductionStrength: number;

  /** Wind noise suppression */
  windNoiseSuppression: boolean;

  /** Transient noise reduction */
  transientNoiseReduction: boolean;
}

/**
 * Directional microphone configuration
 */
export interface DirectionalMicrophone {
  /** Directional mode */
  mode: DirectionalMode;

  /** Beam width (degrees) */
  beamWidth: number;

  /** Front-to-back ratio (dB) */
  frontBackRatio: number;

  /** Adaptive speed (ms) */
  adaptiveSpeed: number;

  /** Wind protection */
  windProtection: boolean;
}

/**
 * Speech enhancement configuration
 */
export interface SpeechEnhancement {
  /** Voiced/unvoiced detection */
  voicedUnvoicedDetection: boolean;

  /** Fundamental frequency tracking */
  fundamentalFrequencyTracking: boolean;

  /** Formant enhancement */
  formantEnhancement: boolean;

  /** Consonant emphasis (dB boost) */
  consonantEmphasis: number;

  /** Pitch shifting (semitones) */
  pitchShifting: number;
}

/**
 * Speech recognition metrics
 */
export interface SpeechRecognitionMetrics {
  /** Quiet conditions (%) */
  quietSentences: number;
  quietWords: number;
  quietPhonemes: number;

  /** Noise conditions (%) */
  noiseSentencesPlus10dB: number;
  noiseSentencesPlus5dB: number;
  noiseSentences0dB: number;

  /** SNR for 50% intelligibility (dB) */
  snr50: number;

  /** Adaptive ratio improvement */
  adaptiveRatio: number;

  /** Bilateral benefit (%) */
  bilateralBenefit: number;
}

// ============================================================================
// Music Perception Types
// ============================================================================

/**
 * Pitch refinement configuration
 */
export interface PitchRefinement {
  /** Pitch refinement enabled */
  enabled: boolean;

  /** Virtual channels (current steering) */
  virtualChannels: number;

  /** Fine structure coding */
  fineStructureCoding: boolean;

  /** Fundamental frequency enhancement */
  fundamentalFrequencyEnhancement: boolean;

  /** Harmonic preservation */
  harmonicPreservation: boolean;
}

/**
 * Harmonic enhancement configuration
 */
export interface HarmonicEnhancement {
  /** Number of harmonics (1-8) */
  harmonicCount: number;

  /** Fundamental boost (dB) */
  fundamentalBoost: number;

  /** Harmonic boost (dB) */
  harmonicBoost: number;

  /** Inharmonicity reduction */
  inharmonicityReduction: boolean;
}

/**
 * Music program configuration
 */
export interface MusicProgram {
  /** Program name */
  name: string;

  /** Compression ratio */
  compressionRatio: number; // 1.5-3:1

  /** Input dynamic range (dB) */
  inputDynamicRange: number; // 60-80

  /** Processing strategy */
  processingStrategy: ProcessingStrategy;

  /** Microphone type */
  microphone: 'music' | 'speech';

  /** Pitch refinement */
  pitchRefinement: PitchRefinement;

  /** Harmonic enhancement */
  harmonicEnhancement: HarmonicEnhancement;

  /** Tempo tracking */
  tempoTracking: boolean;
}

/**
 * Music perception assessment
 */
export interface MusicPerceptionAssessment {
  /** Pitch discrimination (semitones JND) */
  pitchDiscrimination: number;

  /** Melody recognition (% correct) */
  melodyRecognition: number;

  /** Pitch direction (% correct) */
  pitchDirection: number;

  /** Instrument recognition (% correct) */
  instrumentRecognition: number;

  /** Timbre quality (1-10) */
  timbreQuality: number;

  /** Rhythm recognition (% correct) */
  rhythmRecognition: number;

  /** Beat tracking (% accurate) */
  beatTracking: number;

  /** Music enjoyment (1-10) */
  musicEnjoyment: number;

  /** Naturalness (1-10) */
  naturalness: number;
}

// ============================================================================
// Bilateral Synchronization Types
// ============================================================================

/**
 * Bilateral sync mode
 */
export type BilateralSyncMode = 'independent' | 'linked' | 'coordinated';

/**
 * Bilateral synchronization configuration
 */
export interface BilateralSync {
  /** Left device ID */
  leftDevice: string;

  /** Right device ID */
  rightDevice: string;

  /** Synchronization mode */
  syncMode: BilateralSyncMode;

  /** Timing accuracy (microseconds) */
  timingAccuracy: number;

  /** Interaural level difference preservation */
  interauralLevelDifference: boolean;

  /** Interaural time difference preservation */
  interauralTimeDifference: boolean;

  /** Bilateral beamforming */
  bilateralBeamforming: boolean;
}

/**
 * Interaural cues
 */
export interface InterauralCues {
  /** ILD enabled */
  ILD: boolean;

  /** ITD enabled */
  ITD: boolean;

  /** Maximum ITD (microseconds) */
  maxITD: number; // ~700 µs natural

  /** Maximum ILD (dB) */
  maxILD: number; // ~20 dB natural
}

/**
 * Sound localization result
 */
export interface SoundLocation {
  /** Azimuth (degrees, -90 to +90) */
  azimuth: number;

  /** Elevation (degrees, -90 to +90) */
  elevation: number;

  /** Confidence (0-1) */
  confidence?: number;
}

/**
 * Bilateral program link configuration
 */
export interface BilateralProgramLink {
  /** Program sync */
  programSync: boolean;

  /** Volume sync */
  volumeSync: boolean;

  /** Sensitivity sync */
  sensitivitySync: boolean;

  /** Mixer sync */
  mixerSync: boolean;

  /** Streaming mode */
  streamingMode: 'independent' | 'linked' | 'stereo';
}

// ============================================================================
// Tinnitus Suppression Types
// ============================================================================

/**
 * Tinnitus quality
 */
export type TinnitusQuality = 'tonal' | 'noise' | 'pulsatile';

/**
 * Tinnitus stimulation pattern
 */
export type TinnitusPattern = 'continuous' | 'pulsed' | 'modulated';

/**
 * Tinnitus stimulation configuration
 */
export interface TinnitusStimulation {
  /** Tinnitus suppression enabled */
  enabled: boolean;

  /** Matched frequency (Hz) */
  frequency: number;

  /** Stimulation level (dB) */
  level: number;

  /** Stimulation pattern */
  pattern: TinnitusPattern;

  /** Modulation frequency (Hz, if modulated) */
  modulationFrequency?: number;

  /** Active electrodes */
  electrodes: number[];
}

/**
 * Tinnitus masking configuration
 */
export interface TinnitusMasking {
  /** Masking type */
  type: 'total' | 'partial' | 'residual_inhibition';

  /** Noise configuration */
  noise: {
    type: 'white' | 'pink' | 'brown' | 'bandpass';
    centerFrequency?: number; // Hz
    bandwidth?: number; // Hz
    level: number; // dB
  };

  /** Masking schedule */
  schedule: {
    continuous: boolean;
    onDuration?: number; // minutes
    offDuration?: number; // minutes
  };
}

/**
 * Tinnitus suppression degree
 */
export type TinnitusSuppressionDegree =
  | 'complete'
  | 'significant'
  | 'moderate'
  | 'minimal'
  | 'none';

/**
 * Tinnitus assessment
 */
export interface TinnitusAssessment {
  /** Tinnitus Handicap Inventory (0-100) */
  THI: number;

  /** Visual Analog Scale (0-10) */
  VAS: number;

  /** Annoyance level (0-10) */
  annoyance: number;

  /** Tinnitus pitch (Hz) */
  pitch: number;

  /** Tinnitus loudness (dB SL) */
  loudness: number;

  /** Minimum masking level (dB SPL) */
  minimumMaskingLevel: number;

  /** Suppression degree */
  suppressionDegree: TinnitusSuppressionDegree;

  /** Improvement from baseline (%) */
  improvementPercent: number;
}

// ============================================================================
// Environmental Sound Classification Types
// ============================================================================

/**
 * Acoustic scene types
 */
export type AcousticScene =
  | 'quiet'
  | 'speech_in_quiet'
  | 'speech_in_noise'
  | 'noise'
  | 'music'
  | 'outdoor'
  | 'traffic'
  | 'restaurant'
  | 'conference';

/**
 * Scene classifier algorithm
 */
export type ClassifierAlgorithm = 'rule_based' | 'ml_based' | 'hybrid';

/**
 * Scene classifier configuration
 */
export interface SceneClassifier {
  /** Classification algorithm */
  algorithm: ClassifierAlgorithm;

  /** Update rate (Hz) */
  updateRate: number;

  /** Confidence threshold (0-1) */
  confidenceThreshold: number;

  /** Adaptation speed */
  adaptationSpeed: 'slow' | 'medium' | 'fast';
}

/**
 * Scene classification result
 */
export interface SceneClassification {
  /** Detected scene */
  scene: AcousticScene;

  /** Confidence (0-1) */
  confidence: number;

  /** Extracted features */
  features: SceneFeatures;
}

/**
 * Scene features for classification
 */
export interface SceneFeatures {
  /** RMS level (dB) */
  rmsLevel: number;

  /** Peak level (dB) */
  peakLevel: number;

  /** Dynamic range (dB) */
  dynamicRange: number;

  /** Spectral centroid (Hz) */
  spectralCentroid: number;

  /** Spectral rolloff (Hz) */
  spectralRolloff: number;

  /** Spectral flux */
  spectralFlux: number;

  /** Zero-crossing rate */
  zeroCrossingRate: number;

  /** Temporal centroid */
  temporalCentroid: number;

  /** Speech probability (0-1) */
  speechProbability: number;

  /** Voicing rate (0-1) */
  voicingRate: number;

  /** Noise level (dB) */
  noiseLevel: number;

  /** Reverberation time (ms) */
  reverberation: number;
}

/**
 * Program configuration
 */
export interface ProgramConfig {
  /** Processing strategy */
  strategy: ProcessingStrategy;

  /** Noise reduction level */
  noiseReduction: NoiseReductionMode;

  /** Directionality */
  directionality: DirectionalMode;

  /** Compression ratio */
  compression: number;

  /** Gain adjustment (dB) */
  gain: number;
}

// ============================================================================
// Wireless Connectivity Types
// ============================================================================

/**
 * Bluetooth version
 */
export type BluetoothVersion = '5.0' | '5.1' | '5.2' | '5.3';

/**
 * Audio codec
 */
export type AudioCodec = 'SBC' | 'AAC' | 'LC3';

/**
 * Bluetooth configuration
 */
export interface BluetoothConfig {
  /** Bluetooth version */
  version: BluetoothVersion;

  /** LE Audio support */
  leAudio: boolean;

  /** Audio codec */
  codec: AudioCodec;

  /** Target latency (ms) */
  latency: number;

  /** Multipoint connection */
  multipoint: boolean;

  /** Pairing method */
  pairing: 'standard' | 'secure' | 'nfc';

  /** Audio sharing */
  audioSharing: boolean;
}

/**
 * Telecoil configuration
 */
export interface TelecoilConfig {
  /** Telecoil enabled */
  enabled: boolean;

  /** Sensitivity (mA/m) */
  sensitivity: number;

  /** Frequency response */
  frequencyResponse: { low: number; high: number }; // Hz

  /** Mix with microphone */
  mixWithMicrophone: boolean;

  /** Microphone mix ratio (0-1) */
  microphoneMixRatio: number;

  /** Automatic activation */
  automaticTelecoilActivation: boolean;
}

/**
 * Streaming quality preset
 */
export interface StreamingQuality {
  /** Sample rate (Hz) */
  sampleRate: number;

  /** Bit depth */
  bitDepth: number;

  /** Codec */
  codec: string;

  /** Bitrate (kbps) */
  bitrate: number;

  /** Latency (ms) */
  latency: number;

  /** Jitter buffer (ms) */
  jitterBuffer: number;

  /** Packet loss concealment */
  packetLossConcealment: boolean;
}

/**
 * Remote capability
 */
export type RemoteCapability =
  | 'map_adjustment'
  | 'volume_change'
  | 'program_switch'
  | 'firmware_update'
  | 'diagnostics'
  | 'hearing_test';

/**
 * Remote programming configuration
 */
export interface RemoteProgramming {
  /** Remote programming enabled */
  enabled: boolean;

  /** Connection type */
  connection: 'bluetooth' | 'wifi' | 'cellular';

  /** Security level */
  security: 'encrypted' | 'secure_tunnel' | 'vpn';

  /** Available capabilities */
  capabilities: RemoteCapability[];

  /** Requires audiologist approval */
  requiresAudiologist: boolean;
}

// ============================================================================
// Power and Battery Types
// ============================================================================

/**
 * Battery type
 */
export type BatteryType =
  | 'disposable_zinc_air'
  | 'rechargeable_lithium'
  | 'rechargeable_silver_zinc';

/**
 * Battery size (hearing aid standard sizes)
 */
export type BatterySize = '10' | '13' | '312' | '675';

/**
 * Battery specification
 */
export interface BatterySpecification {
  /** Battery chemistry */
  type: BatteryType;

  /** Capacity (mAh) */
  capacity: number;

  /** Voltage (V) */
  voltage: number;

  /** Runtime (hours) */
  runtime: number;

  /** Charging time (hours, rechargeable only) */
  chargingTime?: number;

  /** Charge cycles (rechargeable only) */
  cycles?: number;

  /** Battery size */
  size: BatterySize;
}

/**
 * Battery state
 */
export interface BatteryState {
  /** Current charge level (0-100%) */
  level: number;

  /** Voltage (V) */
  voltage: number;

  /** Temperature (Celsius) */
  temperature: number;

  /** Charging status */
  charging: boolean;

  /** Cycle count (rechargeable) */
  cycleCount?: number;

  /** Estimated runtime (minutes) */
  estimatedRuntime: number;

  /** Health status */
  health: 'excellent' | 'good' | 'fair' | 'replace';
}

/**
 * Power mode
 */
export type PowerMode = 'max_performance' | 'balanced' | 'economy' | 'custom';

/**
 * Power saving feature
 */
export type PowerSavingFeature =
  | 'reduce_sampling_rate'
  | 'reduce_channels'
  | 'disable_wireless'
  | 'reduce_display_brightness'
  | 'limit_peak_current';

/**
 * Power management configuration
 */
export interface PowerManagement {
  /** Power mode */
  mode: PowerMode;

  /** Current battery level (0-100%) */
  batteryLevel: number;

  /** Low power threshold (%) */
  lowPowerThreshold: number;

  /** Critical threshold (%) */
  criticalThreshold: number;

  /** Enabled power saving features */
  powerSavingFeatures: PowerSavingFeature[];
}

// ============================================================================
// Device MAP Types
// ============================================================================

/**
 * Electrode settings for MAP
 */
export interface ElectrodeSettings {
  /** Electrode number */
  electrode: number;

  /** Electrode active */
  active: boolean;

  /** Threshold level (T-level) */
  tLevel: number;

  /** Comfortable level (C-level) */
  cLevel: number;

  /** Individual gain (dB) */
  gain: number;

  /** Frequency allocation */
  frequencyAllocation: { low: number; high: number }; // Hz
}

/**
 * Device program
 */
export interface Program {
  /** Program name */
  name: string;

  /** Processing strategy */
  strategy: ProcessingStrategy;

  /** Microphone sensitivity */
  sensitivity: number;

  /** Noise reduction mode */
  noiseReduction: NoiseReductionMode;

  /** Directionality mode */
  directionality: DirectionalMode;
}

/**
 * Device MAP (programming)
 */
export interface DeviceMAP {
  /** Patient ID (anonymized) */
  patientId: string;

  /** Device ID */
  deviceId: string;

  /** Creation date */
  createdDate: Date;

  /** Audiologist ID */
  audiologist: string;

  /** Processing strategy */
  processingStrategy: ProcessingStrategy;

  /** Stimulation rate (Hz) */
  stimulationRate: number;

  /** Pulse width (µs) */
  pulseWidth: number;

  /** Electrode settings */
  electrodes: ElectrodeSettings[];

  /** Global sensitivity */
  sensitivity: number;

  /** Overall volume (0-255) */
  volume: number;

  /** Compression ratio */
  compression: number;

  /** Noise reduction mode */
  noiseReduction: NoiseReductionMode;

  /** Directionality mode */
  directionality: DirectionalMode;

  /** Frequency map */
  frequencyMap: FrequencyMap[];

  /** Available programs */
  programs: Program[];

  /** Active program index */
  activeProgram: number;
}

/**
 * Threshold measurement
 */
export interface ThresholdMeasurement {
  /** Electrode number */
  electrode: number;

  /** T-level (threshold) */
  tLevel: number;

  /** C-level (comfortable) */
  cLevel: number;

  /** Dynamic range */
  dynamicRange: number;

  /** Measurement method */
  measurementMethod: 'behavioral' | 'objective' | 'hybrid';

  /** Confidence (0-1) */
  confidence: number;
}

// ============================================================================
// Patient and Assessment Types
// ============================================================================

/**
 * Patient information (anonymized)
 */
export interface PatientInfo {
  /** Patient ID (anonymized) */
  id: string;

  /** Age */
  age: number;

  /** Hearing loss type */
  hearingLossType: HearingLossType;

  /** Hearing loss degree */
  hearingLossDegree: HearingLossDegree;

  /** Duration of hearing loss */
  duration: 'congenital' | 'prelingual' | 'postlingual' | 'progressive';

  /** Residual hearing */
  residualHearing: 'none' | 'minimal' | 'moderate' | 'significant';

  /** Cause of hearing loss */
  cause?: 'genetic' | 'infectious' | 'trauma' | 'noise' | 'age' | 'unknown';

  /** Implant experience */
  implantExperience: 'new' | 'experienced' | 'upgrade';

  /** Primary use cases */
  primaryUse: string[];
}

/**
 * Device information
 */
export interface BionicEarDevice {
  /** Device ID */
  id: string;

  /** Device type */
  type: DeviceType;

  /** Manufacturer */
  manufacturer: string;

  /** Model name */
  model: string;

  /** Serial number */
  serialNumber: string;

  /** Firmware version */
  firmwareVersion: string;

  /** Manufacturing date */
  manufactureDate: Date;

  /** Implant date */
  implantDate?: Date;

  /** Activation date */
  activationDate?: Date;

  /** Processing strategy */
  processingStrategy: ProcessingStrategy;

  /** Electrode configuration */
  electrodeConfig: ElectrodeConfig;

  /** Number of electrodes */
  electrodeCount: number;

  /** Battery specification */
  battery: BatterySpecification;
}

/**
 * Performance rating
 */
export type PerformanceRating = 'excellent' | 'good' | 'fair' | 'needs_improvement';

/**
 * Compatibility assessment
 */
export interface CompatibilityAssessment {
  /** Overall compatibility score (0-100) */
  compatibility: number;

  /** Recommended processing strategy */
  recommendedStrategy: ProcessingStrategy;

  /** Expected speech recognition score (0-100) */
  speechRecognitionScore: number;

  /** Training time estimate (hours) */
  estimatedTrainingTime: number;

  /** Recommendations */
  recommendations: string[];

  /** Potential challenges */
  challenges: string[];
}

/**
 * Hearing performance outcome
 */
export interface HearingPerformanceOutcome {
  /** Speech recognition in quiet (%) */
  speechRecognitionQuiet: number;

  /** Speech recognition in noise +10dB SNR (%) */
  speechRecognitionNoise10dB: number;

  /** Speech recognition in noise +5dB SNR (%) */
  speechRecognitionNoise5dB: number;

  /** SNR-50 (dB) */
  snr50: number;

  /** Quality of life score (0-100) */
  qualityOfLife: number;

  /** Usage hours per day */
  usageHoursPerDay: number;

  /** Patient satisfaction (0-10) */
  satisfaction: number;

  /** Performance rating */
  performanceRating: PerformanceRating;
}

/**
 * Comprehensive device report
 */
export interface DeviceReport {
  /** Report ID */
  id: string;

  /** Generation timestamp */
  generatedAt: Date;

  /** Device information */
  device: BionicEarDevice;

  /** Patient information */
  patient: PatientInfo;

  /** Device classification */
  classification: DeviceClassification;

  /** Current MAP */
  currentMAP: DeviceMAP;

  /** Battery state */
  battery: BatteryState;

  /** Performance outcome */
  performance: HearingPerformanceOutcome;

  /** Music perception (if assessed) */
  musicPerception?: MusicPerceptionAssessment;

  /** Tinnitus assessment (if applicable) */
  tinnitus?: TinnitusAssessment;

  /** Overall status */
  status: 'excellent' | 'good' | 'fair' | 'needs_service';

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Bionic ear constants
 */
export const BIONIC_EAR_CONSTANTS = {
  /** Frequency specifications (Hz) */
  FREQUENCY: {
    MIN: 125,
    MAX: 8000,
    EXTENDED_MAX: 12000,
    LOW_FREQ_RANGE: { low: 125, high: 500 },
    MID_FREQ_RANGE: { low: 500, high: 2000 },
    HIGH_FREQ_RANGE: { low: 2000, high: 8000 },
  },

  /** Dynamic range (dB) */
  DYNAMIC_RANGE: {
    MIN: 40,
    TYPICAL: 50,
    MAX: 80,
  },

  /** Battery specifications */
  BATTERY: {
    MIN_RUNTIME_HOURS: 12,
    CRITICAL_LEVEL: 20,
    WARNING_LEVEL: 30,
  },

  /** Performance thresholds */
  PERFORMANCE: {
    MIN_SPEECH_RECOGNITION_QUIET: 70, // %
    TARGET_SPEECH_RECOGNITION_QUIET: 85, // %
    EXCELLENT_SPEECH_RECOGNITION_QUIET: 90, // %
    MAX_PROCESSING_LATENCY_MS: 10,
  },

  /** Electrode specifications */
  ELECTRODE: {
    COCHLEAR_MIN: 12,
    COCHLEAR_MAX: 24,
    ABI_MIN: 8,
    ABI_MAX: 21,
    HYBRID_MIN: 6,
    HYBRID_MAX: 16,
    IMPEDANCE_MIN_KOHM: 2,
    IMPEDANCE_MAX_KOHM: 25,
  },

  /** Stimulation specifications */
  STIMULATION: {
    MAX_CURRENT_MA: 1.75, // per electrode
    MAX_TOTAL_CURRENT_MA: 10,
    MAX_CHARGE_NC: 50, // per phase
    PULSE_WIDTH_MIN_US: 10,
    PULSE_WIDTH_MAX_US: 400,
  },

  /** Speech recognition benchmarks (%) */
  SPEECH: {
    EXCELLENT: 90,
    GOOD: 75,
    FAIR: 60,
    NEEDS_OPTIMIZATION: 50,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Bionic ear error codes
 */
export enum BionicEarErrorCode {
  CLASSIFICATION_INVALID_INPUT = 'BE001',
  CONFIGURATION_FAILED = 'BE002',
  FREQUENCY_MAPPING_ERROR = 'BE003',
  SPEECH_OPTIMIZATION_FAILED = 'BE004',
  MUSIC_ENHANCEMENT_FAILED = 'BE005',
  BILATERAL_SYNC_FAILED = 'BE006',
  TINNITUS_CONFIG_ERROR = 'BE007',
  BATTERY_CRITICAL = 'BE008',
  IMPEDANCE_OUT_OF_RANGE = 'BE009',
  SAFETY_LIMIT_EXCEEDED = 'BE010',
  MAP_INVALID = 'BE011',
  DEVICE_NOT_FOUND = 'BE012',
}

/**
 * Bionic ear error class
 */
export class BionicEarError extends Error {
  constructor(
    public code: BionicEarErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BionicEarError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  DeviceType,
  DeviceCategory,
  HearingLossType,
  HearingLossDegree,
  StimulationType,
  DeviceClassificationInput,
  DeviceClassification,
  ProcessingStrategy,
  SpeakStrategy,
  CISStrategy,
  ACEStrategy,
  FSPStrategy,
  ProcessingConfig,
  ElectrodeConfig,
  ElectrodeMaterial,
  ElectrodeArray,
  CurrentSteering,
  FrequencyBand,
  FrequencyMap,
  FrequencyCustomization,
  FrequencyAdjustment,
  NoiseReductionMode,
  DirectionalMode,
  NoiseReduction,
  DirectionalMicrophone,
  SpeechEnhancement,
  SpeechRecognitionMetrics,
  PitchRefinement,
  HarmonicEnhancement,
  MusicProgram,
  MusicPerceptionAssessment,
  BilateralSyncMode,
  BilateralSync,
  InterauralCues,
  SoundLocation,
  BilateralProgramLink,
  TinnitusQuality,
  TinnitusPattern,
  TinnitusStimulation,
  TinnitusMasking,
  TinnitusSuppressionDegree,
  TinnitusAssessment,
  AcousticScene,
  ClassifierAlgorithm,
  SceneClassifier,
  SceneClassification,
  SceneFeatures,
  ProgramConfig,
  BluetoothVersion,
  AudioCodec,
  BluetoothConfig,
  TelecoilConfig,
  StreamingQuality,
  RemoteCapability,
  RemoteProgramming,
  BatteryType,
  BatterySize,
  BatterySpecification,
  BatteryState,
  PowerMode,
  PowerSavingFeature,
  PowerManagement,
  ElectrodeSettings,
  Program,
  DeviceMAP,
  ThresholdMeasurement,
  PatientInfo,
  BionicEarDevice,
  PerformanceRating,
  CompatibilityAssessment,
  HearingPerformanceOutcome,
  DeviceReport,
};

export { BIONIC_EAR_CONSTANTS, BionicEarErrorCode, BionicEarError };
