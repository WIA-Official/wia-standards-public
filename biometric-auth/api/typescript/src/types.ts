/**
 * WIA Biometric Authentication Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-biometric-auth
 */

export enum BiometricModality {
  Fingerprint = 'fingerprint',
  Face = 'face',
  Iris = 'iris',
  Voice = 'voice',
  Palm = 'palm',
  Vein = 'vein',
  Retina = 'retina',
  Signature = 'signature',
  Gait = 'gait',
  Keystroke = 'keystroke'
}

export enum SecurityLevel {
  Low = 'low',
  Medium = 'medium',
  High = 'high',
  Maximum = 'maximum'
}

export enum LivenessDetection {
  None = 'none',
  Passive = 'passive',
  Active = 'active',
  Challenge = 'challenge'
}

export enum MatchingMode {
  Verification = 'verification',
  Identification = 'identification'
}

export enum EnrollmentStatus {
  Pending = 'pending',
  InProgress = 'in_progress',
  Completed = 'completed',
  Failed = 'failed',
  Revoked = 'revoked'
}

export enum AuthenticationResult {
  Success = 'success',
  Failure = 'failure',
  Spoof = 'spoof',
  Timeout = 'timeout',
  Error = 'error'
}

export interface BiometricTemplate {
  id: string;
  userId: string;
  modality: BiometricModality;
  templateData: string;
  quality: number;
  createdAt: Date;
  updatedAt: Date;
  expiresAt?: Date;
  metadata: TemplateMetadata;
}

export interface TemplateMetadata {
  captureDevice: string;
  captureEnvironment: CaptureEnvironment;
  algorithm: string;
  version: string;
  encrypted: boolean;
}

export interface CaptureEnvironment {
  lighting?: 'bright' | 'normal' | 'dim';
  background?: string;
  distance?: number;
  angle?: number;
}

export interface BiometricSample {
  id: string;
  modality: BiometricModality;
  data: string;
  format: SampleFormat;
  quality: QualityScore;
  capturedAt: Date;
  device: DeviceInfo;
  liveness: LivenessResult;
}

export enum SampleFormat {
  RAW = 'raw',
  ISO = 'iso',
  ANSI = 'ansi',
  JPEG = 'jpeg',
  PNG = 'png',
  WSQ = 'wsq',
  WAV = 'wav'
}

export interface QualityScore {
  overall: number;
  nfiq?: number;
  sharpness?: number;
  brightness?: number;
  contrast?: number;
  resolution?: number;
}

export interface DeviceInfo {
  id: string;
  manufacturer: string;
  model: string;
  type: 'optical' | 'capacitive' | 'ultrasonic' | 'thermal' | 'camera' | 'microphone';
  certification?: string;
  firmwareVersion?: string;
}

export interface LivenessResult {
  method: LivenessDetection;
  passed: boolean;
  score: number;
  challenges?: ChallengeResult[];
}

export interface ChallengeResult {
  type: string;
  passed: boolean;
  responseTime: number;
}

export interface EnrollmentSession {
  id: string;
  userId: string;
  modality: BiometricModality;
  status: EnrollmentStatus;
  samples: BiometricSample[];
  requiredSamples: number;
  template?: BiometricTemplate;
  startedAt: Date;
  completedAt?: Date;
  error?: string;
}

export interface AuthenticationRequest {
  id: string;
  mode: MatchingMode;
  modality: BiometricModality;
  sample: BiometricSample;
  userId?: string;
  threshold?: number;
  maxCandidates?: number;
  timestamp: Date;
}

export interface AuthenticationResponse {
  requestId: string;
  result: AuthenticationResult;
  userId?: string;
  score: number;
  threshold: number;
  matchedTemplate?: string;
  candidates?: MatchCandidate[];
  processingTime: number;
  timestamp: Date;
}

export interface MatchCandidate {
  userId: string;
  templateId: string;
  score: number;
  rank: number;
}

export interface MultiModalRequest {
  id: string;
  userId?: string;
  samples: BiometricSample[];
  fusionMethod: FusionMethod;
  threshold?: number;
  timestamp: Date;
}

export enum FusionMethod {
  ScoreLevel = 'score_level',
  DecisionLevel = 'decision_level',
  FeatureLevel = 'feature_level',
  RankLevel = 'rank_level'
}

export interface MultiModalResponse {
  requestId: string;
  result: AuthenticationResult;
  userId?: string;
  fusedScore: number;
  modalityResults: ModalityResult[];
  processingTime: number;
  timestamp: Date;
}

export interface ModalityResult {
  modality: BiometricModality;
  result: AuthenticationResult;
  score: number;
  weight: number;
}

export interface PerformanceMetrics {
  modality: BiometricModality;
  far: number;
  frr: number;
  eer: number;
  tar: number;
  enrollmentTime: number;
  verificationTime: number;
  sampleSize: number;
  timestamp: Date;
}

export interface AuditLog {
  id: string;
  eventType: AuditEventType;
  userId?: string;
  templateId?: string;
  result?: AuthenticationResult;
  ipAddress?: string;
  deviceId?: string;
  timestamp: Date;
  details: Record<string, unknown>;
}

export enum AuditEventType {
  Enrollment = 'enrollment',
  Verification = 'verification',
  Identification = 'identification',
  TemplateRevoke = 'template_revoke',
  TemplateUpdate = 'template_update',
  SpoofAttempt = 'spoof_attempt',
  SystemError = 'system_error'
}

export interface FIDOCredential {
  id: string;
  userId: string;
  publicKey: string;
  signCount: number;
  attestation: string;
  transports: string[];
  createdAt: Date;
}

export interface PrivacySettings {
  templateEncryption: boolean;
  templateExpiry: number;
  onDeviceMatching: boolean;
  dataRetention: number;
  consentRequired: boolean;
}

export interface BiometricConfig {
  apiEndpoint: string;
  apiKey?: string;
  modalities: BiometricModality[];
  securityLevel: SecurityLevel;
  livenessDetection: LivenessDetection;
  qualityThreshold: number;
  matchThreshold: number;
  privacy: PrivacySettings;
}

export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

export interface ComplianceReport {
  standard: 'WIA-BIOMETRIC-AUTH';
  testDate: string;
  config: BiometricConfig;
  targetLevel: CertificationLevel;
  tests: TestResult[];
  passed: boolean;
  achievedLevel?: CertificationLevel;
}

export interface TestResult {
  testName: string;
  passed: boolean;
  notes?: string;
}
