/**
 * WIA-CONTACT-001: First Contact Protocol
 * TypeScript Type Definitions
 *
 * 弘익人間 (홍익인간) · Benefit All Humanity
 */

// Signal Detection Types
export interface SignalCoordinates {
  ra: number; // Right Ascension (0-360 degrees)
  dec: number; // Declination (-90 to 90 degrees)
  galacticLat?: number;
  galacticLong?: number;
}

export interface SignalDetection {
  id: string;
  timestamp: Date;
  frequency: number; // MHz
  bandwidth: number; // MHz
  strength: number; // dBm
  source: SignalCoordinates;
  polarization?: 'linear' | 'circular' | 'mixed';
  modulation?: ModulationType;
  duration?: number; // seconds
  metadata?: Record<string, any>;
}

export type ModulationType =
  | 'unknown'
  | 'am' // Amplitude Modulation
  | 'fm' // Frequency Modulation
  | 'pm' // Phase Modulation
  | 'pulse' // Pulse Code
  | 'binary' // Binary Encoding
  | 'complex'; // Complex Modulation

// Verification Types
export interface Observatory {
  id: string;
  name: string;
  location: {
    latitude: number;
    longitude: number;
    altitude: number;
  };
  capabilities: string[];
  status: 'active' | 'inactive' | 'maintenance';
}

export interface VerificationResult {
  observatoryId: string;
  observatoryName: string;
  confirmed: boolean;
  confidence: number; // 0-1
  timestamp: Date;
  notes?: string;
}

export interface MultiSiteVerification {
  signalId: string;
  totalObservatories: number;
  confirmedSites: number;
  verifications: VerificationResult[];
  consensus: number; // 0-1
  authentic: boolean;
  completedAt?: Date;
}

// Analysis Types
export interface PatternAnalysis {
  patternType: PatternType;
  mathematicalSignificance: 'low' | 'medium' | 'high';
  artificialProbability: number; // 0-1
  entropy: number;
  decodedContent?: string;
  interpretation?: string;
}

export type PatternType =
  | 'prime-sequence'
  | 'fibonacci'
  | 'mathematical-constants'
  | 'binary-encoding'
  | 'pictorial'
  | 'audio-signature'
  | 'unknown';

export interface ThreatAssessment {
  level: ThreatLevel;
  indicators: string[];
  confidence: number; // 0-1
  recommendation: string;
  timestamp: Date;
}

export type ThreatLevel = 'benign' | 'caution' | 'elevated' | 'high' | 'critical';

// Response Types
export interface ResponseMessage {
  id: string;
  type: ResponseType;
  content: string | Buffer;
  encoding: 'text' | 'binary' | 'mathematical' | 'pictorial' | 'audio';
  targetFrequency: number; // MHz
  transmissionPower: number; // kW
  duration: number; // seconds
  metadata?: Record<string, any>;
}

export type ResponseType =
  | 'mathematical-acknowledgment'
  | 'pictorial-message'
  | 'audio-signal'
  | 'binary-code'
  | 'custom';

export interface DiplomaticProtocol {
  stage: DiplomaticStage;
  objectives: string[];
  constraints: string[];
  approvals: Approval[];
  status: 'draft' | 'under-review' | 'approved' | 'rejected' | 'transmitted';
}

export type DiplomaticStage =
  | 'initial-contact'
  | 'language-establishment'
  | 'cultural-exchange'
  | 'formal-relations'
  | 'ongoing-communication';

export interface Approval {
  authority: string; // e.g., "UN Security Council"
  approved: boolean;
  timestamp: Date;
  votes?: {
    for: number;
    against: number;
    abstain: number;
  };
}

// Protocol Configuration
export interface FirstContactConfig {
  apiKey: string;
  verificationLevel: 'minimal' | 'standard' | 'strict';
  responseMode: 'cautious' | 'balanced' | 'proactive';
  autoVerify?: boolean;
  minConsensus?: number; // 0-1, default 0.95
  threatThreshold?: ThreatLevel;
  enableLogging?: boolean;
  logLevel?: 'error' | 'warn' | 'info' | 'debug';
}

// Main Protocol Interface
export interface FirstContactProtocol {
  // Detection
  detectSignal(params: DetectSignalParams): Promise<SignalDetection>;
  monitorFrequency(frequency: number, duration: number): Promise<SignalDetection[]>;

  // Verification
  verify(signal: SignalDetection, options?: VerifyOptions): Promise<MultiSiteVerification>;
  requestAdditionalVerification(signalId: string, observatories: string[]): Promise<void>;

  // Analysis
  analyzePattern(signal: SignalDetection): Promise<PatternAnalysis>;
  assessThreat(signal: SignalDetection, analysis: PatternAnalysis): Promise<ThreatAssessment>;

  // Response
  generateUniversalMessage(type?: ResponseType): string;
  createResponse(params: CreateResponseParams): ResponseMessage;
  submitForReview(response: ResponseMessage): Promise<DiplomaticProtocol>;
  initiateResponse(params: InitiateResponseParams): Promise<void>;

  // Data Management
  getSignal(id: string): Promise<SignalDetection>;
  listSignals(filter?: SignalFilter): Promise<SignalDetection[]>;
  exportData(format: 'json' | 'csv' | 'xml'): Promise<Buffer>;
}

// Parameter Types
export interface DetectSignalParams {
  frequency: number;
  source: SignalCoordinates;
  strength: number;
  duration?: number;
  modulation?: ModulationType;
}

export interface VerifyOptions {
  multiSiteConfirmation?: boolean;
  falsePositiveCheck?: boolean;
  scientificConsensus?: number; // 0-1
  timeout?: number; // milliseconds
}

export interface CreateResponseParams {
  type: ResponseType;
  content?: string;
  targetFrequency?: number;
  transmissionPower?: number;
}

export interface InitiateResponseParams {
  message: string | ResponseMessage;
  safetyProtocol: 'minimum' | 'standard' | 'maximum';
  diplomaticFramework?: 'peaceful-intent' | 'scientific-exchange' | 'formal-diplomatic';
  requireApproval?: boolean;
}

export interface SignalFilter {
  startDate?: Date;
  endDate?: Date;
  minStrength?: number;
  maxStrength?: number;
  verified?: boolean;
  threatLevel?: ThreatLevel;
}

// Event Types
export interface ContactEvent {
  type: ContactEventType;
  timestamp: Date;
  data: any;
}

export type ContactEventType =
  | 'signal-detected'
  | 'verification-complete'
  | 'pattern-decoded'
  | 'threat-assessed'
  | 'response-sent'
  | 'communication-established';

// Error Types
export class ContactProtocolError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: any
  ) {
    super(message);
    this.name = 'ContactProtocolError';
  }
}

export class VerificationError extends ContactProtocolError {
  constructor(message: string, details?: any) {
    super(message, 'VERIFICATION_ERROR', details);
    this.name = 'VerificationError';
  }
}

export class AuthorizationError extends ContactProtocolError {
  constructor(message: string, details?: any) {
    super(message, 'AUTHORIZATION_ERROR', details);
    this.name = 'AuthorizationError';
  }
}

// Utility Types
export type Timestamp = number | Date | string;

export interface CoordinateRange {
  raMin: number;
  raMax: number;
  decMin: number;
  decMax: number;
}

export interface FrequencyBand {
  name: string;
  minFreq: number; // MHz
  maxFreq: number; // MHz
  priority: number;
}

// Constants
export const HYDROGEN_LINE = 1420.4; // MHz
export const WATER_HOLE_MIN = 1420.4; // MHz
export const WATER_HOLE_MAX = 1666.7; // MHz

export const DEFAULT_VERIFICATION_THRESHOLD = 0.95;
export const MIN_OBSERVATORY_CONFIRMATIONS = 3;
export const VERIFICATION_TIMEOUT_MS = 172800000; // 48 hours

// Export all types
export default FirstContactProtocol;
