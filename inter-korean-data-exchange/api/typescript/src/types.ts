/**
 * WIA-UNI-001 TypeScript Type Definitions
 * Inter-Korean Data Exchange Standard
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

export type Region = 'north' | 'south';

export type MessageType = 'text' | 'voice' | 'video' | 'document' | 'photo';

export type Priority = 'normal' | 'high' | 'critical';

export type MessageStatus = 'pending' | 'sent' | 'delivered' | 'read' | 'failed';

export type TrustAnchorType = 'rok-gov' | 'dprk-gov' | 'un-observer' | 'icrc';

export type HumanitarianCategory =
  | 'family-reunification'
  | 'medical-emergency'
  | 'disaster-response'
  | 'missing-person'
  | 'cultural-preservation';

// ============================================================================
// User & Identity
// ============================================================================

export interface UserIdentity {
  userId: string;
  region: Region;
  publicKey: string;
  verified: boolean;
}

export interface User extends UserIdentity {
  name?: string;
  email?: string;
  phone?: string;
  registeredAt: Date;
  lastActive?: Date;
}

// ============================================================================
// Messages
// ============================================================================

export interface MessageContent {
  text?: string;
  attachments?: Attachment[];
  metadata?: Record<string, any>;
}

export interface Attachment {
  id: string;
  type: 'photo' | 'video' | 'audio' | 'document';
  url: string;
  filename: string;
  size: number;
  mimeType: string;
  thumbnail?: string;
}

export interface EncryptedPayload {
  encrypted: string; // Base64 encoded ciphertext
  ephemeralKey: string; // Base64 encoded ephemeral public key
  authTag: string; // Base64 encoded authentication tag
  algorithm: 'AES-256-GCM';
}

export interface Message {
  id: string;
  version: string;
  timestamp: Date;
  from: UserIdentity;
  to: UserIdentity;
  type: MessageType;
  humanitarian: boolean;
  priority: Priority;
  payload: EncryptedPayload;
  signatures: MessageSignatures;
  status?: MessageStatus;
}

export interface MessageSignatures {
  sender: string; // Base64 signature
  trustAnchors: TrustAnchorSignature[];
}

export interface TrustAnchorSignature {
  anchor: TrustAnchorType;
  signature: string;
  timestamp: Date;
  verified: boolean;
}

// ============================================================================
// Trust & Verification
// ============================================================================

export interface TrustAnchor {
  id: string;
  type: TrustAnchorType;
  organization: string;
  publicKey: string;
  verificationEndpoint: string;
  status: 'active' | 'inactive';
}

export interface VerificationProof {
  transactionId: string;
  blockNumber: number;
  blockHash: string;
  timestamp: Date;
  verifiedBy: TrustAnchorType[];
  merkleProof: string[];
}

export interface BlockchainRecord {
  transactionId: string;
  timestamp: Date;
  messageType: MessageType;
  fromRegion: Region;
  toRegion: Region;
  contentHash: string; // SHA-256
  humanitarian: boolean;
  verifications: Record<TrustAnchorType, string>;
  blockNumber: number;
  previousBlockHash: string;
}

// ============================================================================
// Family Reunification
// ============================================================================

export interface FamilyMember {
  personalInfo: {
    familyName: string;
    givenName: string;
    birthYear: number;
    birthPlace?: string;
    lastKnownLocation?: string;
  };
  searchingFor: {
    relationship: FamilyRelationship;
    name: string;
    estimatedAge?: number;
    lastSeen?: {
      year: number;
      location: string;
    };
  };
  verificationData: {
    photos?: string[]; // URLs or base64
    voiceRecording?: string;
    additionalInfo?: string;
  };
  privacyLevel: 'public' | 'verified-only' | 'private';
}

export type FamilyRelationship =
  | 'parent'
  | 'child'
  | 'sibling'
  | 'spouse'
  | 'grandparent'
  | 'grandchild'
  | 'aunt-uncle'
  | 'niece-nephew'
  | 'cousin'
  | 'other';

export interface FamilyMatch {
  matchId: string;
  confidence: number; // 0-1
  member1: FamilyMember;
  member2: FamilyMember;
  matchReasons: string[];
  verified: boolean;
  verifiedBy?: 'red-cross' | 'dna' | 'documents';
}

// ============================================================================
// Humanitarian
// ============================================================================

export interface HumanitarianMessage extends Message {
  humanitarian: true;
  humanitarianDetails: {
    category: HumanitarianCategory;
    priority: Priority;
    specialHandling: {
      enhancedPrivacy: boolean;
      expeditedDelivery: boolean;
      translationNeeded: boolean;
      assistanceRequired: boolean;
    };
    redCrossVerified: boolean;
    verificationId?: string;
  };
}

export interface EmergencyAlert {
  id: string;
  type: 'medical' | 'disaster' | 'missing' | 'humanitarian-crisis';
  severity: 'low' | 'medium' | 'high' | 'critical';
  location: string;
  description: string;
  reportedBy: UserIdentity;
  reportedAt: Date;
  status: 'active' | 'resolved' | 'false-alarm';
  responses: EmergencyResponse[];
}

export interface EmergencyResponse {
  responderId: string;
  responderType: 'rok-emergency' | 'dprk-emergency' | 'un' | 'red-cross';
  timestamp: Date;
  action: string;
  status: string;
}

// ============================================================================
// Configuration
// ============================================================================

export interface ExchangeConfig {
  trustAnchors: TrustAnchorType[];
  encryption: 'military-grade' | 'standard';
  auditLevel: 'transparent' | 'privacy-preserving';
  environment: 'production' | 'staging' | 'test';
  endpoints?: {
    exchange?: string;
    verification?: string;
    humanitarian?: string;
  };
  timeout?: number;
  retries?: number;
}

export interface AuthCredentials {
  region: Region;
  userId: string;
  privateKey: string;
  certificate?: string;
}

// ============================================================================
// API Responses
// ============================================================================

export interface SendMessageResponse {
  success: boolean;
  messageId: string;
  transactionId: string;
  verifications: TrustAnchorSignature[];
  estimatedDelivery?: Date;
  error?: ErrorResponse;
}

export interface SearchFamilyResponse {
  success: boolean;
  matches: FamilyMatch[];
  totalResults: number;
  error?: ErrorResponse;
}

export interface VerificationResponse {
  success: boolean;
  verified: boolean;
  proof: VerificationProof;
  error?: ErrorResponse;
}

export interface ErrorResponse {
  code: number;
  message: string;
  details?: string;
  retryable: boolean;
}

// ============================================================================
// Events
// ============================================================================

export interface MessageEvent {
  type: 'message';
  message: Message;
}

export interface VerificationEvent {
  type: 'verification';
  transactionId: string;
  verification: TrustAnchorSignature;
}

export interface StatusEvent {
  type: 'status';
  messageId: string;
  status: MessageStatus;
  timestamp: Date;
}

export interface ErrorEvent {
  type: 'error';
  error: ErrorResponse;
}

export type ExchangeEvent =
  | MessageEvent
  | VerificationEvent
  | StatusEvent
  | ErrorEvent;

// ============================================================================
// Utility Types
// ============================================================================

export interface KeyPair {
  publicKey: string;
  privateKey: string;
  algorithm: 'ECDH-P256' | 'CRYSTALS-Kyber-768';
}

export interface EncryptionOptions {
  algorithm?: 'AES-256-GCM';
  pfs?: boolean; // Perfect Forward Secrecy
  authenticate?: boolean;
}

export interface TranslationOptions {
  from: 'north-korean' | 'south-korean' | 'auto';
  to: 'north-korean' | 'south-korean';
  dialect?: boolean; // Preserve dialect characteristics
  formal?: boolean; // Use formal language
}

// ============================================================================
// Constants
// ============================================================================

export const SUPPORTED_MESSAGE_TYPES: MessageType[] = [
  'text',
  'voice',
  'video',
  'document',
  'photo'
];

export const TRUST_ANCHORS: TrustAnchorType[] = [
  'rok-gov',
  'dprk-gov',
  'un-observer',
  'icrc'
];

export const MAX_MESSAGE_SIZE = {
  text: 10 * 1024 * 1024, // 10 MB
  voice: 30 * 60 * 1000, // 30 minutes
  video: 2 * 60 * 60 * 1000, // 2 hours
  photo: 50 * 1024 * 1024, // 50 MB per photo
  document: 100 * 1024 * 1024 // 100 MB
};

export const VERSION = '1.0.0';
