/**
 * WIA-LEG-009 Right to be Forgotten Standard - Type Definitions
 * @version 1.0.0
 * @license MIT
 */

export type RequestType = 'gdpr_article17' | 'ccpa_deletion' | 'lgpd_deletion' | 'custom';
export type VerificationMethod = 'email_otp' | 'sms_otp' | 'account_login' | 'government_id' | 'custom';
export type DeletionStatus = 'pending_verification' | 'processing' | 'completed' | 'denied' | 'failed';
export type DataCategory = 'profile' | 'activity' | 'financial' | 'location' | 'communications' | 'media' | 'behavioral' | 'biometric' | 'health' | string;

export interface IdentityVerification {
  method: VerificationMethod;
  verificationToken?: string;
  verified: boolean;
  verificationTimestamp?: string;
}

export interface DataSubject {
  identifier: string;
  identifierValue: string;
  verification: IdentityVerification;
}

export interface DeletionRequest {
  requestId?: string;
  version: string;
  timestamp?: string;
  requestType: RequestType;
  dataSubject: DataSubject;
  dataCategories: DataCategory[];
  legalBasis: string;
  jurisdiction: string;
  additionalInformation?: string;
}

export interface DeletionRequestResponse {
  requestId: string;
  status: DeletionStatus;
  estimatedCompletion?: string;
  trackingUrl: string;
}

export interface DeletionStatus {
  requestId: string;
  status: DeletionStatus;
  currentStage?: string;
  progress?: number;
  estimatedCompletion?: string;
  events?: DeletionEvent[];
}

export interface DeletionEvent {
  timestamp: string;
  actor: string;
  action: string;
  details?: any;
}

export interface DeletionDetails {
  categoriesDeleted: DataCategory[];
  recordsDeleted: number;
  storageLocations: string[];
  deletionMethod: string;
  completionTimestamp: string;
}

export interface Verification {
  method: string;
  merkleRoot?: string;
  proofElements?: string[];
  witnessSignatures?: string[];
}

export interface CryptographicProof {
  algorithm: string;
  publicKey: string;
  signature: string;
  timestamp: string;
}

export interface BlockchainAnchor {
  enabled: boolean;
  blockchain?: string;
  transactionHash?: string;
  blockNumber?: number;
  timestamp?: string;
}

export interface DeletionCertificate {
  certificateId: string;
  version: string;
  requestId: string;
  issuedAt: string;
  dataSubject: {
    identifierHash: string;
    jurisdiction: string;
  };
  deletionDetails: DeletionDetails;
  verification: Verification;
  cryptographicProof: CryptographicProof;
  blockchainAnchor?: BlockchainAnchor;
  auditTrail: {
    events: DeletionEvent[];
  };
  issuer: {
    organization: string;
    dpoContact: string;
    certificateUrl: string;
  };
}

export interface WIAClientConfig {
  apiKey: string;
  baseUrl?: string;
  jurisdiction?: string;
  timeout?: number;
}
